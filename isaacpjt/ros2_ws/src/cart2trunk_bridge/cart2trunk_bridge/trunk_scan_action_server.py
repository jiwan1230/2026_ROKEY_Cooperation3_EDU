"""trunk_scan_action_server.py

Isaac Sim PC에서 실행하는 ROS2 액션 서버. `/cart2trunk/trunk_scan` 액션 요청을
받으면 89.trunk_scan_holonomic.py를 새로 부팅하지 않는다 - [2026-07-28 변경]
Isaac Sim PC에서 이미 계속 떠있는 isaac_task_runner.py(cart_scan/trunk_scan/
pick_and_place가 하나의 지속 세션 안에서 이어지는 구조, pick_and_place_action_server.py와
동일 설계 배경 참고)의 /isaac_task_runner/run_trunk_scan Trigger 서비스를 대신
호출하고 /isaac_task_runner/status를 Feedback으로 중계한다.

원래 이 서버는 89.py를 직접 서브프로세스로 부팅했었는데, isaac_task_runner.py가
이미 Isaac Sim을 띄운 채로 떠있는 상태에서 그렇게 하면 같은 GPU에 Isaac Sim
인스턴스가 2개 뜨는 셈이라 충돌한다 - cart_scan/trunk_scan/pick_and_place 전부
isaac_task_runner.py의 같은 세션을 거치게 통일했다.

Isaac Sim 스캔이 끝난 뒤 90.export_trunk_map_holonomic.py를 실행해서
trunk_map.json을 만드는 후처리 단계와, 결과 PLY를 float32로 재인코딩해서 청크로
보내는 부분은 그대로다(Isaac Sim이 필요 없는 순수 파이썬 단계라 여기선 안 바뀜).
89.py/90.py 원본 파일은 여전히 subprocess(또는 isaac_task_runner.py 안의 함수
호출)로만 다루며 절대 열어서 수정하지 않는다.
"""
import math
import pathlib
import queue
import time

import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from cart2trunk_interfaces.action import TrunkScan

from cart2trunk_bridge import pipeline_runner

FEEDBACK_QOS_DEPTH = 100
STATUS_QOS_DEPTH = 20


class TrunkScanActionServer(Node):
    def __init__(self):
        super().__init__("trunk_scan_action_server")

        # cart2trunk_dir은 여전히 필요하다 - 90.export_trunk_map_holonomic.py를
        # 실행할 작업 디렉터리이자, 결과 PLY를 읽어올 경로의 기준이기 때문.
        # isaac_python_sh/ros2_bridge_ld_library_path는 더 이상 필요 없다(89.py를
        # 여기서 직접 부팅하지 않으므로) - 파라미터 자체를 없앴다.
        self.declare_parameter("cart2trunk_dir", "")
        # 32768(32KiB) - 실측으로 확인한 안전한 청크 크기. loopback rmw_fastrtps_cpp
        # 기준 실험한 결과 feedback 메시지가 대략 165KB~240KB 구간에서 간헐적으로
        # 통째로 유실되는 현상을 확인함(청크 인덱스가 통째로 클라이언트에 도달하지
        # 않음, 재현: 32~150KB는 매번 성공, 165KB+는 실패 빈발 - UDP 프래그먼트
        # 유실로 추정). 65536까지도 반복 테스트에서 안정적이었지만 여유를 크게 둔다.
        self.declare_parameter("chunk_size_bytes", 32768)
        # 90.export_trunk_map_holonomic.py(순수 파이썬, 몇 초면 끝남)용 타임아웃.
        self.declare_parameter("subprocess_timeout_sec", 600)
        # isaac_task_runner.py의 run_trunk_scan() 완료를 기다리는 타임아웃 - 실측
        # 5~7분 걸렸으니 넉넉히 잡는다.
        self.declare_parameter("isaac_phase_timeout_sec", 900)

        if not self.get_parameter("cart2trunk_dir").value:
            raise RuntimeError(
                "필수 파라미터 'cart2trunk_dir'가 설정되지 않았습니다. "
                "config/trunk_scan_server.params.yaml을 이 머신에 맞게 채워서 "
                "--ros-args --params-file 로 넘겨주세요.")

        self._cart2trunk_dir = pathlib.Path(self.get_parameter("cart2trunk_dir").value)
        self._subprocess_timeout_sec = int(self.get_parameter("subprocess_timeout_sec").value)
        self._isaac_phase_timeout_sec = int(self.get_parameter("isaac_phase_timeout_sec").value)

        self._status_queue: "queue.Queue" = queue.Queue()
        self.create_subscription(
            String, "/isaac_task_runner/status", self._on_status, STATUS_QOS_DEPTH)
        self._trigger_client = self.create_client(
            Trigger, "/isaac_task_runner/run_trunk_scan", callback_group=ReentrantCallbackGroup())

        self._action_server = ActionServer(
            self, TrunkScan, "/cart2trunk/trunk_scan",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            feedback_pub_qos_profile=QoSProfile(depth=FEEDBACK_QOS_DEPTH),
        )
        self.get_logger().info(
            "trunk_scan_action_server 준비 완료 (/cart2trunk/trunk_scan) - "
            "isaac_task_runner.py가 켜져 있어야 실제로 동작함")

    def _on_status(self, msg: String):
        import json
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._status_queue.put(data)

    def _emit(self, goal_handle, feedback, stage, chunk_index=0, total_chunks=0, chunk_data=b""):
        feedback.stage = stage
        feedback.chunk_index = chunk_index
        feedback.total_chunks = total_chunks
        feedback.chunk_data = chunk_data
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(f"[진행] {stage}")

    def _run_isaac_trunk_scan(self, goal_handle, feedback):
        """isaac_task_runner.py의 /isaac_task_runner/run_trunk_scan Trigger를 호출하고,
        /isaac_task_runner/status에서 task=trunk_scan의 done/error를 기다린다.
        실패 시 pipeline_runner.PipelineStageError를 던진다(execute_callback의 기존
        except 절과 동일한 방식으로 처리되게)."""
        while not self._status_queue.empty():
            try:
                self._status_queue.get_nowait()
            except queue.Empty:
                break

        if not self._trigger_client.wait_for_service(timeout_sec=10.0):
            raise pipeline_runner.PipelineStageError(
                "isaac_task_runner.py의 /isaac_task_runner/run_trunk_scan 서비스에 연결할 수 "
                "없습니다 - Isaac Sim PC에서 isaac_task_runner.py가 실행 중인지 확인하세요.")

        trigger_future = self._trigger_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 10.0
        while not trigger_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not trigger_future.done() or trigger_future.result() is None:
            raise pipeline_runner.PipelineStageError("run_trunk_scan 트리거 응답 타임아웃")
        trigger_resp = trigger_future.result()
        if not trigger_resp.success:
            raise pipeline_runner.PipelineStageError(trigger_resp.message)

        deadline = time.monotonic() + self._isaac_phase_timeout_sec
        while time.monotonic() < deadline:
            try:
                data = self._status_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if data.get("task") != "trunk_scan":
                continue
            stage = data.get("stage", "")
            if stage == "done":
                return
            if stage == "error":
                raise pipeline_runner.PipelineStageError(
                    data.get("message", "트렁크 스캔 실패(원인 불명)"))
            self._emit(goal_handle, feedback, f"isaac:{stage}")

        raise pipeline_runner.PipelineStageError(
            f"{self._isaac_phase_timeout_sec}초 안에 트렁크 스캔 완료/실패 신호를 받지 "
            f"못했습니다(isaac_task_runner.py 로그를 직접 확인하세요)")

    def execute_callback(self, goal_handle):
        feedback = TrunkScan.Feedback()
        cwd = str(self._cart2trunk_dir)

        try:
            self._emit(goal_handle, feedback, "scanning")
            self._run_isaac_trunk_scan(goal_handle, feedback)

            self._emit(goal_handle, feedback, "launching_90")
            pipeline_runner.run_stage(
                ["python3", "90.export_trunk_map_holonomic.py"],
                cwd=cwd, timeout_sec=self._subprocess_timeout_sec,
                on_line=lambda line: self._emit(goal_handle, feedback, "exporting"))

            self._emit(goal_handle, feedback, "reading_ply")
            ply_path = self._cart2trunk_dir / "results" / "holonomic_base" / "trunk_pointcloud_filtered_base.ply"
            raw = ply_path.read_bytes()
            data = pipeline_runner.convert_ply_double_to_float32(raw)
            point_count = pipeline_runner.parse_ply_vertex_count(data)

            chunk_size = int(self.get_parameter("chunk_size_bytes").value)
            total_chunks = max(1, math.ceil(len(data) / chunk_size))
            self._emit(goal_handle, feedback, "sending_chunks", total_chunks=total_chunks)
            for i in range(total_chunks):
                chunk = data[i * chunk_size:(i + 1) * chunk_size]
                self._emit(goal_handle, feedback, "sending_chunks", i, total_chunks, chunk)

            # [2026-07-28] "원본" 뷰용 - 89.py가 저장한 필터링 전 포인트클라우드.
            # 90.py를 거친 위 trunk_pointcloud_filtered_base.ply(크롭+이상치제거+
            # voxel다운샘플)와 달리, 이건 그대로다(수백만 점이라 90.py 결과보다
            # 훨씬 큼 - 청크 수도 그만큼 많다).
            self._emit(goal_handle, feedback, "reading_raw_ply")
            raw_npy_path = self._cart2trunk_dir / "results" / "holonomic_base" / "trunk_pointcloud.npy"
            raw_points = np.load(raw_npy_path)
            raw_data = pipeline_runner.write_binary_ply_from_xyz(raw_points)
            raw_point_count = len(raw_points)

            raw_total_chunks = max(1, math.ceil(len(raw_data) / chunk_size))
            self._emit(goal_handle, feedback, "sending_raw_chunks", total_chunks=raw_total_chunks)
            for i in range(raw_total_chunks):
                chunk = raw_data[i * chunk_size:(i + 1) * chunk_size]
                self._emit(goal_handle, feedback, "sending_raw_chunks", i, raw_total_chunks, chunk)

        except pipeline_runner.PipelineStageError as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            return TrunkScan.Result(success=False, message=str(e))
        except Exception as e:  # noqa: BLE001 - 액션 서버가 죽지 않도록 광범위 예외 처리
            self.get_logger().error(f"예상치 못한 오류: {e}")
            goal_handle.abort()
            return TrunkScan.Result(success=False, message=f"예상치 못한 오류: {e}")

        self._emit(goal_handle, feedback, "done", total_chunks=raw_total_chunks)
        goal_handle.succeed()
        import datetime
        stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S_%f}"
        return TrunkScan.Result(
            success=True, message="트렁크 스캔 완료",
            filename=f"trunk_scan_{stamp}.ply", total_bytes=len(data),
            total_chunks=total_chunks, point_count=point_count,
            raw_filename=f"trunk_scan_raw_{stamp}.ply", raw_total_bytes=len(raw_data),
            raw_total_chunks=raw_total_chunks, raw_point_count=raw_point_count,
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrunkScanActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
