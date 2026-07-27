"""cart_scan_action_server.py

Isaac Sim PC에서 실행하는 ROS2 액션 서버. `/cart2trunk/cart_scan` 액션 요청을
받으면 99.cart_scan_dual_side_holonomic.py를 새로 부팅하지 않는다 -
[2026-07-28 변경] trunk_scan_action_server.py와 동일한 이유(isaac_task_runner.py가
이미 Isaac Sim을 띄운 채로 떠있어서, 여기서 또 새로 부팅하면 같은 GPU에 Isaac Sim
인스턴스가 2개 뜨는 셈이라 충돌함) - isaac_task_runner.py의
/isaac_task_runner/run_cart_scan Trigger 서비스를 대신 호출하고
/isaac_task_runner/status를 Feedback으로 중계한다.

Isaac Sim 스캔이 끝난 뒤 perception/multiview_scan.py로 박스를 검출하는 후처리
단계와, 검출된 박스 JSON(전체 텍스트) + 통합 PLY(청크)를 UI PC로 보내는 부분은
그대로다(Isaac Sim이 필요 없는 순수 파이썬 단계라 여기선 안 바뀜) - 다만
CART2TRUNK_DETECTION_TRIALS/CART2TRUNK_MIN_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M 환경변수를
새로 추가했다: multiview_scan.py 기본값(각각 12/0.08)으로 돌리면 실제 박스
4개(Large1/Large2/Medium/Small) 중 위에 쌓인 2개(Small의 실제 짧은 변이 0.07m로
기본 임계값 0.08m보다 작음)가 조용히 누락되는 걸 실측으로 확인했다(memory:
project_perception_env_vars.md 참고) - 150/0.05로 맞춰야 4개 다 나온다.

99.py/multiview_scan.py 원본 파일은 여전히 subprocess(또는 isaac_task_runner.py
안의 함수 호출)로만 다루며 절대 열어서 수정하지 않는다.
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

from cart2trunk_interfaces.action import CartScan

from cart2trunk_bridge import pipeline_runner

FEEDBACK_QOS_DEPTH = 100
STATUS_QOS_DEPTH = 20


class CartScanActionServer(Node):
    def __init__(self):
        super().__init__("cart_scan_action_server")

        # isaac_python_sh/ros2_bridge_ld_library_path는 더 이상 필요 없다(99.py를
        # 여기서 직접 부팅하지 않으므로) - 파라미터 자체를 없앴다. cart2trunk_dir과
        # perception_venv_python은 후처리 단계(multiview_scan.py)에 여전히 필요하다.
        self.declare_parameter("cart2trunk_dir", "")
        self.declare_parameter("perception_venv_python", "")
        # TrunkScan과 동일한 이유(32KB) - trunk_scan_action_server.py 주석 참고.
        self.declare_parameter("chunk_size_bytes", 32768)
        # multiview_scan.py(순수 파이썬, 150 trial 기준 실측 5~8분)용 타임아웃.
        self.declare_parameter("subprocess_timeout_sec", 900)
        # isaac_task_runner.py의 run_cart_scan() 완료를 기다리는 타임아웃 - 실측
        # 1~2분이면 끝나지만 넉넉히 잡는다.
        self.declare_parameter("isaac_phase_timeout_sec", 600)
        # multiview_scan.py 박스 검출 파라미터 - 위 docstring 참고. 이 프로젝트의
        # 실제 박스 크기(특히 Small=0.07x0.08m)에 맞춘 값이라, 박스 세트가 바뀌면
        # 재검토 필요.
        self.declare_parameter("detection_trials", 150)
        self.declare_parameter("min_plausible_box_footprint_side_m", 0.05)

        for name in ("cart2trunk_dir", "perception_venv_python"):
            if not self.get_parameter(name).value:
                raise RuntimeError(
                    f"필수 파라미터 '{name}'가 설정되지 않았습니다. "
                    f"config/cart_scan_server.params.yaml을 이 머신에 맞게 채워서 "
                    f"--ros-args --params-file 로 넘겨주세요.")

        self._cart2trunk_dir = pathlib.Path(self.get_parameter("cart2trunk_dir").value)
        self._perception_venv_python = self.get_parameter("perception_venv_python").value
        self._subprocess_timeout_sec = int(self.get_parameter("subprocess_timeout_sec").value)
        self._isaac_phase_timeout_sec = int(self.get_parameter("isaac_phase_timeout_sec").value)
        self._detection_trials = int(self.get_parameter("detection_trials").value)
        self._min_footprint_side_m = float(self.get_parameter("min_plausible_box_footprint_side_m").value)
        # multiview_scan.py의 SAVE_DIRECTORY(= Path.home()/"box_pointcloud")와
        # 동일한 규칙 - 그 스크립트 자체가 경로를 인자로 안 받으므로 그대로 맞춘다.
        self._box_pointcloud_dir = pathlib.Path.home() / "box_pointcloud"

        self._status_queue: "queue.Queue" = queue.Queue()
        self.create_subscription(
            String, "/isaac_task_runner/status", self._on_status, STATUS_QOS_DEPTH)
        self._trigger_client = self.create_client(
            Trigger, "/isaac_task_runner/run_cart_scan", callback_group=ReentrantCallbackGroup())

        self._action_server = ActionServer(
            self, CartScan, "/cart2trunk/cart_scan",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            feedback_pub_qos_profile=QoSProfile(depth=FEEDBACK_QOS_DEPTH),
        )
        self.get_logger().info(
            "cart_scan_action_server 준비 완료 (/cart2trunk/cart_scan) - "
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

    def _latest_boxes_json(self):
        candidates = sorted(
            self._box_pointcloud_dir.glob("all_boxes_corners_*.json"),
            key=lambda p: p.stat().st_mtime)
        if not candidates:
            raise pipeline_runner.PipelineStageError(
                f"{self._box_pointcloud_dir}에 all_boxes_corners_*.json이 없습니다 - "
                f"multiview_scan.py가 박스를 하나도 검출하지 못했을 수 있습니다.")
        return candidates[-1]

    def _run_isaac_cart_scan(self, goal_handle, feedback):
        """isaac_task_runner.py의 /isaac_task_runner/run_cart_scan Trigger를 호출하고,
        /isaac_task_runner/status에서 task=cart_scan의 done/error를 기다린다."""
        while not self._status_queue.empty():
            try:
                self._status_queue.get_nowait()
            except queue.Empty:
                break

        if not self._trigger_client.wait_for_service(timeout_sec=10.0):
            raise pipeline_runner.PipelineStageError(
                "isaac_task_runner.py의 /isaac_task_runner/run_cart_scan 서비스에 연결할 수 "
                "없습니다 - Isaac Sim PC에서 isaac_task_runner.py가 실행 중인지 확인하세요.")

        trigger_future = self._trigger_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 10.0
        while not trigger_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not trigger_future.done() or trigger_future.result() is None:
            raise pipeline_runner.PipelineStageError("run_cart_scan 트리거 응답 타임아웃")
        trigger_resp = trigger_future.result()
        if not trigger_resp.success:
            raise pipeline_runner.PipelineStageError(trigger_resp.message)

        deadline = time.monotonic() + self._isaac_phase_timeout_sec
        while time.monotonic() < deadline:
            try:
                data = self._status_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if data.get("task") != "cart_scan":
                continue
            stage = data.get("stage", "")
            if stage == "done":
                return
            if stage == "error":
                raise pipeline_runner.PipelineStageError(
                    data.get("message", "카트 스캔 실패(원인 불명)"))
            self._emit(goal_handle, feedback, f"isaac:{stage}")

        raise pipeline_runner.PipelineStageError(
            f"{self._isaac_phase_timeout_sec}초 안에 카트 스캔 완료/실패 신호를 받지 "
            f"못했습니다(isaac_task_runner.py 로그를 직접 확인하세요)")

    def execute_callback(self, goal_handle):
        feedback = CartScan.Feedback()

        try:
            self._emit(goal_handle, feedback, "scanning_cart")
            self._run_isaac_cart_scan(goal_handle, feedback)

            self._emit(goal_handle, feedback, "launching_multiview_scan")
            npy_path = self._cart2trunk_dir / "perception" / "scan_cache" / "merged_cart_scan.npy"
            marker_path = self._cart2trunk_dir / "perception" / "scan_cache" / "multiview_scan.marker"
            env_detect = {
                "CART2TRUNK_DETECTION_TRIALS": str(self._detection_trials),
                "CART2TRUNK_MIN_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M": str(self._min_footprint_side_m),
            }
            pipeline_runner.run_stage(
                [self._perception_venv_python, "multiview_scan.py",
                 "--input", str(npy_path), "--marker", str(marker_path)],
                cwd=str(self._cart2trunk_dir / "perception"), extra_env=env_detect,
                timeout_sec=self._subprocess_timeout_sec,
                on_line=lambda line: self._emit(goal_handle, feedback, "detecting_boxes"))

            self._emit(goal_handle, feedback, "reading_results")
            json_path = self._latest_boxes_json()
            json_text = json_path.read_text(encoding="utf-8")
            import json as _json
            box_count = int(_json.loads(json_text).get("box_count", 0))
            ply_path = json_path.parent / _json.loads(json_text)["completed_ply_file"]
            ply_data = pipeline_runner.convert_ply_double_to_float32(ply_path.read_bytes())

            chunk_size = int(self.get_parameter("chunk_size_bytes").value)
            total_chunks = max(1, math.ceil(len(ply_data) / chunk_size))
            self._emit(goal_handle, feedback, "sending_chunks", total_chunks=total_chunks)
            for i in range(total_chunks):
                chunk = ply_data[i * chunk_size:(i + 1) * chunk_size]
                self._emit(goal_handle, feedback, "sending_chunks", i, total_chunks, chunk)

            # [2026-07-28] "원본" 뷰용 - 99.py가 저장한 박스 검출 전 병합 포인트클라우드.
            # multiview_scan.py를 거친 위 ply_data(박스별 재구성 결과)와 달리 원본
            # 그대로다.
            self._emit(goal_handle, feedback, "reading_raw_ply")
            raw_npy_path = self._cart2trunk_dir / "perception" / "scan_cache" / "merged_cart_scan.npy"
            raw_points = np.load(raw_npy_path)
            raw_ply_data = pipeline_runner.write_binary_ply_from_xyz(raw_points)

            raw_total_chunks = max(1, math.ceil(len(raw_ply_data) / chunk_size))
            self._emit(goal_handle, feedback, "sending_raw_chunks", total_chunks=raw_total_chunks)
            for i in range(raw_total_chunks):
                chunk = raw_ply_data[i * chunk_size:(i + 1) * chunk_size]
                self._emit(goal_handle, feedback, "sending_raw_chunks", i, raw_total_chunks, chunk)

        except pipeline_runner.PipelineStageError as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            return CartScan.Result(success=False, message=str(e))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"예상치 못한 오류: {e}")
            goal_handle.abort()
            return CartScan.Result(success=False, message=f"예상치 못한 오류: {e}")

        self._emit(goal_handle, feedback, "done", total_chunks=raw_total_chunks)
        goal_handle.succeed()
        import datetime
        raw_filename = f"cart_scan_raw_{datetime.datetime.now():%Y%m%d_%H%M%S_%f}.ply"
        return CartScan.Result(
            success=True, message="카트 스캔 완료", box_count=box_count,
            json_filename=json_path.name, json_text=json_text,
            ply_filename=ply_path.name, ply_total_bytes=len(ply_data),
            ply_total_chunks=total_chunks,
            raw_ply_filename=raw_filename, raw_ply_total_bytes=len(raw_ply_data),
            raw_ply_total_chunks=raw_total_chunks,
        )


def main(args=None):
    rclpy.init(args=args)
    node = CartScanActionServer()
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
