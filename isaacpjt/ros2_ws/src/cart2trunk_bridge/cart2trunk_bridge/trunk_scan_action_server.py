"""trunk_scan_action_server.py

Isaac Sim PC에서 실행하는 ROS2 액션 서버. `/cart2trunk/trunk_scan` 액션 요청을
받으면 89.trunk_scan_holonomic.py -> 90.export_trunk_map_holonomic.py를 사람이
지금 쓰는 것과 동일한 방식(python.sh + LD_LIBRARY_PATH)으로 서브프로세스 실행하고,
결과 PLY를 float32로 재인코딩해서 청크 단위로 feedback을 통해 전송한다.

89.py/90.py 원본 파일은 이 노드에서 subprocess로만 다루며 절대 열어서 수정하지 않는다.
"""
import math
import pathlib

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile

from cart2trunk_interfaces.action import TrunkScan

from cart2trunk_bridge import pipeline_runner

FEEDBACK_QOS_DEPTH = 100


class TrunkScanActionServer(Node):
    def __init__(self):
        super().__init__("trunk_scan_action_server")

        self.declare_parameter("cart2trunk_dir", "")
        self.declare_parameter("isaac_python_sh", "")
        self.declare_parameter("ros2_bridge_ld_library_path", "")
        # 32768(32KiB) - 실측으로 확인한 안전한 청크 크기. loopback rmw_fastrtps_cpp
        # 기준 실험한 결과 feedback 메시지가 대략 165KB~240KB 구간에서 간헐적으로
        # 통째로 유실되는 현상을 확인함(청크 인덱스가 통째로 클라이언트에 도달하지
        # 않음, 재현: 32~150KB는 매번 성공, 165KB+는 실패 빈발 - UDP 프래그먼트
        # 유실로 추정). 65536까지도 반복 테스트에서 안정적이었지만 여유를 크게 둔다.
        self.declare_parameter("chunk_size_bytes", 32768)
        self.declare_parameter("subprocess_timeout_sec", 600)

        for name in ("cart2trunk_dir", "isaac_python_sh", "ros2_bridge_ld_library_path"):
            if not self.get_parameter(name).value:
                raise RuntimeError(
                    f"필수 파라미터 '{name}'가 설정되지 않았습니다. "
                    f"config/trunk_scan_server.params.yaml을 이 머신에 맞게 채워서 "
                    f"--ros-args --params-file 로 넘겨주세요.")

        self._cart2trunk_dir = pathlib.Path(self.get_parameter("cart2trunk_dir").value)
        self._isaac_python_sh = self.get_parameter("isaac_python_sh").value
        self._ros2_bridge_ld = self.get_parameter("ros2_bridge_ld_library_path").value
        self._timeout_sec = int(self.get_parameter("subprocess_timeout_sec").value)

        self._action_server = ActionServer(
            self, TrunkScan, "/cart2trunk/trunk_scan",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            feedback_pub_qos_profile=QoSProfile(depth=FEEDBACK_QOS_DEPTH),
        )
        self.get_logger().info("trunk_scan_action_server 준비 완료 (/cart2trunk/trunk_scan)")

    def _emit(self, goal_handle, feedback, stage, chunk_index=0, total_chunks=0, chunk_data=b""):
        feedback.stage = stage
        feedback.chunk_index = chunk_index
        feedback.total_chunks = total_chunks
        feedback.chunk_data = chunk_data
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(f"[진행] {stage}")

    def execute_callback(self, goal_handle):
        feedback = TrunkScan.Feedback()
        headless = goal_handle.request.headless
        cwd = str(self._cart2trunk_dir)

        try:
            self._emit(goal_handle, feedback, "launching_89")
            env89 = {
                "HEADLESS": "1" if headless else "0",
                "LD_LIBRARY_PATH": self._ros2_bridge_ld,
            }
            pipeline_runner.run_stage(
                [self._isaac_python_sh, "89.trunk_scan_holonomic.py"],
                cwd=cwd, extra_env=env89, timeout_sec=self._timeout_sec,
                on_line=lambda line: self._emit(goal_handle, feedback, "scanning"))

            self._emit(goal_handle, feedback, "launching_90")
            pipeline_runner.run_stage(
                ["python3", "90.export_trunk_map_holonomic.py"],
                cwd=cwd, timeout_sec=self._timeout_sec,
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

        except pipeline_runner.PipelineStageError as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            return TrunkScan.Result(success=False, message=str(e))
        except Exception as e:  # noqa: BLE001 - 액션 서버가 죽지 않도록 광범위 예외 처리
            self.get_logger().error(f"예상치 못한 오류: {e}")
            goal_handle.abort()
            return TrunkScan.Result(success=False, message=f"예상치 못한 오류: {e}")

        self._emit(goal_handle, feedback, "done", total_chunks=total_chunks)
        goal_handle.succeed()
        import datetime
        filename = f"trunk_scan_{datetime.datetime.now():%Y%m%d_%H%M%S_%f}.ply"
        return TrunkScan.Result(
            success=True, message="트렁크 스캔 완료",
            filename=filename, total_bytes=len(data),
            total_chunks=total_chunks, point_count=point_count,
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
