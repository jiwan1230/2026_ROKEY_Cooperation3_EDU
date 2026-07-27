"""cart_scan_action_server.py

Isaac Sim PC에서 실행하는 ROS2 액션 서버. `/cart2trunk/cart_scan` 액션 요청을
받으면 99.cart_scan_dual_side_holonomic.py -> perception/multiview_scan.py를
서브프로세스로 실행하고, 검출된 박스 JSON(전체 텍스트)과 통합 PLY(청크)를
UI PC로 전송한다.

99.py/multiview_scan.py 원본 파일은 이 노드에서 subprocess로만 다루며 절대
열어서 수정하지 않는다 - trunk_scan_action_server.py와 동일한 원칙.
"""
import math
import pathlib

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile

from cart2trunk_interfaces.action import CartScan

from cart2trunk_bridge import pipeline_runner

FEEDBACK_QOS_DEPTH = 100


class CartScanActionServer(Node):
    def __init__(self):
        super().__init__("cart_scan_action_server")

        self.declare_parameter("cart2trunk_dir", "")
        self.declare_parameter("isaac_python_sh", "")
        self.declare_parameter("ros2_bridge_ld_library_path", "")
        self.declare_parameter("perception_venv_python", "")
        # TrunkScan과 동일한 이유(32KB) - trunk_scan_action_server.py 주석 참고.
        self.declare_parameter("chunk_size_bytes", 32768)
        self.declare_parameter("subprocess_timeout_sec", 600)

        for name in ("cart2trunk_dir", "isaac_python_sh",
                     "ros2_bridge_ld_library_path", "perception_venv_python"):
            if not self.get_parameter(name).value:
                raise RuntimeError(
                    f"필수 파라미터 '{name}'가 설정되지 않았습니다. "
                    f"config/cart_scan_server.params.yaml을 이 머신에 맞게 채워서 "
                    f"--ros-args --params-file 로 넘겨주세요.")

        self._cart2trunk_dir = pathlib.Path(self.get_parameter("cart2trunk_dir").value)
        self._isaac_python_sh = self.get_parameter("isaac_python_sh").value
        self._ros2_bridge_ld = self.get_parameter("ros2_bridge_ld_library_path").value
        self._perception_venv_python = self.get_parameter("perception_venv_python").value
        self._timeout_sec = int(self.get_parameter("subprocess_timeout_sec").value)
        # multiview_scan.py의 SAVE_DIRECTORY(= Path.home()/"box_pointcloud")와
        # 동일한 규칙 - 그 스크립트 자체가 경로를 인자로 안 받으므로 그대로 맞춘다.
        self._box_pointcloud_dir = pathlib.Path.home() / "box_pointcloud"

        self._action_server = ActionServer(
            self, CartScan, "/cart2trunk/cart_scan",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            feedback_pub_qos_profile=QoSProfile(depth=FEEDBACK_QOS_DEPTH),
        )
        self.get_logger().info("cart_scan_action_server 준비 완료 (/cart2trunk/cart_scan)")

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

    def execute_callback(self, goal_handle):
        feedback = CartScan.Feedback()
        headless = goal_handle.request.headless
        cwd = str(self._cart2trunk_dir)

        try:
            self._emit(goal_handle, feedback, "launching_99")
            env99 = {
                "HEADLESS": "1" if headless else "0",
                "LD_LIBRARY_PATH": self._ros2_bridge_ld,
            }
            pipeline_runner.run_stage(
                [self._isaac_python_sh, "99.cart_scan_dual_side_holonomic.py"],
                cwd=cwd, extra_env=env99, timeout_sec=self._timeout_sec,
                on_line=lambda line: self._emit(goal_handle, feedback, "scanning_cart"))

            self._emit(goal_handle, feedback, "launching_multiview_scan")
            npy_path = self._cart2trunk_dir / "perception" / "scan_cache" / "merged_cart_scan.npy"
            marker_path = self._cart2trunk_dir / "perception" / "scan_cache" / "multiview_scan.marker"
            pipeline_runner.run_stage(
                [self._perception_venv_python, "multiview_scan.py",
                 "--input", str(npy_path), "--marker", str(marker_path)],
                cwd=str(self._cart2trunk_dir / "perception"), timeout_sec=self._timeout_sec,
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

        except pipeline_runner.PipelineStageError as e:
            self.get_logger().error(str(e))
            goal_handle.abort()
            return CartScan.Result(success=False, message=str(e))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"예상치 못한 오류: {e}")
            goal_handle.abort()
            return CartScan.Result(success=False, message=f"예상치 못한 오류: {e}")

        self._emit(goal_handle, feedback, "done", total_chunks=total_chunks)
        goal_handle.succeed()
        return CartScan.Result(
            success=True, message="카트 스캔 완료", box_count=box_count,
            json_filename=json_path.name, json_text=json_text,
            ply_filename=ply_path.name, ply_total_bytes=len(ply_data),
            ply_total_chunks=total_chunks,
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
