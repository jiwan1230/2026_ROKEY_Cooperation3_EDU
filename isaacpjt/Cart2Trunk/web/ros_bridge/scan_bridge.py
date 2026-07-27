"""
web/ros_bridge/scan_bridge.py

UI 컴퓨터에서 실행하는 독립 ROS2 노드 - 아이작심 컴퓨터가 보내는 트렁크 스캔
결과(trunk_map.json + PLY)를 받아서 웹 UI가 바로 쓸 수 있는 위치에 저장한다.

Flask 백엔드(web/backend/)와는 완전히 별도 프로세스로 실행한다 - rclpy를
백엔드 전용 venv에 안 섞으려는 algorism_bridge.py와 같은 이유
(web/backend/algorism_bridge.py 상단 주석 참고). 89/90번, algorism_bridge.py
등 기존 파일은 이 작업에서 전혀 건드리지 않았다.

실행 방법 (ROS2 환경이 소싱된 터미널에서):
    source /opt/ros/humble/setup.bash
    python3 web/ros_bridge/scan_bridge.py

토픽 스펙 (아이작심 컴퓨터 쪽 publisher와 반드시 동일해야 함 - 팀에 공유 필요):
    /cart2trunk/trunk_map_json  (std_msgs/String) - trunk_map.json 텍스트 그대로
    /cart2trunk/trunk_scan_ply  (std_msgs/String) - PLY 파일 바이트를 base64로 인코딩한 문자열
지금은 std_msgs/String + base64로 가장 단순하게 시작 - "일단 되는지" 확인 후
필요하면 커스텀 메시지나 sensor_msgs/PointCloud2로 바꿀 수 있다.
"""
import base64
import datetime
import pathlib

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

_THIS_DIR = pathlib.Path(__file__).resolve().parent
_WEB_DIR = _THIS_DIR.parent
_CART2TRUNK_DIR = _WEB_DIR.parent
# algorism_bridge.py의 _SRC_DIR 계산과 동일한 관례 - 계정명과 무관하게
# 레포 위치 기준 상대 경로로 ROS2 워크스페이스 src/를 찾는다.
_SRC_DIR = _CART2TRUNK_DIR.parent.parent.parent
_FRONTEND_PUBLIC_DIR = _WEB_DIR / "frontend" / "public"

TRUNK_MAP_TOPIC = "/cart2trunk/trunk_map_json"
TRUNK_PLY_TOPIC = "/cart2trunk/trunk_scan_ply"


class ScanBridge(Node):
    def __init__(self):
        super().__init__("scan_bridge")
        _FRONTEND_PUBLIC_DIR.mkdir(parents=True, exist_ok=True)
        self.create_subscription(String, TRUNK_MAP_TOPIC, self.on_trunk_map, 10)
        self.create_subscription(String, TRUNK_PLY_TOPIC, self.on_trunk_ply, 10)
        self.get_logger().info(f"scan_bridge 시작 - 구독 중: {TRUNK_MAP_TOPIC}, {TRUNK_PLY_TOPIC}")

    def on_trunk_map(self, msg: String) -> None:
        # algorism_bridge.list_trunk_maps()가 찾는 "run_*/pointcloud/trunk_map.json"
        # 관례를 그대로 재사용 - 새 백엔드 라우트 없이 UI 드롭다운에 자동으로 뜬다.
        run_id = f"run_{datetime.datetime.now():%Y%m%d_%H%M%S}"
        out_dir = _SRC_DIR / run_id / "pointcloud"
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / "trunk_map.json"
        out_path.write_text(msg.data)
        self.get_logger().info(f"[trunk_map] 저장 완료: {out_path}")

    def on_trunk_ply(self, msg: String) -> None:
        ply_bytes = base64.b64decode(msg.data)
        out_path = _FRONTEND_PUBLIC_DIR / "trunk_pointcloud_filtered_base.ply"
        out_path.write_bytes(ply_bytes)
        self.get_logger().info(f"[trunk_ply] 저장 완료: {out_path} ({len(ply_bytes)} bytes)")


def main():
    rclpy.init()
    node = ScanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
