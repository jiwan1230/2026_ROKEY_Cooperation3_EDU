"""
web/ros_bridge/send_trunk_scan_result.py

아이작심 컴퓨터에서, 89번(트렁크 스캔) + 90번(trunk_map.json/PLY 변환)을 다
실행한 뒤에 이 스크립트 하나만 실행하면 된다. 결과 파일 2개를 읽어서 UI
컴퓨터로 보낸다 - 그게 전부다.

    python3 web/ros_bridge/send_trunk_scan_result.py

(ROS2가 켜져 있는 터미널이어야 함 - 89번 돌릴 때 이미 source /opt/ros/humble/
setup.bash 했으면 같은 터미널에서 그대로 실행하면 됨)

89/90번 파일은 이 스크립트가 전혀 안 건드린다 - 그냥 결과 파일만 읽어서 보낸다.
"""
import base64
import pathlib
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

_THIS_DIR = pathlib.Path(__file__).resolve().parent
_CART2TRUNK_DIR = _THIS_DIR.parent.parent
RESULT_DIR = _CART2TRUNK_DIR / "results" / "holonomic_base"
TRUNK_MAP_PATH = RESULT_DIR / "trunk_map.json"
TRUNK_PLY_PATH = RESULT_DIR / "trunk_pointcloud_filtered_base.ply"

TRUNK_MAP_TOPIC = "/cart2trunk/trunk_map_json"
TRUNK_PLY_TOPIC = "/cart2trunk/trunk_scan_ply"


def main():
    if not TRUNK_MAP_PATH.exists():
        print(f"[에러] {TRUNK_MAP_PATH} 파일이 없습니다. 90번(export_trunk_map_holonomic.py)을 먼저 끝까지 돌렸는지 확인하세요.")
        sys.exit(1)
    if not TRUNK_PLY_PATH.exists():
        print(f"[에러] {TRUNK_PLY_PATH} 파일이 없습니다. 90번(export_trunk_map_holonomic.py)을 먼저 끝까지 돌렸는지 확인하세요.")
        sys.exit(1)

    trunk_map_text = TRUNK_MAP_PATH.read_text()
    ply_bytes = TRUNK_PLY_PATH.read_bytes()
    print(f"[읽기 완료] trunk_map.json ({len(trunk_map_text)}자), PLY ({len(ply_bytes)} bytes)")

    rclpy.init()
    node = Node("send_trunk_scan_result")
    pub_map = node.create_publisher(String, TRUNK_MAP_TOPIC, 10)
    pub_ply = node.create_publisher(String, TRUNK_PLY_TOPIC, 10)

    # UI 컴퓨터의 scan_bridge.py가 이 노드를 "발견"할 시간을 좀 준다 -
    # 너무 빨리 publish하면 구독자가 아직 연결 전이라 못 받을 수 있음.
    print("[대기] UI 컴퓨터와 연결 확인 중(3초)...")
    time.sleep(3.0)

    msg_map = String(); msg_map.data = trunk_map_text
    msg_ply = String(); msg_ply.data = base64.b64encode(ply_bytes).decode("ascii")

    # 한 번만 보내면 구독자가 그 순간 안 듣고 있을 수도 있어서, 확실히
    # 받도록 5번 반복 발행한다 (내용은 매번 동일 - 여러 번 받아도 UI 쪽은
    # 마지막 걸로 덮어쓰기만 해서 문제없음).
    for i in range(5):
        pub_map.publish(msg_map)
        pub_ply.publish(msg_ply)
        rclpy.spin_once(node, timeout_sec=0.3)
        time.sleep(0.3)
        print(f"  전송 {i + 1}/5")

    node.destroy_node()
    rclpy.shutdown()
    print("[완료] UI 컴퓨터 쪽 화면(트렁크 스캔 뷰어)을 확인해보세요.")


if __name__ == "__main__":
    main()
