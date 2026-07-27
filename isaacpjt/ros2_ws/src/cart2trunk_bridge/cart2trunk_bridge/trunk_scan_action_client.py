"""trunk_scan_action_client.py

UI PC에서 `ros2 run cart2trunk_bridge trunk_scan_client`로 실행되는 CLI. Flask
백엔드(`web/backend/robot_bridge.py`)가 서브프로세스로 이 스크립트를 호출한다.
`/cart2trunk/trunk_scan` 액션을 호출해서 청크를 모두 모으고, 인덱스 누락이 없는지
검증한 뒤 파일로 저장하고, 결과를 JSON 한 줄로 stdout에 출력한다.

[2026-07-28] 전처리된 PLY(sending_chunks)와 원본 PLY(sending_raw_chunks) 두
스트림을 stage로 구분해서 따로 모은다 - "원본"/"전처리" UI 토글용.

성공/실패 여부는 종료 코드(0/1)와 stdout 마지막 줄의 JSON으로 판단한다.
"""
import argparse
import json
import pathlib
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

from cart2trunk_interfaces.action import TrunkScan

FEEDBACK_QOS_DEPTH = 100


def _fail(message):
    print(json.dumps({"success": False, "message": message}))
    sys.exit(1)


def _assemble(chunks, total_chunks, total_bytes, label):
    missing = [i for i in range(total_chunks) if i not in chunks]
    if missing:
        _fail(f"{label} 청크 유실: {len(missing)}/{total_chunks}개 누락 (인덱스 예: {missing[:5]})")
    data = b"".join(chunks[i] for i in range(total_chunks))
    if len(data) != total_bytes:
        _fail(f"{label} 수신 바이트 수 불일치: 기대={total_bytes} 실제={len(data)}")
    return data


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True, help="수신한 PLY를 저장할 디렉터리")
    parser.add_argument("--timeout-sec", type=float, default=180.0, help="액션 전체 타임아웃(초)")
    parser.add_argument("--server-wait-sec", type=float, default=10.0, help="액션 서버 연결 대기(초)")
    parser.add_argument("--gui", action="store_true", help="헤드리스 대신 Isaac Sim GUI로 89.py 실행")
    parsed = parser.parse_args(args=args)

    rclpy.init()
    node = rclpy.create_node("trunk_scan_action_client")
    client = ActionClient(
        node, TrunkScan, "/cart2trunk/trunk_scan",
        feedback_sub_qos_profile=QoSProfile(depth=FEEDBACK_QOS_DEPTH),
    )

    chunks = {}
    raw_chunks = {}

    def feedback_cb(feedback_msg):
        fb = feedback_msg.feedback
        if fb.stage == "sending_chunks" and len(fb.chunk_data) > 0:
            chunks[fb.chunk_index] = bytes(fb.chunk_data)
            node.get_logger().info(f"[청크 수신] {fb.chunk_index + 1}/{fb.total_chunks}")
        elif fb.stage == "sending_raw_chunks" and len(fb.chunk_data) > 0:
            raw_chunks[fb.chunk_index] = bytes(fb.chunk_data)
            node.get_logger().info(f"[원본 청크 수신] {fb.chunk_index + 1}/{fb.total_chunks}")
        elif fb.stage not in ("sending_chunks", "sending_raw_chunks"):
            node.get_logger().info(f"[진행] {fb.stage}")

    if not client.wait_for_server(timeout_sec=parsed.server_wait_sec):
        _fail("액션 서버(/cart2trunk/trunk_scan)에 연결할 수 없습니다 - Isaac Sim PC의 "
              "trunk_scan_action_server가 켜져 있는지, ROS_DOMAIN_ID가 일치하는지 확인하세요")

    goal_msg = TrunkScan.Goal()
    goal_msg.headless = not parsed.gui

    send_goal_future = client.send_goal_async(goal_msg, feedback_callback=feedback_cb)
    rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=parsed.timeout_sec)
    goal_handle = send_goal_future.result()
    if goal_handle is None or not goal_handle.accepted:
        _fail("액션 목표가 거부되었습니다")

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=parsed.timeout_sec)
    wrapped = result_future.result()
    if wrapped is None:
        _fail(f"결과 수신 타임아웃({parsed.timeout_sec}초) - 트렁크 스캔이 아직 진행 중일 수 있습니다")

    result = wrapped.result
    if not result.success:
        _fail(result.message or "트렁크 스캔 실패")

    data = _assemble(chunks, result.total_chunks, result.total_bytes, "전처리")
    raw_data = _assemble(raw_chunks, result.raw_total_chunks, result.raw_total_bytes, "원본")

    output_dir = pathlib.Path(parsed.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / result.filename
    out_path.write_bytes(data)
    raw_out_path = output_dir / result.raw_filename
    raw_out_path.write_bytes(raw_data)
    # cart_scan_action_client.py가 all_boxes_corners_*.json을 json_filename/
    # json_text로 받아 그대로 저장하는 것과 같은 패턴 - trunk_map.json도 청크가
    # 아니라 문자열 필드 하나로 통째로 왔으므로 그대로 파일에 쓰면 된다.
    trunk_map_out_path = output_dir / result.trunk_map_filename
    trunk_map_out_path.write_text(result.trunk_map_json, encoding="utf-8")

    print(json.dumps({
        "success": True,
        "filename": result.filename,
        "path": str(out_path),
        "total_bytes": result.total_bytes,
        "total_chunks": result.total_chunks,
        "point_count": result.point_count,
        "raw_filename": result.raw_filename,
        "raw_path": str(raw_out_path),
        "raw_total_bytes": result.raw_total_bytes,
        "raw_point_count": result.raw_point_count,
        "trunk_map_filename": result.trunk_map_filename,
        "trunk_map_path": str(trunk_map_out_path),
    }))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
