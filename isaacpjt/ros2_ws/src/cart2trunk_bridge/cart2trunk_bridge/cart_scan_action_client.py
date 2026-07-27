"""cart_scan_action_client.py

UI PC에서 `ros2 run cart2trunk_bridge cart_scan_client`로 실행되는 CLI. Flask
백엔드(`web/backend/robot_bridge.py`)가 서브프로세스로 이 스크립트를 호출한다.
`/cart2trunk/cart_scan` 액션을 호출해서 PLY 청크를 모두 모으고, JSON(Result에
통째로 옴)과 함께 파일로 저장한 뒤, 결과를 JSON 한 줄로 stdout에 출력한다.

[2026-07-28] 박스 재구성 PLY(sending_chunks)와 원본 병합 PLY(sending_raw_chunks)
두 스트림을 stage로 구분해서 따로 모은다 - "원본"/"전처리" UI 토글용.

trunk_scan_action_client.py와 동일한 패턴 - 성공/실패는 종료 코드(0/1)와
stdout 마지막 줄의 JSON으로 판단한다.
"""
import argparse
import json
import pathlib
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

from cart2trunk_interfaces.action import CartScan

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
    parser.add_argument("--output-dir", required=True, help="수신한 JSON/PLY를 저장할 디렉터리")
    parser.add_argument("--timeout-sec", type=float, default=180.0, help="액션 전체 타임아웃(초)")
    parser.add_argument("--server-wait-sec", type=float, default=10.0, help="액션 서버 연결 대기(초)")
    parser.add_argument("--gui", action="store_true", help="헤드리스 대신 Isaac Sim GUI로 99.py 실행")
    parsed = parser.parse_args(args=args)

    rclpy.init()
    node = rclpy.create_node("cart_scan_action_client")
    client = ActionClient(
        node, CartScan, "/cart2trunk/cart_scan",
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
        _fail("액션 서버(/cart2trunk/cart_scan)에 연결할 수 없습니다 - Isaac Sim PC의 "
              "cart_scan_action_server가 켜져 있는지, ROS_DOMAIN_ID가 일치하는지 확인하세요")

    goal_msg = CartScan.Goal()
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
        _fail(f"결과 수신 타임아웃({parsed.timeout_sec}초) - 카트 스캔이 아직 진행 중일 수 있습니다")

    result = wrapped.result
    if not result.success:
        _fail(result.message or "카트 스캔 실패")

    ply_data = _assemble(chunks, result.ply_total_chunks, result.ply_total_bytes, "전처리")
    raw_ply_data = _assemble(raw_chunks, result.raw_ply_total_chunks, result.raw_ply_total_bytes, "원본")

    output_dir = pathlib.Path(parsed.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / result.ply_filename).write_bytes(ply_data)
    (output_dir / result.json_filename).write_text(result.json_text, encoding="utf-8")
    (output_dir / result.raw_ply_filename).write_bytes(raw_ply_data)

    print(json.dumps({
        "success": True,
        "box_count": result.box_count,
        "json_filename": result.json_filename,
        "ply_filename": result.ply_filename,
        "ply_total_bytes": result.ply_total_bytes,
        "ply_total_chunks": result.ply_total_chunks,
        "raw_ply_filename": result.raw_ply_filename,
        "raw_ply_total_bytes": result.raw_ply_total_bytes,
    }))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
