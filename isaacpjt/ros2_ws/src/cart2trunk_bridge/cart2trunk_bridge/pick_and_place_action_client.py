"""pick_and_place_action_client.py

UI PC(MSI1)에서 `ros2 run cart2trunk_bridge pick_and_place_client`로 실행되는
CLI. Flask 백엔드(`web/backend/robot_bridge.py`)가 서브프로세스로 이 스크립트를
호출한다. `/cart2trunk/pick_and_place` 액션을 호출해서 끝날 때까지 기다리고,
결과를 JSON 한 줄로 stdout에 출력한다.

trunk_scan_client/cart_scan_client와 달리 받을 PLY가 없다 - 그냥 완료를
기다렸다가 성공/실패와 박스 개수만 돌려준다.

성공/실패 여부는 종료 코드(0/1)와 stdout 마지막 줄의 JSON으로 판단한다.
"""
import argparse
import json
import sys

import rclpy
from rclpy.action import ActionClient

from cart2trunk_interfaces.action import PickAndPlace


def _fail(message):
    print(json.dumps({"success": False, "message": message}))
    sys.exit(1)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan-id", default="", help="실행할 계획 id(현재는 정보 제공/로그용)")
    parser.add_argument("--timeout-sec", type=float, default=1800.0,
                         help="액션 전체 타임아웃(초) - 4박스 기준 실측 15~20분 소요")
    parser.add_argument("--server-wait-sec", type=float, default=10.0, help="액션 서버 연결 대기(초)")
    parsed = parser.parse_args(args=args)

    rclpy.init()
    node = rclpy.create_node("pick_and_place_action_client")
    client = ActionClient(node, PickAndPlace, "/cart2trunk/pick_and_place")

    def feedback_cb(feedback_msg):
        fb = feedback_msg.feedback
        node.get_logger().info(
            f"[진행] {fb.stage} box={fb.box_index + 1}/{fb.box_count} id={fb.box_id}"
            if fb.box_count else f"[진행] {fb.stage}")

    if not client.wait_for_server(timeout_sec=parsed.server_wait_sec):
        _fail("액션 서버(/cart2trunk/pick_and_place)에 연결할 수 없습니다 - Isaac Sim PC의 "
              "pick_and_place_action_server가 켜져 있는지, ROS_DOMAIN_ID가 일치하는지 확인하세요")

    goal_msg = PickAndPlace.Goal()
    goal_msg.plan_id = parsed.plan_id

    send_goal_future = client.send_goal_async(goal_msg, feedback_callback=feedback_cb)
    rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=parsed.timeout_sec)
    goal_handle = send_goal_future.result()
    if goal_handle is None or not goal_handle.accepted:
        _fail("액션 목표가 거부되었습니다")

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=parsed.timeout_sec)
    wrapped = result_future.result()
    if wrapped is None:
        _fail(f"결과 수신 타임아웃({parsed.timeout_sec}초) - pick_and_place가 아직 진행 중일 수 있습니다")

    result = wrapped.result
    if not result.success:
        _fail(result.message or "pick_and_place 실패")

    print(json.dumps({
        "success": True,
        "message": result.message,
        "boxes_placed": result.boxes_placed,
        "boxes_total": result.boxes_total,
    }))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
