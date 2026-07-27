"""send_placement_plan_client.py

UI PC(MSI1)에서 `ros2 run cart2trunk_bridge send_placement_plan_client`로
실행되는 CLI. Flask 백엔드(`web/backend/robot_bridge.py`)가 서브프로세스로
이 스크립트를 호출한다. `--json-file`로 받은 파일(승인된 Task JSON,
algorism/20_task_export.build_task_json() 출력)을 읽어서
`/cart2trunk/send_placement_plan` 서비스를 호출하고, 결과를 JSON 한 줄로
stdout에 출력한다.

JSON을 커맨드라인 인자로 직접 안 받고 파일로 받는 이유: Task JSON 안에
따옴표/줄바꿈이 섞여 있을 수 있어서 셸 이스케이핑이 위험하다 - robot_bridge.py가
임시 파일에 써놓고 그 경로만 넘긴다.

성공/실패 여부는 종료 코드(0/1)와 stdout 마지막 줄의 JSON으로 판단한다.
"""
import argparse
import json
import pathlib
import sys

import rclpy

from cart2trunk_interfaces.srv import SendPlacementPlan


def _fail(message):
    print(json.dumps({"success": False, "message": message}))
    sys.exit(1)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--json-file", required=True, help="전송할 Task JSON 파일 경로")
    parser.add_argument("--timeout-sec", type=float, default=30.0, help="서비스 응답 대기(초)")
    parser.add_argument("--server-wait-sec", type=float, default=10.0, help="서비스 연결 대기(초)")
    parsed = parser.parse_args(args=args)

    task_json_text = pathlib.Path(parsed.json_file).read_text(encoding="utf-8")

    rclpy.init()
    node = rclpy.create_node("send_placement_plan_client")
    client = node.create_client(SendPlacementPlan, "/cart2trunk/send_placement_plan")

    if not client.wait_for_service(timeout_sec=parsed.server_wait_sec):
        _fail("서비스(/cart2trunk/send_placement_plan)에 연결할 수 없습니다 - Isaac Sim PC의 "
              "send_placement_plan_service가 켜져 있는지, ROS_DOMAIN_ID가 일치하는지 확인하세요")

    request = SendPlacementPlan.Request()
    request.task_json = task_json_text
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=parsed.timeout_sec)
    result = future.result()
    if result is None:
        _fail(f"응답 타임아웃({parsed.timeout_sec}초)")

    if not result.success:
        _fail(result.message or "적재 계획 전송 실패")

    print(json.dumps({
        "success": True,
        "message": result.message,
        "written_path": result.written_path,
    }))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
