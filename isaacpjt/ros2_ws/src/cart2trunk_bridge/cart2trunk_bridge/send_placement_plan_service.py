"""send_placement_plan_service.py

Isaac Sim PC(MSI2)에서 실행하는 ROS2 서비스 노드. `/cart2trunk/send_placement_plan`
요청으로 승인된 적재 계획(JSON 문자열, algorism/20_task_export.build_task_json()
출력 그대로)을 받아서, isaac_task_runner.py의 run_pick_and_place()가 다음 호출
때 그대로 읽는 파일(results/holonomic_base/placement_result.json)에 덮어쓴다.

isaac_task_runner.py 자체와는 별개의 가벼운 노드다 - Isaac Sim 세션이 켜져
있든 꺼져있든 이 노드는 그냥 파일 하나를 쓸 뿐이라, 계획을 먼저 받아두고
나중에 isaac_task_runner.py를 켜서 픽앤플레이스를 실행해도 된다. 20_task_export.py
스키마(placements/position_base_frame/dimensions 등)와 isaac_task_runner.py가
실제로 읽는 스키마가 이미 동일하게 맞춰져 있어서(2026-07-28) 변환 없이 그대로
써넣기만 한다 - approved=False인 계획은 거부한다("승인 전엔 MSI2로 전달 안 함"
원칙, algorism_bridge.send_task()/trunk_map_planner_node._send_task_to_msi2()와
동일).
"""
import json
import pathlib

import rclpy
from rclpy.node import Node

from cart2trunk_interfaces.srv import SendPlacementPlan


class SendPlacementPlanService(Node):
    def __init__(self):
        super().__init__("send_placement_plan_service")

        self.declare_parameter("cart2trunk_dir", "")
        if not self.get_parameter("cart2trunk_dir").value:
            raise RuntimeError(
                "필수 파라미터 'cart2trunk_dir'가 설정되지 않았습니다 - "
                "--ros-args -p cart2trunk_dir:=<이 머신의 Cart2Trunk 절대경로> 로 넘겨주세요.")

        self._cart2trunk_dir = pathlib.Path(self.get_parameter("cart2trunk_dir").value)
        # 100.py/isaac_task_runner.py의 PLACEMENT_JSON = OUT_DIR / "placement_result.json"
        # (OUT_DIR = <cart2trunk_dir>/results/holonomic_base)와 정확히 같은 경로.
        self._out_path = self._cart2trunk_dir / "results" / "holonomic_base" / "placement_result.json"

        self.create_service(SendPlacementPlan, "/cart2trunk/send_placement_plan", self._handle)
        self.get_logger().info(
            f"send_placement_plan_service 준비 완료 (/cart2trunk/send_placement_plan) - "
            f"쓰기 대상: {self._out_path}")

    def _handle(self, request, response):
        try:
            task = json.loads(request.task_json)
        except json.JSONDecodeError as e:
            response.success = False
            response.message = f"task_json 파싱 실패: {e}"
            return response

        if not task.get("approved", False):
            response.success = False
            response.message = "approved=False인 계획은 MSI2로 전달할 수 없습니다(승인 전 전달 금지 원칙)"
            return response

        if not isinstance(task.get("placements"), list) or not task["placements"]:
            response.success = False
            response.message = "task_json에 placements가 없습니다 - 빈 계획은 전송하지 않습니다."
            return response

        self._out_path.parent.mkdir(parents=True, exist_ok=True)
        self._out_path.write_text(json.dumps(task, ensure_ascii=False, indent=2), encoding="utf-8")

        self.get_logger().info(
            f"[적재 계획 수신] plan_id={task.get('plan_id')} "
            f"박스 {len(task['placements'])}개 -> {self._out_path}")
        response.success = True
        response.message = f"{len(task['placements'])}개 박스 배치 계획을 저장했습니다"
        response.written_path = str(self._out_path)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SendPlacementPlanService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
