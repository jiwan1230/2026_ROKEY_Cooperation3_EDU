"""pick_and_place_action_server.py

Isaac Sim PC(MSI2)에서 실행하는 ROS2 액션 서버. trunk_scan_action_server.py/
cart_scan_action_server.py와 달리 89.py/99.py를 매번 새로 서브프로세스 부팅하지
않는다 - MSI2에서 이미 계속 떠있는 isaac_task_runner.py(cart_scan/trunk_scan/
pick_and_place를 100.py/89.py/99.py 코드 그대로 함수화해서 반복 트리거받는
단일 지속 세션)의 `/isaac_task_runner/run_pick_and_place` Trigger 서비스를
대신 호출하고, isaac_task_runner.py가 내는 `/isaac_task_runner/status` 토픽을
구독해서 Action Feedback으로 중계하는 얇은 어댑터다.

이 노드 자신은 Isaac Sim을 다루지 않는다 - isaac_task_runner.py가 이미
실행 중이어야 하고(별도 터미널), 이 노드는 그 프로세스에 명령만 넣고 진행
상황만 구독한다.
"""
import queue
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from cart2trunk_interfaces.action import PickAndPlace

STATUS_QOS_DEPTH = 20


class PickAndPlaceActionServer(Node):
    def __init__(self):
        super().__init__("pick_and_place_action_server")

        # isaac_task_runner.py가 하나의 작업만 순차 처리하는 구조(이미 처리 중이면
        # Trigger가 success=False로 즉시 거절)라, 여기 전체 타임아웃도 넉넉하게 잡는다 -
        # 2026-07-27 4박스 실측 기준 STAGE0~4 전체가 박스당 수 분, 4개면 15~20분 걸렸다.
        self.declare_parameter("overall_timeout_sec", 1800)
        self._timeout_sec = int(self.get_parameter("overall_timeout_sec").value)

        self._status_queue: "queue.Queue" = queue.Queue()
        self.create_subscription(
            String, "/isaac_task_runner/status", self._on_status, STATUS_QOS_DEPTH)

        self._trigger_client = self.create_client(
            Trigger, "/isaac_task_runner/run_pick_and_place",
            callback_group=ReentrantCallbackGroup())

        self._action_server = ActionServer(
            self, PickAndPlace, "/cart2trunk/pick_and_place",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(
            "pick_and_place_action_server 준비 완료 (/cart2trunk/pick_and_place) - "
            "isaac_task_runner.py가 켜져 있어야 실제로 동작함")

    def _on_status(self, msg: String):
        import json
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._status_queue.put(data)

    def execute_callback(self, goal_handle):
        feedback = PickAndPlace.Feedback()
        plan_id = goal_handle.request.plan_id

        # 큐를 비워서, 이전 goal이 흘려보낸 status(예: 직전 호출의 마지막 "done")가
        # 이번 goal 처리 중 잘못 섞여 들어오지 않게 한다.
        while not self._status_queue.empty():
            try:
                self._status_queue.get_nowait()
            except queue.Empty:
                break

        if not self._trigger_client.wait_for_service(timeout_sec=10.0):
            goal_handle.abort()
            return PickAndPlace.Result(
                success=False,
                message="isaac_task_runner.py의 /isaac_task_runner/run_pick_and_place "
                        "서비스에 연결할 수 없습니다 - Isaac Sim PC에서 isaac_task_runner.py가 "
                        "실행 중인지 확인하세요.")

        self.get_logger().info(f"[pick_and_place] plan_id={plan_id!r} 트리거 요청")
        trigger_future = self._trigger_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 10.0
        while not trigger_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not trigger_future.done() or trigger_future.result() is None:
            goal_handle.abort()
            return PickAndPlace.Result(success=False, message="run_pick_and_place 트리거 응답 타임아웃")

        trigger_resp = trigger_future.result()
        if not trigger_resp.success:
            # 예: "이미 'cart_scan' 처리 중" - isaac_task_runner.py가 이미 다른 작업
            # 중이라 이번 요청을 거절한 것. 즉시 실패로 끝낸다(재시도는 호출부 책임).
            goal_handle.abort()
            return PickAndPlace.Result(success=False, message=trigger_resp.message)

        box_count = 0
        boxes_done = 0
        deadline = time.monotonic() + self._timeout_sec
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return PickAndPlace.Result(
                    success=False, message="취소됨(참고: isaac_task_runner.py 자체는 아직 "
                    "실행 중인 STAGE를 즉시 멈추지 못함 - 로그로 실제 완료까지 지켜봐야 함)",
                    boxes_placed=boxes_done, boxes_total=box_count)
            try:
                data = self._status_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if data.get("task") != "pick_and_place":
                continue

            stage = data.get("stage", "")
            box_index = int(data.get("box_index", 0))
            box_count = max(box_count, int(data.get("box_count", 0)))
            box_id = data.get("box_id", "")

            feedback.stage = stage
            feedback.box_index = box_index
            feedback.box_count = box_count
            feedback.box_id = box_id
            goal_handle.publish_feedback(feedback)

            if stage == "box_done":
                boxes_done += 1
            elif stage == "done":
                goal_handle.succeed()
                return PickAndPlace.Result(
                    success=True, message="pick_and_place 완료",
                    boxes_placed=boxes_done, boxes_total=box_count)
            elif stage == "error":
                goal_handle.abort()
                return PickAndPlace.Result(
                    success=False, message=data.get("message", "pick_and_place 실패(원인 불명)"),
                    boxes_placed=boxes_done, boxes_total=box_count)

        goal_handle.abort()
        return PickAndPlace.Result(
            success=False,
            message=f"{self._timeout_sec}초 안에 완료/실패 신호를 받지 못했습니다 "
                    f"(isaac_task_runner.py 로그를 직접 확인하세요)",
            boxes_placed=boxes_done, boxes_total=box_count)


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceActionServer()
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
