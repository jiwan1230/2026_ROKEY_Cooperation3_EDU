"""
test_09_rescan_replan_order_numbering.py
버그 리포트(사용자): "전체 순번이 왜 있는지 모르겠어, 장애물의 순번은
필요없어."

원인: _run_strategy()가 order_counter를 len(rescanned_placed_boxes)+1로
시작하는데, plan_from_trunk_map_data()가 실제로 넘기는 rescanned_placed_boxes
는 (지금 파이프라인엔 재스캔 연속성이 아직 안 붙어서) 사실상 항상 트렁크
장애물(휠하우스 등)뿐이다 - 장애물은 로봇이 "실은" 게 아니라 원래 거기 있던
고정 구조물인데, 순번을 그만큼 밀어버려서 첫 카트 박스가 order=1이 아니라
order=(장애물 개수+1)로 나온다. Task JSON의 sequence 필드에도 그대로 나가서
MSI2 쪽에도 잘못된 순번이 전달될 뻔했다.

수정: order_counter는 "실제로 이미 놓인 카트 박스"(is_obstacle=False)만
세야 한다 - 장애물(is_obstacle=True)은 순번에서 제외.
"""
import sys, pathlib
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # tests/ -> algorism/
_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
_m09 = import_module("09_rescan_replan")

Trunk = _m02.Trunk
Box = _m03.Box
PlacedBox = _m03.PlacedBox
replan_after_rescan = _m09.replan_after_rescan


def test_obstacles_do_not_consume_order_numbers():
    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    obstacles = [
        PlacedBox(box=Box("Wheel_Front", 0.2, 0.2, 0.2, is_obstacle=True), x=0.0, y=0.0, z=0.0),
        PlacedBox(box=Box("Wheel_Rear", 0.2, 0.2, 0.2, is_obstacle=True), x=0.0, y=0.7, z=0.0),
    ]
    boxes = [Box("A", 0.2, 0.2, 0.15)]

    plans, _ = replan_after_rescan(boxes, trunk, obstacles)

    assert len(plans) == 1
    assert plans[0].order == 1  # 장애물 2개가 있어도 첫 카트 박스는 order=1이어야 함


def test_previously_placed_cart_boxes_still_continue_the_sequence():
    """장애물과 달리, 실제로 이미 놓인 카트 박스(is_obstacle=False)는 여전히
    순번을 이어받아야 한다 - 재스캔 연속성이 실제로 붙었을 때를 위한 대비."""
    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    already_placed = [
        PlacedBox(box=Box("Obstacle", 0.15, 0.15, 0.15, is_obstacle=True), x=0.0, y=0.0, z=0.0),
        PlacedBox(box=Box("PrevCartBox", 0.2, 0.2, 0.15, is_obstacle=False), x=0.5, y=0.0, z=0.0),
    ]
    boxes = [Box("A", 0.2, 0.2, 0.15)]

    plans, _ = replan_after_rescan(boxes, trunk, already_placed)

    assert len(plans) == 1
    assert plans[0].order == 2  # 장애물 1개는 제외, 카트 박스 1개 다음이라 order=2
