"""
test_09_rescan_replan_extended_params.py
⑨ replan_after_rescan()에도 08과 동일한 확장 파라미터(allow_rotation,
wall_margin/obstacle_margin/ceiling_margin, entrance_preference/
contact_preference)가 그대로 관통되는지 확인 - trunk_map_planner_node.py
(ROS2, 로봇 결합 경로)가 실제로 호출하는 게 08이 아니라 이 함수라서 여기도
지원해야 한다 (mode/margin/allow_stacking 때와 같은 이유).
"""
import sys, pathlib
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # tests/ -> algorism/
_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
_m09 = import_module("09_rescan_replan")

Trunk = _m02.Trunk
Box = _m03.Box
replan_after_rescan = _m09.replan_after_rescan


def test_allow_rotation_false_is_threaded_through():
    trunk = Trunk(width=0.6, depth=0.73, height=0.5)
    boxes = [Box("Wide", width=0.65, depth=0.30, height=0.15)]

    plans_default, _ = replan_after_rescan(boxes, trunk, [])
    assert len(plans_default) == 1 and plans_default[0].rotated is True

    plans_no_rotate, unloadable = replan_after_rescan(boxes, trunk, [], allow_rotation=False)
    assert len(plans_no_rotate) == 0
    assert len(unloadable) == 1


def test_ceiling_margin_is_threaded_through():
    trunk = Trunk(width=1.0, depth=1.0, height=0.35)
    boxes = [Box("A", width=0.2, depth=0.2, height=0.2)]

    plans_default, _ = replan_after_rescan(boxes, trunk, [])
    assert len(plans_default) == 0

    plans_relaxed, _ = replan_after_rescan(boxes, trunk, [], ceiling_margin=0.10)
    assert len(plans_relaxed) == 1


def test_entrance_preference_changes_positions():
    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    boxes = [Box(f"B{i}", width=0.2, depth=0.2, height=0.15) for i in range(3)]

    plans_default, _ = replan_after_rescan(boxes, trunk, [])
    plans_entrance_first, _ = replan_after_rescan(boxes, trunk, [], entrance_preference=-1.0)

    pos_default = {p.box_id: p.position for p in plans_default}
    pos_entrance_first = {p.box_id: p.position for p in plans_entrance_first}
    assert pos_default != pos_entrance_first
