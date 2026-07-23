"""
test_09_rescan_replan_stacking.py
⑨ replan_after_rescan()에도 allow_stacking이 전달되는지 검증 - 이게 실제로
trunk_map_planner_node.py(ROS2, 로봇 결합 경로)가 호출하는 함수라 08과
별개로 여기도 지원해야 한다 (mode/margin 때와 같은 이유).
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


def test_default_allow_stacking_false_unchanged_behavior():
    trunk = Trunk(width=0.32, depth=0.32, height=0.9)
    boxes = [Box("Floor1", 0.28, 0.28, 0.2), Box("Floor2", 0.28, 0.28, 0.2)]

    plans_default, _ = replan_after_rescan(boxes, trunk, [])
    plans_explicit, _ = replan_after_rescan(boxes, trunk, [], allow_stacking=False)

    assert [p.position for p in plans_default] == [p.position for p in plans_explicit]
    assert len(plans_default) == 1  # 1층 전용이라 하나만 들어감


def test_allow_stacking_true_stacks_when_floor_is_full():
    trunk = Trunk(width=0.32, depth=0.32, height=0.9)
    boxes = [Box("Floor1", 0.28, 0.28, 0.2), Box("Floor2", 0.28, 0.28, 0.2)]

    plans, unloadable = replan_after_rescan(boxes, trunk, [], allow_stacking=True)

    assert len(unloadable) == 0
    assert len(plans) == 2
    z_values = sorted(p.position[2] for p in plans)
    assert z_values[0] == 0.0
    assert z_values[1] > 0.0
