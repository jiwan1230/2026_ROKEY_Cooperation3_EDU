"""
test_08_generate_loading_plan_stacking.py
⑧ generate_loading_plan()의 allow_stacking 매개변수 - 트렁크 1층이 꽉 찼을 때
자동으로 2층에 쌓을 수 있게 하는 스위치. 기본값 False(하위 호환 - 기존처럼
1층 전용). ⑤ 점수 기준이 원래 "낮은 자리 우선"이라, 켜놔도 바닥에 자리가
있으면 바닥부터 쓰고 꽉 찼을 때만 자동으로 위로 올라간다(별도의 "몇 층"
지정 없이도 동작 - 07_placement_plan_stacking.py에서 이미 검증된 동작을
그대로 파이프라인 레벨로 노출하는 것뿐).
"""
import sys, pathlib
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # tests/ -> algorism/
_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
_m08 = import_module("08_unloadable_reason")

Trunk = _m02.Trunk
Box = _m03.Box
generate_loading_plan = _m08.generate_loading_plan


def test_default_allow_stacking_false_unchanged_behavior():
    """기본값(allow_stacking 안 줌)은 기존과 완전히 동일하게 1층 전용으로 동작해야 한다."""
    trunk = Trunk(width=0.32, depth=0.32, height=0.9)
    boxes = [Box("Floor1", 0.28, 0.28, 0.2), Box("Floor2", 0.28, 0.28, 0.2)]

    plans_default, unloadable_default = generate_loading_plan(boxes, trunk)
    plans_explicit, unloadable_explicit = generate_loading_plan(boxes, trunk, allow_stacking=False)

    assert [p.box_id for p in plans_default] == [p.box_id for p in plans_explicit]
    assert [p.position for p in plans_default] == [p.position for p in plans_explicit]
    # 바닥 면적이 박스 하나만 들어가는 크기라, 1층 전용이면 둘째 박스는 못 들어가야 함
    assert len(plans_default) == 1
    assert len(unloadable_default) == 1


def test_allow_stacking_true_fills_floor_first_then_stacks_automatically():
    """
    쌓기를 켜면: 바닥에 자리가 있는 동안은 바닥부터 채우고, 바닥이 꽉 찬 뒤에야
    자동으로 2층에 올라가야 한다 - "몇 층"을 따로 지정하지 않아도 됨.
    """
    trunk = Trunk(width=0.32, depth=0.32, height=0.9)
    boxes = [Box("Floor1", 0.28, 0.28, 0.2), Box("Floor2", 0.28, 0.28, 0.2)]

    plans, unloadable = generate_loading_plan(boxes, trunk, allow_stacking=True)

    assert len(unloadable) == 0
    assert len(plans) == 2
    z_values = sorted(p.position[2] for p in plans)
    assert z_values[0] == 0.0, "첫 박스는 바닥(z=0)에 놓여야 함"
    assert z_values[1] > 0.0, "둘째 박스는 바닥이 꽉 차서 자동으로 위층(z>0)에 놓여야 함"
