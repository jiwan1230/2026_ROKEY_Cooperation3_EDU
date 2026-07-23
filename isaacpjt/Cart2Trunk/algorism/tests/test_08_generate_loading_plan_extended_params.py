"""
test_08_generate_loading_plan_extended_params.py
⑧ generate_loading_plan()에 "HMI 화면 설계 가이드라인" 문서의 나머지 파라미터
(allow_rotation, wall_margin/obstacle_margin/ceiling_margin,
entrance_preference/contact_preference)가 그대로 관통되는지 확인. 전부
기본값이면 지금까지와 완전히 동일(하위 호환).
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


def test_allow_rotation_false_is_threaded_through():
    trunk = Trunk(width=0.6, depth=0.73, height=0.5)
    boxes = [Box("Wide", width=0.65, depth=0.30, height=0.15)]

    plans_default, _ = generate_loading_plan(boxes, trunk)
    assert len(plans_default) == 1
    assert plans_default[0].rotated is True

    plans_no_rotate, unloadable = generate_loading_plan(boxes, trunk, allow_rotation=False)
    assert len(plans_no_rotate) == 0
    assert len(unloadable) == 1


def test_ceiling_margin_is_threaded_through():
    trunk = Trunk(width=1.0, depth=1.0, height=0.35)
    boxes = [Box("A", width=0.2, depth=0.2, height=0.2)]

    plans_default, unloadable_default = generate_loading_plan(boxes, trunk)
    assert len(plans_default) == 0
    assert len(unloadable_default) == 1

    plans_relaxed, _ = generate_loading_plan(boxes, trunk, ceiling_margin=0.10)
    assert len(plans_relaxed) == 1


def test_entrance_preference_changes_which_position_is_chosen():
    """entrance_preference=-1(입구 우선)이면 기본값(깊은 위치 우선)과 다른 자리를
    골라야 한다. 박스 1개짜리 빈 트렁크는 애초에 후보가 '가장 깊은 자리' 하나뿐이라
    (극점 알고리즘 특성상 첫 박스는 항상 그리로 감) 이 축이 갈릴 여지가 없어서,
    후속 박스들의 후보가 다양해지도록 3개로 검증한다."""
    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    boxes = [Box(f"B{i}", width=0.2, depth=0.2, height=0.15) for i in range(3)]

    plans_default, _ = generate_loading_plan(boxes, trunk)
    plans_entrance_first, _ = generate_loading_plan(boxes, trunk, entrance_preference=-1.0)

    assert len(plans_default) == 3 and len(plans_entrance_first) == 3
    pos_default = {p.box_id: p.position for p in plans_default}
    pos_entrance_first = {p.box_id: p.position for p in plans_entrance_first}
    assert pos_default != pos_entrance_first
    # 평균 x가 입구 우선일 때 더 얕아야(작아야) 함
    avg_x_default = sum(p[0] for p in pos_default.values()) / 3
    avg_x_entrance_first = sum(p[0] for p in pos_entrance_first.values()) / 3
    assert avg_x_entrance_first < avg_x_default
