"""
test_17_margin_check.py
⑰ 박스-벽 / 박스-박스 최소 간격(margin) 확인 검증.

배경: "박스끼리 아예 딱 붙지 말고 아주 조금 여유 공간을 두는 게 좋겠다"는 요청
(박스-벽, 박스-박스 둘 다 적용). 고정 마진(기본 0.01m = 1cm)을 하드 컷으로
확인한다. 바닥 접촉(z=0)이나 쌓기 지지면(z가 딱 맞닿는 경우)은 마진 대상이
아니다 - 그건 오히려 완전히 맞닿아야(⑬ 받침) 하는 관계라서 마진을 넣으면 안 된다.
"""
import sys, pathlib
from importlib import import_module

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # tests/ -> algorism/
_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
_m07 = import_module("07_placement_plan")
_m17 = import_module("17_margin_check")

Trunk = _m02.Trunk
Box = _m03.Box
PlacedBox = _m03.PlacedBox
ExtremePointState = _m03.ExtremePointState
place_one_box = _m07.place_one_box
has_wall_margin = _m17.has_wall_margin
has_box_margin = _m17.has_box_margin
has_sufficient_margin = _m17.has_sufficient_margin
MARGIN = _m17.MARGIN


def test_wall_margin_rejects_when_touching_near_wall():
    """x=0(벽에 딱 붙음)이면 마진(0.01m) 미달로 거부."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", width=0.2, depth=0.2, height=0.2)
    assert has_wall_margin(0.0, 0.5, 0.0, box, trunk) is False


def test_wall_margin_accepts_at_exactly_margin_distance():
    """x=MARGIN(정확히 마진만큼 떨어짐)이면 경계값 통과."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", width=0.2, depth=0.2, height=0.2)
    assert has_wall_margin(MARGIN, 0.5, 0.0, box, trunk) is True


def test_wall_margin_rejects_when_touching_far_wall():
    """반대쪽 벽(x+width == trunk.width)에 딱 붙어도 거부."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", width=0.2, depth=0.2, height=0.2)
    assert has_wall_margin(0.8, 0.5, 0.0, box, trunk) is False  # 0.8+0.2=1.0, 여유 0


def test_wall_margin_rejects_when_touching_side_wall_y():
    """y=0 또는 y+depth==trunk.depth(옆벽)에 딱 붙어도 거부."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", width=0.2, depth=0.2, height=0.2)
    assert has_wall_margin(0.5, 0.0, 0.0, box, trunk) is False
    assert has_wall_margin(0.5, 0.8, 0.0, box, trunk) is False


def test_wall_margin_does_not_apply_to_floor_or_ceiling():
    """z=0(바닥 접촉)은 마진 대상이 아니다 - 바닥엔 딱 붙어야 정상."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", width=0.2, depth=0.2, height=0.2)
    assert has_wall_margin(0.5, 0.5, 0.0, box, trunk) is True


def test_box_margin_rejects_when_flush_against_neighbor_at_same_height():
    """옆에 딱 붙은(간격 0) 박스가 있으면(z 겹침) 마진 미달로 거부."""
    neighbor = PlacedBox(box=Box("N", 0.3, 0.3, 0.2), x=0.0, y=0.0, z=0.0)
    candidate = Box("C", width=0.2, depth=0.2, height=0.2)
    assert has_box_margin(0.3, 0.0, 0.0, candidate, [neighbor]) is False  # x=0.3에 딱 붙음


def test_box_margin_accepts_at_exactly_margin_distance():
    """이웃과 정확히 MARGIN만큼 떨어지면 경계값 통과."""
    neighbor = PlacedBox(box=Box("N", 0.3, 0.3, 0.2), x=0.0, y=0.0, z=0.0)
    candidate = Box("C", width=0.2, depth=0.2, height=0.2)
    assert has_box_margin(0.3 + MARGIN, 0.0, 0.0, candidate, [neighbor]) is True


def test_box_margin_ignores_neighbor_when_stacked_on_top():
    """z 범위가 안 겹치는(딱 위에 쌓인) 관계는 마진 대상이 아니다 - 맞닿아야 정상."""
    base = PlacedBox(box=Box("Base", 0.3, 0.3, 0.2), x=0.0, y=0.0, z=0.0)
    candidate = Box("Top", width=0.3, depth=0.3, height=0.1)
    # Top이 Base 바로 위(z=0.2, Base와 같은 x/y footprint)에 딱 붙어도 통과해야 함
    assert has_box_margin(0.0, 0.0, 0.2, candidate, [base]) is True


def test_has_sufficient_margin_combines_both():
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    neighbor = PlacedBox(box=Box("N", 0.3, 0.3, 0.2), x=0.0, y=0.0, z=0.0)
    candidate = Box("C", width=0.2, depth=0.2, height=0.2)
    # 벽에서는 충분히 떨어졌지만 이웃과는 딱 붙음 -> 거부
    assert has_sufficient_margin(0.3, 0.0, 0.0, candidate, trunk, [neighbor]) is False
    # 벽/이웃 둘 다에서 충분히 떨어짐 -> 통과
    assert has_sufficient_margin(0.3 + MARGIN, MARGIN, 0.0, candidate, trunk, [neighbor]) is True


def test_place_one_box_leaves_margin_from_wall():
    """통합 테스트: 실제 place_one_box()가 벽에 딱 붙이지 않고 MARGIN만큼 띄운다."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    state = ExtremePointState()
    box = Box("A", width=0.3, depth=0.3, height=0.2)

    plan = place_one_box(box, trunk, state, order=1)

    assert plan is not None
    x, y, z = plan.position
    assert x >= MARGIN - 1e-9
    assert y >= MARGIN - 1e-9


def test_place_one_box_leaves_margin_between_boxes():
    """통합 테스트: 두 번째 박스가 첫 번째 박스에 딱 붙지 않고 MARGIN만큼 띄워진다."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    state = ExtremePointState()
    box1 = Box("A", width=0.2, depth=0.2, height=0.2)
    box2 = Box("B", width=0.2, depth=0.2, height=0.2)

    plan1 = place_one_box(box1, trunk, state, order=1)
    plan2 = place_one_box(box2, trunk, state, order=2)

    assert plan1 is not None and plan2 is not None
    x1, y1, z1 = plan1.position
    x2, y2, z2 = plan2.position
    x_gap = max(x1 - (x2 + box2.width), x2 - (x1 + box1.width))
    y_gap = max(y1 - (y2 + box2.depth), y2 - (y1 + box1.depth))
    assert x_gap >= MARGIN - 1e-9 or y_gap >= MARGIN - 1e-9


# ---------------------------------------------------------------------------
# "HMI 화면 설계 가이드라인" 문서가 요구하는 마진 4종 분리(박스간/벽면/천장/
# 장애물) 중 벽면·장애물 부분 - box_margin(margin)과 별개로 wall_margin/
# obstacle_margin을 따로 줄 수 있어야 한다.
# ---------------------------------------------------------------------------

def test_has_sufficient_margin_wall_margin_override_is_independent_of_box_margin():
    """wall_margin을 margin과 다르게 주면 벽 판정에는 wall_margin이 쓰이고
    박스-박스 판정에는 그대로 margin(box_margin)이 쓰여야 한다."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", width=0.2, depth=0.2, height=0.2)
    # 벽에서 0.05m 떨어짐: wall_margin=0.05면 통과, wall_margin=0.1이면 거부
    assert has_sufficient_margin(0.05, 0.5, 0.0, box, trunk, [], margin=0.02, wall_margin=0.05) is True
    assert has_sufficient_margin(0.05, 0.5, 0.0, box, trunk, [], margin=0.02, wall_margin=0.10) is False


def test_has_sufficient_margin_wall_margin_defaults_to_margin_when_not_given():
    """wall_margin을 안 주면 margin(box_margin) 값을 그대로 씀 - 하위 호환."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", width=0.2, depth=0.2, height=0.2)
    assert has_sufficient_margin(0.05, 0.5, 0.0, box, trunk, [], margin=0.05) is True
    assert has_sufficient_margin(0.04, 0.5, 0.0, box, trunk, [], margin=0.05) is False


def test_obstacle_margin_applies_only_to_boxes_flagged_as_obstacle():
    """is_obstacle=True인 PlacedBox와의 간격은 obstacle_margin으로, 일반 카트
    박스와의 간격은 그대로 margin(box_margin)으로 판정해야 한다."""
    trunk = Trunk(width=2.0, depth=2.0, height=1.0)
    obstacle = PlacedBox(box=Box("Wheel", 0.3, 0.3, 0.2, is_obstacle=True), x=0.0, y=0.5, z=0.0)
    cart_box = PlacedBox(box=Box("N", 0.3, 0.3, 0.2), x=1.0, y=0.5, z=0.0)
    candidate = Box("C", width=0.2, depth=0.2, height=0.2)

    # 장애물에서 0.05m: obstacle_margin=0.05면 통과, obstacle_margin=0.1이면 거부
    # (y=0.5로 둘 다 벽 마진은 넉넉히 통과하게 잡아서, 여기서 갈리는 건 오직 박스-박스/장애물 판정뿐)
    assert has_sufficient_margin(0.35, 0.5, 0.0, candidate, trunk, [obstacle],
                                  margin=0.02, obstacle_margin=0.05) is True
    assert has_sufficient_margin(0.35, 0.5, 0.0, candidate, trunk, [obstacle],
                                  margin=0.02, obstacle_margin=0.10) is False
    # 일반 카트 박스는 obstacle_margin이 아니라 margin(0.02)으로 판정되어야 함
    assert has_sufficient_margin(0.65, 0.5, 0.0, candidate, trunk, [cart_box],
                                  margin=0.02, obstacle_margin=0.10) is True


def test_box_is_obstacle_defaults_to_false():
    box = Box("A", width=0.2, depth=0.2, height=0.2)
    assert box.is_obstacle is False


# ---------------------------------------------------------------------------
# "HMI 화면 설계 가이드라인" 문서의 "트렁크 입구 여유 거리" - wall_margin과 별개로
# 입구 쪽 벽만 다른 간격을 줄 수 있어야 한다.
# ---------------------------------------------------------------------------

def test_entrance_margin_applies_only_to_entrance_side_wall():
    """entrance_near_x=True면 x=0쪽이 입구 - entrance_margin은 x=0쪽에만 적용되고
    반대쪽(안쪽 벽)은 margin(또는 wall_margin) 그대로여야 한다."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0, entrance_near_x=True)
    box = Box("A", width=0.2, depth=0.2, height=0.2)

    # 입구(x=0)에서 0.10m: entrance_margin=0.10이면 통과, margin(0.02) 기준이면 거부됐을 값
    assert has_wall_margin(0.10, 0.5, 0.0, box, trunk, margin=0.02, entrance_margin=0.10) is True
    # 안쪽 벽(x+width=1.0)에서는 여전히 margin(0.02)만 있으면 됨 - entrance_margin과 무관
    assert has_wall_margin(0.78, 0.5, 0.0, box, trunk, margin=0.02, entrance_margin=0.10) is True
    # 입구에서 0.05m밖에 안 띄우면 entrance_margin=0.10 기준으로 거부되어야 함
    assert has_wall_margin(0.05, 0.5, 0.0, box, trunk, margin=0.02, entrance_margin=0.10) is False


def test_entrance_margin_flips_side_when_entrance_is_on_far_x():
    """entrance_near_x=False면 x=width쪽이 입구 - entrance_margin이 그쪽에 적용돼야 한다."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0, entrance_near_x=False)
    box = Box("A", width=0.2, depth=0.2, height=0.2)

    # 입구(x+width=1.0)에서 0.10m 띄운 자리: x=1.0-0.2-0.10=0.70
    assert has_wall_margin(0.70, 0.5, 0.0, box, trunk, margin=0.02, entrance_margin=0.10) is True
    # 입구에서 0.05m만 띄우면(x=0.75) entrance_margin=0.10 기준 거부
    assert has_wall_margin(0.75, 0.5, 0.0, box, trunk, margin=0.02, entrance_margin=0.10) is False


def test_entrance_margin_none_falls_back_to_margin():
    """entrance_margin을 안 주면 margin과 동일하게 동작 (하위 호환)."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0, entrance_near_x=True)
    box = Box("A", width=0.2, depth=0.2, height=0.2)
    assert has_wall_margin(0.02, 0.5, 0.0, box, trunk, margin=0.02) is True
    assert has_wall_margin(0.01, 0.5, 0.0, box, trunk, margin=0.02) is False


def test_place_one_box_can_sit_flush_at_entrance_margin_distance():
    """통합 테스트: 빈 트렁크의 '첫 박스'도 entrance_margin 거리의 입구쪽 후보를
    실제로 만날 수 있어야 한다 (기존엔 안쪽 벽 플러시 후보만 명시적으로 생성돼서,
    입구쪽 마진 후보가 아예 없는 경우가 있었음 - entrance_preference=-1로 입구를
    강하게 우선하면 이 후보가 선택돼야 확인 가능)."""
    from importlib import import_module as im
    _m05 = im("05_candidate_scoring")
    make_weighted_score_fn = _m05.make_weighted_score_fn

    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    state = ExtremePointState()
    box = Box("A", width=0.2, depth=0.2, height=0.15)

    entrance_first_score = make_weighted_score_fn(entrance_preference=-1.0)
    plan = place_one_box(box, trunk, state, order=1, margin=0.02, entrance_margin=0.10,
                          score_fn=entrance_first_score)
    assert plan is not None
    x, _y, _z = plan.position
    assert x == pytest.approx(0.10)
