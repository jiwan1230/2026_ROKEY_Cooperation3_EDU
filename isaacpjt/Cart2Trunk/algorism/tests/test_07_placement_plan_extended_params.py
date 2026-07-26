"""
test_07_placement_plan_extended_params.py
⑦ place_one_box()에 "HMI 화면 설계 가이드라인" 문서의 나머지 파라미터
(allow_rotation, wall_margin, obstacle_margin, ceiling_margin)를 주입할 수
있는지 확인. 전부 기본값(None/True)이면 지금까지와 완전히 동일하게 동작
(하위 호환).
"""
import sys, pathlib
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # tests/ -> algorism/
_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
_m07 = import_module("07_placement_plan")

Trunk = _m02.Trunk
Box = _m03.Box
ExtremePointState = _m03.ExtremePointState
PlacedBox = _m03.PlacedBox
place_one_box = _m07.place_one_box


def test_allow_rotation_false_skips_rotation_retry():
    """정자세로 안 들어가고 돌리면 들어가는 박스라도, allow_rotation=False면
    회전 재시도를 아예 안 하고 실패해야 한다."""
    trunk = Trunk(width=0.6, depth=0.73, height=0.5)
    state = ExtremePointState()
    box = Box("Wide", width=0.65, depth=0.30, height=0.15)

    plan_default = place_one_box(box, trunk, state, order=1)
    assert plan_default is not None
    assert plan_default.rotated is True  # 하위 호환: 기본값은 지금까지처럼 회전 시도

    state2 = ExtremePointState()
    plan_no_rotate = place_one_box(box, trunk, state2, order=1, allow_rotation=False)
    assert plan_no_rotate is None


def test_ceiling_margin_override_is_respected():
    """ceiling_margin을 주면 15_overhead_clearance_check.OVERHEAD_CLEARANCE
    기본값(그리퍼 실측 0.12m + 안전 마진 0.03m = 0.15m, 7/25 갱신) 대신 이
    값이 쓰여야 한다."""
    trunk = Trunk(width=1.0, depth=1.0, height=0.35)
    state = ExtremePointState()
    box = Box("A", width=0.2, depth=0.2, height=0.25)  # z=0에 놓으면 남는 여유 0.10m

    # 기본 ceiling_margin(0.15m)으로는 여유(0.10m) 부족 -> 실패
    plan_default = place_one_box(box, trunk, state, order=1)
    assert plan_default is None

    # ceiling_margin=0.05로 낮추면 통과해야 함
    state2 = ExtremePointState()
    plan_relaxed = place_one_box(box, trunk, state2, order=1, ceiling_margin=0.05)
    assert plan_relaxed is not None


def test_wall_margin_larger_than_box_margin_still_generates_a_valid_candidate():
    """버그였던 것: wall_margin > margin(box_margin)일 때 벽 근처 후보가 항상
    margin 거리로만 생성되면, has_wall_margin은 더 큰 wall_margin을 요구해서
    그 후보를 거부하고 대안이 없어 배치 전체가 실패할 수 있다(obstacle_margin과
    같은 종류의 버그가 wall_margin에도 있었음 - generate_wall_flush_candidates
    호출에 wall_margin이 반영되어야 한다)."""
    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    state = ExtremePointState()
    box = Box("A", width=0.3, depth=0.3, height=0.2)

    plan = place_one_box(box, trunk, state, order=1, margin=0.02, wall_margin=0.10)

    assert plan is not None
    x, y, _z = plan.position
    assert x >= 0.10 - 1e-9
    assert y >= 0.10 - 1e-9


def test_obstacle_margin_can_make_the_only_remaining_gap_infeasible():
    """장애물이 트렁크 폭 전체를 가로막고, 남은 건 반대쪽의 좁은 틈(0.3m)뿐인
    상황을 만든다. obstacle_margin이 작으면 그 틈에 들어가고, obstacle_margin을
    틈보다 크게 주면 아예 못 들어가야(None) 한다 - margin=None(box_margin
    기본값)으로는 안 갈리므로 순수하게 obstacle_margin이 관통되는지만 검증."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    state = ExtremePointState()
    # 장애물: 폭 전체(x 0~1.0), y 0~0.7, 높이 0.3 - 남은 공간은 y=[0.7,1.0]인 0.3m 틈
    obstacle = PlacedBox(box=Box("Wheel", 1.0, 0.7, 0.3, is_obstacle=True), x=0.0, y=0.0, z=0.0)
    state.register_placement(obstacle)
    box = Box("C", width=0.2, depth=0.2, height=0.2)

    plan_small_margin = place_one_box(box, trunk, state, order=1, margin=0.02, obstacle_margin=0.05)
    assert plan_small_margin is not None

    state2 = ExtremePointState()
    state2.register_placement(obstacle)
    plan_large_margin = place_one_box(box, trunk, state2, order=1, margin=0.02, obstacle_margin=0.35)
    assert plan_large_margin is None
