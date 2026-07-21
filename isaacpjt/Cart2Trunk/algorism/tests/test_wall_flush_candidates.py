"""
test_wall_flush_candidates.py
③+⑦ "벽에 딱 붙는 자리" 후보 누락 버그 재현 및 수정 검증.

배경: 극점 알고리즘의 후보 생성(register_placement)은 박스 크기와 무관하게 이미
놓인 것들의 모서리만 보고 후보를 만든다. 그런데 "이 박스라면 벽 A에 딱 붙을 수
있는 자리"는 놓으려는 박스의 폭을 알아야 계산되는 좌표라서, 아무 것도 그 자리에
없으면(=근처에 모서리를 만들어줄 게 없으면) 후보 자체가 생성되지 않는다.

실제로 발견된 사례: 손그림 테스트에서 폭 0.28m짜리 박스가 트렁크(폭 0.6m) 안쪽까지
물리적으로 들어갈 자리(x=0.32)가 있었는데도, 그 좌표를 만들어줄 기존 모서리가 없어서
알고리즘이 못 찾고 입구 쪽에 배치했었다. 이 테스트는 그 사례를 최소 재현한다.
"""
import sys, pathlib
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # tests/ -> algorism/
_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
_m04 = import_module("04_candidate_validity_check")
_m07 = import_module("07_placement_plan")

Trunk = _m02.Trunk
Box = _m03.Box
PlacedBox = _m03.PlacedBox
ExtremePointState = _m03.ExtremePointState
generate_wall_flush_candidates = _m03.generate_wall_flush_candidates
is_candidate_valid = _m04.is_candidate_valid
place_one_box = _m07.place_one_box


def test_generate_wall_flush_candidates_adds_wall_a_variant():
    """
    기존 후보 (0, 0.31, 0)이 있을 때, 폭 0.28짜리 박스라면 "벽 A(x=width쪽)에 딱
    붙는" 좌표 (trunk.width - box.width, 0.31, 0)도 추가로 만들어져야 한다.
    """
    trunk = Trunk(width=0.6, depth=0.73, height=0.4)  # entrance_near_x 기본값 True
    box = Box("Wide", width=0.28, depth=0.07, height=0.15)
    existing = {(0.0, 0.31, 0.0)}

    extra = generate_wall_flush_candidates(box, trunk, existing)

    assert (0.6 - 0.28, 0.31, 0.0) in extra


def test_generate_wall_flush_candidates_adds_wall_b_and_c_variants():
    """
    기존 후보 (0.2, 0.1, 0)이 있을 때, 깊이 0.07짜리 박스라면 벽 C(y=0)에 붙는
    (0.2, 0, 0)과 벽 B(y=depth쪽)에 붙는 (0.2, depth-0.07, 0)도 추가돼야 한다.
    """
    trunk = Trunk(width=0.6, depth=0.73, height=0.4)
    box = Box("Thin", width=0.1, depth=0.07, height=0.15)
    existing = {(0.2, 0.1, 0.0)}

    extra = generate_wall_flush_candidates(box, trunk, existing)

    assert (0.2, 0.0, 0.0) in extra
    assert (0.2, 0.73 - 0.07, 0.0) in extra


def test_place_one_box_finds_deep_spot_that_pure_corner_extension_misses():
    """
    실제로 발견된 사례의 최소 재현: 오른쪽 벽 쪽에 장애물 2개(차 바퀴 흉내)와 박스
    하나가 이미 놓여 있고, 그 사이 y밴드는 비어 있지만 그 자리에 도달할 기존
    모서리가 없다. 폭 0.28짜리 박스를 놓으면, 후보 생성 보강 전에는 입구 쪽
    (x=0)에 배치됐지만 보강 후에는 벽 A 쪽(x=0.32, 벽에 딱 붙음)에 배치돼야 한다.
    """
    trunk = Trunk(width=0.6, depth=0.73, height=0.4)
    state = ExtremePointState()
    for obs in [
        PlacedBox(box=Box("Wheel_Front", 0.16, 0.15, 0.20), x=0.44, y=0.00, z=0.0),
        PlacedBox(box=Box("Wheel_Rear", 0.16, 0.21, 0.20), x=0.44, y=0.52, z=0.0),
        PlacedBox(box=Box("Green_Mid", 0.14, 0.16, 0.15), x=0.44, y=0.15, z=0.0),
    ]:
        state.register_placement(obs)

    wide_box = Box("Green_Wide", width=0.28, depth=0.07, height=0.15)

    # 전제 확인: (0.32, 0.31, 0)이 실제로 겹치지 않는 유효한 자리인지 (물리적으로는 항상 있었음)
    assert is_candidate_valid(0.32, 0.31, 0.0, wide_box, trunk, state.placed)
    # 전제 확인: 이 좌표가 순수 모서리 확장만으로는 후보 목록에 없었는지 (버그 재현)
    assert (0.32, 0.31, 0.0) not in state.candidates

    plan = place_one_box(wide_box, trunk, state, order=1)

    assert plan is not None
    x, y, z = plan.position
    assert abs(x - 0.32) < 1e-9, f"벽 A에 딱 붙은 깊은 자리(x=0.32)가 아니라 x={x}에 배치됨"
