"""
test_07_placement_plan_stacking.py
⑦ place_one_box()의 allow_stacking 플래그 배선 확인.

시나리오: 트렁크 바닥 전체를 정확히 채우는 박스를 먼저 놓으면, 두 번째 같은
박스는 바닥에 더 놓을 자리가 없다. allow_stacking=False(기본값)면 이때
"놓을 자리 없음"(None)이어야 하고, allow_stacking=True면 첫 번째 박스 위에
(받침 100%로) 정확히 쌓여야 한다.
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
place_one_box = _m07.place_one_box
MARGIN = _m03.PLACEMENT_SAFETY_MARGIN_M


def _fills_floor_trunk():
    # 바닥 면적을 정확히 채우는 박스 하나가 들어갈 트렁크 (0.3 x 0.3 박스 + 사방
    # 벽 안전 여유(MARGIN) 만큼만 더 큰 트렁크), 두 층 놓을 높이는 있음.
    # PLACEMENT_SAFETY_MARGIN_M 도입 이후 벽에 딱 붙는 배치가 금지되므로, 트렁크를
    # 정확히 0.3x0.3으로 두면 박스가 아예 하나도 안 들어간다 - 이 테스트의 목적
    # (바닥이 꽉 찼을 때 stacking 플래그 배선 확인)과 무관한 조건이라 여유만큼
    # 트렁크를 키워서 "박스 하나가 딱 맞게 들어가는" 시나리오를 유지한다.
    return Trunk(width=0.3 + 2 * MARGIN, depth=0.3 + 2 * MARGIN, height=0.6)


def test_second_box_has_nowhere_to_go_when_stacking_disabled():
    trunk = _fills_floor_trunk()
    state = ExtremePointState()
    filler = Box("Floor", width=0.3, depth=0.3, height=0.3)

    first = place_one_box(filler, trunk, state, order=1)
    assert first is not None
    assert all(abs(a - b) < 1e-9 for a, b in zip(first.position, (MARGIN, MARGIN, 0.0)))

    second = place_one_box(filler, trunk, state, order=2, allow_stacking=False)
    assert second is None  # 바닥엔 자리 없고, z>0은 플래그가 꺼져 있어 거부됨


def test_second_box_stacks_on_top_when_stacking_enabled():
    trunk = _fills_floor_trunk()
    state = ExtremePointState()
    filler = Box("Floor", width=0.3, depth=0.3, height=0.3)

    first = place_one_box(filler, trunk, state, order=1)
    assert first is not None

    second = place_one_box(filler, trunk, state, order=2, allow_stacking=True)
    assert second is not None
    assert all(abs(a - b) < 1e-9 for a, b in zip(second.position, (MARGIN, MARGIN, 0.3)))  # 첫 박스 바로 위, 받침 100% (z는 여유 없음)
