"""
test_03_entrance_wall_flush_candidates.py
③ generate_wall_flush_candidates()가 "입구 쪽 벽" 플러시 후보도 명시적으로
만드는지 확인.

배경(실제 발견된 격차): 기존 코드는 "가장 안쪽 벽(wall A)" 플러시 후보만
margin 반영해서 만들고, 입구 쪽 벽은 만들지 않았다. 빈 트렁크의 첫 박스는
state.candidates={(0,0,0)}뿐이라, 입구쪽 마진 후보가 아예 후보 풀에 없는
경우가 있었다 - entrance_preference로 입구를 아무리 강하게 우선해도 "첫
박스"는 그 자리를 선택할 수조차 없었다는 뜻. 이제 입구쪽도 명시적으로 만들고,
"트렁크 입구 여유 거리"(entrance_margin) 슬라이더도 반영한다.
"""
import sys, pathlib
from importlib import import_module

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # tests/ -> algorism/
_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")

Trunk = _m02.Trunk
Box = _m03.Box
generate_wall_flush_candidates = _m03.generate_wall_flush_candidates


def _has_x_near(candidates, expected_x, tol=1e-6):
    return any(abs(x - expected_x) < tol for (x, _y, _z) in candidates)


def test_entrance_side_flush_candidate_exists_for_a_single_seed_candidate():
    trunk = Trunk(width=1.0, depth=1.0, height=0.5, entrance_near_x=True)
    box = Box("A", width=0.2, depth=0.2, height=0.15)

    result = generate_wall_flush_candidates(box, trunk, {(0.0, 0.0, 0.0)}, margin=0.02)

    assert _has_x_near(result, 0.02)  # 입구(x=0)쪽에서 margin만큼 뗀 자리


def test_entrance_margin_overrides_entrance_side_distance_only():
    trunk = Trunk(width=1.0, depth=1.0, height=0.5, entrance_near_x=True)
    box = Box("A", width=0.2, depth=0.2, height=0.15)

    result = generate_wall_flush_candidates(box, trunk, {(0.0, 0.0, 0.0)}, margin=0.02, entrance_margin=0.10)

    assert _has_x_near(result, 0.10)   # 입구쪽은 entrance_margin
    assert _has_x_near(result, 0.78)   # 안쪽 벽(wall A)은 그대로 margin(0.02): 1.0-0.2-0.02


def test_entrance_side_flush_flips_when_entrance_is_on_far_x():
    trunk = Trunk(width=1.0, depth=1.0, height=0.5, entrance_near_x=False)
    box = Box("A", width=0.2, depth=0.2, height=0.15)

    result = generate_wall_flush_candidates(box, trunk, {(0.0, 0.0, 0.0)}, margin=0.02, entrance_margin=0.10)

    assert _has_x_near(result, 0.70)  # 입구(x=width쪽)에서 0.10 뗀 자리: 1.0-0.2-0.10
