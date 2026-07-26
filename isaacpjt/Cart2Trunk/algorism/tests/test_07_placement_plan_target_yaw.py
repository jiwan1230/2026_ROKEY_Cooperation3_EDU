"""
test_07_placement_plan_target_yaw.py
⑦ PlacementPlan.target_yaw - MSI2로 넘길 최종 목표 Yaw(라디안).

"HMI 화면 설계 가이드라인" 문서가 Task 출력 필수 필드로 "Target Yaw"를
요구하는데, 지금까지 PlacementPlan은 rotated(bool)만 갖고 있어서 실제
각도값이 없었다. target_yaw = box.initial_yaw + (90도 회전했으면 pi/2, 아니면
0), [-pi, pi) 범위로 정규화.
"""
import math
import sys, pathlib
from importlib import import_module

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # tests/ -> algorism/
_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
_m07 = import_module("07_placement_plan")

Trunk = _m02.Trunk
Box = _m03.Box
ExtremePointState = _m03.ExtremePointState
place_one_box = _m07.place_one_box


def test_target_yaw_matches_initial_yaw_when_not_rotated():
    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    state = ExtremePointState()
    box = Box("A", width=0.3, depth=0.2, height=0.15, initial_yaw=0.35)

    plan = place_one_box(box, trunk, state, order=1)

    assert plan.rotated is False
    assert plan.target_yaw == pytest.approx(0.35)


def test_target_yaw_adds_90_degrees_when_rotated():
    # 정자세(가로 0.65)로는 안 들어가지만 90도 돌리면(가로 0.30) 들어가는 경우
    trunk = Trunk(width=0.6, depth=0.73, height=0.5)
    state = ExtremePointState()
    box = Box("Wide", width=0.65, depth=0.30, height=0.15, initial_yaw=0.1)

    plan = place_one_box(box, trunk, state, order=1)

    assert plan.rotated is True
    assert plan.target_yaw == pytest.approx(0.1 + math.pi / 2)


def test_target_yaw_defaults_to_zero_when_box_has_no_initial_yaw():
    trunk = Trunk(width=1.0, depth=1.0, height=0.5)
    state = ExtremePointState()
    box = Box("A", width=0.3, depth=0.2, height=0.15)  # initial_yaw 기본값 0.0

    plan = place_one_box(box, trunk, state, order=1)

    assert plan.target_yaw == pytest.approx(0.0)
