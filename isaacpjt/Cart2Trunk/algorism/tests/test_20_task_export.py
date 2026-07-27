"""
test_20_task_export.py
⑳ build_task_json() - isaac_task_runner.py(100.py 포팅판)가 실제 2026-07-27
4박스 pick&place 전 과정에서 검증한 placement_result.json 스키마와 일치하는지
검증(20_task_export.py 모듈 docstring 참고 - 예전 "시스템 흐름" 문서 4.3절
스키마는 미검증이라 폐기).
"""
import math
import sys, pathlib
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # tests/ -> algorism/
_m07 = import_module("07_placement_plan")
_m20 = import_module("20_task_export")

PlacementPlan = _m07.PlacementPlan
build_task_json = _m20.build_task_json


def _sample_plans():
    return [
        PlacementPlan(box_id="BOX_03", order=1, position=(0.1, 0.2, 0.0),
                       dimensions=(0.3, 0.2, 0.15), score=0.92, touches=2,
                       rotated=True, target_yaw=1.57),
        PlacementPlan(box_id="BOX_01", order=2, position=(0.4, 0.2, 0.0),
                       dimensions=(0.3, 0.2, 0.15), score=0.5, touches=1,
                       rotated=False, target_yaw=0.0),
    ]


def test_builds_task_json_matching_isaac_task_runner_schema():
    parameters = {"box_margin": 0.02, "wall_margin": 0.03, "priority_count": 0.7, "priority_large_box": 0.3}

    task = build_task_json(
        plan_id="load_plan_001", box_snapshot_id="box_scan_001", trunk_map_id="trunk_map_001",
        parameters=parameters, plans=_sample_plans(), offset=(2.5, -0.3, 0.4), approved=True,
    )

    assert task["plan_id"] == "load_plan_001"
    assert task["box_snapshot_id"] == "box_scan_001"
    assert task["trunk_map_id"] == "trunk_map_001"
    assert task["approved"] is True
    assert task["parameters"] == parameters
    assert len(task["placements"]) == 2

    first = task["placements"][0]
    assert first["order"] == 1
    assert first["box_id"] == "BOX_03"
    # position(로컬) + offset = position_base_frame (local_to_base_frame과 동일)
    assert first["position_base_frame"] == [0.1 + 2.5, 0.2 - 0.3, 0.0 + 0.4]
    assert first["dimensions"] == [0.3, 0.2, 0.15]
    assert first["rotated"] is True
    assert first["score"] == 0.92


def test_wrist_yaw_matches_19_run_full_pipeline_formula():
    """target_yaw(=initial_yaw + rotated?90:0)에서 initial_yaw를 정확히
    역산해서, 19_run_full_pipeline_with_yaw.py와 동일한 공식으로
    wrist_yaw_deg를 재계산하는지 확인."""
    # rotated=True, target_yaw=90도(pi/2) -> initial_yaw=0 -> source_yaw_deg=0
    # -> target_deg=90 -> wrist_yaw_deg = ((90-0+90)%180)-90 = (180%180)-90 = -90
    # (사각형 180도 대칭 - +90/-90 둘 다 물리적으로 같은 자세, 공식은 그 중
    # [-90,90] 범위 안의 값을 고른다)
    plan_rotated = PlacementPlan(
        box_id="B", order=1, position=(0.0, 0.0, 0.0), dimensions=(0.3, 0.2, 0.1),
        score=1.0, touches=0, rotated=True, target_yaw=math.pi / 2,
    )
    task = build_task_json(
        plan_id="p", box_snapshot_id="s", trunk_map_id="t",
        parameters={}, plans=[plan_rotated],
    )
    p = task["placements"][0]
    assert p["source_yaw_deg"] == 0.0
    assert p["wrist_yaw_deg"] == -90.0

    # rotated=False, target_yaw=0 -> initial_yaw=0 -> source_yaw_deg=0
    # -> target_deg=0 -> wrist_yaw_deg=0
    plan_flat = PlacementPlan(
        box_id="B", order=1, position=(0.0, 0.0, 0.0), dimensions=(0.3, 0.2, 0.1),
        score=1.0, touches=0, rotated=False, target_yaw=0.0,
    )
    task2 = build_task_json(
        plan_id="p", box_snapshot_id="s", trunk_map_id="t",
        parameters={}, plans=[plan_flat],
    )
    p2 = task2["placements"][0]
    assert p2["source_yaw_deg"] == 0.0
    assert p2["wrist_yaw_deg"] == 0.0


def test_defaults_to_not_approved():
    task = build_task_json(
        plan_id="p", box_snapshot_id="s", trunk_map_id="t",
        parameters={}, plans=_sample_plans(),
    )
    assert task["approved"] is False


def test_defaults_to_zero_offset():
    task = build_task_json(
        plan_id="p", box_snapshot_id="s", trunk_map_id="t",
        parameters={}, plans=_sample_plans(),
    )
    assert task["placements"][0]["position_base_frame"] == [0.1, 0.2, 0.0]


def test_empty_plans_produce_empty_placements_list():
    task = build_task_json(
        plan_id="p", box_snapshot_id="s", trunk_map_id="t",
        parameters={}, plans=[],
    )
    assert task["placements"] == []
