"""
test_20_task_export.py
⑳ build_task_json() - "Cart2Trunk 최종 프로젝트 시나리오 및 시스템 흐름" 문서
4.3절이 정의한 Lenovo -> MSI2 Task JSON 스키마 생성 검증.
"""
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


def test_builds_task_json_matching_documented_schema():
    parameters = {"box_margin": 0.02, "wall_margin": 0.03, "priority_count": 0.7, "priority_large_box": 0.3}

    task = build_task_json(
        plan_id="load_plan_001", box_snapshot_id="box_scan_001", trunk_map_id="trunk_map_001",
        parameters=parameters, plans=_sample_plans(), approved=True,
    )

    assert task["plan_id"] == "load_plan_001"
    assert task["box_snapshot_id"] == "box_scan_001"
    assert task["trunk_map_id"] == "trunk_map_001"
    assert task["approved"] is True
    assert task["parameters"] == parameters
    assert len(task["tasks"]) == 2

    first = task["tasks"][0]
    assert first["sequence"] == 1
    assert first["box_id"] == "BOX_03"
    assert first["target_position"] == [0.1, 0.2, 0.0]
    assert first["target_yaw"] == 1.57
    assert first["placement_score"] == 0.92


def test_defaults_to_not_approved():
    task = build_task_json(
        plan_id="p", box_snapshot_id="s", trunk_map_id="t",
        parameters={}, plans=_sample_plans(),
    )
    assert task["approved"] is False


def test_empty_plans_produce_empty_tasks_list():
    task = build_task_json(
        plan_id="p", box_snapshot_id="s", trunk_map_id="t",
        parameters={}, plans=[],
    )
    assert task["tasks"] == []
