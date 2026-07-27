"""
test_trunk_map_planner_node_task_export.py
trunk_map_planner_node._send_task_to_msi2()의 안전장치 검증.

실제 MSI2 전송 경로(토픽/서비스)가 아직 미확정이라 지금은 로컬 파일 저장만
한다 - 그래도 "승인 전엔 절대 전달 안 함" 원칙(HMI 8절 #3)만큼은 이 함수
레벨에서 강제되어야 한다.
"""
import json
import sys
import pathlib
from importlib import import_module

import pytest

_CART2TRUNK_DIR = str(pathlib.Path(__file__).resolve().parent.parent)
if _CART2TRUNK_DIR not in sys.path:
    sys.path.insert(0, _CART2TRUNK_DIR)

node = import_module("trunk_map_planner_node")


def test_rejects_unapproved_task():
    task = {"plan_id": "p1", "approved": False, "tasks": []}
    with pytest.raises(ValueError, match="approved"):
        node._send_task_to_msi2(task)


def test_writes_approved_task_to_local_file(tmp_path):
    task = {
        "plan_id": "load_plan_001", "box_snapshot_id": "s1", "trunk_map_id": "t1",
        "approved": True, "parameters": {}, "tasks": [],
    }

    out_path = node._send_task_to_msi2(task, out_dir=tmp_path)

    saved = json.loads(pathlib.Path(out_path).read_text())
    assert saved == task
    assert pathlib.Path(out_path).parent == tmp_path
