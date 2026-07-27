"""
routes/approval.py
POST /api/approve, POST /api/send - 계획 승인 및 MSI2 전송.

[2026-07-28] POST /api/send가 로컬 저장만 하고 실제로 MSI2에 아무것도 안 보내던
문제를 고쳤다 - algorism_bridge.send_task()로 로컬에 저장(승인 게이트 + 감사
기록용, 그대로 유지)한 뒤, robot_bridge.send_placement_plan()으로
/cart2trunk/send_placement_plan 서비스를 호출해서 MSI2의
results/holonomic_base/placement_result.json에 실제로 써넣는다.
"""
import subprocess
from datetime import datetime

from flask import Blueprint, jsonify, request

import algorism_bridge as bridge
import robot_bridge
from routes.plan import ApiError

approval_bp = Blueprint("approval", __name__)


@approval_bp.post("/api/approve")
def post_approve():
    body = request.get_json(force=True, silent=True) or {}
    placed = body.get("placed")
    if not isinstance(placed, list) or not placed:
        raise ApiError(
            400, "NO_PLAN_TO_APPROVE", "승인할 배치 계획(placed)이 없습니다.",
            "먼저 POST /api/plan으로 계획을 계산한 뒤 그 결과의 placed를 그대로 보내세요.",
        )

    plan_id = f"load_plan_{datetime.now():%Y%m%d_%H%M%S}"
    task = bridge.build_approved_task(
        plan_id=plan_id,
        box_snapshot_id=body.get("box_snapshot_id", "unknown"),
        trunk_map_id=body.get("trunk_map_id", "unknown"),
        parameters=body.get("parameters", {}),
        placed=placed,
        offset=body.get("trunk_offset_base_frame"),
    )
    return jsonify({"plan_id": plan_id, "task": task})


@approval_bp.post("/api/send")
def post_send():
    body = request.get_json(force=True, silent=True) or {}
    task = body.get("task")
    if not isinstance(task, dict):
        raise ApiError(
            400, "NO_TASK_TO_SEND", "전송할 task가 없습니다.",
            "먼저 POST /api/approve로 승인한 뒤 그 응답의 task를 그대로 보내세요.",
        )
    try:
        out_path = bridge.send_task(task)
    except ValueError as e:
        raise ApiError(
            400, "TASK_NOT_APPROVED", str(e),
            "POST /api/approve를 먼저 호출해서 approved=True인 task를 받은 뒤 다시 시도하세요.",
        )

    try:
        msi2_result = robot_bridge.send_placement_plan(task)
    except (subprocess.TimeoutExpired, RuntimeError) as e:
        return jsonify({
            "status": "error", "message": f"MSI2 전송 실패: {e}", "out_path": out_path,
        }), 502

    return jsonify({
        "out_path": out_path,
        "msi2_message": msi2_result.get("message"),
        "msi2_written_path": msi2_result.get("written_path"),
    })
