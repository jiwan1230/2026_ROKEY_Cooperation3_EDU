"""
routes/robot.py
POST /api/robot/cart-scan, /trunk-scan, /pick-and-place - 로봇(MSI2 - 신지완/
민결) 동작 트리거. 셋 다 실제 ROS2 Action(robot_bridge.py 참고)으로 연동했다:
- trunk-scan: /cart2trunk/trunk_scan - 89.trunk_scan_holonomic.py ->
  90.export_trunk_map_holonomic.py 실행, PLY를 청크로 받아
  GET /api/robot/trunk-scan-file/<filename>으로 서빙.
- cart-scan: /cart2trunk/cart_scan - 99.cart_scan_dual_side_holonomic.py ->
  perception/multiview_scan.py 실행, 박스 JSON+PLY를 받아
  GET /api/robot/cart-scan-file/<filename>으로 서빙.
- pick-and-place: /cart2trunk/pick_and_place - trunk/cart-scan과 달리 매번
  Isaac Sim을 새로 부팅하지 않는다. Isaac Sim PC에서 이미 계속 떠있는
  isaac_task_runner.py의 Trigger 서비스를 pick_and_place_action_server.py가
  대신 호출하는 얇은 어댑터라, isaac_task_runner.py가 실행 중이어야 성공한다.
"""
import subprocess

from flask import Blueprint, jsonify, request, send_from_directory

import robot_bridge

robot_bp = Blueprint("robot", __name__)


@robot_bp.post("/api/robot/cart-scan")
def cart_scan():
    try:
        result = robot_bridge.run_cart_scan()
    except (subprocess.TimeoutExpired, RuntimeError) as e:
        return jsonify({"status": "error", "message": f"카트 스캔 실패: {e}"}), 502
    return jsonify({
        "status": "ok",
        "message": "카트 스캔 완료",
        "box_count": result.get("box_count"),
        "json_filename": result["json_filename"],
        "json_url": f"/api/robot/cart-scan-file/{result['json_filename']}",
        "ply_filename": result["ply_filename"],
        "ply_url": f"/api/robot/cart-scan-file/{result['ply_filename']}",
        "ply_total_bytes": result.get("ply_total_bytes"),
    })


@robot_bp.get("/api/robot/cart-scan-file/<path:filename>")
def get_cart_scan_file(filename):
    return send_from_directory(
        robot_bridge.RECEIVED_SCANS_DIR, filename, mimetype="application/octet-stream")


@robot_bp.post("/api/robot/trunk-scan")
def trunk_scan():
    try:
        result = robot_bridge.run_trunk_scan()
    except (subprocess.TimeoutExpired, RuntimeError) as e:
        return jsonify({"status": "error", "message": f"트렁크 스캔 실패: {e}"}), 502
    return jsonify({
        "status": "ok",
        "message": "트렁크 스캔 완료",
        "filename": result["filename"],
        "url": f"/api/robot/trunk-scan-file/{result['filename']}",
        "point_count": result.get("point_count"),
        "total_bytes": result.get("total_bytes"),
    })


@robot_bp.get("/api/robot/trunk-scan-file/<path:filename>")
def get_trunk_scan_file(filename):
    return send_from_directory(
        robot_bridge.RECEIVED_SCANS_DIR, filename, mimetype="application/octet-stream")


@robot_bp.post("/api/robot/pick-and-place")
def pick_and_place():
    body = request.get_json(force=True, silent=True) or {}
    plan_id = body.get("plan_id", "")
    try:
        result = robot_bridge.run_pick_and_place(plan_id=plan_id)
    except (subprocess.TimeoutExpired, RuntimeError) as e:
        return jsonify({"status": "error", "message": f"pick_and_place 실패: {e}"}), 502
    return jsonify({
        "status": "ok",
        "message": result.get("message", "pick_and_place 완료"),
        "boxes_placed": result.get("boxes_placed"),
        "boxes_total": result.get("boxes_total"),
    })
