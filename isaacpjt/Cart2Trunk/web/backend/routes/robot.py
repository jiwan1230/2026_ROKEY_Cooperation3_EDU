"""
routes/robot.py
POST /api/robot/cart-scan, /trunk-scan, /pick-and-place - 로봇(MSI2 - 신지완/
민결) 동작 트리거. pick-and-place는 아직 ROS2 노드 구조가 설계 중이라
DUMMY_DELAY_SECONDS만큼 대기한 뒤 항상 성공 응답을 돌려주는 더미다.

trunk-scan/cart-scan은 실제 ROS2 Action(robot_bridge.py 참고)으로 연동했다:
- trunk-scan: /cart2trunk/trunk_scan - 89.trunk_scan_holonomic.py ->
  90.export_trunk_map_holonomic.py 실행, PLY를 청크로 받아
  GET /api/robot/trunk-scan-file/<filename>으로 서빙.
- cart-scan: /cart2trunk/cart_scan - 99.cart_scan_dual_side_holonomic.py ->
  perception/multiview_scan.py 실행, 박스 JSON+PLY를 받아
  GET /api/robot/cart-scan-file/<filename>으로 서빙.
"""
import subprocess
import time

from flask import Blueprint, jsonify, send_from_directory

import robot_bridge

robot_bp = Blueprint("robot", __name__)

# 실제 스캔/동작 시간을 흉내내는 더미 지연(초). 테스트에서는 이 값을
# monkeypatch로 0으로 낮춰서 느려지지 않게 한다.
DUMMY_DELAY_SECONDS = 1.5


def _dummy_trigger(step_name: str):
    time.sleep(DUMMY_DELAY_SECONDS)
    # TODO(MSI2): 여기에 실제 ROS2 서비스/액션 호출을 넣고, 그 결과에 따라
    # status/message를 채운다. 지금은 항상 성공하는 더미.
    return jsonify({
        "status": "ok",
        "dummy": True,
        "message": f"{step_name} 완료 (더미 - 실제 로봇 미연동)",
    })


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
    return _dummy_trigger("픽앤플레이스")
