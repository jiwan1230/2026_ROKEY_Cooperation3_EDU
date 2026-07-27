import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from app import create_app
import routes.robot as robot_module


def _client():
    return create_app().test_client()


def test_cart_scan_returns_success_from_robot_bridge(monkeypatch):
    # trunk-scan과 같은 이유(robot_bridge.py 참고) - robot_bridge.run_cart_scan()만
    # monkeypatch해서 실제 ROS2/Isaac Sim 없이 라우트의 응답 구성 로직만 검증한다.
    monkeypatch.setattr(robot_module.robot_bridge, "run_cart_scan", lambda **kw: {
        "success": True,
        "box_count": 2,
        "json_filename": "all_boxes_corners_20260727_000000_000000.json",
        "ply_filename": "all_boxes_completed_20260727_000000_000000.ply",
        "ply_total_bytes": 330836,
        "ply_total_chunks": 11,
        "raw_ply_filename": "cart_scan_raw_20260727_000000_000000.ply",
        "raw_ply_total_bytes": 1308928,
    })
    resp = _client().post("/api/robot/cart-scan")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["status"] == "ok"
    assert "카트 스캔" in body["message"]
    assert body["box_count"] == 2
    assert body["json_filename"] == "all_boxes_corners_20260727_000000_000000.json"
    assert body["json_url"] == "/api/robot/cart-scan-file/all_boxes_corners_20260727_000000_000000.json"
    assert body["ply_filename"] == "all_boxes_completed_20260727_000000_000000.ply"
    assert body["ply_url"] == "/api/robot/cart-scan-file/all_boxes_completed_20260727_000000_000000.ply"
    assert body["ply_total_bytes"] == 330836
    assert body["raw_ply_filename"] == "cart_scan_raw_20260727_000000_000000.ply"
    assert body["raw_ply_url"] == "/api/robot/cart-scan-file/cart_scan_raw_20260727_000000_000000.ply"
    assert body["raw_ply_total_bytes"] == 1308928


def test_cart_scan_returns_error_when_robot_bridge_fails(monkeypatch):
    def _raise(**kw):
        raise RuntimeError("액션 서버에 연결할 수 없습니다")
    monkeypatch.setattr(robot_module.robot_bridge, "run_cart_scan", _raise)
    resp = _client().post("/api/robot/cart-scan")
    assert resp.status_code == 502
    body = resp.get_json()
    assert body["status"] == "error"
    assert "액션 서버에 연결할 수 없습니다" in body["message"]


def test_cart_scan_file_serves_saved_file(tmp_path, monkeypatch):
    monkeypatch.setattr(robot_module.robot_bridge, "RECEIVED_SCANS_DIR", tmp_path)
    (tmp_path / "all_boxes_completed_test.ply").write_bytes(b"ply\nfake")
    resp = _client().get("/api/robot/cart-scan-file/all_boxes_completed_test.ply")
    assert resp.status_code == 200
    assert resp.data == b"ply\nfake"


def test_cart_scan_file_404_when_missing(tmp_path, monkeypatch):
    monkeypatch.setattr(robot_module.robot_bridge, "RECEIVED_SCANS_DIR", tmp_path)
    resp = _client().get("/api/robot/cart-scan-file/does_not_exist.ply")
    assert resp.status_code == 404


def test_trunk_scan_returns_success_from_robot_bridge(monkeypatch):
    # trunk-scan은 이제 더미가 아니라 robot_bridge.run_trunk_scan()(ROS2 액션 클라이언트
    # 서브프로세스 호출)을 거친다 - 실제 ROS2/Isaac Sim 없이 그 함수만 monkeypatch해서
    # 라우트의 응답 구성 로직만 검증한다.
    monkeypatch.setattr(robot_module.robot_bridge, "run_trunk_scan", lambda **kw: {
        "success": True,
        "filename": "trunk_scan_20260727_000000_000000.ply",
        "total_bytes": 100,
        "total_chunks": 1,
        "point_count": 42,
        "raw_filename": "trunk_scan_raw_20260727_000000_000000.ply",
        "raw_total_bytes": 18058669,
        "raw_point_count": 1504879,
    })
    resp = _client().post("/api/robot/trunk-scan")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["status"] == "ok"
    assert "트렁크 스캔" in body["message"]
    assert body["filename"] == "trunk_scan_20260727_000000_000000.ply"
    assert body["url"] == "/api/robot/trunk-scan-file/trunk_scan_20260727_000000_000000.ply"
    assert body["point_count"] == 42
    assert body["total_bytes"] == 100
    assert body["raw_filename"] == "trunk_scan_raw_20260727_000000_000000.ply"
    assert body["raw_url"] == "/api/robot/trunk-scan-file/trunk_scan_raw_20260727_000000_000000.ply"
    assert body["raw_point_count"] == 1504879
    assert body["raw_total_bytes"] == 18058669


def test_trunk_scan_returns_error_when_robot_bridge_fails(monkeypatch):
    def _raise(**kw):
        raise RuntimeError("액션 서버에 연결할 수 없습니다")
    monkeypatch.setattr(robot_module.robot_bridge, "run_trunk_scan", _raise)
    resp = _client().post("/api/robot/trunk-scan")
    assert resp.status_code == 502
    body = resp.get_json()
    assert body["status"] == "error"
    assert "액션 서버에 연결할 수 없습니다" in body["message"]


def test_trunk_scan_file_serves_saved_ply(tmp_path, monkeypatch):
    monkeypatch.setattr(robot_module.robot_bridge, "RECEIVED_SCANS_DIR", tmp_path)
    (tmp_path / "trunk_scan_test.ply").write_bytes(b"ply\nfake")
    resp = _client().get("/api/robot/trunk-scan-file/trunk_scan_test.ply")
    assert resp.status_code == 200
    assert resp.data == b"ply\nfake"


def test_trunk_scan_file_404_when_missing(tmp_path, monkeypatch):
    monkeypatch.setattr(robot_module.robot_bridge, "RECEIVED_SCANS_DIR", tmp_path)
    resp = _client().get("/api/robot/trunk-scan-file/does_not_exist.ply")
    assert resp.status_code == 404


def test_pick_and_place_returns_success_from_robot_bridge(monkeypatch):
    # trunk/cart-scan과 같은 이유 - robot_bridge.run_pick_and_place()(ROS2 액션
    # 클라이언트 서브프로세스 호출, 내부적으로 isaac_task_runner.py의 Trigger
    # 서비스를 대신 호출하는 pick_and_place_action_server.py를 거침)만
    # monkeypatch해서 실제 ROS2/Isaac Sim 없이 라우트의 응답 구성 로직만 검증한다.
    captured = {}

    def _fake(**kw):
        captured.update(kw)
        return {"success": True, "message": "pick_and_place 완료", "boxes_placed": 4, "boxes_total": 4}

    monkeypatch.setattr(robot_module.robot_bridge, "run_pick_and_place", _fake)
    resp = _client().post("/api/robot/pick-and-place", json={"plan_id": "load_plan_001"})
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["status"] == "ok"
    assert body["boxes_placed"] == 4
    assert body["boxes_total"] == 4
    assert captured["plan_id"] == "load_plan_001"


def test_pick_and_place_without_plan_id_defaults_to_empty_string(monkeypatch):
    captured = {}

    def _fake(**kw):
        captured.update(kw)
        return {"success": True, "message": "완료", "boxes_placed": 0, "boxes_total": 0}

    monkeypatch.setattr(robot_module.robot_bridge, "run_pick_and_place", _fake)
    resp = _client().post("/api/robot/pick-and-place")
    assert resp.status_code == 200
    assert captured["plan_id"] == ""


def test_pick_and_place_returns_error_when_robot_bridge_fails(monkeypatch):
    def _raise(**kw):
        raise RuntimeError("isaac_task_runner.py에 연결할 수 없습니다")
    monkeypatch.setattr(robot_module.robot_bridge, "run_pick_and_place", _raise)
    resp = _client().post("/api/robot/pick-and-place")
    assert resp.status_code == 502
    body = resp.get_json()
    assert body["status"] == "error"
    assert "isaac_task_runner.py에 연결할 수 없습니다" in body["message"]


def test_pick_and_place_progress_returns_events_from_robot_bridge(monkeypatch):
    events = [
        {"stage": "box_started", "box_index": 0, "box_count": 4, "box_id": "0"},
        {"stage": "box_done", "box_index": 0, "box_count": 4, "box_id": "0"},
    ]
    monkeypatch.setattr(robot_module.robot_bridge, "read_pick_and_place_progress", lambda: events)
    resp = _client().get("/api/robot/pick-and-place/progress")
    assert resp.status_code == 200
    assert resp.get_json()["events"] == events


def test_pick_and_place_progress_empty_before_any_run(monkeypatch):
    monkeypatch.setattr(robot_module.robot_bridge, "read_pick_and_place_progress", lambda: [])
    resp = _client().get("/api/robot/pick-and-place/progress")
    assert resp.status_code == 200
    assert resp.get_json()["events"] == []


def test_get_method_not_allowed():
    # 실수로 GET으로 호출하는 걸 방지하는 회귀 테스트 - 반드시 POST여야 한다.
    resp = _client().get("/api/robot/cart-scan")
    assert resp.status_code == 405


def test_list_cart_scan_files_returns_saved_filenames(monkeypatch):
    monkeypatch.setattr(
        robot_module.robot_bridge, "list_cart_scan_files",
        lambda: ["all_boxes_corners_20260727_000000_000000.json"])
    resp = _client().get("/api/robot/cart-scan-files")
    assert resp.status_code == 200
    assert resp.get_json()["cart_scan_files"] == ["all_boxes_corners_20260727_000000_000000.json"]


def test_list_cart_scan_files_empty_when_none_saved(monkeypatch):
    monkeypatch.setattr(robot_module.robot_bridge, "list_cart_scan_files", lambda: [])
    resp = _client().get("/api/robot/cart-scan-files")
    assert resp.status_code == 200
    assert resp.get_json()["cart_scan_files"] == []
