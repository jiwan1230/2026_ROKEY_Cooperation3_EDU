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


def test_pick_and_place_returns_dummy_success(monkeypatch):
    monkeypatch.setattr(robot_module, "DUMMY_DELAY_SECONDS", 0)
    resp = _client().post("/api/robot/pick-and-place")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["status"] == "ok"
    assert "픽앤플레이스" in body["message"]


def test_get_method_not_allowed():
    # 실수로 GET으로 호출하는 걸 방지하는 회귀 테스트 - 반드시 POST여야 한다.
    resp = _client().get("/api/robot/cart-scan")
    assert resp.status_code == 405
