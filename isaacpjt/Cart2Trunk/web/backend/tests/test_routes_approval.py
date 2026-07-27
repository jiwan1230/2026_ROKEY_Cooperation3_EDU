import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from app import create_app
import algorism_bridge as bridge
import robot_bridge

_PLACED = [{
    "box_id": "A", "order": 1, "position": [0.1, 0.2, 0.0], "dimensions": [0.3, 0.2, 0.15],
    "score": 0.5, "touches": 2, "rotated": False, "target_yaw": 0.0,
}]


def test_approve_then_send_happy_path(monkeypatch, tmp_path):
    monkeypatch.setattr(bridge, "_PENDING_TASKS_DIR", tmp_path)
    # robot_bridge.send_placement_plan()은 실제로 ros2 run을 서브프로세스로
    # 띄운다 - 이 라우트 로직만 검증하려는 유닛 테스트라 그 함수 자체는
    # monkeypatch(실제 ROS2/Isaac Sim 없이 라우트의 응답 구성만 확인).
    monkeypatch.setattr(robot_bridge, "send_placement_plan", lambda task: {
        "success": True, "message": "1개 박스 배치 계획을 저장했습니다",
        "written_path": "/fake/placement_result.json",
    })
    client = create_app().test_client()

    approve_resp = client.post("/api/approve", json={
        "box_snapshot_id": "snap_1", "trunk_map_id": "trunk_1",
        "parameters": {"mode": "large_first"}, "placed": _PLACED,
    })
    assert approve_resp.status_code == 200
    task = approve_resp.get_json()["task"]
    assert task["approved"] is True

    send_resp = client.post("/api/send", json={"task": task})
    assert send_resp.status_code == 200
    body = send_resp.get_json()
    assert pathlib.Path(body["out_path"]).exists()
    assert body["msi2_written_path"] == "/fake/placement_result.json"


def test_send_returns_error_when_msi2_transmission_fails(monkeypatch, tmp_path):
    monkeypatch.setattr(bridge, "_PENDING_TASKS_DIR", tmp_path)

    def _raise(task):
        raise RuntimeError("send_placement_plan_service에 연결할 수 없습니다")
    monkeypatch.setattr(robot_bridge, "send_placement_plan", _raise)
    client = create_app().test_client()

    approve_resp = client.post("/api/approve", json={
        "box_snapshot_id": "snap_1", "trunk_map_id": "trunk_1",
        "parameters": {}, "placed": _PLACED,
    })
    task = approve_resp.get_json()["task"]

    send_resp = client.post("/api/send", json={"task": task})
    assert send_resp.status_code == 502
    body = send_resp.get_json()
    assert body["status"] == "error"
    assert "send_placement_plan_service에 연결할 수 없습니다" in body["message"]
    # 로컬 저장(감사 기록)은 MSI2 전송 실패와 무관하게 이미 됐어야 한다.
    assert pathlib.Path(body["out_path"]).exists()


def test_approve_without_placed_returns_400():
    client = create_app().test_client()
    resp = client.post("/api/approve", json={})
    assert resp.status_code == 400
    assert resp.get_json()["error_code"] == "NO_PLAN_TO_APPROVE"


def test_send_without_task_returns_400():
    client = create_app().test_client()
    resp = client.post("/api/send", json={})
    assert resp.status_code == 400
    assert resp.get_json()["error_code"] == "NO_TASK_TO_SEND"
