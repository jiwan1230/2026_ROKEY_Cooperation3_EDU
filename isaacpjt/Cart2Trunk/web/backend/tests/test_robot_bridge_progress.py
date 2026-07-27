import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import robot_bridge


def test_read_pick_and_place_progress_returns_empty_when_file_missing(tmp_path, monkeypatch):
    monkeypatch.setattr(robot_bridge, "PICK_AND_PLACE_PROGRESS_FILE", tmp_path / "does_not_exist.jsonl")
    assert robot_bridge.read_pick_and_place_progress() == []


def test_read_pick_and_place_progress_parses_json_lines(tmp_path, monkeypatch):
    progress_file = tmp_path / "progress.jsonl"
    progress_file.write_text(
        '{"stage": "started", "box_index": 0, "box_count": 0, "box_id": ""}\n'
        '{"stage": "box_started", "box_index": 0, "box_count": 4, "box_id": "0"}\n'
    )
    monkeypatch.setattr(robot_bridge, "PICK_AND_PLACE_PROGRESS_FILE", progress_file)

    events = robot_bridge.read_pick_and_place_progress()

    assert events == [
        {"stage": "started", "box_index": 0, "box_count": 0, "box_id": ""},
        {"stage": "box_started", "box_index": 0, "box_count": 4, "box_id": "0"},
    ]


def test_read_pick_and_place_progress_skips_incomplete_trailing_line(tmp_path, monkeypatch):
    # pick_and_place_client가 한창 append 중일 때 폴링하면 마지막 줄이
    # 아직 다 안 써졌을 수 있다 - 그 줄만 건너뛰고 나머지는 그대로 반환해야 한다.
    progress_file = tmp_path / "progress.jsonl"
    progress_file.write_text(
        '{"stage": "started", "box_index": 0, "box_count": 0, "box_id": ""}\n'
        '{"stage": "box_star'
    )
    monkeypatch.setattr(robot_bridge, "PICK_AND_PLACE_PROGRESS_FILE", progress_file)

    events = robot_bridge.read_pick_and_place_progress()

    assert events == [{"stage": "started", "box_index": 0, "box_count": 0, "box_id": ""}]


def test_list_cart_scan_files_returns_empty_when_dir_missing(tmp_path, monkeypatch):
    monkeypatch.setattr(robot_bridge, "RECEIVED_SCANS_DIR", tmp_path / "does_not_exist")
    assert robot_bridge.list_cart_scan_files() == []


def test_list_cart_scan_files_filters_and_sorts(tmp_path, monkeypatch):
    monkeypatch.setattr(robot_bridge, "RECEIVED_SCANS_DIR", tmp_path)
    (tmp_path / "all_boxes_corners_20260727_120000_000000.json").write_text("{}")
    (tmp_path / "all_boxes_corners_20260726_090000_000000.json").write_text("{}")
    (tmp_path / "all_boxes_completed_20260727_120000_000000.ply").write_bytes(b"ply")
    (tmp_path / "trunk_scan_20260727_120000_000000.ply").write_bytes(b"ply")

    assert robot_bridge.list_cart_scan_files() == [
        "all_boxes_corners_20260726_090000_000000.json",
        "all_boxes_corners_20260727_120000_000000.json",
    ]
