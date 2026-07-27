# isaacpjt/Cart2Trunk/web/backend/tests/test_algorism_bridge_resources.py
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import algorism_bridge as bridge


def _isolate_received_scans_dir(tmp_path, monkeypatch):
    """list_trunk_maps()가 robot_bridge.RECEIVED_SCANS_DIR도 glob하므로, 이
    머신의 실제 received_scans/ 안 파일이 테스트 결과에 우연히 섞이지 않도록
    모든 트렁크 맵 목록 테스트에서 빈 디렉터리로 고정한다."""
    empty_dir = tmp_path / "received_scans_unused"
    monkeypatch.setattr(bridge.robot_bridge, "RECEIVED_SCANS_DIR", empty_dir)


def test_list_trunk_maps_finds_run_folders(tmp_path, monkeypatch):
    monkeypatch.setattr(bridge, "_RESULTS_DIR", tmp_path)
    _isolate_received_scans_dir(tmp_path, monkeypatch)
    run_dir = tmp_path / "run_20260101_000000" / "pointcloud"
    run_dir.mkdir(parents=True)
    (run_dir / "trunk_map.json").write_text("{}")

    assert bridge.list_trunk_maps() == ["run_20260101_000000"]


def test_list_trunk_maps_empty_when_none_found(tmp_path, monkeypatch):
    monkeypatch.setattr(bridge, "_RESULTS_DIR", tmp_path)
    _isolate_received_scans_dir(tmp_path, monkeypatch)
    assert bridge.list_trunk_maps() == []


def test_list_trunk_maps_includes_holonomic_base_when_present(tmp_path, monkeypatch):
    # robot_bridge.run_trunk_scan()으로 이어지는 실제 트렁크 스캔 파이프라인
    # (90.export_trunk_map_holonomic.py)은 run_* 폴더가 아니라 이 고정 경로
    # 하나를 스캔마다 덮어쓴다 - 웹 백엔드와 Isaac Sim이 "같은 머신"에서 돌 때만
    # 로컬에 존재한다(회귀 테스트 - 하드코딩된 남의 PC 경로 /home/sunwook/...
    # 때문에 이게 항상 빠졌었다).
    monkeypatch.setattr(bridge, "_RESULTS_DIR", tmp_path)
    _isolate_received_scans_dir(tmp_path, monkeypatch)
    holonomic_dir = tmp_path / "holonomic_base"
    holonomic_dir.mkdir(parents=True)
    (holonomic_dir / "trunk_map.json").write_text("{}")

    assert bridge.list_trunk_maps() == ["holonomic_base"]


def test_list_trunk_maps_includes_received_scans_trunk_maps_when_present(tmp_path, monkeypatch):
    # MSI1/MSI2가 실제로 분리된 환경에서 진짜로 쓰이는 경로 - trunk_scan_client가
    # ROS2 Action으로 받은 trunk_map.json 내용을 스캔마다 새 타임스탬프 파일로
    # 저장한다(trunk_scan_action_server.py/trunk_scan_action_client.py 참고).
    # results/holonomic_base처럼 로컬에 Isaac Sim이 안 떠 있어도 이 경로엔
    # 파일이 생긴다.
    monkeypatch.setattr(bridge, "_RESULTS_DIR", tmp_path / "results_unused")
    received_dir = tmp_path / "received_scans"
    received_dir.mkdir(parents=True)
    monkeypatch.setattr(bridge.robot_bridge, "RECEIVED_SCANS_DIR", received_dir)
    (received_dir / "trunk_map_20260728_060000_000000.json").write_text("{}")
    (received_dir / "all_boxes_corners_20260728_060000_000000.json").write_text("{}")  # 카트 스캔 파일 - 섞이면 안 됨

    assert bridge.list_trunk_maps() == ["trunk_map_20260728_060000_000000"]


def test_trunk_map_path_resolves_received_scans_trunk_map(tmp_path, monkeypatch):
    monkeypatch.setattr(bridge, "_RESULTS_DIR", tmp_path / "results_unused")
    received_dir = tmp_path / "received_scans"
    received_dir.mkdir(parents=True)
    monkeypatch.setattr(bridge.robot_bridge, "RECEIVED_SCANS_DIR", received_dir)
    (received_dir / "trunk_map_20260728_060000_000000.json").write_text("{}")

    resolved = bridge._trunk_map_path("trunk_map_20260728_060000_000000")
    assert resolved == received_dir / "trunk_map_20260728_060000_000000.json"


def test_list_trunk_maps_sorts_all_three_sources_by_mtime(tmp_path, monkeypatch):
    import os
    import time

    monkeypatch.setattr(bridge, "_RESULTS_DIR", tmp_path)
    run_dir = tmp_path / "run_20260101_000000" / "pointcloud"
    run_dir.mkdir(parents=True)
    (run_dir / "trunk_map.json").write_text("{}")

    holonomic_dir = tmp_path / "holonomic_base"
    holonomic_dir.mkdir(parents=True)
    time.sleep(0.01)
    (holonomic_dir / "trunk_map.json").write_text("{}")

    received_dir = tmp_path / "received_scans"
    received_dir.mkdir(parents=True)
    monkeypatch.setattr(bridge.robot_bridge, "RECEIVED_SCANS_DIR", received_dir)
    (received_dir / "trunk_map_20260728_060000_000000.json").write_text("{}")

    # 확실히 순서가 갈리도록 mtime을 명시적으로 못박는다(파일시스템 타임스탬프
    # 해상도가 낮은 환경에서도 안정적으로 통과하도록).
    now = time.time()
    os.utime(run_dir / "trunk_map.json", (now, now))
    os.utime(holonomic_dir / "trunk_map.json", (now + 10, now + 10))
    os.utime(received_dir / "trunk_map_20260728_060000_000000.json", (now + 20, now + 20))

    assert bridge.list_trunk_maps() == [
        "run_20260101_000000", "holonomic_base", "trunk_map_20260728_060000_000000",
    ]


def test_trunk_map_path_resolves_holonomic_base_without_pointcloud_subdir(tmp_path, monkeypatch):
    monkeypatch.setattr(bridge, "_RESULTS_DIR", tmp_path)
    _isolate_received_scans_dir(tmp_path, monkeypatch)
    holonomic_dir = tmp_path / "holonomic_base"
    holonomic_dir.mkdir(parents=True)
    (holonomic_dir / "trunk_map.json").write_text("{}")

    assert bridge._trunk_map_path("holonomic_base") == holonomic_dir / "trunk_map.json"


def test_list_box_presets_includes_default_and_example_files(tmp_path, monkeypatch):
    monkeypatch.setattr(bridge, "_LOCAL_TEST_DATA_DIR", tmp_path)
    custom = [{"id": "A", "width": 0.2, "depth": 0.2, "height": 0.2}]
    (tmp_path / "example_cart_boxes_stress.json").write_text(json.dumps(custom))

    presets = bridge.list_box_presets()

    assert presets["기본값 (Large/Medium/Small)"] == bridge._DEFAULT_CART_BOXES
    assert presets["stress"] == custom


def test_color_for_box_id_is_stable():
    assert bridge.color_for_box_id("Large") == bridge.color_for_box_id("Large")


def test_color_for_box_id_returns_valid_hex_color():
    color = bridge.color_for_box_id("Box1")
    assert len(color) == 7 and color.startswith("#")
    int(color[1:], 16)  # 유효한 16진수여야 함 (예외 안 나면 통과)


def test_color_for_box_id_spreads_many_ids_across_distinct_hues():
    # 팔레트가 7개짜리로 고정돼 있던 예전 버전은 박스가 7개를 넘으면 색이
    # 반드시 겹쳤다 - 지금은 hue를 0~360 전체에서 뽑으므로, 실전에서 흔한
    # 규모(예: 20개)의 박스 id에서는 대부분 서로 다른 색이 나와야 한다.
    ids = [f"Box{i}" for i in range(20)]
    colors = {bridge.color_for_box_id(i) for i in ids}
    assert len(colors) >= 15  # 완전한 유일성은 보장 안 하지만(hue 충돌 가능) 대부분은 달라야 함


def test_generate_random_boxes_count_and_ranges():
    boxes = bridge.generate_random_boxes(5)
    assert len(boxes) == 5
    for b in boxes:
        assert 0.15 <= b["width"] <= 0.45
