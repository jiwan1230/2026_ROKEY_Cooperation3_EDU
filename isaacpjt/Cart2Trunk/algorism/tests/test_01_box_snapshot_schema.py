"""
test_01_box_snapshot_schema.py
① load_box_snapshot_from_json() - "Cart2Trunk 최종 프로젝트 시나리오 및 시스템
흐름" 문서 4.2절이 정의한 box_scan.json 스키마(center/size/yaw/corners/
confidence/snapshot_id)를 파싱하는 새 로더.

주의: 이건 기존 load_boxes_from_vision_json()(실제 샘플 all_boxes_corners_*.json,
corners_m만 있고 yaw/snapshot_id 없음)과는 다른 스키마다. 문서가 정의한 "목표"
스키마 기준으로 만들었고, 준형 쪽 실제 출력이 어느 쪽인지(혹은 둘 다인지)는
아직 확정 안 됨 - 그래서 두 로더를 별개로 남겨둔다.
"""
import json
import sys
import pathlib
from importlib import import_module

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # tests/ -> algorism/
_m01 = import_module("01_object3d_schema")

load_box_snapshot_from_json = _m01.load_box_snapshot_from_json
BoxSnapshot = _m01.BoxSnapshot
Object3D = _m01.Object3D
EXPECTED_BOX_FRAME = _m01.EXPECTED_BOX_FRAME


def _write_json(tmp_path, data):
    path = tmp_path / "box_scan.json"
    path.write_text(json.dumps(data))
    return str(path)


def _sample_data(**overrides):
    data = {
        "snapshot_id": "box_scan_001",
        "frame_id": EXPECTED_BOX_FRAME,
        "quality_score": 0.94,
        "boxes": [
            {
                "box_id": "BOX_01",
                "center": [0.5, 0.1, 0.2],
                "size": [0.3, 0.2, 0.15],
                "yaw": 0.35,
                "corners": [[0.0, 0.0, 0.0]],
                "confidence": 0.95,
            }
        ],
    }
    data.update(overrides)
    return data


def test_parses_snapshot_metadata_and_box_fields():
    snapshot = load_box_snapshot_from_json(_sample_data())

    assert isinstance(snapshot, BoxSnapshot)
    assert snapshot.snapshot_id == "box_scan_001"
    assert snapshot.frame_id == EXPECTED_BOX_FRAME
    assert snapshot.quality_score == 0.94
    assert len(snapshot.boxes) == 1

    box = snapshot.boxes[0]
    assert isinstance(box, Object3D)
    assert box.id == "BOX_01"
    assert box.center_xyz == (0.5, 0.1, 0.2)
    assert box.size_xyz == (0.3, 0.2, 0.15)
    assert box.yaw == pytest.approx(0.35)
    assert box.confidence == pytest.approx(0.95)
    assert box.volume == pytest.approx(0.3 * 0.2 * 0.15)


def test_missing_yaw_and_confidence_default_gracefully():
    data = _sample_data()
    del data["boxes"][0]["yaw"]
    del data["boxes"][0]["confidence"]

    snapshot = load_box_snapshot_from_json(data)

    assert snapshot.boxes[0].yaw == 0.0
    assert snapshot.boxes[0].confidence == 1.0


def test_wrong_frame_raises_instead_of_silently_misplacing():
    data = _sample_data(frame_id="camera_optical_frame")

    with pytest.raises(ValueError, match="frame_id"):
        load_box_snapshot_from_json(data)


def test_accepts_dict_or_file_path(tmp_path):
    data = _sample_data()
    path = _write_json(tmp_path, data)

    from_dict = load_box_snapshot_from_json(data)
    from_path = load_box_snapshot_from_json(path)

    assert from_dict.snapshot_id == from_path.snapshot_id == "box_scan_001"
