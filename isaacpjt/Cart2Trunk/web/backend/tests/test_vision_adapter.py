import json
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import vision_adapter

# "Cart2Trunk 최종 프로젝트 시나리오 및 시스템 흐름" 문서 4.2절 예시를 그대로 옮김.
_BOX_SCAN_JSON = {
    "snapshot_id": "box_scan_001",
    "frame_id": "m0609_base_link",  # 문서 예시는 "m0609_base"라 오타로 보임 -
    # ①의 확정 상수(EXPECTED_BOX_FRAME="m0609_base_link")를 기준으로 검증하므로 여기선 그 값을 씀.
    "quality_score": 0.94,
    "boxes": [
        {
            "box_id": "BOX_01", "center": [0.1, 0.2, 0.075], "size": [0.3, 0.2, 0.15],
            "yaw": 0.35, "corners": [[0.0, 0.0, 0.0]], "confidence": 0.95,
        },
        {
            "box_id": "BOX_02", "center": [0.1, 0.2, 0.06], "size": [0.25, 0.18, 0.12],
            "yaw": 0.0, "corners": [[0.0, 0.0, 0.0]], "confidence": 0.91,
        },
    ],
}


def test_boxes_from_box_scan_converts_to_simple_dicts():
    boxes, snapshot_id = vision_adapter.boxes_from_box_scan(_BOX_SCAN_JSON)

    assert snapshot_id == "box_scan_001"
    assert boxes == [
        {"id": "BOX_01", "width": 0.3, "depth": 0.2, "height": 0.15, "rests_on_id": None, "initial_yaw": 0.35},
        {"id": "BOX_02", "width": 0.25, "depth": 0.18, "height": 0.12, "rests_on_id": None, "initial_yaw": 0.0},
    ]


def test_boxes_from_box_scan_rejects_wrong_frame():
    bad = {**_BOX_SCAN_JSON, "frame_id": "camera_frame"}
    with pytest.raises(ValueError, match="camera_frame"):
        vision_adapter.boxes_from_box_scan(bad)


# 실제 준형 샘플(~/Downloads/all_boxes_corners_20260721_174311_555644.json)과 같은 구조로,
# 좌표계만 올바르게(m0609_base_link) 바꿔서 만든 합성 데이터 - 로직(오리엔티드 풋프린트,
# rests_on_id 매핑)을 검증하기 위함. box_id=2(바닥)의 top_candidate_id=0 위에 box_id=0,1이
# 나란히 얹혀 있다.
def _corner_box(box_id, top_candidate_id, support_candidate_id, support_type, x0, y0, z0, w, d, h):
    return {
        "box_id": box_id, "support_type": support_type,
        "top_candidate_id": top_candidate_id, "support_candidate_id": support_candidate_id,
        "corners_m": [
            [x0, y0, z0 + h], [x0 + w, y0, z0 + h], [x0 + w, y0 + d, z0 + h], [x0, y0 + d, z0 + h],
            [x0, y0, z0], [x0 + w, y0, z0], [x0 + w, y0 + d, z0], [x0, y0 + d, z0],
        ],
    }


_VISION_CORNERS_JSON = {
    "coordinate_frame": "m0609_base_link",
    "unit": "meter",
    "box_count": 3,
    "completed_ply_file": "run_test.ply",
    "boxes": [
        _corner_box(0, top_candidate_id=1, support_candidate_id=0, support_type="box_top",
                    x0=0.0, y0=0.0, z0=0.2, w=0.2, d=0.15, h=0.1),
        _corner_box(1, top_candidate_id=2, support_candidate_id=0, support_type="box_top",
                    x0=0.25, y0=0.0, z0=0.2, w=0.18, d=0.14, h=0.09),
        _corner_box(2, top_candidate_id=0, support_candidate_id=-1, support_type="floor",
                    x0=0.0, y0=0.0, z0=0.0, w=0.5, d=0.35, h=0.2),
    ],
}


def test_boxes_from_vision_corners_computes_oriented_footprint_and_snapshot_id():
    boxes, snapshot_id = vision_adapter.boxes_from_vision_corners(_VISION_CORNERS_JSON)

    assert snapshot_id == "run_test.ply"
    by_id = {b["id"]: b for b in boxes}
    assert by_id["2"]["width"] == pytest.approx(0.5)
    assert by_id["2"]["depth"] == pytest.approx(0.35)
    assert by_id["2"]["height"] == pytest.approx(0.2)
    assert by_id["2"]["initial_yaw"] == pytest.approx(0.0)  # 축 정렬 박스라 yaw=0


def test_boxes_from_vision_corners_uses_oriented_footprint_not_aabb_for_rotated_box():
    """[2026-07-28 회귀 테스트] 예전엔 corners_m 8점 전체의 단순 min/max(AABB)로
    width/depth를 냈는데, 박스가 회전돼 있으면 대각선 길이가 섞여 들어가서 실제
    변 길이보다 부풀려졌다(algorism/19_run_full_pipeline_with_yaw.py가 이미 경고한
    버그). 0.5x0.35 박스를 30도 회전시켜서, 오리엔티드 풋프린트로는 회전과 무관하게
    정확한 변 길이가 나오는지 확인."""
    import math

    w, d, h = 0.5, 0.35, 0.2
    theta = math.radians(30.0)
    local_corners = [(0, 0), (w, 0), (w, d), (0, d)]

    def _rot(pt):
        x, y = pt
        return (x * math.cos(theta) - y * math.sin(theta), x * math.sin(theta) + y * math.cos(theta))

    rotated_xy = [_rot(p) for p in local_corners]
    corners_m = [[x, y, h] for x, y in rotated_xy] + [[x, y, 0.0] for x, y in rotated_xy]

    data = {
        "coordinate_frame": "m0609_base_link", "unit": "meter", "box_count": 1,
        "completed_ply_file": "rotated_test.ply",
        "boxes": [{
            "box_id": 9, "support_type": "floor", "top_candidate_id": 0, "support_candidate_id": -1,
            "corners_m": corners_m,
        }],
    }

    boxes, _ = vision_adapter.boxes_from_vision_corners(data)
    box = boxes[0]

    # AABB였다면 30도 회전한 0.5x0.35 박스의 min/max 폭/깊이가 실제 변 길이보다
    # 커진다 - 오리엔티드 풋프린트는 회전과 무관하게 정확한 변 길이를 낸다.
    assert box["width"] == pytest.approx(0.5, abs=1e-9)
    assert box["depth"] == pytest.approx(0.35, abs=1e-9)
    assert box["initial_yaw"] == pytest.approx(theta, abs=1e-9)


def test_boxes_from_vision_corners_infers_rests_on_id_from_geometry():
    # box_id=0,1은 z0=0.2(=box_id=2의 높이 0.2와 정확히 맞닿는 높이)에 있고
    # XY도 box_id=2 풋프린트 안에 들어간다 - corners_m 좌표만으로 "2 위에
    # 얹혀있다"고 판정돼야 한다(support_candidate_id/top_candidate_id는 이제
    # 안 봄 - 아래 회귀 테스트 참고).
    boxes, _ = vision_adapter.boxes_from_vision_corners(_VISION_CORNERS_JSON)
    by_id = {b["id"]: b for b in boxes}

    assert by_id["0"]["rests_on_id"] == "2"
    assert by_id["1"]["rests_on_id"] == "2"
    assert by_id["2"]["rests_on_id"] is None  # 바닥


def test_boxes_from_vision_corners_ignores_duplicate_top_candidate_ids():
    # [2026-07-28 회귀 테스트] 실제 4박스 스캔(all_boxes_corners_20260728_052906_
    # 279709.json)에서 서로 다른 두 박스(box_id=2,3)가 똑같이 top_candidate_id=3을
    # 가진 게 확인됐다 - support_candidate_id/top_candidate_id로 매칭하던 옛
    # 방식이면 이런 충돌 때문에 관계를 못 찾고 전부 "바닥"으로 잘못 판정되어
    # (실제 증상: "큰 박스를 먼저 넣으려는데 그 위에 작은 박스가 있어서
    # 불가능하다"는 물리적으로 말이 안 되는 적재 순서가 나왔었다). 여기선
    # top_candidate_id를 일부러 전부 9로 충돌시켜도(0,1이 여전히 2 위에 기하학적
    # 으로 있으므로) rests_on_id가 정확히 나와야 한다.
    colliding = {
        **_VISION_CORNERS_JSON,
        "boxes": [
            {**b, "top_candidate_id": 9, "support_candidate_id": -1}
            for b in _VISION_CORNERS_JSON["boxes"]
        ],
    }
    boxes, _ = vision_adapter.boxes_from_vision_corners(colliding)
    by_id = {b["id"]: b for b in boxes}

    assert by_id["0"]["rests_on_id"] == "2"
    assert by_id["1"]["rests_on_id"] == "2"
    assert by_id["2"]["rests_on_id"] is None


def test_boxes_from_vision_corners_rejects_wrong_frame():
    bad = {**_VISION_CORNERS_JSON, "coordinate_frame": "depth_camera_optical_frame_from_message_header"}
    with pytest.raises(ValueError, match="depth_camera_optical_frame_from_message_header"):
        vision_adapter.boxes_from_vision_corners(bad)


# 준형이 실제로 넘겨준 샘플 파일 - 다운로드돼 있으면 "지금 이 파일을 그대로
# 돌리면 좌표계 불일치로 막힌다"는 걸 회귀 테스트로 고정해둔다(안전 문제라
# 임의로 우회하면 안 됨 - vision_adapter.boxes_from_vision_corners 참고).
_REAL_SAMPLE = pathlib.Path.home() / "Downloads" / "all_boxes_corners_20260721_174311_555644.json"


@pytest.mark.skipif(not _REAL_SAMPLE.exists(), reason="실제 샘플 파일이 이 환경에 없음")
def test_real_vision_sample_currently_blocked_by_frame_mismatch():
    data = json.loads(_REAL_SAMPLE.read_text())
    with pytest.raises(ValueError, match="depth_camera_optical_frame_from_message_header"):
        vision_adapter.boxes_from_vision_corners(data)
