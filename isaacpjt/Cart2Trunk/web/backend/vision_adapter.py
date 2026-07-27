"""
vision_adapter.py
비전(준형)이 만드는 원본 JSON 두 가지를 algorism_bridge.compute_plan()이 바로
받는 단순 박스 딕셔너리 목록({id,width,depth,height,rests_on_id})으로 바꾼다.

- box_scan.json: "Cart2Trunk 최종 프로젝트 시나리오 및 시스템 흐름" 문서
  4.2절이 정의한 "목표" 스키마 (아직 준형의 실제 최종 출력이 이 형태인지는
  미확인).
- all_boxes_corners_*.json: 준형이 실제로 넘겨준 샘플 파일의 스키마
  (~/Downloads/all_boxes_corners_20260721_174311_555644.json로 확인).

algorism/ 파일은 이 프로젝트 전체에서 수정 금지라(algorism_bridge.py 최상단
docstring, 05_candidate_scoring.py 주석 등 참고) algorism/01_object3d_schema.py의
기존 로더(load_box_snapshot_from_json, object3d_to_box)를 그대로 재사용하되,
그 파일이 아직 다루지 않는 부분은 여기서 처리한다:
  - all_boxes_corners_*.json의 support_type/support_candidate_id ->
    rests_on_id 매핑 (load_boxes_from_vision_json()은 이걸 안 함)
  - 좌표계가 안 맞을 때 "임의로 변환하지 않고 명확히 에러로 막기"
"""
import math
import sys
import pathlib
from importlib import import_module
from typing import Dict, List, Tuple

_HERE = pathlib.Path(__file__).resolve().parent
_ALGORISM_DIR = _HERE.parent.parent / "algorism"
if str(_ALGORISM_DIR) not in sys.path:
    sys.path.insert(0, str(_ALGORISM_DIR))

_m01 = import_module("01_object3d_schema")
load_box_snapshot_from_json = _m01.load_box_snapshot_from_json
object3d_to_box = _m01.object3d_to_box
EXPECTED_BOX_FRAME = _m01.EXPECTED_BOX_FRAME
_infer_rests_on_ids = _m01._infer_rests_on_ids


def boxes_from_box_scan(box_scan_data: dict) -> Tuple[List[dict], str]:
    """"시스템 흐름" 문서 4.2절 box_scan.json 스키마를 파싱한다. algorism/01의
    load_box_snapshot_from_json()을 그대로 쓴다 - frame_id 검증(안 맞으면
    ValueError)까지 그쪽 로직 그대로 적용된다."""
    snapshot = load_box_snapshot_from_json(box_scan_data)
    boxes = []
    for obj in snapshot.boxes:
        box = object3d_to_box(obj)
        boxes.append({
            "id": box.id, "width": box.width, "depth": box.depth, "height": box.height,
            "rests_on_id": box.rests_on_id, "initial_yaw": box.initial_yaw,
        })
    return boxes, snapshot.snapshot_id


def boxes_from_vision_corners(vision_data: dict) -> Tuple[List[dict], str]:
    """준형의 실제 박스 비전 출력(all_boxes_corners_*.json) 스키마를 파싱한다.

    [rests_on_id 매핑 - 2026-07-28 수정: support_candidate_id/top_candidate_id
    매칭 방식 폐기]
    처음엔 "support_candidate_id가 다른 박스의 top_candidate_id를 가리킨다"고
    보고 그 둘을 매칭했었다(3박스 샘플에서는 우연히 잘 맞았음). 그런데 실제
    4박스 스캔(all_boxes_corners_20260728_052906_279709.json)에서 서로 다른
    박스 2개가 똑같이 top_candidate_id=3을 갖는 게 확인됐다 - algorism/
    01_object3d_schema.py가 이미 정확히 이 문제를 경고하고 있었다: "비전이
    주는 support_candidate_id/top_candidate_id는 RANSAC 트라이얼마다 새로
    매겨지는 임시 번호라 서로 다른 박스가 같은 값을 가질 수 있어 id 매칭에
    쓸 수 없다"(같은 파일 _infer_rests_on_ids() 위 주석 참고). ID 매칭이
    깨지면 전부 "바닥에 놓임"으로 잘못 판정되어, 픽업 순서 제약(06_loading_
    order_decision.py)이 사실상 꺼진 것과 같아지고 "밑에 깔린 큰 박스를
    먼저 집으려는" 물리적으로 불가능한 순서가 나온다(사용자가 실제로 겪은
    증상).

    01_object3d_schema.py는 이미 이 문제의 정답을 갖고 있었다 - id가 아니라
    corners_m(실제 3D 좌표)로 기하학적으로 판정하는 _infer_rests_on_ids()를
    algorism/01이 제공하므로, 여기서도 support_candidate_id/top_candidate_id를
    아예 안 보고 그 함수를 그대로 재사용한다(algorism/ 수정 금지 원칙 - 새로
    만들지 않고 이미 있는 걸 씀).

    [좌표계 - 미해결, 임의 변환 안 함]
    실제 샘플의 coordinate_frame은 "depth_camera_optical_frame_from_message_
    header"인데 알고리즘이 요구하는 건 "m0609_base_link"(EXPECTED_BOX_FRAME)다.
    카메라->로봇 base 외부 파라미터(extrinsic calibration) 없이 임의로 좌표를
    옮기면 박스가 엉뚱한 자리로 계산되는 게 조용히 넘어갈 위험이 있어(안전
    문제), algorism/01과 동일하게 프레임이 안 맞으면 명확한 에러로 막는다 -
    Vision(준형)/시스템통합(지완)과 좌표계를 맞춘 뒤에야 실제 데이터로 계산할
    수 있다는 뜻이다.
    """
    frame = vision_data.get("coordinate_frame")
    if frame != EXPECTED_BOX_FRAME:
        raise ValueError(
            f"박스 비전 데이터의 좌표계가 '{frame}'인데 '{EXPECTED_BOX_FRAME}'이어야 합니다 - "
            f"카메라 좌표계를 로봇 base 좌표계로 변환해서 다시 내보내달라고 "
            f"Vision(준형)/시스템통합(지완)과 확인하세요."
        )

    raw_boxes = vision_data.get("boxes", [])

    # rests_on_id 판정에 쓸 AABB를 먼저 전부 모은다 - _infer_rests_on_ids()가
    # "이 박스보다 아래에 있으면서 바닥이 거의 맞닿고 XY가 겹치는" 부모를
    # 찾으려면 전체 박스의 AABB가 한꺼번에 필요하다(한 박스씩 순서대로는 못 함).
    box_ids: List[str] = []
    corners_by_index: List[list] = []
    box_aabbs: Dict[str, tuple] = {}
    for b in raw_boxes:
        corners = b["corners_m"]
        box_id = str(b["box_id"])
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        zs = [c[2] for c in corners]
        box_ids.append(box_id)
        corners_by_index.append(corners)
        box_aabbs[box_id] = (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))

    rests_on_by_id = _infer_rests_on_ids(box_aabbs)

    boxes = []
    for box_id, corners in zip(box_ids, corners_by_index):
        # [2026-07-28 수정] 예전엔 corners_m 8점 전체의 단순 min/max(AABB)로
        # width/depth를 냈는데, 이건 algorism/19_run_full_pipeline_with_yaw.py가
        # 이미 경고하는 그 버그다 - 박스가 축 정렬이 아니면(회전된 채로 스캔되면)
        # AABB 변 길이가 실제 변 길이보다 부풀려진다(예: 정사각형을 45도 돌리면
        # AABB 변이 대각선 길이가 됨). 19번과 동일하게 윗면 4점의 인접 변 길이로
        # 계산해서 회전과 무관하게 정확한 크기를 낸다 - box_top_extractor.py가
        # corners_m[:4](윗면)를 이미 일관된 회전 순서로 저장해두므로 인접한 두
        # 코너 사이 거리가 곧 실제 변 길이다.
        top = corners[:4]
        edge01 = (top[1][0] - top[0][0], top[1][1] - top[0][1])
        edge12 = (top[2][0] - top[1][0], top[2][1] - top[1][1])
        width = math.hypot(*edge01)
        depth = math.hypot(*edge12)
        zs = [c[2] for c in corners]
        height = max(zs) - min(zs)
        # 박스가 카트 위에서 원래 가지고 있던 yaw(도, [0,180) 범위 - 사각형은
        # 180도 대칭이라 그 이상은 구분 불가) - 19번의 _oriented_footprint()와
        # 동일 공식. algorism/01_object3d_schema.py의 Box.initial_yaw가 이
        # 값을 받아서 07_placement_plan.py의 target_yaw 계산(box.initial_yaw +
        # rotated?90:0)에 쓰인다 - 예전엔 이 필드가 없어서 항상 0.0으로 빠져
        # 있었다(모든 박스가 축 정렬이라고 잘못 가정한 셈).
        yaw_deg = math.degrees(math.atan2(edge01[1], edge01[0])) % 180.0
        initial_yaw = math.radians(yaw_deg)

        boxes.append({
            "id": box_id, "width": width, "depth": depth, "height": height,
            "rests_on_id": rests_on_by_id.get(box_id), "initial_yaw": initial_yaw,
        })

    snapshot_id = vision_data.get("completed_ply_file") or f"vision_corners_{len(boxes)}boxes"
    return boxes, snapshot_id
