#!/usr/bin/env python3
"""
multiview_scan.py
여러 시점에서 모아 base_link 좌표계로 이미 합쳐진 point cloud 파일(.npy, Nx3) 하나를
읽어서, box_geometry.py의 프레임-무관 검출 로직으로 박스 8개 꼭짓점을 뽑고
box_top_extractor.py의 save_current_cloud()와 동일한 JSON/PLY 계약으로 저장한다.

box_top_extractor.py와 달리 ROS2 depth 토픽을 구독하지 않는다(이미 다 모아진
point cloud 하나를 오프라인으로 한 번만 처리) - 그래서 rclpy/cv_bridge가 필요 없고
open3d/opencv/numpy만 있으면 된다(perception/.venv에 이미 있음).

좌표계: 입력 point cloud는 이미 m0609_base_link 좌표계라고 가정한다(35.crate_scan_setup.py
쪽에서 카메라 world_frame point cloud를 base_link로 변환해서 저장함) - 그래서
box_top_extractor.py가 저장 직전에 하던 camera->base_link 변환이 필요 없다.
"""

from __future__ import annotations

import argparse
import json
import os
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import open3d as o3d

import box_geometry as bg

UP_VECTOR = np.array([0.0, 0.0, 1.0], dtype=np.float64)
DOWN_VECTOR = np.array([0.0, 0.0, -1.0], dtype=np.float64)

OUTPUT_FRAME = "m0609_base_link"
SAVE_DIRECTORY = Path.home() / "box_pointcloud"

DEBUG_SUPPORT = os.environ.get("CART2TRUNK_DEBUG_SUPPORT", "0") == "1"

# Open3D의 segment_plane()은 시드 고정 없는 RANSAC이라, 완전히 같은(정적인) 전처리된
# point cloud를 다시 검출해도 매번 평면이 다르게 쪼개진다 - 실측 확인: Small처럼
# 노출 면적이 작은 박스는 이 노이즈 때문에 한 번의 검출 시도만으로는 fill_ratio는
# 넘기지만 크기가 완전히 틀어진("그럴듯한 오검출") 조각이 뽑힐 수 있다(같은 입력을
# 5번 반복 검출했더니 footprint가 0.046~0.162 사이를 오갔음, 그 중 진짜 크기(0.13x0.10)에
# 가까운 건 fill_ratio가 가장 높았던 시도뿐이었다). 로봇을 다시 움직이지 않고도
# 같은 병합 point cloud에 대해 검출만 여러 번 반복해서, 같은 물리적 위치에서 나온
# 후보들 중 fill_ratio가 가장 높은(=가장 완전한 사각형에 가까운) 것을 채택한다 -
# run_scan_once.py가 "여러 프레임 관찰 후 최빈값 채택"으로 카메라 프레임 노이즈를
# 우회했던 것과 같은 원리를, 이제는 노이즈의 실제 근원(RANSAC 시드)에 대해 직접 반복한다.
DETECTION_TRIALS = int(os.environ.get("CART2TRUNK_DETECTION_TRIALS", "12"))
# 같은 물리적 박스로 볼 중심 간 거리(m) - 테이블 위 실제 박스 간 최소 간격보다
# 훨씬 작게 잡아서 서로 다른 박스를 하나로 합치지 않게 한다.
DETECTION_GROUP_RADIUS_M = float(os.environ.get("CART2TRUNK_DETECTION_GROUP_RADIUS_M", "0.06"))
# 전체 시도 중 이 비율 미만으로만 나타난 후보는 노이즈(우연히 한 번 걸린 조각)로 보고
# 버린다 - "여러 시도 중 얼마나 일관되게 같은 자리에서 나오는가"도 신뢰도 신호로 쓴다.
DETECTION_MIN_APPEARANCE_FRACTION = float(
    os.environ.get("CART2TRUNK_DETECTION_MIN_APPEARANCE_FRACTION", "0.25")
)

# box_geometry.MIN_BOX_SIDE_M(0.04m)은 일반 파이프라인의 낮은 하한이라(다른
# 용도로 얇은 후보도 통과시켜야 하는 legacy 단일 시점 경로에 영향을 주지 않기 위해
# box_geometry.py 쪽 값은 건드리지 않는다), 실측 확인: 폭 4.5cm짜리 가늘고 긴
# RANSAC 조각(진짜 박스가 아님 - 이 데모의 실제 박스 중 가장 좁은 변도 0.12m)이
# 이 기준을 겨우 통과해서 최종 결과에 유령 4번째 박스로 남는 사례가 있었다.
# multiview_scan 쪽(최종 선택 단계)에서만 더 엄격한 하한을 추가로 적용한다 -
# 실제 박스 카탈로그 중 가장 좁은 변(Medium 0.12~0.13m)보다는 넉넉히 낮게,
# 관측된 유령 조각(0.045m)보다는 확실히 높게 잡는다.
MIN_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M = float(
    os.environ.get("CART2TRUNK_MIN_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M", "0.08")
)

# 88.cart_scan_holonomic.py(카트 바스켓 스캔)에서 실측 확인: 카트 자체의 철망
# 테두리(림)가 하나의 거대한 평면 후보(0.57x0.77m)로 검출되고, 마침 fill_ratio도
# 높게(0.98+) 나와서 유령 박스로 남았다 - 이 데모의 실제 박스는 테이블/카트
# 시나리오를 통틀어 가장 큰 것도 한 변 0.23m를 넘지 않는다. 실제 박스보다는
# 넉넉히 크게, 카트 림처럼 명백히 큰 구조물보다는 확실히 작게 잡는다.
MAX_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M = float(
    os.environ.get("CART2TRUNK_MAX_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M", "0.35")
)

# 같은 이유(3-10절)로 실측 확인: fill_ratio가 매우 낮은(~0.50, 사각형의 절반
# 정도만 실제로 채워진) 후보가 간헐적으로(5회 중 2회) 최소 등장 횟수 기준은
# 통과해서 유령 4번째 박스로 남는 사례가 있었다 - 진짜 박스 윗면은 8회 이상
# 반복 관측에서 항상 fill_ratio 0.9 이상이었다. 최종 선택 단계에서 이 하한
# 미만인 후보도 제외한다.
MIN_FINAL_FILL_RATIO = float(os.environ.get("CART2TRUNK_MIN_FINAL_FILL_RATIO", "0.75"))


# 최종 선택된 박스들 중, 사각형(윗면 footprint)이 겹치는 후보를 한 번 더 정리하기
# 위한 기준 - _group_by_location()의 반경 밖에서도 실제 사각형은 겹칠 수 있어서
# 별도로 둔다. 높이가 이 이내로 같아야만(=진짜 적층이 아니어야만) 적용한다.
DEDUP_OVERLAP_Z_TOLERANCE_M = float(os.environ.get("CART2TRUNK_DEDUP_OVERLAP_Z_TOLERANCE_M", "0.05"))
DEDUP_OVERLAP_RATIO_MIN = float(os.environ.get("CART2TRUNK_DEDUP_OVERLAP_RATIO_MIN", "0.3"))

# 테이블 표면 전체가 우연히 "유효한 평면 후보"로 검출되면(넓고 평평하니 사각형
# 채움비 검사를 쉽게 통과한다), select_support_candidate가 그걸 정상적인 박스
# 지지면으로 착각할 수 있다(SAME_SIZE_STACK_DETECTION_LOG.md 3-6절 실측) - top
# 자신의 면적보다 이 배수 이상 큰 후보는 지지면에서 제외한다. 실제 박스 카탈로그
# 크기 비율(Large/Small 면적비 ~2.8배)보다는 넉넉하게, 테이블 전체 면적비
# (~13~20배)보다는 훨씬 작게 잡는다.
MAX_SUPPORT_AREA_RATIO = float(os.environ.get("CART2TRUNK_MAX_SUPPORT_AREA_RATIO", "6.0"))


def _debug_log(message: str) -> None:
    print(message, flush=True)


def load_merged_cloud(path: Path) -> np.ndarray:
    path = Path(path)
    if path.suffix == ".npy":
        points = np.load(path)
    elif path.suffix == ".ply":
        pcd = o3d.io.read_point_cloud(str(path))
        points = np.asarray(pcd.points)
    else:
        raise ValueError(f"지원하지 않는 입력 형식: {path.suffix} (.npy 또는 .ply만 가능)")

    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"point cloud shape이 (N,3)이 아닙니다: {points.shape}")
    return points


def _detect_boxes_once(scene_pcd, debug: bool = False) -> list[dict]:
    """box_top_extractor.py의 process_scene_cloud()와 같은 흐름을, 이미 전처리된
    point cloud 하나에 대해 검출 1회(RANSAC 시드 1개) 실행한다."""
    candidates = bg.detect_box_top_candidates_fixed_up(
        scene_pcd,
        up_vector=UP_VECTOR,
        debug=debug,
        debug_log=_debug_log,
    )

    # 카메라와의 거리(단일 시점 개념) 대신 높이(Z, 내림차순) - 맨 위 박스부터 처리.
    ordered_candidates = sorted(
        candidates,
        key=lambda candidate: (-float(candidate.center[2]), candidate.candidate_id),
    )

    boxes = []
    for top_candidate in ordered_candidates:
        support = bg.select_support_candidate(
            top_candidate,
            candidates,
            DOWN_VECTOR,
            support_min_area_ratio=bg.SUPPORT_MIN_AREA_RATIO,
            max_support_area_ratio=MAX_SUPPORT_AREA_RATIO,
            debug=debug,
            debug_log=_debug_log,
        )
        support_type = "box_top"

        if support is None:
            support = bg.detect_floor_boundary(
                top_candidate,
                scene_pcd,
                DOWN_VECTOR,
                debug=debug,
                debug_log=_debug_log,
            )
            support_type = "floor"

        if support is None:
            continue

        corners = bg.compute_box_corners(top_candidate, support, DOWN_VECTOR)
        if corners is None or np.asarray(corners).shape != (8, 3):
            continue
        corners = np.asarray(corners, dtype=np.float32)

        completed_points = bg.generate_completed_box_surface(corners)
        if len(completed_points) == 0:
            continue

        boxes.append(
            {
                "box_id": len(boxes),
                "top": top_candidate,
                "support": support,
                "support_type": support_type,
                "corners": corners,
                "completed_points": completed_points.astype(np.float32),
            }
        )

    return boxes


def _rect_center_xy(box: dict) -> tuple[float, float]:
    """사각형 4꼭짓점의 평균 - box["top"].center(원시 클러스터 점들의 중심)는
    fill_ratio가 낮은(치우친) 조각일수록 실제 사각형 중심과 크게 어긋날 수 있어서
    그룹핑/중복제거에는 항상 이쪽(피팅된 사각형 기준)을 쓴다."""
    corners = box["top"].corners_3d
    return float(corners[:, 0].mean()), float(corners[:, 1].mean())


def _group_by_location(
    all_trial_boxes: list[list[dict]],
    group_radius_m: float,
    z_tolerance_m: float = DEDUP_OVERLAP_Z_TOLERANCE_M,
) -> list[list[dict]]:
    """여러 시도에서 나온 박스들을 (윗면 사각형 중심의 xy 거리 + z 거리 기준) 같은
    물리적 위치끼리 묶는다.

    XY만 보면 안 되는 이유(실측으로 확인한 버그): Small을 Large 바로 위(같은 XY,
    다른 Z)에 스택시킨 경우, XY만으로 그룹을 묶으면 Small의 모든 시도 인스턴스가
    Large 그룹에 합쳐져 버린다 - 그 그룹 안에서 fill_ratio가 항상 더 높은 Large가
    이겨서(각 그룹에서 fill_ratio 최댓값 하나만 채택) Small은 선택될 기회 자체가
    없어진다(적층이 아닌 배치에서는 XY가 겹칠 일이 없어서 이 버그가 안 드러났다).
    35.crate_scan_setup.py의 table_real_boxes dedup에 있던 것과 같은 종류의
    실수라 같은 방식(Z 조건 AND 추가)으로 고친다."""
    groups: list[dict] = []
    for boxes in all_trial_boxes:
        for box in boxes:
            cx, cy = _rect_center_xy(box)
            cz = float(box["top"].center[2])
            placed = False
            for group in groups:
                if (
                    ((cx - group["cx"]) ** 2 + (cy - group["cy"]) ** 2) ** 0.5 < group_radius_m
                    and abs(cz - group["cz"]) < z_tolerance_m
                ):
                    group["items"].append(box)
                    placed = True
                    break
            if not placed:
                groups.append({"cx": cx, "cy": cy, "cz": cz, "items": [box]})
    return [g["items"] for g in groups]


def _footprint_aabb(box: dict) -> tuple[float, float, float, float]:
    corners = box["top"].corners_3d
    return float(corners[:, 0].min()), float(corners[:, 0].max()), float(corners[:, 1].min()), float(corners[:, 1].max())


def _aabb_overlap_ratio(a: tuple, b: tuple) -> float:
    """두 xy AABB가 겹치는 면적을, 더 작은 쪽 면적 대비 비율로 반환한다."""
    ax0, ax1, ay0, ay1 = a
    bx0, bx1, by0, by1 = b
    ix0, ix1 = max(ax0, bx0), min(ax1, bx1)
    iy0, iy1 = max(ay0, by0), min(ay1, by1)
    if ix1 <= ix0 or iy1 <= iy0:
        return 0.0
    inter_area = (ix1 - ix0) * (iy1 - iy0)
    area_a = (ax1 - ax0) * (ay1 - ay0)
    area_b = (bx1 - bx0) * (by1 - by0)
    return inter_area / max(1e-9, min(area_a, area_b))


def _dedup_overlapping_footprints(
    boxes: list[dict],
    z_tolerance_m: float = DEDUP_OVERLAP_Z_TOLERANCE_M,
    overlap_ratio_min: float = DEDUP_OVERLAP_RATIO_MIN,
) -> list[dict]:
    """_group_by_location()의 그룹 반경(group_radius_m)보다 두 후보의 사각형
    중심이 더 멀리 떨어져 있어도, 사각형 자체는 겹칠 수 있다 - 실측 확인: RANSAC이
    같은 테이블/박스 표면을 쪼개서 만든 작은 조각의 사각형이 실제 박스(Large)의
    사각형 안쪽에 들어와 있었는데, 두 중심점 거리(0.088m)가 group_radius_m(0.06m)
    보다 커서 별개의 박스로 살아남은 사례가 있었다. 같은 높이(z_tolerance_m 이내)에
    있으면서 사각형이 상당히(overlap_ratio_min 이상) 겹치는 쌍은 fill_ratio가 낮은
    쪽을 버린다 - 높이가 다르면(진짜 적층) 절대 건드리지 않는다."""
    kept: list[dict] = []
    for box in sorted(boxes, key=lambda b: -float(b["top"].fill_ratio)):
        box_z = float(box["top"].center[2])
        box_aabb = _footprint_aabb(box)
        overlaps_kept = any(
            abs(box_z - float(k["top"].center[2])) < z_tolerance_m
            and _aabb_overlap_ratio(box_aabb, _footprint_aabb(k)) >= overlap_ratio_min
            for k in kept
        )
        if overlaps_kept:
            print(
                f"[multiview_scan] 후보 center_xy={np.round(_rect_center_xy(box), 3).tolist()} "
                f"z={box_z:.3f} fill_ratio={box['top'].fill_ratio:.3f}: 같은 높이의 다른(더 신뢰도 높은) "
                f"후보와 사각형이 겹쳐서 제외", flush=True,
            )
            continue
        kept.append(box)
    return kept


MIN_TRUSTED_FILL_RATIO_FOR_HIDDEN_SEARCH = 0.85


def _split_hidden_same_size_stacks(
    boxes: list[dict],
    scene_pcd,
    debug: bool = False,
) -> list[dict]:
    """"위 박스가 항상 아래 박스보다 작다"는 전제가 깨지는 경우(위아래 박스 크기가
    같음) 대응. 이 경우 아래 박스의 노출면이 없거나 너무 얇아서(4cm 미만)
    detect_box_top_candidates_fixed_up()이 절대 독립 후보로 못 잡는다.

    [설계가 바뀐 이유 - SAME_SIZE_STACK_DETECTION_LOG.md 3-8절 참고]
    처음엔 find_hidden_stacked_box()의 변별 돌출량 히스토그램 하나로 "숨겨진 박스가
    있는지 + 깊이가 얼마인지"를 동시에 찾으려 했다. 실측 확인 결과 이 방식은 노이즈에
    취약했다: 옆면(수직으로 이어지는 면)과 진짜 표면(수평, 뾰족한 밀도 피크)을
    구분하려고 "피크 다음 bin에서 밀도가 떨어지는지" 조건을 추가했더니, 이번엔 진짜
    적층 케이스에서도 그 조건을 계속 통과 못 해서(옆면과 섞인 노이즈 때문에) 못
    찾는 반대쪽 실패가 나타났다.

    대신 detect_floor_boundary()가 원래 갖고 있던 성질을 활용한다: 그 함수는 RANSAC
    평면 후보들 중 "top 바로 아래에서 가장 가까운" 것을 채택하도록 이미 설계돼
    있어서, 숨겨진 동일 크기 박스가 있으면 그 박스 자신의 노출된 윗면을 우연히
    "가장 가까운 지지면"으로 찾아낸다(실측: 15회 반복 중 15회 모두 0.136~0.151m
    범위, 참값 0.14m와 거의 일치). box_geometry.find_stacked_layers()는 이 지지면을
    "혹시 그 자신도 또 다른 지지면 위에 떠 있는 게 아닌지"(=사실은 바닥이 아니라
    또 다른 박스였는지) detect_floor_boundary()를 재귀적으로 다시 호출해서 확인하고,
    노이즈에 대응하기 위해 이 재귀 탐색을 여러 번 반복한 뒤 "몇 겹으로 내려가는가"를
    다수결로 정한다(자세한 실측 수치는 로그 참고).

    depths(오름차순 누적 깊이 리스트)의 길이가 1이면 적층이 아니다(찾은 지지면이
    곧 최종 바닥) - 다만 이 경우에도 다중 시도 median 깊이가 기존 corners 계산에
    쓰인 단일 시도값보다 안정적이므로 항상 재계산해서 교체한다. 길이가 N(>=2)이면
    N-1개의 숨겨진 박스가 있다는 뜻이다 - 각 숨겨진 박스의 XY 오프셋은
    find_hidden_stacked_box()를 forced_depth_m(이미 알고 있는 깊이)와 함께 호출해서
    (히스토그램 탐색 없이) 가장자리 돌출 증거만으로 추정한다.

    fill_ratio가 낮은(신뢰도 낮은) top 후보에 대해서까지 이 탐색을 벌이면, 애초에
    노이즈/유령일 가능성이 높은 후보 위에 또 다른 유령 후보를 만들어 오탐을 배가시킬
    수 있다(실측 확인: fill_ratio=0.716, 12번 중 6번만 나타난 후보에서 실제로
    발생) - 그래서 이미 충분히 신뢰할 만한(fill_ratio가 높은) top 후보에만 이
    탐색을 적용한다."""
    processed_points = np.asarray(scene_pcd.points)
    result = list(boxes)
    next_synthetic_id = -1

    for box in list(boxes):
        top_candidate = box["top"]
        if top_candidate.fill_ratio < MIN_TRUSTED_FILL_RATIO_FOR_HIDDEN_SEARCH:
            continue

        depths = bg.find_stacked_layers(
            top_candidate, scene_pcd, DOWN_VECTOR, debug=debug, debug_log=_debug_log,
        )
        if len(depths) == 0:
            continue  # 지지면 자체를 못 찾음 - 기존(단일 시도) 결과를 그대로 둔다

        # depths[-1](다중 시도 median, 기존 단일 시도 raw_depth보다 안정적)로 이
        # top 자신의 corners를 다시 계산한다 - 적층 여부와 무관하게 정확도가 개선된다.
        terminal_support = bg.flat_plane_support_at_depth(top_candidate, depths[-1], DOWN_VECTOR)
        recomputed_corners = bg.compute_box_corners(top_candidate, terminal_support, DOWN_VECTOR)
        recomputed_corners = np.asarray(recomputed_corners) if recomputed_corners is not None else None
        if recomputed_corners is not None and recomputed_corners.shape == (8, 3):
            recomputed_completed = bg.generate_completed_box_surface(recomputed_corners.astype(np.float32))
            if len(recomputed_completed) > 0:
                box["corners"] = recomputed_corners.astype(np.float32)
                box["completed_points"] = recomputed_completed.astype(np.float32)

        num_hidden = len(depths) - 1
        if num_hidden == 0:
            continue

        print(
            f"[multiview_scan] top={top_candidate.candidate_id}: 재귀 바닥 재탐색으로 숨겨진 "
            f"동일 크기 박스 {num_hidden}개 발견(누적 깊이={[round(d, 4) for d in depths]})",
            flush=True,
        )

        # 각 숨겨진 박스의 top 표면 = depths[i]에서 원래 top의 XY 오프셋을
        # find_hidden_stacked_box(forced_depth_m=depths[i])로 재추정한 것.
        # 마지막 숨겨진 박스의 지지면만 진짜 터미널(depths[-1]); 중간 박스들의
        # 지지면은 그 다음 깊이의 top(forced_depth_m=depths[i+1]).
        layer_tops = []
        for i in range(num_hidden):
            layer_top = bg.find_hidden_stacked_box(
                top_candidate,
                processed_points,
                DOWN_VECTOR,
                forced_depth_m=depths[i],
                debug=debug,
                debug_log=_debug_log,
            )
            if layer_top is None:
                # 가장자리 돌출 증거를 전혀 못 찾은 극단적 경우(현재 구현은
                # require_offset_evidence=False라 이 분기는 사실상 도달하지 않지만,
                # 방어적으로 top의 footprint를 오프셋 0으로 그대로 사용한다.
                layer_top = bg.flat_plane_support_at_depth(top_candidate, depths[i], DOWN_VECTOR)
            layer_top.candidate_id = next_synthetic_id
            next_synthetic_id -= 1
            layer_tops.append(layer_top)

        # 원래 top 자신의 support를 "먼 바닥"에서 "첫 번째로 찾은 숨겨진 박스"로 교체.
        corrected_top_corners = bg.compute_box_corners(top_candidate, layer_tops[0], DOWN_VECTOR)
        corrected_top_corners = np.asarray(corrected_top_corners) if corrected_top_corners is not None else None
        if corrected_top_corners is not None and corrected_top_corners.shape == (8, 3):
            corrected_completed = bg.generate_completed_box_surface(corrected_top_corners.astype(np.float32))
            if len(corrected_completed) > 0:
                old_height = float(np.mean(box["corners"][:4, 2] - box["corners"][4:, 2]))
                new_height = float(np.mean(corrected_top_corners[:4, 2] - corrected_top_corners[4:, 2]))
                print(
                    f"[multiview_scan] top={top_candidate.candidate_id}: 원래 박스 높이 "
                    f"{old_height:.3f}m -> {new_height:.3f}m로 보정(지지면을 숨겨진 박스로 교체)",
                    flush=True,
                )
                box["corners"] = corrected_top_corners.astype(np.float32)
                box["completed_points"] = corrected_completed.astype(np.float32)
                box["support"] = layer_tops[0]
                box["support_type"] = "box_top"

        # 숨겨진 박스들을 각각 독립 박스로 등록 - i번째 박스의 지지면은
        # (i+1)번째 층의 top(마지막이면 진짜 터미널 바닥).
        for i in range(num_hidden):
            this_top = layer_tops[i]
            if i + 1 < num_hidden:
                this_support = layer_tops[i + 1]
                this_support_type = "box_top"
            else:
                this_support = bg.flat_plane_support_at_depth(top_candidate, depths[-1], DOWN_VECTOR)
                this_support_type = "floor"

            hidden_corners = bg.compute_box_corners(this_top, this_support, DOWN_VECTOR)
            hidden_corners = np.asarray(hidden_corners) if hidden_corners is not None else None
            if hidden_corners is None or hidden_corners.shape != (8, 3):
                print(
                    f"[multiview_scan] 숨겨진 박스 후보(top={top_candidate.candidate_id} 아래, "
                    f"층 {i})의 8꼭짓점 복원 실패 -> 무시", flush=True,
                )
                continue
            hidden_completed = bg.generate_completed_box_surface(hidden_corners.astype(np.float32))
            if len(hidden_completed) == 0:
                continue

            result.append(
                {
                    "box_id": -1,  # 호출부에서 전체 재부여
                    "top": this_top,
                    "support": this_support,
                    "support_type": this_support_type,
                    "corners": hidden_corners.astype(np.float32),
                    "completed_points": hidden_completed.astype(np.float32),
                }
            )

    return result


def detect_boxes_in_base_frame(
    points_base: np.ndarray,
    debug: bool = DEBUG_SUPPORT,
    trials: int = DETECTION_TRIALS,
    group_radius_m: float = DETECTION_GROUP_RADIUS_M,
    min_appearance_fraction: float = DETECTION_MIN_APPEARANCE_FRACTION,
) -> list[dict]:
    """base_link 좌표계로 이미 합쳐진 point cloud 하나에서 박스를 검출한다.

    Open3D RANSAC이 시드 고정이 없어 같은 입력도 검출마다 평면이 다르게 쪼개지는
    문제(특히 작은 박스에서 fill_ratio는 통과하지만 크기가 틀어진 조각이 나올 수
    있음, perception/STACKED_BOX_DETECTION_DEBUG_GUIDE.md 6절과 동일 계열) 때문에,
    전처리는 한 번만 하고 검출 자체를 `trials`번 반복한다. 같은 물리적 위치(xy 근접)에서
    나온 후보들을 묶어서 그중 fill_ratio가 가장 높은(가장 완전한 사각형에 가까운) 것을
    채택하고, 전체 시도 중 너무 적게(< min_appearance_fraction) 나타난 후보는
    우연히 걸린 조각으로 보고 버린다."""
    scene_pcd = bg.preprocess_cloud(points_base)
    processed_points = np.asarray(scene_pcd.points)
    print(
        f"[multiview_scan] 입력 {len(points_base)}점 -> 전처리 후 {len(processed_points)}점",
        flush=True,
    )

    if len(processed_points) < bg.MIN_PLANE_POINTS:
        print("[multiview_scan] 전처리 후 point가 너무 적습니다 - 검출 생략", flush=True)
        return []

    all_trial_boxes = [_detect_boxes_once(scene_pcd, debug=debug) for _ in range(trials)]
    trial_counts = [len(b) for b in all_trial_boxes]
    print(
        f"[multiview_scan] 검출 {trials}회 반복, 시도별 박스 개수={trial_counts}",
        flush=True,
    )

    groups = _group_by_location(all_trial_boxes, group_radius_m)
    min_appearances = max(1, int(np.ceil(trials * min_appearance_fraction)))

    # 참고: 같은 시도(trial) 안에서도 하나의 물리적 평면이 근접한 조각 여러 개로
    # 쪼개져 같은 그룹에 들어갈 수 있어서, len(items)가 trials를 넘을 수도 있다 -
    # "이 위치에서 총 몇 개의 후보 인스턴스가 나왔나"이지 "몇 번의 시도에서
    # 나타났나"는 아니다. 그래도 min_appearances 기준(전체 시도 수 대비 비율)으로
    # 보는 것 자체는 유효하다 - 우연히 한두 번만 걸린 조각과, 어떤 형태로든
    # 지속적으로 나타나는 진짜 물체를 구분하는 목적이기 때문이다.
    selected = []
    for items in groups:
        if len(items) < min_appearances:
            rep = items[0]["top"]
            print(
                f"[multiview_scan] 후보 center={np.round(rep.center, 3).tolist()}: "
                f"{len(items)}개 인스턴스만 관측(< {min_appearances}, {trials}회 시도 기준) -> 노이즈로 보고 제외",
                flush=True,
            )
            continue
        best = max(items, key=lambda b: (b["top"].fill_ratio, len(b["top"].points)))
        narrower_side = min(float(best["top"].width), float(best["top"].height))
        wider_side = max(float(best["top"].width), float(best["top"].height))
        if narrower_side < MIN_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M:
            print(
                f"[multiview_scan] 후보 center={np.round(best['top'].center, 3).tolist()}: "
                f"짧은 변이 {narrower_side:.3f}m(<{MIN_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M}m)로 너무 가늘어서 "
                "실제 박스로 보기 어려움 -> 제외", flush=True,
            )
            continue
        if wider_side > MAX_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M:
            print(
                f"[multiview_scan] 후보 center={np.round(best['top'].center, 3).tolist()}: "
                f"긴 변이 {wider_side:.3f}m(>{MAX_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M}m)로 너무 커서 "
                "실제 박스로 보기 어려움(카트/테이블 등 구조물로 추정) -> 제외", flush=True,
            )
            continue
        if float(best["top"].fill_ratio) < MIN_FINAL_FILL_RATIO:
            print(
                f"[multiview_scan] 후보 center={np.round(best['top'].center, 3).tolist()}: "
                f"fill_ratio={best['top'].fill_ratio:.3f}(<{MIN_FINAL_FILL_RATIO})로 사각형이 "
                "부실하게 채워져 실제 박스로 보기 어려움 -> 제외", flush=True,
            )
            continue
        fill_ratios = [round(float(b["top"].fill_ratio), 3) for b in items]
        print(
            f"[multiview_scan] 후보 center={np.round(best['top'].center, 3).tolist()}: "
            f"{len(items)}개 인스턴스 관측, fill_ratio 분포={fill_ratios} -> "
            f"fill_ratio={best['top'].fill_ratio:.3f}인 것 채택",
            flush=True,
        )
        selected.append(best)

    before_overlap_dedup = len(selected)
    selected = _dedup_overlapping_footprints(selected)
    if len(selected) < before_overlap_dedup:
        print(
            f"[multiview_scan] 사각형 겹침 정리: {before_overlap_dedup} -> {len(selected)}개",
            flush=True,
        )

    before_split = len(selected)
    selected = _split_hidden_same_size_stacks(selected, scene_pcd, debug=debug)
    if len(selected) > before_split:
        print(
            f"[multiview_scan] 동일 크기 적층 분리: {before_split} -> {len(selected)}개",
            flush=True,
        )

    # 높이(Z, 내림차순) 순으로 box_id 재부여 - 맨 위 박스부터.
    selected.sort(key=lambda b: -float(b["top"].center[2]))
    for i, box in enumerate(selected):
        box["box_id"] = i

    print(f"[multiview_scan] 복원된 박스 {len(selected)}개", flush=True)
    return selected


def save_boxes(boxes: list[dict], save_directory: Path = SAVE_DIRECTORY) -> Optional[Path]:
    """box_top_extractor.py의 save_current_cloud()와 동일한 JSON/PLY 계약으로 저장한다
    (algorism/14_run_full_pipeline.py의 load_boxes_from_vision_json()이 그대로 소비함) -
    다른 점은, 여기 point들은 이미 base_link 좌표계라 camera->base_link 변환이 없다는 것뿐."""
    if not boxes:
        print("[multiview_scan] 저장할 복원 박스 정보가 없습니다.", flush=True)
        return None

    save_directory = Path(save_directory)
    save_directory.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

    all_completed_points = []
    boxes_payload = []
    point_offset = 0

    for box in boxes:
        corners = np.asarray(box["corners"], dtype=np.float32)
        completed_points = np.asarray(box["completed_points"], dtype=np.float32)

        if corners.shape != (8, 3) or len(completed_points) == 0:
            continue

        point_start_index = point_offset
        point_count = int(len(completed_points))
        point_end_index = point_start_index + point_count - 1

        all_completed_points.append(completed_points)

        top_candidate = box["top"]
        support_candidate = box["support"]

        boxes_payload.append(
            {
                "box_id": int(box["box_id"]),
                "support_type": box["support_type"],
                "top_candidate_id": int(top_candidate.candidate_id),
                "support_candidate_id": int(support_candidate.candidate_id),
                "corner_order": [
                    "top_0", "top_1", "top_2", "top_3",
                    "bottom_0", "bottom_1", "bottom_2", "bottom_3",
                ],
                "corners_m": corners.tolist(),
                "bottom_cut_margin_m": bg.BOTTOM_CUT_MARGIN_M,
                "completed_point_count": point_count,
                "ply_point_start_index": point_start_index,
                "ply_point_end_index": point_end_index,
            }
        )
        point_offset += point_count

    if not all_completed_points:
        print("[multiview_scan] 유효한 박스 point cloud가 없습니다.", flush=True)
        return None

    merged_points = np.vstack(all_completed_points).astype(np.float64)

    ply_path = save_directory / f"all_boxes_completed_{timestamp}.ply"
    merged_pcd = o3d.geometry.PointCloud()
    merged_pcd.points = o3d.utility.Vector3dVector(merged_points)
    ply_success = o3d.io.write_point_cloud(str(ply_path), merged_pcd, write_ascii=True)

    json_path = save_directory / f"all_boxes_corners_{timestamp}.json"
    payload = {
        "coordinate_frame": OUTPUT_FRAME,
        "unit": "meter",
        "box_count": len(boxes_payload),
        "completed_ply_file": ply_path.name,
        "total_completed_point_count": int(len(merged_points)),
        "boxes": boxes_payload,
    }
    json_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    if ply_success:
        print(f"[multiview_scan] Saved {len(boxes_payload)} boxes: {json_path.name}, {ply_path.name}", flush=True)
    else:
        print("[multiview_scan] 통합 PLY 저장에 실패했습니다.", flush=True)

    return json_path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="base_link 좌표계로 합쳐진 point cloud (.npy 또는 .ply)")
    parser.add_argument("--marker", required=True, help="처리 완료를 알리는 마커 파일 경로")
    args = parser.parse_args()

    marker_path = Path(args.marker)
    marker_path.parent.mkdir(parents=True, exist_ok=True)
    if marker_path.exists():
        marker_path.unlink()

    points_base = load_merged_cloud(Path(args.input))
    boxes = detect_boxes_in_base_frame(points_base)
    save_boxes(boxes)

    marker_path.write_text("done")
    print(f"[multiview_scan] 마커 생성: {marker_path}", flush=True)


if __name__ == "__main__":
    main()
