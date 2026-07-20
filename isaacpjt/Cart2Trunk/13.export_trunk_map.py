"""
13.export_trunk_map.py
포인트클라우드(results/run_*/pointcloud/trunk_pointcloud.npy) -> M0609 base_link 좌표계
"선/면 지도"(trunk_map.json) 변환.

Isaac Sim 불필요 (일반 python3 + numpy + open3d로 실행).

    python3 13.export_trunk_map.py                         # 최신 run 자동 선택
    python3 13.export_trunk_map.py results/run_YYYYMMDD_HHMMSS
    python3 13.export_trunk_map.py --visualize              # 처리 후 Open3D로 확인

[좌표계 원칙]
    - 바닥/좌우벽/안쪽벽 4면 + AABB x/y/floor_z: 이번 스캔 포인트클라우드에서 직접 측정한 값
      (edges/faces style="solid").
    - 천장(문이 닫히는 높이 한계선): 트렁크가 열린 채로 스캔했으므로 포인트클라우드에 실제로
      존재하지 않는 면. meta.json의 trunk_bounds.wall_top_z(설계/설정값)를 그대로 사용
      (edges/faces style="dashed") - "측정값 vs 설계 제약값"이 실선/점선 구분과 그대로 대응된다.
    - 출력은 전부 M0609 base_link 좌표계 (meta.json의 base_pos/base_quat로 world -> base 변환).

[알려진 제약]
    - meta.json에 base_pos/base_quat가 없는 과거 run(예: run_20260720_160153)은
      12.trunk_scan_hidden_gripper.py의 고정 상수(ROBOT_XY, MOUNT_Z, FACE_ROT_Z=0)로 근사
      재구성한다 (물리 안정화 오차는 HANDOFF 기록상 각도 0.05~0.08도 수준이라 무시 가능).
      12.trunk_scan_hidden_gripper.py는 이제 base_pos/base_quat를 저장하므로, 이후 run은
      이 근사 없이 정확한 값을 그대로 쓴다.
    - 휠하우스(좌/우 돌출부) 검출은 grid+connected-component 기반 휴리스틱이다. 열린 트렁크
      문(리드)이 공중에 떠서 찍힌 점들은 WHEEL_HOUSE_MAX_HEIGHT_ABOVE_FLOOR보다 높은 곳에
      있다고 보고 애초에 후보에서 제외한다("문 무시"). 자동 검출 결과가 이상하면
      --visualize/trunk_map_preview.png로 확인 후 WHEEL_HOUSE_* 상수를 손으로 조정할 것.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

for _font in ("NanumSquareRound", "NanumGothic", "Noto Sans CJK KR", "Noto Sans CJK JP", "Noto Sans CJK SC"):
    if any(_font in f.name for f in matplotlib.font_manager.fontManager.ttflist):
        matplotlib.rcParams["font.family"] = _font
        break
matplotlib.rcParams["axes.unicode_minus"] = False

_THIS_DIR = Path(__file__).resolve().parent
RESULTS_DIR = _THIS_DIR / "results"

# 과거 run(base_pos/base_quat 미저장)에 대한 근사 재구성 - 12.trunk_scan_hidden_gripper.py와 동일 상수.
FALLBACK_TRUNK_X_MIN = 3.11
FALLBACK_ROBOT_XY = (FALLBACK_TRUNK_X_MIN - 0.85, -0.15)
FALLBACK_MOUNT_Z = 0.42
FALLBACK_FACE_ROT_Z = 0.0

CROP_MARGIN_XY = 0.30   # trunk_bounds x/y 대비 크롭 여유
CROP_MARGIN_Z = 0.15    # floor_z/wall_top_z 대비 크롭 여유
OUTLIER_NB_NEIGHBORS = 20
OUTLIER_STD_RATIO = 2.0
VOXEL_SIZE = 0.01

BOUND_PCTL_LOW = 1.0    # x/y/floor 하한 percentile
BOUND_PCTL_HIGH = 99.0  # x/y 상한 percentile
CEILING_OVERHEAD_PCTL = 5.0  # RANSAC으로 찾은 평면의 점들 중 이 percentile 높이를 실측 천장으로 사용
CEILING_MIN_OVERHEAD_POINTS = 500      # 이보다 점이 적으면 평면 검출을 시도하지 않음
CEILING_PLANE_DIST_THRESHOLD = 0.02    # RANSAC 평면 inlier로 볼 거리(m)
CEILING_PLANE_RANSAC_ITERS = 2000
CEILING_MIN_PLANE_INLIERS = 500        # 이보다 inlier가 적으면 "우연히 찾은 평면"으로 보고 버림
CEILING_PLANE_MIN_HORIZONTALITY = 0.8  # |normal_z|/|normal| - 이보다 작으면 수평면이 아니라고 보고 버림(옆벽 등)

WHEEL_HOUSE_CELL = 0.03        # grid cell 크기 (m)
WHEEL_HOUSE_EDGE_MARGIN = 0.05  # 벽 근처 cell 제외 (벽을 돌출부로 오검출하지 않도록)
WHEEL_HOUSE_BUMP_THRESHOLD = 0.03  # local floor 기준값보다 이 높이(m) 이상 솟은 cell만 돌출부 후보
WHEEL_HOUSE_MIN_CELLS = 6      # 돌출부(덩어리)로 인정할 최소 cell 개수 (노이즈 방지)
WHEEL_HOUSE_MAX_BLOB_AREA_FRACTION = 0.25  # 덩어리 하나가 interior의 이 비율을 넘으면 휠하우스로 안 봄
WHEEL_HOUSE_MAX_Y_SPAN_FRACTION = 0.5  # 덩어리가 y폭 전체의 이 비율을 넘게 가로지르면 제외(문턱 등으로 추정)
# 열린 트렁크 문(리드)이 공중에 떠서 찍히는 높이(대략 바닥+0.5~0.6m, 실측 결과 기준)보다
# 훨씬 낮게 잡아서, 문 점군은 애초에 휠하우스 후보 계산에서 아예 제외한다 ("문 무시").
WHEEL_HOUSE_MAX_HEIGHT_ABOVE_FLOOR = 0.40


# --------------------------------------------------------------------------- #
# 좌표 변환
# --------------------------------------------------------------------------- #

def quat_wxyz_to_matrix(q) -> np.ndarray:
    """Isaac Sim 관례(w, x, y, z) 쿼터니언 -> 3x3 회전행렬."""
    w, x, y, z = q
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array([
        [1 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1 - (xx + yy)],
    ])


def world_to_base(points_world: np.ndarray, base_pos: np.ndarray, R_base: np.ndarray) -> np.ndarray:
    """p_base = R_base^T @ (p_world - base_pos)."""
    return (points_world - base_pos[None, :]) @ R_base


# --------------------------------------------------------------------------- #
# run 폴더 / meta 로드
# --------------------------------------------------------------------------- #

def resolve_run_dir(arg: str | None) -> Path:
    if arg:
        p = Path(arg)
        return p if p.is_absolute() else (_THIS_DIR / p)
    runs = sorted(RESULTS_DIR.glob("run_*"))
    if not runs:
        raise SystemExit(f"[에러] {RESULTS_DIR}에 run_* 폴더가 없습니다.")
    print(f"[자동 선택] 최신 run: {runs[-1].name}")
    return runs[-1]


def load_base_pose(meta: dict) -> tuple[np.ndarray, np.ndarray]:
    if "base_pos" in meta and "base_quat" in meta:
        base_pos = np.asarray(meta["base_pos"], dtype=float)
        base_quat = np.asarray(meta["base_quat"], dtype=float)
        print(f"[base pose] meta.json에서 로드 pos={base_pos} quat={base_quat}")
        return base_pos, base_quat

    print(
        "[경고] meta.json에 base_pos/base_quat가 없습니다 (구버전 run). "
        "12.trunk_scan_hidden_gripper.py 고정 상수로 근사 재구성합니다 "
        "(FACE_ROT_Z=0 가정 -> 회전 없음). 정확한 값이 필요하면 스캔을 다시 실행하세요."
    )
    base_pos = np.array([FALLBACK_ROBOT_XY[0], FALLBACK_ROBOT_XY[1], FALLBACK_MOUNT_Z])
    base_quat = np.array([1.0, 0.0, 0.0, 0.0])  # identity (w,x,y,z), FACE_ROT_Z=0 가정
    print(f"[base pose] 근사값 pos={base_pos} quat={base_quat}(identity)")
    return base_pos, base_quat


# --------------------------------------------------------------------------- #
# 포인트클라우드 정제
# --------------------------------------------------------------------------- #

def filter_points_world(pts_world: np.ndarray, trunk_bounds: dict) -> np.ndarray:
    x_min, x_max = trunk_bounds["x"]
    y_min, y_max = trunk_bounds["y"]
    floor_z = trunk_bounds["floor_z"]
    wall_top_z = trunk_bounds["wall_top_z"]

    mask = (
        (pts_world[:, 0] > x_min - CROP_MARGIN_XY) & (pts_world[:, 0] < x_max + CROP_MARGIN_XY) &
        (pts_world[:, 1] > y_min - CROP_MARGIN_XY) & (pts_world[:, 1] < y_max + CROP_MARGIN_XY) &
        (pts_world[:, 2] > floor_z - CROP_MARGIN_Z) & (pts_world[:, 2] < wall_top_z + CROP_MARGIN_Z)
    )
    cropped = pts_world[mask]
    print(f"[크롭] {len(pts_world)} -> {len(cropped)}점 (trunk_bounds ± margin)")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cropped)
    pcd_clean, _ = pcd.remove_statistical_outlier(
        nb_neighbors=OUTLIER_NB_NEIGHBORS, std_ratio=OUTLIER_STD_RATIO
    )
    cleaned = np.asarray(pcd_clean.points)
    print(f"[이상치 제거] {len(cropped)} -> {len(cleaned)}점")
    return cleaned


# --------------------------------------------------------------------------- #
# AABB (실측: 바닥 + 3벽) 산출
# --------------------------------------------------------------------------- #

def compute_measured_aabb(pts_base: np.ndarray) -> dict:
    x_lo, x_hi = np.percentile(pts_base[:, 0], [BOUND_PCTL_LOW, BOUND_PCTL_HIGH])
    y_lo, y_hi = np.percentile(pts_base[:, 1], [BOUND_PCTL_LOW, BOUND_PCTL_HIGH])
    z_lo = np.percentile(pts_base[:, 2], BOUND_PCTL_LOW)
    return {"x_min": float(x_lo), "x_max": float(x_hi),
            "y_min": float(y_lo), "y_max": float(y_hi),
            "floor_z": float(z_lo)}


def compute_ceiling_z_base(trunk_bounds: dict, base_pos: np.ndarray, R_base: np.ndarray,
                            pts_base: np.ndarray, floor_z: float) -> float:
    """
    천장(점선, "박스를 이 높이 넘게 쌓으면 안 됨") 높이를 설계 상수(wall_top_z)와 실측값 중
    더 낮은(더 보수적인) 쪽으로 정한다.

    설계 상수만 믿으면 안 되는 이유: wall_top_z는 스캔 스크립트에 박힌 값이라, 실제로 트렁크
    안쪽에 그보다 낮게 걸쳐있는 구조물(정체는 열린 문일 수도, 다른 구조물일 수도 있음)이 있어도
    반영이 안 된다. "그 높이에 뭔가 있었다"는 관측 자체는 무시하면 안 된다 - 문이 열려서
    어쩌다 거기 있었든, 다른 구조물이든, 그 높이 위로 박스를 쌓으면 실제로 부딪힐 위험이
    있다는 신호이므로 안전하게 더 낮은 쪽을 택한다.

    단순히 "바닥+0.4m보다 위 점들의 낮은 percentile"을 쓰면 안 되는 이유: 어두운 캐비티 안
    센서 노이즈가 후보 영역 전체에 옅게 깔려있어서, 그 percentile이 진짜 구조물이 아니라
    노이즈 하한선을 잡아버린다(실제로 x 구간별 최소값이 거의 항상 정확히 컷오프 근처에 고정되는
    것으로 확인됨 - 실제 표면이라면 x에 따라 부드럽게 변해야 하는데 그렇지 않았음). 대신
    RANSAC으로 "위쪽 점들 중 가장 크고 뚜렷한 평면"을 찾아서, 그 평면(수평에 가까운 경우만
    채택)의 높이를 실측값으로 쓴다 - 노이즈는 평면을 이루지 않으므로 자연히 걸러진다.
    """
    cx = (trunk_bounds["x"][0] + trunk_bounds["x"][1]) / 2.0
    cy = (trunk_bounds["y"][0] + trunk_bounds["y"][1]) / 2.0
    p_world = np.array([[cx, cy, trunk_bounds["wall_top_z"]]])
    config_ceiling_z = float(world_to_base(p_world, base_pos, R_base)[0][2])

    # 바닥에서 WHEEL_HOUSE_MAX_HEIGHT_ABOVE_FLOOR보다 높이 뜬 점 = "바닥/휠하우스"가 아니라
    # "위에서 뭔가가 관측된" 점으로 취급.
    overhead = pts_base[pts_base[:, 2] > floor_z + WHEEL_HOUSE_MAX_HEIGHT_ABOVE_FLOOR]
    measured_ceiling_z = config_ceiling_z

    if len(overhead) >= CEILING_MIN_OVERHEAD_POINTS:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(overhead)
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=CEILING_PLANE_DIST_THRESHOLD, ransac_n=3,
            num_iterations=CEILING_PLANE_RANSAC_ITERS,
        )
        if len(inliers) >= CEILING_MIN_PLANE_INLIERS:
            a, b, c, _ = plane_model
            horizontality = abs(c) / (np.linalg.norm([a, b, c]) + 1e-9)
            plane_pts = overhead[inliers]
            if horizontality >= CEILING_PLANE_MIN_HORIZONTALITY:
                measured_ceiling_z = float(np.percentile(plane_pts[:, 2], CEILING_OVERHEAD_PCTL))
                print(f"[천장(점선)] RANSAC 평면 검출: inliers={len(inliers)}/{len(overhead)} "
                      f"수평도={horizontality:.3f} z=[{plane_pts[:, 2].min():.3f},{plane_pts[:, 2].max():.3f}] "
                      f"-> 실측 후보={measured_ceiling_z:.3f}")
            else:
                print(f"[천장(점선)] 평면을 찾았지만 수평이 아님(수평도={horizontality:.3f} < "
                      f"{CEILING_PLANE_MIN_HORIZONTALITY}) - 벽면 등으로 보고 무시, 설정값 사용")
        else:
            print(f"[천장(점선)] 뚜렷한 평면을 못 찾음(inliers={len(inliers)}) - 설정값 사용")
    else:
        print(f"[천장(점선)] 상부 관측점 부족({len(overhead)}개) - 설정값 사용")

    ceiling_z = min(config_ceiling_z, measured_ceiling_z)
    source = "실측(RANSAC 평면)" if ceiling_z == measured_ceiling_z and measured_ceiling_z != config_ceiling_z else "설정값(wall_top_z)"
    print(f"[천장(점선)] 설정값={config_ceiling_z:.3f} 실측={measured_ceiling_z:.3f} "
          f"-> 최종 채택={ceiling_z:.3f} ({source}, 더 낮은/보수적인 쪽)")
    return ceiling_z


# --------------------------------------------------------------------------- #
# 휠하우스(좌/우 돌출부) 검출 - grid 휴리스틱 + connected-component
# --------------------------------------------------------------------------- #

def detect_wheel_houses(pts_base: np.ndarray, aabb: dict) -> list[dict]:
    """열린 트렁크 문(리드)이 공중에 떠서 찍힌 점들은 WHEEL_HOUSE_MAX_HEIGHT_ABOVE_FLOOR보다
    훨씬 위(바닥+0.5~0.6m대)에 있으므로, 애초에 후보 점 집합에서 제외하고 시작한다
    ("문 무시"). 남은 낮은 점들에서 좌/우 휠하우스를 각각 별도 덩어리로 검출한다."""
    from scipy import ndimage

    x_min, x_max = aabb["x_min"], aabb["x_max"]
    y_min, y_max = aabb["y_min"], aabb["y_max"]
    floor_z = aabb["floor_z"]

    band_pts = pts_base[pts_base[:, 2] < floor_z + WHEEL_HOUSE_MAX_HEIGHT_ABOVE_FLOOR]
    print(f"[휠하우스] 문 높이 제외 후 후보점: {len(pts_base)} -> {len(band_pts)} "
          f"(z < floor_z+{WHEEL_HOUSE_MAX_HEIGHT_ABOVE_FLOOR}m)")

    nx = max(1, int((x_max - x_min) / WHEEL_HOUSE_CELL))
    ny = max(1, int((y_max - y_min) / WHEEL_HOUSE_CELL))
    x_edges = np.linspace(x_min, x_max, nx + 1)
    y_edges = np.linspace(y_min, y_max, ny + 1)

    ix = np.clip(np.digitize(band_pts[:, 0], x_edges) - 1, 0, nx - 1)
    iy = np.clip(np.digitize(band_pts[:, 1], y_edges) - 1, 0, ny - 1)

    local_min_z = np.full((nx, ny), np.nan)
    for cx in range(nx):
        for cy in range(ny):
            sel = band_pts[(ix == cx) & (iy == cy), 2]
            if len(sel) >= 3:
                local_min_z[cx, cy] = np.percentile(sel, 5)

    edge_margin_cells_x = max(1, int(WHEEL_HOUSE_EDGE_MARGIN / WHEEL_HOUSE_CELL))
    edge_margin_cells_y = max(1, int(WHEEL_HOUSE_EDGE_MARGIN / WHEEL_HOUSE_CELL))
    interior = np.zeros_like(local_min_z, dtype=bool)
    interior[edge_margin_cells_x:nx - edge_margin_cells_x, edge_margin_cells_y:ny - edge_margin_cells_y] = True
    interior &= ~np.isnan(local_min_z)
    n_interior = int(np.count_nonzero(interior))
    if n_interior == 0:
        print("[휠하우스] interior에 유효 데이터 없음 -> 검출 안 함")
        return []

    # 전역 percentile은 노이즈 하나에도 흔들리므로, cell별 local min의 중앙값을 "이 트렁크에서
    # 가장 흔한 바닥 높이" 기준으로 삼는다.
    floor_ref = float(np.median(local_min_z[interior]))
    bump_mask = interior & (local_min_z > (floor_ref + WHEEL_HOUSE_BUMP_THRESHOLD))
    print(f"[휠하우스] floor_ref(local median)={floor_ref:.3f} (전역 1pctl={floor_z:.3f}) "
          f"bump 후보 {int(np.count_nonzero(bump_mask))}/{n_interior} cell")

    labeled, n_blobs = ndimage.label(bump_mask, structure=np.ones((3, 3)))
    y_span_total = y_max - y_min
    blobs = []
    for blob_id in range(1, n_blobs + 1):
        blob = labeled == blob_id
        n_cells = int(np.count_nonzero(blob))
        area_fraction = n_cells / n_interior
        if n_cells < WHEEL_HOUSE_MIN_CELLS:
            continue
        if area_fraction > WHEEL_HOUSE_MAX_BLOB_AREA_FRACTION:
            print(f"[휠하우스] 덩어리(cell {n_cells}개)가 interior의 {area_fraction:.0%}를 차지 "
                  f"(> {WHEEL_HOUSE_MAX_BLOB_AREA_FRACTION:.0%}) - 휠하우스로 보기엔 너무 커서 제외")
            continue

        cx_idx, cy_idx = np.nonzero(blob)
        bx_min, bx_max = x_edges[cx_idx.min()], x_edges[cx_idx.max() + 1]
        by_min, by_max = y_edges[cy_idx.min()], y_edges[cy_idx.max() + 1]
        # 휠하우스는 한쪽 벽에 붙은 좁은 돌출부여야 한다. y방향으로 폭 전체의 절반 이상을
        # 가로지르는 덩어리는 휠하우스가 아니라 입구 문턱/선반 같은 다른 구조물일 가능성이 커서 제외.
        if (by_max - by_min) > WHEEL_HOUSE_MAX_Y_SPAN_FRACTION * y_span_total:
            print(f"[휠하우스] 덩어리(cell {n_cells}개, y=[{by_min:.3f},{by_max:.3f}])가 폭 전체의 "
                  f"{(by_max - by_min) / y_span_total:.0%}를 가로질러 제외 (문턱/선반 등으로 추정)")
            continue

        top_z = float(np.nanpercentile(local_min_z[blob], 90))
        blobs.append({"x_min": float(bx_min), "x_max": float(bx_max),
                       "y_min": float(by_min), "y_max": float(by_max),
                       "floor_z": floor_z, "top_z": top_z, "n_cells": n_cells})

    if not blobs:
        print("[휠하우스] 조건을 만족하는 덩어리 없음 -> 검출 안 함")
        return []

    # 같은 벽(y_min쪽/y_max쪽)에 붙은 덩어리가 여러 개면, 서로 떨어져 있는 별개 구조물(노이즈
    # 포함)일 가능성이 높다 - 합치지 않고 "가장 큰 덩어리 하나"만 그 벽의 휠하우스로 채택한다.
    y_center = (y_min + y_max) / 2
    sides = {"wheel_house_left": [], "wheel_house_right": []}
    for b in blobs:
        side = "wheel_house_left" if (b["y_min"] + b["y_max"]) / 2 < y_center else "wheel_house_right"
        sides[side].append(b)

    results = []
    for name, group in sides.items():
        if not group:
            continue
        if len(group) > 1:
            dropped = sorted(group, key=lambda b: -b["n_cells"])[1:]
            for d in dropped:
                print(f"[휠하우스] {name} 쪽 작은 덩어리(cell {d['n_cells']}개, "
                      f"x=[{d['x_min']:.3f},{d['x_max']:.3f}]) 제외 - 노이즈로 추정")
        best = max(group, key=lambda b: b["n_cells"])
        best = dict(best, name=name)
        results.append(best)
        print(f"[휠하우스] {name}: x=[{best['x_min']:.3f},{best['x_max']:.3f}] "
              f"y=[{best['y_min']:.3f},{best['y_max']:.3f}] top_z={best['top_z']:.3f} "
              f"(cell {best['n_cells']}개)")

    return results


# --------------------------------------------------------------------------- #
# JSON 조립
# --------------------------------------------------------------------------- #

def box_vertices(x_min, x_max, y_min, y_max, z_min, z_max):
    return [
        [x_min, y_min, z_min], [x_max, y_min, z_min],
        [x_max, y_max, z_min], [x_min, y_max, z_min],
        [x_min, y_min, z_max], [x_max, y_min, z_max],
        [x_max, y_max, z_max], [x_min, y_max, z_max],
    ]


def build_trunk_map(aabb: dict, ceiling_z: float, wheel_houses: list[dict],
                     run_id: str, n_raw: int, n_filtered: int) -> dict:
    v = box_vertices(aabb["x_min"], aabb["x_max"], aabb["y_min"], aabb["y_max"],
                      aabb["floor_z"], ceiling_z)

    edges = [
        {"v": [0, 1], "style": "solid"}, {"v": [1, 2], "style": "solid"},
        {"v": [2, 3], "style": "solid"}, {"v": [3, 0], "style": "solid"},
        {"v": [4, 5], "style": "dashed"}, {"v": [5, 6], "style": "dashed"},
        {"v": [6, 7], "style": "dashed"}, {"v": [7, 4], "style": "dashed"},
        {"v": [0, 4], "style": "solid"}, {"v": [1, 5], "style": "solid"},
        {"v": [2, 6], "style": "solid"}, {"v": [3, 7], "style": "solid"},
    ]
    faces = [
        {"name": "floor", "v": [0, 1, 2, 3], "style": "solid"},
        {"name": "wall_y_min", "v": [0, 1, 5, 4], "style": "solid"},
        {"name": "wall_y_max", "v": [3, 2, 6, 7], "style": "solid"},
        {"name": "wall_x_max", "v": [1, 2, 6, 5], "style": "solid"},
        {"name": "ceiling_limit", "v": [4, 5, 6, 7], "style": "dashed"},
    ]

    result = {
        "schema_version": "1.0",
        "run_id": run_id,
        "frame": "m0609_base_link",
        "note": (
            "floor/wall_y_min/wall_y_max/wall_x_max + AABB x/y/floor_z: 이번 스캔에서 실측 (solid). "
            "ceiling_limit: 박스를 이 높이 넘게 쌓으면 안 되는 한계선(dashed) - 설계 상수(wall_top_z)와 "
            "실측(관측된 가장 낮은 상부 구조물, 정체 불문 보수적으로 반영) 중 더 낮은 값을 채택. "
            "x_min 쪽(입구)은 열린 방향이라 벽 없음."
        ),
        "vertices": v,
        "edges": edges,
        "faces": faces,
        "source_stats": {
            "n_points_raw": n_raw,
            "n_points_filtered": n_filtered,
            "crop_margin_xy_m": CROP_MARGIN_XY,
            "crop_margin_z_m": CROP_MARGIN_Z,
            "outlier_removal": {"nb_neighbors": OUTLIER_NB_NEIGHBORS, "std_ratio": OUTLIER_STD_RATIO},
        },
    }

    result["obstacles"] = [
        {
            "name": wh["name"],
            "vertices": box_vertices(wh["x_min"], wh["x_max"], wh["y_min"], wh["y_max"],
                                      wh["floor_z"], wh["top_z"]),
            "style": "solid",
            "note": "grid 휴리스틱 자동 검출 결과 - 필요시 손으로 조정",
        }
        for wh in wheel_houses
    ]

    return result


# --------------------------------------------------------------------------- #
# 시각화
# --------------------------------------------------------------------------- #

def dashed_lineset(p0, p1, n_segments=16, color=(1.0, 0.85, 0.0)):
    p0, p1 = np.asarray(p0), np.asarray(p1)
    pts, lines = [], []
    for i in range(n_segments):
        if i % 2 != 0:
            continue
        t0, t1 = i / n_segments, (i + 1) / n_segments
        pts.append(p0 + (p1 - p0) * t0)
        pts.append(p0 + (p1 - p0) * t1)
        lines.append([len(pts) - 2, len(pts) - 1])
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(np.array(pts))
    ls.lines = o3d.utility.Vector2iVector(np.array(lines))
    ls.colors = o3d.utility.Vector3dVector([color] * len(lines))
    return ls


def build_visual_geometries(trunk_map: dict, pts_base: np.ndarray):
    v = np.array(trunk_map["vertices"])
    solid_edges = [e["v"] for e in trunk_map["edges"] if e["style"] == "solid"]
    dashed_edges = [e["v"] for e in trunk_map["edges"] if e["style"] == "dashed"]

    solid_ls = o3d.geometry.LineSet()
    solid_ls.points = o3d.utility.Vector3dVector(v)
    solid_ls.lines = o3d.utility.Vector2iVector(np.array(solid_edges))
    solid_ls.colors = o3d.utility.Vector3dVector([(0.9, 0.1, 0.1)] * len(solid_edges))

    geoms = [solid_ls]
    for a, b in dashed_edges:
        geoms.append(dashed_lineset(v[a], v[b]))

    for obs in trunk_map.get("obstacles", []):
        ov = np.array(obs["vertices"])
        obs_edges = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4],
                     [0, 4], [1, 5], [2, 6], [3, 7]]
        obs_ls = o3d.geometry.LineSet()
        obs_ls.points = o3d.utility.Vector3dVector(ov)
        obs_ls.lines = o3d.utility.Vector2iVector(np.array(obs_edges))
        obs_ls.colors = o3d.utility.Vector3dVector([(0.1, 0.5, 0.95)] * len(obs_edges))
        geoms.append(obs_ls)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts_base)
    pcd.paint_uniform_color([0.6, 0.6, 0.6])

    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15)
    return [pcd, solid_ls, frame] + geoms[1:] + [frame]


def save_preview_png(trunk_map: dict, pts_base: np.ndarray, out_path: Path) -> None:
    """Open3D GUI(디스플레이) 없이도 결과를 눈으로 확인할 수 있는 2뷰(위/옆) 정적 이미지."""
    v = np.array(trunk_map["vertices"])
    rng = np.random.default_rng(0)
    sample = pts_base if len(pts_base) <= 60000 else pts_base[rng.choice(len(pts_base), 60000, replace=False)]

    fig, (ax_top, ax_side) = plt.subplots(1, 2, figsize=(14, 6))

    def draw_edges(ax, ia, ib, style):
        for e in trunk_map["edges"]:
            a, b = e["v"]
            xa, ya = v[a][ia], v[a][ib]
            xb, yb = v[b][ia], v[b][ib]
            if e["style"] == "solid":
                ax.plot([xa, xb], [ya, yb], color="red", linewidth=2)
            else:
                ax.plot([xa, xb], [ya, yb], color="orange", linewidth=2, linestyle="--")
        for obs in trunk_map.get("obstacles", []):
            ov = np.array(obs["vertices"])
            obs_edges = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4],
                         [0, 4], [1, 5], [2, 6], [3, 7]]
            for a, b in obs_edges:
                ax.plot([ov[a][ia], ov[b][ia]], [ov[a][ib], ov[b][ib]], color="dodgerblue", linewidth=2)

    # 위에서 본 뷰 (x-y)
    ax_top.scatter(sample[:, 0], sample[:, 1], s=0.3, c="gray", alpha=0.4)
    draw_edges(ax_top, 0, 1, "xy")
    ax_top.set_xlabel("x (base, m, +deep)")
    ax_top.set_ylabel("y (base, m)")
    ax_top.set_title("Top view (X-Y)")
    ax_top.set_aspect("equal")

    # 옆에서 본 뷰 (x-z)
    ax_side.scatter(sample[:, 0], sample[:, 2], s=0.3, c="gray", alpha=0.4)
    draw_edges(ax_side, 0, 2, "xz")
    ax_side.set_xlabel("x (base, m, +deep)")
    ax_side.set_ylabel("z (base, m, +up)")
    ax_side.set_title("Side view (X-Z) - 주황 점선 = 문 닫힘 높이 한계")
    ax_side.set_aspect("equal")

    legend = [
        Line2D([0], [0], color="red", lw=2, label="solid = 실측 (바닥/벽)"),
        Line2D([0], [0], color="orange", lw=2, linestyle="--", label="dashed = 설계 한계 (천장/문닫힘)"),
        Line2D([0], [0], color="dodgerblue", lw=2, label="obstacle (예: 타이어 하우징)"),
    ]
    fig.legend(handles=legend, loc="lower center", ncol=3)
    fig.suptitle(f"trunk_map preview - {trunk_map['run_id']} (frame={trunk_map['frame']})")
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", nargs="?", default=None,
                         help="results/run_YYYYMMDD_HHMMSS 경로 (생략 시 최신 run 자동 선택)")
    parser.add_argument("--visualize", action="store_true", help="처리 후 Open3D 창으로 확인")
    args = parser.parse_args()

    run_dir = resolve_run_dir(args.run_dir)
    pc_dir = run_dir / "pointcloud"
    npy_path = pc_dir / "trunk_pointcloud.npy"
    meta_path = pc_dir / "trunk_pointcloud_meta.json"
    if not npy_path.exists() or not meta_path.exists():
        raise SystemExit(f"[에러] {npy_path} 또는 {meta_path}가 없습니다.")

    pts_world = np.load(npy_path)
    meta = json.loads(meta_path.read_text())
    trunk_bounds = meta["trunk_bounds"]
    n_raw = len(pts_world)

    base_pos, base_quat = load_base_pose(meta)
    R_base = quat_wxyz_to_matrix(base_quat)

    pts_world_filtered = filter_points_world(pts_world, trunk_bounds)
    pts_base = world_to_base(pts_world_filtered, base_pos, R_base)

    aabb = compute_measured_aabb(pts_base)
    ceiling_z = compute_ceiling_z_base(trunk_bounds, base_pos, R_base, pts_base, aabb["floor_z"])
    print(f"[AABB 실측] x=[{aabb['x_min']:.3f},{aabb['x_max']:.3f}] "
          f"y=[{aabb['y_min']:.3f},{aabb['y_max']:.3f}] floor_z={aabb['floor_z']:.3f}")
    print(f"[천장(점선) 설정값] ceiling_z={ceiling_z:.3f} "
          f"(base 프레임 높이 = {ceiling_z - aabb['floor_z']:.3f}m)")

    wheel_houses = detect_wheel_houses(pts_base, aabb)

    trunk_map = build_trunk_map(aabb, ceiling_z, wheel_houses, run_dir.name, n_raw, len(pts_base))

    out_path = pc_dir / "trunk_map.json"
    out_path.write_text(json.dumps(trunk_map, indent=2, ensure_ascii=False))
    print(f"[저장] {out_path}")

    pcd_out = o3d.geometry.PointCloud()
    pcd_out.points = o3d.utility.Vector3dVector(pts_base)
    pcd_out = pcd_out.voxel_down_sample(VOXEL_SIZE)
    ply_path = pc_dir / "trunk_pointcloud_filtered_base.ply"
    o3d.io.write_point_cloud(str(ply_path), pcd_out)
    print(f"[저장] {ply_path} ({len(pcd_out.points)}점, base 프레임, voxel={VOXEL_SIZE}m)")

    preview_path = pc_dir / "trunk_map_preview.png"
    save_preview_png(trunk_map, np.asarray(pcd_out.points), preview_path)
    print(f"[저장] {preview_path} (Open3D GUI 없이 볼 수 있는 정적 미리보기)")

    if args.visualize:
        geoms = build_visual_geometries(trunk_map, np.asarray(pcd_out.points))
        o3d.visualization.draw_geometries(
            geoms, window_name=f"Trunk Map (base frame) - {run_dir.name}", width=1280, height=720,
        )


if __name__ == "__main__":
    main()
