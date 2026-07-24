"""
90.export_trunk_map_holonomic.py

13.export_trunk_map.py를 89.trunk_scan_holonomic.py의 출력 레이아웃에 맞게 포팅.
계획 파일(~/.claude/plans/parallel-juggling-sun.md) 89번 항목("13.py 포팅") 참고.

Isaac Sim 불필요 - 13.py와 동일하게 일반 python3 + numpy + open3d + scipy로 실행한다.

    python3 90.export_trunk_map_holonomic.py

알고리즘(좌표변환/AABB 실측/천장 RANSAC/점유영역 grid+connected-component 검출/JSON 조립)은
13.py와 완전히 동일 - 포인트클라우드 기하 처리는 어떤 로봇이 스캔했는지와 무관하기 때문이다.
바뀐 것은 입출력 경로뿐이다:
  - 13.py: results/run_YYYYMMDD_HHMMSS/pointcloud/{trunk_pointcloud.npy,trunk_pointcloud_meta.json}
  - 이 포팅판: results/holonomic_base/{trunk_pointcloud.npy,trunk_pointcloud_meta.json}
    (89.trunk_scan_holonomic.py가 이 위치에 저장함)
89.py가 만드는 meta.json에는 base_pos/base_quat가 항상 들어있으므로(89.py가 실측해서
직접 저장), 13.py에 있던 "구버전 run(좌표 없음) 근사 재구성" fallback은 필요 없어 제거했다.
"""

from __future__ import annotations

import json
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
PC_DIR = _THIS_DIR / "results" / "holonomic_base"

CROP_MARGIN_XY = 0.30
CROP_MARGIN_Z = 0.15
OUTLIER_NB_NEIGHBORS = 20
OUTLIER_STD_RATIO = 2.0
VOXEL_SIZE = 0.01

BOUND_PCTL_LOW = 1.0
BOUND_PCTL_HIGH = 99.0
CEILING_OVERHEAD_PCTL = 5.0
CEILING_MIN_OVERHEAD_POINTS = 500
CEILING_PLANE_DIST_THRESHOLD = 0.02
CEILING_PLANE_RANSAC_ITERS = 2000
CEILING_MIN_PLANE_INLIERS = 500
CEILING_PLANE_MIN_HORIZONTALITY = 0.8

OCCUPIED_CELL = 0.03
OCCUPIED_EDGE_MARGIN = 0.05
OCCUPIED_BUMP_THRESHOLD = 0.03
OCCUPIED_MIN_CELLS = 6
OCCUPIED_MAX_AREA_FRACTION = 0.25
OCCUPIED_MAX_Y_SPAN_FRACTION = 0.5
OCCUPIED_MAX_HEIGHT_ABOVE_FLOOR = 0.40


# --------------------------------------------------------------------------- #
# 좌표 변환 (13.py와 동일)
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
# 포인트클라우드 정제 (13.py와 동일)
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
# AABB (실측: 바닥 + 3벽) 산출 (13.py와 동일)
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
    """설계 상수(wall_top_z)와 실측(RANSAC 평면) 중 더 낮은(보수적인) 쪽을 채택 - 13.py와 동일."""
    cx = (trunk_bounds["x"][0] + trunk_bounds["x"][1]) / 2.0
    cy = (trunk_bounds["y"][0] + trunk_bounds["y"][1]) / 2.0
    p_world = np.array([[cx, cy, trunk_bounds["wall_top_z"]]])
    config_ceiling_z = float(world_to_base(p_world, base_pos, R_base)[0][2])

    overhead = pts_base[pts_base[:, 2] > floor_z + OCCUPIED_MAX_HEIGHT_ABOVE_FLOOR]
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
# 바닥 위 점유 공간(obstacle) 검출 - grid 휴리스틱 + connected-component (13.py와 동일)
# --------------------------------------------------------------------------- #

def detect_occupied_regions(pts_base: np.ndarray, aabb: dict) -> list[dict]:
    from scipy import ndimage

    x_min, x_max = aabb["x_min"], aabb["x_max"]
    y_min, y_max = aabb["y_min"], aabb["y_max"]
    floor_z = aabb["floor_z"]

    band_pts = pts_base[pts_base[:, 2] < floor_z + OCCUPIED_MAX_HEIGHT_ABOVE_FLOOR]
    print(f"[점유영역] 문 높이 제외 후 후보점: {len(pts_base)} -> {len(band_pts)} "
          f"(z < floor_z+{OCCUPIED_MAX_HEIGHT_ABOVE_FLOOR}m)")

    nx = max(1, int((x_max - x_min) / OCCUPIED_CELL))
    ny = max(1, int((y_max - y_min) / OCCUPIED_CELL))
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

    edge_margin_cells_x = max(1, int(OCCUPIED_EDGE_MARGIN / OCCUPIED_CELL))
    edge_margin_cells_y = max(1, int(OCCUPIED_EDGE_MARGIN / OCCUPIED_CELL))
    interior = np.zeros_like(local_min_z, dtype=bool)
    interior[edge_margin_cells_x:nx - edge_margin_cells_x, edge_margin_cells_y:ny - edge_margin_cells_y] = True
    interior &= ~np.isnan(local_min_z)
    n_interior = int(np.count_nonzero(interior))
    if n_interior == 0:
        print("[점유영역] interior에 유효 데이터 없음 -> 검출 안 함")
        return []

    floor_ref = float(np.median(local_min_z[interior]))
    bump_mask = interior & (local_min_z > (floor_ref + OCCUPIED_BUMP_THRESHOLD))
    print(f"[점유영역] floor_ref(local median)={floor_ref:.3f} (전역 1pctl={floor_z:.3f}) "
          f"bump 후보 {int(np.count_nonzero(bump_mask))}/{n_interior} cell")

    labeled, n_blobs = ndimage.label(bump_mask, structure=np.ones((3, 3)))
    y_span_total = y_max - y_min
    blobs = []
    for blob_id in range(1, n_blobs + 1):
        blob = labeled == blob_id
        n_cells = int(np.count_nonzero(blob))
        area_fraction = n_cells / n_interior
        if n_cells < OCCUPIED_MIN_CELLS:
            continue
        if area_fraction > OCCUPIED_MAX_AREA_FRACTION:
            print(f"[점유영역] 덩어리(cell {n_cells}개)가 interior의 {area_fraction:.0%}를 차지 "
                  f"(> {OCCUPIED_MAX_AREA_FRACTION:.0%}) - 바닥 전체 오검출 가능성이 커서 제외")
            continue

        cx_idx, cy_idx = np.nonzero(blob)
        bx_min, bx_max = x_edges[cx_idx.min()], x_edges[cx_idx.max() + 1]
        by_min, by_max = y_edges[cy_idx.min()], y_edges[cy_idx.max() + 1]
        if (by_max - by_min) > OCCUPIED_MAX_Y_SPAN_FRACTION * y_span_total:
            print(f"[점유영역] 덩어리(cell {n_cells}개, y=[{by_min:.3f},{by_max:.3f}])가 폭 전체의 "
                  f"{(by_max - by_min) / y_span_total:.0%}를 가로질러 제외 (문턱/선반 등으로 추정)")
            continue

        top_z = float(np.nanpercentile(local_min_z[blob], 90))
        blobs.append({"x_min": float(bx_min), "x_max": float(bx_max),
                       "y_min": float(by_min), "y_max": float(by_max),
                       "floor_z": floor_z, "top_z": top_z, "n_cells": n_cells})

    if not blobs:
        print("[점유영역] 조건을 만족하는 덩어리 없음 -> 검출 안 함")
        return []

    results = []
    for i, b in enumerate(sorted(blobs, key=lambda b: -b["n_cells"]), start=1):
        name = f"obstacle_{i}"
        entry = dict(b, name=name)
        results.append(entry)
        print(f"[점유영역] {name}: x=[{b['x_min']:.3f},{b['x_max']:.3f}] "
              f"y=[{b['y_min']:.3f},{b['y_max']:.3f}] top_z={b['top_z']:.3f} (cell {b['n_cells']}개)")

    return results


# --------------------------------------------------------------------------- #
# JSON 조립 (13.py와 동일)
# --------------------------------------------------------------------------- #

def box_vertices(x_min, x_max, y_min, y_max, z_min, z_max):
    return [
        [x_min, y_min, z_min], [x_max, y_min, z_min],
        [x_max, y_max, z_min], [x_min, y_max, z_min],
        [x_min, y_min, z_max], [x_max, y_min, z_max],
        [x_max, y_max, z_max], [x_min, y_max, z_max],
    ]


def build_trunk_map(aabb: dict, ceiling_z: float, detected_objects: list[dict],
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
            "name": obj["name"],
            "vertices": box_vertices(obj["x_min"], obj["x_max"], obj["y_min"], obj["y_max"],
                                      obj["floor_z"], obj["top_z"]),
            "style": "solid",
            "note": (
                "grid 휴리스틱 자동 검출 결과 - 휠하우스/기존 물건 구분 없이 바닥 위 점유 공간을 "
                "모두 extreme point(AABB)로 표시. 필요시 손으로 조정"
            ),
        }
        for obj in detected_objects
    ]

    return result


# --------------------------------------------------------------------------- #
# 시각화 (13.py와 동일)
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

    ax_top.scatter(sample[:, 0], sample[:, 1], s=0.3, c="gray", alpha=0.4)
    draw_edges(ax_top, 0, 1, "xy")
    ax_top.set_xlabel("x (base, m, +deep)")
    ax_top.set_ylabel("y (base, m)")
    ax_top.set_title("Top view (X-Y)")
    ax_top.set_aspect("equal")

    ax_side.scatter(sample[:, 0], sample[:, 2], s=0.3, c="gray", alpha=0.4)
    draw_edges(ax_side, 0, 2, "xz")
    ax_side.set_xlabel("x (base, m, +deep)")
    ax_side.set_ylabel("z (base, m, +up)")
    ax_side.set_title("Side view (X-Z) - 주황 점선 = 문 닫힘 높이 한계")
    ax_side.set_aspect("equal")

    legend = [
        Line2D([0], [0], color="red", lw=2, label="solid = 실측 (바닥/벽)"),
        Line2D([0], [0], color="orange", lw=2, linestyle="--", label="dashed = 설계 한계 (천장/문닫힘)"),
        Line2D([0], [0], color="dodgerblue", lw=2, label="obstacle = 바닥 위 점유 공간 (휠하우스/기존 물건 구분 없음)"),
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
    npy_path = PC_DIR / "trunk_pointcloud.npy"
    meta_path = PC_DIR / "trunk_pointcloud_meta.json"
    if not npy_path.exists() or not meta_path.exists():
        raise SystemExit(f"[에러] {npy_path} 또는 {meta_path}가 없습니다. "
                          f"먼저 89.trunk_scan_holonomic.py를 실행하세요.")

    pts_world = np.load(npy_path)
    meta = json.loads(meta_path.read_text())
    trunk_bounds = meta["trunk_bounds"]
    n_raw = len(pts_world)

    # 89.py는 항상 base_pos/base_quat를 실측해서 저장하므로(13.py의 구버전 fallback 불필요).
    base_pos = np.asarray(meta["base_pos"], dtype=float)
    base_quat = np.asarray(meta["base_quat"], dtype=float)
    print(f"[base pose] meta.json에서 로드 pos={base_pos} quat={base_quat}")
    R_base = quat_wxyz_to_matrix(base_quat)

    pts_world_filtered = filter_points_world(pts_world, trunk_bounds)
    pts_base = world_to_base(pts_world_filtered, base_pos, R_base)

    aabb = compute_measured_aabb(pts_base)
    ceiling_z = compute_ceiling_z_base(trunk_bounds, base_pos, R_base, pts_base, aabb["floor_z"])
    print(f"[AABB 실측] x=[{aabb['x_min']:.3f},{aabb['x_max']:.3f}] "
          f"y=[{aabb['y_min']:.3f},{aabb['y_max']:.3f}] floor_z={aabb['floor_z']:.3f}")
    print(f"[천장(점선) 설정값] ceiling_z={ceiling_z:.3f} "
          f"(base 프레임 높이 = {ceiling_z - aabb['floor_z']:.3f}m)")

    detected_objects = detect_occupied_regions(pts_base, aabb)

    run_id = "holonomic_" + npy_path.stat().st_mtime.__str__().split(".")[0]
    trunk_map = build_trunk_map(aabb, ceiling_z, detected_objects, run_id, n_raw, len(pts_base))

    out_path = PC_DIR / "trunk_map.json"
    out_path.write_text(json.dumps(trunk_map, indent=2, ensure_ascii=False))
    print(f"[저장] {out_path}")

    pcd_out = o3d.geometry.PointCloud()
    pcd_out.points = o3d.utility.Vector3dVector(pts_base)
    pcd_out = pcd_out.voxel_down_sample(VOXEL_SIZE)
    ply_path = PC_DIR / "trunk_pointcloud_filtered_base.ply"
    o3d.io.write_point_cloud(str(ply_path), pcd_out)
    print(f"[저장] {ply_path} ({len(pcd_out.points)}점, base 프레임, voxel={VOXEL_SIZE}m)")

    preview_path = PC_DIR / "trunk_map_preview.png"
    save_preview_png(trunk_map, np.asarray(pcd_out.points), preview_path)
    print(f"[저장] {preview_path} (Open3D GUI 없이 볼 수 있는 정적 미리보기)")


if __name__ == "__main__":
    main()
