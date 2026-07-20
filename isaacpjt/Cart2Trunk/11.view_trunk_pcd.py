from pathlib import Path
import sys

import numpy as np
import open3d as o3d

_THIS_DIR = Path(__file__).resolve().parent
RESULTS_DIR = _THIS_DIR / "results"

if len(sys.argv) > 1:
    # 특정 run 폴더 또는 npy 경로를 직접 지정 가능: python3 11.view_trunk_pcd.py results/run_20260720_150700
    arg = Path(sys.argv[1])
    npy_path = arg if arg.suffix == ".npy" else arg / "pointcloud" / "trunk_pointcloud.npy"
else:
    # 인자 없으면 가장 최근 run 폴더를 자동으로 찾는다.
    runs = sorted(RESULTS_DIR.glob("run_*"))
    if not runs:
        raise SystemExit(f"[에러] {RESULTS_DIR}에 run_* 폴더가 없습니다. 스캔 스크립트를 먼저 실행하세요.")
    npy_path = runs[-1] / "pointcloud" / "trunk_pointcloud.npy"
    print(f"[자동 선택] 최신 run: {runs[-1].name}")

print(f"[로드] {npy_path}")
pts = np.load(npy_path)

TRUNK_X_MIN, TRUNK_X_MAX = 3.11 - 0.3, 3.68 + 0.3
TRUNK_Y_MIN, TRUNK_Y_MAX = -0.56 - 0.3, 0.56 + 0.3
Z_MIN, Z_MAX = 0.3, 1.4
mask = (
    (pts[:, 0] > TRUNK_X_MIN) & (pts[:, 0] < TRUNK_X_MAX) &
    (pts[:, 1] > TRUNK_Y_MIN) & (pts[:, 1] < TRUNK_Y_MAX) &
    (pts[:, 2] > Z_MIN) & (pts[:, 2] < Z_MAX)
)
pts = pts[mask]

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pts)

z = pts[:, 2]
t = (z - z.min()) / (z.ptp() + 1e-9)
colors = np.stack([t * 0.2, 0.5 + t * 0.4, 0.6 + t * 0.3], axis=1)
pcd.colors = o3d.utility.Vector3dVector(np.clip(colors, 0, 1))

ply_path = npy_path.with_suffix(".ply")
o3d.io.write_point_cloud(str(ply_path), pcd)
print(pcd)

frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15)

o3d.visualization.draw_geometries(
    [pcd, frame],
    window_name="Trunk Cavity PointCloud",
    width=1280,
    height=720,
)
