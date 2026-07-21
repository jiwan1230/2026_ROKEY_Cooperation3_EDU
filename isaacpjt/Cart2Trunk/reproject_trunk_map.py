"""
reproject_trunk_map.py
서로 다른 세션에서 스캔된 trunk_map.json을, 지금 쓰려는 box 비전 세션의 m0609_base_link
좌표계로 재투영한다. Isaac Sim 불필요 - 순수 numpy.

배경
----
14_run_full_pipeline.py(algorism/)는 트렁크와 박스 데이터가 같은 m0609_base_link 좌표계라고
전제한다(01_object3d_schema.py Q2). 그런데 "m0609_base_link"는 고정된 world 좌표계가 아니라
그 스캔을 할 때 로봇이 실제로 서 있던 world pose에 상대적인 좌표계다 - 트렁크 스캔
(12.trunk_scan_hidden_gripper.py 세션)과 박스 스캔(32.box_table_scan_setup.py 세션)을 서로
다른 로봇 위치에서 했다면, 이름은 같은 "m0609_base_link"라도 실제로는 물리적으로 다른
원점이다. 33.box_table_pick_to_trunk.py 작업 때 이 문제를 실제로 겪었다 - 이 스크립트로
먼저 재투영해서 두 데이터를 진짜 같은 좌표계로 맞춘 뒤에 14_run_full_pipeline.py를 돌려야
position_base_frame 결과가 의미가 있다.

사용법
----
    python3 reproject_trunk_map.py <trunk_map.json> <trunk_meta.json> <base_to_camera_transform.json> <출력경로>

<trunk_meta.json>은 트렁크 스캔 세션의 base_pos/base_quat가 든 파일
(12.trunk_scan_hidden_gripper.py가 저장한 trunk_pointcloud_meta.json).
<base_to_camera_transform.json>은 이번에 실제로 쓸 박스 비전 세션의 base_pos/base_quat가 든
파일(32.box_table_scan_setup.py 출력, measured_base_pos/measured_base_quat 필드).
"""

import json
import sys
from pathlib import Path

import numpy as np


def quat_wxyz_to_matrix(q) -> np.ndarray:
    """13.export_trunk_map.py와 동일."""
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


def reproject(points_base_scan, R_scan, base_pos_scan, R_mine, base_pos_mine):
    """p_base_scan -> world -> p_base_mine."""
    points_base_scan = np.asarray(points_base_scan, dtype=np.float64)
    points_world = points_base_scan @ R_scan.T + base_pos_scan
    return (points_world - base_pos_mine) @ R_mine


def main():
    if len(sys.argv) != 5:
        print(__doc__)
        raise SystemExit(1)

    trunk_map_path, trunk_meta_path, box_transform_path, out_path = (Path(p) for p in sys.argv[1:5])

    trunk_map = json.loads(trunk_map_path.read_text())
    trunk_meta = json.loads(trunk_meta_path.read_text())
    box_transform = json.loads(box_transform_path.read_text())

    base_pos_scan = np.array(trunk_meta["base_pos"], dtype=np.float64)
    R_scan = quat_wxyz_to_matrix(np.array(trunk_meta["base_quat"], dtype=np.float64))

    base_pos_mine = np.array(box_transform["measured_base_pos"], dtype=np.float64)
    R_mine = quat_wxyz_to_matrix(np.array(box_transform["measured_base_quat"], dtype=np.float64))

    print("=== 세션 pose 확인 ===")
    print(f"트렁크 스캔 세션  base_pos={base_pos_scan}")
    print(f"박스 비전 세션    base_pos={base_pos_mine}")

    vertices = reproject(trunk_map["vertices"], R_scan, base_pos_scan, R_mine, base_pos_mine)

    new_obstacles = []
    for obs in trunk_map.get("obstacles", []):
        obs_v = reproject(obs["vertices"], R_scan, base_pos_scan, R_mine, base_pos_mine)
        new_obstacles.append({"name": obs.get("name", "obstacle"), "vertices": obs_v.tolist()})

    new_trunk_map = {
        "frame": "m0609_base_link (box 비전 세션 기준으로 재투영됨)",
        "vertices": vertices.tolist(),
        "edges": trunk_map["edges"],
        "obstacles": new_obstacles,
    }

    out_path.write_text(json.dumps(new_trunk_map, indent=2))
    print(f"\n[저장] {out_path}")

    xs, ys, zs = vertices[:, 0], vertices[:, 1], vertices[:, 2]
    print("\n=== 재투영된 트렁크 (박스 비전 세션 base_link 기준) ===")
    print(f"x: {xs.min():.3f} ~ {xs.max():.3f} (span {xs.max()-xs.min():.3f}m)")
    print(f"y: {ys.min():.3f} ~ {ys.max():.3f} (span {ys.max()-ys.min():.3f}m)")
    print(f"z: {zs.min():.3f} ~ {zs.max():.3f} (span {zs.max()-zs.min():.3f}m)")

    # world 좌표로 역산해서 상식적인 위치(차량 근처)인지 사람이 검산할 수 있게 출력.
    world_v = vertices @ R_mine.T + base_pos_mine
    wxs, wys, wzs = world_v[:, 0], world_v[:, 1], world_v[:, 2]
    print("\n=== world 좌표로 역산 (차량 위치 근처인지 확인용) ===")
    print(f"x: {wxs.min():.3f} ~ {wxs.max():.3f}")
    print(f"y: {wys.min():.3f} ~ {wys.max():.3f}")
    print(f"z: {wzs.min():.3f} ~ {wzs.max():.3f}")


if __name__ == "__main__":
    main()
