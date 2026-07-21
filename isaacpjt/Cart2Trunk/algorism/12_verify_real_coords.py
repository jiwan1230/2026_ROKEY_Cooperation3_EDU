"""
12_verify_real_coords.py
지완이 준 실제 trunk_map.json으로 알고리즘 전체 파이프라인을 검증한다.

경로 구조 (git 저장소 바깥의 run_* 폴더를 직접 참조 - git 충돌 방지):
    cobot3_ws/src/
    ├── 2026_ROKEY_Cooperation3_EDU/                      ← git 저장소
    │   └── isaacpjt/Cart2Trunk/algorism/12_verify_real_coords.py  ← 이 파일
    ├── run_20260720_160153/pointcloud/trunk_map.json
    ├── run_20260720_200104/pointcloud/trunk_map.json
    └── TRUNK_MAP_ROS2_HANDOFF.md
"""
import sys, pathlib
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
m02 = import_module("02_trunk_space_state")
m03 = import_module("03_extreme_point_candidates")
m04 = import_module("04_candidate_validity_check")
m09 = import_module("09_rescan_replan")

# algorism -> Cart2Trunk -> isaacpjt -> 2026_ROKEY_Cooperation3_EDU -> src
_SRC_DIR = pathlib.Path(__file__).resolve().parents[4]


def run(run_folder_name, boxes):
    json_path = str(_SRC_DIR / run_folder_name / "pointcloud" / "trunk_map.json")
    print(f"\n{'='*60}\n{run_folder_name}\n{'='*60}")

    world_map = m02.load_trunk_from_world_map(json_path)
    trunk, offset = world_map.to_bounding_trunk()
    print("트렁크 크기:", trunk)
    print("오프셋:", offset)

    obstacles = m02.load_obstacles_from_world_map(json_path, offset)
    print("장애물 개수:", len(obstacles))

    plans, unloadable = m09.replan_after_rescan(boxes, trunk, obstacles)

    print("\n--- 배치 결과 ---")
    for p in plans:
        print(" ", p)
    print("--- 미적재 ---")
    for u in unloadable:
        print(" ", u)

    return plans, unloadable, trunk, obstacles


def sanity_check_with_bruteforce(trunk, obstacles, box, step=0.01):
    """극점 알고리즘이 놓친 자리가 실제로 있는지 격자로 전수 조사."""
    found = 0
    x = 0.0
    while x + box.width <= trunk.width:
        y = 0.0
        while y + box.depth <= trunk.depth:
            if m04.is_candidate_valid(x, y, 0.0, box, trunk, obstacles):
                found += 1
            y += step
        x += step
    return found


if __name__ == "__main__":
    print("SRC_DIR 확인:", _SRC_DIR, "존재함?", _SRC_DIR.exists())

    boxes = [
        m03.Box("Small", 0.30, 0.20, 0.15),
        m03.Box("Medium", 0.40, 0.30, 0.25),
        m03.Box("Large", 0.50, 0.35, 0.30),
    ]

    for run_folder in ["run_20260720_160153", "run_20260720_200104"]:
        plans, unloadable, trunk, obstacles = run(run_folder, boxes)

        print("\n--- 브루트포스 교차 검증 ---")
        for u in unloadable:
            box = next(b for b in boxes if b.id == u.box_id)
            cnt = sanity_check_with_bruteforce(trunk, obstacles, box)
            flag = "⚠️ 실제로는 자리 있음 (알고리즘 문제)" if cnt > 0 else "진짜로 자리 없음"
            print(f"  {u.box_id}: 격자 전수조사 결과 {cnt}곳 발견 → {flag}")