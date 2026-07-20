"""
02_trunk_space_state.py
② 공간 상태 구성
==================
상태: 🟡 진행 중 — 지완의 실제 스캔 파이프라인 연동 완료, 돌출부(타이어 하우징 등)는 보류

[연동 완료 (2026-07-20)]
    13.export_trunk_map.py가 포인트클라우드(results/run_*/pointcloud/trunk_pointcloud.npy)를
    M0609 base_link 좌표계의 trunk_map.json으로 변환해서 내보낸다. load_trunk_from_scan()이
    이 파일을 읽어 Trunk로 바꾼다 - Q2(좌표계 절대/상대) 답은 "M0609 base_link 기준"으로 확정.

    trunk_map.json 안에서 바닥/좌우벽/안쪽벽(4면)은 그 run의 포인트클라우드에서 실측한 값이고
    (edges/faces style="solid"), 천장(문이 닫히는 높이 한계선)은 트렁크를 연 채로 스캔해서
    실제로는 존재하지 않는 면이라 스캔 스크립트의 설계 상수(wall_top_z)를 그대로 쓴다
    (style="dashed") - Trunk.height는 이 dashed 값까지 포함한 높이이므로, 적재 계획에서
    "이 높이 넘으면 트렁크 문이 안 닫힌다"는 제약으로 그대로 쓰면 된다.

[아직 막힌 지점]
    run_20260720_160153을 처리해보니 바닥 위 약 0.5~0.6m 높이에 넓게 걸쳐있는 점 무리가
    있음 (interior grid의 33%) - 열린 트렁크 문(리드) 자체가 찍힌 것인지, 뒷좌석/파셀shelf
    같은 실제 구조물인지, 어두운 캐비티 안 센서 노이즈인지 아직 구분 못 함. 작은 돌출부로
    보기엔 너무 넓어서 13.export_trunk_map.py가 자동 박스화를 포기하도록 만들어뒀다
    (obstacles: [] 로 저장됨). 실물/스윕 사진(results/*/sweep/*.png) 확인해서 정체를 파악하기
    전까지는 obstacles 리스트가 비어있는 채로 취급한다.

[Trunk 자료구조]
    로컬 좌표계, 원점 (0,0,0) = trunk_map.json의 바닥 안쪽 코너 (x_min, y_min, floor_z).
    이미 트렁크 안에 있는 물품/돌출부가 있다면 load_obstacles_from_scan()으로 PlacedBox
    리스트를 얻어 ExtremePointState에 "처음부터 점유된 자리"로 등록하면 됨.

[참고 - 예전 시뮬레이션 씬 값(8.rescale_and_rebuild.py), 지금은 REAL_TRUNK 검증용으로만 사용]
    월드 절대좌표 기준:
        TRUNK_X_MIN, TRUNK_X_MAX = 3.11, 3.68
        TRUNK_Y_MIN, TRUNK_Y_MAX = -0.56, 0.56
        TRUNK_FLOOR_Z = 1.03
        TRUNK_WALL_TOP = 1.28
    로컬 치수로 환산하면: width=0.57m, depth=1.12m, height=0.25m
    (이건 지완이 만든 시뮬레이션 씬의 손으로 잰 값이라 "실제 스캔 데이터"는 아님 -
     10_verification.py가 참조하므로 그대로 남겨두고, 새 코드는 load_trunk_from_scan() 사용)
"""

from dataclasses import dataclass, field
from typing import List


@dataclass
class Trunk:
    """
    트렁크 공간 (로컬 좌표계, 원점 (0,0,0) 기준).

    ⚠️ TODO (팀원 데이터 도착 시):
        준형의 실제 스캔 결과(Point Cloud / Occupancy Map)가 오면
        - 단순 직육면체 가정으로 충분한지 재검토
        - 이미 트렁크 안에 있는 물품이 있다면 occupied 리스트로 반영
    """
    width: float   # x축 (m)
    depth: float   # y축 (m)
    height: float  # z축 (m)


# ---- 실제 저장소(8.rescale_and_rebuild.py)에서 확인한 값 (임시 - 준형 스캔 데이터로 교체 예정) ----
WORLD_ORIGIN = (3.11, -0.56, 1.03)  # 로컬 (0,0,0)이 가리키는 월드 좌표 (X_MIN, Y_MIN, FLOOR_Z)

REAL_TRUNK = Trunk(
    width=3.68 - 3.11,   # 0.57
    depth=0.56 - (-0.56),  # 1.12
    height=1.28 - 1.03,   # 0.25
)


def local_to_world(x: float, y: float, z: float):
    """로컬 좌표(0,0,0 시작) -> 월드 절대좌표 변환 (예전 REAL_TRUNK용). 새 스캔은 base_link
    프레임이 곧 기준이라 이 변환이 필요 없음 - load_trunk_from_scan() 참고."""
    ox, oy, oz = WORLD_ORIGIN
    return (x + ox, y + oy, z + oz)


def load_trunk_from_scan(trunk_map_path) -> Trunk:
    """
    13.export_trunk_map.py가 만든 trunk_map.json(M0609 base_link 좌표계)을 Trunk로 변환한다.
    Trunk 로컬 원점(0,0,0)은 trunk_map.json의 바닥 안쪽 코너
    (vertices[0] = x_min, y_min, floor_z)로 둔다. height는 dashed(설계 상수) 천장까지 포함한
    값이므로, 그 자체가 "문이 닫히는 높이 제약"이 된다.
    """
    import json
    from pathlib import Path

    data = json.loads(Path(trunk_map_path).read_text())
    x0, y0, z0 = data["vertices"][0]  # 바닥 코너 (x_min, y_min, floor_z) - solid
    x1, y1, z1 = data["vertices"][6]  # 반대편 코너 (x_max, y_max, ceiling_z) - z1은 dashed
    return Trunk(width=x1 - x0, depth=y1 - y0, height=z1 - z0)


def load_obstacles_from_scan(trunk_map_path) -> "List[PlacedBox]":
    """
    trunk_map.json의 obstacles(예: 타이어 하우징처럼 바닥에서 튀어나온 돌출부)를
    ExtremePointState.register_placement()로 바로 등록 가능한 PlacedBox 리스트로 변환한다.
    좌표는 load_trunk_from_scan()과 동일한 로컬 원점(바닥 코너) 기준으로 맞춘다.

    지금은 13.export_trunk_map.py가 애매한 돌출부는 자동 박스화를 포기하므로
    (위 "아직 막힌 지점" 참고) obstacles가 보통 빈 리스트다 - 그 경우 그냥 빈 리스트 반환.
    """
    import json
    import sys
    import pathlib
    from importlib import import_module

    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    _m03 = import_module("03_extreme_point_candidates")
    Box, PlacedBox = _m03.Box, _m03.PlacedBox

    data = json.loads(pathlib.Path(trunk_map_path).read_text())
    ox, oy, oz = data["vertices"][0]

    placed = []
    for i, obs in enumerate(data.get("obstacles", [])):
        xs = [p[0] for p in obs["vertices"]]
        ys = [p[1] for p in obs["vertices"]]
        zs = [p[2] for p in obs["vertices"]]
        x_min, y_min, z_min = min(xs), min(ys), min(zs)
        x_max, y_max, z_max = max(xs), max(ys), max(zs)
        box = Box(id=obs.get("name", f"obstacle_{i}"),
                   width=x_max - x_min, depth=y_max - y_min, height=z_max - z_min)
        placed.append(PlacedBox(box=box, x=x_min - ox, y=y_min - oy, z=z_min - oz))
    return placed
