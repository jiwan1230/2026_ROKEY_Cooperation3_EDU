"""
04_candidate_validity_check.py
④ 후보 유효성 검사
=====================
상태: 🟢 완료·확정

③에서 만든 후보 좌표들이 전부 "쓸 수 있는" 자리는 아니다. 실제로 박스를
놓았을 때 두 가지 기하학 규칙을 어기지 않는지 확인해야 한다.

    1. 겹치지 않는가 — 두 박스가 3D 공간에서 서로 침범하면 물리적으로 불가능
    2. 트렁크 밖으로 나가지 않는가 — 모든 박스가 트렁크 경계 안에 있어야 함

이 두 체크를 통과한 후보만 "유효한 후보"로 ⑤(점수화)에 넘긴다.
"""

import sys, pathlib
from typing import List, Tuple
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m03 = import_module("03_extreme_point_candidates")

Box = _m03.Box
PlacedBox = _m03.PlacedBox
PLACEMENT_SAFETY_MARGIN_M = _m03.PLACEMENT_SAFETY_MARGIN_M


def boxes_overlap(a: "PlacedBox", b: "PlacedBox") -> bool:
    """두 박스가 3축(x,y,z) 모두에서 겹치면 실제로 겹치는 것 (AABB 충돌 판정)."""
    ax0, ax1 = a.x_range  # a박스의 x축 구간 [ax0, ax1]
    bx0, bx1 = b.x_range  # b박스의 x축 구간
    ay0, ay1 = a.y_range
    by0, by1 = b.y_range
    az0, az1 = a.z_range
    bz0, bz1 = b.z_range
    # 각 축마다 "구간이 겹치는가"(ax0 < bx1 and ax1 > bx0)를 확인.
    # 3축 전부 겹쳐야만 실제 3D 공간에서 두 박스가 겹치는 것 (하나라도 안 겹치면 안전).
    return (ax0 < bx1 and ax1 > bx0) and (ay0 < by1 and ay1 > by0) and (az0 < bz1 and az1 > bz0)


def _boxes_too_close(a: "PlacedBox", b: "PlacedBox", margin: float) -> bool:
    """boxes_overlap()과 같은 AABB 겹침 판정이지만, a를 x/y(수평)로만 margin만큼
    부풀린 뒤 검사한다 - 실제로는 안 겹쳐도 margin보다 가까우면 "너무 가깝다"고
    본다. z(높이)는 부풀리지 않는다: 바닥에 딱 놓이는 것도, (allow_stacking일 때)
    다른 박스 위에 딱 얹히는 것도 정상이라 그쪽에는 여유가 필요 없다."""
    ax0, ax1 = a.x_range[0] - margin, a.x_range[1] + margin
    ay0, ay1 = a.y_range[0] - margin, a.y_range[1] + margin
    az0, az1 = a.z_range
    bx0, bx1 = b.x_range
    by0, by1 = b.y_range
    bz0, bz1 = b.z_range
    return (ax0 < bx1 and ax1 > bx0) and (ay0 < by1 and ay1 > by0) and (az0 < bz1 and az1 > bz0)


def find_overlaps(placed: List["PlacedBox"]) -> List[Tuple[str, str]]:
    """겹치는 박스 쌍의 (id, id) 리스트를 반환. 빈 리스트면 겹침 없음 = 통과."""
    pairs = []
    for i in range(len(placed)):
        for j in range(i + 1, len(placed)):
            if boxes_overlap(placed[i], placed[j]):
                pairs.append((placed[i].box.id, placed[j].box.id))
    return pairs


def find_out_of_bounds(placed: List["PlacedBox"], trunk) -> List[str]:
    """트렁크 경계를 벗어난 박스 id 리스트를 반환. 빈 리스트면 전부 경계 안 = 통과."""
    violations = []
    for p in placed:
        if (p.x < 0 or p.y < 0 or p.z < 0
                or p.x_range[1] > trunk.width + 1e-9
                or p.y_range[1] > trunk.depth + 1e-9
                or p.z_range[1] > trunk.height + 1e-9):
            violations.append(p.box.id)
    return violations


def is_candidate_valid(x: float, y: float, z: float, box: "Box", trunk, placed: List["PlacedBox"]) -> bool:
    """
    후보 좌표 (x,y,z)에 box를 놓았을 때 유효한지 종합 판단.
    (경계 안 + 벽/장애물과 안전 여유 확보 + 기존 박스와 안 겹침)

    벽 쪽(x/y 수평면) 경계와 다른 박스와의 간격은 PLACEMENT_SAFETY_MARGIN_M(0.01m)
    이상 확보되도록 요구한다 - 03_extreme_point_candidates.py 모듈 docstring의
    안전 여유 설명 참고. 후보 생성 쪽(register_placement/generate_wall_flush_
    candidates)이 이미 이 여유를 두고 후보를 만들지만, 여기서도 같은 기준으로
    다시 확인해서 다른 경로로 들어온 후보도 동일하게 걸러낸다(방어적 이중 검증).
    z(바닥/높이 방향)는 여유를 두지 않는다 - 바닥에 딱 놓이는 것도, 트렁크 높이
    한계에 딱 맞는 것도 정상이라 그쪽에는 여유가 필요 없다.
    """
    # 박스를 (x,y,z)에 놓았을 때 반대쪽 끝(x+width 등)이 트렁크 경계를 넘는지 확인.
    # +1e-9는 부동소수점 오차로 인해 "딱 맞는 경우"가 false로 튕기는 걸 방지하는 여유값.
    if x < PLACEMENT_SAFETY_MARGIN_M - 1e-9:
        return False
    if y < PLACEMENT_SAFETY_MARGIN_M - 1e-9:
        return False
    if x + box.width > trunk.width - PLACEMENT_SAFETY_MARGIN_M + 1e-9:
        return False
    if y + box.depth > trunk.depth - PLACEMENT_SAFETY_MARGIN_M + 1e-9:
        return False
    if z + box.height > trunk.height + 1e-9:
        return False

    # 경계는 통과했으니, 이미 놓인 박스들과 안전 여유(x/y) 안에 들어오면 무효
    candidate = PlacedBox(box=box, x=x, y=y, z=z)
    return not any(_boxes_too_close(candidate, p, PLACEMENT_SAFETY_MARGIN_M) for p in placed)


if __name__ == "__main__":
    from importlib import import_module
    _m02 = import_module("02_trunk_space_state")
    Trunk = _m02.Trunk

    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    b1 = PlacedBox(box=Box("A", 0.4, 0.3, 0.2), x=0.0, y=0.0, z=0.0)
    b2 = PlacedBox(box=Box("B", 0.4, 0.3, 0.2), x=0.2, y=0.0, z=0.0)  # A와 겹침

    print("겹침 쌍:", find_overlaps([b1, b2]))
    print("경계 이탈:", find_out_of_bounds([b1, b2], trunk))
    print("새 후보 (0.4,0,0)에 같은 박스 놓기 유효한가:",
          is_candidate_valid(0.4, 0.0, 0.0, Box("C", 0.4, 0.3, 0.2), trunk, [b1, b2]))
