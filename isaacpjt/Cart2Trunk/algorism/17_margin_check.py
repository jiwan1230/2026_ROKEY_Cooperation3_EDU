"""
17_margin_check.py
⑰ 박스-벽 / 박스-박스 최소 간격(Margin) 확인
==============================================
상태: 🟢 완료 (7/22, main과 병합하며 마진 값 통일)

[배경]
지금까지는 박스를 벽이나 다른 박스에 딱(간격 0으로) 붙여서 배치했다. 실제
그리퍼로 놓고 빼는 동작을 생각하면, 완전히 맞닿은 자리는 살짝만 어긋나도
옆 박스나 벽에 긁힐 위험이 있다. 그래서 "박스끼리도, 박스-벽 사이도 아주
조금은 여유를 두자"는 요청으로 고정 마진을 하드 컷으로 추가한다.

[마진 값 = PLACEMENT_SAFETY_MARGIN_M]
처음엔 1cm로 시작했는데, 지완님이 실제 로봇으로 PLACE 하강을 반복 검증하며
1cm는 벽 코너(두 벽이 동시에 margin 딱 걸리는 자리)에서 실행 오차를 못
흡수해 박스가 튕겨나가는 걸 실측으로 확인하고 2cm로 올렸다 (근거는
03_extreme_point_candidates.py의 PLACEMENT_SAFETY_MARGIN_M 주석 참고). 이
파일의 하드 컷도 후보 생성(③) 쪽과 다른 값을 쓰면 ③이 만든 후보가 여기서
다시 걸러지는 모순이 생기므로, 별도 상수를 두지 않고 같은 값을 그대로
가져와 쓴다.

[이 파일이 하는 것 / 안 하는 것]
- 좌우(x/y, 트렁크 옆벽·안쪽벽) 방향으로만 마진을 확인한다.
- 바닥(z=0)은 대상이 아니다 - 박스는 바닥에 딱 붙어야(접촉해야) 정상이다.
- 천장 쪽 여유는 이미 ⑮(OVERHEAD_CLEARANCE=0.20m)가 훨씬 큰 마진으로 다루고
  있어서 여기서 따로 안 다룬다.
- 박스끼리도, z 범위가 실제로 겹치는(=옆으로 나란한) 경우에만 마진을 요구한다.
  z가 안 겹치는(한쪽이 다른 쪽 위에 쌓인) 관계는 오히려 완전히 맞닿아야
  (⑬ 받침) 하므로 마진 대상이 아니다.
"""

import sys, pathlib
from typing import List
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m03 = import_module("03_extreme_point_candidates")

Box = _m03.Box
PlacedBox = _m03.PlacedBox

# 그리퍼가 옆 박스/벽에 안 긁히도록 두는 최소 여유. ③의 PLACEMENT_SAFETY_MARGIN_M과
# 반드시 같은 값이어야 한다 - 후보 생성 쪽(③)이 이 값보다 좁게 여유를 두고 후보를
# 만들면, 그 후보가 여기(⑰ 하드 컷)에서 전부 다시 걸러져 자리가 있는데도 못 쓰게 된다.
MARGIN = _m03.PLACEMENT_SAFETY_MARGIN_M


def has_wall_margin(x: float, y: float, z: float, box: "Box", trunk, margin: float = MARGIN) -> bool:
    """트렁크 옆벽/안쪽벽까지 x·y 방향으로 margin 이상 떨어져 있는지 확인 (바닥/천장 제외)."""
    return (
        x >= margin - 1e-9
        and x + box.width <= trunk.width - margin + 1e-9
        and y >= margin - 1e-9
        and y + box.depth <= trunk.depth - margin + 1e-9
    )


def has_box_margin(x: float, y: float, z: float, box: "Box", placed: List["PlacedBox"], margin: float = MARGIN) -> bool:
    """
    z 범위가 실제로 겹치는(옆으로 나란히 놓이는) 다른 박스와 x 또는 y 방향으로
    margin 이상 떨어져 있는지 확인. 셋 중 하나라도 겹치는 게 있으면 그 박스와는
    x_gap, y_gap 둘 다 margin 미만이면 안 된다(대각선으로 너무 가까운 것도 거부).
    """
    x0, x1 = x, x + box.width
    y0, y1 = y, y + box.depth
    z0, z1 = z, z + box.height

    for p in placed:
        pz0, pz1 = p.z_range
        if not (z0 < pz1 - 1e-9 and z1 > pz0 + 1e-9):
            continue  # z가 안 겹침 - 쌓기 관계라 마진 대상 아님

        px0, px1 = p.x_range
        py0, py1 = p.y_range
        x_gap = max(px0 - x1, x0 - px1)
        y_gap = max(py0 - y1, y0 - py1)
        if x_gap < margin - 1e-9 and y_gap < margin - 1e-9:
            return False

    return True


def has_sufficient_margin(
    x: float, y: float, z: float, box: "Box", trunk, placed: List["PlacedBox"], margin: float = MARGIN
) -> bool:
    """벽 마진 + 박스 간 마진을 모두 확인 (하드 컷 - ⑬/⑮와 같은 원칙)."""
    return has_wall_margin(x, y, z, box, trunk, margin) and has_box_margin(x, y, z, box, placed, margin)


if __name__ == "__main__":
    _m02 = import_module("02_trunk_space_state")
    Trunk = _m02.Trunk

    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("demo", width=0.2, depth=0.2, height=0.2)
    print("벽에 딱 붙은(x=0) 자리:", has_wall_margin(0.0, 0.5, 0.0, box, trunk), "(기대: False)")
    print(f"벽에서 {MARGIN}m 뗀 자리:", has_wall_margin(MARGIN, 0.5, 0.0, box, trunk), "(기대: True)")
