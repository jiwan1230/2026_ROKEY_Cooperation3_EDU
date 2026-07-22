"""
06_loading_order_decision.py
⑥ 적재 순서 결정
==================
상태: 🟢 완료·확정 (7/20 부피순 결론) → 🔄 픽업 순서 제약 반영 (7/22, 아래 참고)

[7/20 결론이 왜 틀렸나 - 7/22 재검토]
7/20엔 "Vision은 카메라에 지금 실제로 보이는 박스만 준다 - 밑에 깔린 건 애초에
인식 목록에 안 들어오니, 위 박스 치우면 재스캔으로 자동 해결된다"고 결론 냈었음
(코드 수정 불필요 판단). 근데 실제 비전 데이터(all_boxes_corners_*.json)를 열어
보니, `support_type`("floor"/"box_top") 필드가 있는 걸 보면 **깔려있는 박스도
"뭐 위에 얹혀있는지" 관계 정보와 함께 한 번에 같이 인식**하고 있었음 - 비전이
그 사이에 발전해서 7/20 전제 자체가 더 이상 안 맞음.

실제로 이 문제가 손그림 테스트에서 드러남: 카트에 초록 박스(바닥) 위에 파란
박스 2개가 얹혀 있는 상태를 그대로 하나의 박스 리스트로 넘겼더니, 부피 큰
순서로만 정렬하다 보니 부피가 제일 큰 초록 박스(맨 아래, 실제로는 제일 나중에
집어야 함)가 1번으로 나옴 - 물리적으로 불가능한 순서.

[수정 방식]
`Box.rests_on_id`(③에서 추가)로 "이 박스가 다른 어떤 박스 위에 얹혀 있는지"를
받는다. 매 단계, 아직 안 집힌 박스들 중 **지금 위에 아무것도 안 얹혀 있는(=
픽업 가능한) 것들만** 후보로 놓고, 그중에서 부피가 큰 걸 고른다 - 부피 우선
원칙(BR-09)은 그대로 유지하되, "물리적으로 불가능한 순서"는 애초에 후보에서
제외한다. `rests_on_id`가 전부 None이면 지금까지와 동일하게 순수 부피순으로
동작한다 (하위 호환).
"""

import sys, pathlib
from typing import List
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m03 = import_module("03_extreme_point_candidates")
Box = _m03.Box


def decide_loading_order(boxes: List["Box"]) -> List["Box"]:
    """
    "지금 위에 아무것도 안 얹혀서 픽업 가능한 박스들 중 부피가 큰 것부터" 순서로
    정렬한다 (BR-09 부피 우선 + 픽업 순서 제약). rests_on_id가 전부 없으면
    순수 부피 내림차순과 동일하게 동작한다.
    """
    remaining = {b.id: b for b in boxes}
    order: List["Box"] = []

    while remaining:
        # 아직 안 집힌 박스들 중, 그 위에 얹힌 게 하나도 안 남아있는 것만 픽업 가능
        blocked_ids = {b.rests_on_id for b in remaining.values() if b.rests_on_id is not None}
        available = [b for b in remaining.values() if b.id not in blocked_ids]

        next_box = max(available, key=lambda b: b.volume)
        order.append(next_box)
        del remaining[next_box.id]

    return order


if __name__ == "__main__":
    boxes = [
        Box("Small", 0.30, 0.20, 0.15, mass_kg=1.0),
        Box("Medium", 0.40, 0.30, 0.25, mass_kg=2.0),
        Box("Large", 0.50, 0.35, 0.30, mass_kg=3.5),
    ]
    order = decide_loading_order(boxes)
    for b in order:
        print(f"{b.id}: volume={b.volume*1000:.1f}L, mass={b.mass_kg}kg")

    print("\n--- 픽업 순서 제약 데모: 카트에 초록 박스 위 파란 박스 2개가 얹혀있는 경우 ---")
    stacked = [
        Box("Cart_Green", 0.20, 0.18, 0.15),
        Box("Cart_Blue1", 0.09, 0.10, 0.12, rests_on_id="Cart_Green"),
        Box("Cart_Blue2", 0.12, 0.14, 0.12, rests_on_id="Cart_Green"),
    ]
    for b in decide_loading_order(stacked):
        print(f"{b.id}: volume={b.volume*1000:.2f}L" + (f"  (rests_on={b.rests_on_id})" if b.rests_on_id else ""))
