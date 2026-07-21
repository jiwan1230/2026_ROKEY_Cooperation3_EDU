"""
06_loading_order_decision.py
⑥ 적재 순서 결정
==================
상태: 🟢 완료·확정 — "지금 보이는 것 중 최선" 전제 확인 완료 (준형 답변, 7/20)

[왜 부피 큰 순서만으로 충분한가 - 확인 완료]
카트에 박스가 쌓여있으면 큰 박스가 밑에 깔려있을 수도 있는데, "부피 큰
순서로 무조건 먼저"가 맞는 전략인지 의문이 있었음. 준형에게 확인한 결과:
Vision이 주는 인식 결과는 "카메라에 지금 실제로 보이는 박스만" - 즉 밑에
깔려서 안 보이는 박스는 애초에 이번 인식 목록에 안 들어옴.

그래서 이 함수가 하는 "부피 큰 순서로 정렬"은 사실 "지금 손댈 수 있는
후보들 중에서 큰 순서로"와 같은 뜻이 됨. 위에 있던 박스를 로봇이 치우면
⑨(재스캔 후 재계획)에서 새로 스캔해서 다음 인식 목록에 그 아래 있던
박스가 나타나고, 그때 다시 이 함수로 순서를 매기면 됨.

→ 결론: 코드 수정 불필요. "매 순간 보이는 것 중 최선을 고른다"는 이
로직 + ⑨의 재스캔 루프가 합쳐지면 "쌓인 순서" 문제가 자동으로 해결됨.
"""

import sys, pathlib
from typing import List
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m03 = import_module("03_extreme_point_candidates")
Box = _m03.Box


def decide_loading_order(boxes: List["Box"]) -> List["Box"]:
    """부피가 큰 순서로 정렬 (BR-09)."""
    return sorted(boxes, key=lambda b: b.volume, reverse=True)


if __name__ == "__main__":
    boxes = [
        Box("Small", 0.30, 0.20, 0.15, mass_kg=1.0),
        Box("Medium", 0.40, 0.30, 0.25, mass_kg=2.0),
        Box("Large", 0.50, 0.35, 0.30, mass_kg=3.5),
    ]
    order = decide_loading_order(boxes)
    for b in order:
        print(f"{b.id}: volume={b.volume*1000:.1f}L, mass={b.mass_kg}kg")
