"""
06_loading_order_decision.py
⑥ 적재 순서 결정
==================
상태: 🟢 완료·확정

박스가 여러 개일 때 어떤 순서로 트렁크에 넣을지 정한다.
BR-09(큰 물품·안정적인 정형 물품을 우선 하단에 배치)를 따라
부피가 큰 박스부터 순서를 정한다.

이 순서는 "고정된 시도 순서"다. 순서대로 하나씩 배치를 시도하고,
자리를 못 찾은 박스는 ⑧(미적재 판단)로 넘어가되, 그 다음 순서의
박스는 계속 시도한다 (한 박스가 실패했다고 전체를 멈추지 않음).
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
