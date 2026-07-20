"""
07_placement_plan.py
⑦ 적재 위치 결정 (PlacementPlan)
==================================
상태: 🟢 완료·확정

③(후보 생성) + ④(유효성 검사) + ⑤(점수화)를 하나로 묶어서,
박스 하나를 실제로 어디에 놓을지 결정하고 PlacementPlan을 만든다.

    1. 현재 후보 좌표 집합에서 [④] 유효한 후보만 추림
    2. 유효한 후보들을 [⑤] 점수화해서 가장 낮은 점수(가장 좋은 자리) 선택
    3. 그 좌표에 배치하고 [③] 새 후보 3개 추가
"""

import sys, pathlib
from dataclasses import dataclass
from typing import List, Optional, Tuple
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m03 = import_module("03_extreme_point_candidates")
_m04 = import_module("04_candidate_validity_check")
_m05 = import_module("05_candidate_scoring")

Box = _m03.Box
PlacedBox = _m03.PlacedBox
ExtremePointState = _m03.ExtremePointState
is_candidate_valid = _m04.is_candidate_valid
score_candidate = _m05.score_candidate


@dataclass
class PlacementPlan:
    """협동로봇 Pick & Place 단계(BR-10)에 그대로 전달 가능한 정식 결과 구조."""
    box_id: str
    order: int
    position: Tuple[float, float, float]      # 트렁크 로컬 좌표 (x, y, z)
    dimensions: Tuple[float, float, float]     # (width, depth, height)
    score: float
    touches: int


def place_one_box(box: "Box", trunk, state: "ExtremePointState", order: int) -> Optional["PlacementPlan"]:
    """
    현재 상태(state)에서 box 하나를 놓을 최선의 자리를 찾아 배치한다.
    자리가 없으면 None (이 경우 ⑧ 미적재 판단으로 넘어가야 함).
    """
    valid_candidates = [
        (x, y, z) for (x, y, z) in state.candidates
        if is_candidate_valid(x, y, z, box, trunk, state.placed)
    ]

    if not valid_candidates:
        return None

    scored = [
        (pos, *score_candidate(pos[0], pos[1], pos[2], box, trunk, state.placed))
        for pos in valid_candidates
    ]
    best_pos, best_score, best_touches = min(scored, key=lambda t: t[1])

    placed_box = PlacedBox(box=box, x=best_pos[0], y=best_pos[1], z=best_pos[2])
    state.register_placement(placed_box)

    return PlacementPlan(
        box_id=box.id,
        order=order,
        position=best_pos,
        dimensions=(box.width, box.depth, box.height),
        score=best_score,
        touches=best_touches,
    )


if __name__ == "__main__":
    _m02 = import_module("02_trunk_space_state")
    Trunk = _m02.Trunk

    trunk = Trunk(width=1.5, depth=1.5, height=0.9)
    state = ExtremePointState()
    box = Box("Medium", 0.40, 0.30, 0.25)

    plan = place_one_box(box, trunk, state, order=1)
    print(plan)
