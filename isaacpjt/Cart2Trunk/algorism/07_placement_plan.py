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
_m05 = import_module("05_candidate_scoring")
_m13 = import_module("13_support_check")

Box = _m03.Box
PlacedBox = _m03.PlacedBox
ExtremePointState = _m03.ExtremePointState
generate_wall_flush_candidates = _m03.generate_wall_flush_candidates
is_candidate_valid_with_stacking = _m13.is_candidate_valid_with_stacking
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


def place_one_box(
    box: "Box", trunk, state: "ExtremePointState", order: int,
    allow_stacking: bool = False,
) -> Optional["PlacementPlan"]:
    """
    현재 상태(state)에서 box 하나를 놓을 최선의 자리를 찾아 배치한다.
    자리가 없으면 None (이 경우 ⑧ 미적재 판단으로 넘어가야 함).

    allow_stacking=False(기본값)면 z>0 후보(박스 위에 놓는 자리)는 ⑬에서
    무조건 거부되어 지금의 1층 전용 동작과 동일하게 동작한다.
    """
    # [③ 보강] 순수 모서리 확장만으로는 못 만드는 "이 박스라면 벽에 딱 붙는 자리"를
    # 지금 놓으려는 box 크기 기준으로 추가 생성 - state.candidates에는 저장하지
    # 않고 이번 배치 판단에만 잠깐 섞어 쓴다 (다른 박스 크기에는 안 맞을 수 있어서)
    candidate_pool = state.candidates | generate_wall_flush_candidates(box, trunk, state.candidates)

    # [④+⑬] 후보 좌표들 중, 겹치지도 밖으로 나가지도 않고(④) 충분히
    # 받쳐지는(⑬, allow_stacking일 때만) 것만 추림
    valid_candidates = [
        (x, y, z) for (x, y, z) in candidate_pool
        if is_candidate_valid_with_stacking(x, y, z, box, trunk, state.placed, allow_stacking=allow_stacking)
    ]

    if not valid_candidates:
        return None  # 놓을 자리가 하나도 없음 -> 호출한 쪽(⑧)이 미적재 사유를 판단해야 함

    # [⑤] 유효한 후보 각각을 점수화 (score, touches)까지 같이 계산해서 보관
    scored = [
        (pos, *score_candidate(pos[0], pos[1], pos[2], box, trunk, state.placed))
        for pos in valid_candidates
    ]
    # 점수(t[1])가 가장 낮은(=가장 좋은) 후보 하나를 최종 선택
    best_pos, best_score, best_touches = min(scored, key=lambda t: t[1])

    # [③] 실제로 그 자리에 박스를 배치하고, 그 박스 기준 새 후보 3개를 상태에 등록
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
