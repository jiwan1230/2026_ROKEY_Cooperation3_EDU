"""
08_unloadable_reason.py
⑧ 미적재 판단
===============
상태: 🟢 완료·확정

[비유] 이사 갈 때 가구를 못 넣는 상황 세 가지와 같다.
    SIZE_EXCEEDS_TRUNK            : 소파가 현관문보다 커서, 집이 비어있어도 절대 못 들어감
    INSUFFICIENT_REMAINING_VOLUME : 문은 통과하는데 집 안 남은 공간 자체가 이미 꽉 참
    NO_VALID_CANDIDATE_POSITION   : 남은 공간을 합치면 충분한데 가구 배치 모양 때문에 못 들어감
                                     (이 경우만 재배치하면 들어갈 수도 있음)

1번·2번은 재배치를 시도해봐야 소용없어서 바로 담당자 호출.
3번만 재배치(reshuffle) 시도 가치가 있음 (decide_reshuffle_or_call 참고).

이 파일 하단에는 ⑥⑦⑧을 하나로 묶은 통합 파이프라인
generate_loading_plan()도 함께 있음.
"""

import sys, pathlib
from dataclasses import dataclass
from enum import Enum
from typing import List, Tuple
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m03 = import_module("03_extreme_point_candidates")
_m06 = import_module("06_loading_order_decision")
_m07 = import_module("07_placement_plan")
_m18 = import_module("18_rotation")

Box = _m03.Box
ExtremePointState = _m03.ExtremePointState
fits_dims_any_rotation = _m18.fits_dims_any_rotation
decide_loading_order = _m06.decide_loading_order
place_one_box = _m07.place_one_box
PlacementPlan = _m07.PlacementPlan


class UnloadableReason(Enum):
    SIZE_EXCEEDS_TRUNK = "SIZE_EXCEEDS_TRUNK"
    INSUFFICIENT_REMAINING_VOLUME = "INSUFFICIENT_REMAINING_VOLUME"
    NO_VALID_CANDIDATE_POSITION = "NO_VALID_CANDIDATE_POSITION"


class LoadingAction(Enum):
    RESHUFFLE = "RESHUFFLE"
    CALL_OPERATOR = "CALL_OPERATOR"


@dataclass
class UnloadableItem:
    box_id: str
    reason: UnloadableReason
    detail: str


def _remaining_free_volume(trunk, state: "ExtremePointState") -> float:
    """단순 근사치: 트렁크 전체 부피 - 이미 놓인 박스 부피 합."""
    used = sum(p.box.volume for p in state.placed)
    return trunk.width * trunk.depth * trunk.height - used


def classify_unloadable_reason(box: "Box", trunk, state: "ExtremePointState") -> UnloadableReason:
    # 검사 순서가 중요: 셋 다 걸릴 수 있어도 "가장 근본적인 이유"부터 확인해서 반환한다.
    # 1순위: 박스 자체가 트렁크보다 큼 (자리 배치와 무관하게 애초에 불가능) - ⑱ 90도
    # 회전한 자세까지 감안해서, 둘 다 안 맞을 때만 진짜 SIZE_EXCEEDS_TRUNK로 본다.
    if not fits_dims_any_rotation(box, trunk):
        return UnloadableReason.SIZE_EXCEEDS_TRUNK

    # 2순위: 남은 부피 자체가 박스 부피보다 적음 (배치를 어떻게 하든 물리적으로 불가능)
    if _remaining_free_volume(trunk, state) < box.volume:
        return UnloadableReason.INSUFFICIENT_REMAINING_VOLUME

    # 여기까지 왔으면 이론상 공간은 충분한데 현재 배치 모양 때문에 못 놓는 것
    # -> 재배치(reshuffle)하면 들어갈 가능성이 있는 케이스
    return UnloadableReason.NO_VALID_CANDIDATE_POSITION


def decide_reshuffle_or_call(reason: UnloadableReason) -> LoadingAction:
    """
    사유 코드에 따라 재배치를 시도할지, 바로 담당자를 호출할지 판단.
    SIZE_EXCEEDS_TRUNK / INSUFFICIENT_REMAINING_VOLUME -> 재배치해도 소용없음 -> 바로 호출
    NO_VALID_CANDIDATE_POSITION -> 배치 모양 문제라 재배치 시도 가치 있음
    """
    if reason == UnloadableReason.NO_VALID_CANDIDATE_POSITION:
        return LoadingAction.RESHUFFLE
    return LoadingAction.CALL_OPERATOR


# ---------------------------------------------------------------------------
# ⑥⑦⑧ 통합 파이프라인
# ---------------------------------------------------------------------------

def generate_loading_plan(
    boxes: List["Box"], trunk
) -> Tuple[List["PlacementPlan"], List[UnloadableItem]]:
    """
    boxes, trunk를 받아서:
      1) [⑥] 부피 큰 순서로 시도 순서 고정
      2) [⑦] 순서대로 하나씩 Extreme Point 최적 자리 찾아 배치
      3) 자리를 못 찾으면 [⑧] 사유 코드 부여 (다음 박스는 계속 시도)
    """
    order = decide_loading_order(boxes)  # [⑥] 부피 큰 순서로 정렬된 시도 순서
    state = ExtremePointState()          # 빈 트렁크 상태(후보는 (0,0,0) 하나)에서 시작

    plans: List["PlacementPlan"] = []
    unloadable: List[UnloadableItem] = []
    order_counter = 1

    for box in order:
        # [⑦] 현재 state 기준으로 이 박스의 최적 자리를 찾아 배치 시도
        plan = place_one_box(box, trunk, state, order_counter)
        if plan is not None:
            plans.append(plan)
            order_counter += 1  # 실제로 배치된 것만 순번을 늘림
        else:
            # 자리를 못 찾음 -> [⑧] 왜 못 찾았는지 사유를 분류.
            # 이 박스만 건너뛰고 for문은 계속 돌아 다음 박스는 계속 시도한다 (전체 중단 X)
            reason = classify_unloadable_reason(box, trunk, state)
            unloadable.append(UnloadableItem(
                box_id=box.id, reason=reason,
                detail=f"{box.id}(부피 {box.volume*1000:.1f}L) - 사유: {reason.value}",
            ))

    return plans, unloadable


if __name__ == "__main__":
    _m02 = import_module("02_trunk_space_state")
    Trunk = _m02.Trunk

    boxes = [
        Box("Small", 0.30, 0.20, 0.15),
        Box("Medium", 0.40, 0.30, 0.25),
        Box("Large", 0.50, 0.35, 0.30),
    ]
    trunk = Trunk(width=0.57, depth=1.12, height=0.25)  # 실제 트렁크 실측값

    plans, unloadable = generate_loading_plan(boxes, trunk)
    for p in plans:
        print("PLACED:", p)
    for u in unloadable:
        print("UNLOADABLE:", u, "-> 판단:", decide_reshuffle_or_call(u.reason).value)
