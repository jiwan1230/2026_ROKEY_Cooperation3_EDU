"""
09_rescan_replan.py
⑨ 재스캔 후 재계획
=====================
상태: 🔴 보류 — 팀원 데이터 필요 (준형의 재스캔 결과 + 지완의 트리거 신호)

[핵심 아이디어 - 극점 알고리즘을 고른 이유가 여기서 드러남]
ExtremePointState는 "이미 놓인 박스 리스트 + 후보 좌표 집합"만 있으면
언제든 상태를 재구성할 수 있다. 그래서 재스캔 트리거가 오면, 새로 스캔된
점유 정보로 ExtremePointState를 다시 만들고 generate_loading_plan()을
그대로 다시 호출하면 된다 — 로직을 새로 짤 필요가 없다.

[막힌 지점]
    - "환경 변화 감지"를 언제 트리거로 볼지 (지완이 줄 신호 형식 미정)
    - 재스캔 결과가 이미 놓인 박스들의 위치를 어떤 형식으로 주는지 (준형 미정)
    - 박스 ID가 재스캔 전후로 유지되는지 (준형 확인 필요 - 같은 박스면 같은 id)
"""

import sys, pathlib
from typing import List
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m03 = import_module("03_extreme_point_candidates")
_m08 = import_module("08_unloadable_reason")

Box = _m03.Box
ExtremePointState = _m03.ExtremePointState
PlacedBox = _m03.PlacedBox
generate_loading_plan = _m08.generate_loading_plan


def rebuild_state_from_rescan(rescanned_placed_boxes: List["PlacedBox"]) -> "ExtremePointState":
    """
    준형의 재스캔 결과(이미 트렁크 안에 있는 박스들의 위치)로 ExtremePointState를
    다시 만든다. 후보 좌표는 각 박스를 register_placement()로 다시 등록하면서
    자동으로 재계산된다.

    ⚠️ TODO: rescanned_placed_boxes가 실제로 어떤 포맷으로 오는지는
    준형의 스캔 파이프라인 출력 형식이 확정돼야 함.
    """
    state = ExtremePointState()
    for pb in rescanned_placed_boxes:
        state.register_placement(pb)
    return state


def replan_after_rescan(remaining_boxes: List["Box"], trunk, rescanned_placed_boxes: List["PlacedBox"]):
    """
    재스캔 트리거가 오면 이렇게 재사용하면 됨.

    Args:
        remaining_boxes: 아직 적재 안 된 박스 리스트
        trunk: 트렁크 공간 (실측값)
        rescanned_placed_boxes: 재스캔으로 확인된, 이미 트렁크 안에 있는 박스들

    Returns:
        generate_loading_plan()과 동일한 (plans, unloadable) 튜플
        (단, 이미 놓인 박스들의 상태를 반영한 상태에서 이어서 계산)
    """
    # TODO: generate_loading_plan이 "빈 트렁크 기준"으로만 동작하므로,
    # rebuild_state_from_rescan()으로 만든 state를 재사용하도록
    # generate_loading_plan을 확장하거나 별도 버전을 만들어야 함.
    # 지금은 인터페이스만 준비해둔 상태.
    raise NotImplementedError(
        "재스캔 데이터 포맷 확정 후 구현 예정 "
        "(rebuild_state_from_rescan()은 이미 준비됨, generate_loading_plan 확장만 남음)"
    )


# TODO: 지완의 트리거 신호 연동 (아직 미구현)
# def on_rescan_trigger(signal, remaining_boxes, trunk):
#     """지완이 정의할 트리거 신호 형식이 오면, 여기서 replan_after_rescan()을 호출."""
#     raise NotImplementedError("트리거 신호 형식 확정 후 구현 예정")
