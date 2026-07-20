"""
09_rescan_replan.py
⑨ 재스캔 후 재계획
=====================
상태: 🟡 트리거 정책 확정, ID 추적 이슈로 구현 보류 (7/20)

[지완 답변 - 트리거 정책 확정]
박스를 하나 놓을 때마다("PER_PLACEMENT") 무조건 재스캔해서 트렁크 공간을
갱신하고 다음 작업을 수행. 조건부(위치 변화 임계값 등) 트리거가 아니라
고정 주기 트리거로 확정됨.

[준형 답변 - 새로 발견된 문제: 박스 ID가 재스캔 전후로 안 유지됨]
YOLO 검출만으로는 같은 박스가 재스캔 후에도 같은 id를 유지한다는 보장이
없음. 지속 ID가 필요하면 이전 Object3D 목록과 새 검출 결과를 매칭하는
별도의 객체 추적/상태 관리 컴포넌트가 있어야 함 (준형 제안: Object3D에
object_id 필드 + 중앙 관리 노드).

⚠️ 이게 왜 문제냐면: 극점 알고리즘(③)은 "이미 놓인 박스 리스트"를 다시
만들 때 각 PlacedBox가 어떤 박스(Box)에 대응하는지 id로 식별한다. id가
재스캔마다 바뀌면 "이 박스는 이미 처리했다"를 추적할 수 없어서 중복
배치를 시도하거나, 이미 처리한 박스를 놓쳐서 재작업하는 문제가 생길 수
있다. 즉 ⑨는 트리거 시점은 정해졌지만, "재스캔 결과를 이전 상태와 어떻게
매칭할지"가 막혀서 아직 실제 구현은 못 함.

[막힌 지점]
    - 객체 추적/ID 유지 로직을 누가 만들지 아직 팀 차원에서 미정
      (준형 쪽 Vision 파이프라인? 지완 쪽 중앙 관리 노드?)
    - 그게 정해지기 전까지는 rebuild_state_from_rescan()에 "id가 유지된다"는
      전제를 그대로 둘 수 없음 - 최소한 위치 기반 매칭이라도 임시로 넣을지
      팀 논의 필요.
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

RESCAN_TRIGGER_POLICY = "PER_PLACEMENT"  # 지완 답변 확정: 박스 1개 놓을 때마다 1회


def rebuild_state_from_rescan(rescanned_placed_boxes: List["PlacedBox"]) -> "ExtremePointState":
    """
    재스캔 결과(이미 트렁크 안에 있는 박스들의 위치)로 ExtremePointState를
    다시 만든다. 후보 좌표는 각 박스를 register_placement()로 다시 등록하면서
    자동으로 재계산된다.

    ⚠️ 전제 조건 (아직 불확실): rescanned_placed_boxes 안의 각 박스 id가
    이전 스캔과 동일하게 유지된다고 가정하고 있음. 준형 답변에 따르면
    이 전제가 지금은 보장 안 됨 - 객체 추적 컴포넌트가 붙기 전까지는
    이 함수를 "재스캔마다 트렁크를 처음부터 완전히 새로 인식한다"는
    보수적인 가정 하에만 안전하게 쓸 수 있음 (이미 처리한 박스 구분 불가).
    """
    state = ExtremePointState()
    for pb in rescanned_placed_boxes:
        state.register_placement(pb)
    return state


def replan_after_rescan(remaining_boxes: List["Box"], trunk, rescanned_placed_boxes: List["PlacedBox"]):
    """
    재스캔 트리거(PER_PLACEMENT)가 발생할 때마다 호출될 함수.

    ⚠️ TODO: 객체 추적/ID 유지 방식이 팀 차원에서 정해지기 전까지는
    remaining_boxes를 "이미 처리된 박스를 제외한 나머지"로 안전하게
    걸러낼 방법이 없어서 미구현 상태로 남겨둠.
    """
    raise NotImplementedError(
        "박스 ID 추적 방식(준형 Vision 파이프라인 또는 지완 중앙 관리 노드) "
        "확정 후 구현 예정. rebuild_state_from_rescan()은 이미 준비됨."
    )


# TODO: 트리거 신호 연동 (트리거 정책은 확정, 신호 자체 연동은 미구현)
# def on_rescan_trigger(remaining_boxes, trunk):
#     """
#     박스 1개 배치 완료 시점마다 지완 쪽에서 신호를 주면 호출.
#     신호의 정확한 전달 방식(콜백? 토픽? 폴링?)은 아직 미정.
#     """
#     raise NotImplementedError("신호 전달 방식 확정 후 구현 예정")