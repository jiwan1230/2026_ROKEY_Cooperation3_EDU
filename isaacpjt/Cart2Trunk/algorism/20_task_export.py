"""
20_task_export.py
⑳ MSI2로 넘길 최종 Task JSON 생성
===================================
상태: 🟢 완료 - 스키마를 algorism/19_run_full_pipeline_with_yaw.py ->
isaac_task_runner.py(100.cart_to_trunk_dual_side_holonomic.py 포팅판)가 실제
2026-07-27 4박스 pick&place 전 과정에서 검증한 placement_result.json 스키마에
맞췄다. "Cart2Trunk 최종 프로젝트 시나리오 및 시스템 흐름" 문서 4.3절이
정의한 tasks/target_position/target_yaw 스키마는 실전 검증이 안 된 상태였고
(지완 확인 결과), MSI2가 실제로 읽는 필드(특히 dimensions - 없으면
box_needs_tilt/접근 전략 계산이 아예 불가능함)가 빠져 있어서 폐기했다.

    {"plan_id":..., "box_snapshot_id":..., "trunk_map_id":..., "approved":...,
     "parameters": {...},
     "placements": [{"order":..., "box_id":..., "position_base_frame":[x,y,z],
                      "dimensions":[w,d,h], "rotated":..., "source_yaw_deg":...,
                      "wrist_yaw_deg":..., "score":...}]}

[좌표 변환] PlacementPlan.position은 트렁크 로컬 좌표(0,0,0 코너 기준)라,
19번과 동일하게 02_trunk_space_state.local_to_base_frame(offset)으로
m0609_base_link 좌표로 되돌린 뒤 position_base_frame에 담는다. offset은
02_trunk_space_state.TrunkWorldMap.to_bounding_trunk()이 trunk_map을 파싱할
때 나오는 값 - 호출부(web/backend/algorism_bridge.py)가 POST /api/plan 응답의
trunk_offset_base_frame을 그대로 다시 넘겨줘야 한다.

[source_yaw_deg/wrist_yaw_deg] PlacementPlan.target_yaw는 쓰지 않는다 -
07_placement_plan.py의 `target_yaw = box.initial_yaw + (90도 if rotated else 0)`는
"목표 절대각 - source_yaw_deg"(19번의 검증된 공식, 사각형 180도 대칭 최단
회전 정규화)와 다른 값이라, 그대로 wrist_yaw로 쓰면 틀린 손목 회전이 나간다.
대신 그 식을 역산해서(덧셈이라 오차 없이 정확히 복원됨) initial_yaw =
target_yaw - (90도 if rotated else 0)을 얻고, 거기에 19번과 완전히 동일한
공식을 적용한다. (참고: 오늘(2026-07-28) 검증 결과 100.py/isaac_task_runner.py
자체는 아직 이 두 필드를 읽지 않는다 - 그래도 스키마 일관성과 향후
46.crate_pick_to_place_with_yaw.py류 연동을 위해 정확하게 계산해서 내보낸다.)

approved는 이 함수를 부르는 쪽(GUI 승인 워크플로우)이 결정해서 넘겨준다 -
여기서는 그 값을 그대로 데이터에 반영할 뿐, "승인 안 됐으면 아예 호출하지
마라"같은 강제는 하지 않는다(그건 "승인 전엔 MSI2로 전달 안 함" 원칙을
지키는 호출부의 책임 - trunk_map_planner_node._send_task_to_msi2 참고).
"""

import math
import sys, pathlib
from typing import Dict, List, Tuple
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m02 = import_module("02_trunk_space_state")
_m07 = import_module("07_placement_plan")

local_to_base_frame = _m02.local_to_base_frame
PlacementPlan = _m07.PlacementPlan


def build_task_json(
    plan_id: str, box_snapshot_id: str, trunk_map_id: str,
    parameters: Dict, plans: List["PlacementPlan"],
    offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    approved: bool = False,
) -> dict:
    placements = []
    for p in plans:
        bx, by, bz = local_to_base_frame(*p.position, offset)

        # target_yaw = initial_yaw + (pi/2 if rotated else 0) (07_placement_plan.py,
        # 순수 덧셈이라 역산 정확함) -> initial_yaw(=source_yaw, 라디안) 복원.
        initial_yaw_rad = p.target_yaw - (math.pi / 2.0 if p.rotated else 0.0)
        source_yaw_deg = math.degrees(initial_yaw_rad) % 180.0
        # 19_run_full_pipeline_with_yaw.py와 완전히 동일한 공식 - 사각형 180도
        # 대칭을 이용한 최단 회전으로 정규화([-90,90] 범위).
        target_deg = 90.0 if p.rotated else 0.0
        wrist_yaw_deg = ((target_deg - source_yaw_deg + 90.0) % 180.0) - 90.0

        placements.append({
            "order": p.order,
            "box_id": p.box_id,
            "position_base_frame": [bx, by, bz],
            "dimensions": list(p.dimensions),
            "rotated": p.rotated,
            "source_yaw_deg": source_yaw_deg,
            "wrist_yaw_deg": wrist_yaw_deg,
            "score": p.score,
            "touches": p.touches,
        })

    return {
        "plan_id": plan_id,
        "box_snapshot_id": box_snapshot_id,
        "trunk_map_id": trunk_map_id,
        "approved": approved,
        "parameters": dict(parameters),
        "placements": placements,
    }


if __name__ == "__main__":
    plan = PlacementPlan(
        box_id="BOX_03", order=1, position=(0.1, 0.2, 0.0),
        dimensions=(0.3, 0.2, 0.15), score=0.92, touches=2,
        rotated=True, target_yaw=1.57,
    )
    task = build_task_json(
        plan_id="load_plan_001", box_snapshot_id="box_scan_001", trunk_map_id="trunk_map_001",
        parameters={"box_margin": 0.02}, plans=[plan], offset=(2.5, -0.3, 0.4), approved=True,
    )
    import json
    print(json.dumps(task, ensure_ascii=False, indent=2))
