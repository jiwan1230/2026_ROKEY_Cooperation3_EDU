"""
routes/scenarios.py
POST /api/scenarios/<scenario_id>/plan - 산업 현장 시나리오 4종
(algorism/industry_scenarios/) 미리보기. 각 시나리오 파일에 이미 있는
데모 트렁크/박스 데이터로 실제 시나리오 알고리즘을 호출해서 배치 결과를
반환한다. algorism/ 파일은 이 프로젝트에서 수정 금지라 기존 함수를
import해서 호출만 한다(algorism_bridge.py와 동일한 원칙).
"""
import sys
import pathlib
from importlib import import_module

from flask import Blueprint, jsonify

from routes.plan import ApiError

_HERE = pathlib.Path(__file__).resolve().parent  # .../web/backend/routes
_CART2TRUNK_DIR = _HERE.parent.parent.parent  # routes -> backend -> web -> Cart2Trunk
_ALGORISM_DIR = _CART2TRUNK_DIR / "algorism"
_SCENARIOS_DIR = _ALGORISM_DIR / "industry_scenarios"
for _p in (str(_ALGORISM_DIR), str(_SCENARIOS_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

Trunk = import_module("02_trunk_space_state").Trunk
Box = import_module("03_extreme_point_candidates").Box
ExtremePointState = import_module("03_extreme_point_candidates").ExtremePointState
generate_loading_plan = import_module("08_unloadable_reason").generate_loading_plan
classify_unloadable_reason = import_module("08_unloadable_reason").classify_unloadable_reason
UnloadableItem = import_module("08_unloadable_reason").UnloadableItem
decide_loading_order = import_module("06_loading_order_decision").decide_loading_order
make_weighted_score_fn = import_module("05_candidate_scoring").make_weighted_score_fn
place_one_box = import_module("07_placement_plan").place_one_box
# 위험물 창고는 범용 함수에 없는 하드 컷(비호환 물질 안전거리)이 필요해서
# 전용 안전거리 함수(has_hazmat_clearance)만 재사용하고, 우선순위(contact_
# preference)를 같이 반영할 수 있게 place_one_box 호출 루프는 여기서 직접
# 짠다 - generate_loading_plan_hazmat() 자체는 우선순위 파라미터를 안 받아서
# (scenario4_hazmat.py를 수정하지 않는 한) 그대로는 못 씀. 로직은 그 함수
# 내부와 완전히 동일하고 score_fn만 추가했다.
_hazmat_module = import_module("scenario4_hazmat")
has_hazmat_clearance = _hazmat_module.has_hazmat_clearance


def _lifo_delivery_order(boxes):
    # scenario1_delivery_truck.decide_loading_order_lifo_delivery()와 동일한
    # 정렬(나중 배송지부터) - algorism/ 파일을 수정하지 않고, 범용
    # generate_loading_plan()의 fixed_order로 같은 결과를 재현한다.
    ordered = sorted(boxes, key=lambda b: (b.delivery_stop is None, -(b.delivery_stop or 0)))
    return [b.id for b in ordered]


def _generate_hazmat_weighted(boxes, trunk, contact_preference=1.0):
    # scenario4_hazmat.generate_loading_plan_hazmat()과 동일한 루프(순서 결정
    # + place_one_box + 미적재 분류)에 우선순위 score_fn만 추가한 버전.
    order = decide_loading_order(boxes)
    score_fn = make_weighted_score_fn(contact_preference=contact_preference)
    state = ExtremePointState()
    plans, unloadable = [], []
    order_counter = 1
    for box in order:
        plan = place_one_box(box, trunk, state, order_counter, score_fn=score_fn, extra_validity_fn=has_hazmat_clearance)
        if plan is not None:
            plans.append(plan)
            order_counter += 1
        else:
            reason = classify_unloadable_reason(box, trunk, state)
            unloadable.append(UnloadableItem(box_id=box.id, reason=reason, detail=f"{box.id} - 사유: {reason.value}"))
    return plans, unloadable


scenarios_bp = Blueprint("scenarios", __name__)

# 각 시나리오의 "우선순위"(ControlPanel의 입구↔깊은위치/공간활용↔안정성/
# 마진 등 HMI 파라미터)를 슬라이더가 아니라 그 현장에 맞는 고정값으로 미리
# 박아둔다(사용자 요청 - 조정 가능하게 만들 필요 없이 시나리오마다 그냥
# 고정).
SCENARIO_DEFS = {
    "delivery_truck": {
        "label": "택배 배송 트럭",
        "trunk_kwargs": {"width": 1.2, "depth": 0.8, "height": 0.6},
        "make_boxes": lambda: [
            Box("정류장1_박스", 0.3, 0.25, 0.2, delivery_stop=1),
            Box("정류장2_박스", 0.3, 0.25, 0.2, delivery_stop=2),
            Box("정류장3_박스", 0.3, 0.25, 0.2, delivery_stop=3),
            Box("정류장4_박스", 0.3, 0.25, 0.2, delivery_stop=4),
        ],
        # 나중 배송지 박스부터 고정 순서로 실어서 LIFO(문 열자마자 첫
        # 배송지가 바로 손에 닿음)를 재현. + 운행 중 흔들림에 대비해 접촉면
        # (안정성) 우선순위를 최대(2.0)로 - 트럭은 도로를 달리므로 박스가
        # 최대한 서로/벽에 맞닿아 흔들리지 않아야 한다.
        "generate": lambda boxes, trunk: generate_loading_plan(
            boxes, trunk, fixed_order=_lifo_delivery_order(boxes), contact_preference=2.0,
        ),
    },
    "warehouse": {
        "label": "창고/물류센터",
        "trunk_kwargs": {"width": 0.6, "depth": 0.4, "height": 0.45},
        "make_boxes": lambda: (
            [Box(f"소{i}", 0.1, 0.1, 0.1) for i in range(6)]
            + [Box(f"대{i}", 0.3, 0.2, 0.2) for i in range(2)]
        ),
        # 공간활용 최대화 - count_first 모드(개수 우선) + 마진을 기본(2cm)보다
        # 타이트하게(1cm) 줘서 최대한 빽빽하게 채운다. 우선순위(entrance/
        # contact/height preference)는 일부러 기본값 그대로 둔다 -
        # generate_loading_plan()은 이 값들 중 하나라도 기본값이 아니면
        # count_first 전용 밀도 점수(score_count_first) 대신 범용 가중치
        # 점수로 바뀌어버려서(08_unloadable_reason.py 참고), 오히려 이
        # 시나리오의 핵심인 "최대 개수 적재"가 손해를 본다 - 마진/모드가
        # 이 시나리오의 진짜 손잡이다.
        "generate": lambda boxes, trunk: generate_loading_plan(boxes, trunk, mode="count_first", margin=0.01),
    },
    "cold_chain": {
        "label": "냉동/냉장 물류",
        "trunk_kwargs": {"width": 1.2, "depth": 0.6, "height": 0.5},
        "make_boxes": lambda: [Box(f"냉동박스{i}", 0.3, 0.25, 0.2) for i in range(3)],
        # 냉기 순환 - 마진을 기본(2cm)보다 훨씬 크게(5cm, 팀이 실측 검증한
        # 냉동/냉장 컨테이너 기준값) 줘서 박스 사이 공기 흐름을 확보한다.
        # + 유통기한 회전율 관리를 위해 입구 쪽을 살짝 선호(entrance_
        # preference를 음수 쪽으로).
        "generate": lambda boxes, trunk: generate_loading_plan(boxes, trunk, margin=0.05, entrance_preference=-0.3),
    },
    "hazmat": {
        "label": "위험물 창고",
        "trunk_kwargs": {"width": 1.5, "depth": 1.0, "height": 0.5},
        "make_boxes": lambda: [
            Box("산화제_드럼1", 0.3, 0.3, 0.3, hazard_class="oxidizer"),
            Box("인화물_드럼1", 0.3, 0.3, 0.3, hazard_class="flammable"),
            Box("일반박스1", 0.3, 0.3, 0.3),
        ],
        # 안전거리 하드컷(핵심 우선순위) + 드럼통이 넘어지지 않도록 접촉면
        # (안정성) 우선순위도 높게(1.8).
        "generate": lambda boxes, trunk: _generate_hazmat_weighted(boxes, trunk, contact_preference=1.8),
    },
}


@scenarios_bp.post("/api/scenarios/<scenario_id>/plan")
def compute_scenario_plan(scenario_id):
    scenario_def = SCENARIO_DEFS.get(scenario_id)
    if scenario_def is None:
        raise ApiError(
            404, "SCENARIO_NOT_FOUND", f"'{scenario_id}' 시나리오가 없습니다.",
            "지원하는 시나리오 id(delivery_truck/warehouse/cold_chain/hazmat)인지 확인하세요.",
        )

    trunk = Trunk(**scenario_def["trunk_kwargs"])
    boxes = scenario_def["make_boxes"]()
    plans, unloadable = scenario_def["generate"](boxes, trunk)

    total = len(boxes)
    return jsonify({
        "label": scenario_def["label"],
        "trunk": {
            "width": trunk.width, "depth": trunk.depth, "height": trunk.height,
            "entrance_near_x": trunk.entrance_near_x,
        },
        # 프론트가 Before(카트에 대기 중) 화면을 그리려면 배치 성공/실패와
        # 무관하게 "원래 실으려던 박스 전체"의 크기가 필요하다 - 시뮬레이터
        # 탭의 inputBoxes(state.boxesText)와 같은 shape({id,width,depth,height}).
        "boxes": [
            {"id": b.id, "width": b.width, "depth": b.depth, "height": b.height} for b in boxes
        ],
        "placed": [
            {"box_id": p.box_id, "position": list(p.position), "dimensions": list(p.dimensions), "order": p.order}
            for p in plans
        ],
        "unloadable": [{"box_id": u.box_id, "reason": u.reason.value} for u in unloadable],
        "summary": {"total": total, "placed": len(plans), "unplaced": total - len(plans)},
    })
