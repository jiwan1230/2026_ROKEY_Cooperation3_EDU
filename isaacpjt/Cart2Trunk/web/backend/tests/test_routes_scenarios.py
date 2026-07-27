import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from app import create_app


def _client():
    return create_app().test_client()


def test_delivery_truck_places_later_stops_farther_from_entrance():
    # LIFO 정책 - 나중 배송지(delivery_stop 숫자가 큰) 박스가 입구에서 더
    # 먼(x가 큰) 자리에 있어야 한다(실측: 1/2번은 x=0.56, 3/4번은 x=0.88).
    resp = _client().post("/api/scenarios/delivery_truck/plan")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["summary"]["total"] == 4
    # Before(카트 대기) 화면을 그리려면 배치 성공 여부와 무관하게 박스
    # 전체의 크기 정보(boxes)가 필요하다.
    assert body["boxes"] == [
        {"id": "정류장1_박스", "width": 0.3, "depth": 0.25, "height": 0.2},
        {"id": "정류장2_박스", "width": 0.3, "depth": 0.25, "height": 0.2},
        {"id": "정류장3_박스", "width": 0.3, "depth": 0.25, "height": 0.2},
        {"id": "정류장4_박스", "width": 0.3, "depth": 0.25, "height": 0.2},
    ]
    by_id = {p["box_id"]: p for p in body["placed"]}
    assert by_id["정류장4_박스"]["position"][0] > by_id["정류장1_박스"]["position"][0]


def test_warehouse_places_7_of_8_boxes_with_tight_margin():
    # 공간활용 우선순위 - 마진을 1cm로 타이트하게 줘서(기본 2cm보다 좁음)
    # count_first 모드만 쓸 때(6/8, margin 기본값)보다 더 많이 들어간다(실측 7/8).
    resp = _client().post("/api/scenarios/warehouse/plan")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["summary"] == {"total": 8, "placed": 7, "unplaced": 1}


def test_cold_chain_places_all_3_boxes_with_wide_margin():
    resp = _client().post("/api/scenarios/cold_chain/plan")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["summary"] == {"total": 3, "placed": 3, "unplaced": 0}
    # 냉기 순환 우선순위 - x축으로 나란히 놓인 박스끼리 기본 마진(2cm)보다
    # 넓은(5cm) 간격이 있어야 한다.
    by_x = sorted(body["placed"], key=lambda p: p["position"][0])
    gap = by_x[1]["position"][0] - (by_x[0]["position"][0] + by_x[0]["dimensions"][0])
    assert gap >= 0.05 - 1e-9


def test_hazmat_places_oxidizer_and_flammable_apart():
    resp = _client().post("/api/scenarios/hazmat/plan")
    assert resp.status_code == 200
    body = resp.get_json()
    ids = {p["box_id"] for p in body["placed"]}
    assert {"산화제_드럼1", "인화물_드럼1", "일반박스1"} == ids


def test_unknown_scenario_returns_404():
    resp = _client().post("/api/scenarios/nonexistent/plan")
    assert resp.status_code == 404
    assert resp.get_json()["error_code"] == "SCENARIO_NOT_FOUND"


def test_randomize_true_generates_a_different_valid_box_set_each_time():
    # "무작위로 다시 생성" 버튼용 - 고정 데모 대신 그 시나리오 성격(hazard_class
    # 등)에 맞는 무작위 박스 세트가 나와야 하고, 트렁크/알고리즘 성격은 그대로
    # 유지돼야 한다(delivery_stop이 순차적으로 매겨짐).
    client = _client()
    resp = client.post("/api/scenarios/delivery_truck/plan", json={"randomize": True})
    assert resp.status_code == 200
    body = resp.get_json()
    assert 3 <= body["summary"]["total"] <= 6
    assert body["trunk"] == {"width": 1.2, "depth": 0.8, "height": 0.6, "entrance_near_x": True}

    resp2 = client.post("/api/scenarios/hazmat/plan", json={"randomize": True})
    assert resp2.status_code == 200
    body2 = resp2.get_json()
    box_ids = {b["id"] for b in body2["boxes"]}
    assert any(bid.startswith("산화제") for bid in box_ids)
    assert any(bid.startswith("인화물") for bid in box_ids)
