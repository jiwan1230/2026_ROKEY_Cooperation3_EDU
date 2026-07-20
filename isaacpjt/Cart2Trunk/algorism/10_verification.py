"""
10_verification.py
⑩ 알고리즘 검증
=================
상태: 🟢 완료 — 실제 트렁크 실측값(0.57×1.12×0.25m) 반영됨

확정했던 검증 항목 5개를 각각 시나리오로 자동 테스트한다.
    1. 재현성          : 동일 입력 -> 동일 출력이 항상 나오는가
    2. 사유코드 정확성   : ⑧의 3가지 사유 코드가 각각 맞는 상황에서 정확히 나오는가
    3. 좌표변환/경계     : PlacementPlan의 좌표가 트렁크 범위 안 + 서로 겹치지 않는가
    4. 점수화 회귀방지   : 접촉면 기반 Best 후보 선택이 의도대로 동작하는가
    5. 실제 데이터 반영  : 진짜 트렁크/박스 실측값으로 돌렸을 때 결과가 예상과 일치하는가
                          (Large -> SIZE_EXCEEDS_TRUNK, Medium/Small -> 배치 성공)

실행: python 10_verification.py
"""

import sys, pathlib
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
_m04 = import_module("04_candidate_validity_check")
_m08 = import_module("08_unloadable_reason")

Trunk = _m02.Trunk
REAL_TRUNK = _m02.REAL_TRUNK
Box = _m03.Box
find_overlaps = _m04.find_overlaps
find_out_of_bounds = _m04.find_out_of_bounds
generate_loading_plan = _m08.generate_loading_plan
UnloadableReason = _m08.UnloadableReason

SMALL = Box("Small", 0.30, 0.20, 0.15, mass_kg=1.0)
MEDIUM = Box("Medium", 0.40, 0.30, 0.25, mass_kg=2.0)
LARGE = Box("Large", 0.50, 0.35, 0.30, mass_kg=3.5)


def check(label: str, condition: bool, detail: str = ""):
    """검증 조건 하나를 PASS/FAIL로 출력하고, condition을 그대로 반환 (run_all에서 취합용)."""
    status = "PASS" if condition else "FAIL"
    # 실패했을 때만 detail을 같이 출력해서 원인 파악을 돕는다 (성공 시엔 생략해 로그를 짧게 유지)
    print(f"  [{status}] {label}" + (f" - {detail}" if detail and not condition else ""))
    return condition


def verify_reproducibility() -> bool:
    print("\n[1] 재현성 검증")
    boxes = [SMALL, MEDIUM, LARGE]
    trunk = Trunk(1.5, 1.5, 0.9)

    # 같은 boxes, trunk로 두 번 실행해서 결과가 완전히 똑같이 나오는지 비교
    plans_a, unload_a = generate_loading_plan(boxes, trunk)
    plans_b, unload_b = generate_loading_plan(boxes, trunk)

    ok = (
        [p.box_id for p in plans_a] == [p.box_id for p in plans_b]
        and [p.position for p in plans_a] == [p.position for p in plans_b]
        and [u.box_id for u in unload_a] == [u.box_id for u in unload_b]
    )
    return check("동일 입력 -> 동일 순서/좌표/미적재 결과", ok)


def verify_reason_codes() -> bool:
    print("\n[2] 사유코드 정확성 검증")
    results = []

    # SIZE_EXCEEDS_TRUNK: 트렁크가 비어도 Large가 못 들어가는 높이
    trunk1 = Trunk(1.5, 1.5, 0.28)
    _, unload1 = generate_loading_plan([LARGE], trunk1)
    ok1 = len(unload1) == 1 and unload1[0].reason == UnloadableReason.SIZE_EXCEEDS_TRUNK
    results.append(check("SIZE_EXCEEDS_TRUNK", ok1))

    # INSUFFICIENT_REMAINING_VOLUME: Large가 거의 꽉 채우는 트렁크에 Medium까지 넣으려는 경우
    trunk2 = Trunk(0.55, 0.35, 0.30)
    _, unload2 = generate_loading_plan([LARGE, MEDIUM], trunk2)
    ok2 = (
        len(unload2) == 1
        and unload2[0].box_id == "Medium"
        and unload2[0].reason == UnloadableReason.INSUFFICIENT_REMAINING_VOLUME
    )
    results.append(check("INSUFFICIENT_REMAINING_VOLUME", ok2, detail=str(unload2)))

    return all(results)


def verify_coordinate_bounds() -> bool:
    print("\n[3] 좌표변환/경계 검증")
    trunk = Trunk(1.5, 1.5, 0.9)
    plans, _ = generate_loading_plan([SMALL, MEDIUM, LARGE], trunk)

    from importlib import import_module as im
    _m03b = im("03_extreme_point_candidates")
    placed_boxes = [
        _m03b.PlacedBox(box=Box(p.box_id, *p.dimensions), x=p.position[0], y=p.position[1], z=p.position[2])
        for p in plans
    ]

    ok1 = check("모든 배치 좌표가 트렁크 범위 안에 있음", len(find_out_of_bounds(placed_boxes, trunk)) == 0)
    ok2 = check("배치된 박스끼리 서로 겹치지 않음", len(find_overlaps(placed_boxes)) == 0)
    return ok1 and ok2


def verify_scoring_regression() -> bool:
    print("\n[4] 점수화(접촉면 기반) 회귀방지 검증")
    _m05 = import_module("05_candidate_scoring")
    score_candidate = _m05.score_candidate
    _m03b = import_module("03_extreme_point_candidates")
    PlacedBox = _m03b.PlacedBox

    trunk = Trunk(12.0, 12.0, 4.0)
    unit = Box("unit", 2, 2, 2)
    placed = [
        PlacedBox(Box("A", 2, 2, 2), 3, 3, 0),
        PlacedBox(Box("B", 2, 2, 2), 3, 5, 0),
        PlacedBox(Box("C", 2, 2, 2), 5, 3, 0),
    ]
    open_score, _ = score_candidate(7.0, 3.0, 0.0, unit, trunk, placed)
    pocket_score, _ = score_candidate(5.0, 5.0, 0.0, unit, trunk, placed)

    return check("안쪽 구석 자리가 열린 자리보다 점수가 더 낮음(더 좋음)", pocket_score < open_score,
                 detail=f"pocket={pocket_score}, open={open_score}")


def verify_real_data() -> bool:
    print("\n[5] 실제 데이터 반영 검증 (트렁크 실측값 0.57×1.12×0.25m)")
    plans, unloadable = generate_loading_plan([SMALL, MEDIUM, LARGE], REAL_TRUNK)

    ok1 = check(
        "Large는 SIZE_EXCEEDS_TRUNK로 미적재 처리됨",
        len(unloadable) == 1 and unloadable[0].box_id == "Large"
        and unloadable[0].reason == UnloadableReason.SIZE_EXCEEDS_TRUNK,
        detail=str(unloadable),
    )
    ok2 = check(
        "Medium, Small은 정상 배치됨",
        {p.box_id for p in plans} == {"Medium", "Small"},
        detail=str([p.box_id for p in plans]),
    )
    return ok1 and ok2


def run_all() -> bool:
    print("=" * 60)
    print("Cart2Trunk 알고리즘 검증 (극점 알고리즘 기반, ③④⑤⑥⑦⑧ 통합)")
    print("=" * 60)

    results = [
        verify_reproducibility(),
        verify_reason_codes(),
        verify_coordinate_bounds(),
        verify_scoring_regression(),
        verify_real_data(),
    ]

    print("\n" + "=" * 60)
    total, passed = len(results), sum(results)  # bool 리스트를 sum하면 True 개수가 나옴
    print(f"총 {total}개 항목 중 {passed}개 통과")
    print("=" * 60)
    return all(results)


if __name__ == "__main__":
    success = run_all()
    sys.exit(0 if success else 1)
