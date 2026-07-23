"""
test_08_generate_loading_plan_count_first_best_of.py
버그 리포트: 실사용 중 "count_first(개수 우선)" 모드가 "large_first(큰 거 우선)"
모드보다 오히려 *더 적은* 박스를 담는 역설적인 사례가 나왔다 (10개 중 large_first는
9~10개, count_first는 6~7개). 원인: count_first는 항상 "작은 것부터" 순서로
담는데, 박스 크기가 고르게 섞여 있으면 작은 박스들이 공간을 잘게 조각내서
맨 뒤로 밀린 큰 박스들이 들어갈 자리를 잃는다 (NO_VALID_CANDIDATE_POSITION).

수정: count_first 모드는 이제 "작은 것부터+공간재사용 점수"와
"큰 것부터+기본 점수(=large_first와 동일)" 두 전략을 모두 계산해서 더 많이
담기는 쪽을 채택한다 (best-of-two). large_first 전략이 항상 후보에 포함되므로
count_first가 large_first보다 적게 담는 일은 이제 없어야 한다.
"""
import sys, pathlib
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # tests/ -> algorism/
_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
_m08 = import_module("08_unloadable_reason")

Trunk = _m02.Trunk
Box = _m03.Box
generate_loading_plan = _m08.generate_loading_plan

# 실제로 버그가 재현된 크기 섞임 세트(고르게 섞인 10개) - large_first는 10/10,
# 수정 전 count_first는 9/10이었다.
_MIXED_BOX_DIMS = [
    ("Box1", 0.25, 0.19, 0.23), ("Box2", 0.17, 0.28, 0.17), ("Box3", 0.17, 0.28, 0.11),
    ("Box4", 0.28, 0.17, 0.12), ("Box5", 0.28, 0.36, 0.12), ("Box6", 0.22, 0.31, 0.29),
    ("Box7", 0.32, 0.25, 0.30), ("Box8", 0.16, 0.36, 0.16), ("Box9", 0.19, 0.18, 0.16),
    ("Box10", 0.39, 0.20, 0.22),
]


def test_count_first_never_places_fewer_than_large_first_on_mixed_sizes():
    trunk = Trunk(width=0.85, depth=1.25, height=0.50)
    boxes = [Box(box_id, w, d, h) for box_id, w, d, h in _MIXED_BOX_DIMS]

    plans_large_first, _ = generate_loading_plan(boxes, trunk, mode="large_first", margin=0.02, allow_stacking=True)
    plans_count_first, _ = generate_loading_plan(boxes, trunk, mode="count_first", margin=0.02, allow_stacking=True)

    assert len(plans_count_first) >= len(plans_large_first)


def test_count_first_still_wins_when_small_first_genuinely_helps():
    # 기존 test_08_generate_loading_plan_mode.py의 커버리지와 같은 취지: 작은 것부터
    # 담는 게 실제로 유리한 시나리오에서는 그 이득을 그대로 유지해야 한다
    # (best-of-two가 이 케이스를 큰 것부터 전략으로 덮어써버리면 안 됨).
    trunk = Trunk(width=0.6, depth=0.4, height=0.45)
    boxes = (
        [Box(f"Small{i}", 0.1, 0.1, 0.1) for i in range(6)]
        + [Box(f"Big{i}", 0.3, 0.2, 0.2) for i in range(2)]
    )
    plans_large_first, _ = generate_loading_plan(boxes, trunk, mode="large_first")
    plans_count_first, _ = generate_loading_plan(boxes, trunk, mode="count_first")

    assert len(plans_count_first) > len(plans_large_first)
    assert len(plans_count_first) == 7
