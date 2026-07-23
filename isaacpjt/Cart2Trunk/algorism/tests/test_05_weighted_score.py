"""
test_05_weighted_score.py
⑤ make_weighted_score_fn() - "HMI 화면 설계 가이드라인" 문서의 적재 우선순위
슬라이더 중 "입구 우선 ↔ 깊은 위치 우선"과 "공간활용률 우선 ↔ 안정성 우선"
2개 축을 반영하는 score_fn 팩토리.

주의: "공간활용↔안정성" 축은 지금 알고리즘이 가진 신호(count_touching_faces,
접촉면 수) 하나로 근사한다 - 접촉면이 많을수록 빈틈이 적고(공간활용) 동시에
여러 면이 지지된다(안정성 근사)고 보기 때문. 두 개념을 완전히 독립적으로
분리하지는 못한다는 걸 이 테스트에서도 명시해둔다.
"""
import sys, pathlib
from importlib import import_module

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # tests/ -> algorism/
_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
_m05 = import_module("05_candidate_scoring")

Trunk = _m02.Trunk
Box = _m03.Box
score_candidate = _m05.score_candidate
make_weighted_score_fn = _m05.make_weighted_score_fn


def test_default_weights_match_original_score_candidate_exactly():
    """가중치 배율 (1.0, 1.0)이 기존 score_candidate와 완전히 동일해야 한다 (하위 호환)."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", 0.2, 0.2, 0.2)
    weighted = make_weighted_score_fn(entrance_preference=1.0, contact_preference=1.0)

    for pos in [(0.1, 0.1, 0.0), (0.5, 0.5, 0.0), (0.7, 0.2, 0.0)]:
        expected = score_candidate(*pos, box, trunk, [])
        actual = weighted(*pos, box, trunk, [])
        assert actual == pytest.approx(expected)


def test_negative_entrance_preference_prefers_entrance_over_deep_position():
    """entrance_preference<0이면 '입구 우선' - 얕은 자리가 깊은 자리보다 좋아야(점수가 낮아야) 함.
    기본값(양수)에서는 정반대(깊은 자리가 더 좋음)여야 한다."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", 0.2, 0.2, 0.2)
    shallow = (0.0, 0.4, 0.0)   # 입구 바로 앞
    deep = (0.8, 0.4, 0.0)      # 제일 안쪽

    deep_first = make_weighted_score_fn(entrance_preference=1.0)
    assert deep_first(*deep, box, trunk, [])[0] < deep_first(*shallow, box, trunk, [])[0]

    entrance_first = make_weighted_score_fn(entrance_preference=-1.0)
    assert entrance_first(*shallow, box, trunk, [])[0] < entrance_first(*deep, box, trunk, [])[0]


def test_zero_contact_preference_ignores_touching_faces():
    """contact_preference=0이면 접촉면 수가 점수에 전혀 영향을 주지 않아야 한다."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", 0.2, 0.2, 0.2)
    no_contact_weight = make_weighted_score_fn(contact_preference=0.0)

    # 벽에 붙어 접촉면이 많은 자리 vs 트렁크 중앙(접촉면 거의 없음) - 둘 다 z=0, x 같음
    corner = (0.0, 0.0, 0.0)
    open_area = (0.4, 0.4, 0.0)

    corner_score, corner_touches = no_contact_weight(*corner, box, trunk, [])
    open_score, open_touches = no_contact_weight(*open_area, box, trunk, [])
    assert corner_touches > open_touches  # 접촉면 자체는 여전히 계산됨(반환값 유지)
    # 하지만 entrance/wall_bc 항만으로 비교했을 때와 점수가 같아야 함 (contact_term=0)
    height_term = 0.0
    from importlib import import_module as im
    m05 = im("05_candidate_scoring")
    expected_corner = height_term - m05.WALL_A_WEIGHT * m05.entrance_distance_ratio(0.0, box, trunk) \
        - m05.WALL_BC_WEIGHT * (1 - m05.side_wall_distance_ratio(0.0, box, trunk))
    assert corner_score == pytest.approx(expected_corner)
