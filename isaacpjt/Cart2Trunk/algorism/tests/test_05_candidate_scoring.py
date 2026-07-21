"""
test_05_candidate_scoring.py
⑤ score_candidate()에 "입구에서 먼 정도" 항을 추가한 게 의도대로 동작하는지 검증.

배경: 사용자가 실제로 지적한 문제 - 지금까지 스코어링은 높이/접촉면만 보고 "입구"와
"안쪽"을 전혀 구분하지 않아서, 매번 로컬 원점(=하필 입구 쪽인 경우가 많음) 근처부터
채워버려 트렁크 입구를 막는 결과가 나왔다.

첫 버전은 entrance_distance_ratio를 x/y 두 축 평균으로 계산했는데, 이것도 버그였다 -
로봇은 x축(정해진 한 방향)으로만 트렁크에 접근하고 y축(좌우 위치)은 입구와 아예
무관한데, 평균을 내다 보니 y좌표만 달라도 점수가 달라지는 잘못된 결과가 나왔다
(사용자가 손그림 비교로 직접 발견함). 그래서 entrance_distance_ratio는 x만 본다.
"""
import sys, pathlib
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # tests/ -> algorism/
_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
_m05 = import_module("05_candidate_scoring")

Trunk = _m02.Trunk
Box = _m03.Box
score_candidate = _m05.score_candidate
entrance_distance_ratio = _m05.entrance_distance_ratio


def test_score_prefers_candidate_farther_from_entrance_when_height_and_contact_tie():
    """
    높이(z=0)와 접촉면 수(둘 다 바닥에만 닿음, 벽/다른 박스와는 안 닿음)가 완전히 같은
    두 후보 중, 입구(로컬 x=0)에서 더 먼 쪽이 더 낮은(=더 좋은) 점수를 받아야 한다.
    y는 일부러 똑같이 둬서 x 차이만으로 비교되게 한다.
    """
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)  # entrance_near_x 기본값 True(로컬 원점=입구)
    box = Box("unit", width=0.2, depth=0.2, height=0.2)

    near_entrance_score, near_touches = score_candidate(0.3, 0.3, 0.0, box, trunk, placed=[])
    far_from_entrance_score, far_touches = score_candidate(0.6, 0.3, 0.0, box, trunk, placed=[])

    # 접촉면 조건이 진짜로 동일한지 먼저 확인 - 아니면 입구 거리 항이 아니라 접촉면 차이 때문에
    # 점수가 달라진 것일 수 있어서, 이 전제가 깨지면 테스트 자체가 무의미해짐
    assert near_touches == far_touches == 1
    assert far_from_entrance_score < near_entrance_score


def test_lateral_y_position_does_not_affect_score():
    """
    로봇은 x축으로만 접근하므로, x와 접촉면 조건이 같으면 y(좌우 위치)가 달라도 점수는
    완전히 같아야 한다. (실제로 발견된 버그 재발 방지용 회귀 테스트 - 예전 버전은 x/y를
    평균 내서 y만 달라도 점수가 달라졌었다.)
    """
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("unit", width=0.2, depth=0.2, height=0.2)

    score_y_low, touches_low = score_candidate(0.4, 0.1, 0.0, box, trunk, placed=[])
    score_y_high, touches_high = score_candidate(0.4, 0.7, 0.0, box, trunk, placed=[])

    assert touches_low == touches_high  # 둘 다 바닥에만 닿아서 접촉면 조건도 동일한지 확인
    assert score_y_low == score_y_high


def test_entrance_distance_ratio_flips_when_entrance_is_on_far_side():
    """trunk.entrance_near_x가 False(입구가 반대쪽)면, 같은 x좌표라도 안쪽 정도 계산이 뒤집혀야 한다."""
    trunk_entrance_near = Trunk(width=1.0, depth=1.0, height=1.0, entrance_near_x=True)
    trunk_entrance_far = Trunk(width=1.0, depth=1.0, height=1.0, entrance_near_x=False)
    box = Box("unit", width=0.2, depth=0.2, height=0.2)

    ratio_when_entrance_near = entrance_distance_ratio(0.0, box, trunk_entrance_near)
    ratio_when_entrance_far = entrance_distance_ratio(0.0, box, trunk_entrance_far)

    assert ratio_when_entrance_near < ratio_when_entrance_far
