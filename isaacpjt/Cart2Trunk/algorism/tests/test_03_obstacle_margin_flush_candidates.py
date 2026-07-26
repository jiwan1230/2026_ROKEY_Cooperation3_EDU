"""
test_03_obstacle_margin_flush_candidates.py
버그 리포트: obstacle_margin > margin(box_margin)으로 실행하면, 장애물 근처가
아니라 트렁크 어디든(장애물과 전혀 안 겹치는 자리 포함) 박스 배치가 전부
실패하는 걸 발견했다 (실제 trunk_map으로 재현: obstacle_margin=0.02→0.03로만
올려도 3/3 → 0/3).

원인: generate_box_flush_candidates()가 놓인 물체 종류(일반 박스 vs 장애물)를
구분하지 않고 항상 margin(box_margin)만큼 뗀 좌표로만 후보를 만든다. 장애물
옆 후보는 항상 "margin만큼만 뗀" 자리로 생성되는데, ⑰ 유효성 검사는
obstacle_margin(더 큰 값)을 요구하니 그 후보가 거부되고, 대신 "obstacle_margin
만큼 뗀" 후보 자체가 아예 생성된 적이 없어서 대안이 없다 - 결과적으로 후보
전체가 실패한다.

수정: generate_box_flush_candidates()가 obstacle_margin 파라미터를 받아서,
장애물(p.box.is_obstacle=True)에 대해서는 margin 대신 obstacle_margin만큼
뗀 좌표로 후보를 만든다.
"""
import sys, pathlib
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # tests/ -> algorism/


def _has_x_near(candidates, expected_x, y=0.0, z=0.0, tol=1e-6):
    return any(abs(x - expected_x) < tol and abs(cy - y) < tol and abs(cz - z) < tol
               for (x, cy, cz) in candidates)
_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")

Trunk = _m02.Trunk
Box = _m03.Box
PlacedBox = _m03.PlacedBox
generate_box_flush_candidates = _m03.generate_box_flush_candidates


def test_obstacle_margin_generates_a_candidate_at_the_larger_distance():
    """장애물 옆에 obstacle_margin(0.05)만큼 뗀 후보가 실제로 생성돼야 한다 -
    margin(0.02)만큼만 뗀 후보만 있으면 안 됨."""
    obstacle = PlacedBox(box=Box("Wheel", 0.3, 0.3, 0.2, is_obstacle=True), x=0.5, y=0.0, z=0.0)
    box = Box("C", width=0.2, depth=0.2, height=0.2)
    seed_candidates = {(0.0, 0.0, 0.0)}

    without_obstacle_margin = generate_box_flush_candidates(
        box, None, seed_candidates, [obstacle], margin=0.02)
    with_obstacle_margin = generate_box_flush_candidates(
        box, None, seed_candidates, [obstacle], margin=0.02, obstacle_margin=0.05)

    # margin(0.02)만 뗀 x: 0.5 - 0.2 - 0.02 = 0.28
    assert _has_x_near(without_obstacle_margin, 0.28)
    # obstacle_margin(0.05)을 반영하면 0.5 - 0.2 - 0.05 = 0.25가 나와야 함 (0.28이 아니라)
    assert _has_x_near(with_obstacle_margin, 0.25)
    assert not _has_x_near(with_obstacle_margin, 0.28)


def test_regular_box_neighbor_still_uses_plain_margin_even_when_obstacle_margin_given():
    """is_obstacle=False인 일반 카트 박스 이웃은 obstacle_margin이 주어져도 영향받지 않아야 한다."""
    neighbor = PlacedBox(box=Box("N", 0.3, 0.3, 0.2, is_obstacle=False), x=0.5, y=0.0, z=0.0)
    box = Box("C", width=0.2, depth=0.2, height=0.2)
    seed_candidates = {(0.0, 0.0, 0.0)}

    result = generate_box_flush_candidates(
        box, None, seed_candidates, [neighbor], margin=0.02, obstacle_margin=0.05)

    assert _has_x_near(result, 0.28)  # margin(0.02) 그대로 적용됨
    assert not _has_x_near(result, 0.25)  # obstacle_margin은 일반 박스에 적용 안 됨
