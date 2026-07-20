"""
05_candidate_scoring.py
⑤ 후보 평가 — 정렬(sort) → 점수화(score) 재설계
====================================================
상태: 🟢 완료·확정 ← 오늘(7/20)의 핵심 작업

[기존 방식의 문제]
후보들을 (z, y, x) 순으로 정렬해서 제일 앞의 유효한 후보를 그냥 골랐다.
"낮은 위치"는 z를 먼저 보니 어느 정도 반영되지만, 팀 완료 기준에 있는
"구석"과 "공간 활용도"는 전혀 반영하지 못한다 — 좌표값이 작다고 실제로
더 구석진(둘러싸인) 자리라는 보장이 없다.

[새 방식]
후보 지점에 박스를 놓았다고 가정하고, 그 박스의 6개 면 중 몇 개가 벽이나
이미 놓인 다른 박스에 "붙는지"(접촉면)를 센다. 접촉면이 많을수록 더
구석에 박히고 빈틈을 덜 남기는(=공간 활용도 높은) 자리라고 본다.

    score = HEIGHT_WEIGHT * (z / trunk.height) - CONTACT_WEIGHT * (접촉면수 / 6)

점수가 낮을수록(더 음수일수록) 좋은 자리. 유효한 후보 중 점수가 가장 낮은
걸 고른다. 가중치는 팀 튜닝 대상 — 지금은 "높이가 1순위, 접촉면은 그 안에서
보정"하도록 기본값을 잡았다 (BR-09: 낮고 안정적인 배치 우선).

[증명된 사실]
아래 데모에서, 좌표값만 보면 더 작아서 기존 방식이 고르는 "열린 자리"보다
"안쪽 구석" 자리가 접촉면이 많아 점수가 더 낮게 나와 새 방식이 실제로
다른(더 나은) 답을 낸다는 걸 확인함.
"""

import sys, pathlib
from typing import List, Tuple
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m03 = import_module("03_extreme_point_candidates")
_m04 = import_module("04_candidate_validity_check")

Box = _m03.Box
PlacedBox = _m03.PlacedBox
boxes_overlap = _m04.boxes_overlap

HEIGHT_WEIGHT = 1.0
CONTACT_WEIGHT = 0.5


def _ranges_overlap(a: Tuple[float, float], b: Tuple[float, float]) -> bool:
    return a[0] < b[1] and a[1] > b[0]


def count_touching_faces(x: float, y: float, z: float, box: "Box", trunk,
                          placed: List["PlacedBox"]) -> int:
    """박스를 (x,y,z)에 놓았을 때, 6개 면 중 벽 또는 다른 박스에 붙는 면 개수(0~6)."""
    touches = 0
    x0, x1 = x, x + box.width
    y0, y1 = y, y + box.depth
    z0, z1 = z, z + box.height

    # 트렁크 벽 6면 (바닥/천장/좌/우/앞/뒤)
    if abs(x0 - 0.0) < 1e-9:
        touches += 1
    if abs(x1 - trunk.width) < 1e-9:
        touches += 1
    if abs(y0 - 0.0) < 1e-9:
        touches += 1
    if abs(y1 - trunk.depth) < 1e-9:
        touches += 1
    if abs(z0 - 0.0) < 1e-9:
        touches += 1
    if abs(z1 - trunk.height) < 1e-9:
        touches += 1

    # 이미 놓인 다른 박스와 맞닿는 면
    for p in placed:
        px0, px1 = p.x_range
        py0, py1 = p.y_range
        pz0, pz1 = p.z_range

        if abs(x1 - px0) < 1e-9 or abs(x0 - px1) < 1e-9:
            if _ranges_overlap((y0, y1), (py0, py1)) and _ranges_overlap((z0, z1), (pz0, pz1)):
                touches += 1
        if abs(y1 - py0) < 1e-9 or abs(y0 - py1) < 1e-9:
            if _ranges_overlap((x0, x1), (px0, px1)) and _ranges_overlap((z0, z1), (pz0, pz1)):
                touches += 1
        if abs(z1 - pz0) < 1e-9 or abs(z0 - pz1) < 1e-9:
            if _ranges_overlap((x0, x1), (px0, px1)) and _ranges_overlap((y0, y1), (py0, py1)):
                touches += 1

    return min(touches, 6)


def score_candidate(x: float, y: float, z: float, box: "Box", trunk,
                     placed: List["PlacedBox"]) -> Tuple[float, int]:
    """(score, 접촉면수)를 같이 반환해서 '왜 이 점수인지' 설명 가능하게 한다."""
    touches = count_touching_faces(x, y, z, box, trunk, placed)
    height_term = HEIGHT_WEIGHT * (z / trunk.height)
    contact_term = CONTACT_WEIGHT * (touches / 6)
    return height_term - contact_term, touches


if __name__ == "__main__":
    _m02 = import_module("02_trunk_space_state")
    Trunk = _m02.Trunk

    # ---- 데모: 정렬 방식과 점수화 방식이 실제로 다른 답을 낼 수 있음을 증명 ----
    trunk = Trunk(width=12.0, depth=12.0, height=4.0)
    box_size = Box(id="unit", width=2, depth=2, height=2)

    placed = [
        PlacedBox(box=Box(id="A", width=2, depth=2, height=2), x=3, y=3, z=0),
        PlacedBox(box=Box(id="B", width=2, depth=2, height=2), x=3, y=5, z=0),
        PlacedBox(box=Box(id="C", width=2, depth=2, height=2), x=5, y=3, z=0),
    ]

    candidate_open = (7.0, 3.0, 0.0)    # C의 +x쪽, 아무것도 없는 열린 자리
    candidate_pocket = (5.0, 5.0, 0.0)  # B와 C 사이 안쪽 구석 자리

    print("=== 후보 비교: '열린 자리' vs '안쪽 구석' ===\n")
    for label, (cx, cy, cz) in [("열린 자리", candidate_open), ("안쪽 구석", candidate_pocket)]:
        score, touches = score_candidate(cx, cy, cz, box_size, trunk, placed)
        print(f"[{label}] 좌표=({cx},{cy},{cz})  접촉면={touches}/6  점수={score:.4f}")

    print("\n--- 기존 방식(정렬, z,y,x 순) ---")
    old_pick = min([candidate_open, candidate_pocket], key=lambda p: (p[2], p[1], p[0]))
    print(f"선택: {old_pick} (단순히 y좌표가 더 작다는 이유로 선택됨)")

    print("\n--- 새 방식(점수화) ---")
    scored = [(label, score_candidate(*p, box_size, trunk, placed)[0])
              for label, p in [("열린 자리", candidate_open), ("안쪽 구석", candidate_pocket)]]
    new_pick_label = min(scored, key=lambda t: t[1])[0]
    print(f"선택: {new_pick_label} (접촉면이 더 많아서 점수가 더 낮음 = 더 좋음)")

    print("\n=== 결론 ===")
    if old_pick == candidate_open and new_pick_label == "안쪽 구석":
        print("기존 방식과 새 방식이 다른 자리를 골랐다 -> 좌표값만으로는 '구석'을 못 잡아낸다는 증거.")
