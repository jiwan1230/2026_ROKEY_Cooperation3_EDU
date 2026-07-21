# 지지대(받침) 확인 기능 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 박스를 다른 박스 위에 쌓는 후보 좌표가 실제로 충분히 받쳐지는지 확인하는
`13_support_check.py`를 추가하고, `allow_stacking` 플래그로 잠근 채 `07_placement_plan.py`에
연결한다.

**Architecture:** 새 파일 `13_support_check.py`가 밑면 접촉 면적 비율을 계산하는
`compute_support_ratio()`와, 기존 `04_candidate_validity_check.py`의 `is_candidate_valid()`를
그대로 감싸는 `is_candidate_valid_with_stacking()`을 제공한다. `07_placement_plan.py`의
`place_one_box()`가 `allow_stacking` 파라미터(기본 `False`)를 받아 이 새 함수를 호출하도록
연결 지점만 바꾼다. 04는 전혀 수정하지 않는다.

**Tech Stack:** Python 3.10, dataclasses, pytest (`python3 -m pytest tests/ -v -p no:anyio` —
이 저장소 환경은 전역 `anyio` pytest 플러그인과 충돌이 있어 `-p no:anyio`가 항상 필요함).

## Global Constraints

- 받침 인정 기준: 밑면 면적의 80% 이상이 실제로 닿아있어야 함 (기본값, 파라미터로 오버라이드 가능).
- 기준 미달 후보는 점수 페널티가 아니라 하드 컷(무효 처리) — 후보 목록에서 완전히 제외.
- `allow_stacking` 플래그 기본값은 `False` — 꺼져 있으면 z>0 후보는 무조건 거부되어 지금의
  1층 전용 동작이 100% 그대로 유지되어야 함.
- `04_candidate_validity_check.py`는 수정하지 않는다 (기존 "완료·확정" 상태·테스트 보존).
- `is_fragile`, `mass_kg`, 무게중심, 최대 층수 — 이번 범위에 포함하지 않음 (스펙의 "비목표" 참고).
- 받침 면적 계산은 근사(격자 샘플링) 없이 AABB 사각형 겹침 넓이로 정확히 계산한다.
- 참고 스펙: `docs/superpowers/specs/2026-07-21-support-check-design.md`

---

### Task 1: `13_support_check.py` 핵심 로직 (TDD)

**Files:**
- Create: `isaacpjt/Cart2Trunk/algorism/13_support_check.py`
- Test: `isaacpjt/Cart2Trunk/algorism/tests/test_13_support_check.py`

**Interfaces:**
- Consumes:
  - `03_extreme_point_candidates.Box` (필드: `width`, `depth`, `height`)
  - `03_extreme_point_candidates.PlacedBox` (필드: `box`, `x`, `y`, `z`, 프로퍼티: `x_range`, `y_range`, `z_range` — 각 `(start, end)` 튜플)
  - `04_candidate_validity_check.is_candidate_valid(x, y, z, box, trunk, placed) -> bool`
  - `02_trunk_space_state.Trunk` (필드: `width`, `depth`, `height`)
- Produces:
  - `compute_support_ratio(x: float, y: float, z: float, box: Box, placed: List[PlacedBox]) -> float`
  - `is_candidate_valid_with_stacking(x: float, y: float, z: float, box: Box, trunk, placed: List[PlacedBox], allow_stacking: bool = False, min_support_ratio: float = 0.8) -> bool`
    — Task 2가 이 두 함수를 그대로 가져다 쓴다.

- [ ] **Step 1: 테스트 파일 뼈대 작성 (5개 테스트 전부, 실패 상태로)**

`isaacpjt/Cart2Trunk/algorism/tests/test_13_support_check.py`:

```python
"""
test_13_support_check.py
⑬ 받침(지지대) 확인 로직 검증.

배경: 극점 알고리즘은 박스 윗면도 다음 후보로 등록하지만(③), 그 자리 밑에
실제로 받쳐주는 게 있는지는 확인하지 않았다(④는 겹침/경계만 봄). 이 테스트는
04_candidate_validity_check.is_candidate_valid를 감싸는 13의 지지대 확인이
"밑면의 80% 이상이 실제로 닿아있어야 유효"하다는 규칙을 지키는지 확인한다.
"""
import sys, pathlib
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # tests/ -> algorism/
_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
_m13 = import_module("13_support_check")

Trunk = _m02.Trunk
Box = _m03.Box
PlacedBox = _m03.PlacedBox
compute_support_ratio = _m13.compute_support_ratio
is_candidate_valid_with_stacking = _m13.is_candidate_valid_with_stacking


def test_floor_candidate_always_fully_supported():
    """z=0(바닥) 후보는 placed가 비어 있어도 받침 비율 1.0, allow_stacking=False여도 유효."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    box = Box("A", width=0.4, depth=0.3, height=0.2)

    assert compute_support_ratio(0.0, 0.0, 0.0, box, placed=[]) == 1.0
    assert is_candidate_valid_with_stacking(
        0.0, 0.0, 0.0, box, trunk, placed=[], allow_stacking=False
    ) is True


def test_fully_supported_box_on_top_is_valid_when_stacking_allowed():
    """아래 박스와 같은 크기의 박스를 정확히 그 위에 놓으면 받침 비율 1.0 -> 유효."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    below = PlacedBox(box=Box("Base", width=0.4, depth=0.4, height=0.2), x=0.0, y=0.0, z=0.0)
    on_top = Box("Top", width=0.4, depth=0.4, height=0.2)

    ratio = compute_support_ratio(0.0, 0.0, 0.2, on_top, placed=[below])
    assert ratio == 1.0
    assert is_candidate_valid_with_stacking(
        0.0, 0.0, 0.2, on_top, trunk, placed=[below], allow_stacking=True
    ) is True


def test_small_base_rejects_much_larger_box_on_top():
    """
    회귀 테스트: 작은 박스(0.2x0.2x0.3) 위에 훨씬 넓은 박스(0.6x0.6x0.2)를 놓으면
    받침 비율이 0.04/0.36 ≈ 11%로 80% 기준에 크게 못 미쳐 거부되어야 한다.
    (브레인스토밍 단계에서 발견한 실제 실패 사례)
    """
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    small_base = PlacedBox(box=Box("Small", width=0.2, depth=0.2, height=0.3), x=0.0, y=0.0, z=0.0)
    big_top = Box("Big", width=0.6, depth=0.6, height=0.2)

    ratio = compute_support_ratio(0.0, 0.0, 0.3, big_top, placed=[small_base])
    assert ratio < 0.8

    assert is_candidate_valid_with_stacking(
        0.0, 0.0, 0.3, big_top, trunk, placed=[small_base], allow_stacking=True
    ) is False


def test_combined_support_from_two_adjacent_boxes():
    """
    박스 하나만으로는 50%(0.8 기준 미달)지만, 옆에 딱 붙은 박스 두 개를 합치면
    100%가 되어 유효해져야 한다 - 여러 박스의 받침 넓이를 합산하는지 확인.
    """
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    left = PlacedBox(box=Box("Left", width=0.25, depth=0.4, height=0.2), x=0.0, y=0.0, z=0.0)
    right = PlacedBox(box=Box("Right", width=0.25, depth=0.4, height=0.2), x=0.25, y=0.0, z=0.0)
    on_top = Box("Top", width=0.5, depth=0.4, height=0.2)

    ratio_left_only = compute_support_ratio(0.0, 0.0, 0.2, on_top, placed=[left])
    assert abs(ratio_left_only - 0.5) < 1e-9  # 하나만 있으면 절반만 지지

    ratio_combined = compute_support_ratio(0.0, 0.0, 0.2, on_top, placed=[left, right])
    assert ratio_combined == 1.0

    assert is_candidate_valid_with_stacking(
        0.0, 0.0, 0.2, on_top, trunk, placed=[left, right], allow_stacking=True
    ) is True


def test_allow_stacking_false_rejects_even_fully_supported_candidate():
    """받침이 100%여도 allow_stacking=False(기본값)면 z>0 후보는 무조건 거부되어야 한다."""
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    below = PlacedBox(box=Box("Base", width=0.4, depth=0.4, height=0.2), x=0.0, y=0.0, z=0.0)
    on_top = Box("Top", width=0.4, depth=0.4, height=0.2)

    assert is_candidate_valid_with_stacking(
        0.0, 0.0, 0.2, on_top, trunk, placed=[below], allow_stacking=False
    ) is False
```

- [ ] **Step 2: 테스트 실행해서 전부 실패하는지 확인 (import 에러로 실패해야 정상)**

Run: `cd isaacpjt/Cart2Trunk/algorism && python3 -m pytest tests/test_13_support_check.py -v -p no:anyio`
Expected: FAIL — `ModuleNotFoundError` 또는 `ImportError`, `13_support_check` 모듈이 아직 없다는 메시지.

- [ ] **Step 3: `13_support_check.py` 구현**

`isaacpjt/Cart2Trunk/algorism/13_support_check.py`:

```python
"""
13_support_check.py
⑬ 지지대(받침) 확인 — 적층(2층 이상) 대비
=============================================
상태: 🟡 신규 (allow_stacking 플래그로 잠긴 채 대기)

[배경]
③(register_placement)이 박스 윗면(z+height)도 다음 후보로 등록하기 때문에,
"박스 위에 놓을 자리"는 이미 후보 목록에 들어가 있다. 그런데 ④(is_candidate_valid)는
경계/겹침만 확인하고 "그 자리 밑에 실제로 뭔가 받쳐주고 있는가"는 전혀 안 물어봤다.

실제로 확인된 문제: 작은 박스(0.2x0.2x0.3) 위에 훨씬 넓은 박스(0.6x0.6x0.2)를
놓는 후보도 ④는 "문제없음"으로 판단한다. 실제로는 밑면 대부분이 허공이라
로봇이 이대로 실행하면 박스가 떨어진다.

[이 파일이 하는 일]
1. compute_support_ratio() - 후보 자리 밑면 중 실제로 닿아있는 비율을 계산
   (근사 아님 - AABB 사각형 겹침 넓이를 정확히 계산)
2. is_candidate_valid_with_stacking() - ④의 is_candidate_valid()를 그대로
   감싸서, 받침 비율이 기준(기본 80%) 미만이면 하드 컷으로 무효 처리

[아직 안 켜짐]
allow_stacking 기본값은 False. 꺼져 있으면 z>0 후보는 무조건 거부되어
지금의 1층 전용 동작이 그대로 유지된다. 실제 적층 데이터/운영 결정이
나오면 07_placement_plan.py 호출부에서 True로 켠다.

[비목표 - 스펙 참고: docs/superpowers/specs/2026-07-21-support-check-design.md]
is_fragile 활용, 무게(mass_kg) 기반 강도 검사, 무게중심 안정성, 최대 층수 제한
— 전부 이번 범위 밖.
"""

import sys, pathlib
from typing import List
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m03 = import_module("03_extreme_point_candidates")
_m04 = import_module("04_candidate_validity_check")

Box = _m03.Box
PlacedBox = _m03.PlacedBox
is_candidate_valid = _m04.is_candidate_valid


def compute_support_ratio(x: float, y: float, z: float, box: "Box", placed: List["PlacedBox"]) -> float:
    """
    후보 (x, y, z)에 box를 놓았을 때, 밑면 중 실제로 뭔가에 닿아있는 비율(0.0~1.0).

    z가 바닥(0)이면 트렁크 바닥이 항상 전체를 받쳐주므로 무조건 1.0.
    z가 0보다 크면, placed 중 "윗면이 정확히 z에 닿는" 박스들만 지지대 후보로
    보고, 그 박스들과 candidate 밑면의 x/y 겹침 사각형 넓이를 전부 더한다.
    placed 안의 박스들은 서로 겹치지 않는다는 게 ④에서 이미 보장되므로,
    겹침 넓이를 단순 합산해도 이중 계산될 위험이 없다.
    """
    if z < 1e-9:
        return 1.0

    x0, x1 = x, x + box.width
    y0, y1 = y, y + box.depth
    footprint_area = box.width * box.depth

    supported_area = 0.0
    for p in placed:
        if abs(p.z_range[1] - z) > 1e-9:
            continue  # 이 박스의 윗면이 후보 밑면과 안 맞닿아 있으면 지지대가 아님
        px0, px1 = p.x_range
        py0, py1 = p.y_range
        overlap_x = max(0.0, min(x1, px1) - max(x0, px0))
        overlap_y = max(0.0, min(y1, py1) - max(y0, py0))
        supported_area += overlap_x * overlap_y

    return min(supported_area / footprint_area, 1.0)


def is_candidate_valid_with_stacking(
    x: float, y: float, z: float, box: "Box", trunk, placed: List["PlacedBox"],
    allow_stacking: bool = False,
    min_support_ratio: float = 0.8,
) -> bool:
    """
    ④의 is_candidate_valid()(경계/겹침 확인)에 받침 확인을 추가로 얹은 버전.
    ④는 수정하지 않고 그대로 재사용한다.
    """
    if not is_candidate_valid(x, y, z, box, trunk, placed):
        return False

    if z < 1e-9:
        return True  # 바닥은 항상 통과 - 지지 확인 자체가 불필요

    if not allow_stacking:
        return False  # 적층 기능이 꺼져 있으면 z>0 후보는 무조건 탈락

    return compute_support_ratio(x, y, z, box, placed) >= min_support_ratio - 1e-9


if __name__ == "__main__":
    _m02 = import_module("02_trunk_space_state")
    Trunk = _m02.Trunk

    # 데모: 브레인스토밍 단계에서 발견한 실패 사례가 이제 거부되는지 확인
    trunk = Trunk(width=1.0, depth=1.0, height=1.0)
    small_base = PlacedBox(box=Box("Small", width=0.2, depth=0.2, height=0.3), x=0.0, y=0.0, z=0.0)
    big_top = Box("Big", width=0.6, depth=0.6, height=0.2)

    ratio = compute_support_ratio(0.0, 0.0, 0.3, big_top, placed=[small_base])
    print(f"작은 박스(0.2x0.2) 위 큰 박스(0.6x0.6) 받침 비율: {ratio:.1%}")
    print("allow_stacking=True에서 유효한가:",
          is_candidate_valid_with_stacking(0.0, 0.0, 0.3, big_top, trunk, [small_base], allow_stacking=True))
    print("(기대: False - 받침 비율이 80% 기준에 크게 못 미침)")
```

- [ ] **Step 4: 테스트 실행해서 5개 전부 통과하는지 확인**

Run: `cd isaacpjt/Cart2Trunk/algorism && python3 -m pytest tests/test_13_support_check.py -v -p no:anyio`
Expected: `5 passed`

- [ ] **Step 5: 데모 스크립트로 수동 확인**

Run: `cd isaacpjt/Cart2Trunk/algorism && python3 13_support_check.py`
Expected: 받침 비율이 약 `11.1%`로 출력되고, `allow_stacking=True`에서도 `False`(무효)로 나옴.

- [ ] **Step 6: 기존 테스트 회귀 확인 후 커밋**

Run: `cd isaacpjt/Cart2Trunk/algorism && python3 -m pytest tests/ -v -p no:anyio`
Expected: 기존 `test_03_extreme_point_candidates.py` 1개 + 새 5개 = `6 passed`

```bash
cd isaacpjt/Cart2Trunk/algorism
git add 13_support_check.py tests/test_13_support_check.py
git commit -m "feat: add stacking support-area check (13_support_check.py)"
```

---

### Task 2: `07_placement_plan.py` 연결 — `allow_stacking` 플래그 배선

**Files:**
- Modify: `isaacpjt/Cart2Trunk/algorism/07_placement_plan.py:20-29` (import 블록), `07_placement_plan.py:43-55` (`place_one_box` 시그니처와 검증 호출)
- Test: `isaacpjt/Cart2Trunk/algorism/tests/test_07_placement_plan_stacking.py`

**Interfaces:**
- Consumes:
  - Task 1의 `13_support_check.is_candidate_valid_with_stacking(x, y, z, box, trunk, placed, allow_stacking=False, min_support_ratio=0.8) -> bool`
  - `03_extreme_point_candidates.ExtremePointState`, `PlacedBox`, `Box`
  - `05_candidate_scoring.score_candidate`
- Produces:
  - `place_one_box(box, trunk, state, order, allow_stacking: bool = False) -> Optional[PlacementPlan]`
    (기존 시그니처에 `allow_stacking` 파라미터만 추가, 기본값 `False`라서 기존 호출부는 무변경으로 동작)

- [ ] **Step 1: 실패하는 테스트 작성 (스택 시나리오)**

`isaacpjt/Cart2Trunk/algorism/tests/test_07_placement_plan_stacking.py`:

```python
"""
test_07_placement_plan_stacking.py
⑦ place_one_box()의 allow_stacking 플래그 배선 확인.

시나리오: 트렁크 바닥 전체를 정확히 채우는 박스를 먼저 놓으면, 두 번째 같은
박스는 바닥에 더 놓을 자리가 없다. allow_stacking=False(기본값)면 이때
"놓을 자리 없음"(None)이어야 하고, allow_stacking=True면 첫 번째 박스 위에
(받침 100%로) 정확히 쌓여야 한다.
"""
import sys, pathlib
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # tests/ -> algorism/
_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
_m07 = import_module("07_placement_plan")

Trunk = _m02.Trunk
Box = _m03.Box
ExtremePointState = _m03.ExtremePointState
place_one_box = _m07.place_one_box


def _fills_floor_trunk():
    # 바닥 면적을 정확히 채우는 박스 하나가 들어갈 트렁크 (0.3 x 0.3), 두 층 놓을 높이는 있음
    return Trunk(width=0.3, depth=0.3, height=0.6)


def test_second_box_has_nowhere_to_go_when_stacking_disabled():
    trunk = _fills_floor_trunk()
    state = ExtremePointState()
    filler = Box("Floor", width=0.3, depth=0.3, height=0.3)

    first = place_one_box(filler, trunk, state, order=1)
    assert first is not None
    assert first.position == (0.0, 0.0, 0.0)

    second = place_one_box(filler, trunk, state, order=2, allow_stacking=False)
    assert second is None  # 바닥엔 자리 없고, z>0은 플래그가 꺼져 있어 거부됨


def test_second_box_stacks_on_top_when_stacking_enabled():
    trunk = _fills_floor_trunk()
    state = ExtremePointState()
    filler = Box("Floor", width=0.3, depth=0.3, height=0.3)

    first = place_one_box(filler, trunk, state, order=1)
    assert first is not None

    second = place_one_box(filler, trunk, state, order=2, allow_stacking=True)
    assert second is not None
    assert second.position == (0.0, 0.0, 0.3)  # 첫 박스 바로 위, 받침 100%
```

- [ ] **Step 2: 테스트 실행해서 실패하는지 확인**

Run: `cd isaacpjt/Cart2Trunk/algorism && python3 -m pytest tests/test_07_placement_plan_stacking.py -v -p no:anyio`
Expected: FAIL — `test_second_box_stacks_on_top_when_stacking_enabled`가
`TypeError: place_one_box() got an unexpected keyword argument 'allow_stacking'`로 실패
(`test_second_box_has_nowhere_to_go_when_stacking_disabled`도 같은 이유로 실패).

- [ ] **Step 3: `07_placement_plan.py` 수정**

[07_placement_plan.py:20-29](isaacpjt/Cart2Trunk/algorism/07_placement_plan.py#L20-L29) 교체:

```python
sys.path.insert(0, str(pathlib.Path(__file__).parent))
_m03 = import_module("03_extreme_point_candidates")
_m05 = import_module("05_candidate_scoring")
_m13 = import_module("13_support_check")

Box = _m03.Box
PlacedBox = _m03.PlacedBox
ExtremePointState = _m03.ExtremePointState
is_candidate_valid_with_stacking = _m13.is_candidate_valid_with_stacking
score_candidate = _m05.score_candidate
```

(`_m04 = import_module("04_candidate_validity_check")`와 `is_candidate_valid = _m04.is_candidate_valid`
줄은 제거 — 07은 이제 13을 통해서만 검증한다. 13 내부가 04를 계속 호출하므로 검증 내용은 동일하다.)

[07_placement_plan.py:43-55](isaacpjt/Cart2Trunk/algorism/07_placement_plan.py#L43-L55) 교체:

```python
def place_one_box(
    box: "Box", trunk, state: "ExtremePointState", order: int,
    allow_stacking: bool = False,
) -> Optional["PlacementPlan"]:
    """
    현재 상태(state)에서 box 하나를 놓을 최선의 자리를 찾아 배치한다.
    자리가 없으면 None (이 경우 ⑧ 미적재 판단으로 넘어가야 함).

    allow_stacking=False(기본값)면 z>0 후보(박스 위에 놓는 자리)는 ⑬에서
    무조건 거부되어 지금의 1층 전용 동작과 동일하게 동작한다.
    """
    # [④+⑬] 현재 후보 좌표들 중, 겹치지도 밖으로 나가지도 않고(④) 충분히
    # 받쳐지는(⑬, allow_stacking일 때만) 것만 추림
    valid_candidates = [
        (x, y, z) for (x, y, z) in state.candidates
        if is_candidate_valid_with_stacking(x, y, z, box, trunk, state.placed, allow_stacking=allow_stacking)
    ]
```

(이 다음 줄(기존 54행 `if not valid_candidates:` 이후)은 그대로 둔다 — 변경 범위는 이 두 블록뿐.)

- [ ] **Step 4: 테스트 실행해서 통과하는지 확인**

Run: `cd isaacpjt/Cart2Trunk/algorism && python3 -m pytest tests/test_07_placement_plan_stacking.py -v -p no:anyio`
Expected: `2 passed`

- [ ] **Step 5: 전체 회귀 확인 후 커밋**

Run:
```bash
cd isaacpjt/Cart2Trunk/algorism
python3 10_verification.py
python3 -m pytest tests/ -v -p no:anyio
```
Expected: `10_verification.py`는 `총 5개 항목 중 5개 통과` 그대로 유지, pytest는
`8 passed` (기존 1 + Task 1의 5 + Task 2의 2).

```bash
cd isaacpjt/Cart2Trunk/algorism
git add 07_placement_plan.py tests/test_07_placement_plan_stacking.py
git commit -m "feat: wire allow_stacking flag through place_one_box"
```

---

### Task 3: 문서 업데이트 + 실제 데이터 회귀 확인

**Files:**
- Modify: `isaacpjt/Cart2Trunk/algorism/11_docs.md`

**Interfaces:**
- Consumes: 없음 (문서 전용 작업)
- Produces: 없음 (이후 태스크 없음 — 이 계획의 마지막 태스크)

- [ ] **Step 1: 파일 구성 표에 ⑬ 행 추가**

[11_docs.md:12-27](isaacpjt/Cart2Trunk/algorism/11_docs.md#L12-L27)의 표에서 `| ⑫ | ... |` 행 바로
다음 줄에 추가:

```markdown
| ⑬ | `13_support_check.py` | 🟡 신규 — 받침(지지대) 확인, `allow_stacking` 플래그로 잠금 (7/21) |
```

- [ ] **Step 2: 전체 파이프라인 섹션에 지지대 확인 단계 언급 추가**

[11_docs.md:38-47](isaacpjt/Cart2Trunk/algorism/11_docs.md#L38-L47)의 파이프라인 코드 블록에서
아래 줄:

```
    [④] is_candidate_valid() 로 겹침/경계 체크 통과한 후보만 추림
```

을 아래로 교체:

```
    [④+⑬] is_candidate_valid_with_stacking() 로 겹침/경계(④) + 받침 비율(⑬,
        allow_stacking=True일 때만) 체크 통과한 후보만 추림
```

- [ ] **Step 3: "실행 방법" 섹션에 새 파일 실행 명령 추가**

[11_docs.md:150-157](isaacpjt/Cart2Trunk/algorism/11_docs.md#L150-L157)의 코드 블록 마지막 줄
(`python -m pytest tests/ -v ...`) 앞에 추가:

```
python 13_support_check.py         # ⑬ 받침 비율 데모 (작은 박스 위 큰 박스 -> 거부 증명)
```

- [ ] **Step 4: 실제 스캔 데이터로 최종 회귀 확인**

`12_verify_real_coords.py`는 04의 `is_candidate_valid`를 직접 호출하고 07/13을 거치지 않으므로
이번 변경의 영향을 받지 않는다 — 그래도 아무것도 안 깨졌는지 마지막으로 확인한다.

Run: `cd isaacpjt/Cart2Trunk/algorism && python3 12_verify_real_coords.py`
Expected: 두 run 모두 미적재 0건 (변경 전과 동일한 출력).

- [ ] **Step 5: 문서 커밋**

```bash
cd isaacpjt/Cart2Trunk/algorism
git add 11_docs.md
git commit -m "docs: document 13_support_check.py in pipeline overview"
```
