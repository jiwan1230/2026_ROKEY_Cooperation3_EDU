# 지지대(받침) 확인 기능 설계 — 적층(2층 이상) 대비

날짜: 2026-07-21
대상 저장소: `isaacpjt/Cart2Trunk/algorism`
관련 파일: `03_extreme_point_candidates.py`, `04_candidate_validity_check.py`, `07_placement_plan.py`

## 배경 및 문제

현재 파이프라인은 박스를 트렁크 바닥 1층으로만 쌓는다. 좌표 구조는 이미 3D이고
(`PlacedBox`에 x/y/z), `register_placement()`가 박스 윗면(z+height)도 다음 후보
좌표로 등록한다. 즉 "박스 위에 놓을 자리"는 이미 후보 목록에 들어간다.

문제는 `is_candidate_valid()`(④)가 검사하는 게 딱 두 가지뿐이라는 점이다:

1. 트렁크 경계 안에 있는가
2. 다른 박스와 겹치지 않는가

**"그 자리 밑에 실제로 뭔가 받쳐주고 있는가"는 전혀 확인하지 않는다.** 실증
테스트로 확인됨: 작은 박스(0.2×0.2×0.3m) 위에 훨씬 넓은 박스(0.6×0.6×0.2m)를
놓는 후보도 `is_candidate_valid()`는 `True`를 반환한다. 실제로는 밑면의
대부분이 허공이라 로봇이 이대로 실행하면 박스가 떨어진다.

지금 당장 적층 데이터는 없지만, 나중에 급하게 처리하면 더 위험하므로 지금
구조를 미리 만들어 두고, 실제 적층 사용 시점까지는 플래그로 꺼둔다.

## 목표

- 후보 좌표 (x, y, z)에 박스를 놓았을 때, 밑면이 실제로 얼마나 지지되는지
  계산하는 로직을 추가한다.
- 지지 비율이 기준(80%) 미만이면 그 후보는 완전히 무효 처리한다 (하드 컷 —
  점수 페널티가 아니라 애초에 후보 목록에서 탈락).
- 지금의 1층 전용 동작은 전혀 건드리지 않는다 — 새 기능은 `allow_stacking`
  플래그(기본값 `False`)로 잠가둔다.
- 기존 확정 파일(④ `04_candidate_validity_check.py`)은 수정하지 않는다.

## 비목표 (지금 범위에서 제외)

- `is_fragile` 필드를 이용한 "파손주의 박스 위 적재 금지" — 지금은 다루지
  않음. 받침 면적 기준만 본다. (나중에 필요해지면 별도 설계)
- 무게(`mass_kg`) 기반 강도/하중 검사 — 범위 밖.
- 무게중심(center of mass) 기반 안정성 검사 — 범위 밖. 이번 설계는 순수
  "밑면 접촉 면적 비율"만 본다.
- 최대 적층 층수/높이 제한 — 범위 밖 (트렁크 높이 경계 체크로 이미 어느 정도
  커버됨).

## 설계

### 새 파일: `13_support_check.py`

기존 파일 번호 체계(①~⑫)를 따라 새 단계로 추가한다. ④(`04_candidate_validity_check.py`)는
"완료·확정" 상태를 유지하며 손대지 않고, 13이 04를 감싸는 방식으로 확장한다.

```python
def compute_support_ratio(x, y, z, box, placed) -> float:
    """
    후보 (x, y, z)에 box를 놓았을 때, 밑면 중 실제로 뭔가에 닿아있는 비율(0.0~1.0).
    """
    if z < 1e-9:
        return 1.0  # 트렁크 바닥은 항상 전체가 받쳐줌

    x0, x1 = x, x + box.width
    y0, y1 = y, y + box.depth
    footprint_area = box.width * box.depth

    supported_area = 0.0
    for p in placed:
        # p의 윗면이 정확히 z에 닿는 경우만 지지대 후보
        if abs(p.z_range[1] - z) > 1e-9:
            continue
        px0, px1 = p.x_range
        py0, py1 = p.y_range
        overlap_x = max(0.0, min(x1, px1) - max(x0, px0))
        overlap_y = max(0.0, min(y1, py1) - max(y0, py0))
        supported_area += overlap_x * overlap_y

    return min(supported_area / footprint_area, 1.0)


def is_candidate_valid_with_stacking(
    x, y, z, box, trunk, placed,
    allow_stacking: bool = False,
    min_support_ratio: float = 0.8,
) -> bool:
    """
    04의 is_candidate_valid()에 지지대 확인을 추가로 얹은 버전.
    04는 그대로 두고 이 함수가 감싸서 확장한다.
    """
    if not is_candidate_valid(x, y, z, box, trunk, placed):  # 04 그대로 재사용
        return False

    if z < 1e-9:
        return True  # 바닥은 항상 통과 (지지 확인 불필요)

    if not allow_stacking:
        return False  # 적층 기능 꺼져있으면 z>0 후보는 무조건 탈락 (지금 동작 100% 보존)

    return compute_support_ratio(x, y, z, box, placed) >= min_support_ratio - 1e-9
```

받침 계산은 격자 샘플링 같은 근사 방식이 아니라, 사각형(AABB) 겹침 넓이를
직접 계산하는 정확한 방식이다 (`05_candidate_scoring.py`의 `_ranges_overlap`과
같은 축정렬 사각형 겹침 계산 패턴을 재사용). 지지대 후보가 전부 AABB 박스이므로
근사 없이 정확한 값이 나온다.

여러 박스가 나눠서 받치는 경우(예: 박스 2개가 각각 밑면의 절반씩 받침)도
자연스럽게 합산된다 — `placed` 안의 박스들은 서로 겹치지 않는다는 게 이미
④에서 보장되므로, 겹침 넓이를 단순 합산해도 이중 계산 위험이 없다.

### 연결 지점 변경: `07_placement_plan.py`

- `place_one_box()`에 `allow_stacking: bool = False` 파라미터 추가.
- 51번 줄의 `is_candidate_valid(...)` 호출을 13의
  `is_candidate_valid_with_stacking(..., allow_stacking=allow_stacking)`로 교체.
- 기본값이 `False`이므로 기존 호출부(`__main__` 데모, `10_verification.py`,
  `12_verify_real_coords.py`)는 아무 변경 없이 그대로 동작해야 한다.

### 문서 업데이트: `11_docs.md`

- 파일 구성 표에 `⑬ 13_support_check.py` 행 추가 (상태: 신규).
- 전체 파이프라인 다이어그램에 "지지대 확인 (allow_stacking 플래그)" 단계를
  ④ 다음에 짧게 언급.

## 테스트 계획

`tests/test_13_support_check.py`를 새로 추가, pytest로 아래 5개 케이스를 검증한다:

1. **바닥(z=0) 후보는 항상 통과** — 받침 비율과 무관하게 `True`.
2. **완전히 받쳐진 경우 통과** — 아래 박스와 크기가 같거나 더 넓은 박스 위에
   놓으면 `compute_support_ratio == 1.0`이고 유효.
3. **회귀 테스트 — "작은 박스 위 큰 박스" 실패 사례**: 0.2×0.2×0.3m 박스 위에
   0.6×0.6×0.2m 박스를 놓는 후보가 `allow_stacking=True`에서도 거부되는지
   확인 (받침 비율 ≈ 0.04/0.36 ≈ 11% < 80%).
4. **분산 받침 합산** — 박스 2개가 나란히 놓여 있고 그 위 후보의 밑면을 두
   박스가 합쳐서 80% 이상 덮으면 통과.
5. **플래그 오프 회귀 방지** — `allow_stacking=False`(기본값)면 받침 비율이
   100%여도 z>0 후보는 무조건 거부됨을 확인 — 지금 1층 전용 동작이 그대로
   보존되는지 지키는 테스트.

기존 검증 스크립트 회귀 확인:
- `python 10_verification.py` — 5/5 PASS 유지 (변경 없음, `allow_stacking`
  기본값 `False`라서 영향 없어야 함).
- `python -m pytest tests/ -v` — 기존 `test_03_extreme_point_candidates.py` +
  새 `test_13_support_check.py` 전부 통과.

## 향후 과제 (이번 범위 밖, 메모만)

- `is_fragile` 활용한 적재 금지 규칙
- 무게중심 기반 안정성 (면적은 충분해도 무게중심이 받침 밖에 있으면 기울 수 있음)
- 최대 적층 층수 제한
- 실제 적층 데이터 도착 시 `allow_stacking=True`로 전환하는 시점/조건 결정
