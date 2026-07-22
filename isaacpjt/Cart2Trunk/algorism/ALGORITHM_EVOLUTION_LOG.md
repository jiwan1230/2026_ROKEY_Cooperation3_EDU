# 적재 알고리즘 발전 기록 — 손그림 기반 반복 테스트 (2026-07-21 ~ 07-22)

> 노션 업로드용 정리 문서. 손그림으로 시나리오를 던지고 → 우리 알고리즘으로 직접
> 돌려보고 → 결과가 이상하면 원인을 진단해서 코드를 고치고 → 다시 돌려서 증명하는
> 과정을 라운드별로 정리했다. 손그림 원본 이미지 자체는 대화 중에만 첨부돼서
> 저장소에 파일로 남아있지 않고, **알고리즘이 실제로 만들어낸 결과 이미지**는 전부
> `isaacpjt/Cart2Trunk/algorism/local_test_data/`에 저장돼 있다 (아래 각 라운드에 파일명 표기).

## 한눈에 보기

| # | 라운드 | 트리거 | 핵심 발견 | 수정 파일 |
|---|---|---|---|---|
| 1 | 적층 가능 여부 질문 | 대화("박스 위에 박스 쌓을 수 있어?") | 받침(지지) 확인 로직이 아예 없음 | `13_support_check.py` (신규) |
| 2 | 실제 PDF 데이터 비교 | 팀 PDF + 실제 스캔 데이터 | 우리 알고리즘과 팀 스크립트가 다른 자리를 고름 (다른 목적함수) | `local_test_data/manual_placement_test.py` (신규) |
| 3 | 첫 손그림 (트렁크+차바퀴+초록 3개) | 사용자 손그림 | 다 입구 쪽에 몰려 쌓임 | `local_test_data/sketch_placement_test.py` (신규) |
| 4 | 입구 문제 진단 | 3번 결과 분석 | 로컬 원점이 입구 쪽이라 항상 거기부터 채움 | `02_trunk_space_state.py`, `05_candidate_scoring.py` |
| 5 | 로봇 원점 정정 손그림 | 사용자 손그림 (로봇 위치=원점, 고정 접근축) | y축은 입구와 무관 (x축만 봐야 함) — 첫 수정이 버그였음 | `02_trunk_space_state.py`, `05_candidate_scoring.py` |
| 6 | 벽 A/B/C 우선순위 손그림 | 사용자 손그림 (벽별 점수 지정) | 벽마다 다른 가중치로 우대하는 아이디어 | `05_candidate_scoring.py` |
| 7 | 가중치 재조정 + 후보 누락 (Green_Wide) | 6번 결과 검토 | "닿기만 하면" 보너스가 "더 깊이"를 이겨버림 + 벽에 딱 붙는 후보 자체가 누락되는 경우 발견 | `05_candidate_scoring.py`, `03_extreme_point_candidates.py`, `07_placement_plan.py` |
| 8 | 지완 전달용 코드 | 실제 비전 JSON 샘플 확인 | 박스 좌표계가 카메라 기준이라 트렁크(base frame)와 안 맞음 | `01_object3d_schema.py`, `14_run_full_pipeline.py` (신규) |
| 9 | 2층 적재 손그림 | 사용자 손그림 (초록 1층 + 파랑 2층) | `allow_stacking` 실전 테스트 성공, 장애물 위 적재라는 한계 발견 | `local_test_data/sketch_placement_test_2layer.py` (신규) |
| 10 | 카트 재적재 손그림 | 사용자 손그림 (카트+기존 트렁크) | "기존 유지+추가"보다 "전부 재조합"이 더 나은 배치를 냄 | `local_test_data/sketch_placement_test_cart_reload.py` (신규) |
| 11 | 거의 꽉 찬 트렁크 손그림 | 사용자 손그림 (스트레스 테스트) | 31.8% 찬 상태에서도 카트 박스 전부 성공 | `local_test_data/sketch_placement_test_near_full.py` (신규) |
| 12 | F_BigLeft 위치 질문 | 11번 결과에 대한 사용자 질문 | "다른 박스 옆면에 붙는 자리" 후보 누락 (7번과 같은 종류, 다른 패턴) — **다음 라운드에서 수정 예정** | — |

---

## 1. 적층(2층 쌓기) 가능 여부 — 받침 확인 로직 부재 발견

**계기**: "지금 알고리즘으로 박스 위에 박스도 쌓을 수 있어?"라는 질문.

**발견한 문제**: 좌표 구조는 이미 3D였지만, 후보 유효성 검사(`04_candidate_validity_check.py`)가 "겹침"과 "트렁크 경계"만 확인하고 **"이 자리 밑에 받쳐주는 게 있는가"는 전혀 안 물어봤음**. 작은 박스(0.2×0.2×0.3) 위에 훨씬 큰 박스(0.6×0.6×0.2)를 놓아도 "문제없음"으로 판단되는 것을 실증.

**수정 파일**: `13_support_check.py` (신규)

```python
# 13_support_check.py:45
def compute_support_ratio(x, y, z, box, placed):
    if z < 1e-9:
        return 1.0
    x0, x1 = x, x + box.width
    y0, y1 = y, y + box.depth
    footprint_area = box.width * box.depth
    supported_area = 0.0
    for p in placed:
        if abs(p.z_range[1] - z) > 1e-9:
            continue
        px0, px1 = p.x_range
        py0, py1 = p.y_range
        overlap_x = max(0.0, min(x1, px1) - max(x0, px0))
        overlap_y = max(0.0, min(y1, py1) - max(y0, py0))
        supported_area += overlap_x * overlap_y
    return min(supported_area / footprint_area, 1.0)

# 13_support_check.py:75
def is_candidate_valid_with_stacking(x, y, z, box, trunk, placed,
                                      allow_stacking=False, min_support_ratio=0.8):
    if not is_candidate_valid(x, y, z, box, trunk, placed):
        return False
    if z < 1e-9:
        return True
    if not allow_stacking:
        return False
    return compute_support_ratio(x, y, z, box, placed) >= min_support_ratio - 1e-9
```

`07_placement_plan.py`에 `allow_stacking` 플래그로 연결 (기본값 `False` — 실제로 켜지기
전까진 지금의 1층 전용 동작을 100% 보존).

**검증**: TDD 5케이스, `10_verification.py` 5/5, 실제 데이터 미적재 0건 유지.

---

## 2. 실제 PDF 데이터로 비교 — 목적함수 차이 확인

**계기**: 팀 PDF(`15.check_fit_3d.py` 결과물)에 나온 "박스가 어디 들어가는지" 이미지를
주면서 "우리 알고리즘이면 어디에 배치될까?" 질문.

**발견한 사실**: 같은 트렁크·같은 장애물 5개에 대해, 팀 스크립트는 열린 공간에
배치했지만 우리 알고리즘은 **왼쪽 벽에 붙고 작은 장애물 위쪽 면에도 맞닿는 구석
자리**를 골랐음 (접촉면 3/6). 둘 다 "틀린" 게 아니라 최적화 기준 자체가 다름
(우리는 "구석에 파묻히기" 우선).

**결과 파일**: `local_test_data/manual_placement_test.py` (신규)
**결과 이미지**: `local_test_data/our_algorithm_placement_200104.png`

![우리 알고리즘 vs 팀 PDF 비교](local_test_data/our_algorithm_placement_200104.png)

---

## 3. 첫 손그림 — 입구가 막히는 문제 발견

**계기**: 사용자가 손그림으로 트렁크(차 바퀴 2개 + 초록 박스 3개)를 그려서 전달,
"우리 알고리즘이면 어디에 배치될까?" 질문.

**발견한 문제**: 3개 박스 전부 **입구 쪽 구석에 몰려서 쌓임**. 실제 트렁크 운영
관점에서 "안쪽부터 채워야 다음 짐 넣을 때 편한데, 입구부터 막아버리는" 비현실적인
결과.

**결과 파일**: `local_test_data/sketch_placement_test.py` (신규)

> ⚠️ 이 라운드의 결과 이미지(`sketch_placement_result.png`)는 이후 6번·7번 라운드에서
> **같은 파일 이름으로 재실행하면서 덮어써져서, "입구에 몰린" 당시 화면은 더 이상
> 파일로 안 남아있다.** 최종적으로 이 파일에 남아있는 건 7번 라운드의 결과 화면이다
> (아래 7번 항목에서 확인 가능).

---

## 4. 입구 문제 원인 진단 + 1차 수정 (이후 5번에서 버그로 판명)

**진단**: `to_bounding_trunk()`가 트렁크 점들 중 최솟값을 그냥 로컬 `(0,0,0)`으로
잡는데, 극점 알고리즘은 항상 그 원점부터 채워나가는 구조. 실제 데이터로 확인해보니
이 원점이 로봇(입구)에서 제일 가까운 코너였음 — 우연이 아니라 구조적 문제.

**1차 수정**: 원점 자체는 안 건드리고 (로봇 제어 좌표 변환이 이미 이 원점 기준),
스코어링에 "입구에서 먼 정도" 항을 추가.

```python
# 02_trunk_space_state.py:55 (Trunk 필드 추가)
entrance_near_x: bool = True
entrance_near_y: bool = True   # (5번에서 제거됨 - 버그였음)
```

```python
# 05_candidate_scoring.py (당시 버전 - x/y 평균, 5번에서 수정됨)
def entrance_distance_ratio(x, y, box, trunk):
    depth_x = x if trunk.entrance_near_x else (trunk.width - (x + box.width))
    depth_y = y if trunk.entrance_near_y else (trunk.depth - (y + box.depth))
    return ((depth_x / trunk.width) + (depth_y / trunk.depth)) / 2
```

**검증**: TDD 4케이스, 전체 회귀 통과.

---

## 5. 로봇 원점 정정 손그림 — x/y 평균이 버그였음을 발견

**계기**: 사용자가 손그림 2장으로 "로봇 원점은 트렁크 기준 왼쪽 **중앙**에서 오고,
로봇은 항상 정해진 한 방향(고정된 화살표)으로만 접근한다"를 직접 지정.

**발견한 문제**: 4번의 수정이 x/y 둘 다 평균 내는 방식이었는데, **y(좌우 위치)는
입구와 아예 무관**하다는 게 손그림으로 명확해짐 — 로봇은 한 축으로만 접근하므로
좌우 위치가 달라도 입구에서 먼 정도는 같아야 하는데, 평균을 내다보니 y만 달라도
점수가 달라지는 실제 버그였음.

**수정**: `entrance_near_y` 필드 완전 제거, x축만 사용.

```python
# 05_candidate_scoring.py:109
def entrance_distance_ratio(x: float, box: "Box", trunk) -> float:
    depth_x = x if trunk.entrance_near_x else (trunk.width - (x + box.width))
    return depth_x / trunk.width
```

시각화에도 로봇을 트렁크와 분리된 점 + 접근 화살표로 명시 (`local_test_data/sketch_placement_test.py`
갱신, `ROBOT_Y = TRUNK_DEPTH / 2`).

**검증**: `tests/test_05_candidate_scoring.py::test_lateral_y_position_does_not_affect_score`로
같은 x, 다른 y가 완전히 같은 점수를 받는지 회귀 방지. 전체 pytest 13/13.

---

## 6. 벽 A/B/C 우선순위 손그림

**계기**: 사용자가 손그림으로 벽 3개에 이름을 붙임 — A(가장 안쪽, 입구 반대편,
최우선) > B/C(양쪽 측면, 그다음 우선, 서로 동일).

**수정**: `ENTRANCE_WEIGHT`를 `WALL_A_WEIGHT`로 개명(같은 로직), 측면 벽용
`side_wall_distance_ratio()` + `WALL_BC_WEIGHT` 신규 추가.

```python
# 05_candidate_scoring.py:129
def side_wall_distance_ratio(y: float, box: "Box", trunk) -> float:
    dist_to_c = y
    dist_to_b = trunk.depth - (y + box.depth)
    nearest_wall_dist = min(dist_to_c, dist_to_b)
    max_possible = (trunk.depth - box.depth) / 2
    if max_possible < 1e-9:
        return 0.0
    return min(nearest_wall_dist / max_possible, 1.0)
```

이 라운드에서도 같은 파일(`sketch_placement_result.png`)로 다시 렌더링했지만, 이
버전도 7번 라운드에서 다시 덮어써져서 별도로는 안 남아있다 (아래 7번 최종본 참고).

---

## 7. 가중치 재조정 + 벽-밀착 후보 누락 발견 (Green_Wide 사례)

**계기**: 6번 결과 검토 중 사용자가 "지금은 벽에 붙는 걸 기준으로 점수를 주니까
양쪽 사이드로 빠지는 것 같다"고 지적.

**문제 1 - 가중치**: `WALL_BC_WEIGHT`의 "완전히 붙었을 때" 보너스가 `WALL_A_WEIGHT`로
얻는 "조금 더 깊이" 이득보다 커서, 박스가 얕은 채로 아무 옆벽에나 안주함.

```python
# 05_candidate_scoring.py:53-54
WALL_A_WEIGHT = 0.9   # 이전 0.6
WALL_BC_WEIGHT = 0.2  # 이전 0.3
```

**문제 2 - 후보 생성 누락**: 폭 0.28m 박스가 물리적으로 들어갈 수 있는 "벽 A에 딱
붙는 자리"(x=0.32)가 있었는데도, 그 좌표를 만들어줄 기존 모서리가 우연히 없어서
후보 자체가 안 생겼음 (`(0.32, 0.31, 0)`이 `is_candidate_valid`엔 통과하는데
`state.candidates`엔 없었음을 실증).

```python
# 03_extreme_point_candidates.py:134
def generate_wall_flush_candidates(box: Box, trunk, candidates) -> Set[Tuple[float, float, float]]:
    extra: Set[Tuple[float, float, float]] = set()
    wall_a_x = (trunk.width - box.width) if trunk.entrance_near_x else 0.0
    wall_c_y = 0.0
    wall_b_y = trunk.depth - box.depth
    for (x, y, z) in candidates:
        extra.add((wall_a_x, y, z))
        extra.add((x, wall_c_y, z))
        extra.add((x, wall_b_y, z))
    return extra
```

`07_placement_plan.py::place_one_box()`에서 `state.candidates | generate_wall_flush_candidates(...)`로
후보 풀에 병합.

**부수 효과**: 기존 "pocket vs open" 회귀 데모(`05_candidate_scoring.py`,
`10_verification.py`)가 우연히 정중앙에 걸려서 깨짐 → 좌표 재설계로 해결
(`A(8,1) B(8,3) C(10,1)`, `pocket=(10,3)`, `open=(1,1)`).

**검증**: TDD 3케이스(`tests/test_wall_flush_candidates.py`), 전체 pytest 20/20.

**결과 이미지 (최종본 — 3번·6번의 같은 파일을 최종적으로 덮어쓴 버전)**: `local_test_data/sketch_placement_result.png`

![최종 - 3개 박스가 벽 A에 한 줄로 정렬됨](local_test_data/sketch_placement_result.png)

---

## 8. 지완 전달용 코드 — 실제 비전 데이터 좌표계 불일치 발견

**계기**: "지완님 컴퓨터에서 우리 알고리즘 실행하려면 뭘 보내야 해?" 질문. 실제
박스 비전 샘플(`all_boxes_corners_20260721_174311_555644.json`)을 직접 열어봄.

**발견한 문제**: 박스 데이터가 `center_xyz`/`size_xyz`가 아니라 8개 모서리 좌표로
오고, **좌표계가 `depth_camera_optical_frame_from_message_header`(카메라 기준)**였음
— 트렁크 데이터(`trunk_map.json`)는 이미 `m0609_base_link`(로봇 base)로 나오는데
박스는 그 변환 전 단계. 그대로 섞으면 엉뚱한 자리에 배치됨.

**수정**: 좌표계 검증을 넣어 안전장치 마련 + 실제 8모서리 → AABB 변환 로더 신규.

```python
# 01_object3d_schema.py:54
EXPECTED_BOX_FRAME = "m0609_base_link"

# 01_object3d_schema.py:96
def load_boxes_from_vision_json(path) -> List[Object3D]:
    data = json.loads(Path(path).read_text())
    frame = data.get("coordinate_frame")
    if frame != EXPECTED_BOX_FRAME:
        raise ValueError(
            f"박스 비전 데이터의 좌표계가 '{frame}'인데 '{EXPECTED_BOX_FRAME}'이어야 함 - "
            f"트렁크 데이터와 같은 좌표계로 맞춰서 다시 내보내달라고 요청해야 함"
        )
    # ... corners_m 8개 점 min/max로 AABB 근사 (② to_bounding_trunk()와 같은 방식)
```

`14_run_full_pipeline.py` (신규) — `--trunk-map`, `--boxes`, `--allow-stacking`, `--out`
CLI. 최종 좌표를 `local_to_base_frame()`으로 다시 base frame으로 변환해서 출력.

**검증**: TDD 5케이스, 실제 `trunk_map.json`으로 CLI 스모크 테스트, 카메라 좌표계
샘플 넣으면 의도대로 에러 발생 확인. 전체 pytest 25/25.

---

## 9. 2층 적재 손그림 — `allow_stacking` 실전 테스트

**계기**: 사용자가 손그림에 파란 박스 2개를 추가, "초록은 1층, 파랑은 2층에
적재해봐" 지시.

**결과**: 1층 5개 + 2층 2개 전부 성공. 다만 파란 박스 하나가 초록 박스가 아니라
**차 바퀴(휠하우스) 위**에 쌓이는 걸 발견 — 저희 받침 확인 로직이 장애물과 박스를
구분 안 해서 생긴 한계로, 실제 휠하우스는 둥글어서 이렇게 평평하게 못 얹을 수
있음을 사용자에게 고지 (아직 코드 수정 안 함, 알려진 한계로 기록만).

**결과 파일**: `local_test_data/sketch_placement_test_2layer.py` (신규,
`place_one_box_stacked_only()` 헬퍼로 z=0 후보를 배제해서 "무조건 2층" 지시를 그대로 반영)
**결과 이미지**: `local_test_data/sketch_placement_2layer_result.png`

![1층 초록 + 2층 파랑 적재 결과](local_test_data/sketch_placement_2layer_result.png)

---

## 10. 카트 재적재 손그림 — "유지+추가" vs "전부 재조합"

**계기**: 사용자가 손그림에 왼쪽 쇼핑 카트(초록 1개+파랑 2개)를 추가, "이걸 트렁크에
적재해봐" 지시.

**1차 시도(수정 필요)**: 지난번 트렁크 상태(7개)를 고정해두고 카트의 새 3개만
추가 배치 → 사용자가 "그게 아니라 카트+트렁크 전부 새롭게 조합해서 계산해보라는
뜻이었다"고 정정.

**2차 시도(올바른 해석)**: 10개(1층 6개+2층 4개) 전부를 한 배치로 합쳐서 빈
트렁크부터 다시 최적화. 1차 시도 때보다 `Cart_Green`이 훨씬 좋은 자리(벽 A 바로
옆)를 차지하는 것으로 "한 번에 다 보고 정하는 게 순차 확정보다 낫다"를 실증.

**결과 파일**: `local_test_data/sketch_placement_test_cart_reload.py` (신규)
**결과 이미지**: `local_test_data/sketch_placement_cart_reload_result.png`

![카트+트렁크 통합 재적재 결과](local_test_data/sketch_placement_cart_reload_result.png)

---

## 11. 거의 꽉 찬 트렁크 손그림 — 스트레스 테스트

**계기**: 사용자가 손그림으로 트렁크가 초록 박스 5개로 거의 가득 찬 상태를 그리고,
같은 카트 박스 3개(초록 1층+파랑 2층)가 그런 좁은 공간에도 잘 들어가는지 테스트
요청. 이번엔 "이미 확정된 걸 못 옮긴다"는 현실적 제약이 핵심이라 10번과 달리
"기존 고정 + 신규만 배치" 방식으로 진행.

**결과**: 바닥 31.8% 사용 상태에서도 카트 박스 3개 전부 성공 (미적재 0개) — 빈틈을
정확히 찾아 배치, 2층 박스는 기존 박스/장애물 위에 올바르게 쌓임.

**결과 파일**: `local_test_data/sketch_placement_test_near_full.py` (신규)
**결과 이미지**: `local_test_data/sketch_placement_near_full_result.png`

![거의 꽉 찬 트렁크 + 카트 신규 적재 결과](local_test_data/sketch_placement_near_full_result.png)

---

## 12. F_BigLeft 위치 질문 — 또 다른 후보 누락 패턴 발견 (수정 예정)

**계기**: 11번 결과에서 "`F_BigLeft`가 `F_Tall`보다 입구에 더 가까운 이유가 뭐야?
더 안쪽으로 넣을 수 있었을 것 같은데?" 질문.

**진단**: 직접 좌표를 다시 뽑아서 확인 — `F_BigLeft` 배치 시점에 바닥(z=0) 유효
후보가 딱 4개였고 전부 `x=0.000`이었음. 근데 `x=[0.20,0.38]`(먼저 놓인 `F_BigRight`
바로 옆에 딱 붙는 자리)이 물리적으로는 완전히 비어있었는데도 후보로 안 만들어짐.

**원인**: 7번에서 고친 `generate_wall_flush_candidates()`는 "트렁크 바깥쪽 벽
A/B/C"에 딱 붙는 자리만 다뤘지, **"이미 놓인 다른 박스의 옆면"**에 딱 붙는 자리는
아직 다루지 않음 — 같은 계열의 후보 생성 완전성 문제의 또 다른 패턴.

**상태**: 원인 파악 완료, **다음 라운드에서 `generate_wall_flush_candidates()`를
"다른 박스 옆면"까지 커버하도록 확장 예정** (아직 코드 수정 전).

---

## 파일별 최종 변경 요약

| 파일 | 상태 | 이번 반복에서 추가/변경된 것 |
|---|---|---|
| `01_object3d_schema.py` | 기존 파일 수정 | `load_boxes_from_vision_json()`, `EXPECTED_BOX_FRAME` |
| `02_trunk_space_state.py` | 기존 파일 수정 | `Trunk.entrance_near_x`, `to_bounding_trunk()`의 입구 방향 추정 |
| `03_extreme_point_candidates.py` | 기존 파일 수정 | `generate_wall_flush_candidates()` |
| `05_candidate_scoring.py` | 기존 파일 수정 | `entrance_distance_ratio()`, `side_wall_distance_ratio()`, `WALL_A_WEIGHT`, `WALL_BC_WEIGHT` |
| `07_placement_plan.py` | 기존 파일 수정 | `generate_wall_flush_candidates()` 연결 |
| `13_support_check.py` | 신규 | 받침 확인 전체 |
| `14_run_full_pipeline.py` | 신규 | 지완 실사용 CLI 진입점 |
| `local_test_data/*.py` | 신규 (5개 스크립트) | 손그림 시나리오 재현 + 시각화 |
| `local_test_data/*.png` | 신규 (5개 이미지) | 각 라운드 결과 시각화 |

## 테스트 현황

- 최종 pytest: **25/25 통과** (`tests/` 디렉터리)
- `10_verification.py`: 5/5 통과
- 실제 스캔 데이터(`run_20260720_160153`, `run_20260720_200104`) 2개 run: 미적재 0건, 브루트포스 교차검증 통과
