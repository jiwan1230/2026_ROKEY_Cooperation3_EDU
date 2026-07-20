# ⑪ 문서화 (극점 알고리즘 기반 통합본)

> 완료 기준: **동일 입력으로 결과가 항상 재현 가능해야 한다.** (⑩에서 검증 완료)

## 왜 극점(Extreme Point) 알고리즘인가

층(layer) 쌓기 같은 방식은 "지금 몇 층째인지" 같은 절차적 상태를 계속 들고 있어야 한다.
극점 방식은 **"이미 놓인 박스 리스트 + 후보 좌표 집합"만 있으면 언제든 상태를 재구성**할 수
있다. 이게 트렁크를 매번 재스캔해서 상태를 갱신하는 우리 워크플로우(⑨)와 궁합이 맞는
핵심 이유다.

## 파일 구성 및 상태

| 번호 | 파일명 | 상태 |
|---|---|---|
| ① | `01_object3d_schema.py` | 🟡 초안 완료, Q1~Q3 팀 답변 대기 |
| ② | `02_trunk_space_state.py` | 🔴 보류 — 준형 스캔 데이터 대기 (단, 시뮬레이션 실측값으로 임시 사용 중) |
| ③ | `03_extreme_point_candidates.py` | 🟢 완료·확정 |
| ④ | `04_candidate_validity_check.py` | 🟢 완료·확정 |
| ⑤ | `05_candidate_scoring.py` | 🟢 완료·확정 (오늘의 핵심) |
| ⑥ | `06_loading_order_decision.py` | 🟢 완료·확정 |
| ⑦ | `07_placement_plan.py` | 🟢 완료·확정 |
| ⑧ | `08_unloadable_reason.py` | 🟢 완료·확정 |
| ⑨ | `09_rescan_replan.py` | 🔴 보류 — 재스캔 결과 + 트리거 신호 대기 (인터페이스만 준비) |
| ⑩ | `10_verification.py` | 🟢 완료 — 실제 트렁크 값 반영, 5/5 PASS |
| ⑪ | 이 문서 | 🟢 완료 |

## 전체 파이프라인

```
boxes, trunk
    │
    ▼
[⑥] decide_loading_order(boxes)          부피 큰 순서로 시도 순서 고정
    │
    ▼
for box in order:
    [③] state.candidates 에서 현재 후보 좌표 확인
    [④] is_candidate_valid() 로 겹침/경계 체크 통과한 후보만 추림
        │
        ├─ 유효 후보 있음 → [⑤] score_candidate() 로 접촉면 기반 최고점 선택
        │                   → [⑦] PlacementPlan 생성, state에 새 후보 등록
        │
        └─ 유효 후보 없음 → [⑧] classify_unloadable_reason() 사유 코드 부여
                             (다음 박스는 계속 시도)
    │
    ▼
(PlacementPlan 리스트, UnloadableItem 리스트)
```

## 실제 데이터로 확인한 핵심 결과

트렁크 실측값(폭 0.57m × 깊이 1.12m × 높이 0.25m, `8.rescale_and_rebuild.py` 기준)으로
돌려본 결과:

| 박스 | 결과 | 이유 |
|---|---|---|
| Medium (0.40×0.30×0.25) | ✅ 배치 성공 | |
| Small (0.30×0.20×0.15) | ✅ 배치 성공 | |
| Large (0.50×0.35×0.30) | ❌ 미적재 | 박스 높이(0.30m)가 트렁크 높이(0.25m)를 초과 → `SIZE_EXCEEDS_TRUNK` |

**중요**: 이건 "실제로 안 들어가더라"가 아니라 **"현재 확보한 시뮬레이션 씬 수치로
계산해보니 안 들어간다"**는 뜻. 이 0.25m라는 높이가 진짜 맞는 값인지는 준형의 실제
스캔 데이터로 재확인 필요.

## ⑤ 후보 평가 — 오늘의 핵심 변경 사항

**기존**: 후보를 `(z, y, x)` 순으로 정렬해서 제일 앞의 유효한 후보를 그냥 선택.
"낮은 위치"는 반영되지만 "구석"·"공간 활용도"는 못 잡아냄.

**개선**: 후보에 박스를 놓았다고 가정하고 6개 면 중 벽/다른 박스에 붙는 면(접촉면) 개수를
세서 점수화.

```
score = HEIGHT_WEIGHT × (z / trunk.height) − CONTACT_WEIGHT × (접촉면수 / 6)
```

점수가 낮을수록 좋은 자리. 데모로 "열린 자리"(좌표값 작음)와 "안쪽 구석"(접촉면 많음)이
실제로 다른 결과를 낸다는 것까지 증명함.

## ⑧ 미적재 판단 — 사유 코드별 다음 행동

| 사유 코드 | 재배치 시도 가치 | 다음 행동 |
|---|---|---|
| `SIZE_EXCEEDS_TRUNK` | 없음 | 바로 담당자 호출 |
| `INSUFFICIENT_REMAINING_VOLUME` | 없음 | 바로 담당자 호출 |
| `NO_VALID_CANDIDATE_POSITION` | 있음 | 재배치(reshuffle) 시도 |

`decide_reshuffle_or_call()`이 이 판단을 자동화함.

## 팀원 데이터 도착 시 할 일

| 항목 | 할 일 |
|---|---|
| ① | Q1(confidence 포함 여부, 준형) / Q2(좌표계 절대·상대, 지완) / Q3(봉투 스키마, 준형+지완) 답변 오면 `Object3D` 확정 |
| ② | 준형 스캔 결과 오면 `Trunk` 값을 실측값으로 교체, 단순 직육면체 가정 재검토 |
| ⑨ | 지완 트리거 신호 형식 오면 `on_rescan_trigger()` 구현. `rebuild_state_from_rescan()`은 이미 준비됨 |
| ⑩ | ②의 진짜 스캔 데이터로 검증 스크립트 재실행하여 결과 재확인 |

## 실행 방법

```bash
python 10_verification.py           # 전체 검증 (5/5 PASS 확인됨)
python 08_unloadable_reason.py      # ⑥⑦⑧ 통합 파이프라인 단독 실행 (실제 트렁크 데이터)
python 05_candidate_scoring.py      # ⑤ 점수화 데모 (열린 자리 vs 구석 비교)
```
