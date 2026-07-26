# 시뮬레이터 탭 - 산업현장 시나리오 버튼 4개 (3D 뷰어 테마 전환) 설계

날짜: 2026-07-26
대상 저장소: `isaacpjt/Cart2Trunk`
관련 파일 (신규):
`isaacpjt/Cart2Trunk/web/backend/routes/scenarios.py`,
`isaacpjt/Cart2Trunk/web/frontend/src/components/scenarioTheme.js`
관련 파일 (기존, 수정):
`isaacpjt/Cart2Trunk/web/backend/app.py`(블루프린트 등록),
`isaacpjt/Cart2Trunk/web/frontend/src/api/client.js`(함수 2개 추가),
`isaacpjt/Cart2Trunk/web/frontend/src/components/Scene3DViewer.jsx`(시나리오
버튼 4개 + 시나리오 모드 렌더링 추가)

## 배경

`algorism/industry_scenarios/`에 4개 산업 현장 시나리오(택배 배송 트럭,
창고/물류센터, 냉동/냉장 물류, 위험물 창고)가 이미 팀이 만들어둔 알고리즘
래퍼로 존재하지만, 웹 UI에는 한 번도 연결된 적이 없다. 사용자가 시뮬레이터
탭의 3D 뷰어 툴바(camera preset "top" 버튼 옆)에 이 4개를 버튼으로 추가해서,
누르면 3D 뷰어가 그 시나리오의 실제 배치 결과 + 테마 색상으로 바뀌는 걸
원한다. "지금 작업 중인 계획(`state.result`)은 건드리지 않고" 잠깐 미리보기로
전환하는 것으로 확인받았다.

## 목표

1. 백엔드에 `routes/scenarios.py`(신규) - 시나리오 4개 각각 고정된 더미
   트렁크/박스(각 시나리오 파일의 `__main__` 데모 데이터를 그대로 재사용,
   새로 지어내지 않음)로 **실제 알고리즘 함수**
   (`generate_loading_plan_lifo_delivery` 등)를 호출해서 배치 결과를 반환.
   - `GET /api/scenarios` - 4개 메타데이터(id/label/theme) 목록.
   - `POST /api/scenarios/<id>/plan` - 그 시나리오를 계산해서
     `{trunk, placed:[{box_id,position,dimensions,order}], unloadable:[{box_id,reason}], summary:{total,placed,unplaced}}`
     반환.
2. `Scene3DViewer.jsx` 툴바의 camera preset(front/side/top) 버튼 바로
   옆에 시나리오 버튼 4개를 추가한다. 누르면:
   - 해당 시나리오를 계산해서 로컬 상태(`scenarioResult`)에 저장 -
     `PlannerContext`의 `state.result`는 전혀 건드리지 않는다(지금
     시뮬레이터에서 작업 중인 계획이 사라지면 안 됨).
   - 3D 뷰어가 시나리오의 트렁크(단순 색상 와이어프레임, 테마색) + 배치된
     박스(테마 색상)만 보여준다 - Before/After 토글, "순서대로 재생", 카트
     모양은 시나리오 모드에선 숨긴다(카트 개념이 없는 시나리오도 있어서
     - 예: 위험물 창고).
   - "실시간 계획으로 돌아가기" 버튼으로 언제든 원래(`state.result` 기반)
     화면으로 되돌아간다.
   - 카메라 프리셋(front/side/top)은 시나리오 모드에서도 그대로 동작한다.
3. 테마: 시나리오별 고정 색상(박스/트렁크 와이어프레임)을
   `scenarioTheme.js`(신규, 순수 함수)에 정의한다.
   - 택배 배송 트럭: 회색 계열 (트럭 화물칸 느낌)
   - 창고/물류센터: 소형 박스=노랑, 대형 박스=파랑(데모 데이터의 박스
     id가 이미 "소0..5"/"대0..1"이라 id로 구분 가능)
   - 냉동/냉장 물류: 하늘색 계열 (냉기)
   - 위험물 창고: 산화제=주황, 인화물=빨강, 일반박스=회색(데모 데이터의
     박스 id에 "산화제"/"인화물"이 이미 포함돼 있어 id로 구분 가능)

## 비목표

- 실제 새 3D 모델/텍스처 제작 - 이미 있는 `BoundingBoxWireframe`/
  `SceneBoxMesh`(sceneMeshes.jsx)를 색상만 바꿔 재사용한다(사용자가 확인한
  "색상/밀집 테마" 수준).
- 시나리오 파라미터를 사용자가 직접 조정하는 것(마진 값, 박스 개수 등) -
  지금은 각 시나리오 파일에 이미 있는 고정 데모 데이터를 그대로 쓴다.
- `algorism/` 파일 수정 - `industry_scenarios/*.py`는 기존 함수를 import해서
  호출만 하고 일절 수정하지 않는다.
- 시나리오 결과를 승인/전송(MSI2로 보내기) 파이프라인에 태우는 것 - 순수
  미리보기 용도.
- 장애물(obstacles) 렌더링 - 시나리오 알고리즘 자체가 장애물을 다루지
  않아서(비목표) 원본 데이터에 없음.

## 설계

### 1. 백엔드 - `routes/scenarios.py`

`algorism_bridge.py`와 같은 방식(`sys.path`에 `algorism/`, 이번엔
`algorism/industry_scenarios/`도 추가)으로 4개 시나리오 모듈을 import한다.
시나리오별 고정 데이터(각 파일 `__main__` 블록 그대로):

```python
SCENARIO_DEFS = {
    "delivery_truck": {
        "label": "택배 배송 트럭", "theme": "gray",
        "trunk_kwargs": {"width": 1.2, "depth": 0.8, "height": 0.6},
        "make_boxes": lambda: [
            Box("정류장1_박스", 0.3, 0.25, 0.2, delivery_stop=1),
            Box("정류장2_박스", 0.3, 0.25, 0.2, delivery_stop=2),
            Box("정류장3_박스", 0.3, 0.25, 0.2, delivery_stop=3),
            Box("정류장4_박스", 0.3, 0.25, 0.2, delivery_stop=4),
        ],
        "generate": scenario1.generate_loading_plan_lifo_delivery,
    },
    "warehouse": {
        "label": "창고/물류센터", "theme": "warehouse",
        "trunk_kwargs": {"width": 0.6, "depth": 0.4, "height": 0.45},
        "make_boxes": lambda: (
            [Box(f"소{i}", 0.1, 0.1, 0.1) for i in range(6)]
            + [Box(f"대{i}", 0.3, 0.2, 0.2) for i in range(2)]
        ),
        "generate": scenario2.generate_loading_plan_count_first,
    },
    "cold_chain": {
        "label": "냉동/냉장 물류", "theme": "cold_chain",
        "trunk_kwargs": {"width": 1.2, "depth": 0.6, "height": 0.5},
        "make_boxes": lambda: [Box(f"냉동박스{i}", 0.3, 0.25, 0.2) for i in range(3)],
        "generate": scenario3.generate_loading_plan_cold_chain,
    },
    "hazmat": {
        "label": "위험물 창고", "theme": "hazmat",
        "trunk_kwargs": {"width": 1.5, "depth": 1.0, "height": 0.5},
        "make_boxes": lambda: [
            Box("산화제_드럼1", 0.3, 0.3, 0.3, hazard_class="oxidizer"),
            Box("인화물_드럼1", 0.3, 0.3, 0.3, hazard_class="flammable"),
            Box("일반박스1", 0.3, 0.3, 0.3),
        ],
        "generate": scenario4.generate_loading_plan_hazmat,
    },
}
```

`POST /api/scenarios/<id>/plan`: `trunk = Trunk(**def["trunk_kwargs"])`,
`boxes = def["make_boxes"]()`, `plans, unloadable = def["generate"](boxes, trunk)`
호출 후 `PlacementPlan`/`UnloadableItem` 리스트를 최소 shape의 dict로
직렬화해서 반환(기존 `algorism_bridge.compute_plan()`의 무거운
score_breakdown 재구성 로직은 재사용하지 않는다 - 이 미리보기는 그 정보가
필요 없고, 서로 다른 관심사라 새 파일에 독립적으로 작게 만드는 게 더
명확하다).

### 2. `scenarioTheme.js` (신규, 순수 함수 - 유닛 테스트 가능)

```js
export const SCENARIOS = [
  { id: "delivery_truck", label: "택배 배송 트럭" },
  { id: "warehouse", label: "창고/물류센터" },
  { id: "cold_chain", label: "냉동/냉장 물류" },
  { id: "hazmat", label: "위험물 창고" },
];

export function scenarioTrunkColor(scenarioId) { ... } // 시나리오별 고정 색상 1개
export function scenarioBoxColor(scenarioId, boxId) { ... } // 위 목표 3번 규칙
```

### 3. `Scene3DViewer.jsx` 수정

- 새 로컬 상태: `activeScenarioId`(기본 `null`), `scenarioResult`(기본
  `null`), `scenarioError`(기본 `null`).
- 툴바의 `.presetBar`(front/side/top이 있는 곳) 안, 카메라 프리셋 버튼들
  다음에 시나리오 버튼 4개 추가. 활성 시나리오가 있으면 "실시간 계획으로
  돌아가기" 버튼도 같이 보임.
- 시나리오 버튼 클릭 → `postScenarioPlan(id)` 호출 → 성공하면
  `activeScenarioId`/`scenarioResult` 설정, 실패하면 `scenarioError`에
  메시지 저장(간단한 인라인 텍스트로만 표시, 새 다이얼로그 안 만듦).
- 렌더링: `activeScenarioId`가 있으면 `trunk`/`cartFootprint`/`stagedBoxes`
  대신 `scenarioResult.trunk`를 `BoundingBoxWireframe`(테마 트렁크 색)로,
  `scenarioResult.placed`를 `SceneBoxMesh`(테마 박스 색)로 그린다. 없으면
  기존 코드 경로가 **한 글자도 안 바뀐 채** 그대로 실행된다(회귀 없음).
- `.stageBar`(Before/After)와 "순서대로 재생" 버튼은 `activeScenarioId`가
  있을 때 숨긴다.

### 4. `api/client.js` 추가

```js
export async function fetchScenarios() { ... } // GET /api/scenarios
export async function postScenarioPlan(scenarioId) { ... } // POST /api/scenarios/:id/plan
```

### 5. 테스트

- `web/backend/tests/test_routes_scenarios.py`: 4개 시나리오 각각
  `POST /api/scenarios/<id>/plan`이 200과 함께 `placed`/`unloadable`을
  반환하는지, 존재하지 않는 id는 404인지 확인. (선택) 배송 시나리오는
  배송지 역순으로 실린다는 것까지 한 번 더 검증(알고리즘 회귀 방지).
- `scenarioTheme.test.js`: `scenarioBoxColor`/`scenarioTrunkColor`가 각
  시나리오/박스 id 조합에 대해 기대한 색을 반환하는지(순수 함수라 쉽게
  테스트 가능).
- `Scene3DViewer.jsx` 자체는 기존 관례대로(`<Canvas>` 포함) jsdom 렌더링
  테스트를 하지 않는다 - 수동 브라우저 확인으로 검증한다.
