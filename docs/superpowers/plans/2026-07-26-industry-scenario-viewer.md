# 시뮬레이터 탭 산업현장 시나리오 버튼 4개 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 시뮬레이터 탭 3D 뷰어 툴바에 산업현장 시나리오 버튼 4개(택배 배송
트럭/창고·물류센터/냉동·냉장 물류/위험물 창고)를 추가한다. 누르면
`algorism/industry_scenarios/`의 실제 알고리즘을 돌려서 그 시나리오의
배치 결과를 테마 색상으로 미리보기하고, 지금 작업 중인 계획(`state.result`)은
그대로 유지한다.

**Architecture:** 백엔드에 새 블루프린트(`routes/scenarios.py`)를 추가해
각 시나리오 파일의 기존 데모 데이터로 실제 알고리즘 함수를 호출한다.
프론트엔드는 `Scene3DViewer.jsx`에 로컬 상태(`activeScenarioId`,
`scenarioResult`)를 추가해서, 시나리오가 활성화되면 기존 `state.result`
기반 렌더링 대신 시나리오 결과를 보여주고, 없으면 기존 코드 경로가 그대로
실행된다(회귀 없음). 색상 매핑은 순수 함수(`scenarioTheme.js`)로 분리해서
따로 테스트한다.

**Tech Stack:** React 18 + Vite, react-three-fiber(3D), Flask, Vitest +
@testing-library/react, pytest.

## Global Constraints

- 산출물(주석, 커밋 메시지, 테스트 설명)은 전부 한국어로 작성한다.
- `algorism` 브랜치에 태스크마다 바로 커밋한다.
- `algorism/industry_scenarios/*.py`는 기존 함수를 import해서 호출만 하고
  절대 수정하지 않는다.
- 백엔드 테스트: `cd isaacpjt/Cart2Trunk/web/backend && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./venv/bin/python3 -m pytest -q`
- 프론트엔드 테스트: `cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run`
- `Scene3DViewer.jsx`는 `<Canvas>`를 포함해 jsdom으로 직접 렌더링하지
  않는다(기존 관례) - 이번 작업에서도 순수 색상/데이터 로직만
  `scenarioTheme.js`로 분리해서 테스트하고, `Scene3DViewer.jsx` 자체의
  변경은 수동 브라우저 확인으로 검증한다.
- `activeScenarioId`가 `null`일 때 `Scene3DViewer.jsx`의 기존 렌더링 결과는
  100% 그대로 유지되어야 한다(시뮬레이터 탭 회귀 없음).

---

### Task 1: 백엔드 - `routes/scenarios.py` (시나리오 4종 계산 엔드포인트)

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/backend/routes/scenarios.py`
- Create: `isaacpjt/Cart2Trunk/web/backend/tests/test_routes_scenarios.py`
- Modify: `isaacpjt/Cart2Trunk/web/backend/app.py`

**Interfaces:**
- Produces: `POST /api/scenarios/<scenario_id>/plan` (`scenario_id` ∈
  `delivery_truck`, `warehouse`, `cold_chain`, `hazmat`) - 응답
  `{"label": str, "trunk": {width,depth,height,entrance_near_x},
  "placed": [{box_id,position:[x,y,z],dimensions:[w,d,h],order}],
  "unloadable": [{box_id,reason}], "summary": {total,placed,unplaced}}`
  (HTTP 200). 알 수 없는 id는 404 + `{"error_code":"SCENARIO_NOT_FOUND",...}`.

- [ ] **Step 1: 실패하는 테스트부터 작성**

```python
# isaacpjt/Cart2Trunk/web/backend/tests/test_routes_scenarios.py
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from app import create_app


def _client():
    return create_app().test_client()


def test_delivery_truck_places_later_stops_farther_from_entrance():
    # LIFO 정책 - 나중 배송지(delivery_stop 숫자가 큰) 박스가 입구에서 더
    # 먼(x가 큰) 자리에 있어야 한다(실측: 1/2번은 x=0.56, 3/4번은 x=0.88).
    resp = _client().post("/api/scenarios/delivery_truck/plan")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["summary"]["total"] == 4
    by_id = {p["box_id"]: p for p in body["placed"]}
    assert by_id["정류장4_박스"]["position"][0] > by_id["정류장1_박스"]["position"][0]


def test_warehouse_places_6_of_8_boxes():
    # 실측: 6개(소형6+대형2 중 소형만 다 들어가고 대형 일부는 못 들어감)
    # 데모 트렁크(0.6x0.4x0.45)가 좁아서 8개 전부는 안 들어간다.
    resp = _client().post("/api/scenarios/warehouse/plan")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["summary"] == {"total": 8, "placed": 6, "unplaced": 2}


def test_cold_chain_places_all_3_boxes():
    resp = _client().post("/api/scenarios/cold_chain/plan")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["summary"] == {"total": 3, "placed": 3, "unplaced": 0}


def test_hazmat_places_oxidizer_and_flammable_apart():
    resp = _client().post("/api/scenarios/hazmat/plan")
    assert resp.status_code == 200
    body = resp.get_json()
    ids = {p["box_id"] for p in body["placed"]}
    assert {"산화제_드럼1", "인화물_드럼1", "일반박스1"} == ids


def test_unknown_scenario_returns_404():
    resp = _client().post("/api/scenarios/nonexistent/plan")
    assert resp.status_code == 404
    assert resp.get_json()["error_code"] == "SCENARIO_NOT_FOUND"
```

- [ ] **Step 2: 테스트가 실패하는지 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/backend && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./venv/bin/python3 -m pytest tests/test_routes_scenarios.py -v
```
Expected: FAIL - `ModuleNotFoundError: No module named 'routes.scenarios'`

- [ ] **Step 3: `routes/scenarios.py` 구현**

```python
"""
routes/scenarios.py
POST /api/scenarios/<scenario_id>/plan - 산업 현장 시나리오 4종
(algorism/industry_scenarios/) 미리보기. 각 시나리오 파일에 이미 있는
데모 트렁크/박스 데이터로 실제 시나리오 알고리즘을 호출해서 배치 결과를
반환한다. algorism/ 파일은 이 프로젝트에서 수정 금지라 기존 함수를
import해서 호출만 한다(algorism_bridge.py와 동일한 원칙).
"""
import sys
import pathlib
from importlib import import_module

from flask import Blueprint, jsonify

from routes.plan import ApiError

_HERE = pathlib.Path(__file__).resolve().parent
_CART2TRUNK_DIR = _HERE.parent.parent
_ALGORISM_DIR = _CART2TRUNK_DIR / "algorism"
_SCENARIOS_DIR = _ALGORISM_DIR / "industry_scenarios"
for _p in (str(_ALGORISM_DIR), str(_SCENARIOS_DIR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

Trunk = import_module("02_trunk_space_state").Trunk
Box = import_module("03_extreme_point_candidates").Box
_scenario1 = import_module("scenario1_delivery_truck")
_scenario2 = import_module("scenario2_warehouse_density")
_scenario3 = import_module("scenario3_cold_chain")
_scenario4 = import_module("scenario4_hazmat")

scenarios_bp = Blueprint("scenarios", __name__)

SCENARIO_DEFS = {
    "delivery_truck": {
        "label": "택배 배송 트럭",
        "trunk_kwargs": {"width": 1.2, "depth": 0.8, "height": 0.6},
        "make_boxes": lambda: [
            Box("정류장1_박스", 0.3, 0.25, 0.2, delivery_stop=1),
            Box("정류장2_박스", 0.3, 0.25, 0.2, delivery_stop=2),
            Box("정류장3_박스", 0.3, 0.25, 0.2, delivery_stop=3),
            Box("정류장4_박스", 0.3, 0.25, 0.2, delivery_stop=4),
        ],
        "generate": _scenario1.generate_loading_plan_lifo_delivery,
    },
    "warehouse": {
        "label": "창고/물류센터",
        "trunk_kwargs": {"width": 0.6, "depth": 0.4, "height": 0.45},
        "make_boxes": lambda: (
            [Box(f"소{i}", 0.1, 0.1, 0.1) for i in range(6)]
            + [Box(f"대{i}", 0.3, 0.2, 0.2) for i in range(2)]
        ),
        "generate": _scenario2.generate_loading_plan_count_first,
    },
    "cold_chain": {
        "label": "냉동/냉장 물류",
        "trunk_kwargs": {"width": 1.2, "depth": 0.6, "height": 0.5},
        "make_boxes": lambda: [Box(f"냉동박스{i}", 0.3, 0.25, 0.2) for i in range(3)],
        "generate": _scenario3.generate_loading_plan_cold_chain,
    },
    "hazmat": {
        "label": "위험물 창고",
        "trunk_kwargs": {"width": 1.5, "depth": 1.0, "height": 0.5},
        "make_boxes": lambda: [
            Box("산화제_드럼1", 0.3, 0.3, 0.3, hazard_class="oxidizer"),
            Box("인화물_드럼1", 0.3, 0.3, 0.3, hazard_class="flammable"),
            Box("일반박스1", 0.3, 0.3, 0.3),
        ],
        "generate": _scenario4.generate_loading_plan_hazmat,
    },
}


@scenarios_bp.post("/api/scenarios/<scenario_id>/plan")
def compute_scenario_plan(scenario_id):
    scenario_def = SCENARIO_DEFS.get(scenario_id)
    if scenario_def is None:
        raise ApiError(
            404, "SCENARIO_NOT_FOUND", f"'{scenario_id}' 시나리오가 없습니다.",
            "지원하는 시나리오 id(delivery_truck/warehouse/cold_chain/hazmat)인지 확인하세요.",
        )

    trunk = Trunk(**scenario_def["trunk_kwargs"])
    boxes = scenario_def["make_boxes"]()
    plans, unloadable = scenario_def["generate"](boxes, trunk)

    total = len(boxes)
    return jsonify({
        "label": scenario_def["label"],
        "trunk": {
            "width": trunk.width, "depth": trunk.depth, "height": trunk.height,
            "entrance_near_x": trunk.entrance_near_x,
        },
        "placed": [
            {"box_id": p.box_id, "position": list(p.position), "dimensions": list(p.dimensions), "order": p.order}
            for p in plans
        ],
        "unloadable": [{"box_id": u.box_id, "reason": u.reason.value} for u in unloadable],
        "summary": {"total": total, "placed": len(plans), "unplaced": total - len(plans)},
    })
```

- [ ] **Step 4: `app.py`에 블루프린트 등록**

`robot_bp` 등록 바로 아래에 추가:

```python
    from routes.scenarios import scenarios_bp
    app.register_blueprint(scenarios_bp)
```

- [ ] **Step 5: 테스트 통과 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/backend && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./venv/bin/python3 -m pytest tests/test_routes_scenarios.py -v
```
Expected: PASS (5개 전부)

- [ ] **Step 6: 백엔드 전체 테스트 스위트로 회귀 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/backend && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./venv/bin/python3 -m pytest -q
```
Expected: 기존 39개 + 신규 5개 = 44개 전부 PASS

- [ ] **Step 7: 커밋**

```bash
cd /home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU
git add isaacpjt/Cart2Trunk/web/backend/routes/scenarios.py \
        isaacpjt/Cart2Trunk/web/backend/tests/test_routes_scenarios.py \
        isaacpjt/Cart2Trunk/web/backend/app.py
git commit -m "web backend: 산업현장 시나리오 4종 계산 엔드포인트 추가 (industry_scenarios 최초 웹 연동)"
```

---

### Task 2: 프론트엔드 - `scenarioTheme.js` (순수 테마 함수) + API 클라이언트

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/scenarioTheme.js`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/scenarioTheme.test.js`
- Modify: `isaacpjt/Cart2Trunk/web/frontend/src/api/client.js`
- Modify: `isaacpjt/Cart2Trunk/web/frontend/src/api/client.test.js`

**Interfaces:**
- Consumes: Task 1의 `POST /api/scenarios/<id>/plan`.
- Produces: `SCENARIOS`(`{id,label}[]`, 4개, id는 Task 1의
  `SCENARIO_DEFS` 키와 정확히 일치), `scenarioTrunkColor(scenarioId):
  string`, `scenarioBoxColor(scenarioId, boxId): string`,
  `postScenarioPlan(scenarioId): Promise<{label,trunk,placed,unloadable,summary}>`.

- [ ] **Step 1: 실패하는 테스트부터 작성**

```js
// src/components/scenarioTheme.test.js
import { describe, expect, it } from "vitest";
import { scenarioBoxColor, scenarioTrunkColor, SCENARIOS } from "./scenarioTheme.js";

describe("SCENARIOS", () => {
  it("4개의 시나리오 id/label을 갖는다", () => {
    expect(SCENARIOS.map((s) => s.id)).toEqual(["delivery_truck", "warehouse", "cold_chain", "hazmat"]);
  });
});

describe("scenarioTrunkColor", () => {
  it("시나리오마다 서로 다른 고정 색을 반환한다", () => {
    const colors = SCENARIOS.map((s) => scenarioTrunkColor(s.id));
    expect(new Set(colors).size).toBe(4);
  });
});

describe("scenarioBoxColor", () => {
  it("창고 시나리오는 박스 id 접두사(소/대)로 색을 나눈다", () => {
    expect(scenarioBoxColor("warehouse", "소0")).toBe("#F2C94C");
    expect(scenarioBoxColor("warehouse", "대1")).toBe("#2F80ED");
  });

  it("위험물 시나리오는 hazard 종류별로 다른 경고색을 쓴다", () => {
    expect(scenarioBoxColor("hazmat", "산화제_드럼1")).toBe("#F2994A");
    expect(scenarioBoxColor("hazmat", "인화물_드럼1")).toBe("#EB5757");
    expect(scenarioBoxColor("hazmat", "일반박스1")).toBe("#9CA3AF");
  });

  it("택배/냉동 시나리오는 박스 id와 무관하게 단일 테마색을 쓴다", () => {
    expect(scenarioBoxColor("delivery_truck", "정류장1_박스")).toBe("#5A6472");
    expect(scenarioBoxColor("delivery_truck", "정류장4_박스")).toBe("#5A6472");
    expect(scenarioBoxColor("cold_chain", "냉동박스0")).toBe("#56CCF2");
  });
});
```

- [ ] **Step 2: 테스트가 실패하는지 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/components/scenarioTheme.test.js
```
Expected: FAIL - `Failed to resolve import "./scenarioTheme.js"`

- [ ] **Step 3: `scenarioTheme.js` 구현**

```js
// src/components/scenarioTheme.js
// 산업현장 시나리오 4개의 메타데이터 + 3D 뷰어 테마 색상 - 순수 함수라
// Scene3DViewer.jsx(<Canvas> 포함, jsdom 미검증)와 분리해서 여기서 직접
// 테스트한다. id는 백엔드 routes/scenarios.py의 SCENARIO_DEFS 키와 정확히
// 일치해야 한다.
export const SCENARIOS = [
  { id: "delivery_truck", label: "택배 배송 트럭" },
  { id: "warehouse", label: "창고/물류센터" },
  { id: "cold_chain", label: "냉동/냉장 물류" },
  { id: "hazmat", label: "위험물 창고" },
];

const TRUNK_COLORS = {
  delivery_truck: "#8A8F98",
  warehouse: "#F2C94C",
  cold_chain: "#2D9CDB",
  hazmat: "#F2994A",
};

export function scenarioTrunkColor(scenarioId) {
  return TRUNK_COLORS[scenarioId] || "#B8B8C4";
}

export function scenarioBoxColor(scenarioId, boxId) {
  if (scenarioId === "warehouse") {
    return boxId.startsWith("대") ? "#2F80ED" : "#F2C94C";
  }
  if (scenarioId === "hazmat") {
    if (boxId.startsWith("산화제")) return "#F2994A";
    if (boxId.startsWith("인화물")) return "#EB5757";
    return "#9CA3AF";
  }
  if (scenarioId === "cold_chain") return "#56CCF2";
  if (scenarioId === "delivery_truck") return "#5A6472";
  return "#4A90D9";
}
```

- [ ] **Step 4: `client.js`에 `postScenarioPlan` 추가**

파일 맨 아래(`postPickAndPlace` 다음)에 추가:

```js
// 산업현장 시나리오 미리보기 - routes/scenarios.py 참고. 요청 바디는 필요 없다.
export async function postScenarioPlan(scenarioId) {
  const resp = await fetch(`${BASE}/scenarios/${scenarioId}/plan`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({}),
  });
  return handleResponse(resp);
}
```

`client.test.js`의 마지막 `it` 다음에 추가(import 줄에 `postScenarioPlan`도
추가):

```js
  it("postScenarioPlan posts to /api/scenarios/<id>/plan", async () => {
    const fetchMock = vi.fn().mockResolvedValue({
      ok: true, json: async () => ({ label: "위험물 창고", trunk: {}, placed: [], unloadable: [], summary: {} }),
    });
    vi.stubGlobal("fetch", fetchMock);
    await postScenarioPlan("hazmat");
    expect(fetchMock).toHaveBeenCalledWith("/api/scenarios/hazmat/plan", expect.objectContaining({ method: "POST" }));
  });
```

- [ ] **Step 5: 테스트 통과 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/components/scenarioTheme.test.js src/api/client.test.js
```
Expected: PASS (scenarioTheme 4개 + client 6개 - 기존 5개 + 신규 1개)

- [ ] **Step 6: 커밋**

```bash
cd /home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU
git add isaacpjt/Cart2Trunk/web/frontend/src/components/scenarioTheme.js \
        isaacpjt/Cart2Trunk/web/frontend/src/components/scenarioTheme.test.js \
        isaacpjt/Cart2Trunk/web/frontend/src/api/client.js \
        isaacpjt/Cart2Trunk/web/frontend/src/api/client.test.js
git commit -m "web frontend: 시나리오 테마 색상 함수 + postScenarioPlan API 클라이언트 추가"
```

---

### Task 3: `Scene3DViewer.jsx`에 시나리오 버튼 + 미리보기 렌더링 추가

**Files:**
- Modify: `isaacpjt/Cart2Trunk/web/frontend/src/components/Scene3DViewer.jsx`
- Modify: `isaacpjt/Cart2Trunk/web/frontend/src/components/Scene3DViewer.module.css`

**Interfaces:**
- Consumes: Task 1의 `POST /api/scenarios/<id>/plan`(Task 2의
  `postScenarioPlan`으로 호출), Task 2의 `SCENARIOS`,
  `scenarioTrunkColor`, `scenarioBoxColor`, `sceneMeshes.jsx`의
  `BoundingBoxWireframe`(이미 존재, Task 3 재사용).
- Produces: 없음 (최종 UI 조립).

이 태스크는 `<Canvas>`가 포함된 컴포넌트를 수정하므로(프로젝트 관례상
jsdom 자동 테스트 없음) 자동화된 실패 테스트 단계 없이 직접 구현하고,
마지막에 전체 테스트 스위트(회귀 확인) + 수동 브라우저 확인으로 검증한다.

- [ ] **Step 1: `Scene3DViewer.module.css`에 시나리오 버튼/에러 스타일 추가**

파일 끝에 추가:

```css
.scenarioBar { display: flex; gap: 8px; margin-left: 8px; padding-left: 8px; border-left: 1px solid var(--color-border); }
.scenarioBar button { border: 1px solid var(--color-border); background: var(--color-canvas); border-radius: 6px; padding: 6px 12px; font-size: 13px; cursor: pointer; }
.scenarioActive { border-color: var(--color-accent) !important; color: var(--color-accent); font-weight: 700; }
.scenarioError { padding: 4px 16px; font-size: 12px; color: var(--color-danger); }
```

- [ ] **Step 2: `Scene3DViewer.jsx` import 블록 수정**

```jsx
import { useEffect, useMemo, useRef, useState } from "react";
import { Canvas } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";
import { usePlannerState } from "../state/PlannerContext.jsx";
import { colorForBoxId } from "../utils/color.js";
import { postScenarioPlan } from "../api/client.js";
import {
  toThreeCenter, TrunkWireframe, computeCartFootprint, CartWireframe, SceneBoxMesh, layoutStagingBoxes,
  BoundingBoxWireframe,
} from "./sceneMeshes.jsx";
import { SCENARIOS, scenarioTrunkColor, scenarioBoxColor } from "./scenarioTheme.js";
import styles from "./Scene3DViewer.module.css";
```

- [ ] **Step 3: 컴포넌트 본문에 시나리오 상태 + 핸들러 추가**

`export default function Scene3DViewer() {` 바로 다음(`const state = usePlannerState();` 다음 줄)에 추가:

```jsx
  // 산업현장 시나리오 미리보기 - state.result(지금 작업 중인 계획)는 전혀
  // 건드리지 않는 완전히 별도의 로컬 상태다. 활성화되면 아래 렌더링에서
  // 기존 trunk/placed 기반 씬 대신 이걸 보여준다.
  const [activeScenarioId, setActiveScenarioId] = useState(null);
  const [scenarioResult, setScenarioResult] = useState(null);
  const [scenarioError, setScenarioError] = useState(null);
  const [scenarioLoading, setScenarioLoading] = useState(false);

  const handleSelectScenario = async (id) => {
    setScenarioLoading(true);
    setScenarioError(null);
    try {
      const result = await postScenarioPlan(id);
      setActiveScenarioId(id);
      setScenarioResult(result);
    } catch (err) {
      setScenarioError(err.cause || err.message || "시나리오를 불러오지 못했습니다.");
    } finally {
      setScenarioLoading(false);
    }
  };

  const handleExitScenario = () => {
    setActiveScenarioId(null);
    setScenarioResult(null);
    setScenarioError(null);
  };
```

- [ ] **Step 4: 툴바에 시나리오 버튼 추가 + Before/After·재생 버튼 조건부 숨김**

기존:
```jsx
      <div className={styles.toolbar}>
        <div className={styles.stageBar}>
          {[["before", "Before"], ["after", "After"]].map(([value, text]) => (
            <button
              key={value}
              type="button"
              className={stage === value ? styles.stageActive : styles.stage}
              onClick={() => setStage(value)}
            >
              {text}
            </button>
          ))}
        </div>
        <div className={styles.presetBar}>
          {showPlaced && placed.length > 0 && (
            <button type="button" disabled={animating} onClick={handlePlayStepByStep}>
              {animating ? `▶ 재생 중 (${visibleCount}/${placed.length})` : "▶ 순서대로 재생"}
            </button>
          )}
          {Object.keys(CAMERA_PRESETS).map((name) => (
            <button key={name} type="button" onClick={() => setPreset(name)}>{name}</button>
          ))}
        </div>
      </div>
```

다음으로 교체:
```jsx
      <div className={styles.toolbar}>
        {!activeScenarioId && (
          <div className={styles.stageBar}>
            {[["before", "Before"], ["after", "After"]].map(([value, text]) => (
              <button
                key={value}
                type="button"
                className={stage === value ? styles.stageActive : styles.stage}
                onClick={() => setStage(value)}
              >
                {text}
              </button>
            ))}
          </div>
        )}
        <div className={styles.presetBar}>
          {!activeScenarioId && showPlaced && placed.length > 0 && (
            <button type="button" disabled={animating} onClick={handlePlayStepByStep}>
              {animating ? `▶ 재생 중 (${visibleCount}/${placed.length})` : "▶ 순서대로 재생"}
            </button>
          )}
          {Object.keys(CAMERA_PRESETS).map((name) => (
            <button key={name} type="button" onClick={() => setPreset(name)}>{name}</button>
          ))}
          <div className={styles.scenarioBar}>
            {SCENARIOS.map((s) => (
              <button
                key={s.id}
                type="button"
                disabled={scenarioLoading}
                className={activeScenarioId === s.id ? styles.scenarioActive : undefined}
                onClick={() => handleSelectScenario(s.id)}
              >
                {s.label}
              </button>
            ))}
            {activeScenarioId && (
              <button type="button" onClick={handleExitScenario}>실시간 계획으로 돌아가기</button>
            )}
          </div>
        </div>
      </div>
      {scenarioError && <span className={styles.scenarioError}>{scenarioError}</span>}
```

- [ ] **Step 5: Canvas 내부 렌더링을 시나리오/기본 두 갈래로 분기**

기존:
```jsx
        {trunk && <TrunkWireframe trunk={trunk} />}
        {cartFootprint && (
          <CartWireframe footprint={cartFootprint} entranceNearX={trunk.entrance_near_x !== false} />
        )}
        {state.result?.obstacles?.map((o) => (
          <SceneBoxMesh key={o.id} position={[o.x, o.y, o.z]}
                        dimensions={[o.width, o.depth, o.height]} color="#7f8c8d" />
        ))}
        {showPlaced && visiblePlaced.map((p) => (
          <SceneBoxMesh key={p.box_id} position={p.position} dimensions={p.dimensions}
                        color={colorForBoxId(p.box_id)} dashed={p.position[2] > 1e-6} />
        ))}
        {stagedBoxes.map((b) => (
          <SceneBoxMesh key={b.id} position={b.position} dimensions={b.dimensions}
                        color={colorForBoxId(b.id)} />
        ))}
```

다음으로 교체:
```jsx
        {activeScenarioId && scenarioResult ? (
          <>
            <BoundingBoxWireframe x={0} y={0} z={0}
              width={scenarioResult.trunk.width} depth={scenarioResult.trunk.depth}
              height={scenarioResult.trunk.height} color={scenarioTrunkColor(activeScenarioId)} />
            {scenarioResult.placed.map((p) => (
              <SceneBoxMesh key={p.box_id} position={p.position} dimensions={p.dimensions}
                            color={scenarioBoxColor(activeScenarioId, p.box_id)} />
            ))}
          </>
        ) : (
          <>
            {trunk && <TrunkWireframe trunk={trunk} />}
            {cartFootprint && (
              <CartWireframe footprint={cartFootprint} entranceNearX={trunk.entrance_near_x !== false} />
            )}
            {state.result?.obstacles?.map((o) => (
              <SceneBoxMesh key={o.id} position={[o.x, o.y, o.z]}
                            dimensions={[o.width, o.depth, o.height]} color="#7f8c8d" />
            ))}
            {showPlaced && visiblePlaced.map((p) => (
              <SceneBoxMesh key={p.box_id} position={p.position} dimensions={p.dimensions}
                            color={colorForBoxId(p.box_id)} dashed={p.position[2] > 1e-6} />
            ))}
            {stagedBoxes.map((b) => (
              <SceneBoxMesh key={b.id} position={b.position} dimensions={b.dimensions}
                            color={colorForBoxId(b.id)} />
            ))}
          </>
        )}
```

- [ ] **Step 6: 프론트엔드 전체 테스트 스위트로 회귀 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run
```
Expected: 전부 PASS (직전 계획 종료 시점 86개 + Task2(4+1=5) = 91개 전후)

- [ ] **Step 7: 빌드 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm run build
```
Expected: 에러 없이 성공

- [ ] **Step 8: 커밋**

```bash
cd /home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU
git add isaacpjt/Cart2Trunk/web/frontend/src/components/Scene3DViewer.jsx \
        isaacpjt/Cart2Trunk/web/frontend/src/components/Scene3DViewer.module.css
git commit -m "web frontend: Scene3DViewer에 산업현장 시나리오 버튼 4개 + 테마 미리보기 추가"
```

- [ ] **Step 9: 브라우저 수동 확인**

서버가 이미 떠 있다면(Vite HMR, 필요하면 강제 새로고침) 시뮬레이터 탭에서:
1. 3D 뷰어 툴바의 카메라 프리셋(front/side/top) 옆에 시나리오 버튼 4개가
   보이는지.
2. "위험물 창고" 클릭 → 몇 초 안에 트렁크(주황 와이어프레임) + 박스 3개
   (산화제=주황, 인화물=빨강, 일반=회색)가 보이는지, "실시간 계획으로
   돌아가기" 버튼이 나타나는지.
3. 그 버튼을 누르면 원래 있던(지금 작업 중이던) 계획 화면으로 정확히
   돌아가는지(사라지지 않았는지).
4. 나머지 3개 시나리오도 순서대로 확인, 카메라 프리셋(front/side/top)이
   시나리오 모드에서도 동작하는지.
5. 서버가 안 떠 있다면:
```bash
# 터미널 1
cd isaacpjt/Cart2Trunk/web/backend && source venv/bin/activate && python app.py
# 터미널 2
cd isaacpjt/Cart2Trunk/web/frontend && npm run dev
```
