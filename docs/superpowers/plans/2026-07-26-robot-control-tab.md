# 로봇 동작 트리거 탭(관제뷰) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Cart2Trunk 웹 UI에 "로봇 제어" 탭을 새로 추가한다 - 카트 스캔/트렁크
스캔/픽앤플레이스 트리거 버튼 3개 + 관제뷰(단계별 진행상태 + 시간순 로그).
지금은 실제 ROS2 연동 없이 백엔드가 항상 더미 성공 응답을 주지만, 버튼을
누르면 실제로 상태가 바뀌고 로그가 쌓이는 걸 눈으로 확인할 수 있다.

**Architecture:** 기존 시뮬레이터 탭(`PlannerContext`/`plannerReducer`)은 전혀
건드리지 않는다. 새 탭은 완전히 독립된 컴포넌트(`RobotControlPanel`)와 로컬
`useState`만으로 동작하고, 백엔드도 완전히 별도 네임스페이스
(`/api/robot/*`)의 새 블루프린트(`routes/robot.py`)로 분리한다. 탭 전환은
`react-router` 없이 `App.jsx`의 로컬 `useState` + 조건부 렌더링으로 구현한다.

**Tech Stack:** React 18 + Vite (프론트엔드, `web/frontend`), Flask
(백엔드, `web/backend`), Vitest + @testing-library/react (프론트 테스트),
pytest (백엔드 테스트).

## Global Constraints

- 산출물(코드 주석, 커밋 메시지, 테스트 설명, 이 문서 자체)은 전부 한국어로
  작성한다.
- 기존 시뮬레이터 탭의 `PlannerContext`/`plannerReducer`/기존 컴포넌트는
  이번 작업으로 일절 수정하지 않는다 (단, `App.jsx`는 탭 전환 배선을 위해
  수정한다 - 아래 Task 5).
- 별도 브랜치나 worktree를 만들지 않는다 - 지금 체크아웃된 `algorism`
  브랜치에 태스크마다 바로 커밋한다 (기존 세션 관례 그대로).
- 백엔드 테스트 실행: `cd isaacpjt/Cart2Trunk/web/backend && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./venv/bin/python3 -m pytest -q`
  (ROS2 humble의 전역 PYTHONPATH 때문에 이 환경변수 없이는 pytest 플러그인
  자동로드가 깨진다 - 이 저장소 `pytest.ini`에 이미 문서화된 관례).
- 프론트엔드 테스트 실행: `cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run`
- 각 신규 테스트 파일은 vitest globals가 꺼져 있는 이 프로젝트 관례에 따라
  `afterEach(() => { cleanup(); })`를 직접 import해서 호출해야 한다
  (`ControlPanel.test.jsx`, `VisionDataLoader.test.jsx`와 동일한 이유 -
  자동 cleanup이 동작하지 않아 테스트끼리 DOM이 섞인다).
- CSS는 `design-tokens.css`의 기존 CSS 변수(`--color-*`, `--font-*`)만
  쓴다 - 새 색상 값을 하드코딩하지 않는다.

---

### Task 1: 백엔드 - 로봇 트리거 더미 엔드포인트

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/backend/routes/robot.py`
- Create: `isaacpjt/Cart2Trunk/web/backend/tests/test_routes_robot.py`
- Modify: `isaacpjt/Cart2Trunk/web/backend/app.py`

**Interfaces:**
- Produces: `POST /api/robot/cart-scan`, `POST /api/robot/trunk-scan`,
  `POST /api/robot/pick-and-place` - 각각 body 없이 호출, 응답
  `{"status": "ok", "dummy": true, "message": "<단계명> 완료 (더미 - 실제 로봇 미연동)"}`
  (HTTP 200). `routes/robot.py`의 `DUMMY_DELAY_SECONDS`(모듈 상수, 기본 1.5)를
  monkeypatch하면 테스트에서 대기 시간을 줄일 수 있다.

- [ ] **Step 1: 실패하는 테스트부터 작성**

`isaacpjt/Cart2Trunk/web/backend/tests/test_routes_robot.py` 새로 작성:

```python
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from app import create_app
import routes.robot as robot_module


def _client():
    return create_app().test_client()


def test_cart_scan_returns_dummy_success(monkeypatch):
    monkeypatch.setattr(robot_module, "DUMMY_DELAY_SECONDS", 0)
    resp = _client().post("/api/robot/cart-scan")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["status"] == "ok"
    assert body["dummy"] is True
    assert "카트 스캔" in body["message"]


def test_trunk_scan_returns_dummy_success(monkeypatch):
    monkeypatch.setattr(robot_module, "DUMMY_DELAY_SECONDS", 0)
    resp = _client().post("/api/robot/trunk-scan")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["status"] == "ok"
    assert "트렁크 스캔" in body["message"]


def test_pick_and_place_returns_dummy_success(monkeypatch):
    monkeypatch.setattr(robot_module, "DUMMY_DELAY_SECONDS", 0)
    resp = _client().post("/api/robot/pick-and-place")
    assert resp.status_code == 200
    body = resp.get_json()
    assert body["status"] == "ok"
    assert "픽앤플레이스" in body["message"]


def test_get_method_not_allowed():
    # 실수로 GET으로 호출하는 걸 방지하는 회귀 테스트 - 반드시 POST여야 한다.
    resp = _client().get("/api/robot/cart-scan")
    assert resp.status_code == 405
```

- [ ] **Step 2: 테스트가 실패하는지 확인 (routes/robot.py가 아직 없음)**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/backend && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./venv/bin/python3 -m pytest tests/test_routes_robot.py -v
```
Expected: FAIL - `ModuleNotFoundError: No module named 'routes.robot'` (또는 import 단계 에러)

- [ ] **Step 3: `routes/robot.py` 구현**

```python
"""
routes/robot.py
POST /api/robot/cart-scan, /trunk-scan, /pick-and-place - 로봇(MSI2 - 신지완/
민결) 동작 트리거. ROS2 노드 구조가 아직 설계 중이라, 지금은 실제 서비스/
액션 호출 없이 DUMMY_DELAY_SECONDS만큼 대기한 뒤 항상 성공 응답을 돌려주는
더미다. 실제 연동 시 _dummy_trigger() 안의 TODO(MSI2) 자리에 실제 ROS2 호출을
넣고, 그 결과에 따라 상태/메시지를 채우도록 바꾸면 된다.
"""
import time

from flask import Blueprint, jsonify

robot_bp = Blueprint("robot", __name__)

# 실제 스캔/동작 시간을 흉내내는 더미 지연(초). 테스트에서는 이 값을
# monkeypatch로 0으로 낮춰서 느려지지 않게 한다.
DUMMY_DELAY_SECONDS = 1.5


def _dummy_trigger(step_name: str):
    time.sleep(DUMMY_DELAY_SECONDS)
    # TODO(MSI2): 여기에 실제 ROS2 서비스/액션 호출을 넣고, 그 결과에 따라
    # status/message를 채운다. 지금은 항상 성공하는 더미.
    return jsonify({
        "status": "ok",
        "dummy": True,
        "message": f"{step_name} 완료 (더미 - 실제 로봇 미연동)",
    })


@robot_bp.post("/api/robot/cart-scan")
def cart_scan():
    return _dummy_trigger("카트 스캔")


@robot_bp.post("/api/robot/trunk-scan")
def trunk_scan():
    return _dummy_trigger("트렁크 스캔")


@robot_bp.post("/api/robot/pick-and-place")
def pick_and_place():
    return _dummy_trigger("픽앤플레이스")
```

- [ ] **Step 4: `app.py`에 블루프린트 등록 + 개발 서버 threaded 옵션 추가**

`isaacpjt/Cart2Trunk/web/backend/app.py`에서 `vision_bp` 등록 바로 아래에 추가:

```python
    from routes.vision import vision_bp
    app.register_blueprint(vision_bp)

    from routes.robot import robot_bp
    app.register_blueprint(robot_bp)
```

그리고 파일 맨 아래 `if __name__ == "__main__":` 블록을:

```python
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
```

다음으로 변경 (더미 지연 1.5초 동안 다른 요청 - 예: 시뮬레이터 탭의 트렁크
맵 3초 폴링 - 이 밀리지 않도록 `threaded=True` 추가. 순수 `time.sleep` 대기라
스레드 경합 위험은 없다):

```python
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True, threaded=True)
```

- [ ] **Step 5: 테스트 통과 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/backend && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./venv/bin/python3 -m pytest tests/test_routes_robot.py -v
```
Expected: PASS (4개 테스트 전부)

- [ ] **Step 6: 백엔드 전체 테스트 스위트로 회귀 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/backend && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./venv/bin/python3 -m pytest -q
```
Expected: 기존 35개 + 신규 4개 = 39개 전부 PASS

- [ ] **Step 7: 커밋**

```bash
cd /home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU
git add isaacpjt/Cart2Trunk/web/backend/routes/robot.py \
        isaacpjt/Cart2Trunk/web/backend/tests/test_routes_robot.py \
        isaacpjt/Cart2Trunk/web/backend/app.py
git commit -m "web backend: 로봇 동작 트리거 더미 엔드포인트 3개 추가 (카트/트렁크 스캔, 픽앤플레이스)"
```

---

### Task 2: 프론트엔드 - API 클라이언트 함수 3개

**Files:**
- Modify: `isaacpjt/Cart2Trunk/web/frontend/src/api/client.js`
- Modify: `isaacpjt/Cart2Trunk/web/frontend/src/api/client.test.js`

**Interfaces:**
- Consumes: Task 1의 `POST /api/robot/cart-scan|trunk-scan|pick-and-place`
- Produces: `postCartScan()`, `postTrunkScan()`, `postPickAndPlace()` - 인자
  없이 호출, 성공 시 `{status, dummy, message}`를 resolve하는 Promise 반환,
  실패 시 `error_code`/`cause`/`action`을 담은 Error를 throw (기존
  `postPlan`/`postApprove`/`postSend`와 동일한 `handleResponse` 규약).

- [ ] **Step 1: 실패하는 테스트부터 작성**

`client.test.js`의 기존 `describe("api client", ...)` 블록 안, 마지막 `it`
다음에 추가:

```js
  it("postCartScan resolves with the dummy success payload", async () => {
    vi.stubGlobal("fetch", vi.fn().mockResolvedValue({
      ok: true, json: async () => ({ status: "ok", dummy: true, message: "카트 스캔 완료 (더미)" }),
    }));
    const result = await postCartScan();
    expect(result).toEqual({ status: "ok", dummy: true, message: "카트 스캔 완료 (더미)" });
  });

  it("postTrunkScan posts to /api/robot/trunk-scan", async () => {
    const fetchMock = vi.fn().mockResolvedValue({
      ok: true, json: async () => ({ status: "ok", dummy: true, message: "트렁크 스캔 완료 (더미)" }),
    });
    vi.stubGlobal("fetch", fetchMock);
    await postTrunkScan();
    expect(fetchMock).toHaveBeenCalledWith("/api/robot/trunk-scan", expect.objectContaining({ method: "POST" }));
  });

  it("postPickAndPlace throws an error carrying error_code on failure", async () => {
    vi.stubGlobal("fetch", vi.fn().mockResolvedValue({
      ok: false,
      json: async () => ({ error_code: "ROBOT_TRIGGER_FAILED", cause: "실패", action: "재시도하세요" }),
    }));
    await expect(postPickAndPlace()).rejects.toMatchObject({ error_code: "ROBOT_TRIGGER_FAILED" });
  });
```

그리고 파일 맨 위 import 줄을 다음으로 교체:

```js
import { fetchTrunkMaps, postPlan, postCartScan, postTrunkScan, postPickAndPlace } from "./client.js";
```

- [ ] **Step 2: 테스트가 실패하는지 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/api/client.test.js
```
Expected: FAIL - `postCartScan`/`postTrunkScan`/`postPickAndPlace`가
`client.js`에 없어서 import 에러 또는 `undefined is not a function`

- [ ] **Step 3: `client.js`에 함수 3개 추가**

파일 맨 아래(`postParseVisionCorners` 다음)에 추가:

```js
// 로봇(MSI2) 동작 트리거 - 지금은 백엔드가 실제 ROS2 없이 더미 응답만 준다
// (routes/robot.py 참고). 요청 바디는 필요 없다.
export async function postCartScan() {
  const resp = await fetch(`${BASE}/robot/cart-scan`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({}),
  });
  return handleResponse(resp);
}

export async function postTrunkScan() {
  const resp = await fetch(`${BASE}/robot/trunk-scan`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({}),
  });
  return handleResponse(resp);
}

export async function postPickAndPlace() {
  const resp = await fetch(`${BASE}/robot/pick-and-place`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({}),
  });
  return handleResponse(resp);
}
```

- [ ] **Step 4: 테스트 통과 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/api/client.test.js
```
Expected: PASS (기존 2개 + 신규 3개 = 5개)

- [ ] **Step 5: 커밋**

```bash
cd /home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU
git add isaacpjt/Cart2Trunk/web/frontend/src/api/client.js \
        isaacpjt/Cart2Trunk/web/frontend/src/api/client.test.js
git commit -m "web frontend: 로봇 트리거 API 클라이언트 함수 3개 추가"
```

---

### Task 3: 프론트엔드 - RobotControlPanel 컴포넌트 (버튼 3개 + 관제뷰)

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.jsx`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.module.css`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.test.jsx`

**Interfaces:**
- Consumes: Task 2의 `postCartScan()`, `postTrunkScan()`, `postPickAndPlace()`
  (각각 `{status, dummy, message}`를 resolve하거나 `{error_code,cause,action}`을
  담은 Error를 throw)
- Produces: `export default function RobotControlPanel()` - props 없음, 완전히
  독립된 컴포넌트(`PlannerContext` 미사용). 렌더링되는 `data-testid`:
  `trigger-cartScan`/`trigger-trunkScan`/`trigger-pickAndPlace`(버튼),
  `status-cartScan`/`status-trunkScan`/`status-pickAndPlace`(상태 뱃지,
  textContent는 "대기"/"진행중"/"완료" 중 하나), `robot-log-list`(로그 목록).

- [ ] **Step 1: 실패하는 테스트부터 작성**

`isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.test.jsx`
새로 작성:

```jsx
import { afterEach, describe, expect, it, vi } from "vitest";
import { cleanup, fireEvent, render, screen, waitFor } from "@testing-library/react";
import RobotControlPanel from "./RobotControlPanel.jsx";
import * as client from "../api/client.js";

afterEach(() => { cleanup(); vi.restoreAllMocks(); });

describe("RobotControlPanel", () => {
  it("클릭하면 상태가 대기->진행중->완료로 바뀌고 로그가 쌓인다", async () => {
    vi.spyOn(client, "postCartScan").mockResolvedValue({
      status: "ok", dummy: true, message: "카트 스캔 완료 (더미 - 실제 로봇 미연동)",
    });

    render(<RobotControlPanel />);
    expect(screen.getByTestId("status-cartScan").textContent).toBe("대기");

    fireEvent.click(screen.getByTestId("trigger-cartScan"));
    expect(screen.getByTestId("status-cartScan").textContent).toBe("진행중");
    expect(screen.getByTestId("trigger-cartScan")).toBeDisabled();

    await waitFor(() => expect(screen.getByTestId("status-cartScan").textContent).toBe("완료"));
    expect(screen.getByTestId("robot-log-list").textContent)
      .toContain("카트 스캔 완료 (더미 - 실제 로봇 미연동)");
  });

  it("3개 버튼은 서로 독립적으로 상태를 갖는다", async () => {
    vi.spyOn(client, "postTrunkScan").mockResolvedValue({
      status: "ok", dummy: true, message: "트렁크 스캔 완료 (더미)",
    });

    render(<RobotControlPanel />);
    fireEvent.click(screen.getByTestId("trigger-trunkScan"));
    await waitFor(() => expect(screen.getByTestId("status-trunkScan").textContent).toBe("완료"));

    expect(screen.getByTestId("status-cartScan").textContent).toBe("대기");
    expect(screen.getByTestId("status-pickAndPlace").textContent).toBe("대기");
  });

  it("요청이 실패하면 상태가 대기로 돌아가고 오류 로그가 남는다", async () => {
    vi.spyOn(client, "postPickAndPlace").mockRejectedValue(new Error("네트워크 오류"));

    render(<RobotControlPanel />);
    fireEvent.click(screen.getByTestId("trigger-pickAndPlace"));
    await waitFor(() => expect(screen.getByTestId("status-pickAndPlace").textContent).toBe("대기"));
    expect(screen.getByTestId("robot-log-list").textContent).toContain("픽앤플레이스 시작 요청 실패");
  });

  it("버튼을 다시 눌러서 반복 실행할 수 있다 (순서 강제 없음)", async () => {
    vi.spyOn(client, "postCartScan").mockResolvedValue({
      status: "ok", dummy: true, message: "카트 스캔 완료 (더미)",
    });

    render(<RobotControlPanel />);
    fireEvent.click(screen.getByTestId("trigger-cartScan"));
    await waitFor(() => expect(screen.getByTestId("status-cartScan").textContent).toBe("완료"));

    fireEvent.click(screen.getByTestId("trigger-cartScan"));
    expect(screen.getByTestId("status-cartScan").textContent).toBe("진행중");
    await waitFor(() => expect(client.postCartScan).toHaveBeenCalledTimes(2));
  });
});
```

- [ ] **Step 2: 테스트가 실패하는지 확인 (컴포넌트가 아직 없음)**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/components/RobotControlPanel.test.jsx
```
Expected: FAIL - `Failed to resolve import "./RobotControlPanel.jsx"`

- [ ] **Step 3: `RobotControlPanel.module.css` 작성**

```css
/* src/components/RobotControlPanel.module.css */
.panel { display: flex; flex-direction: column; gap: 20px; padding: 20px; height: 100%; min-height: 0; }

.steps { display: flex; gap: 16px; }
.step {
  flex: 1; background: var(--color-surface); border-radius: 12px; padding: 16px;
  display: flex; flex-direction: column; align-items: center; gap: 10px;
}
.step button {
  width: 100%; border: none; border-radius: 8px; padding: 12px 16px;
  font-size: 14px; font-weight: 700; cursor: pointer;
  background: var(--color-accent); color: white;
}
.step button:disabled { background: var(--color-segment-bg); color: var(--color-text-secondary); cursor: not-allowed; }

.status {
  font-size: 12px; font-weight: 700; padding: 3px 10px; border-radius: 999px;
  background: var(--color-segment-bg); color: var(--color-text-secondary);
}
.status[data-status="running"] { background: #FFF6DE; color: #B98900; }
.status[data-status="done"] { background: #E4F8EA; color: var(--color-success); }

.logPanel {
  background: var(--color-surface); border-radius: 12px; padding: 16px;
  flex: 1; min-height: 0; display: flex; flex-direction: column;
}
.logLabel { font-size: 11px; font-weight: 700; color: var(--color-text-secondary); text-transform: uppercase; }
.logList {
  list-style: none; margin: 8px 0 0; padding: 0; font-family: var(--font-mono);
  font-size: 12px; overflow-y: auto; display: flex; flex-direction: column; gap: 4px;
}
.logEmpty { color: var(--color-text-secondary); }
```

- [ ] **Step 4: `RobotControlPanel.jsx` 구현**

```jsx
// src/components/RobotControlPanel.jsx
// 로봇(MSI2) 동작 트리거 + 관제뷰. 시뮬레이터 탭의 PlannerContext와는 완전히
// 독립된 화면 - usePlannerState/usePlannerDispatch를 쓰지 않고 이 컴포넌트
// 안의 로컬 상태만으로 동작한다. 지금은 백엔드가 실제 ROS2를 호출하지 않고
// 더미 응답만 주므로, 버튼을 누르면 상태(대기->진행중->완료)와 로그가 실제로
// 바뀌는 것까지 눈으로 확인할 수 있다.
import { useState } from "react";
import { postCartScan, postTrunkScan, postPickAndPlace } from "../api/client.js";
import styles from "./RobotControlPanel.module.css";

const STEPS = [
  { key: "cartScan", label: "카트 스캔", trigger: postCartScan },
  { key: "trunkScan", label: "트렁크 스캔", trigger: postTrunkScan },
  { key: "pickAndPlace", label: "픽앤플레이스 시작", trigger: postPickAndPlace },
];

const STATUS_TEXT = { idle: "대기", running: "진행중", done: "완료" };

export default function RobotControlPanel() {
  const [statuses, setStatuses] = useState({ cartScan: "idle", trunkScan: "idle", pickAndPlace: "idle" });
  const [logs, setLogs] = useState([]);

  const appendLog = (message) => {
    const time = new Date().toLocaleTimeString("ko-KR", { hour12: false });
    setLogs((prev) => [{ time, message }, ...prev]);
  };

  const handleTrigger = async (step) => {
    setStatuses((prev) => ({ ...prev, [step.key]: "running" }));
    try {
      const resp = await step.trigger();
      setStatuses((prev) => ({ ...prev, [step.key]: "done" }));
      appendLog(resp.message);
    } catch {
      setStatuses((prev) => ({ ...prev, [step.key]: "idle" }));
      appendLog(`[오류] ${step.label} 요청 실패 - 백엔드가 실행 중인지 확인하세요`);
    }
  };

  return (
    <div className={styles.panel}>
      <div className={styles.steps}>
        {STEPS.map((step) => {
          const status = statuses[step.key];
          return (
            <div key={step.key} className={styles.step}>
              <button
                type="button"
                data-testid={`trigger-${step.key}`}
                disabled={status === "running"}
                onClick={() => handleTrigger(step)}
              >
                {step.label}
              </button>
              <span className={styles.status} data-status={status} data-testid={`status-${step.key}`}>
                {STATUS_TEXT[status]}
              </span>
            </div>
          );
        })}
      </div>

      <div className={styles.logPanel}>
        <label className={styles.logLabel}>관제 로그</label>
        <ul className={styles.logList} data-testid="robot-log-list">
          {logs.length === 0 && <li className={styles.logEmpty}>아직 실행된 동작이 없습니다.</li>}
          {logs.map((entry, i) => (
            <li key={i}>[{entry.time}] {entry.message}</li>
          ))}
        </ul>
      </div>
    </div>
  );
}
```

- [ ] **Step 5: 테스트 통과 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/components/RobotControlPanel.test.jsx
```
Expected: PASS (4개 테스트 전부)

- [ ] **Step 6: 커밋**

```bash
cd /home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU
git add isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.jsx \
        isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.module.css \
        isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.test.jsx
git commit -m "web frontend: RobotControlPanel 컴포넌트 추가 (트리거 버튼 3개 + 관제뷰)"
```

---

### Task 4: 프론트엔드 - TabBar 컴포넌트 (시뮬레이터/로봇 제어 전환)

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/TabBar.jsx`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/TabBar.module.css`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/TabBar.test.jsx`

**Interfaces:**
- Consumes: 없음 (순수 프레젠테이션 컴포넌트)
- Produces: `export default function TabBar({ activeTab, onSelect })` -
  `activeTab`은 `"simulator" | "robot"`, `onSelect(key)`는 클릭된 탭의 key를
  인자로 호출됨. `data-testid`: `tab-simulator`, `tab-robot`.

**참고**: `Scene3DViewer`(react-three-fiber/WebGL)가 포함된 `App.jsx` 전체를
jsdom에서 렌더링하는 건 이 프로젝트에서 하지 않는 패턴이다
(`Scene3DViewer.test.js`는 export된 순수 함수만 테스트하고 컴포넌트 자체는
렌더링하지 않음). 그래서 탭 전환 UI 로직은 이렇게 별도 컴포넌트로 뽑아서
독립적으로 테스트하고, `App.jsx` 배선(Task 5)은 자동 테스트 없이 수동으로
확인한다.

- [ ] **Step 1: 실패하는 테스트부터 작성**

`isaacpjt/Cart2Trunk/web/frontend/src/components/TabBar.test.jsx` 새로 작성:

```jsx
import { afterEach, describe, expect, it, vi } from "vitest";
import { cleanup, fireEvent, render, screen } from "@testing-library/react";
import TabBar from "./TabBar.jsx";

afterEach(() => { cleanup(); });

describe("TabBar", () => {
  it("현재 탭에 aria-current를 표시하고, 클릭하면 onSelect에 해당 키를 넘긴다", () => {
    const onSelect = vi.fn();
    render(<TabBar activeTab="simulator" onSelect={onSelect} />);

    expect(screen.getByTestId("tab-simulator").getAttribute("aria-current")).toBe("page");
    expect(screen.getByTestId("tab-robot").getAttribute("aria-current")).toBeNull();

    fireEvent.click(screen.getByTestId("tab-robot"));
    expect(onSelect).toHaveBeenCalledWith("robot");
  });

  it("activeTab이 robot이면 robot 탭에 aria-current가 표시된다", () => {
    render(<TabBar activeTab="robot" onSelect={() => {}} />);
    expect(screen.getByTestId("tab-robot").getAttribute("aria-current")).toBe("page");
    expect(screen.getByTestId("tab-simulator").getAttribute("aria-current")).toBeNull();
  });
});
```

- [ ] **Step 2: 테스트가 실패하는지 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/components/TabBar.test.jsx
```
Expected: FAIL - `Failed to resolve import "./TabBar.jsx"`

- [ ] **Step 3: `TabBar.module.css` 작성**

```css
/* src/components/TabBar.module.css */
.tabBar {
  display: flex; gap: 4px; padding: 0 28px; background: var(--color-surface);
  border-bottom: 1px solid var(--color-border);
}
.tab, .tabActive {
  border: none; background: transparent; padding: 12px 18px; font-size: 14px;
  font-weight: 600; cursor: pointer; color: var(--color-text-secondary);
  border-bottom: 2px solid transparent;
}
.tabActive { color: var(--color-accent); border-bottom-color: var(--color-accent); }
```

- [ ] **Step 4: `TabBar.jsx` 구현**

```jsx
// src/components/TabBar.jsx
// 시뮬레이터/로봇 제어 탭 전환 - react-router 없이 App.jsx의 로컬 상태로
// 전환하기 위한 순수 프레젠테이션 컴포넌트.
import styles from "./TabBar.module.css";

const TABS = [
  { key: "simulator", label: "시뮬레이터" },
  { key: "robot", label: "로봇 제어" },
];

export default function TabBar({ activeTab, onSelect }) {
  return (
    <nav className={styles.tabBar}>
      {TABS.map(({ key, label }) => (
        <button
          key={key}
          type="button"
          data-testid={`tab-${key}`}
          aria-current={activeTab === key ? "page" : undefined}
          className={activeTab === key ? styles.tabActive : styles.tab}
          onClick={() => onSelect(key)}
        >
          {label}
        </button>
      ))}
    </nav>
  );
}
```

- [ ] **Step 5: 테스트 통과 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/components/TabBar.test.jsx
```
Expected: PASS (2개 테스트 전부)

- [ ] **Step 6: 커밋**

```bash
cd /home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU
git add isaacpjt/Cart2Trunk/web/frontend/src/components/TabBar.jsx \
        isaacpjt/Cart2Trunk/web/frontend/src/components/TabBar.module.css \
        isaacpjt/Cart2Trunk/web/frontend/src/components/TabBar.test.jsx
git commit -m "web frontend: TabBar 컴포넌트 추가 (시뮬레이터/로봇 제어 탭 전환)"
```

---

### Task 5: App.jsx 배선 + 수동 검증 + 전체 테스트 회귀 확인

**Files:**
- Modify: `isaacpjt/Cart2Trunk/web/frontend/src/App.jsx`

**Interfaces:**
- Consumes: Task 3의 `RobotControlPanel`(props 없음), Task 4의
  `TabBar({activeTab, onSelect})`
- Produces: 없음 (최상위 조립 지점)

- [ ] **Step 1: `App.jsx` 전체를 아래 내용으로 교체**

```jsx
// src/App.jsx
import { useState } from "react";
import { PlannerProvider } from "./state/PlannerContext.jsx";
import { useResourceLoader } from "./hooks/useResourceLoader.js";
import { useDebouncedPlan } from "./hooks/useDebouncedPlan.js";
import Header from "./components/Header.jsx";
import TabBar from "./components/TabBar.jsx";
import ControlPanel from "./components/ControlPanel.jsx";
import SummaryCard from "./components/SummaryCard.jsx";
import Scene3DViewer from "./components/Scene3DViewer.jsx";
import BoxDetailPanel from "./components/BoxDetailPanel.jsx";
import LogPanel from "./components/LogPanel.jsx";
import RobotControlPanel from "./components/RobotControlPanel.jsx";
import styles from "./App.module.css";

function SimulatorBody() {
  return (
    <div className={styles.body}>
      <ControlPanel />
      <div className={styles.resultArea}>
        {/* 사용자 손그림 피드백 - 요약 카드("화면 1")는 좁게, 3D 뷰어("메인
            화면 - 2")는 넓게 나란히 배치한다. 결과 로그는 왼쪽 칸 안에
            같이 쌓고 margin-top:auto로 칸 맨 밑에 붙인다. */}
        <div className={styles.topRow}>
          <div className={styles.leftColumn}>
            <SummaryCard />
            <LogPanel />
          </div>
          <Scene3DViewer />
        </div>
        <BoxDetailPanel />
      </div>
    </div>
  );
}

function PlannerLayout() {
  // 폴링/디바운스 계산 훅은 탭과 무관하게 항상 켜둔다(SimulatorBody 안이
  // 아니라 여기서 호출) - 로봇 제어 탭을 보는 동안에도 트렁크 맵 목록이
  // 계속 갱신되고, 시뮬레이터 탭으로 돌아왔을 때 골라뒀던 값들이 탭
  // 전환마다 리셋되지 않는다(언마운트되지 않으므로).
  useResourceLoader();
  useDebouncedPlan();
  const [activeTab, setActiveTab] = useState("simulator");

  return (
    <div className={styles.layout}>
      <Header />
      <TabBar activeTab={activeTab} onSelect={setActiveTab} />
      {activeTab === "simulator" && <SimulatorBody />}
      {activeTab === "robot" && <RobotControlPanel />}
    </div>
  );
}

export default function App() {
  return (
    <PlannerProvider>
      <PlannerLayout />
    </PlannerProvider>
  );
}
```

- [ ] **Step 2: 프론트엔드 전체 테스트 스위트로 회귀 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run
```
Expected: 전부 PASS (기존 68개 + Task 2~4에서 추가된 9개 = 77개)

- [ ] **Step 3: 빌드 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm run build
```
Expected: 에러 없이 성공

- [ ] **Step 4: 수동 브라우저 검증**

`App.jsx`는 `Scene3DViewer`(WebGL)를 포함하고 있어 jsdom 자동화 테스트로
전체를 렌더링하지 않는다(이 프로젝트 관례 - Task 4 참고). 대신 실제로 두
서버를 띄워서 눈으로 확인한다:

```bash
# 터미널 1
cd isaacpjt/Cart2Trunk/web/backend && source venv/bin/activate && python app.py
# 터미널 2
cd isaacpjt/Cart2Trunk/web/frontend && npm run dev
```

브라우저(`http://localhost:5173`)에서:
1. 기본으로 "시뮬레이터" 탭이 보이는지 확인 (기존 화면 그대로).
2. "로봇 제어" 탭 클릭 → 카트 스캔/트렁크 스캔/픽앤플레이스 시작 버튼 3개와
   "관제 로그" 영역이 보이는지 확인.
3. "카트 스캔" 클릭 → 버튼이 잠깐 비활성화되고 상태 뱃지가 "진행중" → 약
   1.5초 뒤 "완료"로 바뀌는지, 관제 로그에 한 줄 추가되는지 확인.
4. 나머지 2개 버튼도 동일하게 확인 (3개가 서로 독립적으로 동작해야 함).
5. "시뮬레이터" 탭으로 돌아가서 기존 화면(트렁크 맵 선택 등)이 그대로
   유지되는지 확인 - 탭 전환으로 리셋되면 안 됨.

문제 없으면 두 서버를 종료한다 (Ctrl+C).

- [ ] **Step 5: 커밋**

```bash
cd /home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU
git add isaacpjt/Cart2Trunk/web/frontend/src/App.jsx
git commit -m "web frontend: App.jsx에 시뮬레이터/로봇 제어 탭 전환 배선"
```

- [ ] **Step 6: 백엔드 + 프론트엔드 전체 테스트 스위트 최종 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/backend && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./venv/bin/python3 -m pytest -q
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run
```
Expected: 백엔드 39개, 프론트엔드 77개 전부 PASS
