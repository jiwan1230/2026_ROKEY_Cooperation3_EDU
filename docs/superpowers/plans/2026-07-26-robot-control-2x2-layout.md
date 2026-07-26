# 로봇 제어 탭 2x2 레이아웃(카메라 + 관제 로그) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** "로봇 제어" 탭을 3칸 가로 배치에서 2x2 그리드로 재배치한다 - 위에
트렁크/카트 Scan, 아래에 카메라 플레이스홀더 + Pick&Place + 관제 로그.

**Architecture:** `RobotControlPanel`이 로그 상태(`logs`)를 소유하고
CSS Grid로 5개 자식을 배치한다. `ScanViewerPanel`/`PickPlacePanel`은 새
선택적 `onLog` prop으로 자기 상태 전이를 부모에게 보고한다(안 넘기면
아무 일도 안 하는 기본값이라 기존 테스트는 그대로 통과).

**Tech Stack:** React 18 + Vite, Vitest + @testing-library/react.

## Global Constraints

- 산출물(주석, 커밋 메시지, 테스트 설명)은 전부 한국어로 작성한다.
- `algorism` 브랜치에 태스크마다 바로 커밋한다.
- 테스트: `cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run`
- CSS는 `design-tokens.css`의 기존 CSS 변수만 쓴다.
- `onLog`는 기본값 `() => {}`인 선택적 prop이라, 안 넘기면 기존
  `ScanViewerPanel.test.jsx`/`PickPlacePanel.test.jsx`의 모든 테스트가
  수정 없이 그대로 통과해야 한다(하위 호환).

---

### Task 1: `CameraPreviewPanel.jsx` (신규, 정적 플레이스홀더)

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/CameraPreviewPanel.jsx`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/CameraPreviewPanel.module.css`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/CameraPreviewPanel.test.jsx`

**Interfaces:**
- Produces: `export default function CameraPreviewPanel()` - props 없음.

- [ ] **Step 1: 실패하는 테스트부터 작성**

```jsx
// src/components/CameraPreviewPanel.test.jsx
import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";
import CameraPreviewPanel from "./CameraPreviewPanel.jsx";

afterEach(() => { cleanup(); });

describe("CameraPreviewPanel", () => {
  it("카메라 미연동 안내 문구를 보여준다", () => {
    render(<CameraPreviewPanel />);
    expect(screen.getByText("로봇 카메라 실시간")).toBeInTheDocument();
    expect(screen.getByText("카메라 미연동 - 더미 화면")).toBeInTheDocument();
  });
});
```

- [ ] **Step 2: 테스트가 실패하는지 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/components/CameraPreviewPanel.test.jsx
```
Expected: FAIL - `Failed to resolve import "./CameraPreviewPanel.jsx"`

- [ ] **Step 3: `CameraPreviewPanel.module.css` 작성**

```css
/* src/components/CameraPreviewPanel.module.css */
.panel {
  background: var(--color-surface); border-radius: 12px; padding: 16px;
  display: flex; flex-direction: column; gap: 10px; min-width: 0;
}
.title { font-size: 13px; font-weight: 700; color: var(--color-text-primary); }
.placeholder {
  flex: 1; min-height: 140px; border-radius: 8px; background: var(--color-canvas);
  display: flex; flex-direction: column; align-items: center; justify-content: center; gap: 6px;
  color: var(--color-text-secondary); font-size: 13px;
}
.icon { font-size: 28px; }
```

- [ ] **Step 4: `CameraPreviewPanel.jsx` 구현**

```jsx
// src/components/CameraPreviewPanel.jsx
// 로봇에 달린 비전 카메라 실시간 화면 - 아직 실제 카메라가 연결되지 않아
// 정적 플레이스홀더만 보여준다. 나중에 실제 영상 스트림이 연결되면 이 안의
// placeholder div를 <video>/이미지 스트림으로 교체하면 된다.
import styles from "./CameraPreviewPanel.module.css";

export default function CameraPreviewPanel() {
  return (
    <div className={styles.panel}>
      <span className={styles.title}>로봇 카메라 실시간</span>
      <div className={styles.placeholder}>
        <span className={styles.icon}>📷</span>
        <span>카메라 미연동 - 더미 화면</span>
      </div>
    </div>
  );
}
```

- [ ] **Step 5: 테스트 통과 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/components/CameraPreviewPanel.test.jsx
```
Expected: PASS

- [ ] **Step 6: 커밋**

```bash
cd /home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU
git add isaacpjt/Cart2Trunk/web/frontend/src/components/CameraPreviewPanel.jsx \
        isaacpjt/Cart2Trunk/web/frontend/src/components/CameraPreviewPanel.module.css \
        isaacpjt/Cart2Trunk/web/frontend/src/components/CameraPreviewPanel.test.jsx
git commit -m "web frontend: CameraPreviewPanel 추가 (로봇 카메라 정적 더미 플레이스홀더)"
```

---

### Task 2: `RobotLogPanel.jsx` (신규, 순수 렌더링)

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/RobotLogPanel.jsx`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/RobotLogPanel.module.css`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/RobotLogPanel.test.jsx`

**Interfaces:**
- Produces: `export default function RobotLogPanel({ logs })` - `logs`는
  `{time: string, message: string}[]` (최신이 배열 맨 앞이라고 가정하고 그
  순서 그대로 렌더링). `data-testid="robot-log-list"`.

- [ ] **Step 1: 실패하는 테스트부터 작성**

```jsx
// src/components/RobotLogPanel.test.jsx
import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";
import RobotLogPanel from "./RobotLogPanel.jsx";

afterEach(() => { cleanup(); });

describe("RobotLogPanel", () => {
  it("로그가 없으면 안내 문구를 보여준다", () => {
    render(<RobotLogPanel logs={[]} />);
    expect(screen.getByText("아직 실행된 동작이 없습니다.")).toBeInTheDocument();
  });

  it("logs를 받은 순서 그대로(최신이 위) 렌더링한다", () => {
    render(<RobotLogPanel logs={[
      { time: "10:00:02", message: "트렁크 스캔 완료" },
      { time: "10:00:00", message: "트렁크 스캔 시작" },
    ]} />);
    const list = screen.getByTestId("robot-log-list");
    expect(list.textContent).toContain("[10:00:02] 트렁크 스캔 완료");
    expect(list.textContent).toContain("[10:00:00] 트렁크 스캔 시작");
  });
});
```

- [ ] **Step 2: 테스트가 실패하는지 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/components/RobotLogPanel.test.jsx
```
Expected: FAIL - `Failed to resolve import "./RobotLogPanel.jsx"`

- [ ] **Step 3: `RobotLogPanel.module.css` 작성**

```css
/* src/components/RobotLogPanel.module.css */
.panel {
  background: var(--color-surface); border-radius: 12px; padding: 16px;
  display: flex; flex-direction: column; min-width: 0;
}
.label { font-size: 11px; font-weight: 700; color: var(--color-text-secondary); text-transform: uppercase; }
.logList {
  list-style: none; margin: 8px 0 0; padding: 0; font-family: var(--font-mono);
  font-size: 12px; overflow-y: auto; display: flex; flex-direction: column; gap: 4px;
}
.logEmpty { color: var(--color-text-secondary); }
```

- [ ] **Step 4: `RobotLogPanel.jsx` 구현**

```jsx
// src/components/RobotLogPanel.jsx
// 관제 로그 - 트렁크/카트 Scan, Pick&Place가 onLog로 보고한 이벤트를
// 시간순(최신이 위, 부모가 이미 그 순서로 넘겨줌)으로 보여주기만 하는
// 순수 렌더링 컴포넌트.
import styles from "./RobotLogPanel.module.css";

export default function RobotLogPanel({ logs }) {
  return (
    <div className={styles.panel}>
      <label className={styles.label}>관제 로그</label>
      <ul className={styles.logList} data-testid="robot-log-list">
        {logs.length === 0 && <li className={styles.logEmpty}>아직 실행된 동작이 없습니다.</li>}
        {logs.map((entry, i) => (
          <li key={i}>[{entry.time}] {entry.message}</li>
        ))}
      </ul>
    </div>
  );
}
```

- [ ] **Step 5: 테스트 통과 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/components/RobotLogPanel.test.jsx
```
Expected: PASS (2개 테스트 전부)

- [ ] **Step 6: 커밋**

```bash
cd /home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU
git add isaacpjt/Cart2Trunk/web/frontend/src/components/RobotLogPanel.jsx \
        isaacpjt/Cart2Trunk/web/frontend/src/components/RobotLogPanel.module.css \
        isaacpjt/Cart2Trunk/web/frontend/src/components/RobotLogPanel.test.jsx
git commit -m "web frontend: RobotLogPanel 추가 (관제 로그 순수 렌더링 컴포넌트)"
```

---

### Task 3: `ScanViewerPanel.jsx`에 `onLog` prop 추가

**Files:**
- Modify: `isaacpjt/Cart2Trunk/web/frontend/src/components/ScanViewerPanel.jsx`
- Modify: `isaacpjt/Cart2Trunk/web/frontend/src/components/ScanViewerPanel.test.jsx`

**Interfaces:**
- Produces: `ScanViewerPanel({ kind, onLog })` - `onLog`는 옵션(기본값
  `() => {}`), `(message: string) => void`. 성공 시
  `"${KIND_LABELS[kind]} 완료"`, 실패 시 `"[오류] ${KIND_LABELS[kind]} 요청 실패"`로
  1회씩 호출된다.

- [ ] **Step 1: 실패하는 테스트부터 추가**

`ScanViewerPanel.test.jsx`의 마지막 `it`(요청 실패 테스트) 바로 다음에 추가:

```jsx
  it("성공하면 onLog가 완료 메시지로 호출된다", async () => {
    vi.spyOn(client, "postTrunkScan").mockResolvedValue({ status: "ok", dummy: true, message: "완료" });
    const onLog = vi.fn();

    render(<ScanViewerPanel kind="trunk" onLog={onLog} />);
    fireEvent.click(screen.getByTestId("trigger-trunk"));
    await waitFor(() => expect(screen.getByTestId("status-trunk").textContent).toBe("완료"));

    expect(onLog).toHaveBeenCalledWith("트렁크 스캔 완료");
  });

  it("실패하면 onLog가 오류 메시지로 호출된다", async () => {
    vi.spyOn(client, "postCartScan").mockRejectedValue(new Error("네트워크 오류"));
    const onLog = vi.fn();

    render(<ScanViewerPanel kind="cart" onLog={onLog} />);
    fireEvent.click(screen.getByTestId("trigger-cart"));
    await waitFor(() => expect(screen.getByTestId("status-cart").textContent).toBe("대기"));

    expect(onLog).toHaveBeenCalledWith("[오류] 카트 스캔 요청 실패");
  });
```

- [ ] **Step 2: 테스트가 실패하는지 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/components/ScanViewerPanel.test.jsx
```
Expected: 기존 4개는 PASS, 새 2개는 FAIL(`onLog`가 아직 호출 안 됨 -
`expect(jest.fn()).toHaveBeenCalledWith(...)` 실패)

- [ ] **Step 3: `ScanViewerPanel.jsx` 수정**

함수 시그니처를 바꾸고:
```jsx
export default function ScanViewerPanel({ kind, onLog = () => {} }) {
```

`handleTrigger`를 다음으로 교체:
```jsx
  const handleTrigger = async () => {
    setStatus("running");
    try {
      // TODO(비전팀 연동 시): 여기서 실제 스캔 결과를 받으면 DUMMY_TRUNK/
      // DUMMY_CART_BOXES 대신 그 값을 써야 한다. 지금은 성공 여부만 보고
      // 더미 message 내용은 쓰지 않는다.
      await callScanTrigger(kind);
      setStatus("done");
      onLog(`${KIND_LABELS[kind]} 완료`);
    } catch {
      setStatus("idle");
      onLog(`[오류] ${KIND_LABELS[kind]} 요청 실패`);
    }
  };
```

- [ ] **Step 4: 테스트 통과 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/components/ScanViewerPanel.test.jsx
```
Expected: PASS (6개 전부 - 기존 4개 + 신규 2개)

- [ ] **Step 5: 커밋**

```bash
cd /home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU
git add isaacpjt/Cart2Trunk/web/frontend/src/components/ScanViewerPanel.jsx \
        isaacpjt/Cart2Trunk/web/frontend/src/components/ScanViewerPanel.test.jsx
git commit -m "web frontend: ScanViewerPanel에 onLog prop 추가 (관제 로그 연동용)"
```

---

### Task 4: `PickPlacePanel.jsx`에 `onLog` prop 추가

**Files:**
- Modify: `isaacpjt/Cart2Trunk/web/frontend/src/components/PickPlacePanel.jsx`
- Modify: `isaacpjt/Cart2Trunk/web/frontend/src/components/PickPlacePanel.test.jsx`

**Interfaces:**
- Produces: `PickPlacePanel({ onLog })` - `onLog`는 옵션(기본값
  `() => {}`), `(message: string) => void`. 시작 시 `"픽앤플레이스 시작"`,
  마지막 단계 도달 시 `"픽앤플레이스 완료"`로 1회씩 호출된다.

- [ ] **Step 1: 실패하는 테스트부터 추가**

`PickPlacePanel.test.jsx`의 마지막 `it` 다음에 추가:

```jsx
  it("시작하면 onLog가 시작 메시지로, 끝나면 완료 메시지로 호출된다", async () => {
    vi.useFakeTimers({ shouldAdvanceTime: true });
    vi.spyOn(client, "postPickAndPlace").mockResolvedValue({ status: "ok", dummy: true, message: "완료" });
    const onLog = vi.fn();

    render(<PickPlacePanel onLog={onLog} />);
    fireEvent.click(screen.getByTestId("trigger-pickAndPlace"));
    expect(onLog).toHaveBeenCalledWith("픽앤플레이스 시작");

    await act(async () => { await vi.advanceTimersByTimeAsync(700 * 6); });
    expect(onLog).toHaveBeenCalledWith("픽앤플레이스 완료");

    vi.useRealTimers();
  });
```

- [ ] **Step 2: 테스트가 실패하는지 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/components/PickPlacePanel.test.jsx
```
Expected: 기존 3개는 PASS, 새 1개는 FAIL

- [ ] **Step 3: `PickPlacePanel.jsx` 수정**

함수 시그니처를 바꾸고:
```jsx
export default function PickPlacePanel({ onLog = () => {} }) {
```

`advance`와 `handleStart`를 다음으로 교체:
```jsx
  const advance = (nextIndex) => {
    if (nextIndex >= PICK_PLACE_STEPS.length) {
      setRunState("stopped");
      onLog("픽앤플레이스 완료");
      return;
    }
    setStepIndex(nextIndex);
    timerRef.current = setTimeout(() => advance(nextIndex + 1), STEP_INTERVAL_MS);
  };

  const handleStart = () => {
    setRunState("running");
    setStepIndex(-1);
    // TODO(로봇 연동 시): 여기 응답으로 실제 진행 상태가 오면, 아래 advance()의
    // 프론트 자체 타이머 시뮬레이션 대신 그 값을 그대로 반영하도록 바꾼다.
    postPickAndPlace().catch(() => {});
    onLog("픽앤플레이스 시작");
    advance(0);
  };
```

- [ ] **Step 4: 테스트 통과 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/components/PickPlacePanel.test.jsx
```
Expected: PASS (4개 전부 - 기존 3개 + 신규 1개)

- [ ] **Step 5: 커밋**

```bash
cd /home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU
git add isaacpjt/Cart2Trunk/web/frontend/src/components/PickPlacePanel.jsx \
        isaacpjt/Cart2Trunk/web/frontend/src/components/PickPlacePanel.test.jsx
git commit -m "web frontend: PickPlacePanel에 onLog prop 추가 (관제 로그 연동용)"
```

---

### Task 5: `RobotControlPanel.jsx` 2x2 그리드로 재배치 + 전체 검증

**Files:**
- Modify: `isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.jsx`
- Modify: `isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.module.css`

**Interfaces:**
- Consumes: Task 1의 `CameraPreviewPanel()`, Task 2의
  `RobotLogPanel({logs})`, Task 3의 `ScanViewerPanel({kind, onLog})`,
  Task 4의 `PickPlacePanel({onLog})`.
- Produces: 없음 (조립 지점, `App.jsx`는 변경 불필요 - 여전히
  `<RobotControlPanel />`로 그대로 사용)

- [ ] **Step 1: `RobotControlPanel.module.css`를 2x2 그리드로 교체**

```css
/* src/components/RobotControlPanel.module.css */
.panel {
  display: grid; gap: 16px; padding: 20px; height: 100%; min-height: 0;
  grid-template-columns: repeat(4, 1fr);
  grid-template-rows: 1fr 1fr;
  grid-template-areas:
    "trunk  trunk  cart   cart"
    "camera pick   pick   log";
}
.trunk { grid-area: trunk; }
.cart { grid-area: cart; }
.camera { grid-area: camera; }
.pick { grid-area: pick; }
.log { grid-area: log; }

@media (max-width: 1100px) {
  .panel {
    grid-template-columns: 1fr;
    grid-template-areas: "trunk" "cart" "camera" "pick" "log";
  }
}
```

- [ ] **Step 2: `RobotControlPanel.jsx`를 2x2 컨테이너로 교체**

```jsx
// src/components/RobotControlPanel.jsx
// 로봇 제어 탭 - 2x2 그리드(트렁크/카트 Scan 위, 카메라+Pick&Place+관제
// 로그 아래). 관제 로그 상태를 여기서 소유하고, 자식들이 onLog로 보고하는
// 이벤트를 시간순으로 쌓는다.
import { useState } from "react";
import ScanViewerPanel from "./ScanViewerPanel.jsx";
import PickPlacePanel from "./PickPlacePanel.jsx";
import CameraPreviewPanel from "./CameraPreviewPanel.jsx";
import RobotLogPanel from "./RobotLogPanel.jsx";
import styles from "./RobotControlPanel.module.css";

export default function RobotControlPanel() {
  const [logs, setLogs] = useState([]);

  const appendLog = (message) => {
    const time = new Date().toLocaleTimeString("ko-KR", { hour12: false });
    setLogs((prev) => [{ time, message }, ...prev]);
  };

  return (
    <div className={styles.panel}>
      <div className={styles.trunk}><ScanViewerPanel kind="trunk" onLog={appendLog} /></div>
      <div className={styles.cart}><ScanViewerPanel kind="cart" onLog={appendLog} /></div>
      <div className={styles.camera}><CameraPreviewPanel /></div>
      <div className={styles.pick}><PickPlacePanel onLog={appendLog} /></div>
      <div className={styles.log}><RobotLogPanel logs={logs} /></div>
    </div>
  );
}
```

`ScanViewerPanel`/`PickPlacePanel`이 이미 `flex: 1; min-width: 0;`으로
자기 자신을 채우도록 만들어져 있으므로(`ScanViewerPanel.module.css`,
`PickPlacePanel.module.css`), 그리드 셀 안에 `<div className={styles.trunk}>`
같은 래퍼로 한 번 더 감싸서 그리드 영역에 맞춰 크기가 늘어나게 한다 - 자식
컴포넌트 내부는 손대지 않는다.

- [ ] **Step 3: 프론트엔드 전체 테스트 스위트로 회귀 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run
```
Expected: 전부 PASS. 직전 계획(2026-07-26-robot-control-scan-viewers.md)
종료 시점 80개 + Task1(1) + Task2(2) + Task3(2) + Task4(1) = 86개 전후가
되어야 한다("전부 PASS"가 핵심).

- [ ] **Step 4: 빌드 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm run build
```
Expected: 에러 없이 성공

- [ ] **Step 5: 커밋**

```bash
cd /home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU
git add isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.jsx \
        isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.module.css
git commit -m "web frontend: RobotControlPanel을 2x2 그리드로 재배치 (카메라 + 관제 로그 추가)"
```

- [ ] **Step 6: 브라우저 수동 확인 (가능한 범위)**

`ScanViewerPanel`이 `<Canvas>`를 포함해 jsdom으로 전체를 검증할 수 없으므로,
이미 떠 있는 백엔드/프론트엔드 서버가 있다면(Vite HMR로 자동 반영, 새 파일이
많이 생겼으니 필요하면 브라우저에서 강제 새로고침) "로봇 제어" 탭에서
2x2 레이아웃(트렁크/카트 Scan 위, 카메라/Pick&Place/관제 로그 아래)이 보이고,
트렁크/카트/픽앤플레이스 버튼을 누르면 오른쪽 아래 "관제 로그"에 시간순으로
줄이 쌓이는지 확인한다. 서버가 안 떠 있다면:

```bash
# 터미널 1
cd isaacpjt/Cart2Trunk/web/backend && source venv/bin/activate && python app.py
# 터미널 2
cd isaacpjt/Cart2Trunk/web/frontend && npm run dev
```

