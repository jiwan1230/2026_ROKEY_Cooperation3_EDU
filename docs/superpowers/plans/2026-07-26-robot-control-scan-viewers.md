# 로봇 제어 탭 3칸 레이아웃(트렁크/카트 3D 미리보기 + Pick&Place 진행현황) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** "로봇 제어" 탭을 손그림 레이아웃대로 3칸(트렁크 Scan / 카트 Scan /
Pick&Place)으로 재구성한다. 트렁크/카트 칸은 원본·전처리 토글이 달린 더미
3D 미리보기를, Pick&Place 칸은 진행현황 바 + 현재 작업 텍스트 + Run/Stop/
Warning 상태를 보여준다.

**Architecture:** 시뮬레이터 탭의 `Scene3DViewer.jsx`에서 `usePlannerState`에
의존하지 않는 순수 렌더링 부품을 `sceneMeshes.jsx`로 추출해 공용화한다.
더미 데이터(`robotDummyData.js`)는 실제 스캔 데이터와 동일한 shape로 만들어
나중에 그 자리만 교체하면 되게 한다. 트렁크/카트 칸은 뼈대가 동일해서
`kind` prop 하나로 공유하는 `ScanViewerPanel.jsx`로, Pick&Place는 완전히
다른 구조라 별도 `PickPlacePanel.jsx`로 분리한다.

**Tech Stack:** React 18 + Vite, react-three-fiber/drei(3D), Flask(백엔드,
이번 계획에서는 변경 없음), Vitest + @testing-library/react.

## Global Constraints

- 산출물(주석, 커밋 메시지, 테스트 설명)은 전부 한국어로 작성한다.
- `algorism` 브랜치에 태스크마다 바로 커밋한다 (별도 브랜치/worktree 없음).
- 프론트엔드 테스트: `cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run`
- CSS는 `design-tokens.css`의 기존 CSS 변수만 쓴다.
- **함수 참조를 모듈 로드 시점에 배열/객체 리터럴에 미리 캡쳐해두면
  `vi.spyOn(client, "postXxx")` 목이 반영되지 않는다** (이전 태스크에서 실제로
  겪은 버그) - 반드시 호출 시점에 `postCartScan`/`postTrunkScan`/
  `postPickAndPlace` 식별자를 직접 참조해야 한다. 이번 계획의 모든 컴포넌트가
  이 패턴을 따른다.
- `react-three-fiber`의 `<Canvas>`를 포함한 컴포넌트는 이 프로젝트에서 jsdom
  으로 직접 렌더링하지 않는다(`Scene3DViewer.jsx`가 지금까지 그래왔던 관례).
  `ScanViewerPanel.jsx`는 예외적으로 `@react-three/fiber`/`@react-three/drei`/
  `./sceneMeshes.jsx`를 `vi.mock`으로 대체해서 테스트한다(아래 Task 3 참고) -
  실제 WebGL 렌더링 자체는 여전히 안 하지만, 상태 전이/토글 로직은 검증한다.

---

### Task 1: `sceneMeshes.jsx` 추출 (Scene3DViewer 리팩터, 동작 변경 없음)

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/sceneMeshes.jsx`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/sceneMeshes.test.js`
- Modify: `isaacpjt/Cart2Trunk/web/frontend/src/components/Scene3DViewer.jsx`
- Delete: `isaacpjt/Cart2Trunk/web/frontend/src/components/Scene3DViewer.test.js`
  (모든 테스트가 추출된 함수 대상이라 `sceneMeshes.test.js`로 완전히
  옮겨가고, 이 파일엔 아무것도 안 남는다)

**Interfaces:**
- Produces: `sceneMeshes.jsx`가 export하는 것 - `toThreeCenter(x,y,z,w,d,h)`,
  `TrunkWireframe({trunk})`, `computeCartFootprint(fullBoxLayout)`,
  `CartWireframe({footprint, entranceNearX})`, `SceneBoxMesh({position,
  dimensions, color, dashed})`, `layoutStagingBoxes(boxSpecs, trunk)`. 전부
  props/인자만 받고 전역 상태(`usePlannerState`)에 의존하지 않는다.

- [ ] **Step 1: `sceneMeshes.jsx` 생성 - Scene3DViewer.jsx에서 순수 부품만 그대로 옮김**

```jsx
// src/components/sceneMeshes.jsx
// Scene3DViewer.jsx에서 추출한 순수 3D 렌더링 부품 - props만 받고
// PlannerContext 등 전역 상태에 의존하지 않는다. 시뮬레이터 탭(Scene3DViewer)과
// 로봇 제어 탭(ScanViewerPanel)이 같이 쓴다.

const STAGING_GAP = 0.05; // 대기 박스끼리, 그리고 트렁크 입구 면과의 간격(m)
const STAGING_OFFSET = 0.3; // 트렁크 입구 면에서 대기 구역까지의 거리(m)
const CART_GRID_SPAN_COLUMNS = 3; // 카트 폭 방향(옆으로) 칸 수
const CART_GRID_DEPTH_ROWS = 2; // 카트 안쪽(입구 방향) 칸 수

export function layoutStagingBoxes(boxSpecs, trunk) {
  if (!trunk || boxSpecs.length === 0) return [];
  const entranceNearX = trunk.entrance_near_x !== false;

  const cellWidth = Math.max(...boxSpecs.map((b) => b.width));
  const cellDepth = Math.max(...boxSpecs.map((b) => b.depth));
  const numColumns = CART_GRID_SPAN_COLUMNS * CART_GRID_DEPTH_ROWS;
  const columnHeights = new Array(numColumns).fill(0);

  return boxSpecs.map((b, i) => {
    const col = i % numColumns;
    const depthIndex = Math.floor(col / CART_GRID_SPAN_COLUMNS);
    const spanIndex = col % CART_GRID_SPAN_COLUMNS;

    const rowDepthOffset = depthIndex * (cellWidth + STAGING_GAP);
    const spanOffset = spanIndex * (cellDepth + STAGING_GAP);

    const cellOriginX = entranceNearX
      ? -(STAGING_OFFSET + rowDepthOffset + cellWidth)
      : trunk.width + STAGING_OFFSET + rowDepthOffset;
    const x = cellOriginX + (cellWidth - b.width) / 2;
    const y = spanOffset + (cellDepth - b.depth) / 2;
    const z = columnHeights[col];
    columnHeights[col] += b.height;

    return { id: b.id, position: [x, y, z], dimensions: [b.width, b.depth, b.height] };
  });
}

export function toThreeCenter(x, y, z, w, d, h) {
  return [x + w / 2, z + h / 2, y + d / 2];
}

const TRUNK_LID_COLOR = "#9C1F1F";
const TRUNK_TAILLIGHT_COLOR = "#C62828";
const TRUNK_LID_TILT = (32 * Math.PI) / 180;

export function TrunkWireframe({ trunk }) {
  const entranceNearX = trunk.entrance_near_x !== false;
  const outwardSign = entranceNearX ? -1 : 1;
  const entranceX = entranceNearX ? 0 : trunk.width;
  const tailX = entranceNearX ? -0.015 : trunk.width - 0.015;
  const lidLength = trunk.height * 0.8;
  const lidRotationZ = Math.PI / 2 + outwardSign * TRUNK_LID_TILT;

  return (
    <group>
      <mesh position={toThreeCenter(0, 0, 0, trunk.width, trunk.depth, trunk.height)}>
        <boxGeometry args={[trunk.width, trunk.height, trunk.depth]} />
        <meshBasicMaterial color="#6E6E73" wireframe />
      </mesh>

      {[0.08, Math.max(0.08, trunk.depth - 0.22)].map((yStart, i) => (
        <mesh key={i} position={toThreeCenter(tailX, yStart, trunk.height - 0.14, 0.03, 0.14, 0.1)}>
          <boxGeometry args={[0.03, 0.1, 0.14]} />
          <meshStandardMaterial color={TRUNK_TAILLIGHT_COLOR} emissive={TRUNK_TAILLIGHT_COLOR} emissiveIntensity={0.4} />
        </mesh>
      ))}

      <group position={toThreeCenter(entranceX, 0, trunk.height, 0, trunk.depth, 0)} rotation={[0, 0, lidRotationZ]}>
        <mesh position={[lidLength / 2, 0, 0]}>
          <boxGeometry args={[lidLength, 0.05, trunk.depth]} />
          <meshStandardMaterial color={TRUNK_LID_COLOR} roughness={0.5} metalness={0.2} />
        </mesh>
      </group>
    </group>
  );
}

const CART_MARGIN = 0.1;
const CART_WALL_CLEARANCE = 0.12;
const CART_WHEEL_RADIUS = 0.045;

export function computeCartFootprint(fullBoxLayout) {
  if (!fullBoxLayout || fullBoxLayout.length === 0) return null;
  const minX = Math.min(...fullBoxLayout.map((b) => b.position[0]));
  const maxX = Math.max(...fullBoxLayout.map((b) => b.position[0] + b.dimensions[0]));
  const maxY = Math.max(...fullBoxLayout.map((b) => b.position[1] + b.dimensions[1]));
  const maxHeight = Math.max(...fullBoxLayout.map((b) => b.dimensions[2]));
  return {
    minX: minX - CART_MARGIN,
    maxX: maxX + CART_MARGIN,
    minY: -CART_MARGIN,
    maxY: maxY + CART_MARGIN,
    height: maxHeight + CART_WALL_CLEARANCE,
  };
}

export function CartWireframe({ footprint, entranceNearX }) {
  const { minX, maxX, minY, maxY, height } = footprint;
  const width = maxX - minX;
  const depth = maxY - minY;
  const handleX = entranceNearX ? minX : maxX;
  const wheelY = [minY + CART_WHEEL_RADIUS, maxY - CART_WHEEL_RADIUS];
  const wheelX = [minX + CART_WHEEL_RADIUS, maxX - CART_WHEEL_RADIUS];

  return (
    <group>
      <mesh position={toThreeCenter(minX, minY, 0, width, depth, height)}>
        <boxGeometry args={[width, height, depth]} />
        <meshBasicMaterial color="#A2724F" wireframe />
      </mesh>
      {wheelX.flatMap((x) => wheelY.map((y) => (
        <mesh key={`${x}-${y}`} position={toThreeCenter(x, y, CART_WHEEL_RADIUS, 0, 0, 0)}>
          <sphereGeometry args={[CART_WHEEL_RADIUS, 12, 12]} />
          <meshStandardMaterial color="#3A3A3C" />
        </mesh>
      )))}
      <mesh position={toThreeCenter(handleX, minY, height * 0.75, 0, depth, 0)}>
        <boxGeometry args={[0.03, 0.03, depth]} />
        <meshStandardMaterial color="#3A3A3C" />
      </mesh>
    </group>
  );
}

export function SceneBoxMesh({ position, dimensions, color, dashed }) {
  const [w, d, h] = dimensions;
  const [x, y, z] = position;
  return (
    <group position={toThreeCenter(x, y, z, w, d, h)}>
      <mesh>
        <boxGeometry args={[w, h, d]} />
        <meshStandardMaterial color={color} transparent opacity={dashed ? 0.55 : 0.9} />
      </mesh>
      <mesh>
        <boxGeometry args={[w, h, d]} />
        <meshBasicMaterial color="#1D1D1F" wireframe />
      </mesh>
    </group>
  );
}
```

- [ ] **Step 2: `sceneMeshes.test.js` 생성 - 기존 `Scene3DViewer.test.js` 내용을 import 경로만 바꿔 그대로 옮김**

```js
// src/components/sceneMeshes.test.js
import { describe, expect, it } from "vitest";
import { computeCartFootprint, layoutStagingBoxes, toThreeCenter } from "./sceneMeshes.jsx";

describe("toThreeCenter", () => {
  it("maps our z-up corner coords to three.js y-up center coords", () => {
    expect(toThreeCenter(0, 0, 0, 0.4, 0.3, 0.2)).toEqual([0.2, 0.1, 0.15]);
  });

  it("keeps depth(y) mapped to three.js z axis", () => {
    const [, , threeZ] = toThreeCenter(0, 1.0, 0, 0.2, 0.2, 0.2);
    expect(threeZ).toBeCloseTo(1.1);
  });
});

describe("layoutStagingBoxes", () => {
  const trunk = { width: 1.0, depth: 1.0, height: 0.5, entrance_near_x: true };

  it("places boxes outside the entrance-side face (negative x) when entrance is near x=0", () => {
    const boxes = [{ id: "A", width: 0.3, depth: 0.2, height: 0.15 }];
    const [staged] = layoutStagingBoxes(boxes, trunk);
    expect(staged.position[0]).toBeLessThan(0);
  });

  it("places boxes beyond the far face (x > trunk.width) when entrance is near x=width", () => {
    const farTrunk = { ...trunk, entrance_near_x: false };
    const boxes = [{ id: "A", width: 0.3, depth: 0.2, height: 0.15 }];
    const [staged] = layoutStagingBoxes(boxes, farTrunk);
    expect(staged.position[0]).toBeGreaterThan(farTrunk.width);
  });

  it("returns an empty array when there is no trunk yet", () => {
    expect(layoutStagingBoxes([{ id: "A", width: 0.3, depth: 0.2, height: 0.15 }], undefined)).toEqual([]);
  });

  it("places up to 6 boxes (3x2 grid) side by side on the floor without overlapping footprints", () => {
    const boxes = Array.from({ length: 6 }, (_, i) => ({ id: `B${i}`, width: 0.2, depth: 0.2, height: 0.1 }));
    const staged = layoutStagingBoxes(boxes, trunk);
    staged.forEach((b) => expect(b.position[2]).toBe(0));
    const cells = staged.map((b) => `${b.position[0]},${b.position[1]}`);
    expect(new Set(cells).size).toBe(6);
  });

  it("wraps the 7th box onto the 1st column, stacking flush on top with no gap", () => {
    const boxes = Array.from({ length: 7 }, (_, i) => ({ id: `B${i}`, width: 0.2, depth: 0.2, height: 0.1 }));
    const staged = layoutStagingBoxes(boxes, trunk);
    const first = staged[0];
    const seventh = staged[6];
    expect(seventh.position[0]).toBe(first.position[0]);
    expect(seventh.position[1]).toBe(first.position[1]);
    expect(seventh.position[2]).toBeCloseTo(first.position[2] + first.dimensions[2]);
  });

  it("centers a smaller box within its grid cell when box sizes vary", () => {
    const boxes = [
      { id: "Big", width: 0.4, depth: 0.4, height: 0.1 },
      { id: "Small", width: 0.2, depth: 0.2, height: 0.1 },
    ];
    const [, small] = layoutStagingBoxes(boxes, trunk);
    expect(small.position[1]).toBeCloseTo(0.55);
  });
});

describe("computeCartFootprint", () => {
  const trunk = { width: 0.85, depth: 1.25, height: 0.5, entrance_near_x: true };

  it("returns null when there are no boxes to stage", () => {
    expect(computeCartFootprint([])).toBeNull();
  });

  it("bounds the full staged layout with margin on every side", () => {
    const boxes = [
      { id: "Large", width: 0.5, depth: 0.35, height: 0.3 },
      { id: "Medium", width: 0.4, depth: 0.3, height: 0.25 },
    ];
    const layout = layoutStagingBoxes(boxes, trunk);
    const footprint = computeCartFootprint(layout);
    expect(footprint.minX).toBeCloseTo(-0.9);
    expect(footprint.maxX).toBeCloseTo(-0.2);
    expect(footprint.minY).toBeCloseTo(-0.1);
    expect(footprint.maxY).toBeCloseTo(0.825);
    expect(footprint.height).toBeCloseTo(0.42);
  });
});
```

- [ ] **Step 3: 기존 `Scene3DViewer.test.js` 삭제**

```bash
rm isaacpjt/Cart2Trunk/web/frontend/src/components/Scene3DViewer.test.js
```

- [ ] **Step 4: `Scene3DViewer.jsx` 수정 - 추출된 부분을 import로 교체**

파일 맨 위 import 블록을 다음으로 교체(`toThreeCenter` 등을 더 이상 여기서
정의하지 않고 import):

```jsx
import { useEffect, useMemo, useRef, useState } from "react";
import { Canvas } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";
import { usePlannerState } from "../state/PlannerContext.jsx";
import { colorForBoxId } from "../utils/color.js";
import {
  toThreeCenter, TrunkWireframe, computeCartFootprint, CartWireframe, SceneBoxMesh, layoutStagingBoxes,
} from "./sceneMeshes.jsx";
import styles from "./Scene3DViewer.module.css";
```

그리고 원래 파일에서 `STAGING_GAP` 상수부터 `SceneBoxMesh` 함수 끝까지
(원래 8번째 줄 `const STAGING_GAP = ...`부터 `SceneBoxMesh` 함수의 닫는
`}`까지 - `sceneMeshes.jsx`로 옮긴 부분 전체)를 통째로 삭제한다. 그 아래
`const STEP_DELAY_MS = 700;`부터 파일 끝(`export default function
Scene3DViewer() {...}`)까지는 **한 글자도 바꾸지 않고 그대로 둔다** - 이
함수 안에서 `toThreeCenter`/`TrunkWireframe`/`CartWireframe`/`SceneBoxMesh`/
`layoutStagingBoxes`/`computeCartFootprint`를 참조하는 부분은 이제 로컬 정의
대신 위에서 import한 것을 그대로 쓴다.

- [ ] **Step 5: 프론트엔드 전체 테스트로 회귀 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run
```
Expected: 전부 PASS. `sceneMeshes.test.js`(9개)가 새로 생기고
`Scene3DViewer.test.js`(9개)가 없어져서 총 테스트 수는 그대로 유지되어야
한다.

- [ ] **Step 6: 빌드 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm run build
```
Expected: 에러 없이 성공 (Scene3DViewer.jsx가 여전히 정상적으로 번들링됨)

- [ ] **Step 7: 커밋**

```bash
cd /home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU
git add isaacpjt/Cart2Trunk/web/frontend/src/components/sceneMeshes.jsx \
        isaacpjt/Cart2Trunk/web/frontend/src/components/sceneMeshes.test.js \
        isaacpjt/Cart2Trunk/web/frontend/src/components/Scene3DViewer.jsx \
        isaacpjt/Cart2Trunk/web/frontend/src/components/Scene3DViewer.test.js
git commit -m "web frontend: Scene3DViewer의 순수 렌더링 부품을 sceneMeshes.jsx로 추출 (동작 변경 없음)"
```

---

### Task 2: `robotDummyData.js` 생성

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/robotDummyData.js`

**Interfaces:**
- Produces: `DUMMY_TRUNK`(`{width, depth, height, entrance_near_x}` 형태의
  상수 객체), `DUMMY_CART_BOXES`(`{id, width, depth, height}` 배열),
  `PICK_PLACE_STEPS`(`{pct, label}` 배열, 6개).

이 태스크는 순수 데이터 상수만 정의하므로 별도 테스트 없이(로직이 없음)
바로 작성한다.

- [ ] **Step 1: 파일 작성**

```js
// src/components/robotDummyData.js
// 로봇 제어 탭에서 쓰는 더미 데이터. 실제 스캔/로봇 데이터와 동일한 shape로
// 맞춰뒀다 - 비전 팀 데이터가 들어오면 이 값들 대신 실제 값을 그 자리에
// 넣기만 하면 된다(shape는 그대로 유지).

// TODO(비전팀 연동 시): 트렁크 스캔 결과(algorism_bridge.load_trunk_from_world_map
// 결과와 같은 shape - {width, depth, height, entrance_near_x})로 교체.
export const DUMMY_TRUNK = {
  width: 0.65, depth: 1.10, height: 0.45, entrance_near_x: true,
};

// TODO(비전팀 연동 시): vision_adapter.boxes_from_vision_corners()가 돌려주는
// 것과 같은 shape({id, width, depth, height, rests_on_id})의 실제 카트 스캔
// 결과로 교체.
export const DUMMY_CART_BOXES = [
  { id: "Large", width: 0.30, depth: 0.30, height: 0.20 },
  { id: "Medium", width: 0.20, depth: 0.13, height: 0.15 },
  { id: "Small", width: 0.15, depth: 0.13, height: 0.10 },
];

// TODO(로봇 연동 시): ROS2에서 실제 픽앤플레이스 진행 상태를 받아오면 이
// 고정 배열 대신 그 값을 그대로 pct/label로 매핑해서 쓰면 된다.
export const PICK_PLACE_STEPS = [
  { pct: 15, label: "박스1 pick 접근" },
  { pct: 35, label: "박스1 파지" },
  { pct: 55, label: "박스1 트렁크로 이동" },
  { pct: 70, label: "박스1 place" },
  { pct: 85, label: "박스2 pick 접근" },
  { pct: 100, label: "완료" },
];
```

- [ ] **Step 2: 커밋**

```bash
cd /home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU
git add isaacpjt/Cart2Trunk/web/frontend/src/components/robotDummyData.js
git commit -m "web frontend: 로봇 제어 탭용 더미 트렁크/박스/픽앤플레이스 단계 데이터 추가"
```

---

### Task 3: `ScanViewerPanel.jsx` (트렁크/카트 Scan 공용 3D 미리보기)

**Files:**
- Modify: `isaacpjt/Cart2Trunk/web/frontend/src/components/sceneMeshes.jsx`
  (새 헬퍼 `BoundingBoxWireframe` 추가)
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/ScanViewerPanel.jsx`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/ScanViewerPanel.module.css`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/ScanViewerPanel.test.jsx`

**Interfaces:**
- Consumes: Task 1의 `toThreeCenter`, `TrunkWireframe`, `CartWireframe`,
  `SceneBoxMesh`, `layoutStagingBoxes`, `computeCartFootprint` (+ 이 태스크에서
  추가하는 `BoundingBoxWireframe`), Task 2의 `DUMMY_TRUNK`,
  `DUMMY_CART_BOXES`, 기존 `postTrunkScan()`/`postCartScan()`
  (`web/frontend/src/api/client.js`).
- Produces: `export default function ScanViewerPanel({ kind })` -
  `kind`는 `"trunk" | "cart"`. `data-testid`: `trigger-${kind}`,
  `status-${kind}`, `${kind}-viewmode-raw`, `${kind}-viewmode-processed`.

- [ ] **Step 1: `sceneMeshes.jsx`에 `BoundingBoxWireframe` 헬퍼 추가**

`sceneMeshes.jsx` 맨 끝(`SceneBoxMesh` 함수 다음)에 추가:

```jsx
// "원본"(가공 전) 더미 뷰용 - 개별 물체 구분 없이 전체를 감싸는 단순
// 바운딩박스 와이어프레임 하나만 그린다.
export function BoundingBoxWireframe({ x, y, z, width, depth, height, color = "#B8B8C4" }) {
  return (
    <mesh position={toThreeCenter(x, y, z, width, depth, height)}>
      <boxGeometry args={[width, height, depth]} />
      <meshBasicMaterial color={color} wireframe />
    </mesh>
  );
}
```

- [ ] **Step 2: 실패하는 테스트부터 작성**

```jsx
// src/components/ScanViewerPanel.test.jsx
import { afterEach, describe, expect, it, vi } from "vitest";
import { cleanup, fireEvent, render, screen, waitFor } from "@testing-library/react";
import ScanViewerPanel from "./ScanViewerPanel.jsx";
import * as client from "../api/client.js";

// <Canvas>는 실제 WebGL이 필요해서 jsdom에서 직접 렌더링하지 않는 프로젝트
// 관례(Scene3DViewer.jsx)를 따르되, 이 컴포넌트의 상태 전이/토글 로직은
// r3f 프리미티브를 간단한 div로 대체해서 검증한다.
vi.mock("@react-three/fiber", () => ({ Canvas: ({ children }) => <div data-testid="canvas">{children}</div> }));
vi.mock("@react-three/drei", () => ({ Grid: () => null, OrbitControls: () => null }));
vi.mock("./sceneMeshes.jsx", () => ({
  toThreeCenter: () => [0, 0, 0],
  TrunkWireframe: () => <div data-testid="trunk-mesh" />,
  CartWireframe: () => <div data-testid="cart-mesh" />,
  SceneBoxMesh: () => <div data-testid="box-mesh" />,
  BoundingBoxWireframe: () => <div data-testid="raw-mesh" />,
  layoutStagingBoxes: () => [{ id: "Large", position: [0, 0, 0], dimensions: [0.3, 0.3, 0.2] }],
  computeCartFootprint: () => ({ minX: 0, maxX: 1, minY: 0, maxY: 1, height: 0.5 }),
}));

afterEach(() => { cleanup(); vi.restoreAllMocks(); });

describe("ScanViewerPanel", () => {
  it("완료 전에는 3D 콘텐츠가 안 보이다가, 완료되면 기본(원본) 콘텐츠가 나타난다", async () => {
    vi.spyOn(client, "postTrunkScan").mockResolvedValue({ status: "ok", dummy: true, message: "트렁크 스캔 완료" });

    render(<ScanViewerPanel kind="trunk" />);
    expect(screen.getByTestId("status-trunk").textContent).toBe("대기");
    expect(screen.queryByTestId("raw-mesh")).toBeNull();
    expect(screen.queryByTestId("trunk-mesh")).toBeNull();

    fireEvent.click(screen.getByTestId("trigger-trunk"));
    expect(screen.getByTestId("status-trunk").textContent).toBe("진행중");
    expect(screen.getByTestId("trigger-trunk")).toBeDisabled();

    await waitFor(() => expect(screen.getByTestId("status-trunk").textContent).toBe("완료"));
    expect(screen.getByTestId("raw-mesh")).toBeInTheDocument();
    expect(screen.queryByTestId("trunk-mesh")).toBeNull();
  });

  it("전처리 토글을 누르면 완료 후 디테일 렌더링(TrunkWireframe)으로 바뀐다", async () => {
    vi.spyOn(client, "postTrunkScan").mockResolvedValue({ status: "ok", dummy: true, message: "완료" });

    render(<ScanViewerPanel kind="trunk" />);
    fireEvent.click(screen.getByTestId("trigger-trunk"));
    await waitFor(() => expect(screen.getByTestId("status-trunk").textContent).toBe("완료"));

    fireEvent.click(screen.getByTestId("trunk-viewmode-processed"));
    expect(screen.getByTestId("trunk-mesh")).toBeInTheDocument();
    expect(screen.queryByTestId("raw-mesh")).toBeNull();
  });

  it("카트 칸은 전처리 모드에서 CartWireframe와 박스 메쉬를 함께 보여준다", async () => {
    vi.spyOn(client, "postCartScan").mockResolvedValue({ status: "ok", dummy: true, message: "완료" });

    render(<ScanViewerPanel kind="cart" />);
    fireEvent.click(screen.getByTestId("trigger-cart"));
    await waitFor(() => expect(screen.getByTestId("status-cart").textContent).toBe("완료"));

    fireEvent.click(screen.getByTestId("cart-viewmode-processed"));
    expect(screen.getByTestId("cart-mesh")).toBeInTheDocument();
    expect(screen.getByTestId("box-mesh")).toBeInTheDocument();
  });

  it("요청이 실패하면 상태가 대기로 돌아간다", async () => {
    vi.spyOn(client, "postCartScan").mockRejectedValue(new Error("네트워크 오류"));

    render(<ScanViewerPanel kind="cart" />);
    fireEvent.click(screen.getByTestId("trigger-cart"));
    await waitFor(() => expect(screen.getByTestId("status-cart").textContent).toBe("대기"));
  });
});
```

- [ ] **Step 3: 테스트가 실패하는지 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/components/ScanViewerPanel.test.jsx
```
Expected: FAIL - `Failed to resolve import "./ScanViewerPanel.jsx"`

- [ ] **Step 4: `ScanViewerPanel.module.css` 작성**

```css
/* src/components/ScanViewerPanel.module.css */
.panel {
  flex: 1; background: var(--color-surface); border-radius: 12px; padding: 16px;
  display: flex; flex-direction: column; gap: 10px; min-width: 0;
}
.header { display: flex; align-items: center; justify-content: space-between; }
.title { font-size: 13px; font-weight: 700; color: var(--color-text-primary); }
.toggle { display: flex; background: var(--color-segment-bg); border-radius: 8px; padding: 2px; }
.toggleBtn, .toggleActive {
  border: none; background: transparent; border-radius: 6px; padding: 4px 10px;
  font-size: 12px; font-weight: 600; cursor: pointer; color: var(--color-text-secondary);
}
.toggleActive { background: var(--color-accent); color: white; }

.canvasWrap { height: 220px; border-radius: 8px; overflow: hidden; background: var(--color-canvas); }

.panel > button {
  border: none; border-radius: 8px; padding: 10px 16px; font-size: 13px; font-weight: 700;
  cursor: pointer; background: var(--color-accent); color: white;
}
.panel > button:disabled { background: var(--color-segment-bg); color: var(--color-text-secondary); cursor: not-allowed; }

.status {
  align-self: flex-start; font-size: 12px; font-weight: 700; padding: 3px 10px; border-radius: 999px;
  background: var(--color-segment-bg); color: var(--color-text-secondary);
}
.status[data-status="running"] { background: #FFF6DE; color: #B98900; }
.status[data-status="done"] { background: #E4F8EA; color: var(--color-success); }
```

- [ ] **Step 5: `ScanViewerPanel.jsx` 구현**

```jsx
// src/components/ScanViewerPanel.jsx
// 트렁크 Scan / 카트 Scan 칸 - 뼈대(토글+상태+버튼+3D 캔버스)가 완전히
// 같아서 kind prop으로 내용만 갈아끼운다. 시뮬레이터 탭의 PlannerContext와
// 무관하게 독립적으로 동작한다.
import { useState } from "react";
import { Canvas } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";
import { postCartScan, postTrunkScan } from "../api/client.js";
import {
  TrunkWireframe, CartWireframe, SceneBoxMesh, BoundingBoxWireframe,
  layoutStagingBoxes, computeCartFootprint,
} from "./sceneMeshes.jsx";
import { DUMMY_TRUNK, DUMMY_CART_BOXES } from "./robotDummyData.js";
import styles from "./ScanViewerPanel.module.css";

const KIND_LABELS = { trunk: "트렁크 스캔", cart: "카트 스캔" };
const STATUS_TEXT = { idle: "대기", running: "진행중", done: "완료" };

// callTrigger처럼 호출 시점에 postTrunkScan/postCartScan을 직접 참조한다 -
// 미리 객체에 캡쳐해두면 vi.spyOn 목이 반영 안 되는 문제(RobotControlPanel.jsx
// 참고)를 피하기 위함.
function callScanTrigger(kind) {
  return kind === "trunk" ? postTrunkScan() : postCartScan();
}

function RawTrunkPreview() {
  return (
    <BoundingBoxWireframe x={0} y={0} z={0}
      width={DUMMY_TRUNK.width} depth={DUMMY_TRUNK.depth} height={DUMMY_TRUNK.height} />
  );
}

function ProcessedTrunkPreview() {
  return <TrunkWireframe trunk={DUMMY_TRUNK} />;
}

function RawCartPreview() {
  const layout = layoutStagingBoxes(DUMMY_CART_BOXES, DUMMY_TRUNK);
  const footprint = computeCartFootprint(layout);
  if (!footprint) return null;
  const { minX, maxX, minY, maxY, height } = footprint;
  return <BoundingBoxWireframe x={minX} y={minY} z={0} width={maxX - minX} depth={maxY - minY} height={height} />;
}

function ProcessedCartPreview() {
  const layout = layoutStagingBoxes(DUMMY_CART_BOXES, DUMMY_TRUNK);
  const footprint = computeCartFootprint(layout);
  return (
    <>
      {footprint && <CartWireframe footprint={footprint} entranceNearX={DUMMY_TRUNK.entrance_near_x} />}
      {layout.map((b) => (
        <SceneBoxMesh key={b.id} position={b.position} dimensions={b.dimensions} color="#4A90D9" />
      ))}
    </>
  );
}

const SCENE_CONTENT = {
  trunk: { raw: RawTrunkPreview, processed: ProcessedTrunkPreview },
  cart: { raw: RawCartPreview, processed: ProcessedCartPreview },
};

export default function ScanViewerPanel({ kind }) {
  const [status, setStatus] = useState("idle");
  const [viewMode, setViewMode] = useState("raw");

  const handleTrigger = async () => {
    setStatus("running");
    try {
      // TODO(비전팀 연동 시): 여기서 실제 스캔 결과를 받으면 DUMMY_TRUNK/
      // DUMMY_CART_BOXES 대신 그 값을 써야 한다. 지금은 성공 여부만 보고
      // 더미 message 내용은 쓰지 않는다.
      await callScanTrigger(kind);
      setStatus("done");
    } catch {
      setStatus("idle");
    }
  };

  const Content = status === "done" ? SCENE_CONTENT[kind][viewMode] : null;

  return (
    <div className={styles.panel}>
      <div className={styles.header}>
        <span className={styles.title}>{KIND_LABELS[kind]}</span>
        <div className={styles.toggle}>
          <button type="button" data-testid={`${kind}-viewmode-raw`}
                  className={viewMode === "raw" ? styles.toggleActive : styles.toggleBtn}
                  onClick={() => setViewMode("raw")}>원본</button>
          <button type="button" data-testid={`${kind}-viewmode-processed`}
                  className={viewMode === "processed" ? styles.toggleActive : styles.toggleBtn}
                  onClick={() => setViewMode("processed")}>전처리</button>
        </div>
      </div>

      <div className={styles.canvasWrap}>
        <Canvas camera={{ position: [1.2, 1.0, 1.6], fov: 50 }}>
          <ambientLight intensity={0.7} />
          <directionalLight position={[3, 5, 3]} intensity={0.6} />
          <OrbitControls />
          <Grid position={[0, -0.001, 0]} args={[4, 4]} cellSize={0.25} cellThickness={0.5}
                cellColor="#D8D8DC" sectionSize={1} sectionThickness={1} sectionColor="#B8B8C4"
                fadeDistance={5} fadeStrength={1.2} infiniteGrid />
          {Content && <Content />}
        </Canvas>
      </div>

      <button type="button" data-testid={`trigger-${kind}`} disabled={status === "running"} onClick={handleTrigger}>
        {KIND_LABELS[kind]}
      </button>
      <span className={styles.status} data-status={status} data-testid={`status-${kind}`}>
        {STATUS_TEXT[status]}
      </span>
    </div>
  );
}
```

- [ ] **Step 6: 테스트 통과 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/components/ScanViewerPanel.test.jsx
```
Expected: PASS (4개 테스트 전부)

- [ ] **Step 7: 커밋**

```bash
cd /home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU
git add isaacpjt/Cart2Trunk/web/frontend/src/components/sceneMeshes.jsx \
        isaacpjt/Cart2Trunk/web/frontend/src/components/ScanViewerPanel.jsx \
        isaacpjt/Cart2Trunk/web/frontend/src/components/ScanViewerPanel.module.css \
        isaacpjt/Cart2Trunk/web/frontend/src/components/ScanViewerPanel.test.jsx
git commit -m "web frontend: ScanViewerPanel 추가 (트렁크/카트 Scan 더미 3D 미리보기, 원본/전처리 토글)"
```

---

### Task 4: `PickPlacePanel.jsx` (진행현황 + 현재 작업 + 상태)

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/PickPlacePanel.jsx`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/PickPlacePanel.module.css`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/PickPlacePanel.test.jsx`

**Interfaces:**
- Consumes: Task 2의 `PICK_PLACE_STEPS`, 기존 `postPickAndPlace()`.
- Produces: `export default function PickPlacePanel()` - props 없음.
  `data-testid`: `trigger-pickAndPlace`, `current-task`, `pick-place-status`.

- [ ] **Step 1: 실패하는 테스트부터 작성**

```jsx
// src/components/PickPlacePanel.test.jsx
import { afterEach, describe, expect, it, vi } from "vitest";
import { act, cleanup, fireEvent, render, screen } from "@testing-library/react";
import PickPlacePanel from "./PickPlacePanel.jsx";
import * as client from "../api/client.js";

afterEach(() => { cleanup(); vi.restoreAllMocks(); });

describe("PickPlacePanel", () => {
  it("대기 중엔 Stop 상태와 '대기 중' 텍스트를 보여준다", () => {
    render(<PickPlacePanel />);
    expect(screen.getByTestId("pick-place-status").textContent).toBe("Stop");
    expect(screen.getByTestId("current-task").textContent).toBe("대기 중");
  });

  it("시작을 누르면 즉시 Run으로 바뀌고 첫 단계 텍스트가 보인다", () => {
    vi.useFakeTimers({ shouldAdvanceTime: true });
    vi.spyOn(client, "postPickAndPlace").mockResolvedValue({ status: "ok", dummy: true, message: "완료" });

    render(<PickPlacePanel />);
    fireEvent.click(screen.getByTestId("trigger-pickAndPlace"));

    expect(screen.getByTestId("pick-place-status").textContent).toBe("Run");
    expect(screen.getByTestId("current-task").textContent).toBe("박스1 pick 접근");
    expect(screen.getByTestId("trigger-pickAndPlace")).toBeDisabled();
    expect(client.postPickAndPlace).toHaveBeenCalledTimes(1);

    vi.useRealTimers();
  });

  it("모든 단계(6개, 700ms 간격)를 다 지나면 완료 텍스트와 함께 Stop으로 돌아온다", async () => {
    vi.useFakeTimers({ shouldAdvanceTime: true });
    vi.spyOn(client, "postPickAndPlace").mockResolvedValue({ status: "ok", dummy: true, message: "완료" });

    render(<PickPlacePanel />);
    fireEvent.click(screen.getByTestId("trigger-pickAndPlace"));

    await act(async () => { await vi.advanceTimersByTimeAsync(700 * 6); });

    expect(screen.getByTestId("current-task").textContent).toBe("완료");
    expect(screen.getByTestId("pick-place-status").textContent).toBe("Stop");
    expect(screen.getByTestId("trigger-pickAndPlace")).not.toBeDisabled();

    vi.useRealTimers();
  });
});
```

- [ ] **Step 2: 테스트가 실패하는지 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/components/PickPlacePanel.test.jsx
```
Expected: FAIL - `Failed to resolve import "./PickPlacePanel.jsx"`

- [ ] **Step 3: `PickPlacePanel.module.css` 작성**

```css
/* src/components/PickPlacePanel.module.css */
.panel {
  flex: 1; background: var(--color-surface); border-radius: 12px; padding: 16px;
  display: flex; flex-direction: column; gap: 16px; min-width: 0;
}
.title { font-size: 13px; font-weight: 700; color: var(--color-text-primary); }
.label { font-size: 11px; font-weight: 700; color: var(--color-text-secondary); text-transform: uppercase; }

.progressSection { display: flex; flex-direction: column; gap: 6px; }
.progressBar { height: 10px; border-radius: 999px; background: var(--color-segment-bg); overflow: hidden; }
.progressFill { height: 100%; background: var(--color-accent); transition: width 0.3s ease; }
.progressPct { align-self: flex-end; font-size: 12px; color: var(--color-text-secondary); }

.taskSection { display: flex; flex-direction: column; gap: 6px; }
.taskText { font-size: 14px; font-weight: 600; color: var(--color-text-primary); }

.statusSection { display: flex; flex-direction: column; gap: 6px; }
.statusBadge {
  align-self: flex-start; font-size: 12px; font-weight: 700; padding: 3px 12px; border-radius: 999px;
}
.statusBadge[data-status="stopped"] { background: #FFEEEC; color: var(--color-danger); }
.statusBadge[data-status="running"] { background: #E4F8EA; color: var(--color-success); }
.statusBadge[data-status="warning"] { background: #E6F1FF; color: var(--color-accent); }

.panel > button {
  border: none; border-radius: 8px; padding: 10px 16px; font-size: 13px; font-weight: 700;
  cursor: pointer; background: var(--color-accent); color: white;
}
.panel > button:disabled { background: var(--color-segment-bg); color: var(--color-text-secondary); cursor: not-allowed; }
```

- [ ] **Step 4: `PickPlacePanel.jsx` 구현**

```jsx
// src/components/PickPlacePanel.jsx
// Pick&Place 칸 - 진행현황 바 + 현재 진행 작업 텍스트 + 상태(Run/Stop/
// Warning). 백엔드 더미 호출(postPickAndPlace)은 지금까지와 동일하게 1회만
// 하고, 화면에 보이는 단계별 진행 애니메이션은 프론트엔드가 자체적으로
// PICK_PLACE_STEPS를 순회하며 만든다(실제 ROS2 진행 상태 스트리밍은
// 아직 없음).
import { useRef, useState } from "react";
import { postPickAndPlace } from "../api/client.js";
import { PICK_PLACE_STEPS } from "./robotDummyData.js";
import styles from "./PickPlacePanel.module.css";

const STEP_INTERVAL_MS = 700;

export default function PickPlacePanel() {
  const [runState, setRunState] = useState("stopped"); // "stopped" | "running"
  const [stepIndex, setStepIndex] = useState(-1); // -1 = 아직 시작 안 함
  const timerRef = useRef(null);

  const advance = (nextIndex) => {
    if (nextIndex >= PICK_PLACE_STEPS.length) {
      setRunState("stopped");
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
    advance(0);
  };

  const currentStep = stepIndex >= 0 ? PICK_PLACE_STEPS[stepIndex] : null;

  return (
    <div className={styles.panel}>
      <span className={styles.title}>Pick&amp;Place</span>

      <div className={styles.progressSection}>
        <label className={styles.label}>진행현황</label>
        <div className={styles.progressBar}>
          <div className={styles.progressFill} style={{ width: `${currentStep?.pct ?? 0}%` }} />
        </div>
        <span className={styles.progressPct}>{currentStep?.pct ?? 0}%</span>
      </div>

      <div className={styles.taskSection}>
        <label className={styles.label}>현재 진행 작업</label>
        <div className={styles.taskText} data-testid="current-task">
          {currentStep ? currentStep.label : "대기 중"}
        </div>
      </div>

      <div className={styles.statusSection}>
        <label className={styles.label}>현재 상태</label>
        <span className={styles.statusBadge} data-status={runState} data-testid="pick-place-status">
          {runState === "running" ? "Run" : "Stop"}
        </span>
      </div>

      <button type="button" data-testid="trigger-pickAndPlace" disabled={runState === "running"} onClick={handleStart}>
        픽앤플레이스 시작
      </button>
    </div>
  );
}
```

- [ ] **Step 5: 테스트 통과 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run src/components/PickPlacePanel.test.jsx
```
Expected: PASS (3개 테스트 전부)

- [ ] **Step 6: 커밋**

```bash
cd /home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU
git add isaacpjt/Cart2Trunk/web/frontend/src/components/PickPlacePanel.jsx \
        isaacpjt/Cart2Trunk/web/frontend/src/components/PickPlacePanel.module.css \
        isaacpjt/Cart2Trunk/web/frontend/src/components/PickPlacePanel.test.jsx
git commit -m "web frontend: PickPlacePanel 추가 (진행현황 바 + 현재 작업 텍스트 + Run/Stop 상태)"
```

---

### Task 5: `RobotControlPanel.jsx` 3칸 컨테이너로 재구성 + 전체 검증

**Files:**
- Modify: `isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.jsx`
- Modify: `isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.module.css`
- Delete: `isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.test.jsx`
  (옛 "버튼3개+로그" 구조를 테스트하던 파일 - 그 구조 자체가 이번 작업으로
  없어지므로 삭제한다. 새 `RobotControlPanel`은 이미 각자 테스트된
  `ScanViewerPanel`/`PickPlacePanel`을 배치만 하는 컨테이너라 - 조건부 로직이
  없어 별도 테스트가 새로 필요하지 않다. 이 판단은 `TabBar`/`App.jsx`에서 이미
  쓴 것과 같은 근거: `<Canvas>`가 섞인 트리를 jsdom에서 통째로 렌더링하지
  않는다는 프로젝트 관례)

**Interfaces:**
- Consumes: Task 3의 `ScanViewerPanel({kind})`, Task 4의 `PickPlacePanel()`.
- Produces: 없음 (조립 지점, `App.jsx`가 그대로 `<RobotControlPanel />`로 사용 -
  기존 `App.jsx` 배선은 변경 불필요)

- [ ] **Step 1: 기존 `RobotControlPanel.test.jsx` 삭제**

```bash
rm isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.test.jsx
```

- [ ] **Step 2: `RobotControlPanel.module.css`를 3칸 레이아웃으로 교체**

```css
/* src/components/RobotControlPanel.module.css */
.panel { display: flex; gap: 16px; padding: 20px; height: 100%; min-height: 0; }

@media (max-width: 1100px) {
  .panel { flex-direction: column; }
}
```

- [ ] **Step 3: `RobotControlPanel.jsx`를 3칸 컨테이너로 교체**

```jsx
// src/components/RobotControlPanel.jsx
// 로봇 제어 탭 - 트렁크 Scan / 카트 Scan / Pick&Place 3칸 컨테이너.
// 실제 로직은 각 자식 컴포넌트(ScanViewerPanel, PickPlacePanel)에 있고,
// 여기는 배치만 담당한다.
import ScanViewerPanel from "./ScanViewerPanel.jsx";
import PickPlacePanel from "./PickPlacePanel.jsx";
import styles from "./RobotControlPanel.module.css";

export default function RobotControlPanel() {
  return (
    <div className={styles.panel}>
      <ScanViewerPanel kind="trunk" />
      <ScanViewerPanel kind="cart" />
      <PickPlacePanel />
    </div>
  );
}
```

- [ ] **Step 4: 프론트엔드 전체 테스트 스위트로 회귀 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm test -- --run
```
Expected: 전부 PASS. Task 1(sceneMeshes 9개, Scene3DViewer.test.js 삭제로
±0), Task 3(ScanViewerPanel 4개 신규), Task 4(PickPlacePanel 3개 신규),
RobotControlPanel.test.jsx(4개 삭제) - 직전 계획(2026-07-26-robot-control-tab.md)
종료 시점 77개에서 net `+4(ScanViewerPanel) +3(PickPlacePanel) -4(구
RobotControlPanel.test.jsx) = +3`으로 80개 전후가 되어야 한다(정확한 숫자보다
"전부 PASS"가 중요).

- [ ] **Step 5: 빌드 확인**

Run:
```bash
cd isaacpjt/Cart2Trunk/web/frontend && npm run build
```
Expected: 에러 없이 성공

- [ ] **Step 6: 커밋**

```bash
cd /home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU
git add isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.jsx \
        isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.module.css \
        isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.test.jsx
git commit -m "web frontend: RobotControlPanel을 트렁크/카트/Pick&Place 3칸 레이아웃으로 재구성"
```

- [ ] **Step 7: 브라우저 수동 검증**

`ScanViewerPanel`은 실제 `<Canvas>`(WebGL)를 그리므로 자동화 테스트로는
"3D 장면이 실제로 보이는지"까지 확인할 수 없다(Task 3 참고 - 로직만 목으로
검증함). 백엔드/프론트엔드 서버가 이미 떠 있다면(Vite HMR로 자동 반영됨)
브라우저에서 다음을 직접 확인한다:

1. "로봇 제어" 탭에서 트렁크 Scan / 카트 Scan / Pick&Place 3칸이 나란히
   보이는지.
2. "트렁크 스캔" 버튼 클릭 → 상태가 대기→진행중→완료로 바뀌고, 완료 후
   3D 캔버스에 단순 와이어프레임(원본 모드 기본값)이 나타나는지.
3. "전처리" 토글 클릭 → 테일램프/트렁크 리드가 있는 디테일한 트렁크
   모양으로 바뀌는지.
4. "카트 스캔"도 동일하게 확인(전처리 모드에서 카트 안에 박스 3개가 보여야
   함).
5. "픽앤플레이스 시작" 클릭 → 진행현황 바가 차오르고, "현재 진행 작업"
   텍스트가 몇 초에 걸쳐 바뀌고, 상태가 Run(초록)이었다가 끝나면 Stop
   (빨강)으로 돌아오는지.
6. 서버가 안 떠 있다면 두 터미널에서 각각 띄운다:
   ```bash
   # 터미널 1
   cd isaacpjt/Cart2Trunk/web/backend && source venv/bin/activate && python app.py
   # 터미널 2
   cd isaacpjt/Cart2Trunk/web/frontend && npm run dev
   ```
   `http://localhost:5173` 접속 후 위 1~5번 확인.

문제가 있으면(3D가 안 보임, 콘솔 에러 등) 그 내용을 그대로 알려준다 - 코드
수정은 별도 확인 후 진행한다.
