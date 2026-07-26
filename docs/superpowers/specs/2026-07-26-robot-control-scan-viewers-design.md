# 로봇 제어 탭 - 트렁크/카트 3D 미리보기 + Pick&Place 진행현황 설계

날짜: 2026-07-26
대상 저장소: `isaacpjt/Cart2Trunk`
선행 스펙: `docs/superpowers/specs/2026-07-26-robot-control-tab-design.md` (이 문서는
그 결과물인 "로봇 제어" 탭을 사용자 손그림 레이아웃에 맞춰 확장한다)
관련 파일 (신규):
`isaacpjt/Cart2Trunk/web/frontend/src/components/sceneMeshes.jsx`,
`isaacpjt/Cart2Trunk/web/frontend/src/components/robotDummyData.js`,
`isaacpjt/Cart2Trunk/web/frontend/src/components/ScanViewerPanel.jsx`,
`isaacpjt/Cart2Trunk/web/frontend/src/components/PickPlacePanel.jsx`
관련 파일 (기존, 수정):
`isaacpjt/Cart2Trunk/web/frontend/src/components/Scene3DViewer.jsx`(순수
렌더링 부품을 sceneMeshes.jsx로 추출, 동작 변경 없음),
`isaacpjt/Cart2Trunk/web/frontend/src/components/Scene3DViewer.test.js`(추출된
함수의 import 경로만 변경),
`isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.jsx`(3칸
레이아웃으로 재구성)

## 배경 및 문제

직전 스펙으로 "로봇 제어" 탭(카트 스캔/트렁크 스캔/픽앤플레이스 버튼 3개 +
텍스트 로그)까지는 만들었다. 사용자가 실제로 눌러보고 "버튼 눌러도 텍스트
로그 말고는 아무 변화가 없어서 심심하다"는 피드백을 줬고, 손그림으로 원하는
화면을 그려줬다: 트렁크 Scan/카트 Scan은 각각 자기만의 3D 미리보기(원본/전처리
토글)를, Pick&Place는 3D 대신 진행현황 바 + 현재 작업 텍스트 + 상태(Run/Stop/
Warning)를 보여주는 3칸 레이아웃.

비전 팀 데이터가 아직 없어서 전부 더미로 채우되, "실제 데이터가 들어오면
그 자리만 바꾸면 바로 동작"하도록 더미 데이터의 모양(shape)을 실제 데이터와
동일하게 맞춰야 한다(사용자 명시적 요구사항).

## 목표

1. `RobotControlPanel`을 3칸 레이아웃으로 재구성한다: 트렁크 Scan / 카트 Scan /
   Pick&Place.
2. 트렁크 Scan, 카트 Scan 칸: "원본"/"전처리" 토글 버튼 + 3D 뷰어. 해당 스캔
   버튼을 눌러 상태가 "완료"가 되면 더미 3D 장면이 나타난다(누르기 전엔 빈
   캔버스). "원본" = 단순 바운딩박스 와이어프레임(가공 전 느낌), "전처리" =
   디테일한 렌더링(트렁크는 테일램프+리드, 카트는 박스들이 얹힌 모습).
3. Pick&Place 칸: "픽앤플레이스 시작"을 누르면 프론트엔드 자체 시뮬레이션으로
   진행현황 바(0→100%)와 "현재 진행 작업" 텍스트가 몇 단계에 걸쳐 바뀌고,
   상태가 Stop(대기, 빨강) → Run(진행 중, 초록) → Stop(완료, 빨강)으로 바뀐다.
   Warning(파랑)은 상태값 자리만 만들어 두고 이번 범위에서는 실제로 쓰지
   않는다(실패 시나리오가 아직 없음).
4. Scene3DViewer.jsx의 순수 렌더링 부품(TrunkWireframe, CartWireframe,
   SceneBoxMesh, toThreeCenter, layoutStagingBoxes, computeCartFootprint)을
   `sceneMeshes.jsx`로 추출해서 시뮬레이터 탭과 로봇 제어 탭이 같이 쓴다 -
   시뮬레이터 탭의 동작/모양은 전혀 바뀌지 않는다(부품 위치만 이동).
5. 더미 트렁크/박스 데이터는 실제 데이터와 동일한 shape로 만든다:
   트렁크 `{width, depth, height, entrance_near_x}`, 박스
   `{id, width, depth, height}` 배열. 실제 연동 시 교체할 정확한 지점에
   `// TODO(비전팀 연동 시): ...` 주석을 남긴다(`routes/robot.py`의
   `TODO(MSI2)`와 같은 방식).

## 비목표 (지금 범위에서 제외)

- 실제 원본/전처리 포인트클라우드 렌더링 - 아직 그런 데이터 자체가 없다.
  "원본"/"전처리"는 지금 있는 3D 프리미티브로 흉내낸 두 가지 뷰일 뿐이다.
- Pick&Place 진행률을 백엔드가 실제로 스트리밍/폴링으로 보고하는 것 - 지금은
  백엔드 호출은 기존과 동일하게 1회성이고, 진행 애니메이션은 순전히
  프론트엔드 로컬 시뮬레이션이다.
- Warning 상태를 실제로 트리거하는 로직 - 상태값과 뱃지 스타일만 준비해두고,
  이번 범위에서 그 상태로 전이시키는 코드는 만들지 않는다.
- 비전 카메라 실시간 피드 - 이전 논의에서 이번 범위는 진행현황/상태 패널로
  대체하기로 확정했다(나중에 별도로 다시 다룸).
- 트렁크 Scan/카트 Scan 3D 뷰어에 시뮬레이터 탭처럼 카메라 프리셋(front/side/
  top)이나 "순서대로 재생" 기능을 넣는 것 - 손그림에 없고, 고정된 각도로도
  "작동 여부 확인"이라는 목적은 충분히 달성된다.

## 설계

### 1. 컴포넌트 구조

```
RobotControlPanel (수정 - 3칸 컨테이너)
├── ScanViewerPanel kind="trunk"  (신규)
├── ScanViewerPanel kind="cart"   (신규)
└── PickPlacePanel                (신규)

sceneMeshes.jsx (신규) - Scene3DViewer.jsx에서 추출한 순수 렌더링 부품
robotDummyData.js (신규) - 더미 트렁크/박스/픽앤플레이스 단계 데이터
```

`ScanViewerPanel`은 트렁크/카트 두 칸이 뼈대(토글 버튼 + 상태 뱃지 + 트리거
버튼 + Canvas)가 완전히 같아서 하나로 만들고 `kind` prop으로 내용만
갈아끼운다. `PickPlacePanel`은 구조가 완전히 달라서(진행바/텍스트/상태) 별도
컴포넌트로 분리한다 - 각 컴포넌트가 "무슨 일을 하는지" 하나로 명확하게
설명 가능하게 유지하기 위함.

### 2. `sceneMeshes.jsx` (신규, Scene3DViewer.jsx에서 추출)

`Scene3DViewer.jsx`에 지금 정의돼 있는 것 중 **props만 받고 `usePlannerState`를
쓰지 않는 것들**을 그대로 옮긴다 (로직 변경 없이 위치만 이동):
`toThreeCenter`, `TrunkWireframe`, `computeCartFootprint`, `CartWireframe`,
`SceneBoxMesh`, `layoutStagingBoxes`. `Scene3DViewer.jsx`는 이 파일에서
import해서 그대로 쓰도록 바꾼다 - 렌더링 결과물은 100% 동일해야 한다.

`Scene3DViewer.test.js`가 지금 `toThreeCenter`/`layoutStagingBoxes`를
`./Scene3DViewer.jsx`에서 import하고 있는데, 추출 후에는
`./sceneMeshes.jsx`에서 import하도록 경로만 바꾼다(테스트 내용 자체는
불변 - 순수 함수라 동작이 똑같이 유지됨을 그대로 검증).

### 3. `robotDummyData.js` (신규)

```js
// 실제 스캔 데이터와 동일한 shape - 나중에 비전 팀 데이터가 들어오면
// 이 값들을 실제 값으로 교체하면 된다(shape는 그대로 유지).
export const DUMMY_TRUNK = {
  width: 0.65, depth: 1.10, height: 0.45, entrance_near_x: true,
};

export const DUMMY_CART_BOXES = [
  { id: "Large", width: 0.30, depth: 0.30, height: 0.20 },
  { id: "Medium", width: 0.20, depth: 0.13, height: 0.15 },
  { id: "Small", width: 0.15, depth: 0.13, height: 0.10 },
];

// Pick&Place 진행 시뮬레이션 단계 - 진행률과 "현재 진행 작업" 텍스트 쌍.
// 실제 연동 시 이 배열 대신 ROS2에서 오는 진행 상태를 그대로 매핑하면 된다.
export const PICK_PLACE_STEPS = [
  { pct: 15, label: "박스1 pick 접근" },
  { pct: 35, label: "박스1 파지" },
  { pct: 55, label: "박스1 트렁크로 이동" },
  { pct: 70, label: "박스1 place" },
  { pct: 85, label: "박스2 pick 접근" },
  { pct: 100, label: "완료" },
];
```

### 4. `ScanViewerPanel.jsx` (신규)

Props: `kind` (`"trunk" | "cart"`).

- 내부 상태: `status`(`idle`|`running`|`done`, `RobotControlPanel`의 기존
  버튼 로직과 동일한 패턴), `viewMode`(`"raw"|"processed"`, 기본 `"raw"`).
- 트리거 버튼("트렁크 스캔"/"카트 스캔")은 기존 `RobotControlPanel`에 있던
  것과 동일한 클릭→POST→상태 전이 로직을 그대로 쓴다(`postTrunkScan`/
  `postCartScan` 호출).
- "원본"/"전처리" 토글 버튼 2개 - `status`와 무관하게 항상 클릭 가능.
- `<Canvas>` 영역: `status !== "done"`이면 빈 캔버스(격자만). `status ===
  "done"`이면 `kind`와 `viewMode`에 따라:
  - `kind="trunk"`, `viewMode="raw"`: `DUMMY_TRUNK` 크기의 단순 박스
    와이어프레임(테일램프/리드 없음)
  - `kind="trunk"`, `viewMode="processed"`: `sceneMeshes.jsx`의
    `TrunkWireframe` 그대로(테일램프+리드 포함)
  - `kind="cart"`, `viewMode="raw"`: `DUMMY_CART_BOXES` 전체를 감싸는 단순
    바운딩박스 하나만(개별 박스 구분 없음)
  - `kind="cart"`, `viewMode="processed"`: `layoutStagingBoxes(DUMMY_CART_BOXES,
    DUMMY_TRUNK)`로 배치 계산 후 `CartWireframe` + 박스별 `SceneBoxMesh` -
    "인식이 개별 물체로 분리해냈다"는 느낌

### 5. `PickPlacePanel.jsx` (신규)

- 내부 상태: `runState`(`"stopped"|"running"`), `stepIndex`(현재
  `PICK_PLACE_STEPS` 인덱스).
- "픽앤플레이스 시작" 클릭 → `postPickAndPlace()` 호출(기존과 동일, 백엔드
  더미 1회 호출) + 동시에 `runState="running"`으로 바꾸고
  `PICK_PLACE_STEPS`를 700ms 간격으로 하나씩 순회하는 타이머 시작.
- 마지막 단계(`pct:100`) 도달 시 `runState="stopped"`로 되돌림.
- 렌더링: 진행률 바(현재 단계의 `pct`), "현재 진행 작업"
  텍스트(`stepIndex`가 없으면 "대기 중"), 상태 뱃지
  (`runState==="running"` → "Run"(초록), 아니면 "Stop"(빨강); "Warning"(파랑)
  스타일은 정의만 해두고 이번 범위에서 트리거하는 코드는 없음).

### 6. `RobotControlPanel.jsx` (수정)

기존의 평평한 버튼 3개 + 로그 목록 구조를 걷어내고, 위 3개 컴포넌트를 가로로
배치하는 컨테이너로 바꾼다:

```jsx
<div className={styles.panel}>
  <ScanViewerPanel kind="trunk" />
  <ScanViewerPanel kind="cart" />
  <PickPlacePanel />
</div>
```

(직전 스펙에 있던 "관제 로그" 텍스트 목록은 이번 손그림에 없으므로 뺀다 - 각
칸이 자기 상태를 직접 보여주므로 별도 텍스트 로그의 필요성이 줄었다는 판단.
사용자가 다시 원하면 쉽게 되돌릴 수 있는 범위의 변경이다.)

### 7. 테스트

- `sceneMeshes.test.js`: 기존 `Scene3DViewer.test.js`의 `toThreeCenter`/
  `layoutStagingBoxes` 테스트를 이 파일로 옮긴다(import 경로만 바뀜, 내용
  동일).
- `Scene3DViewer.test.js`: 추출 후에도 남아있는 테스트가 있다면 import
  경로만 `./sceneMeshes.jsx`로 수정.
- `ScanViewerPanel.test.jsx`: 트리거 버튼 클릭 → 상태 전이 확인(기존
  `RobotControlPanel.test.jsx` 패턴 재사용), "완료" 전에는 캔버스가
  비어있고 이후에 3D 콘텐츠가 나타나는지는 react-three-fiber 렌더링이라
  jsdom에서 직접 검증하지 않고(Scene3DViewer 관례와 동일) 상태값/토글
  버튼 클릭 시 `viewMode`가 바뀌는지 등 순수 로직만 테스트한다.
- `PickPlacePanel.test.jsx`: 클릭 시 `runState`가 "running"으로 바뀌고,
  타이머 진행에 따라 `stepIndex`/진행률 텍스트가 바뀌다가 마지막에
  "stopped"로 돌아오는지 확인(vitest fake timers 사용,
  `useResourceLoader.test.jsx`에서 이미 쓴 패턴 재사용).
