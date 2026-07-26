# 로봇 제어 탭 2x2 레이아웃 (카메라 + 관제 로그 추가) 설계

날짜: 2026-07-26
대상 저장소: `isaacpjt/Cart2Trunk`
선행 스펙: `docs/superpowers/specs/2026-07-26-robot-control-scan-viewers-design.md`
(그 결과물인 3칸 가로 배치를 손그림 기반으로 2x2 그리드로 재배치)
관련 파일 (신규):
`isaacpjt/Cart2Trunk/web/frontend/src/components/CameraPreviewPanel.jsx`,
`isaacpjt/Cart2Trunk/web/frontend/src/components/RobotLogPanel.jsx`
관련 파일 (기존, 수정):
`isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.jsx`(2x2
배치 + 로그 상태 소유),
`isaacpjt/Cart2Trunk/web/frontend/src/components/ScanViewerPanel.jsx`(선택적
`onLog` prop 추가),
`isaacpjt/Cart2Trunk/web/frontend/src/components/PickPlacePanel.jsx`(선택적
`onLog` prop 추가)

## 배경

직전 스펙으로 트렁크/카트/Pick&Place 3칸을 가로로 나란히 배치했는데, 사용자가
손그림으로 2x2 레이아웃을 다시 그려줬다: 위쪽엔 트렁크/카트 Scan 그대로,
아래쪽엔 로봇 카메라 실시간(더미) + Pick&Place + (사용자가 "여기에 뭘 넣지?"로
직접 질문한 네 번째 칸). 네 번째 칸은 예전(3버튼+텍스트 로그 버전)에 있다가
3칸 레이아웃으로 넘어오며 뺐던 "관제 로그"를 다시 넣기로 확인받았다 - 각 칸이
자기 상태만 보여주고 "지금까지 무슨 일이 있었는지"를 시간순으로 보는 곳이
없었기 때문.

## 목표

1. `RobotControlPanel`을 2x2 그리드로 재배치한다: 1행(트렁크 Scan, 카트
   Scan), 2행(카메라 미리보기, Pick&Place, 관제 로그).
2. `CameraPreviewPanel`(신규) - 실제 카메라가 아직 연결 안 됐으므로 정적
   플레이스홀더("📷 카메라 미연동 - 더미 화면")만 보여준다.
3. `RobotLogPanel`(신규) - 트렁크 스캔/카트 스캔/픽앤플레이스 3개 동작의 상태
   전이를 시간순(최신이 위)으로 보여준다.
4. 로그를 채우려면 `ScanViewerPanel`(트렁크/카트 둘 다)과 `PickPlacePanel`이
   자기 상태가 바뀔 때마다 `RobotControlPanel`에 알려줘야 한다 - 선택적
   `onLog(message)` prop을 추가한다(안 넘기면 아무 일도 안 함, 하위 호환 -
   기존 `ScanViewerPanel.test.jsx`/`PickPlacePanel.test.jsx`가 그대로
   통과해야 함).

## 비목표

- 카메라 실시간 영상 실제 연동 - 여전히 정적 더미(이전 논의에서 이미 확정).
- 로그 영속화(새로고침하면 사라짐, 지금까지의 다른 로그들과 동일한 전제).
- 관제 로그를 시뮬레이터 탭의 `LogPanel`/`PlannerContext`와 공유 - 로봇 제어
  탭은 계속 시뮬레이터와 완전히 독립(브레인스토밍에서 이미 확정된 원칙 유지).

## 설계

### 1. 레이아웃 (CSS Grid)

```
grid-template-columns: repeat(4, 1fr);
grid-template-areas:
  "trunk  trunk  cart   cart"
  "camera pick   pick   log";
```

`ScanViewerPanel(kind="trunk")`가 `trunk` 영역(2칸 폭), `ScanViewerPanel
(kind="cart")`가 `cart` 영역(2칸 폭), `CameraPreviewPanel`이 `camera`
영역(1칸), `PickPlacePanel`이 `pick` 영역(2칸 폭), `RobotLogPanel`이 `log`
영역(1칸)을 차지한다.

### 2. `CameraPreviewPanel.jsx` (신규)

Props 없음. 카드 안에 아이콘+텍스트만 표시:

```jsx
<div className={styles.panel}>
  <span className={styles.title}>로봇 카메라 실시간</span>
  <div className={styles.placeholder}>
    <span className={styles.icon}>📷</span>
    <span>카메라 미연동 - 더미 화면</span>
  </div>
</div>
```

### 3. `RobotLogPanel.jsx` (신규)

Props: `logs`(`{time, message}[]`, 최신이 배열 맨 앞). 렌더링만 담당하는
순수 프레젠테이션 컴포넌트(상태는 `RobotControlPanel`이 들고 있음) -
기존(3칸 버전 이전) `RobotControlPanel`의 로그 목록 마크업과 동일한 모양을
재사용한다.

### 4. `ScanViewerPanel.jsx` / `PickPlacePanel.jsx` 수정

두 컴포넌트 다 새 prop `onLog`(옵션, 기본값 `() => {}`)를 받는다.
`ScanViewerPanel`은 `handleTrigger`의 성공/실패 시점에, `PickPlacePanel`은
`handleStart`(시작) + 마지막 단계 도달(완료) 시점에 `onLog(message)`를
호출한다. 기존 두 컴포넌트의 독립 동작(로컬 상태, 테스트)은 전혀 안 바뀌고
로그 콜백 호출만 추가된다.

### 5. `RobotControlPanel.jsx` 수정

로그 상태(`logs`, `appendLog`)를 여기서 소유하고, 5개 자식에게 배치 +
`onLog={appendLog}`(카메라 패널 제외 3곳)를 내려준다.

### 6. 테스트

- `CameraPreviewPanel.test.jsx`: 고정 텍스트가 보이는지만 확인(상태 없음).
- `RobotLogPanel.test.jsx`: `logs` prop을 넘기면 그대로 렌더링되는지,
  비어있으면 "아직 실행된 동작이 없습니다" 문구가 보이는지.
- `ScanViewerPanel.test.jsx` / `PickPlacePanel.test.jsx`: 기존 테스트는 그대로
  두고(하위 호환 확인), `onLog`를 넘겼을 때 상태 전이 시점에 호출되는지
  확인하는 테스트를 각각 1개씩 추가.
- `RobotControlPanel.jsx`는 이전과 마찬가지로(Task 5 판단 근거 동일 -
  `<Canvas>` 포함 트리는 jsdom에서 렌더링 안 함) 별도 컴포넌트 테스트 없이
  수동 브라우저 확인으로 검증한다.
