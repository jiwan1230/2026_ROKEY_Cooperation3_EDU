# 로봇 동작 트리거 탭(관제뷰) 설계

날짜: 2026-07-26
대상 저장소: `isaacpjt/Cart2Trunk`
관련 파일 (신규):
`isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.jsx`,
`isaacpjt/Cart2Trunk/web/frontend/src/components/RobotControlPanel.module.css`,
`isaacpjt/Cart2Trunk/web/backend/routes/robot.py`
관련 파일 (기존, 변경): `web/frontend/src/App.jsx`(탭 전환 추가),
`web/frontend/src/api/client.js`(신규 함수 3개 추가),
`web/backend/app.py`(블루프린트 등록)

## 배경 및 문제

지금까지 웹 플래너 UI(시뮬레이터)는 "트렁크 스캔 파일 + 박스 데이터를 골라
알고리즘으로 적재 계획을 계산/승인/전송"까지만 다뤘다. 실제 로봇(MSI2 - 신지완,
민결)을 곧 전체 통합해야 하는데, 그 쪽 ROS2 노드 구조는 아직 설계 중이라 실제
서비스/토픽 이름이 확정되지 않았다.

그래도 운영자가 카트 스캔/트렁크 스캔/픽앤플레이스를 언제 트리거했고 지금
어느 단계인지 한눈에 보는 화면(관제뷰)은 로봇 연동 전에 미리 만들어 둘 수
있다 - 지금은 실제 ROS2 호출 없이 전부 더미로 동작하고, 나중에 MSI2가 백엔드
함수 내부만 채우면 되는 형태로 만든다.

## 목표

1. 같은 React 앱(Vite, 별도 라우터 없음) 안에 탭 2개를 추가한다 -
   "시뮬레이터"(기존 화면 그대로) / "로봇 제어"(신규).
2. "로봇 제어" 탭에 버튼 3개: **카트 스캔 / 트렁크 스캔 / 픽앤플레이스 시작**.
   언제든 클릭 가능(순서 강제 없음), 시뮬레이터의 승인 상태와 **무관하게
   독립적으로** 동작한다.
3. 관제뷰: 이 3단계 각각의 상태(대기/진행중/완료)를 시각적으로 보여주는 단계
   표시기 + 시간순 로그 목록.
4. 백엔드에 신규 블루프린트 `routes/robot.py` - `POST /api/robot/cart-scan`,
   `/trunk-scan`, `/pick-and-place` 3개. 지금은 실제 ROS2 호출 없이 서버에서
   잠깐 대기 후 고정된 성공 응답만 반환한다.
5. 나중에 MSI2가 각 엔드포인트 함수 안의 "TODO: 실제 ROS2 서비스 호출" 자리에
   실제 연동 코드만 넣으면 되도록 함수 경계를 명확히 나눠 둔다.

## 비목표 (지금 범위에서 제외)

- 실제 ROS2 서비스/토픽 연동 - 신지완/민결의 노드 구조가 아직 미확정이라
  지금은 전부 더미(고정 지연 + 고정 성공 응답)로 대체한다.
- "픽앤플레이스 시작"을 시뮬레이터의 계획 승인 상태와 연동해 잠그는 것 - 지금은
  두 탭이 완전히 독립적으로 동작한다(사용자 확인 완료). 실제 연동 시점에
  다시 논의한다.
- 실패 시나리오 더미화(타임아웃/에러 응답 시뮬레이션) - 지금은 항상 성공만
  반환한다. 실제 ROS2 연동 시 자연스럽게 실패 케이스가 생기므로 그때 추가한다.
- 실시간 카메라 피드 - 지금은 로봇/카메라가 연결되지 않아 표시할 실제 영상이
  없다. 관제뷰는 단계 진행상태 + 로그로만 구성한다.
- 인증/다중 세션/배포 - 기존 시뮬레이터와 동일한 전제(로컬에서 팀원이 각자
  실행).
- `react-router` 등 라우팅 라이브러리 도입 - 탭 2개 전환에는 과함. `App.jsx`의
  로컬 `useState`로 충분하다.

## 설계

### 1. 아키텍처

```
App.jsx
├── Header (공용 - 제목 + EMERGENCY STOP, 탭과 무관하게 항상 표시)
├── 탭 전환 버튼 2개 (로컬 useState: activeTab = "simulator" | "robot")
├── activeTab === "simulator" 이면 기존 화면 그대로
└── activeTab === "robot" 이면 <RobotControlPanel />
```

`RobotControlPanel`은 전역 `PlannerContext`를 구독하지도, 거기에 dispatch하지도
않는다 - 자체 로컬 상태(`useState`/`useReducer`)만 쓴다. 기존 시뮬레이터의
리듀서/테스트를 건드릴 위험이 없고, "지금은 완전히 독립적으로 동작"한다는
요구사항과도 맞는다.

### 2. 프론트엔드 컴포넌트

**`RobotControlPanel.jsx`** (신규)

- 내부 상태: 3단계 각각의 상태(`idle` | `running` | `done`)와 로그 배열
  (`{ time, message }[]`)을 로컬 state로 관리.
- 버튼 3개(카트 스캔/트렁크 스캔/픽앤플레이스 시작): 클릭 시
  1. 해당 단계 상태를 `running`으로 바꾸고 버튼을 잠깐 비활성화
  2. `api/client.js`의 대응 함수(`postCartScan` 등) 호출
  3. 응답 오면 상태를 `done`으로 바꾸고, 로그 목록 맨 뒤에
     `[HH:MM:SS] <단계명> 완료 (더미)` 한 줄 추가
  4. 버튼 재활성화 (다시 눌러서 재시도/반복 가능 - 순서 강제 없음)
- 관제뷰 영역: 3단계를 가로로 나열한 단계 표시기(색으로 대기/진행중/완료
  구분) + 그 아래 로그를 시간 역순(최신이 위)으로 나열.

**`api/client.js`** (기존 파일에 추가)

```js
export function postCartScan() { return postJson("/api/robot/cart-scan", {}); }
export function postTrunkScan() { return postJson("/api/robot/trunk-scan", {}); }
export function postPickAndPlace() { return postJson("/api/robot/pick-and-place", {}); }
```
(기존 `postPlan`/`postApprove` 등과 동일한 내부 `postJson` 헬퍼 재사용)

**`App.jsx`** (기존 파일 수정)

- 탭 버튼 2개 추가, `activeTab` state로 조건부 렌더링만 추가. 기존 `resultArea`
  JSX 구조(SummaryCard/LogPanel/Scene3DViewer/BoxDetailPanel)는 손대지 않는다.

### 3. 백엔드

**`routes/robot.py`** (신규 블루프린트)

```python
robot_bp = Blueprint("robot", __name__)

def _dummy_trigger(step_name: str):
    time.sleep(1.5)  # 실제 스캔/동작 시간을 흉내내는 더미 지연
    # TODO(MSI2): 여기에 실제 ROS2 서비스/액션 호출을 넣는다.
    return jsonify({"status": "ok", "dummy": True,
                     "message": f"{step_name} 완료 (더미 - 실제 로봇 미연동)"})

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

`app.py`에 `app.register_blueprint(robot_bp)` 한 줄만 추가.

**구현 노트**: Flask 개발 서버(`app.run(...)`)는 기본적으로 요청을 한 번에
하나씩 처리한다(`threaded=False` 기본값). 위 더미 지연(1.5초) 동안 다른 요청
(예: 시뮬레이터 탭의 트렁크 맵 3초 폴링)이 밀릴 수 있으므로, 이번 작업에서
`app.run(..., threaded=True)`로 같이 바꿔 둔다 - 순수 I/O 대기(sleep)라 스레드
경합 위험은 없다.

### 4. 데이터 흐름

```
[사용자] 버튼 클릭
   -> RobotControlPanel: 로컬 상태 running으로 변경
   -> POST /api/robot/{cart-scan|trunk-scan|pick-and-place}
   -> 백엔드: 1.5초 대기 후 {"status":"ok","dummy":true,"message":"..."}
   -> RobotControlPanel: 상태 done, 로그 한 줄 추가
```

시뮬레이터 탭의 `/api/plan`, `/api/approve` 등과는 완전히 분리된 별도 API
네임스페이스(`/api/robot/*`)라 서로 간섭하지 않는다.

### 5. 에러 처리

지금은 항상 성공만 반환하므로(비목표 참고) 별도 에러 UI는 만들지 않는다.
다만 네트워크 자체가 끊긴 경우(백엔드 미실행 등)에 대비해, `postJson`이 이미
쓰던 것과 동일한 방식으로 `fetch` 실패 시 해당 단계를 `idle`로 되돌리고 로그에
`[오류] <단계명> 요청 실패 - 백엔드가 실행 중인지 확인하세요` 한 줄만 남긴다
(이 이상의 재시도 로직 등은 만들지 않는다).

### 6. 테스트

- `RobotControlPanel.test.jsx`: 버튼 클릭 → API 호출 → 상태 `done` 전환 →
  로그 한 줄 추가까지 확인 (기존 `VisionDataLoader.test.jsx` 등과 같은 패턴 -
  `vi.spyOn(client, "postCartScan")`으로 목).
- `web/backend/tests/test_routes_robot.py`: 3개 엔드포인트가 각각
  `{"status":"ok","dummy":true}`를 반환하는지 확인. 실제 1.5초 대기는 테스트를
  느리게 만드므로, 테스트에서는 `_dummy_trigger`의 sleep 시간을 매우 짧게
  주입할 수 있게 상수로 분리(`DUMMY_DELAY_SECONDS`)해서 monkeypatch로 줄인다.
