# 웹 기반 적재 플래너 UI 설계 — React + Flask

날짜: 2026-07-24
대상 저장소: `isaacpjt/Cart2Trunk`
관련 파일 (신규): `isaacpjt/Cart2Trunk/web/backend/`, `isaacpjt/Cart2Trunk/web/frontend/`
관련 파일 (기존, 변경 없음): `planner_gui.py`, `algorism/*.py`, `trunk_map_planner_node.py`

## 배경 및 문제

지금까지 `planner_gui.py`(Tkinter)로 적재 알고리즘 파라미터를 조정하고 결과를
matplotlib 정적 이미지 3장(Before/After/Top/Side)으로 확인하는 데스크톱 GUI를
완성했다. 담당 강사가 "웹 기반으로도 만들어보라"는 요구를 추가로 제시했다.

Tkinter GUI는 이미 완성되어 팀 데모/검증에 쓰이고 있으므로 그대로 유지한다.
이번 설계는 완전히 별도의 웹 프로젝트를 신규로 구축하는 것이며, 기존
`algorism/`, `planner_gui.py`, `trunk_map_planner_node.py`는 이번 작업으로
일절 수정하지 않는다.

웹 버전에서 새로 요구되는 것:

1. matplotlib 정적 이미지 대신 3D 뷰어로 결과를 인터랙티브하게 확인.
2. 파라미터를 바꾸면 결과가 실시간(입력 중 자동 재계산)으로 갱신.
3. 알고리즘이 "왜 이 위치를 골랐는지" 판단 과정을 로그로 보여줌.
4. 배치 결과를 점수로 분해해서 보여줌 — 여러 케이스를 통해 만든 최적 배치라는
   것을 정량적으로 입증하는 용도.
5. 전체적인 시각 디자인(Palette/Font 톤)은 Tkinter GUI와 통일감을 유지.
6. 사용자(선욱)는 웹 프론트엔드 경험이 없음 — 실행 구조를 포함해 상세한 안내가
   필요.

## 목표

- `Cart2Trunk/web/` 아래에 `backend/`(Flask), `frontend/`(React)로 분리된 새
  프로젝트를 만든다.
- 기존 `algorism/` 계산 로직을 백엔드에서 그대로 재사용한다 (재구현 금지,
  import해서 호출).
- 프론트엔드는 react-three-fiber 기반 3D 뷰어 + 파라미터 컨트롤 패널 + 점수
  분해 패널 + 로그 패널로 구성한다.
- 파라미터 변경 시 디바운스 후 자동 재계산되는 흐름을 REST API로 구현한다.
- Tkinter GUI가 지원하는 기능을 전부(마진 5종, 선호도 3종, 회전/적층 옵션,
  트렁크 스캔 파일/박스 프리셋 선택, fixed_order, Emergency Stop, 요약 카드,
  박스 상세 조회, 에러 다이얼로그 등) 웹에서도 동일하게 제공한다.

## 비목표 (지금 범위에서 제외)

- 기존 Tkinter GUI(`planner_gui.py`)의 수정·대체 — 원본 그대로 유지.
- `algorism/` 계산 로직 자체의 변경 — 웹 백엔드는 기존 함수를 그대로 호출만
  한다.
- 실제 로봇/ROS2 노드(`trunk_map_planner_node.py`)와의 실시간 연동 — 이번
  범위는 오프라인 플래너 UI(트렁크 스캔 파일 + 박스 프리셋 기반 시뮬레이션)에
  한정하며, 기존 Tkinter GUI와 동일한 전제.
- 인증/다중 사용자/배포(프로덕션 서버, HTTPS, 클라우드 호스팅) — 로컬 개발
  환경에서 팀원이 각자 실행하는 것을 전제로 한다.
- TypeScript 도입, Redux/Zustand 같은 별도 상태관리 라이브러리, Tailwind 등
  추가 프레임워크 — 지금 팀 규모와 요구사항에는 과함.

## 실행 구조 (사용자 질문에 대한 답)

로컬 개발 환경에서 두 개의 독립된 프로세스를 각자 띄운다:

- **백엔드**: `python app.py` (Flask) → `http://localhost:5000` 에서 API만
  제공. 사람이 브라우저로 열어보는 화면이 아니라, 프론트엔드가 호출하는
  JSON API 서버.
- **프론트엔드**: `npm run dev` (Vite) → `http://localhost:5173` 에서 뜨는
  주소가 실제로 브라우저에 열어서 쓰는 화면. 여기서 파라미터를 조작하면
  내부적으로 5000번 포트의 백엔드를 호출해 결과를 받아온다.

즉 실제로 눈으로 보고 쓰는 UI는 `localhost:5173`이며, `localhost:5000`은
백그라운드에서 같이 켜져 있어야 하는 데이터 서버다. 두 서버 모두 터미널
2개에서 각각 실행해두면 된다 (하나로 합치는 프로덕션 빌드는 이번 범위 밖).

## 설계

### 1. 아키텍처 / 툴체인

```
Cart2Trunk/
├── algorism/                  # 기존, 변경 없음
├── planner_gui.py             # 기존, 변경 없음
├── trunk_map_planner_node.py  # 기존, 변경 없음
└── web/
    ├── backend/
    │   ├── app.py
    │   ├── algorism_bridge.py
    │   ├── routes/
    │   │   ├── plan.py
    │   │   ├── resources.py
    │   │   └── approval.py
    │   ├── requirements.txt
    │   └── venv/               # 전용 가상환경 (ROS2 파이썬 환경과 분리)
    └── frontend/
        ├── index.html
        ├── vite.config.js
        ├── package.json
        └── src/
            ├── App.jsx
            ├── state/           # Context + useReducer
            ├── components/
            └── styles/          # Palette/Font 토큰 이식한 CSS
```

- **백엔드**: Flask + `flask-cors`. `perception/.venv` 선례를 따라 전용
  venv를 만들어 ROS2 시스템 파이썬 환경과 의존성이 섞이지 않게 한다.
- **프론트엔드**: React + Vite. **TypeScript 대신 순수 JavaScript** 사용 —
  웹 경험이 없는 상태에서 타입 시스템까지 같이 배우면 진입장벽이 커지므로,
  우선 동작하는 UI를 완성하는 데 집중한다.
- **스타일링**: Tailwind 등 프레임워크 없이 일반 CSS(CSS Modules)로, 기존
  `planner_gui.py`의 `Palette`(색상)와 `Font`(폰트) 상수를 그대로 값만
  옮겨써서 두 UI가 같은 톤을 유지하게 한다.
- **상태 관리**: React Context + `useReducer`. Redux/Zustand 같은 추가
  라이브러리 없이, 파라미터 묶음 + 계산 결과 + 로그를 하나의 전역 상태로
  다룬다. 프로젝트 규모상 충분하다.

### 2. 백엔드 (Flask)

**`algorism_bridge.py`**: 기존 `algorism/` 모듈들(`08_unloadable_reason.py`의
`generate_loading_plan`, `09_rescan_replan.py`의 `replan_after_rescan`,
Task JSON 빌더, 색상 매핑 함수 등)을 그대로 import해서 얇게 감싸는 어댑터
계층. 계산 로직은 절대 재구현하지 않는다.

점수 분해(요구사항 4)는 `05_candidate_scoring.py`가 이미 공개해둔 조각들
(`entrance_distance_ratio`, `side_wall_distance_ratio`, `count_touching_faces`,
가중치 상수)을 `algorism_bridge.py`에서 재사용해 각 배치된 박스마다
`{height_term, contact_term, wall_a_term, wall_bc_term}` 형태로 재구성한다.
`algorism/` 파일 자체는 건드리지 않는다.

**엔드포인트**:

| 메서드 | 경로 | 설명 |
|---|---|---|
| GET | `/api/trunk-maps` | 사용 가능한 트렁크 스캔 파일 목록 |
| GET | `/api/box-presets` | 박스 프리셋 목록 (박스 배열을 응답에 그대로 포함) |
| POST | `/api/plan` | 파라미터 + 박스 목록을 받아 배치 계획을 계산 (핵심 엔드포인트) |
| POST | `/api/approve` | 계획 승인 처리 |
| POST | `/api/send` | Task JSON을 MSI2로 전송 |

무작위 박스 생성은 백엔드 왕복 없이 **프론트엔드 JS에서 직접** 처리한다
(순수 랜덤 생성이라 서버 호출이 불필요한 오버헤드).

### 3. 프론트엔드 (React)

**컴포넌트 트리**:

```
App.jsx (Context + useReducer 전역 상태)
├── Header (타이틀 + Emergency Stop 버튼)
├── ControlPanel
│   ├── 리소스 선택 (트렁크 스캔 파일 / 박스 프리셋 드롭다운)
│   ├── 모드 토글 (large_first / count_first / fixed_order)
│   ├── 마진 입력 5종
│   ├── 선호도 슬라이더 3종 (entrance / contact / height)
│   ├── 옵션 토글 3종 (회전 허용 / 적층 허용 / …)
│   ├── 박스 목록 JSON 편집기
│   └── 액션 버튼 (재계산 / 초기화 / 승인 / 전송)
└── ResultArea
    ├── SummaryCard
    ├── Scene3DViewer (react-three-fiber + drei OrbitControls)
    ├── ScoreBreakdownPanel
    ├── BoxDetailSelector
    └── LogPanel
```

**핵심 설계 포인트**: Tkinter GUI의 Before/After/Side/Top 3장의 정적
matplotlib 이미지는, 별도로 3번 렌더링하는 대신 **하나의 3D 씬에 대한 카메라
각도 프리셋**으로 대체한다. `OrbitControls`로 자유 회전이 가능하면서, 버튼
클릭으로 "정면", "측면", "상단" 프리셋 각도로 즉시 전환할 수 있게 한다 —
정적 이미지보다 더 많은 정보를 하나의 뷰로 제공.

**실시간 갱신**: 모든 파라미터에 대해 균일하게 400ms 디바운스를 적용한다.
Tkinter에서 발견됐던 "위젯 재구성 도중 키 이벤트 처리" 버그 클래스는 React의
선언적 렌더링 모델에서는 애초에 발생하지 않으므로, 필드별로 특수 케이스를
나눌 필요가 없다 (마진 입력 필드를 따로 처리해야 했던 Tkinter와의 차이점).

### 4. API 계약 및 데이터 흐름

**`POST /api/plan` 요청** 예시 필드: 트렁크 스캔 파일명, 박스 프리셋명 또는
직접 입력한 박스 배열, `mode`, `margin`/`wall_margin`/`obstacle_margin`/
`ceiling_margin`/`entrance_margin`, `entrance_preference`/`contact_preference`/
`height_preference`, `allow_stacking`, `allow_rotation`, `fixed_order`.

**응답** 필드: 배치된 박스 목록(각각 좌표/크기/회전 여부/순번/
`score_breakdown`), 미적재 박스 목록(사유 포함), `summary`(전체 배치율 등
요약 통계), `log_lines`(판단 과정 로그 문자열 배열).

**에러 응답**: Tkinter의 `_show_error(code, cause, action)` 패턴을 그대로
이식한 표준 에러 봉투 `{"error_code": ..., "cause": ..., "action": ...}`.

**테스트 전략**:
- 백엔드: pytest + Flask test client로 라우트 레벨만 검증 (계산 로직 자체는
  `algorism/`에 이미 149개 테스트가 있으므로 중복 검증하지 않음).
- 프론트엔드: Vitest + React Testing Library로 가벼운 컴포넌트/리듀서 테스트.
- 수동 브라우저 검증을 UX 검증의 주 채널로 삼는다 — 현재 작업 환경에서는
  브라우저 화면을 직접 볼 수 없으므로(스크린샷 확인 불가 제약, 이전
  Tkinter 작업에서와 동일), 사용자가 직접 `localhost:5173`을 열어 확인하고
  피드백을 주는 방식으로 진행한다.

## 로컬 실행 방법 요약

```bash
# 터미널 1 — 백엔드
cd Cart2Trunk/web/backend
source venv/bin/activate
python app.py                 # http://localhost:5000 (API 전용, 화면 없음)

# 터미널 2 — 프론트엔드
cd Cart2Trunk/web/frontend
npm run dev                   # http://localhost:5173 (브라우저로 여는 실제 UI)
```
