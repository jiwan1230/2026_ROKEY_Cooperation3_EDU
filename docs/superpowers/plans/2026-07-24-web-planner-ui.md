# 웹 기반 적재 플래너 UI (React + Flask) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `Cart2Trunk/web/` 아래에 Flask 백엔드 + React(Vite) 프론트엔드로 구성된 새 웹 UI를 만든다. 기존 `algorism/`, `planner_gui.py`, `trunk_map_planner_node.py`는 전혀 수정하지 않는다.

**Architecture:** 백엔드(`web/backend/`)는 `algorism_bridge.py`가 기존 `algorism/` 번호 모듈(02/03/05/09/17/20)을 직접 import해서 계산을 위임하고, Flask 라우트 3종(`resources`, `plan`, `approval`)이 그 결과를 JSON으로 감싼다. 프론트엔드(`web/frontend/`)는 React Context+`useReducer`로 상태를 관리하고, 파라미터가 바뀔 때마다 400ms 디바운스 후 `POST /api/plan`을 호출해 결과를 갱신하며, react-three-fiber로 트렁크/박스를 인터랙티브 3D로 렌더링한다.

**Tech Stack:** 백엔드 - Python 3.10, Flask, flask-cors, pytest (전용 venv). 프론트엔드 - React 18, Vite, 순수 JavaScript(TS 아님), react-three-fiber+drei(Three.js), Vitest+React Testing Library, CSS Modules.

## Global Constraints

- `algorism/*.py`, `planner_gui.py`, `trunk_map_planner_node.py`는 이번 작업으로 한 글자도 수정하지 않는다 — 새 코드는 전부 `web/` 밑에만 만든다.
- `algorism_bridge.py`는 계산 로직을 재구현하지 않고 기존 함수를 그대로 호출한다. 단, `trunk_map_planner_node.py`는 직접 import하지 않는다 — 그 파일이 최상단에서 `import rclpy`를 해서, ROS2와 분리하기로 한 백엔드 전용 venv에 rclpy 설치를 강제하게 되기 때문이다. 대신 그 파일과 동일한 방식으로 `algorism/` 번호 모듈(02/03/05/09/17/20)을 직접 import하고, `trunk_map_planner_node.py`에 있던 소규모 글루 코드(`plan_from_trunk_map_data`의 본문, `_color_for_box_id`, `DEFAULT_MARGIN`, `_DEFAULT_CART_BOXES`, `_send_task_to_msi2`)만 `algorism_bridge.py` 안에 그대로 옮겨 재구현한다 — 이건 알고리즘이 아니라 오케스트레이션 글루라 중복이 아니다.
- 프론트엔드는 TypeScript, Redux/Zustand, Tailwind를 쓰지 않는다 (스펙 확정 사항).
- 디자인 토큰(`Palette`/`Font`)은 `planner_gui.py`의 값을 그대로 `design-tokens.css`에 CSS 변수로 옮긴다.
- 모든 파라미터 변경은 프론트엔드에서 균일하게 400ms 디바운스 후 `POST /api/plan`을 호출한다 (필드별 특수 케이스 없음 — tkinter와 다른 점, 스펙에 명시됨).
- 무작위 박스 생성은 서버 왕복 없이 프론트엔드 JS에서 직접 처리한다.
- 에러 응답은 항상 `{"error_code":..., "cause":..., "action":...}` 형태 (tkinter `_show_error` 패턴과 동일).
- 백엔드 테스트는 라우트 레벨만 검증한다 (계산 로직 자체는 `algorism/`에 이미 149개 테스트가 있어 중복 검증하지 않음).
- 3D 뷰(react-three-fiber `Canvas`)는 WebGL이 필요해 jsdom에서 온전히 렌더 테스트하기 어렵다 — 좌표 변환 같은 순수 로직만 단위 테스트하고, 실제 렌더링은 사용자의 수동 브라우저 확인에 맡긴다 (이 세션은 브라우저 화면을 볼 수 없다는 기존 제약과 동일).
- tkinter의 `ScoreBreakdownPanel`+`BoxDetailSelector`는 웹에서 `BoxDetailPanel.jsx` 하나로 합친다 — 드롭다운 선택이 점수 분해 표시를 직접 구동하므로 분리할 이유가 없다는 판단(구현 단계에서의 의도적 단순화).
- 참고 스펙: `docs/superpowers/specs/2026-07-24-web-planner-ui-design.md`

---

## Phase A — 백엔드 (Flask)

### Task 1: 백엔드 프로젝트 뼈대 + 헬스체크

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/backend/requirements.txt`
- Create: `isaacpjt/Cart2Trunk/web/backend/app.py`

**Interfaces:**
- Produces: `create_app() -> Flask` (다음 태스크들이 라우트를 등록하기 위해 import), `app` 모듈 레벨 인스턴스.

- [ ] **Step 1: 디렉터리 + venv 준비**

```bash
mkdir -p "/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU/isaacpjt/Cart2Trunk/web/backend/routes"
cd "/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU/isaacpjt/Cart2Trunk/web/backend"
python3 -m venv venv
source venv/bin/activate
```

- [ ] **Step 2: `requirements.txt` 작성 후 설치**

```
Flask==3.0.3
flask-cors==4.0.1
pytest==8.2.0
```

```bash
pip install -r requirements.txt
```

- [ ] **Step 3: `app.py` 작성**

```python
"""
app.py
Cart2Trunk 웹 플래너 백엔드 진입점. algorism/ 계산 로직은 algorism_bridge.py를
통해서만 호출한다 - 이 파일은 라우트 등록과 에러 처리만 담당한다.
"""
from flask import Flask, jsonify
from flask_cors import CORS
from werkzeug.exceptions import HTTPException


def create_app():
    app = Flask(__name__)
    CORS(app, origins=["http://localhost:5173"])

    @app.get("/api/health")
    def health():
        return jsonify({"status": "ok"})

    @app.errorhandler(Exception)
    def handle_unexpected_error(err):
        if isinstance(err, HTTPException):
            return err
        return jsonify({
            "error_code": type(err).__name__,
            "cause": str(err),
            "action": "입력값(트렁크 스캔 파일, 박스 목록, 마진/우선순위 파라미터)을 확인한 뒤 다시 시도하세요.",
        }), 500

    return app


app = create_app()

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
```

- [ ] **Step 4: 헬스체크로 부팅 확인**

```bash
python app.py &
sleep 1
curl -s http://localhost:5000/api/health
kill %1
```

Expected: `{"status":"ok"}`

- [ ] **Step 5: Commit**

```bash
cd "/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU"
git add isaacpjt/Cart2Trunk/web/backend/requirements.txt isaacpjt/Cart2Trunk/web/backend/app.py
git commit -m "web backend: Flask 프로젝트 뼈대 + 헬스체크"
```

---

### Task 2: `algorism_bridge.py` — 리소스 조회 (트렁크맵/박스프리셋/색상)

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/backend/algorism_bridge.py`
- Test: `isaacpjt/Cart2Trunk/web/backend/tests/test_algorism_bridge_resources.py`

**Interfaces:**
- Consumes: `algorism/02_trunk_space_state.py`(`load_trunk_from_world_map`, `load_obstacles_from_world_map`), `03_extreme_point_candidates.py`(`Box`, `PlacedBox`), `05_candidate_scoring.py`(가중치 상수+함수), `09_rescan_replan.py`(`replan_after_rescan`), `17_margin_check.py`(`MARGIN`), `20_task_export.py`(`build_task_json`).
- Produces: `list_trunk_maps() -> List[str]`, `list_box_presets() -> Dict[str, list]`, `generate_random_boxes(count: int) -> List[dict]`, `color_for_box_id(box_id: str) -> str`, 모듈 상수 `_SRC_DIR`, `_LOCAL_TEST_DATA_DIR`, `_PENDING_TASKS_DIR`, `_DEFAULT_CART_BOXES`, `DEFAULT_MARGIN`.

- [ ] **Step 1: `algorism_bridge.py` 뼈대 + import 작성**

```python
"""
algorism_bridge.py
Cart2Trunk/algorism/ 의 기존 계산 로직을 웹 백엔드 라우트에서 쓸 수 있게 얇게
감싸는 어댑터. 계산 로직은 절대 재구현하지 않고 전부 import해서 그대로 쓴다.

trunk_map_planner_node.py를 직접 import하지 않는 이유: 그 파일은 최상단에서
`import rclpy`를 하기 때문에, ROS2 파이썬 환경과 분리하기로 한 이 백엔드
전용 venv에도 rclpy 설치를 강제하게 된다 (설계 목표와 충돌). 대신 그 파일이
쓰는 것과 똑같은 방식으로 algorism/ 안의 번호 붙은 모듈들(02, 03, 05, 09,
17, 20)을 직접 import한다 - trunk_map_planner_node.py의
plan_from_trunk_map_data()/_color_for_box_id()/_send_task_to_msi2() 같은
소규모 글루 코드만 그대로 옮겨왔고, 그 함수들이 호출하는 실제 알고리즘
(02/03/05/09/17/20)은 한 줄도 재구현하지 않았다.
"""
import json
import sys
import pathlib
from importlib import import_module
from typing import Dict, List, Optional

_HERE = pathlib.Path(__file__).resolve().parent
_CART2TRUNK_DIR = _HERE.parent.parent
_ALGORISM_DIR = _CART2TRUNK_DIR / "algorism"
_LOCAL_TEST_DATA_DIR = _ALGORISM_DIR / "local_test_data"
for p in (str(_ALGORISM_DIR), str(_LOCAL_TEST_DATA_DIR)):
    if p not in sys.path:
        sys.path.insert(0, p)

_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
_m05 = import_module("05_candidate_scoring")
_m09 = import_module("09_rescan_replan")
_m17 = import_module("17_margin_check")
_m20 = import_module("20_task_export")

load_trunk_from_world_map = _m02.load_trunk_from_world_map
load_obstacles_from_world_map = _m02.load_obstacles_from_world_map
Box = _m03.Box
PlacedBox = _m03.PlacedBox
replan_after_rescan = _m09.replan_after_rescan
count_touching_faces = _m05.count_touching_faces
entrance_distance_ratio = _m05.entrance_distance_ratio
side_wall_distance_ratio = _m05.side_wall_distance_ratio
HEIGHT_WEIGHT = _m05.HEIGHT_WEIGHT
CONTACT_WEIGHT = _m05.CONTACT_WEIGHT
WALL_A_WEIGHT = _m05.WALL_A_WEIGHT
WALL_BC_WEIGHT = _m05.WALL_BC_WEIGHT
DEFAULT_MARGIN = _m17.MARGIN
build_task_json = _m20.build_task_json

# planner_gui.py의 _discover_trunk_maps()와 같은 경로 관례 - 트렁크 스캔
# run_* 폴더는 ROS2 워크스페이스 src/ 바로 밑에 쌓인다.
_SRC_DIR = pathlib.Path("/home/sunwook/cobot3_ws/src")
_PENDING_TASKS_DIR = _ALGORISM_DIR / "local_test_data" / "pending_tasks"

_BOX_COLOR_PALETTE = ["#3498db", "#e67e22", "#2ecc71", "#9b59b6", "#e74c3c", "#1abc9c", "#f1c40f"]

_DEFAULT_CART_BOXES = [
    {"id": "Large", "width": 0.50, "depth": 0.35, "height": 0.30},
    {"id": "Medium", "width": 0.40, "depth": 0.30, "height": 0.25},
    {"id": "Small", "width": 0.30, "depth": 0.20, "height": 0.15},
]


def color_for_box_id(box_id: str) -> str:
    return _BOX_COLOR_PALETTE[hash(box_id) % len(_BOX_COLOR_PALETTE)]


def list_trunk_maps() -> List[str]:
    """run_*/pointcloud/trunk_map.json이 있는 run 폴더 이름 목록 (오래된 순)."""
    paths = sorted(_SRC_DIR.glob("run_*/pointcloud/trunk_map.json"))
    return [p.parent.parent.name for p in paths]


def _trunk_map_path(run_name: str) -> pathlib.Path:
    path = _SRC_DIR / run_name / "pointcloud" / "trunk_map.json"
    if not path.exists():
        raise ValueError(f"'{run_name}' 트렁크 스캔 파일을 찾을 수 없습니다: {path}")
    return path


def list_box_presets() -> Dict[str, list]:
    """프리셋 이름 -> 박스 배열. planner_gui.py의 _discover_box_presets()와
    달리 파일 경로가 아니라 박스 배열 자체를 값으로 담아 반환한다 - 프론트엔드가
    별도 요청 없이 바로 쓸 수 있게 하기 위함."""
    presets = {"기본값 (Large/Medium/Small)": list(_DEFAULT_CART_BOXES)}
    for f in sorted(_LOCAL_TEST_DATA_DIR.glob("example_cart_boxes_*.json")):
        name = f.stem.replace("example_cart_boxes_", "")
        presets[name] = json.loads(f.read_text())
    return presets


def generate_random_boxes(count: int) -> List[dict]:
    """planner_gui.py의 _generate_random_boxes()와 동일한 범위. 실제 무작위
    생성은 프론트엔드 JS가 서버 왕복 없이 직접 하므로(설계 문서 참고), 이
    함수는 백엔드 단독 테스트/디버깅용으로만 남겨둔다."""
    import random
    rng = random.Random()
    boxes = []
    for i in range(count):
        boxes.append({
            "id": f"Box{i + 1}",
            "width": round(rng.uniform(0.15, 0.45), 2),
            "depth": round(rng.uniform(0.15, 0.40), 2),
            "height": round(rng.uniform(0.10, 0.30), 2),
        })
    return boxes
```

- [ ] **Step 2: 실패하는 테스트 작성**

```python
# isaacpjt/Cart2Trunk/web/backend/tests/test_algorism_bridge_resources.py
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import algorism_bridge as bridge


def test_list_trunk_maps_finds_run_folders(tmp_path, monkeypatch):
    monkeypatch.setattr(bridge, "_SRC_DIR", tmp_path)
    run_dir = tmp_path / "run_20260101_000000" / "pointcloud"
    run_dir.mkdir(parents=True)
    (run_dir / "trunk_map.json").write_text("{}")

    assert bridge.list_trunk_maps() == ["run_20260101_000000"]


def test_list_trunk_maps_empty_when_none_found(tmp_path, monkeypatch):
    monkeypatch.setattr(bridge, "_SRC_DIR", tmp_path)
    assert bridge.list_trunk_maps() == []


def test_list_box_presets_includes_default_and_example_files(tmp_path, monkeypatch):
    monkeypatch.setattr(bridge, "_LOCAL_TEST_DATA_DIR", tmp_path)
    custom = [{"id": "A", "width": 0.2, "depth": 0.2, "height": 0.2}]
    (tmp_path / "example_cart_boxes_stress.json").write_text(json.dumps(custom))

    presets = bridge.list_box_presets()

    assert presets["기본값 (Large/Medium/Small)"] == bridge._DEFAULT_CART_BOXES
    assert presets["stress"] == custom


def test_color_for_box_id_is_stable():
    assert bridge.color_for_box_id("Large") == bridge.color_for_box_id("Large")


def test_generate_random_boxes_count_and_ranges():
    boxes = bridge.generate_random_boxes(5)
    assert len(boxes) == 5
    for b in boxes:
        assert 0.15 <= b["width"] <= 0.45
```

- [ ] **Step 3: 테스트 실행 (통과 확인)**

```bash
cd "/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU/isaacpjt/Cart2Trunk/web/backend"
source venv/bin/activate
python -m pytest tests/test_algorism_bridge_resources.py -v
```

Expected: 5 passed (Step 1의 구현이 이미 완성돼 있으므로 RED 없이 바로 GREEN — 이 파일 자체가 얇은 어댑터라 TDD의 가치는 다음 태스크의 `compute_plan()`에서 더 크다).

- [ ] **Step 4: Commit**

```bash
cd "/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU"
git add isaacpjt/Cart2Trunk/web/backend/algorism_bridge.py isaacpjt/Cart2Trunk/web/backend/tests/test_algorism_bridge_resources.py
git commit -m "web backend: algorism_bridge 리소스 조회(트렁크맵/박스프리셋/색상)"
```

---

### Task 3: `GET /api/trunk-maps`, `GET /api/box-presets`

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/backend/routes/__init__.py` (빈 파일)
- Create: `isaacpjt/Cart2Trunk/web/backend/routes/resources.py`
- Modify: `isaacpjt/Cart2Trunk/web/backend/app.py`
- Test: `isaacpjt/Cart2Trunk/web/backend/tests/test_routes_resources.py`

**Interfaces:**
- Consumes: Task 2의 `bridge.list_trunk_maps()`, `bridge.list_box_presets()`.
- Produces: Flask Blueprint `resources_bp` (`app.py`가 등록).

- [ ] **Step 1: `routes/__init__.py` 빈 파일 생성**

```bash
touch "/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU/isaacpjt/Cart2Trunk/web/backend/routes/__init__.py"
```

- [ ] **Step 2: `routes/resources.py` 작성**

```python
"""
routes/resources.py
GET /api/trunk-maps, GET /api/box-presets - 선택 가능한 리소스 목록 조회.
"""
from flask import Blueprint, jsonify

import algorism_bridge as bridge

resources_bp = Blueprint("resources", __name__)


@resources_bp.get("/api/trunk-maps")
def get_trunk_maps():
    return jsonify({"trunk_maps": bridge.list_trunk_maps()})


@resources_bp.get("/api/box-presets")
def get_box_presets():
    return jsonify({"presets": bridge.list_box_presets()})
```

- [ ] **Step 3: `app.py`에 블루프린트 등록**

`create_app()` 안, `CORS(app, ...)` 다음 줄에 추가:

```python
    from routes.resources import resources_bp
    app.register_blueprint(resources_bp)
```

- [ ] **Step 4: 실패하는 테스트 작성 후 실행**

```python
# isaacpjt/Cart2Trunk/web/backend/tests/test_routes_resources.py
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from app import create_app
import algorism_bridge as bridge


def test_get_trunk_maps_returns_list(monkeypatch):
    monkeypatch.setattr(bridge, "list_trunk_maps", lambda: ["run_x"])
    client = create_app().test_client()

    resp = client.get("/api/trunk-maps")

    assert resp.status_code == 200
    assert resp.get_json() == {"trunk_maps": ["run_x"]}


def test_get_box_presets_returns_dict(monkeypatch):
    monkeypatch.setattr(bridge, "list_box_presets", lambda: {"foo": []})
    client = create_app().test_client()

    resp = client.get("/api/box-presets")

    assert resp.status_code == 200
    assert resp.get_json() == {"presets": {"foo": []}}
```

```bash
python -m pytest tests/test_routes_resources.py -v
```

Expected: 2 passed.

- [ ] **Step 5: Commit**

```bash
git add isaacpjt/Cart2Trunk/web/backend/routes/ isaacpjt/Cart2Trunk/web/backend/app.py isaacpjt/Cart2Trunk/web/backend/tests/test_routes_resources.py
git commit -m "web backend: GET /api/trunk-maps, GET /api/box-presets"
```

---

### Task 4: `algorism_bridge.compute_plan()` — 배치 계산 + 점수 분해 재구성

**Files:**
- Modify: `isaacpjt/Cart2Trunk/web/backend/algorism_bridge.py`
- Test: `isaacpjt/Cart2Trunk/web/backend/tests/test_algorism_bridge_compute_plan.py`

**Interfaces:**
- Consumes: Task 2의 import들 (`load_trunk_from_world_map`, `load_obstacles_from_world_map`, `Box`, `PlacedBox`, `replan_after_rescan`, `count_touching_faces`, `entrance_distance_ratio`, `side_wall_distance_ratio`, `HEIGHT_WEIGHT`/`CONTACT_WEIGHT`/`WALL_A_WEIGHT`/`WALL_BC_WEIGHT`, `DEFAULT_MARGIN`, `color_for_box_id`).
- Produces: `compute_plan(trunk_map_data: dict, boxes_raw: List[dict], box_source_label="custom", mode="large_first", margin=None, allow_stacking=False, allow_rotation=True, wall_margin=None, obstacle_margin=None, ceiling_margin=None, entrance_margin=None, entrance_preference=1.0, contact_preference=1.0, height_preference=1.0, fixed_order=False) -> dict` — 응답 스키마는 아래 Step 1 코드의 반환값 그대로 (Task 5가 이 dict를 그대로 `jsonify`).

- [ ] **Step 1: `_reconstruct_score_breakdown()` + `compute_plan()`을 `algorism_bridge.py` 끝에 추가**

```python
def _reconstruct_score_breakdown(plans, obstacles, trunk, entrance_preference, contact_preference, height_preference):
    """plans는 order로 정렬돼 있다고 가정. place_one_box가 매 박스를 놓을 때
    실제로 봤던 상태(그 이전까지 놓인 박스+장애물)를 그대로 재현해서
    count_touching_faces 등을 다시 계산한다 - 원본 점수를 따로 캐시하지 않고
    재계산하는 이유는 05_candidate_scoring.py를 전혀 수정하지 않고 이미
    공개된 building block만으로 점수를 "설명"하기 위함."""
    placed_so_far = list(obstacles)
    breakdown_by_box_id = {}
    for p in plans:
        box = Box(id=p.box_id, width=p.dimensions[0], depth=p.dimensions[1], height=p.dimensions[2])
        x, y, z = p.position
        touches = count_touching_faces(x, y, z, box, trunk, placed_so_far)
        breakdown_by_box_id[p.box_id] = {
            "height_term": HEIGHT_WEIGHT * height_preference * (z / trunk.height),
            "contact_term": CONTACT_WEIGHT * contact_preference * (touches / 6),
            "wall_a_term": WALL_A_WEIGHT * entrance_preference * entrance_distance_ratio(x, box, trunk),
            "wall_bc_term": WALL_BC_WEIGHT * (1 - side_wall_distance_ratio(y, box, trunk)),
        }
        placed_so_far.append(PlacedBox(box=box, x=x, y=y, z=z))
    return breakdown_by_box_id


def compute_plan(
    trunk_map_data: dict, boxes_raw: List[dict], box_source_label: str = "custom",
    mode: str = "large_first", margin: Optional[float] = None,
    allow_stacking: bool = False, allow_rotation: bool = True,
    wall_margin: Optional[float] = None, obstacle_margin: Optional[float] = None,
    ceiling_margin: Optional[float] = None, entrance_margin: Optional[float] = None,
    entrance_preference: float = 1.0, contact_preference: float = 1.0, height_preference: float = 1.0,
    fixed_order: bool = False,
) -> dict:
    """POST /api/plan 하나가 필요로 하는 전체 응답 payload를 만든다.
    trunk_map_planner_node.plan_from_trunk_map_data()와 같은 순서로 02의
    파서를 직접 호출한다 (그 함수 자체를 import하지 않는 이유는 이 파일
    최상단 docstring 참고)."""
    import time

    world_map = load_trunk_from_world_map(trunk_map_data)
    trunk, offset = world_map.to_bounding_trunk()
    obstacles = load_obstacles_from_world_map(trunk_map_data, offset)
    cart_boxes = [Box(**b) for b in boxes_raw]
    fixed_order_ids = [b["id"] for b in boxes_raw] if fixed_order else None

    t0 = time.perf_counter()
    plans, unloadable = replan_after_rescan(
        cart_boxes, trunk, obstacles, mode=mode, margin=margin, allow_stacking=allow_stacking,
        allow_rotation=allow_rotation, wall_margin=wall_margin, obstacle_margin=obstacle_margin,
        ceiling_margin=ceiling_margin, entrance_margin=entrance_margin,
        entrance_preference=entrance_preference, contact_preference=contact_preference,
        height_preference=height_preference, fixed_order=fixed_order_ids,
    )
    calc_time_ms = (time.perf_counter() - t0) * 1000

    plans_by_order = sorted(plans, key=lambda p: p.order)
    breakdown = _reconstruct_score_breakdown(
        plans_by_order, obstacles, trunk, entrance_preference, contact_preference, height_preference)

    effective_margin = margin if margin is not None else DEFAULT_MARGIN
    placed_volume = sum(p.dimensions[0] * p.dimensions[1] * p.dimensions[2] for p in plans)
    trunk_volume = trunk.width * trunk.depth * trunk.height
    utilization_pct = (placed_volume / trunk_volume * 100) if trunk_volume > 1e-9 else 0.0
    avg_score = (sum(p.score for p in plans) / len(plans)) if plans else 0.0

    log_lines = [
        f"[{trunk_map_data.get('run_id', '?')}] mode={mode}, margin={effective_margin:.2f}m, "
        f"쌓기={'허용' if allow_stacking else '1층전용'}, 회전={'허용' if allow_rotation else '비허용'}, "
        f"입구/깊이축={entrance_preference:+.1f}, 접촉면가중치={contact_preference:.1f}, "
        f"바닥우선강도={height_preference:.1f}, 순서고정={'예' if fixed_order_ids else '아니오'} "
        f"-> {len(plans)}/{len(boxes_raw)}개 배치"
    ]
    for p in plans_by_order:
        log_lines.append(
            f"  PLACED {p.box_id}: pos=({p.position[0]:.2f},{p.position[1]:.2f},{p.position[2]:.2f}) "
            f"rotated={p.rotated}"
        )
    for u in unloadable:
        log_lines.append(f"  UNLOADABLE {u.box_id}: {u.reason.value}")

    return {
        "trunk": {"width": trunk.width, "depth": trunk.depth, "height": trunk.height},
        "obstacles": [
            {"id": o.box.id, "x": o.x, "y": o.y, "z": o.z,
             "width": o.box.width, "depth": o.box.depth, "height": o.box.height}
            for o in obstacles
        ],
        "placed": [
            {
                "box_id": p.box_id, "order": p.order, "position": list(p.position),
                "dimensions": list(p.dimensions), "rotated": p.rotated, "target_yaw": p.target_yaw,
                "score": p.score, "touches": p.touches, "color": color_for_box_id(p.box_id),
                "score_breakdown": breakdown[p.box_id],
            }
            for p in plans_by_order
        ],
        "unloadable": [
            {"box_id": u.box_id, "reason": u.reason.value, "detail": u.detail}
            for u in unloadable
        ],
        "summary": {
            "total": len(boxes_raw), "placed": len(plans), "unplaced": len(unloadable),
            "utilization_pct": utilization_pct, "calc_time_ms": calc_time_ms, "avg_score": avg_score,
        },
        "log_lines": log_lines,
        "trunk_map_id": trunk_map_data.get("run_id", "?"),
        "box_snapshot_id": f"manual_input:{box_source_label}",
    }
```

- [ ] **Step 2: 테스트 작성**

```python
# isaacpjt/Cart2Trunk/web/backend/tests/test_algorism_bridge_compute_plan.py
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import algorism_bridge as bridge

_TRUNK_MAP = {
    "run_id": "test_run",
    "vertices": [
        [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0],
        [0.0, 0.0, 0.5], [1.0, 0.0, 0.5], [0.0, 1.0, 0.5], [1.0, 1.0, 0.5],
    ],
    "edges": [{"v": [0, 1]}, {"v": [0, 2]}, {"v": [0, 4]}],
    "obstacles": [],
}


def test_compute_plan_places_boxes_and_returns_full_payload():
    boxes = [{"id": "A", "width": 0.3, "depth": 0.2, "height": 0.15}]

    result = bridge.compute_plan(_TRUNK_MAP, boxes, box_source_label="테스트")

    assert result["trunk"] == {"width": 1.0, "depth": 1.0, "height": 0.5}
    assert len(result["placed"]) == 1
    assert result["placed"][0]["box_id"] == "A"
    assert set(result["placed"][0]["score_breakdown"].keys()) == {
        "height_term", "contact_term", "wall_a_term", "wall_bc_term"}
    assert result["summary"]["total"] == 1
    assert result["summary"]["placed"] == 1
    assert result["box_snapshot_id"] == "manual_input:테스트"
    assert any("PLACED A" in line for line in result["log_lines"])


def test_compute_plan_reports_unloadable_when_box_too_big():
    boxes = [{"id": "Huge", "width": 5.0, "depth": 5.0, "height": 5.0}]

    result = bridge.compute_plan(_TRUNK_MAP, boxes)

    assert result["placed"] == []
    assert len(result["unloadable"]) == 1
    assert result["unloadable"][0]["box_id"] == "Huge"
    assert result["unloadable"][0]["reason"] == "SIZE_EXCEEDS_TRUNK"


def test_compute_plan_fixed_order_true_preserves_input_order():
    boxes = [
        {"id": "Large", "width": 0.50, "depth": 0.35, "height": 0.30},
        {"id": "Small", "width": 0.30, "depth": 0.20, "height": 0.15},
    ]

    result = bridge.compute_plan(_TRUNK_MAP, boxes, fixed_order=True)

    assert [p["box_id"] for p in result["placed"]] == ["Large", "Small"]
```

- [ ] **Step 3: 테스트 실행**

```bash
python -m pytest tests/test_algorism_bridge_compute_plan.py -v
```

Expected: 3 passed.

- [ ] **Step 4: Commit**

```bash
git add isaacpjt/Cart2Trunk/web/backend/algorism_bridge.py isaacpjt/Cart2Trunk/web/backend/tests/test_algorism_bridge_compute_plan.py
git commit -m "web backend: algorism_bridge.compute_plan() - 배치 계산 + 점수 분해 재구성"
```

---

### Task 5: `POST /api/plan`

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/backend/routes/plan.py`
- Modify: `isaacpjt/Cart2Trunk/web/backend/app.py`
- Test: `isaacpjt/Cart2Trunk/web/backend/tests/test_routes_plan.py`

**Interfaces:**
- Consumes: Task 2의 `bridge._trunk_map_path`, Task 4의 `bridge.compute_plan`.
- Produces: Flask Blueprint `plan_bp`, 예외 클래스 `ApiError(status_code, error_code, cause, action)` — Task 7이 재사용.

- [ ] **Step 1: `routes/plan.py` 작성**

```python
"""
routes/plan.py
POST /api/plan - 파라미터 + 박스 목록을 받아 배치 계획을 계산한다 (핵심 엔드포인트).
"""
import json

from flask import Blueprint, jsonify, request

import algorism_bridge as bridge

plan_bp = Blueprint("plan", __name__)


class ApiError(Exception):
    def __init__(self, status_code: int, error_code: str, cause: str, action: str):
        super().__init__(cause)
        self.status_code = status_code
        self.error_code = error_code
        self.cause = cause
        self.action = action

    def to_response(self):
        return jsonify({
            "error_code": self.error_code, "cause": self.cause, "action": self.action,
        }), self.status_code


@plan_bp.post("/api/plan")
def post_plan():
    body = request.get_json(force=True, silent=True)
    if body is None:
        raise ApiError(
            400, "REQUEST_JSON_INVALID", "요청 본문이 올바른 JSON이 아닙니다.",
            "Content-Type: application/json으로 보냈는지, 본문이 올바른 JSON인지 확인하세요.",
        )

    trunk_map_name = body.get("trunk_map")
    if not trunk_map_name:
        raise ApiError(
            400, "TRUNK_MAP_NOT_SELECTED", "trunk_map 필드가 비어 있습니다.",
            "GET /api/trunk-maps 목록 중 하나를 선택해서 보내세요.",
        )

    boxes_raw = body.get("boxes")
    if not isinstance(boxes_raw, list):
        raise ApiError(
            400, "BOX_JSON_INVALID", "boxes 필드가 배열이 아닙니다.",
            "박스 목록 편집기의 JSON 문법(쉼표, 중괄호, 따옴표)을 확인한 뒤 다시 계산하세요.",
        )

    try:
        trunk_map_path = bridge._trunk_map_path(trunk_map_name)
        trunk_map_data = json.loads(trunk_map_path.read_text())
    except ValueError as e:
        raise ApiError(404, "TRUNK_MAP_NOT_FOUND", str(e),
                        "트렁크 스캔 파일 목록을 새로고침(GET /api/trunk-maps)한 뒤 다시 선택하세요.")

    try:
        result = bridge.compute_plan(
            trunk_map_data, boxes_raw,
            box_source_label=body.get("box_source_label", "custom"),
            mode=body.get("mode", "large_first"),
            margin=body.get("margin"), allow_stacking=body.get("allow_stacking", False),
            allow_rotation=body.get("allow_rotation", True),
            wall_margin=body.get("wall_margin"), obstacle_margin=body.get("obstacle_margin"),
            ceiling_margin=body.get("ceiling_margin"), entrance_margin=body.get("entrance_margin"),
            entrance_preference=body.get("entrance_preference", 1.0),
            contact_preference=body.get("contact_preference", 1.0),
            height_preference=body.get("height_preference", 1.0),
            fixed_order=body.get("fixed_order", False),
        )
    except (KeyError, TypeError) as e:
        raise ApiError(
            400, "BOX_JSON_INVALID", f"박스 목록 필드가 올바르지 않습니다: {e}",
            "각 박스가 id/width/depth/height 필드를 모두 갖고 있는지 확인하세요.",
        )

    return jsonify(result)
```

- [ ] **Step 2: `app.py`에 블루프린트 + 에러 핸들러 등록**

```python
    from routes.plan import plan_bp, ApiError
    app.register_blueprint(plan_bp)

    @app.errorhandler(ApiError)
    def handle_api_error(err):
        return err.to_response()
```

(`resources_bp` 등록 바로 다음 줄에 추가. `handle_unexpected_error`보다 먼저 등록되든 나중이든 Flask는 예외 타입별로 가장 구체적인 핸들러를 찾으므로 순서 무관.)

- [ ] **Step 3: 테스트 작성**

```python
# isaacpjt/Cart2Trunk/web/backend/tests/test_routes_plan.py
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from app import create_app
import algorism_bridge as bridge

_TRUNK_MAP = {
    "run_id": "test_run",
    "vertices": [
        [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0],
        [0.0, 0.0, 0.5], [1.0, 0.0, 0.5], [0.0, 1.0, 0.5], [1.0, 1.0, 0.5],
    ],
    "edges": [{"v": [0, 1]}],
    "obstacles": [],
}


def _client(monkeypatch, tmp_path):
    run_dir = tmp_path / "run_test" / "pointcloud"
    run_dir.mkdir(parents=True)
    (run_dir / "trunk_map.json").write_text(json.dumps(_TRUNK_MAP))
    monkeypatch.setattr(bridge, "_SRC_DIR", tmp_path)
    return create_app().test_client()


def test_post_plan_happy_path(monkeypatch, tmp_path):
    client = _client(monkeypatch, tmp_path)
    resp = client.post("/api/plan", json={
        "trunk_map": "run_test",
        "boxes": [{"id": "A", "width": 0.3, "depth": 0.2, "height": 0.15}],
    })
    assert resp.status_code == 200
    assert len(resp.get_json()["placed"]) == 1


def test_post_plan_missing_trunk_map_returns_400(monkeypatch, tmp_path):
    client = _client(monkeypatch, tmp_path)
    resp = client.post("/api/plan", json={"boxes": []})
    assert resp.status_code == 400
    assert resp.get_json()["error_code"] == "TRUNK_MAP_NOT_SELECTED"


def test_post_plan_invalid_boxes_returns_400(monkeypatch, tmp_path):
    client = _client(monkeypatch, tmp_path)
    resp = client.post("/api/plan", json={"trunk_map": "run_test", "boxes": "not-a-list"})
    assert resp.status_code == 400
    assert resp.get_json()["error_code"] == "BOX_JSON_INVALID"


def test_post_plan_unknown_trunk_map_returns_404(monkeypatch, tmp_path):
    client = _client(monkeypatch, tmp_path)
    resp = client.post("/api/plan", json={"trunk_map": "run_missing", "boxes": []})
    assert resp.status_code == 404
    assert resp.get_json()["error_code"] == "TRUNK_MAP_NOT_FOUND"
```

- [ ] **Step 4: 테스트 실행**

```bash
python -m pytest tests/test_routes_plan.py -v
```

Expected: 4 passed.

- [ ] **Step 5: Commit**

```bash
git add isaacpjt/Cart2Trunk/web/backend/routes/plan.py isaacpjt/Cart2Trunk/web/backend/app.py isaacpjt/Cart2Trunk/web/backend/tests/test_routes_plan.py
git commit -m "web backend: POST /api/plan"
```

---

### Task 6: `algorism_bridge.py` — 승인/전송 (`build_approved_task`, `send_task`)

**Files:**
- Modify: `isaacpjt/Cart2Trunk/web/backend/algorism_bridge.py`
- Test: `isaacpjt/Cart2Trunk/web/backend/tests/test_algorism_bridge_approval.py`

**Interfaces:**
- Consumes: Task 2의 `build_task_json`, `_PENDING_TASKS_DIR`; `algorism/07_placement_plan.py`의 `PlacementPlan`.
- Produces: `build_approved_task(plan_id, box_snapshot_id, trunk_map_id, parameters, placed) -> dict`, `send_task(task_json) -> str`.

- [ ] **Step 1: `algorism_bridge.py` 끝에 추가**

```python
def build_approved_task(plan_id: str, box_snapshot_id: str, trunk_map_id: str,
                         parameters: dict, placed: List[dict]) -> dict:
    """approve 단계 - 20_task_export.build_task_json을 그대로 쓰기 위해,
    프론트가 보낸 placed(POST /api/plan 응답의 placed 배열 그대로)를
    잠깐 PlacementPlan 객체로 복원한다."""
    _m07 = import_module("07_placement_plan")
    PlacementPlan = _m07.PlacementPlan

    plans = [
        PlacementPlan(
            box_id=p["box_id"], order=p["order"], position=tuple(p["position"]),
            dimensions=tuple(p["dimensions"]), score=p["score"], touches=p["touches"],
            rotated=p["rotated"], target_yaw=p["target_yaw"],
        )
        for p in placed
    ]
    return build_task_json(
        plan_id=plan_id, box_snapshot_id=box_snapshot_id, trunk_map_id=trunk_map_id,
        parameters=parameters, plans=plans, approved=True,
    )


def send_task(task_json: dict) -> str:
    """trunk_map_planner_node._send_task_to_msi2()와 동일한 로직(승인 전
    전송 금지 원칙 포함) - rclpy 의존성 없이 이 파일 안에서 재구현했다
    (이유는 이 파일 최상단 docstring 참고). 실제 MSI2 전송 경로가 정해지기
    전까지는 그쪽과 똑같이 로컬 파일로만 저장한다."""
    if not task_json.get("approved", False):
        raise ValueError("approved=False인 Task는 MSI2로 보낼 수 없음 (승인 전 전달 금지 원칙)")

    _PENDING_TASKS_DIR.mkdir(parents=True, exist_ok=True)
    out_path = _PENDING_TASKS_DIR / f"{task_json['plan_id']}.json"
    out_path.write_text(json.dumps(task_json, ensure_ascii=False, indent=2))
    return str(out_path)
```

- [ ] **Step 2: 테스트 작성**

```python
# isaacpjt/Cart2Trunk/web/backend/tests/test_algorism_bridge_approval.py
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import algorism_bridge as bridge

_PLACED = [{
    "box_id": "A", "order": 1, "position": [0.1, 0.2, 0.0], "dimensions": [0.3, 0.2, 0.15],
    "score": 0.5, "touches": 2, "rotated": False, "target_yaw": 0.0,
}]


def test_build_approved_task_marks_approved_true():
    task = bridge.build_approved_task("plan_1", "snap_1", "trunk_1", {"mode": "large_first"}, _PLACED)
    assert task["approved"] is True
    assert task["tasks"][0]["box_id"] == "A"
    assert task["tasks"][0]["sequence"] == 1


def test_send_task_writes_file(tmp_path, monkeypatch):
    monkeypatch.setattr(bridge, "_PENDING_TASKS_DIR", tmp_path)
    task = bridge.build_approved_task("plan_2", "snap_1", "trunk_1", {}, _PLACED)

    out_path = bridge.send_task(task)

    assert pathlib.Path(out_path).exists()
    assert pathlib.Path(out_path).name == "plan_2.json"


def test_send_task_rejects_unapproved():
    task = bridge.build_approved_task("plan_3", "snap_1", "trunk_1", {}, _PLACED)
    task["approved"] = False
    try:
        bridge.send_task(task)
        assert False, "예외가 발생해야 함"
    except ValueError:
        pass
```

- [ ] **Step 3: 테스트 실행**

```bash
python -m pytest tests/test_algorism_bridge_approval.py -v
```

Expected: 3 passed.

- [ ] **Step 4: Commit**

```bash
git add isaacpjt/Cart2Trunk/web/backend/algorism_bridge.py isaacpjt/Cart2Trunk/web/backend/tests/test_algorism_bridge_approval.py
git commit -m "web backend: algorism_bridge - build_approved_task/send_task"
```

---

### Task 7: `POST /api/approve`, `POST /api/send`

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/backend/routes/approval.py`
- Modify: `isaacpjt/Cart2Trunk/web/backend/app.py`
- Test: `isaacpjt/Cart2Trunk/web/backend/tests/test_routes_approval.py`

**Interfaces:**
- Consumes: Task 5의 `ApiError`; Task 6의 `bridge.build_approved_task`, `bridge.send_task`.
- Produces: Flask Blueprint `approval_bp`.

- [ ] **Step 1: `routes/approval.py` 작성**

```python
"""
routes/approval.py
POST /api/approve, POST /api/send - 계획 승인 및 MSI2 전송(현재는 로컬 저장까지만).
"""
from datetime import datetime

from flask import Blueprint, jsonify, request

import algorism_bridge as bridge
from routes.plan import ApiError

approval_bp = Blueprint("approval", __name__)


@approval_bp.post("/api/approve")
def post_approve():
    body = request.get_json(force=True, silent=True) or {}
    placed = body.get("placed")
    if not isinstance(placed, list) or not placed:
        raise ApiError(
            400, "NO_PLAN_TO_APPROVE", "승인할 배치 계획(placed)이 없습니다.",
            "먼저 POST /api/plan으로 계획을 계산한 뒤 그 결과의 placed를 그대로 보내세요.",
        )

    plan_id = f"load_plan_{datetime.now():%Y%m%d_%H%M%S}"
    task = bridge.build_approved_task(
        plan_id=plan_id,
        box_snapshot_id=body.get("box_snapshot_id", "unknown"),
        trunk_map_id=body.get("trunk_map_id", "unknown"),
        parameters=body.get("parameters", {}),
        placed=placed,
    )
    return jsonify({"plan_id": plan_id, "task": task})


@approval_bp.post("/api/send")
def post_send():
    body = request.get_json(force=True, silent=True) or {}
    task = body.get("task")
    if not isinstance(task, dict):
        raise ApiError(
            400, "NO_TASK_TO_SEND", "전송할 task가 없습니다.",
            "먼저 POST /api/approve로 승인한 뒤 그 응답의 task를 그대로 보내세요.",
        )
    try:
        out_path = bridge.send_task(task)
    except ValueError as e:
        raise ApiError(
            400, "TASK_NOT_APPROVED", str(e),
            "POST /api/approve를 먼저 호출해서 approved=True인 task를 받은 뒤 다시 시도하세요.",
        )
    return jsonify({"out_path": out_path})
```

- [ ] **Step 2: `app.py`에 블루프린트 등록**

`plan_bp` 등록 다음 줄에 추가:

```python
    from routes.approval import approval_bp
    app.register_blueprint(approval_bp)
```

- [ ] **Step 3: 테스트 작성**

```python
# isaacpjt/Cart2Trunk/web/backend/tests/test_routes_approval.py
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from app import create_app
import algorism_bridge as bridge

_PLACED = [{
    "box_id": "A", "order": 1, "position": [0.1, 0.2, 0.0], "dimensions": [0.3, 0.2, 0.15],
    "score": 0.5, "touches": 2, "rotated": False, "target_yaw": 0.0,
}]


def test_approve_then_send_happy_path(monkeypatch, tmp_path):
    monkeypatch.setattr(bridge, "_PENDING_TASKS_DIR", tmp_path)
    client = create_app().test_client()

    approve_resp = client.post("/api/approve", json={
        "box_snapshot_id": "snap_1", "trunk_map_id": "trunk_1",
        "parameters": {"mode": "large_first"}, "placed": _PLACED,
    })
    assert approve_resp.status_code == 200
    task = approve_resp.get_json()["task"]
    assert task["approved"] is True

    send_resp = client.post("/api/send", json={"task": task})
    assert send_resp.status_code == 200
    assert pathlib.Path(send_resp.get_json()["out_path"]).exists()


def test_approve_without_placed_returns_400():
    client = create_app().test_client()
    resp = client.post("/api/approve", json={})
    assert resp.status_code == 400
    assert resp.get_json()["error_code"] == "NO_PLAN_TO_APPROVE"


def test_send_without_task_returns_400():
    client = create_app().test_client()
    resp = client.post("/api/send", json={})
    assert resp.status_code == 400
    assert resp.get_json()["error_code"] == "NO_TASK_TO_SEND"
```

- [ ] **Step 4: 테스트 실행 + 전체 백엔드 스위트 확인**

```bash
python -m pytest tests/test_routes_approval.py -v
python -m pytest -v
```

Expected: approval 3 passed; 백엔드 전체 스위트 20 passed(Task 1~7 누적).

- [ ] **Step 5: Commit**

```bash
git add isaacpjt/Cart2Trunk/web/backend/routes/approval.py isaacpjt/Cart2Trunk/web/backend/app.py isaacpjt/Cart2Trunk/web/backend/tests/test_routes_approval.py
git commit -m "web backend: POST /api/approve, POST /api/send"
```

---

## Phase B — 프론트엔드 (React + Vite)

### Task 8: Vite 프로젝트 뼈대 + 디자인 토큰 + 테스트 인프라

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/package.json`
- Create: `isaacpjt/Cart2Trunk/web/frontend/vite.config.js`
- Create: `isaacpjt/Cart2Trunk/web/frontend/index.html`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/main.jsx`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/App.jsx` (자리표시자 - Task 18에서 완성)
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/design-tokens.css`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/test-setup.js`

**Interfaces:**
- Produces: `npm run dev`(포트 5173, `/api` 프록시 -> `localhost:5000`), `npm test`(Vitest), CSS 변수(`--color-*`, `--font-*`) — 이후 모든 컴포넌트의 `.module.css`가 사용.

- [ ] **Step 1: `package.json` 작성**

```json
{
  "name": "cart2trunk-web-frontend",
  "private": true,
  "version": "0.0.1",
  "type": "module",
  "scripts": {
    "dev": "vite",
    "build": "vite build",
    "test": "vitest run"
  },
  "dependencies": {
    "react": "^18.3.1",
    "react-dom": "^18.3.1",
    "three": "^0.165.0",
    "@react-three/fiber": "^8.17.6",
    "@react-three/drei": "^9.114.0"
  },
  "devDependencies": {
    "@vitejs/plugin-react": "^4.3.1",
    "vite": "^5.4.2",
    "vitest": "^2.0.5",
    "jsdom": "^25.0.0",
    "@testing-library/react": "^16.0.0",
    "@testing-library/jest-dom": "^6.5.0",
    "@testing-library/user-event": "^14.5.2"
  }
}
```

- [ ] **Step 2: `vite.config.js` 작성**

```js
import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      "/api": "http://localhost:5000",
    },
  },
  test: {
    environment: "jsdom",
    setupFiles: "./src/test-setup.js",
  },
});
```

- [ ] **Step 3: 나머지 파일 작성**

```html
<!-- index.html -->
<!doctype html>
<html lang="ko">
  <head>
    <meta charset="UTF-8" />
    <title>Cart2Trunk 웹 플래너</title>
  </head>
  <body>
    <div id="root"></div>
    <script type="module" src="/src/main.jsx"></script>
  </body>
</html>
```

```jsx
// src/main.jsx
import React from "react";
import ReactDOM from "react-dom/client";
import App from "./App.jsx";
import "./design-tokens.css";

ReactDOM.createRoot(document.getElementById("root")).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>,
);
```

```jsx
// src/App.jsx (Task 18에서 완성)
export default function App() {
  return <div>Cart2Trunk 웹 플래너 - 준비 중</div>;
}
```

```css
/* src/design-tokens.css - planner_gui.py의 Palette/Font를 그대로 옮김 */
:root {
  --color-canvas: #F5F5F7;
  --color-surface: #FFFFFF;
  --color-border: #E5E5EA;
  --color-text-primary: #1D1D1F;
  --color-text-secondary: #6E6E73;
  --color-accent: #007AFF;
  --color-accent-pressed: #0060DF;
  --color-segment-bg: #E9E9EB;
  --color-success: #34C759;
  --color-danger: #FF3B30;

  --font-family: "Noto Sans CJK KR", "Pretendard", sans-serif;
  --font-mono: "monospace";
}

* { box-sizing: border-box; }

body {
  margin: 0;
  background: var(--color-canvas);
  color: var(--color-text-primary);
  font-family: var(--font-family);
}
```

```js
// src/test-setup.js
import "@testing-library/jest-dom/vitest";
```

- [ ] **Step 4: 의존성 설치 + 부팅 확인**

```bash
cd "/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU/isaacpjt/Cart2Trunk/web/frontend"
npm install
npm run dev
```

`http://localhost:5173`을 브라우저로 직접 열어서 "Cart2Trunk 웹 플래너 - 준비 중"이 보이는지 확인 (이 세션은 브라우저를 볼 수 없으므로 사용자 확인 필요). 확인 후 `Ctrl+C`로 종료.

- [ ] **Step 5: Commit**

```bash
cd "/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU"
git add isaacpjt/Cart2Trunk/web/frontend/package.json isaacpjt/Cart2Trunk/web/frontend/package-lock.json \
        isaacpjt/Cart2Trunk/web/frontend/vite.config.js isaacpjt/Cart2Trunk/web/frontend/index.html \
        isaacpjt/Cart2Trunk/web/frontend/src/main.jsx isaacpjt/Cart2Trunk/web/frontend/src/App.jsx \
        isaacpjt/Cart2Trunk/web/frontend/src/design-tokens.css isaacpjt/Cart2Trunk/web/frontend/src/test-setup.js
git commit -m "web frontend: Vite 프로젝트 뼈대 + 디자인 토큰 + 테스트 인프라"
```

---

### Task 9: `plannerReducer.js` (상태 로직, TDD)

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/state/plannerReducer.js`
- Test: `isaacpjt/Cart2Trunk/web/frontend/src/state/plannerReducer.test.js`

**Interfaces:**
- Produces: `initialState`, `DEFAULT_STRATEGY_PARAMS`, `plannerReducer(state, action)` — Task 10(`PlannerContext`)이 `useReducer`에 그대로 꽂는다. 액션 타입: `RESOURCES_LOADED`, `SET_TRUNK_MAP`, `SELECT_PRESET`, `SET_BOXES_TEXT`, `GENERATE_RANDOM_BOXES`, `SET_PARAM`, `RESET_STRATEGY_DEFAULTS`, `COMPUTE_START`, `COMPUTE_SUCCESS`, `COMPUTE_ERROR`, `SELECT_BOX`, `APPROVE_SUCCESS`, `REJECT`, `SEND_SUCCESS`, `EMERGENCY_STOP`.

- [ ] **Step 1: 실패하는 테스트 먼저 작성**

```js
// src/state/plannerReducer.test.js
import { describe, expect, it } from "vitest";
import { plannerReducer, initialState } from "./plannerReducer.js";

describe("plannerReducer", () => {
  it("loads resources and selects sensible defaults", () => {
    const state = plannerReducer(initialState, {
      type: "RESOURCES_LOADED",
      payload: { trunkMaps: ["run_a", "run_b"], boxPresets: { "기본값": [{ id: "A", width: 0.3, depth: 0.2, height: 0.15 }] } },
    });
    expect(state.trunkMap).toBe("run_b");
    expect(state.boxPresetName).toBe("기본값");
    expect(JSON.parse(state.boxesText)).toHaveLength(1);
  });

  it("invalidates a computed plan when a param changes", () => {
    const computed = { ...initialState, planState: "COMPUTED" };
    const next = plannerReducer(computed, { type: "SET_PARAM", payload: { key: "mode", value: "count_first" } });
    expect(next.planState).toBe("NOT_COMPUTED");
    expect(next.params.mode).toBe("count_first");
    expect(next.logLines[0]).toMatch("무효화");
  });

  it("invalidation also cancels an approval and notes it in the log", () => {
    const approved = { ...initialState, planState: "APPROVED", pendingTask: { approved: true } };
    const next = plannerReducer(approved, { type: "SET_TRUNK_MAP", payload: "run_c" });
    expect(next.planState).toBe("NOT_COMPUTED");
    expect(next.pendingTask).toBeNull();
    expect(next.logLines[0]).toMatch("승인도 함께 취소됨");
  });

  it("does not invalidate a plan that has not been computed yet", () => {
    const next = plannerReducer(initialState, { type: "SET_PARAM", payload: { key: "mode", value: "count_first" } });
    expect(next.planState).toBe("NOT_COMPUTED");
    expect(next.logLines).toHaveLength(0);
  });

  it("stores the compute result and selects the first placed box", () => {
    const payload = { placed: [{ box_id: "A" }, { box_id: "B" }], log_lines: ["line1"] };
    const next = plannerReducer(initialState, { type: "COMPUTE_SUCCESS", payload });
    expect(next.planState).toBe("COMPUTED");
    expect(next.selectedBoxId).toBe("A");
    expect(next.logLines).toEqual(["line1"]);
  });

  it("emergency stop cancels approval regardless of current state", () => {
    const approved = { ...initialState, planState: "APPROVED", pendingTask: { approved: true } };
    const next = plannerReducer(approved, { type: "EMERGENCY_STOP" });
    expect(next.planState).toBe("NOT_COMPUTED");
    expect(next.pendingTask).toBeNull();
  });
});
```

- [ ] **Step 2: 테스트 실행해서 실패 확인**

```bash
cd "/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU/isaacpjt/Cart2Trunk/web/frontend"
npm test -- plannerReducer
```

Expected: FAIL ("Failed to resolve import ./plannerReducer.js" — 파일이 아직 없음)

- [ ] **Step 3: `plannerReducer.js` 구현**

```js
// src/state/plannerReducer.js
export const DEFAULT_STRATEGY_PARAMS = {
  mode: "large_first",
  margin: "",
  wallMargin: "",
  obstacleMargin: "",
  ceilingMargin: "",
  entranceMargin: "",
  entrancePreference: 1.0,
  contactPreference: 1.0,
  heightPreference: 1.0,
  allowStacking: false,
  allowRotation: true,
  fixedOrder: false,
};

export const initialState = {
  trunkMaps: [],
  boxPresets: {},
  trunkMap: "",
  boxPresetName: "",
  boxesText: "[]",
  boxSourceLabel: "custom",
  params: { ...DEFAULT_STRATEGY_PARAMS },
  planState: "NOT_COMPUTED", // NOT_COMPUTED | COMPUTING | COMPUTED | APPROVED
  result: null,
  error: null,
  selectedBoxId: "",
  logLines: [],
  pendingTask: null,
};

function appendLog(state, line) {
  return { ...state, logLines: [...state.logLines, line] };
}

function invalidateIfNeeded(state) {
  if (state.planState === "NOT_COMPUTED" || state.planState === "COMPUTING") return state;
  const wasApproved = state.planState === "APPROVED";
  const next = { ...state, planState: "NOT_COMPUTED", pendingTask: null };
  return appendLog(next, `[무효화] 파라미터가 변경되어 기존 계획을 무효화했습니다${wasApproved ? " (승인도 함께 취소됨)" : ""}.`);
}

export function plannerReducer(state, action) {
  switch (action.type) {
    case "RESOURCES_LOADED": {
      const { trunkMaps, boxPresets } = action.payload;
      const firstPresetName = Object.keys(boxPresets)[0] || "";
      return {
        ...state,
        trunkMaps,
        boxPresets,
        trunkMap: trunkMaps.length ? trunkMaps[trunkMaps.length - 1] : "",
        boxPresetName: firstPresetName,
        boxesText: JSON.stringify(boxPresets[firstPresetName] || [], null, 2),
        boxSourceLabel: firstPresetName,
      };
    }
    case "SET_TRUNK_MAP":
      return invalidateIfNeeded({ ...state, trunkMap: action.payload });
    case "SELECT_PRESET": {
      const boxes = state.boxPresets[action.payload] || [];
      return invalidateIfNeeded({
        ...state, boxPresetName: action.payload,
        boxesText: JSON.stringify(boxes, null, 2), boxSourceLabel: action.payload,
      });
    }
    case "SET_BOXES_TEXT":
      return invalidateIfNeeded({ ...state, boxesText: action.payload, boxSourceLabel: "custom" });
    case "GENERATE_RANDOM_BOXES":
      return invalidateIfNeeded({
        ...state, boxesText: JSON.stringify(action.payload, null, 2), boxSourceLabel: "random",
      });
    case "SET_PARAM":
      return invalidateIfNeeded({
        ...state, params: { ...state.params, [action.payload.key]: action.payload.value },
      });
    case "RESET_STRATEGY_DEFAULTS":
      return invalidateIfNeeded({ ...state, params: { ...DEFAULT_STRATEGY_PARAMS } });
    case "COMPUTE_START":
      return { ...state, planState: "COMPUTING", error: null };
    case "COMPUTE_SUCCESS":
      return {
        ...state, planState: "COMPUTED", result: action.payload,
        logLines: action.payload.log_lines, error: null,
        selectedBoxId: action.payload.placed.length ? action.payload.placed[0].box_id : "",
      };
    case "COMPUTE_ERROR":
      return appendLog(
        { ...state, planState: "NOT_COMPUTED", error: action.payload },
        `[오류] ${action.payload.error_code}: ${action.payload.cause}`,
      );
    case "SELECT_BOX":
      return { ...state, selectedBoxId: action.payload };
    case "APPROVE_SUCCESS":
      return appendLog(
        { ...state, planState: "APPROVED", pendingTask: action.payload.task },
        `[승인] plan_id=${action.payload.plan_id}`,
      );
    case "REJECT":
      return appendLog(
        { ...state, planState: "NOT_COMPUTED", pendingTask: null },
        "[거부] 계획을 거부했습니다 - 파라미터를 조정하고 다시 계산하세요.",
      );
    case "SEND_SUCCESS":
      return appendLog(state, `[승인 및 실행] 로컬에 저장됨: ${action.payload.out_path} (MSI2 실전송 경로 확정 대기)`);
    case "EMERGENCY_STOP":
      return appendLog(
        { ...state, planState: "NOT_COMPUTED", pendingTask: null },
        "[EMERGENCY STOP] 승인/전송을 즉시 취소했습니다. 실제 로봇 정지는 MSI2/하드웨어 E-Stop 담당입니다.",
      );
    default:
      return state;
  }
}
```

- [ ] **Step 4: 테스트 재실행 (통과 확인)**

```bash
npm test -- plannerReducer
```

Expected: 6 passed.

- [ ] **Step 5: Commit**

```bash
git add isaacpjt/Cart2Trunk/web/frontend/src/state/plannerReducer.js isaacpjt/Cart2Trunk/web/frontend/src/state/plannerReducer.test.js
git commit -m "web frontend: plannerReducer 상태 로직 (TDD)"
```

---

### Task 10: `PlannerContext.jsx`

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/state/PlannerContext.jsx`
- Test: `isaacpjt/Cart2Trunk/web/frontend/src/state/PlannerContext.test.jsx`

**Interfaces:**
- Consumes: Task 9의 `initialState`, `plannerReducer`.
- Produces: `PlannerProvider`, `usePlannerState()`, `usePlannerDispatch()` — 이후 모든 컴포넌트/훅이 사용.

- [ ] **Step 1: `PlannerContext.jsx` 작성**

```jsx
// src/state/PlannerContext.jsx
import { createContext, useContext, useReducer } from "react";
import { initialState, plannerReducer } from "./plannerReducer.js";

const PlannerStateContext = createContext(null);
const PlannerDispatchContext = createContext(null);

export function PlannerProvider({ children }) {
  const [state, dispatch] = useReducer(plannerReducer, initialState);
  return (
    <PlannerStateContext.Provider value={state}>
      <PlannerDispatchContext.Provider value={dispatch}>
        {children}
      </PlannerDispatchContext.Provider>
    </PlannerStateContext.Provider>
  );
}

export function usePlannerState() {
  const ctx = useContext(PlannerStateContext);
  if (ctx === null) throw new Error("usePlannerState는 PlannerProvider 안에서만 사용할 수 있습니다");
  return ctx;
}

export function usePlannerDispatch() {
  const ctx = useContext(PlannerDispatchContext);
  if (ctx === null) throw new Error("usePlannerDispatch는 PlannerProvider 안에서만 사용할 수 있습니다");
  return ctx;
}
```

- [ ] **Step 2: 테스트 작성 + 실행**

```jsx
// src/state/PlannerContext.test.jsx
import { describe, expect, it } from "vitest";
import { render, screen } from "@testing-library/react";
import { PlannerProvider, usePlannerState } from "./PlannerContext.jsx";

function Probe() {
  const state = usePlannerState();
  return <div>planState:{state.planState}</div>;
}

describe("PlannerProvider", () => {
  it("provides the reducer's initial state to descendants", () => {
    render(
      <PlannerProvider>
        <Probe />
      </PlannerProvider>,
    );
    expect(screen.getByText("planState:NOT_COMPUTED")).toBeInTheDocument();
  });
});
```

```bash
npm test -- PlannerContext
```

Expected: 1 passed.

- [ ] **Step 3: Commit**

```bash
git add isaacpjt/Cart2Trunk/web/frontend/src/state/PlannerContext.jsx isaacpjt/Cart2Trunk/web/frontend/src/state/PlannerContext.test.jsx
git commit -m "web frontend: PlannerContext (Context + useReducer 배선)"
```

---

### Task 11: API 클라이언트 (`src/api/client.js`)

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/api/client.js`
- Test: `isaacpjt/Cart2Trunk/web/frontend/src/api/client.test.js`

**Interfaces:**
- Produces: `fetchTrunkMaps()`, `fetchBoxPresets()`, `postPlan(body)`, `postApprove(body)`, `postSend(body)` — 전부 실패 시 `error_code`/`cause`/`action` 속성이 달린 `Error`를 throw. Task 12/14/18이 사용.

- [ ] **Step 1: 실패하는 테스트 작성**

```js
// src/api/client.test.js
import { afterEach, describe, expect, it, vi } from "vitest";
import { fetchTrunkMaps, postPlan } from "./client.js";

afterEach(() => {
  vi.unstubAllGlobals();
});

describe("api client", () => {
  it("fetchTrunkMaps returns the trunk_maps array from the response", async () => {
    vi.stubGlobal("fetch", vi.fn().mockResolvedValue({
      ok: true, json: async () => ({ trunk_maps: ["run_a"] }),
    }));
    const maps = await fetchTrunkMaps();
    expect(maps).toEqual(["run_a"]);
  });

  it("postPlan throws an error carrying error_code/cause/action on failure", async () => {
    vi.stubGlobal("fetch", vi.fn().mockResolvedValue({
      ok: false,
      json: async () => ({ error_code: "BOX_JSON_INVALID", cause: "잘못된 형식", action: "고치세요" }),
    }));
    await expect(postPlan({})).rejects.toMatchObject({ error_code: "BOX_JSON_INVALID" });
  });
});
```

```bash
npm test -- client.test
```

Expected: FAIL (모듈 없음)

- [ ] **Step 2: `client.js` 구현**

```js
// src/api/client.js
const BASE = "/api";

async function handleResponse(resp) {
  const body = await resp.json();
  if (!resp.ok) {
    const err = new Error(body.cause || "요청이 실패했습니다");
    err.error_code = body.error_code;
    err.cause = body.cause;
    err.action = body.action;
    throw err;
  }
  return body;
}

export async function fetchTrunkMaps() {
  const resp = await fetch(`${BASE}/trunk-maps`);
  const body = await handleResponse(resp);
  return body.trunk_maps;
}

export async function fetchBoxPresets() {
  const resp = await fetch(`${BASE}/box-presets`);
  const body = await handleResponse(resp);
  return body.presets;
}

export async function postPlan(requestBody) {
  const resp = await fetch(`${BASE}/plan`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(requestBody),
  });
  return handleResponse(resp);
}

export async function postApprove(requestBody) {
  const resp = await fetch(`${BASE}/approve`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(requestBody),
  });
  return handleResponse(resp);
}

export async function postSend(requestBody) {
  const resp = await fetch(`${BASE}/send`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(requestBody),
  });
  return handleResponse(resp);
}
```

- [ ] **Step 3: 테스트 재실행**

```bash
npm test -- client.test
```

Expected: 2 passed.

- [ ] **Step 4: Commit**

```bash
git add isaacpjt/Cart2Trunk/web/frontend/src/api/client.js isaacpjt/Cart2Trunk/web/frontend/src/api/client.test.js
git commit -m "web frontend: API 클라이언트 (fetch 래퍼)"
```

---

### Task 12: `useDebouncedPlan` 훅 (400ms 디바운스 자동 재계산)

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/hooks/useDebouncedPlan.js`
- Test: `isaacpjt/Cart2Trunk/web/frontend/src/hooks/useDebouncedPlan.test.jsx`

**Interfaces:**
- Consumes: Task 10의 `usePlannerState`/`usePlannerDispatch`, Task 11의 `postPlan`.
- Produces: `useDebouncedPlan()` — 부수효과만 있는 훅, 반환값 없음. Task 18의 `App.jsx`가 호출.

- [ ] **Step 1: `useDebouncedPlan.js` 작성**

```js
// src/hooks/useDebouncedPlan.js
import { useEffect, useRef } from "react";
import { postPlan } from "../api/client.js";
import { usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";

const DEBOUNCE_MS = 400;

export function useDebouncedPlan() {
  const state = usePlannerState();
  const dispatch = usePlannerDispatch();
  const timerRef = useRef(null);

  const { trunkMap, boxesText, params, boxSourceLabel } = state;

  useEffect(() => {
    if (!trunkMap) return undefined;

    let parsedBoxes;
    try {
      parsedBoxes = JSON.parse(boxesText);
    } catch {
      return undefined; // 박스 JSON이 아직 타이핑 중이라 문법이 깨진 상태 - 조용히 대기
    }
    if (!Array.isArray(parsedBoxes)) return undefined;

    if (timerRef.current) clearTimeout(timerRef.current);
    timerRef.current = setTimeout(() => {
      dispatch({ type: "COMPUTE_START" });
      postPlan({
        trunk_map: trunkMap,
        boxes: parsedBoxes,
        box_source_label: boxSourceLabel,
        mode: params.mode,
        margin: params.margin === "" ? null : Number(params.margin),
        wall_margin: params.wallMargin === "" ? null : Number(params.wallMargin),
        obstacle_margin: params.obstacleMargin === "" ? null : Number(params.obstacleMargin),
        ceiling_margin: params.ceilingMargin === "" ? null : Number(params.ceilingMargin),
        entrance_margin: params.entranceMargin === "" ? null : Number(params.entranceMargin),
        entrance_preference: params.entrancePreference,
        contact_preference: params.contactPreference,
        height_preference: params.heightPreference,
        allow_stacking: params.allowStacking,
        allow_rotation: params.allowRotation,
        fixed_order: params.fixedOrder,
      })
        .then((result) => dispatch({ type: "COMPUTE_SUCCESS", payload: result }))
        .catch((err) =>
          dispatch({
            type: "COMPUTE_ERROR",
            payload: { error_code: err.error_code || "UNKNOWN", cause: err.cause || err.message, action: err.action || "" },
          }),
        );
    }, DEBOUNCE_MS);

    return () => clearTimeout(timerRef.current);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [trunkMap, boxesText, params, boxSourceLabel]);
}
```

- [ ] **Step 2: 테스트 작성**

```jsx
// src/hooks/useDebouncedPlan.test.jsx
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { act, render, screen } from "@testing-library/react";
import { PlannerProvider, usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";
import { useDebouncedPlan } from "./useDebouncedPlan.js";
import * as client from "../api/client.js";

function Harness() {
  useDebouncedPlan();
  const state = usePlannerState();
  const dispatch = usePlannerDispatch();
  return (
    <div>
      <div data-testid="plan-state">{state.planState}</div>
      <button onClick={() => dispatch({
        type: "RESOURCES_LOADED",
        payload: { trunkMaps: ["run_a"], boxPresets: { "기본값": [{ id: "A", width: 0.3, depth: 0.2, height: 0.15 }] } },
      })}>load</button>
    </div>
  );
}

describe("useDebouncedPlan", () => {
  beforeEach(() => vi.useFakeTimers());
  afterEach(() => { vi.useRealTimers(); vi.restoreAllMocks(); });

  it("fires postPlan 400ms after params settle and dispatches COMPUTE_SUCCESS", async () => {
    vi.spyOn(client, "postPlan").mockResolvedValue({ placed: [], log_lines: [] });
    render(<PlannerProvider><Harness /></PlannerProvider>);

    await act(async () => {
      screen.getByText("load").click();
    });
    await act(async () => {
      vi.advanceTimersByTime(400);
    });

    expect(client.postPlan).toHaveBeenCalledTimes(1);
    expect(screen.getByTestId("plan-state").textContent).toBe("COMPUTED");
  });
});
```

- [ ] **Step 3: 테스트 실행**

```bash
npm test -- useDebouncedPlan
```

Expected: 1 passed.

- [ ] **Step 4: Commit**

```bash
git add isaacpjt/Cart2Trunk/web/frontend/src/hooks/useDebouncedPlan.js isaacpjt/Cart2Trunk/web/frontend/src/hooks/useDebouncedPlan.test.jsx
git commit -m "web frontend: useDebouncedPlan - 400ms 디바운스 자동 재계산"
```

---

### Task 13: `Header.jsx` (제목 + Emergency Stop)

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/Header.jsx`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/Header.module.css`
- Test: `isaacpjt/Cart2Trunk/web/frontend/src/components/Header.test.jsx`

**Interfaces:**
- Consumes: Task 10의 `usePlannerDispatch`.
- Produces: `Header` 컴포넌트 (기본 export) — Task 18의 `App.jsx`가 사용.

- [ ] **Step 1: 컴포넌트 작성**

```jsx
// src/components/Header.jsx
import styles from "./Header.module.css";
import { usePlannerDispatch } from "../state/PlannerContext.jsx";

export default function Header() {
  const dispatch = usePlannerDispatch();

  return (
    <header className={styles.header}>
      <h1 className={styles.title}>Cart2Trunk — 적재 알고리즘 시뮬레이터</h1>
      <button
        type="button"
        className={styles.estop}
        onClick={() => dispatch({ type: "EMERGENCY_STOP" })}
      >
        EMERGENCY STOP
      </button>
    </header>
  );
}
```

```css
/* src/components/Header.module.css */
.header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 16px 28px;
  background: var(--color-surface);
  border-bottom: 1px solid var(--color-border);
}

.title {
  font-size: 18px;
  font-weight: 700;
  margin: 0;
  color: var(--color-text-primary);
}

.estop {
  background: var(--color-danger);
  color: white;
  border: none;
  border-radius: 8px;
  padding: 10px 18px;
  font-weight: 700;
  cursor: pointer;
}

.estop:hover {
  opacity: 0.85;
}
```

- [ ] **Step 2: 테스트 작성 + 실행**

```jsx
// src/components/Header.test.jsx
import { describe, expect, it } from "vitest";
import { render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { PlannerProvider, usePlannerState } from "../state/PlannerContext.jsx";
import Header from "./Header.jsx";

function StateProbe() {
  const state = usePlannerState();
  return <div data-testid="plan-state">{state.planState}</div>;
}

describe("Header", () => {
  it("emergency stop button dispatches EMERGENCY_STOP", async () => {
    render(
      <PlannerProvider>
        <Header />
        <StateProbe />
      </PlannerProvider>,
    );
    await userEvent.click(screen.getByText("EMERGENCY STOP"));
    expect(screen.getByTestId("plan-state").textContent).toBe("NOT_COMPUTED");
  });
});
```

```bash
npm test -- Header
```

Expected: 1 passed.

- [ ] **Step 3: Commit**

```bash
git add isaacpjt/Cart2Trunk/web/frontend/src/components/Header.jsx isaacpjt/Cart2Trunk/web/frontend/src/components/Header.module.css isaacpjt/Cart2Trunk/web/frontend/src/components/Header.test.jsx
git commit -m "web frontend: Header 컴포넌트 (제목 + Emergency Stop)"
```

---

### Task 14: `ControlPanel.jsx` (리소스 선택 + 마진 + 우선순위 + 옵션 + 박스 편집 + 액션 버튼)

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/ControlPanel.jsx`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/ControlPanel.module.css`
- Test: `isaacpjt/Cart2Trunk/web/frontend/src/components/ControlPanel.test.jsx`

**Interfaces:**
- Consumes: Task 10의 `usePlannerState`/`usePlannerDispatch`, Task 11의 `postApprove`/`postSend`.
- Produces: `ControlPanel` 컴포넌트 — Task 18의 `App.jsx`가 사용. 무작위 박스 생성은 서버 호출 없이 이 파일 안 `generateRandomBoxes()`가 담당(설계 문서 확정 사항).

- [ ] **Step 1: 컴포넌트 작성**

```jsx
// src/components/ControlPanel.jsx
import { usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";
import { postApprove, postSend } from "../api/client.js";
import styles from "./ControlPanel.module.css";

function generateRandomBoxes(count) {
  const boxes = [];
  for (let i = 0; i < count; i++) {
    boxes.push({
      id: `Box${i + 1}`,
      width: Math.round((0.15 + Math.random() * 0.30) * 100) / 100,
      depth: Math.round((0.15 + Math.random() * 0.25) * 100) / 100,
      height: Math.round((0.10 + Math.random() * 0.20) * 100) / 100,
    });
  }
  return boxes;
}

const MARGIN_FIELDS = [
  { key: "margin", label: "박스 간격" },
  { key: "wallMargin", label: "벽면 간격" },
  { key: "ceilingMargin", label: "천장 여유" },
  { key: "obstacleMargin", label: "장애물 간격" },
  { key: "entranceMargin", label: "입구 여유거리" },
];

const PREFERENCE_FIELDS = [
  { key: "entrancePreference", label: "입구 ↔ 깊은 위치", min: -1, max: 1, step: 0.1 },
  { key: "contactPreference", label: "공간활용 ↔ 안정성", min: 0, max: 2, step: 0.1 },
  { key: "heightPreference", label: "바닥부터 채우기 강도", min: 0, max: 2, step: 0.1 },
];

export default function ControlPanel() {
  const state = usePlannerState();
  const dispatch = usePlannerDispatch();

  const setParam = (key, value) => dispatch({ type: "SET_PARAM", payload: { key, value } });

  const handleApprove = async () => {
    if (state.planState !== "COMPUTED" || !state.result) return;
    try {
      const resp = await postApprove({
        box_snapshot_id: state.result.box_snapshot_id,
        trunk_map_id: state.result.trunk_map_id,
        parameters: state.params,
        placed: state.result.placed,
      });
      dispatch({ type: "APPROVE_SUCCESS", payload: resp });
    } catch (err) {
      dispatch({ type: "COMPUTE_ERROR", payload: { error_code: err.error_code, cause: err.cause, action: err.action } });
    }
  };

  const handleSend = async () => {
    if (state.planState !== "APPROVED" || !state.pendingTask) return;
    try {
      const resp = await postSend({ task: state.pendingTask });
      dispatch({ type: "SEND_SUCCESS", payload: resp });
    } catch (err) {
      dispatch({ type: "COMPUTE_ERROR", payload: { error_code: err.error_code, cause: err.cause, action: err.action } });
    }
  };

  const locked = state.planState === "APPROVED";

  return (
    <div className={styles.panel}>
      <section className={styles.section}>
        <label className={styles.label}>트렁크 스캔 파일</label>
        <select
          className={styles.select}
          value={state.trunkMap}
          disabled={locked}
          onChange={(e) => dispatch({ type: "SET_TRUNK_MAP", payload: e.target.value })}
        >
          {state.trunkMaps.map((name) => <option key={name} value={name}>{name}</option>)}
        </select>

        <label className={styles.label}>카트 박스 프리셋</label>
        <select
          className={styles.select}
          value={state.boxPresetName}
          disabled={locked}
          onChange={(e) => dispatch({ type: "SELECT_PRESET", payload: e.target.value })}
        >
          {Object.keys(state.boxPresets).map((name) => <option key={name} value={name}>{name}</option>)}
        </select>
      </section>

      <section className={styles.section}>
        <label className={styles.label}>적재 모드</label>
        <div className={styles.segmented}>
          {[["large_first", "큰 것 우선"], ["count_first", "개수 우선"]].map(([value, text]) => (
            <button
              key={value}
              type="button"
              disabled={locked}
              className={state.params.mode === value ? styles.segmentActive : styles.segment}
              onClick={() => setParam("mode", value)}
            >
              {text}
            </button>
          ))}
        </div>
      </section>

      <section className={styles.section}>
        <label className={styles.label}>마진 (m, 비우면 기본값)</label>
        {MARGIN_FIELDS.map(({ key, label }) => (
          <div key={key} className={styles.fieldRow}>
            <span className={styles.fieldLabel}>{label}</span>
            <input
              type="text"
              inputMode="decimal"
              className={styles.input}
              disabled={locked}
              value={state.params[key]}
              onChange={(e) => setParam(key, e.target.value)}
            />
          </div>
        ))}
      </section>

      <section className={styles.section}>
        <label className={styles.label}>우선순위</label>
        {PREFERENCE_FIELDS.map(({ key, label, min, max, step }) => (
          <div key={key} className={styles.fieldRow}>
            <span className={styles.fieldLabel}>{label} ({Number(state.params[key]).toFixed(1)})</span>
            <input
              type="range"
              min={min} max={max} step={step}
              disabled={locked}
              value={state.params[key]}
              onChange={(e) => setParam(key, Number(e.target.value))}
            />
          </div>
        ))}
      </section>

      <section className={styles.section}>
        <label className={styles.toggleRow}>
          <input type="checkbox" disabled={locked} checked={state.params.allowStacking}
                 onChange={(e) => setParam("allowStacking", e.target.checked)} />
          2층 이상 쌓기 허용
        </label>
        <label className={styles.toggleRow}>
          <input type="checkbox" disabled={locked} checked={state.params.allowRotation}
                 onChange={(e) => setParam("allowRotation", e.target.checked)} />
          90도 회전 허용
        </label>
        <label className={styles.toggleRow}>
          <input type="checkbox" disabled={locked} checked={state.params.fixedOrder}
                 onChange={(e) => setParam("fixedOrder", e.target.checked)} />
          적재 순서 고정 (박스 목록 순서 그대로)
        </label>
      </section>

      <section className={styles.section}>
        <label className={styles.label}>박스 목록 (JSON)</label>
        <div className={styles.fieldRow}>
          <button type="button" disabled={locked} onClick={() => dispatch({
            type: "GENERATE_RANDOM_BOXES", payload: generateRandomBoxes(6),
          })}>
            무작위 6개 생성
          </button>
        </div>
        <textarea
          className={styles.boxEditor}
          data-testid="box-editor"
          rows={10}
          disabled={locked}
          value={state.boxesText}
          onChange={(e) => dispatch({ type: "SET_BOXES_TEXT", payload: e.target.value })}
        />
      </section>

      <section className={styles.section}>
        <div className={styles.actions}>
          <button type="button" disabled={locked} onClick={() => dispatch({ type: "RESET_STRATEGY_DEFAULTS" })}>
            기본값으로 초기화
          </button>
          <button type="button" disabled={state.planState !== "COMPUTED"} onClick={handleApprove}>
            승인
          </button>
          <button type="button" disabled={!(state.planState === "COMPUTED" || state.planState === "APPROVED")}
                  onClick={() => dispatch({ type: "REJECT" })}>
            거부
          </button>
          <button type="button" disabled={state.planState !== "APPROVED"} onClick={handleSend}>
            MSI2로 전송
          </button>
        </div>
        {state.error && (
          <div className={styles.errorBox}>
            <strong>오류: {state.error.error_code}</strong>
            <p>{state.error.cause}</p>
            <p>권장 조치: {state.error.action}</p>
          </div>
        )}
      </section>
    </div>
  );
}
```

```css
/* src/components/ControlPanel.module.css */
.panel {
  display: flex;
  flex-direction: column;
  gap: 20px;
  padding: 20px;
  background: var(--color-surface);
  border-right: 1px solid var(--color-border);
  overflow-y: auto;
}

.section { display: flex; flex-direction: column; gap: 8px; }

.label {
  font-size: 11px;
  font-weight: 700;
  color: var(--color-text-secondary);
  text-transform: uppercase;
}

.select, .input, .boxEditor {
  border: 1px solid var(--color-border);
  border-radius: 8px;
  padding: 8px 10px;
  font-family: var(--font-family);
  font-size: 13px;
  background: var(--color-canvas);
}

.boxEditor { font-family: var(--font-mono); font-size: 12px; }

.fieldRow { display: flex; align-items: center; justify-content: space-between; gap: 8px; }
.fieldLabel { font-size: 12px; color: var(--color-text-primary); flex: 1; }

.segmented { display: flex; background: var(--color-segment-bg); border-radius: 10px; padding: 3px; }
.segment, .segmentActive {
  flex: 1; border: none; background: transparent; padding: 8px 0; border-radius: 8px;
  font-weight: 600; font-size: 12px; cursor: pointer;
}
.segmentActive { background: var(--color-accent); color: white; }

.toggleRow { display: flex; align-items: center; gap: 8px; font-size: 13px; }

.actions { display: flex; flex-wrap: wrap; gap: 8px; }
.actions button {
  flex: 1; min-width: 100px; padding: 10px; border-radius: 8px; border: none;
  background: var(--color-accent); color: white; font-weight: 700; cursor: pointer;
}
.actions button:disabled { background: var(--color-segment-bg); color: var(--color-text-secondary); cursor: not-allowed; }

.errorBox {
  background: #FFF1F0; border: 1px solid var(--color-danger); border-radius: 8px;
  padding: 10px; font-size: 12px; color: var(--color-danger);
}
```

- [ ] **Step 2: 테스트 작성 + 실행**

```jsx
// src/components/ControlPanel.test.jsx
import { describe, expect, it } from "vitest";
import { render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { PlannerProvider, usePlannerState } from "../state/PlannerContext.jsx";
import ControlPanel from "./ControlPanel.jsx";

function ModeProbe() {
  const state = usePlannerState();
  return <div data-testid="mode">{state.params.mode}</div>;
}

describe("ControlPanel", () => {
  it("selecting count_first mode updates shared state", async () => {
    render(
      <PlannerProvider>
        <ControlPanel />
        <ModeProbe />
      </PlannerProvider>,
    );
    await userEvent.click(screen.getByText("개수 우선"));
    expect(screen.getByTestId("mode").textContent).toBe("count_first");
  });

  it("generating random boxes fills the box editor with 6 entries", async () => {
    render(<PlannerProvider><ControlPanel /></PlannerProvider>);
    await userEvent.click(screen.getByText("무작위 6개 생성"));
    const editor = screen.getByTestId("box-editor");
    const boxes = JSON.parse(editor.value);
    expect(boxes).toHaveLength(6);
  });
});
```

```bash
npm test -- ControlPanel
```

Expected: 2 passed.

- [ ] **Step 3: Commit**

```bash
git add isaacpjt/Cart2Trunk/web/frontend/src/components/ControlPanel.jsx isaacpjt/Cart2Trunk/web/frontend/src/components/ControlPanel.module.css isaacpjt/Cart2Trunk/web/frontend/src/components/ControlPanel.test.jsx
git commit -m "web frontend: ControlPanel - 파라미터 입력 + 승인/거부/전송 액션"
```

---

### Task 15: `Scene3DViewer.jsx` (react-three-fiber 3D 뷰어 + 카메라 프리셋)

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/Scene3DViewer.jsx`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/Scene3DViewer.module.css`
- Test: `isaacpjt/Cart2Trunk/web/frontend/src/components/Scene3DViewer.test.js`

**Interfaces:**
- Consumes: Task 10의 `usePlannerState` (`state.result.trunk/obstacles/placed`).
- Produces: `Scene3DViewer` 컴포넌트(기본 export), `toThreeCenter(x,y,z,w,d,h)` (named export, 좌표 변환 순수 함수 - 단위 테스트 대상).

- [ ] **Step 1: 컴포넌트 작성**

```jsx
// src/components/Scene3DViewer.jsx
import { useEffect, useRef, useState } from "react";
import { Canvas } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import { usePlannerState } from "../state/PlannerContext.jsx";
import styles from "./Scene3DViewer.module.css";

// 우리 좌표계(x=width, y=depth, z=height, (0,0,0) 코너 기준)를 three.js의
// y-up 좌표계로 옮긴다: three.x=our.x, three.y=our.z(높이), three.z=our.y(깊이).
export function toThreeCenter(x, y, z, w, d, h) {
  return [x + w / 2, z + h / 2, y + d / 2];
}

function TrunkWireframe({ trunk }) {
  return (
    <mesh position={toThreeCenter(0, 0, 0, trunk.width, trunk.depth, trunk.height)}>
      <boxGeometry args={[trunk.width, trunk.height, trunk.depth]} />
      <meshBasicMaterial color="#6E6E73" wireframe />
    </mesh>
  );
}

function SceneBoxMesh({ position, dimensions, color, dashed }) {
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

const CAMERA_PRESETS = {
  front: { position: [3, 1.5, 0.01], target: [0, 0, 0] },
  side: { position: [0.01, 1.5, 3], target: [0, 0, 0] },
  top: { position: [0.01, 4, 0.01], target: [0, 0, 0] },
};

export default function Scene3DViewer() {
  const state = usePlannerState();
  const controlsRef = useRef(null);
  const [preset, setPreset] = useState("front");

  useEffect(() => {
    const controls = controlsRef.current;
    if (!controls) return;
    const { position, target } = CAMERA_PRESETS[preset];
    controls.object.position.set(...position);
    controls.target.set(...target);
    controls.update();
  }, [preset]);

  const trunk = state.result?.trunk;

  return (
    <div className={styles.wrapper}>
      <div className={styles.presetBar}>
        {Object.keys(CAMERA_PRESETS).map((name) => (
          <button key={name} type="button" onClick={() => setPreset(name)}>{name}</button>
        ))}
      </div>
      <Canvas camera={{ position: CAMERA_PRESETS.front.position, fov: 50 }}>
        <ambientLight intensity={0.7} />
        <directionalLight position={[3, 5, 3]} intensity={0.6} />
        <OrbitControls ref={controlsRef} />
        {trunk && <TrunkWireframe trunk={trunk} />}
        {state.result?.obstacles.map((o) => (
          <SceneBoxMesh key={o.id} position={[o.x, o.y, o.z]}
                        dimensions={[o.width, o.depth, o.height]} color="#7f8c8d" />
        ))}
        {state.result?.placed.map((p) => (
          <SceneBoxMesh key={p.box_id} position={p.position} dimensions={p.dimensions}
                        color={p.color} dashed={p.position[2] > 1e-6} />
        ))}
      </Canvas>
    </div>
  );
}
```

```css
/* src/components/Scene3DViewer.module.css */
.wrapper { display: flex; flex-direction: column; height: 420px; background: var(--color-surface); border-radius: 12px; overflow: hidden; }
.presetBar { display: flex; gap: 6px; padding: 8px; border-bottom: 1px solid var(--color-border); }
.presetBar button {
  border: 1px solid var(--color-border); background: var(--color-canvas); border-radius: 6px;
  padding: 4px 10px; font-size: 11px; cursor: pointer;
}
```

- [ ] **Step 2: 좌표 변환 단위 테스트 작성 + 실행**

react-three-fiber의 `Canvas`는 WebGL이 필요해 jsdom에서 온전히 마운트 테스트하기 어렵다 (Global Constraints 참고) — 여기서는 버그가 나기 쉬운 좌표 변환 로직만 단위 테스트하고, 실제 3D 렌더링은 Task 18에서 사용자가 브라우저로 수동 확인한다.

```js
// src/components/Scene3DViewer.test.js
import { describe, expect, it } from "vitest";
import { toThreeCenter } from "./Scene3DViewer.jsx";

describe("toThreeCenter", () => {
  it("maps our z-up corner coords to three.js y-up center coords", () => {
    expect(toThreeCenter(0, 0, 0, 0.4, 0.3, 0.2)).toEqual([0.2, 0.1, 0.15]);
  });

  it("keeps depth(y) mapped to three.js z axis", () => {
    const [, , threeZ] = toThreeCenter(0, 1.0, 0, 0.2, 0.2, 0.2);
    expect(threeZ).toBeCloseTo(1.1);
  });
});
```

```bash
npm install
npm test -- Scene3DViewer
```

Expected: 2 passed.

- [ ] **Step 3: Commit**

```bash
git add isaacpjt/Cart2Trunk/web/frontend/src/components/Scene3DViewer.jsx isaacpjt/Cart2Trunk/web/frontend/src/components/Scene3DViewer.module.css isaacpjt/Cart2Trunk/web/frontend/src/components/Scene3DViewer.test.js
git commit -m "web frontend: Scene3DViewer - react-three-fiber 3D 뷰어 + 카메라 프리셋"
```

---

### Task 16: `BoxDetailPanel.jsx` (박스 선택 + 점수 분해)

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/BoxDetailPanel.jsx`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/BoxDetailPanel.module.css`
- Test: `isaacpjt/Cart2Trunk/web/frontend/src/components/BoxDetailPanel.test.jsx`

**Interfaces:**
- Consumes: Task 10의 `usePlannerState`/`usePlannerDispatch` (`state.result.placed`, `state.selectedBoxId`).
- Produces: `BoxDetailPanel` 컴포넌트 — Task 18의 `App.jsx`가 사용.

- [ ] **Step 1: 컴포넌트 작성**

```jsx
// src/components/BoxDetailPanel.jsx
import { usePlannerDispatch, usePlannerState } from "../state/PlannerContext.jsx";
import styles from "./BoxDetailPanel.module.css";

export default function BoxDetailPanel() {
  const state = usePlannerState();
  const dispatch = usePlannerDispatch();
  const placed = state.result?.placed || [];
  const selected = placed.find((p) => p.box_id === state.selectedBoxId);

  return (
    <div className={styles.panel}>
      <label className={styles.label}>박스 상세 조회</label>
      <select
        className={styles.select}
        value={state.selectedBoxId}
        onChange={(e) => dispatch({ type: "SELECT_BOX", payload: e.target.value })}
      >
        {placed.map((p) => (
          <option key={p.box_id} value={p.box_id}>{p.order}. {p.box_id}</option>
        ))}
      </select>

      {selected ? (
        <div className={styles.detail}>
          <p>
            <strong>{selected.box_id}</strong> · 적재순서 {selected.order} ·
            Target=({selected.position.map((v) => v.toFixed(2)).join(", ")})m ·
            Yaw={selected.target_yaw.toFixed(2)}rad
          </p>
          <p>접촉면 {selected.touches}/6개, {selected.rotated ? "90도 회전됨" : "정자세"}, 점수 {selected.score.toFixed(3)}(낮을수록 좋은 자리)</p>
          <table className={styles.table}>
            <tbody>
              <tr><td>높이 항(불리)</td><td>{selected.score_breakdown.height_term.toFixed(3)}</td></tr>
              <tr><td>접촉면 항(유리)</td><td>-{selected.score_breakdown.contact_term.toFixed(3)}</td></tr>
              <tr><td>안쪽 벽(A) 항(유리)</td><td>-{selected.score_breakdown.wall_a_term.toFixed(3)}</td></tr>
              <tr><td>측면 벽(B/C) 항(유리)</td><td>-{selected.score_breakdown.wall_bc_term.toFixed(3)}</td></tr>
            </tbody>
          </table>
        </div>
      ) : (
        <p className={styles.placeholder}>계획 계산 후 박스를 선택하면 상세정보가 표시됩니다</p>
      )}
    </div>
  );
}
```

```css
/* src/components/BoxDetailPanel.module.css */
.panel { background: var(--color-surface); border-radius: 12px; padding: 16px; display: flex; flex-direction: column; gap: 10px; }
.label { font-size: 11px; font-weight: 700; color: var(--color-text-secondary); text-transform: uppercase; }
.select { border: 1px solid var(--color-border); border-radius: 8px; padding: 6px 8px; }
.detail { font-size: 12px; display: flex; flex-direction: column; gap: 6px; }
.table { border-collapse: collapse; font-size: 12px; }
.table td { padding: 3px 8px; border-bottom: 1px solid var(--color-border); }
.placeholder { font-size: 12px; color: var(--color-text-secondary); }
```

- [ ] **Step 2: 테스트 작성 + 실행**

```jsx
// src/components/BoxDetailPanel.test.jsx
import { describe, expect, it } from "vitest";
import { render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { PlannerProvider, usePlannerDispatch } from "../state/PlannerContext.jsx";
import BoxDetailPanel from "./BoxDetailPanel.jsx";

const RESULT = {
  placed: [
    { box_id: "A", order: 1, position: [0.1, 0.2, 0.0], dimensions: [0.3, 0.2, 0.15],
      rotated: false, target_yaw: 0, score: 0.42, touches: 3,
      score_breakdown: { height_term: 0, contact_term: 0.25, wall_a_term: 0.3, wall_bc_term: 0.1 } },
  ],
  log_lines: [],
};

function Loader() {
  const dispatch = usePlannerDispatch();
  return <button onClick={() => dispatch({ type: "COMPUTE_SUCCESS", payload: RESULT })}>load</button>;
}

describe("BoxDetailPanel", () => {
  it("shows score breakdown for the selected box after a plan is computed", async () => {
    render(
      <PlannerProvider>
        <Loader />
        <BoxDetailPanel />
      </PlannerProvider>,
    );
    await userEvent.click(screen.getByText("load"));
    expect(screen.getByText(/적재순서 1/)).toBeInTheDocument();
    expect(screen.getByText("-0.250")).toBeInTheDocument();
  });
});
```

```bash
npm test -- BoxDetailPanel
```

Expected: 1 passed.

- [ ] **Step 3: Commit**

```bash
git add isaacpjt/Cart2Trunk/web/frontend/src/components/BoxDetailPanel.jsx isaacpjt/Cart2Trunk/web/frontend/src/components/BoxDetailPanel.module.css isaacpjt/Cart2Trunk/web/frontend/src/components/BoxDetailPanel.test.jsx
git commit -m "web frontend: BoxDetailPanel - 박스 선택 + 점수 분해 표시"
```

---

### Task 17: `SummaryCard.jsx` + `LogPanel.jsx`

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/SummaryCard.jsx`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/SummaryCard.module.css`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/LogPanel.jsx`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/components/LogPanel.module.css`
- Test: `isaacpjt/Cart2Trunk/web/frontend/src/components/SummaryCard.test.jsx`
- Test: `isaacpjt/Cart2Trunk/web/frontend/src/components/LogPanel.test.jsx`

**Interfaces:**
- Consumes: Task 10의 `usePlannerState`(+`usePlannerDispatch` for LogPanel test harness).
- Produces: `SummaryCard`, `LogPanel` 컴포넌트 — Task 18의 `App.jsx`가 사용.

- [ ] **Step 1: `SummaryCard.jsx` 작성**

```jsx
// src/components/SummaryCard.jsx
import { usePlannerState } from "../state/PlannerContext.jsx";
import styles from "./SummaryCard.module.css";

const STATUS_LABEL = {
  NOT_COMPUTED: "① 파라미터를 입력하면 자동으로 계산됩니다",
  COMPUTING: "계산 중...",
  COMPUTED: "계산됨 - 승인하거나 파라미터를 조정하세요",
  APPROVED: "승인됨 - 파라미터 잠김 - MSI2로 전송할 수 있습니다",
};

export default function SummaryCard() {
  const state = usePlannerState();
  const summary = state.result?.summary;

  return (
    <div className={styles.card}>
      <div className={styles.row}><span>전체</span><strong>{summary ? summary.total : "-"}</strong></div>
      <div className={styles.row}><span>적재됨</span><strong>{summary ? summary.placed : "-"}</strong></div>
      <div className={styles.row}><span>미적재</span><strong>{summary ? summary.unplaced : "-"}</strong></div>
      <div className={styles.row}><span>공간 활용률</span><strong>{summary ? `${summary.utilization_pct.toFixed(1)}%` : "-"}</strong></div>
      <div className={styles.row}><span>평균 점수</span><strong>{summary ? summary.avg_score.toFixed(3) : "-"}</strong></div>
      <div className={styles.row}><span>계산 시간</span><strong>{summary ? `${summary.calc_time_ms.toFixed(0)}ms` : "-"}</strong></div>
      <div className={styles.status}>상태: {STATUS_LABEL[state.planState]}</div>
    </div>
  );
}
```

```css
/* src/components/SummaryCard.module.css */
.card { background: var(--color-surface); border-radius: 12px; padding: 16px; display: flex; flex-direction: column; gap: 6px; }
.row { display: flex; justify-content: space-between; font-size: 13px; }
.status { margin-top: 8px; font-size: 12px; color: var(--color-text-secondary); }
```

- [ ] **Step 2: `LogPanel.jsx` 작성**

```jsx
// src/components/LogPanel.jsx
import { usePlannerState } from "../state/PlannerContext.jsx";
import styles from "./LogPanel.module.css";

export default function LogPanel() {
  const state = usePlannerState();
  return (
    <div className={styles.panel}>
      <label className={styles.label}>결과 로그</label>
      <pre className={styles.log}>{state.logLines.join("\n")}</pre>
    </div>
  );
}
```

```css
/* src/components/LogPanel.module.css */
.panel { background: var(--color-surface); border-top: 1px solid var(--color-border); padding: 12px 20px; max-height: 160px; overflow-y: auto; }
.label { font-size: 11px; font-weight: 700; color: var(--color-text-secondary); text-transform: uppercase; }
.log { font-family: var(--font-mono); font-size: 11px; white-space: pre-wrap; margin: 6px 0 0; }
```

- [ ] **Step 3: 테스트 작성 + 실행**

```jsx
// src/components/SummaryCard.test.jsx
import { describe, expect, it } from "vitest";
import { render, screen } from "@testing-library/react";
import { PlannerProvider } from "../state/PlannerContext.jsx";
import SummaryCard from "./SummaryCard.jsx";

describe("SummaryCard", () => {
  it("shows placeholders before any plan is computed", () => {
    render(<PlannerProvider><SummaryCard /></PlannerProvider>);
    expect(screen.getByText("① 파라미터를 입력하면 자동으로 계산됩니다")).toBeInTheDocument();
  });
});
```

```jsx
// src/components/LogPanel.test.jsx
import { describe, expect, it } from "vitest";
import { render, screen } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { PlannerProvider, usePlannerDispatch } from "../state/PlannerContext.jsx";
import LogPanel from "./LogPanel.jsx";

function Loader() {
  const dispatch = usePlannerDispatch();
  return <button onClick={() => dispatch({ type: "EMERGENCY_STOP" })}>stop</button>;
}

describe("LogPanel", () => {
  it("renders appended log lines", async () => {
    render(<PlannerProvider><Loader /><LogPanel /></PlannerProvider>);
    await userEvent.click(screen.getByText("stop"));
    expect(screen.getByText(/EMERGENCY STOP/)).toBeInTheDocument();
  });
});
```

```bash
npm test -- SummaryCard LogPanel
```

Expected: 2 passed.

- [ ] **Step 4: Commit**

```bash
git add isaacpjt/Cart2Trunk/web/frontend/src/components/SummaryCard.* isaacpjt/Cart2Trunk/web/frontend/src/components/LogPanel.*
git commit -m "web frontend: SummaryCard, LogPanel"
```

---

### Task 18: `App.jsx` 최종 배선 + `useResourceLoader` + README

**Files:**
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/hooks/useResourceLoader.js`
- Test: `isaacpjt/Cart2Trunk/web/frontend/src/hooks/useResourceLoader.test.jsx`
- Modify: `isaacpjt/Cart2Trunk/web/frontend/src/App.jsx`
- Create: `isaacpjt/Cart2Trunk/web/frontend/src/App.module.css`
- Create: `isaacpjt/Cart2Trunk/web/README.md`

**Interfaces:**
- Consumes: Task 11(`fetchTrunkMaps`/`fetchBoxPresets`), Task 12(`useDebouncedPlan`), Task 13/14/15/16/17의 모든 컴포넌트.
- Produces: 완성된 `App` (기본 export) — 실제로 실행되는 최종 화면.

- [ ] **Step 1: `useResourceLoader.js` 작성**

```js
// src/hooks/useResourceLoader.js
import { useEffect } from "react";
import { fetchBoxPresets, fetchTrunkMaps } from "../api/client.js";
import { usePlannerDispatch } from "../state/PlannerContext.jsx";

export function useResourceLoader() {
  const dispatch = usePlannerDispatch();

  useEffect(() => {
    Promise.all([fetchTrunkMaps(), fetchBoxPresets()])
      .then(([trunkMaps, boxPresets]) => {
        dispatch({ type: "RESOURCES_LOADED", payload: { trunkMaps, boxPresets } });
      })
      .catch((err) => {
        dispatch({
          type: "COMPUTE_ERROR",
          payload: {
            error_code: err.error_code || "RESOURCE_LOAD_FAILED",
            cause: err.cause || err.message,
            action: "백엔드(localhost:5000)가 실행 중인지 확인한 뒤 페이지를 새로고침하세요.",
          },
        });
      });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
}
```

- [ ] **Step 2: 테스트 작성 + 실행**

```jsx
// src/hooks/useResourceLoader.test.jsx
import { describe, expect, it, vi } from "vitest";
import { act, render, screen } from "@testing-library/react";
import { PlannerProvider, usePlannerState } from "../state/PlannerContext.jsx";
import { useResourceLoader } from "./useResourceLoader.js";
import * as client from "../api/client.js";

function Harness() {
  useResourceLoader();
  const state = usePlannerState();
  return <div data-testid="trunk-map">{state.trunkMap}</div>;
}

describe("useResourceLoader", () => {
  it("loads resources on mount and dispatches RESOURCES_LOADED", async () => {
    vi.spyOn(client, "fetchTrunkMaps").mockResolvedValue(["run_a"]);
    vi.spyOn(client, "fetchBoxPresets").mockResolvedValue({ "기본값": [] });

    render(<PlannerProvider><Harness /></PlannerProvider>);
    await act(async () => {});

    expect(screen.getByTestId("trunk-map").textContent).toBe("run_a");
  });
});
```

```bash
cd "/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU/isaacpjt/Cart2Trunk/web/frontend"
npm test -- useResourceLoader
```

Expected: 1 passed.

- [ ] **Step 3: `App.jsx` 완성**

```jsx
// src/App.jsx
import { PlannerProvider } from "./state/PlannerContext.jsx";
import { useResourceLoader } from "./hooks/useResourceLoader.js";
import { useDebouncedPlan } from "./hooks/useDebouncedPlan.js";
import Header from "./components/Header.jsx";
import ControlPanel from "./components/ControlPanel.jsx";
import SummaryCard from "./components/SummaryCard.jsx";
import Scene3DViewer from "./components/Scene3DViewer.jsx";
import BoxDetailPanel from "./components/BoxDetailPanel.jsx";
import LogPanel from "./components/LogPanel.jsx";
import styles from "./App.module.css";

function PlannerLayout() {
  useResourceLoader();
  useDebouncedPlan();

  return (
    <div className={styles.layout}>
      <Header />
      <div className={styles.body}>
        <ControlPanel />
        <div className={styles.resultArea}>
          <SummaryCard />
          <Scene3DViewer />
          <BoxDetailPanel />
        </div>
      </div>
      <LogPanel />
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

```css
/* src/App.module.css */
.layout { display: flex; flex-direction: column; height: 100vh; }
.body { display: grid; grid-template-columns: 340px 1fr; flex: 1; overflow: hidden; }
.resultArea { display: flex; flex-direction: column; gap: 16px; padding: 20px; overflow-y: auto; }
```

- [ ] **Step 4: 전체 프론트엔드 테스트 스위트 실행**

```bash
npm test
```

Expected: 모든 스위트 통과 (Task 9~18 누적, 약 20개 테스트).

- [ ] **Step 5: `web/README.md` 작성**

```markdown
# Cart2Trunk 웹 플래너 - 실행 방법

두 개의 터미널에서 백엔드와 프론트엔드를 각각 띄워야 한다.

## 1. 백엔드 (Flask, API 전용 - 화면 없음)

```bash
cd isaacpjt/Cart2Trunk/web/backend
source venv/bin/activate   # 최초 1회: python3 -m venv venv && pip install -r requirements.txt
python app.py
```

`http://localhost:5000/api/health`가 `{"status":"ok"}`를 반환하면 정상.

## 2. 프론트엔드 (React+Vite - 실제로 보고 쓰는 화면)

```bash
cd isaacpjt/Cart2Trunk/web/frontend
npm install   # 최초 1회
npm run dev
```

브라우저로 `http://localhost:5173`을 열면 실제 UI가 뜬다. 백엔드가 5000번 포트에서
먼저(또는 동시에) 켜져 있어야 트렁크 스캔 파일/박스 프리셋 목록을 불러올 수 있다.

## 테스트

```bash
# 백엔드
cd isaacpjt/Cart2Trunk/web/backend && source venv/bin/activate && python -m pytest -v

# 프론트엔드
cd isaacpjt/Cart2Trunk/web/frontend && npm test
```
```

- [ ] **Step 6: 수동 브라우저 확인 (사용자 작업)**

이 세션은 브라우저 화면을 직접 볼 수 없다 - 아래는 사용자가 두 서버를 띄운 뒤 직접 확인해야 하는 체크리스트:

1. `localhost:5173` 접속 시 트렁크 스캔 파일/박스 프리셋 드롭다운이 자동으로 채워지는가.
2. 아무 파라미터나 바꾸면 약 0.4초 뒤 3D 뷰와 요약 카드가 자동으로 갱신되는가.
3. 3D 뷰에서 마우스 드래그로 회전, `front`/`side`/`top` 버튼으로 시점이 바뀌는가.
4. 박스 상세 조회 드롭다운 선택 시 점수 분해 표가 바뀌는가.
5. 승인 → 전송 버튼이 순서대로 활성화/비활성화되고, 로그 패널에 각 단계가 기록되는가.
6. EMERGENCY STOP이 언제나(파라미터 잠금 중에도) 눌리는가.

- [ ] **Step 7: Commit**

```bash
git add isaacpjt/Cart2Trunk/web/frontend/src/hooks/useResourceLoader.js isaacpjt/Cart2Trunk/web/frontend/src/hooks/useResourceLoader.test.jsx \
        isaacpjt/Cart2Trunk/web/frontend/src/App.jsx isaacpjt/Cart2Trunk/web/frontend/src/App.module.css \
        isaacpjt/Cart2Trunk/web/README.md
git commit -m "web frontend: App.jsx 최종 배선 + useResourceLoader + 실행 방법 README"
```

---

## 완료 기준

- 백엔드: `cd web/backend && source venv/bin/activate && python -m pytest -v` 전부 통과.
- 프론트엔드: `cd web/frontend && npm test` 전부 통과.
- 사용자가 두 서버를 띄우고 Task 18 Step 6 체크리스트를 직접 확인.
- `isaacpjt/Cart2Trunk/algorism/`, `planner_gui.py`, `trunk_map_planner_node.py`에 diff가 전혀 없음 (`git diff --stat`로 확인).
