# Trunk Map 데이터 인수인계 (적재 알고리즘 파트 ROS2 연동용)

작성일: 2026-07-21 / 작성자: 지완 (스캔·트렁크 맵 파이프라인 담당)
대상: 적재 알고리즘 파트(선욱, `algorism/`) — ROS2 통신으로 trunk map을 받아 적재 계획을 세우는 쪽

---

## 1. 한 줄 요약

`13.export_trunk_map.py`가 트렁크 포인트클라우드를 **M0609 base_link 좌표계의 `trunk_map.json`**
으로 변환한다. 이 파일 하나가 "빈 트렁크"든 "박스가 있는 트렁크"든 항상 같은 스키마로 나온다
(차이는 `obstacles` 배열의 내용물뿐). 지금은 파일로만 존재하고, 이걸 ROS2 topic으로 실시간
전송하는 부분이 아직 없다 — 이 문서는 그 연동을 위한 현황 정리 + 계획이다.

---

## 2. 데이터가 지금 어디 있는지

```
isaacpjt/Cart2Trunk/results/run_YYYYMMDD_HHMMSS/pointcloud/
├── trunk_pointcloud.npy               # 원본 스캔 포인트클라우드 (world frame)
├── trunk_pointcloud_meta.json         # trunk_bounds, base_pos/base_quat, waypoint별 메타
├── trunk_map.json                     # ★ 팀에 넘길 최종 산출물 (13번 스크립트 출력)
├── trunk_pointcloud_filtered_base.ply # 정제된 포인트클라우드 (base frame, 시각 확인용)
└── trunk_map_preview.png              # 위/옆 2뷰 정적 미리보기 (Open3D GUI 없이 확인용)
```

실제 예시 두 개를 기준으로 정리함 (둘 다 이번에 생성/확인함):

| run | 원본 스크립트 | 테스트 박스 | trunk_map.json 상태 |
|---|---|---|---|
| `run_20260720_160153` | `12.trunk_scan_hidden_gripper.py` (박스 없음) | 없음 | 방금 13번 실행해서 생성함 |
| `run_20260720_200104` | `14.trunk_scan_with_test_box.py` (검증용 박스 1개) | 있음 | 이미 존재 |

두 preview.png를 비교하면 "빈 트렁크"에도 이미 obstacle 3개(휠하우스, 차량 구조물)가 잡히고,
박스가 있는 run은 거기에 2개가 더 추가된 것을 볼 수 있다 — **"완전히 빈" 트렁크는 이 차량
모델 기준으로는 없다** (휠하우스가 항상 물리적으로 존재). 적재 알고리즘 쪽에서 "빈 트렁크"를
가정할 때는 "obstacles가 0개"가 아니라 "휠하우스만 있는 상태"로 이해하면 된다.

---

## 3. 생성 파이프라인

```
12.trunk_scan_hidden_gripper.py   ─┐  (Isaac Sim 필요, isaac_python으로 실행)
14.trunk_scan_with_test_box.py    ─┤  → trunk_pointcloud.npy + meta.json
   (실제로는 향후 실물 스캔/다른   ─┘
    스캔 스크립트로 대체될 수 있음)
              │
              ▼
13.export_trunk_map.py   (Isaac Sim 불필요 — 일반 python3 + numpy/open3d/scipy/matplotlib)
   python3 13.export_trunk_map.py [run_dir]   # 생략 시 최신 run 자동 선택
              │
              ▼
        trunk_map.json  (M0609 base_link 좌표계)
```

13번은 Isaac Sim이 필요 없어서, ROS2 노드로 감싸기에 제일 적당한 지점이다 (일반 파이썬
프로세스로 바로 실행 가능).

---

## 4. `trunk_map.json` 스키마

```jsonc
{
  "schema_version": "1.0",
  "run_id": "run_20260720_200104",
  "frame": "m0609_base_link",           // 팀 Q2 합의: 트렁크/카트/박스 전부 이 좌표계 하나로 통일
  "note": "...",
  "vertices": [ [x,y,z] * 8 ],          // 0~3: 바닥 4점(실측), 4~7: 천장 4점(설계값/실측 중 낮은쪽)
  "edges":  [ {"v":[i,j], "style":"solid|dashed"} * 12 ],
  "faces":  [ {"name":"floor|wall_y_min|wall_y_max|wall_x_max|ceiling_limit",
               "v":[...], "style":"solid|dashed"} * 5 ],
  "source_stats": { "n_points_raw":.., "n_points_filtered":.., ... },
  "obstacles": [                        // 바닥 위 점유 공간 (휠하우스/기존 물건 구분 없음)
    { "name":"obstacle_1", "vertices":[[x,y,z]*8], "style":"solid", "note":"..." },
    ...
  ]
}
```

- **solid** = 이번 스캔 포인트클라우드에서 직접 실측 (바닥 + 좌/우/안쪽벽 3면).
- **dashed** = 천장(문 닫힘 높이 한계선) — 트렁크를 열고 스캔해서 실제로 존재하지 않는 면이라,
  설계 상수(`wall_top_z`)와 실측(RANSAC 평면 검출) 중 더 낮은/보수적인 값을 채택.
- **입구(x_min) 쪽은 벽 없음** — 열린 방향이라 3면(바닥+좌+우+안쪽)만 실측, 하나는 빠짐.
- `obstacles`는 AABB(8꼭짓점) 박스 리스트. "왜 막혀있는지"(휠하우스 vs 기존 물건)는 구분하지
  않고 "막혀있다는 사실"만 담는다 — 적재 알고리즘 쪽에서 extreme point 후보가 이 AABB와
  겹치는지만 체크하면 됨.

---

## 5. 이미 이 포맷을 소비하는 코드가 있음

`algorism/02_trunk_space_state.py`에 이미 파서가 구현돼 있다:

- `load_trunk_from_world_map(raw_scan_data)` — `trunk_map.json` 경로(str/Path) **또는 이미
  로드된 dict**를 받아서 `TrunkWorldMap`으로 변환.
- `load_obstacles_from_world_map(raw_scan_data, offset)` — 같은 파일의 `obstacles`를
  `PlacedBox` 리스트로 변환.

**중요**: 두 함수 다 dict를 직접 받을 수 있게 이미 짜여 있다. 즉 ROS2로 JSON 문자열을 보내고
받는 쪽에서 `json.loads(msg.data)`만 하면, 기존 파서를 한 줄도 안 고치고 그대로 붙일 수 있다.

좌표계는 Q2에서 이미 "트렁크·카트·박스 전부 M0609 base 좌표계 하나로 통일"로 합의됐고, 이
`trunk_map.json`도 정확히 그 프레임(`m0609_base_link`)으로 나온다 — 별도 좌표 변환 협의 불필요.

---

## 6. 알려진 한계 (적재 알고리즘 쪽에서 알아야 할 것)

1. **obstacle 분열**: grid+connected-component 휴리스틱이라, 실측 박스 하나가 인접한 두 개의
   `obstacle_N`으로 쪼개져 나올 수 있다 (`run_20260720_200104`의 obstacle_2/obstacle_4가 실제로
   테스트 박스 하나가 갈라진 예). 개수를 신뢰하지 말고 "겹치면 막힌 것"으로만 취급할 것.
2. **base_pos/base_quat 없는 구버전 run**: `run_20260720_160153`처럼 오래된 run은 로봇 베이스
   pose가 meta.json에 없어서 고정 상수로 근사 재구성한다 (각도 오차 0.05~0.08도 수준, 무시 가능
   하지만 완전히 정확하진 않음). 최신 run(base_pos/base_quat 저장됨)을 우선 사용할 것.
3. **재스캔 정책**: 박스 1개 배치할 때마다 트렁크를 다시 스캔한다 (`RESCAN_TRIGGER_POLICY =
   "PER_PLACEMENT"`, `02_trunk_space_state.py`에 이미 기록됨). 즉 trunk_map은 "누적 diff"가
   아니라 매번 "현재 전체 상태"를 통째로 다시 보내는 것으로 설계돼 있음.

---

## 7. ROS2 전송 계획 (제안)

목표: 위 `trunk_map.json`을 파일이 아니라 ROS2 topic으로 실시간 전달 (기획안 PDF 요구사항:
PC 간 통신은 ROS2 Topic/Service/Action). 1차 통합(7/22) 전까지 최소 구현으로 맞추는 걸 우선한다.

### 7.1 MVP안 (권장 — 파서 재작성 없이 바로 붙는 방식)

- **토픽**: `/cart2trunk/trunk_map`
- **메시지 타입**: `std_msgs/String` — payload는 `trunk_map.json`을 그대로 직렬화한 JSON 문자열
  (디스크에 쓰는 것과 바이트 단위로 동일하게).
  - 커스텀 msg(`geometry_msgs/Point[]` 등 타입화)를 새로 만드는 대안도 있지만, 그러려면
    interface 패키지 새로 만들고 `02_trunk_space_state.py`의 파서도 다시 써야 함 — 내일(7/22)
    통합 기준으로는 오버스펙. JSON string이 기존 `load_trunk_from_world_map()`을 그대로 재사용
    가능해서 제일 빠르다.
- **QoS**: `reliable` + `TRANSIENT_LOCAL`(durability), `depth=1`.
  → "래치드 토픽"처럼 동작 — Planner 노드가 스캔 완료 *이후에* 늦게 켜져도, 구독하는 순간 마지막
  trunk_map을 바로 받는다. depth=1인 이유: "diff 아니라 항상 최신 전체 상태"이므로 과거 값은
  의미 없음(6-3항 재스캔 정책과 일치).
- **발행 시점**: `13.export_trunk_map.py`가 `trunk_map.json` 저장을 마친 직후 1회. 재스캔마다
  (박스 배치 1회당 1회) 같은 토픽에 새 메시지로 재발행 — Planner는 매번 "새 전체 상태"로 갱신.
- **발행 주체**: `13.export_trunk_map.py` 끝부분에 rclpy 노드를 짧게 붙여서 저장 직후 publish +
  `spin_once` 후 종료하는 방식이 제일 간단 (별도 상시 노드/파일 watcher 불필요, 지금 파이프라인이
  "실행할 때마다 1회성 스크립트"이므로 구조가 맞음).

### 7.2 받는 쪽(Planner) 구현 스케치

```python
# 새 change: 기존 파서를 그대로 재사용
def on_trunk_map_msg(msg: String):
    data = json.loads(msg.data)
    world_map = load_trunk_from_world_map(data)          # 이미 dict 지원함
    trunk, offset = world_map.to_bounding_trunk()
    obstacles = load_obstacles_from_world_map(data, offset)
    # 이후 03~08 파이프라인에 trunk/obstacles 그대로 투입
```

### 7.3 이번 범위에서 제외 (다음 단계)

- **재스캔 트리거 신호**: `09_rescan_replan.py`의 `on_rescan_trigger()`가 아직 자리만 잡아둔
  상태 — "박스 배치 완료 → 재스캔 요청" 방향의 별도 topic/service는 이번 인수인계 범위 밖.
  (지완 쪽에서 트리거 신호 포맷 확정되면 별도로 문서화 필요)
- **커스텀 typed message**: MVP가 자리잡고 나면 `cart2trunk_msgs/TrunkMap.msg`로 정식 타입화하는
  걸 고려할 것 (지금 JSON string은 스키마 검증이 없다는 단점이 있음).

---

## 8. 액션 아이템

| 담당 | 할 일 |
|---|---|
| 지완 | `13.export_trunk_map.py`에 rclpy publish 로직 추가 (7.1 MVP안) |
| 선욱 | Planner 쪽에 `/cart2trunk/trunk_map` subscriber 노드 추가 (7.2 스케치 참고) |
| 공통 | 1차 통합(7/22) 전에 `run_20260720_200104/pointcloud/trunk_map.json`을 고정 샘플로 두고
  ROS2 없이도 파일 기반으로 먼저 파이프라인 붙여보고, 되면 topic으로 교체 |
