# Cart2Trunk — 작업 인계 문서 (다른 PC에서 이어서 하기용)

이 문서는 다른 PC에서 Claude와 함께 이 프로젝트를 이어서 진행할 때 그대로 붙여넣어 주는 용도입니다.
프로젝트 기획안 PDF(`최종_System_Requirements_Cart2Trunk.pdf`)도 같이 첨부해서 시작하세요.

## 0. 프로젝트 한 줄 요약

Cart2Trunk: NVIDIA Isaac Sim 안에서 쇼핑카트의 물품을 인식(RGB-D 스캔) → 차량 트렁크 공간을 분석 →
이동형 매니퓰레이터(모바일 베이스 + 로봇팔)가 물품을 트렁크에 자동으로 적재하는 시뮬레이션 프로젝트.
ROS2 Humble 기반, MVP 범위는 트렁크/카트 고정, 정형 박스 3종, 3~5개 물품.

## 1. 이 PC의 환경 (다른 PC에도 동일하게 필요)

- ROS2 워크스페이스 루트: `~/cobot3_ws`
- Isaac Sim 설치 위치: `~/dev_ws/isaac_sim/isaacsim` (pip 기반 설치, 5.1.0 버전)
- 실행 alias (`~/.bashrc`에 등록되어 있었음):
  ```bash
  alias isaac='~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/isaac-sim.sh'
  alias isaac_python='~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh'
  ```
- 스크립트 실행은 항상 `isaac_python <스크립트경로>.py` 형태 (일반 python3 아님, Isaac Sim 번들 파이썬 필요)
- **git 원격 저장소가 아직 없음** — 이 PC에서 다른 PC로 옮길 때 `git push`가 안 되니, 아래 방법 중 하나로 직접 옮겨야 함:
  - `isaacpjt/` 폴더 전체를 USB/클라우드로 복사, 또는
  - 이번 기회에 GitHub 등에 원격 저장소 만들고 push (권장)

## 2. 디렉토리 구조

```
cobot3_ws/isaacpjt/
├── M0609/                          # 매니퓰레이터 (기존 작업, Doosan M0609 + OnRobot RG2 그리퍼)
│   ├── Collected_m0609_camera/m0609_camera.usd   # 카메라 포함 버전
│   ├── Collected_m0609_gripper/m0609_gripper.usd # 그리퍼만
│   ├── rmpflow/                    # RMPflow 컨트롤러 설정
│   └── 1~6번 스크립트 (USD 로드 → pick&place 단계별 학습용)
└── Cart2Trunk/                     # ← 이번에 새로 만든 폴더
    ├── assets/
    │   ├── Metal_Shopping_Cart.usdz          # 쇼핑카트 (Sketchfab, CC-BY, 작가 CodAnum)
    │   ├── Lexus_IS300_Trunk_Open_No_More_Hell_Room.usdz  # 트렁크 열린 차량 (Sketchfab, CC-BY)
    │   └── shoppingcart.obj                  # (예비용, 미사용) OpenGameArt LGPL 카트
    ├── 1.load_assets_check.py         # usdz 두 개를 단위/축 보정해서 배치 + 물리 부여하는 스크립트
    ├── 2.open_saved_scene.py          # 저장된 씬(cart2trunk_scene.usd)을 바로 여는 런처
    ├── cart2trunk_scene.usd           # 지금까지 작업한 씬을 저장한 결과물 (카트+차량+큐브+콘+물리)
    └── cart2trunk_scene.zip           # Save As 시 Kit이 같이 만든 의존성 번들 (참고용)
```

## 3. 지금까지 한 일 (시간순)

1. **에셋 조사**: Isaac Sim 표준 라이브러리(SimReady, Warehouse pack 등)에는 쇼핑카트·트렁크 열린 차량이
   없다는 걸 확인. 박스 3종은 기본 Cube로 충분, 주차장은 PDF 요구사항상 Simple Grid + 구역 표시로
   충분해서 실제 3D 모델 불필요.
2. **에셋 확보**: Sketchfab에서 무료 CC-BY 라이선스로 `Metal Shopping Cart`, `Lexus IS300 Trunk Open`
   usdz 다운로드 → `isaacpjt/Cart2Trunk/assets/`로 이동.
3. **1차 로드 시도 실패 → 원인 진단**: `world_prim.GetReferences().AddReference(usdz경로)`로 단순
   참조했더니 카트가 190m, 차량이 860m 크기로 로드되고 카메라가 메쉬 안에 파묻힘.
   **원인**: Sketchfab usdz는 `metersPerUnit=0.01`(1유닛=1cm), `upAxis=Y`로 export되는데, Isaac Sim
   스테이지는 `metersPerUnit=1.0`, `upAxis=Z`. **USD의 AddReference는 단위/축을 자동 변환해주지 않음.**
4. **보정 로직 작성**: 참조하기 전에 `Usd.Stage.Open(usdz경로)`로 원본 스테이지를 열어
   `UsdGeom.GetStageMetersPerUnit`/`GetStageUpAxis`를 읽고, 스케일(×0.01)과 회전(rotateX 90도)을
   자동 계산해서 wrapper Xform에 적용하도록 `1.load_assets_check.py`의 `add_asset()` 함수를 작성함.
5. **배치**: 카트를 원점 (0,0,0)에, 차량을 (9,0,0)에 배치 (차량이 실제로 8.6m나 되는 과대 스케일
   모델이라 겹치지 않으려면 이 정도 간격 필요했음). 차량 `rotateZ=0`일 때 트렁크가 카트 쪽(-X 방향)을
   향하는 걸 사용자가 화면으로 확인함 (180도로 하면 반대로 앞머리가 향함).
6. **물리 부여**: `UsdPhysics.RigidBodyAPI` + `CollisionAPI` + `MeshCollisionAPI(convexHull)` +
   `MassAPI`를 코드로 직접 적용 (카트 15kg, 차량 1500kg). Z를 0.3m 띄워서 재생(▶) 버튼 누르면
   중력으로 떨어져 바닥에 착지하는 걸 확인용으로 넣어둠.
7. **사용자가 GUI에서 직접 큐브/콘 생성** 후 물리 적용 시도 → 여러 시행착오 끝에 성공 (아래 4번 항목 참고).
8. **씬 저장**: Isaac Sim의 File → Save As로 로컬 경로(`/home/.../Cart2Trunk/cart2trunk_scene.usd`)에
   저장 성공. `2.open_saved_scene.py`로 다음에 바로 재실행 가능하게 만들어둠.

## 4. 이번에 겪은 삽질 & 알아낸 것 (다음 PC에서 또 겪지 않도록 기록)

- **Sketchfab usdz 참조 시 반드시 단위/축 보정 필요** (3번 항목 참고). 그냥 AddReference만 하면 안 됨.
- **Isaac Sim Property 패널의 "+Add" 메뉴에 Physics 항목이 안 뜸** — 이 빌드에는 Physics UI 확장이 안
  켜져 있음. 대신 파이썬 코드(`pxr.UsdPhysics`)로 직접 스키마를 붙이면 됨 (GUI 없이도 정상 작동).
- **화면 하단 "Console" 탭은 파이썬 REPL이 아니라 명령어 전용 콘솔** (`CLEAR, HELP, HISTORY, QUIT,
  OPEN, CLOSE`만 가능). 여러 줄 코드를 붙여넣으면 줄바꿈이 사라지고 `import`를 명령어로 착각해서 에러남.
  - 진짜 파이썬을 실행하려면 **Window → Script Editor**를 찾아서 써야 함 (또는 못 찾으면 아래 방법).
  - 대안: 파이썬 스크립트 파일을 아래 경로 중 하나에 저장해두면 Console에서 `OPEN <파일이름(확장자 제외)>`
    명령으로 즉시 실행 가능:
    - `~/.cache/packman/chk/kit-kernel/<버전문자열>/scripts`
    - `~/Documents/Kit/apps/Isaac-Sim Python/scripts`
    - `~/Documents/Kit/shared/scripts`  ← 이번에 이 경로에 `apply_physics.py`를 만들어서 씀
- **File → Save As 기본값이 Omniverse Nucleus 경로(`omniverse://...`)라 저장 실패함** — Nucleus 서버가
  없으니 당연히 실패. 저장 다이얼로그에서 "Local Drives / My Computer"로 바꾸거나 경로 입력창에
  `/home/...` 로 시작하는 절대경로를 직접 타이핑해야 로컬 저장이 됨.
- **`isaac_python` 스크립트를 stdout을 파일로 리다이렉트해서 백그라운드 실행하면 파이썬 print가
  버퍼링돼서 로그에 안 찍힘** — 확인용 스크립트는 항상 `PYTHONUNBUFFERED=1` 붙여서 실행할 것.

## 5. 다음 PC에서 바로 이어서 하는 법

```bash
# 1) 이 폴더 전체를 새 PC의 동일 경로(~/cobot3_ws/isaacpjt/Cart2Trunk)로 복사
# 2) Isaac Sim이 설치돼 있고 alias가 잡혀있는지 확인 (없으면 새로 설치 필요)
cd ~/cobot3_ws/isaacpjt/Cart2Trunk
isaac_python 2.open_saved_scene.py    # 저장된 씬 그대로 열림
# 재생(▶) 버튼 눌러서 물리 확인
```

## 5-1. 콜리전 버그 수정 (2026-07-17)

**증상**: 카트 바구니/트렁크 안에 박스를 떨어뜨리면 안에 안착하지 않고 튕기거나 카트/차량
윗면에 얹힘.

**원인**: `1.load_assets_check.py`의 `add_rigid_physics()`가 카트·차량 메쉬 전체에
`approximation="convexHull"`을 적용했는데, 두 usdz 에셋이 전부 서브메쉬 분리 없이
통짜 메쉬 1개(`Object_0`, Sketchfab의 `_materialmerger_gles` 병합 export)라서
convexHull이 바구니/트렁크의 오목한 내부 공간을 통째로 "채워진 덩어리"로 뭉개버렸음.
`isaac_python`으로 usdz 내부를 직접 열어 `pxr.Usd`로 메쉬 개수를 확인해서 원인 확정.

**조치**: `3.fix_container_collision.py` 작성.
- 시각 메쉬(`/World/ShoppingCart`, `/World/Vehicle`)에는 콜리전을 아예 안 붙임 (순수 시각용)
- 대신 `/World/ShoppingCart_Collision/{Floor,Wall_Front,Wall_Back,Wall_Left,Wall_Right}`,
  `/World/Vehicle_Collision/{Floor,Wall_Near,Wall_Far,Wall_Left,Wall_Right}` 를
  보이지 않는 `UsdGeom.Cube` 프록시로 만들어 바닥+벽 형태의 단순 콜라이더로 붙임
  (기획안 9.4/9.5절 "권장 Stage 구조"와 동일한 접근)
- 트렁크 치수는 PhysX `raycast_closest`로 실측 (바닥 z=0.61, x=[9.3,10.0], y=[-1.1,0.9]).
  카트 바구니는 철사 메쉬 틈으로 광선이 대부분 통과해버려서(관통) 정밀 실측이 안 됐고,
  전체 bbox 대비 inset 비율로 추정 (x_half=0.40, y_half=0.55, floor_z=0.30, wall_top_z=0.80).
  → **나중에 실제 3종 박스 배치할 때 카트 쪽 치수는 눈으로 보면서 미세조정 필요할 수 있음.**
- 카트/차량은 기획안 MVP 스펙대로 완전 정적(RigidBody 없음, 고정)으로 둠. 이전 버전에 있던
  "0.3m 띄워서 중력 낙하 확인" 테스트는 제거 (MVP는 카트/트렁크 고정이 맞는 스펙이라 불필요).
- 검증: PDF 규격 Small 박스(300×200×150mm)를 바구니/트렁크 위에서 낙하시켜 180스텝 시뮬레이션
  후 최종 위치를 자동 체크 → 둘 다 PASS (바구니: z=0.39 안착, 트렁크: z=0.70 안착, 둘 다
  경계 안쪽). 씬을 다시 열어(reload) 재시뮬레이션해도 같은 위치 유지 확인 완료.
- `cart2trunk_scene.usd`를 이 결과로 덮어씀 (기존 파일은 `cart2trunk_scene.usd.bak_before_collision_fix`로 백업).
  검증용 낙하 박스 2개(`/World/TestBox_Cart`, `/World/TestBox_Trunk`)는 씬에 그대로 남아있음 —
  다음에 실제 3종 박스로 교체/삭제하면 됨.

**참고**: 차량 에셋 자체가 실제 크기보다 약 2배 과대 스케일(HANDOFF 3번 항목에서 이미 알던 문제,
전체 길이 8.6m)이라 트렁크 실측값도 그 스케일 그대로 반영된 것 — 이번 수정에서는 건드리지 않음.

**추가 버그 수정 (같은 날)**: 사용자가 실제로 `isaac_python 3.fix_container_collision.py`를 GUI로
실행했더니 검증(PASS)과 저장까지는 잘 되는데, 그 직후 `world.step()`에서
`AttributeError: 'World' object has no attribute '_scene'`로 죽는 문제 발견.
원인: `omni.usd.get_context().save_as_stage()`가 스테이지를 다시 여는 이벤트를 내부적으로
발생시켜서 `World` 싱글턴을 무효화시키는데("A new stage was opened, World or Simulation Object
are invalidated" 경고 로그로 확인), 그 뒤에도 계속 `world.step()`을 부르고 있어서 크래시남.
조치: 저장을 인터랙티브 루프 "이후"(창 닫을 때)로 옮기고, 루프 안에서는 `world.step()` 대신
`simulation_app.update()`만 사용하도록 수정 — World 객체를 아예 안 건드리니 무효화 문제 자체가
발생하지 않음. 60프레임 bounded 테스트로 재검증 완료(PASS + 저장 성공, 크래시 없음).

**좌표 재수정 (같은 날, 중요)**: 사용자가 GUI에서 직접 확인해보니 카트 박스는 바구니가 아니라
바퀴 축/프레임 높이에 떠있었고, 트렁크 박스는 트렁크 근처에도 없었음 (스크린샷으로 확인).
원인 분석: 숫자(raycast, bbox)만 믿고 스크린샷으로 검증을 안 했던 게 근본 문제.
- **카트**: "바구니 바닥 z=0.30"이 실제로는 바퀴 축 높이였음. 여러 후보 높이(0.30/0.55/0.80/1.00/1.20)를
  세로로 마커로 찍어 스크린샷 비교한 결과, 실제 바구니 바닥은 **z≈0.78** (예전 raycast에서 "구조물"로
  오인해서 버렸던 신호가 사실 바닥이었음).
- **트렁크**: 이게 더 큰 실수였음 — CAR_POS=(9,0,0)이 차량 "중심" 좌표인데, 예전에 raycast로 측정한
  "x=9.3~10.0"은 차량 중심 부근(캐빈/뒷좌석 부근)이었지 트렁크가 아니었음. 트렁크는 차량 bbox의
  **낮은 x쪽 끝(범퍼 방향)** 에 있음 (차량 지붕 위에 x=5.0/9.0/13.0 마커를 찍어 스크린샷으로 확정).
  그 구간(x=4.6~7.2)을 다시 raycast로 정밀 측정해서 **바닥 z≈0.88, x=[5.15,6.38], y=[-0.85,0.85]**
  로 재확정. 트렁크 안에 스페어타이어 커버로 보이는 돌출부가 있어서 그 옆 평평한 바닥에 안착함.
- `3.fix_container_collision.py`의 CART_BASKET_FLOOR_Z=0.78, TRUNK 좌표를 위 값으로 수정.
  낙하 테스트 후 **스크린샷까지 자동 촬영**하도록 스크립트에 `snapshot()` 단계 추가함
  (`_verify_cart.png`, `_verify_trunk.png`로 저장, 트렁크는 원점에만 있는 기본 조명이 9m 밖까지
  안 닿아서 트렁크 위에 조명(`TrunkAreaLight`)도 추가로 배치). 두 스크린샷 모두에서 박스가 실제
  바닥에 놓인 것을 육안 확인 완료.
- **교훈**: 숫자 좌표 검증(PASS/FAIL)만으로는 "그 좌표 자체가 틀렸을 가능성"을 못 잡는다.
  자체 정의한 기준(bounds)이 잘못되면 그 기준 안에서는 항상 PASS가 나옴. 컨테이너 좌표를 다룰 땐
  항상 스크린샷으로 실제 메쉬와 겹쳐서 확인할 것.

## 5-2. 박스 근사를 버리고 SDF 콜리전으로 전환 (같은 날, 최종)

좌표를 고친 뒤에도 사용자가 GUI에서 테스트박스를 직접 옆으로 옮겨보고 구조적 한계를 지적함:
- **트렁크**: 옆쪽에 휠하우스(바퀴집) 돌출부가 있는데, 손으로 만든 "바닥 1장 + 수직벽 4장" 박스는
  이 돌출부를 무시하고 평평하게만 깔려있어서 중앙은 맞아도 사이드는 실제 메쉬와 어긋남.
- **카트**: 바구니가 아래보다 위가 넓은 사다리꼴(테이퍼) 형태인데, 수직벽으로 근사하면 바닥
  기준으로 맞추면 위쪽에서 실제 벽보다 안쪽에 콜리전이 뜨고, 위쪽 기준으로 맞추면 아래쪽에서
  실제 메쉬 바깥으로 콜리전이 붕 뜸 — 손으로 측정해서 벽/바닥 몇 개로 근사하는 방식 자체의
  구조적 한계.

**최종 조치**: 손으로 만든 Box 프록시 콜라이더(`ShoppingCart_Collision`, `Vehicle_Collision`)를
전부 제거하고, **시각 메쉬(`Object_0`) 자체에 SDF(Signed Distance Field) 콜리전을 직접 적용**하는
방식으로 전환:
```python
UsdPhysics.CollisionAPI.Apply(mesh_prim)
UsdPhysics.MeshCollisionAPI.Apply(mesh_prim).CreateApproximationAttr().Set("sdf")
PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(mesh_prim).CreateSdfResolutionAttr().Set(256)
```
SDF는 오목한 형태를 그대로 보존하면서 실제 삼각형 표면을 따라가는 PhysX의 콜리전 방식이라
(convexHull처럼 오목한 부분을 채우지도 않고, 손으로 만든 박스처럼 테이퍼/돌출부를 무시하지도
않음), 카트의 사다리꼴 벽이나 트렁크의 휠하우스 굴곡을 치수 측정 없이 있는 그대로 반영한다.
정적 메쉬(RigidBody 없음)에 붙이는 방식이라 카트 바구니 철사 틈으로 물체가 새는 문제도 없음
(해상도 256이면 철사 두께보다 충분히 촘촘함).

**검증**: 중앙 + 옆쪽(휠하우스/사다리꼴 벽 근처) 총 4곳에 낙하 테스트박스를 놓고 4초 시뮬레이션 →
전부 실제 지형을 따라 자연스럽게 안착 (옆쪽 박스는 굴곡을 따라 미끄러져 이동한 뒤 정착 —
평평한 박스 콜라이더였다면 절대 안 나오는 움직임이라 실제 메쉬를 따라가고 있다는 증거).
스크린샷(`_verify_cart.png`, `_verify_trunk.png`)으로 육안 확인 완료, 씬 저장 후 재로드해도
같은 위치 유지 확인 완료.

이제 `ShoppingCart_Collision`/`Vehicle_Collision`/`CART_BASKET_*`/`TRUNK_*` 좌표 상수는 스크립트에서
전부 삭제됨 — 더 이상 좌표를 손으로 잴 필요가 없다.

## 5-3. 박스 3종 + 8개 구역 마커 추가 (2026-07-17)

`4.add_boxes_and_zones.py`: 기존 씬을 열어서 검증용 테스트박스 4개를 지우고, 기획안 규격의
실제 작업 대상 박스 3종(Small 300x200x150mm / Medium 400x300x250mm / Large 500x350x300mm,
질량은 PDF에 명시 안 돼서 1.0/2.0/3.5kg으로 추정)을 카트 위에서 낙하시켜 SDF 콜리전 바닥에
안착시킴. 3개가 서로 기대어 자연스럽게 쌓인 형태로 정착 (스크린샷 `_verify_boxes.png`로 확인).

`5.fix_zone_layout.py`: PDF "1.2 환경 구성"의 8개 구역(Vehicle Parking Slot, Cart Waiting Zone,
Robot Waiting Zone, Loading Working Zone, Safety Working Zone, AMR Navigation Path,
Trunk Occupancy Region, Cart Return Zone)을 색깔 있는 평면 마커로 표시. 1차 배치는 구역들이
서로 너무 붙어있어서(특히 Safety Working Zone이 Loading Working Zone을 꽉 채운 사각형이라
겹쳐 보임) "다 한 공간처럼 보인다"는 피드백을 받고 재배치함:
- Safety Working Zone을 꽉 찬 사각형이 아니라 **테두리(프레임) 4조각**으로 바꿔서 안쪽
  Loading Working Zone과 시각적으로 명확히 분리
- Vehicle Parking Slot을 실제 차량 bbox 크기로 줄여서 Loading Working Zone과의 겹침 최소화
- Robot Waiting Zone / Cart Return Zone을 카트 기준 서로 다른 방향으로 떨어뜨려 배치
- AMR Navigation Path는 Robot Waiting Zone → Loading Working Zone을 잇는 좁은 통로로 재설계

구역 좌표는 PDF에 정확한 수치가 없어서 현재 씬 배치(카트=원점, 차량=(9,0,0), 트렁크 캐비티=
x:5.15~6.38) 기준으로 합리적으로 설계한 추정치임 — 스크린샷(`_verify_zones_top.png`,
`_verify_zones_wide.png`)으로 8개가 시각적으로 구분되는 것 확인 완료.

## 6. 아직 안 한 것 / 다음에 할 일 (PDF 기획안 기준)

1. ~~**박스 3종 추가**~~ — **완료 (2026-07-17, 5-3절)**
2. ~~**환경 구역 표시**~~ — **완료 (2026-07-17, 5-3절)**
3. ~~**이동형 매니퓰레이터 결합**~~ — **완료 (2026-07-17, 5-4절 참고, Nova Carter + M0609)**

   (아래는 RidgebackFranka 시도 → 폐기 과정 기록. 최종 결과는 5-4절 참고)

   **확인한 것**: `isaacsim.storage.native.get_assets_root_path()`로 Isaac Sim 5.1 표준 에셋 CDN
   (S3, 인터넷 접근 됨)에 접근 가능함을 확인. 여러 후보 경로를 직접 열어봐서(`Usd.Stage.Open`)
   실존 여부 테스트:
   - `Isaac/Robots/Clearpath/Ridgeback/ridgeback.usd` (베이스 단독) → **없음**
   - `Isaac/Robots/Clearpath/RidgebackFranka/ridgeback_franka.usd` (Ridgeback+Franka 팔 결합) → **있음**
   - `Isaac/Robots/UniversalRobots/ur5.usd`, `ur10.usd` → 있음
   - `Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd` (다른 회사 AMR) → 있음

   `ridgeback_franka.usd`를 실제로 씬(Robot Waiting Zone 위치)에 참조해서 넣어보니 로드 성공,
   `world.reset()` + 60스텝 물리도 에러 없이 통과, 스크린샷(`_try_ridgeback.png`)으로 렌더링까지
   확인함. **즉 "Ridgeback 베이스 자체"는 이 환경에서 문제없이 쓸 수 있음.**

   **막히는 지점**: Ridgeback이 **Franka 팔과 하나의 스키마로 완전히 합쳐진 단일 articulation**으로만
   제공되고, "베이스만 따로" 참조할 수 있는 모듈형 에셋이 아님 (`panda_link0~7`, `panda_hand`,
   손가락까지 전부 `ridgeback_franka_robot_schema.usd` 한 레이어 안에서 base_link/arm_mount_link와
   함께 정의됨). 그래서 M0609로 교체하려면:
   1. `panda_link0~7`, `panda_hand`, `panda_leftfinger`, `panda_rightfinger` 및 관련 조인트를 전부 삭제
   2. `arm_mount_link` 밑에 기존 `isaacpjt/M0609/Collected_m0609_gripper/m0609_gripper.usd`를 새로
      참조 + `panda_arm_mount_joint`를 대체할 새 Fixed Joint 작성
   3. M0609의 6개 회전 조인트 drive(stiffness/damping)가 제대로 물려있는지, 전체가 하나의
      Articulation으로 유효하게 묶이는지 검증
   4. `isaacpjt/M0609/rmpflow/`의 RMPflow 컨트롤러는 **고정 베이스** 기준으로 설정된 것이라, 모바일
      베이스 위로 옮기면 기준 프레임(base frame)이 더 이상 고정이 아니게 되어 컨트롤러 설정도
      다시 손봐야 함
   
   이건 "에셋 참조 + 배치" 수준(카트/차량 때 했던 작업)이 아니라 **로봇 구조 자체를 재조립하는 작업**이라
   상당한 추가 작업이 필요함. 참고로 Ridgeback 베이스도 실제 4륜 구동 물리가 아니라 X/Y/yaw
   "dummy" prismatic/revolute 조인트로 단순화된 형태라(RL 데모용), 이 자체도 완전한 AMR 물리
   시뮬레이션은 아님.

   **최종 결론: RidgebackFranka 폐기, Nova Carter + M0609로 완료함.**

## 5-4. Nova Carter + M0609 모바일 매니퓰레이터 결합 완료 (2026-07-17)

RidgebackFranka 미리보기(`6.preview_ridgeback_franka.py`)를 사용자가 직접 확인해보고 "팔이
너무 작아서 카트 물건 옮기기 어려워 보인다"는 피드백 → RidgebackFranka 폐기, **Nova Carter**
(NVIDIA 표준 AMR, `Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd`)로 전환.

Nova Carter를 까보니 (RidgebackFranka와 달리) **베이스 전용 모듈형 에셋**임을 확인:
- `chassis_link`(몸체, `PhysicsArticulationRootAPI` 보유) + `wheel_left`/`wheel_right`(실제
  `PhysicsDriveAPI`가 붙은 차동구동 바퀴) + 캐스터 바퀴 2조
- 카메라(hawk×3, owl×3) + LiDAR×3 + IMU 센서 기본 탑재
- 팔이 전혀 없어서 M0609 교체 "수술"이 필요 없음

`7.build_mobile_manipulator.py`로 결합:
1. Nova Carter를 Robot Waiting Zone(-0.6, -2.65)에 배치, 카트/작업존 방향(약 37도)으로 회전
2. M0609(`isaacpjt/M0609/Collected_m0609_gripper/m0609_gripper.usd`의 `/World/m0609`
   서브패스만)를 Nova Carter chassis 위(Z+0.42m)에 같은 회전으로 배치
3. **두 개의 별도 articulation을 하나로 합침**: M0609의 `root_joint`(원래 "world에 고정"용
   FixedJoint + ArticulationRoot)에서 `ArticulationRootAPI`/`PhysxArticulationAPI`를 제거하고,
   `body0` 관계를 Nova Carter의 `chassis_link`로 재지정 → Nova Carter의 `chassis_link` 하나가
   바퀴+팔 전체를 아우르는 유일한 ArticulationRoot가 됨
4. **조인트 드라이브 강성 재설정 필수**: `4.pick_place.py`/`5.pick_place_color.py`에 있던 것과
   동일한 단계 — USD에 저장된 기본 드라이브 게인이 약해서 이 단계 없이는 팔이 축 늘어짐.
   `stiffness=1e8, damping=1e4, maxForce=1e8`를 모든 조인트에 적용해야 함.

**중간 삽질**: 처음 스크린샷에서 팔이 앞으로 축 처져 보여서 "드라이브 강성 문제"로 오판하고
강성을 줬는데도 똑같은 모양이길래, M0609를 고정 베이스로 단독으로 띄워서 비교해봄 →
**완전히 같은 모양**이었음. 즉 이건 버그가 아니라 M0609의 정상적인 "0도 관절 자세"(팔꿈치가
앞으로 접힌 형태)였음 — 판단하기 전에 항상 "원래 이렇게 생겼나?"부터 비교 확인할 것.

**두 번째 진짜 버그 (사용자가 스크린샷 비교로 잡아냄)**: 위 3번 단계에서 `body0`을
chassis_link로 재지정할 때 **`localPos0`(조인트 앵커가 chassis_link 로컬 좌표계 어디인지)를
안 바꿔서 계속 `(0,0,0)`**으로 남아있었음. 그 결과 M0609 Xform에는 분명 Z+0.42 이동을
줬는데도, 실제 물리 시뮬레이션은 조인트가 정한 위치(=chassis_link 원점, 거의 바닥 높이)로
`base_link`를 그대로 붙여버렸음 — 즉 **팔이 Nova Carter 위가 아니라 사실상 바닥에 붙어있는
것처럼 보였음**. Xform의 translate는 물리 관절로 묶인 바디에는 초기값일 뿐, 실제 시뮬레이션
위치는 조인트의 localPos0/localPos1이 결정한다는 걸 놓쳤던 것.
- 원인 파악: `UsdPhysics.Joint(root_joint_prim).GetLocalPos0Attr().Get()`으로 직접 확인하니
  `(0,0,0)`이었음.
- 조치: `joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, MOUNT_Z))`로 chassis_link 로컬
  +Z 방향으로 0.42m 앵커를 옮김.
- 검증: 재로드 후 `base_link` 월드 z=0.420(정확히 MOUNT_Z), chassis_link 기울어짐도
  0.05~0.08도로 사실상 완전 수평이 됨 (수정 전에는 무게중심 쏠림으로 3.5도 기울어 있었는데,
  올바른 위치로 고정되니 그 문제도 같이 해결됨). 스크린샷으로 Nova Carter 위에 수직 기둥이
  서고 그 위에서 팔이 뻗어나가는 정상적인 모양 확인 완료.

물리 검증(120스텝) 통과, 스크린샷(`_verify_mobile_manip_wide.png`, `_verify_mobile_manip_close.png`)
으로 육안 확인, 씬 저장 후 재로드해도 위치/articulation 구조 그대로 유지됨을 확인.
(사소한 경고 로그 하나 있음: `onrobot_rg2ft/world/visuals`의 서브 USD 참조 경로 하나가
못 풀리는데 렌더링에는 영향 없어 보임 — 나중에 원인 파악하면 좋음.)

**교훈**: 물리 조인트로 연결된 바디는 USD Xform의 translate/rotate가 "초기값 힌트"일 뿐,
실제 시뮬레이션 위치는 조인트의 localPos/localRot이 결정한다. body0/body1 관계(누구에게
붙일지)만 바꾸고 localPos는 안 바꾸면, 딱 "새 부모의 원점"에 눌러붙는다.

**남은 것**: 지금은 정지 상태로 "얹혀만" 있음. 다음 단계는 Nova Carter의 실제 차동구동
바퀴를 코드로 제어해서 이동시키는 것, 그리고 M0609 RMPflow 컨트롤러가 고정 베이스 가정을
쓰고 있어서 모바일 베이스 기준으로 좌표계를 다시 잡아야 pick&place가 가능함.

4. ~~**트렁크 콜리전 단순화**~~ — **완료 (2026-07-17, 5-1/5-2절 참고)**. 처음엔 Box 프록시 콜라이더로
   시도했으나 좌표 오류 + 테이퍼/휠하우스를 못 따라가는 구조적 한계 둘 다 발견되어, 최종적으로
   **SDF 콜리전**(실제 메쉬 형상을 그대로 따라감)으로 전환. `3.fix_container_collision.py`가 카트/차량
   메쉬에 SDF를 직접 적용하고, 중앙+사이드 4곳 낙하 테스트 + 스크린샷 검증까지 통과.
5. **RGB-D 센서 부착 및 스캔 파이프라인** — M0609 손목에 달린 RealSense D455로 카트/트렁크를 다중 시점
   스캔해서 Point Cloud 생성하는 로직 (PDF 2.1절 4~7단계).
6. **물품 분석·적재 계획 로직** — Point Cloud → 물체 분할/부피 계산 → 집기 후보 생성 → 트렁크 점유맵
   계산 → 적재 순서/위치 결정 (PDF 2.1절 5, 6, 8, 9단계). 아직 전혀 구현 안 됨.
7. **ROS2 Humble 브릿지 연결** — PDF는 PC 간 통신을 ROS2 Topic/Service/Action으로 하라고 명시.
   `isaacsim.ros2.bridge` 익스텐션은 M0609 스크립트에서 이미 써본 적 있음 (`enable_extension`), 이걸
   Cart2Trunk 파이프라인에도 연결해야 함.

## 7. 참고 — 지금 배치 좌표 (재현/조정용)

```python
CART_POS = (0.0, 0.0, 0.3)     # 쇼핑카트, 원점
CAR_POS  = (9.0, 0.0, 0.3)     # 차량, 카트에서 9m 떨어짐 (차량이 8.6m로 과대 스케일이라 이 정도 간격 필요)
CAR_ROT_Z = 0.0                # 이 값일 때 트렁크가 카트 쪽(-X)을 향함 (확인 완료)
```
