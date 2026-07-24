# Cart2Trunk Vision/Perception 작업 기록 (노션용)

**범위**: 테이블 다중 시점 스캔 최초 구축 + 동일 크기 적층 박스 검출 + 카트 홀로노믹 베이스 다중 시점 스캔 + 카트 안 "다른 크기 박스 여러 개가 하나의 큰 박스 위에 나란히 적층된" 구조 인식
**기간**: 2026-07-23 ~ 2026-07-24
**관련 담당 영역**: `Cart2Trunk 담당자별 최종 실행 가이드라인.pdf` 2번 "준형님 — Vision / 3D Perception"

이 문서는 사소한 파라미터 조정 하나하나가 아니라, **스크립트가 죽거나 결과물이 완전히
틀리게 나왔던 크리티컬한 문제**만 골라서 원인 → 조치 → 그 시점에 만들어진 파일 순으로
정리했다. 스캔 "동작"(로봇 움직임) 자체를 봐야 하는 항목은 영상 녹화용으로 나중에
Isaac Sim으로 재현 요청하면 그때 열 수 있도록, 재현에 필요한 정보만 기록해두고 지금은
로그/스크린샷으로만 남긴다.

---

## Part A. 테이블 다중 시점 스캔 최초 구축 (35.crate_scan_setup.py)

모든 작업의 출발점. 원래 있던 단일 시점 검출(`box_top_extractor.py`)을 대체하는 새
다중 시점 파이프라인을 이 단계에서 처음 만들었다.

### A-1. 기존 단일 시점 검출의 근본적 한계 (착수 배경)
- 기존 `box_top_extractor.py`는 카메라 좌표계 기준 "카메라를 향하는 평면 + 대표 법선
  재선정" 2단계 동적 필터를 썼는데, 대표 법선이 옆면으로 잘못 뽑히는 등 오탐/미검출이
  반복됐다. 단일 시점이라 박스가 적층되거나 서로 가까이 있으면 가려짐(occlusion) 때문에
  윗면 일부만 관측되는 구조적 한계도 있었다.
- **조치**: 여러 시점의 point cloud를 `m0609_base_link` 좌표계로 합쳐서, 카메라 상대
  법선 대신 절대 up 벡터(0,0,1) 기준 단일 필터로 검출하는 완전히 새로운 파이프라인 설계.
- **신규 생성 파일**: `perception/box_geometry.py`(핵심 검출 로직), `perception/multiview_scan.py`(배치 파이프라인), `perception/run_scan_batch.py`(CLI 진입점)

### A-2. Small 박스 윗면이 비스듬하게 잘리는 문제
- **증상**: 스캔한 PLY를 확인해보니 적층된 작은 박스(Small)의 윗면이 평평하지 않고
  비스듬하게 잘려 나옴. 사용자가 직접 PLY를 열어보고 지적.
- **원인**: RANSAC이 피팅한 평면의 법선이 노이즈로 미세하게 기울어져 있었는데, 그
  기울어진 법선을 그대로 써서 윗면을 복원하고 있었음.
- **조치**: 검증된 평면은 실제 fit된 법선 대신 절대 up 벡터로 강제 정렬 — 노이즈나
  미세한 기울임과 무관하게 항상 평평한 윗면이 복원되도록 변경.
- **관련 파일**: `perception/box_geometry.py`

### A-3. 스캔 궤적(azimuth) 폭 반복 조정
- **사용자 지시**: "섀시를 반대로 이동하는 로직은 안 만든다 — 팔의 azimuth 스윙 폭을
  넓혀서 왼쪽면-앞쪽-오른쪽면 시점을 다양하게 만들어라"
- ±40도로 시작 → 시야 다양성 부족 → ±60도까지 넓혀서 테스트 → **IK 오차 0.17m로 팔의
  도달 한계를 확실히 초과**함을 확인(정렬 자체는 0.995~0.996으로 유지됐지만 오차가 너무
  컸음) → 최종 **±50도**로 확정(사용자 명시적 지시: "±50도로 안전 마진 두고 다시
  실행시켜봐")
- 추가로 az=0 고정, tilt만 다르게(8도/28도) 주는 시점 2개를 더해 총 9개 시점 확정 —
  같은 방향으로만 편향되는 관측을 완화하기 위함.
- **관련 파일**: `35.crate_scan_setup.py`

### A-4. 스캔 속도 최적화 후 정확도 검증
- 조기 종료(plateau) 수렴 로직 추가 — 목표 근처에서 더 이상 안 움직이면 남은 스텝을
  다 채우지 않고 바로 멈춤. 전체 실행 시간 약 27% 단축.
- 속도만 빠르고 정확도가 떨어지면 안 되므로, 같은 시나리오를 조기종료 적용 전/후로 각각
  돌려서 최종 자세·오차를 직접 비교(A/B 테스트) — 차이 없음을 확인한 뒤 채택.
- **관련 파일**: `35.crate_scan_setup.py`

### A 최종 산출물
- 다중 시점 원본 point cloud: `perception/scan_cache/merged_table_scan.npy`, `merged_table_scan.ply`, `merged_table_scan_stacked.ply`, `merged_table_scan_stacked_az50.ply`
- 신규 코드: `perception/box_geometry.py`, `perception/multiview_scan.py`, `perception/run_scan_batch.py`
- 최종 확정 스캔 설정: azimuth ±50/±40/±20/0도 + tilt 8/28도(총 9개 시점)
- 변경사항 요약 PDF: `cart2trunk_multiview_scan_changes.pdf` (참고: 처음엔 reportlab으로 만들려다 한글 폰트(CJK) 렌더링이 깨져서, HTML + headless Chrome 방식으로 재작성함)

---

## Part B. 동일 크기 적층 박스 검출 (perception/)

Part A의 파이프라인은 "위 박스는 항상 아래 박스보다 작다"는 전제를 깔고 있었는데,
위아래 박스 크기가 완전히 같은 경우까지 대응해야 했다. 시행착오 전체 원문은
`perception/SAME_SIZE_STACK_DETECTION_LOG.md`에 더 상세히 남아있고, 여기는 그중
크리티컬한 것만 요약.

### B-1. 그룹핑/지지면 오탐으로 작은 박스가 아예 안 잡힘
- **증상**: Small을 Large 위에 스택시켰는데 검출 결과에 Small이 없고, Large만 높이가
  실제의 2배로 잘못 나옴.
- **원인**: ① `_group_by_location()`이 후보를 XY 거리만으로 묶어서, 같은 XY·다른 Z에
  있는 적층 박스들이 하나의 그룹으로 합쳐지고 그 안에서 fill_ratio가 항상 높은 Large만
  선택됨. ② 그 다음엔 테이블 표면 전체(넓고 평평해서 사각형 채움비를 쉽게 통과)가
  우연히 Small의 "지지면"으로 오인됨.
- **조치**: 그룹핑에 Z-tolerance AND 조건 추가, `select_support_candidate()`에
  `max_support_area_ratio` 파라미터 추가(자기 면적의 6배 넘는 후보는 지지면에서 제외).
- **관련 파일**: `perception/multiview_scan.py`, `perception/box_geometry.py`

### B-2. 숨겨진 박스 탐색 알고리즘 설계를 통째로 갈아엎음
- **1차 시도(폐기)**: 경계 돌출량을 깊이별 히스토그램으로 찾는 방식. 독립된 박스의
  옆면(수직면)을 진짜 표면으로 오인하거나, 반대로 진짜 적층 케이스도 "옆면 같다"며
  계속 걸러내는 반대쪽 실패가 동시에 발생 — 민감도를 조정해도 한쪽이 항상 깨짐.
- **2차 설계(채택)**: `detect_floor_boundary()`가 원래 "가장 가까운 평면"을 채택하도록
  설계돼 있다는 점을 이용해, 그 지지면을 다시 새로운 top으로 놓고 재귀적으로
  `detect_floor_boundary()`를 반복 호출(`find_stacked_layers()`) — 몇 겹까지 내려가는지
  자체를 여러 번 반복 + 다수결로 판정.
- **부가 버그**: 2단계(진짜 바닥) 재탐색이 전체 scene을 대상으로 하다 보니 "우연히
  비슷한 높이의 다른 물체"와 평면이 합쳐져 깊이가 짧게 나옴 → 1단계는 전체 scene,
  2단계부터는 top 주변 국소 영역으로 크롭(`_crop_local_region`)해서 해결.
- **관련 파일**: `perception/box_geometry.py` (`find_stacked_layers`, `_single_descent_trial`, `_crop_local_region`, `flat_plane_support_at_depth`)

### B-3. 유령 박스 3종 세트
같은 클래스의 문제가 세 번 다른 모습으로 나타났다 — 전부 "실제 박스가 아닌데 후보
기준을 통과해서 최종 결과에 남는" 패턴.
- 폭 4~5cm짜리 가늘고 긴 RANSAC 조각 → `MIN_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M`(0.08m) 추가
- fill_ratio ~0.50짜리 부실한 후보가 간헐적으로 등장 횟수 기준을 통과 → `MIN_FINAL_FILL_RATIO`(0.75) 추가
- (카트 시나리오에서) 테이블/카트 림처럼 실제보다 훨씬 큰 구조물이 통째로 하나의 박스로 잡힘 → `MAX_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M`(0.35m) 추가
- **관련 파일**: `perception/multiview_scan.py`

### B-4. XY 오프셋 방향이 실제로 반대로 나왔던 버그
- **증상**: 재구성한 숨겨진 박스 사각형이 실제 점군과 전혀 다른(빈 공간) 위치를 가리킴.
  사용자가 PLY를 직접 열어보고 "완벽하게 안 겹친다"고 지적하며 발견됨.
- **원인**: 경계 양쪽 변에 증거가 다 있을 때 평균을 냈는데, 한쪽은 진짜 신호(점 많음)고
  반대쪽은 노이즈(점 적음)라 평균이 부호까지 뒤집힘.
- **검증 방법**: `m0609_base_link`이 world 대비 정확히 90도 회전돼 있다는 걸 테이블
  자신의 검출된 변 방향으로 확인 → 스폰 오프셋을 정확히 변환해서 참값과 비교 →
  raw point cloud를 직접 시각화(matplotlib)해서 눈으로 재확인.
- **조치**: "점 개수가 더 많은 쪽만 채택"으로 변경. 이후 3.6cm였던 시나리오 오프셋을
  7.2cm로 넓혀서(물리적 안정성 범위 내) 절대오차 대비 상대오차를 40~70% → ~10%로 개선.
- **관련 파일**: `perception/box_geometry.py`, `35.crate_scan_setup.py` (TABLE_BOXES 오프셋 값)

### B 최종 산출물
- JSON: `~/box_pointcloud/all_boxes_corners_20260724_094802_067812.json`
- PLY: `~/box_pointcloud/all_boxes_completed_20260724_094802_067812.ply`
- 시행착오 원문: `perception/SAME_SIZE_STACK_DETECTION_LOG.md`

---

## Part C. 카트 홀로노믹 베이스 다중 시점 스캔 (88.cart_scan_holonomic.py)

원래 88.py는 고정 시점 1곳만 보는 스크립트였다. 여기에 Part A 방식(다중 시점 누적)을
이식하는 과정에서 실제 버그 5개 + 설계 재검토 1건을 거쳤다.

### C-1. 카메라 모델 교체 후 에셋 참조 경로 깨짐
- **증상**: 사용자가 흡착 그리퍼(`vgp20_suction_plate`)로 카메라 마운트를 바꾼 뒤 실행하니
  `RuntimeError: 카메라 프림을 못 찾음 - 발견된 카메라 후보: []`
- **원인**: `m0609_vgp20_camera.usd`가 참조하는 `rsd455.usd`가 실제로는
  `Collected_m0609_vgp20_camera/Collected_m0609_camera/SubUSDs/rsd455.usd`에 있는데,
  참조 경로는 한 단계 바깥(`M0609/Collected_m0609_camera/...`)을 찾고 있었음 — Isaac Sim
  Collect 과정에서 생긴 상대 경로 불일치로 추정.
- **조치**: 심볼릭 링크 생성(`M0609/Collected_m0609_camera` → 실제 위치). 원본 에셋
  파일은 건드리지 않음.
- **관련 파일 변경 없음** (파일시스템 심볼릭 링크만 추가)

### C-2. depth annotator 누락으로 point cloud가 항상 빈 배열
- **증상**: 첫 다중 시점 스캔 시도에서 `IndexError: too many indices for array: array is
  1-dimensional` — 모든 시점에서 크래시.
- **원인**: `camera.initialize()`만 호출하고 `camera.add_distance_to_image_plane_to_frame()`을
  안 불러서 depth 프레임 자체가 안 붙음(35.py는 이 호출이 있었음).
- **조치**: 초기화 시 `add_distance_to_image_plane_to_frame()` / `add_rgb_to_frame()` 추가.
  + 방어적으로 get_pointcloud() 결과 shape 체크, 실패 시 최대 3회 재시도, 그래도 실패하면
  해당 시점만 건너뛰고 전체 스캔은 안 죽게 처리.
- **관련 파일**: `88.cart_scan_holonomic.py`

### C-3. 팔의 물리적 도달거리 초과 (가장 근본적인 원인)
- **증상**: IK 수렴 오차가 350스텝을 다 써도 8~12cm 수준에서 안 줄어듦. 시점을 거칠수록
  9.5cm → 11.6cm → 9.4cm → 18.2cm → 26.1cm로 계속 나빠지는 현상도 동반.
- **원인 분석**: 목표까지 필요한 3D 거리를 계산해보니 약 0.91m — M0609(이름 자체가
  0.9m reach/6kg payload를 의미) 최대 도달거리와 거의 같거나 넘음. 물리적으로 못 닿는
  거리라 스텝을 늘려도 소용없었던 것.
- **조치**: 리프트 최대 높이 0.45m→0.55m로 상향, `EYE_HEIGHT_ABOVE_CART` 0.75m→0.55m로
  하향 — 팔 base와 목표 사이 필요 도달거리 자체를 줄임. 결과: 오차 8~12cm → **3mm**.
- **부가 조치(임시, 이후 C-7에서 재검토)**: 이 문제를 모르던 중간 단계에서, 오차가
  시점마다 누적되는 걸 막으려고 매 시점 관절을 초기 자세로 리셋하는 로직을 넣었었음.
- **관련 파일**: `88.cart_scan_holonomic.py`

### C-4. 대기 루프 버그로 인한 "중력 낙하" — depth가 엉뚱한 높이에서 캡처됨
- **증상**: IK는 3mm로 잘 수렴했는데 캡처된 raw point cloud의 최대 높이가 z=0.58 정도로,
  카트 바스켓 바닥(z=0.68 가정)보다도 낮게 나옴 → 박스 검출 0개.
- **원인**: 자세 수렴 후 렌더 파이프라인이 따라잡을 시간을 주는 대기 루프가
  `step_hold()`(매 스텝 `set_lift_height()` 호출) 대신 순수 `world.step()`만 20번 돌리고
  있었음. M0609는 독립 articulation이라 매 프레임 텔레포트로 "붙잡아" 둬야 하는데, 이
  20스텝 동안 그게 빠지면서 실제로 중력에 낙하함 — 그 낙하한 상태에서 depth를 캡처한 것.
- **조치**: `step_hold(20)`으로 교체. 재시도 루프에도 동일하게 적용.
- **관련 파일**: `88.cart_scan_holonomic.py`

### C-5. `CART_BASKET_FLOOR_Z` 하드코딩 값이 실제보다 높게 잡혀 있었음
- **증상**: C-4를 고친 뒤에도 여전히 박스 검출 0개. 전처리된 point cloud를 시각화해보니
  카트 철망 테두리(림)만 사각형 고리 모양으로 보이고, 바닥과 박스는 아예 안 보임.
- **원인**: `CART_BASKET_FLOOR_Z=0.68`이 실측이 아니라 추정값이었는데, 실제 바닥이 이보다
  낮아서 ROI 하한(`floor - 0.05`)이 진짜 바닥과 박스를 통째로 잘라내고 있었음.
- **조치**: ROI 하한을 `floor - 0.30`으로 넉넉히 낮춤(정확한 실측치로 상수 자체를
  바로잡는 건 나중 과제로 남김). 이후 시각화로 바닥 그리드와 박스 2개(정사각형 블롭)가
  명확히 보이는 것 확인.
- **관련 파일**: `88.cart_scan_holonomic.py`

### C-6. 카트 철망 림이 유령 박스로 잡힘
- **증상**: C-5 수정 후 박스 3개 검출 — 그런데 실제 박스는 2개뿐. 나머지 하나는
  footprint 0.565×0.770m, height 0.39m로 명백히 이상함.
- **원인**: 카트 자신의 철망 테두리(림)가 fill_ratio 0.96~0.98의 "그럴듯한" 평면
  후보로 잡혀서 통과됨.
- **조치**: `MAX_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M`(0.35m) 필터 추가(Part B-3에 통합 서술).
  이후 3회 연속 정확히 2개(Box_A 0.159×0.122, Box_B 0.131×0.103 — 실제 0.16×0.12,
  0.13×0.10과 거의 일치)만 검출됨을 확인.
- **관련 파일**: `perception/multiview_scan.py`

### C-7. 동작이 부자연스러움 — 리셋 후 재수렴하는 왕복 동작
- **증상(사용자 지적)**: Isaac Sim 화면에서 시점을 옮길 때마다 팔이 "원상태로 돌아갔다가
  다시 스캔 자세로 이동"하는 게 번거롭고 부자연스러워 보임.
- **1차 진단**: C-3에서 넣어뒀던 "매 시점 초기 관절각으로 순간이동(reset)" 로직이
  원인 — `set_joint_positions()`로 보간 없이 순간이동시키니 화면에서 "뚝" 끊겨 보임.
- **1차 조치(임시)**: 60스텝에 걸쳐 선형 보간하도록 변경 — 순간이동은 없어졌지만
  "리셋 후 재수렴" 자체의 왕복 구조는 남아있었음.
- **근본 재검토(사용자 아이디어)**: "카트 본체만 움직이고 팔은 필요하면 각도만 조금
  조정하면 되지 않냐"는 지적을 받고 재검토 — C-3에서 팔의 물리적 도달거리 문제를 이미
  해결했으므로, 각 시점의 목표는 베이스 기준 상대 자세로는 사실상 동일(순수
  평행이동 관계)해서 애초에 리셋이 불필요했다는 결론.
- **최종 조치**: 리셋 로직을 완전히 제거. 첫 시점만 350스텝 전체 수렴, 이후 시점은
  이전 자세를 그대로 이어받아 90스텝만 미세조정. 헤드리스 검증 결과 5곳 모두 여전히
  2.8~3.6mm로 수렴(드리프트 누적 없음), 검출 정확도도 그대로 유지됨.
- **관련 파일**: `88.cart_scan_holonomic.py`

### C 최종 산출물
- JSON: `~/box_pointcloud/all_boxes_corners_20260724_112805_581226.json`
- PLY: `~/box_pointcloud/all_boxes_completed_20260724_112805_581226.ply`
- 스크린샷: `results/holonomic_base/_cartscan_*.png`
- Isaac Sim 씬: `results/holonomic_base/cart_scan_holonomic_scene.usd`
- 원본 누적 스캔: `perception/scan_cache/merged_cart_scan.npy`

---

## Part D. 카트 안 적층 박스 구조(Large 바닥 + Medium/Small 나란히 위) 인식 (88.cart_scan_holonomic.py, perception/)

Part C까지는 카트 안에 떨어져 있는 박스 2개(적층 아님)만 다뤘다. 이번엔 "큰 박스가
바닥에 깔리고, 그 위에 중간+작은 박스가 나란히(서로 위아래로는 안 쌓이고) 올라간" 더
현실적인 구조를 새로 시나리오화하고 인식했다 — 박스 배치 설계 문제 4건과, 지지면(바닥)
오검출 문제 1건(진짜 근본 원인)을 순서대로 진단·수정했다.

### D-1. Large가 통째로 검출 실패 (0개)
- **증상**: Large/Medium/Small을 물리 낙하로 스택시킨 뒤(직접 자세 측정으로 스택 자체는
  0.1mm 오차로 정확함을 먼저 확인) 스캔 → 검출 0개.
- **원인**: Medium/Small을 Large 중심에 두면 Large 윗면 전체를 거의 다 덮어버려서, 남는
  노출부가 가장자리의 가는 조각(폭 2cm 미만)뿐 — `MIN_BOX_SIDE_M`(0.04m) 필터에 전부
  걸려 Large 자체가 하나의 사각형 후보로 못 잡힘.
- **조치**: Medium/Small을 Large 뒤쪽(+Y) 절반에만 나란히 배치해서, 앞쪽 절반을 아무것도
  안 덮인 깨끗한 직사각형으로 남김.
- **관련 파일**: `88.cart_scan_holonomic.py` (`CART_BOX_SPECS`)

### D-2. Medium/Small이 하나로 뭉개짐
- **증상**: D-1 조치 후 Large는 검출되지만, Medium/Small 자리에서 fill_ratio가
  0.55~0.98로 요동치는 불안정한 후보 1개만 나옴(둘 다 안 잡힘).
- **원인**: 둘 사이 간격이 2cm(`DBSCAN_EPS_M` 2.5cm보다 작음)였고 높이차도 2cm(0.09 vs
  0.07m, `PLANE_DISTANCE_THRESHOLD_M` 1cm와 비슷한 수준)라 RANSAC이 하나의 "타협 평면"
  으로 묶어버림.
- **조치**: 간격을 4cm(DBSCAN eps보다 확실히 넓게), 높이차도 4cm(Small 0.07→0.05m)로
  벌림.
- **관련 파일**: `88.cart_scan_holonomic.py`

### D-3. 카트 벽/테두리가 RANSAC 예산을 먼저 차지
- **증상**: D-2까지 고쳐도 Medium/Small이 12회 시도 중 1~2회만 관측(노이즈로 판정돼
  제외).
- **원인**: 기존 XY 크롭이 카트 바깥쪽 bbox(벽/철망 테두리까지 포함)에 마진을 "바깥쪽
  으로" 더한 범위였음 — 벽 평면(포인트 수천~9천 개)이 RANSAC의 앞쪽 반복(인라이어 많은
  평면부터 순서대로 제거)을 먼저 차지해버려서, 노출 면적이 작은 Medium/Small이 시도마다
  다르게(불안정하게) 걸림.
- **조치**: XY 크롭을 카트 중심 기준 실제 박스 적재 영역(±0.22m)으로 좁혀서 벽 자체가
  아예 안 들어오게 함.
- **관련 파일**: `88.cart_scan_holonomic.py` (`CART_SCAN_ROI_HALF_X/Y_M`)

### D-4. Small이 가려짐 때문에 짧은 변 필터에 걸림
- **증상**: Large/Medium은 안정적으로 잡히는데, Small만 짧은 변이 0.05~0.074m로
  `MIN_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M`(0.08m)에 못 미쳐 계속 제외됨.
- **원인**: 가장 작고 가장 높이 있는 Small은 옆에서 보는 strafe 스캔 특성상 윗면 관측
  면적 자체가 작아서, 매번 조금씩 다르게(항상 실제보다 작게) 잡힘 — 유령이 아니라 진짜
  박스가 가려짐으로 작게 잡힌 경우.
- **조치**: 이 시나리오에 한해 임계값을 0.065m로 완화(`CART2TRUNK_MIN_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M` 환경변수, 코드 기본값은 유지).
- **관련 파일**: 없음(실행 시 환경변수만 사용)

### D-5. 지지면(바닥) 탐색이 가려진 영역에서 실패 → 카트 바닥까지 뚫고 내려감
가장 오래 걸린 문제. Medium/Small의 윗면 위치 자체는 정확한데, 완성된 박스의 **높이**가
실제(0.09~0.11m)보다 두 배 가까이(0.17~0.21m) 부풀려져 나왔다 — 즉 바닥이 Large 윗면이
아니라 카트 진짜 바닥으로 잘못 잡힌 것.

- **1차 원인**: Medium/Small이 Large의 뒤쪽 절반만 덮고 있어서(D-1 조치), 그 바로 밑
  Large 표면은 카메라가 원천적으로 못 본 영역 — `select_support_candidate()`의 "그
  지점에 실제 관측 포인트가 있어야 함" 검사가 데이터 부재로 실패하고 카트 진짜 바닥으로
  대체됨.
  **조치**: `select_support_candidate()`에 `allow_plane_only_fallback` 옵션 추가 — 엄격한
  포인트 근접 검사가 실패하면, 포인트 근접 조건만 뺀 나머지(법선 정렬/거리 범위/편차/
  면적비) 기하 조건은 만족하는 후보로 대체 채택. 라이브 단일 시점 경로
  (`box_top_extractor.py`)는 이 옵션을 안 쓰므로 기존 동작 그대로 유지됨(기본값 False).
- **2차 원인**: `MAX_SUPPORT_AREA_RATIO`(6.0, top 자기 면적의 6배 넘는 후보는 지지면에서
  제외하는 필터)가 Large처럼 원래 큰 박스에는 너무 엄격해서(Large/Small 면적비 ~11.6배)
  Large가 지지면 후보에서 아예 배제됨.
  **조치**: 실행 시 `CART2TRUNK_MAX_SUPPORT_AREA_RATIO=13`으로 완화.
- **3차 원인(재현 확인용 삽질)**: Open3D `segment_plane()`의 RANSAC이 "시드 고정 없음"
  이라 매 trial마다 명시적으로 재시드하는 코드를 추가했는데, `o3d.utility.random.seed()`
  가 부호 있는 32비트 int만 받는다는 걸 몰라서 `os.urandom(4)`를 그대로 넘겼다가 절반
  확률로 `TypeError`로 크래시 — 이전까지 "고쳐진 것처럼 보였던" 재현 여러 건이 사실은
  매번 크래시해서 새로 저장을 못 하고 이전 결과 파일을 계속 읽고 있었을 뿐이었다(재현 시
  반드시 stdout에서 `Traceback` 유무를 직접 확인하지 않으면 놓치는 함정).
  **조치**: `int.from_bytes(os.urandom(4), "little") % (2**31 - 1)`로 범위를 맞춤.
- **4차 원인(진짜 근본 원인)**: 위 3가지를 다 고쳐도 여전히 대부분의 실행에서 Medium
  높이가 부풀려짐. 매 trial의 디버그 로그를 직접 추적한 결과, **개별 trial에서는**
  `select_support_candidate`가 Large를 정확히 찾고 있었다(`median_distance=0.069~0.105m`,
  올바른 높이) — 그런데 최종 결과에는 반영 안 됨. 원인 추적 결과
  `_split_hidden_same_size_stacks()`(Part B에서 "완전히 같은 크기의 적층" 대응용으로 만든
  함수)가 `support_type`을 전혀 확인하지 않고 fill_ratio가 높은 **모든** 선택된 박스에
  대해 무조건 `find_stacked_layers()`로 **독자적으로 다시** 지지면을 재탐색하고 있었다.
  이 재탐색은 `select_support_candidate`와 별개의(더 약한) 알고리즘이라 같은 가려짐
  문제에 걸려 카트 바닥까지 내려갔고, 그 결과로 이미 올바르게 계산돼 있던 `corners`를
  조용히 덮어썼다(로그에는 안 남아서 원인 추적이 특히 오래 걸림).
  **조치**: `support_type == "box_top"`(이미 다른 박스를 지지면으로 정상적으로 찾음)인
  박스는 이 재탐색에서 제외 — `support_type == "floor"`로 떨어진(진짜 지지 박스를 못
  찾은) 경우에만 적용하도록 조건 추가. Part B의 원래 용도(완전히 가려진 동일 크기 적층)
  는 그대로 보존됨(원래도 항상 "floor"로 떨어지는 케이스였음).
- **검증**: 10회 반복 실행 중 7회가 Large/Medium/Small 모두 정확한 높이(0.107/0.094/0.060m,
  실제 0.12/0.11/0.07m와 근접)로 검출됨. 나머지 3회도 검출된 개수만 적었을 뿐(RANSAC
  stochastic 특성상 일부 시도에서 Medium/Small 자체가 안 걸림), 검출된 박스의 높이는
  전부 정상 범위 — 더 이상 바닥 관통 없음.
- **관련 파일**: `perception/box_geometry.py`(`select_support_candidate`),
  `perception/multiview_scan.py`(`_detect_boxes_once`, `detect_boxes_in_base_frame`,
  `_split_hidden_same_size_stacks`)

### D 최종 산출물
- 원본 누적 스캔(3단 구조 전부 포함, 검증됨): `perception/scan_cache/merged_cart_scan.npy`
- 원본 스캔 시각화 PLY: `results/holonomic_base/cart_stacked_scan_raw.ply`
- 최종 검출 결과: `results/holonomic_base/cart_stacked_boxes_detected.ply`, `.json`
  (Large 0.248×0.319×0.107m, Medium 0.102×0.139×0.094m, Small 0.086×0.103×0.060m)
- 권장 실행 설정: `CART2TRUNK_MAX_SUPPORT_AREA_RATIO=13 CART2TRUNK_MIN_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M=0.065 CART2TRUNK_DETECTION_TRIALS=24`
- 스크린샷: `results/holonomic_base/_cartscan_view_*.png` (Large 위 Medium+Small 나란히
  적층된 모습)

---

## 가이드라인 대조 결과 (참고)

`Cart2Trunk 담당자별 최종 실행 가이드라인.pdf` 2번 "카트 내부 박스 인식" 필수 처리
11단계와 대조한 결과(2026-07-24 검증):

- 1~9단계(Depth→PointCloud 변환, ROI, 다운샘플링/Outlier 제거, 반복 RANSAC, 윗면 후보
  분리, 개별 박스 분할, Boundary 생성, 법선 확장, 8꼭짓점 생성): **모두 구현 완료**
- 10단계(Box ID와 초기 Yaw 부여): **부분적** — Box ID는 있으나 Yaw가 명시적 필드로
  출력되지 않음(코너 좌표에 암묵적으로만 포함)
- 11단계(검출 품질 및 실패 사유 출력): **미구현** — `success`/`quality_score`/`error_code`
  등 구조화된 필드, 표준 실패 코드 없음
- 산출물 파일명이 가이드라인의 `box_scan.json`과 다름(현재 `all_boxes_corners_*.json`)
- 입력 방식이 가이드라인의 ROS2 토픽/Scan Action 구조가 아니라 파일 기반(.npy) 핸드오프

## 재현 완료 자료 (스크린샷/PLY)

2026-07-24, `git commit a0b631c` 시점을 안전한 복원 지점으로 남겨두고, 각 문제를 코드에
임시로 되돌려서(끝나면 항상 백업본과 `diff` 없음을 확인 후 복구) 스크린샷/PLY로 재현했다.
전부 `results/worklog_screenshots/{partA,partB,partC}/`에 있다.

### Part A
| 항목 | 파일 | 비고 |
|---|---|---|
| 로봇 팔 스캔 궤적(±50도/tilt 8,28도, 9개 시점) | `partA/trajectory__verify_crate_scan_table_view_{0-8}.png` | 현재 확정 설정 그대로, 코드 변경 없음 |
| 스캔 속도 - 빠른 버전(조기 종료) | `partA/trajectory__verify_crate_scan_table_view_*.png` (위와 동일 실행) | 9개 시점 소요 20.1초 |
| 스캔 속도 - 느린 버전(조기 종료 비활성화) | `partA/slow__verify_crate_scan_table_view_{0-8}.png` | 9개 시점 소요 41.3초(약 2배) - `CONVERGENCE_MIN_STEPS`를 999999로 임시 변경 후 복구 |
| Small 윗면 비스듬 - 버그 버전 | `partA/partA_skew_buggy.png`, `partA/partA_skew_buggy_box.ply` | z 편차 2.63~4.85mm(시도마다 다름) - `_cluster_plane_into_candidates()` 호출에 `up_vector` 대신 `normal` 전달 후 복구 |
| Small 윗면 비스듬 - 수정 버전 | `partA/partA_skew_fixed.png`, `partA/partA_skew_fixed_box.ply` | z 편차 0.0000mm(완전 평평) |

### Part B
| 항목 | 파일 | 비고 |
|---|---|---|
| 적층 박스 구조(Isaac Sim 배치 장면) | `partB/stacked_structure_layout.png` | Small(주황)이 Large(파랑) 위에 적층, Medium(초록)은 별도 |
| XY-only 그룹핑 - 버그 버전 | `partB/partB_grouping_buggy.ply` | 박스 2개만 검출(Small 누락) - `_group_by_location()`의 Z-tolerance 조건 임시 제거 후 복구, 원래 크기 다른 적층 시나리오 캐시(`scan_cache/merged_table_scan_stacked_az50.ply`) 사용 |
| XY-only 그룹핑 - 수정 버전 | `partB/partB_grouping_fixed.ply` | 박스 3개 정확히 검출 |
| 유령 박스 - 버그 버전 | `partB/partB_ghost_buggy.ply` | 박스 5개(진짜 3 + 유령 2) - `CART2TRUNK_MIN_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M=0.001` 환경변수만 사용(코드 변경 없음) |
| 유령 박스 - 수정 버전 | `partB/partB_ghost_fixed.ply` | 박스 3개 정확히 검출 |

### Part C
| 항목 | 파일 | 비고 |
|---|---|---|
| 뚝뚝 끊기는 모습(순간이동 리셋) | `partC/choppy_reset__cartscan_view_{0-4}.png` | `set_joint_positions(_init_joints)` 순간이동 블록 임시 재삽입 후 복구 |
| 원상태 복귀 후 스캔(부드럽지만 왕복) | `partC/roundtrip__cartscan_view_{0-4}.png` | 60스텝 보간 리셋 블록 임시 재삽입 후 복구(순간이동은 아니지만 여전히 왕복) |
| 최종 성공 - 카트 안 박스 PLY | `partC/final_cart_boxes_success.ply`, `partC/final_cart_boxes_success.json` | Box_A 0.121×0.159(실제 0.16×0.12), Box_B 0.129×0.107(실제 0.13×0.10) |

### Part D
| 항목 | 파일 | 비고 |
|---|---|---|
| D-1: Large 검출 실패 - 박스 배치(카트 안, Medium+Small이 Large 중앙을 덮음) | `partD/D1_layout.png` | Isaac Sim 스크린샷 - 이 각도에서 Large 가장자리가 전혀 안 보임 |
| D-1: Large 검출 실패 - 원본 스캔 PLY | `partD/D1_large_undetected_raw.ply` | `CART_BOX_SPECS`를 dy=0(중앙 배치)로 임시 변경 후 재현 - 검출 0개(로그: `시도별 박스 개수=[0,0,0,1,0,...]`) |
| D-2: Medium/Small 뭉개짐 - 박스 배치(간격 2cm) | `partD/D2_layout.png` | Isaac Sim 스크린샷 |
| D-2: Medium/Small 뭉개짐 - 검출 결과 PLY/JSON | `partD/D2_merged_medium_small.ply`, `.json` | 간격/높이차를 2cm로 임시 축소 후 재현 - 병합된 사각형 하나(0.123×0.249m, 어느 쪽 실제 크기와도 안 맞음)로 잘못 검출 |
| D-3: 카트 벽 노이즈 - 원본 스캔 PLY(벽 포함) | `partD/D3_wall_noise_raw.ply` | XY ROI를 카트 바깥쪽 bbox+마진(옛 버전)으로 임시 변경 후 재현 - 495,624점(타이트 크롭 버전의 약 5배) |
| D-3: 카트 벽 노이즈 - 불안정한 검출 결과 | `partD/D3_unstable_detection.ply`, `.json` | 거대한 카트 구조물 후보 2개(0.77~0.87m) 발생, Large 자체 fill_ratio도 0.732~0.998로 요동(타이트 크롭 시 항상 0.996+였던 것과 대비), Medium/Small은 아예 미검출 |
| D-4: Small 가려짐 - 기본 임계값(0.08m)으로 제외됨 | `partD/D4_small_excluded_strict.ply`, `.json` | 코드 변경 없음(기본값 그대로) - Small 짧은 변 0.078m로 임계값에 살짝 못 미쳐 제외, 박스 2개(Large+Medium)만 |
| D-4: Small 가려짐 - 완화된 임계값(0.065m)으로 포함됨 | `partD/D4_small_included_relaxed.ply`, `.json` | `CART2TRUNK_MIN_PLAUSIBLE_BOX_FOOTPRINT_SIDE_M=0.065` 환경변수만 사용(코드 변경 없음) - Small 짧은 변 0.077m로 통과, 박스 3개 모두 검출 |

D-1/D-2/D-3는 각각 다른 물리적 배치/스캔 설정이 필요해서 그때마다 Isaac Sim을 헤드리스로
재실행했고(`88.cart_scan_holonomic.py`의 `CART_BOX_SPECS`/XY ROI 크롭을 임시로 되돌림),
D-4는 이미 있는(D-5 최종 확정) 스캔 데이터에 검출 임계값만 환경변수로 바꿔 오프라인
재현했다(코드 변경 없음).

### 아직 재현 안 한 것 (요청 시 진행)
| 상황 | 재현 방법 |
|---|---|
| A-3: azimuth ±60도로 IK 오차 0.17m 나던 모습 | `35.crate_scan_setup.py`의 `SCAN_AZIMUTH_DEG`를 ±60도로 임시 변경 후 실행(현재 ±50도) |
| C-3 이전: IK 오차 8~12cm로 팔이 목표에 못 닿는 모습 | `EYE_HEIGHT_ABOVE_CART=0.75`, `LIFT_TRAVEL_M=0.45`로 되돌리고 실행(현재 0.55/0.55) |

모든 임시 수정은 각각 적용 직후 재현·촬영하고 바로
`diff backups/<파일> <실제파일>`로 원상복구를 확인한 뒤 다음 항목으로 넘어갔다 - 지금
작업 디렉토리는 커밋 시점(`a0b631c`) + Part D 재현/수정을 반영한 최신 상태다(88.py,
multiview_scan.py, box_geometry.py, scan_cache/merged_cart_scan.npy 모두 Part D 재현
전후로 diff 없음을 확인함).
