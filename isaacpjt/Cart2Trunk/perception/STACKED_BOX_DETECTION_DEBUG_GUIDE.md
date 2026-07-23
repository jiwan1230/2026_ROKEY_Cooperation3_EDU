# 적층/조밀 배치 박스 검출 디버깅 가이드 (40번 스크립트 기준)

이 문서는 `40.vision_roi_rectfill_upgrade.py`(35번 기반 + ROI/사각형 필터 강화 버전)에서
"박스가 쌓여있는데 하나로만 검출된다"는 문제를 실제로 진단하고 고친 전체 과정을 정리한
것이다. Vision(`perception/box_top_extractor.py`) 쪽 작업을 이어받을 사람이 같은 함정에
다시 빠지지 않도록, **증상 → 원인 → 진단 방법 → 수정**을 버그별로 정리했다.

## 0. 요약 표

| # | 증상 | 진짜 원인 | 파일:라인 | 수정 |
|---|---|---|---|---|
| 1 | 적층된 두 박스(Small on Large)가 항상 1개로만 출력 | 테이블 박스 dedup이 XY 중심 거리만 보고 Z(높이)를 안 봄 | `40.py` `table_real_boxes` dedup 루프 | Z도 같이 비교(`TABLE_BOX_DEDUP_Z_TOLERANCE_M=0.05`) |
| 2 | Large의 노출된 윗면(테두리)이 매번 "no lower boundary or floor"로 탈락 | `FLOOR_BOUNDARY_HIT_DISTANCE_M=0.12`가 적층/조밀 배치로 가려진 만큼의 관측 여백을 못 버팀 | `box_top_extractor.py:220` | `0.12 → 0.30` |
| 3 | Medium(+가끔 Large)이 fill_ratio 미달로 간헐적 탈락 | RGB로 가림이 아님을 확인 → `PLANE_DISTANCE_THRESHOLD_M=0.006`(6mm)이 너무 타이트해서 depth 노이즈로 같은 물리 평면이 RANSAC 반복(iteration)에 걸쳐 쪼개짐 | `box_top_extractor.py:126`, `:167` | `PLANE_DISTANCE_THRESHOLD_M 0.006→0.008`, `MIN_RECTANGULAR_FILL_RATIO 0.60→0.50` |
| 4 (부작용) | 완화하자마자 테이블에 없던 "박스"가 하나 더 나타남(크레이트 조각) | 크기만 보는 필터라 "크기는 맞는데 위치가 테이블 밖(크레이트 쪽)"인 오탐을 못 거름 | `40.py` `_matches_known_table_box` | 위치 필터 `_is_on_table_world` 추가 |
| 5 (미해결) | Small이 몇몇 스캔 프레임에서 아예 후보로도 안 나타남 | 프레임마다 RANSAC 시드가 없어 후보 집합이 바뀌는 기존 노이즈([[cart2trunk_box_table_scan]] Round 4와 동일 계열) | - | 미해결. 재현 빈도만 확인함 |

---

## 1. 배경

`40.py`는 테이블 위 3개 박스(Medium/Large/Small) 배치를 바꿔서 **Small을 Large 위에
스택**시키고, ROI(0.15~0.85)와 `MIN_RECTANGULAR_FILL_RATIO`(0.60)를 강하게 걸어 오탐을
줄이는 실험이었다. 실행 결과 "적층된 두 박스가 1개로만 잡힌다"는 증상이 보고됐다.

## 2. 진단 도구 (재사용 가능)

이번에 두 가지 진단 도구를 `box_top_extractor.py`/`40.py`에 추가했다. 앞으로 비슷한
"왜 이 박스가 안 잡히지" 류 문제에 그대로 재사용할 수 있다.

### 2.1 `CART2TRUNK_DEBUG_SUPPORT=1` — 후보 기각 사유 전수 로깅

`run_scan_once.py` 실행 시 이 환경변수를 켜면, RANSAC 평면 검출 → DBSCAN 클러스터링 →
사각형 검사 → 지지면(바닥/box_top) 매칭까지 **모든 단계에서 기각된 후보의 구체적 수치**
(footprint, fill_ratio, ray_distance, nearest_distance, hit_count 등)를 로그로 남긴다.
기존에는 최종적으로 `"Candidate N: no lower boundary or floor."` 한 줄만 찍혀서 정확히
어느 단계 어느 수치 때문에 떨어졌는지 알 수 없었다.

```bash
export CART2TRUNK_DEBUG_SUPPORT=1
DISPLAY=:1 python3 run_scan_once.py --marker <marker경로>
```

기각 사유별 로그 형식 (모두 `box_top_extractor.py` 안에 `if DEBUG_SUPPORT:` 로 게이트됨,
평소엔 완전히 비활성 — 성능 영향 없음):

- `[DEBUG plane] ... normal_consistency=... -> reject whole plane` — 평면 전체가 대표
  법선과 안 맞아서 통째로 제외 (보통 박스 옆면).
- `[DEBUG cluster] ... points=N < MIN_CLUSTER_POINTS -> drop` — DBSCAN 클러스터가 너무
  작아서 버림.
- `[DEBUG make_candidate] ... width/height/aspect_ratio/fill_ratio ... -> reject` —
  사각형 검사 단계 기각. **`center_base`(base_link 좌표)까지 같이 찍히므로, 이 값을
  `TABLE_BOXES`의 알려진 위치와 대조하면 "이 조각이 실제로 어느 물리 박스였는지" 바로
  확인 가능.**
- `[DEBUG floor] ... ray_distances / nearest_distances / hit_count -> reject` — 지지면
  (바닥) 매칭 단계에서 정확히 어떤 조건(높이 범위, spread, 근접 포인트 거리)에 걸렸는지.
- `[DEBUG box_top support] ...` — "다른 박스 위/아래" 매칭 단계 성공/실패.

### 2.2 카메라 자체 RGB + ROI 오버레이 스크린샷

`40.py`의 테이블 스캔 수렴 직후, `box_top_extractor.py`가 실제로 보는 것과 **동일한
카메라 prim**에서 RGB 프레임을 뽑아 ROI 경계선(빨간 사각형)을 그려서 저장한다
(`_debug_table_scan_rgb_roi.png`, 640x480). "가려서 안 보이는 건지 ROI에 잘린 건지"를
가정으로 때려맞추지 말고 **한 장 찍어서 눈으로 확인**할 것 — 이번 케이스에서 실제로
이 스크린샷 한 장이 "팔/그리퍼에 가려졌다"는 원래 가설을 틀렸다고 확정지었다(박스가
ROI 안에 완전히, 여유 있게 노출되어 있었음).

```python
_rgba = camera.get_rgba()  # isaacsim.sensors.camera.Camera, 동일 camera_prim_path
# ... IMAGE_ROI 픽셀 좌표 계산 후 PIL로 저장 (40.py 참고)
```

**교훈: depth 파이프라인의 이상 증상을 "가림/시야각" 가설로 설명하고 싶어질 때,
말로 추론하기 전에 같은 카메라의 RGB 프레임 한 장을 실제로 찍어서 확인할 것.** 이번에
그 한 장이 없었으면 "가림" 가설을 계속 파다가 진짜 원인(RANSAC 평면 분할)을 못 찾았을
것이다.

---

## 3. 버그 1 — dedup이 Z를 안 봐서 스택된 박스가 병합됨

**증상**: `TABLE_BOXES`에서 Small을 Large와 완전히 같은 (dx, dy)로 스택시켰더니, 최종
`table_boxes_filtered.json`에 둘 중 하나만 남았다.

**원인**: `40.py`의 `table_real_boxes` dedup 로직(원래는 크레이트 더미 박스가 RANSAC
후보 2개로 겹쳐 잡히는 걸 막으려고 만든 것)이 **XY 중심 거리만** 보고 0.10m 이내면
"같은 물체의 중복 검출"로 간주했다. 크레이트 더미의 원래 상황은 "같은 높이의 같은
물체가 후보 2개로 겹친 것"이었지만, 스택 시나리오는 "XY는 같고 Z만 다른 진짜 별개의
두 물체"라 이 필터가 정확히 그 경우를 오판했다.

**수정**: XY 거리 조건에 Z 거리 조건(`TABLE_BOX_DEDUP_Z_TOLERANCE_M=0.05`)을 AND로
추가. 크레이트 더미 중복(같은 높이)은 그대로 걸러지고, 스택(다른 높이, 실측 z 중심차
~0.11m)은 살아남는다.

**교훈**: 다른 컨텍스트에서 만든 필터를 재사용할 때 "이 정도면 여전히 맞겠지"라고
넘어가지 말 것 — 입력 조건(이번엔 "박스가 스택될 수 있다")이 바뀌면 원래 필터가
전제했던 가정이 깨질 수 있다.

---

## 4. 버그 2 — Large 테두리가 지지면(바닥) 매칭에서 항상 탈락

**증상**: 버그 1을 고친 뒤에도 Large가 안 나타남. 로그에 `no lower boundary or floor`
반복.

**진단**: `CART2TRUNK_DEBUG_SUPPORT=1`로 재실행해서 확인:
```
[DEBUG floor] top=3 plane_index=0: nearest_distances=[0.163, 0.212, 0.249, 0.249]
              hit_count=0 < MIN_BOUNDARY_RAY_HITS(3) -> reject
```
`plane_index=0`(테이블 표면, 가장 지배적인 평면)은 법선/높이/기울기 검사를 전부
통과했다 — 즉 Large의 진짜 지지면을 **찾긴 찾았다**. 문제는 Large의 네 모서리
근처에서 "실제로 관측된" 테이블 포인트까지의 거리가 16~25cm였는데,
`FLOOR_BOUNDARY_HIT_DISTANCE_M=0.12`(12cm)라 전부 기각된 것. Small이 위에 스택되고
박스들이 0.65배로 더 촘촘히 배치되면서, Large 테두리 주변의 맨 테이블 표면이 그만큼
가려/멀게 보인 것.

**수정**: `FLOOR_BOUNDARY_HIT_DISTANCE_M 0.12 → 0.30`. 이미 법선/높이가 검증된
평면이므로, 그 위의 관측 포인트가 조금 멀리 있어도(가려짐) 받아들이도록 완화 —
평면 자체의 정합성 검사(parallel_score, median_distance, spread)는 그대로 두고
근접성 조건만 완화한 것이라 위험도가 낮다.

---

## 5. 버그 3 — Medium/Large가 fill_ratio 미달로 간헐적 탈락 (진짜 원인은 가림이 아니었음)

**증상**: 버그 2를 고친 뒤에도 Medium이 프레임마다 나왔다 안 나왔다 함.

**처음 세운 가설(틀림)**: Medium이 로봇에 가장 가깝게 배치돼 있어서 ROI 크롭이나 팔/
그리퍼에 부분적으로 가려지는 게 아닐까.

**실제 확인**: `_debug_table_scan_rgb_roi.png`를 찍어보니 Medium(초록 박스)이 ROI
안쪽에 완전히, 여유 있게 노출되어 있었다. 가림이 아니었다.

**진짜 원인 (디버그 로그로 확정)**:
```
[DEBUG make_candidate] center_base=[0.647,-0.068,0.058] footprint=(0.225,0.156)
                        fill_ratio=0.587 < 0.6 -> reject   (Large 크기와 거의 일치)
[DEBUG make_candidate] center_base=[0.624, 0.06, 0.062] footprint=(0.179,0.068)
                        fill_ratio=0.591 < 0.6 -> reject   (Medium 위치와 일치)
```
기각된 fill_ratio 값들이 하나같이 0.52~0.59라는 **좁은 밴드**에 몰려 있었다. 이는
`PLANE_DISTANCE_THRESHOLD_M=0.006`(6mm)이 너무 타이트해서, 실제로는 가려지지 않은
평평한 박스 윗면도 depth 노이즈가 국소적으로 6mm를 넘는 지점에서 RANSAC이 **같은
물리적 평면을 서로 다른 추출 반복(iteration)으로 쪼개버리기** 때문이다. 쪼개진 조각은
서로 다른 `detected_planes` 항목이 되어 DBSCAN도 따로 돌기 때문에, 물리적으로 붙어있는
점들이어도 하나의 완전한 사각형으로 합쳐지지 못하고, 각 조각은 불완전한(직사각형이
아닌) 형태로 남아 fill_ratio가 낮게 나온다.

**수정**: `PLANE_DISTANCE_THRESHOLD_M 0.006 → 0.008`(분할 자체를 줄임) +
`MIN_RECTANGULAR_FILL_RATIO 0.60 → 0.50`(그래도 남는 약간의 분할은 여기서 흡수).
두 값 다 `box_top_extractor.py`에서 `CART2TRUNK_*` 환경변수로 오버라이드 가능하게
만들어 뒀다.

**교훈**: 간헐적 검출 실패를 "가려서 그런가보다"로 설명하고 싶은 유혹이 크지만,
fill_ratio처럼 **수치로 기각되는 필터**가 있다면 그 수치들이 어떤 패턴(좁은 밴드에
몰림, 특정 파라미터 임계값 바로 아래 등)을 보이는지 먼저 확인하는 게 훨씬 빠르고
정확하다. 이번엔 "0.52~0.59가 0.60 바로 아래에 몰려있다"는 패턴 자체가 원인(순수
임계값 문제, 개별 사례의 우연이 아님)을 가리키고 있었다.

### 5.1 이 수정이 낳은 부작용 — 크기만 보는 필터의 구멍

fill_ratio/거리 임계값을 완화하자마자, 테이블에 없던 박스가 하나 더 나타났다:
```
id=5 support=floor footprint=(0.172,0.247) height=0.141 center_base=(0.666,-0.678,-0.340)
```
크기(0.172×0.247×0.141)가 Large(0.16×0.23×0.14)와 거의 똑같아서 "알려진 박스"로
통과됐다. 그런데 `center_base`를 world 좌표로 역산해보면:
```
world = base_pos + R_base @ (cx, cy, 0)  →  (0.678, 0.116)
```
테이블은 `TABLE_SIZE=(0.8,0.6)`, `CART_POS=(0,0)` 기준으로 world x∈[-0.4,0.4]까지만
차지한다. 이 후보의 world x=0.678은 테이블 밖이고, 오히려 **크레이트**(`CRATE_CENTER_
XY=(0.9,0)`, x범위 [0.455,1.345]) 안에 들어간다 — 이 스캔 포즈에서는 원래 안 보여야
할 크레이트 구조물 일부가 시야 가장자리에 살짝 걸려서, 완화된 임계값 덕분에 크기
조건까지 우연히 통과한 것.

**수정**: `_matches_known_table_box`(크기 필터)와 별개로 `_is_on_table_world`(위치
필터)를 추가 — 이미 알고 있는 `TABLE_SIZE`/`CART_POS`로 world 범위를 계산해서, 그
범위 밖의 후보는 크기가 아무리 맞아도 제외한다.

**교훈: 크기 기반 필터는 "크기는 맞는데 위치가 엉뚱한" 오탐을 절대 못 거른다.**
우리가 이미 알고 있는 장면의 기하 정보(테이블 범위, 크레이트 범위처럼 우리가 직접
만든 값)가 있다면, 크기 필터와 별개로 위치 타당성도 항상 같이 확인할 것 — 특히 필터를
완화하는 방향으로 튜닝할 때는 이런 종류의 새 오탐이 들어올 위험이 항상 있다는 걸
전제하고 재검증해야 한다.

---

## 6. 미해결 — Small이 일부 프레임에서 후보로도 안 나타남

버그 1~4를 다 고친 뒤에도, Small(스택 맨 위, 오히려 가장 잘 보여야 할 위치)이 raw
후보 목록에 아예 없는 프레임이 있었다. `CART2TRUNK_DEBUG_SUPPORT` 로그로도 관련
기각 흔적을 못 찾음 — 즉 필터에 걸려서 떨어진 게 아니라 애초에 후보 자체가 생성되지
않은 것으로 보인다. [[cart2trunk_box_table_scan]] Round 4에서 이미 확인된 "Open3D
`segment_plane()` RANSAC은 고정 시드가 없어서 정적인 장면도 프레임마다 후보 집합이
달라진다"는 노이즈와 같은 계열로 추정되지만, 이번 세션에서 근본 원인을 확정하지는
못했다. `run_scan_once.py`의 다중 프레임 샘플링/최빈값 선택이 "박스 개수"의 최빈값만
보장하지 "어떤 박스들이 포함됐는지"는 보장하지 않는다는 것도 이 문제를 더 어렵게
만든다 — 다음에 이어서 팔 사람은 여기서부터 시작하면 된다.

---

## 7. 변경된 파일/파라미터 정리

| 파라미터 | 기본값 (이전 → 이후) | 위치 |
|---|---|---|
| `TABLE_BOX_DEDUP_Z_TOLERANCE_M` (신규) | - → 0.05 | `40.py` |
| `FLOOR_BOUNDARY_HIT_DISTANCE_M` | 0.12 → 0.30 | `box_top_extractor.py:220` (env: `CART2TRUNK_FLOOR_BOUNDARY_HIT_DISTANCE_M`) |
| `MIN_RECTANGULAR_FILL_RATIO` | 0.60 → 0.50 | `box_top_extractor.py:167` (env: `CART2TRUNK_MIN_RECTANGULAR_FILL_RATIO`) |
| `PLANE_DISTANCE_THRESHOLD_M` | 0.006 → 0.008 | `box_top_extractor.py:126` (env: `CART2TRUNK_PLANE_DISTANCE_THRESHOLD_M`) |
| `TABLE_POSITION_MARGIN_M` (신규) | - → 0.05 | `40.py` (`_is_on_table_world`) |
| `CART2TRUNK_DEBUG_SUPPORT` (신규, 진단용) | 기본 꺼짐(`0`) | `box_top_extractor.py` 전역 |

**주의 (별개로 발견한 기존 버그, 이번에 같이 고침)**: `40.py`가 `os.environ.
setdefault()`로 설정하는 `CART2TRUNK_*` 값들은 **자기 자신의 프로세스에만** 적용되고,
`box_top_extractor.py`/`run_scan_once.py`는 완전히 별도의 OS 프로세스(다른 터미널)로
실행되기 때문에 이 값을 물려받지 못한다. 실제 동작은 `box_top_extractor.py` 자체의
기본값이 결정하므로, **파라미터를 바꿀 땐 `box_top_extractor.py`의 기본값을 직접
바꾸거나, `run_scan_once.py`를 실행하는 터미널에서 명시적으로 `export`할 것.** `40.py`
는 이제 `[대기]` 안내 문구에 필요한 `export` 명령을 자동으로 포함해서 출력한다.

---

## 8. 일반 체크리스트 (다음에 비슷한 문제 만나면)

1. **"안 잡힌다"가 필터 기각인지 애초에 후보가 없는 건지부터 구분한다** —
   `CART2TRUNK_DEBUG_SUPPORT=1`로 각 단계 로그를 켜서 확인.
2. **기각 수치가 있다면 그 값들의 분포를 본다** — 임계값 바로 아래에 몰려있으면
   순수 임계값 문제(개별 사례 우연이 아님)일 가능성이 높다.
3. **"가려서 그런가?" 가설이 떠오르면 실제 RGB 프레임을 한 장 찍어서 확인한다** —
   추론만으로 결론 내지 말 것.
4. **base_link 좌표로 나온 후보는 world로 역산해서 알고 있는 장면 배치(테이블/
   크레이트 범위 등)와 대조한다** — 크기가 맞아도 위치가 엉뚱하면 다른 물체다.
5. **필터를 완화하는 방향으로 튜닝했다면 반드시 재검증한다** — 억제하던 오탐이
   다시 들어올 수 있다(이번 크레이트 누출 사례).
6. **다른 컨텍스트에서 만든 필터(dedup 등)를 재사용할 때는 그 필터가 전제했던
   가정이 새 상황에서도 유지되는지 확인한다.**
