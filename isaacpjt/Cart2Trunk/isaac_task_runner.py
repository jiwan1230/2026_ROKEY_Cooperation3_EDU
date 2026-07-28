"""
100.cart_to_trunk_dual_side_holonomic.py

94.cart_to_trunk_stage3_0_holonomic.py를 베이스로 한 "카트 양쪽 접근" 대규모 재설계
(사용자 설계 지시, 2026-07-26). 94번까지의 개발에서 확인된 문제 - 트렁크 왼쪽에
배치해야 하는 박스는 매니퓰레이터를 조인트3/5 반전(_fold_target_mirrored)으로 접어
반대쪽 solution branch를 쓰게 만들어야 link_2/link_5가 트렁크 입구와 부딪히지 않는데,
이 반전된 자세로 "카트의 같은 쪽에서" PICK하면 이번엔 팔이 카트의 손잡이(handle)와
부딪혀 파지가 어긋난다(사용자 실측 확인) - 즉 "트렁크쪽 요구사항(solution space)"과
"카트쪽 요구사항(손잡이 회피)"이 한 곳에서 동시에 만족될 수 없는 경우가 있었다.

해결책(사용자 설계) - 카트를 90도 회전시켜 손잡이(A단)가 트렁크/차량에서 먼 쪽에,
손잡이 없는 입구(B단)가 차량에 가까운 쪽에 오도록 배치한다("카트 입구가 차량을
바라보도록"). 이러면 카트에 A→B 축(월드 X와 나란함, CAR_POS가 +X쪽에 있으므로)이
생기고, "A에서 B를 바라봤을 때의 왼쪽/오른쪽"(월드 +Y/-Y)이라는 고정된 기준으로 카트
옆 어느 쪽에 서서 PICK할지를 매 박스마다 고를 수 있다. solution space(트렁크 배치
위치가 결정)와 박스의 A/B쪽 치우침을 XOR로 조합해 카트 접근 방향을 정하면, 트렁크
요구사항과 손잡이 회피 요구사항이 항상 동시에 만족된다(아래 "사용자 설계 - 카트
양쪽 접근 XOR 규칙" 섹션 참고). 94번의 "PICK은 항상 카트의 한쪽 고정 standoff에서만"
+ "그 안에서 조인트만 반전"이라는 절충이었던 접근을 대체한다.

94번과 동일하게 유지되는 부분 - 91번(카트 PICK 알고리즘 자체)과 92번(트렁크 PLACE,
STAGE 1~4 전체)을 문자 그대로 재사용한다(box_id별 배치 목표만 compute_place_targets()
로 매 박스 다시 계산) - 92번 STAGE 1~4 코드 자체는 건드리지 않는다. 씬 구성도 91번의
카트(CART_POS)와 92번의 차량(CAR_POS=(5,0,0))을 같은 스테이지에 배치하는 것은 동일 -
차이는 카트에 CART_ROT_Z=90도 회전을 추가로 준다는 것뿐(아래 CART_ROT_Z 정의부 참고 -
이 값의 부호는 "손잡이가 실제로 -X쪽/먼 쪽에 오는지" 스크린샷으로 확인 후 필요하면
뒤집어야 하는, 유일하게 눈으로 검증해야 하는 상수다).

94번 대비 구조적으로 달라진 부분
----
1) 카트가 90도 회전하면서 카트의 "길이축"(A-B, 손잡이-입구)이 월드 Y에서 월드 X로
   바뀐다 - CART_BOX_SPECS의 박스 오프셋도 그에 맞춰 Y축에서 X축으로 옮겼다(Box_A는
   -X쪽/손잡이쪽, Box_B는 +X쪽/입구쪽으로 치우치게 스폰).
2) 카트 standoff가 "카트 옆(+X쪽) 고정 1곳"에서 "카트의 폭(width, 회전 후 월드 Y)
   방향 양쪽 2곳"(CART_BASE_LEFT_XY/CART_BASE_RIGHT_XY)으로 늘었다 - CART_STANDOFF_DIST
   계산도 cart_half_x(옛 폭)에서 cart_half_y(회전 후 폭)로 바뀌었다.
3) box_place_needs_mirrored_pick()(트렁크 배치가 왼쪽인지, solution space를 결정)과
   box_leans_toward_cart_A_end()(신규 - 박스가 카트의 A단/손잡이쪽에 치우쳤는지)를
   XOR으로 조합한 approach_cart_left_side()가 "이 박스는 카트의 어느 쪽에서 집어야
   하는지"를 정한다(사용자가 직접 표로 준 4가지 경우 그대로 구현 - 아래 해당 함수
   정의부의 표 참고).
4) 박스 루프 구조 변경 - 94번은 "다음 박스가 미러링이 필요하면 카트 복귀 주행 도중
   멈춰서 조인트만 반전"하는 방식이었다(카트 접근 위치/각도는 항상 고정). 100번은 반대로
   "이번 박스가 필요로 하는 (카트 접근 위치+각도, solution space fold)를 루프 시작
   시점에 한 번에 계산해서 그리로 주행"하는 방식이다 - 이러면 박스 1도 박스 2 이상과
   완전히 동일하게 다룰 수 있어(루프 시작에 "카트 standoff로 주행"을 매번 넣고, 스폰
   직후 부트스트랩 자세가 이미 표준 접힘+LIFT_MIN이라 박스 1의 이 단계는 자연히
   무동작에 가깝다) 94번에 있던 "박스1은 이미 서 있다/박스2+는 되돌아가야 한다"는
   비대칭, 그리고 "카트 복귀 중간에 멈춰서 조인트 반전" 같은 특수 로직이 전부 사라진다.
   PICK 자체는 이제 항상 (트렁크 배치가 요구하는) 표준 solution space로만 이뤄지므로
   -PICK 도중 조인트를 반전시킨 채 손잡이를 스치는 상황 자체가 구조적으로 없어진다.

94번과 완전히 동일하게 유지되는 부분(참고용 원본 설명, 아래는 94번 설명 그대로) - PICK
자체(hover->하강->흡착), 안전 운송 자세/리프트 하강, STAGE 0.8 홀로노믹 주행(카트
standoff->92번 BASE_START_XY), STAGE 1~4(92번 원본)는 전부 그대로다.

주의(94번에서 이미 확인된 함정, 동일하게 적용) - LIFT_MAX는 92번의 STAGE 3.x 코드
여러 곳(STAGE3.2.0의 STAGE3_2_0_LIFT_TARGET=LIFT_MAX+0.2 등)에서 직접 참조되므로,
반드시 92번과 동일한 값(LIFT_MIN+0.35)을 유지해야 한다 - 91번의 PICK용 높은 리프트
(0.75 travel)는 별도 이름(PICK_LIFT_H)으로 분리돼 있다.
"""

from isaacsim import SimulationApp

import os

HEADLESS = os.environ.get("HEADLESS", "0") == "1"
# 사용자 지시 - 한 번에 다 돌리지 말고 단계별로 나눠서 확인한다(이 파일 전용 STAGE 체계).
# STAGE=0    : 카트에서 박스 PICK(흡착)까지만.
# STAGE=0.5  : 위 + 안전 운송 자세(조인트 접기) 확립 + 리프트 하강(LIFT_MIN).
# STAGE=0.8  : 위 + 홀로노믹 주행+회전으로 92번의 BASE_START_XY(트렁크 standoff)까지 이동.
# STAGE=1~3  : 92.trunk_place_holonomic.py와 완전히 동일한 의미(1=홀딩자세 확립,
#              1.1=입구 턱 클리어, 2=근접 이동, 3=정밀접근 시작/STAGE3.0).
# STAGE=3.1~3.4/4: 92번과 완전히 동일(3.1=천장 아래로 접기, 3.2=팔 펴기+X접근, 3.3=X/Y
#              정렬, 3.4=최종 하강+릴리즈, 4=역순 후퇴).
# 이 모든 단계를 pick_order(카트 안 모든 박스)에 대해 순서대로 반복한다 - 한 박스를 놓고
# 후퇴한 뒤(STAGE>=4), 마지막 박스가 아니면 카트로 되돌아가 다음 박스를 집는다.
STAGE = float(os.environ.get("STAGE", "4"))
_sim_app_config = {"headless": HEADLESS}
if not HEADLESS:
    _sim_app_config.update({"width": 640, "height": 480})
simulation_app = SimulationApp(_sim_app_config)

import json
import sys
from pathlib import Path

import numpy as np
import omni.usd
import omni.kit.viewport.utility as vp_util
from omni.physx import get_physx_scene_query_interface
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, UsdLux, UsdShade, Sdf, Gf, Vt

from isaacsim.core.api import World
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.core.utils.rotations import quat_to_euler_angles, euler_angles_to_quat
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

_THIS_DIR = Path(__file__).resolve().parent
OUT_DIR = _THIS_DIR / "results" / "holonomic_base"
OUT_DIR.mkdir(parents=True, exist_ok=True)

M0609_DIR = _THIS_DIR.parent / "M0609"
RMPFLOW_DIR = str(M0609_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController  # noqa: E402

# ---------------- 89.py와 완전히 동일 - 차량/트렁크 실측 상수 ----------------
CAR_USD = str(_THIS_DIR / "assets/Lexus_IS300_Trunk_Open_No_More_Hell_Room.usdz")
CAR_POS = (5.0, 0.0, 0.0)
CAR_EXTRA_SCALE = 0.53
CAR_ROT_Z = 0.0
# 사용자 결정 - 트렁크 입구~천장 실제 여유가 4.3cm뿐이라 차량 스케일을 0.50->0.53으로 키움
# (89.py 재스캔 -> 90.py 재추출로 나온 trunk_map.json 실측 AABB를 world로 재투영한 값).
# 아래 4개는 지금까지처럼 여전히 "설계용" 근사 상수이고, 실제 판정 기준은 trunk_map.json/
# trunk_pointcloud_meta.json에서 동적으로 계산되는 CEILING_WORLD_Z 등을 우선한다.
TRUNK_X_MIN, TRUNK_X_MAX = 2.945, 3.702
TRUNK_Y_MIN, TRUNK_Y_MAX = -0.663, 0.664
TRUNK_FLOOR_Z = 0.459
TRUNK_WALL_TOP = 1.010
SDF_RESOLUTION = 256
ANCHOR_Y = 0.0

# 사용자 지적 - TRUNK_X_MIN 하나를 "적재 공간 시작점"/"실제 차량 개구부 평면"/"박스 통과
# 판정 평면" 세 용도로 동시에 쓰면 안 된다(차량 형상상 몇 cm씩 차이 날 수 있음). 실측 확인됨:
# 예전 TRUNK_X_MIN(=3.11, 0.50 스케일 기준)은 8.rescale_and_rebuild.py가 인용한 (지금은
# 소실된) rescale_probe.py의 레이캐스트 결과였는데, 같은 프로브가 낸 floor_z=1.03도 이미 다른
# 스크립트(12.trunk_scan_hidden_gripper.py)에서 "사실 입구 쪽 얕은 턱이었다, 진짜 바닥은 물리
# 낙하 테스트로 0.43~0.44에서 찾음"이라고 정정된 전례가 있다 - x=3.11도 레이캐스트/포인트
# 클라우드가 처음 표면을 감지한 지점(안쪽 턱/선반)이었을 뿐, 진짜 개구부 평면이 아니었다.
# STAGE 2 마커 평면(EntrancePlane=초록, SuccessPlane=노랑) 스크린샷으로 대조한 결과 옛
# TRUNK_X_MIN 기준 -0.15m 보정 시 성공 확인됨.
#
# 차량 스케일 변경(0.50->0.53) 이후 - 위 TRUNK_X_MIN 자체는 새로 재측정한 값(2.945)으로
# 갱신됐고, "-0.15"였던 보정폭도 옛 스케일/기하에서 튜닝된 경험값이라 그대로 안 맞을 걸로
# 예상했던 대로, STAGE=2 마커(EntrancePlane/SuccessPlane) 스크린샷 재검증 결과 -0.09m로
# 다시 튜닝해서 성공 확인됨(사용자 실측 확인).
TRUNK_ENTRANCE_X = TRUNK_X_MIN - 0.09

# ---------------- 82~91번과 동일 홀로노믹 베이스 구성 ----------------
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 0.0, 50.0, 20.0
BASE_PATH = "/World/HoloBase"
CHASSIS_PATH = f"{BASE_PATH}/chassis"
BASE_FACE_ROT_Z = 90.0  # 91.cart_pick_holonomic.py와 동일값 그대로 - PICK 기하를 임의로 바꾸지 않는다.

ROLLER_COUNT = 9
ROLLER_MASS = 0.02
HUB_MASS = 1.0
CHASSIS_MASS = 15.0

M0609_USD = str(M0609_DIR / "Collected_m0609_vgp20_camera" / "m0609_vgp20_camera.usd")
M0609_URDF_PATH = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH = str(M0609_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(M0609_DIR / "rmpflow/m0609_rmpflow_common.yaml")
M0609_MOUNT_Z_ABOVE_CHASSIS_TOP = 0.02
LIFT_COLUMN_RADIUS = 0.045
# 91.cart_pick_holonomic.py와 동일값 - 카트 손잡이 높이(~1.03m) 위에서 호버하려면 이만큼
# 필요하다(91번 docstring 참고, 여러 차례 실측 튜닝된 값). PICK 전용(PICK_LIFT_H)으로만
# 쓴다 - LIFT_MAX 자체는 92번과 동일한 0.35 기준을 유지한다(STAGE 3.x 여러 곳이 LIFT_MAX를
# 직접 참조하므로 절대 이 값으로 덮으면 안 된다 - 실측으로 확인된 함정, 아래 LIFT_MAX
# 정의부 주석 참고).
LIFT_TRAVEL_M = 0.75

EE_LINK_NAME = "link_6"
GRIPPER_BODY_NAME = "vgp20_suction_plate"

GRIPPER_RANGE_JSON = M0609_DIR / "Collected_m0609_vgp20_camera" / "_gripper_physical_range.json"
if GRIPPER_RANGE_JSON.exists():
    _range = json.loads(GRIPPER_RANGE_JSON.read_text())
    TIP_LOCAL_OFFSET = tuple(_range["tip_local_offset"])
else:
    TIP_LOCAL_OFFSET = (0.0, 0.0, 0.0188)

STANDOFF_MARGIN = 0.15  # 92번 값 - 트렁크 standoff(STANDOFF_TRUNK) 전용, 아래 CART_STANDOFF_MARGIN과 다른 값이다.
CART_STANDOFF_MARGIN = 0.10  # 91.cart_pick_holonomic.py와 동일값 - 카트 standoff 전용(같은 이름을 재사용하면 91번 튜닝값이 92번 값으로 조용히 바뀌어버리므로 이름을 분리했다).
WAYPOINT_STEPS = 300
DOWN_QUAT = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))
RELEASE_CLEARANCE_ABOVE_FLOOR = 0.02
# PLACE_DESCENT_SUBSTEPS(구 36.py식 천장-근처 호버 후 여러 단계로 나눠 하강하던 상수)는
# 2차 설계에서 폐기 - HOLDING_Z가 이미 release_z 바로 위(0.05m)라 한 번에 내려도 된다.

# 91.cart_pick_holonomic.py와 동일한 원통형 흡착 판정 여유값 (DynamicSuctionGripper 정의부 참고).
GRASP_HORIZONTAL_MARGIN = 0.03
GRASP_VERTICAL_TOLERANCE = 0.02

PLACEMENT_JSON = OUT_DIR / "placement_result.json"
TRUNK_META_JSON = OUT_DIR / "trunk_pointcloud_meta.json"
TRUNK_MAP_JSON = OUT_DIR / "trunk_map.json"
# TEST_BOX_SIZE는 92번과 달리 상수가 아니다 - PICK 단계에서 실제로 집은 박스의 진짜 크기
# (BOX_KNOWN_SIZE[picked_prim_path])로 나중에 할당된다(아래 PICK 섹션 참고). STAGE1 이후
# 92번 코드를 그대로 재사용하는 함수들(_get_box_x_edges 등)이 이 이름을 참조한다.

# ---------------- 91.cart_pick_holonomic.py와 동일 - 카트/박스 구성 ----------------
PERCEPTION_DIR = _THIS_DIR / "perception"
CART_USD = str(_THIS_DIR / "assets/Metal_Shopping_Cart.usdz")
CART_POS = (0.0, 0.0, 0.0)
CART_EXTRA_SCALE = 0.55
CART_BASKET_FLOOR_Z = 0.68
# 사용자 설계(카트 양쪽 접근 재설계) - 카트를 90도 돌려 손잡이(A단)가 차량/트렁크에서
# 먼 쪽(-X)에, 손잡이 없는 입구(B단)가 가까운 쪽(+X)에 오도록 만든다("카트 입구가
# 차량을 바라보도록"). Metal_Shopping_Cart.usdz는 이름 붙은 하위 부품이 없는 통짜
# 메시라 어느 회전 부호가 실제로 손잡이를 -X쪽에 두는지 코드만으로 확정할 수 없다 -
# +90.0을 우선 시도값으로 넣었다. 스폰 직후 스크린샷(_cart2trunk_00_start.png와 동일한
# 지점에서 찍음)에서 손잡이가 카트의 +X쪽(차량에 가까운 쪽)에 보이면 이 값의 부호만
# -90.0으로 뒤집으면 된다 - 그 아래의 A/B단 정의(-X=A, +X=B)나 왼쪽/오른쪽 접근 로직은
# 전혀 손댈 필요 없다(부호를 뒤집으면 메시 전체가 반대로 돌아 손잡이가 -X쪽으로
# 오므로, 우리 코드의 "A=-X" 정의와 실제 메시가 다시 일치하게 된다).
CART_ROT_Z = 90.0
BASE_TO_CAMERA_TRANSFORM_JSON = PERCEPTION_DIR / "base_to_camera_transform.json"
# 91번의 GRASP_STANDOFF/DESCENT_MAX_SPEED/DESCENT_OVERTRAVEL/RIM_CLEARANCE/HOVER_ABOVE_BOX_TOP -
# 카트 PICK 전용 값이라 92번엔 없던 것들, 그대로 가져온다.
GRASP_STANDOFF = 0.01
DESCENT_MAX_SPEED = 0.005
DESCENT_OVERTRAVEL = 0.06
RIM_CLEARANCE = 0.10
HOVER_ABOVE_BOX_TOP = 0.30


def load_recommended_dims():
    import csv
    csv_path = OUT_DIR / "_evaluate_low_profile_base.csv"
    if csv_path.exists():
        with csv_path.open() as f:
            rows = [r for r in csv.DictReader(f) if r["feasible"] == "True"]
        if rows:
            rows.sort(key=lambda r: (-float(r["trunk_insertion_depth_m"]), float(r["base_length"])))
            best = rows[0]
            return float(best["base_length"]), float(best["base_width"]), float(best["base_height"])
    return 0.50, 0.50, 0.15


BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT = load_recommended_dims()
WHEEL_RADIUS = max(0.05, BASE_HEIGHT / 2.0)
CHASSIS_BODY_HEIGHT = min(BASE_HEIGHT, 2 * WHEEL_RADIUS) * 0.7
ROLLER_RADIUS = WHEEL_RADIUS * 0.22
ROLLER_LENGTH = (2 * np.pi * (WHEEL_RADIUS - ROLLER_RADIUS)) / ROLLER_COUNT * 1.15
HUB_RADIUS = WHEEL_RADIUS - ROLLER_RADIUS * 0.85
HUB_THICKNESS = WHEEL_RADIUS * 0.55
CHASSIS_LENGTH_EXTENDED = 1.00
WHEEL_MOUNT_HALF_L = BASE_LENGTH / 2.0 - WHEEL_RADIUS * 0.6
CHASSIS_HALF_LENGTH_EFFECTIVE = CHASSIS_LENGTH_EXTENDED / 2.0 + WHEEL_RADIUS * 0.6
# 91.cart_pick_holonomic.py와 동일 - 카트는 옆에서 붙으므로(트렁크처럼 정면이 아니라) 폭
# 기준 standoff가 필요하다.
_wheel_half_thickness_y = HUB_THICKNESS / 2.0 + ROLLER_LENGTH * 0.5 + ROLLER_RADIUS
CHASSIS_HALF_WIDTH_EFFECTIVE = BASE_WIDTH / 2.0 + _wheel_half_thickness_y * 1.3


def add_asset(stage, prim_path, usd_path, position, extra_scale, target_mpu, target_up, rot_z=0.0):
    src_stage = Usd.Stage.Open(usd_path)
    src_mpu = UsdGeom.GetStageMetersPerUnit(src_stage)
    src_up = UsdGeom.GetStageUpAxis(src_stage)
    scale = (src_mpu / target_mpu if target_mpu else src_mpu) * extra_scale
    xform = UsdGeom.Xform.Define(stage, prim_path)
    prim = xform.GetPrim()
    prim.GetReferences().AddReference(usd_path)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(position)
    if rot_z:
        xform.AddRotateZOp().Set(rot_z)
    if src_up == UsdGeom.Tokens.y and target_up == UsdGeom.Tokens.z:
        xform.AddRotateXOp().Set(90.0)
    xform.AddScaleOp().Set((scale, scale, scale))
    return xform


def add_sdf_collision(stage, root_prim_path, sdf_resolution=SDF_RESOLUTION):
    root_prim = stage.GetPrimAtPath(root_prim_path)
    n = 0
    for prim in Usd.PrimRange(root_prim):
        if prim.GetTypeName() == "Mesh":
            UsdPhysics.CollisionAPI.Apply(prim)
            mc = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mc.CreateApproximationAttr().Set("sdf")
            sdf_api = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
            sdf_api.CreateSdfResolutionAttr().Set(sdf_resolution)
            n += 1
    print(f"[SDF] {root_prim_path}: {n} mesh", flush=True)


def add_static_collision(stage, root_prim_path):
    """배경 환경(GarageEnvironment.usd)용 - SDF가 아니라 기본 삼각형 메시 콜리전을 쓴다.
    차량(add_sdf_collision)과 달리 이 메시는 항상 정적(RigidBodyAPI 없음)이라 동적 바디와의
    concave-vs-concave 충돌이 발생하지 않으므로 SDF 베이크(메시 77개 x 256 해상도, 매우 느림)가
    필요 없다 - 정적 삼각형 메시 콜리전은 동적 바디(로봇 바퀴/박스)와 문제없이 충돌한다."""
    root_prim = stage.GetPrimAtPath(root_prim_path)
    n = 0
    for prim in Usd.PrimRange(root_prim):
        if prim.GetTypeName() == "Mesh":
            UsdPhysics.CollisionAPI.Apply(prim)
            n += 1
    print(f"[정적 콜리전] {root_prim_path}: {n} mesh", flush=True)


def add_textured_sign(stage, prim_path, texture_path, position, size, rot_z=0.0):
    """GarageEnvironment.usd 안의 /World/SignRokeyMart는 원본 SignRokeyMart.usd 참조가 빠져있어
    (사용자가 로고 이미지(png) 파일만 제공함) 빈 프림으로 남는다 - 대신 그 png를 텍스처로 쓰는
    평면(XY 아님, XZ 평면 - 벽에 거는 사인판이라 세워서 배치)을 직접 만들어 붙인다."""
    w, h = size
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.CreatePointsAttr([
        Gf.Vec3f(-w / 2, 0.0, -h / 2), Gf.Vec3f(w / 2, 0.0, -h / 2),
        Gf.Vec3f(w / 2, 0.0, h / 2), Gf.Vec3f(-w / 2, 0.0, h / 2),
    ])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateExtentAttr([(-w / 2, 0.0, -h / 2), (w / 2, 0.0, h / 2)])
    mesh.CreateDoubleSidedAttr(True)
    uv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    uv.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    uv.SetIndices(Vt.IntArray([0, 1, 2, 3]))

    xform = UsdGeom.Xformable(mesh)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    if rot_z:
        xform.AddRotateZOp().Set(rot_z)

    mat_path = f"{prim_path}/Material"
    material = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    st_reader = UsdShade.Shader.Define(stage, f"{mat_path}/STReader")
    st_reader.CreateIdAttr("UsdPrimvarReader_float2")
    st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex = UsdShade.Shader.Define(stage, f"{mat_path}/Texture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture_path)
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        st_reader.ConnectableAPI(), "result")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    return mesh


def bbox_of(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    bbox = bbox_cache.ComputeWorldBound(prim)
    rng = bbox.ComputeAlignedRange()
    return np.array(rng.GetMin()), np.array(rng.GetMax())


def quat_between(v_from, v_to):
    v_from = np.array(v_from, dtype=float); v_from = v_from / np.linalg.norm(v_from)
    v_to = np.array(v_to, dtype=float); v_to = v_to / np.linalg.norm(v_to)
    dot = float(np.clip(np.dot(v_from, v_to), -1.0, 1.0))
    if dot > 0.999999:
        return Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    if dot < -0.999999:
        ortho = np.array([1.0, 0.0, 0.0]) if abs(v_from[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        axis = np.cross(v_from, ortho); axis = axis / np.linalg.norm(axis)
        return Gf.Quatf(0.0, float(axis[0]), float(axis[1]), float(axis[2]))
    axis = np.cross(v_from, v_to)
    w = 1.0 + dot
    q = np.array([w, axis[0], axis[1], axis[2]])
    q = q / np.linalg.norm(q)
    return Gf.Quatf(float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def build_mecanum_wheel(stage, wheel_root_path, chassis_path, local_pos, wheel_material_path, chirality, name):
    wx, wy, wz = local_pos
    hub_path = f"{wheel_root_path}/hub"
    hub = UsdGeom.Cylinder.Define(stage, hub_path)
    hub.CreateRadiusAttr(HUB_RADIUS)
    hub.CreateHeightAttr(HUB_THICKNESS)
    hub.CreateAxisAttr("Y")
    hub.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.2, 0.2)])
    hub_xform = UsdGeom.Xformable(hub)
    hub_xform.ClearXformOpOrder()
    hub_xform.AddTranslateOp().Set(Gf.Vec3d(wx, wy, wz))
    hub_prim = hub.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(hub_prim)
    UsdPhysics.MassAPI.Apply(hub_prim).CreateMassAttr().Set(HUB_MASS)

    hub_joint_path = f"{wheel_root_path}/joint_hub_{name}"
    hub_joint = UsdPhysics.RevoluteJoint.Define(stage, hub_joint_path)
    hub_joint.CreateAxisAttr("Y")
    hub_joint.CreateBody0Rel().SetTargets([Sdf.Path(chassis_path)])
    hub_joint.CreateBody1Rel().SetTargets([Sdf.Path(hub_path)])
    hub_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(wx, wy, 0.0))
    hub_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    drive = UsdPhysics.DriveAPI.Apply(hub_joint.GetPrim(), "angular")
    drive.CreateTypeAttr().Set("force")
    drive.CreateStiffnessAttr().Set(DRIVE_STIFFNESS)
    drive.CreateDampingAttr().Set(DRIVE_DAMPING)
    drive.CreateMaxForceAttr().Set(DRIVE_MAX_FORCE)
    drive.CreateTargetVelocityAttr().Set(0.0)

    for i in range(ROLLER_COUNT):
        theta = 2 * np.pi * i / ROLLER_COUNT
        place_r = HUB_RADIUS + ROLLER_RADIUS * 0.7
        rpos = np.array([place_r * np.cos(theta), 0.0, place_r * np.sin(theta)])
        tangent = np.array([-np.sin(theta), 0.0, np.cos(theta)])
        y_hat = np.array([0.0, 1.0, 0.0])
        roller_axis = tangent + chirality * y_hat
        roller_axis = roller_axis / np.linalg.norm(roller_axis)

        roller_path = f"{wheel_root_path}/roller_{i}"
        roller = UsdGeom.Capsule.Define(stage, roller_path)
        roller.CreateRadiusAttr(ROLLER_RADIUS)
        roller.CreateHeightAttr(ROLLER_LENGTH)
        roller.CreateAxisAttr("X")
        roller.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.35, 0.05)])
        quat = quat_between([1.0, 0.0, 0.0], roller_axis)
        r_xform = UsdGeom.Xformable(roller)
        r_xform.ClearXformOpOrder()
        r_xform.AddTranslateOp().Set(Gf.Vec3d(wx + rpos[0], wy + rpos[1], wz + rpos[2]))
        r_xform.AddOrientOp().Set(quat)
        r_prim = roller.GetPrim()
        UsdPhysics.CollisionAPI.Apply(r_prim)
        UsdPhysics.RigidBodyAPI.Apply(r_prim)
        UsdPhysics.MassAPI.Apply(r_prim).CreateMassAttr().Set(ROLLER_MASS)
        UsdShade.MaterialBindingAPI.Apply(r_prim).Bind(
            UsdShade.Material(stage.GetPrimAtPath(wheel_material_path)), materialPurpose="physics"
        )

        roller_joint_path = f"{wheel_root_path}/joint_roller_{name}_{i}"
        rjoint = UsdPhysics.RevoluteJoint.Define(stage, roller_joint_path)
        rjoint.CreateAxisAttr("X")
        rjoint.CreateBody0Rel().SetTargets([Sdf.Path(hub_path)])
        rjoint.CreateBody1Rel().SetTargets([Sdf.Path(roller_path)])
        rjoint.CreateLocalPos0Attr().Set(Gf.Vec3f(*rpos))
        rjoint.CreateLocalRot0Attr().Set(quat)
        rjoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        rjoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    return hub_joint_path


def build_holonomic_base(stage, start_xy, length, width, height):
    base_xform = UsdGeom.Xform.Define(stage, BASE_PATH)
    base_xform.ClearXformOpOrder()
    base_xform.AddTranslateOp().Set(Gf.Vec3d(start_xy[0], start_xy[1], 0.0))
    base_xform.AddRotateZOp().Set(BASE_FACE_ROT_Z)

    chassis_root = UsdGeom.Xform.Define(stage, CHASSIS_PATH)
    chassis_root.ClearXformOpOrder()
    chassis_root.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, WHEEL_RADIUS))
    chassis_prim = chassis_root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(chassis_prim)
    UsdPhysics.MassAPI.Apply(chassis_prim).CreateMassAttr().Set(CHASSIS_MASS)
    UsdPhysics.ArticulationRootAPI.Apply(chassis_prim)

    chassis_geom = UsdGeom.Cube.Define(stage, f"{CHASSIS_PATH}/geom")
    chassis_geom.CreateSizeAttr(1.0)
    chassis_geom_xform = UsdGeom.Xformable(chassis_geom)
    chassis_geom_xform.ClearXformOpOrder()
    chassis_geom_xform.AddScaleOp().Set(Gf.Vec3f(CHASSIS_LENGTH_EXTENDED, width, CHASSIS_BODY_HEIGHT))
    chassis_geom.CreateDisplayColorAttr([Gf.Vec3f(0.25, 0.30, 0.35)])
    UsdPhysics.CollisionAPI.Apply(chassis_geom.GetPrim())

    wheel_material = PhysicsMaterial(
        prim_path=f"{BASE_PATH}/roller_material",
        static_friction=1.0, dynamic_friction=0.9, restitution=0.0,
    )

    corner_signs = [(1, 1, 1), (1, -1, -1), (-1, 1, -1), (-1, -1, 1)]
    corner_names = ["FL", "FR", "RL", "RR"]
    wheel_half_thickness_y = HUB_THICKNESS / 2.0 + ROLLER_LENGTH * 0.5 + ROLLER_RADIUS
    half_l = WHEEL_MOUNT_HALF_L
    half_w = width / 2.0 + wheel_half_thickness_y * 1.3
    hub_joint_paths = []

    for (sx, sy, chirality), name in zip(corner_signs, corner_names):
        wx, wy, wz = sx * half_l, sy * half_w, 0.0
        wheel_root_path = f"{BASE_PATH}/wheel_{name}"
        hub_joint_path = build_mecanum_wheel(stage, wheel_root_path, CHASSIS_PATH, (wx, wy, wz),
                                              wheel_material.prim_path, chirality, name)
        hub_joint_paths.append(hub_joint_path)

    k_factor = half_l + half_w
    return CHASSIS_PATH, hub_joint_paths, k_factor


def add_drive_stiffness(stage, root_path, stiffness=1e8, damping=1e4, max_force=1e8):
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        for dof_type in ["angular", "linear"]:
            drive = UsdPhysics.DriveAPI.Get(prim, dof_type)
            if drive:
                drive.GetStiffnessAttr().Set(stiffness)
                drive.GetDampingAttr().Set(damping)
                drive.GetMaxForceAttr().Set(max_force)
                n += 1
    return n


def mount_m0609(stage, initial_h):
    m0609_path = "/World/HoloBase/M0609"
    m0609_xform = UsdGeom.Xform.Define(stage, m0609_path)
    m0609_xform.GetPrim().GetReferences().AddReference(M0609_USD, "/World/m0609")
    m0609_xform.ClearXformOpOrder()
    m0609_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, initial_h))

    for _ in range(20):
        simulation_app.update()

    base_link_path = f"{m0609_path}/base_link"
    root_joint_path = f"{m0609_path}/root_joint"
    if stage.GetPrimAtPath(root_joint_path).IsValid():
        stage.RemovePrim(root_joint_path)

    base_link_prim = stage.GetPrimAtPath(base_link_path)
    UsdPhysics.ArticulationRootAPI.Apply(base_link_prim)

    chassis_prim = stage.GetPrimAtPath(CHASSIS_PATH)
    filt_chassis = UsdPhysics.FilteredPairsAPI.Apply(chassis_prim)
    filt_chassis.CreateFilteredPairsRel().AddTarget(Sdf.Path(base_link_path))
    filt_base = UsdPhysics.FilteredPairsAPI.Apply(base_link_prim)
    filt_base.CreateFilteredPairsRel().AddTarget(Sdf.Path(CHASSIS_PATH))
    print(f"[필터] {CHASSIS_PATH} <-> {base_link_path} 충돌 필터링 적용", flush=True)

    lift_column_path = "/World/LiftColumnVisual"
    lift_column = UsdGeom.Cylinder.Define(stage, lift_column_path)
    lift_column.CreateRadiusAttr().Set(LIFT_COLUMN_RADIUS)
    lift_column.CreateHeightAttr().Set(1.0)
    lift_column.CreateAxisAttr("Z")
    lift_column.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.45, 0.1)])
    lift_column_xform = UsdGeom.Xformable(lift_column)
    lift_column_xform.ClearXformOpOrder()
    lift_translate_op = lift_column_xform.AddTranslateOp()
    lift_scale_op = lift_column_xform.AddScaleOp()
    lift_scale_op.Set(Gf.Vec3f(1.0, 1.0, 0.001))

    n = add_drive_stiffness(stage, m0609_path)
    print(f"[DRIVE] M0609={n}개 조인트 강성 적용, initial_h={initial_h:.3f}", flush=True)
    return m0609_path, base_link_path, lift_translate_op, lift_scale_op


def mecanum_wheel_speeds(vx, vy, wz, wheel_radius, k):
    vy = -vy
    return [
        (vx - vy - k * wz) / wheel_radius,
        (vx + vy + k * wz) / wheel_radius,
        (vx + vy - k * wz) / wheel_radius,
        (vx - vy + k * wz) / wheel_radius,
    ]


def quat_wxyz_to_matrix(q) -> np.ndarray:
    w, x, y, z = q
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array([
        [1 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1 - (xx + yy)],
    ])


class DynamicSuctionGripper(SurfaceGripper):
    """91.cart_pick_holonomic.py와 동일한 원통형(cylinder) 판정 - 구형 거리 판정은 수평/수직
    오차를 하나로 합쳐서 봐서, 스캔 좌표 기반 목표에 수평 오차가 있으면 수직 여유를 통째로
    잡아먹는 문제가 있었다(91.py에서 실측 확인 후 교체). 92번은 테스트 박스를 시작 시점에
    강제로 붙이는 용도라 이 판정 정밀도가 크게 문제되진 않았지만, 코드 일관성을 위해 91.py와
    동일한 API로 교체한다."""

    def __init__(self, end_effector_prim_path, gripper_body_path, tip_local_offset=(0.0, 0.0, 0.0)):
        SurfaceGripper.__init__(self, end_effector_prim_path=end_effector_prim_path, surface_gripper_path="")
        self._gripper_body_path = gripper_body_path
        self._tip_local_offset = Gf.Vec3d(*tip_local_offset)
        self._joint_path = f"{gripper_body_path}/suction_attach_joint"
        self._attached = False
        self._target_prim_path = None
        self._half_height = 0.0
        self._horizontal_tolerance = 0.0
        self._vertical_tolerance = 0.0

    def set_target(self, target_prim_path, half_height, horizontal_tolerance, vertical_tolerance):
        self._target_prim_path = target_prim_path
        self._half_height = half_height
        self._horizontal_tolerance = horizontal_tolerance
        self._vertical_tolerance = vertical_tolerance

    def close(self) -> None:
        if self._attached or self._target_prim_path is None:
            return
        stage = omni.usd.get_context().get_stage()
        target_prim = stage.GetPrimAtPath(self._target_prim_path)
        if not target_prim.IsValid():
            return
        gripper_mat = UsdGeom.Xformable(stage.GetPrimAtPath(self._gripper_body_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        target_mat = UsdGeom.Xformable(target_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        target_pos = target_mat.ExtractTranslation()
        tip_world = gripper_mat.Transform(self._tip_local_offset)
        horiz_dist = float(np.hypot(tip_world[0] - target_pos[0], tip_world[1] - target_pos[1]))
        vert_gap = float(tip_world[2] - (target_pos[2] + self._half_height))
        if horiz_dist > self._horizontal_tolerance or abs(vert_gap) > self._vertical_tolerance:
            return
        rel_local = target_mat.GetInverse().Transform(tip_world)
        gripper_rot = gripper_mat.ExtractRotationQuat()
        target_rot = target_mat.ExtractRotationQuat()
        local_rot1 = target_rot.GetInverse() * gripper_rot
        joint = UsdPhysics.FixedJoint.Define(stage, self._joint_path)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(self._gripper_body_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(self._target_prim_path)])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(self._tip_local_offset))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(rel_local))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(local_rot1))
        self._attached = True
        print(f"  [흡착] horiz={horiz_dist:.4f}m vert_gap={vert_gap:+.4f}m -> {self._joint_path} 생성", flush=True)

    def open(self) -> None:
        if self._attached:
            stage = omni.usd.get_context().get_stage()
            if stage.GetPrimAtPath(self._joint_path).IsValid():
                stage.RemovePrim(self._joint_path)
        self._attached = False

    def is_closed(self) -> bool:
        return self._attached

    def is_open(self) -> bool:
        return not self._attached


def get_world_pos(prim):
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array(mat.ExtractTranslation())


# ---------------- 91.cart_pick_holonomic.py와 동일 - 스캔 매칭용 헬퍼 ----------------
def world_aabb_from_base_corners(corners_base, base_pos, R_base):
    pts_base = np.asarray(corners_base, dtype=np.float64)
    pts_world = pts_base @ R_base.T + base_pos
    mn = pts_world.min(axis=0)
    mx = pts_world.max(axis=0)
    return (mn + mx) / 2.0, mx - mn, mn


def discover_box_prim_paths(stage):
    world_prim = stage.GetPrimAtPath("/World")
    return [str(c.GetPath()) for c in world_prim.GetChildren() if c.GetName().startswith("Box_")]


def match_physical_prim(stage, scan_center_world, available_paths):
    # XY만으로 매칭하면 같은 XY에 쌓인 박스(예: Large1 위의 Medium)를 구분할 수 없다
    # (실측 확인 - 100.py PICK 세션에서 box_id=0(Medium 치수)이 XY만 보고 /World/Box_Large1에
    # 잘못 매칭돼 Large1을 집으려다 Medium 윗면 높이를 목표로 삼는 바람에 흡착 실패했다).
    # scan_center_world는 Z까지 포함해야 스택된 박스를 구분할 수 있다.
    best_path, best_dist = None, None
    for path in available_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        pos = get_world_pos(prim)
        dist = float(np.linalg.norm(pos - np.asarray(scan_center_world)))
        if best_dist is None or dist < best_dist:
            best_path, best_dist = path, dist
    return best_path, best_dist



# ---- isaac_task_runner 추가: CEILING_WORLD_Z/SAFE_TRANSIT_Z 헤더 시점 폴백값 ----
# (진짜 값은 run_pick_and_place()가 trunk_map.json에서 매번 새로 읽어 이 이름을 덮어씀)
CEILING_WORLD_Z = TRUNK_WALL_TOP
SAFE_TRANSIT_Z = CEILING_WORLD_Z - 0.05


def compute_place_targets(placement_entry):
    """92번의 "PLACE 목표 사전계산"(place_release_z/place_world_xy/HOLDING_Z/
    ENTRY_HOLDING_Z) 그대로를 함수화한 것 - 박스 하나짜리 실행이던 92번은 이걸 로봇 스폰
    전에 한 번만 계산했지만, 94번은 박스마다(placement_entry가 다름) 다시 계산해야 한다.
    수식 자체는 92번과 완전히 동일 - TEST_BOX_SIZE는 여기선 아직 실제로 집기 전의
    placement_entry(비전 dimensions) 기반 잠정값이고, PICK 완료 후 실측 오프셋으로
    다시 한번 재보정된다(PICK 섹션 참고)."""
    global place_pos_base, place_dims, TEST_BOX_SIZE, PLACE_WORLD_MIN, PLACE_WORLD_CENTER
    global place_release_z, place_world_xy, HOLDING_Z, entry_box_bottom_clearance, ENTRY_HOLDING_Z

    place_pos_base = np.asarray(placement_entry["position_base_frame"], dtype=np.float64)
    place_dims = np.asarray(placement_entry["dimensions"], dtype=np.float64)
    TEST_BOX_SIZE = tuple(float(v) for v in place_dims)
    PLACE_WORLD_MIN = SCAN_R_BASE @ place_pos_base + SCAN_BASE_POS
    PLACE_WORLD_CENTER = SCAN_R_BASE @ (place_pos_base + place_dims / 2.0) + SCAN_BASE_POS
    print(f"[재투영] box_id={placement_entry['box_id']} place_world_min={np.round(PLACE_WORLD_MIN, 3)} "
          f"place_world_center={np.round(PLACE_WORLD_CENTER, 3)}", flush=True)

    place_release_z = (float(PLACE_WORLD_MIN[2]) + RELEASE_CLEARANCE_ABOVE_FLOOR
                        + float(place_dims[2]) + TIP_LOCAL_OFFSET[2])
    place_world_xy = (float(PLACE_WORLD_CENTER[0]), float(PLACE_WORLD_CENTER[1]))

    # 사용자 실측 확인("벽에 살짝살짝씩 계속 부딪혀") - algorism 배치 알고리즘은 94번이
    # 트렁크 안에 추가로 세운 가상 뒷벽(ArtificialBackWall, Isaac Sim 콜리전 전용)의 존재를
    # 모른 채 목표 X를 계산한다 - 박스 앞쪽 끝(목표X + 박스 반길이)이 벽 콜리전 앞면에서
    # 실측 6mm밖에 안 떨어진 경우가 있었다(STAGE 3.3이 흔들리는 동안 반복 충돌하는 원인).
    # 여기서 박스 앞쪽 끝이 벽 앞면 - PLACE_WALL_SAFETY_MARGIN보다 안쪽에 있도록 목표 X를
    # 강제로 당긴다.
    _box_half_x = float(place_dims[0]) / 2.0
    _box_front_edge_x = place_world_xy[0] + _box_half_x
    _max_allowed_front_edge_x = ARTIFICIAL_TRUNK_BACK_WALL_FRONT_X - PLACE_WALL_SAFETY_MARGIN
    if _box_front_edge_x > _max_allowed_front_edge_x:
        _pullback = _box_front_edge_x - _max_allowed_front_edge_x
        print(f"[가상 뒷벽 안전여유 보정] box_id={placement_entry['box_id']} 박스 앞쪽 끝({_box_front_edge_x:.3f})이 "
              f"벽 안전선({_max_allowed_front_edge_x:.3f})을 {_pullback:.3f}m 넘어서 목표X를 뒤로 당깁니다: "
              f"{place_world_xy[0]:.3f} -> {place_world_xy[0] - _pullback:.3f}", flush=True)
        place_world_xy = (place_world_xy[0] - _pullback, place_world_xy[1])

    HOLDING_Z = place_release_z + CARRY_CLEARANCE_ABOVE_RELEASE
    print(f"[PLACE 목표 사전계산] place_world_xy={np.round(place_world_xy, 3)} "
          f"place_release_z={place_release_z:.3f} HOLDING_Z={HOLDING_Z:.3f}", flush=True)
    if HOLDING_Z > SAFE_TRANSIT_Z:
        print(f"[경고] HOLDING_Z({HOLDING_Z:.3f})가 천장 안전 한계 SAFE_TRANSIT_Z({SAFE_TRANSIT_Z:.3f})를 "
              "넘습니다 - 이 배치 위치는 저상 측면 진입 전략으로 처리할 수 없습니다(재검토 필요).", flush=True)

    entry_box_bottom_clearance = (
        float(PLACE_WORLD_MIN[2]) + RELEASE_CLEARANCE_ABOVE_FLOOR + TIP_LOCAL_OFFSET[2]
        + ENTRY_CLEARANCE_ABOVE_RELEASE
    )
    ENTRY_HOLDING_Z = min(entry_box_bottom_clearance + TEST_BOX_SIZE[2], SAFE_TRANSIT_Z - 0.03)
    print(f"[STAGE 1.1 사전계산] ENTRY_HOLDING_Z={ENTRY_HOLDING_Z:.3f} "
          f"(문턱클리어런스{entry_box_bottom_clearance:.3f}+박스두께{TEST_BOX_SIZE[2]:.3f}, "
          f"천장한계 {SAFE_TRANSIT_Z:.3f} 이내로 클램프)", flush=True)


def box_place_needs_mirrored_pick(placement_entry):
    """이 박스의 최종 트렁크 배치 Y가 ANCHOR_Y(차량 중심선)보다 왼쪽(+Y)인지만 계산한다
    (compute_place_targets()의 PLACE_WORLD_CENTER 계산과 동일한 식 - 가상 뒷벽 안전여유
    보정은 X만 건드리므로 Y 판정엔 영향 없어 생략). True(왼쪽 배치)면 solution space
    2(조인트3/5 반전, _fold_target_mirrored)가 필요하다는 뜻 - link_2/link_5가 트렁크
    입구 왼쪽과 부딪히는 것을 막기 위해서다(94번에서 확인됨). 이 함수 자체는 "트렁크
    쪽 요구사항"만 판단한다 - "카트 쪽 요구사항"(손잡이 회피)은 아래
    box_leans_toward_cart_A_end()가 별도로 판단하고, 최종 카트 접근 방향은 이 둘을
    approach_cart_left_side()에서 조합해 정한다."""
    _pos_base = np.asarray(placement_entry["position_base_frame"], dtype=np.float64)
    _dims = np.asarray(placement_entry["dimensions"], dtype=np.float64)
    _world_center = SCAN_R_BASE @ (_pos_base + _dims / 2.0) + SCAN_BASE_POS
    return float(_world_center[1]) > ANCHOR_Y


def box_leans_toward_cart_A_end(picked_prim_path):
    """이 박스가 카트의 A단(손잡이, -X, 차량에서 먼 쪽)에 치우쳐 있는지를 스캔된 박스
    월드 X 위치(scan_box_top, 아래 PICK 섹션에서 pick_order와 함께 구성됨)와
    cart_center_xy[0]을 비교해서 판정한다 - CART_BOX_SPECS로 우리가 직접 스폰한 합성
    박스 위치가 아니라, 실제 PICK이 참조하는 것과 동일한 비전 매칭 결과를 쓴다(스캔
    캘리브레이션이 이번 카트 회전에 맞게 아직 재측정되지 않았다면 - 별도 후속 작업 -
    이 판정도 나머지 PICK 목표들과 똑같이 부정확해지지만, 최소한 "이 실행 안에서는
    일관되게" 어긋난다는 뜻이라 새로운 실패 모드는 아니다)."""
    return scan_box_top[picked_prim_path][0] < cart_center_xy[0]


def approach_cart_left_side(box_near_cart_A_end, solution_space_1):
    """사용자 설계 - 카트 양쪽 접근 XOR 규칙. "A에서 B를 바라봤을 때"를 기준으로 카트의
    왼쪽(+Y)/오른쪽(-Y) 중 어디서 PICK해야 손잡이와 부딪히지 않는지를, 박스의 A/B단
    치우침과 요구되는 solution space를 조합해서 정한다. 사용자가 직접 준 4가지 경우:
        박스 A단 + 트렁크 오른쪽(space1) -> 카트 왼쪽 접근
        박스 A단 + 트렁크 왼쪽(space2)   -> 카트 오른쪽 접근
        박스 B단 + 트렁크 오른쪽(space1) -> 카트 오른쪽 접근
        박스 B단 + 트렁크 왼쪽(space2)   -> 카트 왼쪽 접근
    표를 그대로 옮기면 "박스가 A단이라는 것"과 "solution space가 1(트렁크 오른쪽)이라는
    것"이 서로 같을 때(둘 다 참이거나 둘 다 거짓일 때) 왼쪽 접근, 다를 때 오른쪽
    접근이 되는 XOR-동치 관계다."""
    return box_near_cart_A_end == solution_space_1


# 사용자 설계 문서(Stage 2 한계 극복) - 지금까지의 STAGE 2/3(수평 이동)는 "박스+그리퍼 스택이
# 입구 수직 개구부에 다 들어가는" 박스에서만 통한다. 큰 박스는 수평으로는 절대 못 들어가므로
# 별도의 Tilt-and-Insert(문턱 앞에서 피치업 -> 기울인 채 통과 -> 내부에서 다시 수평 복원)가
# 필요하다 - 아래는 "이 박스가 어느 쪽인지"를 하드코딩된 박스별 임계값 없이 치수만으로
# 판정하는 함수. PROBE_ARM_ENVELOPE로 실측한 GRIPPER_ARM_OVERHEAD(그리퍼가 박스 위로 튀어
# 나오는 길이, 하드웨어 상수라 박스 크기와 무관)를 박스 높이에 더해 실제로 필요한 수직
# 공간을 구하고, 이미 계산된 CEILING_WORLD_Z/TRUNK_FLOOR_Z와 비교한다.
GRIPPER_ARM_OVERHEAD = 0.1127  # PROBE_ARM_ENVELOPE 실측(박스 바닥~그리퍼 최상단 - 박스 두께)
HORIZONTAL_PASS_MARGIN = 0.03  # 수평 통과 시 위아래 각각 남겨야 하는 최소 여유


def box_needs_tilt(box_height_z, ceiling_z=CEILING_WORLD_Z, floor_ref_z=TRUNK_FLOOR_Z):
    """box_height_z(박스를 수평으로 들었을 때의 두께)만으로 Tilt-and-Insert가 필요한지
    판정한다 - 박스별 하드코딩 임계값이 아니라 실측 상수(GRIPPER_ARM_OVERHEAD)와 이미 계산된
    천장/바닥 기준값으로 매번 새로 계산되므로 다른 크기 박스에도 그대로 재사용된다."""
    required = float(box_height_z) + GRIPPER_ARM_OVERHEAD + 2.0 * HORIZONTAL_PASS_MARGIN
    available = float(ceiling_z) - float(floor_ref_z)
    return required > available, required, available


# 사용자 설계(5차) - LIFT_TRAVEL_M=0.35(LIFT_MAX≈0.388)는 "차체 밑을 지나는" 시나리오의
# 안전마진인데, 스크립트 시작부터 계속 LIFT_MAX에 고정해두고 그 이후 ENTRY_HOLDING_Z(0.83)
# ->place_release_z 낙차를 팔 혼자서만 커버해왔다 - 팔이 그 큰 낙차+수평 reach를 동시에
# 감당하는 자세에서 팔꿈치/팔뚝이 트렁크 입구 프레임을 스쳤다(91.py PICK Phase A와 같은
# 원리로 해결: 리프트로 마운트 자체를 목표 높이 가까이 올리면 팔은 작은 나머지 거리만
# 커버하면 되어 자세가 컴팩트하게 유지된다). 이 시점(STAGE 3 마지막 PLACE 하강)에는 섀시가
# 이미 차체 밑이 아니라 트렁크 입구/안쪽에 있으므로, under-car 캡(LIFT_MAX) 대신 트렁크
# 천장 안전한계(SAFE_TRANSIT_Z)까지 리프트를 더 올려도 된다.


# ================= 씬 구성 =================
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
target_mpu = UsdGeom.GetStageMetersPerUnit(stage)
target_up = UsdGeom.GetStageUpAxis(stage)

# 배경 환경(지하주차장, 사용자 제공 GarageEnvironment.usd) - 카트(CART_POS)/차량(CAR_POS)/로봇
# 스폰(CHASSIS_SPAWN_XY, cart_center_xy 기준으로 아래에서 계산됨)은 전부 이 배경과 무관하게
# 정해지는 고정 좌표라 절대 건드리지 않는다 - 배경 쪽을 그 좌표들 위에 얹기만 한다.
# GARAGE_POS=(0,0,0)/스케일 1.0/회전 0으로 그대로 둬도 되는 이유(사용자 실측 확인) - 원본
# GarageEnvironment.usd를 열어 /World/Garage의 world bbox를 재보면 x=[-14.8, 19.8],
# y=[-12.1, 11.0], 바닥 z≈0(이미 target 스테이지와 동일 Z-up/미터 단위라 add_asset()의 축
# 변환도 필요 없음)이라 카트(x=0)와 차량(x=5)이 둘 다 넉넉히 안에 들어온다.
# world.scene.add_default_ground_plane()이 암묵적으로 제공하던 장면 전체 조명이 배경 교체로
# 같이 사라져서(실측 확인 - 배경 제거 전엔 안 보이던 문제) 직접 라이트를 추가한다. 기존
# area_light(SphereLight, 트렁크 근처 국소 조명)와는 별개로 카트/차량/배경 전체를 비추는
# 광역 조명이 필요하다.
_env_sun = UsdLux.DistantLight.Define(stage, "/World/DefaultSunLight")
_env_sun.CreateIntensityAttr(3000.0)
_env_sun.CreateAngleAttr(1.0)
UsdGeom.Xformable(_env_sun).AddRotateXOp().Set(-60.0)
UsdGeom.Xformable(_env_sun).AddRotateYOp().Set(20.0)
_env_dome = UsdLux.DomeLight.Define(stage, "/World/DefaultDomeLight")
_env_dome.CreateIntensityAttr(1000.0)

GARAGE_USD = os.environ.get("GARAGE_USD", "/home/rokey/Downloads/GarageEnvironment.usd")
GARAGE_POS = (0.0, 0.0, 0.0)
GARAGE_EXTRA_SCALE = 1.0
GARAGE_ROT_Z = 0.0

if Path(GARAGE_USD).exists():
    add_asset(stage, "/World/Environment", GARAGE_USD, Gf.Vec3d(*GARAGE_POS), GARAGE_EXTRA_SCALE,
              target_mpu, target_up, rot_z=GARAGE_ROT_Z)
    for _ in range(20):
        simulation_app.update()
    add_static_collision(stage, "/World/Environment")

    # GarageEnvironment.usd 안의 /World/SignRokeyMart는 원본 SignRokeyMart.usd 참조가 빠져있어
    # 빈 프림으로 남는다(사용자가 로고 이미지 파일만 줬음) - 대신 그 png로 직접 사인판을 만든다.
    # 실측 확인 - add_textured_sign()의 double-sided 평면을 기본 GUI Perspective 카메라로
    # 보면 글자가 좌우 반전되어 보여서(뷰포트 파일 캡처(vp_util.capture_viewport_to_file)로
    # 재확인해보니 그쪽은 오히려 항상 반전 저장되는 별개의 버그였다 - 실제 라이브 GUI 창을
    # X11로 직접 캡처해 대조한 뒤에야 확인됨), 3D 지오메트리 대신 텍스처 자체를 미리
    # 좌우 반전해 상쇄시킨다.
    SIGN_PNG = str(_THIS_DIR / "assets/rokey_mart_sign_mirrored.png")
    if Path(SIGN_PNG).exists():
        # 벽에 거는 형태(z=2.2, 배경 벽 메시에 의존) 대신 바닥에 서 있는 독립 사인판으로
        # 배치한다 - 배경(GarageEnvironment.usd)의 실제 벽 위치/존재 여부와 무관하게 항상
        # 보이도록 size 하단(= position z - size_h/2)을 지면(z=0)에 맞춘다.
        _SIGN_SIZE = (2.0, 1.2)
        add_textured_sign(stage, "/World/RokeyMartSign", SIGN_PNG,
                           position=(2.5, 3.0, _SIGN_SIZE[1] / 2), size=_SIGN_SIZE, rot_z=0.0)
else:
    print(f"[배경] {GARAGE_USD} 없음 - 기본 그리드 지면으로 대체", flush=True)
    world.scene.add_default_ground_plane()

add_asset(stage, "/World/Vehicle", CAR_USD, Gf.Vec3d(*CAR_POS), CAR_EXTRA_SCALE, target_mpu, target_up, rot_z=CAR_ROT_Z)
for _ in range(20):
    simulation_app.update()
add_sdf_collision(stage, "/World/Vehicle")

# 사용자 설계(STAGE 3.2.1 실측 후) - placement_result.json(적재 목표)이 실제 차체 메시보다
# 훨씬 깊은 트렁크를 가정하고 계산된 값이라(trunk_local depth=1.252m vs 실측 trunk_bounds
# x폭≈0.674m), STAGE 3.2.1이 실제로 도달 가능한 최대치까지 접근해도 목표에 못 미쳤다
# (실측: box center x≈3.427가 한계, 목표는 3.578). 차량 메시 자체를 수정하는 대신(단일
# 병합 메시라 트렁크 안쪽 벽만 따로 편집할 수 없음 - Blender 등 외부 툴 필요), 트렁크
# 내부에 실제로 도달 가능한 위치보다 살짝 안쪽에 콜리전이 있는 얇은 가상의 뒷벽을 세운다.
#
# 사용자 실측 확인 - 처음에 이 벽을 /World/Vehicle의 자식으로 넣었었는데, /World/Vehicle
# 자체가 add_asset()에서 usdz 단위 변환용 scale(≈0.005~0.006)+90도 회전+CAR_POS 이동을
# 이미 xformOp로 걸어둔 프림이다 - 그 밑에 자식을 넣으면 내가 준 "world 좌표" x=3.52가
# 그 부모 변환 안에서 로컬 좌표로 다시 해석돼(스케일까지 곱해져) 차량 원점 근처의 초소형
# 조각으로 렌더링됐다(실측: GUI에서 안 보임). 다른 마커들(_add_x_marker 등)과 동일하게
# /World/ 바로 밑에 독립 프림으로 만들어 world 좌표를 그대로 쓴다 - 대신 raycast 필터가
# "/World/Vehicle" 접두사만 인정하므로, 이 벽의 경로도 별도로 필터에 추가해줘야 한다
# (아래 _RAYCAST_VEHICLE_PREFIX 정의부에서 튜플로 확장).
# 사용자 지적(2차 수정) - x는 마지막 STAGE 3.2.1 실행에서 생긴 노란 마커(Stage3_2_1TargetPlane,
# 접근 한계 기반 목표 박스x)의 위치가 딱 맞다고 확인됨 - 3.52가 아니라 그 값(≈3.453)으로
# 당긴다(박스를 내려놓을 여유까지 감안한 값이라 3.52보다 입구 쪽으로 더 가까움).
#
# z는 원래 바닥 살짝 아래~천장 한참 위(0.41~1.31)까지 넉넉하게 잡았는데, 그러면 실제 고정
# 천장보다 훨씬 높은, 아직 열린 리드 밑면 구간(~1.4m대)까지 벽이 뻗어있는 것처럼 보인다는
# 지적. trunk_map.json을 직접 확인해보니 x별로 다른 천장 높이 데이터는 없고(단일 flat
# AABB - "ceiling_limit" 꼭짓점 하나만 있음, 설계상수/실측 중 더 낮은 값 채택), 그 값이
# 바로 이미 CEILING_WORLD_Z로 로드해서 쓰고 있는 값이다 - 벽의 z 상단을 CEILING_WORLD_Z로,
# 하단을 TRUNK_FLOOR_Z로 맞춘다(trunk_map이 실제로 갖고 있는 유일한 바닥/천장 값 그대로).
ARTIFICIAL_TRUNK_BACK_WALL_X = 3.400  # 사용자 최종 조정 - 노란 마커(3.453)보다 살짝 더 안전하게
_back_wall_z_lo = TRUNK_FLOOR_Z
_back_wall_z_hi = CEILING_WORLD_Z
_back_wall_z_center = (_back_wall_z_lo + _back_wall_z_hi) / 2.0
_back_wall_z_span = _back_wall_z_hi - _back_wall_z_lo
_back_wall = UsdGeom.Cube.Define(stage, "/World/ArtificialBackWall")
_back_wall.CreateSizeAttr(1.0)
_back_wall.CreateDisplayColorAttr([Gf.Vec3f(0.9, 0.1, 0.1)])
_back_wall_xform = UsdGeom.Xformable(_back_wall)
_back_wall_xform.ClearXformOpOrder()
_back_wall_xform.AddTranslateOp().Set(Gf.Vec3d(ARTIFICIAL_TRUNK_BACK_WALL_X, 0.0, _back_wall_z_center))
_back_wall_xform.AddScaleOp().Set(Gf.Vec3f(0.05, 1.6, _back_wall_z_span))  # 얇은 X, 넓은 Y, trunk_map 바닥~천장만큼 Z
UsdPhysics.CollisionAPI.Apply(_back_wall.GetPrim())
print(f"[가상 뒷벽] /World/ArtificialBackWall x={ARTIFICIAL_TRUNK_BACK_WALL_X:.3f} "
      f"z=[{_back_wall_z_lo:.3f}, {_back_wall_z_hi:.3f}](trunk_map 바닥~천장) 에 설치", flush=True)
# 사용자 실측 확인("벽에 살짝살짝씩 계속 부딪혀") - 벽은 크기(size=1.0)*스케일(0.05)이라
# X 반폭이 0.025m, 실제 콜리전 앞면은 ARTIFICIAL_TRUNK_BACK_WALL_X-0.025 = 3.375다.
# placement_result.json이 계산한 목표 X(예: 3.309)에 박스 반길이(예: 0.06)를 더한 앞쪽
# 끝은 3.369로, 벽 앞면(3.375)까지 여유가 실측 6mm뿐이었다 - STAGE 3.3이 목표에 딱
# 붙기도 전에(주행 중 흔들리는 동안) 박스 모서리가 벽에 반복해서 부딪히는 원인이었다.
# algorism 배치 알고리즘은 이 가상 벽(Isaac Sim 콜리전 전용, 오프라인 계산엔 없음)의
# 존재를 모르므로, 여기서 목표 X 자체에 확실한 안전 여유를 강제한다.
ARTIFICIAL_TRUNK_BACK_WALL_FRONT_X = ARTIFICIAL_TRUNK_BACK_WALL_X - 0.025
PLACE_WALL_SAFETY_MARGIN = 0.05

# ================= 실시간 지오메트리 함수 (설계 문서 6.1) =================
# 사용자 설계 문서 - "EE를 목표점으로 이동시키는 코드"에서 "박스+로봇 전체의 포락선이
# 트렁크의 구간별 자유공간 안에 유지되도록 하는 코드"로 전환하기 위한 기반. 93번 진단
# 스크립트가 이미 확인한 것처럼, 트렁크는 CEILING_WORLD_Z 하나로 대표되는 평평한 천장이
# 아니라 (1) 입구~내부천장 시작점까지는 열린 트렁크 리드 밑면이 실질적 천장, (2) 그 이후는
# 안쪽으로 갈수록 완만히 낮아지는 진짜 고정 지붕, 이렇게 두 구간이다. 바닥도 슬로프다.
# 하드코딩된 구간 경계(예: x=3.125) 대신, 차량 SDF 콜리전에 직접 raycast를 쏴서 "그 시점
# 실제 형상"을 재는 함수로 만든다 - 차량 스케일이 또 바뀌어도 코드 수정이 필요 없다(이번
# 세션에서 반복된 "예전 스케일 튜닝값이 새 스케일에서 안 맞음" 문제의 구조적 해결).
# 사용자 설계 - ArtificialBackWall은 /World/Vehicle의 자식이 아니라(부모 스케일/회전에
# 좌표가 다시 해석되는 문제 때문에) 독립된 /World/ArtificialBackWall로 만들었다 - 그래도
# 기존 raycast들이 이걸 "차체"로 인식하도록 접두사 튜플에 같이 넣는다(str.startswith는
# 튜플을 그대로 받는다).
_RAYCAST_VEHICLE_PREFIX = ("/World/Vehicle", "/World/ArtificialBackWall")


def _raycast_z(x, y, z_start, direction_z, max_dist=4.0):
    """93번 진단은 로봇/바닥 콜리전이 없는 격리된 씬이라 raycast_closest()만으로 충분했다.
    92.py는 홀로노믹 베이스/M0609(SDF 콜리전 있음)와 기본 지면(add_default_ground_plane)이
    함께 있는 씬이라, 필터 없이 raycast하면 로봇 자신이나 바닥을 맞혀 "천장"이 0m 근처로
    잘못 나오는 문제가 실측으로 확인됐다(STAGE 1에서 차량과 먼 위치일 때 특히 심함).
    raycast_all()로 모든 히트를 모으고 /World/Vehicle 콜리전만 걸러서 그중 가장 가까운
    것을 취한다."""
    closest = {"dist": None, "z": None}

    def _report(hit):
        path = hit.rigid_body or hit.collision
        if path and path.startswith(_RAYCAST_VEHICLE_PREFIX):
            if closest["dist"] is None or hit.distance < closest["dist"]:
                closest["dist"] = hit.distance
                closest["z"] = float(hit.position[2])
        return True  # 계속 다른 히트도 모은다(가장 가까운 차량 히트를 찾아야 하므로)

    get_physx_scene_query_interface().raycast_all(
        Gf.Vec3f(float(x), float(y), float(z_start)), Gf.Vec3f(0.0, 0.0, float(direction_z)), max_dist, _report)
    return closest["z"]


def ceiling_z_at(x, y=0.0):
    """위->아래 raycast로 그 (x,y)의 실제 천장(구간에 따라 열린 리드 밑면 또는 고정 지붕) z."""
    return _raycast_z(x, y, z_start=2.5, direction_z=-1.0)


def floor_z_at(x, y=0.0):
    """아래->위 raycast로 그 (x,y)의 실제 바닥/문턱 z."""
    return _raycast_z(x, y, z_start=-0.5, direction_z=1.0)


def _raycast_x(x_start, y, z, direction_x, max_dist=4.0):
    """_raycast_z와 완전히 같은 원리의 수평(X축) 버전 - ceiling_z_at/floor_z_at은 "위/아래"로
    막힌 면(트렁크 캐비티 개구부)을 재는데, 홀로노믹 베이스/리프트가 실제로 부딪히는 건
    "앞/뒤" 방향으로 막힌 차체 외피(범퍼 등) 표면이다 - 그건 수직 raycast로는 못 잰다."""
    closest = {"dist": None, "x": None}

    def _report(hit):
        path = hit.rigid_body or hit.collision
        if path and path.startswith(_RAYCAST_VEHICLE_PREFIX):
            if closest["dist"] is None or hit.distance < closest["dist"]:
                closest["dist"] = hit.distance
                closest["x"] = float(hit.position[0])
        return True

    get_physx_scene_query_interface().raycast_all(
        Gf.Vec3f(float(x_start), float(y), float(z)), Gf.Vec3f(float(direction_x), 0.0, 0.0), max_dist, _report)
    return closest["x"]


def vehicle_rear_surface_x_at(y=0.0, z=0.3):
    """차량 뒤쪽(-X, 차 밖) x=0에서 +X로 수평 raycast를 쏴서 차체 표면(범퍼 등)에 처음
    맞는 x를 잰다 - TRUNK_X_MIN/TRUNK_ENTRANCE_X(트렁크 캐비티 개구부 기준)와 달리, 이건
    차체 외피 자체의 실제 돌출 위치라 홀로노믹 베이스가 앞으로 접근할 수 있는 진짜 한계선이다."""
    return _raycast_x(x_start=0.0, y=y, z=z, direction_x=1.0)


def _raycast_y(x, y_start, z, direction_y, max_dist=4.0):
    """_raycast_x와 완전히 같은 원리의 Y축(좌우) 버전. 사용자 실측 확인("왼쪽 휠하우스는
    적용 안 된 것 같다 - 안 멈추는데?") - trunk_map.json은 y_min/y_max가 있는 단일 flat
    AABB일 뿐이라 휠하우스처럼 안쪽으로 튀어나온 좌우 비대칭 구조물은 전혀 반영하지
    못한다(실측 확인 - trunk_map.json에 그런 세부 형상 데이터 자체가 없음). 지금까지
    ceiling_z_at/floor_z_at(위/아래), vehicle_rear_surface_x_at(앞/뒤)만 있고 좌우
    방향 raycast가 아예 없었다 - STAGE 3.3이 Y로 스윕할 때 한쪽은 우연히 천장 클리어런스
    체크에 걸렸지만(로그상 "오른쪽은 적용된 것처럼" 보인 이유), 반대쪽은 아무 체크도
    없어 그냥 통과해버린 것이다."""
    closest = {"dist": None, "y": None}

    def _report(hit):
        path = hit.rigid_body or hit.collision
        if path and path.startswith(_RAYCAST_VEHICLE_PREFIX):
            if closest["dist"] is None or hit.distance < closest["dist"]:
                closest["dist"] = hit.distance
                closest["y"] = float(hit.position[1])
        return True

    get_physx_scene_query_interface().raycast_all(
        Gf.Vec3f(float(x), float(y_start), float(z)), Gf.Vec3f(0.0, float(direction_y), 0.0), max_dist, _report)
    return closest["y"]


def interior_side_wall_y_at(x, z, direction_y):
    """트렁크 내부에서 direction_y 방향(+1=왼쪽/-1=오른쪽)으로 쏴서 처음 맞는 표면
    (휠하우스 돌출/내벽)의 y - 실시간 raycast라 trunk_map.json에 없는 좌우 비대칭 형상도
    그 시점 실제 차체 콜리전 그대로 반영한다."""
    return _raycast_y(x, y_start=0.0, z=z, direction_y=direction_y)


def detect_internal_ceiling_start_x(y=0.0, x_lo=None, x_hi=None, step=0.02, drop_threshold=0.10):
    """TRUNK_ENTRANCE_X부터 TRUNK_X_MAX까지 ceiling_z_at()을 스캔해서 급격한 하강(열린
    트렁크 리드 밑면 -> 고정 내부 지붕으로의 전환)을 자동 검출한다 - x=3.125라는 매직넘버를
    하드코딩하지 않고, 지금 이 씬의 실제 차량 형상에서 실측으로 찾는다. 전환점을 못 찾으면
    (예: 이 차량 모델에 열린 리드에 의한 단차가 없는 경우) 안전하게 TRUNK_ENTRANCE_X를
    반환한다(구간 구분 없이 전체를 "고정 지붕"으로 취급하는 것과 동일 - 더 보수적)."""
    x_lo = TRUNK_ENTRANCE_X if x_lo is None else x_lo
    x_hi = TRUNK_X_MAX if x_hi is None else x_hi
    xs = np.arange(x_lo, x_hi + 1e-9, step)
    prev_z = None
    for x in xs:
        z = ceiling_z_at(x, y)
        if z is not None and prev_z is not None and (prev_z - z) > drop_threshold:
            return float(x)
        if z is not None:
            prev_z = z
    return float(x_lo)


area_light = UsdLux.SphereLight.Define(stage, "/World/TrunkPlaceAreaLight")
area_light.CreateRadiusAttr(0.3)
area_light.CreateIntensityAttr(80000)
UsdGeom.Xformable(area_light).AddTranslateOp().Set(Gf.Vec3d(TRUNK_X_MIN + 0.3, 0.0, TRUNK_FLOOR_Z + 1.0))

# ================= 91.cart_pick_holonomic.py와 동일 - 카트 + 박스 구성 =================
# CAR_POS=(5,0,0)과 CART_POS=(0,0,0)이 원래 두 파일에서 이미 서로 다른 x라 같은 씬에
# 합쳐도 겹치지 않는다 - 그 사이(약 5m)가 이번에 새로 검증하는 실제 주행 구간이다.
# 사용자 설계(카트 양쪽 접근 재설계) - rot_z=CART_ROT_Z로 카트를 90도 돌린다(add_asset의
# rot_z는 /World/Vehicle에서 이미 검증된 매개변수 그대로 재사용, 새 기능 아님).
add_asset(stage, "/World/ShoppingCart", CART_USD, CART_POS, CART_EXTRA_SCALE, target_mpu, target_up,
          rot_z=CART_ROT_Z)
for _ in range(20):
    simulation_app.update()
add_sdf_collision(stage, "/World/ShoppingCart")

cart_min, cart_max = bbox_of(stage, "/World/ShoppingCart")
cart_center_xy = ((cart_min[0] + cart_max[0]) / 2.0, (cart_min[1] + cart_max[1]) / 2.0)
# 90도 회전 후 cart_half_x/cart_half_y의 의미가 바뀐다 - cart_half_x는 이제 카트의
# "길이"(A-B, 손잡이-입구) 반쪽 폭, cart_half_y는 "너비"(양쪽 접근이 갈라지는 축) 반쪽
# 폭이다(bbox_of()가 회전 후 실제 월드 AABB에서 매번 새로 재는 값이라 이름은 그대로
# 둬도 값 자체는 자동으로 올바르게 뒤바뀐다 - 하드코딩 없음).
cart_half_x = (cart_max[0] - cart_min[0]) / 2.0
cart_half_y = (cart_max[1] - cart_min[1]) / 2.0
print(f"[카트 bbox] min={cart_min} max={cart_max} center_xy={cart_center_xy} "
      f"(회전후 half_x=길이/2={cart_half_x:.3f} half_y=너비/2={cart_half_y:.3f})", flush=True)

box_material = PhysicsMaterial(
    prim_path="/World/Physics_Materials/box_material",
    static_friction=1.2, dynamic_friction=1.0, restitution=0.0,
)
# 88.cart_scan_holonomic.py(3b061bd)/99.cart_scan_dual_side_holonomic.py에서 이식한
# 3박스 적층 시나리오(Large가 바닥, 그 위에 Medium+Small이 나란히) - 스캔/검출
# 파이프라인이 이제 이 구조로 카트를 채우므로(all_boxes_corners_*.json이 3개
# box_id를 담음), PICK&PLACE 데모 씬도 물리적으로 동일한 배치를 재현해야 비전
# 매칭(scan_by_box_id/match_physical_prim)이 실제 스폰된 박스와 맞아떨어진다 -
# 이전처럼 2개(Box_A/Box_B, 나란히 배치)만 스폰하면 3번째 비전 검출 결과와 매칭될
# 물리 박스가 없어 그 박스는 조용히 건너뛰어진다.
# 오프셋 축 매핑은 99번과 동일 - 88번은 dx=폭(옛 X)/dy=길이(옛 Y)였는데, 100번(회전
# 후)은 dx=길이(X)/dy=폭(Y)이므로 88번의 (dx, dy)를 (dy, dx)로 맞바꿔서 옮긴다.
# 사용자 요청(d0f2830 "Cart_Box_Specs 원상복구" 형태를 기반으로, 겹치지 않는 Large를
# 하나 더 추가) - 단일 Large 위에 Medium/Small이 나란히 앉는 원래(검증된) 상대
# 배치는 그대로 보존하고, 그 Large를 "Large1"로 옮긴 뒤 옆에 자식 없는 "Large2"만
# 하나 더 세운다(Large1/Large2 사이 간격 0.06m는 이전 Large1/Large2 리팩터에서
# 이미 실측 검증된 값 그대로 재사용 - 새로 값을 고르지 않는다). Medium/Small의
# 절대 offset은 "Large1 offset + 원래 Large 기준 로컬 offset"으로 그대로 평행이동
# 했을 뿐이라, Large 기준으로 봤을 때 각 자식과 Large 가장자리 사이 여유(이미
# 물리적으로 안정적이라고 확인된 값)는 전혀 바뀌지 않는다.
# [실측으로 확인된 버그 - 2026-07-26] Large 2개만 떨어뜨렸는데 착지 중 하나가
# 옆으로 넘어져 세로로 서버렸다 - 간격 0.06m가 너무 좁아서 낙하 중 살짝만 흔들려도
# 서로 모서리가 닿아 토크를 받고, 게다가 CART_BOX_DROP_HEIGHT_ABOVE_FLOOR=0.10m는
# Large 바닥면 기준 실제 자유낙하 거리가 0.04m(0.10 - 반두께 0.06)나 돼서 착지
# 충격/바운스가 컸다. 간격을 넉넉히 늘리고(카트 half_x=0.448m라 여유는 충분히
# 있음, 99_scan_run*.log의 [카트 bbox] 실측값 참고), 낙하 높이는 바닥에서 거의
# 뜨지 않을 정도(바닥면 기준 약 0.01m)로 낮춰서 두 Large가 각각 조용히 내려앉게
# 한다.
CART_LARGE_SIZE_XY = 0.18  # Large의 x/y 한 변 - 아래 간격 계산과 CART_BOX_SPECS가 항상 같은 값을 보게 여기서 한 번만 정의
CART_LARGE_GAP_M = 0.15
_cart_large_dx = (CART_LARGE_SIZE_XY + CART_LARGE_GAP_M) / 2.0  # 각 Large 중심이 카트 중심에서 떨어질 거리
# [실측으로 확인된 버그 - 2026-07-27, 99번과 동일] Medium/Small을 둘 다 Large2 위에
# 올렸더니(마진을 아무리 균등하게 나눠도) 둘이 Large2 폭의 ~90%를 차지해서 노출된
# 윗면이 얇은 테두리 조각들로만 남았고, RANSAC이 그 조각을 매번 다른 위치에서
# 우연히만 잡아서(10회/1회/1회 관측) 사각형 하나로 안정적으로 못 잡히고 전부
# 필터링됐다(Large1은 반대로 완전히 노출돼 있어서 151/150회 전부 검출됨 - 노출
# 면적 문제임을 확인). 그래서 Medium은 Large1 위에, Small은 Large2 위에 하나씩만
# 올린다 - 이러면 각 Large가 자식 하나(0.08~0.085m)만 갖고 나머지(0.18m 중 절반
# 가까이)는 그대로 노출되어 있어 둘 다 Large1처럼 안정적으로 검출된다. 위치는
# 각자의 부모 Large 중심에 그대로 올린다(모든 방향 마진이 넉넉함 - flush/타이트한
# 배치로 되돌아가지 않는다).
_medium_size = (0.085, 0.085, 0.11)
_small_size = (0.07, 0.08, 0.07)
CART_STACK_BASE_NAMES = ["Large1", "Large2"]
_STACK_PARENT = {"Medium": "Large1", "Small": "Large2"}  # 자식이 어느 Large 위에 앉는지
CART_BOX_SPECS = [
    # (name, size(x,y,z), 카트 중심 기준 offset(dx=길이축, dy=폭축), mass_kg)
    ("Large1", (CART_LARGE_SIZE_XY, CART_LARGE_SIZE_XY, 0.12), (-_cart_large_dx, 0.0), 1.2),
    ("Large2", (CART_LARGE_SIZE_XY, CART_LARGE_SIZE_XY, 0.12), (_cart_large_dx, 0.0), 1.2),
    ("Medium", _medium_size, (-_cart_large_dx, 0.0), 0.6),  # Large1 중심에 그대로(centered)
    ("Small", _small_size, (_cart_large_dx, 0.0), 0.3),  # Large2 중심에 그대로(centered)
]
CART_BOX_DROP_HEIGHT_ABOVE_FLOOR = 0.07
_CART_STACK_TOP_SPAWN_MARGIN_M = 0.05
# 사용자 요청 - 박스 전체 그룹을 카트 앞쪽(B단=입구, 손잡이 반대쪽, +X/차량에 가까운
# 쪽 - 위 CART_ROT_Z 주석 참고)으로 조금씩 옮긴다. CART_BOX_SPECS 안의 dx는 그대로
# 두고(Large1/Large2/Medium/Small 서로간 상대 배치 유지), 스폰 위치에서만 이 값을
# 더해 그룹 전체를 평행이동시킨다.
CART_BOX_FRONT_SHIFT_M = 0.07
# [사용자 실측 확인 - 2026-07-27, 99번과 동일] Large1이 카트 손잡이쪽(-X) 턱에
# 살짝 걸린다. Large2/Small은 문제 없으니 그대로 두고, Large1과 그 위에 얹힌
# Medium(같은 dx를 씀)만 추가로 조금 더 앞(+X, 손잡이 반대/입구 방향)으로 옮긴다 -
# 그룹 전체를 옮기는 CART_BOX_FRONT_SHIFT_M과는 별개로, 이 두 박스에만 얹어서 더한다.
CART_LARGE1_EXTRA_FRONT_SHIFT_M = 0.04
_extra_front_shift_by_name = {"Large1": CART_LARGE1_EXTRA_FRONT_SHIFT_M, "Medium": CART_LARGE1_EXTRA_FRONT_SHIFT_M}
_cart_box_size_by_name = {name: size for name, size, _off, _m in CART_BOX_SPECS}
_cart_large_spawn_z = CART_BASKET_FLOOR_Z + CART_BOX_DROP_HEIGHT_ABOVE_FLOOR
cart_box_objects = {}  # prim_path -> DynamicCuboid 래퍼(PICK 뒤 test_box로 재사용하기 위해 저장)
for _box_name, _box_size, (_dx, _dy), _mass_kg in CART_BOX_SPECS:
    if _box_name in CART_STACK_BASE_NAMES:
        _spawn_z = _cart_large_spawn_z
    else:
        _parent_size = _cart_box_size_by_name[_STACK_PARENT[_box_name]]
        _spawn_z = (
            _cart_large_spawn_z
            + _parent_size[2] / 2.0
            + _box_size[2] / 2.0
            + _CART_STACK_TOP_SPAWN_MARGIN_M
        )
    _box_prim_path = f"/World/Box_{_box_name}"
    cart_box_objects[_box_prim_path] = DynamicCuboid(
        prim_path=_box_prim_path, name=_box_name.lower(),
        position=np.array([
            cart_center_xy[0] + _dx + CART_BOX_FRONT_SHIFT_M + _extra_front_shift_by_name.get(_box_name, 0.0),
            cart_center_xy[1] + _dy, _spawn_z]),
        scale=np.array(_box_size), color=np.array([0.85, 0.55, 0.15]), mass=_mass_kg,
        physics_material=box_material,
    )
print(f"[박스 배치] 카트 안에 적층 구조 {len(CART_BOX_SPECS)}개 낙하 예정 "
      f"(바닥=Large1/Large2, Large1 위에 Medium, Large2 위에 Small)", flush=True)
# prim_path -> (sx, sy, sz) 스폰 시점의 진짜 크기(91번과 동일 이유 - bbox_of()로 매번 다시
# 재면 박스가 기울어졌을 때 신뢰할 수 없다).
BOX_KNOWN_SIZE = {f"/World/Box_{name}": size for name, size, _off, _m in CART_BOX_SPECS}

# 사용자 설계(카트 양쪽 접근 재설계) - 카트 옆 standoff가 이제 "폭(Y) 방향 양쪽 2곳"이다.
# CART_STANDOFF_DIST는 91번의 CART_STANDOFF_X와 완전히 동일한 계산이되, 카트의 길이가
# 아니라 너비(cart_half_y, 회전 후 의미)에 여유를 더한다 - 접근이 카트의 폭 방향
# 옆면이므로 91번의 "카트 옆에 홀로노믹 베이스 측면이 붙는다"는 기하학적 전제를 그대로
# 유지한다(BASE_FACE_ROT_Z=90도가 yaw=0/180에서 정확히 폭(width) 축을 Y로 보내므로,
# 이 standoff에 도착해서 yaw=0/180이 되면 실제로 옆면이 카트를 향한다 - 아래 CART_CLEAR_X
# 설명 참고, "스폰 시점"엔 아직 이 yaw가 아니라는 게 실측으로 확인된 함정이었다).
CART_STANDOFF_DIST = CHASSIS_HALF_WIDTH_EFFECTIVE + cart_half_y + CART_STANDOFF_MARGIN
# "A(손잡이,-X)에서 B(입구,+X)를 바라봤을 때"를 기준으로 왼쪽(+Y)/오른쪽(-Y) standoff.
CART_BASE_LEFT_XY = (cart_center_xy[0], cart_center_xy[1] + CART_STANDOFF_DIST)
CART_BASE_RIGHT_XY = (cart_center_xy[0], cart_center_xy[1] - CART_STANDOFF_DIST)

# 사용자 실측 확인("생성하자마자 충돌이 나서 터졌어") - 원인 분석: BASE_FACE_ROT_Z=90도가
# 고정이라 섀시는 항상 yaw=90도로 스폰된다. yaw=90도에서는 섀시의 "길이"(LENGTH, 바퀴
# 배치상 CHASSIS_HALF_LENGTH_EFFECTIVE, 팔/리프트를 얹기 위해 CHASSIS_LENGTH_EXTENDED=1.0m
# 로 늘려둔 축) 축이 월드 Y를 향하고 "폭"(WIDTH, CHASSIS_HALF_WIDTH_EFFECTIVE) 축이 월드
# X를 향한다(yaw=0/180에서는 반대: 길이->X, 폭->Y). CART_BASE_LEFT_XY/RIGHT_XY는 "도착
# 후(yaw=0/180) 폭 축이 Y를 향한다"는 전제로 Y 오프싯을 CHASSIS_HALF_WIDTH_EFFECTIVE
# 기준으로 계산했다 - 그런데 스폰 직후(아직 yaw=90도인 채)는 반대로 "긴" 길이 축이 Y를
# 향하므로, 그 standoff 자리에 yaw=90도로 그냥 스폰하면 섀시 몸체(길이 0.53m 반경)가
# 카트(폭 0.3m 반경) 쪽으로 그만큼 더 파고들어 스폰 즉시 겹친다(실측: 두 AABB가 X/Y
# 모두 겹침).
#
# 고침 - 카트의 "길이"(A-B) 축 연장선상, 카트 B단(+X, 입구/차량쪽)보다 한참 바깥에
# "회전 안전지대" CART_CLEAR_X를 둔다. 이 X에서는 섀시 중심이 카트 X범위와 이미
# (CHASSIS_HALF_LENGTH_EFFECTIVE+CHASSIS_HALF_WIDTH_EFFECTIVE, 즉 대각선 최댓값보다도
# 넉넉한) 여유로 떨어져 있어 X축만으로 두 AABB가 분리되므로, yaw가 0/90/180 중 무엇이든
# (심지어 회전 도중 45도 같은 중간값이어도) 절대 카트와 겹치지 않는다 - AABB 겹침
# 판정은 "한 축만 분리돼도 안 겹친다"는 성질을 이용한 것. 스폰은 이 안전지대에서 하고
# (yaw=90도가 뭘 향하든 안전), 아래 박스 루프의 "카트 접근" 주행을 3단계로 나눈다:
#   1단계(회전지대로 후퇴/유지) - X만 CART_CLEAR_X로(Y/yaw는 지금 값 유지, 회전 없음)
#   2단계(회전+횡이동) - X는 CART_CLEAR_X 고정(카트에서 먼 채), Y와 yaw만 목표로
#   3단계(최종 접근) - Y/yaw는 이미 목표값, X만 CART_CLEAR_X->cart_center_x로 직진
#     (이 3단계는 Y가 목표 standoff Y로 고정된 채라 카트와의 Y축 분리가 시종일관
#     유지되므로, 얼마나 카트 쪽으로 다가가든 안전하다).
CART_CLEAR_X = (cart_center_xy[0] + cart_half_x
                + CHASSIS_HALF_LENGTH_EFFECTIVE + CHASSIS_HALF_WIDTH_EFFECTIVE
                + CART_STANDOFF_MARGIN)

# 트렁크 standoff(92번과 완전히 동일한 계산) - 92번은 섀시를 여기서 스폰했지만, 100번은
# 카트 옆(CART_BASE_LEFT_XY/CART_BASE_RIGHT_XY 중 하나)에서 스폰하고 이 지점까지는
# 나중에 실제로 주행해서 도달한다(아래 "카트->트렁크 주행" 섹션 참고) - STAGE 3.2.0 등
# 92번 STAGE 3 코드가 여전히 BASE_START_XY를 "최초 대기 위치" 기준으로 참조하므로
# 이름/의미를 그대로 유지한다.
CHASSIS_HALF_LENGTH_EFFECTIVE_LOCAL = CHASSIS_HALF_LENGTH_EFFECTIVE
STANDOFF_TRUNK = CHASSIS_HALF_LENGTH_EFFECTIVE_LOCAL + STANDOFF_MARGIN
BASE_START_XY = (TRUNK_X_MIN - STANDOFF_TRUNK - 0.3, ANCHOR_Y)
# 사용자 설계 - 스폰은 CART_CLEAR_X(회전 안전지대)에서 한다(Y는 임의로 cart_center_xy[1]
# - 안전지대에서는 yaw=90도든 뭐든 Y값과 무관하게 안전하다). 박스 루프가 시작되면 첫
# 박스 역시 다른 모든 박스와 동일하게 "이 박스가 필요로 하는 standoff로 3단계 주행"부터
# 시작하므로(아래 박스 루프 참고) 박스1을 특별 취급할 필요가 없다.
CHASSIS_SPAWN_XY = (CART_CLEAR_X, cart_center_xy[1])
chassis_path, hub_joint_paths, k_factor = build_holonomic_base(
    stage, CHASSIS_SPAWN_XY, BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)

MEASURED_CHASSIS_TOP_OFFSET = 0.0180
LIFT_MIN = MEASURED_CHASSIS_TOP_OFFSET + M0609_MOUNT_Z_ABOVE_CHASSIS_TOP
# 사용자 실측 확인(STAGE 3.2.0 - "너무 높아졌는데??") - 92.trunk_place_holonomic.py의
# STAGE 3.x 코드는 그대로 재사용하는데, 그 코드 안에 "LIFT_MAX"를 직접 참조하는 곳이
# 여러 군데 있다(STAGE3.2.0의 STAGE3_2_0_LIFT_TARGET=LIFT_MAX+0.2, STAGE3.2.1의
# 천장 여유 리프트 상승 캡 등) - 92번에서 그 이름은 항상 "트렁크 접근용 리프트 상한"
# (LIFT_TRAVEL_M=0.35 -> 0.388)을 의미했다. 그런데 94번은 카트 PICK을 위해 LIFT_MAX를
# 91번 값(0.75 travel)으로 키워뒀었다 - 그러면 STAGE3.2.0이 LIFT_MAX+0.2를 계산할 때
# 0.788+0.2=0.988까지 리프트를 올려버려서(92번 의도 0.588의 거의 2배), 마운트가 팔의
# 도달범위를 넘어설 만큼 높아져 고정 ee 목표를 못 따라가고 발산했다(실측 err=0.056m로
# 중단). 고침: LIFT_MAX는 92번과 완전히 동일한 값(트렁크용)으로 되돌리고, PICK 전용
# 높은 리프트는 별도 이름(PICK_LIFT_H)으로 분리한다 - STAGE 3.x의 LIFT_MAX 참조는 전부
# 92번이 의도한 대로 동작하게 된다.
LIFT_MAX = LIFT_MIN + 0.35  # 92.trunk_place_holonomic.py와 완전히 동일값 - STAGE 3.x 전체가 이 이름을 직접 참조한다.
PICK_LIFT_H = LIFT_MIN + LIFT_TRAVEL_M  # 91.cart_pick_holonomic.py와 동일값(0.75 travel) - 카트 손잡이 위 호버 전용.
m0609_path, m0609_base_link_path, lift_translate_op, lift_scale_op = mount_m0609(stage, LIFT_MIN)
gripper_body_path = f"{m0609_path}/{GRIPPER_BODY_NAME}"
ee_path = f"{m0609_path}/{EE_LINK_NAME}"

# ================= 운반 포락선 측정 (설계 문서 6.2) =================
# 사용자 지적 - 실제로 입구/천장을 통과해야 하는 건 박스 혼자가 아니라 "박스 + 그리퍼 +
# link_6 + 손목/전완 링크"까지 포함한 하나의 강체 뭉치다. PROBE_ARM_ENVELOPE로 이미 검증한
# "메시 포인트를 Usd.TraverseInstanceProxies()로 인스턴스 안까지 순회하며 world로 직접
# 변환" 방식(BBoxCache의 치수 제곱 버그를 우회함이 실측으로 확인됨)을 Z축 전용에서 XYZ
# 전체로 일반화한다. M0609 URDF 기준 link_1~6 확인됨 - 손목/전완에 해당하는 link_4/5를
# 기본으로 포함한다(필요시 조정 가능).
CARRY_ENVELOPE_PARTS = [gripper_body_path, ee_path, f"{m0609_path}/link_5", f"{m0609_path}/link_4"]


def _mesh_world_aabb(root_prim_path):
    """PROBE_ARM_ENVELOPE의 _mesh_world_z_range를 XYZ 전체로 일반화 - 메시 포인트를
    직접 world로 변환해 축별 min/max를 구한다(BBoxCache 스케일 제곱 버그 회피, 검증됨)."""
    root_prim = stage.GetPrimAtPath(root_prim_path)
    mins = [None, None, None]
    maxs = [None, None, None]
    for prim in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies()):
        if prim.GetTypeName() != "Mesh":
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        if not pts:
            continue
        mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        for p in pts:
            wp = mat.Transform(p)
            for i in range(3):
                v = float(wp[i])
                mins[i] = v if mins[i] is None else min(mins[i], v)
                maxs[i] = v if maxs[i] is None else max(maxs[i], v)
    return tuple(mins), tuple(maxs)

for _ in range(20):
    simulation_app.update()

gripper = DynamicSuctionGripper(
    end_effector_prim_path=ee_path, gripper_body_path=gripper_body_path, tip_local_offset=TIP_LOCAL_OFFSET,
)
m0609_robot = SingleManipulator(
    prim_path=m0609_base_link_path, end_effector_prim_path=ee_path, name="m0609_arm", gripper=gripper,
)
base_robot = SingleArticulation(prim_path=chassis_path, name="holo_base")

world.reset()
base_robot.initialize(physics_sim_view=world.physics_sim_view)
m0609_robot.initialize(physics_sim_view=world.physics_sim_view)
m0609_robot.gripper.initialize(physics_sim_view=world.physics_sim_view, articulation_num_dofs=m0609_robot.num_dof)
print(f"[초기화] 섀시 dof_names={base_robot.dof_names}", flush=True)
print(f"[초기화] M0609 dof_names={m0609_robot.dof_names} num_dof={m0609_robot.num_dof}", flush=True)

hub_dof_indices = [base_robot.dof_names.index(Path(p).name) for p in hub_joint_paths]

_init_joints = np.zeros(m0609_robot.num_dof)
if "joint_3" in m0609_robot.dof_names:
    _init_joints[m0609_robot.dof_names.index("joint_3")] = np.pi / 2
if "joint_5" in m0609_robot.dof_names:
    _init_joints[m0609_robot.dof_names.index("joint_5")] = np.pi / 2
m0609_robot.set_joint_positions(_init_joints)

lift_state = {"h": LIFT_MIN}


def set_lift_height(h):
    chassis_pos, chassis_quat = base_robot.get_world_pose()
    target_pos = np.array([float(chassis_pos[0]), float(chassis_pos[1]), float(chassis_pos[2]) + h])
    m0609_robot.set_world_pose(position=target_pos, orientation=chassis_quat)
    m0609_robot.set_linear_velocity(np.zeros(3))
    m0609_robot.set_angular_velocity(np.zeros(3))
    column_base_z = float(chassis_pos[2]) + LIFT_MIN
    column_len = max(float(h) - LIFT_MIN, 0.001)
    lift_scale_op.Set(Gf.Vec3f(1.0, 1.0, column_len))
    lift_translate_op.Set(Gf.Vec3d(float(chassis_pos[0]), float(chassis_pos[1]), column_base_z + column_len / 2.0))


# [사용자 실측 확인 - 2026-07-27] PICK 하강/PLACE 입구 통과/Y축 정렬 구간에서 렉이
# 심하다는 제보 - 이 세 구간은 전부 스텝 수가 많은 저속 폐루프(drive_until의
# step_hold(), move_link6_smooth())라 공통점이 있다: 매 스텝 world.step(render=True)
# 로 "물리 스텝 + 화면 렌더링"을 항상 같이 했다. 물리는 결과/조건 판정에 필요해서
# 매 스텝 반드시 진행해야 하지만, 렌더링(화면 그리기)은 그렇지 않다 - 화면 갱신
# 빈도를 줄여도 물리 상태나 조건 충족 타이밍은 전혀 안 바뀐다. 그래서 물리는 계속
# 매 스텝 진행하되(world.step() 자체는 매번 호출) 렌더링만 RENDER_EVERY_N_STEPS
# 스텝에 한 번으로 줄여서 렉의 실제 원인(과도한 프레임 그리기)만 줄인다.
RENDER_EVERY_N_STEPS = 4
_render_step_counter = {"n": 0}


def sim_step():
    _render_step_counter["n"] += 1
    world.step(render=(_render_step_counter["n"] % RENDER_EVERY_N_STEPS == 0))


def step_hold(n=1):
    for _ in range(n):
        set_lift_height(lift_state["h"])
        sim_step()


def move_lift_to(target_h, steps=90):
    start_h = lift_state["h"]
    for i in range(steps):
        h = start_h + (target_h - start_h) * (i + 1) / steps
        set_lift_height(h)
        world.step(render=True)
    lift_state["h"] = target_h
    print(f"[리프트] {start_h:.3f} -> {target_h:.3f}", flush=True)


def holo_forward(vx, vy, wz):
    speeds = mecanum_wheel_speeds(vx, vy, wz, WHEEL_RADIUS, k_factor)
    return ArticulationAction(joint_velocities=speeds, joint_indices=hub_dof_indices)


SMOOTH_ALPHA = 0.12
_smooth_state = {"vx": 0.0, "vy": 0.0, "wz": 0.0}


def drive_to(target_x=None, target_y=None, target_yaw_deg=None, tolerance_xy=0.03, tolerance_yaw_deg=2.0,
             max_speed=0.4, max_wz=0.2, kp_xy=1.8, kp_yaw=0.25, max_steps=3000, label=""):
    start_pos, start_quat = base_robot.get_world_pose()
    start_yaw = float(np.degrees(quat_to_euler_angles(start_quat)[2]))
    tx = target_x if target_x is not None else float(start_pos[0])
    ty = target_y if target_y is not None else float(start_pos[1])
    tyaw = target_yaw_deg if target_yaw_deg is not None else start_yaw
    print(f"\n[주행 시작]{' ' + label if label else ''} 목표=({tx:.3f},{ty:.3f},{tyaw:.1f}deg)", flush=True)

    STALL_WINDOW, STALL_MIN_PROGRESS = 150, 0.008
    last_check_pos = np.array([float(start_pos[0]), float(start_pos[1])])
    stalled = False
    step = 0
    for step in range(1, max_steps + 1):
        pos, quat = base_robot.get_world_pose()
        yaw_deg = float(np.degrees(quat_to_euler_angles(quat)[2]))
        ex_w, ey_w = tx - float(pos[0]), ty - float(pos[1])
        eyaw = ((tyaw - yaw_deg + 180) % 360) - 180
        if abs(ex_w) < tolerance_xy and abs(ey_w) < tolerance_xy and abs(eyaw) < tolerance_yaw_deg:
            break
        yaw_rad = np.radians(yaw_deg)
        ex_l = ex_w * np.cos(yaw_rad) + ey_w * np.sin(yaw_rad)
        ey_l = -ex_w * np.sin(yaw_rad) + ey_w * np.cos(yaw_rad)
        vx_t = float(np.clip(kp_xy * ex_l, -max_speed, max_speed))
        vy_t = float(np.clip(kp_xy * ey_l, -max_speed, max_speed))
        wz_t = float(np.clip(np.radians(kp_yaw * eyaw), -max_wz, max_wz))
        _smooth_state["vx"] += SMOOTH_ALPHA * (vx_t - _smooth_state["vx"])
        _smooth_state["vy"] += SMOOTH_ALPHA * (vy_t - _smooth_state["vy"])
        _smooth_state["wz"] += SMOOTH_ALPHA * (wz_t - _smooth_state["wz"])
        base_robot.apply_action(holo_forward(_smooth_state["vx"], _smooth_state["vy"], _smooth_state["wz"]))
        step_hold(1)
        if step % STALL_WINDOW == 0:
            cur = np.array([float(pos[0]), float(pos[1])])
            progress = float(np.linalg.norm(cur - last_check_pos))
            if progress < STALL_MIN_PROGRESS and (abs(ex_w) > tolerance_xy or abs(ey_w) > tolerance_xy):
                stalled = True
                print(f"  [정체 감지] {progress:.4f}m밖에 못 움직임 - 중단", flush=True)
                break
            last_check_pos = cur
    for _ in range(30):
        _smooth_state["vx"] *= 1 - SMOOTH_ALPHA
        _smooth_state["vy"] *= 1 - SMOOTH_ALPHA
        _smooth_state["wz"] *= 1 - SMOOTH_ALPHA
        base_robot.apply_action(holo_forward(_smooth_state["vx"], _smooth_state["vy"], _smooth_state["wz"]))
        step_hold(1)
    final_pos, final_quat = base_robot.get_world_pose()
    final_yaw = float(np.degrees(quat_to_euler_angles(final_quat)[2]))
    print(f"[주행 완료]{' ' + label if label else ''} {step}스텝, 최종=({final_pos[0]:.3f},{final_pos[1]:.3f},"
          f"{final_yaw:.1f}deg) 정체={stalled}", flush=True)
    return final_pos, final_yaw, not stalled


def drive_until(condition_fn, target_x=None, target_y=None, target_yaw_deg=None,
                 tolerance_xy=0.03, tolerance_yaw_deg=2.0, max_speed=0.4, max_wz=0.2,
                 kp_xy=1.8, kp_yaw=0.25, max_steps=3000, label="",
                 debug_interval=0, debug_fn=None,
                 per_step_fn=None, abort_fn=None, hard_stop_on_condition=False,
                 max_speed_fn=None):
    """drive_to()와 동일한 폐루프 주행이되, (target_x,target_y) 도달 전이라도 매 스텝
    condition_fn()이 True가 되는 순간 즉시 멈춘다(사용자 설계 - "박스가 트렁크 입구를 완전히
    넘는 순간"처럼 실측 조건으로 정지해야 하는 경우, 섀시-박스 간 정확한 오프셋을 미리 계산
    하기보다 실측값을 직접 보고 멈추는 게 더 안전하다 - 33.py의 raise_lift_and_link6_until()과
    동일 원칙). target_x/y는 condition_fn이 끝내 만족되지 않을 때의 안전 상한(fallback)이다.

    사용자 지적(STAGE 2 충돌 분석) - 이전 버전은 매 스텝 step_hold(1)만 불러서 팔에 아무
    명령도 안 보냈다. set_lift_height()가 매 프레임 M0609 전체를 섀시 기준으로 텔레포트
    하므로 "명령을 안 보내면 관절값이 그대로 유지되며 따라오겠지"라고 기대했지만, 실측
    (STAGE=2 디버그 로그)으로 95~108스텝 구간에서 팁-섀시 X 오프셋이 0.360m -> 0.322m로
    줄고 Y가 갑자기 18cm 튀는 게 확인됐다 - 물리 충돌로 팔 자세가 눌린 것. 그런데 조인트
    명령을 안 보내는 것 자체가 "자세를 안 지킨다"는 뜻은 아니다(articulation이 자체
    강성/드라이브로 버티고 있었음) - 그보다 중요한 문제는 **그 충돌을 감지해서 즉시 멈추는
    장치가 없었다**는 것과 **충돌 후에도 30스텝 감속 관성으로 계속 밀고 들어갔다**는 것이다.
    고침:
    1. per_step_fn - 매 스텝(월드 스텝 직전) 호출해서 팔 관절을 명시적으로 다시 명령한다
       (관절 자체가 흔들리는 걸 최대한 억제 - 충돌해도 팔이 순순히 밀리지 않고 버티게 함).
    2. abort_fn - condition_fn과 별개로 "자세가 무너졌다"를 감지하는 별도 조건. 참이 되면
       실패로 즉시 중단한다(성공 조건과 구분 - 충돌로 튄 걸 성공으로 오판하지 않음).
    3. hard_stop_on_condition - 조건 충족/abort/정체 시 기존 30스텝 부드러운 감속(관성으로
       계속 전진) 대신 그 자리에서 즉시 속도를 0으로 만든다 - 이미 충돌했다면 더 밀어넣지
       않는다.
    4. max_speed_fn() - 인자 없이 매 스텝 호출되는 콜백. 주어지면 이 반환값을 속도 상한으로
       쓴다(입구 근접 시 저속 접근) - 호출부가 클로저로 "박스가 입구까지 얼마나 남았는지"
       같은 필요한 상태를 직접 캡처해서 판단한다."""
    start_pos, start_quat = base_robot.get_world_pose()
    start_yaw = float(np.degrees(quat_to_euler_angles(start_quat)[2]))
    tx = target_x if target_x is not None else float(start_pos[0])
    ty = target_y if target_y is not None else float(start_pos[1])
    tyaw = target_yaw_deg if target_yaw_deg is not None else start_yaw
    print(f"\n[주행 시작]{' ' + label if label else ''} 조건 충족 시 조기 정지, "
          f"안전상한=({tx:.3f},{ty:.3f},{tyaw:.1f}deg)", flush=True)

    STALL_WINDOW, STALL_MIN_PROGRESS = 150, 0.008
    last_check_pos = np.array([float(start_pos[0]), float(start_pos[1])])
    stalled = False
    condition_met = False
    aborted = False
    step = 0
    for step in range(1, max_steps + 1):
        if debug_interval and debug_fn is not None and step % debug_interval == 0:
            debug_fn(step)
        # 사용자 지적 - 같은 프레임에서 성공/붕괴 조건이 동시에 True가 될 수 있는데, 성공
        # 조건을 먼저 봐야 한다(성공 조건 자체에 이미 Y중앙/흡착유지 검사가 들어있으므로,
        # 그걸 만족했다면 그게 우선이다 - abort를 먼저 보면 정상 성공 프레임도 실패로
        # 오판할 수 있다).
        if condition_fn():
            condition_met = True
            print(f"  [조건 충족] {step}스텝에서 condition_fn() True - 주행 중단", flush=True)
            if debug_fn is not None:
                debug_fn(step)
            break
        if abort_fn is not None and abort_fn():
            aborted = True
            print(f"  [자세 붕괴 감지] {step}스텝에서 abort_fn() True - 주행 즉시 중단(실패)", flush=True)
            if debug_fn is not None:
                debug_fn(step)
            break
        pos, quat = base_robot.get_world_pose()
        yaw_deg = float(np.degrees(quat_to_euler_angles(quat)[2]))
        ex_w, ey_w = tx - float(pos[0]), ty - float(pos[1])
        eyaw = ((tyaw - yaw_deg + 180) % 360) - 180
        if abs(ex_w) < tolerance_xy and abs(ey_w) < tolerance_xy and abs(eyaw) < tolerance_yaw_deg:
            break
        # max_speed_fn은 인자 없이 호출한다 - 호출부가 클로저로 필요한 상태(예: 박스가
        # 입구까지 얼마나 남았는지)를 직접 캡처해서 판단하게 한다(chassis 자체의 목표
        # 거리(ex_w)는 "박스가 얼마나 남았는지"와 무관한 값이라 여기서 넘기지 않는다).
        step_max_speed = max_speed_fn() if max_speed_fn is not None else max_speed
        yaw_rad = np.radians(yaw_deg)
        ex_l = ex_w * np.cos(yaw_rad) + ey_w * np.sin(yaw_rad)
        ey_l = -ex_w * np.sin(yaw_rad) + ey_w * np.cos(yaw_rad)
        vx_t = float(np.clip(kp_xy * ex_l, -step_max_speed, step_max_speed))
        vy_t = float(np.clip(kp_xy * ey_l, -step_max_speed, step_max_speed))
        wz_t = float(np.clip(np.radians(kp_yaw * eyaw), -max_wz, max_wz))
        _smooth_state["vx"] += SMOOTH_ALPHA * (vx_t - _smooth_state["vx"])
        _smooth_state["vy"] += SMOOTH_ALPHA * (vy_t - _smooth_state["vy"])
        _smooth_state["wz"] += SMOOTH_ALPHA * (wz_t - _smooth_state["wz"])
        base_robot.apply_action(holo_forward(_smooth_state["vx"], _smooth_state["vy"], _smooth_state["wz"]))
        if per_step_fn is not None:
            per_step_fn()
        step_hold(1)
        if step % STALL_WINDOW == 0:
            cur = np.array([float(pos[0]), float(pos[1])])
            progress = float(np.linalg.norm(cur - last_check_pos))
            if progress < STALL_MIN_PROGRESS and (abs(ex_w) > tolerance_xy or abs(ey_w) > tolerance_xy):
                stalled = True
                print(f"  [정체 감지] {progress:.4f}m밖에 못 움직임 - 중단", flush=True)
                if debug_fn is not None:
                    debug_fn(step)
                break
            last_check_pos = cur

    if hard_stop_on_condition and (condition_met or aborted):
        # 이미 조건 충족(성공) 또는 자세 붕괴(실패)가 감지된 상황 - 관성으로 더 밀고 들어가지
        # 않도록 부드러운 감속 대신 그 자리에서 즉시 속도를 0으로 만든다.
        _smooth_state["vx"] = 0.0
        _smooth_state["vy"] = 0.0
        _smooth_state["wz"] = 0.0
        zero_action = holo_forward(0.0, 0.0, 0.0)
        for _ in range(8):
            base_robot.apply_action(zero_action)
            if per_step_fn is not None:
                per_step_fn()
            step_hold(1)
    else:
        for _ in range(30):
            _smooth_state["vx"] *= 1 - SMOOTH_ALPHA
            _smooth_state["vy"] *= 1 - SMOOTH_ALPHA
            _smooth_state["wz"] *= 1 - SMOOTH_ALPHA
            base_robot.apply_action(holo_forward(_smooth_state["vx"], _smooth_state["vy"], _smooth_state["wz"]))
            if per_step_fn is not None:
                per_step_fn()
            step_hold(1)
    final_pos, final_quat = base_robot.get_world_pose()
    final_yaw = float(np.degrees(quat_to_euler_angles(final_quat)[2]))
    print(f"[주행 완료]{' ' + label if label else ''} {step}스텝, 최종=({final_pos[0]:.3f},{final_pos[1]:.3f},"
          f"{final_yaw:.1f}deg) 조건충족={condition_met} 자세붕괴={aborted} 정체={stalled}", flush=True)
    return final_pos, final_yaw, condition_met, aborted


step_hold(60)
print("\n[안정화 완료]\n", flush=True)

# 사용자 설계 문서(1차: 기하학 측정 및 로그 출력) - 물리가 안정화된 지금 시점에 차량 SDF
# 콜리전에 직접 raycast를 쏴서 실제 입구~내부천장 전환점을 찾는다(하드코딩된 x=3.125 대신).
INTERNAL_CEILING_START_X = detect_internal_ceiling_start_x() - 0.05
print(f"[지오메트리 실측] INTERNAL_CEILING_START_X={INTERNAL_CEILING_START_X:.3f} "
      f"(TRUNK_ENTRANCE_X={TRUNK_ENTRANCE_X:.3f}~TRUNK_X_MAX={TRUNK_X_MAX:.3f} 구간 raycast 스캔)", flush=True)
for _x in [TRUNK_ENTRANCE_X, TRUNK_X_MIN, INTERNAL_CEILING_START_X, TRUNK_X_MAX]:
    _cz, _fz = ceiling_z_at(_x), floor_z_at(_x)
    print(f"  x={_x:.3f}: ceiling_z={_cz} floor_z={_fz}", flush=True)

# ================= 진입 가능성 판정 (설계 문서 2차: 6.4-6.5) =================
# 사용자 설계 문서 - box_needs_tilt()는 "박스 높이 vs 천장-바닥 단일 차이"만 봐서, 박스의
# X방향 길이가 진입 포켓(입구~내부천장 시작점) 길이보다 긴지는 아예 확인하지 않는다(전략
# B가 필요한 진짜 조건 중 하나가 누락돼 있었음). 아래는 그 둘 다 보는 새 판정 함수 -
# 지금은 옛 box_needs_tilt()와 나란히 로그만 비교하고, 실제 STAGE 2 경로 분기는 아직
# 안 바꾼다(3~4차에서 교체 예정).
def box_corners_local(box_dims):
    hx, hy, hz = np.asarray(box_dims, dtype=float) / 2.0
    return np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])


def rotated_corner_extent(box_dims, pitch_deg, pivot_local_z):
    """피벗(그리퍼 tip = 대략 박스 상단) 기준으로 pitch_deg만큼 피치 회전했을 때 8개
    꼭짓점의 피벗-상대 world X 스윕과 최저/최고 Z를 반환한다 - box_height*sin(theta) 근사
    대신 실제 회전으로 계산한다(설계 문서 4B-1/6.5). 박스는 그리퍼 하향(DOWN_QUAT) 자세라
    로컬 X/Y/Z가 world X/Y/Z와 정렬돼 있다고 가정 - tilt_quat도 같은 축(월드 Y, 진행방향
    X-Z 평면) 기준 피치 델타로 만들어져 있으므로 일관된 근사다."""
    corners = box_corners_local(box_dims) - np.array([0.0, 0.0, pivot_local_z])
    theta = np.radians(pitch_deg)
    c, s = np.cos(theta), np.sin(theta)
    rotated_x = corners[:, 0] * c + corners[:, 2] * s
    rotated_z = -corners[:, 0] * s + corners[:, 2] * c
    return float(rotated_x.max() - rotated_x.min()), float(rotated_z.min()), float(rotated_z.max())


def find_min_tilt_angle(box_dims, pivot_world_z, floor_clear_z, ceiling_clear_z,
                         tilt_deg_candidates=range(2, 31, 2), bottom_margin=0.02, top_margin=0.02):
    """pivot_world_z(그리퍼 tip 높이)에서 회전시켰을 때, 회전된 최저점이 floor_clear_z보다
    높고 최고점이 ceiling_clear_z보다 낮은 가장 작은 각도를 찾는다(고정 12도 대신 탐색)."""
    pivot_local_z = float(box_dims[2]) / 2.0
    for deg in tilt_deg_candidates:
        _, lo_z, hi_z = rotated_corner_extent(box_dims, deg, pivot_local_z)
        world_lo, world_hi = pivot_world_z + lo_z, pivot_world_z + hi_z
        if world_lo >= floor_clear_z + bottom_margin and world_hi <= ceiling_clear_z - top_margin:
            return deg
    return None


# [실측으로 확인된 버그 - 2026-07-26] classify_entry_strategy()의 "box_x_len <=
# transition_pocket_length" 판정이 여유(margin) 없이 딱 맞아 떨어지는지만 봤다 -
# 그런데 실제 STAGE2 성공 조건(_box_cleared_entrance(), 아래 ENTRANCE_CLEAR_MARGIN/
# FRONT_CLEAR_MARGIN 정의부 참고)은 "박스 뒤가 입구+0.005m를 넘고 박스 앞이
# 천장시작-0.01m 전이어야" 하므로, 실질적으로 여유 있게 통과하려면
# box_x_len <= transition_pocket_length - 0.015가 필요하다. 이 마진을 안 뺀 채로
# classify_entry_strategy가 "0.199 <= 0.21이니 HORIZONTAL_INSERT"라고 판정해버리면,
# 실제로는 여유가 없어 STAGE2가 입구~천장 사이에 낀 채로 자세 붕괴로 실패한다
# (실측 확인: 이 계산 없이 라우팅만 연결했을 때도 여전히 실패 재현됨). 두 마진
# 상수와 반드시 같은 값을 유지해야 하므로 여기서 함께 정의하고, 아래 STAGE 2의
# 지역 변수(ENTRANCE_CLEAR_MARGIN/FRONT_CLEAR_MARGIN)는 이 상수를 그대로 참조한다.
ENTRANCE_CLEAR_MARGIN_M = 0.005
FRONT_CLEAR_MARGIN_M = 0.01


def classify_entry_strategy(box_dims):
    """반환: (strategy, info) - strategy는 "HORIZONTAL_INSERT"|"TILT_AND_INSERT"|"INFEASIBLE".
    설계 문서 6.4 - box_needs_tilt()가 놓쳤던 "박스 X길이 vs 진입포켓 길이" 조건을 추가하고,
    Tilt 필요 시 find_min_tilt_angle()로 실제 실현 가능한 최소 각도까지 확인한다."""
    transition_pocket_length = INTERNAL_CEILING_START_X - TRUNK_ENTRANCE_X
    # STAGE2 실제 성공 조건과 동일한 마진을 빼서 비교한다(위 주석 참고) - 마진 없이
    # 딱 맞기만 하면 이론상은 HORIZONTAL_INSERT지만 실전에서는 여유가 없어 실패한다.
    usable_pocket_length = transition_pocket_length - ENTRANCE_CLEAR_MARGIN_M - FRONT_CLEAR_MARGIN_M
    box_x_len = float(box_dims[0])
    envelope_height = float(box_dims[2]) + GRIPPER_ARM_OVERHEAD + 2.0 * HORIZONTAL_PASS_MARGIN

    openings = []
    for x in np.arange(TRUNK_ENTRANCE_X, INTERNAL_CEILING_START_X, 0.02):
        cz, fz = ceiling_z_at(x), floor_z_at(x)
        if cz is not None and fz is not None:
            openings.append(cz - fz)
    worst_opening = min(openings) if openings else None

    info = {
        "transition_pocket_length": transition_pocket_length,
        "usable_pocket_length": usable_pocket_length, "box_x_len": box_x_len,
        "envelope_height": envelope_height, "worst_opening": worst_opening,
    }
    if worst_opening is not None and envelope_height <= worst_opening and box_x_len <= usable_pocket_length:
        info["strategy"] = "HORIZONTAL_INSERT"
        return "HORIZONTAL_INSERT", info

    floor_ref, ceiling_ref = floor_z_at(TRUNK_ENTRANCE_X), ceiling_z_at(TRUNK_ENTRANCE_X)
    angle = None
    if floor_ref is not None and ceiling_ref is not None:
        angle = find_min_tilt_angle(box_dims, pivot_world_z=ENTRY_HOLDING_Z,
                                     floor_clear_z=floor_ref, ceiling_clear_z=ceiling_ref)
    info["tilt_angle_deg"] = angle
    info["strategy"] = "TILT_AND_INSERT" if angle is not None else "INFEASIBLE"
    return info["strategy"], info


# 91.cart_pick_holonomic.py와 동일 - 리프트 상승과 조인트 3/5=90/90 접기를 같은 진행률로
# 동시에 보간한다(팔이 명령 없이 방치되는 구간을 없앰). 92번의 부트스트랩(_init_joints)이
# 이미 t=0에 이 접힘 값을 텔레포트로 넣어뒀으므로, 여기서는 사실상 리프트만 올라가고
# 조인트는 매 스텝 같은 값을 재확인만 한다(91번과 달리 0에서 접는 게 아님) - 그래도 "팔이
# 방치되는 구간이 없다"는 원칙은 동일하게 지킨다.
_fold_target = np.zeros(m0609_robot.num_dof)
if "joint_3" in m0609_robot.dof_names:
    _fold_target[m0609_robot.dof_names.index("joint_3")] = np.pi / 2
if "joint_5" in m0609_robot.dof_names:
    _fold_target[m0609_robot.dof_names.index("joint_5")] = np.pi / 2

# 사용자 지시(link_2가 왼쪽 place때 그리퍼보다 튀어나와 부딪히는 문제 대응 실험) - 조인트
# 3/5 부호를 반대로 접은 "미러 접힘" 자세. base의 위치/yaw는 건드리지 않고 팔 조인트만
# 반대로 접어서 RMPflow가 반대쪽 solution branch(팔꿈치 방향)에 붙도록 유도해본다.
_fold_target_mirrored = _fold_target.copy()
if "joint_3" in m0609_robot.dof_names:
    _fold_target_mirrored[m0609_robot.dof_names.index("joint_3")] = -np.pi / 2
if "joint_5" in m0609_robot.dof_names:
    _fold_target_mirrored[m0609_robot.dof_names.index("joint_5")] = -np.pi / 2


def raise_lift_and_fold(target_h, target_joints, steps=200):
    start_h = lift_state["h"]
    start_joints = np.array(m0609_robot.get_joint_positions(), dtype=float)
    for i in range(steps):
        alpha = (i + 1) / steps
        h = start_h + (target_h - start_h) * alpha
        j = start_joints + (target_joints - start_joints) * alpha
        m0609_robot.apply_action(ArticulationAction(joint_positions=j))
        set_lift_height(h)
        world.step(render=True)
    lift_state["h"] = target_h
    step_hold(20)
    print(f"[리프트+접기] 리프트 {start_h:.3f} -> {target_h:.3f}, 조인트 3/5=90/90(나머지 0) "
          f"완료: {np.round(m0609_robot.get_joint_positions(), 3)}", flush=True)


# ================= 91.cart_pick_holonomic.py와 동일 - 리프트 상승+접기 + 조인트 1(방위각)
# 조준을 하나의 함수로 묶는다 =================
# 사용자 지시("박스 2개 전부다 성공하도록") - 박스 1개만 다루던 처음엔 이 시퀀스를 스폰
# 직후 한 번만 실행했다. 이제 카트<->트렁크를 여러 번 왕복해야 하므로(각 박스마다 카트에서
# 다시 집어야 함), 매번 카트 옆 standoff에 도착했을 때 이 시퀀스를 다시 실행해야 한다
# (돌아올 때 리프트를 도킹 높이로 낮춰두므로, 다시 호버하려면 이 리프트 재상승 + 조준이
# 반드시 필요하다 - 안 하면 팔 혼자 리프트 없이 카트 손잡이 높이까지 뻗어야 해서 reach가
# 부족해진다). 함수로 묶어서 박스 루프 맨 앞(아래)에서 매번 호출한다.
def pick_raise_and_aim(fold_target=_fold_target):
    print(f"\n[리프트] 도킹({LIFT_MIN:.3f}) -> PICK 높이({PICK_LIFT_H:.3f}) + 조인트 3/5 접기"
          f"{'(반전)' if fold_target is _fold_target_mirrored else '(91번과 동일)'}", flush=True)
    raise_lift_and_fold(PICK_LIFT_H, fold_target, steps=200)

    # FK 실측 기반 계산(하드코딩 없음) - 지금 접은 자세에서 그리퍼가 실제로 어느 방향을 보고
    # 있는지와 지금 섀시 기준 카트 중심이 어느 방향인지를 둘 다 계산해서 그 차이만큼만 돌린다.
    # 91번과 완전히 동일한 계산(BASE_FACE_ROT_Z=90도도 그대로라 결과도 동일해야 정상).
    chassis_pos_now, chassis_quat_now = base_robot.get_world_pose()
    R_chassis_now = quat_wxyz_to_matrix(np.asarray(chassis_quat_now, dtype=float))
    ee_folded_pos0, _ = m0609_robot.end_effector.get_world_pose()
    delta_ee_local = R_chassis_now.T @ (
        np.array(ee_folded_pos0, dtype=float) - np.array(chassis_pos_now, dtype=float))
    ref_angle = float(np.arctan2(delta_ee_local[1], delta_ee_local[0]))

    delta_cart_local = R_chassis_now.T @ np.array([
        cart_center_xy[0] - float(chassis_pos_now[0]),
        cart_center_xy[1] - float(chassis_pos_now[1]),
        0.0,
    ])
    cart_angle = float(np.arctan2(delta_cart_local[1], delta_cart_local[0]))
    joint1_delta = ((cart_angle - ref_angle + np.pi) % (2 * np.pi)) - np.pi
    print(f"[조인트1 조준] 접은 자세 팁 방향각={np.degrees(ref_angle):.1f}deg "
          f"(수평거리={float(np.linalg.norm(delta_ee_local[:2])):.4f}m), "
          f"카트 방향각={np.degrees(cart_angle):.1f}deg -> joint_1 회전량={np.degrees(joint1_delta):.1f}deg",
          flush=True)

    aim_current = np.array(m0609_robot.get_joint_positions(), dtype=float)
    aim_target = aim_current.copy()
    if "joint_1" in m0609_robot.dof_names:
        aim_target[m0609_robot.dof_names.index("joint_1")] += joint1_delta
    aim_steps = 150
    for i in range(aim_steps):
        alpha = (i + 1) / aim_steps
        j = aim_current + (aim_target - aim_current) * alpha
        m0609_robot.apply_action(ArticulationAction(joint_positions=j))
        set_lift_height(lift_state["h"])
        world.step(render=True)
    step_hold(20)
    print(f"[조인트1 조준 완료] {np.round(m0609_robot.get_joint_positions(), 3)}", flush=True)


controller = RMPFlowController(
    name="cart_to_trunk_holonomic", robot_articulation=m0609_robot,
    urdf_path=M0609_URDF_PATH, robot_description_path=M0609_DESCRIPTION_PATH,
    rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH, end_effector_frame_name=EE_LINK_NAME,
)


def sync_rmp_base():
    chassis_pos, chassis_quat = base_robot.get_world_pose()
    base_pos = np.array([float(chassis_pos[0]), float(chassis_pos[1]), float(chassis_pos[2]) + lift_state["h"]])
    controller._default_position = base_pos
    controller._default_orientation = chassis_quat
    controller.rmp_flow.set_robot_base_pose(robot_position=base_pos, robot_orientation=chassis_quat)


def move_link6(target_pos, steps=WAYPOINT_STEPS, hold_gripper_closed=True, label="", orientation=DOWN_QUAT):
    for i in range(steps):
        sync_rmp_base()
        actions = controller.forward(
            target_end_effector_position=np.array(target_pos, dtype=float),
            target_end_effector_orientation=orientation,
        )
        m0609_robot.apply_action(actions)
        if hold_gripper_closed:
            m0609_robot.gripper.close()
        set_lift_height(lift_state["h"])
        world.step(render=True)
    ee_pos, _ = m0609_robot.end_effector.get_world_pose()
    err = np.linalg.norm(np.array(ee_pos) - np.array(target_pos))
    print(f"[웨이포인트{' ' + label if label else ''}] target={np.round(target_pos, 3)} "
          f"ee={np.round(ee_pos, 3)} err={err:.4f}m", flush=True)
    return ee_pos, err


def measure_tip_world_pos():
    # 91.cart_pick_holonomic.py와 동일 - DynamicSuctionGripper.close()와 완전히 같은
    # 방식(gripper_body_path의 실제 world transform + tip_local_offset)으로 흡착 팁의
    # 진짜 world 위치를 구한다. link_6과 vgp20_suction_plate 사이엔 TIP_LOCAL_OFFSET(약
    # 2cm)보다 훨씬 큰 고정 마운트 오프셋이 있어서(실측 13cm대), link_6 위치만 보고
    # "팁이 여기 있겠지"라고 가정하면 안 된다.
    gripper_mat = UsdGeom.Xformable(stage.GetPrimAtPath(gripper_body_path)).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    return np.array(gripper_mat.Transform(Gf.Vec3d(*TIP_LOCAL_OFFSET)), dtype=float)


def move_link6_smooth(target_tip_pos, tolerance=0.004, max_speed=0.01, kp=3.0, max_steps=2500,
                       hold_gripper_closed=False, try_grasp=False, label="", orientation=DOWN_QUAT):
    """91.cart_pick_holonomic.py와 동일 - link_6이 아니라 흡착 팁 자체를 폐루프로 제어한다
    (매 스텝 measure_tip_world_pos()로 실측, kp*오차를 max_speed로 클리핑). try_grasp=True면
    매 스텝 흡착을 시도해 붙는 순간 바로 멈춘다(카트 PICK의 크립 하강용)."""
    STALL_WINDOW, STALL_MIN_IMPROVEMENT = 200, 0.003
    target_tip_pos = np.array(target_tip_pos, dtype=float)
    step = 0
    tip_pos = None
    stalled = False
    last_check_err = None
    for step in range(1, max_steps + 1):
        if try_grasp and m0609_robot.gripper.is_closed():
            break
        tip_pos = measure_tip_world_pos()
        err_vec = target_tip_pos - tip_pos
        err = float(np.linalg.norm(err_vec))
        if err < tolerance:
            break
        if step % STALL_WINDOW == 0:
            if last_check_err is not None and (last_check_err - err) < STALL_MIN_IMPROVEMENT:
                stalled = True
                print(f"  [정체 감지{' ' + label if label else ''}] {step}스텝 동안 err {last_check_err:.4f}m -> "
                      f"{err:.4f}m밖에 안 줄어듦 - 목표 도달 불가(자기충돌/고착 의심)로 보고 중단", flush=True)
                break
            last_check_err = err
        step_vec = kp * err_vec
        step_norm = float(np.linalg.norm(step_vec))
        if step_norm > max_speed:
            step_vec = step_vec / step_norm * max_speed
        ee_pos, _ = m0609_robot.end_effector.get_world_pose()
        sync_rmp_base()
        actions = controller.forward(
            target_end_effector_position=np.array(ee_pos, dtype=float) + step_vec,
            target_end_effector_orientation=orientation,
        )
        m0609_robot.apply_action(actions)
        if try_grasp or hold_gripper_closed:
            m0609_robot.gripper.close()
        set_lift_height(lift_state["h"])
        sim_step()
    tip_pos = measure_tip_world_pos()
    err = float(np.linalg.norm(tip_pos - target_tip_pos))
    ee_pos, _ = m0609_robot.end_effector.get_world_pose()
    print(f"[완만한 접근{' ' + label if label else ''}] target_tip={np.round(target_tip_pos, 3)} "
          f"tip={np.round(tip_pos, 3)} link6={np.round(ee_pos, 3)} err={err:.4f}m steps={step} "
          f"stalled={stalled} grasped={m0609_robot.gripper.is_closed() if try_grasp else None}", flush=True)
    return tip_pos, err


def descend_and_raise_lift(target_xy, target_z, target_lift_h, steps=250,
                            orientation=DOWN_QUAT, hold_gripper_closed=True, label=""):
    """사용자 설계(5차, PICK Phase A와 동일 원리) - 리프트가 고정된 채로 팔만으로 큰 낙차를
    내리면, 그 낙차+수평 reach를 동시에 감당하는 자세에서 팔꿈치/팔뚝이 옆 구조물(여기서는
    트렁크 입구 프레임)을 스칠 수 있다. 리프트를 target_lift_h까지 올리면서(마운트 자체가
    목표 높이로 다가감) 동시에 link6의 절대 목표 z를 target_z까지 내린다 - 마운트가 목표에
    가까워질수록 팔이 커버해야 할 나머지 거리가 자연스럽게 줄어들어, 자세가 훨씬 컴팩트하게
    유지된다."""
    start_h = lift_state["h"]
    start_ee, _ = m0609_robot.end_effector.get_world_pose()
    start_z = float(start_ee[2])
    for i in range(steps):
        alpha = (i + 1) / steps
        h = start_h + (target_lift_h - start_h) * alpha
        z = start_z + (target_z - start_z) * alpha
        lift_state["h"] = h
        sync_rmp_base()
        actions = controller.forward(
            target_end_effector_position=np.array([target_xy[0], target_xy[1], z], dtype=float),
            target_end_effector_orientation=orientation,
        )
        m0609_robot.apply_action(actions)
        if hold_gripper_closed:
            m0609_robot.gripper.close()
        set_lift_height(h)
        world.step(render=True)
    ee_pos, _ = m0609_robot.end_effector.get_world_pose()
    err = float(np.linalg.norm(np.array(ee_pos) - np.array([target_xy[0], target_xy[1], target_z])))
    print(f"[리프트+하강{' ' + label if label else ''}] 리프트 {start_h:.3f} -> {lift_state['h']:.3f} "
          f"팔목표z {start_z:.3f} -> {target_z:.3f} ee={np.round(ee_pos, 3)} err={err:.4f}m", flush=True)
    return ee_pos, err


def reach_with_lift(target_ee_pos, target_lift_h, ee_orientation=DOWN_QUAT,
                     steps=250, hold_gripper_closed=True, label="",
                     abort_fn=None, hard_stop_on_condition=True,
                     debug_interval=0, debug_fn=None):
    """사용자 설계(STAGE 3 재설계 3차, 리프트-매니퓰레이터 협조 제어) - descend_and_raise_lift()와
    같은 원리로 리프트 높이를 선형 보간하지만, ee 목표도 시작점에서 target까지 같은 alpha로
    함께 보간한다(고정값을 한 번에 주지 않음).

    사용자 실측 확인(1차 구현의 버그) - 처음엔 move_link6/drive_and_reach처럼 target_ee_pos를
    고정값으로 주고 RMPflow가 알아서 수렴하게 했다. 그런데 STAGE 3.2는 X를 한 번에 0.5m 넘게
    (3.001->3.578) 점프시키는 큰 목표라, RMPflow가 그 큰 오차를 줄이는 과정에서 중간 경로가
    우리가 통제 못 하는 자세로 튀었다 - 실측 로그에서 ee z가 목표(0.765)에서 오히려 더 멀어지며
    (0.750->0.671로 하강, 목표와 반대 방향) 계속 낮아졌고, 결국 박스가 트렁크 바닥을 찍었다.
    큰 목표를 한 번에 주면 RMPflow가 이상한 해로 튈 수 있다는 이 프로젝트의 기존 교훈(STAGE 2/3
    재설계 과정에서 반복 확인됨)이 여기도 그대로 적용됐다. 고침: ee 목표도 리프트처럼 매 스텝
    아주 작은 걸음(waypoint)으로 나눠서 준다 - 그러면 RMPflow는 매 순간 "지금 위치에서 조금만
    움직이면 되는" 작은 오차만 풀면 되므로 중간 자세가 안정적으로 유지된다. target z를 시작
    z와 같게 주면(사용자 요청 - "Link5 상단 z값을 최대한 유지") 보간 경로 내내 z는 그대로
    유지된 채 x/y만 서서히 움직인다.

    사용자 지적(자세 붕괴 후 처리) - 원래는 붕괴 감지 후에도 계속 원래 target_ee_pos를 추종시켰다
    (drive_and_reach의 옛 방식과 동일) - 이미 위험한 목표를 향해 계속 풀려다 상황이 더 나빠질
    수 있다(STAGE 2/Tilt Phase3에서 이미 검증된 "얼려서 정지" 패턴과 반대). 고침: 붕괴 감지
    시점의 관절값을 그대로 얼려서 유지한다."""
    target_ee_pos = np.array(target_ee_pos, dtype=float)
    start_h = lift_state["h"]
    start_ee, _ = m0609_robot.end_effector.get_world_pose()
    start_ee = np.array(start_ee, dtype=float)
    aborted = False
    freeze_q = None
    step = 0
    for step in range(1, steps + 1):
        if debug_interval and debug_fn is not None and step % debug_interval == 0:
            debug_fn(step)
        if abort_fn is not None and abort_fn():
            aborted = True
            freeze_q = np.asarray(m0609_robot.get_joint_positions(), dtype=float).copy()
            print(f"  [자세 붕괴 감지] {step}스텝에서 abort_fn() True - 즉시 중단(실패)", flush=True)
            if debug_fn is not None:
                debug_fn(step)
            break
        alpha = step / steps
        h = start_h + (target_lift_h - start_h) * alpha
        ee_pt = start_ee + (target_ee_pos - start_ee) * alpha
        lift_state["h"] = h
        sync_rmp_base()
        actions = controller.forward(
            target_end_effector_position=ee_pt, target_end_effector_orientation=ee_orientation,
        )
        m0609_robot.apply_action(actions)
        if hold_gripper_closed:
            m0609_robot.gripper.close()
        set_lift_height(h)
        world.step(render=True)

    if hard_stop_on_condition and aborted:
        for _ in range(8):
            m0609_robot.apply_action(ArticulationAction(joint_positions=freeze_q))
            if hold_gripper_closed:
                m0609_robot.gripper.close()
            set_lift_height(lift_state["h"])
            world.step(render=True)

    ee_pos, _ = m0609_robot.end_effector.get_world_pose()
    ee_err = float(np.linalg.norm(np.array(ee_pos) - target_ee_pos))
    print(f"[리프트+팔 협조{' ' + label if label else ''}] {step}스텝, 리프트={lift_state['h']:.3f} "
          f"팔ee={np.round(ee_pos, 3)} err={ee_err:.4f}m 자세붕괴={aborted}", flush=True)
    return ee_pos, ee_err, aborted


def retreat_and_raise(target_chassis_x, target_lift_h, ee_target_pos, ee_orientation=DOWN_QUAT,
                       tolerance_xy=0.03, max_speed=0.06, kp_xy=1.0, lift_speed=0.002,
                       max_steps=2000, hold_gripper_closed=True, label="",
                       condition_fn=None, abort_fn=None, hard_stop_on_condition=True,
                       debug_interval=0, debug_fn=None, record_into=None, joint_trajectory=None):
    """사용자 설계(STAGE 3 재설계 5차) - 그리퍼/박스는 ee_target_pos(STAGE 3.1이 잡아둔 자리)에
    고정한 채, 섀시를 target_chassis_x까지 후진시키고 동시에 리프트를 target_lift_h까지
    올린다. 마운트가 그 고정 지점에서 뒤로+위로 물러날수록, 계속 그 자리를 추종해야 하는
    팔은 자연히 쭉 뻗은(수평에 가까운) 자세가 된다 - 굽은 팔꿈치보다 입구 프레임의 좁은
    세로 공간을 덜 차지하게 만드는 게 목적이다(link_2가 반복적으로 입구에 부딪히던 문제의
    기구학적 우회). drive_and_reach()(섀시+팔, 리프트 고정)와 reach_with_lift()(리프트+팔,
    섀시 고정)를 합친 버전 - condition_fn(관절이 충분히 폈는지)이 True가 되면 목표 지점
    전이라도 drive_until()과 동일한 원칙으로 조기 정지한다.

    사용자 질문("Link3이 아래쪽으로 접히는 자세를 후퇴 때도 그대로 재현하고 싶은데,
    RMPflow/IK라서 방향을 제어할 수 없나?") - RMPflow는 매 스텝 "지금 관절 상태에서
    목표에 가장 가까운 국소해"만 반응적으로 찾는 솔버라, 후퇴(STAGE 4)처럼 이 함수를
    거꾸로 다시 부르면 그 시점의 관절 상태가 조금만 달라도 다른 solution branch로
    수렴할 수 있다(이번 세션에서 반복 확인된 현상) - "IK라서 제어 불가능"한 게 아니라,
    후퇴 때도 매번 RMPflow에게 branch 선택을 새로 맡기고 있었던 게 문제다. 고침:
    RMPflow에게 매번 다시 풀게 하는 대신, 정방향(펴기, STAGE 3.2.0) 통과 때 실제로
    거쳐간 관절 궤적을 리프트 높이를 키(key)로 기록해뒀다가(record_into), 역방향
    (접기, STAGE 4-4)에서는 IK를 아예 다시 안 풀고 그 기록을 리프트 높이로
    보간(joint_trajectory)해서 직접 재생한다 - 같은 물리적 경로를 그대로 되짚으므로
    RMPflow가 다른 branch를 고를 여지 자체가 없어진다.
    - record_into: 리스트를 넘기면, 매 스텝 (리프트 높이, 그 순간 실제 관절값)을
      append한다(정방향 호출에서만 의미 있음 - RMPflow가 실제로 그 관절을 명령했을 때).
    - joint_trajectory: (heights_array, joints_matrix) 튜플을 넘기면, RMPflow를 아예
      건너뛰고 그 자리에서 현재 리프트 높이로 관절을 선형보간해 직접 명령한다(재생
      모드) - 리프트 높이는 두 방향 모두에서 공통으로 측정 가능한 물리량이라, 정방향
      기록과 역방향 재생을 이어주는 키로 쓸 수 있다."""
    ee_target_pos = np.array(ee_target_pos, dtype=float)
    start_pos, _ = base_robot.get_world_pose()
    ty = float(start_pos[1])
    aborted = False
    condition_met = False
    freeze_q = None
    last_replay_q = np.asarray(m0609_robot.get_joint_positions(), dtype=float).copy()
    step = 0
    for step in range(1, max_steps + 1):
        if debug_interval and debug_fn is not None and step % debug_interval == 0:
            debug_fn(step)
        if condition_fn is not None and condition_fn():
            condition_met = True
            print(f"  [조건 충족] {step}스텝에서 condition_fn() True - 후진/상승 중단", flush=True)
            if debug_fn is not None:
                debug_fn(step)
            break
        if abort_fn is not None and abort_fn():
            aborted = True
            freeze_q = np.asarray(m0609_robot.get_joint_positions(), dtype=float).copy()
            print(f"  [자세 붕괴 감지] {step}스텝에서 abort_fn() True - 즉시 중단(실패)", flush=True)
            if debug_fn is not None:
                debug_fn(step)
            break
        pos, quat = base_robot.get_world_pose()
        yaw_deg = float(np.degrees(quat_to_euler_angles(quat)[2]))
        ex_w, ey_w = target_chassis_x - float(pos[0]), ty - float(pos[1])
        chassis_done = abs(ex_w) < tolerance_xy and abs(ey_w) < tolerance_xy
        lift_done = abs(lift_state["h"] - target_lift_h) < 0.002
        if chassis_done and lift_done:
            break
        yaw_rad = np.radians(yaw_deg)
        ex_l = ex_w * np.cos(yaw_rad) + ey_w * np.sin(yaw_rad)
        ey_l = -ex_w * np.sin(yaw_rad) + ey_w * np.cos(yaw_rad)
        vx_t = float(np.clip(kp_xy * ex_l, -max_speed, max_speed))
        vy_t = float(np.clip(kp_xy * ey_l, -max_speed, max_speed))
        _smooth_state["vx"] += SMOOTH_ALPHA * (vx_t - _smooth_state["vx"])
        _smooth_state["vy"] += SMOOTH_ALPHA * (vy_t - _smooth_state["vy"])
        _smooth_state["wz"] *= (1 - SMOOTH_ALPHA)
        base_robot.apply_action(holo_forward(_smooth_state["vx"], _smooth_state["vy"], _smooth_state["wz"]))

        lift_delta = float(np.clip(target_lift_h - lift_state["h"], -lift_speed, lift_speed))
        lift_state["h"] += lift_delta

        if joint_trajectory is not None:
            # 재생 모드 - RMPflow를 건너뛰고, 정방향 통과가 기록해둔 (리프트 높이, 관절값)
            # 궤적을 지금 리프트 높이로 보간해 그대로 명령한다.
            _heights, _joints_matrix = joint_trajectory
            _replay_q = np.array([
                np.interp(lift_state["h"], _heights, _joints_matrix[:, _j])
                for _j in range(_joints_matrix.shape[1])
            ])
            m0609_robot.apply_action(ArticulationAction(joint_positions=_replay_q))
            last_replay_q = _replay_q
        else:
            sync_rmp_base()
            actions = controller.forward(
                target_end_effector_position=ee_target_pos, target_end_effector_orientation=ee_orientation,
            )
            m0609_robot.apply_action(actions)
        if hold_gripper_closed:
            m0609_robot.gripper.close()
        set_lift_height(lift_state["h"])
        world.step(render=True)
        if record_into is not None:
            record_into.append((float(lift_state["h"]),
                                 np.asarray(m0609_robot.get_joint_positions(), dtype=float).copy()))

    if hard_stop_on_condition and aborted:
        for _ in range(8):
            m0609_robot.apply_action(ArticulationAction(joint_positions=freeze_q))
            if hold_gripper_closed:
                m0609_robot.gripper.close()
            set_lift_height(lift_state["h"])
            world.step(render=True)
    else:
        for _ in range(20):
            _smooth_state["vx"] *= 1 - SMOOTH_ALPHA
            _smooth_state["vy"] *= 1 - SMOOTH_ALPHA
            base_robot.apply_action(holo_forward(_smooth_state["vx"], _smooth_state["vy"], _smooth_state["wz"]))
            if joint_trajectory is not None:
                # 재생 모드의 감속 꼬리 - IK를 다시 부르지 않고 마지막으로 재생한 관절값을
                # 그대로 유지한다(RMPflow를 부르면 이 마지막 몇 스텝에서 새삼 다른 branch로
                # 튈 여지를 다시 열어주는 셈이라 의미가 없다).
                m0609_robot.apply_action(ArticulationAction(joint_positions=last_replay_q))
            else:
                sync_rmp_base()
                actions = controller.forward(
                    target_end_effector_position=ee_target_pos, target_end_effector_orientation=ee_orientation,
                )
                m0609_robot.apply_action(actions)
            if hold_gripper_closed:
                m0609_robot.gripper.close()
            set_lift_height(lift_state["h"])
            world.step(render=True)

    final_pos, _ = base_robot.get_world_pose()
    ee_pos, _ = m0609_robot.end_effector.get_world_pose()
    ee_err = float(np.linalg.norm(np.array(ee_pos) - ee_target_pos))
    print(f"[후진+상승 완료]{' ' + label if label else ''} {step}스텝, 섀시x={float(final_pos[0]):.3f} "
          f"리프트={lift_state['h']:.3f} ee={np.round(ee_pos, 3)} err={ee_err:.4f}m "
          f"조건충족={condition_met} 자세붕괴={aborted}", flush=True)
    return final_pos, ee_pos, ee_err, condition_met, aborted


def drive_and_reach(target_x, target_y, ee_target_pos, ee_orientation=DOWN_QUAT,
                     tolerance_xy=0.03, max_speed=0.4, kp_xy=1.8, max_steps=3000,
                     hold_gripper_closed=True, label="",
                     condition_fn=None, abort_fn=None, hard_stop_on_condition=False, max_speed_fn=None,
                     debug_interval=0, debug_fn=None):
    """홀로노믹 베이스 전진과 매니퓰레이터 목표 추종을 같은 스텝에서 동시에 진행한다(사용자
    설계). 원래 drive_to()는 주행 중 step_hold(1)만 불러서 팔에 아무 명령도 안 보냈다 - 리프트
    텔레포트(set_lift_height)가 매 프레임 팔 전체를 섀시 기준으로 재배치하므로, 직전에
    move_link6()로 잡아둔 조인트값이 그대로 얼어붙은 채(자체적으로 안 움직이는데) 섀시가
    전진하는 대로 그냥 끌려갔다 - 이미 릴리즈 높이(범퍼 높이대)까지 뻗어있던 팔이 실시간 보정
    없이 차체 쪽으로 그대로 밀려들어가 충돌했다(사용자가 GUI로 직접 확인).
    고침: 매 스텝 (a) 바퀴 속도 명령과 (b) RMPflow 목표 추종 명령을 함께 내린다. ee_target_pos는
    고정된 world 목표(트렁크 안 최종 배치 지점)라서, 섀시가 X축으로 다가갈수록 팔이 그 목표에
    필요한 수평 reach를 실시간으로 줄여가며 자연스럽게 따라온다 - "Z축 정렬 후 하강"이 아니라
    "X축 기준으로 옆에서 안쪽으로 밀어넣는" 동작이 여기서 나온다.

    사용자 지적(STAGE 2 충돌 분석 이후 STAGE 3에도 동일 적용) - drive_until()에 추가했던
    안전장치(자세 붕괴 감지/즉시 정지/근접 시 저속화)가 여기엔 없었다. STAGE 3은 팔이
    "얼어붙은" 게 아니라 능동적으로 추종하므로 drive_until의 "기준 오프셋 대비 편차" 감지는
    그대로 못 쓰지만(추종 중엔 원래도 오차가 있으므로), abort_fn/hard_stop_on_condition/
    max_speed_fn을 동일한 인터페이스로 지원해서 호출부가 이 상황에 맞는 감지 로직(예: 박스
    Y 이탈, 그리퍼 이탈)을 넣을 수 있게 한다.

    사용자 실측 확인(STAGE 3.3) - 정지 조건이 "섀시가 자기 목표에 도달했는가"뿐이었다.
    섀시-박스 오프셋을 기준으로 섀시 목표를 잡다 보니, 팔이 능동 추종으로 박스를 이미
    목표에 딱 붙여놨는데도 섀시가 (팔보다 훨씬 느리게 움직여서) 자기 목표에 도달할 때까지
    한참을 더 크리핑했다("타겟에 다 왔는데도 섀시가 더 움직인다"는 관찰과 일치). drive_until()
    처럼 condition_fn(예: 박스가 실제로 목표에 도달했는지)을 추가해서, 섀시가 자기 목표에
    못 미쳤어도 실제 목표(박스 위치)가 달성됐으면 조기 정지할 수 있게 한다.

    사용자 실측 확인(2차, 카트 박스 2개 통합 검증 중 STAGE 3.3에서 재현) - ee_target_pos를
    step 1부터 고정된 "최종" 목표로 그대로 줬더니, 섀시는 kp_xy*오차를 max_speed(0.06)로
    클리핑해서 천천히 따라가는데 RMPflow는 그 큰 오차를 한 번에 풀려다 중간에 자세가
    튀었다(박스가 목표의 반대쪽까지 오버슈트 - reach_with_lift를 고칠 때 이미 확인된
    "큰 목표를 한 번에 주면 RMPflow가 이상한 해로 튄다"는 것과 동일한 패턴).

    사용자 실측 확인(3차) - 처음엔 ee 목표를 "경과 스텝 수" 기준으로 선형 보간했는데(고정
    ee_ramp_steps), 그러면 섀시의 실제 진행 속도와 이 보간 속도가 서로 안 맞을 때(둘 다
    독립적으로 "목표에 다가가는" 두 스케줄이라 우연히 비슷한 속도가 아니면) 팔이 감당해야
    할 "남은 reach"가 줄어들다가 다시 늘어나는 비단조적인 움직임이 나올 수 있다(실측:
    박스 Y가 목표 반대 방향으로 갔다가 다시 넘어가는 S자 오버슈트, box_id=1에서 재현).
    고침: ee 목표를 시간이 아니라 "섀시가 실제로 얼마나 다가갔는지"(남은 거리 비율)에
    비례해서 보간한다 - 섀시가 실측으로 목표에 가까워진 만큼만 정확히 팔 목표도 같이
    다가가므로, "팔의 남은 reach"가 항상 단조 감소한다(늘어나는 구간이 원천적으로 없음)."""
    ee_target_pos = np.array(ee_target_pos, dtype=float)
    start_pos, start_quat = base_robot.get_world_pose()
    tx = target_x if target_x is not None else float(start_pos[0])
    ty = target_y if target_y is not None else float(start_pos[1])
    start_ee_pos, _ = m0609_robot.end_effector.get_world_pose()
    start_ee_pos = np.array(start_ee_pos, dtype=float)
    _chassis_start_xy = np.array([float(start_pos[0]), float(start_pos[1])])
    _chassis_target_xy = np.array([tx, ty])
    _initial_chassis_dist = float(np.linalg.norm(_chassis_target_xy - _chassis_start_xy))
    print(f"\n[주행+추종 시작]{' ' + label if label else ''} 섀시목표=({tx:.3f},{ty:.3f}) "
          f"팔목표={np.round(ee_target_pos, 3)}", flush=True)

    STALL_WINDOW, STALL_MIN_PROGRESS = 150, 0.008
    last_check_pos = np.array([float(start_pos[0]), float(start_pos[1])])
    stalled = False
    aborted = False
    condition_met = False
    freeze_q = None
    step = 0
    for step in range(1, max_steps + 1):
        if debug_interval and debug_fn is not None and step % debug_interval == 0:
            debug_fn(step)
        if condition_fn is not None and condition_fn():
            condition_met = True
            print(f"  [조건 충족] {step}스텝에서 condition_fn() True - 섀시 목표 미달이어도 조기 정지",
                  flush=True)
            if debug_fn is not None:
                debug_fn(step)
            break
        if abort_fn is not None and abort_fn():
            aborted = True
            freeze_q = np.asarray(m0609_robot.get_joint_positions(), dtype=float).copy()
            print(f"  [자세 붕괴 감지] {step}스텝에서 abort_fn() True - 주행 즉시 중단(실패)", flush=True)
            if debug_fn is not None:
                debug_fn(step)
            break
        pos, quat = base_robot.get_world_pose()
        yaw_deg = float(np.degrees(quat_to_euler_angles(quat)[2]))
        ex_w, ey_w = tx - float(pos[0]), ty - float(pos[1])
        if abs(ex_w) < tolerance_xy and abs(ey_w) < tolerance_xy:
            break
        step_max_speed = max_speed_fn() if max_speed_fn is not None else max_speed
        yaw_rad = np.radians(yaw_deg)
        ex_l = ex_w * np.cos(yaw_rad) + ey_w * np.sin(yaw_rad)
        ey_l = -ex_w * np.sin(yaw_rad) + ey_w * np.cos(yaw_rad)
        vx_t = float(np.clip(kp_xy * ex_l, -step_max_speed, step_max_speed))
        vy_t = float(np.clip(kp_xy * ey_l, -step_max_speed, step_max_speed))
        _smooth_state["vx"] += SMOOTH_ALPHA * (vx_t - _smooth_state["vx"])
        _smooth_state["vy"] += SMOOTH_ALPHA * (vy_t - _smooth_state["vy"])
        _smooth_state["wz"] *= (1 - SMOOTH_ALPHA)  # 회전 없음 - yaw는 그대로 유지
        base_robot.apply_action(holo_forward(_smooth_state["vx"], _smooth_state["vy"], _smooth_state["wz"]))

        # 섀시 주행과 완전히 같은 프레임에서 팔도 매 스텝 목표를 추종한다 - ee 목표는
        # "섀시가 실측으로 얼마나 다가갔는지"(남은 거리 비율)에 비례해서 시작점->최종
        # 목표로 보간한다(시간 기준 보간이 아님 - 함수 docstring 3차 수정 참고). 섀시가
        # 멈춰있으면(예: 이미 도착) 팔 목표도 그대로 안 움직인다.
        _remaining_dist = float(np.hypot(ex_w, ey_w))
        if _initial_chassis_dist > 1e-6:
            ee_alpha = 1.0 - min(1.0, _remaining_dist / _initial_chassis_dist)
        else:
            ee_alpha = 1.0
        ee_pt = start_ee_pos + (ee_target_pos - start_ee_pos) * ee_alpha
        sync_rmp_base()
        actions = controller.forward(
            target_end_effector_position=ee_pt,
            target_end_effector_orientation=ee_orientation,
        )
        m0609_robot.apply_action(actions)
        if hold_gripper_closed:
            m0609_robot.gripper.close()
        set_lift_height(lift_state["h"])
        world.step(render=True)

        if step % STALL_WINDOW == 0:
            cur = np.array([float(pos[0]), float(pos[1])])
            progress = float(np.linalg.norm(cur - last_check_pos))
            if progress < STALL_MIN_PROGRESS and (abs(ex_w) > tolerance_xy or abs(ey_w) > tolerance_xy):
                stalled = True
                print(f"  [정체 감지] {progress:.4f}m밖에 못 움직임 - 중단", flush=True)
                if debug_fn is not None:
                    debug_fn(step)
                break
            last_check_pos = cur

    if hard_stop_on_condition and aborted:
        # 사용자 지적(STAGE 3.2 Phase B 실측, reach_with_lift에서 먼저 발견/수정) - 이미
        # 자세 붕괴(충돌 의심)가 감지된 상황에서도 계속 ee_target_pos를 추종시키면, 이미
        # 위험하다고 판정된 목표를 향해 계속 풀려다 상황이 더 나빠질 수 있다(STAGE 2/Tilt
        # Phase3의 "얼려서 정지" 패턴과 반대). 감지 시점의 관절값을 그대로 얼려서 유지한다.
        _smooth_state["vx"] = 0.0
        _smooth_state["vy"] = 0.0
        _smooth_state["wz"] = 0.0
        zero_action = holo_forward(0.0, 0.0, 0.0)
        for _ in range(8):
            base_robot.apply_action(zero_action)
            m0609_robot.apply_action(ArticulationAction(joint_positions=freeze_q))
            if hold_gripper_closed:
                m0609_robot.gripper.close()
            set_lift_height(lift_state["h"])
            world.step(render=True)
    else:
        for _ in range(30):
            _smooth_state["vx"] *= 1 - SMOOTH_ALPHA
            _smooth_state["vy"] *= 1 - SMOOTH_ALPHA
            base_robot.apply_action(holo_forward(_smooth_state["vx"], _smooth_state["vy"], _smooth_state["wz"]))
            sync_rmp_base()
            actions = controller.forward(
                target_end_effector_position=ee_target_pos, target_end_effector_orientation=ee_orientation,
            )
            m0609_robot.apply_action(actions)
            if hold_gripper_closed:
                m0609_robot.gripper.close()
            set_lift_height(lift_state["h"])
            world.step(render=True)
    final_pos, final_quat = base_robot.get_world_pose()
    ee_pos, _ = m0609_robot.end_effector.get_world_pose()
    ee_err = float(np.linalg.norm(np.array(ee_pos) - ee_target_pos))
    print(f"[주행+추종 완료]{' ' + label if label else ''} {step}스텝, 섀시=({float(final_pos[0]):.3f},"
          f"{float(final_pos[1]):.3f}) 팔ee={np.round(ee_pos, 3)} ee_err={ee_err:.4f}m "
          f"조건충족={condition_met} 자세붕괴={aborted} 정체={stalled}", flush=True)
    return final_pos, ee_pos, ee_err, not stalled, aborted


viewport = vp_util.get_active_viewport()


def snapshot(eye, target, fname):
    set_camera_view(eye=eye, target=target)
    step_hold(15)
    out = str(OUT_DIR / fname)
    vp_util.capture_viewport_to_file(viewport, out)
    step_hold(30)
    print(f"[SCREENSHOT] {out}", flush=True)


def pause_for_inspection(message):
    """사용자 요청 - 충돌 의심으로 중단됐을 때 raise SystemExit로 곧장 스크립트를 끝내버리면
    Isaac Sim GUI도 같이 닫혀서 실제로 어디가 부딪혔는지 직접 눈으로 확인할 방법이 없다.
    대신 그 순간의 자세 그대로 시뮬레이션만 계속 돌리고(world.step 반복) 로봇에는 아무 새
    명령도 보내지 않는다 - GUI 카메라를 자유롭게 돌려가며 충돌 지점을 확인한 뒤, 다 봤으면
    터미널에서 Ctrl+C로 직접 스크립트를 끝내면 된다."""
    print(f"\n{message}", flush=True)
    print("[일시정지] 마지막 자세를 그대로 유지한 채 멈췄습니다 - Isaac Sim GUI에서 카메라를 "
          "돌려 충돌/문제 지점을 확인하세요. 다 확인했으면 터미널에서 Ctrl+C로 종료하세요.\n",
          flush=True)
    while True:
        world.step(render=True)



# ---- isaac_task_runner 추가: 89/99.py가 쓰는 카메라 관련 임포트(100.py엔 원래 없음) ----
import omni.graph.core as og
import omni.kit.viewport.utility as vp_util  # noqa: F811 (100.py에 이미 있으면 재바인딩, 없으면 새로 추가)
from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats
from isaacsim.sensors.camera import Camera
import matplotlib.pyplot as plt  # noqa: F401 - 89.py의 스윕 디버그 이미지 저장(plt.imsave)용

# ---- isaac_task_runner 추가: 89/99.py 공통 카메라 유틸(100.py엔 원래 없음) ----
WORLD_UP = (0.0, 0.0, 1.0)
def find_camera_prim_path(stage, root_path, name_hint):
    """32.py와 동일 - M0609 서브트리에서 카메라 프림을 이름으로 찾는다(경로 하드코딩 회피)."""
    root_prim = stage.GetPrimAtPath(root_path)
    candidates = []
    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdGeom.Camera):
            candidates.append(str(prim.GetPath()))
    for c in candidates:
        if name_hint.lower() in c.lower():
            return c, candidates
    return (candidates[0] if candidates else None), candidates

def _normalize(v, eps=1e-9):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < eps:
        raise ValueError(f"영벡터는 방향으로 사용할 수 없습니다: {v}")
    return v / n

def make_usd_camera_rotation(eye, look_at, up_ref=WORLD_UP):
    """USD Camera 축(+Y up, -Z forward)에 맞는 world rotation matrix (12/32.py와 동일)."""
    eye = np.asarray(eye, dtype=float)
    look_at = np.asarray(look_at, dtype=float)
    forward = _normalize(look_at - eye)
    up_ref = _normalize(up_ref)

    if abs(float(np.dot(forward, up_ref))) > 0.97:
        alt = np.array([0.0, 1.0, 0.0])
        if abs(float(np.dot(forward, alt))) > 0.97:
            alt = np.array([1.0, 0.0, 0.0])
        up_ref = alt

    right = _normalize(np.cross(forward, up_ref))
    backward = -forward
    camera_up = _normalize(np.cross(backward, right))
    R_cam_target = np.column_stack((right, camera_up, backward))

    det = float(np.linalg.det(R_cam_target))
    if det < 0.99:
        raise RuntimeError(f"카메라 회전행렬이 우수좌표계가 아닙니다. det={det:.6f}")
    return R_cam_target


def return_to_spawn_pose():
    """모든 태스크(run_cart_scan/run_trunk_scan/run_pick_and_place) 종료 시 공통으로
    호출 - 사용자 지시: 각 태스크가 서로 다른 위치/자세에서 끝나면 다음 태스크가
    가정하는 시작 상태와 어긋나서 카트/트렁크와 부딪히거나 엉뚱하게 주행할 위험이
    있다(실측: cart_scan 직후 곧장 trunk_scan을 실행했을 때 이 복귀가 없어서
    문제가 생김). capture_cart_scan.py가 이미 검증한 것과 동일한 패턴 - 팔을
    _init_joints([0,0,90,0,90,0]) 접힘 자세로 되돌리고 리프트를 LIFT_MIN까지
    내린 뒤, 섀시를 CART_CLEAR_X 안전지대를 거쳐(1단계: X만 이동, 2단계: 그
    안전지대에서 Y/yaw를 스폰값으로) CHASSIS_SPAWN_XY/BASE_FACE_ROT_Z로 복귀한다."""
    raise_lift_and_fold(LIFT_MIN, _init_joints, steps=250)
    _cur_pos, _ = base_robot.get_world_pose()
    drive_to(target_x=CART_CLEAR_X, target_y=float(_cur_pos[1]), target_yaw_deg=0.0,
             label="스폰 복귀 1/2: X만 안전지대로")
    drive_to(target_x=CART_CLEAR_X, target_y=float(cart_center_xy[1]), target_yaw_deg=BASE_FACE_ROT_Z,
             label="스폰 복귀 2/2: Y/yaw를 스폰값으로")
    print("[isaac_task_runner] 스폰 위치/접힘 자세로 복귀 완료", flush=True)


# ---- isaac_task_runner 추가: MSI1(UI)/Action 서버가 진행 상황을 구독할 수 있게
# /isaac_task_runner/status에 상태를 publish하는 헬퍼. _STATUS_PUB은 아래
# dispatch 섹션(파일 맨 아래, rclpy 노드가 만들어진 뒤)에서만 실제 값이
# 채워진다 - 이 함수 자체는 파일 앞쪽(run_pick_and_place 등)에서 정의되지만,
# 실제로 "호출"되는 시점은 항상 dispatch 루프가 노드를 만든 뒤이므로(Python은
# 전역 이름을 def 시점이 아니라 call 시점에 찾는다) 문제없다 - WORLD_UP 등
# 이 파일의 다른 헤더 폴백값과 같은 원리.
_STATUS_PUB = None


def _publish_status(task, stage, box_index=0, box_count=0, box_id="", message=""):
    print(f"[isaac_task_runner:status] task={task} stage={stage} "
          f"box={box_index}/{box_count} box_id={box_id} {message}", flush=True)
    if _STATUS_PUB is not None:
        import json as _json_for_status
        from std_msgs.msg import String as _StringForStatus
        _msg = _StringForStatus()
        _msg.data = _json_for_status.dumps({
            "task": task, "stage": stage, "box_index": box_index,
            "box_count": box_count, "box_id": box_id, "message": message,
        })
        _STATUS_PUB.publish(_msg)


def run_pick_and_place():
    """100.py STAGE0~4 본문(pick_order 매칭 + 박스 루프) 그대로 - 코드 변경 없이 함수로만 감쌈."""
    global scan_box_top, ENTRY_HOLDING_Z, HOLDING_Z, TEST_BOX_SIZE, place_release_z
    global placement_data, placements, trunk_meta, SCAN_BASE_POS, SCAN_BASE_QUAT, SCAN_R_BASE
    global trunk_map, CEILING_WORLD_Z, SAFE_TRANSIT_Z, CARRY_CLEARANCE_ABOVE_RELEASE, ENTRY_CLEARANCE_ABOVE_RELEASE
    global PLACE_LIFT_MAX
    _publish_status("pick_and_place", "started")
    # ================= placement_result.json + trunk 좌표계 로드 =================
    # 사용자 지시("카트에 있는 박스 2개 전부다 성공하도록") - placements가 이제 여러 개일 수
    # 있다. 92번은 첫 박스(placements[0])만 보고 place_release_z/HOLDING_Z/ENTRY_HOLDING_Z를
    # 로봇 스폰보다 먼저 한 번만 계산했는데, 그 값들은 STAGE 1 이후(로봇 스폰 한참 뒤)에나
    # 쓰이므로 로봇 스폰 전에 계산해야 할 이유가 실제로는 없다(module 스코프 검증 완료) -
    # compute_place_targets()로 함수화해서 박스마다(아래 PICK 루프에서) 다시 부를 수 있게 한다.
    placement_data = json.loads(PLACEMENT_JSON.read_text())
    placements = placement_data["placements"]
    if not placements:
        raise SystemExit("[에러] placement_result.json에 배치된 박스가 없습니다.")
    print(f"[적재 계획] {len(placements)}개 박스 배치 예정", flush=True)
    for _p in placements:
        print(f"  box_id={_p['box_id']} position_base_frame={_p['position_base_frame']} "
              f"dimensions={_p['dimensions']} rotated={_p.get('rotated')}", flush=True)

    trunk_meta = json.loads(TRUNK_META_JSON.read_text())
    SCAN_BASE_POS = np.asarray(trunk_meta["base_pos"], dtype=np.float64)
    SCAN_BASE_QUAT = np.asarray(trunk_meta["base_quat"], dtype=np.float64)
    SCAN_R_BASE = quat_wxyz_to_matrix(SCAN_BASE_QUAT)

    # 천장 한계(CEILING_WORLD_Z/SAFE_TRANSIT_Z)는 트렁크 자체의 실측값이라 박스와 무관 -
    # 박스 루프 밖에서 한 번만 계산한다.
    trunk_map = json.loads(TRUNK_MAP_JSON.read_text()) if TRUNK_MAP_JSON.exists() else None
    if trunk_map is not None:
        ceiling_z_base = max(v[2] for v in trunk_map["vertices"][4:8])
        CEILING_WORLD_Z = float((SCAN_R_BASE @ np.array([0.0, 0.0, ceiling_z_base]) + SCAN_BASE_POS)[2])
        print(f"[트렁크맵] ceiling_z(world)={CEILING_WORLD_Z:.3f}", flush=True)
    else:
        CEILING_WORLD_Z = TRUNK_WALL_TOP
        print(f"[경고] {TRUNK_MAP_JSON} 없음 - TRUNK_WALL_TOP({TRUNK_WALL_TOP})을 천장 한계로 사용", flush=True)
    SAFE_TRANSIT_Z = CEILING_WORLD_Z - 0.05

    CARRY_CLEARANCE_ABOVE_RELEASE = 0.05  # 홀로노믹 접근 중 바닥에 안 끌리게 release_z보다 이만큼 위에서 든다.
    ENTRY_CLEARANCE_ABOVE_RELEASE = 0.25  # 입구 턱을 넘기기 위한 release_z 대비 추가 상승분.
    PLACE_LIFT_MAX = min(0.65, SAFE_TRANSIT_Z - 0.05)
    print(f"[PLACE 하강용 리프트 상한] PLACE_LIFT_MAX={PLACE_LIFT_MAX:.3f} "
          f"(천장한계 {SAFE_TRANSIT_Z:.3f} 이내로 클램프)", flush=True)
    print(f"\n[STAGE] {STAGE}단계까지 진행합니다 "
          "(0=PICK 0.5=운송자세+리프트하강 0.8=트렁크standoff까지 주행 "
          "1=홀딩자세 1.1=입구턱클리어 2=근접이동 3=정밀접근/STAGE3.0까지)", flush=True)

    chassis_pos_pick0, _ = base_robot.get_world_pose()
    snapshot(eye=[chassis_pos_pick0[0] - 1.0, chassis_pos_pick0[1] - 1.3, 1.4],
             target=[cart_center_xy[0], cart_center_xy[1], 0.7], fname="_cart2trunk_00_start.png")

    # ================= STAGE 0: 카트에서 박스 PICK (91.cart_pick_holonomic.py와 동일 로직) =================
    # 91번과 다른 점 - 집은 뒤 바로 놓지 않고 계속 붙잡는다(pick_order의 첫 박스 하나만 취급 -
    # 트렁크에는 어차피 한 번에 하나씩만 넣는다).
    CANDIDATE_BOX_PRIM_PATHS = discover_box_prim_paths(stage)
    print(f"[박스 프림 탐색] {CANDIDATE_BOX_PRIM_PATHS}", flush=True)

    placement_data_pick = json.loads(PLACEMENT_JSON.read_text())
    placements_pick = placement_data_pick["placements"]

    pick_transform_data = json.loads(BASE_TO_CAMERA_TRANSFORM_JSON.read_text())
    PICK_SCAN_BASE_POS = np.asarray(pick_transform_data["measured_base_pos"], dtype=np.float64)
    PICK_SCAN_BASE_QUAT = np.asarray(pick_transform_data["measured_base_quat"], dtype=np.float64)
    PICK_SCAN_R_BASE = quat_wxyz_to_matrix(PICK_SCAN_BASE_QUAT)

    _vision_dir = Path.home() / "box_pointcloud"
    _vision_files = sorted(_vision_dir.glob("all_boxes_corners_*.json"))
    if not _vision_files:
        raise SystemExit(f"[에러] {_vision_dir}에 all_boxes_corners_*.json이 없습니다.")
    vision_data = json.loads(_vision_files[-1].read_text())
    scan_by_box_id = {str(b["box_id"]): b for b in vision_data["boxes"]}
    print(f"[비전 로드] {_vision_files[-1].name} - box_id={list(scan_by_box_id.keys())}", flush=True)

    used_prim_paths = set()
    pick_order = []  # [(prim_path, placement_dict)]
    scan_box_top = {}  # prim_path -> (world_x, world_y, world_top_z)
    for placement in placements_pick:
        box_id = str(placement["box_id"])
        scan_entry = scan_by_box_id.get(box_id)
        if scan_entry is None:
            print(f"[경고] box_id={box_id}가 비전 결과에 없음 - 건너뜀", flush=True)
            continue
        scan_center, scan_size, scan_min = world_aabb_from_base_corners(
            scan_entry["corners_m"], PICK_SCAN_BASE_POS, PICK_SCAN_R_BASE)
        available = [p for p in CANDIDATE_BOX_PRIM_PATHS if p not in used_prim_paths]
        prim_path, match_dist = match_physical_prim(stage, scan_center, available)
        if prim_path is None:
            continue
        used_prim_paths.add(prim_path)
        pick_order.append((prim_path, placement))
        scan_box_top[prim_path] = (
            float(scan_center[0]), float(scan_center[1]), float(scan_min[2] + scan_size[2]))
        print(f"[매칭] box_id={box_id} -> {prim_path} (거리={match_dist:.3f}m)", flush=True)

    if not pick_order:
        raise SystemExit("[에러] 비전 결과와 매칭되는 물리 박스가 없습니다.")

    # 사용자 지시("카트에 있는 박스 2개 전부다 성공하도록") - pick_order의 모든 박스를
    # 순서대로 (카트 접근 계획 수립 -> 카트 standoff 주행 ->) PICK -> 운송 -> 주행 ->
    # STAGE1~4(배치+후퇴)까지 반복한다.
    for _box_num, (picked_prim_path, picked_placement) in enumerate(pick_order):
        _publish_status("pick_and_place", "box_started", box_index=_box_num, box_count=len(pick_order), box_id=str(picked_placement['box_id']))
        print(f"\n########## 박스 {_box_num + 1}/{len(pick_order)} 시작: {picked_prim_path} "
              f"(box_id={picked_placement['box_id']}) ##########\n", flush=True)
        compute_place_targets(picked_placement)

        # ================= 사용자 설계 - 이 박스의 solution space + 카트 접근 방향 계획 =================
        # 94번은 "카트 접근 위치/각도는 항상 고정, 조인트만 나중에(카트 복귀 도중) 반전"하는
        # 절충이었고, 그 결과 반전된 팔이 카트 손잡이와 부딪히는 문제가 있었다(사용자 실측
        # 확인). 100번은 반대로, 이 박스가 필요로 하는 solution space(트렁크 배치 위치가
        # 결정)와 카트 접근 방향(박스의 A/B단 치우침과 XOR로 결정)을 여기서 한 번에 계산해서,
        # PICK 자체를 처음부터 "손잡이와 부딪히지 않는 쪽"에서 "요구되는 solution space
        # 그대로" 시작한다 - PICK 도중 조인트를 반전시킨 채 손잡이를 스치는 상황 자체가
        # 구조적으로 없어진다.
        _need_space2 = box_place_needs_mirrored_pick(picked_placement)
        _pick_fold = _fold_target_mirrored if _need_space2 else _fold_target
        # 트렁크 standoff에서 이 yaw여야 한다(STAGE0.8과 STAGE1.9 양쪽에서 재사용) - 94번과
        # 동일한 근거: 조인트3/5를 반전해서 픽/운송한 박스는 팔이 반대쪽 solution branch에
        # 붙어있으므로, 트렁크 standoff에서도 섀시를 180도 반대로 세워야 그 branch에 맞는
        # 자세가 나온다.
        _trunk_approach_yaw_deg = 180.0 if _need_space2 else 0.0
        _box_near_cart_A_end = box_leans_toward_cart_A_end(picked_prim_path)
        _approach_cart_left = approach_cart_left_side(_box_near_cart_A_end, not _need_space2)
        _cart_target_xy = CART_BASE_LEFT_XY if _approach_cart_left else CART_BASE_RIGHT_XY
        _cart_approach_yaw_deg = 180.0 if _approach_cart_left else 0.0
        print(f"[카트 접근 계획] box_id={picked_placement['box_id']} "
              f"solution_space={'2(반전)' if _need_space2 else '1(표준)'} "
              f"박스치우침={'A단(손잡이쪽)' if _box_near_cart_A_end else 'B단(입구쪽)'} "
              f"-> 카트 {'왼쪽' if _approach_cart_left else '오른쪽'} 접근 "
              f"목표=({_cart_target_xy[0]:.3f},{_cart_target_xy[1]:.3f}) "
              f"yaw={_cart_approach_yaw_deg:.0f}도", flush=True)

        # ================= 카트 standoff로 주행(팔은 표준 접힘+LIFT_MIN으로 고정) =================
        # 스폰 직후 부트스트랩(_init_joints/lift_state 초기값)이 이미 정확히 "표준 접힘 +
        # LIFT_MIN"이므로, 박스 1은 이 fold/lift 보간이 사실상 무동작이고 주행만 실제로
        # 움직인다 - 박스 2+는 직전 STAGE4가 남긴 HOLDING_Z 자세에서 실제로 접고 내려온다.
        # 박스1과 이후 박스를 특별 취급하지 않고 완전히 동일한 코드로 다룬다(94번엔 있었던
        # "박스1은 이미 서 있다"는 비대칭이 사라진다).
        raise_lift_and_fold(lift_state["h"], _fold_target, steps=200)
        raise_lift_and_fold(LIFT_MIN, _fold_target, steps=250)
        base_robot.apply_action(holo_forward(0.0, 0.0, 0.0))
        base_robot.set_linear_velocity(np.zeros(3))
        base_robot.set_angular_velocity(np.zeros(3))

        # 사용자 실측 확인("생성하자마자 충돌이 나서 터졌어") - CART_CLEAR_X 정의부에서 분석한
        # 대로, yaw=90도(스폰 시)든 회전 도중의 중간 각도든 카트와 겹치지 않으려면 "카트와의
        # X축 분리"(CART_CLEAR_X)만으로 안전을 확보한 채로 회전을 마치고, Y/yaw가 이미 최종
        # 목표값이 된 뒤에만(이제부터는 "카트와의 Y축 분리"로 안전 확보) X를 좁혀 최종
        # standoff까지 접근한다 - 3단계로 나눈다(각 단계는 팔을 얼린 채 진행, hold_q/abort
        # 기준은 이 구간 내내 팔이 전혀 안 움직이므로 한 번만 계산해서 3단계 모두 재사용).
        _cartgo_hold_q = np.asarray(m0609_robot.get_joint_positions(), dtype=float).copy()
        _cartgo_chassis_start, _cartgo_chassis_quat_start = base_robot.get_world_pose()
        _cartgo_R_start = quat_wxyz_to_matrix(np.asarray(_cartgo_chassis_quat_start, dtype=float))
        _cartgo_tip_start = measure_tip_world_pos()
        _cartgo_tip_rel_local_ref = _cartgo_R_start.T @ (
            _cartgo_tip_start - np.asarray(_cartgo_chassis_start, dtype=float))

        def _hold_cartgo_arm():
            m0609_robot.apply_action(ArticulationAction(joint_positions=_cartgo_hold_q))
            m0609_robot.gripper.close()

        def _cartgo_broken():
            chassis_pos, chassis_quat = base_robot.get_world_pose()
            R_now = quat_wxyz_to_matrix(np.asarray(chassis_quat, dtype=float))
            tip_pos = measure_tip_world_pos()
            tip_rel_local = R_now.T @ (tip_pos - np.asarray(chassis_pos, dtype=float))
            relative_error = float(np.linalg.norm(tip_rel_local - _cartgo_tip_rel_local_ref))
            return relative_error > 0.025

        def _drive_cartgo_leg(target_x, target_y, target_yaw, label):
            _, _, _, _aborted = drive_until(
                lambda: False, target_x=target_x, target_y=target_y, target_yaw_deg=target_yaw,
                max_speed=0.15, per_step_fn=_hold_cartgo_arm, abort_fn=_cartgo_broken,
                hard_stop_on_condition=True, label=label,
            )
            if _aborted:
                pause_for_inspection(f"[중단] 카트 접근({label}) 도중 자세 붕괴가 감지돼 즉시 중단했습니다.")
            base_robot.apply_action(holo_forward(0.0, 0.0, 0.0))
            base_robot.set_linear_velocity(np.zeros(3))
            base_robot.set_angular_velocity(np.zeros(3))

        _cartgo_start_pos, _cartgo_start_quat = base_robot.get_world_pose()
        _cartgo_start_yaw = float(np.degrees(quat_to_euler_angles(_cartgo_start_quat)[2]))
        _side_label = '왼쪽' if _approach_cart_left else '오른쪽'

        # 1단계 - 회전 안전지대(X=CART_CLEAR_X)로 이동. Y/yaw는 지금 값 그대로 유지(회전 없음) -
        # 카트와 X축만으로 분리되므로 지금 yaw가 무엇이든(스폰 직후의 90도 포함) 안전하다.
        _drive_cartgo_leg(CART_CLEAR_X, float(_cartgo_start_pos[1]), _cartgo_start_yaw,
                          f"카트 접근 1/3: {'스폰 위치' if _box_num == 0 else '트렁크 standoff'} -> "
                          f"회전 안전지대(팔 자세 고정, X만 이동)")

        # 2단계 - 안전지대(X 고정)에서 목표 Y + 목표 yaw로 회전+횡이동. X가 카트와 계속
        # 분리돼 있으므로 회전 도중의 중간 각도도 안전하다.
        _drive_cartgo_leg(CART_CLEAR_X, _cart_target_xy[1], _cart_approach_yaw_deg,
                          f"카트 접근 2/3: 회전 안전지대에서 카트 {_side_label} 방향으로 회전+횡이동")

        # 3단계 - 이미 목표 yaw(도착 후 폭 축이 Y를 향함)+목표 Y인 채로 X만 좁혀 최종
        # standoff까지 직진 접근. Y가 내내 고정돼 카트와 Y축 분리가 유지되므로 안전하다 -
        # 사용자 지시("카트에 접근할 때는 홀로노믹 베이스의 옆면이 닿아있게") 그대로, 폭(짧은)
        # 축이 카트를 향한 채로 곧장 다가간다.
        _drive_cartgo_leg(_cart_target_xy[0], _cart_target_xy[1], _cart_approach_yaw_deg,
                          f"카트 접근 3/3: 회전 안전지대 -> 카트 {_side_label} standoff 최종 접근")
        print(f"[성공] 카트 {_side_label} standoff 도착.", flush=True)

        # 매 박스마다 카트 옆에서 리프트 재상승+접기(이번 박스가 필요로 하는 solution space
        # 그대로)+joint_1 재조준부터 다시 시작한다.
        pick_raise_and_aim(_pick_fold)

        box_dim_x, box_dim_y, box_dim_z = BOX_KNOWN_SIZE[picked_prim_path]
        half_height = float(box_dim_z) / 2.0
        horizontal_tolerance = float(max(box_dim_x, box_dim_y)) / 2.0 + GRASP_HORIZONTAL_MARGIN
        gripper.set_target(picked_prim_path, half_height, horizontal_tolerance, GRASP_VERTICAL_TOLERANCE)

        scan_top_x, scan_top_y, scan_top_z = scan_box_top[picked_prim_path]
        hover_z = max(scan_top_z + HOVER_ABOVE_BOX_TOP, float(cart_max[2]) + RIM_CLEARANCE)
        hover_target = np.array([scan_top_x, scan_top_y, hover_z])
        print(f"\n===== PICK 시작 {picked_prim_path} (스캔 박스상단={np.round(scan_box_top[picked_prim_path], 3)}) =====",
              flush=True)

        # [진단 - PICK 흡착 실패 원인 조사] scan_box_top은 99번 스캔 세션에서 base_to_
        # camera_transform.json으로 재투영한 "스캔 당시" 위치다. 100번은 카트/박스를
        # CART_BOX_SPECS로 이 세션에서 새로 떨어뜨리므로, 물리 낙하 결과(특히 Medium이
        # Large1 위에 새로 얹히게 된 뒤)가 스캔 시점과 미세하게 달라질 수 있다 - 그
        # 차이가 GRASP_STANDOFF/DESCENT_OVERTRAVEL 여유(각각 0.01m/0.06m)보다 크면
        # 흡착 시도가 박스 표면을 완전히 놓치거나 잘못된 위치에서 접촉한다. 흡착 실패
        # 시 바로 확인할 수 있도록, 흡착 시도 직전에 박스의 "지금 이 시뮬레이션의" 실제
        # world 위치/윗면 Z를 재서 스캔값과 함께 찍는다.
        _live_box_pos, _ = cart_box_objects[picked_prim_path].get_world_pose()
        _live_box_top_z = float(_live_box_pos[2]) + float(BOX_KNOWN_SIZE[picked_prim_path][2]) / 2.0
        _pos_delta = np.array([scan_top_x, scan_top_y, scan_top_z]) - np.array(
            [_live_box_pos[0], _live_box_pos[1], _live_box_top_z])
        print(f"[진단 스캔-실측 위치차] 실측 박스중심={np.round(_live_box_pos, 3)} 실측윗면Z={_live_box_top_z:.3f} "
              f"스캔값과의 차이(XYZ)={np.round(_pos_delta, 3)} (|Z차이|={abs(_pos_delta[2]):.3f}m, "
              f"GRASP_STANDOFF+DESCENT_OVERTRAVEL 여유={GRASP_STANDOFF + DESCENT_OVERTRAVEL:.3f}m)", flush=True)

        move_link6_smooth(hover_target, label="박스 위 호버")

        chassis_pos_pick1, _ = base_robot.get_world_pose()
        snapshot(eye=[chassis_pos_pick1[0] - 0.8, chassis_pos_pick1[1] - 1.0, hover_z + 0.3],
                 target=[scan_top_x, scan_top_y, scan_top_z], fname="_cart2trunk_01_hover.png")

        target_tip_z = scan_top_z + GRASP_STANDOFF
        descent_target = np.array([hover_target[0], hover_target[1], target_tip_z - DESCENT_OVERTRAVEL])
        move_link6_smooth(descent_target, max_speed=DESCENT_MAX_SPEED, try_grasp=True, label="하강")
        picked_grasped = bool(m0609_robot.gripper.is_closed())
        if not picked_grasped:
            pause_for_inspection(f"[중단] PICK 실패 - {picked_prim_path} 흡착이 안 됐습니다.")

        # [진단 - 2번째 박스 Y축 정렬 붕괴 조사, 사용자 지적] DOWN_QUAT=euler([0,pi,0])는
        # Y축 180도 회전만 지정하고 Z축(그리퍼 자기축 회전=joint_6과 직결)은 전혀 구속하지
        # 않는다 - "아래를 본다"는 조건은 만족해도 "어느 방향으로 돌아서 아래를 보는지"는
        # RMPflow가 시작 시드(=_pick_fold, solution space별로 다름)에 따라 알아서 정한다.
        # solution space 1/2가 이 남는 자유도(joint_6)를 다르게 수렴시키면, 흡착 FixedJoint가
        # "부착 순간의 상대 회전을 그대로 고정"하므로 박스가 매번 다른 각도로 붙들려서 이후
        # Y축 정렬이 흔들릴 수 있다(사용자 가설) - 정확한 오프셋을 추측 대신 실측으로 확인한다.
        if "joint_6" in m0609_robot.dof_names:
            _diag_joint6_rad = float(m0609_robot.get_joint_positions()[m0609_robot.dof_names.index("joint_6")])
        else:
            _diag_joint6_rad = float("nan")
        _diag_box_pos, _diag_box_quat = cart_box_objects[picked_prim_path].get_world_pose()
        _diag_box_yaw_deg = float(np.degrees(quat_to_euler_angles(np.asarray(_diag_box_quat, dtype=float))[2]))
        print(f"[진단 PICK 흡착 자세] solution_space={'2(반전)' if _need_space2 else '1(표준)'} "
              f"joint_6={np.degrees(_diag_joint6_rad):.1f}deg({_diag_joint6_rad:.4f}rad) "
              f"박스yaw={_diag_box_yaw_deg:.1f}deg 박스pos={np.round(np.asarray(_diag_box_pos, dtype=float), 3)}",
              flush=True)

        # 91번과 달리 여기서 gripper.open()을 부르지 않는다 - 계속 붙잡은 채로 다음 단계(안전 운송
        # 자세)로 넘어간다.
        move_link6_smooth(hover_target, hold_gripper_closed=True, label="파지 후 후퇴")
        print(f"[PICK 성공] {picked_prim_path} grasped={picked_grasped}", flush=True)

        chassis_pos_pick2, _ = base_robot.get_world_pose()
        snapshot(eye=[chassis_pos_pick2[0] - 0.8, chassis_pos_pick2[1] - 1.1, hover_z + 0.3],
                 target=[scan_top_x, scan_top_y, scan_top_z + 0.2], fname="_cart2trunk_02_picked.png")

        # 92번 STAGE1~3.0 코드가 그대로 test_box/TEST_BOX_SIZE를 참조하므로, 실제로 집은 박스를
        # 그 이름으로 alias한다(합성 스폰 없이 이 두 이름만 바꿔치기).
        test_box = cart_box_objects[picked_prim_path]
        TEST_BOX_SIZE = BOX_KNOWN_SIZE[picked_prim_path]

        # 사용자 지적 기반 진단 - place_release_z/HOLDING_Z/ENTRY_HOLDING_Z는 로봇 스폰보다도
        # 전에(모듈 맨 위에서) place_dims(비전이 잰 박스 두께)와 "ee가 박스 상단에 딱 닿아있다"는
        # 가정만으로 미리 계산해뒀다. 그런데 실제 PICK은 (a) 비전 dimensions(0.094m)와 실제
        # 스폰된 박스 두께(0.110m)가 다르고, (b) 원통형 흡착 판정이 수직 허용치(2cm) 안에서 붙기
        # 때문에 흡착판이 박스 상단에서 뜬 채로(실측 2cm) 그대로 고정된다 - 이 두 오차가 겹쳐서
        # "ee가 박스 상단에 닿아있다"는 가정이 실제로는 약 4cm 어긋난다(로그로 확인됨). 고침:
        # 가정 대신 지금 실제로 파지한 상태에서 "ee z - 박스 바닥 z"를 직접 측정해서, 그 실측값으로
        # 세 목표 높이를 다시 계산한다(박스가 기울어져도 맞도록 회전 투영으로 바닥 z를 구함).
        def _measure_ee_to_box_bottom_offset():
            ee_pos, _ = m0609_robot.end_effector.get_world_pose()
            box_pos, box_quat = test_box.get_world_pose()
            R = quat_wxyz_to_matrix(np.asarray(box_quat, dtype=float))
            half_dims = np.asarray(TEST_BOX_SIZE, dtype=float) / 2.0
            projected_half_z = (
                abs(R[2, 0]) * half_dims[0] + abs(R[2, 1]) * half_dims[1] + abs(R[2, 2]) * half_dims[2]
            )
            box_bottom_z = float(box_pos[2]) - projected_half_z
            return float(ee_pos[2]) - box_bottom_z


        EE_TO_BOX_BOTTOM_OFFSET = _measure_ee_to_box_bottom_offset()
        _old_place_release_z, _old_holding_z, _old_entry_holding_z = place_release_z, HOLDING_Z, ENTRY_HOLDING_Z
        place_release_z = float(PLACE_WORLD_MIN[2]) + RELEASE_CLEARANCE_ABOVE_FLOOR + EE_TO_BOX_BOTTOM_OFFSET
        HOLDING_Z = place_release_z + CARRY_CLEARANCE_ABOVE_RELEASE
        ENTRY_HOLDING_Z = min(
            float(PLACE_WORLD_MIN[2]) + RELEASE_CLEARANCE_ABOVE_FLOOR + ENTRY_CLEARANCE_ABOVE_RELEASE + EE_TO_BOX_BOTTOM_OFFSET,
            SAFE_TRANSIT_Z - 0.03,
        )
        print(f"[PICK 후 목표 높이 재계산] 실측 ee-박스바닥 오프셋={EE_TO_BOX_BOTTOM_OFFSET:.4f}m "
              f"place_release_z {_old_place_release_z:.3f}->{place_release_z:.3f} "
              f"HOLDING_Z {_old_holding_z:.3f}->{HOLDING_Z:.3f} "
              f"ENTRY_HOLDING_Z {_old_entry_holding_z:.3f}->{ENTRY_HOLDING_Z:.3f}", flush=True)

        if STAGE < 0.5:
            print("\n[STAGE 0 완료] 카트에서 박스를 집어 계속 붙잡고 있는지 스크린샷으로 확인하세요. "
                  "STAGE=0.5 이상으로 다시 실행하면 안전 운송 자세 확립까지 진행합니다.\n", flush=True)

        if STAGE >= 0.5:
            # ================= STAGE 0.5: 안전 운송 자세(조인트 3/5=90/90, 나머지 0) + 리프트 하강 =================
            # 사용자 지적 - HOLDING_Z(RMPflow ee 목표)는 92번이 트렁크 근처용으로 계산해둔 절대
            # world 높이라, 카트 옆(전혀 다른 좌표 영역)에서 그 목표로 move_link6를 부르면 RMPflow가
            # 큰 목표 오차를 풀려다 이상하게 움직인다(이 프로젝트에서 반복 확인된 "큰 목표를 한 번에
            # 주면 RMPflow가 이상한 해로 튄다"는 교훈과 동일). 고침: RMPflow ee 목표 대신, 이미
            # PICK 전에 안전하다고 검증된 "조인트 3/5=90/90(나머지 0)" 접기 자세(raise_lift_and_fold의
            # _fold_target과 동일, 단 이번 박스가 solution space 2라면 _pick_fold가
            # _fold_target_mirrored라 그대로 반영된다)로 되돌아간다 - 순수 관절 보간이라 RMPflow가
            # 관여하지 않고, 리프트도 안 건드리므로(target_h=현재값) 두 단계(접기/리프트하강)가
            # 서로 안 섞인다.
            raise_lift_and_fold(lift_state["h"], _pick_fold, steps=200)
            print(f"[STAGE0.5 접기 완료] 조인트={np.round(m0609_robot.get_joint_positions(), 3)} "
                  f"grasped={gripper.is_closed()}", flush=True)

            # 접은 자세(관절값)는 그대로 유지한 채 리프트만 LIFT_MIN(도킹 높이)까지 내린다 -
            # raise_lift_and_fold를 그대로 재사용(target_joints가 지금 값과 같아서 관절은 안 움직이고
            # 리프트만 보간된다).
            raise_lift_and_fold(LIFT_MIN, _pick_fold, steps=250)
            print(f"[STAGE0.5 완료] 리프트={lift_state['h']:.3f} grasped={gripper.is_closed()}", flush=True)

            chassis_pos_transit, _ = base_robot.get_world_pose()
            _ee_transit_now, _ = m0609_robot.end_effector.get_world_pose()
            snapshot(eye=[chassis_pos_transit[0] - 1.0, chassis_pos_transit[1] - 1.3, 1.2],
                     target=[float(_ee_transit_now[0]), float(_ee_transit_now[1]), float(_ee_transit_now[2])],
                     fname="_cart2trunk_03_transit_pose.png")

            if STAGE < 0.8:
                print("\n[STAGE 0.5 완료] 안전 운송 자세 + 리프트 하강까지 확인하세요. STAGE=0.8 이상으로 "
                      "다시 실행하면 트렁크 standoff까지 주행합니다.\n", flush=True)

        if STAGE >= 0.8:
            # ================= STAGE 0.8: 홀로노믹 주행+회전으로 카트 옆 -> 트렁크 standoff(BASE_START_XY) =================
            # 사용자 지적 - 섀시가 91번 픽 자세(BASE_FACE_ROT_Z=90도)로 카트 옆에 서 있는 채로 그냥
            # x/y만 이동하면, 트렁크에 도착해도 여전히 90도를 향한 채다 - 92번의 STAGE 1~3.0은
            # "긴 축이 트렁크를 정면으로 향한"(yaw=0도) 섀시를 전제로 튜닝됐으므로(좁은 입구 통과
            # 등), 그 자세 그대로 STAGE 1로 넘기면 92번이 검증한 것과 다른(옆으로 넓은) 프로파일로
            # 입구에 접근하게 된다. 고침: drive_until의 target_yaw_deg=0.0을 같이 줘서 주행하는
            # 동안 회전도 함께 마친다(홀로노믹이라 번역+회전 동시 가능, drive_to/drive_until이
            # 원래 지원하던 기능인데 이 프로젝트에서 실제로 0이 아닌 목표 yaw를 쓴 적은 처음이다).
            #
            # 회전이 새로 생기면서 - _transit_pose_broken()이 "팁-섀시 상대 위치"를 월드좌표 차이로
            # 비교하던 기존 STAGE2 방식(섀시가 회전 안 한다고 전제)을 그대로 쓰면, 섀시가 90도->0도로
            # 돌아가는 것 자체가 이 상대위치를 크게 바꿔버려서 실제 충돌이 없어도 즉시 "자세 붕괴"로
            # 오판한다. 고침: 팁-섀시 상대위치를 섀시의 그 순간 회전을 역으로 곱해 "섀시 로컬 좌표계"
            # 기준으로 재표현한 뒤 비교한다 - 순수 회전만으로는 이 값이 안 변하고, 조인트가 실제로
            # 밀리는(충돌) 경우에만 변한다.
            _transit_hold_q = np.asarray(m0609_robot.get_joint_positions(), dtype=float).copy()
            _transit_chassis_start, _transit_chassis_quat_start = base_robot.get_world_pose()
            _transit_R_start = quat_wxyz_to_matrix(np.asarray(_transit_chassis_quat_start, dtype=float))
            _transit_tip_start = measure_tip_world_pos()
            _transit_tip_rel_local_ref = _transit_R_start.T @ (
                _transit_tip_start - np.asarray(_transit_chassis_start, dtype=float))

            def _hold_transit_arm():
                m0609_robot.apply_action(ArticulationAction(joint_positions=_transit_hold_q))
                m0609_robot.gripper.close()

            def _transit_pose_broken():
                chassis_pos, chassis_quat = base_robot.get_world_pose()
                R_now = quat_wxyz_to_matrix(np.asarray(chassis_quat, dtype=float))
                tip_pos = measure_tip_world_pos()
                tip_rel_local = R_now.T @ (tip_pos - np.asarray(chassis_pos, dtype=float))
                relative_error = float(np.linalg.norm(tip_rel_local - _transit_tip_rel_local_ref))
                detached = not m0609_robot.gripper.is_closed()
                return relative_error > 0.025 or detached

            # 사용자 지적("이런거는 다 고려해줘야 하는거 아니야?") - 맞는 말이다. "카트 접근"
            # 3단계를 만들 때 카트로 다가가는 방향만 고치고, 카트에서 트렁크로 떠나는 이
            # STAGE0.8은 94번의 단일 drive_until을 그대로 복붙해서 정확히 같은 함정을
            # 다시 심어놨었다 - 지금 섀시는 카트 옆 standoff(X≈cart_center_x, 즉 카트 길이
            # 범위 한복판)에 서 있고, 목표 yaw가 현재값과 다르면(이번처럼 180->0) 카트 바로
            # 옆에서 그대로 회전해야 하는데, 회전 도중 섀시의 "긴" 길이 축이 카트 쪽(Y)을
            # 향하는 중간 각도를 반드시 거친다(CART_CLEAR_X 정의부의 축 반전 설명과
            # 완전히 동일한 원리) - 그 순간 카트와 겹쳐서 부딪힌다. "카트 접근"과 완전히
            # 대칭인 3단계(카트와 X축 분리를 먼저 확보 -> 그 상태에서만 회전 -> 이미 목표
            # yaw가 된 채로 최종 접근)로 고친다 - hold_q/abort 기준은 이 구간 내내 팔이
            # 전혀 안 움직이므로 그대로 재사용한다.
            def _drive_transit_leg(target_x, target_y, target_yaw, label):
                _, _, _, _aborted = drive_until(
                    lambda: False, target_x=target_x, target_y=target_y, target_yaw_deg=target_yaw,
                    max_speed=0.15, per_step_fn=_hold_transit_arm, abort_fn=_transit_pose_broken,
                    hard_stop_on_condition=True, label=label,
                )
                if _aborted:
                    pause_for_inspection(f"[중단] STAGE 0.8({label}) 도중 자세 붕괴/흡착 이탈이 감지돼 즉시 중단했습니다.")
                base_robot.apply_action(holo_forward(0.0, 0.0, 0.0))
                base_robot.set_linear_velocity(np.zeros(3))
                base_robot.set_angular_velocity(np.zeros(3))

            # 1단계 - 회전 없이 X만 안전지대로(카트와 X축 분리 확보, 지금 Y/yaw 유지).
            _drive_transit_leg(CART_CLEAR_X, float(_transit_chassis_start[1]),
                                float(np.degrees(quat_to_euler_angles(_transit_chassis_quat_start)[2])),
                                "STAGE0.8 1/3: 카트 standoff -> 회전 안전지대(X만 이동)")
            # 2단계 - 안전지대(X 고정)에서 목표 yaw로 회전 + Y를 트렁크 standoff Y로.
            _drive_transit_leg(CART_CLEAR_X, BASE_START_XY[1], _trunk_approach_yaw_deg,
                                f"STAGE0.8 2/3: 회전 안전지대에서 yaw->{_trunk_approach_yaw_deg:.0f}도 회전+횡이동")
            # 3단계 - 이미 목표 yaw/Y인 채로 X만 좁혀 트렁크 standoff까지 최종 접근.
            _drive_transit_leg(BASE_START_XY[0], BASE_START_XY[1], _trunk_approach_yaw_deg,
                                "STAGE0.8 3/3: 회전 안전지대 -> 트렁크 standoff 최종 접근")
            print(f"[성공] STAGE 0.8 - 트렁크 standoff(BASE_START_XY)까지 "
                  f"yaw={_trunk_approach_yaw_deg:.0f}도로 자세 붕괴 없이 도달했습니다.", flush=True)

            # 사용자 지적 - "자세나 이런것들도 우리 place 처음 시작할때 자세가 되도록". 92번은
            # STAGE 1이 시작되기 전에 이미 리프트를 LIFT_MAX(92번 기준 0.388)까지 올려둔 채였다
            # (스폰 직후 1회, 이 파일 앞부분의 raise_lift_and_fold와 동일 원리) - LIFT_MAX는
            # 이제 92번과 완전히 동일값으로 고정돼 있으므로(PICK 전용 높이는 PICK_LIFT_H로 분리)
            # 그대로 쓰면 된다. 관절 목표는 _fold_target이 아니라 _pick_fold를 써야 한다 - 안
            # 그러면 solution space 2(joint_3/5=-90/-90)가 STAGE 1 진입 직전에 여기서 다시
            # +90/+90으로 풀려버려 트렁크 배치가 요구하는 solution space가 무효화된다.
            raise_lift_and_fold(LIFT_MAX, _pick_fold, steps=200)

            # 사용자 실측 확인(2차 - 리프트 높이를 92번과 맞춘 뒤에도 STAGE1/1.1 동안 yaw가 여전히
            # -0.5도->-4.2도로 틀어짐) - 리프트 높이는 원인이 아니었다. 실제 원인은 홀로노믹 바퀴가
            # DRIVE_STIFFNESS=0(위치 유지 없음, 감쇠만 있는 속도 드라이브)라는 점이다.
            # drive_until()의 감속 꼬리(30스텝)는 속도를 "거의" 0으로 줄일 뿐 정확히 0으로 만들지
            # 않고, 그 이후 STAGE 1(400스텝)+1.1(400스텝) 동안은 move_link6()가 팔만 움직이고
            # base_robot에는 그 어떤 명령도 다시 보내지 않는다 - 마지막으로 명령된 그 미세한 잔여
            # 속도가 바퀴 속도 드라이브의 목표값으로 800스텝 내내 그대로 유지되며 조금씩 계속
            # 밀렸을 가능성이 크다(STAGE1/1.1은 92번 원본 코드라 손대지 않는다 - 대신 그 코드로
            # 넘어가기 직전, 이 파일이 소유한 STAGE 0.8 끝에서 섀시 속도를 명시적으로 완전히
            # 0으로 만들어 넘긴다).
            base_robot.apply_action(holo_forward(0.0, 0.0, 0.0))
            base_robot.set_linear_velocity(np.zeros(3))
            base_robot.set_angular_velocity(np.zeros(3))
            print(f"[STAGE0.8 완료] 리프트={lift_state['h']:.3f}(92번 시작 자세와 동일) grasped={gripper.is_closed()} "
                  f"섀시 속도 0으로 강제 정지", flush=True)

            chassis_pos_arrived, _ = base_robot.get_world_pose()
            snapshot(eye=[chassis_pos_arrived[0] - 1.5, chassis_pos_arrived[1] - 2.2, 1.4],
                     target=[float(chassis_pos_arrived[0]), ANCHOR_Y, 0.7], fname="_cart2trunk_04_arrived_standoff.png")

            if STAGE < 1:
                print("\n[STAGE 0.8 완료] 트렁크 standoff 도착 + yaw=0도 + 리프트 재상승까지 확인하세요. "
                      "STAGE=1 이상으로 다시 실행하면 92번과 동일한 STAGE 1(홀딩 자세 재확립)부터 이어집니다.\n",
                      flush=True)

        if STAGE < 1:
            # 92번은 STAGE 최솟값이 1이라 이 분기 자체가 없었다 - 94번은 0/0.5/0.8이 새로 생겨서
            # 여기서 명시적으로 끝내야 아래 STAGE 1 코드(가드 없이 무조건 실행되는 92번 원본 구조
            # 그대로)가 의도치 않게 같이 실행되는 걸 막는다. sys.exit()으로 확실히 끝낸다 -
            # simulation_app.close() 뒤에도 파이썬 코드는 계속 실행되므로 close()만으로는 안 멈춘다.
            if HEADLESS:
                simulation_app.close()
            else:
                print("[안내] 창을 직접 둘러보세요 - 닫으면 스크립트가 종료됩니다.\n", flush=True)
                while simulation_app.is_running():
                    step_hold(1)
                simulation_app.close()
            sys.exit(0)

        # ================= STAGE 1: 안전 홀딩 자세 확립 (그리퍼가 아래를 보게, 목표 높이 근처) =================
        # 92.trunk_place_holonomic.py와 완전히 동일 - 다만 이번엔 이미 STAGE 0.5/0.8에서 이 근처
        # 자세로 온 뒤라, 이 move_link6는 실질적으로 미세 보정(재확립)만 한다. 박스는 STAGE 0에서
        # 실제로 이미 집었으므로(92번의 합성 스폰+흡착 블록은 여기선 필요 없다 - 위에서
        # test_box/TEST_BOX_SIZE를 이미 실제 박스로 alias했다), STAGE 2 진입 판정(BOX_NEEDS_TILT)
        # 만 새로 계산한다.
        _init_ee_pos, _ = m0609_robot.end_effector.get_world_pose()
        holding_pos = (float(_init_ee_pos[0]), float(_init_ee_pos[1]), HOLDING_Z)
        move_link6(holding_pos, steps=400, hold_gripper_closed=False, orientation=DOWN_QUAT,
                   label="안전 홀딩 자세(그리퍼 하향, 목표 높이 근처) 확립")
        print(f"[박스 확인] grasped={gripper.is_closed()}", flush=True)

        # 사용자 설계 문서(Stage 2 한계 극복) - 지금 박스가 수평 통과 가능한지, Tilt-and-Insert가
        # 필요한지 미리 판정해둔다(실제 STAGE 2 진입은 아래에서 이 값을 보고 분기).
        BOX_NEEDS_TILT, _tilt_required, _tilt_available = box_needs_tilt(TEST_BOX_SIZE[2], ceiling_z=CEILING_WORLD_Z)
        if os.environ.get("FORCE_TILT_TEST") == "1":
            print("[FORCE_TILT_TEST] 실제 판정과 무관하게 Tilt-and-Insert 경로를 강제로 사용합니다"
                  "(대형 박스 시나리오 없이 새 경로를 스모크 테스트하기 위한 진단 플래그).", flush=True)
            BOX_NEEDS_TILT = True
        print(f"[문턱 통과 방식 판정(레거시, 참고용)] 박스높이={TEST_BOX_SIZE[2]:.3f}m 필요공간={_tilt_required:.3f}m "
              f"가용공간={_tilt_available:.3f}m -> {'TILT_AND_INSERT 필요' if BOX_NEEDS_TILT else '수평 통과 가능'}",
              flush=True)

        # [실측으로 확인된 버그 - 2026-07-26] classify_entry_strategy()가 지금까지 로그로만
        # 비교되고 실제 라우팅엔 연결이 안 돼 있었다(주석에 "3~4차에서 교체 예정"이라고
        # 적혀만 있었음). 게다가 TEST_BOX_SIZE(placement_result.json의 dimensions)는
        # algorism이 _oriented_footprint()로 회전을 보정해서 뽑은 "진짜 변 길이"라 -
        # 이 박스가 스캔 당시 완벽히 축 정렬이 아니라 약간(실측 ~2도) 돌아간 채로
        # 놓여 있었는데도, PICK 이후 그 회전을 되돌리는 조작(wrist_yaw 보정)이 전혀
        # 없어서 트렁크 입구에 다가갈 때는 여전히 그 회전이 그대로 살아있다 - 즉
        # TEST_BOX_SIZE[0](0.1898)을 그대로 넣으면 "회전 때문에 실제로 커진 폭"을
        # 놓친다(실측: 이 회전만으로 축정렬 폭이 0.1898->약 0.196로 커짐, 로그의
        # 실제 박스 폭 0.199와 거의 일치). _get_box_x_edges()와 동일한 분리축(SAT)
        # 투영 공식으로 "지금 이 순간 박스의 실제 world X 폭"을 직접 재서 그 값으로
        # classify_entry_strategy를 판정한다 - 정적 설계값이 아니라 실측값 기준.
        _entry_box_pos, _entry_box_quat = test_box.get_world_pose()
        _entry_R = quat_wxyz_to_matrix(np.asarray(_entry_box_quat, dtype=float))
        _entry_half_dims = np.asarray(TEST_BOX_SIZE, dtype=float) / 2.0
        _entry_projected_half_x = (
            abs(_entry_R[0, 0]) * _entry_half_dims[0]
            + abs(_entry_R[0, 1]) * _entry_half_dims[1]
            + abs(_entry_R[0, 2]) * _entry_half_dims[2]
        )
        _entry_real_box_x_len = 2.0 * _entry_projected_half_x
        print(f"[진입전략 실측 폭 보정] 설계값(회전보정됨) TEST_BOX_SIZE[0]={TEST_BOX_SIZE[0]:.4f}m -> "
              f"현재 world 회전 반영 실측 폭={_entry_real_box_x_len:.4f}m", flush=True)
        _entry_dims_for_strategy = (_entry_real_box_x_len, TEST_BOX_SIZE[1], TEST_BOX_SIZE[2])
        ENTRY_STRATEGY, ENTRY_STRATEGY_INFO = classify_entry_strategy(_entry_dims_for_strategy)
        print(f"[진입전략 판정(라우팅에 실제로 사용됨)] strategy={ENTRY_STRATEGY} info={ENTRY_STRATEGY_INFO}",
              flush=True)
        if ENTRY_STRATEGY == "INFEASIBLE":
            pause_for_inspection(
                f"[중단] 이 박스(실측 폭={_entry_real_box_x_len:.3f}m)는 현재 트렁크 입구 형상으로는 "
                "수평 통과도 Tilt-and-Insert도 불가능하다고 판정됐습니다(INFEASIBLE) - "
                "박스 회전/크기 또는 진입 전략을 재검토하세요."
            )

        # 사용자 지적 - 93번 진단은 "박스 자체의 수직 두께"만 필요공간으로 계산했는데, 실제로 입구를
        # 통과해야 하는 건 박스 혼자가 아니라 "박스를 아래에 매달고 있는 그리퍼 + 그 그리퍼가 붙은
        # link_6(팔 최종 세그먼트)"까지 포함한 강체 전체다(그림 참고 - link_6가 박스 바로 위에서
        # 수직으로 튀어나와 있음). 이 전체 스택의 z 길이(박스 바닥 ~ link_6 최상단)를 실측한다.
        #
        # 시행착오 - (1) 메시 포인트를 그냥 Usd.PrimRange로 순회했더니 link_6가 None이었다(link_6
        # 시각 메시가 instanceable 참조라서 기본 순회가 안으로 안 들어감 - 인스턴스 프록시 predicate
        # 필요). (2) UsdGeom.BBoxCache로 바꿨더니 이번엔 test_box 높이가 0.0112m로 나왔는데 이건
        # 정확히 TEST_BOX_SIZE[2]^2(0.106^2=0.01124)과 일치한다 - 예전 91/92번에서 겪었던 그
        # "BBoxCache가 치수를 제곱해서 반환하는" 버그가 여기서도 똑같이 재현된 것(DynamicCuboid의
        # scale 오퍼레이터와 BBoxCache가 서로 스케일을 이중 적용하는 것으로 추정, 근본 원인 미해결).
        # 최종: 메시 포인트 직접 변환 방식을 유지하되 Usd.TraverseInstanceProxies()로 인스턴스 안까지
        # 들어가도록 고쳤다. 박스 자체는 이미 정확한 치수(TEST_BOX_SIZE)와 실측 world pose를 알고
        # 있으므로 bbox 계산 없이 그대로 쓴다(불필요하게 버그가 있는 API에 또 의존할 이유가 없음).
        # ================= 박스 가장자리 / 운반 포락선 / 자세 클리어런스 (설계 문서 2, 6.2-6.3) =================
        # 사용자 지적 - 박스만 봐서는 안 되고 "박스 + 그리퍼 + link_6 + 전완"까지 포함한 전체 포락선이
        # 트렁크 구간별 자유공간 안에 있는지를 봐야 한다. _get_box_x_edges()는 예전에 STAGE>=2 블록
        # 안에서만 쓰던 함수인데, OBB->AABB 투영 공식(abs(R[0,i])*half_dim[i] 합산 - 분리축 정리와
        # 동일, 이미 정확함이 검증됨)이라 그대로 재사용하고 위치만 모듈 스코프로 옮긴다(STAGE 1
        # 시점부터도 로그로 확인할 수 있게).
        def _get_box_x_edges():
            """박스의 실시간 world pose(중심+회전)와 실제 치수(TEST_BOX_SIZE)로 world X축 투영
            반길이를 직접 계산한다(BBoxCache의 치수 제곱 버그를 우회, 검증됨)."""
            box_pos, box_quat = test_box.get_world_pose()
            center = np.asarray(box_pos, dtype=float)
            rotation = quat_wxyz_to_matrix(np.asarray(box_quat, dtype=float))
            half_dims = np.asarray(TEST_BOX_SIZE, dtype=float) / 2.0
            projected_half_x = (
                abs(rotation[0, 0]) * half_dims[0]
                + abs(rotation[0, 1]) * half_dims[1]
                + abs(rotation[0, 2]) * half_dims[2]
            )
            rear_x = float(center[0] - projected_half_x)
            front_x = float(center[0] + projected_half_x)
            return rear_x, front_x, center


        def measure_carry_envelope():
            """박스 + CARRY_ENVELOPE_PARTS(그리퍼/link_6/전완)의 결합 world AABB.
            X는 이미 검증된 _get_box_x_edges()의 회전-투영값과 메시 AABB 중 더 바깥쪽을 취한다."""
            mins = [None, None, None]
            maxs = [None, None, None]
            for part in CARRY_ENVELOPE_PARTS:
                part_min, part_max = _mesh_world_aabb(part)
                for i in range(3):
                    if part_min[i] is None:
                        continue
                    mins[i] = part_min[i] if mins[i] is None else min(mins[i], part_min[i])
                    maxs[i] = part_max[i] if maxs[i] is None else max(maxs[i], part_max[i])
            box_rear_x, box_front_x, box_center = _get_box_x_edges()
            box_bottom_z = float(box_center[2]) - TEST_BOX_SIZE[2] / 2.0
            return {
                "bottom_z": box_bottom_z,
                "top_z": maxs[2] if maxs[2] is not None else box_bottom_z + TEST_BOX_SIZE[2],
                "rear_x": min(box_rear_x, mins[0]) if mins[0] is not None else box_rear_x,
                "front_x": max(box_front_x, maxs[0]) if maxs[0] is not None else box_front_x,
                "y_min": mins[1], "y_max": maxs[1],
            }


        def evaluate_pose_clearance():
            """설계 문서 6.3 - 지금 자세가 문턱/천장 대비 얼마나 여유 있는지 실측해서 반환한다.
            ceiling_z_at/floor_z_at은 라이브 raycast라 x=3.125 같은 매직넘버 없이 그 시점 실제
            형상을 그대로 반영한다."""
            env = measure_carry_envelope()
            ceiling_here = ceiling_z_at(env["front_x"])
            floor_here = floor_z_at(env["rear_x"])
            box_to_threshold = (env["bottom_z"] - floor_here) if floor_here is not None else None
            box_to_ceiling = (ceiling_here - env["top_z"]) if ceiling_here is not None else None
            candidates = [v for v in [box_to_threshold, box_to_ceiling] if v is not None]
            return {
                "box_to_threshold": box_to_threshold,
                "box_to_ceiling": box_to_ceiling,
                "minimum_clearance": min(candidates) if candidates else None,
                "envelope": env,
            }


        def _log_clearance(label):
            c = evaluate_pose_clearance()
            env = c["envelope"]
            print(f"\n[포락선 클리어런스: {label}] rear_x={env['rear_x']:.3f} front_x={env['front_x']:.3f} "
                  f"bottom_z={env['bottom_z']:.3f} top_z={env['top_z']:.3f}", flush=True)
            print(f"[클리어런스: {label}] box_to_threshold={c['box_to_threshold']} "
                  f"box_to_ceiling={c['box_to_ceiling']} minimum_clearance={c['minimum_clearance']}", flush=True)
            return c


        PROBE_ARM_ENVELOPE = os.environ.get("PROBE_ARM_ENVELOPE") == "1"
        _log_clearance("STAGE1(HOLDING_Z)")

        chassis_pos0, _ = base_robot.get_world_pose()
        snapshot(eye=[chassis_pos0[0] - 2.2, chassis_pos0[1] - 3.2, chassis_pos0[2] + 1.6],
                 target=[(chassis_pos0[0] + CAR_POS[0]) / 2, 0.0, 1.0], fname="_trunkplace_00_start.png")

        if STAGE < 1.1:
            print("\n[STAGE 1 완료] 홀딩 자세(그리퍼 하향) + 박스 파지 확인용 스크린샷 저장 완료 - "
                  "STAGE=1.1 이상으로 다시 실행하면 다음 단계로 진행합니다.\n", flush=True)

        if STAGE >= 1.1:
            # ================= STAGE 1.1: 박스 하단이 트렁크 입구 턱을 넘도록 높이 올리기 =================
            # 사용자 지시 - 지금 상태(release 높이 근처)로 그냥 전진하면 팔이 트렁크 입구 턱에
            # 부딪힌다 - 박스 하단 좌표를 그 턱보다 높은 위치(ENTRY_HOLDING_Z)까지 올리되, 그리퍼/
            # 매니퓰레이터가 천장에는 부딪히지 않는 자세를 만든다. XY는 그대로(자기 몸 근처) 유지한
            # 채 Z만 바꾼다 - STAGE 2/3에서 이 높이로 접근한 뒤 마지막에만 release 높이로 내린다.
            entry_pos = (float(_init_ee_pos[0]), float(_init_ee_pos[1]), ENTRY_HOLDING_Z)
            move_link6(entry_pos, steps=200, hold_gripper_closed=True, orientation=DOWN_QUAT,
                       label="STAGE1.1: 입구 턱 클리어 높이로 상승")

            _log_clearance("STAGE1.1(ENTRY_HOLDING_Z)")

            chassis_pos0, _ = base_robot.get_world_pose()
            snapshot(eye=[chassis_pos0[0] - 2.0, chassis_pos0[1] - 2.8, chassis_pos0[2] + 1.4],
                     target=[(chassis_pos0[0] + CAR_POS[0]) / 2, 0.0, ENTRY_HOLDING_Z],
                     fname="_trunkplace_00c_entry_height.png")

            if STAGE < 2:
                print(f"\n[STAGE 1.1 완료] ENTRY_HOLDING_Z={ENTRY_HOLDING_Z:.3f}로 상승 완료 - "
                      "스크린샷에서 박스 하단이 트렁크 입구 턱보다 높은지, 그리퍼/팔이 천장에 닿지 "
                      "않는지 확인하세요. 부족하면 ENTRY_CLEARANCE_ABOVE_RELEASE 값을 조정 후 재실행. "
                      "STAGE=2 이상으로 다시 실행하면 다음 단계로 진행합니다.\n", flush=True)

        if STAGE >= 2:
            # ================= STAGE 1.9(신규): STAGE 2 진입 전 섀시 y/yaw 재정렬 =================
            # 사용자 실측 확인 - STAGE 0.8 종료 시 yaw는 0도 근처였는데, STAGE 1/1.1(둘 다 팔만
            # 움직이고 섀시엔 아무 명령도 없음)을 지나는 동안 yaw가 몇 도씩 틀어지고(-4도대) 그
            # 상태로 STAGE 2(92번 원본, target_yaw_deg 없이 "시작 시점 yaw"를 그대로 유지)에 들어가면
            # 그 틀어진 방향으로 계속 전진해 박스가 Y 허용범위(STAGE2_Y_TOLERANCE)를 넘겨 중단됐다.
            # STAGE 2/3.0 코드(92번 원본)는 그대로 두고, 그 대신 여기서 "STAGE 2가 시작하기 직전"에
            # 섀시를 yaw=_trunk_approach_yaw_deg(보통 0도, 조인트3/5 반전 박스는 180도)로 다시
            # 맞춘다 - X는 그대로 둔다(STAGE 1/1.1은 X를 안 바꾸므로 여기서도 건드릴 이유가 없다).
            # STAGE 0.8이 이미 이 yaw로 도착했지만, 그 뒤 STAGE 1/1.1 동안 팔만 움직이고 섀시엔
            # 명령이 없어 yaw가 미세하게 틀어지므로 여기서 같은 목표로 다시 잡아준다(0.8과 다른
            # yaw를 쓰면 방금 세운 반전 자세가 여기서 도로 풀려버린다).
            #
            # 사용자 실측 확인(2차 - 조인트3/5 반전 박스에서 STAGE2가 1스텝만에 "박스 Y가
            # STAGE2_Y_TOLERANCE를 넘었다"로 중단) - 원인은 흡착 오차(PICK 로그의 horiz=8.5cm
            # 같은, 항상 존재하는 그리퍼-박스 중심 어긋남)가 손목(joint_6) 각도에 따라 world
            # Y/X 어느 쪽으로 투영되는지가 달라진다는 것: 반전 branch는 손목 각도가 달라져서 이
            # 어긋남이 X 대신 Y로 크게 튀었다. "섀시 Y=ANCHOR_Y"가 아니라 "박스 중심 Y=ANCHOR_Y"가
            # 진짜 목표이므로, 여기서 그 차이(박스중심Y - 섀시Y, 흡착 오차 그대로 반영된 실측값)를
            # 재고 섀시 목표 Y를 그만큼 보정한다 - 92번 STAGE2 코드는 안 건드리고 이 신규 정렬
            # 단계의 목표만 고친다.
            _align_hold_q = np.asarray(m0609_robot.get_joint_positions(), dtype=float).copy()
            _align_chassis_start, _align_chassis_quat_start = base_robot.get_world_pose()
            _align_R_start = quat_wxyz_to_matrix(np.asarray(_align_chassis_quat_start, dtype=float))
            _align_tip_start = measure_tip_world_pos()
            _align_tip_rel_local_ref = _align_R_start.T @ (
                _align_tip_start - np.asarray(_align_chassis_start, dtype=float))
            _align_target_x = float(_align_chassis_start[0])
            _, _, _align_box_center_start = _get_box_x_edges()
            _align_box_y_offset_from_chassis = float(_align_box_center_start[1]) - float(_align_chassis_start[1])
            _align_target_y = ANCHOR_Y - _align_box_y_offset_from_chassis
            print(f"[STAGE1.9 목표Y 보정] 박스중심Y-섀시Y(흡착오차 반영)={_align_box_y_offset_from_chassis:.4f}m "
                  f"-> 섀시 목표Y={_align_target_y:.4f}(ANCHOR_Y={ANCHOR_Y:.3f} 대신)", flush=True)

            def _hold_align_arm():
                m0609_robot.apply_action(ArticulationAction(joint_positions=_align_hold_q))
                m0609_robot.gripper.close()

            def _align_broken():
                chassis_pos, chassis_quat = base_robot.get_world_pose()
                R_now = quat_wxyz_to_matrix(np.asarray(chassis_quat, dtype=float))
                tip_pos = measure_tip_world_pos()
                tip_rel_local = R_now.T @ (tip_pos - np.asarray(chassis_pos, dtype=float))
                relative_error = float(np.linalg.norm(tip_rel_local - _align_tip_rel_local_ref))
                detached = not m0609_robot.gripper.is_closed()
                return relative_error > 0.025 or detached

            _, _, _, align_aborted = drive_until(
                lambda: False, target_x=_align_target_x, target_y=_align_target_y,
                target_yaw_deg=_trunk_approach_yaw_deg,
                tolerance_xy=0.005, tolerance_yaw_deg=0.5, kp_xy=0.8, kp_yaw=0.8,
                max_speed=0.04, max_wz=0.08, per_step_fn=_hold_align_arm, abort_fn=_align_broken,
                hard_stop_on_condition=True, label="STAGE1.9: STAGE 2 진입 전 섀시 y/yaw 재정렬",
            )
            if align_aborted:
                pause_for_inspection("[중단] STAGE 1.9 재정렬 도중 자세 붕괴/흡착 이탈이 감지됐습니다.")
            _align_final_pos, _align_final_quat = base_robot.get_world_pose()
            _align_final_yaw = float(np.degrees(quat_to_euler_angles(_align_final_quat)[2]))
            print(f"[STAGE1.9 완료] 섀시=({float(_align_final_pos[0]):.3f},{float(_align_final_pos[1]):.3f}) "
                  f"yaw={_align_final_yaw:.1f}deg grasped={gripper.is_closed()}", flush=True)

        if STAGE >= 2:
            # ================= STAGE 2: 홀로노믹 근접 접근 (팔 자세 그대로, 박스가 입구를 완전히
            # 넘을 때까지 섀시만 전진) =================
            # 사용자 지시(재검토) - 목표를 "섀시 중심이 j1_x=TRUNK_X_MIN에 오는 것"으로 미리 계산해서
            # 잡는 대신, "파지한 박스가 트렁크 입구를 완전히 넘어서는 순간"을 실측으로 직접 보고
            # 멈춘다 - 섀시-박스 간 정확한 오프셋(팔 모양에 따라 달라짐)을 계산에 넣을 필요가 없어
            # 더 안전하다(drive_until(), 33.py의 raise_lift_and_link6_until()과 동일 원칙).
            #
            # 사용자 지적(2차, 충돌 분석 기반 전면 재작성) - 디버그 로그(STAGE=2 1차 실행)로 95~108
            # 스텝 구간에서 실제 물리 충돌이 확인됐다: 팁-섀시 X 오프셋이 0.360m -> 0.322m로 줄고
            # 박스 Y가 3스텝 만에 18cm 튀었는데, 기존 종료 조건은 "박스 X가 입구를 넘었는가"만 봐서
            # 이 충돌로 튄 위치도 "성공"으로 오판했다. 아래를 전부 새로 만든다:
            #   1) stage2_hold_q - STAGE 1.1에서 확립한 조인트값을 저장해두고, per_step_fn으로 매
            #      스텝 다시 명령해서 팔이 충돌에 순순히 밀리지 않도록 버틴다.
            #   2) stage2_tip_rel_ref - 시작 시점의 "팁-섀시 상대 위치"를 기준값으로 저장, 매 스텝
            #      이 기준에서 얼마나 벗어났는지를 자세 붕괴(=충돌) 감지에 쓴다 - 실측(디버그 로그)
            #      으로 100스텝 시점 오프셋 오차가 이미 0.038m였다(108스텝의 큰 충격 전에 감지 가능).
            #   3) _box_cleared_entrance - 박스 CENTER+회전+실제 치수로 world X축 투영 반길이를 직접
            #      계산해서 뒤쪽/앞쪽 가장자리를 구한다(아래 4번 참고 - BBoxCache가 치수를 잘못
            #      돌려주는 문제를 우회). Y가 중앙선에서 크게 벗어나지 않았는지 + 그리퍼가 여전히
            #      붙어있는지도 함께 확인한다 - 충돌로 틀어진 상태를 성공으로 오판하지 않기 위함.
            #   4) hard_stop_on_condition=True - 조건 충족/자세붕괴 시 기존 30스텝 관성 감속 대신
            #      즉시 정지 - 이미 부딪혔다면 더 밀어넣지 않는다.
            #   5) max_speed_fn - 입구에 가까워질수록(박스 뒤쪽 가장자리 기준 남은 거리) 속도를
            #      줄인다 - 기존 0.4m/s는 입구 통과 속도로는 너무 빨라 충돌 시 충격이 컸다.
            #
            # 사용자 지적(3차, 재검토) - 실제 재현 로그를 다시 맞춰보니 두 가지가 더 있었다:
            #   a) ENTRANCE_CLEAR_MARGIN=0.05는 "입구를 넘은 뒤 바로 안쪽"이 아니라 "박스 전체가
            #      입구를 넘은 뒤 추가로 5cm 더 들어가야 성공"이라는 뜻이었다 - 박스 반길이(6.75cm)
            #      까지 합치면 박스 중심이 입구보다 11.75cm나 더 들어가야 했다. 화면상 이미 들어간
            #      것처럼 보여도 이 과도한 여유 때문에 조건이 계속 False였고, 그 사이 팔이 먼저
            #      차량에 부딪혔다. 0.005로 대폭 줄인다(추가 삽입 거리는 STAGE 3의 몫으로 미룬다).
            #   b) get_world_aabb()(BBoxCache 기반)가 반환한 박스 크기가 실제 TEST_BOX_SIZE의 제곱에
            #      가까운 값(예: 0.135^2≈0.018 - 실측 AABB 폭과 거의 일치)이었다 - scale이 이중
            #      적용되는 것으로 보이는 버그. BBoxCache를 아예 안 쓰고, 이미 알고 있는 실제 치수
            #      (TEST_BOX_SIZE)와 박스의 실시간 world pose(중심+회전)로 X축 투영 반길이를 직접
            #      계산한다 - 이 버그 자체를 우회한다.
            #   c) TRUNK_X_MIN을 "적재 공간 시작점"과 "실제 차량 개구부 평면" 두 용도로 같이 쓰면
            #      안 된다(차량 형상상 몇 cm 차이 날 수 있음) - TRUNK_ENTRANCE_X로 분리했다(현재는
            #      같은 값, 아래 마커로 실제 위치 확인 후 필요하면 이 값만 조정).
            # classify_entry_strategy()가 쓰는 것과 반드시 같은 값이어야 하므로(위 함수 정의부
            # 근처 ENTRANCE_CLEAR_MARGIN_M 참고) 리터럴을 다시 안 쓰고 그 상수를 그대로 참조한다.
            ENTRANCE_CLEAR_MARGIN = ENTRANCE_CLEAR_MARGIN_M  # 입구를 "박스 뒤쪽 끝"이 넘은 뒤 아주 약간만 더 여유를 둔다.
            STAGE2_Y_TOLERANCE = 0.04  # 박스 중심이 중앙선(ANCHOR_Y)에서 이 이상 벗어나면 이상으로 본다.
            STAGE2_POSE_DRIFT_TOLERANCE = 0.025  # 팁-섀시 상대 위치가 시작 시점 대비 이 이상 벗어나면 충돌로 본다.

            # 디버그/확인용 마커 평면 - 초록=TRUNK_ENTRANCE_X(입구 평면), 노랑=성공 판정 평면
            # (TRUNK_ENTRANCE_X+ENTRANCE_CLEAR_MARGIN). 스크린샷에서 이 두 평면이 실제 범퍼/개구부
            # 위치와 얼마나 차이 나는지 눈으로 바로 확인할 수 있다. 충돌 콜리전은 없는 순수 시각 마커.
            def _add_x_marker(name, x, color):
                marker = UsdGeom.Cube.Define(stage, f"/World/{name}")
                marker.CreateSizeAttr(1.0)
                marker.CreateDisplayColorAttr([Gf.Vec3f(*color)])
                xform = UsdGeom.Xformable(marker)
                xform.ClearXformOpOrder()
                xform.AddTranslateOp().Set(Gf.Vec3d(x, 0.0, 0.75))
                xform.AddScaleOp().Set(Gf.Vec3f(0.003, 0.55, 0.30))

            _add_x_marker("EntrancePlane", TRUNK_ENTRANCE_X, (0.0, 1.0, 0.0))
            _add_x_marker("SuccessPlane", TRUNK_ENTRANCE_X + ENTRANCE_CLEAR_MARGIN, (1.0, 1.0, 0.0))

            def _measure_tip_pos():
                gripper_mat = UsdGeom.Xformable(stage.GetPrimAtPath(gripper_body_path)).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default())
                return np.array(gripper_mat.Transform(Gf.Vec3d(*TIP_LOCAL_OFFSET)), dtype=float)

            # _get_box_x_edges()는 이제 모듈 스코프(STAGE 1 부착 직후)에 정의돼 있다 - 여기서 다시
            # 정의하지 않고 그대로 재사용한다.

            # ---- 기준값(자세 붕괴 감지용) - STAGE 2 시작 시점, 아직 충돌 전의 "정상" 상대 위치 ----
            stage2_hold_q = np.asarray(m0609_robot.get_joint_positions(), dtype=float).copy()
            _stage2_chassis_start, _ = base_robot.get_world_pose()
            _stage2_tip_start = _measure_tip_pos()
            stage2_tip_rel_ref = _stage2_tip_start - np.asarray(_stage2_chassis_start, dtype=float)
            print(f"[STAGE2 기준값] 조인트={np.round(stage2_hold_q, 3)} "
                  f"팁-섀시 상대위치(기준)={np.round(stage2_tip_rel_ref, 3)}", flush=True)

            def _hold_stage2_arm():
                # 매 스텝 STAGE 1.1의 조인트값을 다시 명령한다 - 충돌 등으로 자세가 흔들려도 최대한
                # 원래 모양으로 버티게 한다(사용자 지적 - 예전엔 명령을 전혀 안 보내서 충돌에 그대로
                # 밀렸었다).
                m0609_robot.apply_action(ArticulationAction(joint_positions=stage2_hold_q))
                m0609_robot.gripper.close()

            def _stage2_pose_broken():
                chassis_pos, _ = base_robot.get_world_pose()
                tip_pos = _measure_tip_pos()
                tip_rel = tip_pos - np.asarray(chassis_pos, dtype=float)
                relative_error = float(np.linalg.norm(tip_rel - stage2_tip_rel_ref))
                _, _, box_center = _get_box_x_edges()
                y_broken = abs(float(box_center[1]) - ANCHOR_Y) > STAGE2_Y_TOLERANCE
                pose_broken = relative_error > STAGE2_POSE_DRIFT_TOLERANCE
                detached = not m0609_robot.gripper.is_closed()
                return pose_broken or y_broken or detached

            # 사용자 설계 문서(3차: VERIFY_TRANSITION_POCKET) - "박스 뒤쪽이 입구를 넘었는가"만으로는
            # 부족하다. 박스 앞쪽이 (안쪽으로 갈수록 낮아지는) 내부천장 시작점을 넘어서기 전이어야
            # 하고, 지금 자세의 실측 클리어런스(그리퍼/link_6/전완 포함 포락선 기준)도 최소 여유를
            # 만족해야 한다 - 그래야 "다음 하강/자세복원 스윕이 안전한 상태"임을 확실히 하는
            # VERIFY_TRANSITION_POCKET 조건이 된다.
            # classify_entry_strategy()와 반드시 같은 값(FRONT_CLEAR_MARGIN_M 참조, 위 ENTRANCE_
            # CLEAR_MARGIN과 동일한 이유).
            FRONT_CLEAR_MARGIN = FRONT_CLEAR_MARGIN_M  # 박스 앞쪽이 내부천장 시작점보다 이만큼 못 미쳐야 한다.
            MIN_CLEARANCE_MARGIN = HORIZONTAL_PASS_MARGIN  # 포락선 최소 여유(기존 마진 상수 재사용)

            def _box_cleared_entrance():
                rear_x, front_x, box_center = _get_box_x_edges()
                x_cleared = rear_x >= TRUNK_ENTRANCE_X + ENTRANCE_CLEAR_MARGIN
                front_clear = front_x <= INTERNAL_CEILING_START_X - FRONT_CLEAR_MARGIN
                y_centered = abs(float(box_center[1]) - ANCHOR_Y) < STAGE2_Y_TOLERANCE
                attached = m0609_robot.gripper.is_closed()
                if not (x_cleared and front_clear and y_centered and attached):
                    return False
                clearance = evaluate_pose_clearance()["minimum_clearance"]
                return clearance is not None and clearance >= MIN_CLEARANCE_MARGIN

            def _stage2_max_speed():
                # 사용자 지적 - 0.4m/s는 트렁크 입구를 통과하는 속도치고 너무 빠르다(충돌 시 충격
                # 큼). 박스 뒤쪽 가장자리 기준 입구까지 남은 거리에 따라 속도를 단계적으로 낮춘다.
                rear_x, _, _ = _get_box_x_edges()
                remaining = (TRUNK_ENTRANCE_X + ENTRANCE_CLEAR_MARGIN) - rear_x
                if remaining < 0.03:
                    return 0.015
                if remaining < 0.08:
                    return 0.025
                if remaining < 0.15:
                    return 0.05
                return 0.10

            # 사용자 요청 - 종료 조건이 실제로 언제/왜 걸리는지(또는 안 걸리는지) 매 구간 실측값으로
            # 직접 확인할 수 있게, 트렁크 입구 x, 섀시 중심, 그리퍼 팁, 박스 뒤/앞 가장자리를
            # 주기적으로 찍는다. 팁-섀시 오프셋을 기준값과 비교해서 편차도 같이 보여준다.
            def _stage2_debug(step):
                chassis_pos, _ = base_robot.get_world_pose()
                rear_x, front_x, box_center = _get_box_x_edges()
                tip_pos = _measure_tip_pos()
                tip_rel = tip_pos - np.asarray(chassis_pos, dtype=float)
                rel_error = float(np.linalg.norm(tip_rel - stage2_tip_rel_ref))
                threshold_x = TRUNK_ENTRANCE_X + ENTRANCE_CLEAR_MARGIN
                print(f"  [DEBUG step={step}] 트렁크입구 x={TRUNK_ENTRANCE_X:.3f} 임계값={threshold_x:.3f} | "
                      f"섀시중심=({float(chassis_pos[0]):.3f},{float(chassis_pos[1]):.3f},{float(chassis_pos[2]):.3f}) | "
                      f"그리퍼팁=({tip_pos[0]:.3f},{tip_pos[1]:.3f},{tip_pos[2]:.3f}) "
                      f"팁-섀시상대오차(기준대비)={rel_error:.4f}m | "
                      f"박스 뒤={rear_x:.3f} 앞={front_x:.3f} 중심={np.round(box_center,3)} "
                      f"임계값까지남은거리={threshold_x - rear_x:+.3f} "
                      f"내부천장시작x={INTERNAL_CEILING_START_X:.3f}(앞쪽까지여유={INTERNAL_CEILING_START_X - front_x:+.3f}) "
                      f"붙어있음={m0609_robot.gripper.is_closed()}",
                      flush=True)

            # 사용자 설계 문서(Stage 2 한계 극복) - Tilt-and-Insert: 박스+그리퍼 스택이 수평으로는
            # 입구 개구부에 안 들어가는 큰 박스용 대안 진입. 4단계(그림 참고):
            #   1) 문턱 전방 접근 - 박스 수평 유지한 채 입구 앞 approach_standoff까지 이동(기존
            #      STAGE2 "얼려서 드라이브" 패턴 재사용, 목표 지점만 다름).
            #   2) 진입 전 피치 회전 - 섀시 정지, 그리퍼(및 박스)를 tilt_deg만큼 피치업해서 선단
            #      하부 모서리가 문턱보다 높아지게 한다(천장으로 더 올리는 대신 "기울여서" 유효
            #      높이를 줄이는 방식 - 천장 한계는 그대로 유지해야 하므로 절대 위로 더 올리지 않음).
            #   3) 기울임 자세로 문턱 통과 - 그 자세를 유지한 채 섀시를 전진(팔은 능동 추종,
            #      drive_and_reach와 동일 패턴이나 orientation을 기울인 채 고정).
            #   4) 내부 자세 복원 - 문턱을 지난 뒤 섀시 정지, 그리퍼를 다시 수평(DOWN_QUAT)으로.
            # 각 단계 실패(자세 붕괴/그리퍼 이탈/수렴 실패) 시 SystemExit으로 즉시 중단 - STAGE 3/4가
            # 잘못된 상태 위에서 이어지지 않게 한다(STAGE 3에서 이미 확립한 원칙과 동일).
            #
            # 주의 - 이 함수는 지금 실제로 "수평 통과 불가능한 큰 박스" 시나리오가 없어서(현재
            # placement_result.json의 박스는 수평 통과 가능 판정) 물리적 충돌 검증을 아직 못 했다.
            # FORCE_TILT_TEST=1로 강제 진입시켜 코드 경로 자체는 확인할 수 있지만, restore_clear_margin
            # 값은 실제 대형 박스가 생기면 그때 실측 기반으로 다시 튜닝해야 한다(이 프로젝트의 다른
            # 모든 단계도 그렇게 완성됨).
            def tilt_and_insert_through_entrance(entrance_x, box_dims, tilt_deg=None, approach_standoff=None,
                                                  restore_clear_margin=0.10, tilt_steps=150,
                                                  drive_max_speed=0.05, tilt_standoff_safety_margin=0.05):
                box_height = float(box_dims[2])
                pivot_local_z = box_height / 2.0

                if tilt_deg is None:
                    # 사용자 설계 문서(4차) - 고정 12도 대신, 문턱/천장을 동시에 만족하는 가장 작은
                    # 각도를 find_min_tilt_angle()로 탐색한다(2차에서 이미 만든 함수 재사용).
                    floor_ref, ceiling_ref = floor_z_at(entrance_x), ceiling_z_at(entrance_x)
                    if floor_ref is not None and ceiling_ref is not None:
                        tilt_deg = find_min_tilt_angle(box_dims, pivot_world_z=ENTRY_HOLDING_Z,
                                                        floor_clear_z=floor_ref, ceiling_clear_z=ceiling_ref)
                    if tilt_deg is None:
                        raise SystemExit(
                            "[중단] Tilt-and-Insert: 문턱/천장을 동시에 만족하는 피치 각도를 찾지 "
                            "못했습니다(INFEASIBLE) - 이 박스는 이 입구로 통과할 수 없습니다."
                        )
                tilt_quat = euler_angles_to_quat(np.array([0.0, np.pi - np.radians(tilt_deg), 0.0]))

                if approach_standoff is None:
                    # 사용자 실측 재현(이전 라운드) - box_height*sin(tilt_deg) 근사는 큰 박스에서
                    # 실측으로 충돌(err=0.06m)을 일으킨 버그였다. 4차: 근사 대신 실제 8개 꼭짓점을
                    # 회전시켜 얻은 정확한 수평 스윕(rotated_corner_extent, 2차에서 만든 함수 재사용)
                    # 으로 필요 여유를 계산한다.
                    sweep_x, _, _ = rotated_corner_extent(box_dims, tilt_deg, pivot_local_z)
                    approach_standoff = sweep_x + tilt_standoff_safety_margin
                    print(f"[TILT 파라미터 탐색] tilt_deg={tilt_deg} sweep_x={sweep_x:.3f} "
                          f"approach_standoff={approach_standoff:.3f}", flush=True)

                def _tilt_broken():
                    return not m0609_robot.gripper.is_closed()

                # ---- Phase 1: 문턱 전방 접근(수평 유지, 팔 얼려서 드라이브) ----
                # 첫 스모크 테스트(FORCE_TILT_TEST)에서 실제로 발견된 버그 - approach_standoff를
                # "섀시 x" 기준으로 재면, 팔이 뻗은 채 얼어있어 박스가 섀시보다 훨씬 앞으로 튀어나와
                # 있는 만큼(offset~0.37m) 착각이 생긴다 - 섀시는 아직 여유 있어 보여도 박스 앞쪽은
                # 이미 입구 안쪽 깊숙이 들어가 있었다. 그 상태에서 TILT-2가 방향을 틀려다 err=1.33m로
                # 터졌다(IK 결과가 터무니없으면 먼저 물리 충돌을 의심하라는 이 프로젝트의 기존 교훈과
                # 일치 - 실제로 박스가 이미 차체에 박혀있었다). "박스 앞쪽 가장자리" 기준으로 다시 잰다.
                _, box_front_start, _ = _get_box_x_edges()
                chassis_start, _ = base_robot.get_world_pose()
                box_chassis_offset = float(box_front_start) - float(chassis_start[0])
                approach_target_x = (entrance_x - approach_standoff) - box_chassis_offset
                _, _, _, p1_aborted = drive_until(
                    lambda: False, target_x=approach_target_x, target_y=ANCHOR_Y,
                    max_speed=0.10, per_step_fn=_hold_stage2_arm, abort_fn=_tilt_broken,
                    hard_stop_on_condition=True, label="TILT-1: 문턱 전방 접근(수평 유지)",
                )
                if p1_aborted:
                    raise SystemExit("[중단] TILT-1(문턱 전방 접근) 중 그리퍼 이탈 감지")

                # ---- Phase 2: 진입 전 피치 회전(섀시 정지) ----
                tilt_pos, _ = m0609_robot.end_effector.get_world_pose()
                tilt_ee, tilt_err = move_link6(tilt_pos, steps=tilt_steps, hold_gripper_closed=True,
                                                orientation=tilt_quat, label="TILT-2: 진입 전 피치 회전")
                if tilt_err > 0.03 or not m0609_robot.gripper.is_closed():
                    raise SystemExit(f"[중단] TILT-2(피치 회전) 실패: err={tilt_err:.3f}m")

                # ---- Phase 3: 기울임 자세로 문턱 통과 ----
                # 처음엔 drive_and_reach(고정 ee 목표로 능동 추종)를 썼는데, 그러면 박스가 실제로는
                # 전진하지 않고 tilt_ee 근처에 계속 머물러버린다(STAGE 3에서는 "고정된 최종 배치
                # 지점으로 뻗어가는" 용도라 맞았지만, 여긴 그냥 "이 기울인 자세 그대로 통과해서
                # 전진"이 필요하므로 안 맞는 패턴이었음). Phase 1/STAGE 2와 같은 "얼려서 드라이브"로
                # 바꾼다 - 다만 얼리는 자세가 수평이 아니라 Phase 2에서 만든 기울인 자세다.
                tilt_hold_q = np.asarray(m0609_robot.get_joint_positions(), dtype=float).copy()

                def _hold_tilt_arm():
                    m0609_robot.apply_action(ArticulationAction(joint_positions=tilt_hold_q))
                    m0609_robot.gripper.close()

                # [실측으로 확인된 버그 - 2026-07-26] 원래 "통과 완료" 기준이 entrance_x+
                # restore_clear_margin(2.855+0.10=2.955)이었는데, 낮은 천장 제약 구간은
                # entrance_x부터 INTERNAL_CEILING_START_X(3.065)까지다(classify_entry_
                # strategy()의 worst_opening 계산 구간과 동일) - 즉 2.955는 아직 낮은 천장
                # 밑에 있는 위치였다. 실측: 이 기준으로 TILT-3가 "통과 완료"라고 멈췄는데
                # 그 직후 TILT-4(수평 복원)가 낮은 천장에 팔이 걸려 IK가 크게 발산했다
                # (err=1.38m, IK 발산=물리 충돌 의심이라는 이 프로젝트의 기존 교훈과 일치).
                # 기준점을 entrance_x 대신 INTERNAL_CEILING_START_X로 바꿔서, 낮은 천장
                # 구간을 박스 뒤쪽 끝까지 완전히 벗어난 뒤에만 수평 복원을 시도하게 한다.
                def _tilt_cleared_entrance():
                    rear_x, _, box_center = _get_box_x_edges()
                    x_cleared = rear_x >= INTERNAL_CEILING_START_X + restore_clear_margin
                    y_centered = abs(float(box_center[1]) - ANCHOR_Y) < 0.04
                    return x_cleared and y_centered and m0609_robot.gripper.is_closed()

                _, _, p3_condition_met, p3_aborted = drive_until(
                    _tilt_cleared_entrance, target_x=TRUNK_X_MAX, target_y=ANCHOR_Y,
                    kp_xy=0.8, max_speed=drive_max_speed, per_step_fn=_hold_tilt_arm,
                    abort_fn=_tilt_broken, hard_stop_on_condition=True,
                    label="TILT-3: 기울임 자세 문턱 통과(팔 자세 고정, 저속)",
                )
                if p3_aborted or not p3_condition_met:
                    raise SystemExit(
                        f"[중단] TILT-3(문턱 통과) 실패(자세붕괴={p3_aborted}, 조건충족={p3_condition_met})"
                    )

                # ---- Phase 4: 내부 자세 복원(섀시 정지, 다시 수평) ----
                # [실측으로 확인된 버그 - 2026-07-26] tilt_quat(피치 -tilt_deg만큼 기울어짐)에서
                # DOWN_QUAT(완전 수평)로 move_link6 한 번 호출로 한번에 재요청하면, RMPflow가
                # "지금 위치 유지"조차 못 하고 err=0.50~1.38m 규모로 발산했다 - 그런데 세 번의
                # 재현 실행에서 발산 방향이 Z/X붕괴 -> Y쏠림 -> X/Z쏠림으로 매번 달랐다. 특정
                # 벽/천장에 고정적으로 걸리는 거라면 발산 방향도 매번 비슷해야 하는데 그렇지
                # 않았다 - 즉 move_link6()이 매 스텝 동일한 최종 target을 그대로
                # controller.forward()에 넣을 뿐 내부적으로 보간하지 않아서(위 정의 참고),
                # 좁고 꺾인 자세에서 한 번에 너무 큰 목표(피치 tilt_deg 전체)를 주면 RMPflow가
                # 매번 다른 국소해로 튀는 불안정 문제로 판단된다. 여기서 직접 tilt_deg->0을
                # 여러 단계로 나눠 점진적으로 목표를 이동시켜, RMPflow가 매번 작은 변화만
                # 쫓도록 한다.
                restore_pos, _ = m0609_robot.end_effector.get_world_pose()
                RESTORE_SUBSTEPS = 6
                restore_ee, restore_err = None, None
                for _sub_i in range(1, RESTORE_SUBSTEPS + 1):
                    _sub_tilt_deg = tilt_deg * (1.0 - _sub_i / RESTORE_SUBSTEPS)
                    _sub_quat = euler_angles_to_quat(np.array([0.0, np.pi - np.radians(_sub_tilt_deg), 0.0]))
                    restore_ee, restore_err = move_link6(
                        restore_pos, steps=max(20, tilt_steps // RESTORE_SUBSTEPS), hold_gripper_closed=True,
                        orientation=_sub_quat,
                        label=f"TILT-4: 내부 자세 복원(수평) {_sub_i}/{RESTORE_SUBSTEPS}(잔여기울임={_sub_tilt_deg:.2f}deg)")
                    if restore_err > 0.03 or not m0609_robot.gripper.is_closed():
                        raise SystemExit(
                            f"[중단] TILT-4(자세 복원) 실패: 단계 {_sub_i}/{RESTORE_SUBSTEPS}"
                            f"(잔여기울임={_sub_tilt_deg:.2f}deg) err={restore_err:.3f}m"
                        )
                return restore_ee, restore_err

            # [실측으로 확인된 버그 수정 - 2026-07-26] 레거시 BOX_NEEDS_TILT(박스 높이만 봄) 대신
            # ENTRY_STRATEGY(박스의 실측 회전-반영 폭까지 함께 보는 classify_entry_strategy() 결과)로
            # 분기한다 - 위 STAGE 1에서 이미 계산해뒀다(INFEASIBLE은 거기서 즉시 중단됨).
            if ENTRY_STRATEGY == "TILT_AND_INSERT":
                print(f"[STAGE2 경로] {ENTRY_STRATEGY_INFO} -> Tilt-and-Insert 경로 사용", flush=True)
                tilt_and_insert_through_entrance(
                    TRUNK_ENTRANCE_X, TEST_BOX_SIZE, tilt_deg=ENTRY_STRATEGY_INFO.get("tilt_angle_deg"))
                condition_met, aborted = True, False
            else:
                final_pos, final_yaw, condition_met, aborted = drive_until(
                    _box_cleared_entrance, target_x=TRUNK_X_MAX, target_y=ANCHOR_Y,
                    kp_xy=0.8, max_speed=0.08, max_speed_fn=_stage2_max_speed,
                    per_step_fn=_hold_stage2_arm, abort_fn=_stage2_pose_broken,
                    hard_stop_on_condition=True,
                    label="STAGE2: 박스가 트렁크 입구를 완전히 넘을 때까지 전진(팔 자세 고정, 저속)",
                    debug_interval=5, debug_fn=_stage2_debug,
                )
            if aborted:
                print("[실패] STAGE 2 도중 자세 붕괴(충돌 의심)가 감지돼 즉시 중단했습니다 - "
                      "ENTRY_HOLDING_Z를 더 올리거나 진입 경로를 재검토하세요. STAGE 3으로 넘어가지 마세요.",
                      flush=True)
            elif not condition_met:
                print("[경고] 안전 상한(TRUNK_X_MAX)까지 갔는데도 박스가 입구를 못 넘었습니다 - "
                      "팔-섀시 오프셋/ENTRY_HOLDING_Z 재검토 필요.", flush=True)
            else:
                print("[성공] STAGE 2 - 박스가 자세 붕괴 없이 트렁크 입구를 넘었습니다.", flush=True)

            # 사용자 지적(STAGE 4 역순 재검토) - STAGE 4가 나중에 "STAGE 2가 끝난 바로 그 지점"으로
            # 정확히 되돌아가려면, 그 시점의 섀시/팔(둘 다 - 팔은 얼어붙은 채 섀시에 실려왔으므로
            # 섀시가 움직인 만큼 팔의 world 위치도 같이 움직여 있음) 위치를 실측해서 저장해둬야 한다 -
            # STAGE 1의 xy(_init_ee_pos)를 재사용하면 섀시가 실제로 멈춘 지점과 안 맞아 진짜 역순이
            # 아니게 된다.
            stage2_end_chassis_pos, _ = base_robot.get_world_pose()
            stage2_end_ee_pos, _ = m0609_robot.end_effector.get_world_pose()
            print(f"[STAGE2 체크포인트 저장] 섀시={np.round(stage2_end_chassis_pos, 3)} "
                  f"팔ee={np.round(stage2_end_ee_pos, 3)}", flush=True)
            _log_clearance("STAGE2 종료(입구 통과 직후)")

            chassis_pos0, _ = base_robot.get_world_pose()
            snapshot(eye=[chassis_pos0[0] - 1.5, chassis_pos0[1] - 2.2, chassis_pos0[2] + 1.4],
                     target=[float(chassis_pos0[0]), ANCHOR_Y, 0.5],
                     fname="_trunkplace_00b_close_approach.png")

            if STAGE < 3:
                print(f"\n[STAGE 2 완료] 확인용 스크린샷 저장 완료(성공={condition_met and not aborted}) - "
                      "STAGE=3으로 다시 실행하면 정밀 접근/PLACE까지 진행합니다.\n", flush=True)

        # 92번 원본은 STAGE 2가 실패해도(aborted=True 또는 condition_met=False) 경고만 찍고 그냥
        # STAGE 3.0으로 넘어간다(92번 자체의 동작이라 STAGE 2/3.0 코드는 안 건드림) - 대신 그 경계에
        # 이 가드만 새로 추가한다. STAGE 2가 실패한 상태로 이어진 STAGE 3.0 결과는 어차피 신뢰할 수
        # 없으므로(입구를 못 넘었거나 충돌 의심 상태), 여기서 멈춰서 GUI로 직접 확인하게 한다.
        if STAGE >= 3 and (aborted or not condition_met):
            pause_for_inspection(
                f"[중단] STAGE 2가 실패/미완주 상태입니다(자세붕괴={aborted}, 조건충족={condition_met}) - "
                "이 상태로는 STAGE 3.0 결과를 신뢰할 수 없어 여기서 멈춥니다."
            )

        if STAGE >= 3:
            # ================= STAGE 3.0: 전완 앞끝을 내부천장 시작점 바로 앞까지 전진(Z 고정) =================
            # 사용자 설계(STAGE 3 재설계 1차) - STAGE 3을 여러 세부 단계(3.0/3.1/3.2/3.3/3.4)로 쪼갠다.
            # 3.0은 STAGE 2와 완전히 같은 방식(팔을 STAGE 2 종료 자세로 완전히 얼린 채 섀시만 전진 -
            # STAGE 2가 이미 493스텝 무충돌로 증명한 안전 패턴 그대로 재사용)으로, "박스+그리퍼+link_6+
            # 전완(포락선)의 가장 앞쪽 x"가 내부 고정천장이 시작되는 지점(INTERNAL_CEILING_START_X)
            # 바로 앞까지 오도록 만든다. 아직 열린 트렁크 리드 밑면(높은 천장) 구간 안에서만 움직이므로
            # Z(높이)는 전혀 바꿀 필요가 없다 - 팔을 접어 내리는 건 3.1의 몫이다. 이렇게 하면 박스가
            # 트렁크 입구 X에서 충분히 멀어져 안전거리가 확보된 상태로 다음 단계를 시작할 수 있다.

            # 사용자 지시 - 접근 경계 확인용 마커는(콜리전 없는 순수 시각 마커, 실측으로 이미
            # 확인됨) 그 단계가 끝나면 항상 숨긴다 - 되돌리기 쉽게 삭제 대신 MakeInvisible()만
            # 쓴다. 여러 단계에서 재사용하도록 헬퍼로 뺀다.
            def _hide_markers(names):
                for _marker_name in names:
                    _marker_prim = stage.GetPrimAtPath(f"/World/{_marker_name}")
                    if _marker_prim.IsValid():
                        UsdGeom.Imageable(_marker_prim).MakeInvisible()

            # STAGE 2용 입구 마커(초록=EntrancePlane/노랑=SuccessPlane)는 이제 다 썼다 - 시야를 가린다.
            _hide_markers(["EntrancePlane", "SuccessPlane"])

            # STAGE 2와 같은 스타일의 확인용 마커 - 청록=내부천장이 시작되는 실측 지점
            # (INTERNAL_CEILING_START_X), 주황=3.0의 실제 정지 목표(안전마진 포함). 스크린샷에서
            # 전완 앞쪽이 이 마커들을 넘지 않았는지, 실제 천장 메시와 비교해 눈으로 확인할 수 있다.
            STAGE3_0_FRONT_MARGIN = 0.01  # 전완 앞쪽이 내부천장 시작점보다 이만큼 못 미쳐야 한다.
            _add_x_marker("CeilingStartPlane", INTERNAL_CEILING_START_X, (0.0, 1.0, 1.0))
            _add_x_marker("Stage3TargetPlane", INTERNAL_CEILING_START_X - STAGE3_0_FRONT_MARGIN, (1.0, 0.5, 0.0))

            # 실측 클리어런스(전완 앞쪽 x에서의 천장 여유)가 이 밑으로 떨어지면 즉시 중단 - 혹시
            # 목표 마진을 넘어서더라도 실제 천장에 닿기 전에 멈추기 위한 이중 안전장치(STAGE 2의
            # front_clear 판정과 같은 원리).
            STAGE3_0_CEILING_ABORT_MARGIN = 0.01
            # 사용자 지적(성능, 실측 렉 확인) - 예전 STAGE 3 설계가 abort_fn/condition_fn 안에서
            # measure_carry_envelope()/evaluate_pose_clearance()를 매 물리 스텝 그대로 호출해서 심한
            # 렉을 일으켰었다(그래서 그 설계 자체를 되돌렸었는데, 3.0을 새로 짜면서 똑같은 실수를
            # 반복했었다). measure_carry_envelope()는 4개 파츠의 모든 메시 정점을 Usd.PrimRange로
            # 순회하며 world로 변환하는 순수 Python 루프라 원래 무겁다 - 이걸 최대 300스텝(drive_until
            # 상한) 동안 매 스텝 다시 돌리면 감당이 안 된다. 그런데 이 단계는 팔이 STAGE 2 자세로
            # 완전히 얼어있어 섀시만 움직이므로, "전완 앞쪽 x - 섀시 x"와 "전완 최상단 z"는 시작부터
            # 끝까지 상수다 - 시작 시점에 딱 한 번만 무거운 측정을 하고, 루프 안에서는 섀시 위치에
            # 상수 오프셋만 더하는 값싼 연산으로 대체한다. 천장 raycast(ceiling_z_at, PhysX 네이티브
            # 쿼리 1회라 메시 순회보다 훨씬 가벼움)만 그것도 매 스텝이 아니라
            # STAGE3_0_CLEARANCE_CHECK_INTERVAL마다 한 번씩만 확인한다.
            STAGE3_0_CLEARANCE_CHECK_INTERVAL = 20

            # 사용자 실측 확인 - LiveEnvelopeFrontMarker(4파츠 결합 front_x)가 그리퍼/전완이 아니라
            # 홀로노믹 베이스 앞쪽에 잡혀있는 게 스크린샷으로 확인됐다. 4파츠 결합 대신 link_5(전완)
            # 하나만 기준으로 쓴다 - 사용자 결정.
            _LINK5_PATH = f"{m0609_path}/link_5"
            # 사용자 실측 확인(GUI 스크린샷) - 박스를 중심에서 왼쪽/오른쪽으로 옮길 때, 그
            # 방향으로 팔이 굽는 쪽의 상완(link_2, 어깨~팔꿈치)이 그리퍼/박스보다 더 바깥으로
            # 튀어나온다 - CARRY_ENVELOPE_PARTS/link_5 어느 쪽도 link_2를 측정하지 않는
            # 이 프로젝트의 기존 known gap(위 STAGE3.2.0 주석 "link_2가 반복적으로 입구에
            # 부딪히던 문제" 참고)이 실제로 재현된 것 - STAGE 3.3/3.4에서 실측해서 직접 잡는다.
            _LINK2_PATH = f"{m0609_path}/link_2"
            _stage3_0_chassis0, _ = base_robot.get_world_pose()
            _link5_min0, _link5_max0 = _mesh_world_aabb(_LINK5_PATH)
            if _link5_max0[0] is None:
                raise SystemExit(f"[중단] STAGE 3.0: {_LINK5_PATH} 메시를 찾지 못했습니다 - 경로를 확인하세요.")
            _stage3_0_front_offset = float(_link5_max0[0]) - float(_stage3_0_chassis0[0])
            _stage3_0_top_z = float(_link5_max0[2])
            print(f"[STAGE3.0 사전측정, 1회] 기준=link_5 섀시x={float(_stage3_0_chassis0[0]):.3f} "
                  f"link_5앞x={_link5_max0[0]:.3f} 오프셋={_stage3_0_front_offset:.3f} "
                  f"link_5상단z={_stage3_0_top_z:.3f}(팔이 얼어있는 동안 상수로 취급)", flush=True)

            # 자홍색 마커 - 지금 정지 조건이 실제로 보고 있는 link_5 앞x. 스크린샷에서 link_5(전완)
            # 끝에 있는지 바로 눈으로 확인 가능.
            _add_x_marker("LiveEnvelopeFrontMarker", _link5_max0[0], (1.0, 0.0, 1.0))
            _gripper_probe_pos, _ = m0609_robot.end_effector.get_world_pose()
            snapshot(eye=[float(_gripper_probe_pos[0]) - 0.8, float(_gripper_probe_pos[1]) - 1.2, float(_gripper_probe_pos[2]) + 0.6],
                     target=[float(_gripper_probe_pos[0]), float(_gripper_probe_pos[1]), float(_gripper_probe_pos[2])],
                     fname="_trunkplace_03_0_front_x_diag.png")

            def _stage3_0_front_x():
                chassis_pos, _ = base_robot.get_world_pose()
                return float(chassis_pos[0]) + _stage3_0_front_offset

            def _stage3_0_condition():
                return _stage3_0_front_x() >= INTERNAL_CEILING_START_X - STAGE3_0_FRONT_MARGIN

            _stage3_0_clearance_counter = {"n": 0}

            def _stage3_0_broken():
                _, _, box_center = _get_box_x_edges()
                y_broken = abs(float(box_center[1]) - ANCHOR_Y) > STAGE2_Y_TOLERANCE
                detached = not m0609_robot.gripper.is_closed()
                if y_broken or detached:
                    print(f"  [DIAG STAGE3.0] y_broken={y_broken} detached={detached}", flush=True)
                    return True
                _stage3_0_clearance_counter["n"] += 1
                if _stage3_0_clearance_counter["n"] % STAGE3_0_CLEARANCE_CHECK_INTERVAL != 0:
                    return False
                front_x = _stage3_0_front_x()
                ceiling_here = ceiling_z_at(front_x)
                if ceiling_here is None:
                    return False
                clearance = ceiling_here - _stage3_0_top_z
                broken = clearance < STAGE3_0_CEILING_ABORT_MARGIN
                if broken:
                    print(f"  [DIAG STAGE3.0] 천장clearance={clearance:.4f} front_x={front_x:.3f} "
                          f"ceiling={ceiling_here:.3f}", flush=True)
                return broken

            def _stage3_0_debug(step):
                chassis_pos, _ = base_robot.get_world_pose()
                front_x = _stage3_0_front_x()
                print(f"  [DEBUG STAGE3.0 step={step}] 섀시x={float(chassis_pos[0]):.3f} "
                      f"전완앞x={front_x:.3f} 목표={INTERNAL_CEILING_START_X - STAGE3_0_FRONT_MARGIN:.3f} "
                      f"전완상단z={_stage3_0_top_z:.3f}", flush=True)
                # 자홍색 마커를 계속 이 front_x로 옮겨서, 접근하는 동안에도 실제 그리퍼/전완 위치와
                # 계속 일치하는지(=오프셋 가정이 여전히 유효한지) 눈으로 추적할 수 있게 한다.
                _add_x_marker("LiveEnvelopeFrontMarker", front_x, (1.0, 0.0, 1.0))

            _, _, stage3_0_met, stage3_0_aborted = drive_until(
                _stage3_0_condition, target_x=TRUNK_X_MAX, target_y=ANCHOR_Y,
                kp_xy=0.8, max_speed=0.08, per_step_fn=_hold_stage2_arm,
                abort_fn=_stage3_0_broken, hard_stop_on_condition=True,
                label="STAGE3.0: 전완 앞끝이 내부천장 시작점 바로 앞까지 전진(팔 자세 고정, Z 유지)",
                debug_interval=10, debug_fn=_stage3_0_debug,
            )
            if stage3_0_aborted:
                pause_for_inspection("[중단] STAGE 3.0 도중 자세 붕괴/클리어런스 부족이 감지돼 즉시 중단했습니다.")
            if not stage3_0_met:
                pause_for_inspection(
                    "[중단] STAGE 3.0 - 안전 상한(TRUNK_X_MAX)까지 갔는데도 전완 앞끝이 목표에 도달하지 "
                    "못했습니다 - INTERNAL_CEILING_START_X/마진을 재검토하세요."
                )
            print("[성공] STAGE 3.0 - 전완 앞끝이 내부천장 시작점 바로 앞까지 자세 붕괴 없이 도달했습니다.",
                  flush=True)
            _log_clearance("STAGE3.0 종료(내부천장 시작점 직전)")

            # STAGE 4(역순 후퇴)용 체크포인트 - 92번과 동일(그대로 재사용).
            _stage3_0_end_chassis_x = float(base_robot.get_world_pose()[0][0])
            _stage3_0_end_ee_pos, _ = m0609_robot.end_effector.get_world_pose()
            _stage3_0_end_lift_h = lift_state["h"]

            chassis_pos0, _ = base_robot.get_world_pose()
            snapshot(eye=[chassis_pos0[0] - 1.2, chassis_pos0[1] - 2.0, chassis_pos0[2] + 1.3],
                     target=[INTERNAL_CEILING_START_X, ANCHOR_Y, TRUNK_FLOOR_Z + 0.3],
                     fname="_trunkplace_03_0_ceiling_start_approach.png")

            if STAGE < 3.1:
                print("\n[STAGE 3.0 완료] STAGE 3.1 이상으로 다시 실행하면 최종 배치까지 진행합니다.\n",
                      flush=True)

        if STAGE >= 3.1:
            # ================= STAGE 3.1: link_5(전완) 최상단이 로컬 천장보다 낮아지도록 팔을 접어
            # 하강(섀시는 그대로, Z만 변경) =================
            # 92.trunk_place_holonomic.py와 완전히 동일(그대로 재사용).
            # 사용자 지시 - STAGE 3.0용 마커는 이제 다 썼으니 숨긴다.
            _hide_markers(["CeilingStartPlane", "Stage3TargetPlane", "LiveEnvelopeFrontMarker"])
            STAGE3_1_CEILING_MARGIN = 0.01  # 노란 마커 = 천장(파랑) - 이 마진.
            STAGE3_1_FLOOR_MARGIN = 0.02    # 박스 바닥이 트렁크 바닥보다 이만큼은 위에 있어야 한다.
            STAGE3_1_MAX_ITERS = 6          # ee_z 하강량=link_5 하강량이라는 선형근사가 완벽하지 않으므로
                                             # 잔여 오차를 반복 보정한다(측정->하강->재측정).

            def _link5_top_z():
                _, _max = _mesh_world_aabb(_LINK5_PATH)
                return float(_max[2])

            def _link5_front_x():
                _, _max = _mesh_world_aabb(_LINK5_PATH)
                return float(_max[0])

            def _box_floor_clearance():
                box_rear_x, _, box_center = _get_box_x_edges()
                box_bottom_z = float(box_center[2]) - TEST_BOX_SIZE[2] / 2.0
                floor_here = floor_z_at(box_rear_x)
                if floor_here is None:
                    return None
                return box_bottom_z - floor_here

            def _add_z_marker(name, x, z, color, half_x=0.15, half_y=0.45, half_z=0.003):
                marker = UsdGeom.Cube.Define(stage, f"/World/{name}")
                marker.CreateSizeAttr(1.0)
                marker.CreateDisplayColorAttr([Gf.Vec3f(*color)])
                xform = UsdGeom.Xformable(marker)
                xform.ClearXformOpOrder()
                xform.AddTranslateOp().Set(Gf.Vec3d(x, 0.0, z))
                xform.AddScaleOp().Set(Gf.Vec3f(half_x, half_y, half_z))

            _stage3_1_front_x0 = _link5_front_x()
            _stage3_1_ceiling_here = CEILING_WORLD_Z
            _stage3_1_target_top_z = _stage3_1_ceiling_here - STAGE3_1_CEILING_MARGIN
            print(f"[STAGE3.1 사전측정] link_5앞x(지금)={_stage3_1_front_x0:.3f} "
                  f"천장(CEILING_WORLD_Z)={_stage3_1_ceiling_here:.3f} "
                  f"목표(link_5상단)={_stage3_1_target_top_z:.3f} "
                  f"현재link_5상단={_link5_top_z():.3f}", flush=True)

            _add_z_marker("CeilingHeightMarker", _stage3_1_front_x0, _stage3_1_ceiling_here, (0.2, 0.4, 1.0))
            _add_z_marker("Stage3_1TargetMarker", _stage3_1_front_x0, _stage3_1_target_top_z, (1.0, 1.0, 0.0))
            _add_z_marker("LiveLink5TopMarker", _stage3_1_front_x0, _link5_top_z(), (1.0, 0.0, 1.0))

            if _link5_top_z() <= _stage3_1_target_top_z:
                print("[STAGE 3.1] 이미 link_5 최상단이 목표보다 낮습니다 - 하강 없이 통과.", flush=True)
            else:
                _ee_pos0, _ = m0609_robot.end_effector.get_world_pose()
                _stage3_1_ee_xy = (float(_ee_pos0[0]), float(_ee_pos0[1]))
                for _iter in range(STAGE3_1_MAX_ITERS):
                    current_top = _link5_top_z()
                    gap = current_top - _stage3_1_target_top_z
                    if gap <= 0:
                        print(f"[STAGE 3.1] {_iter}회 보정 후 목표 도달(link_5상단={current_top:.3f}).",
                              flush=True)
                        break
                    clearance = _box_floor_clearance()
                    if clearance is not None and clearance < STAGE3_1_FLOOR_MARGIN:
                        pause_for_inspection(
                            f"[중단] STAGE 3.1: 박스 바닥이 트렁크 바닥에 너무 가까워졌습니다"
                            f"(여유={clearance:.3f}m) - 더 내리면 바닥 충돌 위험이라 멈춥니다."
                        )
                    ee_pos_now, _ = m0609_robot.end_effector.get_world_pose()
                    target_ee_z = float(ee_pos_now[2]) - gap
                    target_lift_h = max(LIFT_MIN, lift_state["h"] - gap)
                    descend_and_raise_lift(
                        _stage3_1_ee_xy, target_ee_z, target_lift_h, steps=120, hold_gripper_closed=True,
                        label=f"STAGE3.1[{_iter + 1}/{STAGE3_1_MAX_ITERS}]: link_5 상단 하강(잔여 {gap:.3f}m)",
                    )
                    _add_z_marker("LiveLink5TopMarker", _link5_front_x(), _link5_top_z(), (1.0, 0.0, 1.0))
                else:
                    print(f"[경고] STAGE 3.1: {STAGE3_1_MAX_ITERS}회 반복해도 link_5 상단이 목표에 도달하지 "
                          f"못했습니다(현재={_link5_top_z():.3f}, 목표={_stage3_1_target_top_z:.3f}).", flush=True)

            _stage3_1_final_clearance = _box_floor_clearance()
            print(f"[STAGE 3.1 종료] link_5상단={_link5_top_z():.3f} 목표={_stage3_1_target_top_z:.3f} "
                  f"박스-바닥여유={_stage3_1_final_clearance}", flush=True)
            _log_clearance("STAGE3.1 종료(전완 접어 하강 후)")

            STAGE3_1_END_LIFT_H = lift_state["h"]

            chassis_pos0, _ = base_robot.get_world_pose()
            snapshot(eye=[chassis_pos0[0] - 1.0, chassis_pos0[1] - 1.8, chassis_pos0[2] + 1.0],
                     target=[_stage3_1_front_x0, ANCHOR_Y, TRUNK_FLOOR_Z + 0.3],
                     fname="_trunkplace_03_1_folded_low.png")

            if STAGE < 3.2:
                print("\n[STAGE 3.1 완료] STAGE 3.2 이상으로 다시 실행하면 다음 단계로 진행합니다.\n", flush=True)

        if STAGE >= 3.2:
            # ================= STAGE 3.2.0: 그리퍼/박스 위치 고정 + 섀시 후진/리프트 상승으로 팔 펴기 =================
            # 92.trunk_place_holonomic.py와 거의 동일 - 리프트 목표만 다르다(사용자 지시
            # "리프트 좀 더 올리자" - 2번째 박스(반전 branch) STAGE3.4 최종 하강 도중 link_2가
            # 트렁크 입구 프레임에 부딪혀 IK가 발산한 실측 확인 이후).
            # 사용자 지시 - STAGE 3.1용 마커는 이제 다 썼으니 숨긴다.
            _hide_markers(["CeilingHeightMarker", "Stage3_1TargetMarker", "LiveLink5TopMarker"])
            # 사용자 실측 확인(2번째 박스, STAGE4-4에서 ee가 고정 목표에서 0.277m 벗어남,
            # 자세붕괴=False - 즉 조용히 딴 곳으로 수렴한 것) - 원인: joint_3(어깨~팔꿈치를
            # 잇는 축)이 0에 너무 가깝게(구 tolerance=0.15rad≈8.6도, 실측 정지값 -0.121rad
            # ≈6.9도) 펴진 채로 STAGE3.2.0을 마쳤다. joint_3=0 근방은 팔꿈치가 "쭉 편" 특이
            # 자세라 elbow-up/elbow-down 두 solution branch가 사실상 붙어있는 경계다 - RMPflow는
            # 전역 IK가 아니라 반응형(reactive) 솔버라 이 경계를 매끄럽게 못 건너간다는 걸
            # 이 프로젝트에서 반복 확인했다(솔루션 스페이스 관련 이전 커밋들 참고). STAGE4-4가
            # 이 자세를 거꾸로 되짚어 다시 굽히는 과정에서 반대쪽 branch로 넘어가버려, 접히는
            # 방향이 원래와 달라진 팔이 트렁크와 부딪혔다(사용자 진단 - "관절이 아래로 꺾이면서
            # 다른 솔루션 스페이스로 접힘").
            # 고침 - tolerance를 0.35rad(~20도)로 넉넉히 키운다. STAGE3.2.0의 "펴기" 정지
            # 조건(_stage3_2_0_joint_flat)이 이 값을 그대로 쓰므로, 이제 joint_3이 0에서
            # 최소 20도는 떨어진 채로(원래 접힌 각도 -90~100도 근처에서 시작해 20도까지만
            # 펴짐 - 여전히 대부분의 "쭉 펴기" 효과는 유지) 멈춘다 - STAGE4-4가 되짚어갈 때도
            # 이 여유 안에서만 움직이므로 경계를 건널 일이 없다. STAGE3.2.1 이후 단계는 전부
            # STAGE3.2.0이 실제로 남긴 자세를 매 스텝 실측해서 계산하므로(하드코딩 없음)
            # 이 값이 바뀌어도 별도 수정 없이 그대로 적응한다.
            STAGE3_2_0_FLAT_JOINT_TOLERANCE = 0.35  # rad(~20.1도) - joint_3이 이 안이면 "폈다"로 본다.
            STAGE3_2_0_RETREAT_X = BASE_START_XY[0]  # 안전 상한 - 최초 대기 위치까지만 후진 허용.
            # 사용자 설계(5차, PLACE_LIFT_MAX 정의부 주석 참고) - "팔이 큰 낙차+수평 reach를
            # 동시에 감당하는 자세에서 팔꿈치/팔뚝이 트렁크 입구 프레임을 스친다"는 문제를
            # 이미 문서화해뒀고, 그 해법으로 "리프트로 마운트 자체를 목표 높이 가까이 올려
            # 팔이 작은 나머지 거리만 커버하게" 하는 PLACE_LIFT_MAX(천장 안전한계까지 클램프된
            # 값, 92번 기준 LIFT_MAX+0.2=0.588보다 높은 0.650)를 미리 계산해뒀었다 - 그런데
            # 실제로 이 STAGE3.2.0의 목표에는 연결이 안 돼 있었다(계산만 해두고 안 쓰던 상태).
            # 지금 실측(2번째 박스 STAGE3.4 link_2 충돌)이 정확히 그 문서화된 문제와 일치해서,
            # 이제 실제로 연결한다 - PLACE_LIFT_MAX 자체가 이미 천장 여유(SAFE_TRANSIT_Z-0.05)로
            # 클램프돼 있으므로 이 값을 그대로 써도 안전하다.
            STAGE3_2_0_LIFT_TARGET = PLACE_LIFT_MAX

            _stage3_2_0_fixed_ee, _ = m0609_robot.end_effector.get_world_pose()
            _stage3_2_0_fixed_ee = tuple(float(v) for v in _stage3_2_0_fixed_ee)
            print(f"[STAGE3.2.0 사전측정] 고정 ee 목표={np.round(_stage3_2_0_fixed_ee, 3)} "
                  f"후진목표x={STAGE3_2_0_RETREAT_X:.3f} 리프트목표={STAGE3_2_0_LIFT_TARGET:.3f} "
                  f"평탄판정오차={STAGE3_2_0_FLAT_JOINT_TOLERANCE:.3f}rad", flush=True)

            def _stage3_2_0_joint_flat():
                q = np.asarray(m0609_robot.get_joint_positions(), dtype=float)
                return abs(q[2]) < STAGE3_2_0_FLAT_JOINT_TOLERANCE

            def _stage3_2_0_broken():
                _, _, box_center = _get_box_x_edges()
                y_broken = abs(float(box_center[1]) - ANCHOR_Y) > STAGE2_Y_TOLERANCE
                detached = not m0609_robot.gripper.is_closed()
                if y_broken or detached:
                    print(f"  [DIAG STAGE3.2.0] y_broken={y_broken} detached={detached}", flush=True)
                    return True
                return False

            def _stage3_2_0_debug(step):
                q = np.asarray(m0609_robot.get_joint_positions(), dtype=float)
                chassis_pos, _ = base_robot.get_world_pose()
                ee_pos, _ = m0609_robot.end_effector.get_world_pose()
                print(f"  [DEBUG STAGE3.2.0 step={step}] 섀시x={float(chassis_pos[0]):.3f} "
                      f"리프트={lift_state['h']:.3f} joint2={q[1]:.3f} joint3={q[2]:.3f} "
                      f"ee=({ee_pos[0]:.3f},{ee_pos[1]:.3f},{ee_pos[2]:.3f})", flush=True)

            # 사용자 질문("Link3이 아래로 접히는 자세, RMPflow라서 방향 제어 못 하나?") 대응 -
            # 이 정방향 통과(펴기)의 실제 관절 궤적을 리프트 높이를 키로 기록해둔다. STAGE4-4
            # (이 단계의 역방향, 접기)에서 RMPflow에게 다시 풀게 하는 대신 이 기록을 그대로
            # 재생하면, 이번에 실제로 밟은 solution branch를 후퇴할 때도 그대로 되짚으므로
            # 다른 branch로 튈 여지가 없어진다(retreat_and_raise의 joint_trajectory 문서 참고).
            _stage3_2_0_joint_record = []

            _, _stage3_2_0_ee_final, _stage3_2_0_ee_err, stage3_2_0_met, stage3_2_0_aborted = retreat_and_raise(
                STAGE3_2_0_RETREAT_X, STAGE3_2_0_LIFT_TARGET, _stage3_2_0_fixed_ee,
                condition_fn=_stage3_2_0_joint_flat, max_speed=0.06, lift_speed=0.002,
                hold_gripper_closed=True, abort_fn=_stage3_2_0_broken, hard_stop_on_condition=True,
                label="STAGE3.2.0: 그리퍼 위치 고정 + 섀시 후진/리프트 상승(팔 펴기)",
                debug_interval=10, debug_fn=_stage3_2_0_debug, record_into=_stage3_2_0_joint_record,
            )
            # (heights_array, joints_matrix) 형태로 미리 정리해둔다 - np.interp가 요구하는
            # x축(heights)이 오름차순이어야 하므로 정렬한다(리프트가 항상 증가하는 방향으로만
            # 움직이므로 이미 정렬돼 있을 것이지만, 부동소수점 동률/역전에 대비해 명시적으로
            # 정렬한다). 최소 2개 표본이 없으면(펴기 시작 즉시 조건이 충족돼 한 스텝도 기록
            # 못 한 극단적인 경우) 재생 자체를 못 하므로 None으로 둔다 - STAGE4-4가 이 경우
            # 자동으로 기존 RMPflow 방식으로 되돌아간다(아래 참고).
            if len(_stage3_2_0_joint_record) >= 2:
                _stage3_2_0_heights = np.array([h for h, _ in _stage3_2_0_joint_record])
                _stage3_2_0_joints_matrix = np.array([q for _, q in _stage3_2_0_joint_record])
                _sort_idx = np.argsort(_stage3_2_0_heights)
                _stage3_2_0_joint_trajectory = (
                    _stage3_2_0_heights[_sort_idx], _stage3_2_0_joints_matrix[_sort_idx])
            else:
                _stage3_2_0_joint_trajectory = None
            if stage3_2_0_aborted:
                pause_for_inspection("[중단] STAGE 3.2.0 도중 자세 붕괴가 감지돼 즉시 중단했습니다.")
            if _stage3_2_0_ee_err > 0.05:
                pause_for_inspection(
                    f"[중단] STAGE 3.2.0 - 그리퍼가 고정 목표에서 너무 벗어났습니다(err={_stage3_2_0_ee_err:.3f}m)."
                )
            _stage3_2_0_final_chassis, _ = base_robot.get_world_pose()
            print(f"[STAGE3.2.0 종료] 조건충족(관절이 폈는지)={stage3_2_0_met} "
                  f"섀시x={float(_stage3_2_0_final_chassis[0]):.3f} 리프트={lift_state['h']:.3f}", flush=True)

            chassis_pos0, _ = base_robot.get_world_pose()
            snapshot(eye=[chassis_pos0[0] - 1.2, chassis_pos0[1] - 2.0, chassis_pos0[2] + 1.2],
                     target=[_stage3_2_0_fixed_ee[0], ANCHOR_Y, _stage3_2_0_fixed_ee[2]],
                     fname="_trunkplace_03_2_0_arm_flattened.png")

            # ================= STAGE 3.2.1: 홀로노믹 베이스로 적재 X까지 접근(팔은 3.2.0에서 편 자세로 고정) =================
            STAGE3_2_1_TARGET_X = place_world_xy[0]
            STAGE3_2_1_SURFACE_SCAN_ZS = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45]
            STAGE3_2_1_APPROACH_SAFETY_MARGIN = 0.05  # 리프트 반지름(0.045m) + 약간의 여유
            STAGE3_2_1_CEILING_SCAN_STEP = 0.02
            STAGE3_2_1_CEILING_MARGIN = STAGE3_1_CEILING_MARGIN
            STAGE3_2_1_FLOOR_MARGIN = STAGE3_1_FLOOR_MARGIN
            STAGE3_2_1_Y_TOLERANCE = STAGE2_Y_TOLERANCE

            _stage3_2_1_surface_xs = [vehicle_rear_surface_x_at(ANCHOR_Y, z) for z in STAGE3_2_1_SURFACE_SCAN_ZS]
            _stage3_2_1_surface_xs = [x for x in _stage3_2_1_surface_xs if x is not None]
            if _stage3_2_1_surface_xs:
                _stage3_2_1_surface_x = min(_stage3_2_1_surface_xs)
            else:
                _stage3_2_1_surface_x = TRUNK_ENTRANCE_X - 0.10
                print("[경고] STAGE 3.2.1: 차체 표면 raycast가 하나도 안 잡혀서 가설값으로 대체합니다.", flush=True)
            _stage3_2_1_approach_limit_x = _stage3_2_1_surface_x - STAGE3_2_1_APPROACH_SAFETY_MARGIN  # 리프트앞끝 기준

            _stage3_2_1_chassis0, _ = base_robot.get_world_pose()
            _, _, _stage3_2_1_box_center0 = _get_box_x_edges()
            _stage3_2_1_box_offset = float(_stage3_2_1_box_center0[0]) - float(_stage3_2_1_chassis0[0])
            _stage3_2_1_naive_target_chassis_x = STAGE3_2_1_TARGET_X - _stage3_2_1_box_offset
            _stage3_2_1_target_chassis_x = min(_stage3_2_1_naive_target_chassis_x,
                                                _stage3_2_1_approach_limit_x - LIFT_COLUMN_RADIUS)
            _stage3_2_1_target_box_x = _stage3_2_1_target_chassis_x + _stage3_2_1_box_offset
            print(f"[STAGE3.2.1 사전측정] 목표 박스x={STAGE3_2_1_TARGET_X:.3f} "
                  f"차체표면x(실측)={_stage3_2_1_surface_x:.3f} 접근한계(리프트앞x)={_stage3_2_1_approach_limit_x:.3f} "
                  f"필요섀시x={_stage3_2_1_naive_target_chassis_x:.3f} -> 실제목표섀시x={_stage3_2_1_target_chassis_x:.3f}",
                  flush=True)

            _scan_lo, _scan_hi = sorted([float(_stage3_2_1_box_center0[0]), _stage3_2_1_target_box_x])
            _stage3_2_1_scan_xs = np.arange(_scan_lo, _scan_hi + 1e-9, STAGE3_2_1_CEILING_SCAN_STEP)
            _stage3_2_1_scan_ceilings = [c for c in (ceiling_z_at(x) for x in _stage3_2_1_scan_xs) if c is not None]
            if not _stage3_2_1_scan_ceilings:
                pause_for_inspection("[중단] STAGE 3.2.1: 목표 구간의 천장 실측(raycast)이 하나도 안 잡힙니다.")
            _stage3_2_1_min_ceiling = min(_stage3_2_1_scan_ceilings)
            _stage3_2_1_target_top_z = _stage3_2_1_min_ceiling - STAGE3_2_1_CEILING_MARGIN

            _stage3_2_1_env0 = measure_carry_envelope()
            _stage3_2_1_slack = _stage3_2_1_target_top_z - _stage3_2_1_env0["top_z"]
            print(f"[STAGE3.2.1 천장 사전스캔] x={_scan_lo:.3f}~{_scan_hi:.3f} "
                  f"({len(_stage3_2_1_scan_ceilings)}개 표본) 최저천장={_stage3_2_1_min_ceiling:.3f} "
                  f"포락선상단 목표={_stage3_2_1_target_top_z:.3f} 현재포락선상단={_stage3_2_1_env0['top_z']:.3f} "
                  f"여유={_stage3_2_1_slack:.3f}", flush=True)
            if _stage3_2_1_slack > 0.005:
                _ee_now, _ = m0609_robot.end_effector.get_world_pose()
                # 사용자 지시("리프트 좀 더 올리자") 관련 - 이 상한도 STAGE3.2.0과 같은 이유로
                # LIFT_MAX(0.388, under-car 캡)가 아니라 PLACE_LIFT_MAX(트렁크 천장 안전한계)를
                # 써야 한다. STAGE3.2.0이 이미 리프트를 PLACE_LIFT_MAX까지 올려둔 뒤라
                # lift_state["h"]가 LIFT_MAX보다 항상 높으므로, 예전 코드는
                # LIFT_MAX-lift_state["h"]가 항상 음수가 돼 이 "천장 여유만큼 추가 상승" 자체가
                # 사실상 죽은 코드였다(_raise_amount>0.001을 절대 못 만족).
                _raise_amount = min(_stage3_2_1_slack, PLACE_LIFT_MAX - lift_state["h"])
                if _raise_amount > 0.001:
                    descend_and_raise_lift(
                        (float(_ee_now[0]), float(_ee_now[1])), float(_ee_now[2]) + _raise_amount,
                        lift_state["h"] + _raise_amount, steps=100,
                        label="STAGE3.2.1: 천장 여유만큼 리프트 추가 상승(전체를 그대로 들어올림)",
                    )
                    print(f"[STAGE3.2.1] 천장 여유 활용 - 리프트 {_raise_amount:.3f}m 추가 상승", flush=True)

            _stage3_2_1_hold_q = np.asarray(m0609_robot.get_joint_positions(), dtype=float).copy()

            def _hold_stage3_2_1_arm():
                m0609_robot.apply_action(ArticulationAction(joint_positions=_stage3_2_1_hold_q))
                m0609_robot.gripper.close()

            _stage3_2_1_top0 = measure_carry_envelope()["top_z"]
            _stage3_2_1_clearance_counter = {"n": 0}

            def _stage3_2_1_broken():
                _, _, box_center = _get_box_x_edges()
                y_broken = abs(float(box_center[1]) - ANCHOR_Y) > STAGE3_2_1_Y_TOLERANCE
                detached = not m0609_robot.gripper.is_closed()
                if y_broken or detached:
                    print(f"  [DIAG STAGE3.2.1] y_broken={y_broken} detached={detached}", flush=True)
                    return True
                floor_clear = _box_floor_clearance()
                if floor_clear is not None and floor_clear < STAGE3_2_1_FLOOR_MARGIN:
                    print(f"  [DIAG STAGE3.2.1] 바닥여유={floor_clear:.4f} < 마진 - 중단", flush=True)
                    return True
                _stage3_2_1_clearance_counter["n"] += 1
                if _stage3_2_1_clearance_counter["n"] % 10 != 0:
                    return False
                _, box_front_x, _ = _get_box_x_edges()
                ceiling_here = ceiling_z_at(box_front_x)
                if ceiling_here is None:
                    return False
                ceiling_margin_now = ceiling_here - _stage3_2_1_top0
                if ceiling_margin_now < STAGE3_2_1_CEILING_MARGIN:
                    print(f"  [DIAG STAGE3.2.1] 천장여유={ceiling_margin_now:.4f} < 마진 - 중단", flush=True)
                    return True
                return False

            def _stage3_2_1_debug(step):
                chassis_pos, chassis_quat = base_robot.get_world_pose()
                chassis_yaw = float(np.degrees(quat_to_euler_angles(chassis_quat)[2]))
                _, box_front_x, box_center = _get_box_x_edges()
                print(f"  [DEBUG STAGE3.2.1 step={step}] 섀시x={float(chassis_pos[0]):.3f} "
                      f"섀시y={float(chassis_pos[1]):.3f} 섀시yaw={chassis_yaw:.1f}deg "
                      f"박스중심x={box_center[0]:.3f} 목표={_stage3_2_1_target_box_x:.3f} "
                      f"박스중심y={float(box_center[1]):.4f}(ANCHOR_Y={ANCHOR_Y:.3f})", flush=True)
                _add_x_marker("Stage3_2_1TargetPlane", _stage3_2_1_target_box_x, (1.0, 1.0, 0.0))
                _add_x_marker("LiveBoxXMarker", box_center[0], (1.0, 0.0, 1.0))

            # 사용자 실측 확인(2번째 박스, solution space 2/반전 branch) - STAGE3.2.0(그리퍼
            # 위치를 RMPflow로 직접 고정)은 자체 y_broken 검사를 통과했는데, 바로 다음인 이
            # STAGE3.2.1(팔 관절만 얼리고 섀시만 이동)에서 y_broken으로 중단됐다 - 즉 드리프트가
            # "STAGE1.9 보정이 낡아서"가 아니라 이 구간 자체에서 새로 생겼다는 뜻이다. 원인 추정:
            # 이 구간은 target_yaw_deg를 안 줘서 drive_until이 시작 시점 yaw를 그대로 목표로
            # 쓰는데, kp_yaw가 함수 기본값(0.25)인 채로 남아있었다 - eyaw(도)*kp_yaw를 다시
            # radians()로 변환하는 계산식 특성상 각도오차가 작을수록(예: 1도 미만) 복원 각속도가
            # 거의 0에 가까워져서, 지속적인 외란(3.2.0에서 막 펴진 팔의 무게가 만드는 요잉
            # 토크 - 반전 branch는 무게 분포가 표준 branch와 달라 이 토크 크기/방향도 다를
            # 것으로 보인다)이 있으면 작은 정상상태 yaw 오차가 계속 남는다. 팔이 이미 3.2.0에서
            # 완전히 펴진 상태라 섀시 중심<->그리퍼 사이 거리(지렛대 팔)가 길어, 그 작은 yaw
            # 오차만으로도 박스 Y가 눈에 띄게 틀어진다(지렛대팔*sin(오차)). kp_xy는 이미 이
            # 구간에서 0.8로(함수 기본값 1.8보다 오히려 낮춤 - 정밀 접근을 위한 감속) 튜닝돼
            # 있었지만 kp_yaw는 손대지 않았었다 - 여기서만 4배로 올려 정상상태 yaw 오차를
            # 줄인다(max_wz는 그대로라 폭주 위험은 없다 - 작은 오차 영역의 복원력만 커진다).
            _, _, stage3_2_1_condition_met, stage3_2_1_aborted = drive_until(
                lambda: False, target_x=_stage3_2_1_target_chassis_x, target_y=float(_stage3_2_1_chassis0[1]),
                tolerance_xy=0.005, kp_xy=0.8, kp_yaw=1.0, max_speed=0.08, per_step_fn=_hold_stage3_2_1_arm,
                abort_fn=_stage3_2_1_broken, hard_stop_on_condition=True,
                label="STAGE3.2.1: 홀로노믹 베이스로 적재 X까지 접근(팔 자세 고정)",
                debug_interval=10, debug_fn=_stage3_2_1_debug,
            )
            if stage3_2_1_aborted:
                pause_for_inspection("[중단] STAGE 3.2.1 도중 자세 붕괴/클리어런스 부족이 감지돼 즉시 중단했습니다.")
            _, _, _stage3_2_1_final_box_center = _get_box_x_edges()
            print(f"[STAGE3.2.1 종료] 박스중심x={_stage3_2_1_final_box_center[0]:.3f} "
                  f"목표={_stage3_2_1_target_box_x:.3f}(원래 목표={STAGE3_2_1_TARGET_X:.3f})", flush=True)
            _log_clearance("STAGE3.2.1 종료(적재 X 접근 후)")

            chassis_pos0, _ = base_robot.get_world_pose()
            snapshot(eye=[chassis_pos0[0] - 0.8, chassis_pos0[1] - 1.6, chassis_pos0[2] + 0.9],
                     target=[_stage3_2_1_target_box_x, ANCHOR_Y, TRUNK_FLOOR_Z + 0.3],
                     fname="_trunkplace_03_2_1_approach_x.png")

            if STAGE < 3.3:
                print("\n[STAGE 3.2.1 완료] STAGE 3.3 이상으로 다시 실행하면 다음 단계로 진행합니다.\n", flush=True)

        if STAGE >= 3.3:
            # ================= STAGE 3.3: 홀로노믹 베이스 X/Y 동시 이동 + 매니퓰레이터 능동 추종으로 정렬 =================
            # 92.trunk_place_holonomic.py와 완전히 동일(그대로 재사용).
            # 사용자 지시 - STAGE 3.2.1용 마커는 이제 다 썼으니 숨긴다.
            _hide_markers(["Stage3_2_1TargetPlane", "LiveBoxXMarker"])
            STAGE3_3_TARGET_Y = place_world_xy[1]
            STAGE3_3_TARGET_X = place_world_xy[0]
            STAGE3_3_CEILING_MARGIN = STAGE3_1_CEILING_MARGIN
            STAGE3_3_FLOOR_MARGIN = STAGE3_1_FLOOR_MARGIN
            STAGE3_3_SIDE_MARGIN = 0.02  # 사용자 지시 - 좌우(휠하우스/내벽) 실시간 raycast 여유.

            _stage3_3_chassis0, _ = base_robot.get_world_pose()
            _, _, _stage3_3_box_center0 = _get_box_x_edges()
            _ee_now_3_3, _ = m0609_robot.end_effector.get_world_pose()

            _stage3_3_surface_xs = [vehicle_rear_surface_x_at(STAGE3_3_TARGET_Y, z) for z in STAGE3_2_1_SURFACE_SCAN_ZS]
            _stage3_3_surface_xs = [x for x in _stage3_3_surface_xs if x is not None]
            _stage3_3_surface_x = min(_stage3_3_surface_xs) if _stage3_3_surface_xs else _stage3_2_1_surface_x
            _stage3_3_approach_limit_x = _stage3_3_surface_x - STAGE3_2_1_APPROACH_SAFETY_MARGIN

            _stage3_3_ee_box_x_offset = float(_stage3_3_box_center0[0]) - float(_ee_now_3_3[0])
            _stage3_3_ee_box_y_offset = float(_stage3_3_box_center0[1]) - float(_ee_now_3_3[1])
            _stage3_3_target_ee = (
                STAGE3_3_TARGET_X - _stage3_3_ee_box_x_offset,
                STAGE3_3_TARGET_Y - _stage3_3_ee_box_y_offset,
                float(_ee_now_3_3[2]),
            )
            _stage3_3_chassis_box_x_offset = float(_stage3_3_box_center0[0]) - float(_stage3_3_chassis0[0])
            _stage3_3_chassis_box_y_offset = float(_stage3_3_box_center0[1]) - float(_stage3_3_chassis0[1])
            _stage3_3_naive_chassis_x = STAGE3_3_TARGET_X - _stage3_3_chassis_box_x_offset
            _stage3_3_chassis_target_x = min(_stage3_3_naive_chassis_x, _stage3_3_approach_limit_x - LIFT_COLUMN_RADIUS)
            _stage3_3_chassis_target_y = STAGE3_3_TARGET_Y - _stage3_3_chassis_box_y_offset
            print(f"[STAGE3.3 목표] 현재박스=({float(_stage3_3_box_center0[0]):.3f},{float(_stage3_3_box_center0[1]):.3f}) "
                  f"목표=({STAGE3_3_TARGET_X:.3f},{STAGE3_3_TARGET_Y:.3f}) "
                  f"목표Y에서 차체표면={_stage3_3_surface_x:.3f} 접근한계={_stage3_3_approach_limit_x:.3f} "
                  f"섀시목표=({_stage3_3_chassis_target_x:.3f},{_stage3_3_chassis_target_y:.3f}) "
                  f"ee목표={np.round(_stage3_3_target_ee, 3)}", flush=True)

            def _add_y_marker(name, y, color, half_x=0.45, half_y_thickness=0.003, half_z=0.15):
                marker = UsdGeom.Cube.Define(stage, f"/World/{name}")
                marker.CreateSizeAttr(1.0)
                marker.CreateDisplayColorAttr([Gf.Vec3f(*color)])
                xform = UsdGeom.Xformable(marker)
                xform.ClearXformOpOrder()
                xform.AddTranslateOp().Set(Gf.Vec3d(STAGE3_3_TARGET_X, y, 0.86))
                xform.AddScaleOp().Set(Gf.Vec3f(half_x, half_y_thickness, half_z))

            _add_y_marker("Stage3_3TargetPlane", STAGE3_3_TARGET_Y, (1.0, 1.0, 0.0))
            _add_x_marker("Stage3_3TargetXPlane", STAGE3_3_TARGET_X, (1.0, 1.0, 0.0))
            _add_y_marker("LiveBoxYMarker", float(_stage3_3_box_center0[1]), (1.0, 0.0, 1.0))

            _stage3_3_clearance_counter = {"n": 0}

            def _stage3_3_broken():
                detached = not m0609_robot.gripper.is_closed()
                if detached:
                    print("  [DIAG STAGE3.3] detached=True", flush=True)
                    return True
                _, box_front_x, box_center = _get_box_x_edges()
                floor_clear = _box_floor_clearance()
                if floor_clear is not None and floor_clear < STAGE3_3_FLOOR_MARGIN:
                    print(f"  [DIAG STAGE3.3] 바닥여유={floor_clear:.4f} < 마진 - 중단", flush=True)
                    return True
                _stage3_3_clearance_counter["n"] += 1
                if _stage3_3_clearance_counter["n"] % 10 != 0:
                    return False
                ceiling_here = ceiling_z_at(box_front_x)
                if ceiling_here is not None:
                    env_top = _link5_top_z()
                    ceiling_margin_now = ceiling_here - env_top
                    if ceiling_margin_now < STAGE3_3_CEILING_MARGIN:
                        print(f"  [DIAG STAGE3.3] 천장여유={ceiling_margin_now:.4f} < 마진 - 중단", flush=True)
                        return True
                # 사용자 실측 확인("왼쪽 휠하우스는 적용 안 된 것 같다 - 안 멈추는데?") - 지금까지
                # 위/아래(천장/바닥) 클리어런스만 봤고 좌우(휠하우스/내벽) 방향은 아무도 확인하지
                # 않았다 - trunk_map.json은 좌우 비대칭 돌출을 담을 수 없는 단일 flat AABB라
                # 이 정보 자체가 없다(위 interior_side_wall_y_at 정의부 참고). Y로 다가가는
                # 방향으로 실시간 raycast를 쏴서 박스 앞쪽(진행 방향) 모서리가 실제 벽/휠하우스를
                # 넘지 않는지 확인한다.
                _side_dir = 1.0 if STAGE3_3_TARGET_Y >= float(box_center[1]) else -1.0
                _side_wall_y = interior_side_wall_y_at(float(box_center[0]), float(box_center[2]), _side_dir)
                if _side_wall_y is not None:
                    _box_half_y = float(TEST_BOX_SIZE[1]) / 2.0
                    _box_leading_edge_y = float(box_center[1]) + _side_dir * _box_half_y
                    _side_margin_now = _side_dir * (_side_wall_y - _box_leading_edge_y)
                    if _side_margin_now < STAGE3_3_SIDE_MARGIN:
                        print(f"  [DIAG STAGE3.3] 좌우여유={_side_margin_now:.4f} < 마진(방향={_side_dir:+.0f}) - "
                              f"중단", flush=True)
                        return True

                # 사용자 실측 확인(GUI 스크린샷) - 박스/그리퍼가 아니라 link_2(상완, 어깨~팔꿈치)가
                # 진행 방향으로 더 튀어나와 있어서 부딪히는 경우가 있다 - 이 프로젝트의 기존
                # known gap(link_2 전용 충돌 측정 없음)이 실제로 재현된 것. link_2의 실측 세계
                # 좌표 AABB로 같은 방식의 좌우 여유를 직접 확인한다(박스처럼 대칭 반폭을 가정할
                # 수 없어 - 실제로 굽은 방향으로만 비대칭하게 튀어나오므로 - 그 방향의 실측
                # 모서리(min/max)를 그대로 쓴다).
                _link2_min, _link2_max = _mesh_world_aabb(_LINK2_PATH)
                if _link2_max[1] is not None:
                    _link2_leading_y = _link2_max[1] if _side_dir > 0 else _link2_min[1]
                    _link2_x = (float(_link2_min[0]) + float(_link2_max[0])) / 2.0
                    _link2_z = (float(_link2_min[2]) + float(_link2_max[2])) / 2.0
                    _link2_wall_y = interior_side_wall_y_at(_link2_x, _link2_z, _side_dir)
                    if _link2_wall_y is not None:
                        _link2_margin_now = _side_dir * (_link2_wall_y - float(_link2_leading_y))
                        if _link2_margin_now < STAGE3_3_SIDE_MARGIN:
                            print(f"  [DIAG STAGE3.3] link_2 좌우여유={_link2_margin_now:.4f} < 마진"
                                  f"(방향={_side_dir:+.0f}) - 중단", flush=True)
                            return True
                return False

            def _stage3_3_debug(step):
                chassis_pos, _ = base_robot.get_world_pose()
                _, _, box_center = _get_box_x_edges()
                print(f"  [DEBUG STAGE3.3 step={step}] 섀시=({float(chassis_pos[0]):.3f},{float(chassis_pos[1]):.3f}) "
                      f"박스=({box_center[0]:.3f},{box_center[1]:.3f}) 목표=({STAGE3_3_TARGET_X:.3f},{STAGE3_3_TARGET_Y:.3f})",
                      flush=True)
                _add_y_marker("LiveBoxYMarker", box_center[1], (1.0, 0.0, 1.0))

            STAGE3_3_BOX_TOLERANCE = 0.01

            def _stage3_3_box_reached():
                _, _, box_center = _get_box_x_edges()
                return (abs(float(box_center[0]) - STAGE3_3_TARGET_X) < STAGE3_3_BOX_TOLERANCE
                        and abs(float(box_center[1]) - STAGE3_3_TARGET_Y) < STAGE3_3_BOX_TOLERANCE)

            _, _stage3_3_ee_final, _stage3_3_ee_err, _stage3_3_progressed, _stage3_3_aborted = drive_and_reach(
                target_x=_stage3_3_chassis_target_x, target_y=_stage3_3_chassis_target_y,
                ee_target_pos=_stage3_3_target_ee, ee_orientation=DOWN_QUAT, hold_gripper_closed=True,
                max_speed=0.06, tolerance_xy=0.005,
                label="STAGE3.3: 섀시+팔 동시 X/Y 정렬",
                condition_fn=_stage3_3_box_reached,
                abort_fn=_stage3_3_broken, hard_stop_on_condition=True,
                debug_interval=10, debug_fn=_stage3_3_debug,
            )
            if _stage3_3_aborted:
                pause_for_inspection("[중단] STAGE 3.3 도중 자세 붕괴/클리어런스 부족이 감지돼 즉시 중단했습니다.")
            if _stage3_3_ee_err > 0.05:
                pause_for_inspection(
                    f"[중단] STAGE 3.3 - 목표에 충분히 도달하지 못했습니다(err={_stage3_3_ee_err:.3f}m)."
                )
            _, _, _stage3_3_final_box_center = _get_box_x_edges()
            print(f"[STAGE3.3 종료] 박스=({_stage3_3_final_box_center[0]:.3f},{_stage3_3_final_box_center[1]:.3f}) "
                  f"목표=({STAGE3_3_TARGET_X:.3f},{STAGE3_3_TARGET_Y:.3f})", flush=True)
            _log_clearance("STAGE3.3 종료(X/Y 정렬 후)")

            # [진단 - PICK 시점 로그(solution_space/joint_6/박스yaw)와 직접 비교하기 위한
            # 동일 형식 - 사용자 지적한 "place 정렬하면서 박스를 돌려버린다"를 실측으로
            # 확인한다. PICK 직후엔 joint_6이 달라도(-96.5도 vs +92.4도) 박스yaw는
            # 거의 동일(0.2도)했다 - 여기서 값이 달라져 있으면 STAGE3.3(또는 그 사이
            # 어느 단계)에서 joint_6이 다시 움직이며 박스를 같이 돌렸다는 뜻이다.
            if "joint_6" in m0609_robot.dof_names:
                _diag2_joint6_rad = float(
                    m0609_robot.get_joint_positions()[m0609_robot.dof_names.index("joint_6")])
            else:
                _diag2_joint6_rad = float("nan")
            _diag2_box_pos, _diag2_box_quat = cart_box_objects[picked_prim_path].get_world_pose()
            _diag2_box_yaw_deg = float(np.degrees(quat_to_euler_angles(np.asarray(_diag2_box_quat, dtype=float))[2]))
            print(f"[진단 STAGE3.3 종료 흡착 자세] solution_space={'2(반전)' if _need_space2 else '1(표준)'} "
                  f"joint_6={np.degrees(_diag2_joint6_rad):.1f}deg({_diag2_joint6_rad:.4f}rad) "
                  f"박스yaw={_diag2_box_yaw_deg:.1f}deg 박스pos={np.round(np.asarray(_diag2_box_pos, dtype=float), 3)}",
                  flush=True)

            chassis_pos0, _ = base_robot.get_world_pose()
            snapshot(eye=[chassis_pos0[0] - 0.8, chassis_pos0[1] - 1.6, chassis_pos0[2] + 0.9],
                     target=[STAGE3_3_TARGET_X, STAGE3_3_TARGET_Y, TRUNK_FLOOR_Z + 0.3],
                     fname="_trunkplace_03_3_align_y.png")

            if STAGE < 3.4:
                print("\n[STAGE 3.3 완료] STAGE 3.4 이상으로 다시 실행하면 최종 하강/배치까지 진행합니다.\n",
                      flush=True)

        if STAGE >= 3.4:
            # ================= STAGE 3.4: 최종 하강 + 릴리즈 (X/Y는 3.3이 이미 맞춤, Z만 목표 release 높이로) =================
            # 92.trunk_place_holonomic.py와 동일 - 다만 "/World/TestCarryBox" 하드코딩 대신 실제로
            # 집은 박스 경로(picked_prim_path)를 쓴다(92번은 합성 테스트 박스라 고정 경로였음).
            # 사용자 지시 - STAGE 3.3용 마커는 이제 다 썼으니 숨긴다.
            _hide_markers(["Stage3_3TargetPlane", "Stage3_3TargetXPlane", "LiveBoxYMarker"])
            _ee_now_3_4, _ = m0609_robot.end_effector.get_world_pose()
            _stage3_4_target_ee = (float(_ee_now_3_4[0]), float(_ee_now_3_4[1]), place_release_z)
            print(f"[STAGE3.4 목표] 현재ee={np.round(_ee_now_3_4, 3)} 목표ee={np.round(_stage3_4_target_ee, 3)} "
                  f"(release_z={place_release_z:.3f})", flush=True)

            # 사용자 실측 확인(2번째 박스, 왼쪽 배치/solution space 2) - STAGE3.3(X/Y 정렬)까지는
            # 깨끗하게 통과했는데 STAGE3.4(X/Y 고정, Z만 낮추는 최종 하강)에서 ee가 목표 근처에서
            # 갑자기 사방으로 튀며 발산했다(err=0.914m로 중단) - IK divergence 후 물리 충돌
            # 재확인 패턴([[feedback_isaac_sim_ik_divergence_debugging]]). 원인: STAGE3.3까지의
            # link_2 좌우 체크(바로 위 _stage3_3_broken 참고)는 "X/Y가 목표로 이동하는 동안"만
            # 감시했는데, STAGE3.4는 X/Y를 고정한 채 ee를 아래로만 내린다 - 이때도 팔꿈치(joint2/3)
            # 는 계속 움직여야(펴진 자세에서 다시 굽어야) ee가 수직으로 내려가므로, link_2가
            # 좌우로 추가 회전해 들어오다 STAGE3.3에서는 안 걸리던 벽/휠하우스에 새로 부딪힐 수
            # 있다 - STAGE3.3과 동일한 link_2 좌우 실측 체크를 여기도 그대로 적용한다(다만 X/Y가
            # 이미 고정 목표라 "다가가는 방향"이 없으므로, side_dir는 최종 배치 Y가 ANCHOR_Y의
            # 어느 쪽인지로 고정한다 - place_world_xy는 compute_place_targets()가 이미 계산해둔
            # 값이라 이 시점에 항상 유효하다).
            _stage3_4_side_dir = 1.0 if float(place_world_xy[1]) >= ANCHOR_Y else -1.0
            _stage3_4_clearance_counter = {"n": 0}

            def _stage3_4_broken():
                detached = not m0609_robot.gripper.is_closed()
                if detached:
                    print("  [DIAG STAGE3.4] detached=True", flush=True)
                    return True
                # STAGE3.3과 동일하게(_stage3_3_clearance_counter 참고) raycast 비용을 줄이려고
                # 10스텝마다 한 번만 검사한다.
                _stage3_4_clearance_counter["n"] += 1
                if _stage3_4_clearance_counter["n"] % 10 != 0:
                    return False
                _link2_min, _link2_max = _mesh_world_aabb(_LINK2_PATH)
                if _link2_max[1] is not None:
                    _link2_leading_y = _link2_max[1] if _stage3_4_side_dir > 0 else _link2_min[1]
                    _link2_x = (float(_link2_min[0]) + float(_link2_max[0])) / 2.0
                    _link2_z = (float(_link2_min[2]) + float(_link2_max[2])) / 2.0
                    _link2_wall_y = interior_side_wall_y_at(_link2_x, _link2_z, _stage3_4_side_dir)
                    if _link2_wall_y is not None:
                        _link2_margin_now = _stage3_4_side_dir * (_link2_wall_y - float(_link2_leading_y))
                        if _link2_margin_now < STAGE3_3_SIDE_MARGIN:
                            print(f"  [DIAG STAGE3.4] link_2 좌우여유={_link2_margin_now:.4f} < 마진"
                                  f"(방향={_stage3_4_side_dir:+.0f}) - 중단", flush=True)
                            return True
                return False

            def _stage3_4_debug(step):
                ee_pos, _ = m0609_robot.end_effector.get_world_pose()
                print(f"  [DEBUG STAGE3.4 step={step}] ee=({ee_pos[0]:.3f},{ee_pos[1]:.3f},{ee_pos[2]:.3f}) "
                      f"목표z={place_release_z:.3f}", flush=True)

            _stage3_4_ee_final, _stage3_4_ee_err, _stage3_4_aborted = reach_with_lift(
                _stage3_4_target_ee, lift_state["h"], steps=250,
                hold_gripper_closed=True, label="STAGE3.4: 목표 (X,Y) 위에서 release 높이로 수직 하강",
                abort_fn=_stage3_4_broken, hard_stop_on_condition=True,
                debug_interval=10, debug_fn=_stage3_4_debug,
            )
            if _stage3_4_aborted:
                pause_for_inspection("[중단] STAGE 3.4 도중 흡착 이탈 또는 link_2 좌우 여유 부족이 "
                                      "감지돼 즉시 중단했습니다 - 하강 중 충돌 의심.")
            if _stage3_4_ee_err > 0.05:
                pause_for_inspection(
                    f"[중단] STAGE 3.4 - 목표 하강 높이에 충분히 도달하지 못했습니다(err={_stage3_4_ee_err:.3f}m)."
                )
            print("[성공] STAGE 3.4 - release 높이까지 하강 완료.", flush=True)

            chassis_pos0, _ = base_robot.get_world_pose()
            snapshot(eye=[chassis_pos0[0] - 0.8, chassis_pos0[1] - 1.6, chassis_pos0[2] + 0.9],
                     target=[place_world_xy[0], place_world_xy[1], TRUNK_FLOOR_Z],
                     fname="_trunkplace_03_4_descended.png")

            # ---- 릴리즈 ----
            gripper.open()
            box_rigid_prim = SingleRigidPrim(picked_prim_path)
            box_rigid_prim.initialize(physics_sim_view=world.physics_sim_view)
            box_rigid_prim.set_linear_velocity(np.array([0.0, 0.0, -0.3]))
            step_hold(60)

            final_box_pos = get_world_pos(stage.GetPrimAtPath(picked_prim_path))
            err_xy = float(np.linalg.norm(final_box_pos[:2] - np.array(place_world_xy)))
            print(f"\n[완료] 최종 박스 world 위치={np.round(final_box_pos, 3)} 목표 xy={np.round(place_world_xy, 3)} "
                  f"xy 오차={err_xy:.4f}m", flush=True)

            snapshot(eye=[chassis_pos0[0] - 0.8, chassis_pos0[1] - 1.6, chassis_pos0[2] + 0.9],
                     target=[place_world_xy[0], place_world_xy[1], TRUNK_FLOOR_Z],
                     fname="_trunkplace_03_4_placed.png")

            result = {
                "picked_prim_path": picked_prim_path,
                "place_world_xy": list(place_world_xy),
                "target_release_z": place_release_z,
                "final_box_pos": final_box_pos.tolist(),
                "xy_error_m": err_xy,
            }
            (OUT_DIR / f"_trunkplace_result_{Path(picked_prim_path).name}.json").write_text(json.dumps(result, indent=2))
            print(f"[저장 완료] {OUT_DIR / ('_trunkplace_result_' + Path(picked_prim_path).name + '.json')}", flush=True)

            if STAGE < 4:
                print("\n[STAGE 3.4 완료] 박스를 최종 배치했습니다 - PLACE까지 완료. STAGE=4로 다시 실행하면 "
                      "후퇴까지 진행합니다.\n", flush=True)

        if STAGE >= 4:
            # ================= STAGE 4: 후퇴 (STAGE 3.4 -> ... -> STAGE 1 상태로 역순 복귀) =================
            # 92.trunk_place_holonomic.py와 완전히 동일(그대로 재사용) - 박스는 3.4에서 이미
            # 내려놨으므로 이 구간 전체에서 hold_gripper_closed=False로 둔다.
            print("\n[STAGE 4] 후퇴 시작 - STAGE 3.4 -> STAGE 1 상태로 역순 복귀", flush=True)

            STAGE4_CEILING_MARGIN = STAGE3_1_CEILING_MARGIN

            # 사용자 GUI 관찰(스크린샷) - STAGE4-3 종료 시점(섀시는 이미 BASE_START_XY까지
            # 멀리 후퇴했는데 팔은 아직 STAGE3.2.0의 "쭉 편" 자세 그대로라 트렁크 안쪽까지
            # 길게 뻗어 있다) 자세를 보고 "이러면 계속 부딪힌다"고 지적함 - 맞다. STAGE4의
            # 나머지 단계(4-1/4-2/4-4/4-5)는 지금까지 천장 여유(_stage4_ceiling_ok)만
            # 감시했는데, 이 자세에서 팔을 다시 접어들이는 동안 link_2(상완)가 좌우로도
            # 트렁크 입구/내벽에 부딪힐 수 있다(STAGE3.3/3.4에서 이미 확인된 것과 동일한
            # 원리 - 박스는 이제 없지만 link_2 자체의 굽음 방향은 그대로다). 같은 방식의
            # 좌우 실측 체크를 여기에도 추가한다 - side_dir는 이 박스의 최종 배치가
            # ANCHOR_Y의 어느 쪽이었는지로 고정(위 STAGE3.4의 _stage3_4_side_dir와 동일
            # 계산, 박스를 놓은 뒤에도 팔이 굽은 방향 자체는 안 바뀌므로 그대로 재사용 가능).
            def _stage4_lateral_ok():
                _link2_min, _link2_max = _mesh_world_aabb(_LINK2_PATH)
                if _link2_max[1] is None:
                    return True
                _link2_leading_y = _link2_max[1] if _stage3_4_side_dir > 0 else _link2_min[1]
                _link2_x = (float(_link2_min[0]) + float(_link2_max[0])) / 2.0
                _link2_z = (float(_link2_min[2]) + float(_link2_max[2])) / 2.0
                _link2_wall_y = interior_side_wall_y_at(_link2_x, _link2_z, _stage3_4_side_dir)
                if _link2_wall_y is None:
                    return True
                _link2_margin_now = _stage3_4_side_dir * (_link2_wall_y - float(_link2_leading_y))
                if _link2_margin_now < STAGE3_3_SIDE_MARGIN:
                    print(f"  [DIAG STAGE4] link_2 좌우여유={_link2_margin_now:.4f} < 마진"
                          f"(방향={_stage3_4_side_dir:+.0f}) - 중단", flush=True)
                    return False
                return True

            def _stage4_ceiling_ok(counter, interval=10):
                counter["n"] += 1
                if counter["n"] % interval != 0:
                    return True
                if not _stage4_lateral_ok():
                    return False
                front_x = _link5_front_x()
                ceiling_here = ceiling_z_at(front_x)
                if ceiling_here is None:
                    return True
                return (ceiling_here - _link5_top_z()) >= STAGE4_CEILING_MARGIN

            _stage4_1_counter = {"n": 0}

            def _stage4_1_broken():
                return not _stage4_ceiling_ok(_stage4_1_counter)

            _ee_now_4_1, _ = m0609_robot.end_effector.get_world_pose()
            _stage4_1_target_ee = (float(_ee_now_4_1[0]), float(_ee_now_4_1[1]), float(_ee_now_3_3[2]))
            _, stage4_1_err, stage4_1_aborted = reach_with_lift(
                _stage4_1_target_ee, lift_state["h"], steps=250, hold_gripper_closed=False,
                label="STAGE4-1(3.4 역): release 높이 -> 3.3 종료 높이로 수직 상승",
                abort_fn=_stage4_1_broken, hard_stop_on_condition=True,
            )
            if stage4_1_aborted:
                pause_for_inspection("[중단] STAGE4-1 도중 천장 클리어런스 또는 link_2 좌우여유 부족이 감지돼 즉시 중단했습니다.")
            if stage4_1_err > 0.05:
                pause_for_inspection(f"[중단] STAGE4-1 - 목표 높이로 충분히 복귀하지 못했습니다(err={stage4_1_err:.3f}m).")
            print("[성공] STAGE4-1 - 3.3 종료 높이로 복귀 완료.", flush=True)

            _stage4_2_counter = {"n": 0}

            def _stage4_2_broken():
                return not _stage4_ceiling_ok(_stage4_2_counter)

            _stage4_2_target_ee = tuple(float(v) for v in _ee_now_3_3)
            _stage4_2_target_chassis_x = _stage3_2_1_target_chassis_x
            _stage4_2_target_chassis_y = float(_stage3_2_1_chassis0[1])

            def _stage4_2_reached():
                ee_pos, _ = m0609_robot.end_effector.get_world_pose()
                return float(np.linalg.norm(np.array(ee_pos) - np.array(_stage4_2_target_ee))) < 0.01

            _, _, stage4_2_ee_err, _, stage4_2_aborted = drive_and_reach(
                target_x=_stage4_2_target_chassis_x, target_y=_stage4_2_target_chassis_y,
                ee_target_pos=_stage4_2_target_ee, ee_orientation=DOWN_QUAT, hold_gripper_closed=False,
                max_speed=0.06, tolerance_xy=0.005,
                label="STAGE4-2(3.3 역): 섀시+팔 동시 추종으로 3.2.1 종료 지점 복귀",
                condition_fn=_stage4_2_reached, abort_fn=_stage4_2_broken, hard_stop_on_condition=True,
            )
            if stage4_2_aborted:
                pause_for_inspection("[중단] STAGE4-2 도중 천장 클리어런스 또는 link_2 좌우여유 부족이 감지돼 즉시 중단했습니다.")
            if stage4_2_ee_err > 0.05:
                pause_for_inspection(f"[중단] STAGE4-2 - 목표 지점으로 충분히 복귀하지 못했습니다(err={stage4_2_ee_err:.3f}m).")
            print("[성공] STAGE4-2 - 3.2.1 종료 지점(섀시/팔)으로 복귀 완료.", flush=True)

            _stage4_3_hold_q = np.asarray(m0609_robot.get_joint_positions(), dtype=float).copy()
            _stage4_3_chassis_start, _ = base_robot.get_world_pose()
            _stage4_3_tip_start = measure_tip_world_pos()
            _stage4_3_tip_rel_ref = _stage4_3_tip_start - np.asarray(_stage4_3_chassis_start, dtype=float)

            def _hold_stage4_3_arm():
                m0609_robot.apply_action(ArticulationAction(joint_positions=_stage4_3_hold_q))

            def _stage4_3_broken():
                chassis_pos, _ = base_robot.get_world_pose()
                tip_pos = measure_tip_world_pos()
                tip_rel = tip_pos - np.asarray(chassis_pos, dtype=float)
                relative_error = float(np.linalg.norm(tip_rel - _stage4_3_tip_rel_ref))
                return relative_error > STAGE2_POSE_DRIFT_TOLERANCE

            _, _, _, stage4_3_aborted = drive_until(
                lambda: False, target_x=STAGE3_2_0_RETREAT_X, target_y=_stage4_2_target_chassis_y,
                tolerance_xy=0.005, max_speed=0.08, per_step_fn=_hold_stage4_3_arm,
                abort_fn=_stage4_3_broken, hard_stop_on_condition=True,
                label="STAGE4-3(3.2.1 역): 팔 자세 고정, 섀시만 3.2.0 종료 x까지 후진",
            )
            if stage4_3_aborted:
                pause_for_inspection("[중단] STAGE4-3 도중 자세 붕괴(충돌 의심)가 감지돼 즉시 중단했습니다.")
            print("[성공] STAGE4-3 - 3.2.0 종료 지점(섀시)으로 복귀 완료.", flush=True)

            _stage4_4_counter = {"n": 0}

            def _stage4_4_broken():
                return not _stage4_ceiling_ok(_stage4_4_counter)

            _ee_now_4_4, _ = m0609_robot.end_effector.get_world_pose()
            _stage4_4_fixed_ee = tuple(float(v) for v in _ee_now_4_4)

            # 사용자 질문 대응(재생 모드) - _stage3_2_0_joint_trajectory가 있으면(정상적인 경우
            # 거의 항상 있음) RMPflow에게 다시 풀게 하는 대신 정방향(STAGE3.2.0)이 기록해둔
            # 관절 궤적을 리프트 높이 기준으로 그대로 재생한다 - 그때 실제로 밟은 solution
            # branch를 그대로 되짚으므로 다른 branch로 튈 여지가 없다. 기록이 없었다면(극단적
            # 예외 상황) 기존 RMPflow 추종 방식으로 자동 대체된다.
            _stage4_4_replay_kwargs = (
                {"joint_trajectory": _stage3_2_0_joint_trajectory}
                if _stage3_2_0_joint_trajectory is not None else {}
            )
            _, _, stage4_4_ee_err, _, stage4_4_aborted = retreat_and_raise(
                _stage3_0_end_chassis_x, STAGE3_1_END_LIFT_H, _stage4_4_fixed_ee,
                max_speed=0.06, lift_speed=0.002, hold_gripper_closed=False,
                abort_fn=_stage4_4_broken, hard_stop_on_condition=True,
                label="STAGE4-4(3.2.0 역): ee 고정, 섀시 전진+리프트 하강으로 3.1 종료 지점 복귀",
                **_stage4_4_replay_kwargs,
            )
            if stage4_4_aborted:
                pause_for_inspection("[중단] STAGE4-4 도중 천장 클리어런스 또는 link_2 좌우여유 부족이 감지돼 즉시 중단했습니다.")
            if _stage3_2_0_joint_trajectory is None:
                # RMPflow 추종 모드일 때만 ee_err가 "목표에 도달했는지"를 뜻한다.
                if stage4_4_ee_err > 0.05:
                    pause_for_inspection(f"[중단] STAGE4-4 - ee가 고정 목표에서 너무 벗어났습니다(err={stage4_4_ee_err:.3f}m).")
            else:
                # 재생 모드에서는 ee_target_pos를 추종한 게 아니므로(관절값을 직접 재생) 이
                # 오차는 참고용일 뿐 성공 기준이 아니다 - 실제 검증은 abort_fn(천장/link_2)과
                # 이후 STAGE4-5가 재확인하는 3.0 종료 자세 도달 여부로 이뤄진다.
                print(f"[STAGE4-4 재생 모드] 참고용 ee_err={stage4_4_ee_err:.4f}m "
                      "(관절 궤적을 그대로 재생했으므로 이 오차는 성공 기준이 아님)", flush=True)
            print("[성공] STAGE4-4 - 3.1 종료 지점(섀시/리프트)으로 복귀 완료.", flush=True)

            _stage4_5_counter = {"n": 0}

            def _stage4_5_broken():
                return not _stage4_ceiling_ok(_stage4_5_counter)

            _ee_now_4_5, _ = m0609_robot.end_effector.get_world_pose()
            _stage4_5_target_ee = (float(_ee_now_4_5[0]), float(_ee_now_4_5[1]), float(_stage3_0_end_ee_pos[2]))
            _, stage4_5_err, stage4_5_aborted = reach_with_lift(
                _stage4_5_target_ee, _stage3_0_end_lift_h, steps=250, hold_gripper_closed=False,
                label="STAGE4-5(3.1 역): 팔/리프트를 3.0 종료 높이로 되올림",
                abort_fn=_stage4_5_broken, hard_stop_on_condition=True,
            )
            if stage4_5_aborted:
                pause_for_inspection("[중단] STAGE4-5 도중 천장 클리어런스 또는 link_2 좌우여유 부족이 감지돼 즉시 중단했습니다.")
            if stage4_5_err > 0.05:
                pause_for_inspection(f"[중단] STAGE4-5 - 목표 높이로 충분히 복귀하지 못했습니다(err={stage4_5_err:.3f}m).")
            print("[성공] STAGE4-5 - 3.0 종료 높이(팔/리프트)로 복귀 완료.", flush=True)

            _stage4_6_hold_q = np.asarray(m0609_robot.get_joint_positions(), dtype=float).copy()
            _stage4_6_chassis_start, _ = base_robot.get_world_pose()
            _stage4_6_tip_start = measure_tip_world_pos()
            _stage4_6_tip_rel_ref = _stage4_6_tip_start - np.asarray(_stage4_6_chassis_start, dtype=float)

            def _hold_stage4_6_arm():
                m0609_robot.apply_action(ArticulationAction(joint_positions=_stage4_6_hold_q))

            def _stage4_6_broken():
                chassis_pos, _ = base_robot.get_world_pose()
                tip_pos = measure_tip_world_pos()
                tip_rel = tip_pos - np.asarray(chassis_pos, dtype=float)
                relative_error = float(np.linalg.norm(tip_rel - _stage4_6_tip_rel_ref))
                return relative_error > STAGE2_POSE_DRIFT_TOLERANCE

            def _stage4_6_max_speed():
                chassis_pos, _ = base_robot.get_world_pose()
                remaining = abs(float(chassis_pos[0]) - BASE_START_XY[0])
                return 0.05 if remaining < 0.15 else 0.10

            _, _, _, stage4_6_aborted = drive_until(
                lambda: False, target_x=BASE_START_XY[0], target_y=BASE_START_XY[1],
                max_speed=0.10, max_speed_fn=_stage4_6_max_speed,
                per_step_fn=_hold_stage4_6_arm, abort_fn=_stage4_6_broken, hard_stop_on_condition=True,
                label="STAGE4-6(3.0+STAGE2 역): 팔 자세 고정, 트렁크 밖 BASE_START_XY까지 후진",
            )
            if stage4_6_aborted:
                pause_for_inspection("[중단] STAGE4-6 도중 자세 붕괴(충돌 의심)가 감지돼 즉시 중단했습니다.")
            print("[성공] STAGE4-6 - BASE_START_XY로 후진 완료.", flush=True)

            _final_ee_pos, _ = m0609_robot.end_effector.get_world_pose()
            move_link6((float(_final_ee_pos[0]), float(_final_ee_pos[1]), HOLDING_Z), steps=200,
                       hold_gripper_closed=False, orientation=DOWN_QUAT,
                       label="STAGE4-7(STAGE1.1 역): STAGE 1 홀딩 높이로 복귀")

            chassis_pos0, _ = base_robot.get_world_pose()
            snapshot(eye=[chassis_pos0[0] - 2.2, chassis_pos0[1] - 3.2, chassis_pos0[2] + 1.6],
                     target=[(chassis_pos0[0] + CAR_POS[0]) / 2, 0.0, 1.0], fname="_trunkplace_04_retreated.png")
            print(f"\n[STAGE 4 완료] 박스({picked_prim_path}) 배치 + 후퇴 완료 - STAGE 1 상태(홀딩 자세, "
                  f"BASE_START_XY 근처)로 복귀됨.\n", flush=True)
            _publish_status("pick_and_place", "box_done", box_index=_box_num, box_count=len(pick_order), box_id=str(picked_placement['box_id']))

        # 사용자 설계(카트 양쪽 접근 재설계) - 94번엔 여기(루프 끝)에 "다음 박스를 위해 카트로
        # 복귀"하는 별도 블록이 있었다(카트 접근 위치/각도가 항상 고정이라, 다음 박스가
        # 반전을 필요로 하면 복귀 주행 도중 멈춰서 조인트만 바꾸는 방식). 100번은 카트 접근
        # 위치 자체가 박스마다 달라지므로, 그 이동은 "다음 반복의 시작"(이 루프의 맨 위,
        # "카트 접근 계획"+"카트 standoff로 주행" 부분)이 담당한다 - 마지막 박스 뒤에는 다음
        # 반복이 없으므로 이 자리에서 아무것도 할 필요가 없다(빈 채로 루프가 자연히 끝난다).

    return_to_spawn_pose()
    _publish_status("pick_and_place", "done")


def run_cart_scan():
    """99.py 본문(카메라/컨트롤러/조준 함수는 이 함수 안에서만 유효한 로컬 재정의 -
    LIFT_MAX 등 이름은 같아도 스크립트마다 값이 다른 상수가 있어 절대 전역과 공유하지 않는다).
    씬/로봇을 다시 만드는 부분(월드/스테이지/카트 에셋/박스 스폰/로봇 생성/world.reset)만
    제외했고, 나머지는 99.py 원문 그대로다."""
    _publish_status("cart_scan", "started")
    LIFT_TRAVEL_M = 0.55

    EE_LINK_NAME = "link_6"
    GRIPPER_BODY_NAME = "vgp20_suction_plate"
    DEPTH_CAMERA_NAME_HINT = "Depth"

    STANDOFF_MARGIN = 0.10
    WAYPOINT_STEPS = 300
    SETTLE_STEPS = 60
    DOWN_QUAT = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))
    WORLD_UP = (0.0, 0.0, 1.0)
    CAMERA_AXES = "usd"

    # 35.crate_scan_setup.py의 converge_to_pose()와 동일한 조기 종료(plateau) 수렴 -
    # 목표 근처에서 정체되면 남은 스텝을 다 채우지 않고 바로 멈춘다(속도 개선, 정확도는
    # 그대로 - 35.py에서 A/B 비교로 이미 검증됨).
    CONVERGENCE_CHECK_INTERVAL_STEPS = 25
    CONVERGENCE_MIN_STEPS = 75
    CONVERGENCE_PLATEAU_TOLERANCE_M = 0.001

    # 실측(2026-07-24, 다중시점 카트 스캔 실사용) - 5개 시점 모두에서 검출된 박스 위치가
    # 실제 물리 위치와 9~12cm씩, 게다가 두 박스 모두 비슷한 방향으로 어긋났다(무작위
    # 노이즈가 아니라 일정한 방향의 조직적 오차로 보임 - 사용자 지적). C-4에서 이미 한 번
    # 겪은 것과 같은 클래스의 버그(정지 직후 렌더가 물리를 못 따라간 상태에서 캡처)가
    # Z축이 아니라 X/Y에도 영향을 줬을 가능성이 있어, 각 시점 캡처 전 정지 시간을
    # 20 -> 90스텝으로 늘려 렌더 파이프라인이 확실히 새 카메라 자세를 반영한 뒤에
    # point cloud를 얻도록 한다(실험 - 오차가 줄어드는지 재측정 필요).
    POST_CONVERGENCE_SETTLE_STEPS = 90

    # ---- 스캔 자세 파라미터 (35.crate_scan_setup.py의 검증된 공식 그대로 재사용) ----
    # 사용자 지적 - 손목 조인트를 직접 돌리거나 사후 회전을 추가하는 방식은 전부 발산/엉뚱한
    # 방향을 봄으로 실패했다. 35.py를 보니 21도는 회전 트릭이 아니라 **eye 위치의 수평
    # 오프셋을 height*tan(21도)로 계산**해서 eye/look_at 자체의 기하학적 배치로 21도가
    # 자연히 나오게 하는 방식이었다(35.py 205-212행) - 그 공식을 그대로 가져온다.
    CART_BASKET_FLOOR_Z = 0.68
    # [다중 시점 스캔 추가 후 실측] 0.75(35.py 값 그대로 가져온 것)로는 IK 도달 거리가
    # 팔의 물리적 최대 도달 범위를 넘어서 매 시점 8~12cm 오차로 수렴 실패했다(위
    # LIFT_TRAVEL_M 주석 참고) - 0.55로 낮춰서 필요 도달 거리를 줄인다. LIFT_TRAVEL_M을
    # 같이 올린 것과 합쳐서 팔 base<->목표 거리를 충분히 줄이는 게 목표.
    EYE_HEIGHT_ABOVE_CART = 0.55
    SCAN_TILT_FROM_VERTICAL_DEG = 30.0  # 35.py의 SCAN_TILT_FROM_VERTICAL_DEG와 동일
    _scan_horizontal_offset = EYE_HEIGHT_ABOVE_CART * np.tan(np.radians(SCAN_TILT_FROM_VERTICAL_DEG))

    DEPTH_TOPIC = "/camera/depth"
    CAMERA_INFO_TOPIC = "/camera/camera_info"
    CAMERA_FRAME_ID = "m0609_depth_camera_optical_frame"
    CAMERA_WIDTH, CAMERA_HEIGHT = 640, 480

    def setup_ros2_camera_bridge(camera_prim_path):
        """32.box_table_scan_setup.py와 동일 패턴 - box_top_extractor.py가 구독하는
        /camera/depth, /camera/camera_info 토픽과 정확히 일치시킨다."""
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/World/ROS2_Cart_Scan_Camera_Graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "DepthPublish.inputs:execIn"),
                    ("CreateRenderProduct.outputs:renderProductPath", "DepthPublish.inputs:renderProductPath"),
                    ("CreateRenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                    ("CreateRenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
                ],
                keys.SET_VALUES: [
                    ("CreateRenderProduct.inputs:cameraPrim", camera_prim_path),
                    ("CreateRenderProduct.inputs:width", CAMERA_WIDTH),
                    ("CreateRenderProduct.inputs:height", CAMERA_HEIGHT),
                    ("DepthPublish.inputs:type", "depth"),
                    ("DepthPublish.inputs:topicName", DEPTH_TOPIC),
                    ("DepthPublish.inputs:frameId", CAMERA_FRAME_ID),
                    ("DepthPublish.inputs:resetSimulationTimeOnStop", True),
                    ("CameraInfoPublish.inputs:topicName", CAMERA_INFO_TOPIC),
                    ("CameraInfoPublish.inputs:frameId", CAMERA_FRAME_ID),
                    ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                ],
            },
        )

    # 사용자 설계(100번과 동일) - rot_z=CART_ROT_Z로 카트를 90도 돌린다.


    # ---- 카트 안에 적층 박스 3개 배치 (88.cart_scan_holonomic.py 3b061bd에서 이식,
    # 오프셋 축만 100번/99번과 동일하게 변경) ----
    # 시나리오: 큰 박스(Large)가 바닥에 깔리고, 그 위에 중간(Medium)+작은(Small) 박스가
    # Large의 윗면에 나란히(서로 위아래로 쌓이지 않고) 올라간 구조. 88번에서 실측으로
    # 확인한 3가지 설계 원칙(자세한 근거는 88.py 해당 위치 주석 참고)을 축만 바꿔 그대로
    # 재사용한다: (1) 자식을 부모 중심(dy=0 조건에 해당하는 축)에 두면 부모 윗면이
    # 거의 다 덮여 부모 자체가 검출 실패하므로 부모의 한쪽 절반에만 배치, (2) 형제
    # 박스(Medium-Small) 간격은 DBSCAN_EPS_M(2.5cm)보다 확실히 넓게(4cm), (3) 자식-부모
    # 높이차는 DEDUP_OVERLAP_Z_TOLERANCE_M(0.05m)보다 확실히 크게.
    # 88번은 dx=폭(옛 X)/dy=길이(옛 Y) offset이었는데, 99번은 회전 후 dx=길이(X)/dy=폭(Y)
    # 이므로(위 cart_half_x/y 주석 참고) 88번의 (dx, dy)를 (dy, dx)로 맞바꿔서 옮긴다.

    # 사용자 설계(100번과 동일 - 카트 옆 접근이 이제 폭(Y) 방향) - STANDOFF_X를 카트의
    # 너비(cart_half_y, 회전 후 의미)에 여유를 더하는 것으로 바꾼다. 스캔은 방향 구분이
    # 없어도 되므로(월드 좌표로 저장되니 어느 쪽에서 스캔하든 결과는 동일) 임의로 "오른쪽"
    # (-Y)에서만 스캔한다 - 100번의 스폰 기본값과 맞춰 일관성을 유지.
    STANDOFF_Y = CHASSIS_HALF_WIDTH_EFFECTIVE + cart_half_y + STANDOFF_MARGIN
    print(f"[STANDOFF] {CHASSIS_HALF_WIDTH_EFFECTIVE:.3f}(섀시 반폭) + {cart_half_y:.3f}(카트 반너비) + "
          f"{STANDOFF_MARGIN:.3f}(여유) = {STANDOFF_Y:.3f}m", flush=True)
    SCAN_STANDOFF_XY = (cart_center_xy[0], cart_center_xy[1] - STANDOFF_Y)

    # 사용자 실측 확인(100번과 완전히 동일한 함정, "생성하자마자 충돌") - BASE_FACE_ROT_Z=
    # 90도 고정이라 섀시는 항상 yaw=90도로 스폰되고, 이 각도에서는 섀시의 "긴" 길이 축이
    # 월드 Y를 향한다(폭 축은 X). SCAN_STANDOFF_XY는 "도착 후(yaw=0/180) 폭 축이 Y를
    # 향한다"는 전제로 계산했으므로, 그 자리에 yaw=90도로 그냥 스폰하면 스폰 즉시
    # 카트와 겹친다. 100번과 동일한 해법 - 카트 길이축 연장선상(B단보다 바깥)에 회전
    # 안전지대 CART_CLEAR_X를 두고 거기서 스폰한다(마진은 길이+폭 반경을 더해 회전
    # 도중 어떤 중간 각도든 안전하도록 넉넉히 잡음 - AABB는 한 축만 분리돼도 안 겹친다).
    CART_CLEAR_X = (cart_center_xy[0] + cart_half_x
                    + CHASSIS_HALF_LENGTH_EFFECTIVE + CHASSIS_HALF_WIDTH_EFFECTIVE
                    + STANDOFF_MARGIN)
    CHASSIS_SPAWN_XY = (CART_CLEAR_X, cart_center_xy[1])

    area_light = UsdLux.SphereLight.Define(stage, "/World/ScanAreaLight")
    area_light.CreateRadiusAttr(0.3)
    area_light.CreateIntensityAttr(60000)
    UsdGeom.Xformable(area_light).AddTranslateOp().Set(Gf.Vec3d(cart_center_xy[0], cart_center_xy[1], 2.0))

    # 베이스는 회전 안전지대(CHASSIS_SPAWN_XY)에서 스폰한다(아래 "1. 카트 옆면 접근"에서
    # 안전지대 -> 최종 standoff까지 3단계로 나눠 이동한다 - 100번과 동일한 안전 패턴).

    MEASURED_CHASSIS_TOP_OFFSET = 0.0180
    LIFT_MIN = MEASURED_CHASSIS_TOP_OFFSET + M0609_MOUNT_Z_ABOVE_CHASSIS_TOP
    LIFT_MAX = LIFT_MIN + LIFT_TRAVEL_M





    _init_joints = np.zeros(m0609_robot.num_dof)
    if "joint_3" in m0609_robot.dof_names:
        _init_joints[m0609_robot.dof_names.index("joint_3")] = np.pi / 2
    if "joint_5" in m0609_robot.dof_names:
        _init_joints[m0609_robot.dof_names.index("joint_5")] = np.pi / 2
    m0609_robot.set_joint_positions(_init_joints)



    def set_lift_height(h):
        chassis_pos, chassis_quat = base_robot.get_world_pose()
        target_pos = np.array([float(chassis_pos[0]), float(chassis_pos[1]), float(chassis_pos[2]) + h])
        m0609_robot.set_world_pose(position=target_pos, orientation=chassis_quat)
        m0609_robot.set_linear_velocity(np.zeros(3))
        m0609_robot.set_angular_velocity(np.zeros(3))
        column_base_z = float(chassis_pos[2]) + LIFT_MIN
        column_len = max(float(h) - LIFT_MIN, 0.001)
        lift_scale_op.Set(Gf.Vec3f(1.0, 1.0, column_len))
        lift_translate_op.Set(Gf.Vec3d(float(chassis_pos[0]), float(chassis_pos[1]), column_base_z + column_len / 2.0))


    def step_hold(n=1):
        for _ in range(n):
            set_lift_height(lift_state["h"])
            world.step(render=True)


    def move_lift_to(target_h, steps=90):
        start_h = lift_state["h"]
        for i in range(steps):
            h = start_h + (target_h - start_h) * (i + 1) / steps
            set_lift_height(h)
            world.step(render=True)
        lift_state["h"] = target_h
        print(f"[리프트] {start_h:.3f} -> {target_h:.3f}", flush=True)


    def holo_forward(vx, vy, wz):
        speeds = mecanum_wheel_speeds(vx, vy, wz, WHEEL_RADIUS, k_factor)
        return ArticulationAction(joint_velocities=speeds, joint_indices=hub_dof_indices)


    SMOOTH_ALPHA = 0.12
    _smooth_state = {"vx": 0.0, "vy": 0.0, "wz": 0.0}


    def drive_to(target_x=None, target_y=None, target_yaw_deg=None, tolerance_xy=0.03, tolerance_yaw_deg=2.0,
                 max_speed=0.4, max_wz=0.2, kp_xy=1.8, kp_yaw=0.25, max_steps=3000, label=""):
        """83/85/86/87번과 동일한 폐루프 주행."""
        start_pos, start_quat = base_robot.get_world_pose()
        start_yaw = float(np.degrees(quat_to_euler_angles(start_quat)[2]))
        tx = target_x if target_x is not None else float(start_pos[0])
        ty = target_y if target_y is not None else float(start_pos[1])
        tyaw = target_yaw_deg if target_yaw_deg is not None else start_yaw
        print(f"\n[주행 시작]{' ' + label if label else ''} 목표=({tx:.3f},{ty:.3f},{tyaw:.1f}deg)", flush=True)

        STALL_WINDOW, STALL_MIN_PROGRESS = 150, 0.008
        last_check_pos = np.array([float(start_pos[0]), float(start_pos[1])])
        stalled = False
        step = 0
        for step in range(1, max_steps + 1):
            pos, quat = base_robot.get_world_pose()
            yaw_deg = float(np.degrees(quat_to_euler_angles(quat)[2]))
            ex_w, ey_w = tx - float(pos[0]), ty - float(pos[1])
            eyaw = ((tyaw - yaw_deg + 180) % 360) - 180
            if abs(ex_w) < tolerance_xy and abs(ey_w) < tolerance_xy and abs(eyaw) < tolerance_yaw_deg:
                break
            yaw_rad = np.radians(yaw_deg)
            ex_l = ex_w * np.cos(yaw_rad) + ey_w * np.sin(yaw_rad)
            ey_l = -ex_w * np.sin(yaw_rad) + ey_w * np.cos(yaw_rad)
            vx_t = float(np.clip(kp_xy * ex_l, -max_speed, max_speed))
            vy_t = float(np.clip(kp_xy * ey_l, -max_speed, max_speed))
            wz_t = float(np.clip(np.radians(kp_yaw * eyaw), -max_wz, max_wz))
            _smooth_state["vx"] += SMOOTH_ALPHA * (vx_t - _smooth_state["vx"])
            _smooth_state["vy"] += SMOOTH_ALPHA * (vy_t - _smooth_state["vy"])
            _smooth_state["wz"] += SMOOTH_ALPHA * (wz_t - _smooth_state["wz"])
            base_robot.apply_action(holo_forward(_smooth_state["vx"], _smooth_state["vy"], _smooth_state["wz"]))
            step_hold(1)
            if step % STALL_WINDOW == 0:
                cur = np.array([float(pos[0]), float(pos[1])])
                progress = float(np.linalg.norm(cur - last_check_pos))
                if progress < STALL_MIN_PROGRESS and (abs(ex_w) > tolerance_xy or abs(ey_w) > tolerance_xy):
                    stalled = True
                    print(f"  [정체 감지] {progress:.4f}m밖에 못 움직임 - 중단", flush=True)
                    break
                last_check_pos = cur
        for _ in range(30):
            _smooth_state["vx"] *= 1 - SMOOTH_ALPHA
            _smooth_state["vy"] *= 1 - SMOOTH_ALPHA
            _smooth_state["wz"] *= 1 - SMOOTH_ALPHA
            base_robot.apply_action(holo_forward(_smooth_state["vx"], _smooth_state["vy"], _smooth_state["wz"]))
            step_hold(1)
        final_pos, final_quat = base_robot.get_world_pose()
        final_yaw = float(np.degrees(quat_to_euler_angles(final_quat)[2]))
        print(f"[주행 완료]{' ' + label if label else ''} {step}스텝, 최종=({final_pos[0]:.3f},{final_pos[1]:.3f},"
              f"{final_yaw:.1f}deg) 정체={stalled}", flush=True)
        return final_pos, final_yaw, not stalled


    step_hold(60)
    print("\n[안정화 완료]\n", flush=True)

    # ================= 카메라 + link6<->camera 오프셋 측정 (32/12.py와 동일 패턴) =================
    # 사용자 지적: 이전 "hover 위치 XYZ로 이동 후 joint_6만 사후에 비틀기" 방식이 계속 발산했다
    # (스캔 목표가 대각선/역방향으로 튐). 32.box_table_scan_setup.py에서 이미 검증된 방식을
    # 재사용한다 - 카메라가 그리퍼에 고정 마운트돼 있으므로 "지금(임의의 관절각) link6 자세"와
    # "지금 카메라 world 자세"의 상대 오프셋은 관절각과 무관하게 항상 일정하다. 이 오프셋을
    # 한 번만 측정해두면, 이후 "카메라가 어디서(eye) 어디를(look_at) 봐야 하는지"만으로 RMPflow가
    # 풀어야 할 link6 목표 pos+quat을 역산할 수 있다(hover+사후 비틀기보다 훨씬 안정적으로 수렴).
    camera_prim_path, all_cameras = find_camera_prim_path(stage, m0609_path, DEPTH_CAMERA_NAME_HINT)
    if camera_prim_path is None:
        raise RuntimeError(f"카메라 프림을 못 찾음 - 발견된 카메라 후보: {all_cameras}")
    print(f"[CAMERA] 스캔에 사용할 depth 카메라: {camera_prim_path} (후보 전체: {all_cameras})", flush=True)
    camera = Camera(prim_path=camera_prim_path, resolution=(CAMERA_WIDTH, CAMERA_HEIGHT))
    camera.initialize()
    # 35.crate_scan_setup.py와 동일 - get_pointcloud()가 내부적으로 이 depth annotator에
    # 의존한다. 이걸 빼먹으면 camera.initialize()만으로는 depth 프레임이 붙지 않아서
    # get_pointcloud()가 매번 빈(1차원) 배열을 반환한다(88.py 실측 확인 - 5개 시점 전부
    # shape=(0,)로 실패했었음).
    camera.add_distance_to_image_plane_to_frame()
    camera.add_rgb_to_frame()
    step_hold(10)

    link6_pos0, link6_quat0 = m0609_robot.end_effector.get_world_pose()
    cam_pos0, cam_quat0 = camera.get_world_pose(camera_axes=CAMERA_AXES)
    R_link6_0 = quats_to_rot_matrices(np.array([link6_quat0]))[0]
    R_cam_0 = quats_to_rot_matrices(np.array([cam_quat0]))[0]
    R_offset = R_link6_0.T @ R_cam_0
    cam_local_pos_offset = R_link6_0.T @ (np.array(cam_pos0) - np.array(link6_pos0))
    print(f"[오프셋] R_offset(link6->camera)=\n{R_offset}\ncamera pos offset in link6 frame={cam_local_pos_offset}",
          flush=True)


    def lookat_to_link6_target(anchor_world, look_at, up=WORLD_UP):
        """35/32.py의 lookat_to_link6_target과 완전히 동일 - 21도는 여기서 만드는 게 아니라
        호출하는 쪽에서 eye/look_at의 기하학적 배치(height*tan(21도) 수평 오프셋)로 이미
        반영돼서 들어온다."""
        camera_eye = np.asarray(anchor_world, dtype=float)
        look_at = np.asarray(look_at, dtype=float)
        R_cam_target = make_usd_camera_rotation(camera_eye, look_at, up)
        R_link6_target = R_cam_target @ R_offset.T
        link6_target_pos = camera_eye - R_link6_target @ cam_local_pos_offset
        q_link6_target = rot_matrices_to_quats(np.array([R_link6_target]))[0]
        return link6_target_pos, q_link6_target


    # ================= RMPflow 컨트롤러 =================
    controller = RMPFlowController(
        name="cart_scan_holonomic",
        robot_articulation=m0609_robot,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=M0609_DESCRIPTION_PATH,
        rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
    )


    def sync_rmp_base():
        chassis_pos, chassis_quat = base_robot.get_world_pose()
        base_pos = np.array([float(chassis_pos[0]), float(chassis_pos[1]), float(chassis_pos[2]) + lift_state["h"]])
        controller._default_position = base_pos
        controller._default_orientation = chassis_quat
        controller.rmp_flow.set_robot_base_pose(robot_position=base_pos, robot_orientation=chassis_quat)


    def move_link6(target_pos, steps=WAYPOINT_STEPS, label="", orientation=DOWN_QUAT,
                    early_exit=True, min_steps=CONVERGENCE_MIN_STEPS,
                    check_interval=CONVERGENCE_CHECK_INTERVAL_STEPS,
                    plateau_tolerance=CONVERGENCE_PLATEAU_TOLERANCE_M):
        """35.crate_scan_setup.py의 converge_to_pose()와 동일한 조기 종료 로직 - 목표
        근처에서 더 이상 움직이지 않으면(plateau) min_steps 이후부터 check_interval마다
        확인해서 바로 멈춘다. base(섀시)가 스캔 도중 움직일 수 있으므로(88.py 고유
        상황) sync_rmp_base()를 매 스텝 호출해 RMPflow가 항상 현재 base pose를
        기준으로 풀게 한다."""
        target_pos = np.array(target_pos, dtype=float)
        last_check_pos = None
        steps_run = 0
        for step in range(steps):
            sync_rmp_base()
            actions = controller.forward(
                target_end_effector_position=target_pos,
                target_end_effector_orientation=orientation,
            )
            m0609_robot.apply_action(actions)
            set_lift_height(lift_state["h"])
            world.step(render=True)
            steps_run += 1

            if not early_exit:
                continue
            if step + 1 < min_steps:
                continue
            if (step + 1) % check_interval != 0:
                continue
            current_pos, _ = m0609_robot.end_effector.get_world_pose()
            current_pos = np.array(current_pos)
            if last_check_pos is not None:
                movement = float(np.linalg.norm(current_pos - last_check_pos))
                if movement < plateau_tolerance:
                    break
            last_check_pos = current_pos

        ee_pos, _ = m0609_robot.end_effector.get_world_pose()
        err = np.linalg.norm(np.array(ee_pos) - target_pos)
        print(f"[웨이포인트{' ' + label if label else ''}] {steps_run}/{steps}스텝, target={np.round(target_pos, 3)} "
              f"ee={np.round(ee_pos, 3)} err={err:.4f}m", flush=True)
        if err > 0.05:
            print(f"[경고]{' ' + label if label else ''} IK 수렴 오차가 5cm를 넘습니다.", flush=True)
        return ee_pos, err


    viewport = vp_util.get_active_viewport()


    def snapshot(eye, target, fname):
        set_camera_view(eye=eye, target=target)
        step_hold(15)
        out = str(OUT_DIR / fname)
        vp_util.capture_viewport_to_file(viewport, out)
        step_hold(30)
        print(f"[SCREENSHOT] {out}", flush=True)


    chassis_pos0, _ = base_robot.get_world_pose()
    snapshot(
        eye=[chassis_pos0[0] - 1.0, chassis_pos0[1] - 1.3, 1.4],
        target=[cart_center_xy[0], cart_center_xy[1], 0.5],
        fname="_cartscan_00_start.png",
    )

    # ================= 리프트를 최고 높이로 (카트 바스켓까지 reach 확보, 84번과 동일 이유) =================
    print(f"\n[리프트] 도킹({LIFT_MIN:.3f}) -> 최고({LIFT_MAX:.3f})", flush=True)
    move_lift_to(LIFT_MAX, steps=120)

    # ================= 1. 회전 안전지대 -> 카트 옆면(오른쪽 standoff) 접근 =================
    # 사용자 실측 확인(100번과 동일한 함정) - 스폰 직후 yaw=90도(길이 축이 Y를 향함)인 채로
    # Y 오프셋 standoff 자리에 그냥 서 있으면 카트와 겹친다. CART_CLEAR_X(카트와 X축만으로
    # 분리되는 안전지대)에서 먼저 회전을 마친 뒤(1,2단계), 목표 yaw=0도(폭 축이 Y를 향함)가
    # 된 채로 X만 좁혀 최종 접근한다(3단계) - 100번의 3단계 패턴과 동일한 안전 논리
    # (AABB는 한 축만 분리돼도 안 겹친다), 다만 88번의 더 단순한 drive_to(abort_fn 없음)를
    # 그대로 쓴다 - 이 스크립트는 아직 박스를 들고 있지 않아 100번만큼 충돌 감지가
    # 절실하지 않다(팔은 카트 위로 아직 뻗지 않은 접힌 상태).
    _spawn_pos, _spawn_quat = base_robot.get_world_pose()
    _spawn_yaw = float(np.degrees(quat_to_euler_angles(_spawn_quat)[2]))
    drive_to(target_x=CART_CLEAR_X, target_y=float(_spawn_pos[1]), target_yaw_deg=_spawn_yaw,
             label="1/3: 스폰 위치 -> 회전 안전지대(X만 이동)")
    drive_to(target_x=CART_CLEAR_X, target_y=SCAN_STANDOFF_XY[1], target_yaw_deg=0.0,
             label="2/3: 회전 안전지대에서 오른쪽 standoff 방향으로 회전+횡이동")
    drive_to(target_x=SCAN_STANDOFF_XY[0], target_y=SCAN_STANDOFF_XY[1], target_yaw_deg=0.0,
             label="3/3: 회전 안전지대 -> 카트 옆면(오른쪽) standoff 최종 접근")
    snapshot(
        eye=[SCAN_STANDOFF_XY[0] - 1.0, SCAN_STANDOFF_XY[1] - 1.3, 1.4],
        target=[cart_center_xy[0], cart_center_xy[1], 0.5],
        fname="_cartscan_01_approached.png",
    )

    # ================= 2. 다중 시점 스캔 (베이스 strafe로 시점 다양화) =================
    # 35.crate_scan_setup.py의 테이블 스캔은 "섀시 고정 + 팔 azimuth 스윙"으로 시점을
    # 늘렸다(테이블이 넓고 평평해서 팔이 도달 범위 안에서 넓게 돌아볼 여지가 있었음).
    # 카트 바스켓은 다르다: 로봇이 도킹한 지점에서 긴 축(회전 후 X, ~0.9m)이 멀리
    # 뻗어있는 좁고 긴 형태이고, 도킹 거리 자체가 빠듯하다(STANDOFF_MARGIN=0.10m) - 팔만
    # 크게 스윙하면 카트 벽/철망에 부딪힐 위험이 있다. 대신 이 홀로노믹 베이스는 옴니휠로
    # 회전 없이 옆으로(strafe) 미끄러질 수 있다(88.py 파일 설계 의도, 사용자 확인) -
    # 팔 대신 섀시 자체를 카트의 긴 축(회전 후 X)을 따라 여러 위치로 옮기고, 각 위치에서는
    # 고정된 tilt로 아래를 보는 방식으로 시점을 다양화한다(88번은 이 축이 Y였다 - 카트가
    # 90도 돌면서 standoff/strafe 축도 함께 90도 돌았을 뿐, 로직 자체는 동일하다).
    #
    # 섀시가 매 시점 실제로 이동하므로(35.py는 섀시가 고정이라 base_pos/R_base를
    # 한 번만 재고 모든 시점에 그대로 썼음), 각 시점의 world 좌표 point cloud를
    # "그 시점의" base_link 기준으로 바로 변환하면 시점마다 원점이 달라져서 어긋난다.
    # 그래서 스윕 도중에는 world 좌표 그대로 누적해두고, 스윕이 끝나고 베이스가
    # 중앙(기준 위치)으로 돌아온 뒤 base_link를 딱 한 번만 측정해서 전체 누적
    # point cloud를 그 기준 프레임으로 한 번에 변환한다.
    CART_SCAN_STRAFE_X_OFFSETS = [-0.28, -0.14, 0.0, 0.14, 0.28]
    CART_SCAN_ROI_MAX_HEIGHT_M = 0.40  # CART_BASKET_FLOOR_Z 위로 이만큼까지만(카트 손잡이/배경 배제)
    # 88.cart_scan_holonomic.py(3b061bd)에서 이식 - 적층 시나리오 실측 확인(중요 버그):
    # 기존 XY 크롭은 cart_min/max(카트 바깥쪽 bbox, 철망 벽/테두리까지 포함)에 마진을
    # "바깥쪽으로" 더한 범위라 카트 벽/테두리 자체가 통째로 point cloud에 들어왔다. 이
    # 벽 평면이 RANSAC segment_plane()의 앞쪽 반복을 먼저 차지해버려서 Medium/Small처럼
    # 노출 면적이 작은 박스 윗면 검출이 불안정해졌다(88번 실측: 12회 중 1~2회만 관측).
    # 카트 중심 기준 실제 박스 적재 영역보다 넉넉하되 카트 벽(cart_half_x/y)보다는
    # 확실히 안쪽인 반경으로 크롭하면 벽이 아예 안 들어와서 문제가 사라진다. 값은 88번과
    # 동일(0.22/0.22, 대칭이라 축이 바뀌어도 그대로 이식 가능).
    # [실측으로 확인된 버그 - 2026-07-26] CART_BOX_FRONT_SHIFT_M으로 박스 그룹을 +X로
    # 옮긴 뒤, 대칭 크롭(중심=카트 중심, 반경 0.22)이 가장 앞쪽(+X) 박스인 Large2를
    # 더 이상 다 못 담았다 - 실측: box_id=1이 폭 0.18->0.074로 잘려서 검출됨(딱 ROI
    # 경계에서 잘린 크기). Large2의 실제 먼 쪽 끝은 cart_center + (_cart_large_dx +
    # CART_LARGE_SIZE_XY/2 + CART_BOX_FRONT_SHIFT_M) = 0.165+0.09+0.05 = 0.305m라
    # 0.22로는 어차피 못 덮는다. 크롭 중심도 박스 그룹과 함께 +X로 옮기고, 반경도
    # 그룹 전체 반스팬(0.255)을 여유 있게 덮도록 키운다(카트 벽 cart_half_x=0.448보다는
    # 여전히 확실히 안쪽).
    CART_SCAN_ROI_CENTER_X_SHIFT_M = CART_BOX_FRONT_SHIFT_M
    CART_SCAN_ROI_HALF_X_M = 0.29
    CART_SCAN_ROI_HALF_Y_M = 0.22

    OPTICAL_TO_USD_CAMERA_AXES = np.diag([1.0, -1.0, -1.0])

    accumulated_world_points = []

    for i, x_offset in enumerate(CART_SCAN_STRAFE_X_OFFSETS):
        strafe_x = cart_center_xy[0] + x_offset
        drive_to(target_x=strafe_x, target_y=SCAN_STANDOFF_XY[1], target_yaw_deg=0.0,
                 label=f"스캔 위치 {i}(x_offset={x_offset:+.2f})")

        # [설계 변경 - 사용자 지적] 원래는 매 시점마다 관절을 초기 자세로 리셋(보간
        # 이동)한 뒤 처음부터 다시 350스텝 수렴시켰다(IK 오차가 시점을 거칠수록
        # 누적되는 문제를 막기 위한 조치였음). 그런데 이 방식은 "카메라를 원상태로
        # 되돌렸다가 다시 스캔 자세로 이동"하는 불필요한 왕복 동작으로 보여서
        # 부자연스럽다는 지적을 받았다.
        #
        # 각 시점의 목표(target_pos/target_quat)는 "베이스 기준 상대 자세"로 보면
        # 거의 동일하다 - look_at이 strafe_x를 그대로 따라가는 순수 평행이동 관계라,
        # 팔의 물리적 도달 거리 문제(리프트 높이/EYE_HEIGHT 조정으로 이미 해결, 3mm
        # 수렴)만 없었다면 애초에 관절이 시점마다 크게 바뀔 이유가 없었다. 그래서
        # 팔을 리셋하지 않고 이전 시점에서 수렴된 자세를 그대로 이어받는다 - 베이스가
        # strafe로 이동하는 동안 팔은 가만히 있다가, 도착 후 아주 짧게만(이미 거의
        # 맞는 자세이므로) 미세 조정한다. 첫 시점(i==0)만 초기 자세에서 출발하므로
        # 조금 더 긴 스텝 예산을 준다.
        #
        # eye/look_at의 축 배정이 88번과 반대다 - 88번은 standoff가 X(폭)라 eye의 X에
        # 틸트 오프셋을, Y(길이)에 strafe_y를 넣었다. 이제 standoff가 Y(폭)이므로 틸트
        # 오프셋은 eye의 Y에(로봇이 SCAN_STANDOFF_XY처럼 -Y쪽에서 접근하므로 부호도
        # 마이너스 - eye가 카트 중심보다 로봇 쪽(-Y)으로 더 물러나 있어야 35.py와 동일한
        # 기하학적 틸트가 나온다), strafe_x는 eye/look_at의 X에 그대로 들어간다.
        scan_eye_i = np.array([
            strafe_x,
            cart_center_xy[1] - _scan_horizontal_offset,
            CART_BASKET_FLOOR_Z + EYE_HEIGHT_ABOVE_CART,
        ])
        scan_look_at_i = np.array([strafe_x, cart_center_xy[1], CART_BASKET_FLOOR_Z])
        target_pos, target_quat = lookat_to_link6_target(scan_eye_i, scan_look_at_i)
        move_steps = 350 if i == 0 else 90
        move_link6(target_pos, steps=move_steps, label=f"스캔 위치 {i} 자세 수렴", orientation=target_quat)

        # 실측 확인(중요 버그): 여기서 순수 world.step()만 여러 번 돌리면 set_lift_height()가
        # 호출되지 않아서(step_hold()와 달리) M0609가 그 스텝 동안 텔레포트로 "붙잡혀"
        # 있지 않고 중력에 그대로 노출된다 - 독립 articulation이라 실제로 아래로 떨어지고,
        # 그 상태에서 depth를 캡처하니 포인트클라우드가 의도한 높이보다 한참 낮게(z<0.6)
        # 나왔다. step_hold()를 써서 계속 텔레포트로 고정한 채 렌더 파이프라인만 따라잡게 한다.
        # (POST_CONVERGENCE_SETTLE_STEPS 정의부 참고 - 20 -> 90으로 늘려 X/Y 위치 오차
        # 원인이 렌더 지연인지 실험한다.)
        step_hold(POST_CONVERGENCE_SETTLE_STEPS)
        vp_util.capture_viewport_to_file(viewport, str(OUT_DIR / f"_cartscan_view_{i}.png"))

        # 실측 확인: 수렴 직후 첫 호출에서 렌더 파이프라인이 아직 안 따라와 get_pointcloud()가
        # 빈/기형(1차원) 배열을 반환하는 경우가 있었다(스캔 위치 0에서 실제로 발생 - IndexError로
        # 스크립트 전체가 죽음). 렌더를 몇 스텝 더 돌리며 최대 3회 재시도하고, 그래도 안 되면
        # 이 시점만 건너뛴다(전체 스캔을 죽이지 않음 - 다른 시점들로도 충분히 커버 가능).
        pts_world_i = None
        for retry in range(3):
            candidate = np.asarray(camera.get_pointcloud(world_frame=True))
            if candidate.ndim == 2 and candidate.shape[1] == 3 and len(candidate) > 0:
                pts_world_i = candidate
                break
            print(f"[경고] 스캔 위치 {i}: get_pointcloud() 결과가 비정상(shape={candidate.shape}) "
                  f"-> 재시도 {retry + 1}/3", flush=True)
            step_hold(15)

        if pts_world_i is None:
            print(f"[경고] 스캔 위치 {i}: point cloud 획득 실패 - 이 시점은 건너뜀", flush=True)
            continue

        keep = (
            (pts_world_i[:, 0] >= cart_center_xy[0] + CART_SCAN_ROI_CENTER_X_SHIFT_M - CART_SCAN_ROI_HALF_X_M)
            & (pts_world_i[:, 0] <= cart_center_xy[0] + CART_SCAN_ROI_CENTER_X_SHIFT_M + CART_SCAN_ROI_HALF_X_M)
            & (pts_world_i[:, 1] >= cart_center_xy[1] - CART_SCAN_ROI_HALF_Y_M)
            & (pts_world_i[:, 1] <= cart_center_xy[1] + CART_SCAN_ROI_HALF_Y_M)
            # 실측 확인: CART_BASKET_FLOOR_Z(0.68)가 하드코딩된 추정값이라, 처리된
            # point cloud에 바스켓 철망 테두리만 잡히고 바닥/박스가 전혀 안 보였다 -
            # 진짜 바닥이 이 추정치보다 낮은 곳에 있을 가능성이 커서, 실제 위치를
            # 알아내기 위해 하한을 훨씬 넉넉하게 낮춘다(원인 파악 후 상수 자체를 보정 예정).
            & (pts_world_i[:, 2] >= CART_BASKET_FLOOR_Z - 0.30)
            & (pts_world_i[:, 2] <= CART_BASKET_FLOOR_Z + CART_SCAN_ROI_MAX_HEIGHT_M)
        )
        pts_world_i = pts_world_i[keep]
        accumulated_world_points.append(pts_world_i)
        print(f"[카트 스캔 {i}] x_offset={x_offset:+.2f} world_points={len(pts_world_i)}", flush=True)

    # ================= 3. 기준 위치(중앙)로 복귀 + base_link 기준 변환/저장 =================
    # 88번은 strafe 축(Y)만 중앙으로 되돌렸다(standoff 축 X는 애초에 안 바뀌었으므로) - 이제
    # strafe 축이 X이므로 X를 cart_center_xy[0]으로 되돌리고, standoff 축(Y)은 그대로 둔다.
    drive_to(target_x=cart_center_xy[0], target_y=SCAN_STANDOFF_XY[1], target_yaw_deg=0.0,
             label="스캔 기준 위치(중앙) 복귀")
    snapshot(
        eye=[SCAN_STANDOFF_XY[0] - 0.8, SCAN_STANDOFF_XY[1] - 1.0, cart_center_xy[1] + 1.5],
        target=[cart_center_xy[0], cart_center_xy[1], 0.4],
        fname="_cartscan_02_scan_center.png",
    )

    if not accumulated_world_points:
        raise RuntimeError("모든 스캔 시점에서 point cloud 획득에 실패했습니다 - 카메라/렌더 파이프라인을 점검하세요.")

    base_pos_final, base_quat_final = base_robot.get_world_pose()
    base_pos_final = np.array(base_pos_final) + np.array([0.0, 0.0, lift_state["h"]])
    R_base = quat_wxyz_to_matrix(np.array(base_quat_final))

    merged_world_points = np.vstack(accumulated_world_points)
    merged_base_points = (R_base.T @ (merged_world_points - base_pos_final).T).T.astype(np.float32)

    scan_cache_path = PERCEPTION_DIR / "scan_cache" / "merged_cart_scan.npy"
    scan_cache_path.parent.mkdir(parents=True, exist_ok=True)
    np.save(scan_cache_path, merged_base_points)
    print(f"[카트 스캔] {len(CART_SCAN_STRAFE_X_OFFSETS)}개 시점 누적, 총 {len(merged_base_points)}포인트 "
          f"-> {scan_cache_path}", flush=True)

    # ================= 4. base_to_camera_transform.json 저장 + ROS2 카메라 브리지 연결 =================
    # (레거시 단일 프레임 경로용 - box_top_extractor.py가 그대로 읽을 수 있도록 유지.
    # 새 다중 시점 파이프라인은 위에서 저장한 merged_cart_scan.npy를
    # perception/run_scan_batch.py로 직접 처리하므로 이 섹션과 무관하게 동작한다.)
    step_hold(10)
    cam_pos_final, cam_quat_final = camera.get_world_pose(camera_axes=CAMERA_AXES)
    R_cam_final = quat_wxyz_to_matrix(np.array(cam_quat_final))
    R_base_to_cam = R_base.T @ R_cam_final @ OPTICAL_TO_USD_CAMERA_AXES
    t_base_to_cam = R_base.T @ (np.array(cam_pos_final) - base_pos_final)

    transform_path = PERCEPTION_DIR / "base_to_camera_transform.json"
    transform_payload = {
        "R": R_base_to_cam.tolist(),
        "t": t_base_to_cam.tolist(),
        "note": (
            "88.cart_scan_holonomic.py가 만든 스캔 기준(중앙) 자세 전용(레거시 단일 프레임 경로용). "
            "팔/베이스가 이 자세를 벗어나면 무효 - 재측정 필요. 다중 시점 파이프라인은 "
            "scan_cache/merged_cart_scan.npy를 직접 쓴다."
        ),
        "measured_base_pos": base_pos_final.tolist(),
        "measured_base_quat": np.asarray(base_quat_final).tolist(),
        "measured_camera_pos": np.asarray(cam_pos_final).tolist(),
        "measured_camera_quat": np.asarray(cam_quat_final).tolist(),
        "scan_tilt_from_vertical_deg": SCAN_TILT_FROM_VERTICAL_DEG,
    }
    transform_path.write_text(json.dumps(transform_payload, indent=2))
    print(f"[저장] {transform_path}", flush=True)

    try:
        setup_ros2_camera_bridge(camera_prim_path)
        print(f"[ROS2] {DEPTH_TOPIC}, {CAMERA_INFO_TOPIC} 발행 시작 (frame_id={CAMERA_FRAME_ID})", flush=True)
    except Exception as e:
        # 로봇 자세/좌표 결과(오늘의 핵심)는 이미 전부 확보됐으므로, ROS2 브리지 환경 문제
        # (예: ROS2 미소싱으로 isaacsim.ros2.bridge 익스텐션이 조용히 shutdown됨) 때문에
        # 스크립트 전체가 죽지 않게 한다 - 별도로 환경을 점검해야 하는 문제.
        print(f"[경고] ROS2 카메라 브리지 연결 실패 - {e}", flush=True)
        print("[경고] ROS2 환경(source /opt/ros/humble/setup.bash 등)이 Isaac Sim 실행 전에 "
              "소싱됐는지 확인 필요. 로봇 자세/transform은 정상 저장됨.", flush=True)

    # ================= 카메라 자신이 보는 화면 저장 (FOV 클리핑 확인용) =================
    step_hold(20)
    cam_out = str(OUT_DIR / "_cartscan_04_camera_view.png")
    try:
        rgba = camera.get_rgba()
        if rgba is not None and rgba.size > 0:
            import matplotlib.pyplot as plt
            plt.imsave(cam_out, rgba)
            print(f"[SCREENSHOT] {cam_out} (카메라 시점)", flush=True)
    except Exception as e:
        print(f"[경고] 카메라 시점 저장 실패: {e}", flush=True)

    print("\n[안내] 다중 시점 카트 스캔 완료. 다음 단계:", flush=True)
    print("  1) 별도 터미널에서 (perception venv, ROS2 불필요):", flush=True)
    print("       source perception/.venv/bin/activate", flush=True)
    print("       cd perception", flush=True)
    print(f"       python3 run_scan_batch.py --input {scan_cache_path} --marker <marker_path>", flush=True)
    print(f"  2) 결과는 ~/box_pointcloud/all_boxes_corners_*.json, all_boxes_completed_*.ply 에 저장됨", flush=True)
    print("  (레거시: 라이브 단일 프레임이 필요하면 box_top_extractor.py를 별도 venv에서 실행 - "
          "위 base_to_camera_transform.json이 이 스크립트가 남긴 중앙 자세 기준)", flush=True)
    print("     (다음 단계 스크립트에서 box_scan.json 스키마로 변환 예정 - 계획 파일 참고)\n", flush=True)


    return_to_spawn_pose()
    _publish_status("cart_scan", "done")


def run_trunk_scan():
    """89.py 본문(카메라/컨트롤러/조준 함수는 이 함수 안에서만 유효한 로컬 재정의) -
    run_cart_scan()과 동일 원칙. LIFT_MAX=LIFT_MIN+0.45(89.py 고유값)도 로컬로만 쓴다."""
    _publish_status("trunk_scan", "started")
    TRUNK_Y_MIN, TRUNK_Y_MAX = -0.663, 0.664
    TRUNK_FLOOR_Z = 0.459
    TRUNK_WALL_TOP = 1.010
    SDF_RESOLUTION = 256

    # 12.py와 동일한 앵커/스윕 기하 - 차량 모델 실측값 기준이라 로봇이 바뀌어도 그대로 재사용.
    ANCHOR_Y = 0.0
    ANCHOR_OUTSIDE_OFFSET = 0.20
    ANCHOR_HEIGHT_ABOVE_FLOOR = 0.33
    DEEP_WALL_MARGIN = 0.08
    DEEP_CENTER_HEIGHT = 0.4
    SIDE_MARGIN = 0.10
    FLOOR_MARGIN = 0.02
    CEILING_MARGIN = 0.05
    CAMERA_AXES = "usd"
    WORLD_UP = (0.0, 0.0, 1.0)

    BASIC_STEPS = 350
    SWEEP_STEPS = 200

    # ---------------- 82~88번과 동일 홀로노믹 베이스 구성 ----------------
    DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 0.0, 50.0, 20.0
    BASE_PATH = "/World/HoloBase"
    CHASSIS_PATH = f"{BASE_PATH}/chassis"
    # 트렁크는 카트와 달리 접근축 전체가 뚫려있어(옆벽 제약 없음) 12.py의 FACE_ROT_Z=0(긴 축이
    # 트렁크를 정면으로 향함) 관례를 그대로 쓸 수 있다 - 91번(PLACE, 차량 하부 진입)에서만
    # 이 축이 실제로 의미를 가지며, 지금(스캔)은 표준 standoff에서 대기한다.
    BASE_FACE_ROT_Z = 0.0

    ROLLER_COUNT = 9
    ROLLER_MASS = 0.02
    HUB_MASS = 1.0
    CHASSIS_MASS = 15.0

    M0609_USD = str(M0609_DIR / "Collected_m0609_vgp20_camera" / "m0609_vgp20_camera.usd")
    M0609_URDF_PATH = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
    M0609_DESCRIPTION_PATH = str(M0609_DIR / "rmpflow/m0609_description.yaml")
    M0609_RMPFLOW_CONFIG_PATH = str(M0609_DIR / "rmpflow/m0609_rmpflow_common.yaml")
    M0609_MOUNT_Z_ABOVE_CHASSIS_TOP = 0.02
    LIFT_COLUMN_RADIUS = 0.045
    LIFT_TRAVEL_M = 0.45

    EE_LINK_NAME = "link_6"
    GRIPPER_BODY_NAME = "vgp20_suction_plate"
    DEPTH_CAMERA_NAME_HINT = "Depth"

    GRIPPER_RANGE_JSON = M0609_DIR / "Collected_m0609_vgp20_camera" / "_gripper_physical_range.json"
    if GRIPPER_RANGE_JSON.exists():
        _range = json.loads(GRIPPER_RANGE_JSON.read_text())
        TIP_LOCAL_OFFSET = tuple(_range["tip_local_offset"])
        print(f"[그리퍼] {GRIPPER_RANGE_JSON}에서 로드: tip_local_offset={TIP_LOCAL_OFFSET}", flush=True)
    else:
        TIP_LOCAL_OFFSET = (0.0, 0.0, 0.0188)
        print(f"[경고] {GRIPPER_RANGE_JSON} 없음 - 플레이스홀더 tip_local_offset={TIP_LOCAL_OFFSET} 사용", flush=True)

    STANDOFF_MARGIN = 0.15
    DEPTH_TOPIC = "/camera/depth"
    CAMERA_INFO_TOPIC = "/camera/camera_info"
    CAMERA_FRAME_ID = "m0609_depth_camera_optical_frame"
    CAMERA_WIDTH, CAMERA_HEIGHT = 640, 480
    def setup_ros2_camera_bridge(camera_prim_path):
        """32/88.py와 동일 패턴."""
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/World/ROS2_Trunk_Scan_Camera_Graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "DepthPublish.inputs:execIn"),
                    ("CreateRenderProduct.outputs:renderProductPath", "DepthPublish.inputs:renderProductPath"),
                    ("CreateRenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                    ("CreateRenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
                ],
                keys.SET_VALUES: [
                    ("CreateRenderProduct.inputs:cameraPrim", camera_prim_path),
                    ("CreateRenderProduct.inputs:width", CAMERA_WIDTH),
                    ("CreateRenderProduct.inputs:height", CAMERA_HEIGHT),
                    ("DepthPublish.inputs:type", "depth"),
                    ("DepthPublish.inputs:topicName", DEPTH_TOPIC),
                    ("DepthPublish.inputs:frameId", CAMERA_FRAME_ID),
                    ("DepthPublish.inputs:resetSimulationTimeOnStop", True),
                    ("CameraInfoPublish.inputs:topicName", CAMERA_INFO_TOPIC),
                    ("CameraInfoPublish.inputs:frameId", CAMERA_FRAME_ID),
                    ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                ],
            },
        )




    # 92.trunk_place_holonomic.py와 동일 환경 - 실제 로봇이 도달 가능한 한계(x≈3.4, STAGE 3.2.1
    # 실측 기반)보다 안쪽은 어차피 못 쓰므로, 스캔 단계에서부터 트렁크 안에 얇은 가상 뒷벽을
    # 세워둔다. 92.py와 달리 여기서는 raycast가 아니라 RGB-D 카메라로 점유 공간을 스캔하므로,
    # 이 벽이 카메라에 실제로 "보여야" 포인트클라우드/trunk_map.json의 depth에 반영된다 -
    # 즉 여기서는 라이브 raycast 필터 갱신이 필요 없고(89.py는 raycast 인프라 자체가 없음),
    # 카메라가 인식할 불투명 솔리드 프림이면 충분하다. 92.py와 동일하게 /World/Vehicle의
    # 자식이 아니라 독립 프림으로 만든다(부모의 usdz 단위변환 scale/회전 때문에 좌표가 다시
    # 해석되는 문제를 92.py에서 실측으로 확인했음 - 처음부터 그 실수를 피한다). z는 92.py의
    # CEILING_WORLD_Z(trunk_map 실측 천장 - 89/90번이 아직 안 만들어졌으니 여기선 그 값이 될
    # 설계상수 TRUNK_WALL_TOP)와 TRUNK_FLOOR_Z를 그대로 쓴다.

    area_light = UsdLux.SphereLight.Define(stage, "/World/TrunkScanAreaLight")
    area_light.CreateRadiusAttr(0.3)
    area_light.CreateIntensityAttr(80000)
    UsdGeom.Xformable(area_light).AddTranslateOp().Set(Gf.Vec3d(TRUNK_X_MIN + 0.3, 0.0, TRUNK_FLOOR_Z + 1.0))

    # ---- 로봇 standoff: 12.py의 ROBOT_XY=(TRUNK_X_MIN-0.85,-0.15) 대신, 이 홀로노믹 베이스의
    # 실제 반길이 기준으로 표준 standoff를 계산한다(84/88번과 동일 공식, 축만 반길이로 교체) ----


    MEASURED_CHASSIS_TOP_OFFSET = 0.0180
    LIFT_MIN = MEASURED_CHASSIS_TOP_OFFSET + M0609_MOUNT_Z_ABOVE_CHASSIS_TOP
    LIFT_MAX = LIFT_MIN + LIFT_TRAVEL_M





    _init_joints = np.zeros(m0609_robot.num_dof)
    if "joint_3" in m0609_robot.dof_names:
        _init_joints[m0609_robot.dof_names.index("joint_3")] = np.pi / 2
    if "joint_5" in m0609_robot.dof_names:
        _init_joints[m0609_robot.dof_names.index("joint_5")] = np.pi / 2
    m0609_robot.set_joint_positions(_init_joints)



    def set_lift_height(h):
        chassis_pos, chassis_quat = base_robot.get_world_pose()
        target_pos = np.array([float(chassis_pos[0]), float(chassis_pos[1]), float(chassis_pos[2]) + h])
        m0609_robot.set_world_pose(position=target_pos, orientation=chassis_quat)
        m0609_robot.set_linear_velocity(np.zeros(3))
        m0609_robot.set_angular_velocity(np.zeros(3))
        column_base_z = float(chassis_pos[2]) + LIFT_MIN
        column_len = max(float(h) - LIFT_MIN, 0.001)
        lift_scale_op.Set(Gf.Vec3f(1.0, 1.0, column_len))
        lift_translate_op.Set(Gf.Vec3d(float(chassis_pos[0]), float(chassis_pos[1]), column_base_z + column_len / 2.0))


    def step_hold(n=1):
        for _ in range(n):
            set_lift_height(lift_state["h"])
            world.step(render=True)


    def move_lift_to(target_h, steps=90):
        start_h = lift_state["h"]
        for i in range(steps):
            h = start_h + (target_h - start_h) * (i + 1) / steps
            set_lift_height(h)
            world.step(render=True)
        lift_state["h"] = target_h
        print(f"[리프트] {start_h:.3f} -> {target_h:.3f}", flush=True)


    def holo_forward(vx, vy, wz):
        speeds = mecanum_wheel_speeds(vx, vy, wz, WHEEL_RADIUS, k_factor)
        return ArticulationAction(joint_velocities=speeds, joint_indices=hub_dof_indices)


    SMOOTH_ALPHA = 0.12
    _smooth_state = {"vx": 0.0, "vy": 0.0, "wz": 0.0}


    def drive_to(target_x=None, target_y=None, target_yaw_deg=None, tolerance_xy=0.03, tolerance_yaw_deg=2.0,
                 max_speed=0.4, max_wz=0.2, kp_xy=1.8, kp_yaw=0.25, max_steps=3000, label=""):
        """83/85/86/87/88번과 동일한 폐루프 주행."""
        start_pos, start_quat = base_robot.get_world_pose()
        start_yaw = float(np.degrees(quat_to_euler_angles(start_quat)[2]))
        tx = target_x if target_x is not None else float(start_pos[0])
        ty = target_y if target_y is not None else float(start_pos[1])
        tyaw = target_yaw_deg if target_yaw_deg is not None else start_yaw
        print(f"\n[주행 시작]{' ' + label if label else ''} 목표=({tx:.3f},{ty:.3f},{tyaw:.1f}deg)", flush=True)

        STALL_WINDOW, STALL_MIN_PROGRESS = 150, 0.008
        last_check_pos = np.array([float(start_pos[0]), float(start_pos[1])])
        stalled = False
        step = 0
        for step in range(1, max_steps + 1):
            pos, quat = base_robot.get_world_pose()
            yaw_deg = float(np.degrees(quat_to_euler_angles(quat)[2]))
            ex_w, ey_w = tx - float(pos[0]), ty - float(pos[1])
            eyaw = ((tyaw - yaw_deg + 180) % 360) - 180
            if abs(ex_w) < tolerance_xy and abs(ey_w) < tolerance_xy and abs(eyaw) < tolerance_yaw_deg:
                break
            yaw_rad = np.radians(yaw_deg)
            ex_l = ex_w * np.cos(yaw_rad) + ey_w * np.sin(yaw_rad)
            ey_l = -ex_w * np.sin(yaw_rad) + ey_w * np.cos(yaw_rad)
            vx_t = float(np.clip(kp_xy * ex_l, -max_speed, max_speed))
            vy_t = float(np.clip(kp_xy * ey_l, -max_speed, max_speed))
            wz_t = float(np.clip(np.radians(kp_yaw * eyaw), -max_wz, max_wz))
            _smooth_state["vx"] += SMOOTH_ALPHA * (vx_t - _smooth_state["vx"])
            _smooth_state["vy"] += SMOOTH_ALPHA * (vy_t - _smooth_state["vy"])
            _smooth_state["wz"] += SMOOTH_ALPHA * (wz_t - _smooth_state["wz"])
            base_robot.apply_action(holo_forward(_smooth_state["vx"], _smooth_state["vy"], _smooth_state["wz"]))
            step_hold(1)
            if step % STALL_WINDOW == 0:
                cur = np.array([float(pos[0]), float(pos[1])])
                progress = float(np.linalg.norm(cur - last_check_pos))
                if progress < STALL_MIN_PROGRESS and (abs(ex_w) > tolerance_xy or abs(ey_w) > tolerance_xy):
                    stalled = True
                    print(f"  [정체 감지] {progress:.4f}m밖에 못 움직임 - 중단", flush=True)
                    break
                last_check_pos = cur
        for _ in range(30):
            _smooth_state["vx"] *= 1 - SMOOTH_ALPHA
            _smooth_state["vy"] *= 1 - SMOOTH_ALPHA
            _smooth_state["wz"] *= 1 - SMOOTH_ALPHA
            base_robot.apply_action(holo_forward(_smooth_state["vx"], _smooth_state["vy"], _smooth_state["wz"]))
            step_hold(1)
        final_pos, final_quat = base_robot.get_world_pose()
        final_yaw = float(np.degrees(quat_to_euler_angles(final_quat)[2]))
        print(f"[주행 완료]{' ' + label if label else ''} {step}스텝, 최종=({final_pos[0]:.3f},{final_pos[1]:.3f},"
              f"{final_yaw:.1f}deg) 정체={stalled}", flush=True)
        return final_pos, final_yaw, not stalled


    step_hold(60)
    print("\n[안정화 완료]\n", flush=True)

    # ================= 카메라 + link6<->camera/그리퍼끝 오프셋 측정 (12.py와 동일 패턴) =================
    camera_prim_path, all_cameras = find_camera_prim_path(stage, m0609_path, DEPTH_CAMERA_NAME_HINT)
    if camera_prim_path is None:
        raise RuntimeError(f"카메라 프림을 못 찾음 - 발견된 카메라 후보: {all_cameras}")
    print(f"[CAMERA] 스캔에 사용할 depth 카메라: {camera_prim_path} (후보 전체: {all_cameras})", flush=True)
    camera = Camera(prim_path=camera_prim_path, resolution=(CAMERA_WIDTH, CAMERA_HEIGHT))
    camera.initialize()
    camera.add_distance_to_image_plane_to_frame()
    camera.add_pointcloud_to_frame()
    camera.add_rgb_to_frame()
    step_hold(10)

    link6_pos0, link6_quat0 = m0609_robot.end_effector.get_world_pose()
    cam_pos0, cam_quat0 = camera.get_world_pose(camera_axes=CAMERA_AXES)
    R_link6_0 = quats_to_rot_matrices(np.array([link6_quat0]))[0]
    R_cam_0 = quats_to_rot_matrices(np.array([cam_quat0]))[0]
    R_offset = R_link6_0.T @ R_cam_0
    cam_local_pos_offset = R_link6_0.T @ (np.array(cam_pos0) - np.array(link6_pos0))
    print(f"[오프셋] R_offset(link6->camera)=\n{R_offset}\ncamera pos offset in link6 frame={cam_local_pos_offset}",
          flush=True)

    # 12.py는 2-finger 그리퍼라 좌우 finger 링크 평균으로 손끝을 측정했지만, 이 로봇은 평평한
    # 흡착판이라 84~88번처럼 그리퍼 바디 프림 + TIP_LOCAL_OFFSET(_gripper_physical_range.json)로
    # "흡착판 끝" 월드 위치를 직접 계산한다.
    gripper_body_mat0 = UsdGeom.Xformable(stage.GetPrimAtPath(gripper_body_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    gripper_tip_world0 = np.array(gripper_body_mat0.Transform(Gf.Vec3d(*TIP_LOCAL_OFFSET)))
    gripper_tip_local_offset = R_link6_0.T @ (gripper_tip_world0 - np.array(link6_pos0))
    print(f"[오프셋] gripper tip(흡착판 끝) pos offset in link6 frame={gripper_tip_local_offset}", flush=True)


    def lookat_to_link6_target(anchor_world, look_at, up=WORLD_UP):
        """12.py의 anchor_mode="gripper_tip" 분기와 완전히 동일 - 흡착판 끝(anchor_world)을
        고정하고 카메라가 look_at을 보도록 link6 목표 pos+quat을 역산한다."""
        tip_world = np.asarray(anchor_world, dtype=float)
        look_at = np.asarray(look_at, dtype=float)
        R_link6_target = R_link6_0.copy()
        for _ in range(4):
            camera_eye = tip_world + R_link6_target @ (cam_local_pos_offset - gripper_tip_local_offset)
            R_cam_target = make_usd_camera_rotation(camera_eye, look_at, up)
            R_link6_target = R_cam_target @ R_offset.T
        link6_target_pos = tip_world - R_link6_target @ gripper_tip_local_offset
        q_link6_target = rot_matrices_to_quats(np.array([R_link6_target]))[0]
        return link6_target_pos, q_link6_target


    def camera_alignment_check(look_at):
        cam_pos_now, cam_quat_now = camera.get_world_pose(camera_axes=CAMERA_AXES)
        R_cam_now = quats_to_rot_matrices(np.array([cam_quat_now]))[0]
        forward_now = R_cam_now @ np.array([0.0, 0.0, -1.0])
        up_now = R_cam_now @ np.array([0.0, 1.0, 0.0])
        to_target_dir = _normalize(np.asarray(look_at, dtype=float) - np.asarray(cam_pos_now, dtype=float))
        alignment = float(np.dot(forward_now, to_target_dir))
        upright = float(np.dot(up_now, np.array(WORLD_UP)))
        return alignment, upright, cam_pos_now, cam_quat_now


    # ================= 리프트를 최고 높이로 (12.py의 MOUNT_Z=0.42보다 낮은 저상 마운트라
    # reach 확보를 위해 88번과 동일하게 최고 높이로 올린다) =================
    print(f"\n[리프트] 도킹({LIFT_MIN:.3f}) -> 최고({LIFT_MAX:.3f})", flush=True)
    move_lift_to(LIFT_MAX, steps=120)

    # ================= 1. 표준 주행으로 트렁크 앞 standoff 위치까지 이동 =================
    drive_to(target_x=BASE_START_XY[0], target_y=BASE_START_XY[1], target_yaw_deg=BASE_FACE_ROT_Z, label="트렁크 앞 대기 위치")

    viewport = vp_util.get_active_viewport()
    chassis_pos0, _ = base_robot.get_world_pose()
    set_camera_view(eye=[chassis_pos0[0] - 2.2, chassis_pos0[1] - 3.2, chassis_pos0[2] + 1.6],
                     target=[(chassis_pos0[0] + CAR_POS[0]) / 2, 0.0, 1.0])
    step_hold(20)
    vp_util.capture_viewport_to_file(viewport, str(OUT_DIR / "_trunkscan_00_start.png"))
    step_hold(5)
    print(f"[SCREENSHOT] {OUT_DIR / '_trunkscan_00_start.png'}", flush=True)

    # ================= 2. 기본 자세: 흡착판 끝을 트렁크 입구 낮은 위치에 고정, 카메라는
    # 트렁크 가장 깊은 곳 중앙을 본다 (12.py와 완전히 동일한 앵커/좌표) =================
    anchor_pos = np.array([
        TRUNK_X_MIN - ANCHOR_OUTSIDE_OFFSET,
        ANCHOR_Y,
        TRUNK_FLOOR_Z + ANCHOR_HEIGHT_ABOVE_FLOOR,
    ], dtype=float)

    deep_x = TRUNK_X_MAX - DEEP_WALL_MARGIN
    deep_center = np.array([deep_x, 0.0, TRUNK_FLOOR_Z + DEEP_CENTER_HEIGHT], dtype=float)

    target_pos, target_quat = lookat_to_link6_target(anchor_pos, deep_center)
    print(f"[기본 자세 목표] link6_pos={np.round(target_pos, 3)} anchor={np.round(anchor_pos, 3)} "
          f"look_at=deep_center={np.round(deep_center, 3)}", flush=True)

    controller = RMPFlowController(
        name="trunk_scan_holonomic",
        robot_articulation=m0609_robot,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=M0609_DESCRIPTION_PATH,
        rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
    )


    def sync_rmp_base():
        chassis_pos, chassis_quat = base_robot.get_world_pose()
        base_pos = np.array([float(chassis_pos[0]), float(chassis_pos[1]), float(chassis_pos[2]) + lift_state["h"]])
        controller._default_position = base_pos
        controller._default_orientation = chassis_quat
        controller.rmp_flow.set_robot_base_pose(robot_position=base_pos, robot_orientation=chassis_quat)


    def move_link6(target_pos, target_quat, steps):
        for _ in range(steps):
            sync_rmp_base()
            actions = controller.forward(target_end_effector_position=target_pos, target_end_effector_orientation=target_quat)
            m0609_robot.apply_action(actions)
            set_lift_height(lift_state["h"])
            world.step(render=True)


    move_link6(target_pos, target_quat, steps=BASIC_STEPS)

    ee_pos, ee_quat = m0609_robot.end_effector.get_world_pose()
    err = np.linalg.norm(np.array(ee_pos) - target_pos)
    alignment, upright, cam_pos_now, cam_quat_now = camera_alignment_check(deep_center)
    R_link6_now = quats_to_rot_matrices(np.array([ee_quat]))[0]
    gripper_tip_now = np.array(ee_pos) + R_link6_now @ gripper_tip_local_offset
    tip_err = np.linalg.norm(gripper_tip_now - anchor_pos)
    print(f"[기본 자세 도달] ee_pos(link6)={np.round(ee_pos, 3)} err(link6)={err:.4f}m "
          f"gripper_tip={np.round(gripper_tip_now, 3)} tip_err={tip_err:.4f}m "
          f"alignment={alignment:.3f} upright={upright:.3f} cam_pos={np.round(cam_pos_now, 3)}", flush=True)
    if tip_err > 0.05:
        print("[경고] 흡착판 끝 위치 오차가 5cm를 넘습니다 - STANDOFF_TRUNK/ANCHOR 재조정이 필요할 수 있습니다.", flush=True)
    pose_is_valid = alignment >= 0.90 and upright >= 0.80
    if not pose_is_valid:
        print("[경고] alignment 또는 upright가 낮습니다 - 카메라 마운트/오프셋을 확인하세요.", flush=True)

    vp_util.capture_viewport_to_file(viewport, str(OUT_DIR / "_trunkscan_01_basic_pose.png"))
    step_hold(5)
    print(f"[SCREENSHOT] {OUT_DIR / '_trunkscan_01_basic_pose.png'}", flush=True)

    try:
        setup_ros2_camera_bridge(camera_prim_path)
        print(f"[ROS2] {DEPTH_TOPIC}, {CAMERA_INFO_TOPIC} 발행 시작 (frame_id={CAMERA_FRAME_ID})", flush=True)
    except Exception as e:
        print(f"[경고] ROS2 카메라 브리지 연결 실패 - {e}", flush=True)
        print("[경고] ROS2 환경(source /opt/ros/humble/setup.bash 등)이 Isaac Sim 실행 전에 "
              "소싱됐는지 확인 필요.", flush=True)

    # ================= 3. 앵커 고정, look_at만 바꿔가며 5방향 스윕 (12.py와 완전히 동일) =================
    SWEEP_WAYPOINTS = [
        ("deep_center", deep_center),
        ("deep_left", np.array([deep_x, TRUNK_Y_MIN + SIDE_MARGIN, TRUNK_FLOOR_Z + DEEP_CENTER_HEIGHT])),
        ("deep_right", np.array([deep_x, TRUNK_Y_MAX - SIDE_MARGIN, TRUNK_FLOOR_Z + DEEP_CENTER_HEIGHT])),
        ("deep_floor", np.array([deep_x, 0.0, TRUNK_FLOOR_Z + FLOOR_MARGIN])),
        ("deep_ceiling", np.array([deep_x, 0.0, TRUNK_WALL_TOP - CEILING_MARGIN])),
    ]

    captured_clouds = []
    scan_meta = []

    if not pose_is_valid:
        print("\n[중단] 기본 자세가 유효하지 않아 스윕을 건너뜁니다. 스크린샷을 확인하세요.\n", flush=True)
    else:
        for name, look_at in SWEEP_WAYPOINTS:
            t_pos, t_quat = lookat_to_link6_target(anchor_pos, look_at)
            move_link6(t_pos, t_quat, steps=SWEEP_STEPS)
            ee_pos_s, _ = m0609_robot.end_effector.get_world_pose()
            err_s = np.linalg.norm(np.array(ee_pos_s) - t_pos)
            alignment, upright, cam_pos_s, cam_quat_s = camera_alignment_check(look_at)
            print(f"[스윕:{name}] look_at={np.round(look_at, 3)} ee_pos={np.round(ee_pos_s, 3)} err={err_s:.4f}m "
                  f"cam_pos={np.round(cam_pos_s, 3)} alignment={alignment:.3f} upright={upright:.3f}", flush=True)

            rgb = camera.get_rgba()[:, :, :3]
            plt.imsave(str(OUT_DIR / f"_trunkscan_sweep_{name}.png"), rgb)

            step_hold(5)
            pcd = camera.get_pointcloud(world_frame=True)
            n_pts = 0 if pcd is None else len(pcd)
            print(f"[스윕:{name}] pointcloud 점 개수={n_pts}", flush=True)
            if n_pts > 0:
                captured_clouds.append(np.asarray(pcd))
                scan_meta.append({
                    "name": name, "look_at": look_at.tolist(),
                    "cam_pos": np.asarray(cam_pos_s).tolist(), "cam_quat": np.asarray(cam_quat_s).tolist(),
                    "n_points": int(n_pts),
                })

            set_camera_view(eye=[chassis_pos0[0] - 1.2, chassis_pos0[1] - 1.5, chassis_pos0[2] + 1.3],
                             target=[float(cam_pos_s[0]), float(cam_pos_s[1]), float(cam_pos_s[2])])
            step_hold(10)
            vp_util.capture_viewport_to_file(viewport, str(OUT_DIR / f"_trunkscan_sweep_wide_{name}.png"))
            step_hold(5)

        if captured_clouds:
            merged = np.concatenate(captured_clouds, axis=0)
            pc_out = OUT_DIR / "trunk_pointcloud.npy"
            np.save(pc_out, merged)
            print(f"\n[병합] waypoint {len(captured_clouds)}개, 전체 포인트 개수={len(merged)}", flush=True)
            print(f"[저장] {pc_out}", flush=True)

            base_pos_final, base_quat_final = base_robot.get_world_pose()
            base_pos_final = np.array(base_pos_final) + np.array([0.0, 0.0, lift_state["h"]])
            meta_path = OUT_DIR / "trunk_pointcloud_meta.json"
            meta_path.write_text(json.dumps({
                "trunk_bounds": {
                    "x": [TRUNK_X_MIN, TRUNK_X_MAX], "y": [TRUNK_Y_MIN, TRUNK_Y_MAX],
                    "floor_z": TRUNK_FLOOR_Z, "wall_top_z": TRUNK_WALL_TOP,
                },
                "anchor_pos": anchor_pos.tolist(),
                "base_pos": base_pos_final.tolist(),
                "base_quat": np.asarray(base_quat_final).tolist(),
                "waypoints": scan_meta,
            }, indent=2))
            print(f"[저장] {meta_path}", flush=True)
        else:
            print("\n[경고] 캡처된 포인트가 없음", flush=True)

    print("\n[안내] 트렁크 스캔 완료. 다음 단계에서 13.export_trunk_map.py 포팅 버전으로 "
          "trunk_pointcloud.npy -> trunk_map.json 변환 예정 (계획 파일 참고).\n", flush=True)


    return_to_spawn_pose()
    _publish_status("trunk_scan", "done")



# =====================================================================
# 명령 대기 디스패치 - MSI1/UI가 표준 ROS2 서비스로 트리거한다. 호출 사이에는
# 아무 것도 안 하고 명령만 기다린다(즉 매 스텝 "저장된 목표"를 재적용하는
# 상시 루프가 없다) - execute_pick_place_action_server.py의 ROS2 재구현판이
# 겪은 RMPflow 발산 버그(HANDOFF_2026-07-27_PICK_PLACE.md 참고)는 바로 그
# 상시 루프 때문이었다. 각 run_*() 함수 자신이 100.py/89.py/99.py와 똑같이
# 내부에서 world.step()을 직접 반복 호출하는 블로킹 함수이므로, 호출 도중엔
# 경쟁이 구조적으로 불가능하다.
# =====================================================================
import threading

# platform_controller_node.py와 동일 이유 - Isaac Sim 번들 파이썬(3.11)은 시스템
# ROS2 Humble(3.10용으로 빌드된 rclpy C 확장)을 못 읽는다. isaacsim.ros2.bridge가
# 자체 포함한 3.11용 rclpy 빌드를 sys.path 맨 앞에 꽂아야 import rclpy가 성공한다.
_ROS2_BRIDGE_HUMBLE = (
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/"
    "exts/isaacsim.ros2.bridge/humble"
)
if os.path.isdir(_ROS2_BRIDGE_HUMBLE):
    sys.path.insert(0, os.path.join(_ROS2_BRIDGE_HUMBLE, "rclpy"))
    _lib_dir = os.path.join(_ROS2_BRIDGE_HUMBLE, "lib")
    os.environ["LD_LIBRARY_PATH"] = _lib_dir + os.pathsep + os.environ.get("LD_LIBRARY_PATH", "")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger as _TriggerSrv
from std_msgs.msg import String as _StatusStringMsg


class _IsaacTaskRunnerNode(Node):

    def __init__(self):
        super().__init__("isaac_task_runner")
        self._pending = None
        self._lock = threading.Lock()
        self.create_service(_TriggerSrv, "/isaac_task_runner/run_cart_scan",
                             lambda req, res: self._enqueue("cart_scan", res))
        self.create_service(_TriggerSrv, "/isaac_task_runner/run_trunk_scan",
                             lambda req, res: self._enqueue("trunk_scan", res))
        self.create_service(_TriggerSrv, "/isaac_task_runner/run_pick_and_place",
                             lambda req, res: self._enqueue("pick_and_place", res))
        # pick_and_place_action_server.py(및 향후 cart/trunk_scan 어댑터)가 구독해서
        # Action Feedback으로 중계하는 상태 토픽 - _publish_status() 헬퍼(파일 위쪽,
        # run_pick_and_place 정의부 근처) 참고. TRANSIENT_LOCAL 없이 그냥 depth=20
        # 큐로 충분 - 늦게 붙는 구독자는 "그 시점부터"만 받아도 되는 진행상황 스트림이라
        # trunk_map 같은 "래치드 최신값"과는 성격이 다르다.
        self.status_pub = self.create_publisher(_StatusStringMsg, "/isaac_task_runner/status", 20)
        self.get_logger().info("isaac_task_runner ready - cart_scan/trunk_scan/pick_and_place 대기 중")

    def _enqueue(self, name, res):
        with self._lock:
            if self._pending is not None:
                res.success = False
                res.message = f"이미 '{self._pending}' 처리 중 - 끝난 뒤 다시 시도하세요"
                return res
            self._pending = name
        res.success = True
        res.message = f"'{name}' 접수됨"
        return res

    def take_pending(self):
        with self._lock:
            name, self._pending = self._pending, None
        return name


rclpy.init()
_node = _IsaacTaskRunnerNode()
_STATUS_PUB = _node.status_pub

# ---- 실시간 로봇 카메라(RGB) ROS2 스트리밍 - [2026-07-28] 로봇 제어 UI의
# "로봇 카메라 실시간" 패널용(우선 Pick & Place 작업 화면 용도). run_cart_scan()
# 안의 setup_ros2_camera_bridge()와 완전히 같은 패턴(OnPlaybackTick으로
# 구동되는 OmniGraph - Isaac Sim 자체 ROS2 카메라 브리지)을 재사용하되 depth
# 대신 rgb, 별도 그래프 경로/토픽을 쓴다(그쪽 /World/ROS2_Cart_Scan_Camera_Graph
# 와 안 겹침).
#
# 이 블록은 world.step()이나 idle 디스패치 루프(아래 while)를 전혀 안
# 건드린다 - OnPlaybackTick은 world.step(render=True)가 실제로 렌더링을
# 수행할 때만 발화하는데, idle 루프는 대기 중엔 world.step()을 아예 안 부르니
# (RMPflow 발산 버그 재발 방지를 위해 일부러 그렇게 설계됨 - 이 섹션 맨 위
# 주석 참고) 이 스트림은 "자연히" 실제 작업(스캔/픽앤플레이스) 중에만
# 갱신되고 대기 중엔 마지막 프레임에서 멈춘다 - 대기 루프 자체의 동작은
# 손끝 하나 안 바꿔서 안전하다.
#
# og.Controller.edit() 실패(카메라 프림 못 찾음, OGN 노드 타입 이름 불일치
# 등)가 나도 cart_scan/trunk_scan/pick_and_place 자체는 계속 동작해야 하므로
# (카메라 스트리밍은 부가 기능이지 핵심 기능이 아님) 여기서만 넓게 예외
# 처리하고 넘어간다.
try:
    # [실측 확인] SimulationApp() 직후(이 시점)엔 isaacsim.ros2.bridge 익스텐션이
    # "활성화 예정"이어도 OGN 노드 타입 등록이 아직 안 끝나 있어서
    # ROS2CameraHelper 생성이 "unrecognized type"으로 실패했다 - run_cart_scan()의
    # 같은 코드는 한참 뒤(사용자가 실제로 스캔을 트리거한 시점)에 실행돼서
    # 그때는 이미 다 로드돼 있어 문제없이 동작한다. enable_extension()을 명시적으로
    # 부르고 simulation_app.update()를 몇 번 돌려서 등록이 끝날 시간을 준다.
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("isaacsim.ros2.bridge")
    for _ in range(10):
        simulation_app.update()

    _live_camera_prim_path, _live_camera_candidates = find_camera_prim_path(stage, m0609_path, "Depth")
    if _live_camera_prim_path is None:
        _node.get_logger().warn(
            f"[실시간 카메라] 카메라 프림을 못 찾아 /camera/rgb 스트리밍을 건너뜁니다 "
            f"(후보: {_live_camera_candidates})")
    else:
        _og_keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/World/ROS2_Live_RGB_Camera_Graph", "evaluator_name": "execution"},
            {
                _og_keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("RgbPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                _og_keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "RgbPublish.inputs:execIn"),
                    ("CreateRenderProduct.outputs:renderProductPath", "RgbPublish.inputs:renderProductPath"),
                ],
                _og_keys.SET_VALUES: [
                    ("CreateRenderProduct.inputs:cameraPrim", _live_camera_prim_path),
                    ("CreateRenderProduct.inputs:width", 640),
                    ("CreateRenderProduct.inputs:height", 480),
                    ("RgbPublish.inputs:type", "rgb"),
                    ("RgbPublish.inputs:topicName", "/camera/rgb"),
                    ("RgbPublish.inputs:frameId", "m0609_depth_camera_optical_frame"),
                    ("RgbPublish.inputs:resetSimulationTimeOnStop", True),
                ],
            },
        )
        _node.get_logger().info(f"[실시간 카메라] {_live_camera_prim_path} -> /camera/rgb 스트리밍 그래프 생성 완료")
except Exception as _camera_setup_err:  # noqa: BLE001 - 부가 기능이라 실패해도 본 기능엔 영향 없어야 함
    _node.get_logger().warn(f"[실시간 카메라] 스트리밍 설정 실패(무시하고 계속 진행): {_camera_setup_err}")

_TASKS = {"cart_scan": run_cart_scan, "trunk_scan": run_trunk_scan, "pick_and_place": run_pick_and_place}

try:
    while simulation_app.is_running():
        rclpy.spin_once(_node, timeout_sec=0.1)
        task_name = _node.take_pending()
        if task_name is not None:
            _node.get_logger().info(f"[isaac_task_runner] '{task_name}' 시작")
            try:
                _TASKS[task_name]()
                _node.get_logger().info(f"[isaac_task_runner] '{task_name}' 완료")
            except Exception as _e:
                import traceback
                traceback.print_exc()
                _node.get_logger().error(f"[isaac_task_runner] '{task_name}' 실패 - 위 트레이스백 참고")
                _publish_status(task_name, "error", message=str(_e))
finally:
    _node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()
