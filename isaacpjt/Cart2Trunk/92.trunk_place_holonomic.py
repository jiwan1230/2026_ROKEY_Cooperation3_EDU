"""
92.trunk_place_holonomic.py

Cart2Trunk 최종 시나리오 4단계 - 트렁크 PLACE (1차 시도).
계획 파일(~/.claude/plans/parallel-juggling-sun.md) 91번 항목("저상 베이스 트렁크 PLACE,
최고 위험") 참고 - 번호는 90/91이 이미 다른 스크립트에 쓰여서 92로 이어감.

이 스크립트는 격리된 PLACE 단독 테스트다(91.cart_pick_holonomic.py와 별개 프로세스) -
실제로 카트에서 집어오는 대신, 그리퍼에 이미 박스가 붙어있는 상태로 시작해서 PLACE
동작 자체(트렁크 접근 -> 목표 위치 하강 -> release)만 검증한다.

algorism이 계산한 placement_result.json의 position_base_frame(트렁크 스캔 당시
m0609_base_link 좌표계)을, 89.trunk_scan_holonomic.py가 trunk_pointcloud_meta.json에
저장해둔 base_pos/base_quat로 이 씬의 world 좌표로 재투영한다(크로스 세션 좌표 재투영,
33/36번에서도 쓴 패턴).

1차 시도 전략 (계획 파일 참고 - 반복 필요할 수 있음)
----
트렁크는 천장이 있어 위에서 내려가는 접근이 위험하다. 89번 스캔과 동일한 표준 standoff
위치에서, trunk_map.json이 계산한 ceiling_z(문 닫힘/뚜껑 높이 한계)보다 낮은
SAFE_TRANSIT_Z를 잡아 "수평 접근 -> 순수 수직 하강"(36.py의 PLACE 원칙과 동일)으로
목표에 도달한다. 아직 다루지 않는 것: 계획서에 명시된 "저상 베이스로 차량 하부까지
파고들어 깊이 reach를 늘리는" 전략은 이번 1차 시도에 없다 - 먼저 표준 접근으로 어디까지
되는지 확인한 뒤, reach가 부족하면 다음 라운드에서 추가한다.
"""

from isaacsim import SimulationApp

import os

HEADLESS = os.environ.get("HEADLESS", "0") == "1"
# 사용자 지시 - 한 번에 다 돌리지 말고 단계별로 나눠서 확인한다.
# STAGE=1: 박스 파지 + 트렁크에 손 넣을 준비 자세(그리퍼 하향, 박스 바닥과 평행)만 확립.
# STAGE=1.1: 위 + 박스 하단이 트렁크 입구 턱을 넘도록 높이만 올림(천장 충돌은 없게 클램프).
# STAGE=2: 위 + 홀로노믹 베이스가 의도한 근접 위치(j1_x=TRUNK_X_MIN)까지 이동(팔은 그대로).
# STAGE=3: 위 + 홀로노믹/매니퓰레이터를 함께 조금씩 조정하며 배치 위치로 정밀 접근 + PLACE.
# STAGE=4: 위 + PLACE 완료 후 지금까지의 전진 시퀀스를 역순으로 밟아 STAGE 1 상태(홀딩 자세,
#          BASE_START_XY 근처)로 후퇴.
STAGE = float(os.environ.get("STAGE", "1"))
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
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, UsdLux, UsdShade, Sdf, Gf

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
# 갱신했지만, "-0.15"라는 보정폭은 옛 스케일/기하에서 시각적으로 튜닝된 경험값이라 스케일이
# 바뀐 지금 그대로 맞는다는 보장이 없다. 일단 같은 보정폭을 그대로 적용해 시작점으로 삼되,
# STAGE=2를 돌려 EntrancePlane/SuccessPlane 마커가 실제 입구와 맞는지 스크린샷으로 반드시
# 재확인하고 필요하면 다시 튜닝할 것.
TRUNK_ENTRANCE_X = TRUNK_X_MIN - 0.15

# ---------------- 82~91번과 동일 홀로노믹 베이스 구성 ----------------
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 0.0, 50.0, 20.0
BASE_PATH = "/World/HoloBase"
CHASSIS_PATH = f"{BASE_PATH}/chassis"
BASE_FACE_ROT_Z = 0.0  # 89번과 동일 - 긴 축이 트렁크를 정면으로 향함

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
# 사용자 설계 재검토(2차) - 이 시나리오는 저상 베이스가 차량 하부 가까이(또는 밑으로) 붙는
# 경우다. 82/83/85번(차 밑 통과가 필요한 스크립트들)은 정확히 이 이유로 리프트 이동거리를
# 0.35m로 제한해뒀는데(85.py 주석: "차 밑을 지나 트렁크에 접근하는 시나리오라... 여기서는
# 그 여유가 없다"), 92번은 그 제약 없이 0.45로 커져 있었다 - 다시 0.35로 낮춰 그 안전
# 마진을 되살린다.
LIFT_TRAVEL_M = 0.35

EE_LINK_NAME = "link_6"
GRIPPER_BODY_NAME = "vgp20_suction_plate"

GRIPPER_RANGE_JSON = M0609_DIR / "Collected_m0609_vgp20_camera" / "_gripper_physical_range.json"
if GRIPPER_RANGE_JSON.exists():
    _range = json.loads(GRIPPER_RANGE_JSON.read_text())
    TIP_LOCAL_OFFSET = tuple(_range["tip_local_offset"])
else:
    TIP_LOCAL_OFFSET = (0.0, 0.0, 0.0188)

STANDOFF_MARGIN = 0.15
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

# 시험용 - 실제로는 91번이 이미 카트에서 집어온 박스를 그대로 들고 온다. 이 격리 테스트는
# 그 상태를 흉내내기 위해 그리퍼에 미리 박스를 붙여서 시작한다.
TEST_BOX_SIZE = (0.135, 0.177, 0.106)  # placement_result.json의 첫 박스 치수 참고


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


# ================= placement_result.json + trunk 좌표계 로드 =================
placement_data = json.loads(PLACEMENT_JSON.read_text())
placements = placement_data["placements"]
if not placements:
    raise SystemExit("[에러] placement_result.json에 배치된 박스가 없습니다.")
first_placement = placements[0]
print(f"[적재 계획] 첫 박스 box_id={first_placement['box_id']} "
      f"position_base_frame={first_placement['position_base_frame']} "
      f"dimensions={first_placement['dimensions']} rotated={first_placement.get('rotated')}", flush=True)

trunk_meta = json.loads(TRUNK_META_JSON.read_text())
SCAN_BASE_POS = np.asarray(trunk_meta["base_pos"], dtype=np.float64)
SCAN_BASE_QUAT = np.asarray(trunk_meta["base_quat"], dtype=np.float64)
SCAN_R_BASE = quat_wxyz_to_matrix(SCAN_BASE_QUAT)

place_pos_base = np.asarray(first_placement["position_base_frame"], dtype=np.float64)
place_dims = np.asarray(first_placement["dimensions"], dtype=np.float64)
PLACE_WORLD_MIN = SCAN_R_BASE @ place_pos_base + SCAN_BASE_POS
PLACE_WORLD_CENTER = SCAN_R_BASE @ (place_pos_base + place_dims / 2.0) + SCAN_BASE_POS
print(f"[재투영] place_world_min={np.round(PLACE_WORLD_MIN, 3)} "
      f"place_world_center={np.round(PLACE_WORLD_CENTER, 3)}", flush=True)

# 사용자 설계 재검토(2차) - 트렁크는 천장이 있어서 36.py(뚜껑 없는 크레이트) 방식인 "천장 근처
# 안전높이에서 호버 -> 수직 하강"을 그대로 쓰면, 그 높은 호버 위치에서 트렁크 안쪽까지 팔을
# 수평으로 멀리 뻗어야 해서 팔이 쭉 펴지는 부자연스러운 자세가 나온다(85번이 이미 이 전략을
# 그리드 9곳에서 실측 테스트해서 0/9 도달 실패로 확인해둔 것과 같은 문제). 대신 "박스를 이미
# 목표 높이 근처에서 들고, 옆에서 홀로노믹으로 들어가서 XY만 맞추고 살짝 내리는" 방식으로
# 바꾼다 - 이러려면 이 place_release_z/place_world_xy를 로봇 스폰보다 먼저 계산해둬야
# "안전 홀딩 자세"를 처음부터 이 높이에 잡을 수 있다.
place_release_z = float(PLACE_WORLD_MIN[2]) + RELEASE_CLEARANCE_ABOVE_FLOOR + float(place_dims[2]) + TIP_LOCAL_OFFSET[2]
place_world_xy = (float(PLACE_WORLD_CENTER[0]), float(PLACE_WORLD_CENTER[1]))
# CARRY_CLEARANCE_ABOVE_RELEASE: 홀로노믹으로 접근하는 동안 바닥에 안 끌리게 release_z보다
# 이만큼 위에서 들고 이동한다 - 36.py의 PICK_HOVER_HEIGHT_ABOVE_BOX와 같은 역할이지만 훨씬
# 작다(천장 없는 크레이트와 달리 트렁크는 애초에 낮은 높이로 들어가야 하므로).
CARRY_CLEARANCE_ABOVE_RELEASE = 0.05
HOLDING_Z = place_release_z + CARRY_CLEARANCE_ABOVE_RELEASE
print(f"[PLACE 목표 사전계산] place_world_xy={np.round(place_world_xy, 3)} "
      f"place_release_z={place_release_z:.3f} HOLDING_Z={HOLDING_Z:.3f}", flush=True)

trunk_map = json.loads(TRUNK_MAP_JSON.read_text()) if TRUNK_MAP_JSON.exists() else None
if trunk_map is not None:
    ceiling_z_base = max(v[2] for v in trunk_map["vertices"][4:8])
    CEILING_WORLD_Z = float((SCAN_R_BASE @ np.array([0.0, 0.0, ceiling_z_base]) + SCAN_BASE_POS)[2])
    print(f"[트렁크맵] ceiling_z(world)={CEILING_WORLD_Z:.3f}", flush=True)
else:
    CEILING_WORLD_Z = TRUNK_WALL_TOP
    print(f"[경고] {TRUNK_MAP_JSON} 없음 - TRUNK_WALL_TOP({TRUNK_WALL_TOP})을 천장 한계로 사용", flush=True)

# 사용자 설계 재검토(2차)로 SAFE_TRANSIT_Z 근처에서 호버하는 방식은 더 이상 안 쓰지만
# (HOLDING_Z가 대신 그 역할), 천장 한계 자체는 여전히 유용한 안전 상한선이라 남겨둔다 - 아래
# HOLDING_Z가 혹시 이 한계를 넘으면(예: 트렁크 위쪽에 쌓는 배치) 경고를 찍는다.
SAFE_TRANSIT_Z = CEILING_WORLD_Z - 0.05
if HOLDING_Z > SAFE_TRANSIT_Z:
    print(f"[경고] HOLDING_Z({HOLDING_Z:.3f})가 천장 안전 한계 SAFE_TRANSIT_Z({SAFE_TRANSIT_Z:.3f})를 "
          "넘습니다 - 이 배치 위치는 저상 측면 진입 전략으로 처리할 수 없습니다(재검토 필요).", flush=True)

# 사용자 설계(4차, STAGE 1.1) - "옆에서 밀어넣는" 진입이 되려면 박스 하단이 트렁크 입구 턱
# (문턱/범퍼 상단 실루엣)보다 높아야 한다. 정확한 턱 높이 실측값은 코드에 없다(80/81번 원본
# 프로브 스크립트가 소실돼 결과 파일만 남음) - 일단 release 높이보다 이만큼 더 올려서
# 스크린샷으로 직접 눈으로 확인하고, 부족하면 ENTRY_CLEARANCE_ABOVE_RELEASE를 조금씩 늘려가며
# 튜닝한다. 천장 안전 한계(SAFE_TRANSIT_Z)는 절대 넘지 않도록 클램프한다.
ENTRY_CLEARANCE_ABOVE_RELEASE = 0.25
ENTRY_HOLDING_Z = min(place_release_z + ENTRY_CLEARANCE_ABOVE_RELEASE, SAFE_TRANSIT_Z - 0.03)
print(f"[STAGE 1.1 사전계산] ENTRY_HOLDING_Z={ENTRY_HOLDING_Z:.3f} "
      f"(release_z+{ENTRY_CLEARANCE_ABOVE_RELEASE:.2f}, 천장한계 {SAFE_TRANSIT_Z:.3f} 이내로 클램프)", flush=True)

# 사용자 설계(5차) - LIFT_TRAVEL_M=0.35(LIFT_MAX≈0.388)는 "차체 밑을 지나는" 시나리오의
# 안전마진인데, 스크립트 시작부터 계속 LIFT_MAX에 고정해두고 그 이후 ENTRY_HOLDING_Z(0.83)
# ->place_release_z 낙차를 팔 혼자서만 커버해왔다 - 팔이 그 큰 낙차+수평 reach를 동시에
# 감당하는 자세에서 팔꿈치/팔뚝이 트렁크 입구 프레임을 스쳤다(91.py PICK Phase A와 같은
# 원리로 해결: 리프트로 마운트 자체를 목표 높이 가까이 올리면 팔은 작은 나머지 거리만
# 커버하면 되어 자세가 컴팩트하게 유지된다). 이 시점(STAGE 3 마지막 PLACE 하강)에는 섀시가
# 이미 차체 밑이 아니라 트렁크 입구/안쪽에 있으므로, under-car 캡(LIFT_MAX) 대신 트렁크
# 천장 안전한계(SAFE_TRANSIT_Z)까지 리프트를 더 올려도 된다.
# 사용자 실측(재검토) - release_z-0.05(≈0.53) 기준으로 STAGE 3을 돌려보니 효과는 확인됐지만
# 조금 더 올려도 여유가 있어 보였다 - 0.65로 상향(마운트가 release_z보다 살짝 높아져 팔이
# 아주 약간만 아래로 향하면 되는 상태, 여전히 천장 안전한계 이내).
PLACE_LIFT_MAX = min(0.65, SAFE_TRANSIT_Z - 0.05)
print(f"[PLACE 하강용 리프트 상한] PLACE_LIFT_MAX={PLACE_LIFT_MAX:.3f} "
      f"(천장한계 {SAFE_TRANSIT_Z:.3f} 이내로 클램프)", flush=True)


# ================= 씬 구성 =================
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
target_mpu = UsdGeom.GetStageMetersPerUnit(stage)
target_up = UsdGeom.GetStageUpAxis(stage)

add_asset(stage, "/World/Vehicle", CAR_USD, Gf.Vec3d(*CAR_POS), CAR_EXTRA_SCALE, target_mpu, target_up, rot_z=CAR_ROT_Z)
for _ in range(20):
    simulation_app.update()
add_sdf_collision(stage, "/World/Vehicle")

area_light = UsdLux.SphereLight.Define(stage, "/World/TrunkPlaceAreaLight")
area_light.CreateRadiusAttr(0.3)
area_light.CreateIntensityAttr(80000)
UsdGeom.Xformable(area_light).AddTranslateOp().Set(Gf.Vec3d(TRUNK_X_MIN + 0.3, 0.0, TRUNK_FLOOR_Z + 1.0))

CHASSIS_HALF_LENGTH_EFFECTIVE_LOCAL = CHASSIS_HALF_LENGTH_EFFECTIVE
STANDOFF_TRUNK = CHASSIS_HALF_LENGTH_EFFECTIVE_LOCAL + STANDOFF_MARGIN
BASE_START_XY = (TRUNK_X_MIN - STANDOFF_TRUNK - 0.3, ANCHOR_Y)
chassis_path, hub_joint_paths, k_factor = build_holonomic_base(stage, BASE_START_XY, BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)

MEASURED_CHASSIS_TOP_OFFSET = 0.0180
LIFT_MIN = MEASURED_CHASSIS_TOP_OFFSET + M0609_MOUNT_Z_ABOVE_CHASSIS_TOP
LIFT_MAX = LIFT_MIN + LIFT_TRAVEL_M
m0609_path, m0609_base_link_path, lift_translate_op, lift_scale_op = mount_m0609(stage, LIFT_MIN)
gripper_body_path = f"{m0609_path}/{GRIPPER_BODY_NAME}"
ee_path = f"{m0609_path}/{EE_LINK_NAME}"

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

print(f"\n[리프트] 도킹({LIFT_MIN:.3f}) -> 최고({LIFT_MAX:.3f})", flush=True)
move_lift_to(LIFT_MAX, steps=120)

# 사용자 지적 - 원래 여기 있던 drive_to(BASE_START_XY)는 목표가 스폰 위치와 완전히 같아서
# 실질적으로 아무 데도 안 움직이는 겉치레 호출이었다(로봇은 이미 build_holonomic_base()로
# BASE_START_XY에 스폰됨). 진짜 "트렁크에 더 가깝게 붙는" 주행은 안전 홀딩 자세 확립 +
# 시험용 박스 부착 이후로 미룬다(아래 참고) - 지금은 넉넉한 standoff(BASE_START_XY, 스폰
# 위치 그대로)에서 시작한다는 뜻만 남긴다.
print(f"[대기 위치] 넉넉한 standoff에서 시작 (BASE_START_XY={np.round(BASE_START_XY, 3)})", flush=True)

# RMPflow 컨트롤러 - 시험용 박스를 부착하기 *전에* 만들어야 한다(사용자 지적 버그 2 수정).
# 원래는 박스 부착 뒤에 만들어져서, 부착 시점의 팔 자세가 joint_3/5=90/90 접은 자세(88/91번의
# "안전 접기") 그대로였다 - 이 접은 자세는 91번(BASE_FACE_ROT_Z=90도)에서는 우연히 그리퍼가
# 아래를 보게 나왔지만, 92번은 BASE_FACE_ROT_Z=0도라 같은 관절값이 다른 절대 방향(앞으로 쭉
# 뻗음)을 향해버렸다 - 섀시 회전과 무관하게 아래를 보장하는 자세가 아니었다는 뜻. 검증 없이
# 가져온 게 문제였다. 고침: RMPflow로 명시적으로 DOWN_QUAT 방향의 "안전 홀딩 자세"를 잡은
# 뒤에야 박스를 그 자리에 부착한다.
controller = RMPFlowController(
    name="trunk_place_holonomic", robot_articulation=m0609_robot,
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


def drive_and_reach(target_x, target_y, ee_target_pos, ee_orientation=DOWN_QUAT,
                     tolerance_xy=0.03, max_speed=0.4, kp_xy=1.8, max_steps=3000,
                     hold_gripper_closed=True, label="",
                     abort_fn=None, hard_stop_on_condition=False, max_speed_fn=None,
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
    Y 이탈, 그리퍼 이탈)을 넣을 수 있게 한다."""
    ee_target_pos = np.array(ee_target_pos, dtype=float)
    start_pos, start_quat = base_robot.get_world_pose()
    tx = target_x if target_x is not None else float(start_pos[0])
    ty = target_y if target_y is not None else float(start_pos[1])
    print(f"\n[주행+추종 시작]{' ' + label if label else ''} 섀시목표=({tx:.3f},{ty:.3f}) "
          f"팔목표={np.round(ee_target_pos, 3)}", flush=True)

    STALL_WINDOW, STALL_MIN_PROGRESS = 150, 0.008
    last_check_pos = np.array([float(start_pos[0]), float(start_pos[1])])
    stalled = False
    aborted = False
    step = 0
    for step in range(1, max_steps + 1):
        if debug_interval and debug_fn is not None and step % debug_interval == 0:
            debug_fn(step)
        if abort_fn is not None and abort_fn():
            aborted = True
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

        # 섀시 주행과 완전히 같은 프레임에서 팔도 매 스텝 목표를 추종한다.
        sync_rmp_base()
        actions = controller.forward(
            target_end_effector_position=ee_target_pos,
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
        # 이미 자세 붕괴(충돌 의심)가 감지된 상황 - 관성으로 더 밀고 들어가지 않도록 부드러운
        # 감속 대신 그 자리에서 즉시 속도를 0으로 만든다. 팔은 계속 ee_target_pos를 추종시켜서
        # (충돌 지점에서 그냥 buzz하지 않고) RMPflow가 알아서 안전한 쪽으로 풀게 둔다.
        _smooth_state["vx"] = 0.0
        _smooth_state["vy"] = 0.0
        _smooth_state["wz"] = 0.0
        zero_action = holo_forward(0.0, 0.0, 0.0)
        for _ in range(8):
            base_robot.apply_action(zero_action)
            sync_rmp_base()
            actions = controller.forward(
                target_end_effector_position=ee_target_pos, target_end_effector_orientation=ee_orientation,
            )
            m0609_robot.apply_action(actions)
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
          f"자세붕괴={aborted} 정체={stalled}", flush=True)
    return final_pos, ee_pos, ee_err, not stalled, aborted


viewport = vp_util.get_active_viewport()


def snapshot(eye, target, fname):
    set_camera_view(eye=eye, target=target)
    step_hold(15)
    out = str(OUT_DIR / fname)
    vp_util.capture_viewport_to_file(viewport, out)
    step_hold(30)
    print(f"[SCREENSHOT] {out}", flush=True)


print(f"\n[STAGE] {STAGE}단계까지 진행합니다 "
      "(1=홀딩자세만 2=+홀로노믹 근접이동 3=+정밀접근/PLACE)", flush=True)

# ================= STAGE 1: 안전 홀딩 자세 확립 (그리퍼가 아래를 보게, 목표 높이 근처) =================
# 사용자 설계 재검토(2차) - 위 place_release_z/HOLDING_Z 사전계산 설명 참고. 천장 근처가
# 아니라 처음부터 목표 높이 근처에서 박스를 들고 있어야 이후 "옆에서 진입"이 자연스럽다.
_init_ee_pos, _ = m0609_robot.end_effector.get_world_pose()
holding_pos = (float(_init_ee_pos[0]), float(_init_ee_pos[1]), HOLDING_Z)
move_link6(holding_pos, steps=400, hold_gripper_closed=False, orientation=DOWN_QUAT,
           label="안전 홀딩 자세(그리퍼 하향, 목표 높이 근처) 확립")

# ================= 시험용 박스를 그리퍼에 미리 부착 (91번이 이미 집어온 상태를 흉내) =================
# 사용자 지적 버그 1 수정 - 원래는 박스를 스폰한 뒤 step_hold(10)으로 그냥 기다리다가
# gripper.close()를 불렀다. 그 사이 중력으로 박스가 떨어지는데, 91번에서 만든 원통형 판정은
# 수직 허용치(GRASP_VERTICAL_TOLERANCE=0.02m)가 좁아서 10스텝의 자유낙하만으로도 조건을
# 놓쳐버렸다(예전 구형 판정은 마진이 +0.05m라 우연히 버텼음). 고침: 스폰 직후 곧바로
# 부착한다(중력이 작용할 시간 자체를 안 준다) - 부착 후에는 FixedJoint가 물리적으로 붙잡고
# 있으므로 그 다음 step_hold(10)은 안전하다.
box_material = PhysicsMaterial(
    prim_path="/World/Physics_Materials/box_material", static_friction=1.2, dynamic_friction=1.0, restitution=0.0,
)
gripper_body_mat = UsdGeom.Xformable(stage.GetPrimAtPath(gripper_body_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
tip_world0 = np.array(gripper_body_mat.Transform(Gf.Vec3d(*TIP_LOCAL_OFFSET)))
test_box = DynamicCuboid(
    prim_path="/World/TestCarryBox", name="test_carry_box",
    position=np.array([tip_world0[0], tip_world0[1], tip_world0[2] - TEST_BOX_SIZE[2] / 2.0]),
    scale=np.array(TEST_BOX_SIZE), color=np.array([1.0, 0.15, 0.0]), mass=0.3, physics_material=box_material,
)
_test_box_half_height = TEST_BOX_SIZE[2] / 2.0
_test_box_horizontal_tolerance = max(TEST_BOX_SIZE[0], TEST_BOX_SIZE[1]) / 2.0 + GRASP_HORIZONTAL_MARGIN
gripper.set_target("/World/TestCarryBox", _test_box_half_height, _test_box_horizontal_tolerance, GRASP_VERTICAL_TOLERANCE)
gripper.close()
step_hold(10)
print(f"[시험용 박스 부착] grasped={gripper.is_closed()}", flush=True)

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
PROBE_ARM_ENVELOPE = os.environ.get("PROBE_ARM_ENVELOPE") == "1"
if PROBE_ARM_ENVELOPE:
    def _mesh_world_z_range(root_prim_path):
        root_prim = stage.GetPrimAtPath(root_prim_path)
        z_min, z_max = None, None
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
                z = float(wp[2])
                z_min = z if z_min is None else min(z_min, z)
                z_max = z if z_max is None else max(z_max, z)
        return z_min, z_max

    def _probe_arm_envelope(label):
        link6_zmin, link6_zmax = _mesh_world_z_range(ee_path)
        gripper_zmin, gripper_zmax = _mesh_world_z_range(gripper_body_path)
        box_center_z = float(test_box.get_world_pose()[0][2])
        box_bottom_z = box_center_z - TEST_BOX_SIZE[2] / 2.0
        box_top_z = box_center_z + TEST_BOX_SIZE[2] / 2.0
        _candidates = [v for v in [link6_zmax, gripper_zmax] if v is not None]
        print(f"\n[팔 수직 포락선 실측: {label}] link_6 z=[{link6_zmin}, {link6_zmax}] "
              f"gripper_body z=[{gripper_zmin}, {gripper_zmax}] "
              f"박스 top={box_top_z:.4f} bottom={box_bottom_z:.4f}", flush=True)
        if not _candidates:
            print("[실패] link_6/gripper_body 둘 다 메시를 못 찾았습니다 - 프림 경로를 재확인하세요.", flush=True)
            return None
        arm_top_z = max(_candidates)
        print(f"[결론: {label}] 박스 바닥({box_bottom_z:.4f}) ~ 팔 최상단({arm_top_z:.4f}) "
              f"전체 수직 길이 = {arm_top_z - box_bottom_z:.4f}m "
              f"(박스만의 두께는 {TEST_BOX_SIZE[2]:.4f}m - 차이 {arm_top_z - box_bottom_z - TEST_BOX_SIZE[2]:.4f}m는 "
              "93번 진단이 빠뜨렸던, link_6/그리퍼가 박스 위로 튀어나온 길이)", flush=True)
        if CEILING_WORLD_Z:
            print(f"[천장 대비] CEILING_WORLD_Z={CEILING_WORLD_Z:.4f} - 팔 최상단({arm_top_z:.4f}) "
                  f"= 여유 {CEILING_WORLD_Z - arm_top_z:.4f}m", flush=True)
        return box_bottom_z, arm_top_z

    _probe_arm_envelope("STAGE1(HOLDING_Z)")

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

    if PROBE_ARM_ENVELOPE:
        _probe_arm_envelope("STAGE1.1(ENTRY_HOLDING_Z)")

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
    ENTRANCE_CLEAR_MARGIN = 0.005  # 입구를 "박스 뒤쪽 끝"이 넘은 뒤 아주 약간만 더 여유를 둔다.
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

    def _get_box_x_edges():
        """BBoxCache 대신 박스의 실시간 world pose(중심+회전)와 이미 알고 있는 실제 치수
        (TEST_BOX_SIZE)로 world X축 투영 반길이를 직접 계산한다 - get_world_aabb()가 치수를
        잘못 돌려주던 문제(제곱값처럼 나옴, 원인 미상 - scale 이중 적용 추정)를 우회한다."""
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

    def _box_cleared_entrance():
        rear_x, _, box_center = _get_box_x_edges()
        x_cleared = rear_x >= TRUNK_ENTRANCE_X + ENTRANCE_CLEAR_MARGIN
        y_centered = abs(float(box_center[1]) - ANCHOR_Y) < STAGE2_Y_TOLERANCE
        attached = m0609_robot.gripper.is_closed()
        return x_cleared and y_centered and attached

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
              f"임계값까지남은거리={threshold_x - rear_x:+.3f} 붙어있음={m0609_robot.gripper.is_closed()}",
              flush=True)

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

    chassis_pos0, _ = base_robot.get_world_pose()
    snapshot(eye=[chassis_pos0[0] - 1.5, chassis_pos0[1] - 2.2, chassis_pos0[2] + 1.4],
             target=[float(chassis_pos0[0]), ANCHOR_Y, 0.5],
             fname="_trunkplace_00b_close_approach.png")

    if STAGE < 3:
        print(f"\n[STAGE 2 완료] 확인용 스크린샷 저장 완료(성공={condition_met and not aborted}) - "
              "STAGE=3으로 다시 실행하면 정밀 접근/PLACE까지 진행합니다.\n", flush=True)

if STAGE >= 3:
    # ================= STAGE 3: 정밀 접근(홀로노믹+매니퓰레이터 동시 조정) + PLACE =================
    # 사용자 설계 재검토(3차) - STAGE 2에서 이미 박스가 입구를 넘겼으므로, 여기서부터는 남은
    # X/Y 차이를 홀로노믹+팔이 "같은 스텝에서 동시에" 조금씩 좁혀나간다(drive_and_reach) -
    # "Z축 정렬 후 하강"이 아니라 "X축 기준으로 옆에서 안쪽으로 밀어넣는" 동작. 진입 높이는
    # STAGE 1.1의 ENTRY_HOLDING_Z를 그대로 유지하다가, XY가 다 맞은 뒤에야 release 높이로
    # 내린다(입구 턱을 넘긴 높이를 여기서 미리 낮추면 STAGE 1.1의 의미가 없어진다).
    #
    # 사용자 지적(4차, STAGE 2 통과 후 재검토) - 예전 버전은 drive_and_reach()의 섀시 목표를
    # "지금 섀시가 있는 바로 그 자리"로 넣었다 - drive_and_reach는 섀시 오차가 tolerance_xy
    # 이내면 바로 끝나므로, 사실상 섀시는 한 발짝도 안 움직이고 팔만 뻗는 결과가 됐다(주석의
    # "홀로노믹+팔 동시 조정"이 실제로는 발생하지 않음). 고침: 섀시 목표를 place_world_xy 쪽으로
    # 실제로 전진시키되, 목표 지점 바로 위까지 밀어붙이지 않고 STAGE3_ARM_REACH_MARGIN만큼
    # 앞에서 멈춘다 - STAGE 1/2 디버그 로그에서 실측된 "팔이 접은 자세로 섀시보다 자연스럽게
    # 앞서는 정도"(~0.36m)와 비슷한 여유를 남겨서, 팔이 과도하게 뻗지 않고도 도달하게 한다.
    # 또한 STAGE 2에서 만든 안전장치(자세 붕괴 감지/즉시 정지/근접 시 저속화)를 여기도 그대로
    # 적용한다 - STAGE 2는 팔이 얼어붙어 있어 "기준 오프셋 대비 편차"로 충돌을 감지했지만,
    # 여기는 팔이 능동적으로 추종 중이므로 대신 박스 Y 이탈/흡착 해제만으로 감지한다.

    # 사용자 지적(STAGE 2->3 전환에서 충돌) - STAGE 2까지는 리프트가 LIFT_MAX(차체 밑을
    # 지나는 안전마진)로 고정돼 있었다. 박스가 입구를 이미 넘은 지금은 더 이상 차체 밑이
    # 아니라 트렁크 입구/안쪽이라 아래쪽에 여유가 있으므로, STAGE 3(홀로노믹+팔 동시 접근)에
    # 들어가기 전에 리프트를 살짝 낮춰본다. 처음엔 0.15m를 내리면서 ee 목표 높이(ENTRY_HOLDING_Z)를
    # 그대로 유지했더니 너무 많이 내려갔고, 그리고 사용자가 "그리퍼(ee)는 고정한 채 리프트만
    # 내려서 팔이 그만큼 더 위로 뻗어 보정하는" 방식이 아니라 "매니퓰레이터(ee)도 리프트와
    # 같이 내려갔으면 좋겠다"고 지적 - 리프트/ee가 같은 양만큼 함께 내려가도록 target_z도
    # STAGE3_PRE_LIFT_DROP만큼 낮춘다(팔이 보정용으로 더 뻗지 않고, 자세 자체는 유지한 채
    # 통째로 하강).
    STAGE3_PRE_LIFT_DROP = 0.05
    STAGE3_PRE_LIFT_H = max(LIFT_MIN, LIFT_MAX - STAGE3_PRE_LIFT_DROP)
    # STAGE 3 전체(정밀 접근 + 마무리 정렬)에서 쓸 진입 높이 - 원래 ENTRY_HOLDING_Z가 아니라
    # 여기서 낮춘 높이를 그대로 써야 한다. 안 그러면 바로 다음 drive_and_reach()의 ee_target_pos가
    # 다시 원래 ENTRY_HOLDING_Z를 가리켜서, 방금 내린 팔이 접근 도중 도로 올라가버려
    # 이 사전 하강이 무의미해진다.
    STAGE3_ENTRY_Z = ENTRY_HOLDING_Z - STAGE3_PRE_LIFT_DROP
    # 사용자 지적(재조정) - TRUNK_ENTRANCE_X/STAGE3_PRE_LIFT_DROP만으로는 한계가 있었다 -
    # 순수 수직 하강이라 그리퍼가 입구 아래쪽 턱을 정면으로 긁는다. 하강하는 동안 그리퍼가
    # 대각선으로 살짝 더 안쪽(+X, 트렁크 쪽)까지 들어가게 만들어서 "아래로 내려가며 동시에
    # 조금 전진"하는 경로로 턱을 피해가게 한다 - target_xy를 stage2_end_ee_pos 그대로가 아니라
    # STAGE3_PRE_X_ADVANCE만큼 앞으로 옮긴 지점으로 준다(z는 기존처럼 alpha로 보간, xy는
    # RMPflow가 매 스텝 그 앞쪽 목표를 향해 수렴하므로 자연스럽게 대각선 경로가 나온다).
    STAGE3_PRE_X_ADVANCE = 0.02
    stage3_pre_target_xy = (
        float(stage2_end_ee_pos[0]) + STAGE3_PRE_X_ADVANCE,
        float(stage2_end_ee_pos[1]),
    )
    stage3_pre_ee, stage3_pre_err = descend_and_raise_lift(
        stage3_pre_target_xy,
        STAGE3_ENTRY_Z,
        STAGE3_PRE_LIFT_H, steps=150, hold_gripper_closed=True,
        label="STAGE3 사전: 입구 통과 후 리프트+팔 함께 대각선(하강+소폭 전진)",
    )
    if stage3_pre_err > 0.03:
        raise SystemExit(f"[중단] STAGE3 사전 리프트 하강 실패: err={stage3_pre_err:.3f}m")

    STAGE3_ARM_REACH_MARGIN = 0.35
    STAGE3_Y_TOLERANCE = 0.04
    stage3_target_x = min(place_world_xy[0] - STAGE3_ARM_REACH_MARGIN, TRUNK_X_MAX)
    print(f"[PLACE 목표] xy={np.round(place_world_xy, 3)} release_z={place_release_z:.3f} "
          f"entry_holding_z={ENTRY_HOLDING_Z:.3f} 섀시목표_x={stage3_target_x:.3f}", flush=True)

    def _stage3_pose_broken():
        # STAGE 2와 달리 팔이 능동적으로 목표를 추종 중이라 "기준 오프셋 대비 편차"는 못
        # 쓴다(추종 중엔 원래도 오차가 있음) - 대신 박스가 중앙선에서 벗어났는지, 흡착이
        # 풀렸는지로 충돌을 감지한다(STAGE 2에서 실제로 관측된 붕괴 신호와 동일한 종류).
        _, _, box_center = _get_box_x_edges()
        y_broken = abs(float(box_center[1]) - ANCHOR_Y) > STAGE3_Y_TOLERANCE
        detached = not m0609_robot.gripper.is_closed()
        return y_broken or detached

    def _stage3_max_speed():
        chassis_pos, _ = base_robot.get_world_pose()
        remaining = abs(stage3_target_x - float(chassis_pos[0]))
        if remaining < 0.08:
            return 0.025
        if remaining < 0.15:
            return 0.05
        return 0.10

    def _stage3_debug(step):
        chassis_pos, _ = base_robot.get_world_pose()
        ee_pos, _ = m0609_robot.end_effector.get_world_pose()
        rear_x, front_x, box_center = _get_box_x_edges()
        print(f"  [DEBUG step={step}] 섀시목표_x={stage3_target_x:.3f} | "
              f"섀시중심=({float(chassis_pos[0]):.3f},{float(chassis_pos[1]):.3f}) | "
              f"팔ee=({ee_pos[0]:.3f},{ee_pos[1]:.3f},{ee_pos[2]:.3f}) | "
              f"박스 뒤={rear_x:.3f} 앞={front_x:.3f} 중심={np.round(box_center, 3)} "
              f"붙어있음={m0609_robot.gripper.is_closed()}", flush=True)

    _, stage3_ee_pos, stage3_ee_err, _, stage3_aborted = drive_and_reach(
        target_x=stage3_target_x, target_y=ANCHOR_Y,
        ee_target_pos=(place_world_xy[0], place_world_xy[1], STAGE3_ENTRY_Z),
        ee_orientation=DOWN_QUAT, hold_gripper_closed=True,
        max_speed=0.10, max_speed_fn=_stage3_max_speed,
        abort_fn=_stage3_pose_broken, hard_stop_on_condition=True,
        label="STAGE3: 정밀 접근(홀로노믹+팔 동시 조정, 저속)",
        debug_interval=5, debug_fn=_stage3_debug,
    )
    # 사용자 지적(중대 버그) - 원래는 여기서 경고만 찍고 그대로 PLACE/후퇴까지 계속 진행했다.
    # 그러면 "충돌로 틀어진 임의의 상태"에서 박스를 내려놓고 후퇴를 시작하게 되어, 그 뒤
    # 모든 단계가 성공한 전진 경로가 아니라 잘못된 상태를 기준으로 동작하게 된다 - 실제로
    # 이 때문에 STAGE 4 후퇴가 연쇄적으로 잘못됐다. 여기서 확실히 멈춘다.
    if stage3_aborted or stage3_ee_err > 0.03:
        raise SystemExit(
            f"[중단] STAGE 3 정밀 접근 실패(자세붕괴={stage3_aborted}, ee_err={stage3_ee_err:.3f}m) - "
            "PLACE/후퇴를 진행하지 않습니다. STAGE3_ARM_REACH_MARGIN을 늘리거나 접근 경로를 "
            "재검토하세요."
        )

    # 위 결합 이동이 정체/시간 부족으로 덜 수렴했을 경우를 대비한 마무리 정렬(여전히 진입 높이).
    side_entry_pos = (place_world_xy[0], place_world_xy[1], STAGE3_ENTRY_Z)
    move_link6(side_entry_pos, steps=200, label="PLACE 측면 진입 마무리 정렬(진입 높이 유지)")

    snapshot(eye=[chassis_pos0[0] - 1.2, chassis_pos0[1] - 2.0, chassis_pos0[2] + 1.3],
             target=[place_world_xy[0], place_world_xy[1], TRUNK_FLOOR_Z], fname="_trunkplace_01_approaching.png")

    # 사용자 설계(5차) - ENTRY_HOLDING_Z -> place_release_z 낙차를 팔 혼자 감당하게 하는 대신,
    # 리프트를 PLACE_LIFT_MAX까지 같이 올려서(마운트 자체가 목표 높이로 다가감) 팔이 커버할
    # 나머지 거리를 최소화한다(91.py PICK Phase A와 같은 원리) - 팔꿈치/팔뚝이 트렁크 입구
    # 프레임을 스치는 걸 막기 위함. 섀시는 이미 입구를 지나 트렁크 안쪽에 있으므로 under-car
    # 안전캡(LIFT_MAX) 대신 천장 안전한계까지 리프트를 더 써도 된다.
    descend_and_raise_lift(
        (place_world_xy[0], place_world_xy[1]), place_release_z, PLACE_LIFT_MAX, steps=250,
        label="PLACE 하강(진입높이 -> release 높이, 리프트 동시 상승)",
    )

    snapshot(eye=[chassis_pos0[0] - 1.0, chassis_pos0[1] - 1.6, chassis_pos0[2] + 1.0],
             target=[place_world_xy[0], place_world_xy[1], TRUNK_FLOOR_Z], fname="_trunkplace_02_descended.png")

    gripper.open()
    box_rigid_prim = SingleRigidPrim("/World/TestCarryBox")
    box_rigid_prim.initialize(physics_sim_view=world.physics_sim_view)
    box_rigid_prim.set_linear_velocity(np.array([0.0, 0.0, -0.3]))
    step_hold(60)

    final_box_pos = get_world_pos(stage.GetPrimAtPath("/World/TestCarryBox"))
    err_xy = float(np.linalg.norm(final_box_pos[:2] - np.array(place_world_xy)))
    print(f"\n[완료] 최종 박스 world 위치={np.round(final_box_pos, 3)} 목표 xy={np.round(place_world_xy, 3)} "
          f"xy 오차={err_xy:.4f}m", flush=True)

    snapshot(eye=[chassis_pos0[0] - 1.0, chassis_pos0[1] - 1.6, chassis_pos0[2] + 1.0],
             target=[place_world_xy[0], place_world_xy[1], TRUNK_FLOOR_Z], fname="_trunkplace_03_placed.png")

    result = {
        "place_world_xy": list(place_world_xy),
        "target_release_z": place_release_z,
        "final_box_pos": final_box_pos.tolist(),
        "xy_error_m": err_xy,
    }
    (OUT_DIR / "_trunkplace_result.json").write_text(json.dumps(result, indent=2))
    print(f"[저장 완료] {OUT_DIR / '_trunkplace_result.json'}", flush=True)

    if STAGE < 4:
        print("\n[STAGE 3 완료] PLACE까지 완료 - STAGE=4로 다시 실행하면 STAGE 1 상태로 "
              "후퇴하는 것까지 진행합니다.\n", flush=True)

if STAGE >= 4:
    # ================= STAGE 4: 후퇴 (STAGE 3 -> ... -> STAGE 1 상태로 역순 복귀) =================
    # 사용자 설계(재검토) - 지금까지 밟은 전진 시퀀스를 그대로 거꾸로 밟는다:
    #   4-1) descend_and_raise_lift의 역방향 - PLACE_LIFT_MAX/release_z에서
    #        LIFT_MAX/ENTRY_HOLDING_Z로 복귀(리프트 하강 + 팔 목표 상승, 동시 진행). XY는
    #        아직 place_world_xy 그대로 - 이 스텝은 순수 Z 변화만 담당(STAGE 3c와 대칭).
    #   4-2) STAGE 3a(정밀 접근)의 역 - 사용자 지적: 팔이 place_world_xy까지 "쭉 뻗은 상태"
    #        그대로인데 여기서 곧장 얼려서 끌고 가면 뻗은 채 계속 앞을 향하게 된다. STAGE 2
    #        방식(얼림)이 아니라 STAGE 3과 동일한 능동 추종(drive_and_reach)으로 팔 XY도
    #        원래(STAGE 1) 근처로 되돌리면서 섀시도 같이 후퇴시킨다.
    #   4-3) STAGE 2(홀로노믹 근접 접근)의 역 - 팔이 4-2에서 이미 컴팩트해졌으니, 이제는
    #        그 자세로 고정한 채(STAGE 2와 동일한 "얼려서 드라이브" + 안전장치) 섀시만
    #        BASE_START_XY까지 마저 후퇴.
    #   4-4) STAGE 1.1의 역 - ENTRY_HOLDING_Z에서 STAGE 1의 HOLDING_Z로 복귀.
    # 박스는 STAGE 3에서 이미 내려놓고 그리퍼를 열었으므로, 이 구간 전체에서
    # hold_gripper_closed=False로 둔다(더 이상 잡을 대상이 없음).
    print("\n[STAGE 4] 후퇴 시작 - STAGE 3 -> STAGE 1 상태로 역순 복귀", flush=True)

    # ---- 4-1) PLACE 하강+리프트상승의 역: 진입높이 복귀 + 리프트 하강 (XY는 place_world_xy 유지) ----
    stage4_1_ee, stage4_1_err = descend_and_raise_lift(
        (place_world_xy[0], place_world_xy[1]), ENTRY_HOLDING_Z, LIFT_MAX, steps=250,
        hold_gripper_closed=False, label="STAGE4-1: 진입높이 복귀 + 리프트 하강(역방향)",
    )
    if stage4_1_err > 0.03:
        raise SystemExit(f"[중단] STAGE4-1 복귀 실패: err={stage4_1_err:.3f}m")

    # ---- 4-2A) 정밀 접근의 역: 섀시는 그대로 두고 팔만 먼저 STAGE 2 종료 지점으로 정확히
    # 복귀시킨다(수렴 확인 전에 섀시를 같이 움직이면, 섀시가 우연히 이미 목표 근처라
    # drive_and_reach의 종료 조건(섀시 tolerance만 체크)이 팔 수렴 전에 만족돼버려
    # "얼린 자세"가 실제로는 수렴 안 된 자세일 수 있다 - 사용자가 실측 로그로 지적한 버그).
    #
    # 사용자 지적(재검토) - 처음엔 여기서 _init_ee_pos(STAGE 1 맨 처음, 섀시가 BASE_START_XY에
    # 있을 때 캡처한 값)를 팔 목표로 쓰면서 섀시 목표는 TRUNK_ENTRANCE_X로 줬다 - 이 둘은
    # 서로 다른 시점의 좌표라 앞뒤가 안 맞았다(진짜 역순이 아니었음). STAGE 2 종료 시점의
    # 섀시/팔 위치를 실측해서 저장해둔 stage2_end_chassis_pos/stage2_end_ee_pos를 대신 쓴다 -
    # 팔이 "얼어붙은 채 섀시에 실려" STAGE 2 내내 이동했으므로, 그 종료 시점의 실제 팔
    # world 위치가 곧 "이 시점에 팔이 있어야 할 정확한 자리"다.
    stage4_2_ee, stage4_2_err = move_link6(
        stage2_end_ee_pos, steps=300, hold_gripper_closed=False, orientation=DOWN_QUAT,
        label="STAGE4-2A: 팔을 STAGE2 종료 자세로 복귀(섀시 정지)",
    )
    if stage4_2_err > 0.02:
        raise SystemExit(f"[중단] STAGE4-2A 팔 복귀 실패: {stage4_2_err:.3f}m")

    # ---- 4-2B) 위에서 수렴이 확인된 관절값을 고정 ----
    stage4_hold_q = np.asarray(m0609_robot.get_joint_positions(), dtype=float).copy()

    # ---- 4-3) 근접 접근의 역: 팔은 4-2에서 이미 컴팩트해졌으니 그 자세로 고정한 채 섀시만
    # STAGE2 종료 지점(stage2_end_chassis_pos)까지 후퇴 - STAGE 2와 동일한 "얼려서 드라이브" +
    # 안전장치 재사용 ----
    _stage4_chassis_start, _ = base_robot.get_world_pose()
    _stage4_tip_start = _measure_tip_pos()
    stage4_tip_rel_ref = _stage4_tip_start - np.asarray(_stage4_chassis_start, dtype=float)

    def _hold_stage4_arm():
        m0609_robot.apply_action(ArticulationAction(joint_positions=stage4_hold_q))

    def _stage4_pose_broken():
        chassis_pos, _ = base_robot.get_world_pose()
        tip_pos = _measure_tip_pos()
        tip_rel = tip_pos - np.asarray(chassis_pos, dtype=float)
        relative_error = float(np.linalg.norm(tip_rel - stage4_tip_rel_ref))
        return relative_error > STAGE2_POSE_DRIFT_TOLERANCE

    def _stage4_max_speed():
        chassis_pos, _ = base_robot.get_world_pose()
        remaining = abs(float(chassis_pos[0]) - BASE_START_XY[0])
        if remaining < 0.15:
            return 0.05
        return 0.10

    final_pos, final_yaw, _, stage4b_aborted = drive_until(
        lambda: False, target_x=BASE_START_XY[0], target_y=BASE_START_XY[1],
        max_speed=0.10, max_speed_fn=_stage4_max_speed,
        per_step_fn=_hold_stage4_arm, abort_fn=_stage4_pose_broken,
        hard_stop_on_condition=True,
        label="STAGE4-3: 트렁크 밖으로 후퇴(팔 자세 고정, 저속)",
    )
    stage4_aborted = stage4b_aborted
    if stage4_aborted:
        print("[실패] STAGE 4 후퇴 중 자세 붕괴(충돌 의심)가 감지돼 중단했습니다.", flush=True)

    # ---- 4-4) STAGE 1.1의 역: 진입높이 -> STAGE 1 홀딩 높이로 복귀 ----
    _final_ee_pos, _ = m0609_robot.end_effector.get_world_pose()
    move_link6((float(_final_ee_pos[0]), float(_final_ee_pos[1]), HOLDING_Z), steps=200,
               hold_gripper_closed=False, orientation=DOWN_QUAT,
               label="STAGE4-4: STAGE 1 홀딩 높이로 복귀")

    chassis_pos0, _ = base_robot.get_world_pose()
    snapshot(eye=[chassis_pos0[0] - 2.2, chassis_pos0[1] - 3.2, chassis_pos0[2] + 1.6],
             target=[(chassis_pos0[0] + CAR_POS[0]) / 2, 0.0, 1.0], fname="_trunkplace_04_retreated.png")
    print(f"\n[STAGE 4 완료] 후퇴 완료(성공={not stage4_aborted}) - STAGE 1 상태(홀딩 자세, "
          f"BASE_START_XY 근처)로 복귀됨.\n", flush=True)

if HEADLESS:
    simulation_app.close()
else:
    print("[안내] 창을 직접 둘러보세요 - 닫으면 스크립트가 종료됩니다.\n", flush=True)
    while simulation_app.is_running():
        step_hold(1)
    simulation_app.close()
