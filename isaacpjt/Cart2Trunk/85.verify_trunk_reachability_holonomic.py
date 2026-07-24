"""
85.verify_trunk_reachability_holonomic.py

이 트랙의 마지막 확인 단계: 저상 대칭 홀로노믹 베이스(80~83번) + RMPflow/새 흡착 그리퍼
결합(84번)을 합쳐서, 실제로 박스를 들고 차량 밑으로 들어가 트렁크 내부 목표 그리드에
얼마나 도달할 수 있는지 검증한다. 84번과 달리 여기서는 "차 밑을 지나 트렁크에 접근"하는
시나리오라 82/83번과 동일하게 리프트 이동거리가 0.35m로 제한된다(84번은 카트라 차량 제약이
없어서 0.70m까지 늘렸었다 - 여기서는 그 여유가 없다).

84번에서 얻은 교훈 그대로 적용
----
1. **섀시 방향**: 84번은 카트 옆에 설 때 섀시의 "짧은 축"이 카트를 향하게 90도로 세웠지만,
   여기서는 차량 밑을 "긴 축 방향"(정면)으로 뚫고 들어가야 하므로 82/83번과 동일하게
   BASE_FACE_ROT_Z=0(정면 전진)을 그대로 쓴다 - 이건 카트처럼 "옆에 서는" 상황이 아니라
   "터널을 통과하는" 상황이라 84번의 회전 트릭이 적용되지 않는다.
2. **정차 위치**: 81번이 추천한 j1_x(=TRUNK_X_MIN=3.11, chassis 중심이 트렁크 입구 경계에
   오는 지점)를 그대로 쓴다 - 83번의 "차량 접근" 테스트에서 이미 검증된 값.
3. **리프트 높이**: 트렁크 바닥(TRUNK_FLOOR_Z=0.44)이 LIFT_MAX(0.35m 이동, world 약 0.52m)와
   가까워서 위/아래 어느 쪽으로도 큰 reach가 필요 없다 - LIFT_MAX 그대로 사용.
4. **WAYPOINT_STEPS**: 84번 5차 시도에서 스텝 부족(150)이 실패 원인 중 하나였다(첫
   웨이포인트가 초기 시드에서 멀어서 못 수렴) - 처음부터 400으로 시작한다.
5. **박스는 84번과 동일하게 낮은 스탠드에서 미리 집는다**(카트가 아니라 단순 스탠드 -
   84번 1차에서 이미 이 패턴으로 PICK 자체는 검증됐으므로 반복 안 함). 운반 전 팔을 접는
   자세는 28.py의 CARRY_JOINT_POSITIONS(joint_1=180도, Nova Carter의 BASE_FACE_ROT_Z=180
   기준으로 검증된 값)를 1차로 그대로 가져다 썼는데, 이 스크립트는 BASE_FACE_ROT_Z=0이라
   팔이 엉뚱한 방향으로 튀어서 주행이 곧바로 정체됐다(실측 확인). 조인트 각도를 다시 추정하는
   대신 이미 검증된 move_link6(Cartesian)로 "섀시 중심 바로 위"를 목표로 후퇴시키는 방식으로
   교체했다 - 섀시 방향에 무관하게 항상 안전하다.

시퀀스: 시작 위치에서 낮은 스탠드 위 박스 PICK -> 섀시 중심 위로 후퇴 -> 차량 접근 위치(x=81번
추천 j1_x)까지 주행(83번 drive_to 재사용, 박스는 흡착 조인트로 붙어서 자동으로 같이 이동) ->
리프트를 LIFT_MAX로 -> 트렁크 내부 3x3 목표 그리드(x 3개 x y 3개) 각각에 move_link6 시도,
도달 성공/실패(err<tolerance)와 오차를 CSV로 기록 -> 그리드 중 가장 깊이 들어간 성공 지점에
실제로 내려놓기(그리퍼 open)까지 시도 -> 스크린샷 + 결과 CSV/JSON 저장.
"""

from isaacsim import SimulationApp

import os
simulation_app = SimulationApp({"headless": os.environ.get("HEADLESS", "0") == "1"})

import csv
import json
import sys
import time
from pathlib import Path

import numpy as np
import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, UsdLux, UsdShade, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleArticulation
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

# ---------------- 80~83번과 동일 (카트/차량/베이스 구성 재사용) ----------------
CART_USD = str(_THIS_DIR / "assets/Metal_Shopping_Cart.usdz")
CAR_USD = str(_THIS_DIR / "assets/Lexus_IS300_Trunk_Open_No_More_Hell_Room.usdz")
CART_POS = (0.0, 0.0, 0.0)
CART_EXTRA_SCALE = 0.55
CAR_POS = (5.0, 0.0, 0.0)
CAR_EXTRA_SCALE = 0.50
CAR_ROT_Z = 0.0
SDF_RESOLUTION = 256
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 0.0, 50.0, 20.0

BASE_PATH = "/World/HoloBase"
CHASSIS_PATH = f"{BASE_PATH}/chassis"
BASE_START_XY = (-0.3, -1.5)
BASE_FACE_ROT_Z = 0.0  # 84번과 달리 정면 전진(차 밑 터널 통과) - 1절 참고

ROLLER_COUNT = 9
ROLLER_MASS = 0.02
HUB_MASS = 1.0
CHASSIS_MASS = 15.0
WZ_SIGN = 1.0

M0609_USD = str(M0609_DIR / "Collected_m0609_vgp20_camera" / "m0609_vgp20_camera.usd")
M0609_URDF_PATH = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH = str(M0609_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(M0609_DIR / "rmpflow/m0609_rmpflow_common.yaml")
M0609_MOUNT_Z_ABOVE_CHASSIS_TOP = 0.02
LIFT_COLUMN_RADIUS = 0.045
LIFT_TRAVEL_M = 0.35  # 82/83번과 동일 - 차 밑 통과 제약이 있어서 84번처럼 늘릴 수 없다

EE_LINK_NAME = "link_6"
GRIPPER_BODY_NAME = "vgp20_suction_plate"

GRIPPER_RANGE_JSON = M0609_DIR / "Collected_m0609_vgp20_camera" / "_gripper_physical_range.json"
if GRIPPER_RANGE_JSON.exists():
    _range = json.loads(GRIPPER_RANGE_JSON.read_text())
    TIP_LOCAL_OFFSET = tuple(_range["tip_local_offset"])
    GRASP_RADIUS = float(_range["grasp_radius_m"])
    print(f"[그리퍼] {GRIPPER_RANGE_JSON}에서 로드: tip_local_offset={TIP_LOCAL_OFFSET} "
          f"grasp_radius={GRASP_RADIUS}", flush=True)
else:
    TIP_LOCAL_OFFSET = (0.0, 0.0, 0.0188)
    GRASP_RADIUS = 0.0515
    print(f"[경고] {GRIPPER_RANGE_JSON} 없음 - 플레이스홀더 값 사용", flush=True)

CUBE_SIZE = 0.05
CUBE_MASS = 0.05
CUBE_STATIC, CUBE_DYNAMIC = 1.2, 1.0
DROP_HEIGHT_ABOVE_STAND = 0.05
STAND_HEIGHT = 0.35
STAND_SIZE = (0.22, 0.20, STAND_HEIGHT)
STAND_DIST_X = 0.75  # 84번 1차와 동일 - 이미 검증된 PICK 거리

# ---------------- 28.py 실측 상수 재사용 (12.trunk_scan_hidden_gripper.py 스캔값) ----------------
TRUNK_X_MIN, TRUNK_X_MAX = 3.11, 3.68
TRUNK_Y_MIN, TRUNK_Y_MAX = -0.56, 0.56
TRUNK_FLOOR_Z = 0.44

WAYPOINT_STEPS = 400  # 84번 교훈 - 처음부터 충분히 크게
SETTLE_STEPS = 60
GRID_TOLERANCE = 0.03  # 이 오차 이내면 "도달 성공"으로 판정 (grasp_radius 0.0515m보다 살짝 타이트)
DOWN_QUAT = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))


def load_recommended_dims():
    csv_path = OUT_DIR / "_evaluate_low_profile_base.csv"
    if csv_path.exists():
        with csv_path.open() as f:
            rows = [r for r in csv.DictReader(f) if r["feasible"] == "True"]
        if rows:
            rows.sort(key=lambda r: (-float(r["trunk_insertion_depth_m"]), float(r["base_length"])))
            best = rows[0]
            return (float(best["base_length"]), float(best["base_width"]), float(best["base_height"]),
                    float(best["j1_x"]))
    print("[경고] _evaluate_low_profile_base.csv 없음 - 플레이스홀더 치수 사용", flush=True)
    return 0.50, 0.50, 0.15, 3.0


BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT, APPROACH_TARGET_X = load_recommended_dims()
WHEEL_RADIUS = max(0.05, BASE_HEIGHT / 2.0)
CHASSIS_BODY_HEIGHT = min(BASE_HEIGHT, 2 * WHEEL_RADIUS) * 0.7
ROLLER_RADIUS = WHEEL_RADIUS * 0.22
ROLLER_LENGTH = (2 * np.pi * (WHEEL_RADIUS - ROLLER_RADIUS)) / ROLLER_COUNT * 1.15
HUB_RADIUS = WHEEL_RADIUS - ROLLER_RADIUS * 0.85
HUB_THICKNESS = WHEEL_RADIUS * 0.55
CHASSIS_LENGTH_EXTENDED = 1.00
WHEEL_MOUNT_HALF_L = BASE_LENGTH / 2.0 - WHEEL_RADIUS * 0.6


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
    """82/83/84.py와 동일 - 허브(구동) + 롤러 ROLLER_COUNT개(45도 자유회전)로 실제 메카넘 휠을 만든다."""
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
    """82/83/84.py와 동일 패턴(LiftColumnVisual 마운트 버그 수정 버전 포함)."""
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

    stray_prim = stage.GetPrimAtPath(f"{m0609_path}/onrobot_rg2ft/world")
    if stray_prim.IsValid() and stray_prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
        stray_prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)

    n = add_drive_stiffness(stage, m0609_path)
    print(f"[DRIVE] M0609={n}개 조인트 강성 적용 (독립 articulation, 리프트 텔레포트 방식), "
          f"initial_h={initial_h:.3f}", flush=True)
    return m0609_path, base_link_path, lift_translate_op, lift_scale_op


def bbox_of(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    bbox = bbox_cache.ComputeWorldBound(prim)
    rng = bbox.ComputeAlignedRange()
    return np.array(rng.GetMin()), np.array(rng.GetMax())


def mecanum_wheel_speeds(vx, vy, wz, wheel_radius, k):
    """82/83.py와 동일 - 표준 메카넘 역기하학 공식."""
    vy = -vy
    return [
        (vx - vy - k * wz) / wheel_radius,
        (vx + vy + k * wz) / wheel_radius,
        (vx + vy - k * wz) / wheel_radius,
        (vx - vy + k * wz) / wheel_radius,
    ]


# ╔══════════════════════════════════════════════════════════════╗
# ║  DynamicSuctionGripper (28/84.py와 동일)                          ║
# ╚══════════════════════════════════════════════════════════════╝
class DynamicSuctionGripper(SurfaceGripper):
    def __init__(self, end_effector_prim_path, gripper_body_path, target_prim_path,
                 tip_local_offset=(0.0, 0.0, 0.0), grasp_radius=0.03):
        SurfaceGripper.__init__(self, end_effector_prim_path=end_effector_prim_path, surface_gripper_path="")
        self._gripper_body_path = gripper_body_path
        self._target_prim_path = target_prim_path
        self._tip_local_offset = Gf.Vec3d(*tip_local_offset)
        self._grasp_radius = grasp_radius
        self._joint_path = f"{gripper_body_path}/suction_attach_joint"
        self._attached = False

    def close(self) -> None:
        if self._attached:
            return
        stage = omni.usd.get_context().get_stage()
        target_prim = stage.GetPrimAtPath(self._target_prim_path)
        if not target_prim.IsValid():
            return
        gripper_mat = UsdGeom.Xformable(stage.GetPrimAtPath(self._gripper_body_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        target_mat = UsdGeom.Xformable(target_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        target_pos = target_mat.ExtractTranslation()
        tip_world = gripper_mat.Transform(self._tip_local_offset)
        dist = (tip_world - target_pos).GetLength()
        if dist > self._grasp_radius:
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
        print(f"  [흡착] dist={dist:.4f}m <= {self._grasp_radius}m -> {self._joint_path} 생성", flush=True)

    def open(self) -> None:
        if self._attached:
            stage = omni.usd.get_context().get_stage()
            if stage.GetPrimAtPath(self._joint_path).IsValid():
                stage.RemovePrim(self._joint_path)
            print(f"  [해제] {self._joint_path} 삭제", flush=True)
        self._attached = False

    def is_closed(self) -> bool:
        return self._attached

    def is_open(self) -> bool:
        return not self._attached


# ================= 씬 구성 =================
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
target_mpu = UsdGeom.GetStageMetersPerUnit(stage)
target_up = UsdGeom.GetStageUpAxis(stage)

add_asset(stage, "/World/ShoppingCart", CART_USD, CART_POS, CART_EXTRA_SCALE, target_mpu, target_up)
add_asset(stage, "/World/Vehicle", CAR_USD, CAR_POS, CAR_EXTRA_SCALE, target_mpu, target_up, rot_z=CAR_ROT_Z)
for _ in range(20):
    simulation_app.update()
add_sdf_collision(stage, "/World/ShoppingCart")
add_sdf_collision(stage, "/World/Vehicle")

# 84번 1차와 동일한 낮은 스탠드+박스 (시작 위치 바로 앞, +X 방향 - 이미 검증된 PICK 패턴)
stand_xy = (BASE_START_XY[0] + STAND_DIST_X, BASE_START_XY[1])
stand_box = FixedCuboid(
    prim_path="/World/PickStandBox",
    name="pick_stand_box",
    position=np.array([stand_xy[0], stand_xy[1], STAND_HEIGHT / 2.0]),
    scale=np.array(STAND_SIZE),
    color=np.array([0.55, 0.40, 0.25]),
)

trunk_center_xy = ((TRUNK_X_MIN + TRUNK_X_MAX) / 2.0, (TRUNK_Y_MIN + TRUNK_Y_MAX) / 2.0)
area_light = UsdLux.SphereLight.Define(stage, "/World/PickAreaLight")
area_light.CreateRadiusAttr(0.3)
area_light.CreateIntensityAttr(60000)
UsdGeom.Xformable(area_light).AddTranslateOp().Set(Gf.Vec3d(stand_xy[0], stand_xy[1], 2.0))
trunk_light = UsdLux.SphereLight.Define(stage, "/World/TrunkAreaLight")
trunk_light.CreateRadiusAttr(0.2)
trunk_light.CreateIntensityAttr(200000)
UsdGeom.Xformable(trunk_light).AddTranslateOp().Set(Gf.Vec3d(trunk_center_xy[0], trunk_center_xy[1], 1.8))

chassis_path, hub_joint_paths, k_factor = build_holonomic_base(stage, BASE_START_XY, BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)

# 이론식(WHEEL_RADIUS+CHASSIS_BODY_HEIGHT/2) 대신 86번에서 실측(큐브 낙하+GUI 육안 튜닝)한 값 사용
MEASURED_CHASSIS_TOP_OFFSET = 0.0180
LIFT_MIN = MEASURED_CHASSIS_TOP_OFFSET + M0609_MOUNT_Z_ABOVE_CHASSIS_TOP
LIFT_MAX = LIFT_MIN + LIFT_TRAVEL_M
m0609_path, m0609_base_link_path, lift_translate_op, lift_scale_op = mount_m0609(stage, LIFT_MIN)
gripper_body_path = f"{m0609_path}/{GRIPPER_BODY_NAME}"
ee_path = f"{m0609_path}/{EE_LINK_NAME}"

for _ in range(20):
    simulation_app.update()

gripper = DynamicSuctionGripper(
    end_effector_prim_path=ee_path,
    gripper_body_path=gripper_body_path,
    target_prim_path="/World/PickBox",
    tip_local_offset=TIP_LOCAL_OFFSET,
    grasp_radius=GRASP_RADIUS,
)
m0609_robot = SingleManipulator(
    prim_path=m0609_base_link_path,
    end_effector_prim_path=ee_path,
    name="m0609_arm",
    gripper=gripper,
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


def holo_forward(vx, vy, wz):
    speeds = mecanum_wheel_speeds(vx, vy, WZ_SIGN * wz, WHEEL_RADIUS, k_factor)
    return ArticulationAction(joint_velocities=speeds, joint_indices=hub_dof_indices)


# ---- 리프트 제어 (82/83/84.py와 동일한 텔레포트 패턴) ----
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


step_hold(60)
print("\n[안정화 완료] 카트/차량/베이스 정지 상태\n", flush=True)

viewport = vp_util.get_active_viewport()


def snapshot(eye, target, fname):
    set_camera_view(eye=eye, target=target)
    step_hold(15)
    out = str(OUT_DIR / fname)
    vp_util.capture_viewport_to_file(viewport, out)
    step_hold(5)
    print(f"[SCREENSHOT] {out}", flush=True)


# ================= 박스를 스탠드 위에 낙하 =================
box_material = PhysicsMaterial(
    prim_path="/World/Physics_Materials/box_material",
    static_friction=CUBE_STATIC, dynamic_friction=CUBE_DYNAMIC, restitution=0.0,
)
box = DynamicCuboid(
    prim_path="/World/PickBox",
    name="pick_box",
    position=np.array([stand_xy[0], stand_xy[1], STAND_HEIGHT + DROP_HEIGHT_ABOVE_STAND]),
    scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]),
    color=np.array([1.0, 0.15, 0.0]),
    mass=CUBE_MASS,
    physics_material=box_material,
)
step_hold(150)
box_pos, _ = box.get_world_pose()
print(f"[박스 낙하 완료] 최종 위치=({box_pos[0]:.3f},{box_pos[1]:.3f},{box_pos[2]:.3f})", flush=True)
snapshot(eye=[stand_xy[0] - 0.6, stand_xy[1] - 1.0, 1.2], target=[box_pos[0], box_pos[1], box_pos[2]],
         fname="_trunkverify_00_box_on_stand.png")

# ================= RMPflow 컨트롤러 =================
move_lift_to(LIFT_MAX, steps=120)

controller = RMPFlowController(
    name="holo_trunk_controller",
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
    return base_pos, chassis_quat


def move_link6(target_pos, steps=WAYPOINT_STEPS, hold_gripper_closed=False, label=""):
    for i in range(steps):
        sync_rmp_base()
        actions = controller.forward(
            target_end_effector_position=np.array(target_pos),
            target_end_effector_orientation=DOWN_QUAT,
        )
        m0609_robot.apply_action(actions)
        if hold_gripper_closed:
            m0609_robot.gripper.close()
        set_lift_height(lift_state["h"])
        world.step(render=True)
    ee_pos, _ = m0609_robot.end_effector.get_world_pose()
    err = np.linalg.norm(np.array(ee_pos) - np.array(target_pos))
    print(f"[웨이포인트{' ' + label if label else ''}] target={np.round(target_pos,3)} "
          f"ee={np.round(ee_pos,3)} err={err:.4f}m", flush=True)
    return ee_pos, err


# ================= PICK (84번 1차와 동일 패턴) =================
APPROACH_HOVER = 0.18
pick_hover_pos = (box_pos[0], box_pos[1], box_pos[2] + APPROACH_HOVER)
move_link6(pick_hover_pos, label="박스 위 호버")
snapshot(eye=[stand_xy[0] - 0.6, stand_xy[1] - 0.8, box_pos[2] + 0.5],
         target=[box_pos[0], box_pos[1], box_pos[2]], fname="_trunkverify_01_hover.png")

grasp_pos = (box_pos[0], box_pos[1], box_pos[2] + TIP_LOCAL_OFFSET[2])
move_link6(grasp_pos, label="박스 그랩 위치 하강")
m0609_robot.gripper.close()
step_hold(30)
grasped = m0609_robot.gripper.is_closed()
print(f"\n[흡착 시도] gripper.is_closed()={grasped}\n", flush=True)
snapshot(eye=[stand_xy[0] - 0.5, stand_xy[1] - 0.6, box_pos[2] + 0.3],
         target=[box_pos[0], box_pos[1], box_pos[2]], fname="_trunkverify_02_grasped.png")

# ================= 스탠드 치우기 (실측으로 발견한 버그) =================
# 1/2차 시도 모두 주행이 (0.186,-0.919) 부근에서 똑같이 정체됐다 - 팔 자세를 바꿔도 동일한
# 지점에서 멈춰서 팔 문제가 아니라고 판단, 직접 스크린샷/기하 확인한 결과 원인은 스탠드
# (PickStandBox, 섀시 전방 +X 0.75m 지점)였다: 섀시 몸체가 1.0m 길이(중심에서 0.5m가 진행
# 방향으로 뻗어나옴)라 출발 직후 이 스탠드에 부딪힌다. 박스는 이미 집었으니 스탠드는 더 이상
# 필요 없다 - SetActive(False)는 이미 돌고 있는 PhysX 씬에 바로 반영 안 될 수 있어(11번
# 스크립트에서 이미 겪은 문제) 대신 실제로 멀리 텔레포트시켜 확실히 경로를 비운다.
stand_box.set_world_pose(position=np.array([stand_xy[0], stand_xy[1], -5.0]))
print("[스탠드 제거] PickStandBox를 지면 아래로 치움 (주행 경로 확보)", flush=True)

# ================= CARRY 자세로 후퇴 (차 밑 통과 대비) =================
# 1차 시도: 28.py의 CARRY_JOINT_POSITIONS(joint_1=180도)를 그대로 가져다 썼는데, 그건
# Nova Carter가 BASE_FACE_ROT_Z=180(카트 쪽 -X를 보는 상태)에서 검증된 값이었다 - 이
# 스크립트는 BASE_FACE_ROT_Z=0(정면 +X 전진)이라 그대로 재사용하면 팔이 엉뚱한 방향/위치로
# 튀었다(스크린샷으로 확인: 박스가 카메라 프레임 구석에 멀리 떨어져 보임, 그 상태로 주행하니
# 몇 십 cm만에 정체 감지로 멈춤). 조인트 각도를 다시 추정하는 대신, 이미 hover/descend에서
# 검증된 move_link6(Cartesian) 방식으로 "섀시 중심 바로 위"를 목표로 잡아 후퇴시킨다 -
# 어느 섀시 방향에서도 항상 안전하게 통하는 방법.
chassis_pos_now, _ = base_robot.get_world_pose()
retract_target = (float(chassis_pos_now[0]), float(chassis_pos_now[1]),
                   float(chassis_pos_now[2]) + lift_state["h"] + 0.05)
move_link6(retract_target, hold_gripper_closed=True, label="운반 자세로 후퇴(섀시 중심 위)")
move_lift_to(LIFT_MIN, steps=90)
snapshot(eye=[BASE_START_XY[0], BASE_START_XY[1] + 1.0, 0.8], target=[BASE_START_XY[0], BASE_START_XY[1], 0.2],
         fname="_trunkverify_03_carry_pose.png")

# ================= 차량 접근 (81번 추천 j1_x, 83번에서 검증된 값) =================
x0, y0 = BASE_START_XY
trajectory_log = []


def drive_to(target_x=None, target_y=None, target_yaw_deg=None, tolerance_xy=0.03, tolerance_yaw_deg=2.0,
             max_speed=0.4, max_wz=0.2, kp_xy=1.8, kp_yaw=0.25, max_steps=3000, label=""):
    """83.test_holonomic_drive.py와 동일한 폐루프 주행(정체 감지+저역통과 필터 포함),
    리프트를 매 스텝 lift_state['h']에 붙잡아둔다(step_hold(1) 사용)."""
    start_pos, start_quat = base_robot.get_world_pose()
    start_yaw = float(np.degrees(quat_to_euler_angles(start_quat)[2]))
    tx = target_x if target_x is not None else float(start_pos[0])
    ty = target_y if target_y is not None else float(start_pos[1])
    tyaw = target_yaw_deg if target_yaw_deg is not None else start_yaw
    print(f"\n[주행 시작]{' ' + label if label else ''} 목표=({tx:.3f},{ty:.3f},{tyaw:.1f}deg)", flush=True)

    STALL_WINDOW, STALL_MIN_PROGRESS = 150, 0.008
    last_check_pos = np.array([float(start_pos[0]), float(start_pos[1])])
    stalled = False
    smooth = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
    SMOOTH_ALPHA = 0.12
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
        smooth["vx"] += SMOOTH_ALPHA * (vx_t - smooth["vx"])
        smooth["vy"] += SMOOTH_ALPHA * (vy_t - smooth["vy"])
        smooth["wz"] += SMOOTH_ALPHA * (wz_t - smooth["wz"])
        base_robot.apply_action(holo_forward(smooth["vx"], smooth["vy"], smooth["wz"]))
        m0609_robot.gripper.close()
        step_hold(1)
        if step % STALL_WINDOW == 0:
            cur = np.array([float(pos[0]), float(pos[1])])
            progress = float(np.linalg.norm(cur - last_check_pos))
            if progress < STALL_MIN_PROGRESS and (abs(ex_w) > tolerance_xy or abs(ey_w) > tolerance_xy):
                stalled = True
                print(f"  [정체 감지] {progress:.4f}m밖에 못 움직임 - 중단", flush=True)
                break
            last_check_pos = cur
            trajectory_log.append({"step": step, "x": float(pos[0]), "y": float(pos[1])})
    for _ in range(30):
        smooth["vx"] *= 1 - SMOOTH_ALPHA
        smooth["vy"] *= 1 - SMOOTH_ALPHA
        smooth["wz"] *= 1 - SMOOTH_ALPHA
        base_robot.apply_action(holo_forward(smooth["vx"], smooth["vy"], smooth["wz"]))
        m0609_robot.gripper.close()
        step_hold(1)
    final_pos, final_quat = base_robot.get_world_pose()
    print(f"[주행 완료]{' ' + label if label else ''} {step}스텝, 최종=({final_pos[0]:.3f},{final_pos[1]:.3f}) "
          f"정체={stalled}", flush=True)
    return final_pos, not stalled


print(f"\n[차량 접근] 목표 x={APPROACH_TARGET_X:.3f} (81번 추천 j1_x)", flush=True)
final_pos, drive_ok = drive_to(target_x=APPROACH_TARGET_X, target_y=0.0, target_yaw_deg=0.0,
                                max_speed=0.15, label="차량 접근(저속, 박스 운반 중)")
box_pos_after_drive, _ = box.get_world_pose()
print(f"[주행 후 박스 위치] ({box_pos_after_drive[0]:.3f},{box_pos_after_drive[1]:.3f},"
      f"{box_pos_after_drive[2]:.3f}) - 흡착 조인트로 붙어서 같이 이동했는지 확인", flush=True)
snapshot(eye=[APPROACH_TARGET_X - 1.0, -1.0, 0.8], target=[APPROACH_TARGET_X, 0.0, 0.3],
         fname="_trunkverify_04_vehicle_approach.png")

# ================= 리프트 상승 (뻗기 준비) =================
move_lift_to(LIFT_MAX, steps=120)

# ================= 트렁크 내부 3x3 목표 그리드 도달성 검증 =================
GRID_MARGIN = 0.08
grid_xs = np.linspace(TRUNK_X_MIN + GRID_MARGIN, TRUNK_X_MAX - GRID_MARGIN, 3)
grid_ys = np.linspace(TRUNK_Y_MIN + GRID_MARGIN, TRUNK_Y_MAX - GRID_MARGIN, 3)
grid_z = TRUNK_FLOOR_Z + CUBE_SIZE / 2.0 + 0.02  # 바닥보다 살짝 위(박스 반높이+여유)

print(f"\n[그리드 검증] x={np.round(grid_xs,3)} y={np.round(grid_ys,3)} z={grid_z:.3f}\n", flush=True)
grid_results = []
best_reachable = None
for gx in grid_xs:
    for gy in grid_ys:
        target = (float(gx), float(gy), float(grid_z))
        ee_pos, err = move_link6(target, hold_gripper_closed=True, label=f"그리드(x={gx:.2f},y={gy:.2f})")
        reachable = bool(err < GRID_TOLERANCE)
        grid_results.append({"x": float(gx), "y": float(gy), "z": float(grid_z),
                              "ee_x": float(ee_pos[0]), "ee_y": float(ee_pos[1]), "ee_z": float(ee_pos[2]),
                              "err_m": float(err), "reachable": reachable})
        if reachable and (best_reachable is None or gx > best_reachable[0]):
            best_reachable = (float(gx), float(gy))

n_reachable = sum(1 for r in grid_results if r["reachable"])
print(f"\n[그리드 결과] {n_reachable}/{len(grid_results)} 지점 도달 성공(err<{GRID_TOLERANCE}m)\n", flush=True)
snapshot(eye=[TRUNK_X_MIN - 1.0, -1.0, 1.5], target=[trunk_center_xy[0], trunk_center_xy[1], TRUNK_FLOOR_Z + 0.2],
         fname="_trunkverify_05_grid_overview.png")

csv_path = OUT_DIR / "_trunk_reachability_grid.csv"
with csv_path.open("w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=["x", "y", "z", "ee_x", "ee_y", "ee_z", "err_m", "reachable"])
    writer.writeheader()
    writer.writerows(grid_results)
print(f"[저장 완료] {csv_path}", flush=True)

# ================= 가장 깊이 도달 가능한 지점에 실제로 내려놓기 =================
place_result = {"attempted": False}
if best_reachable is not None:
    place_target = (best_reachable[0], best_reachable[1], grid_z)
    print(f"\n[PLACE] 가장 깊은 도달 지점 {np.round(place_target,3)}에 실제로 내려놓기 시도\n", flush=True)
    move_link6(place_target, hold_gripper_closed=True, label="PLACE 목표 재접근")
    m0609_robot.gripper.open()
    step_hold(60)
    final_box_pos, _ = box.get_world_pose()
    in_trunk_x = TRUNK_X_MIN - 0.10 <= final_box_pos[0] <= TRUNK_X_MAX + 0.05
    in_trunk_y = TRUNK_Y_MIN <= final_box_pos[1] <= TRUNK_Y_MAX
    near_floor = final_box_pos[2] <= TRUNK_FLOOR_Z + 0.3
    place_result = {
        "attempted": True,
        "target": list(place_target),
        "final_box_pos": [float(v) for v in final_box_pos],
        "in_trunk_x": bool(in_trunk_x), "in_trunk_y": bool(in_trunk_y), "near_floor": bool(near_floor),
        "pass": bool(in_trunk_x and in_trunk_y and near_floor),
    }
    print(f"[PLACE 결과] 박스 최종 위치={np.round(final_box_pos,3)} "
          f"in_trunk_x={in_trunk_x} in_trunk_y={in_trunk_y} near_floor={near_floor} "
          f"-> {'PASS' if place_result['pass'] else 'FAIL'}", flush=True)
    snapshot(eye=[TRUNK_X_MIN - 1.2, -1.2, 1.3], target=[final_box_pos[0], final_box_pos[1], TRUNK_FLOOR_Z],
             fname="_trunkverify_06_final_placed.png")
else:
    print("\n[PLACE] 도달 가능한 그리드 지점이 하나도 없어서 내려놓기 생략\n", flush=True)

# ================= 결과 저장 =================
result = {
    "approach_target_x": APPROACH_TARGET_X,
    "drive_ok": bool(drive_ok),
    "grasped_before_drive": bool(grasped),
    "grid_reachable_count": n_reachable,
    "grid_total": len(grid_results),
    "place": place_result,
}
result_path = OUT_DIR / "_trunk_reachability_result.json"
result_path.write_text(json.dumps(result, indent=2))
print(f"\n[저장 완료] {result_path}", flush=True)
print(f"[전체 결과] {result}\n", flush=True)

print("[안내] 85번(트렁크 도달성 검증) 완료. 창을 계속 열어두고 확인하거나 종료하세요.")
print("[안내] 창을 닫으면 결과가 자동 저장됩니다.\n", flush=True)

while simulation_app.is_running():
    step_hold(1)
    time.sleep(0.01)

SCENE_USD = str(OUT_DIR / "trunk_reachability_scene.usd")
omni.usd.get_context().save_as_stage(SCENE_USD)
print(f"\n[저장 완료] {SCENE_USD}", flush=True)

simulation_app.close()
