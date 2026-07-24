"""
89.trunk_scan_holonomic.py

Cart2Trunk 최종 시나리오 2단계 - 트렁크 다중 시점 스캔.
계획 파일(~/.claude/plans/parallel-juggling-sun.md) 89번 항목 참고.

12.trunk_scan_hidden_gripper.py에서 이미 검증된 "그리퍼 끝 고정 앵커 + look_at만 바꿔가며
스윕" 방식을 82~88번에서 완성한 홀로노믹 베이스 + M0609 + 리프트 + 새 흡착 그리퍼 조합으로
그대로 포팅한다. 88번에서 사용자가 확인해준 교훈 그대로: 좌표/자세는 직접 재발명하지 않고
과거에 검증된 스크립트의 수식을 그대로 가져온다.

12.py 대비 달라진 점
----
1. **로봇 자체**: Nova Carter+고정 마운트 -> 홀로노믹 베이스+리프트(82~88번). 로봇 베이스는
   85/87번과 마찬가지로 실제 메카넘 휠 주행(drive_to)으로 이동하고, 리프트로 마운트 높이를
   확보한다(이번 1차 버전은 트렁크 밖 표준 standoff에서 스캔 - 저상 베이스로 차량 하부까지
   파고드는 것은 PLACE(91번) 전용 과제로 남겨둔다, 계획 파일 참고).
2. **anchor_mode="gripper_tip" 오프셋**: 12.py는 2-finger 그리퍼라 좌우 finger 링크의 평균
   위치로 손가락 끝단을 측정했지만, 이 로봇은 평평한 흡착판이라 84~88번처럼
   `_gripper_physical_range.json`의 TIP_LOCAL_OFFSET(그리퍼 바디 프림 기준 로컬 오프셋)을
   그대로 재사용해서 "흡착판 끝" 월드 위치를 계산한다.
3. 트렁크 자체의 물리 상수(TRUNK_X_MIN/MAX, TRUNK_Y_MIN/MAX, TRUNK_FLOOR_Z, TRUNK_WALL_TOP)와
   앵커/스윕 waypoint 공식은 전부 12.py 그대로 재사용 - 이건 차량 모델 자체의 실측값이라
   로봇이 바뀌어도 안 바뀐다.

출력
----
- results/holonomic_base/trunk_pointcloud.npy, trunk_pointcloud_meta.json (12.py와 동일 스키마) -
  다음 단계(13.export_trunk_map.py 포팅)에서 그대로 읽어서 trunk_map.json을 만드는 데 쓴다.
"""

from isaacsim import SimulationApp

import os

HEADLESS = os.environ.get("HEADLESS", "0") == "1"
_sim_app_config = {"headless": HEADLESS}
if not HEADLESS:
    _sim_app_config.update({"width": 640, "height": 480})
simulation_app = SimulationApp(_sim_app_config)

import json
import sys
from pathlib import Path

import numpy as np
import omni.graph.core as og
import omni.usd
import omni.kit.viewport.utility as vp_util
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, UsdLux, UsdShade, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.rotations import quat_to_euler_angles
from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.sensors.camera import Camera
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")

_THIS_DIR = Path(__file__).resolve().parent
OUT_DIR = _THIS_DIR / "results" / "holonomic_base"
OUT_DIR.mkdir(parents=True, exist_ok=True)
PERCEPTION_DIR = _THIS_DIR / "perception"
PERCEPTION_DIR.mkdir(parents=True, exist_ok=True)

M0609_DIR = _THIS_DIR.parent / "M0609"
RMPFLOW_DIR = str(M0609_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController  # noqa: E402

# ---------------- 12.py와 완전히 동일 - 차량/트렁크 실측 상수 ----------------
CAR_USD = str(_THIS_DIR / "assets/Lexus_IS300_Trunk_Open_No_More_Hell_Room.usdz")
CAR_POS = (5.0, 0.0, 0.0)
CAR_EXTRA_SCALE = 0.50
CAR_ROT_Z = 0.0
TRUNK_X_MIN, TRUNK_X_MAX = 3.11, 3.68
TRUNK_Y_MIN, TRUNK_Y_MAX = -0.56, 0.56
TRUNK_FLOOR_Z = 0.44
TRUNK_WALL_TOP = 1.28
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

# FACE_ROT_Z=0이라 긴 축(CHASSIS_LENGTH_EXTENDED/2)이 트렁크 쪽으로 뻗는다 - standoff는
# 반길이 기준으로 계산한다(84/88번의 반폭 기준 공식과 동일 원리, 축만 다름).
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
    """82~88번과 동일 패턴 - 독립 articulation + 매 프레임 텔레포트."""
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


def find_camera_prim_path(stage, root_path, name_hint):
    root_prim = stage.GetPrimAtPath(root_path)
    candidates = []
    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdGeom.Camera):
            candidates.append(str(prim.GetPath()))
    for c in candidates:
        if name_hint.lower() in c.lower():
            return c, candidates
    return (candidates[0] if candidates else None), candidates


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


def _normalize(v, eps=1e-9):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < eps:
        raise ValueError(f"영벡터는 방향으로 사용할 수 없습니다: {v}")
    return v / n


def make_usd_camera_rotation(eye, look_at, up_ref=WORLD_UP):
    """12/32/88.py와 완전히 동일."""
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

area_light = UsdLux.SphereLight.Define(stage, "/World/TrunkScanAreaLight")
area_light.CreateRadiusAttr(0.3)
area_light.CreateIntensityAttr(80000)
UsdGeom.Xformable(area_light).AddTranslateOp().Set(Gf.Vec3d(TRUNK_X_MIN + 0.3, 0.0, TRUNK_FLOOR_Z + 1.0))

# ---- 로봇 standoff: 12.py의 ROBOT_XY=(TRUNK_X_MIN-0.85,-0.15) 대신, 이 홀로노믹 베이스의
# 실제 반길이 기준으로 표준 standoff를 계산한다(84/88번과 동일 공식, 축만 반길이로 교체) ----
STANDOFF_TRUNK = CHASSIS_HALF_LENGTH_EFFECTIVE + STANDOFF_MARGIN
print(f"[STANDOFF] {CHASSIS_HALF_LENGTH_EFFECTIVE:.3f}(섀시 반길이) + {STANDOFF_MARGIN:.3f}(여유) "
      f"= {STANDOFF_TRUNK:.3f}m (참고: 12.py는 0.85m 고정값 사용)", flush=True)

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

m0609_robot = SingleManipulator(
    prim_path=m0609_base_link_path,
    end_effector_prim_path=ee_path,
    name="m0609_arm",
)
base_robot = SingleArticulation(prim_path=chassis_path, name="holo_base")

world.reset()
base_robot.initialize(physics_sim_view=world.physics_sim_view)
m0609_robot.initialize(physics_sim_view=world.physics_sim_view)
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
drive_to(target_x=BASE_START_XY[0], target_y=BASE_START_XY[1], label="트렁크 앞 대기 위치")

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

if HEADLESS:
    SCENE_OUT = str(OUT_DIR / "trunk_scan_holonomic_scene.usd")
    omni.usd.get_context().save_as_stage(SCENE_OUT)
    print(f"[저장 완료] {SCENE_OUT}", flush=True)
    simulation_app.close()
else:
    print("[안내] 창을 직접 둘러보세요 - 닫으면 스크립트가 종료됩니다.\n", flush=True)
    while simulation_app.is_running():
        step_hold(1)
    simulation_app.close()
