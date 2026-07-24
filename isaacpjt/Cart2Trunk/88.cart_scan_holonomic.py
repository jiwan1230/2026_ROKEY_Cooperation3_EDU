"""
88.cart_scan_holonomic.py

Cart2Trunk 최종 시나리오(3PC ROS2 분산 시스템) 1단계 - 카트 옆면 스캔.
82~87번에서 완성한 저상 홀로노믹 베이스 + M0609 + 리프트 + 새 흡착 그리퍼 조합을
실제 카트+차량 트렁크 시나리오에 투입하는 첫 스크립트. 계획 파일
(~/.claude/plans/parallel-juggling-sun.md) 88번 항목 참고.

이 스크립트가 하는 일
----
1. 카트를 씬에 배치(84.py와 동일한 add_asset+SDF 콜리전 패턴).
2. 홀로노믹 베이스를 카트 옆에서 "짧은 축"이 카트를 향하도록 세운다(84/87번에서
   이미 검증된 CHASSIS_HALF_WIDTH_EFFECTIVE 기반 standoff 공식 재사용).
3. 옴니휠 평행 이동(strafe, drive_to로 y만 변경, 회전 없음)으로 카트 옆면에 접근한다
   (Nova Carter로는 안 되고 이 홀로노믹 베이스라서 가능한 동작 - 최종 시나리오 문서
   "옴니휠 특성을 이용해서 접근" 항목).
4. 35.crate_scan_setup.py에서 검증된 스캔 자세 공식을 그대로 재사용한다 - eye를 목표
   바로 위에서 height*tan(21도)만큼 로봇 쪽으로 수평 오프셋을 준 지점에 두면 look_at과의
   관계로 21도 틸트가 기하학적으로 자연히 나온다(회전 트릭 아님). lookat_to_link6_target()
   으로 이 eye/look_at을 link6 목표로 역산해서 RMPflow로 한 번에 수렴시킨다.
5. 32.box_table_scan_setup.py에서 검증된 ROS2CameraHelper 패턴으로 /camera/depth,
   /camera/camera_info를 발행하고, 이 SCAN_POSE 기준 base_to_camera_transform.json을
   저장한다(perception/box_top_extractor.py가 그대로 읽어서 쓸 수 있도록 동일 스키마).
6. box_top_extractor.py는 별도 터미널(별도 venv, rclpy)에서 사용자가 직접 띄운다 -
   이 스크립트는 그 안내 문구만 출력하고 카메라를 그 자세에 계속 고정해둔다.

이 스크립트가 다루지 않는 것 (다음 단계로 미룸, 계획 파일 참고)
----
- box_top_extractor.py의 결과 JSON -> box_scan.json 스키마 변환 어댑터(center/size/yaw를
  corners_m으로 minAreaRect 재계산, confidence는 fill_ratio 노출) - 89번 이후 별도 작업.
- 트렁크 스캔(89번), PICK/PLACE(90/91번), 전체 루프(92번).
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
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, UsdLux, UsdShade, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.rotations import quat_to_euler_angles, euler_angles_to_quat
from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
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

# ---------------- 84.py와 동일한 카트/베이스 구성 ----------------
CART_USD = str(_THIS_DIR / "assets/Metal_Shopping_Cart.usdz")
CART_POS = (0.0, 0.0, 0.0)
CART_EXTRA_SCALE = 0.55
SDF_RESOLUTION = 256
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 0.0, 50.0, 20.0

BASE_PATH = "/World/HoloBase"
CHASSIS_PATH = f"{BASE_PATH}/chassis"
# 84번과 동일 이유 - 섀시의 "짧은 축"(폭 ~0.4m)이 카트를 향하게 90도로 세운다.
BASE_FACE_ROT_Z = 90.0

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
# 사용자 지적: 카트+바스켓을 잘 내려다보려면 리프트를 조금 더 올려야 한다(0.35 -> 0.45).
LIFT_TRAVEL_M = 0.45

EE_LINK_NAME = "link_6"
GRIPPER_BODY_NAME = "vgp20_suction_plate"
DEPTH_CAMERA_NAME_HINT = "Depth"

STANDOFF_MARGIN = 0.10
WAYPOINT_STEPS = 300
SETTLE_STEPS = 60
DOWN_QUAT = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))
WORLD_UP = (0.0, 0.0, 1.0)
CAMERA_AXES = "usd"

# ---- 스캔 자세 파라미터 (35.crate_scan_setup.py의 검증된 공식 그대로 재사용) ----
# 사용자 지적 - 손목 조인트를 직접 돌리거나 사후 회전을 추가하는 방식은 전부 발산/엉뚱한
# 방향을 봄으로 실패했다. 35.py를 보니 21도는 회전 트릭이 아니라 **eye 위치의 수평
# 오프셋을 height*tan(21도)로 계산**해서 eye/look_at 자체의 기하학적 배치로 21도가
# 자연히 나오게 하는 방식이었다(35.py 205-212행) - 그 공식을 그대로 가져온다.
CART_BASKET_FLOOR_Z = 0.68
EYE_HEIGHT_ABOVE_CART = 0.75  # 35.py의 EYE_HEIGHT_ABOVE_TABLE과 동일값
SCAN_TILT_FROM_VERTICAL_DEG = 30.0  # 35.py의 SCAN_TILT_FROM_VERTICAL_DEG와 동일
_scan_horizontal_offset = EYE_HEIGHT_ABOVE_CART * np.tan(np.radians(SCAN_TILT_FROM_VERTICAL_DEG))

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
    """82~87번과 동일 패턴 - 독립 articulation + 매 프레임 텔레포트."""
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


# ================= 씬 구성 =================
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
target_mpu = UsdGeom.GetStageMetersPerUnit(stage)
target_up = UsdGeom.GetStageUpAxis(stage)

add_asset(stage, "/World/ShoppingCart", CART_USD, CART_POS, CART_EXTRA_SCALE, target_mpu, target_up)
for _ in range(20):
    simulation_app.update()
add_sdf_collision(stage, "/World/ShoppingCart")

cart_min, cart_max = bbox_of(stage, "/World/ShoppingCart")
cart_center_xy = ((cart_min[0] + cart_max[0]) / 2.0, (cart_min[1] + cart_max[1]) / 2.0)
cart_half_x = (cart_max[0] - cart_min[0]) / 2.0
cart_half_y = (cart_max[1] - cart_min[1]) / 2.0
print(f"[카트 bbox] min={cart_min} max={cart_max} center_xy={cart_center_xy} half_x={cart_half_x:.3f}", flush=True)

# ---- 카트 안에 박스 2개 배치 (84.py와 동일 패턴) - 우선 실제 비전 검증용으로 2개만 ----
box_material = PhysicsMaterial(
    prim_path="/World/Physics_Materials/box_material",
    static_friction=1.2, dynamic_friction=1.0, restitution=0.0,
)
CART_BOX_SPECS = [
    ("Box_A", (0.16, 0.12, 0.11), (0.0, -cart_half_y * 0.35)),
    ("Box_B", (0.13, 0.10, 0.09), (0.0, cart_half_y * 0.35)),
]
CART_BOX_DROP_HEIGHT_ABOVE_FLOOR = 0.10
for name, size, (dx, dy) in CART_BOX_SPECS:
    DynamicCuboid(
        prim_path=f"/World/{name}",
        name=name.lower(),
        position=np.array([
            cart_center_xy[0] + dx,
            cart_center_xy[1] + dy,
            CART_BASKET_FLOOR_Z + CART_BOX_DROP_HEIGHT_ABOVE_FLOOR,
        ]),
        scale=np.array(size),
        color=np.array([0.85, 0.55, 0.15]),
        mass=0.3,
        physics_material=box_material,
    )
print(f"[박스 배치] 카트 안에 {len(CART_BOX_SPECS)}개 낙하 예정: {[s[0] for s in CART_BOX_SPECS]}", flush=True)

STANDOFF_X = CHASSIS_HALF_WIDTH_EFFECTIVE + cart_half_x + STANDOFF_MARGIN
print(f"[STANDOFF] {CHASSIS_HALF_WIDTH_EFFECTIVE:.3f}(섀시 반폭) + {cart_half_x:.3f}(카트 반폭) + "
      f"{STANDOFF_MARGIN:.3f}(여유) = {STANDOFF_X:.3f}m", flush=True)

area_light = UsdLux.SphereLight.Define(stage, "/World/ScanAreaLight")
area_light.CreateRadiusAttr(0.3)
area_light.CreateIntensityAttr(60000)
UsdGeom.Xformable(area_light).AddTranslateOp().Set(Gf.Vec3d(cart_center_xy[0], cart_center_xy[1], 2.0))

# 베이스 시작 위치: 카트에서 STANDOFF_X만큼 떨어진 곳에서 대기 -> strafe로 접근
BASE_START_XY = (cart_center_xy[0] + STANDOFF_X + 0.3, cart_center_xy[1])
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


def move_link6(target_pos, steps=WAYPOINT_STEPS, label="", orientation=DOWN_QUAT):
    for i in range(steps):
        sync_rmp_base()
        actions = controller.forward(
            target_end_effector_position=np.array(target_pos, dtype=float),
            target_end_effector_orientation=orientation,
        )
        m0609_robot.apply_action(actions)
        set_lift_height(lift_state["h"])
        world.step(render=True)
    ee_pos, _ = m0609_robot.end_effector.get_world_pose()
    err = np.linalg.norm(np.array(ee_pos) - np.array(target_pos))
    print(f"[웨이포인트{' ' + label if label else ''}] target={np.round(target_pos, 3)} "
          f"ee={np.round(ee_pos, 3)} err={err:.4f}m", flush=True)
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

# ================= 1. 옴니휠 평행이동(strafe)으로 카트 옆면 접근 =================
target_xy = (cart_center_xy[0] + STANDOFF_X, cart_center_xy[1])
drive_to(target_x=target_xy[0], target_y=target_xy[1], label="카트 옆면 접근(strafe)")
snapshot(
    eye=[target_xy[0] - 1.0, target_xy[1] - 1.3, 1.4],
    target=[cart_center_xy[0], cart_center_xy[1], 0.5],
    fname="_cartscan_01_approached.png",
)

# ================= 2. SCAN_POSE (35.crate_scan_setup.py의 검증된 공식/패턴 그대로) =================
# 35.py 205~212행과 완전히 동일한 구성: eye는 목표(바스켓) 바로 위에서 height*tan(21도)만큼
# 로봇 쪽으로(우리 접근축인 +X 방향으로) 수평 오프셋을 준 지점, look_at은 바스켓 바닥
# 중심. lookat_to_link6_target()이 이 eye/look_at을 바로 link6 목표 pos+quat으로 역산하고,
# 35.py의 converge_to_pose()처럼 웨이포인트 분할 없이 한 번에 수렴시킨다(먼 대각선 재배치가
# 아니라 원래 이 근방에 있던 eye 근처로 가는 것이라 한 번에 가도 안정적으로 수렴함).
SCAN_EYE = np.array([
    cart_center_xy[0] + _scan_horizontal_offset,
    cart_center_xy[1],
    CART_BASKET_FLOOR_Z + EYE_HEIGHT_ABOVE_CART,
])
SCAN_LOOK_AT = np.array([cart_center_xy[0], cart_center_xy[1], CART_BASKET_FLOOR_Z])

target_pos, target_quat = lookat_to_link6_target(SCAN_EYE, SCAN_LOOK_AT)
print(f"[SCAN_POSE 목표] link6_pos={np.round(target_pos, 3)} eye={np.round(SCAN_EYE, 3)} "
      f"look_at={np.round(SCAN_LOOK_AT, 3)}", flush=True)
move_link6(target_pos, steps=350, label="스캔 자세 수렴", orientation=target_quat)
snapshot(
    eye=[target_xy[0] - 0.8, target_xy[1] - 1.0, SCAN_EYE[2] + 0.3],
    target=[cart_center_xy[0], cart_center_xy[1], 0.4],
    fname="_cartscan_02_hover.png",
)

ee_pos_scan, ee_quat_scan = m0609_robot.end_effector.get_world_pose()
print(f"[SCAN_POSE 도달] ee_pos={np.round(ee_pos_scan, 4)} ee_quat={np.round(ee_quat_scan, 4)}", flush=True)
snapshot(
    eye=[target_xy[0] - 0.8, target_xy[1] - 1.0, SCAN_EYE[2] + 0.3],
    target=[cart_center_xy[0], cart_center_xy[1], 0.4],
    fname="_cartscan_03_scan_pose.png",
)

# ================= 3. base_to_camera_transform.json 저장 + ROS2 카메라 브리지 연결 =================
step_hold(10)
base_pos_final, base_quat_final = base_robot.get_world_pose()
base_pos_final = np.array(base_pos_final) + np.array([0.0, 0.0, lift_state["h"]])
cam_pos_final, cam_quat_final = camera.get_world_pose(camera_axes=CAMERA_AXES)
R_cam_final = quat_wxyz_to_matrix(np.array(cam_quat_final))

# 32.py와 동일 보정 - optical(+Y down,+Z forward) <-> USD 카메라 축(+Y up,-Z forward)
OPTICAL_TO_USD_CAMERA_AXES = np.diag([1.0, -1.0, -1.0])
R_base = quat_wxyz_to_matrix(np.array(base_quat_final))
R_base_to_cam = R_base.T @ R_cam_final @ OPTICAL_TO_USD_CAMERA_AXES
t_base_to_cam = R_base.T @ (np.array(cam_pos_final) - base_pos_final)

transform_path = PERCEPTION_DIR / "base_to_camera_transform.json"
transform_payload = {
    "R": R_base_to_cam.tolist(),
    "t": t_base_to_cam.tolist(),
    "note": (
        "88.cart_scan_holonomic.py가 만든 고정 카트 스캔 자세(SCAN_POSE, eye/look_at 기하학적 "
        f"{SCAN_TILT_FROM_VERTICAL_DEG}deg 틸트) 전용. 팔이 이 자세를 벗어나면 무효 - 재측정 필요."
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

print("\n[안내] SCAN_POSE 고정 완료. 다음 단계:", flush=True)
print("  1) 별도 터미널에서:", flush=True)
print("       source /opt/ros/humble/setup.bash", flush=True)
print("       source perception/.venv/bin/activate", flush=True)
print("       python3 perception/box_top_extractor.py", flush=True)
print("  2) 화면에서 카트+박스가 잘 보이면 S키로 저장, Q로 종료.", flush=True)
print(f"  3) 결과는 ~/box_pointcloud/all_boxes_corners_*.json 에 저장됨", flush=True)
print("     (다음 단계 스크립트에서 box_scan.json 스키마로 변환 예정 - 계획 파일 참고)\n", flush=True)

if HEADLESS:
    SCENE_OUT = str(OUT_DIR / "cart_scan_holonomic_scene.usd")
    omni.usd.get_context().save_as_stage(SCENE_OUT)
    print(f"[저장 완료] {SCENE_OUT}", flush=True)
    simulation_app.close()
else:
    print("[안내] 창을 직접 둘러보세요 - 닫으면 스크립트가 종료됩니다.\n", flush=True)
    # M0609는 독립 articulation이라 매 스텝 set_lift_height()로 텔레포트해줘야 섀시에
    # "붙어" 있다(82~87번 전체의 공통 설계) - 여기서 world.step()만 부르면 그 텔레포트가
    # 멈춰서 중력으로 떨어진다(실측 확인). step_hold(1)을 써서 계속 붙잡아둔다.
    while simulation_app.is_running():
        step_hold(1)
    simulation_app.close()
