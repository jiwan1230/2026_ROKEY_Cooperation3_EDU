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
import random

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
# [다중 시점 스캔 추가 후 실측] 원래 SCAN_EYE(EYE_HEIGHT_ABOVE_CART=0.75, tilt=30도)로
# 계산한 link6 목표까지 필요한 3D 거리가 약 0.91m로 나왔다 - M0609(Doosan, 이름 자체가
# 0.9m reach/6kg payload를 뜻함)의 최대 도달 거리와 거의 같거나 넘는다. 그래서 IK가
# 350스텝을 다 써도 수렴 못 하고 8~12cm 오차로 멈췄다(물리적으로 못 닿는 거리라 스텝을
# 늘려도 소용없음). 리프트를 더 올려서 팔 자신의 base 높이를 목표에 가깝게 만들면
# 수직 방향 도달 거리가 줄어든다(카메라 자세 자체는 안 바뀜 - 순수하게 팔이 닿기 쉬워짐).
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

# ---- 카트 안에 적층 박스 5개 배치 (2단 피라미드) ----
# 84.py의 단순 낙하 패턴 + 35.py의 "같은 높이차 유지" 낙하 트릭을, 3개(Base+2개)
# 구조에서 검증된 뒤 부모-자식 체인으로 일반화했다. 구조:
#   Base(바닥, 최대) - M1(Base 뒤-왼쪽) - XS1(M1 뒤쪽에 적층)
#                    - M2(Base 뒤-오른쪽) - XS2(M2 뒤쪽에 적층)
# "위 박스는 항상 아래 박스보다 작다"는 각 부모-자식 관계마다 개별적으로 지킨다
# (M1/M2끼리는 서로 위아래로 안 쌓이므로 크기 비교 대상이 아님).
#
# 3개짜리 구조에서 실측으로 확인한 3가지 설계 원칙을 그대로 재사용한다:
#  1) (검출 0개 버그) 자식 박스를 부모 중심에 두면 부모 윗면이 거의 다 덮여서 부모 자체가
#     통째로 검출 실패한다 - 자식을 부모의 뒤쪽(+Y) "절반"에만 둬서 앞쪽 절반을 항상
#     깨끗하게 남긴다. 이 원칙을 M1/M2 자신에게도(XS1/XS2를 다는 부모 입장에서) 그대로
#     적용한다.
#  2) (평면 병합 버그) 같은 부모를 공유하는 형제 박스(M1-M2)는 간격 4cm 이상
#     (DBSCAN_EPS_M=2.5cm보다 확실히 넓게)로 벌린다.
#  3) (지지면 오검출 버그, D-5) 자식의 높이는 부모와의 높이차가
#     DEDUP_OVERLAP_Z_TOLERANCE_M(0.05m)보다 확실히 크도록(margin 포함 0.06m 이상) 잡는다.
#
# world.reset()이 아직 호출되기 전이라 이 시점의 world.step()은 물리 타임라인을
# 진행시키지 않는다 - 그래서 spawn_z 계산은 "낙하 후 정착 높이"가 아니라 "spawn 시점
# 기준 상대 높이차"다. 자식의 spawn_z를 (부모 spawn_z + 부모 높이 + margin)으로 잡으면,
# world.reset() 이후 전부 같은 중력으로 동시에 떨어지기 시작해도 이 높이차가 낙하 내내
# 유지되다가 부모가 먼저 바닥/조상 위에 닿아 멈추면 자식은 남은 差만큼만 더 떨어져 그
# 위에 안착한다(35.crate_scan_setup.py의 STACK_TOP_NAME 낙하와 동일 원리) - 조상까지
# 거슬러 올라가는 체인이어도 각 단계가 "그 직속 부모"만 기준으로 계산되면 그대로 성립한다.
box_material = PhysicsMaterial(
    prim_path="/World/Physics_Materials/box_material",
    static_friction=1.2, dynamic_friction=1.0, restitution=0.0,
)
# 실측 확인(5개 적층 배치 후): M1 쪽은 안정적으로 검출되는데 M2 쪽은 시도마다 높이가
# 0.045~0.13m로 들쭉날쭉했다 - M2가 M1보다 작게 설계돼서(자기 노출 깨끗한 면적이
# M1의 약 73% 수준) RANSAC이 안정적으로 잡을 만한 면적 자체가 더 작았던 것으로 보임.
# M2/XS2를 M1/XS1과 비슷한 크기로 키워서 노출 면적을 넓힌다(그래도 Base보다는
# 작음 - "위 박스는 항상 아래 박스보다 작다" 유지).
# 실측 확인(10개 시점 스캔 이후에도): XS1/XS2는 2단 위(Base->M1/M2->XS)에 있는 가장
# 작고 가장 높은 박스라 여전히 검출이 불안정했다(150회 시도해도 둘 다 동시에 잡힌
# 적이 없었음). M2 때와 같은 방식으로 XS1/XS2 자체를 키워서 노출 면적에 여유를 준다
# (그래도 M1/M2보다는 작음 - "위 박스는 항상 아래 박스보다 작다" 유지). XS가 커진
# 만큼 M1/M2의 앞쪽 깨끗한 노출 띠가 줄지 않도록 M1/M2 깊이도 같이 늘리고, 그만큼
# Base 깊이도 늘려서 뒤쪽 배치 여유를 유지한다.
# 실측 확인(RANSAC 완화 패스 + 지지면 매칭 정확도 검증): XS1/XS2가 부모(M1/M2)와
# 같은 시도에서 동시에 정확히 잡혀야 지지면이 올바르게 매칭되는데, M1/M2의 "깨끗한
# 노출 띠"(XS가 덮지 않는 부분)가 좁을수록(현재 X축 마진 약 1.5cm) 그 확률이 낮아져
# 지지면 매칭이 종종 한 단계 건너뛰어 높이를 과소평가한다(XS2 실측 0.06m -> 검출
# 0.031m, 48% 오차). Base/M1/M2를 XS는 그대로 둔 채 대칭으로 키워서 노출 띠
# 마진을 넓히면 이 문제가 완화되는지 확인하기 위해 마진 증가량을 환경변수로 뺀다
# (기본값 0 = 기존 크기 그대로, 다른 실행에 영향 없음). "위 박스는 항상 아래
# 박스보다 작다" 규칙은 XS는 안 키우고 Base/M1/M2만 키우므로 계속 유지된다.
#
# [실측 확인 - 단순 균등 확대의 버그] 처음엔 M1/M2/Base를 같은 오프셋을 유지한 채
# 크기만 키웠는데, M1과 M2 사이 간격(오프셋 차 0.195m)은 그대로인 채 두 박스의
# 반너비 합이 그 간격을 넘어서자(약 CART2TRUNK_MARGIN_GROWTH_M=0.03부터) 두
# 박스가 물리적으로 겹쳐 스폰되어(0.06에서는 Base 자체도 겹침) 물리 안정화가
# 완전히 깨졌다(검출 0/3). 그래서 오프셋과 Base 크기를 M1/M2 크기에 맞춰 매번
# 재계산해서, 성장량과 무관하게 항상 최소 간격(_MIN_CLEARANCE_M)을 유지하도록
# 고친다.
# 기본값 0.015 = 실측 확인된 최소 안정 크기(box-size 탐색 결론): 물리적으로
# 안정적으로 정착하면서(단계별 낙하+정착 도입 이후) 검출 위치 정확도가 가장
# 크게 개선된 지점 - 더 키우면(>=0.03) 이 카트 내부 공간에서 형제 박스끼리
# 충돌하거나 착지 충격에 자식 박스가 튕기는 새로운 물리 불안정이 나타난다.
_MARGIN_GROWTH_M = float(os.environ.get("CART2TRUNK_MARGIN_GROWTH_M", "0.015"))
# 실측 확인(growth=0.03): 2cm 여유로는 M1/M2가 각자 Base에 착지하면서 받는 충격만으로도
# 서로 충돌해 크게 밀려날 수 있었다(단계별 정착 자체는 정상 동작 - 문제는 형제 박스
# 사이 여유 부족). 환경변수로 빼서 필요시 더 넓힐 수 있게 한다.
_MIN_CLEARANCE_M = float(os.environ.get("CART2TRUNK_MIN_CLEARANCE_M", "0.02"))
_BASE_EDGE_MARGIN_M = 0.03


def _grown(size_xy):
    return (size_xy[0] + 2.0 * _MARGIN_GROWTH_M, size_xy[1] + 2.0 * _MARGIN_GROWTH_M)


_m1_size = _grown((0.16, 0.16))
_m2_size = _grown((0.15, 0.17))
_m1_dx = -(_m1_size[0] / 2.0 + _MIN_CLEARANCE_M / 2.0)
_m2_dx = +(_m2_size[0] / 2.0 + _MIN_CLEARANCE_M / 2.0)
_base_half_x = max(
    abs(_m1_dx) + _m1_size[0] / 2.0, abs(_m2_dx) + _m2_size[0] / 2.0
) + _BASE_EDGE_MARGIN_M
_base_half_y = max(0.07 + _m1_size[1] / 2.0, 0.07 + _m2_size[1] / 2.0) + _BASE_EDGE_MARGIN_M
_base_size = (2.0 * _base_half_x, 2.0 * _base_half_y)

# 실측 확인(CART2TRUNK_MARGIN_GROWTH_M 박스 크기 탐색 - 물리 버그 원인 규명): 크기만
#키우고 질량은 고정해두면(면적당 밀도가 growth=0.015에서 이미 약 33% 낮아짐) 큰
# 박스일수록 착지 충격에 더 쉽게 밀리는데, 그 위에 얹힐 자식(XS1/XS2)은 "부모와
# 함께 동시에 자유낙하하다가 부모가 먼저 멈추면 남은 낙차만큼만 떨어져 안착"하는
# 트릭(548행 설명)을 쓰므로 부모가 착지 중 옆으로 밀리면 자식은 이미 정해진(부모가
# 밀리기 전) XY로 계속 떨어져 부모를 완전히 빗나간다 - 실측: growth=0.015에서
# XS1이 스캔 시작 전(초기 정착 직후)부터 이미 Y로 12.6cm 밀려나 있었고(안정화
# 스텝을 60->200으로 늘려도 동일 - 정착 시간 부족이 아니라 착지 충격 자체의
# 문제), growth=0.03에서는 아예 카트 밖으로 튕겨나갔다. 면적이 커진 만큼 질량도
# 같이 늘려서(밀도 고정) 원래 크기에서 검증됐던 충격 반응을 그대로 유지한다.
_base_area_0 = 0.38 * 0.34
_m1_area_0 = 0.16 * 0.16
_m2_area_0 = 0.15 * 0.17
_base_density = 1.8 / _base_area_0
_m1_density = 0.7 / _m1_area_0
_m2_density = 0.6 / _m2_area_0

# 랜덤 배치 검증(사용자 요청): "더 큰 박스가 항상 더 작은 박스 아래에 깔린다"는
# 적층 순서(Base>M1/M2>XS1/XS2)는 그대로 고정하고, 그 안에서 각 자식이 부모 위
# 어디에 놓이는지(offset)만 매 시드마다 무작위로 바꿔서 이번에 고친 파이프라인이
# 특정 배치 하나에만 맞춰진 게 아닌지 확인한다. CART2TRUNK_RANDOM_SEED가 없으면
# (기본) 지금까지 실측 검증해온 고정 offset을 그대로 쓴다 - 기존 동작 불변.
# 범위는 형제 박스 간 최소 간격(_MIN_CLEARANCE_M)과 Base/부모 가장자리 여유를
# 침범하지 않도록 넉넉히 보수적으로 잡았다(M1/M2 dy는 Base 안에, XS dx/dy는
# 부모 노출 스트립이 완전히 없어지지 않을 만큼만).
_RANDOM_SEED = os.environ.get("CART2TRUNK_RANDOM_SEED")
if _RANDOM_SEED is not None:
    _rng = random.Random(int(_RANDOM_SEED))
    _m1_dy = _rng.uniform(0.05, 0.09)
    _m2_dy = _rng.uniform(0.05, 0.09)
    _xs1_off = (_rng.uniform(-0.015, 0.015), _rng.uniform(0.0, 0.035))
    _xs2_off = (_rng.uniform(-0.015, 0.015), _rng.uniform(0.0, 0.035))
    print(f"[랜덤 배치] seed={_RANDOM_SEED} M1_dy={_m1_dy:.4f} M2_dy={_m2_dy:.4f} "
          f"XS1_off={tuple(round(v,4) for v in _xs1_off)} XS2_off={tuple(round(v,4) for v in _xs2_off)}",
          flush=True)
else:
    _m1_dy, _m2_dy = 0.07, 0.07
    _xs1_off, _xs2_off = (0.0, 0.02), (0.0, 0.02)

CART_BOX_TOPOLOGY = [
    # (name, size(x,y,z), parent_name(None=바닥), 부모 중심 기준 offset(dx,dy), mass_kg)
    ("Base", (*_base_size, 0.12), None, (0.0, 0.0), _base_density * _base_size[0] * _base_size[1]),
    ("M1", (*_m1_size, 0.11), "Base", (_m1_dx, _m1_dy), _m1_density * _m1_size[0] * _m1_size[1]),
    ("M2", (*_m2_size, 0.09), "Base", (_m2_dx, _m2_dy), _m2_density * _m2_size[0] * _m2_size[1]),
    ("XS1", (0.13, 0.10, 0.07), "M1", _xs1_off, 0.25),
    ("XS2", (0.12, 0.11, 0.06), "M2", _xs2_off, 0.22),
]
print(
    f"[박스 크기] CART2TRUNK_MARGIN_GROWTH_M={_MARGIN_GROWTH_M} -> "
    + ", ".join(f"{n}={sz[0]:.3f}x{sz[1]:.3f}" for n, sz, *_r in CART_BOX_TOPOLOGY),
    flush=True,
)
CART_BOX_DROP_HEIGHT_ABOVE_FLOOR = 0.10
_CART_STACK_TOP_SPAWN_MARGIN_M = 0.05

# 실측 확인(CART2TRUNK_MARGIN_GROWTH_M 박스 크기 탐색 - 물리 버그 근본 원인): 기존
# 방식은 전체 5개를 한 번에 스폰하고 "부모와 자식이 동시에 자유낙하하다가 부모가
# 먼저 멈추면 자식은 남은 낙차만큼만 더 떨어져 안착"하는 트릭(자식의 spawn_z를
# 부모 spawn_z 기준 상대값으로 계산)에 의존했다 - 이 트릭은 "부모가 착지 중 옆으로
# 안 밀린다"는 가정이 성립해야만 맞는데, 박스가 커질수록(질량을 면적에 비례해
# 키워도 마찬가지 - 확인 완료) 착지 충격에 더 쉽게 밀리고, 자식은 이미 스폰 시점에
# 고정된 XY로 계속 떨어지므로 부모가 밀리면 그대로 빗나간다(실측: growth=0.015에서
# XS1이 초기 정착 직후부터 이미 12.6cm 밀려나 있었음, growth=0.03에서는 카트 밖으로
# 튕겨나감). 그래서 부모-자식 관계를 depth 기준으로 "레벨"별로 나눠서, 레벨 하나를
# 스폰하고 실제 물리로 정착시킨 뒤 그 실측 위치를 읽어와 다음 레벨(자식)의 스폰
# 좌표로 쓴다 - 부모가 얼마나 밀리든 자식은 항상 부모의 "실제" 최종 위치 위에
# 스폰되므로 이 문제 자체가 원천적으로 사라진다.
_cart_box_levels = []
_remaining_topology = list(CART_BOX_TOPOLOGY)
_placed_names = set()
while _remaining_topology:
    level = [t for t in _remaining_topology if t[2] is None or t[2] in _placed_names]
    if not level:
        raise RuntimeError("CART_BOX_TOPOLOGY에 알 수 없는 parent_name이 있습니다 (순환 참조 의심).")
    _cart_box_levels.append(level)
    _placed_names.update(t[0] for t in level)
    _remaining_topology = [t for t in _remaining_topology if t not in level]
print(f"[박스 배치] 카트 안에 적층 구조 {len(CART_BOX_TOPOLOGY)}개, "
      f"{len(_cart_box_levels)}단계로 나눠 순차 낙하+정착 예정: "
      f"{[[t[0] for t in lvl] for lvl in _cart_box_levels]}", flush=True)

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


_STABILIZE_STEPS = int(os.environ.get("CART2TRUNK_STABILIZE_STEPS", "60"))
_cart_box_xy_by_name = {}
_cart_box_top_z_by_name = {}
_cart_box_size_by_name = {}
for _level_idx, _level in enumerate(_cart_box_levels):
    for name, size, parent_name, (dx, dy), mass_kg in _level:
        if parent_name is None:
            parent_x, parent_y = cart_center_xy[0], cart_center_xy[1]
            spawn_z = CART_BASKET_FLOOR_Z + CART_BOX_DROP_HEIGHT_ABOVE_FLOOR
        else:
            parent_x, parent_y = _cart_box_xy_by_name[parent_name]
            spawn_z = (
                _cart_box_top_z_by_name[parent_name] + size[2] / 2.0 + _CART_STACK_TOP_SPAWN_MARGIN_M
            )
        abs_x, abs_y = parent_x + dx, parent_y + dy
        _cart_box_size_by_name[name] = size
        DynamicCuboid(
            prim_path=f"/World/Box_{name}",
            name=name.lower(),
            position=np.array([abs_x, abs_y, spawn_z]),
            scale=np.array(size),
            color=np.array([0.85, 0.55, 0.15]),
            mass=mass_kg,
            physics_material=box_material,
        )
    # 이 레벨만 실제 물리로 정착시킨 뒤, 다음 레벨(자식)이 쓸 "실제" 위치를 읽는다 -
    # 부모가 착지 중 밀렸어도 자식은 그 실제 위치 위에 스폰되므로 안전하다.
    step_hold(_STABILIZE_STEPS)
    for name, size, *_r in _level:
        prim = stage.GetPrimAtPath(f"/World/Box_{name}")
        actual = np.array(prim.GetAttribute("xformOp:translate").Get())
        _cart_box_xy_by_name[name] = (float(actual[0]), float(actual[1]))
        _cart_box_top_z_by_name[name] = float(actual[2]) + size[2] / 2.0
    print(
        f"[박스 배치] {_level_idx}단계({[t[0] for t in _level]}) 정착 완료: "
        + ", ".join(f"{n}=({_cart_box_xy_by_name[n][0]:.3f},{_cart_box_xy_by_name[n][1]:.3f})" for n, *_r in _level),
        flush=True,
    )
print(f"\n[안정화 완료] (레벨당 {_STABILIZE_STEPS}스텝)\n", flush=True)

if os.environ.get("CART2TRUNK_GT_DEBUG", "0") == "1":
    print("[GT_DEBUG early] 초기 낙하/정착 직후 (스캔 전, world frame) ===", flush=True)
    for _gte_name, _gte_size, _gte_parent, _gte_off, _gte_mass in CART_BOX_TOPOLOGY:
        _gte_prim = stage.GetPrimAtPath(f"/World/Box_{_gte_name}")
        _gte_world = np.array(_gte_prim.GetAttribute("xformOp:translate").Get())
        print(f"[GT_DEBUG early] {_gte_name}: world={_gte_world.tolist()} parent={_gte_parent}", flush=True)

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

# ================= 1. 옴니휠 평행이동(strafe)으로 카트 옆면 접근 =================
target_xy = (cart_center_xy[0] + STANDOFF_X, cart_center_xy[1])
drive_to(target_x=target_xy[0], target_y=target_xy[1], label="카트 옆면 접근(strafe)")
snapshot(
    eye=[target_xy[0] - 1.0, target_xy[1] - 1.3, 1.4],
    target=[cart_center_xy[0], cart_center_xy[1], 0.5],
    fname="_cartscan_01_approached.png",
)

# ================= 2. 다중 시점 스캔 (베이스 strafe로 시점 다양화) =================
# 35.crate_scan_setup.py의 테이블 스캔은 "섀시 고정 + 팔 azimuth 스윙"으로 시점을
# 늘렸다(테이블이 넓고 평평해서 팔이 도달 범위 안에서 넓게 돌아볼 여지가 있었음).
# 카트 바스켓은 다르다: 로봇이 도킹한 지점에서 긴 축(Y, ~0.9m)이 멀리 뻗어있는
# 좁고 긴 형태이고, 도킹 거리 자체가 빠듯하다(STANDOFF_MARGIN=0.10m) - 팔만 크게
# 스윙하면 카트 벽/철망에 부딪힐 위험이 있다. 대신 이 홀로노믹 베이스는 옴니휠로
# 회전 없이 옆으로(strafe) 미끄러질 수 있다(88.py 파일 설계 의도, 사용자 확인) -
# 팔 대신 섀시 자체를 카트의 긴 축(Y)을 따라 여러 위치로 옮기고, 각 위치에서는
# 고정된 tilt로 아래를 보는 방식으로 시점을 다양화한다.
#
# 섀시가 매 시점 실제로 이동하므로(35.py는 섀시가 고정이라 base_pos/R_base를
# 한 번만 재고 모든 시점에 그대로 썼음), 각 시점의 world 좌표 point cloud를
# "그 시점의" base_link 기준으로 바로 변환하면 시점마다 원점이 달라져서 어긋난다.
# 그래서 스윕 도중에는 world 좌표 그대로 누적해두고, 스윕이 끝나고 베이스가
# 중앙(기준 위치)으로 돌아온 뒤 base_link를 딱 한 번만 측정해서 전체 누적
# point cloud를 그 기준 프레임으로 한 번에 변환한다.
CART_SCAN_STRAFE_Y_OFFSETS = [-0.28, -0.14, 0.0, 0.14, 0.28]
# 실측 확인(5개 적층 시나리오, 사용자 지적) - 1차 시도: tilt(위에서 내려다보는
# 각도)만 30도->14도로 바꾼 시점을 추가해봤지만, 개선 폭이 기대만큼 크지 않았다.
# 이유: eye의 수평 오프셋이 항상 순수 +X 방향 하나뿐이라(look_at과 Y는 항상 같음),
# tilt를 아무리 바꿔도 "카메라가 서 있는 좌우 방향(방위각, azimuth)"은 한 번도
# 안 바뀌었던 것 - M1/M2는 서로 world X 방향으로 나란히 배치돼 있는데, strafe는
# Y만 옮기므로 M1-M2를 좌우로 갈라보는 각도 자체가 항상 동일했다(같은 축의 시점
# 반복, 사용자 지적).
# 2차 설계: eye 오프셋에 azimuth(방위각) 성분을 추가한다 - 기존 "eye_x = center+
# offset, eye_y = strafe_y"(azimuth=0, 순수 X옵셋)에 더해, eye_y를 strafe_y에서
# 벌려서(offset*sin(azimuth)) 대각선 방향에서 보는 시점을 만든다. azimuth=0일 때는
# 기존 공식과 완전히 같아서 검증된 baseline(정면 5곳, tilt=30도)은 그대로 유지하고,
# M1/M2/XS가 있는 Y대(0, +0.14 부근)에서만 좌우 대각선(±20도) 시점을 추가한다 -
# 카메라 위치만 회전시키는 것이라(reach 거리는 baseline과 동일) IK 문제 재발 위험도
# tilt 변경보다 낮다.
# 실측 확인(3차, 사용자 지적: "가까운 쪽 Medium/Small이 붕 떠있다"): M1은 대각선
# 시점 덕에 XS1 가장자리 너머로 설계한 노출 띠(5cm)보다 훨씬 넓게(관측: 14cm까지)
# "돌아서 보이는" 효과를 얻는데, M2는 여전히 설계한 5cm 띠만큼만 잡혔다 - 원본
# point cloud 자체는 M2 쪽이 오히려 더 촘촘하고 깨끗했으므로(직접 측정 확인)
# 데이터 부족이 아니라 시야각 다양성 부족이 원인이다. M2(dx=+0.105)가 M1(dx=-0.09)
# 보다 카메라가 접근하는 +X 방향에 더 가까워서, 같은 물리적 eye 이동량이라도 M2
# 기준으로는 상대적으로 더 작은 각도 변화(패럴랙스)만 만든다 - 그래서 같은 ±20도로는
# XS2 가장자리 너머를 "돌아서 보는" 효과가 M1만큼 안 났다. M2가 있는 각도 폭을
# 더 넓혀서(±20도->±20/35도) 부족한 패럴랙스를 보충한다.
CART_SCAN_AZIMUTH_DIAGONAL_Y_OFFSETS = [0.0, 0.14]
CART_SCAN_AZIMUTH_DIAGONAL_DEG = [-35.0, -20.0, 20.0, 35.0]
CART_SCAN_VIEWPOINTS = (
    [(SCAN_TILT_FROM_VERTICAL_DEG, y_offset, 0.0) for y_offset in CART_SCAN_STRAFE_Y_OFFSETS]
    + [
        (SCAN_TILT_FROM_VERTICAL_DEG, y_offset, az_deg)
        for y_offset in CART_SCAN_AZIMUTH_DIAGONAL_Y_OFFSETS
        for az_deg in CART_SCAN_AZIMUTH_DIAGONAL_DEG
    ]
)
CART_SCAN_ROI_MAX_HEIGHT_M = 0.40  # CART_BASKET_FLOOR_Z 위로 이만큼까지만(카트 손잡이/배경 배제)
# 적층 시나리오 실측 확인(중요 버그): 기존 XY 크롭은 cart_min/max(카트 바깥쪽 bbox,
# 철망 벽/테두리까지 포함)에 마진을 "바깥쪽으로" 더한 범위라 카트
# 벽/테두리 자체가 통째로 point cloud에 들어왔다. 이 벽 평면(포인트 수천~9천개, 거의
# 수직이라 up_alignment는 낮지만)이 RANSAC segment_plane()의 앞쪽 반복(가장 인라이어가
# 많은 평면부터 순서대로 제거)을 먼저 차지해버려서, Medium/Small처럼 노출 면적이 작고
# 옆에서(strafe 스캔) 봐서 옆면과 살짝 섞인 박스 윗면은 검출 시도마다 정상적으로
# 분리되지 못했다(같은 물리적 위치에서도 시도마다 다른 조각이 나와 그룹핑 3회 미만으로
# 노이즈 처리됨 - 실측: 12회 중 1~2회만 관측). 카트 중심 기준 실제 박스 적재 영역
# (Large 최대 반폭 0.15m, Medium/Small 최대 오프셋+반폭 0.155m)보다 넉넉하되 카트
# 벽(cart_half_x=0.300, cart_half_y=0.448)보다는 확실히 안쪽인 반경으로 크롭하면
# 벽이 아예 안 들어와서 문제가 사라진다(오프라인 재현으로 확인: 5회 연속 시도 중
# 4~5회 Medium/Small 모두 fill_ratio 0.95+ 로 안정적으로 검출).
# 5개 적층(2단 피라미드)로 늘리면서 Base 자체가 커졌다(반폭 0.19m, 반깊이 0.16m) -
# 카트 벽(0.300/0.448)보다는 여전히 확실히 안쪽이면서, Base 가장자리에 여유(margin
# 0.03~0.06m)를 더 주기 위해 0.22->0.24로 소폭 확장.
# 실측 확인(CART2TRUNK_MARGIN_GROWTH_M 박스 크기 탐색): ROI를 고정값(0.24)으로
# 두면 Base가 그보다 커지는 순간(약 growth=0.02부터) ROI 크롭이 Base 가장자리를
# 잘라내서 검출 오차가 커진다 - Base 실제 반폭/반깊이(_base_half_x/_base_half_y)에
# 맞춰 자동으로 따라가게 한다.
CART_SCAN_ROI_HALF_X_M = _base_half_x + 0.03
CART_SCAN_ROI_HALF_Y_M = _base_half_y + 0.03

OPTICAL_TO_USD_CAMERA_AXES = np.diag([1.0, -1.0, -1.0])

accumulated_world_points = []

for i, (tilt_deg, y_offset, azimuth_deg) in enumerate(CART_SCAN_VIEWPOINTS):
    strafe_y = cart_center_xy[1] + y_offset
    drive_to(
        target_x=target_xy[0], target_y=strafe_y,
        label=f"스캔 위치 {i}(tilt={tilt_deg:.0f}deg,y_offset={y_offset:+.2f},az={azimuth_deg:+.0f}deg)",
    )

    # [설계 변경 - 사용자 지적] 원래는 매 시점마다 관절을 초기 자세로 리셋(보간
    # 이동)한 뒤 처음부터 다시 350스텝 수렴시켰다(IK 오차가 시점을 거칠수록
    # 누적되는 문제를 막기 위한 조치였음). 그런데 이 방식은 "카메라를 원상태로
    # 되돌렸다가 다시 스캔 자세로 이동"하는 불필요한 왕복 동작으로 보여서
    # 부자연스럽다는 지적을 받았다.
    #
    # 같은 (tilt, azimuth) 조합 안에서는 목표(target_pos/target_quat)가 "베이스
    # 기준 상대 자세"로 보면 거의 동일하다 - look_at이 strafe_y를 그대로 따라가는
    # 순수 평행이동 관계라, 팔의 물리적 도달 거리 문제(리프트 높이/EYE_HEIGHT
    # 조정으로 이미 해결, 3mm 수렴)만 없었다면 애초에 관절이 시점마다 크게 바뀔
    # 이유가 없었다. 그래서 팔을 리셋하지 않고 이전 시점에서 수렴된 자세를 그대로
    # 이어받는다 - 베이스가 strafe로 이동하는 동안 팔은 가만히 있다가, 도착 후
    # 아주 짧게만(이미 거의 맞는 자세이므로) 미세 조정한다. 다만 tilt/azimuth
    # 자체가 바뀌는 시점(첫 시점 포함)은 목표 자세가 실제로 크게 달라지므로
    # 처음부터 다시 충분한 스텝을 준다.
    horizontal_offset_i = EYE_HEIGHT_ABOVE_CART * np.tan(np.radians(tilt_deg))
    azimuth_rad_i = np.radians(azimuth_deg)
    scan_eye_i = np.array([
        cart_center_xy[0] + horizontal_offset_i * np.cos(azimuth_rad_i),
        strafe_y + horizontal_offset_i * np.sin(azimuth_rad_i),
        CART_BASKET_FLOOR_Z + EYE_HEIGHT_ABOVE_CART,
    ])
    scan_look_at_i = np.array([cart_center_xy[0], strafe_y, CART_BASKET_FLOOR_Z])
    target_pos, target_quat = lookat_to_link6_target(scan_eye_i, scan_look_at_i)
    is_new_pose_family = (i == 0) or (
        (tilt_deg, azimuth_deg) != CART_SCAN_VIEWPOINTS[i - 1][0::2]
    )
    move_steps = 350 if is_new_pose_family else 90
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
        (pts_world_i[:, 0] >= cart_center_xy[0] - CART_SCAN_ROI_HALF_X_M)
        & (pts_world_i[:, 0] <= cart_center_xy[0] + CART_SCAN_ROI_HALF_X_M)
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
    print(f"[카트 스캔 {i}] tilt={tilt_deg:.0f}deg y_offset={y_offset:+.2f} az={azimuth_deg:+.0f}deg "
          f"world_points={len(pts_world_i)}", flush=True)

# ================= 3. 기준 위치(중앙)로 복귀 + base_link 기준 변환/저장 =================
drive_to(target_x=target_xy[0], target_y=cart_center_xy[1], label="스캔 기준 위치(중앙) 복귀")
snapshot(
    eye=[target_xy[0] - 0.8, target_xy[1] - 1.0, cart_center_xy[1] + 1.5],
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
print(f"[카트 스캔] {len(CART_SCAN_VIEWPOINTS)}개 시점 누적, 총 {len(merged_base_points)}포인트 "
      f"-> {scan_cache_path}", flush=True)

# 박스 크기 탐색(CART2TRUNK_MARGIN_GROWTH_M 반복 실험)용 실측 정답 - 검출 결과와
# 직접 대조하기 위해 각 박스의 실제 base_link 좌표를 남긴다. 끄면(기본) 기존 동작과
# 완전히 동일 - 이후 단계(ROS2 브리지 등)까지 그대로 이어진다.
if os.environ.get("CART2TRUNK_GT_DEBUG", "0") == "1":
    print("\n[GT_DEBUG] 각 박스 실측 위치 (base_link 프레임 변환) ===", flush=True)
    for _gt_name, _gt_size, _gt_parent, _gt_off, _gt_mass in CART_BOX_TOPOLOGY:
        _gt_prim = stage.GetPrimAtPath(f"/World/Box_{_gt_name}")
        _gt_world = np.array(_gt_prim.GetAttribute("xformOp:translate").Get())
        _gt_base = R_base.T @ (_gt_world - base_pos_final)
        print(f"[GT_DEBUG] {_gt_name}: base_frame_center={_gt_base.tolist()} "
              f"top_z={_gt_base[2] + _gt_size[2] / 2.0:.4f} size={_gt_size} parent={_gt_parent}", flush=True)
    simulation_app.close()
    raise SystemExit(0)

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
