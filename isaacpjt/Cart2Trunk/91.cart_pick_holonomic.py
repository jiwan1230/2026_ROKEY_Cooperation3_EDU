"""
91.cart_pick_holonomic.py

Cart2Trunk 최종 시나리오 3단계 - 카트 PICK (RMPflow 벽 회피).
계획 파일(~/.claude/plans/parallel-juggling-sun.md) 90번 항목("카트 PICK 벽 회피") 참고 -
번호는 90이 이미 트렁크 맵 포팅에 쓰여서 91로 이어감.

88.cart_scan_holonomic.py와 동일한 카트+박스2개 씬을 다시 구성하고(같은 스펙이라 물리
위치가 동일), algorism이 계산한 placement_result.json의 순서대로 두 박스를 흡착 PICK한다.

카트 옆벽 회피 설계 결정 (3차 시도 끝에 확정)
----
1차: RMPflow world obstacle로 벽 등록 -> 벽이 손잡이 높이까지 덮여서 낮은 호버 목표가
     아예 도달 불가(근처도 못 감).
2차: 28.py 패턴 그대로(rim 위 호버 -> 수직 하강) -> 먼 standoff에서 큰 수평 reach를
     쓰다 보니 팔이 벽과 충돌.
3차: joint_1 조준 + joint_3/5=90도 접기 + 리프트만 하강(순수 수직) -> 사용자 직접 검증:
     joint_1(방위각)/리프트(높이)만으로는 반경(radial) 오차를 못 고쳐서 폐기
     ("1,6번 조인트와 그리퍼 사이에 리프트처럼 길어지는 게 없는 이상 불가능").
**최종(4차, 이 파일 구현)**: 리프트를 더 높이 올려(LIFT_TRAVEL_M 확대) 그리퍼를
cart_max[2](손잡이 높이) 위에서 호버하게 하고, "위에서 아래로 손을 뻗어 잡는" 순수 RMPflow
Cartesian IK로 되돌아간다. 대신 2차가 실패했던 "먼 standoff + 큰 수평 reach" 조합을
없애기 위해, 접근을 2단계로 나눈다:
  (a) 먼 STANDOFF_X에서 팔을 자기 몸 위(카트 쪽 아님)로 세워 안전한 "기본 자세"부터 만들고,
  (b) 그 자세를 유지한 채(팔 명령 없이 섀시만 이동) PICK_STANDOFF_X까지 붙인 뒤,
  (c) 그제서야 짧아진 수평 거리로 박스 위 호버 -> 수직 하강 -> 파지 -> 수직 후퇴를 반복한다.
(a)(b) 구간은 팔 관절이 카트 쪽으로 전혀 안 움직이므로 벽과 만날 경로가 없고, (c)는 항상
cart_max[2] 위 높이에서만 수평 이동하므로 벽/손잡이보다 항상 높다.

박스 매칭
----
algorism 결과의 box_id는 비전(box_top_extractor.py)이 매긴 번호라 물리 프림 이름
(Box_A/Box_B)과 직접 대응이 없다. 36.py와 동일한 "스캔 위치와 가장 가까운 물리 프림"
매칭 방식을 쓰되, 스캔 당시 base_link 자세는 88.py가 저장해둔
perception/base_to_camera_transform.json의 measured_base_pos/measured_base_quat를
그대로 재사용한다(이 스크립트는 88.py와 별개 프로세스라 실시간 base_link 자세를 알 방법이
없음 - 크로스 세션 좌표 재투영, 이전에도 여러 번 쓴 패턴).

이 스크립트가 다루지 않는 것
----
- 트렁크 PLACE(92번, 저상 베이스 차량 하부 진입 - 훨씬 위험한 신규 과제라 분리).
- 전체 통합 루프(재스캔/재계획 포함, 93번 예정).
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
import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, UsdLux, UsdShade, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.rotations import quat_to_euler_angles, euler_angles_to_quat
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

_THIS_DIR = Path(__file__).resolve().parent
OUT_DIR = _THIS_DIR / "results" / "holonomic_base"
OUT_DIR.mkdir(parents=True, exist_ok=True)
PERCEPTION_DIR = _THIS_DIR / "perception"

M0609_DIR = _THIS_DIR.parent / "M0609"
RMPFLOW_DIR = str(M0609_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController  # noqa: E402

# ---------------- 88.py와 완전히 동일한 카트/박스 구성 (같은 위치 재현) ----------------
CART_USD = str(_THIS_DIR / "assets/Metal_Shopping_Cart.usdz")
CART_POS = (0.0, 0.0, 0.0)
CART_EXTRA_SCALE = 0.55
SDF_RESOLUTION = 256
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 0.0, 50.0, 20.0

BASE_PATH = "/World/HoloBase"
CHASSIS_PATH = f"{BASE_PATH}/chassis"
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
# 트렁크 PLACE(92번)와 달리 카트 PICK은 차량 하부를 통과할 필요가 없어 낮은 차체 제약이 없다.
# 카트 손잡이 높이(cart_max[2], 실측 약 1.03m)보다 그리퍼가 위에 오려면 82~87번에서 쓰던
# 0.35~0.45 travel로는 부족해서(사용자 지적: "리프트 더 올려서") 트렁크와 무관하게 이 스크립트만
# travel을 키운다.
LIFT_TRAVEL_M = 0.75

EE_LINK_NAME = "link_6"
GRIPPER_BODY_NAME = "vgp20_suction_plate"

GRIPPER_RANGE_JSON = M0609_DIR / "Collected_m0609_vgp20_camera" / "_gripper_physical_range.json"
if GRIPPER_RANGE_JSON.exists():
    _range = json.loads(GRIPPER_RANGE_JSON.read_text())
    TIP_LOCAL_OFFSET = tuple(_range["tip_local_offset"])
else:
    TIP_LOCAL_OFFSET = (0.0, 0.0, 0.0188)

# 실측(run10) - 크립 하강 중 도달한 최소 dist=0.0882m가 grasp_radius=0.085m를 3.2mm 차이로
# 놓쳤다(그 근처에서 박스가 살짝 흔들리며 dist가 다시 벌어짐). 0.03 -> 0.05로 살짝 넓혀
# 이 여유분과 호버 때 남는 XY 오차(수 mm~수 cm)를 함께 흡수한다.
GRASP_RADIUS_MARGIN = 0.05
GRASP_STANDOFF = 0.01
# STANDOFF_MARGIN: 씬 시작/스캔 때 쓰는 "안전한 먼 거리"(84/87번과 동일 값 유지).
STANDOFF_MARGIN = 0.15
# PICK_STANDOFF_MARGIN: 사용자 지적("카트와 holonomic base를 좀 더 붙여야한다") - PICK 직전에만
# 이만큼 더 붙는다. 붙는 동안은 팔을 자기 몸 위(카트 쪽으로 뻗지 않은 상태)로 접어 두고 섀시만
# 평행이동하므로(88~90번과 동일한 "리프트 텔레포트로 팔이 섀시에 용접된 것처럼 붙어있다" 특성 활용),
# 팔이 옆으로 휘두르며 벽에 부딪힐 경로 자체가 없다.
# 실측(0.04로 실행) - 호버까지는 문제없었지만 하강 중 정체 스크린샷(_cartpick_stall_...)에서
# 팔꿈치가 마운트 바로 옆 카트 벽/모서리 기둥에 그대로 꽂혀 있는 게 확인됐다 - 0.04는 팔꿈치가
# 그 모서리를 피해 내려갈 여유 공간 자체가 없었다. 0.10으로 늘려 여유를 준다.
PICK_STANDOFF_MARGIN = 0.18
# HOVER_CLEARANCE_ABOVE_RIM: "그리퍼가 카트 외벽보다 높은 상태"를 만들 때 cart_max(카트 손잡이
# 높이 포함 전체 bbox 상단) 위로 얼마나 더 띄울지.
HOVER_CLEARANCE_ABOVE_RIM = 0.15
DOWN_QUAT = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))

CART_BASKET_FLOOR_Z = 0.68

PLACEMENT_JSON = OUT_DIR / "placement_result.json"
BASE_TO_CAMERA_TRANSFORM_JSON = PERCEPTION_DIR / "base_to_camera_transform.json"


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


def get_world_pos(prim):
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array(mat.ExtractTranslation())


def world_aabb_from_base_corners(corners_base, base_pos, R_base):
    pts_base = np.asarray(corners_base, dtype=np.float64)
    pts_world = pts_base @ R_base.T + base_pos
    mn = pts_world.min(axis=0)
    mx = pts_world.max(axis=0)
    return (mn + mx) / 2.0, mx - mn, mn


def discover_box_prim_paths(stage):
    world_prim = stage.GetPrimAtPath("/World")
    return [str(c.GetPath()) for c in world_prim.GetChildren() if c.GetName().startswith("Box_")]


def match_physical_prim(stage, scan_center_world_xy, available_paths):
    best_path, best_dist = None, None
    for path in available_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        pos = get_world_pos(prim)
        dist = float(np.linalg.norm(pos[:2] - np.asarray(scan_center_world_xy)))
        if best_dist is None or dist < best_dist:
            best_path, best_dist = path, dist
    return best_path, best_dist


class DynamicSuctionGripper(SurfaceGripper):
    """84/87/88번과 동일한 흡착 로직 - set_target()으로 박스마다 대상을 바꿀 수 있다."""

    def __init__(self, end_effector_prim_path, gripper_body_path, tip_local_offset=(0.0, 0.0, 0.0)):
        SurfaceGripper.__init__(self, end_effector_prim_path=end_effector_prim_path, surface_gripper_path="")
        self._gripper_body_path = gripper_body_path
        self._tip_local_offset = Gf.Vec3d(*tip_local_offset)
        self._joint_path = f"{gripper_body_path}/suction_attach_joint"
        self._attached = False
        self._target_prim_path = None
        self._grasp_radius = 0.0

    def set_target(self, target_prim_path, grasp_radius):
        self._target_prim_path = target_prim_path
        self._grasp_radius = grasp_radius

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
        dist = (tip_world - target_pos).GetLength()
        if dist > self._grasp_radius:
            print(f"  [흡착 실패] dist={dist:.4f}m > grasp_radius={self._grasp_radius}m", flush=True)
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
        self._attached = False

    def is_closed(self) -> bool:
        return self._attached

    def is_open(self) -> bool:
        return not self._attached


# ================= 씬 구성 (88.py와 동일 - 카트 + 박스 2개) =================
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
print(f"[카트 bbox] min={cart_min} max={cart_max} center_xy={cart_center_xy}", flush=True)

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
        prim_path=f"/World/{name}", name=name.lower(),
        position=np.array([cart_center_xy[0] + dx, cart_center_xy[1] + dy,
                            CART_BASKET_FLOOR_Z + CART_BOX_DROP_HEIGHT_ABOVE_FLOOR]),
        scale=np.array(size), color=np.array([0.85, 0.55, 0.15]), mass=0.3,
        physics_material=box_material,
    )
print(f"[박스 배치] 카트 안에 {len(CART_BOX_SPECS)}개 낙하 예정", flush=True)
# prim_path -> (sx, sy, sz) 스폰 시점의 진짜 크기. bbox_of()로 매 PICK 시점마다 다시 재는
# world AABB는 박스가 기울어지면(아래 실측 버그 참고) 신뢰할 수 없어 대신 이 값을 쓴다.
BOX_KNOWN_SIZE = {f"/World/{name}": size for name, size, _ in CART_BOX_SPECS}

STANDOFF_X = CHASSIS_HALF_WIDTH_EFFECTIVE + cart_half_x + STANDOFF_MARGIN
PICK_STANDOFF_X = CHASSIS_HALF_WIDTH_EFFECTIVE + cart_half_x + PICK_STANDOFF_MARGIN
BASE_START_XY = (cart_center_xy[0] + STANDOFF_X, cart_center_xy[1])
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


def move_lift_to(target_h, steps=90, hold_gripper_closed=False):
    start_h = lift_state["h"]
    for i in range(steps):
        h = start_h + (target_h - start_h) * (i + 1) / steps
        set_lift_height(h)
        if hold_gripper_closed:
            m0609_robot.gripper.close()
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


step_hold(60)
print("\n[안정화 완료]\n", flush=True)

# ================= 리프트를 최고 높이로 =================
print(f"\n[리프트] 도킹({LIFT_MIN:.3f}) -> 최고({LIFT_MAX:.3f})", flush=True)
move_lift_to(LIFT_MAX, steps=120)

# ================= 조인트 3/5=90/90 접은 자세로 재확립 (특이점 회피 시드) =================
# 사용자 지적 - Phase A/B 및 박스별 루프에서 RMPflow가 joint_1(베이스 회전)을 돌려가며 IK를
# 풀기 전에, 팔꿈치/손목을 90도씩 접어둔 자세를 시드로 줘두면 완전히 펴진 자세 근처에서
# 생기는 특이점을 피하기 쉬워진다.
# 주의(실측 버그) - 처음엔 "현재 조인트값을 복사해 3/5번만 덮어쓰는" 방식으로 짰다가 자기충돌이
# 났다: 리프트를 올리고 먼 standoff로 이동하는 동안(step_hold/move_lift_to/drive_to는 리프트
# 텔레포트만 하고 팔 관절에는 아무 명령도 안 보낸다) joint_1/2/4/6이 0에서 미세하게 흘러가
# 있었고, 그 값들을 그대로 들고 온 채 3/5만 90도로 만들면 29.carry_pose_calibration.py가
# 스크린샷으로 직접 비교했던 두 후보(seed_pose_v1=[0,0,90,0,90,0] 자기충돌 없음 vs
# v2_more_elbow=[0,0,150,0,90,0] 훨씬 위험) 중 어느 쪽도 아닌 "제3의 조합"이 돼버려서 팔이
# 스스로에게 부딪혔다. 고정: 나머지 조인트를 "현재값 유지"가 아니라 검증된 seed_pose_v1과
# 똑같이 명시적으로 전부 0으로 못박는다(joint_1도 포함 - 이 구간은 RMPflow 아니라 직접
# 조인트 제어라 joint_1=0 고정도 카트 쪽으로 휘두를 경로를 만들지 않는다).
_fold_current = np.array(m0609_robot.get_joint_positions(), dtype=float)
_fold_target = np.zeros(m0609_robot.num_dof)
if "joint_3" in m0609_robot.dof_names:
    _fold_target[m0609_robot.dof_names.index("joint_3")] = np.pi / 2
if "joint_5" in m0609_robot.dof_names:
    _fold_target[m0609_robot.dof_names.index("joint_5")] = np.pi / 2
FOLD_STEPS = 150
for i in range(FOLD_STEPS):
    alpha = (i + 1) / FOLD_STEPS
    j = _fold_current + (_fold_target - _fold_current) * alpha
    m0609_robot.apply_action(ArticulationAction(joint_positions=j))
    set_lift_height(lift_state["h"])
    world.step(render=True)
step_hold(20)
print(f"[특이점 회피 시드] joint_3/5=90/90(나머지 전부 0) 재확립 완료: "
      f"{np.round(m0609_robot.get_joint_positions(), 3)}", flush=True)

# ================= 옴니휠 평행이동으로 카트 옆면 접근 (88번과 동일) =================
target_xy = (cart_center_xy[0] + STANDOFF_X, cart_center_xy[1])
drive_to(target_x=target_xy[0], target_y=target_xy[1], label="카트 옆면 접근")

# ================= RMPflow 컨트롤러 =================
# 파일 상단 docstring 참고 - 4차 설계(리프트 확장 + 2단계 접근 + top-down 호버/하강)로 확정.
controller = RMPFlowController(
    name="cart_pick_holonomic", robot_articulation=m0609_robot,
    urdf_path=M0609_URDF_PATH, robot_description_path=M0609_DESCRIPTION_PATH,
    rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH, end_effector_frame_name=EE_LINK_NAME,
)


def sync_rmp_base():
    chassis_pos, chassis_quat = base_robot.get_world_pose()
    base_pos = np.array([float(chassis_pos[0]), float(chassis_pos[1]), float(chassis_pos[2]) + lift_state["h"]])
    controller._default_position = base_pos
    controller._default_orientation = chassis_quat
    controller.rmp_flow.set_robot_base_pose(robot_position=base_pos, robot_orientation=chassis_quat)


def measure_tip_world_pos():
    # DynamicSuctionGripper.close()와 완전히 동일한 방식(gripper_body_path의 실제 world
    # transform + tip_local_offset)으로 흡착 팁의 진짜 world 위치를 구한다. link_6과
    # vgp20_suction_plate 사이엔 TIP_LOCAL_OFFSET(약 2cm)보다 훨씬 큰 고정 마운트 오프셋이
    # 있어서(실측 13cm대), link_6 위치만 보고 "팁이 여기 있겠지"라고 가정하면 안 된다.
    gripper_mat = UsdGeom.Xformable(stage.GetPrimAtPath(gripper_body_path)).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    return np.array(gripper_mat.Transform(Gf.Vec3d(*TIP_LOCAL_OFFSET)), dtype=float)


def move_link6_smooth(target_tip_pos, tolerance=0.004, max_speed=0.01, kp=3.0, max_steps=2500,
                       hold_gripper_closed=False, label="", orientation=DOWN_QUAT):
    # 사용자 지적(1차) - 고정 목표를 N스텝 그대로 넣는 방식은 "임계값을 갓 벗어난 잔여 오차에도
    # 고정 스텝 크기를 그대로 명령"해서 마지막 접근에서 오버슈트가 났다. 고쳐야 할 건 "잔여
    # 오차에 비례해서 속도가 줄어드는" P제어다 - drive_to()가 섀시에 쓰는 것과 같은 방식
    # (kp*오차를 max_speed로 클리핑)을 link_6에도 적용한다.
    #
    # 사용자 지적(2차, 이번 수정) - "박스 위치보다 더 내려가서 충돌난다"는 재현으로 드러난
    # 진짜 버그: 이전 버전은 호버 시점에 딱 한 번 tip_to_link6 오프셋을 측정해서 하강 내내
    # "고정값"처럼 썼다. 이 오프셋은 link_6의 그 순간 방향(orientation)에 종속된 값이라,
    # 하강하면서 RMPflow가 방향을 DOWN_QUAT에 완벽히 붙잡아두지 못하고 조금이라도 틀어지면
    # (박스에 닿을 만큼 팔을 크게 접어야 하는 자세일수록 이 틀어짐이 커진다) 그 고정 오프셋
    # 자체가 더 이상 안 맞아서, link_6은 "제 위치"에 왔다고 판단해도 실제 팁은 그보다 더
    # 내려가 있거나 박스를 뚫고 지나가는 식으로 어긋났다 - 매 스텝 방향이 바뀌는데 오프셋만
    # 고정해서 생기는 구조적 오차였다.
    # 고침: 오프셋을 미리 계산해서 목표에 빼주는 방식을 버리고, 매 스텝 measure_tip_world_pos()로
    # "지금 실제 팁이 어디 있는지"를 직접 재보고 그 오차를 기준으로 제어한다(폐루프 피드백 -
    # link_6이 아니라 팁이 제어 대상). 한 스텝 동안은 방향이 거의 안 바뀌므로 "링크6을 오차
    # 방향으로 조금 옮기면 팁도 같은 방향으로 그만큼 움직인다"는 근사가 성립하고, 이게 매
    # 스텝 실측으로 갱신되니 방향이 서서히 틀어져도 결코 고정 오차로 쌓이지 않는다.
    STALL_WINDOW, STALL_MIN_IMPROVEMENT = 200, 0.003
    target_tip_pos = np.array(target_tip_pos, dtype=float)
    step = 0
    tip_pos = None
    stalled = False
    last_check_err = None
    for step in range(1, max_steps + 1):
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
                try:
                    safe_label = "".join(c if c.isalnum() else "_" for c in label) or "stall"
                    stall_out = str(OUT_DIR / f"_cartpick_stall_{safe_label}.png")
                    vp_util.capture_viewport_to_file(viewport, stall_out)
                    print(f"  [SCREENSHOT] {stall_out}", flush=True)
                except NameError:
                    pass
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
        if hold_gripper_closed:
            m0609_robot.gripper.close()
        set_lift_height(lift_state["h"])
        world.step(render=True)
    tip_pos = measure_tip_world_pos()
    err = float(np.linalg.norm(tip_pos - target_tip_pos))
    ee_pos, _ = m0609_robot.end_effector.get_world_pose()
    print(f"[완만한 접근{' ' + label if label else ''}] target_tip={np.round(target_tip_pos, 3)} "
          f"tip={np.round(tip_pos, 3)} link6={np.round(ee_pos, 3)} err={err:.4f}m steps={step} "
          f"stalled={stalled}", flush=True)
    return tip_pos, err


viewport = vp_util.get_active_viewport()


def snapshot(eye, target, fname):
    set_camera_view(eye=eye, target=target)
    step_hold(15)
    out = str(OUT_DIR / fname)
    vp_util.capture_viewport_to_file(viewport, out)
    step_hold(30)
    print(f"[SCREENSHOT] {out}", flush=True)


chassis_pos0, _ = base_robot.get_world_pose()
snapshot(eye=[chassis_pos0[0] - 1.0, chassis_pos0[1] - 1.3, 1.4],
         target=[cart_center_xy[0], cart_center_xy[1], 0.7], fname="_cartpick_00_start.png")

# ================= placement_result.json 순서대로 박스 매칭 (36.py 패턴 재사용) =================
CANDIDATE_BOX_PRIM_PATHS = discover_box_prim_paths(stage)
print(f"[박스 프림 탐색] {CANDIDATE_BOX_PRIM_PATHS}", flush=True)

placement_data = json.loads(PLACEMENT_JSON.read_text())
placements = placement_data["placements"]

transform_data = json.loads(BASE_TO_CAMERA_TRANSFORM_JSON.read_text())
SCAN_BASE_POS = np.asarray(transform_data["measured_base_pos"], dtype=np.float64)
SCAN_BASE_QUAT = np.asarray(transform_data["measured_base_quat"], dtype=np.float64)
SCAN_R_BASE = quat_wxyz_to_matrix(SCAN_BASE_QUAT)

# 최신 비전 스캔 결과(88.py가 쓴 것과 동일 파일 계열)에서 box_id별 corners_m을 읽어 매칭용
# world 좌표를 재투영한다.
_vision_dir = Path.home() / "box_pointcloud"
_vision_files = sorted(_vision_dir.glob("all_boxes_corners_*.json"))
if not _vision_files:
    raise SystemExit(f"[에러] {_vision_dir}에 all_boxes_corners_*.json이 없습니다.")
vision_data = json.loads(_vision_files[-1].read_text())
scan_by_box_id = {str(b["box_id"]): b for b in vision_data["boxes"]}
print(f"[비전 로드] {_vision_files[-1].name} - box_id={list(scan_by_box_id.keys())}", flush=True)

used_prim_paths = set()
pick_order = []  # [(prim_path, placement_dict)]
for placement in placements:
    box_id = str(placement["box_id"])
    scan_entry = scan_by_box_id.get(box_id)
    if scan_entry is None:
        print(f"[경고] box_id={box_id}가 비전 결과에 없음 - 건너뜀", flush=True)
        continue
    scan_center, _, _ = world_aabb_from_base_corners(scan_entry["corners_m"], SCAN_BASE_POS, SCAN_R_BASE)
    available = [p for p in CANDIDATE_BOX_PRIM_PATHS if p not in used_prim_paths]
    prim_path, match_dist = match_physical_prim(stage, scan_center[:2], available)
    if prim_path is None:
        continue
    used_prim_paths.add(prim_path)
    pick_order.append((prim_path, placement))
    print(f"[매칭] box_id={box_id} -> {prim_path} (거리={match_dist:.3f}m)", flush=True)

# ================= 검증: 스캔 기반 좌표 vs 실제 물리 박스 위치 =================
# 사용자 지적 - "지금 접근 좌표가 스캔 결과 기반인데 실제 스폰 위치랑 맞는지" 확인 필요.
# 실제 PICK 목표(아래 루프의 box_pos)는 get_world_pos(box_prim)로 "현재 물리 위치"를 그대로
# 읽어와 쓴다 - 스캔 좌표(scan_center)를 직접 이동 목표로 쓰지 않는다. scan_center는 오직
# 바로 위 매칭 단계("여러 물리 프림 중 어느 게 어느 vision box_id인가")에만 쓰인다.
# 하지만 88.py 스캔은 별도 세션이라(박스가 새로 떨어져 정착하는 위치가 이번 실행과 완전히
# 같다는 보장이 없다) 매칭 자체가 "스캔 좌표와 실제 위치가 충분히 가깝다"는 가정에 기대고
# 있다 - 이 가정을 수치(offset)와 시각(마젠타 마커 vs 실제 박스) 둘 다로 직접 확인한다.
verification = []
for prim_path, placement in pick_order:
    box_id = str(placement["box_id"])
    scan_entry = scan_by_box_id[box_id]
    scan_center, _, _ = world_aabb_from_base_corners(scan_entry["corners_m"], SCAN_BASE_POS, SCAN_R_BASE)
    actual_pos = get_world_pos(stage.GetPrimAtPath(prim_path))
    offset = np.array(actual_pos) - np.array(scan_center)
    offset_xy = float(np.linalg.norm(offset[:2]))
    offset_total = float(np.linalg.norm(offset))
    # 임계값 0.05m: 지금 씬의 더 작은 박스(Box_B, 0.13x0.10x0.09) 절반 폭 정도 - 이보다
    # 크게 어긋나면 매칭이 다른 박스로 잘못 붙었을 위험이 있다는 뜻.
    flag = "경고: 오차 큼(매칭 오류 의심)" if offset_xy > 0.05 else "정상"
    print(f"[검증] box_id={box_id} prim={prim_path}: 스캔추정={np.round(scan_center, 3)} "
          f"실제={np.round(actual_pos, 3)} offset_xy={offset_xy:.4f}m offset_total={offset_total:.4f}m "
          f"-> {flag}", flush=True)
    verification.append({
        "box_id": box_id, "prim_path": prim_path,
        "scan_center": scan_center.tolist(), "actual_pos": actual_pos.tolist(),
        "offset_xy_m": offset_xy, "offset_total_m": offset_total, "flag": flag,
    })

    marker_path = f"/World/DebugMarker_scan_{box_id}"
    marker = UsdGeom.Sphere.Define(stage, marker_path)
    marker.CreateRadiusAttr(0.02)
    marker.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.0, 1.0)])
    UsdGeom.Xformable(marker).AddTranslateOp().Set(
        Gf.Vec3d(float(scan_center[0]), float(scan_center[1]), float(scan_center[2])))

verify_path = OUT_DIR / "_cartpick_scan_vs_actual.json"
verify_path.write_text(json.dumps(verification, indent=2))
print(f"[검증 저장] {verify_path}", flush=True)

snapshot(eye=[chassis_pos0[0] - 1.0, chassis_pos0[1] - 1.3, 1.3],
         target=[cart_center_xy[0], cart_center_xy[1], 0.75], fname="_cartpick_00b_scan_vs_actual.png")

# ================= Phase A: 먼 standoff에서 "기본 자세" 만들기 =================
# 원래 여기서 move_link6()(RMPflow)로 "자기 몸 위, 안전 높이"를 목표 지점으로 IK를 풀게
# 했었는데, 그 목표 자체가 문제였다 - "자기 마운트 축 바로 위 지점을, 그리퍼가 정확히
# 아래를 보게" 만드는 건 팔 입장에서 자기 마운트 위로 손을 뻗어 자기 머리를 만지는 것과
# 같은 퇴화(degenerate) 자세라, RMPflow가 매번 팔을 자기 자신 쪽으로 구부려 넣는 (지금
# 보고된) 자기충돌 자세로 수렴해버렸다 - 시드를 아무리 깨끗하게 줘도 IK 목표 자체가
# 그 모양을 요구하니 소용없었다. 수정: Phase A에서는 RMPflow를 아예 쓰지 않는다. 검증된
# 접은 자세(joint_3/5=90/90, 나머지 0 - 29.carry_pose_calibration.py로 자기충돌 없음을
# 스크린샷으로 확인한 그 모양) 그대로 두고, 그 자세의 그리퍼가 안전 높이에 못 미치면
# 리프트만 더 올려서 높이를 맞춘다 - 팔 모양은 그대로이므로 자기충돌 경로 자체가 없다.
safe_hover_z = float(cart_max[2]) + HOVER_CLEARANCE_ABOVE_RIM
ee_folded_pos, _ = m0609_robot.end_effector.get_world_pose()
extra_lift = float(safe_hover_z) - float(ee_folded_pos[2])
print(f"\n[Phase A] 안전 높이={safe_hover_z:.3f}m (cart_max[2]={float(cart_max[2]):.3f}+"
      f"{HOVER_CLEARANCE_ABOVE_RIM:.2f}), 접은 자세 그리퍼 z={float(ee_folded_pos[2]):.3f}m "
      f"-> 리프트 추가 상승 필요={extra_lift:.3f}m", flush=True)
if extra_lift > 0.0:
    move_lift_to(lift_state["h"] + extra_lift, steps=150)
ee_folded_pos, _ = m0609_robot.end_effector.get_world_pose()
print(f"[Phase A 완료] 접은 자세 유지한 채 그리퍼 z={float(ee_folded_pos[2]):.3f}m "
      f"(목표 {safe_hover_z:.3f}m)", flush=True)
snapshot(eye=[chassis_pos0[0] - 1.0, chassis_pos0[1] - 1.3, safe_hover_z + 0.3],
         target=[chassis_pos0[0], chassis_pos0[1], safe_hover_z], fname="_cartpick_01_safe_pose.png")

# ================= Phase B: 팔은 그대로, 섀시만 카트 쪽으로 붙이기 =================
# drive_to()는 휠 관절만 명령하고 팔에는 아무 명령도 보내지 않는다 - 팔은 직전 자세(위 안전
# 높이)를 그대로 유지한 채(리프트 텔레포트로 섀시에 "용접"되어 있으므로) 섀시와 함께 통째로
# 평행이동한다. 수평 reach가 없는 상태로만 움직이므로 이 구간도 벽과 만날 경로가 없다.
pick_target_xy = (cart_center_xy[0] + PICK_STANDOFF_X, cart_center_xy[1])
print(f"[Phase B] standoff {STANDOFF_MARGIN:.2f}m -> {PICK_STANDOFF_MARGIN:.2f}m로 붙이기 "
      f"(팔은 안전 높이 유지, 명령 없음)", flush=True)
drive_to(target_x=pick_target_xy[0], target_y=pick_target_xy[1], label="카트에 더 붙이기(PICK 접근)")
chassis_pos1, _ = base_robot.get_world_pose()
snapshot(eye=[chassis_pos1[0] - 0.9, chassis_pos1[1] - 1.1, safe_hover_z + 0.3],
         target=[cart_center_xy[0], cart_center_xy[1], safe_hover_z], fname="_cartpick_02_close_approach.png")

# ================= 박스마다 PICK (top-down: 안전 높이에서 호버 -> 수직 하강 -> 파지 -> 수직 후퇴) =================
results = []
for idx, (prim_path, placement) in enumerate(pick_order):
    box_prim = stage.GetPrimAtPath(prim_path)
    box_pos = get_world_pos(box_prim)
    # 실측 버그 - bbox_of()로 이 시점에 world AABB를 다시 재면(예전 코드) 박스가 조금이라도
    # 기울어져 있을 때 터무니없이 작은 높이가 나왔다(Box_A 실측 half_height=0.0062 - 진짜
    # 치수 0.11m짜리 박스의 world Z 폭이 1.2cm일 수는 없다. USD 쿼리가 물리 시뮬레이션이
    # 갱신한 실제 자세를 못 따라간 것으로 보인다). 대신 스폰 시점에 확정된 진짜 크기
    # (BOX_KNOWN_SIZE)를 그대로 쓴다 - 박스가 기울어져도 이 값은 변하지 않는다.
    half_height = float(BOX_KNOWN_SIZE[prim_path][2]) / 2.0
    box_top_z = float(box_pos[2]) + half_height
    grasp_radius = half_height + GRASP_RADIUS_MARGIN
    gripper.set_target(prim_path, grasp_radius)

    print(f"\n===== [{idx + 1}/{len(pick_order)}] {prim_path} PICK 시작 (world pos={np.round(box_pos, 3)}) =====",
          flush=True)

    # (c) 박스 바로 위, 안전 높이(cart_max 위)에서 수평 호버 - 이 높이에서는 항상 벽/손잡이보다
    # 높으므로 옆으로 이동해도 충돌 경로가 없다. Phase B로 붙였기 때문에 수평 거리가 짧다.
    # move_link6_smooth는 이제 "흡착 팁" 자체를 폐루프로 제어하므로(매 스텝 실측), 여기서
    # 넘기는 좌표도 link_6이 아니라 팁이 있어야 할 자리 그대로다 - link_6<->팁 오프셋을
    # 호출부에서 따로 계산/보정할 필요가 없다.
    hover_target = np.array([box_pos[0], box_pos[1], safe_hover_z])
    move_link6_smooth(hover_target, label=f"박스 위 호버(#{idx})")
    hover_lift_h = lift_state["h"]

    if idx == 0:
        snapshot(eye=[chassis_pos1[0] - 0.8, chassis_pos1[1] - 1.0, safe_hover_z + 0.3],
                 target=[box_pos[0], box_pos[1], box_pos[2]], fname="_cartpick_03_hover_above_box.png")

    # (d) 실측(standoff 0.04/0.10/0.18, 속도 여러 조합으로 반복 테스트) - RMPflow(move_link6_smooth)로
    # 호버 높이(cart_max 위, 꽤 높음)에서 흡착 높이까지 큰 수직 낙차를 IK로 직접 풀게 하면 매번
    # 팔이 도중에 스스로/카트벽/박스와 부딪혀 발산했다(정체 감지 스크린샷으로 확인 - standoff를
    # 늘려도 개선 폭이 작았다 = 벽과의 거리 문제가 아니라 이 낙차 자체를 RMPflow가 자기충돌 없이
    # 못 풀고 있었다는 뜻). Phase A/B에서 이미 검증된 원칙 그대로 여기도 적용한다 - 호버에서
    # RMPflow로 이미 XY를 맞춰뒀으니, 그 뒤로는 팔 관절을 전혀 건드리지 않고 "리프트만" 내려서
    # 순수 수직 하강한다(팔 형상이 안 바뀌므로 자기충돌 경로 자체가 없다 - Phase B가 섀시를
    # 수평으로 안전하게 옮긴 것과 동일한 원리를 수직 축에 적용).
    # 실측(이 원칙 첫 적용 - 리프트 하강은 성공, 하지만 목표를 한 번에 계산해서 move_lift_to로
    # "확 내려갔더니" 흡착판 몸체(팁이 아니라)가 내려가는 도중 박스를 스치면서 박스가 넘어졌다
    # (스크린샷 확인 - 박스가 넘어진 채 기울어져 있음). box_top_z는 루프 시작 시점에 한 번만
    # 잰 값이라 박스가 조금이라도 밀리면 더 이상 안 맞는다. 목표 높이를 미리 계산해서 한 번에
    # 점프하는 대신, 1mm씩 아주 천천히 내리면서 매 스텝 흡착을 직접 시도해 "붙는 그 순간"
    # 바로 멈춘다 - 박스가 살짝 밀리거나 계산이 조금 어긋나도 그만큼 더/덜 내려가다가 성공하는
    # 지점에서 바로 서므로 과도하게 파고들 일이 없다(오버트래블 상한만 안전장치로 둔다).
    target_tip_z = box_top_z + GRASP_STANDOFF
    print(f"[크립 하강 준비] box_top_z={box_top_z:.4f} half_height={half_height:.4f} "
          f"grasp_radius={grasp_radius:.4f} target_tip_z={target_tip_z:.4f} "
          f"hover_lift_h={hover_lift_h:.4f}", flush=True)
    CREEP_STEP_H = 0.001
    CREEP_OVERTRAVEL_LIMIT = 0.06
    h = hover_lift_h
    n_creep_steps = 0
    grasped = False
    while True:
        tip_now = measure_tip_world_pos()
        if float(tip_now[2]) <= target_tip_z - CREEP_OVERTRAVEL_LIMIT:
            print(f"[크립 하강] 오버트래블 한계 도달(tip_z={float(tip_now[2]):.4f}) - 흡착 실패로 중단",
                  flush=True)
            break
        h -= CREEP_STEP_H
        set_lift_height(h)
        world.step(render=True)
        lift_state["h"] = h
        n_creep_steps += 1
        m0609_robot.gripper.close()
        if m0609_robot.gripper.is_closed():
            grasped = True
            break
    print(f"[크립 하강 완료] {n_creep_steps}스텝, 리프트 {hover_lift_h:.3f} -> {lift_state['h']:.3f} "
          f"grasped={grasped}", flush=True)

    # (e) 같은 원리로 순수 수직 후퇴 - 리프트만 다시 호버 때 높이로 되돌린다(팔은 그대로).
    move_lift_to(hover_lift_h, steps=300, hold_gripper_closed=grasped)

    box_pos_after = get_world_pos(box_prim)
    lifted_with_box = bool(box_pos_after[2] > box_pos[2] + 0.15)
    cleared_rim = bool((box_pos_after[2] - half_height) > float(cart_max[2]))
    print(f"[카트이탈 확인] 박스 바닥 z={box_pos_after[2] - half_height:.3f} vs rim={float(cart_max[2]):.3f} "
          f"-> cleared_rim={cleared_rim}", flush=True)

    results.append({
        "prim_path": prim_path, "box_id": placement["box_id"],
        "grasped": grasped, "lifted_with_box": lifted_with_box, "cleared_rim": cleared_rim,
    })
    print(f"[결과] {prim_path}: grasped={grasped} lifted_with_box={lifted_with_box}", flush=True)

    snapshot(eye=[chassis_pos1[0] - 0.8, chassis_pos1[1] - 1.1, box_pos[2] + 0.6],
             target=[box_pos[0], box_pos[1], box_pos[2] + 0.3], fname=f"_cartpick_04_lifted_{idx}.png")

    if grasped:
        m0609_robot.gripper.open()
        step_hold(20)

result_path = OUT_DIR / "_cartpick_result.json"
result_path.write_text(json.dumps(results, indent=2))
print(f"\n[저장 완료] {result_path}", flush=True)
print(f"[전체 결과] {results}\n", flush=True)

if HEADLESS:
    simulation_app.close()
else:
    print("[안내] 창을 직접 둘러보세요 - 닫으면 스크립트가 종료됩니다.\n", flush=True)
    while simulation_app.is_running():
        step_hold(1)
    simulation_app.close()
