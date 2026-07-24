"""
87.crate_pick_to_place_holonomic.py

36.crate_pick_to_place.py(테이블 박스들을 크레이트 적재 알고리즘 자리로 옮기는 데모)를
Nova Carter+고정 M0609 대신, 80~86번에서 완성한 "저상 대칭 홀로노믹 베이스 + 리프트 +
새 흡착 그리퍼 + 카메라" 조합으로 대체해서 실제로 동작하는지 검증한다.
기존 35/36번 파일은 건드리지 않고, 새 스크립트로 만든다(사용자 지시).

36.py 대비 바뀐 것
----
1. **로봇 자체 교체**: SingleManipulator 하나(섀시+팔이 한 articulation, root_joint로
   고정 마운트) 대신, 82~86번 패턴대로 base_robot(홀로노믹 섀시, 독립 articulation)과
   m0609_robot(M0609 팔, 독립 articulation)을 분리하고 매 스텝 set_lift_height()로
   리프트 높이만큼 띄워서 텔레포트 동기화한다(25~27번 이력에서 검증된 유일하게 안정적인
   패턴). 이 덕분에 36.py가 겪었던 "팔 반작용력으로 섀시가 밀림"(chassis_pin으로 매
   스텝 강제 재고정해야 했던 문제) 자체가 구조적으로 없다 - 두 몸체가 물리적으로 연결돼
   있지 않고 텔레포트로만 동기화되기 때문(84번에서 이미 chassis_pin 없이 err<3mm 확인).
2. **텔레포트 대신 실제 주행**: 36.py의 reposition_chassis()는 순수 텔레포트(회전 없음)
   였지만, 이 홀로노믹 베이스는 메카넘 휠로 실제로 주행할 수 있으므로(83/85/86번에서
   검증) 테이블<->크레이트 사이를 drive_to()로 실제 주행시킨다.
3. **씬/스캔 결과 재사용, 좌표계만 안전하게 분리**: 35.py의 실제 카메라 스캔(외부
   run_scan_once.py 프로세스 필요)은 다시 돌리지 않는다 - table_boxes_filtered.json/
   placement_result.json은 이미 있고, 그 안의 좌표는 전부 "스캔 당시 M0609 base_link
   기준 상대좌표"로 저장되어 있다. 새 로봇은 마운트 높이가 훨씬 낮아서(도킹 0.038m ~
   최고 0.388m, Nova Carter는 고정 0.42m) base_link 자세가 다르므로, 만약 새 로봇의
   base_link로 재투영하면 place 목표가 그 차이만큼 통째로 어긋난다(높이 차이가 그대로
   z 오차가 됨). 그래서 **씬을 열자마자 구(舊) Nova Carter+M0609의 base_link 월드
   자세를 딱 한 번 측정해서 BASE_POS/R_BASE로 고정하고, 그 다음에 구 로봇을 통째로
   삭제한다** - 이후 모든 스캔/배치 좌표 재투영은 이 "박제된" 기준으로만 계산하고, 새
   로봇의 실제 자세와는 완전히 분리한다. 물리적 PICK 좌표(박스를 실제로 어디서 집을지)는
   원래부터 스캔된 물리 프림의 실측 위치(get_world_pos)를 그대로 쓰므로 이 문제와 무관.
4. **리프트를 계속 최고 높이로**: 84.py에서 카트 바스켓처럼 마운트보다 높은 목표에
   손을 뻗을 때 리프트를 최고 높이로 올려서 reach를 확보하는 게 유효했던 것과 같은
   이유로, 테이블(z~0.34~0.5)/크레이트(z~0.15) 모두 마운트보다 오차 범위가 크므로
   시작하자마자 LIFT_MAX로 올리고 데모 내내 유지한다.

이 스크립트가 다루지 않는 것
----
- 실제 카메라 스캔 재실행(35.py 영역) - 기존 JSON 결과를 그대로 재사용한다.
- CARRY_OFFSET_FROM_CHASSIS/SAFE_TRANSIT_Z 등은 36.py 값을 초기값으로 재사용 - 새
  섀시 형상(홀로노믹, 대칭)에 맞게 재조정이 필요할 수 있음(다음 라운드에서 스크린샷
  보고 튜닝 예정).
"""

from isaacsim import SimulationApp

import os

HEADLESS = os.environ.get("HEADLESS", "0") == "1"
_sim_app_config = {"headless": HEADLESS}
if not HEADLESS:
    _sim_app_config.update({"width": 640, "height": 480})
simulation_app = SimulationApp(_sim_app_config)

import csv
import json
import shutil
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

_THIS_DIR = Path(__file__).resolve().parent
M0609_DIR = _THIS_DIR.parent / "M0609"
RMPFLOW_DIR = str(M0609_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController  # noqa: E402

SCENE_USD = str(_THIS_DIR / "crate_scan_scene.usd")
OUT_DIR = _THIS_DIR / "results" / "holonomic_base"
OUT_DIR.mkdir(parents=True, exist_ok=True)
RESULTS_DIR = _THIS_DIR / "results" / "crate_demo"
TABLE_BOXES_JSON = RESULTS_DIR / "table_boxes_filtered.json"
PLACEMENT_JSON = RESULTS_DIR / "placement_result.json"

M0609_USD = str(M0609_DIR / "Collected_m0609_vgp20_camera" / "m0609_vgp20_camera.usd")
M0609_URDF_PATH = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH = str(M0609_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(M0609_DIR / "rmpflow/m0609_rmpflow_common.yaml")

# 구(舊) Nova Carter+M0609 - 좌표 기준 측정 후 삭제한다(위 docstring 3번 참고)
OLD_MOBILE_MANIPULATOR_PATH = "/World/MobileManipulator"
OLD_M0609_BASE_LINK_PATH = f"{OLD_MOBILE_MANIPULATOR_PATH}/M0609/base_link"

EE_LINK_NAME = "link_6"
GRIPPER_BODY_NAME = "vgp20_suction_plate"

GRIPPER_RANGE_JSON = M0609_DIR / "Collected_m0609_vgp20_camera" / "_gripper_physical_range.json"
if GRIPPER_RANGE_JSON.exists():
    _range = json.loads(GRIPPER_RANGE_JSON.read_text())
    TIP_LOCAL_OFFSET = tuple(_range["tip_local_offset"])
    print(f"[그리퍼] {GRIPPER_RANGE_JSON}에서 로드: tip_local_offset={TIP_LOCAL_OFFSET}", flush=True)
else:
    TIP_LOCAL_OFFSET = (0.0, 0.0, 0.0188)
    print(f"[경고] {GRIPPER_RANGE_JSON} 없음 - 플레이스홀더 tip_local_offset={TIP_LOCAL_OFFSET} 사용", flush=True)

# DynamicSuctionGripper.close()의 판정 거리는 박스 반높이로 수렴한다(36.py와 동일 이유) -
# 박스마다 실행 시점에 half_height + GRASP_RADIUS_MARGIN으로 계산.
GRASP_RADIUS_MARGIN = 0.03
GRASP_STANDOFF = 0.01

DOWN_QUAT = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))
DOWN_QUAT_ROTATED90 = euler_angles_to_quat(np.array([0.0, np.pi, np.pi / 2.0]))
WAYPOINT_STEPS = 300

# ---------------- 홀로노믹 베이스 (82~86번과 동일 패턴) ----------------
BASE_PATH = "/World/HoloBase"
CHASSIS_PATH = f"{BASE_PATH}/chassis"
# 84.py에서 이미 검증한 것과 같은 이유(사용자 지적) - 섀시는 "긴 축"(1.0m, local X)과
# "짧은 축"(폭 ~0.4m, local Y)이 다른데, 처음엔 35.py의 Nova Carter 관례(FACE_ROT_Z=90)를
# 그대로 베껴써서 긴 축이 테이블/크레이트를 향하게 세웠다 - Nova Carter는 몸체 자체가
# 작아서 상관없었지만, 이 1.0m 길이 섀시는 그 상태로 y=-0.55에 세우면 앞쪽 끝이 테이블
# 앞면을 0.25m나 뚫고 들어가 물리 폭발이 났다(실측 확인). 84.py의 카트 접근과 동일하게
# ROT_Z=0으로 바꿔서 "짧은 축"이 테이블/크레이트를 향하게 한다 - 필요 standoff가
# 섀시 반길이(0.5m) 대신 반폭(~0.25m) 기준으로 줄어든다.
BASE_FACE_ROT_Z = 0.0

ROLLER_COUNT = 9
ROLLER_MASS = 0.02
HUB_MASS = 1.0
CHASSIS_MASS = 15.0
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 0.0, 50.0, 20.0
M0609_MOUNT_Z_ABOVE_CHASSIS_TOP = 0.02
LIFT_COLUMN_RADIUS = 0.045
LIFT_TRAVEL_M = 0.35
WZ_SIGN = 1.0
SETTLE_STEPS = 60

# ---- PICK/PLACE 안전 동작 파라미터 (36.py 초기값 재사용 - 새 섀시 형상에 맞게 다음
# 라운드에서 스크린샷 보고 재조정할 수 있음) ----
PICK_HOVER_HEIGHT_ABOVE_BOX = 0.20
RELEASE_CLEARANCE_ABOVE_FLOOR = 0.02
PLACE_DESCENT_SUBSTEPS = 3
SAFE_TRANSIT_Z = 0.85
CARRY_OFFSET_FROM_CHASSIS = (0.22, 0.0)  # 홀로노믹 베이스는 좌우 대칭이라 y오프셋은 0에서 시작


def load_recommended_dims():
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

# 84.py와 동일 공식 - ROT_Z=0일 때 테이블/크레이트 쪽으로 실제로 뻗어나오는 반경은
# 섀시 몸체(width/2)가 아니라 휠 트랙 폭(wheel_half_thickness_y 포함)이 더 크다.
_wheel_half_thickness_y = HUB_THICKNESS / 2.0 + ROLLER_LENGTH * 0.5 + ROLLER_RADIUS
CHASSIS_HALF_WIDTH_EFFECTIVE = BASE_WIDTH / 2.0 + _wheel_half_thickness_y * 1.3
STANDOFF_MARGIN = 0.15

# 35.py 기준: TABLE_SIZE=(0.8,0.6,...) at CART_POS=(0,0) -> 테이블 앞면 y=-0.3.
# 이 값 + 섀시 반폭(짧은 축 기준) + 여유만큼 뒤에서 시작해야 스폰 즉시 충돌이 안 난다
# (실측: ROT_Z=90(긴 축 정면)으로 y=-0.55에서 시작했다가 60~80스텝 사이 섀시가
# z=7.9m까지 튕겨나가는 물리 폭발로 확인된 문제).
_TABLE_HALF_Y = 0.30
_STANDOFF_Y = _TABLE_HALF_Y + CHASSIS_HALF_WIDTH_EFFECTIVE + STANDOFF_MARGIN
ROBOT_START_XY = (0.0, -_STANDOFF_Y)
CHASSIS_TARGET_XY = (0.9, -_STANDOFF_Y)
print(f"[STANDOFF] 테이블 반폭 {_TABLE_HALF_Y:.3f} + 섀시 반폭(짧은 축) {CHASSIS_HALF_WIDTH_EFFECTIVE:.3f} "
      f"+ 여유 {STANDOFF_MARGIN:.3f} = {_STANDOFF_Y:.3f}m -> ROBOT_START_XY/CHASSIS_TARGET_XY y={-_STANDOFF_Y:.3f}",
      flush=True)


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
    """82~86번과 동일 - 허브(구동) + 롤러 ROLLER_COUNT개(45도 자유회전)로 실제 메카넘 휠을 만든다."""
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
    """82~86번과 동일 패턴 - 독립 articulation + 매 프레임 텔레포트."""
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


class DynamicSuctionGripper(SurfaceGripper):
    """36.py와 동일 - set_target()으로 박스마다 흡착 대상을 바꿀 수 있다."""

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
        if self._attached:
            return
        if self._target_prim_path is None:
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
            print(f"  [해제] {self._joint_path} 삭제", flush=True)
        self._attached = False

    def is_closed(self) -> bool:
        return self._attached

    def is_open(self) -> bool:
        return not self._attached


def discover_box_prim_paths(stage):
    """36.py와 동일 - /World 바로 아래 "Box_"로 시작하는 프림을 전부 후보로 찾는다."""
    world_prim = stage.GetPrimAtPath("/World")
    return [
        str(child.GetPath())
        for child in world_prim.GetChildren()
        if child.GetName().startswith("Box_")
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


def world_aabb_from_base_corners(corners_base, base_pos, R_base):
    pts_base = np.asarray(corners_base, dtype=np.float64)
    pts_world = pts_base @ R_base.T + base_pos
    mn = pts_world.min(axis=0)
    mx = pts_world.max(axis=0)
    return (mn + mx) / 2.0, mx - mn, mn


def world_aabb_from_base_pos_dims(pos_base, dims_base, base_pos, R_base):
    pos_base = np.asarray(pos_base, dtype=np.float64)
    dims = np.asarray(dims_base, dtype=np.float64)
    corners_world = []
    for dx in (0.0, dims[0]):
        for dy in (0.0, dims[1]):
            for dz in (0.0, dims[2]):
                corners_world.append(R_base @ (pos_base + np.array([dx, dy, dz])) + base_pos)
    corners_world = np.array(corners_world)
    mn = corners_world.min(axis=0)
    mx = corners_world.max(axis=0)
    return (mn + mx) / 2.0, mx - mn, mn


def get_world_pos(prim):
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array(mat.ExtractTranslation())


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


# ================= 결과 JSON 로드 =================
table_data = json.loads(TABLE_BOXES_JSON.read_text())
placement_data = json.loads(PLACEMENT_JSON.read_text())
scan_by_box_id = {str(b["box_id"]): b for b in table_data["boxes"]}
placements = placement_data["placements"]
print(f"[로드] {TABLE_BOXES_JSON.name}: 스캔된 박스 {len(table_data['boxes'])}개, "
      f"{PLACEMENT_JSON.name}: 배치 성공 {len(placements)}개(미적재 {len(placement_data.get('unloadable', []))}개)",
      flush=True)

# ================= 씬 열기 =================
# World()를 먼저 만들면 그 시점의 스테이지 상태로 물리 씬이 바인딩되는데, 그 뒤에
# 구 로봇을 통째로 RemovePrim하면 broadphase가 깨진다(실측 확인 - M0609 전체 링크의
# PhysX transform이 invalid, chassis 쿼터니언까지 zero-norm). 그렇다고 RemovePrim 직후
# world.reset()을 끼워넣으면(2차 시도) 이번엔 base_robot의 dof_names가 빈 리스트로
# 깨진다(86번 인라인 실측 실험 때와 동일 증상 - reset을 두 번 하면 나중 articulation의
# dof 인식이 깨지는 것으로 보임). 그래서 World()는 "모든 프림 편집(제거+새 로봇 빌드)이
# 다 끝난 뒤" 딱 한 번만 만들고, reset()도 82~86번과 동일하게 전체 스텝의 맨 마지막에
# 딱 한 번만 호출한다.
omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    simulation_app.update()

stage = omni.usd.get_context().get_stage()

CANDIDATE_BOX_PRIM_PATHS = discover_box_prim_paths(stage)
print(f"[박스 프림 탐색] {len(CANDIDATE_BOX_PRIM_PATHS)}개 발견: {CANDIDATE_BOX_PRIM_PATHS}", flush=True)

# ---- 구 로봇의 base_link 좌표를 "박제"해서 좌표 기준으로 고정 (docstring 3번 참고) ----
old_base_link_prim = stage.GetPrimAtPath(OLD_M0609_BASE_LINK_PATH)
assert old_base_link_prim.IsValid(), f"{OLD_M0609_BASE_LINK_PATH} 없음 - crate_scan_scene.usd 확인 필요"
old_mat = UsdGeom.Xformable(old_base_link_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
BASE_POS = np.array(old_mat.ExtractTranslation())
old_quat_gf = old_mat.ExtractRotation().GetQuat()
BASE_QUAT = np.array([old_quat_gf.GetReal(), *old_quat_gf.GetImaginary()])
R_BASE = quat_wxyz_to_matrix(BASE_QUAT)
print(f"[구 로봇 기준좌표 측정] BASE_POS={np.round(BASE_POS, 4)} BASE_QUAT={np.round(BASE_QUAT, 4)} "
      f"(이후 스캔/배치 좌표 재투영은 전부 이 값 고정 사용)", flush=True)

stage.RemovePrim(OLD_MOBILE_MANIPULATOR_PATH)
print(f"[제거] {OLD_MOBILE_MANIPULATOR_PATH} (구 Nova Carter+M0609 - 기준좌표는 이미 확보)", flush=True)
for _ in range(10):
    simulation_app.update()

# ================= 새 홀로노믹 베이스 + M0609 구성 =================
chassis_path, hub_joint_paths, k_factor = build_holonomic_base(
    stage, ROBOT_START_XY, BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT
)

# 86번에서 실측(큐브 낙하+GUI 육안 튜닝)한 값 - 82~86번과 동일하게 사용
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
    tip_local_offset=TIP_LOCAL_OFFSET,
)
m0609_robot = SingleManipulator(
    prim_path=m0609_base_link_path,
    end_effector_prim_path=ee_path,
    name="m0609_arm",
    gripper=gripper,
)
base_robot = SingleArticulation(prim_path=chassis_path, name="holo_base")

# World()는 모든 프림 편집(구 로봇 제거 + 새 홀로노믹 베이스/M0609 빌드)이 다 끝난 뒤
# 여기서 딱 한 번만 만든다(위 씬 열기 섹션 주석 참고).
world = World(stage_units_in_meters=1.0)
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

# ---- 리프트 제어 (82~86번과 동일한 텔레포트 패턴) ----
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
    speeds = mecanum_wheel_speeds(vx, vy, WZ_SIGN * wz, WHEEL_RADIUS, k_factor)
    return ArticulationAction(joint_velocities=speeds, joint_indices=hub_dof_indices)


SMOOTH_ALPHA = 0.12
_smooth_state = {"vx": 0.0, "vy": 0.0, "wz": 0.0}


def drive_to(target_x=None, target_y=None, target_yaw_deg=None, tolerance_xy=0.03, tolerance_yaw_deg=2.0,
             max_speed=0.4, max_wz=0.2, kp_xy=1.8, kp_yaw=0.25, max_steps=3000, label=""):
    """83/85/86번과 동일한 폐루프 주행(정체 감지+저역통과 필터) - 텔레포트가 아니라 실제
    메카넘 휠 구동으로 테이블<->크레이트를 오간다."""
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

# ================= RMPflow 컨트롤러 =================
controller = RMPFlowController(
    name="crate_pick_to_place_holonomic",
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


def move_link6(target_pos, steps=WAYPOINT_STEPS, hold_gripper_closed=False, label="", orientation=DOWN_QUAT):
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
        if (i + 1) % 100 == 0:
            print(f"  [진행{' ' + label if label else ''}] {i + 1}/{steps} 스텝", flush=True)
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
    target=[0.0, 0.0, 0.5],
    fname="_crate87_00_start.png",
)

# ================= 리프트를 최고 높이로 (테이블/크레이트 모두 마운트보다 reach가 필요) =================
print(f"\n[리프트] 도킹({LIFT_MIN:.3f}) -> 최고({LIFT_MAX:.3f})", flush=True)
move_lift_to(LIFT_MAX, steps=120)

# ================= 박스마다 PICK -> 실제 주행(크레이트) -> PLACE -> (다음 박스면) 실제 주행(테이블) =================
used_prim_paths = set()
print(f"\n[계획] 적재 알고리즘이 배치에 성공한 박스 {len(placements)}개를 순서대로 집어서 옮긴다", flush=True)

for idx, placement in enumerate(placements):
    box_id = str(placement["box_id"])
    scan_entry = scan_by_box_id.get(box_id)
    if scan_entry is None:
        print(f"[경고] box_id={box_id}가 테이블 스캔 결과에 없음 - 건너뜀", flush=True)
        continue

    rotated = bool(placement.get("rotated", False))
    box_orientation = DOWN_QUAT_ROTATED90 if rotated else DOWN_QUAT
    if rotated:
        print(f"[회전] box_id={box_id}는 90도 회전 배치 - PICK 직후 손목을 돌린다", flush=True)

    scan_center, scan_dims, _ = world_aabb_from_base_corners(scan_entry["corners_m"], BASE_POS, R_BASE)
    available = [p for p in CANDIDATE_BOX_PRIM_PATHS if p not in used_prim_paths]
    prim_path, match_dist = match_physical_prim(stage, scan_center[:2], available)
    if prim_path is None:
        print(f"[경고] box_id={box_id}에 매칭되는 물리 프림을 못 찾음 - 건너뜀", flush=True)
        continue
    used_prim_paths.add(prim_path)
    box_prim = stage.GetPrimAtPath(prim_path)
    print(f"\n===== [{idx + 1}/{len(placements)}] box_id={box_id} -> {prim_path} "
          f"(스캔 위치와의 실측 거리={match_dist:.3f}m) =====", flush=True)

    # ---- 1. PICK (물리 프림 실측 위치 그대로 사용 - 로봇 종류와 무관) ----
    box_pos = get_world_pos(box_prim)
    half_height = float(scan_dims[2]) / 2.0
    box_top_z = float(box_pos[2]) + half_height
    grasp_radius = half_height + GRASP_RADIUS_MARGIN
    gripper.set_target(prim_path, grasp_radius)

    print(f"[PICK] {prim_path} 실측 world pos={np.round(box_pos, 3)}, "
          f"half_height={half_height:.3f}, grasp_radius={grasp_radius:.3f}", flush=True)

    pick_hover_pos = (box_pos[0], box_pos[1], box_top_z + PICK_HOVER_HEIGHT_ABOVE_BOX)
    pick_grasp_pos = (box_pos[0], box_pos[1], box_top_z + TIP_LOCAL_OFFSET[2] + GRASP_STANDOFF)

    current_ee_pos, _ = m0609_robot.end_effector.get_world_pose()
    lift_pos = (float(current_ee_pos[0]), float(current_ee_pos[1]), SAFE_TRANSIT_Z)
    approach_pos = (box_pos[0], box_pos[1], SAFE_TRANSIT_Z)

    move_link6(lift_pos, steps=200, label=f"pick_lift[{idx}]")
    move_link6(approach_pos, steps=200, label=f"pick_approach[{idx}]")
    move_link6(pick_hover_pos, steps=200, label=f"pick_hover[{idx}]")
    move_link6(pick_grasp_pos, steps=200, label=f"pick_grasp[{idx}]")
    for _ in range(30):
        m0609_robot.gripper.close()
        set_lift_height(lift_state["h"])
        world.step(render=True)

    if not m0609_robot.gripper.is_closed():
        print(f"[경고] box_id={box_id} 흡착 실패 - 이 박스는 건너뜀", flush=True)
        move_link6((box_pos[0], box_pos[1], SAFE_TRANSIT_Z), steps=150, label=f"pick_실패_후퇴[{idx}]")
        continue

    # ---- 흡착 성공: 텔레포트 대신 "실제 주행" 전에도 동일하게 안전 운반 자세로 만든다 ----
    post_grasp_lift_pos = (box_pos[0], box_pos[1], SAFE_TRANSIT_Z)
    move_link6(post_grasp_lift_pos, steps=200, hold_gripper_closed=True, label=f"pick_post_lift[{idx}]")

    if rotated:
        move_link6(post_grasp_lift_pos, steps=200, hold_gripper_closed=True,
                   label=f"pick_rotate[{idx}]", orientation=box_orientation)

    chassis_pos_now, _ = base_robot.get_world_pose()
    carry_pos = (
        float(chassis_pos_now[0]) + CARRY_OFFSET_FROM_CHASSIS[0],
        float(chassis_pos_now[1]) + CARRY_OFFSET_FROM_CHASSIS[1],
        SAFE_TRANSIT_Z,
    )
    move_link6(carry_pos, steps=200, hold_gripper_closed=True, label=f"pick_carry[{idx}]", orientation=box_orientation)

    if idx == 0:
        snapshot(
            eye=[chassis_pos0[0] - 1.0, chassis_pos0[1] - 1.3, 1.4],
            target=[box_pos[0], box_pos[1], box_top_z],
            fname="_crate87_01_picked.png",
        )

    # ---- 2. 실제 주행: 테이블 -> 크레이트 (텔레포트 아님 - 메카넘 휠로 실제 이동) ----
    drive_to(target_x=CHASSIS_TARGET_XY[0], target_y=CHASSIS_TARGET_XY[1], label=f"to_crate[{idx}]")

    if idx == 0:
        chassis_pos_now, _ = base_robot.get_world_pose()
        snapshot(
            eye=[chassis_pos_now[0] - 1.0, chassis_pos_now[1] - 1.3, 1.5],
            target=[CHASSIS_TARGET_XY[0], CHASSIS_TARGET_XY[1], 0.3],
            fname="_crate87_02_repositioned.png",
        )

    # ---- 3. PLACE (박제해둔 구 로봇 기준좌표(BASE_POS/R_BASE)로 재투영 - 새 로봇 자세와 무관) ----
    place_center, place_dims, place_min = world_aabb_from_base_pos_dims(
        placement["position_base_frame"], placement["dimensions"], BASE_POS, R_BASE)
    place_world_xy = (float(place_center[0]), float(place_center[1]))
    crate_floor_world_z = float(place_min[2])
    place_release_z = (
        crate_floor_world_z + RELEASE_CLEARANCE_ABOVE_FLOOR
        + float(place_dims[2]) + TIP_LOCAL_OFFSET[2]
    )
    place_hover_z = place_release_z + 0.15

    place_approach_pos = (place_world_xy[0], place_world_xy[1], SAFE_TRANSIT_Z)
    place_hover_pos = (place_world_xy[0], place_world_xy[1], place_hover_z)

    move_link6(place_approach_pos, steps=300, hold_gripper_closed=True, label=f"place_approach[{idx}]", orientation=box_orientation)
    move_link6(place_hover_pos, steps=300, hold_gripper_closed=True, label=f"place_hover[{idx}]", orientation=box_orientation)

    if idx == 0:
        chassis_pos_now, _ = base_robot.get_world_pose()
        snapshot(
            eye=[chassis_pos_now[0] - 1.0, chassis_pos_now[1] - 1.3, 1.5],
            target=[place_world_xy[0], place_world_xy[1], crate_floor_world_z],
            fname="_crate87_03_approaching.png",
        )

    descent_span = place_hover_z - place_release_z
    substep_moves = max(300 // PLACE_DESCENT_SUBSTEPS, 1)
    for sub_i in range(1, PLACE_DESCENT_SUBSTEPS + 1):
        sub_z = place_hover_z - descent_span * sub_i / PLACE_DESCENT_SUBSTEPS
        sub_pos = (place_world_xy[0], place_world_xy[1], sub_z)
        move_link6(
            sub_pos, steps=substep_moves, hold_gripper_closed=True,
            label=f"place_release_sub{sub_i}/{PLACE_DESCENT_SUBSTEPS}[{idx}]", orientation=box_orientation,
        )

    m0609_robot.gripper.open()

    box_rigid_prim = SingleRigidPrim(prim_path)
    box_rigid_prim.initialize(physics_sim_view=world.physics_sim_view)
    box_rigid_prim.set_linear_velocity(np.array([0.0, 0.0, -0.3]))
    print(f"[낙하 유도] {prim_path}에 -z 속도 부여해서 sleep 상태 강제 해제", flush=True)

    for i in range(180):
        set_lift_height(lift_state["h"])
        world.step(render=True)
        if (i + 1) % 30 == 0:
            _z = get_world_pos(box_prim)[2]
            print(f"  [낙하 확인] {i + 1}/180 스텝, 박스 z={_z:.3f}", flush=True)

    final_box_pos = get_world_pos(box_prim)
    err_xy = np.linalg.norm(final_box_pos[:2] - np.array(place_world_xy))
    print(
        f"\n[완료 {idx + 1}/{len(placements)}] {prim_path} 최종 world 위치={np.round(final_box_pos, 3)}, "
        f"목표 xy={np.round(place_world_xy, 3)}, xy 오차={err_xy:.4f}m",
        flush=True,
    )

    move_link6(place_hover_pos, steps=200, label=f"place_retract_hover[{idx}]", orientation=box_orientation)
    place_retract_safe_pos = (place_world_xy[0], place_world_xy[1], SAFE_TRANSIT_Z)
    move_link6(place_retract_safe_pos, steps=200, label=f"place_retract_safe[{idx}]", orientation=box_orientation)

    chassis_pos_now, _ = base_robot.get_world_pose()
    snapshot(
        eye=[chassis_pos_now[0] - 1.0, chassis_pos_now[1] - 1.3, 1.5],
        target=[place_world_xy[0], place_world_xy[1], crate_floor_world_z],
        fname=f"_crate87_04_placed_box{idx + 1}.png",
    )
    if idx == len(placements) - 1:
        snapshot(
            eye=[chassis_pos_now[0] - 1.0, chassis_pos_now[1] - 1.3, 1.5],
            target=[place_world_xy[0], place_world_xy[1], crate_floor_world_z],
            fname="_crate87_04_placed.png",
        )

    # ---- 4. 다음 박스가 남아 있으면 테이블로 실제 주행 복귀 ----
    if idx < len(placements) - 1:
        chassis_pos_now, _ = base_robot.get_world_pose()
        return_carry_pos = (
            float(chassis_pos_now[0]) + CARRY_OFFSET_FROM_CHASSIS[0],
            float(chassis_pos_now[1]) + CARRY_OFFSET_FROM_CHASSIS[1],
            SAFE_TRANSIT_Z,
        )
        move_link6(return_carry_pos, steps=200, label=f"place_carry_before_return[{idx}]")
        drive_to(target_x=ROBOT_START_XY[0], target_y=ROBOT_START_XY[1], label=f"to_table[{idx}]")

print(f"\n[안내] 전체 검증 완료 - 배치 시도 {len(placements)}개.\n", flush=True)

# ================= 이번 실행 결과를 날짜별 폴더에 보관 =================
_run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
_archive_dir = RESULTS_DIR / "runs" / f"{_run_stamp}_87pickplace_holonomic"
_archive_dir.mkdir(parents=True, exist_ok=True)
_src_names = [_f.name for _f in OUT_DIR.glob("_crate87_0*.png")]
_archived = []
for _name in _src_names:
    _archived_name = f"{_run_stamp}_{_name}"
    shutil.copy2(OUT_DIR / _name, _archive_dir / _archived_name)
    _archived.append(_archived_name)
print(f"[보관] {_archive_dir} 에 {len(_archived)}개 파일 복사: {sorted(_archived)}", flush=True)

SCENE_OUT = str(OUT_DIR / "crate_pick_to_place_holonomic_scene.usd")
omni.usd.get_context().save_as_stage(SCENE_OUT)
print(f"[저장 완료] {SCENE_OUT}", flush=True)

if HEADLESS:
    simulation_app.close()
else:
    print("[안내] 창을 직접 둘러보세요 - 닫으면 스크립트가 종료됩니다.\n", flush=True)
    # M0609는 독립 articulation이라 매 스텝 set_lift_height()로 텔레포트해줘야 섀시에
    # "붙어" 있다(82~88번 전체의 공통 설계) - world.step()만 부르면 텔레포트가 멈춰서
    # 중력으로 떨어진다(88번에서 실측 확인된 동일 버그).
    while simulation_app.is_running():
        step_hold(1)
    simulation_app.close()
