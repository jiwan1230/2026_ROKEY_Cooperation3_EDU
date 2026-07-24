"""
84.holonomic_pick_verify.py

이 트랙(80~83번)에서 완성한 저상 대칭 홀로노믹 베이스 + M0609 + 리프트 + 새 흡착 그리퍼
조합에, RMPflow 컨트롤러와 DynamicSuctionGripper(28.cart_to_trunk_pick_place_lift.py 계열)를
결합해서 실제로 박스를 PICK할 수 있는지 검증한다.

버전 이력
----
1차(카트 없이 낮은 스탠드): STANDOFF_X=0.55로 카트 앞에 세웠는데, 섀시 몸체(1.0m 길이,
정중앙 마운트라 중심에서 절반인 0.5m가 카트 쪽으로 뻗어나옴)와 카트가 실제로 겹쳐서 스폰
직후 물리 폭발이 났다(박스가 낙하 지점에서 1m 넘게 튕겨나감, 실측으로 확인 - "물리 수치만
으론 안 믿는다"교훈이 여기서도 재확인됨). RMPflow+그리퍼 결합 자체가 되는지부터 먼저 좁혀서
보려고, 카트를 빼고 낮은(0.35m) 스탠드+박스로 단순화해서 재시도 - 호버/그랩/흡착/들어올리기
전부 성공(err 3cm 이하, `_pickverify_04_lifted.png`로 시각 확인).
2차(카트 재도입, yaw=180): 1차가 통과했으니 실제 쇼핑카트 기준으로 되돌렸다. 28.py의 PICK
방향 관례(yaw=180, 섀시의 "긴 축"이 카트를 마주봄)를 그대로 따랐더니, standoff가 섀시 반
"길이"(0.5m, CHASSIS_LENGTH_EXTENDED/2) + 카트 반폭(0.3m) + 여유(0.15m) = 0.95m나 필요했고,
그 결과 M0609 reach가 완전히 부족했다(호버 err=0.29m, 그랩 하강 err=0.74m - 팔이 접힌
자세로 되돌아갈 정도로 못 미침). 박스/카트 배치 자체는 정상(스크린샷으로 확인), 순수하게
reach 부족.
3차(이 버전, 사용자 지적 반영): 섀시는 "긴 축"(1.0m)과 "짧은 축"(폭 ~0.4m)이 다르다 - 굳이
긴 축으로 카트를 마주볼 필요가 없다. 83번에서 제자리 회전이 이미 검증됐으니, yaw=90으로
세워서 섀시의 "짧은 축"이 카트를 향하게 한다. standoff 기준이 섀시 반길이(0.5m)에서 섀시
반폭(휠 트랙 포함, ~0.25m)으로 줄어들어 0.95m -> 약 0.7m로 짧아진다 - 2차에서 부족했던
reach 여유(약 0.3~0.4m)를 정확히 상쇄하는 크기라 이번엔 통과할 가능성이 높다(아래 4번 항목).

이 스크립트가 다루지 않는 것 (다음 단계로 미룸)
----
- 주행(83번에서 이미 별도 검증 완료) - 여기서는 베이스를 고정 위치에 세워두고 시작한다.
- 트렁크 PLACE(85번 계획) - PICK 성공/실패와 IK 오차만 우선 좁혀서 본다.

28.py 대비 달라진 점
----
1. **마운트 오프셋 없음**: 28.py는 Nova Carter 섀시가 좌우 비대칭이라 MOUNT_LOCAL_OFFSET_X로
   마운트를 한쪽으로 밀어 PICK/PLACE reach를 절충해야 했다. 이 홀로노믹 베이스는 애초에
   "M0609를 섀시 정중앙에 대칭으로 얹어서 그 절충 자체를 없앤다"가 존재 이유이므로, mounted_xy()
   보정이 필요 없다 - chassis_pos를 그대로 베이스 포즈로 쓴다.
2. **마운트 높이가 훨씬 낮음**: Nova Carter는 LIFT_REACH_H=0.42m 고정이었지만, 이 저상
   베이스는 도킹 0.166m ~ 최고 0.516m(둘 다 world Z, 82번 실측 기준)로 훨씬 낮다. 카트
   바스켓(z=0.68) 위 박스를 집으려면 위로 뻗어야 하는 거리가 오히려 늘어날 수 있어서,
   이번 검증은 리프트를 LIFT_MAX까지 올린 상태에서 시도한다(82/83번에서 검증된 리프트
   텔레포트 그대로 재사용).
3. **그리퍼가 훨씬 작음**: 28.py의 TIP_LOCAL_OFFSET=0.121m/GRASP_RADIUS=0.10m는 구형 VGP20
   그리퍼 기준이다. 새 흡착판은 M0609/11.replace_vgp20_suction_plate.py가 실측해서 저장해둔
   `M0609/Collected_m0609_vgp20_camera/_gripper_physical_range.json`의
   TIP_LOCAL_OFFSET=0.0188m/GRASP_RADIUS=0.0515m를 그대로 읽어서 쓴다(하드코딩 금지 - 그리퍼가
   또 바뀌면 이 json만 갱신하면 됨). 그리퍼 바디 프림 이름도 구형 "vgp20"이 아니라 새
   경로 "vgp20_suction_plate"다(11번 스크립트가 구형과 안 겹치게 새로 만든 경로).
4. **BASE_FACE_ROT_Z=90 + STANDOFF_X는 섀시 "짧은 축" 기준 계산값**: 28.py는 Nova Carter가
   원래 차동구동이라 방향 전환이 costly해서 "긴 축으로 정면 접근"이 자연스러웠지만, 이
   홀로노믹 베이스는 제자리 회전이 공짜에 가깝다(83번 검증 완료). 그래서 카트를 마주볼 때
   섀시를 90도로 세워 "짧은 축"(폭, 휠 트랙 포함 반경 `CHASSIS_HALF_WIDTH_EFFECTIVE`≈0.25m)이
   카트를 향하게 한다 - standoff = `CHASSIS_HALF_WIDTH_EFFECTIVE` + 카트 반폭(bbox 실측) +
   여유(0.15m). 2차 시도(yaw=180, 긴 축 0.5m 기준)는 이보다 0.95-0.7≈0.25~0.3m나 더
   먼 곳에서 시작해야 했고, 그 초과분이 그대로 reach 부족(err 0.3~0.7m)으로 나타났다.

시퀀스: 카트 배치 -> 베이스를 90도로 세워 M0609+리프트 마운트(82/83과 동일 코드, 섀시 짧은
축이 카트를 향하도록 STANDOFF_X만큼 띄움) -> 카트 바스켓 안에 스탠드+박스 배치(28.py와
동일 패턴) -> RMPflow+그리퍼 결합 -> 박스 위 호버 -> 하강 -> 그리퍼 close(흡착 시도) ->
다시 들어올림 -> 결과(흡착 성공 여부, IK 오차) 로깅 + 단계별 스크린샷.
"""

from isaacsim import SimulationApp

import os
simulation_app = SimulationApp({"headless": os.environ.get("HEADLESS", "0") == "1"})

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

# ---------------- 82/83번과 동일 (베이스/카트 구성 재사용) ----------------
CART_USD = str(_THIS_DIR / "assets/Metal_Shopping_Cart.usdz")
CART_POS = (0.0, 0.0, 0.0)
CART_EXTRA_SCALE = 0.55
SDF_RESOLUTION = 256
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 0.0, 50.0, 20.0

BASE_PATH = "/World/HoloBase"
CHASSIS_PATH = f"{BASE_PATH}/chassis"
# 사용자 지적(2026-07-23, 카트 reach 실패 원인 논의): 섀시는 "긴 축"(1.0m, local X)과
# "짧은 축"(폭 ~0.4m, local Y)이 있는데, 28.py의 PICK 방향 관례(yaw=180, 긴 축이 카트를
# 마주봄)를 그대로 따라 하면서 필요 이상으로 standoff가 커졌었다. 83번에서 제자리 회전이
# 이미 검증됐으니, 카트 앞에서는 섀시를 90도 돌려 "짧은 축"이 카트를 향하게 세운다 -
# standoff가 섀시 반길이(0.5m) 대신 반폭(~0.25m) 기준으로 줄어든다(아래 STANDOFF_X 계산 참고).
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
# 82/83번은 0.35m(차 밑 통과 높이 제한과 균형 맞춘 데모용 값)이지만, 이 스크립트는 차량이
# 없어서 그 제약이 없다 - 4차 시도(90도 회전 후에도 hover/grasp err가 0.39/0.40m로 남음,
# 스크린샷 확인 결과 팔이 카트 림 근처에서 못 들어가고 멈춤)에서 남은 오차가 수평이 아니라
# 수직 reach 부족 쪽에 가깝다고 판단, 리프트를 카트 바스켓 높이 근처까지 더 올려서 수직
# 거리를 줄이고 그만큼 수평 reach 여유를 확보한다(28.py도 LIFT_MAX=1.20으로 이런 식으로
# 리프트를 reach 확보 수단으로 썼던 전례가 있음).
LIFT_TRAVEL_M = 0.70

EE_LINK_NAME = "link_6"
# 구형 "vgp20"이 아니라 11.replace_vgp20_suction_plate.py가 새로 만든 경로 - 구형은
# SetActive(False)로 꺼져있고 이 경로에 새 흡착판이 달려있다.
GRIPPER_BODY_NAME = "vgp20_suction_plate"

# ---- 새 그리퍼 물리 작동 범위 (하드코딩 대신 11번이 실측해서 저장해둔 json에서 읽음) ----
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
    print(f"[경고] {GRIPPER_RANGE_JSON} 없음 - 플레이스홀더 값 사용: "
          f"tip_local_offset={TIP_LOCAL_OFFSET} grasp_radius={GRASP_RADIUS}", flush=True)

CUBE_SIZE = 0.05
CUBE_MASS = 0.05
CART_DROP_HEIGHT_ABOVE_FLOOR = 0.10
CUBE_STATIC, CUBE_DYNAMIC = 1.2, 1.0
CART_BASKET_FLOOR_Z = 0.68

# 카트 반폭은 실측(bbox)해야 정확하다 - 씬 구성 단계에서
# CHASSIS_HALF_WIDTH_EFFECTIVE(위에서 계산, 90도로 세웠을 때 카트 쪽으로 뻗는 반경) + 카트
# 반폭 + STANDOFF_MARGIN으로 계산한다.
STANDOFF_MARGIN = 0.15
# 5차 시도 관찰: hover(0.29m err, 첫 웨이포인트 - 초기 시드 자세에서 멀리 떨어진 목표)보다
# grasp/lift(0.19m/0.003m err, 이전 웨이포인트에서 이미 가까워진 상태에서 시작)가 오히려 더
# 잘 수렴했다 - 하드 리치 한계보다는 RMPflow가 초기 시드에서 목표까지 수렴할 시간이
# 부족했을 가능성이 높다고 보고 스텝 수를 늘려 재검증한다.
WAYPOINT_STEPS = 400
SETTLE_STEPS = 60
DOWN_QUAT = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))


def load_recommended_dims():
    csv_path = OUT_DIR / "_evaluate_low_profile_base.csv"
    if csv_path.exists():
        import csv
        with csv_path.open() as f:
            rows = [r for r in csv.DictReader(f) if r["feasible"] == "True"]
        if rows:
            rows.sort(key=lambda r: (-float(r["trunk_insertion_depth_m"]), float(r["base_length"])))
            best = rows[0]
            return float(best["base_length"]), float(best["base_width"]), float(best["base_height"])
    print("[경고] _evaluate_low_profile_base.csv 없음 - 플레이스홀더 치수 사용", flush=True)
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

# build_holonomic_base()의 half_w 계산과 동일한 공식(82/83.py와 동일) - 90도로 세웠을 때
# 카트 쪽으로 실제로 뻗어나오는 반경은 섀시 몸체 자체(width/2)가 아니라 휠 트랙 폭
# (wheel_half_thickness_y 포함)이 더 크므로 이 값을 기준으로 standoff를 잡아야 한다.
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
    """82/83.py와 동일 - 허브(구동) + 롤러 ROLLER_COUNT개(45도 자유회전)로 실제 메카넘 휠을 만든다."""
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

    return CHASSIS_PATH, hub_joint_paths


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
    """82/83.py와 동일 패턴(LiftColumnVisual 마운트 버그 수정 버전 포함) - 마운트 오프셋 없이
    섀시 정중앙 위에 M0609를 리프트(텔레포트 방식)로 얹는다."""
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

    # 버그 이력(2026-07-23) - 82.py의 mount_m0609() 주석 참고: BASE_PATH 밑에 두면 월드 좌표를
    # 로컬 translate로 그대로 쓸 때 BASE_START_XY 오프셋이 이중으로 적용된다. /World 바로
    # 밑에 둬서 translate 값이 곧바로 월드 좌표가 되게 한다.
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


# ╔══════════════════════════════════════════════════════════════╗
# ║  DynamicSuctionGripper (28.py와 동일, M0609/8 계열)               ║
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
for _ in range(20):
    simulation_app.update()
add_sdf_collision(stage, "/World/ShoppingCart")

cart_min, cart_max = bbox_of(stage, "/World/ShoppingCart")
cart_center_xy = ((cart_min[0] + cart_max[0]) / 2.0, (cart_min[1] + cart_max[1]) / 2.0)
cart_half_x = (cart_max[0] - cart_min[0]) / 2.0
print(f"[카트 bbox] min={cart_min} max={cart_max} center_xy={cart_center_xy} half_x={cart_half_x:.3f}", flush=True)

# 실측 기반 standoff (docstring 4번 항목 참고, 사용자 지적 반영: 섀시를 90도로 세워 "짧은
# 축"이 카트를 향하게 해서 필요 standoff 자체를 줄인다) - 임의 상수 대신 항상 이 공식으로 계산.
STANDOFF_X = CHASSIS_HALF_WIDTH_EFFECTIVE + cart_half_x + STANDOFF_MARGIN
print(f"[STANDOFF] {CHASSIS_HALF_WIDTH_EFFECTIVE:.3f}(섀시 반폭, 90도 회전 기준) + "
      f"{cart_half_x:.3f}(카트 반폭) + {STANDOFF_MARGIN:.3f}(여유) = {STANDOFF_X:.3f}m", flush=True)

STAND_BOX_SIZE = (0.22, 0.20, CART_BASKET_FLOOR_Z - 0.05)
stand_box = FixedCuboid(
    prim_path="/World/CartStandBox",
    name="cart_stand_box",
    position=np.array([cart_center_xy[0], cart_center_xy[1], CART_BASKET_FLOOR_Z - STAND_BOX_SIZE[2] / 2.0]),
    scale=np.array(STAND_BOX_SIZE),
    color=np.array([0.55, 0.40, 0.25]),
)

area_light = UsdLux.SphereLight.Define(stage, "/World/PickAreaLight")
area_light.CreateRadiusAttr(0.3)
area_light.CreateIntensityAttr(60000)
UsdGeom.Xformable(area_light).AddTranslateOp().Set(Gf.Vec3d(cart_center_xy[0], cart_center_xy[1], 2.0))

# 마운트 오프셋이 없으므로(1절 참고) 베이스 시작 위치는 카트 중심에서 +X로 STANDOFF_X만큼
# 떨어진 지점, yaw=BASE_FACE_ROT_Z=90(섀시의 "짧은 축"이 카트를 향하도록, 위 docstring
# 4번 항목 참고 - 83번에서 검증된 제자리 회전을 활용해 28.py의 "긴 축 정면 접근" 관례를 깼다).
BASE_START_XY = (cart_center_xy[0] + STANDOFF_X, cart_center_xy[1])
chassis_path, hub_joint_paths = build_holonomic_base(stage, BASE_START_XY, BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)

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

# 28.py와 동일 시드 자세(joint_3/5=90도) - 전부 0도(특이점 근처)로 시작하지 않는다.
_init_joints = np.zeros(m0609_robot.num_dof)
if "joint_3" in m0609_robot.dof_names:
    _init_joints[m0609_robot.dof_names.index("joint_3")] = np.pi / 2
if "joint_5" in m0609_robot.dof_names:
    _init_joints[m0609_robot.dof_names.index("joint_5")] = np.pi / 2
m0609_robot.set_joint_positions(_init_joints)

# ---- 리프트 제어 (82/83.py와 동일한 텔레포트 패턴, 마운트 오프셋 없이 chassis_pos 그대로) ----
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
print("\n[안정화 완료] 카트/베이스 정지 상태\n", flush=True)

# ================= 박스를 카트 위에 낙하 =================
box_material = PhysicsMaterial(
    prim_path="/World/Physics_Materials/box_material",
    static_friction=CUBE_STATIC, dynamic_friction=CUBE_DYNAMIC, restitution=0.0,
)
box = DynamicCuboid(
    prim_path="/World/PickBox",
    name="pick_box",
    position=np.array([cart_center_xy[0], cart_center_xy[1], CART_BASKET_FLOOR_Z + CART_DROP_HEIGHT_ABOVE_FLOOR]),
    scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]),
    color=np.array([1.0, 0.15, 0.0]),
    mass=CUBE_MASS,
    physics_material=box_material,
)
step_hold(150)
box_pos, _ = box.get_world_pose()
print(f"[박스 낙하 완료] 최종 위치=({box_pos[0]:.3f},{box_pos[1]:.3f},{box_pos[2]:.3f})", flush=True)

viewport = vp_util.get_active_viewport()
results = {}


def snapshot(eye, target, fname):
    set_camera_view(eye=eye, target=target)
    step_hold(15)
    out = str(OUT_DIR / fname)
    vp_util.capture_viewport_to_file(viewport, out)
    step_hold(5)
    print(f"[SCREENSHOT] {out}", flush=True)


snapshot(eye=[cart_center_xy[0] - 0.6, cart_center_xy[1] - 1.0, 1.2],
         target=[box_pos[0], box_pos[1], box_pos[2]], fname="_pickverify_00_box_on_cart.png")

# ================= 리프트를 최고 높이로 (마운트가 낮아서 카트 바스켓까지 reach 확보용) =================
print(f"\n[리프트] 도킹({LIFT_MIN:.3f}) -> 최고({LIFT_MAX:.3f}) - 카트 바스켓(z={CART_BASKET_FLOOR_Z})까지 "
      f"reach 확보", flush=True)
move_lift_to(LIFT_MAX, steps=120)

# ================= RMPflow 컨트롤러 =================
controller = RMPFlowController(
    name="holo_pick_controller",
    robot_articulation=m0609_robot,
    urdf_path=M0609_URDF_PATH,
    robot_description_path=M0609_DESCRIPTION_PATH,
    rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
    end_effector_frame_name=EE_LINK_NAME,
)


def sync_rmp_base():
    """마운트 오프셋이 없으므로 chassis_pos를 그대로 RMPflow 베이스 포즈로 알려준다
    (28.py의 mounted_xy() 보정 불필요 - 1절 참고)."""
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


# ================= PICK 시도 =================
APPROACH_HOVER = 0.18
pick_hover_pos = (box_pos[0], box_pos[1], box_pos[2] + APPROACH_HOVER)
ee_pos, hover_err = move_link6(pick_hover_pos, label="박스 위 호버")
results["hover_err_m"] = float(hover_err)
snapshot(eye=[BASE_START_XY[0] - 0.6, BASE_START_XY[1] - 0.8, box_pos[2] + 0.5],
         target=[box_pos[0], box_pos[1], box_pos[2]], fname="_pickverify_01_hover.png")

grasp_pos = (box_pos[0], box_pos[1], box_pos[2] + TIP_LOCAL_OFFSET[2])
ee_pos, grasp_err = move_link6(grasp_pos, label="박스 그랩 위치 하강")
results["grasp_approach_err_m"] = float(grasp_err)
snapshot(eye=[BASE_START_XY[0] - 0.5, BASE_START_XY[1] - 0.6, box_pos[2] + 0.3],
         target=[box_pos[0], box_pos[1], box_pos[2]], fname="_pickverify_02_descended.png")

m0609_robot.gripper.close()
step_hold(30)
grasped = m0609_robot.gripper.is_closed()
results["grasped"] = bool(grasped)
print(f"\n[흡착 시도] gripper.is_closed()={grasped}\n", flush=True)
snapshot(eye=[BASE_START_XY[0] - 0.5, BASE_START_XY[1] - 0.6, box_pos[2] + 0.3],
         target=[box_pos[0], box_pos[1], box_pos[2]], fname="_pickverify_03_grasp_attempt.png")

lift_hover_pos = (box_pos[0], box_pos[1], box_pos[2] + 0.30)
ee_pos, lift_err = move_link6(lift_hover_pos, hold_gripper_closed=True, label="박스 들어올리기")
results["lift_err_m"] = float(lift_err)
box_pos_after, _ = box.get_world_pose()
lifted_with_box = bool(box_pos_after[2] > box_pos[2] + 0.15)
results["lifted_with_box"] = lifted_with_box
print(f"[들어올리기 확인] 박스 z {box_pos[2]:.3f} -> {box_pos_after[2]:.3f} "
      f"(0.15m 이상 같이 올라갔으면 성공) lifted_with_box={lifted_with_box}", flush=True)
snapshot(eye=[BASE_START_XY[0] - 0.6, BASE_START_XY[1] - 0.9, box_pos[2] + 0.6],
         target=[box_pos[0], box_pos[1], box_pos[2] + 0.3], fname="_pickverify_04_lifted.png")

# ================= 결과 저장 =================
results["all_ok"] = bool(results["grasped"] and results["lifted_with_box"])
result_path = OUT_DIR / "_pickverify_result.json"
result_path.write_text(json.dumps(results, indent=2))
print(f"\n[저장 완료] {result_path}", flush=True)
print(f"[전체 결과] {results}\n", flush=True)

print("[안내] 84번(PICK 검증) 완료. 창을 계속 열어두고 확인하거나 종료하세요.")
print("[안내] 창을 닫으면 결과가 자동 저장됩니다.\n", flush=True)

while simulation_app.is_running():
    step_hold(1)
    time.sleep(0.01)

SCENE_USD = str(OUT_DIR / "holonomic_pick_verify_scene.usd")
omni.usd.get_context().save_as_stage(SCENE_USD)
print(f"\n[저장 완료] {SCENE_USD}", flush=True)

simulation_app.close()
