"""
83.test_holonomic_drive.py

82번에서 만든 저상 메카넘 홀로노믹 베이스(실제 롤러 서브지오메트리 버전)를 실제로 움직여본다.
28.py의 drive_to_x(폐루프 속도제어 + 정체 감지)가 검증해둔 패턴을 x/y/yaw 3-DOF로 확장한
drive_to()를 만들고, 전진/후진/횡이동/회전/대각선/차량 접근 순서로 테스트한다.

버전 이력
----
1차: 원통 휠(롤러 없음) + Isaac Sim HolonomicController. 실측 결과 전진은 됐지만 횡이동이
    전혀 안 되거나 엉뚱한 축으로 새서 폐기(82번 docstring에 상세 기록). 원인은 (a) 롤러 없는
    원통은 일반 타이어처럼 옆으로 안 미끄러지는 물리적 한계, (b) HolonomicController API가
    mecanum_angles를 바꿔도 응답이 안 바뀌는 등 신뢰할 수 없는 동작.
2차(이 버전): 82번과 동일하게 실제 롤러 서브지오메트리(허브+롤러9개)로 교체하고, 표준 메카넘
    역기하학 공식(mecanum_wheel_speeds)을 직접 구현해서 HolonomicController를 대체했다.
3차(2026-07-23, 82번과 동기화): 82번이 M0609+리프트를 다시 넣었는데(리프트 텔레포트 방식,
    HOLONOMIC_BASE_HANDOFF.md 3-6/3-7절 참고) 이 스크립트는 그 이전(정적 용접-이전, M0609
    없음) 구조 그대로 남아있어서 실측 결과가 최신 82번과 안 맞았다(3-3-11번 교훈이 이 트랙에서
    두 번째로 재발한 것). mount_m0609()/set_lift_height()/step_hold()/move_lift_to()를 82번
    그대로 옮기고, 주행 루프(drive_to) 안의 모든 world.step 호출을 step_hold(1)로 바꿔서
    주행 중에도 리프트가 도킹 높이에 계속 붙잡혀 있게 했다 - 안 그러면 M0609가 텔레포트를
    못 받아서 그 자리에서 중력에 떨어진다. 리프트 상승(도킹 0.105m -> 최고 0.455m) 데모를
    트인 공간(원위치 복귀 직후)에서 스크린샷과 함께 확인하고(_drive_09/10_lift_up_*.png),
    차량 하부 진입 뒤에는 같은 상승 동작을 스크린샷 없이 수치로만 한 번 더 확인한다(차 밑은
    카메라가 차체에 가려 스크린샷이 전부 새까맣게 나오는 걸 실측으로 확인 - 처음엔 차량
    하부에서 스크린샷을 찍으려다 이 문제를 발견하고 트인 공간으로 옮겼다).

drive_to_x 대비 단순해진 점: Nova Carter는 차동구동이라 "방향을 잘못 짚으면 반대쪽으로 전속력
질주"하는 실패 모드가 있어서 저속 시험 구간이 필요했다. 홀로노믹은 구조적으로 그 실패 모드가
없어서 매 스텝 오차 기반 비례제어만으로 충분하다. 정체 감지(stall detection)는 28번과 동일하게
유지한다(장애물에 막혔는데 계속 힘만 주는 것 방지).

출력: 각 구간 CSV 궤적(_test_drive_trajectory.csv), 최종 결과 JSON(_test_drive_result.json),
구간별 스크린샷(M0609+리프트가 매 구간 화면에 함께 찍힌다 - results/holonomic_base/ 밑에 저장).
"""

from isaacsim import SimulationApp

import os
simulation_app = SimulationApp({"headless": os.environ.get("HEADLESS", "0") == "1"})

import csv
import json
import time
from pathlib import Path

import numpy as np
import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, UsdLux, UsdShade, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.rotations import quat_to_euler_angles
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view

_THIS_DIR = Path(__file__).resolve().parent
# 80/81/82번과 동일하게 이 트랙 전용 결과 폴더를 씀 (results/holonomic_base/)
OUT_DIR = _THIS_DIR / "results" / "holonomic_base"
OUT_DIR.mkdir(parents=True, exist_ok=True)

# ---------------- 82.py와 동일 (카트/차량/베이스 구성 재사용) ----------------
CART_USD = str(_THIS_DIR / "assets/Metal_Shopping_Cart.usdz")
CAR_USD = str(_THIS_DIR / "assets/Lexus_IS300_Trunk_Open_No_More_Hell_Room.usdz")
CART_POS = (0.0, 0.0, 0.0)
CART_EXTRA_SCALE = 0.55
CAR_POS = (5.0, 0.0, 0.0)
CAR_EXTRA_SCALE = 0.50
CAR_ROT_Z = 0.0
SDF_RESOLUTION = 256
# 82번에서 실측으로 잡은 버그와 동일한 수정: DRIVE_MAX_FORCE=1e6은 가벼운 휠 부품에 비해
# 터무니없이 커서 방향 전환 시 섀시가 뒤집히고 멀리 날아갔다(roll=179.93deg 등).
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 0.0, 50.0, 20.0

BASE_PATH = "/World/HoloBase"
CHASSIS_PATH = f"{BASE_PATH}/chassis"
BASE_START_XY = (-0.3, -1.5)
BASE_FACE_ROT_Z = 0.0

ROLLER_COUNT = 9
ROLLER_MASS = 0.02
HUB_MASS = 1.0
CHASSIS_MASS = 15.0

# 휠 위치를 원래 값(half_l≈0.17)으로 되돌리면서 WZ_SIGN도 그때 검증됐던 +1.0으로 복귀.
WZ_SIGN = 1.0

# ---- M0609 + 리프트 (82번 최신 구조와 동기화 - 이전엔 이 스크립트가 82번에서 M0609/리프트를
# 다시 넣은 걸 반영 못 하고 정적 용접-이전(M0609 없음) 구조로 남아있었다. 82번과 동일한 상수/
# 마운트 패턴을 그대로 가져온다) ----
M0609_DIR = _THIS_DIR.parent / "M0609"
M0609_USD = str(M0609_DIR / "Collected_m0609_vgp20_camera" / "m0609_vgp20_camera.usd")
M0609_MOUNT_Z_ABOVE_CHASSIS_TOP = 0.02  # 섀시 윗면에서 이만큼 띄워서 마운트(리프트 최저/도킹 높이)
LIFT_COLUMN_RADIUS = 0.045
LIFT_TRAVEL_M = 0.35        # 도킹 높이 대비 리프트가 올라가는 거리(82번과 동일)


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

# 3차 수정 (사용자 피드백): 휠을 섀시 끝쪽(half_l=0.425)으로 넓혔더니 회전 응답이 부호까지
# 뒤집히고 불안정해졌다(오프라인 격리 테스트로 확인 - 슬립이 지배적인 영역으로 넘어간 것으로
# 보임). "차체 길이는 좋다, 대신 주행이 원활해야 한다"는 우선순위에 따라 섀시 길이(1.0m)는
# 유지하되 휠 위치는 처음에 검증됐던 값(half_l≈0.17)으로 되돌린다 - 휠은 원래도 섀시
# 폭/길이보다 훨씬 작아서 "차체 안에 들어가는" 조건은 이 값에서도 그대로 만족한다.
CHASSIS_LENGTH_EXTENDED = 1.00
WHEEL_MOUNT_HALF_L = BASE_LENGTH / 2.0 - WHEEL_RADIUS * 0.6  # 원래 검증된 값으로 복귀
print(f"[치수 확정] WHEEL_RADIUS={WHEEL_RADIUS:.4f}m CHASSIS_BODY_HEIGHT={CHASSIS_BODY_HEIGHT:.4f}m "
      f"HUB_RADIUS={HUB_RADIUS:.4f}m ROLLER_RADIUS={ROLLER_RADIUS:.4f}m ROLLER_LENGTH={ROLLER_LENGTH:.4f}m", flush=True)
print(f"[치수 확정] CHASSIS_LENGTH_EXTENDED={CHASSIS_LENGTH_EXTENDED:.4f}m, WHEEL_MOUNT_HALF_L="
      f"{WHEEL_MOUNT_HALF_L:.4f}m (원래 검증된 위치로 복귀)", flush=True)


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
    """82.py와 동일 - 허브(구동) + 롤러 ROLLER_COUNT개(45도 자유회전)로 실제 메카넘 휠을 만든다."""
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

    # 섀시 바디는 CHASSIS_LENGTH_EXTENDED(늘어난 값), 휠 장착 위치(half_l, 아래)는 원래 length
    # 기준 그대로 - 바디만 휠 바깥으로 더 뻗어나간다(참고 이미지 비율).
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

    corner_signs = [(1, 1, 1), (1, -1, -1), (-1, 1, -1), (-1, -1, 1)]  # (sx, sy, chirality)
    corner_names = ["FL", "FR", "RL", "RR"]
    wheel_half_thickness_y = HUB_THICKNESS / 2.0 + ROLLER_LENGTH * 0.5 + ROLLER_RADIUS
    half_l = WHEEL_MOUNT_HALF_L  # 섀시가 길어져도 휠 장착 위치는 고정
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
    """82.py와 동일 - M0609 조인트 드라이브 강성을 재적용한다(마운트 직후 기본값으로
    리셋되는 걸 막기 위함)."""
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
    """82.py와 동일한 패턴 - 섀시 정중앙 위에 M0609(vgp20 흡착 그리퍼+카메라 버전)를
    리프트(텔레포트 방식)로 얹는다. 독립 articulation + 매 프레임 set_world_pose()로 리프트
    높이에 계속 붙잡아두는 방식(root_joint는 삭제, base_link에 ArticulationRootAPI를 새로
    적용, 섀시와의 충돌은 FilteredPairsAPI로 걸러서 두 바디가 겹쳐 있어도 서로 밀어내지
    않게 한다). 상세 배경은 82.py의 mount_m0609() docstring/HOLONOMIC_BASE_HANDOFF.md 3-6,
    3-7절 참고.
    반환: (m0609_path, base_link_path, lift_translate_op, lift_scale_op)"""
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

    # 버그 이력(2026-07-23, 스크린샷으로 실측 확인, 82.py와 동일한 수정) - 상세 배경은
    # 82.py의 mount_m0609() 주석 참고: BASE_PATH 밑에 두면 chassis_pos(월드 좌표)를 로컬
    # translate로 그대로 쓸 때 BASE_START_XY 오프셋이 이중으로 적용된다. /World 바로 밑에
    # 둬서 translate 값이 곧바로 월드 좌표가 되게 한다.
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


def mecanum_wheel_speeds(vx, vy, wz, wheel_radius, k):
    """표준 메카넘 역기하학 공식(순서 FL,FR,RL,RR).
    버그 이력(이 스크립트로 실측): 공식 그대로 vy를 쓰면 실제로는 반대 방향(-y)으로 움직였다
    (vy=+0.3을 150스텝 명령 -> dy=-1.121). 롤러 chirality convention이 교과서 공식과 반대라서
    - vy 부호를 반전해서 보정한다(82.py와 동일)."""
    vy = -vy
    return [
        (vx - vy - k * wz) / wheel_radius,
        (vx + vy + k * wz) / wheel_radius,
        (vx + vy - k * wz) / wheel_radius,
        (vx - vy + k * wz) / wheel_radius,
    ]


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

chassis_path, hub_joint_paths, k_factor = build_holonomic_base(stage, BASE_START_XY, BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)

# ---- M0609 + 리프트 마운트 (82번 최신 구조와 동기화, 3-3-11번 교훈 반영) ----
# 이론식(WHEEL_RADIUS+CHASSIS_BODY_HEIGHT/2) 대신 86번에서 실측(큐브 낙하+GUI 육안 튜닝)한 값 사용
MEASURED_CHASSIS_TOP_OFFSET = 0.0180
LIFT_MIN = MEASURED_CHASSIS_TOP_OFFSET + M0609_MOUNT_Z_ABOVE_CHASSIS_TOP  # 도킹(최저) 높이
LIFT_MAX = LIFT_MIN + LIFT_TRAVEL_M

M0609_ASSET_AVAILABLE = Path(M0609_USD).exists()
if M0609_ASSET_AVAILABLE:
    m0609_path, m0609_base_link_path, lift_translate_op, lift_scale_op = mount_m0609(stage, LIFT_MIN)
else:
    print(f"[건너뜀] M0609 에셋 없음({M0609_USD}) - 섀시/휠만으로 주행 테스트", flush=True)
    m0609_path = None
    lift_translate_op = lift_scale_op = None

light = UsdLux.SphereLight.Define(stage, "/World/HoloBaseLight")
light.CreateRadiusAttr(0.3)
light.CreateIntensityAttr(200000)
UsdGeom.Xformable(light).AddTranslateOp().Set(Gf.Vec3d(BASE_START_XY[0], BASE_START_XY[1], 1.5))
trunk_light = UsdLux.SphereLight.Define(stage, "/World/TrunkAreaLight")
trunk_light.CreateRadiusAttr(0.2)
trunk_light.CreateIntensityAttr(200000)
UsdGeom.Xformable(trunk_light).AddTranslateOp().Set(Gf.Vec3d(CAR_POS[0], 0.0, 1.8))

for _ in range(20):
    simulation_app.update()

# M0609는 독립 articulation(리프트 텔레포트 방식)이라 섀시(base_robot)와 별도로 초기화한다.
base_robot = SingleArticulation(prim_path=chassis_path, name="holo_base")
world.reset()
base_robot.initialize(physics_sim_view=world.physics_sim_view)
print(f"[초기화] 섀시 dof_names={base_robot.dof_names}", flush=True)

if M0609_ASSET_AVAILABLE:
    m0609_robot = SingleArticulation(prim_path=m0609_base_link_path, name="m0609_arm")
    m0609_robot.initialize(physics_sim_view=world.physics_sim_view)
    print(f"[초기화] M0609 dof_names={m0609_robot.dof_names} num_dof={m0609_robot.num_dof}", flush=True)
else:
    m0609_robot = None

hub_dof_indices = [base_robot.dof_names.index(Path(p).name) for p in hub_joint_paths]
print(f"[허브 DOF 인덱스] {hub_dof_indices} (FL,FR,RL,RR 순서)", flush=True)


def holo_forward(vx, vy, wz):
    speeds = mecanum_wheel_speeds(vx, vy, WZ_SIGN * wz, WHEEL_RADIUS, k_factor)
    return ArticulationAction(joint_velocities=speeds, joint_indices=hub_dof_indices)


# ---- 리프트 제어 (82.py와 동일한 텔레포트 패턴, 마운트 오프셋은 (0,0) 중앙) ----
lift_state = {"h": LIFT_MIN}


def set_lift_height(h):
    """섀시 현재 위치 바로 위 h로 m0609_robot을 텔레포트하고, 시각용 원기둥도 갱신한다.
    매 프레임 계속 불러줘야 리프트가 그 높이에 붙어있는다(안 그러면 중력으로 떨어짐) - 이게
    바로 82/83번을 동기화해야 하는 핵심 이유: 주행 루프(drive_to) 안에서도 매 스텝 이걸
    불러줘야 M0609가 섀시를 따라 이동한다."""
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
    """리프트를 lift_state['h']에 붙잡아둔 채로 물리 스텝을 n번 진행한다. M0609가 마운트돼
    있으면 주행 중에도 이 함수를 통해서만 world.step을 호출해야 리프트가 안 떨어진다."""
    for _ in range(n):
        if M0609_ASSET_AVAILABLE:
            set_lift_height(lift_state["h"])
        world.step(render=True)


def move_lift_to(target_h, steps=90):
    """리프트를 target_h까지 부드럽게(선형 보간) 움직인 뒤 lift_state를 갱신한다."""
    start_h = lift_state["h"]
    for i in range(steps):
        h = start_h + (target_h - start_h) * (i + 1) / steps
        set_lift_height(h)
        world.step(render=True)
    lift_state["h"] = target_h
    print(f"[리프트] {start_h:.3f} -> {target_h:.3f}", flush=True)


step_hold(60)

viewport = vp_util.get_active_viewport()
trajectory_log = []


def snapshot(eye, target, fname):
    set_camera_view(eye=eye, target=target)
    step_hold(15)
    out = str(OUT_DIR / fname)
    vp_util.capture_viewport_to_file(viewport, out)
    step_hold(5)
    print(f"[SCREENSHOT] {out}", flush=True)


SMOOTH_ALPHA = 0.12  # 속도 명령 저역통과 필터 계수 - 낮을수록 부드럽지만 반응은 느려짐
_smooth_state = {"vx": 0.0, "vy": 0.0, "wz": 0.0}


def drive_to(target_x=None, target_y=None, target_yaw_deg=None, tolerance_xy=0.03, tolerance_yaw_deg=2.0,
             max_speed=0.4, max_wz=0.2, kp_xy=1.8, kp_yaw=0.25, max_steps=2500, label=""):
    """세계좌표 목표(x,y,yaw)로 폐루프 비례제어. None인 축은 현재값 유지(그 축은 제어 안 함).
    버그 이력: kp_yaw=2.0/max_wz=1.2였을 때 회전 명령 wz=0.5(rad/s)를 줬더니 실제 각속도가
    그 3배(~85deg/s)로 나왔다(오프라인 격리 테스트로 실측: 300스텝 만에 yaw가 여러 바퀴
    돎). 실제 응답이 가정보다 훨씬 빠른데 게인은 그걸 모르고 세게 밀어붙이니 진동->발산
    (roll이 179도까지 뒤집힌 사례도 있었음). 게인을 낮춰서 재검증(kp_yaw=0.5, max_wz=0.35) -
    회전 테스트에서 tilt 1도 이내로 안정적으로 수렴하는 것 확인함.

    버그 이력 2(사용자가 시각적으로 확인): 매 스텝 순수 비례오차로 vx/vy/wz를 다시 계산해서
    그대로 명령하면, 오차가 요동칠 때(특히 새 drive_to() 호출 직후나 목표 근처에서 부호가
    바뀔 때) 명령값이 스텝 사이에 불연속적으로 튀어서 "공중제비" 치듯 움직이는 것처럼
    보였다. SMOOTH_ALPHA로 저역통과 필터를 걸어(이전 프레임 명령과 새 명령을 블렌딩)
    부드럽게 만든다 - 반응 속도는 약간 느려지지만(수렴까지 걸리는 스텝 수 증가) 훨씬
    자연스럽게 움직인다."""
    start_pos, start_quat = base_robot.get_world_pose()
    start_yaw = float(np.degrees(quat_to_euler_angles(start_quat)[2]))
    tx = target_x if target_x is not None else float(start_pos[0])
    ty = target_y if target_y is not None else float(start_pos[1])
    tyaw = target_yaw_deg if target_yaw_deg is not None else start_yaw
    print(f"\n[주행 시작]{' ' + label if label else ''} 목표=({tx:.3f},{ty:.3f},{tyaw:.1f}deg)", flush=True)

    STALL_WINDOW = 150
    STALL_MIN_PROGRESS = 0.008
    last_check_pos = np.array([float(start_pos[0]), float(start_pos[1])])
    stalled = False
    step = 0
    for step in range(1, max_steps + 1):
        pos, quat = base_robot.get_world_pose()
        yaw_deg = float(np.degrees(quat_to_euler_angles(quat)[2]))
        ex_w = tx - float(pos[0])
        ey_w = ty - float(pos[1])
        eyaw = ((tyaw - yaw_deg + 180) % 360) - 180
        if abs(ex_w) < tolerance_xy and abs(ey_w) < tolerance_xy and abs(eyaw) < tolerance_yaw_deg:
            break
        yaw_rad = np.radians(yaw_deg)
        ex_l = ex_w * np.cos(yaw_rad) + ey_w * np.sin(yaw_rad)
        ey_l = -ex_w * np.sin(yaw_rad) + ey_w * np.cos(yaw_rad)
        vx_target = float(np.clip(kp_xy * ex_l, -max_speed, max_speed))
        vy_target = float(np.clip(kp_xy * ey_l, -max_speed, max_speed))
        wz_target = float(np.clip(np.radians(kp_yaw * eyaw), -max_wz, max_wz))
        _smooth_state["vx"] += SMOOTH_ALPHA * (vx_target - _smooth_state["vx"])
        _smooth_state["vy"] += SMOOTH_ALPHA * (vy_target - _smooth_state["vy"])
        _smooth_state["wz"] += SMOOTH_ALPHA * (wz_target - _smooth_state["wz"])
        base_robot.apply_action(holo_forward(_smooth_state["vx"], _smooth_state["vy"], _smooth_state["wz"]))
        step_hold(1)  # 82번과 동기화: 주행 중에도 리프트를 lift_state["h"]에 계속 붙잡아둠

        if step % STALL_WINDOW == 0:
            cur = np.array([float(pos[0]), float(pos[1])])
            progress = float(np.linalg.norm(cur - last_check_pos))
            need_xy = abs(ex_w) > tolerance_xy or abs(ey_w) > tolerance_xy
            if progress < STALL_MIN_PROGRESS and need_xy:
                stalled = True
                print(f"  [정체 감지] 최근 {STALL_WINDOW}스텝 동안 {progress:.4f}m밖에 못 움직임 - 중단", flush=True)
                break
            last_check_pos = cur
            trajectory_log.append({"label": label, "step": step, "x": float(pos[0]), "y": float(pos[1]), "yaw_deg": yaw_deg})

    # 정지도 급정거 대신 같은 저역통과 필터로 부드럽게 0까지 램프다운
    for _ in range(30):
        _smooth_state["vx"] += SMOOTH_ALPHA * (0.0 - _smooth_state["vx"])
        _smooth_state["vy"] += SMOOTH_ALPHA * (0.0 - _smooth_state["vy"])
        _smooth_state["wz"] += SMOOTH_ALPHA * (0.0 - _smooth_state["wz"])
        base_robot.apply_action(holo_forward(_smooth_state["vx"], _smooth_state["vy"], _smooth_state["wz"]))
        step_hold(1)
    final_pos, final_quat = base_robot.get_world_pose()
    final_yaw = float(np.degrees(quat_to_euler_angles(final_quat)[2]))
    roll, pitch, _ = quat_to_euler_angles(final_quat)
    ok = (not stalled and abs(tx - float(final_pos[0])) < tolerance_xy
          and abs(ty - float(final_pos[1])) < tolerance_xy
          and abs(((tyaw - final_yaw + 180) % 360) - 180) < tolerance_yaw_deg)
    print(f"[주행 완료]{' ' + label if label else ''} {step}스텝, 최종=({final_pos[0]:.3f},{final_pos[1]:.3f},"
          f"{final_yaw:.1f}deg) tilt(roll={np.degrees(roll):.2f},pitch={np.degrees(pitch):.2f}) "
          f"정체={stalled} 성공={ok}", flush=True)
    trajectory_log.append({"label": label, "step": step, "x": float(final_pos[0]), "y": float(final_pos[1]), "yaw_deg": final_yaw})
    return final_pos, final_yaw, ok


# ================= 테스트 시퀀스 =================
results = {}
x0, y0 = BASE_START_XY
snapshot(eye=[x0, y0 + 1.0, 0.8], target=[x0, y0, 0.05], fname="_drive_00_start.png")

_, _, results["forward_0.5m"] = drive_to(target_x=x0 + 0.5, label="전진 0.5m")
snapshot(eye=[x0 + 0.5, y0 + 1.0, 0.8], target=[x0 + 0.5, y0, 0.05], fname="_drive_01_forward.png")

_, _, results["backward_to_start"] = drive_to(target_x=x0, label="후진 원위치")

_, _, results["lateral_+0.4m"] = drive_to(target_y=y0 + 0.4, label="횡이동 +0.4m")
snapshot(eye=[x0 + 1.0, y0 + 0.4, 0.8], target=[x0, y0 + 0.4, 0.05], fname="_drive_02_lateral.png")

_, _, results["lateral_back"] = drive_to(target_y=y0, label="횡이동 원위치")

_, final_yaw, results["rotate_+90deg"] = drive_to(target_yaw_deg=90.0, label="제자리 회전 +90deg")
snapshot(eye=[x0, y0 + 1.0, 0.8], target=[x0, y0, 0.05], fname="_drive_03_rotate90.png")
print(f"[회전 확인] 목표 +90deg, 실제 최종 yaw={final_yaw:.1f}deg - 반대 방향으로 돌았다면 "
      f"WZ_SIGN을 -1로 바꿔서 재실행할 것", flush=True)

_, _, results["rotate_back_0deg"] = drive_to(target_yaw_deg=0.0, label="회전 원위치")

_, _, results["diagonal"] = drive_to(target_x=x0 + 0.3, target_y=y0 + 0.3, label="대각선 이동")
snapshot(eye=[x0 + 1.2, y0 + 1.2, 0.9], target=[x0 + 0.3, y0 + 0.3, 0.05], fname="_drive_04_diagonal.png")

_, _, results["return_to_start"] = drive_to(target_x=x0, target_y=y0, target_yaw_deg=0.0, label="원위치 복귀")

# ---- 리프트 상승 데모 (82.py와 동일 위치: 차 밑으로 들어가기 전, 트인 공간에서 먼저 확인).
# 첫 시도에서는 이 데모를 차량 하부 진입 이후에 넣었는데, 그 자리는 카메라가 차량 SDF 바디
# 안쪽/근접한 곳을 비추게 돼서 스크린샷이 전부 새까맣게 나왔다(차체가 주변을 가려서 광원이
# 안 닿음 - _drive_06/07/08도 원래부터 이 문제가 있었다). 실측(스크린샷 확인)으로 발견한
# 문제라 트인 공간(BASE_START_XY 근처)으로 옮겨서 실제로 보이는 스크린샷을 남긴다. 차량
# 하부에서의 리프트 안정성은 아래 undercar_deep 이후 구간에서 스크린샷 없이 수치로만 확인. ----
if M0609_ASSET_AVAILABLE:
    print(f"\n[리프트 테스트] 트인 공간에서 {LIFT_MIN:.3f} -> {LIFT_MAX:.3f} 상승\n", flush=True)
    move_lift_to(LIFT_MAX, steps=120)
    pos_up, quat_up = base_robot.get_world_pose()
    roll_up, pitch_up, _ = quat_to_euler_angles(quat_up)
    lift_tilt_ok = abs(np.degrees(roll_up)) < 2.0 and abs(np.degrees(pitch_up)) < 2.0
    results["lift_up_open_area"] = lift_tilt_ok
    print(f"[리프트 결과] chassis pos={np.round(pos_up,3)} roll={np.degrees(roll_up):.3f}deg "
          f"pitch={np.degrees(pitch_up):.3f}deg tilt_ok={lift_tilt_ok}", flush=True)
    snapshot(eye=[x0, y0 + 1.7, 0.9], target=[x0, y0, LIFT_MAX * 0.7], fname="_drive_09_lift_up_side.png")
    snapshot(eye=[x0 + 1.7, y0, 0.9], target=[x0, y0, LIFT_MAX * 0.7], fname="_drive_10_lift_up_front.png")
    move_lift_to(LIFT_MIN, steps=90)
    print("[리프트] 도킹 높이로 복귀 완료\n", flush=True)

# ---- 차량 접근 (81번이 계산한 j1_x까지, y=0 중심선으로 저속 접근) ----
print(f"\n[차량 접근] 목표 x={APPROACH_TARGET_X:.3f} (81번 추천 조합의 j1_x)", flush=True)
_, _, results["approach_vehicle"] = drive_to(
    target_x=APPROACH_TARGET_X, target_y=0.0, target_yaw_deg=0.0,
    max_speed=0.15, label="차량 접근(저속)",
)
final_pos, final_quat = base_robot.get_world_pose()
roll, pitch, _ = quat_to_euler_angles(final_quat)
tilt_ok = abs(np.degrees(roll)) < 2.0 and abs(np.degrees(pitch)) < 2.0
print(f"[차량 접근 결과] 최종 위치={np.round(final_pos,3)} tilt(roll={np.degrees(roll):.2f},"
      f"pitch={np.degrees(pitch):.2f}) tilt_ok={tilt_ok}", flush=True)
snapshot(eye=[APPROACH_TARGET_X - 1.0, -1.0, 0.8], target=[APPROACH_TARGET_X, 0.0, 0.1], fname="_drive_05_vehicle_approach.png")

# ---- 차량 하부로 더 깊이 진입 (80번 실측: car_min[0]=2.843부터 x=4.84까지 중앙 대역 clearance
# 0.20m+ 확인됨 - 이 베이스 전체 높이(~0.10m)는 그 안에 여유있게 들어감. 사용자 요청: 실제로
# 차 밑으로 들어갈 수 있는지 동작으로 확인) ----
UNDERCAR_DEEP_X = 4.5  # 80번 프로브 범위(2.84~4.84) 안쪽, 안전 clearance 확인된 구간
print(f"\n[차량 하부 깊이 진입] 목표 x={UNDERCAR_DEEP_X:.3f} (80번 실측 안전 구간 안쪽)", flush=True)
_, _, results["undercar_deep"] = drive_to(
    target_x=UNDERCAR_DEEP_X, target_y=0.0, target_yaw_deg=0.0,
    max_speed=0.15, label="차량 하부 깊이 진입(저속)",
)
final_pos, final_quat = base_robot.get_world_pose()
roll, pitch, _ = quat_to_euler_angles(final_quat)
undercar_tilt_ok = abs(np.degrees(roll)) < 2.0 and abs(np.degrees(pitch)) < 2.0
print(f"[차량 하부 진입 결과] 최종 위치={np.round(final_pos,3)} tilt(roll={np.degrees(roll):.2f},"
      f"pitch={np.degrees(pitch):.2f}) tilt_ok={undercar_tilt_ok}, 차량 뒷면(x=2.843)부터 "
      f"진입 깊이={float(final_pos[0])-2.843:.3f}m", flush=True)
snapshot(eye=[float(final_pos[0]) - 0.8, -0.9, 0.35], target=[float(final_pos[0]), 0.0, 0.05], fname="_drive_06_undercar_side.png")
snapshot(eye=[float(final_pos[0]), 0.0, 1.8], target=[float(final_pos[0]), 0.0, 0.05], fname="_drive_07_undercar_top.png")
snapshot(eye=[float(final_pos[0]) - 1.5, -2.0, 1.0], target=[float(final_pos[0]), 0.0, 0.3], fname="_drive_08_undercar_wide.png")

# ---- 리프트 안정성 재확인 (차량 하부 상태, 스크린샷 없음) ----
# 여기서도 처음엔 스크린샷을 찍었는데, 차 밑에 들어간 위치라 카메라가 차량 SDF 바디에
# 가려진 어두운/근접한 면만 비춰서 스크린샷이 전부 새까맣게 나왔다(실측으로 확인 -
# _drive_06/07/08도 원래부터 같은 문제가 있었다. 위 "리프트 상승 데모" 절 참고). 실제로
# 보이는 스크린샷은 트인 공간에서 이미 찍었으니(_drive_09/10_lift_up_*.png), 여기서는
# "차량 하부에서도 리프트를 올렸을 때 섀시가 안정적인가"라는 수치 검증만 한다.
if M0609_ASSET_AVAILABLE:
    print(f"\n[리프트 테스트] 차량 하부 진입 상태에서 {LIFT_MIN:.3f} -> {LIFT_MAX:.3f} 상승\n", flush=True)
    move_lift_to(LIFT_MAX, steps=120)
    pos_up, quat_up = base_robot.get_world_pose()
    roll_up, pitch_up, _ = quat_to_euler_angles(quat_up)
    lift_tilt_ok = abs(np.degrees(roll_up)) < 2.0 and abs(np.degrees(pitch_up)) < 2.0
    results["lift_up_after_undercar"] = lift_tilt_ok
    print(f"[리프트 결과] chassis pos={np.round(pos_up,3)} roll={np.degrees(roll_up):.3f}deg "
          f"pitch={np.degrees(pitch_up):.3f}deg tilt_ok={lift_tilt_ok} (차량 하부에서 리프트 "
          f"올려도 섀시가 안정적인지 확인)", flush=True)
    move_lift_to(LIFT_MIN, steps=90)
    print("[리프트] 도킹 높이로 복귀 완료", flush=True)

# ================= 결과 저장 =================
csv_path = OUT_DIR / "_test_drive_trajectory.csv"
with csv_path.open("w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=["label", "step", "x", "y", "yaw_deg"])
    writer.writeheader()
    writer.writerows(trajectory_log)
print(f"\n[저장 완료] {csv_path}", flush=True)

all_ok = all(results.values())
result_json = {
    "results": {k: bool(v) for k, v in results.items()},
    "all_passed": bool(all_ok),
    "final_position": [float(v) for v in final_pos],
    "final_tilt_deg": {"roll": float(np.degrees(roll)), "pitch": float(np.degrees(pitch))},
    "tilt_ok": bool(tilt_ok),
    "wz_sign_used": WZ_SIGN,
    "note": "rotate_+90deg 방향이 스크린샷(_drive_03_rotate90.png)과 다르면 WZ_SIGN=-1로 재실행 필요",
}
json_path = OUT_DIR / "_test_drive_result.json"
json_path.write_text(json.dumps(result_json, indent=2))
print(f"[저장 완료] {json_path}", flush=True)
print(f"\n[전체 결과] {results}", flush=True)
print(f"[전체 통과] {all_ok}\n", flush=True)

print("[안내] 83번(주행 테스트) 완료. 창을 계속 열어두고 확인하거나 종료하세요.")
print("[안내] 창을 닫으면 결과가 자동 저장됩니다.\n", flush=True)

while simulation_app.is_running():
    simulation_app.update()
    time.sleep(0.01)

SCENE_USD = str(OUT_DIR / "holonomic_base_drive_scene.usd")
omni.usd.get_context().save_as_stage(SCENE_USD)
print(f"\n[저장 완료] {SCENE_USD}", flush=True)

simulation_app.close()
