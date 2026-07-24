"""
82.build_low_profile_holonomic_base.py

81번이 추천한 치수(_evaluate_low_profile_base.csv 1행)로 저상 대칭 박스 섀시 + 진짜 메카넘 휠 4개를
실제 articulation으로 만든다. Nova Carter를 교체하는 새 베이스로, 리프트/M0609를 나중에(84번)
정중앙(MOUNT_LOCAL_OFFSET=0,0)에 얹을 수 있게 하는 게 목적이다 - 28.py가 겪었던 "섀시가
비대칭이라 마운트를 어디 둬도 한쪽 reach를 손해본다" 문제를 구조적으로 없애는 핵심 변경.

버전 이력 (83번 실측으로 밝혀진 문제들, 전부 이 버전에서 반영)
----
1차: 원통 하나 + 등방 고마찰로 메카넘 휠을 근사하고 Isaac Sim HolonomicController로 구동.
    실측 결과: 전진(vx)은 완벽하게 됐지만 횡이동(vy)은 거의 안 움직이거나 엉뚱한 축으로
    샜다. 원인은 두 가지가 겹쳐 있었다:
    (a) 물리: 실제 롤러 형상 없이 "매끈한 원통 + 높은 마찰"이면, 일반 타이어처럼 옆으로
        미끄러지길 거부한다 - 메카넘의 핵심(45도 롤러가 대각선으로 미끄러지는 것) 자체가
        원천적으로 불가능한 구조였다.
    (b) API: Isaac Sim HolonomicController.forward(command=[a,b,wz])의 a,b 채널이 기대한
        vy/vx로 안 나뉘고, mecanum_angles 값을 바꿔도 응답이 안 바뀌는 등 이해 못 할 동작을
        보여서(오프라인 스윕으로 확인) 신뢰할 수 없다고 판단, 표준 메카넘 역기하학 공식을
        직접 구현해서 대체했다(mecanum_wheel_speeds 함수 - 검증 용이하고 투명함).
2차(이 버전): 사용자가 "실제 롤러 지오메트리 구현"을 선택. 각 휠을 허브(구동, RevoluteJoint+
    DriveAPI) + 롤러 9개(허브 테두리에 45도로 박힌 캡슐, 각자 축을 중심으로 자유회전하는
    수동 RevoluteJoint, 구동 없음)로 만든다. 허브 자체는 콜리전이 없고, 롤러들만 지면과
    접촉한다 - 이게 실제 메카넘 휠의 옆미끄러짐 물리를 만들어낸다.

이 스크립트가 하는 일
----
1. 81번 CSV 최상단 추천 치수를 읽어 BASE_LENGTH/WIDTH/HEIGHT 결정 (없으면 플레이스홀더+경고)
2. 카트/차량 배치 (28.py와 동일 좌표, add_asset/add_sdf_collision 재사용)
3. 저상 박스 섀시(ArticulationRootAPI) + 4개 메카넘 휠(허브+롤러9개) 생성
4. mecanum_wheel_speeds()로 (vx,vy,wz) -> 4휠 속도 매핑, 자가진단 출력
5. world.reset() + 정지 안정화, 섀시 기울어짐 확인
6. 스크린샷 + 씬 저장 (holonomic_base_scene.usd)
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import csv
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
# 80/81/83번과 동일하게 이 트랙 전용 결과 폴더를 씀 (results/holonomic_base/)
OUT_DIR = _THIS_DIR / "results" / "holonomic_base"
OUT_DIR.mkdir(parents=True, exist_ok=True)

# ---------------- 28.py/8.py와 동일한 카트/차량 배치 (재사용) ----------------
CART_USD = str(_THIS_DIR / "assets/Metal_Shopping_Cart.usdz")
CAR_USD = str(_THIS_DIR / "assets/Lexus_IS300_Trunk_Open_No_More_Hell_Room.usdz")
CART_POS = (0.0, 0.0, 0.0)
CART_EXTRA_SCALE = 0.55
CAR_POS = (5.0, 0.0, 0.0)
CAR_EXTRA_SCALE = 0.50
CAR_ROT_Z = 0.0
SDF_RESOLUTION = 256
# 버그 이력(83번 실측): DRIVE_MAX_FORCE=1e6은 허브(1kg)/롤러(0.02kg) 같은 가벼운 부품 기준으로
# 터무니없이 큰 값이라(전체 조립체도 20kg 안팎), 방향 전환 시 한 스텝 안에 거의 무한대에
# 가까운 토크가 걸려 섀시가 뒤집히고 멀리 날아가버렸다(횡이동 테스트 중 roll=179.93deg,
# 위치가 (-12.7,-16.3)까지 튐). 실제 필요한 토크(가속도 1~5 m/s^2 기준 휠당 1N·m 내외)의
# 수십 배 여유만 남기고 대폭 낮춘다.
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 0.0, 50.0, 20.0  # 허브 속도구동 (stiffness=0, damping이 속도게인)

BASE_PATH = "/World/HoloBase"
CHASSIS_PATH = f"{BASE_PATH}/chassis"
BASE_START_XY = (-0.3, -1.5)  # 28.py의 Robot_Waiting_Zone과 동일 위치에서 시작
BASE_FACE_ROT_Z = 0.0

ROLLER_COUNT = 9              # 허브 테두리에 박는 롤러 개수 (실측 프로토타입에서 확인한 값)
ROLLER_MASS = 0.02
HUB_MASS = 1.0
CHASSIS_MASS = 15.0   # 섀시 자체 무게(잠정) - 84번에서 리프트+M0609(~30kg+) 얹으면 총 페이로드 재검토 필요


def load_recommended_dims():
    csv_path = OUT_DIR / "_evaluate_low_profile_base.csv"
    if csv_path.exists():
        with csv_path.open() as f:
            rows = [r for r in csv.DictReader(f) if r["feasible"] == "True"]
        if rows:
            rows.sort(key=lambda r: (-float(r["trunk_insertion_depth_m"]), float(r["base_length"])))
            best = rows[0]
            length, width, height = float(best["base_length"]), float(best["base_width"]), float(best["base_height"])
            print(f"[치수] 81번 추천 채택: length={length} width={width} height={height} "
                  f"(예상 진입깊이={best['trunk_insertion_depth_m']}m)", flush=True)
            return length, width, height
    print("[경고] _evaluate_low_profile_base.csv 없음 - 81.evaluate_low_profile_base.py를 먼저 "
          "실행하세요. 지금은 플레이스홀더 치수(0.50 x 0.50 x 0.15)로 진행합니다.", flush=True)
    return 0.50, 0.50, 0.15


BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT = load_recommended_dims()

# 83번 실측 버그: WHEEL_RADIUS가 섀시 반높이(BASE_HEIGHT/2)보다 작으면 섀시 배가 땅에 끌리고
# 휠은 허공에서 헛돈다. 휠이 실제 접지점이 되려면 WHEEL_RADIUS >= BASE_HEIGHT/2 여야 한다.
WHEEL_RADIUS = max(0.05, BASE_HEIGHT / 2.0)
CHASSIS_BODY_HEIGHT = min(BASE_HEIGHT, 2 * WHEEL_RADIUS) * 0.7
ROLLER_RADIUS = WHEEL_RADIUS * 0.22
ROLLER_LENGTH = (2 * np.pi * (WHEEL_RADIUS - ROLLER_RADIUS)) / ROLLER_COUNT * 1.15
HUB_RADIUS = WHEEL_RADIUS - ROLLER_RADIUS * 0.85
HUB_THICKNESS = WHEEL_RADIUS * 0.55

# ---- 3차 수정 (사용자 피드백): "차체 길이는 좋은데 주행이 원활하지 않다" ----
# 2차에서 휠을 섀시 끝쪽(half_l=0.425)으로 넓혔더니 회전 응답이 부호까지 뒤집히고
# 불안정해졌다(오프라인 격리 테스트로 확인 - 슬립이 지배적인 영역으로 넘어간 것으로 보임).
# "주행 원활함"이 우선순위이므로 섀시 길이(1.0m)는 유지하되 휠 위치는 처음 검증됐던 값
# (half_l≈0.17)으로 되돌린다 - 휠은 원래도 섀시 폭/길이보다 훨씬 작아서 "차체 안에 들어가는"
# 조건은 이 값에서도 그대로 만족한다.
CHASSIS_LENGTH_EXTENDED = 1.00
WHEEL_MOUNT_HALF_L = BASE_LENGTH / 2.0 - WHEEL_RADIUS * 0.6  # 원래 검증된 값으로 복귀
print(f"[치수 확정] WHEEL_RADIUS={WHEEL_RADIUS:.4f}m CHASSIS_BODY_HEIGHT={CHASSIS_BODY_HEIGHT:.4f}m "
      f"HUB_RADIUS={HUB_RADIUS:.4f}m ROLLER_RADIUS={ROLLER_RADIUS:.4f}m ROLLER_LENGTH={ROLLER_LENGTH:.4f}m", flush=True)
print(f"[치수 확정] CHASSIS_LENGTH_EXTENDED={CHASSIS_LENGTH_EXTENDED:.4f}m, WHEEL_MOUNT_HALF_L="
      f"{WHEEL_MOUNT_HALF_L:.4f}m (원래 검증된 위치로 복귀)", flush=True)

# ---- M0609 장착 (사용자 요청: 무게중심 안정성 확인용, RMPflow/그리퍼 로직 없이 정적 결합만) ----
M0609_DIR = _THIS_DIR.parent / "M0609"
M0609_USD = str(M0609_DIR / "Collected_m0609_vgp20_camera" / "m0609_vgp20_camera.usd")
M0609_MOUNT_Z_ABOVE_CHASSIS_TOP = 0.02  # 섀시 윗면에서 이만큼 띄워서 마운트(리프트 최저/도킹 높이)

# ---- 리프트 (사용자 요청으로 재추가: "새 홀로노믹 베이스에 리프트 다시 추가해줘") ----
# 방식은 27/28번에서 검증된 텔레포트 그대로 재사용한다(플랜 문서에 이미 확정: "매 프레임
# set_world_pose 텔레포트 + 시각 원기둥", 진짜 PrismaticJoint는 시도하지 않음).
LIFT_COLUMN_RADIUS = 0.045  # 저상 베이스 비례에 맞춰 28번(0.06)보다 살짝 가늘게
LIFT_TRAVEL_M = 0.35        # 도킹 높이 대비 리프트가 올라가는 거리(데모용, 저상 베이스 규모에 비례)


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
    """v_from을 v_to로 회전시키는 최단 회전 쿼터니언(Gf.Quatf, wxyz)을 계산한다.
    롤러의 기본 축(X)을 45도 기울어진 실제 방향으로 돌리는 데 쓴다."""
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
    """허브(구동) + 롤러 ROLLER_COUNT개(각자 45도로 기울어진 축을 중심으로 자유 회전, 무구동)로
    실제 메카넘 휠을 만든다. chirality=+1/-1로 롤러가 기울어지는 방향(좌/우 대각선)을 정한다 -
    표준 "X 패턴" 메카넘은 대각선 위치(FL/RR 대 FR/RL)에 따라 이 값이 반대여야 한다.
    name(FL/FR/RL/RR)은 조인트 프림 이름에 넣어 4개 휠의 허브/롤러 조인트가 서로 겹치지 않게
    한다 - 처음에 전부 "joint_hub"로 똑같이 지었더니 dof_names에서 4개가 구분이 안 돼서
    (Path(p).name으로 찾으면 전부 첫 번째 항목만 잡힘) apply_action이 엉뚱한 DOF를 건드렸다.
    반환: hub_joint_path(구동 조인트, 이걸로 apply_action)"""
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
    # 허브 자체는 콜리전 없음 - 롤러들만 지면과 접촉(실제 메카넘 휠 구조)

    hub_joint_path = f"{wheel_root_path}/joint_hub_{name}"
    hub_joint = UsdPhysics.RevoluteJoint.Define(stage, hub_joint_path)
    hub_joint.CreateAxisAttr("Y")
    hub_joint.CreateBody0Rel().SetTargets([Sdf.Path(chassis_path)])
    hub_joint.CreateBody1Rel().SetTargets([Sdf.Path(hub_path)])
    hub_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(wx, wy, 0.0))  # chassis_root 프레임은 이미 휠 축 높이
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
        rjoint.CreateAxisAttr("X")  # 롤러 로컬 X(=orient 적용 후 roller_axis 방향)를 축으로 자유 회전
        rjoint.CreateBody0Rel().SetTargets([Sdf.Path(hub_path)])
        rjoint.CreateBody1Rel().SetTargets([Sdf.Path(roller_path)])
        rjoint.CreateLocalPos0Attr().Set(Gf.Vec3f(*rpos))
        rjoint.CreateLocalRot0Attr().Set(quat)
        rjoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        rjoint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        # 구동 없음(DriveAPI 미적용) - 접촉력에 의해서만 수동으로 자유 회전

    return hub_joint_path


def build_holonomic_base(stage, start_xy, length, width, height):
    """저상 대칭 박스 섀시(ArticulationRootAPI) + 4개 진짜 메카넘 휠(허브+롤러9개)을 만든다.
    반환: (chassis_path, hub_joint_names[FL,FR,RL,RR], wheel_positions_local, k_factor)
    """
    base_xform = UsdGeom.Xform.Define(stage, BASE_PATH)
    base_xform.ClearXformOpOrder()
    base_xform.AddTranslateOp().Set(Gf.Vec3d(start_xy[0], start_xy[1], 0.0))
    base_xform.AddRotateZOp().Set(BASE_FACE_ROT_Z)

    # ---- 섀시 ----
    # 물리(RigidBody/ArticulationRoot/조인트 기준 프레임)는 스케일 없는 부모 Xform에 두고,
    # 시각/충돌용 Cube는 스케일이 걸린 채로 그 자식으로만 둔다(비균등 스케일 프림을 조인트
    # body로 쓰면 localPos가 왜곡되는 PhysX 문제 회피, 82번 1차 버전에서 실측으로 확인).
    chassis_root = UsdGeom.Xform.Define(stage, CHASSIS_PATH)
    chassis_root.ClearXformOpOrder()
    chassis_root.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, WHEEL_RADIUS))
    chassis_prim = chassis_root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(chassis_prim)
    UsdPhysics.MassAPI.Apply(chassis_prim).CreateMassAttr().Set(CHASSIS_MASS)
    UsdPhysics.ArticulationRootAPI.Apply(chassis_prim)

    # 섀시 바디 길이는 CHASSIS_LENGTH_EXTENDED(늘어난 값)을 쓴다 - 휠 장착 위치(half_l, 아래)는
    # 원래 length 기준 그대로라 바디만 휠 바깥으로 더 뻗어나간다(참고 이미지처럼).
    chassis_geom_path = f"{CHASSIS_PATH}/geom"
    chassis_geom = UsdGeom.Cube.Define(stage, chassis_geom_path)
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

    # chirality: 대각선 위치(FL/RR 대 FR/RL)에 따라 롤러 기울기 방향이 반대여야 표준 "X 패턴"
    # 메카넘이 된다 - FL/RR은 +1, FR/RL은 -1 (39/83번에서 쓰던 MECANUM_ANGLES_DEG 부호 관례 계승)
    corner_signs = [(1, 1, 1), (1, -1, -1), (-1, 1, -1), (-1, -1, 1)]  # (sx, sy, chirality)
    corner_names = ["FL", "FR", "RL", "RR"]
    # 버그 이력: half_w에 ROLLER_LENGTH(롤러가 자기 축 방향으로 얼마나 긴지)를 그대로 썼더니
    # 실측(82번 이 버전 최초 실행)에서 chassis pos가 NaN으로 터졌다 - ROLLER_LENGTH는 45도로
    # 기울어진 "롤러 자체의 길이"이지 "휠이 Y(옆) 방향으로 얼마나 두꺼운지"가 아니라서, 실제
    # 휠 옆면 두께를 과소평가해 롤러가 섀시 바디(폭 width)와 겹쳐버렸다(관통 -> 물리 발산).
    # 휠의 진짜 Y방향 반두께 = 허브 반두께 + (45도 기울어진 롤러의 Y축 투영 절반) + 롤러 반지름.
    wheel_half_thickness_y = HUB_THICKNESS / 2.0 + ROLLER_LENGTH * 0.5 + ROLLER_RADIUS
    half_l = WHEEL_MOUNT_HALF_L  # 섀시가 길어져도 휠 장착 위치는 고정(사용자 요청: "바퀴 위치 그대로")
    half_w = width / 2.0 + wheel_half_thickness_y * 1.3  # 30% 여유
    wheel_positions_local = []
    hub_joint_paths = []

    for (sx, sy, chirality), name in zip(corner_signs, corner_names):
        wx, wy, wz = sx * half_l, sy * half_w, 0.0  # z=0: chassis_root 프레임이 이미 휠 축 높이
        wheel_root_path = f"{BASE_PATH}/wheel_{name}"
        hub_joint_path = build_mecanum_wheel(stage, wheel_root_path, CHASSIS_PATH, (wx, wy, wz),
                                              wheel_material.prim_path, chirality, name)
        wheel_positions_local.append((wx, wy, wz))
        hub_joint_paths.append(hub_joint_path)
        print(f"[휠] {name} pos_local=({wx:.3f},{wy:.3f}) chirality={chirality:+d}", flush=True)

    k_factor = half_l + half_w
    return CHASSIS_PATH, hub_joint_paths, wheel_positions_local, k_factor


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
    """섀시 정중앙 위에 M0609(vgp20 그리퍼+카메라 버전)를 리프트(텔레포트 방식)로 얹는다.

    버전 이력: 한 번은 M0609의 root_joint를 섀시에 FixedJoint로 용접해서 정적으로 고정한
    적이 있다(리프트가 없던 버전, 8.rescale_and_rebuild.py 패턴 재사용) - 그런데 사용자가
    "새 홀로노믹 베이스에 리프트 다시 추가해줘"라고 요청했다. 리프트 높이가 매 프레임 바뀌어야
    하는데 FixedJoint는 정적이라 이 요구와 안 맞는다. 그보다 먼저(더 오래된) 버전에서는
    M0609를 독립 articulation으로만 만들고 텔레포트를 안 걸어서 허공에서 떨어진 적도 있었다
    (DOOSAN 각인이 바닥에 누워있는 스크린샷으로 확인) - 그건 "독립 articulation" 자체가
    문제가 아니라 텔레포트 루프가 없었던 게 문제였다. 이번엔 27/28번에서 검증된 정석대로
    "독립 articulation + 매 프레임 set_world_pose()로 리프트 높이에 계속 붙잡아두기" 패턴을
    제대로 구현한다(root_joint는 삭제, base_link에 ArticulationRootAPI를 새로 적용, 섀시와의
    충돌은 FilteredPairsAPI로 걸러서 두 바디가 겹쳐 있어도 서로 밀어내지 않게 한다).
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

    # 버그 이력(2026-07-23, 스크린샷으로 실측 확인): 이 프림을 BASE_PATH(=/World/HoloBase) 밑
    # 자식으로 두면, set_lift_height()에서 넣는 chassis_pos가 이미 WORLD 좌표인데
    # BASE_PATH 자신도 BASE_START_XY만큼 translate가 걸려있어서 로컬 translate로 그대로
    # 쓰면 오프셋이 이중으로 적용된다(리프트 기둥이 팔/섀시에서 BASE_START_XY만큼 떨어진
    # 엉뚱한 위치에 렌더링됨 - 사용자가 스크린샷에서 직접 발견). /World 바로 밑(최상위)에
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
    """표준 메카넘 역기하학 공식(순서 FL,FR,RL,RR). k=half_l+half_w.
    HolonomicController 대신 이 공식을 직접 쓴다 - 오프라인 스윕에서 HolonomicController가
    mecanum_angles를 바꿔도 응답이 안 바뀌는 등 이해 못 할 동작을 보여서 신뢰할 수 없었고,
    이 공식은 직접 검증 가능하고 투명하다(vx 단독 테스트에서 순수 전진 확인됨).

    버그 이력(83번 실측): 교과서 공식 그대로 vy를 넣었더니 실제로는 반대 방향(-y)으로
    움직였다(vy=+0.3을 150스텝 명령했더니 dy=-1.121). 원인은 롤러 조립 시 정한 chirality
    부호(build_mecanum_wheel의 FL/RR=+1, FR/RL=-1) convention이 이 교과서 공식이 가정하는
    롤러 방향과 정반대였기 때문으로 보인다 - 롤러 형상을 다시 만드는 대신, 공식에서 vy 부호를
    뒤집는 쪽이 훨씬 간단하고 검증도 쉽다(open-loop 테스트로 실측 확인 완료)."""
    vy = -vy  # 실측으로 확인된 부호 반전 (위 버그 이력 참고)
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

chassis_path, hub_joint_paths, wheel_positions_local, k_factor = build_holonomic_base(
    stage, BASE_START_XY, BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT
)

# 이론식(WHEEL_RADIUS+CHASSIS_BODY_HEIGHT/2) 대신 86번에서 실측(큐브 낙하+GUI 육안 튜닝)한 값 사용
MEASURED_CHASSIS_TOP_OFFSET = 0.0180
LIFT_MIN = MEASURED_CHASSIS_TOP_OFFSET + M0609_MOUNT_Z_ABOVE_CHASSIS_TOP  # 도킹(최저) 높이, chassis_root 원점 기준
LIFT_MAX = LIFT_MIN + LIFT_TRAVEL_M

M0609_ASSET_AVAILABLE = Path(M0609_USD).exists()
if M0609_ASSET_AVAILABLE:
    m0609_path, m0609_base_link_path, lift_translate_op, lift_scale_op = mount_m0609(stage, LIFT_MIN)
else:
    print(f"[건너뜀] M0609 에셋 없음({M0609_USD}) - 섀시/휠 형태만 먼저 완성", flush=True)
    m0609_path = None
    lift_translate_op = lift_scale_op = None

light = UsdLux.SphereLight.Define(stage, "/World/HoloBaseLight")
light.CreateRadiusAttr(0.3)
light.CreateIntensityAttr(200000)
UsdGeom.Xformable(light).AddTranslateOp().Set(Gf.Vec3d(BASE_START_XY[0], BASE_START_XY[1], 1.5))

for _ in range(20):
    simulation_app.update()

# M0609는 다시 독립 articulation(리프트 텔레포트 방식)이라 섀시(base_robot)와 별도로
# 초기화해야 한다.
base_robot = SingleArticulation(prim_path=chassis_path, name="holo_base")
world.reset()
base_robot.initialize(physics_sim_view=world.physics_sim_view)
print(f"[초기화] 섀시 dof_names={base_robot.dof_names} num_dof={base_robot.num_dof}", flush=True)

if M0609_ASSET_AVAILABLE:
    m0609_robot = SingleArticulation(prim_path=m0609_base_link_path, name="m0609_arm")
    m0609_robot.initialize(physics_sim_view=world.physics_sim_view)
    print(f"[초기화] M0609 dof_names={m0609_robot.dof_names} num_dof={m0609_robot.num_dof}", flush=True)
else:
    m0609_robot = None

hub_dof_indices = [base_robot.dof_names.index(Path(p).name) for p in hub_joint_paths]
print(f"[허브 DOF 인덱스] {hub_dof_indices} (FL,FR,RL,RR 순서)", flush=True)


def holo_forward(vx, vy, wz):
    speeds = mecanum_wheel_speeds(vx, vy, wz, WHEEL_RADIUS, k_factor)
    return ArticulationAction(joint_velocities=speeds, joint_indices=hub_dof_indices)


# ---- 리프트 제어 (27/28번과 동일한 텔레포트 패턴, 마운트 오프셋은 (0,0) 중앙) ----
lift_state = {"h": LIFT_MIN}


def set_lift_height(h):
    """섀시 현재 위치 바로 위 h로 m0609_robot을 텔레포트하고, 시각용 원기둥도 갱신한다.
    매 프레임 계속 불러줘야 리프트가 그 높이에 붙어있는다(안 그러면 중력으로 떨어짐)."""
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
    """리프트를 lift_state['h']에 붙잡아둔 채로 물리 스텝을 n번 진행한다."""
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


# ---- 자가진단 ----
print("\n[자가진단] (FL,FR,RL,RR 순서, 허브 목표 각속도)", flush=True)
for label, (vx, vy, wz) in [("vx=+0.3(전진)", (0.3, 0.0, 0.0)), ("vy=+0.3(횡이동)", (0.0, 0.3, 0.0)), ("wz=+0.5(회전)", (0.0, 0.0, 0.5))]:
    speeds = mecanum_wheel_speeds(vx, vy, wz, WHEEL_RADIUS, k_factor)
    print(f"  {label}: hub_speeds={np.round(speeds, 3)}", flush=True)

print("\n[물리 검증] 60스텝 정지 안정화 (리프트 도킹 높이 유지)\n", flush=True)
step_hold(60)

pos, quat = base_robot.get_world_pose()
roll, pitch, _ = quat_to_euler_angles(quat)
print(f"[결과] chassis pos={np.round(pos,3)} roll={np.degrees(roll):.3f}deg pitch={np.degrees(pitch):.3f}deg "
      f"(목표: Nova Carter 검증 기준과 동일하게 0.1도 이하, M0609 얹은 상태)", flush=True)

viewport = vp_util.get_active_viewport()


def snapshot(eye, target, fname):
    set_camera_view(eye=eye, target=target)
    for _ in range(20):
        if M0609_ASSET_AVAILABLE:
            set_lift_height(lift_state["h"])
        world.step(render=True)
    out = str(OUT_DIR / fname)
    vp_util.capture_viewport_to_file(viewport, out)
    for _ in range(10):
        if M0609_ASSET_AVAILABLE:
            set_lift_height(lift_state["h"])
        world.step(render=True)
    print(f"[SCREENSHOT] {out}", flush=True)


snapshot(eye=[BASE_START_XY[0], BASE_START_XY[1] + 0.9, 0.5], target=[BASE_START_XY[0], BASE_START_XY[1], 0.15],
         fname="_holobase_00_side.png")
snapshot(eye=[BASE_START_XY[0] + 0.9, BASE_START_XY[1], 0.5], target=[BASE_START_XY[0], BASE_START_XY[1], 0.15],
         fname="_holobase_00b_front.png")
snapshot(eye=[BASE_START_XY[0], BASE_START_XY[1], 1.6], target=[BASE_START_XY[0], BASE_START_XY[1], 0.05],
         fname="_holobase_01_top.png")
snapshot(eye=[BASE_START_XY[0] + 2.5, BASE_START_XY[1] - 3.0, 2.0], target=[1.0, -0.5, 0.3],
         fname="_holobase_02_overview.png")

# ---- 리프트 상승 데모 (사용자 요청: "리프트 다시 추가... 작동하는 거 스샷") ----
if M0609_ASSET_AVAILABLE:
    print(f"\n[리프트 테스트] {LIFT_MIN:.3f} -> {LIFT_MAX:.3f} 상승\n", flush=True)
    move_lift_to(LIFT_MAX, steps=120)
    pos_up, quat_up = base_robot.get_world_pose()
    roll_up, pitch_up, _ = quat_to_euler_angles(quat_up)
    print(f"[리프트 결과] chassis pos={np.round(pos_up,3)} roll={np.degrees(roll_up):.3f}deg "
          f"pitch={np.degrees(pitch_up):.3f}deg (리프트 상승 중 섀시 안정성)", flush=True)
    # 팔 자체도 리프트 위에서 위로 더 뻗어있어 전체 높이가 LIFT_MAX보다 크다 - 카메라를
    # 충분히 뒤로 빼고 타겟도 더 높게 잡아야 잘림 없이 전체가 프레임에 들어온다.
    snapshot(eye=[BASE_START_XY[0], BASE_START_XY[1] + 1.7, 0.9],
             target=[BASE_START_XY[0], BASE_START_XY[1], LIFT_MAX * 0.7],
             fname="_holobase_03_lift_up_side.png")
    snapshot(eye=[BASE_START_XY[0] + 1.7, BASE_START_XY[1], 0.9],
             target=[BASE_START_XY[0], BASE_START_XY[1], LIFT_MAX * 0.7],
             fname="_holobase_03_lift_up_front.png")
    print("[리프트] 최고 높이에서 스크린샷 저장 완료", flush=True)

print("\n[안내] 82번(베이스 빌드, 메카넘 롤러 + 리프트 버전) 검증 완료. 창을 계속 열어두고 확인하거나 종료하세요.")
print("[안내] 창을 닫으면 결과가 자동 저장됩니다.\n", flush=True)

while simulation_app.is_running():
    if M0609_ASSET_AVAILABLE:
        set_lift_height(lift_state["h"])
    simulation_app.update()
    time.sleep(0.01)

SCENE_USD = str(OUT_DIR / "holonomic_base_scene.usd")
omni.usd.get_context().save_as_stage(SCENE_USD)
print(f"\n[저장 완료] {SCENE_USD}", flush=True)

simulation_app.close()
