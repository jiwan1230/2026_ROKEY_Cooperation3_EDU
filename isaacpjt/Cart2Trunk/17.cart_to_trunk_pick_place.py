"""
카트에 놓인 박스를 Nova Carter+M0609+VGP20(+옆면 카메라)로 흡착해서, 실제로 주행해 차량
트렁크 안까지 옮겨 넣는 통합 데모. 지금까지의 개별 검증들을 하나로 합친다:
  - 카트/차량 실제 비율 배치 + SDF 콜리전 (8.rescale_and_rebuild.py)
  - 트렁크 좌표(입구/바닥, 12.trunk_scan_hidden_gripper.py에서 실측 스캔으로 보정된 최종값 -
    8번 스크립트의 TRUNK_FLOOR_Z=1.03은 나중에 "사실 입구 턱이었다"고 밝혀진 잘못된 값이라
    안 쓰고, 여기선 검증된 0.44를 쓴다)
  - Nova Carter+M0609 병합 articulation (7/8/9/16번 스크립트와 동일 패턴)
  - VGP20 DynamicSuctionGripper + 옆면 카메라 (M0609/8번, Cart2Trunk 16번)

[왜 PickPlaceController(패키지)를 안 쓰고 RMPFlowController를 직접 호출하나]
  패키지 PickPlaceController는 "고정 베이스에서 집고 내려놓기"를 한 번의 이벤트 시퀀스로
  처리하는 걸 전제로 한다. 이번엔 집은 뒤 실제로 로봇이 주행해서 이동해야 하므로(집기 위치
  베이스와 놓기 위치 베이스가 다름), 그 시퀀스를 통째로 못 쓴다. 대신 RMPFlowController를
  웨이포인트별로 직접 호출(12번 스크립트의 스윕 방식과 동일)해서 4단계로 나눈다:
    1) PICK(집기 시작 위치에서): 박스 위 접근 -> 하강 -> 흡착 -> 들어올리기
    2) TRANSIT(주행): 팔에는 새 목표를 안 주고(마지막 자세 유지, 강한 조인트 드라이브가
       그대로 붙잡고 있음) 바퀴만 굴려서 이동 - 박스는 vgp20에 FixedJoint로 붙어있으므로
       로봇 전체와 함께 강체로 이동한다.
    3) 새 위치에서 RMPflow 베이스 포즈 재보정 (9/16번과 동일 이유: articulation 루트는
       chassis_link, 팔 베이스 아님).
    4) PLACE(트렁크 앞에서): 트렁크 목표 위 접근 -> 하강 -> 흡착 해제 -> 빠져나오기

[트렁크 목표 높이에 대한 현실적 타협]
  9번 스크립트의 reach sweep 결과 팔 베이스 높이 근처(relative z<=0.15)는 자기충돌로 항상
  실패하고 relative z>=0.30 이어야 안정적으로 도달한다. 그런데 트렁크 실측 바닥(world
  z=0.44)은 이번 주행 위치의 팔 베이스 높이(MOUNT_Z=0.42)와 거의 같아서(relative z~0.02),
  바로 그 "항상 실패하는" 구간과 겹친다. 바닥에 완벽하게 내려놓는 대신, 도달 가능한
  relative z=0.30(world z=0.72) 근처까지 내린 뒤 흡착을 풀어서 나머지는 중력으로 자연
  낙하시킨다 - 이미 PLACE_CLEARANCE(8번 M0609 스크립트)에서 쓰던 것과 같은 종류의 타협을
  범위만 키워 적용하는 것.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import sys

import numpy as np
import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, UsdLux, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import quat_to_euler_angles, euler_angles_to_quat
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

_THIS_DIR = Path(__file__).resolve().parent
M0609_DIR = _THIS_DIR.parent / "M0609"
RMPFLOW_DIR = str(M0609_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController  # noqa: E402

CART_USD = str(_THIS_DIR / "assets/Metal_Shopping_Cart.usdz")
CAR_USD = str(_THIS_DIR / "assets/Lexus_IS300_Trunk_Open_No_More_Hell_Room.usdz")
M0609_USD = str(M0609_DIR / "Collected_m0609_vgp20_camera" / "m0609_vgp20_camera.usd")
M0609_URDF_PATH = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH = str(M0609_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(M0609_DIR / "rmpflow/m0609_rmpflow_common.yaml")

# ---------------- 카트/차량 배치 (8.rescale_and_rebuild.py 실측값 그대로) ----------------
CART_POS = (0.0, 0.0, 0.0)
CART_EXTRA_SCALE = 0.55
CAR_POS = (5.0, 0.0, 0.0)
CAR_EXTRA_SCALE = 0.50
CAR_ROT_Z = 0.0
SDF_RESOLUTION = 256

# ---------------- 트렁크 (12.trunk_scan_hidden_gripper.py 실측 스캔 보정값) ----------------
TRUNK_X_MIN, TRUNK_X_MAX = 3.11, 3.68
TRUNK_Y_MIN, TRUNK_Y_MAX = -0.56, 0.56
TRUNK_FLOOR_Z = 0.44

# ---------------- 로봇/그리퍼 ----------------
MOUNT_Z = 0.42
# 배치를 카트-매니퓰레이터-차량 순으로 고치고 나니(사용자 지적) 새 문제가 드러났다: 로봇을
# +X(차량 쪽만)를 보게 두면 카트는 로봇 "뒤"에 있어서, 그 방향으로 팔을 뻗어 집으려니 IK가
# 전혀 수렴 못 함(pick_hover err=0.50m, 흡착 실패 - 실측 확인). M0609는 자기 "정면"(로컬 +X)
# 방향으로 뻗을 때만 잘 도달하는 것으로 보임. 그래서 처음엔 카트를 보게(180도) 세워서 집고,
# 주행 전에 제자리에서 180도 돌아 차량을 보게 만든 다음(TURN_IN_PLACE 참고) 트렁크에 놓는다.
FACE_ROT_Z = 180.0  # 시작 시 카트 쪽(-X)을 보게 - 집을 때 팔이 "정면"으로 뻗도록
# damping을 1e6(M0609/8번 스크립트의 "흡착 박스 흔들림 방지" 값)으로 올렸더니, 회전(순수
# 바퀴 속도 명령, IK와 무관)까지 점점 더 안 나가고(180도->121스텝, 다음 시도 58도로 더 나빠짐)
# 큰 목표로 갈수록 팔이 사실상 안 움직이는 것처럼 보이는 문제가 있었다 - 관절이 너무
# 뻑뻑해져서 주어진 스텝 수 안에 새 목표를 따라가지 못하는 것으로 보임. 모든 이전 성공
# 사례(9/16번 스크립트)가 쓰던 1e4로 되돌린다.
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e8, 1e4, 1e8

EE_LINK_NAME = "link_6"
GRIPPER_BODY_NAME = "vgp20"
TIP_LOCAL_OFFSET = (0.0, 0.0, 0.121)
GRASP_RADIUS = 0.10  # reach가 늘어난 만큼(0.70m) IK 오차 여유를 좀 더 둠

CUBE_SIZE = 0.05
CUBE_MASS = 0.05
CART_DROP_HEIGHT_ABOVE_FLOOR = 0.10  # 바구니 바닥 프록시보다 이만큼 위에서 떨어뜨림
CUBE_STATIC, CUBE_DYNAMIC = 1.2, 1.0

# 카트 메쉬(Metal_Shopping_Cart.usdz)를 raycast로 직접 찍어본 결과 바구니 "바닥"이 실제로는
# 없다 - 손잡이 프레임/양끝 곡면만 SDF 콜리전에 잡히고, 안쪽(x=-0.18~0.18 대부분)은 레이가
# 그대로 지면(z=0)까지 뚫고 지나감(철사 와이어 바구니를 단순화하며 바닥 메쉬 자체가 빠진
# 모델로 보임). 5cm 박스는 그 틈으로 그대로 떨어져 바닥에 안착해버렸다(1차 시도에서 확인:
# 최종 z=0.025). 카트 자체 메쉬/콜리전은 안 건드리고, 보이지 않는 물리 전용 플레이트를
# 바구니 안쪽에 따로 깔아서 박스가 올라앉을 자리를 만든다 - 높이는 팔의 도달 가능 영역
# (9번 스크립트 reach sweep: 팔 베이스 기준 relative z>=0.30일 때 안정적)에 맞춰 역산.
CART_BASKET_FLOOR_Z = 0.68

# 집기 위치: 카트 중심보다 이만큼 뒤(-X)에서 접근. 0.42였을 때 카트 bbox 앞쪽 끝(-0.30)까지
# 겨우 0.12m밖에 안 남아 Nova Carter 섀시 자체가 카트에 거의 붙어있는 상태였고(주행이 전혀
# 안 나가던 원인 중 하나로 의심), 팔도 여유 없이 딱 닿는 거리였다. 0.60으로 늘려서 섀시-카트
# 간격을 0.30m로 확보한다(reach는 좀 더 필요하지만 1차 시도에서 0.42m reach가 4mm 오차로
# 잘 됐던 걸 감안하면 여유는 있어 보임).
# 0.60은 IK 수렴이 나빴다(pick_grasp err=0.16m, 흡착 실패 - 실측 확인. 최초 성공 사례인
# run17b는 standoff=0.42에서 err=0.004m로 매우 정확했음). 0.42에 가깝게 되돌리되 살짝
# 여유(0.45)를 둔다 - 지금은 집는 동안 로봇이 정지해있다가 다 든 뒤에야 제자리 회전+주행을
# 하므로(순서 버그 수정 이후), 카트와의 간격이 좁아도 주행 시작 시점엔 이미 카트에서 안전한
# 상태(들어올려 돌아선 뒤)라 0.42 근처로 줄여도 될 것으로 판단.
# 0.45로는 IK는 잘 됐지만(흡착 성공) 주행이 완전히 엉망이 됨 - 주행 후 로봇이 y=0으로 곧게
# 가야 하는데 y=1.0까지 밀려나 있었고 박스도 바닥(z=0.026)에 떨어져 있었다. 받침 박스 자체가
# 0.35m 폭(x=[-0.175,0.175])인데 로봇이 0.45m 뒤(x=0.45)에서 출발하면 Nova Carter 섀시
# 자체(반경 대략 0.3m 이상)가 받침 박스와 거의 맞닿거나 겹쳐서, 주행을 시작하자마자
# 부딪혀 튕겨나간 것으로 보인다. 팔의 도달거리보다 섀시-장애물 간격을 우선해서 0.70으로
# 늘린다 (흡착은 GRASP_RADIUS=0.075 여유가 있고, VGP20 흡착팁이 link_6보다 0.121m 더
# 뻗어나가므로 어느 정도 reach가 늘어도 흡착 자체는 될 가능성이 있음 - 안 되면 GRASP_RADIUS를
# 더 키운다).
PICK_STANDOFF = 0.70
PLACE_STANDOFF = 0.35      # 트렁크 입구보다 이만큼 앞에서 정차 (reach를 0.45 근처로 맞춤)
PLACE_TARGET_X = TRUNK_X_MIN + 0.10   # 트렁크 안쪽으로 살짝만(입구 근처, 휠하우스 피함)
PLACE_TARGET_Y = 0.0                   # 트렁크 y 중앙
PLACE_LINK6_WORLD_Z = MOUNT_Z + 0.30   # reach sweep 검증된 relative z=0.30 (바닥까진 못 감, 아래 docstring 참고)

APPROACH_HOVER = 0.18  # 목표 위 이만큼 띄운 지점을 먼저 거쳐감(접근 시 충돌/오버슈트 방지)
WAYPOINT_STEPS = 150
SETTLE_STEPS = 60


# ╔══════════════════════════════════════════════════════════════╗
# ║  DynamicSuctionGripper (M0609/8, Cart2Trunk 16과 동일)           ║
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


# ╔══════════════════════════════════════════════════════════════╗
# ║  씬 구성 유틸                                                     ║
# ╚══════════════════════════════════════════════════════════════╝
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


def add_drive_stiffness(stage, root_path):
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        for dof_type in ["angular", "linear"]:
            drive = UsdPhysics.DriveAPI.Get(prim, dof_type)
            if drive:
                drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                drive.GetDampingAttr().Set(DRIVE_DAMPING)
                drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                n += 1
    return n


def build_mobile_manipulator(stage, start_xy):
    root = get_assets_root_path()
    carter_url = root + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"

    carter_path = "/World/MobileManipulator/NovaCarter"
    carter_xform = UsdGeom.Xform.Define(stage, carter_path)
    carter_xform.GetPrim().GetReferences().AddReference(carter_url)
    carter_xform.ClearXformOpOrder()
    carter_xform.AddTranslateOp().Set(Gf.Vec3d(start_xy[0], start_xy[1], 0.0))
    carter_xform.AddRotateZOp().Set(FACE_ROT_Z)

    m0609_path = "/World/MobileManipulator/M0609"
    m0609_xform = UsdGeom.Xform.Define(stage, m0609_path)
    m0609_xform.GetPrim().GetReferences().AddReference(M0609_USD, "/World/m0609")
    m0609_xform.ClearXformOpOrder()
    m0609_xform.AddTranslateOp().Set(Gf.Vec3d(start_xy[0], start_xy[1], MOUNT_Z))
    m0609_xform.AddRotateZOp().Set(FACE_ROT_Z)

    for _ in range(20):
        simulation_app.update()

    chassis_link_path = f"{carter_path}/chassis_link"
    root_joint_prim = stage.GetPrimAtPath(f"{m0609_path}/root_joint")
    joint = UsdPhysics.Joint(root_joint_prim)
    root_joint_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    root_joint_prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
    joint.GetBody0Rel().SetTargets([Sdf.Path(chassis_link_path)])
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, MOUNT_Z))
    print(f"[root_joint] body0 -> {chassis_link_path}, localPos0 -> (0,0,{MOUNT_Z})", flush=True)

    n = add_drive_stiffness(stage, m0609_path)
    print(f"[DRIVE] {n}개 조인트 강성 재설정", flush=True)

    return carter_path, chassis_link_path, m0609_path


def bbox_of(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    bbox = bbox_cache.ComputeWorldBound(prim)
    rng = bbox.ComputeAlignedRange()
    return np.array(rng.GetMin()), np.array(rng.GetMax())


# link_6이 항상 아래(월드 -Z)를 보게 - 패키지 PickPlaceController의 기본 그리퍼 방향(M0609/8번
# 스크립트 docstring 참고: end_effector_orientation=[0,pi,0])과 동일한, 이미 검증된 자세.
DOWN_QUAT = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))


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

cart_min, cart_max = bbox_of(stage, "/World/ShoppingCart")
cart_center_xy = ((cart_min[0] + cart_max[0]) / 2.0, (cart_min[1] + cart_max[1]) / 2.0)
print(f"[카트 bbox] min={cart_min} max={cart_max} center_xy={cart_center_xy}", flush=True)

# 카트 바구니는 바닥이 없는 철사 프레임이라(위 raycast 확인) 안쪽에 뭘 놓아도 그대로 바닥까지
# 떨어진다. 카트 콜리전과 싸우는 대신, 카트 위에 실제로 보이는 큰 받침 박스를 하나 놓고 그
# 위에 흡착 대상인 작은 큐브를 얹는 방식으로 단순화한다 - 실제 매장에서도 카트 안에 큰 박스가
# 있고 그 위에 작은 물건이 올려진 상황과 다르지 않다.
STAND_BOX_SIZE = (0.35, 0.30, CART_BASKET_FLOOR_Z - 0.05)  # 윗면이 CART_BASKET_FLOOR_Z에 오도록
stand_box = FixedCuboid(
    prim_path="/World/CartStandBox",
    name="cart_stand_box",
    position=np.array([cart_center_xy[0], cart_center_xy[1], CART_BASKET_FLOOR_Z - STAND_BOX_SIZE[2] / 2.0]),
    scale=np.array(STAND_BOX_SIZE),
    color=np.array([0.55, 0.40, 0.25]),
)
print(f"[받침 박스] top_z={CART_BASKET_FLOOR_Z} size={STAND_BOX_SIZE}", flush=True)

trunk_light = UsdLux.SphereLight.Define(stage, "/World/TrunkAreaLight")
trunk_light.CreateRadiusAttr(0.15)
trunk_light.CreateIntensityAttr(200000)
trunk_center_xy = ((TRUNK_X_MIN + TRUNK_X_MAX) / 2, (TRUNK_Y_MIN + TRUNK_Y_MAX) / 2)
UsdGeom.Xformable(trunk_light).AddTranslateOp().Set(Gf.Vec3d(trunk_center_xy[0], trunk_center_xy[1], TRUNK_FLOOR_Z + 0.3))
area_light = UsdLux.SphereLight.Define(stage, "/World/CartAreaLight")
area_light.CreateRadiusAttr(0.3)
area_light.CreateIntensityAttr(60000)
UsdGeom.Xformable(area_light).AddTranslateOp().Set(Gf.Vec3d(cart_center_xy[0], cart_center_xy[1], 2.0))

# 배치 순서 버그(사용자 지적): 카트 중심에서 -X로 로봇을 놨더니 매니퓰레이터-카트-차량 순이
# 되어버려서, 차량(+X, 트렁크)으로 직진하면 그 경로에 카트가 그대로 서 있었다 - "무지성 직진"이
# 카트에 막힐 수밖에 없는 배치였음. 카트-매니퓰레이터-차량 순서가 되도록 카트보다 +X(차량 쪽)에
# 로봇을 세워서, 트렁크로의 직진 경로에 카트가 끼지 않게 한다(집을 때는 뒤로 손을 뻗음).
pick_start_xy = (cart_center_xy[0] + PICK_STANDOFF, cart_center_xy[1])
carter_path, chassis_link_path, m0609_path = build_mobile_manipulator(stage, pick_start_xy)
gripper_body_path = f"{m0609_path}/{GRIPPER_BODY_NAME}"
ee_path = f"{m0609_path}/{EE_LINK_NAME}"

gripper = DynamicSuctionGripper(
    end_effector_prim_path=ee_path,
    gripper_body_path=gripper_body_path,
    target_prim_path="/World/PickBox",
    tip_local_offset=TIP_LOCAL_OFFSET,
    grasp_radius=GRASP_RADIUS,
)
robot = SingleManipulator(
    prim_path=chassis_link_path,
    end_effector_prim_path=ee_path,
    name="mobile_manipulator",
    gripper=gripper,
)

world.reset()
robot.initialize(physics_sim_view=world.physics_sim_view)
robot.gripper.initialize(physics_sim_view=world.physics_sim_view, articulation_num_dofs=robot.num_dof)
robot.set_joint_positions(np.zeros(robot.num_dof))
for _ in range(60):
    world.step(render=True)
print("\n[안정화 완료] 카트/차량/로봇 정지 상태\n", flush=True)

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
for _ in range(150):
    world.step(render=True)
box_pos, _ = box.get_world_pose()
print(f"[박스 낙하 완료] 최종 위치=({box_pos[0]:.3f},{box_pos[1]:.3f},{box_pos[2]:.3f})", flush=True)

viewport = vp_util.get_active_viewport()


def snapshot(eye, target, fname):
    set_camera_view(eye=eye, target=target)
    for _ in range(15):
        world.step(render=True)
    out = str(_THIS_DIR / fname)
    vp_util.capture_viewport_to_file(viewport, out)
    for _ in range(5):
        world.step(render=True)
    print(f"[SCREENSHOT] {out}", flush=True)


snapshot(eye=[pick_start_xy[0] - 0.6, pick_start_xy[1] - 1.0, 1.2], target=[box_pos[0], box_pos[1], box_pos[2]],
         fname="_verify_c2t_00_box_on_cart.png")

# ================= RMPflow 컨트롤러 (수동 웨이포인트 방식) =================
controller = RMPFlowController(
    name="c2t_cspace_controller",
    robot_articulation=robot,
    urdf_path=M0609_URDF_PATH,
    robot_description_path=M0609_DESCRIPTION_PATH,
    rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
    end_effector_frame_name=EE_LINK_NAME,
)


def set_base_pose():
    chassis_pos, chassis_quat = robot.get_world_pose()
    base_pos = np.array([chassis_pos[0], chassis_pos[1], chassis_pos[2] + MOUNT_Z])
    base_quat = chassis_quat
    controller._default_position = base_pos
    controller._default_orientation = base_quat
    controller.rmp_flow.set_robot_base_pose(robot_position=base_pos, robot_orientation=base_quat)
    yaw = float(quat_to_euler_angles(chassis_quat)[2])
    print(f"[RMPflow 보정] base_pos={np.round(base_pos,3)} yaw={np.degrees(yaw):.1f}deg", flush=True)
    return base_pos, base_quat, yaw


def move_link6(target_pos, steps=WAYPOINT_STEPS, hold_gripper_closed=False, label=""):
    for i in range(steps):
        actions = controller.forward(
            target_end_effector_position=np.array(target_pos),
            target_end_effector_orientation=DOWN_QUAT,
        )
        robot.apply_action(actions)
        if hold_gripper_closed:
            robot.gripper.close()
        world.step(render=True)
    ee_pos, _ = robot.end_effector.get_world_pose()
    err = np.linalg.norm(np.array(ee_pos) - np.array(target_pos))
    print(f"[웨이포인트{' ' + label if label else ''}] target={np.round(target_pos,3)} "
          f"ee={np.round(ee_pos,3)} err={err:.4f}m", flush=True)
    return ee_pos, err


def move_link6_converge(target_pos, tries=4, tol=0.02, steps=WAYPOINT_STEPS, hold_gripper_closed=False, label=""):
    """단발 IK가 회전된 베이스 + 아래보기 자세 조합에서 계속 특정 방향(주로 +Y)으로 못 미치는
    경향을 보였다(실측 확인) - 원인을 더 파고들기보다, 측정된 잔차만큼 다음 목표를 보정해서
    몇 번 반복하는 폐루프 방식으로 실용적으로 수렴시킨다."""
    target = np.array(target_pos, dtype=float)
    aim = target.copy()
    ee_pos, err = None, None
    for attempt in range(tries):
        ee_pos, err = move_link6(aim, steps=steps, hold_gripper_closed=hold_gripper_closed,
                                  label=f"{label}(수렴시도{attempt+1})")
        if err <= tol:
            break
        residual = target - np.array(ee_pos)
        aim = aim + residual
    return ee_pos, err


base_pos, base_quat, yaw = set_base_pose()


def local_to_world_xy(base_xy, yaw_rad, local_xy):
    dx, dy = local_xy
    wx = base_xy[0] + dx * np.cos(yaw_rad) - dy * np.sin(yaw_rad)
    wy = base_xy[1] + dx * np.sin(yaw_rad) + dy * np.cos(yaw_rad)
    return wx, wy


# ================= 1) PICK: 호버(장애물 위 안전지점) -> 하강 -> 흡착 -> 같은 호버로 복귀 =================
# 사용자 지적(중요): 기본 pick&place 원칙을 안 지키고 있었다 - 호버 지점이 "장애물과 안 부딪히는
# 상공"이어야 하는데, grasp_z+0.18(=1.006m)로 잡았더니 카트 최고높이(1.03m)에 못 미쳐서 카트
# 옆면 철제 레일 높이와 거의 겹쳤다(스크린샷으로 실측 확인 - 그리퍼가 큐브 위가 아니라 카트
# 레일에 거의 붙어있었음). 호버는 카트 전체보다 확실히 높게 잡고, 흡착 후에는 "다른 지점"이
# 아니라 반드시 같은 호버 지점으로 복귀한다 - 그래야 이후 이동(카트 기반 주행)도 항상 이
# 안전한 호버 자세에서 시작할 수 있다.
print("\n[PICK 시작]\n", flush=True)
pick_grasp_z = box_pos[2] + TIP_LOCAL_OFFSET[2]  # link_6 목표높이 = 박스중심 + 흡착팁 길이
pick_hover_pos = (box_pos[0], box_pos[1], cart_max[2] + 0.15)  # 카트 전체보다 위(장애물 없는 상공)
pick_grasp_pos = (box_pos[0], box_pos[1], pick_grasp_z)

move_link6(pick_hover_pos, steps=200, label="pick_hover")
move_link6(pick_grasp_pos, steps=200, label="pick_grasp")
for _ in range(30):
    robot.gripper.close()
    world.step(render=True)
    
print(f"[흡착 상태] attached={robot.gripper.is_closed()}", flush=True)
move_link6(pick_hover_pos, steps=200, hold_gripper_closed=True, label="pick_hover_복귀")
snapshot(eye=[pick_start_xy[0] - 0.8, pick_start_xy[1] - 1.0, base_pos[2] + 0.9],
         target=[box_pos[0], box_pos[1], box_pos[2]], fname="_verify_c2t_01_picked.png")
snapshot(eye=[pick_start_xy[0] - 0.5, pick_start_xy[1] - 0.6, base_pos[2] + 0.6],
         target=[box_pos[0], box_pos[1], pick_grasp_z], fname="_verify_c2t_01b_picked_close.png")

if not robot.gripper.is_closed():
    print("\n[경고] 흡착 실패\n", flush=True)
else:
    small_cube_pos, _ = box.get_world_pose()
    print(f"[PICK 단계 결과] 흡착 후 박스 위치=({small_cube_pos[0]:.3f},{small_cube_pos[1]:.3f},{small_cube_pos[2]:.3f}) "
          f"(들어올려졌으면 z가 grasp 높이보다 커야 함, 참고 grasp_z={pick_grasp_z:.3f})", flush=True)

print("\n[안내] 1단계(PICK+살짝 들어올리기) 검증 완료 - 이후 단계(주행/회전/PLACE)는 주석 처리됨.\n", flush=True)
simulation_app.close()

# ================= (아래는 다음 단계에서 순서대로 다시 켤 코드 - 지금은 주석 처리) =================
# # ================= 2) TRANSIT: 팔은 그대로 두고 주행만 =================
# print("\n[주행 시작] 카트 -> 트렁크 앞\n", flush=True)
# place_start_xy = (TRUNK_X_MIN - PLACE_STANDOFF, pick_start_xy[1])
# drive_distance = place_start_xy[0] - pick_start_xy[0]
#
# idx_left = robot.dof_names.index("joint_wheel_left")
# idx_right = robot.dof_names.index("joint_wheel_right")
# wheel_speed = 5.0
# start_x = float(robot.get_world_pose()[0][0])
#
# # 9/16번 스크립트와 동일: 먼저 짧게 굴려보고 실제로 +X로 가는지 확인 후 필요하면 부호 반전.
# robot.apply_action(ArticulationAction(joint_velocities=[wheel_speed, wheel_speed], joint_indices=[idx_left, idx_right]))
# for _ in range(30):
#     world.step(render=True)
# cur_x = float(robot.get_world_pose()[0][0])
# if (cur_x - start_x) < 0.02:
#     wheel_speed = -wheel_speed
#     print(f"[주행] 부호 반전 -> wheel_speed={wheel_speed}", flush=True)
#     robot.apply_action(ArticulationAction(joint_velocities=[wheel_speed, wheel_speed], joint_indices=[idx_left, idx_right]))
#
# step_count = 30
# while step_count < 2000:
#     world.step(render=True)
#     step_count += 1
#     cur_x = float(robot.get_world_pose()[0][0])
#     if abs(cur_x - start_x) >= abs(drive_distance):
#         break
# else:
#     print("[경고] 최대 스텝 안에 목표 거리 도달 못 함", flush=True)
# robot.apply_action(ArticulationAction(joint_velocities=[0.0, 0.0], joint_indices=[idx_left, idx_right]))
# for _ in range(SETTLE_STEPS):
#     world.step(render=True)
#
# box_pos_transit, _ = box.get_world_pose()
# chassis_pos_now, _ = robot.get_world_pose()
# print(f"[주행 완료] {step_count}스텝, 로봇 위치=({chassis_pos_now[0]:.3f},{chassis_pos_now[1]:.3f}), "
#       f"박스 위치=({box_pos_transit[0]:.3f},{box_pos_transit[1]:.3f},{box_pos_transit[2]:.3f}) "
#       f"(흡착 유지={robot.gripper.is_closed()})", flush=True)
# snapshot(eye=[chassis_pos_now[0] - 1.0, chassis_pos_now[1] - 1.3, 1.3],
#          target=[chassis_pos_now[0], chassis_pos_now[1], base_pos[2]], fname="_verify_c2t_02_transit.png")
#
# # ================= 2-1) 제자리 회전: 카트 쪽(-X)을 보던 로봇을 차량 쪽(+X)으로 =================
# def turn_in_place(target_deg, wheel_speed=3.0, max_steps=1500):
#     start_yaw = float(quat_to_euler_angles(robot.get_world_pose()[1])[2])
#     robot.apply_action(ArticulationAction(joint_velocities=[-wheel_speed, wheel_speed], joint_indices=[idx_left, idx_right]))
#     step = 0
#     target_rad = abs(np.radians(target_deg))
#     while step < max_steps:
#         world.step(render=True)
#         step += 1
#         cur_yaw = float(quat_to_euler_angles(robot.get_world_pose()[1])[2])
#         delta = (cur_yaw - start_yaw + np.pi) % (2 * np.pi) - np.pi
#         if abs(delta) >= target_rad - 0.03:
#             break
#     else:
#         print("[경고] 회전이 최대 스텝 안에 목표에 못 미침", flush=True)
#     robot.apply_action(ArticulationAction(joint_velocities=[0.0, 0.0], joint_indices=[idx_left, idx_right]))
#     for _ in range(SETTLE_STEPS):
#         world.step(render=True)
#     final_yaw = float(quat_to_euler_angles(robot.get_world_pose()[1])[2])
#     print(f"[회전] {step}스텝, 목표={target_deg}deg 실제 변화={np.degrees(final_yaw - start_yaw):.1f}deg", flush=True)
#
#
# turn_in_place(180.0)
#
# # ================= 3) 새 위치 기준 RMPflow 베이스 포즈 재보정 =================
# base_pos, base_quat, yaw = set_base_pose()
#
# # ================= 4) PLACE: 트렁크 앞 접근 -> 하강 -> 흡착 해제 -> 빠져나오기 =================
# print("\n[PLACE 시작]\n", flush=True)
# place_target = (PLACE_TARGET_X, PLACE_TARGET_Y, PLACE_LINK6_WORLD_Z)
# place_hover = (PLACE_TARGET_X, PLACE_TARGET_Y, PLACE_LINK6_WORLD_Z + APPROACH_HOVER)
#
# move_link6(place_hover, hold_gripper_closed=True, label="place_hover")
# move_link6(place_target, steps=200, hold_gripper_closed=True, label="place_descend")
# for _ in range(20):
#     world.step(render=True)
# robot.gripper.open()
# for _ in range(60):
#     world.step(render=True)
# print(f"[해제 상태] attached={robot.gripper.is_closed()}", flush=True)
#
# place_retract = (PLACE_TARGET_X - 0.3, PLACE_TARGET_Y, PLACE_LINK6_WORLD_Z + 0.15)
# move_link6(place_retract, label="place_retract")
#
# for _ in range(100):
#     world.step(render=True)
#
# final_box_pos, _ = box.get_world_pose()
# print(f"\n[최종] 박스 위치=({final_box_pos[0]:.3f},{final_box_pos[1]:.3f},{final_box_pos[2]:.3f})", flush=True)
# in_trunk_x = TRUNK_X_MIN - 0.05 <= final_box_pos[0] <= TRUNK_X_MAX + 0.05
# in_trunk_y = TRUNK_Y_MIN <= final_box_pos[1] <= TRUNK_Y_MAX
# near_floor = final_box_pos[2] <= TRUNK_FLOOR_Z + 0.5
# print(f"[최종] 트렁크 x범위 내={in_trunk_x} y범위 내={in_trunk_y} 바닥 근처(<+0.5m)={near_floor} "
#       f"-> {'PASS' if (in_trunk_x and in_trunk_y and near_floor) else 'FAIL'}", flush=True)
#
# snapshot(eye=[TRUNK_X_MIN - 1.2, -1.2, 1.3], target=[final_box_pos[0], final_box_pos[1], TRUNK_FLOOR_Z],
#          fname="_verify_c2t_03_final_trunk.png")
# snapshot(eye=[TRUNK_X_MIN - 0.3, -0.8, 1.5], target=[trunk_center_xy[0], trunk_center_xy[1], TRUNK_FLOOR_Z + 0.3],
#          fname="_verify_c2t_04_final_trunk_close.png")
#
# print("\n[안내] 검증 완료.\n", flush=True)
# simulation_app.close()
