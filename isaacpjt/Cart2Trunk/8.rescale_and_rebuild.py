"""
카트/차량이 실제보다 약 1.6~2배 크게 들어와 있던 문제를 고친다 (Nova Carter/M0609를 키우는 대신
카트/차량을 실제 비율로 축소하는 쪽을 선택함 — 로봇을 키우면 그리퍼 파지/RMPflow/센서 캘리브레이션이
전부 실물과 안 맞게 되기 때문. HANDOFF.md 참고).

씬 전체를 새 스케일 기준으로 처음부터 다시 짠다:
- 카트: 기존 스케일에 추가로 0.55배 (측정 결과 0.60x0.90x1.03m, 실제 카트와 근접)
- 차량: 기존 스케일에 추가로 0.50배 (측정 결과 4.31x1.98x1.50m, 실제 세단과 근접),
  차량 위치도 9m -> 5m로 당김 (더 이상 과대스케일 車 때문에 멀리 뗄 필요 없음)
- 트렁크/구역/로봇 위치는 새 배치에 맞게 전부 재설계 (raycast로 새 트렁크 위치 재측정함,
  rescale_probe.py 결과: x=[3.11,3.68] y=[-0.56,0.56] floor_z=1.03)
- 박스 3종, 8개 구역, Nova Carter+M0609 전부 새 좌표로 재배치
- Nova Carter+M0609 결합은 지난번에 잡은 두 가지 버그(조인트 드라이브 강성, localPos0)를
  처음부터 반영해서 만든다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import time

import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, UsdLux, Sdf, Gf
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path

_THIS_DIR = Path(__file__).resolve().parent
SCENE_USD = str(_THIS_DIR / "cart2trunk_scene.usd")
CART_USD = str(_THIS_DIR / "assets/Metal_Shopping_Cart.usdz")
CAR_USD = str(_THIS_DIR / "assets/Lexus_IS300_Trunk_Open_No_More_Hell_Room.usdz")
M0609_USD = str(_THIS_DIR.parent / "M0609" / "Collected_m0609_camera" / "m0609_camera.usd")

# ---- 1. 카트/차량 배치 (실제 비율 보정) ----
CART_POS = (0.0, 0.0, 0.0)
CART_EXTRA_SCALE = 0.55
CAR_POS = (5.0, 0.0, 0.0)
CAR_EXTRA_SCALE = 0.50
CAR_ROT_Z = 0.0

# ---- 2. 트렁크 (raycast 재측정 결과, rescale_probe.py) ----
TRUNK_X_MIN, TRUNK_X_MAX = 3.11, 3.68
TRUNK_Y_MIN, TRUNK_Y_MAX = -0.56, 0.56
TRUNK_FLOOR_Z = 1.03
TRUNK_WALL_TOP = 1.28

# ---- 3. 박스 3종 (기획안 규격 그대로, 절대 크기 불변) ----
# 카트가 이제 실제 크기(0.6x0.9m)라 셋을 동시에 떨어뜨리면 서로 밀쳐서 튕겨나감(1차 시도에서 확인).
# 큰 것부터 순서대로 하나씩 떨어뜨려 안착시킨 뒤 다음 박스를 떨어뜨리는 방식으로 바꿈.
BOXES = [
    ("Large", (0.50, 0.35, 0.30), (0.20, 0.35, 0.85), (0.0, 0.0)),
    ("Medium", (0.40, 0.30, 0.25), (0.25, 0.65, 0.30), (0.0, 0.0)),
    ("Small", (0.30, 0.20, 0.15), (0.85, 0.25, 0.20), (0.05, -0.05)),
]
BOX_MASS_KG = {"Small": 1.0, "Medium": 2.0, "Large": 3.5}
CART_DROP_Z = 1.4

# ---- 4. 8개 구역 (새 카트-차량 간격에 맞춰 압축 재설계) ----
def rect(x0, x1, y0, y1):
    return ((x0 + x1) / 2, (y0 + y1) / 2), (x1 - x0, y1 - y0)


ZONES = []
c, s = rect(-0.45, 0.45, -0.65, 0.65)
ZONES.append(("Cart_Waiting_Zone", c, s, (0.20, 0.55, 0.95), 0.007))
c, s = rect(-1.75, -0.85, 0.25, 1.35)
ZONES.append(("Cart_Return_Zone", c, s, (0.10, 0.70, 0.70), 0.007))
c, s = rect(-0.75, 0.15, -1.95, -1.05)
ZONES.append(("Robot_Waiting_Zone", c, s, (0.95, 0.60, 0.10), 0.008))
c, s = rect(-0.15, 0.75, -1.15, -0.65)
ZONES.append(("AMR_Navigation_Path", c, s, (0.60, 0.20, 0.90), 0.009))
c, s = rect(0.6, 2.6, -0.7, 0.7)
ZONES.append(("Loading_Working_Zone", c, s, (0.20, 0.80, 0.30), 0.010))
SAFETY_OUTER = (0.3, 2.9, -1.1, 1.1)
T = 0.22
ox0, ox1, oy0, oy1 = SAFETY_OUTER
c, s = rect(ox0, ox1, oy0, oy0 + T)
ZONES.append(("Safety_Working_Zone_Bottom", c, s, (0.95, 0.85, 0.10), 0.006))
c, s = rect(ox0, ox1, oy1 - T, oy1)
ZONES.append(("Safety_Working_Zone_Top", c, s, (0.95, 0.85, 0.10), 0.006))
c, s = rect(ox0, ox0 + T, oy0 + T, oy1 - T)
ZONES.append(("Safety_Working_Zone_Left", c, s, (0.95, 0.85, 0.10), 0.006))
c, s = rect(ox1 - T, ox1, oy0 + T, oy1 - T)
ZONES.append(("Safety_Working_Zone_Right", c, s, (0.95, 0.85, 0.10), 0.006))
c, s = rect(2.7, 7.3, -1.1, 1.1)
ZONES.append(("Vehicle_Parking_Slot", c, s, (0.45, 0.45, 0.60), 0.005))
c, s = rect(TRUNK_X_MIN, TRUNK_X_MAX, TRUNK_Y_MIN, TRUNK_Y_MAX)
ZONES.append(("Trunk_Occupancy_Region", c, s, (0.90, 0.20, 0.20), 0.011))

# ---- 5. Nova Carter + M0609 (Robot Waiting Zone 중심, 작업존 방향으로 회전) ----
ROBOT_XY = (-0.3, -1.5)
MOUNT_Z = 0.42
FACE_ROT_Z = 38.0
SDF_RESOLUTION = 256
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e8, 1e4, 1e8


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


def add_dynamic_box(stage, prim_path, center, size, color, mass_kg):
    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube.GetSizeAttr().Set(1.0)
    xform = UsdGeom.Xformable(cube)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*center))
    xform.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(mass_kg)
    print(f"[BOX] {prim_path} size={size} mass={mass_kg}kg drop={center}", flush=True)


def add_zone_marker(stage, prim_path, center_xy, size_xy, color, z):
    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube.GetSizeAttr().Set(1.0)
    xform = UsdGeom.Xformable(cube)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(center_xy[0], center_xy[1], z))
    xform.AddScaleOp().Set(Gf.Vec3f(size_xy[0], size_xy[1], 0.012))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    print(f"[ZONE] {prim_path} center={center_xy} size={size_xy}", flush=True)


# ================= 씬 구성 시작 =================
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

for name, center_xy, size_xy, color, z in ZONES:
    add_zone_marker(stage, f"/World/Zone_{name}", center_xy, size_xy, color, z)

# ---- Nova Carter + M0609 ----
root = get_assets_root_path()
carter_url = root + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"

carter_path = "/World/MobileManipulator/NovaCarter"
carter_xform = UsdGeom.Xform.Define(stage, carter_path)
carter_xform.GetPrim().GetReferences().AddReference(carter_url)
carter_xform.ClearXformOpOrder()
carter_xform.AddTranslateOp().Set(Gf.Vec3d(ROBOT_XY[0], ROBOT_XY[1], 0.0))
carter_xform.AddRotateZOp().Set(FACE_ROT_Z)
print(f"[NOVA-CARTER] pos=({ROBOT_XY[0]},{ROBOT_XY[1]},0.0) rotZ={FACE_ROT_Z}", flush=True)

m0609_path = "/World/MobileManipulator/M0609"
m0609_xform = UsdGeom.Xform.Define(stage, m0609_path)
m0609_xform.GetPrim().GetReferences().AddReference(M0609_USD, "/World/m0609")
m0609_xform.ClearXformOpOrder()
m0609_xform.AddTranslateOp().Set(Gf.Vec3d(ROBOT_XY[0], ROBOT_XY[1], MOUNT_Z))
m0609_xform.AddRotateZOp().Set(FACE_ROT_Z)
print(f"[M0609] pos=({ROBOT_XY[0]},{ROBOT_XY[1]},{MOUNT_Z}) rotZ={FACE_ROT_Z}", flush=True)

for _ in range(20):
    simulation_app.update()

chassis_link_path = f"{carter_path}/chassis_link"
root_joint_path = f"{m0609_path}/root_joint"
root_joint_prim = stage.GetPrimAtPath(root_joint_path)
joint = UsdPhysics.Joint(root_joint_prim)
root_joint_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
root_joint_prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
joint.GetBody0Rel().SetTargets([Sdf.Path(chassis_link_path)])
joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, MOUNT_Z))
print(f"[root_joint] body0 -> {chassis_link_path}, localPos0 -> (0,0,{MOUNT_Z})", flush=True)

# M0609 gripper(onrobot_rg2ft) 서브트리 안에 남아있는 두 번째 articulation 흔적 제거.
# onrobot_rg2ft/world는 원래 OnRobot RG2 그리퍼가 "독립된 로봇"으로 임포트됐을 때 쓰던
# 더미 world 링크인데, ArticulationRootAPI는 없지만 PhysxArticulationAPI가 남아있어서
# PhysX/Tensor API가 이걸 여전히 별도 articulation 후보로 헷갈려함. 실제로 이게
# "Incompatible size of velocity tensor: expected 6, received 12 shape(2,6)" 에러의
# 원인이었음 (연결된 몸 전체를 1개 articulation으로 합쳤는데, 텐서 API는 여전히 2개로 셈).
stray_articulation_path = f"{m0609_path}/onrobot_rg2ft/world"
stray_prim = stage.GetPrimAtPath(stray_articulation_path)
if stray_prim.IsValid() and stray_prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
    stray_prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
    print(f"[버그 수정] {stray_articulation_path}에 남아있던 PhysxArticulationAPI 제거 "
          f"(velocity tensor 크기 불일치 원인)", flush=True)

drive_count = 0
for prim in Usd.PrimRange(stage.GetPrimAtPath(m0609_path)):
    for dof_type in ["angular", "linear"]:
        drive = UsdPhysics.DriveAPI.Get(prim, dof_type)
        if drive:
            drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
            drive.GetDampingAttr().Set(DRIVE_DAMPING)
            drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
            drive_count += 1
print(f"[DRIVE] {drive_count}개 조인트 강성 적용", flush=True)

trunk_light = UsdLux.SphereLight.Define(stage, "/World/TrunkAreaLight")
trunk_light.CreateRadiusAttr(0.2)
trunk_light.CreateIntensityAttr(200000)
UsdGeom.Xformable(trunk_light).AddTranslateOp().Set(Gf.Vec3d(CAR_POS[0], 0.0, 1.8))

# ================= 물리 검증 =================
world.reset()
print("\n[물리 검증] 로봇/차량/카트 4초 안정화\n", flush=True)
for _ in range(240):
    world.step(render=True)

# 박스는 큰 것부터 순서대로 하나씩 떨어뜨려서 안착시킨 뒤 다음 박스를 떨어뜨림
# (동시에 떨어뜨리면 카트가 실제 크기라 서로 밀쳐서 튕겨나감)
for name, size, color, (dx, dy) in BOXES:
    add_dynamic_box(stage, f"/World/Box_{name}", (dx, dy, CART_DROP_Z), size, color, BOX_MASS_KG[name])
    for _ in range(100):
        world.step(render=True)
    print(f"[낙하 완료] Box_{name}", flush=True)

xform_cache = UsdGeom.XformCache()
for name, *_ in BOXES:
    prim = stage.GetPrimAtPath(f"/World/Box_{name}")
    pos = xform_cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    print(f"[RESULT] Box_{name} -> ({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})", flush=True)
for p in ["/World/MobileManipulator/NovaCarter/chassis_link", "/World/MobileManipulator/M0609/base_link"]:
    prim = stage.GetPrimAtPath(p)
    pos = xform_cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    print(f"[RESULT] {p} -> ({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})", flush=True)

viewport = vp_util.get_active_viewport()


def snapshot(eye, target, fname):
    set_camera_view(eye=eye, target=target)
    for _ in range(20):
        world.step(render=True)
    out = str(_THIS_DIR / fname)
    vp_util.capture_viewport_to_file(viewport, out)
    for _ in range(20):
        world.step(render=True)
    print(f"[SCREENSHOT] {out}", flush=True)


snapshot(eye=[2.5, -4.5, 3.0], target=[2.0, 0.0, 0.3], fname="_verify_v2_overview.png")
snapshot(eye=[0.7, -0.9, 1.1], target=[0.0, 0.0, 0.6], fname="_verify_v2_cart.png")
snapshot(eye=[3.9, -1.8, 1.9], target=[3.4, 0.0, 1.1], fname="_verify_v2_trunk.png")
snapshot(eye=[0.6, -2.6, 1.2], target=[-0.3, -1.5, 0.6], fname="_verify_v2_robot.png")

print("\n[안내] 재생 버튼은 이미 눌린 상태로 검증 완료. 창을 계속 열어두고 확인하거나 종료하세요.")
print("[안내] 창을 닫으면 결과가 자동 저장됩니다.\n", flush=True)

while simulation_app.is_running():
    simulation_app.update()
    time.sleep(0.01)

omni.usd.get_context().save_as_stage(SCENE_USD)
print(f"\n[저장 완료] {SCENE_USD}", flush=True)

simulation_app.close()
