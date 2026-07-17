from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import time

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view

_THIS_DIR = Path(__file__).resolve().parent

CART_USD = str(_THIS_DIR / "assets/Metal_Shopping_Cart.usdz")
CAR_USD = str(_THIS_DIR / "assets/Lexus_IS300_Trunk_Open_No_More_Hell_Room.usdz")

# 배치 파라미터 - 겹침/방향 문제 생기면 여기만 조절
# z를 0.3만큼 띄워서 중력이 실제로 작동하는지(떨어져서 바닥에 안착하는지) 눈으로 확인
CART_POS = (0.0, 0.0, 0.3)
CAR_POS = (9.0, 0.0, 0.3)      # 카트와 충분히 간격 (차량 길이 약 8.6m 감안)
CAR_ROT_Z = 0.0                # 트렁크(뒷부분)가 카트 쪽(-X)을 향하도록 회전 (180 -> 0으로 반대 확인됨)


def add_rigid_physics(root_prim_path, mass_kg):
    root_prim = stage.GetPrimAtPath(root_prim_path)
    UsdPhysics.RigidBodyAPI.Apply(root_prim)
    mass_api = UsdPhysics.MassAPI.Apply(root_prim)
    mass_api.CreateMassAttr().Set(mass_kg)

    mesh_count = 0
    for prim in Usd.PrimRange(root_prim):
        if prim.GetTypeName() == "Mesh":
            UsdPhysics.CollisionAPI.Apply(prim)
            mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mesh_collision.CreateApproximationAttr().Set("convexHull")
            mesh_count += 1
    print(f"[PHYSICS] {root_prim_path} rigidBody+mass={mass_kg}kg, colliders on {mesh_count} mesh(es)", flush=True)

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

target_mpu = UsdGeom.GetStageMetersPerUnit(stage)
target_up = UsdGeom.GetStageUpAxis(stage)
print(f"[STAGE] metersPerUnit={target_mpu}  upAxis={target_up}", flush=True)


def inspect_source(usd_path):
    src_stage = Usd.Stage.Open(usd_path)
    mpu = UsdGeom.GetStageMetersPerUnit(src_stage)
    up = UsdGeom.GetStageUpAxis(src_stage)
    print(f"[SOURCE] {Path(usd_path).name}: metersPerUnit={mpu}  upAxis={up}", flush=True)
    return mpu, up


def add_asset(prim_path, usd_path, position, rot_z=0.0):
    src_mpu, src_up = inspect_source(usd_path)
    scale = src_mpu / target_mpu if target_mpu else src_mpu

    xform = UsdGeom.Xform.Define(stage, prim_path)
    prim = xform.GetPrim()
    ok = prim.GetReferences().AddReference(usd_path)

    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(position)
    if rot_z:
        xform.AddRotateZOp().Set(rot_z)
    if src_up == UsdGeom.Tokens.y and target_up == UsdGeom.Tokens.z:
        xform.AddRotateXOp().Set(90.0)
    xform.AddScaleOp().Set((scale, scale, scale))

    print(f"[REF] {prim_path} AddReference={ok}  scale={scale:.5f}  rotZ={rot_z}", flush=True)
    return xform


add_asset("/World/ShoppingCart", CART_USD, CART_POS)
add_asset("/World/Vehicle", CAR_USD, CAR_POS, rot_z=CAR_ROT_Z)

for _ in range(30):
    simulation_app.update()

add_rigid_physics("/World/ShoppingCart", mass_kg=15.0)
add_rigid_physics("/World/Vehicle", mass_kg=1500.0)


def describe(prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    bbox = bbox_cache.ComputeWorldBound(prim)
    rng = bbox.ComputeAlignedRange()
    print(f"[BBOX] {prim_path} min={rng.GetMin()} max={rng.GetMax()}", flush=True)


describe("/World/ShoppingCart")
describe("/World/Vehicle")

world.reset()
set_camera_view(eye=[4.5, -9.0, 5.0], target=[4.5, 0.0, 0.5])

print("\n[루프 시작] Isaac Sim 창에서 확인하세요\n", flush=True)

while simulation_app.is_running():
    world.step(render=True)
    time.sleep(0.01)

simulation_app.close()
