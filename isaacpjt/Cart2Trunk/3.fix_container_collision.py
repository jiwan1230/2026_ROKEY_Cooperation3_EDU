"""
카트 바구니 / 트렁크 내부에 물품이 안정적으로 쌓이도록 콜리전을 고친다.

[1차 시도] 카트/차량 메쉬 전체에 convexHull 콜리전을 씌웠음 → 오목한 내부 공간이
"채워진 덩어리"로 뭉개져서 물품이 바깥 표면에서 튕기거나 얹힘.

[2차 시도] convexHull 대신 손으로 측정한 바닥+벽 Box 콜라이더(ShoppingCart_Collision,
Vehicle_Collision)를 만들어 붙임. 숫자(raycast) 검증은 통과했지만, 실제로는 좌표 자체가
틀려서(카트=바퀴 높이, 트렁크=차량 캐빈 부근) 스크린샷으로 보니 완전히 엉뚱한 곳에 있었음.
좌표를 재실측해서 고쳤더니 위치는 맞았지만, 이번엔 사용자가 GUI에서 옆쪽으로 옮겨보고
근본적인 한계를 지적함: 트렁크는 옆에 휠하우스(바퀴집) 돌출부가 있고, 카트 바구니는
아래보다 위가 넓은 사다리꼴 형태인데, 손으로 만든 "바닥+수직벽 4개" 박스는 이런 굴곡/테이퍼를
전혀 반영하지 못함 (사이드 쪽에서 실제 메쉬와 안 맞음).

[3차 = 최종] 박스 근사를 완전히 버리고, 시각 메쉬 자체에 **SDF(Signed Distance Field) 콜리전**을
직접 적용. SDF는 오목한 형태를 그대로 보존하면서 실제 삼각형 메쉬 표면을 따라가는 PhysX의
콜리전 방식이라, 사람이 치수를 재서 근사할 필요 없이 트렁크의 휠하우스 돌출부나 카트 바구니의
테이퍼진 벽을 있는 그대로 반영한다. 카트/차량은 RigidBody 없이 완전 정적으로 두고
(MVP 스펙: "카트는 지정 위치에 고정"), 정적 메쉬에 SDF 콜리전을 붙이는 방식이라 카트 바구니의
철사 틈 사이로 물체가 빠지는 문제도 없다 (SDF 해상도 256이면 철사 두께보다 충분히 촘촘함).

검증: 중앙 + 옆쪽(휠하우스/사다리꼴 벽 근처) 두 지점에 낙하 테스트박스를 놓고 물리 시뮬레이션 후
좌표 확인 + 스크린샷 촬영까지 자동으로 한다 (숫자만으로는 안 믿는다 — 이전에 두 번이나 좌표
자체가 틀렸던 걸 스크린샷으로만 잡아냈다).
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import time

import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, UsdLux, Gf
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view

_THIS_DIR = Path(__file__).resolve().parent

CART_USD = str(_THIS_DIR / "assets/Metal_Shopping_Cart.usdz")
CAR_USD = str(_THIS_DIR / "assets/Lexus_IS300_Trunk_Open_No_More_Hell_Room.usdz")

CART_POS = (0.0, 0.0, 0.0)
CAR_POS = (9.0, 0.0, 0.0)
CAR_ROT_Z = 0.0

# 검증용 낙하 지점 (대략적인 위치면 충분 — SDF가 실제 메쉬를 따라가므로 정확한 바닥 높이를
# 미리 잴 필요가 없다. "옆쪽" 지점은 카트는 사다리꼴 벽, 트렁크는 휠하우스 돌출부를 일부러 노림)
CART_DROP_CENTER = (0.0, 0.0, 1.6)
CART_DROP_SIDE = (0.35, 0.0, 1.6)
TRUNK_DROP_CENTER = (5.75, 0.0, 1.8)
TRUNK_DROP_SIDE = (5.75, 0.6, 1.8)

SDF_RESOLUTION = 256


def add_asset(stage, prim_path, usd_path, position, rot_z, target_mpu, target_up):
    src_stage = Usd.Stage.Open(usd_path)
    src_mpu = UsdGeom.GetStageMetersPerUnit(src_stage)
    src_up = UsdGeom.GetStageUpAxis(src_stage)
    scale = src_mpu / target_mpu if target_mpu else src_mpu

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
    """실제 시각 메쉬에 SDF 콜리전을 직접 적용 (정적 - RigidBody 없음).
    박스 근사와 달리 테이퍼/돌출부 등 실제 굴곡을 그대로 따라간다."""
    root_prim = stage.GetPrimAtPath(root_prim_path)
    n = 0
    for prim in Usd.PrimRange(root_prim):
        if prim.GetTypeName() == "Mesh":
            UsdPhysics.CollisionAPI.Apply(prim)
            mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mesh_collision.CreateApproximationAttr().Set("sdf")
            sdf_api = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
            sdf_api.CreateSdfResolutionAttr().Set(sdf_resolution)
            n += 1
    print(f"[SDF-COLLISION] {root_prim_path}: mesh {n}개, resolution={sdf_resolution}", flush=True)


def add_dynamic_test_box(stage, prim_path, center, size, mass_kg=0.5):
    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube.GetSizeAttr().Set(1.0)
    xform = UsdGeom.Xformable(cube)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*center))
    xform.AddScaleOp().Set(Gf.Vec3f(*size))
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr().Set(mass_kg)
    print(f"[TEST-BOX] {prim_path} drop from {center}, size={size}", flush=True)
    return prim


world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
target_mpu = UsdGeom.GetStageMetersPerUnit(stage)
target_up = UsdGeom.GetStageUpAxis(stage)

add_asset(stage, "/World/ShoppingCart", CART_USD, CART_POS, 0.0, target_mpu, target_up)
add_asset(stage, "/World/Vehicle", CAR_USD, CAR_POS, CAR_ROT_Z, target_mpu, target_up)

for _ in range(20):
    simulation_app.update()

add_sdf_collision(stage, "/World/ShoppingCart")
add_sdf_collision(stage, "/World/Vehicle")

# 검증용 낙하 테스트 박스 (기획안 박스 규격: Small 300x200x150mm) — 중앙 + 옆쪽(테이퍼/휠하우스 근처)
add_dynamic_test_box(stage, "/World/TestBox_Cart_Center", CART_DROP_CENTER, (0.30, 0.20, 0.15))
add_dynamic_test_box(stage, "/World/TestBox_Cart_Side", CART_DROP_SIDE, (0.30, 0.20, 0.15))
add_dynamic_test_box(stage, "/World/TestBox_Trunk_Center", TRUNK_DROP_CENTER, (0.30, 0.20, 0.15))
add_dynamic_test_box(stage, "/World/TestBox_Trunk_Side", TRUNK_DROP_SIDE, (0.30, 0.20, 0.15))

# 트렁크 쪽은 기본 조명이 카트(원점) 근처에만 있어서 9m 떨어진 차량이 안 보이므로 조명 추가
trunk_light = UsdLux.SphereLight.Define(stage, "/World/TrunkAreaLight")
trunk_light.CreateRadiusAttr(0.3)
trunk_light.CreateIntensityAttr(500000)
UsdGeom.Xformable(trunk_light).AddTranslateOp().Set(Gf.Vec3d(5.75, 0.0, 2.2))

world.reset()
set_camera_view(eye=[4.5, -9.0, 5.0], target=[4.5, 0.0, 0.5])

print("\n[물리 검증 시작] 4초간 시뮬레이션 후 박스 최종 위치 확인\n", flush=True)
for _ in range(240):
    world.step(render=True)

xform_cache = UsdGeom.XformCache()


def report(path):
    prim = stage.GetPrimAtPath(path)
    pos = xform_cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    print(f"[RESULT] {path} final_pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})", flush=True)


for p in ["/World/TestBox_Cart_Center", "/World/TestBox_Cart_Side",
          "/World/TestBox_Trunk_Center", "/World/TestBox_Trunk_Side"]:
    report(p)

# 숫자만으로는 못 믿는다 (이전에 좌표 자체가 두 번이나 틀렸었음) — 낙하 결과를 스크린샷으로 남긴다.
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


snapshot(eye=[1.8, -2.2, 1.6], target=[0.1, 0.1, 0.9], fname="_verify_cart.png")
snapshot(eye=[2.5, -4.0, 2.8], target=[5.75, 0.1, 1.0], fname="_verify_trunk.png")

print("\n[안내] 재생 버튼은 이미 눌린 상태로 검증 완료. 창을 계속 열어두고 확인하거나 종료하세요.")
print("[안내] 창을 닫으면 결과가 자동 저장됩니다.\n", flush=True)

# NOTE: omni.usd.get_context().save_as_stage()는 스테이지를 다시 여는 이벤트를 발생시켜서
# World 싱글턴을 무효화시킨다 ("World or Simulation Object are invalidated" 경고).
# 그 뒤에 world.step()을 계속 호출하면 World._scene 접근에서 AttributeError로 죽으므로,
# 저장은 인터랙티브 루프가 끝난 뒤(창 종료 시) 마지막에 한 번만 하고, 루프 안에서는
# World 객체 대신 simulation_app.update()만 사용해 무효화 문제를 아예 피한다.
while simulation_app.is_running():
    simulation_app.update()
    time.sleep(0.01)

SCENE_USD = str(_THIS_DIR / "cart2trunk_scene.usd")
omni.usd.get_context().save_as_stage(SCENE_USD)
print(f"\n[저장 완료] {SCENE_USD} (SDF 콜리전 + 검증용 낙하 박스 4개 포함)", flush=True)

simulation_app.close()
