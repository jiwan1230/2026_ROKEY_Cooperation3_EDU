"""
기존 저장된 씬(cart2trunk_scene.usd, SDF 콜리전 적용된 상태)을 열어서 이어서 작업한다.

1. 검증용 임시 낙하박스(TestBox_*) 4개를 지우고, 기획안 규격의 실제 작업 대상 박스 3종을
   카트 바구니 위에서 낙하시켜 SDF 콜리전 바닥에 안착시킨다.
   (시스템 요구사항 PDF "2.2 MVP 정상 시나리오 범위": 크기가 다른 정형 박스 3종)
   - Small  300x200x150mm
   - Medium 400x300x250mm
   - Large  500x350x300mm
2. PDF "1.2 환경 구성" 표에 정의된 8개 구역을 Simple Grid 바닥 위에 색깔 있는 평면 마커로
   표시한다 (Vehicle Parking Slot, Cart Waiting Zone, Robot Waiting Zone, Loading Working Zone,
   Safety Working Zone, AMR Navigation Path, Trunk Occupancy Region, Cart Return Zone).
   구역 좌표는 현재 씬 배치(카트=원점, 차량=(9,0,0), 트렁크 캐비티=x:5.15~6.38)를 기준으로
   합리적으로 설계한 것이며 PDF에 정확한 수치가 없으므로 추정치다.

박스/구역 모두 넣은 뒤 물리 시뮬레이션 + 스크린샷으로 실제로 잘 놓였는지 확인하고 저장한다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import time

import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view

_THIS_DIR = Path(__file__).resolve().parent
SCENE_USD = str(_THIS_DIR / "cart2trunk_scene.usd")

# ---- 1. 박스 3종 (기획안 규격, LxWxH meters) ----
BOXES = [
    # name,    size(L,W,H),         color(RGB),          drop_xy
    ("Small",  (0.30, 0.20, 0.15),  (0.85, 0.25, 0.20),  (-0.15, -0.35)),
    ("Medium", (0.40, 0.30, 0.25),  (0.25, 0.65, 0.30),  (0.00, 0.05)),
    ("Large",  (0.50, 0.35, 0.30),  (0.20, 0.35, 0.85),  (0.10, 0.45)),
]
BOX_MASS_KG = {"Small": 1.0, "Medium": 2.0, "Large": 3.5}  # PDF에 명시 안 됨, 합리적 추정치
CART_DROP_Z = 1.6

# ---- 2. 8개 구역 (PDF 1.2절 정의, 좌표는 현재 씬 배치 기준 설계) ----
# (name, center_xy, size_xy, color, z_offset)
ZONES = [
    ("Vehicle_Parking_Slot",   (9.0, 0.0),  (9.2, 4.2), (0.45, 0.45, 0.60), 0.006),
    ("Cart_Waiting_Zone",      (0.0, 0.0),  (1.6, 2.2), (0.20, 0.55, 0.95), 0.007),
    ("Robot_Waiting_Zone",     (2.0, -3.0), (1.6, 1.6), (0.95, 0.60, 0.10), 0.008),
    ("Loading_Working_Zone",   (3.75, 0.0), (5.5, 3.2), (0.20, 0.80, 0.30), 0.010),
    ("Safety_Working_Zone",    (3.75, 0.0), (6.5, 4.6), (0.95, 0.85, 0.10), 0.005),
    ("AMR_Navigation_Path",    (2.0, -2.3), (0.6, 1.4), (0.60, 0.20, 0.90), 0.009),
    ("Trunk_Occupancy_Region", (5.765, 0.0), (1.23, 1.70), (0.90, 0.20, 0.20), 0.011),
    ("Cart_Return_Zone",       (-2.2, 0.0), (1.6, 2.0), (0.10, 0.70, 0.70), 0.007),
]


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
    print(f"[BOX] {prim_path} size={size} mass={mass_kg}kg drop_center={center}", flush=True)
    return prim


def add_zone_marker(stage, prim_path, center_xy, size_xy, color, z):
    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube.GetSizeAttr().Set(1.0)
    xform = UsdGeom.Xformable(cube)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(center_xy[0], center_xy[1], z))
    xform.AddScaleOp().Set(Gf.Vec3f(size_xy[0], size_xy[1], 0.012))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    print(f"[ZONE] {prim_path} center={center_xy} size={size_xy}", flush=True)
    return cube.GetPrim()


omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    simulation_app.update()

world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

# 검증용 임시 낙하박스 정리
for old in ["/World/TestBox_Cart_Center", "/World/TestBox_Cart_Side",
            "/World/TestBox_Trunk_Center", "/World/TestBox_Trunk_Side",
            "/World/TestBox_Cart", "/World/TestBox_Trunk"]:
    prim = stage.GetPrimAtPath(old)
    if prim.IsValid():
        stage.RemovePrim(old)
        print(f"[삭제] {old}", flush=True)

# 박스 3종 배치
for name, size, color, (dx, dy) in BOXES:
    add_dynamic_box(stage, f"/World/Box_{name}", (dx, dy, CART_DROP_Z), size, color, BOX_MASS_KG[name])

# 8개 구역 마커 배치
for name, center_xy, size_xy, color, z in ZONES:
    add_zone_marker(stage, f"/World/Zone_{name}", center_xy, size_xy, color, z)

world.reset()
print("\n[물리 검증] 4초간 시뮬레이션 후 박스 최종 위치 확인\n", flush=True)
for _ in range(240):
    world.step(render=True)

xform_cache = UsdGeom.XformCache()
for name, *_ in BOXES:
    prim = stage.GetPrimAtPath(f"/World/Box_{name}")
    pos = xform_cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    print(f"[RESULT] Box_{name} final_pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})", flush=True)

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


# 카트 안 박스 3종 클로즈업
snapshot(eye=[1.8, -2.2, 1.7], target=[0.0, 0.1, 0.9], fname="_verify_boxes.png")
# 전체 구역 배치 확인용 위에서 내려다보는 샷
snapshot(eye=[3.5, -0.3, 11.0], target=[3.5, 0.0, 0.0], fname="_verify_zones_top.png")
# 낮은 각도로 전체 레이아웃 (카트~트렁크~로봇대기존)
snapshot(eye=[2.0, -8.0, 4.5], target=[4.5, 0.0, 0.3], fname="_verify_zones_wide.png")

print("\n[안내] 재생 버튼은 이미 눌린 상태로 검증 완료. 창을 계속 열어두고 확인하거나 종료하세요.")
print("[안내] 창을 닫으면 결과가 자동 저장됩니다.\n", flush=True)

while simulation_app.is_running():
    simulation_app.update()
    time.sleep(0.01)

omni.usd.get_context().save_as_stage(SCENE_USD)
print(f"\n[저장 완료] {SCENE_USD}", flush=True)

simulation_app.close()
