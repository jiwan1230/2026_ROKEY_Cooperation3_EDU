"""
4.add_boxes_and_zones.py에서 만든 8개 구역이 서로 너무 겹치고 붙어있어서 "다 한 공간"처럼
보인다는 피드백을 받고 레이아웃을 다시 짠다.

바뀐 점:
- Safety Working Zone을 꽉 찬 사각형(Loading Working Zone과 거의 겹침) 대신 **테두리(프레임)
  형태**로 바꿔서, 안쪽 초록(Loading)과 확실히 구분되게 함.
- Vehicle Parking Slot을 실제 차량 bbox에 맞게 줄여서 Loading Working Zone과의 겹침을 최소화.
- Robot Waiting Zone / Cart Return Zone을 서로 다른 방향(카트 기준 남쪽/북쪽)으로 떨어뜨려 배치.
- AMR Navigation Path는 Robot Waiting Zone → Loading Working Zone을 잇는 좁은 통로로 재설계.

기존 Zone_* prim들을 지우고 새 좌표로 다시 만든 뒤, 스크린샷으로 확인하고 저장한다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import time

import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, Gf
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view

_THIS_DIR = Path(__file__).resolve().parent
SCENE_USD = str(_THIS_DIR / "cart2trunk_scene.usd")


def rect(x0, x1, y0, y1):
    return ((x0 + x1) / 2, (y0 + y1) / 2), (x1 - x0, y1 - y0)


# ---- 재설계된 구역 (겹침 최소화, 간격 확보) ----
ZONES = []

c, s = rect(-0.8, 0.8, -1.1, 1.1)
ZONES.append(("Cart_Waiting_Zone", c, s, (0.20, 0.55, 0.95), 0.007))

c, s = rect(-3.0, -1.4, 0.4, 2.2)
ZONES.append(("Cart_Return_Zone", c, s, (0.10, 0.70, 0.70), 0.007))

c, s = rect(-1.3, 0.1, -3.4, -1.9)
ZONES.append(("Robot_Waiting_Zone", c, s, (0.95, 0.60, 0.10), 0.008))

c, s = rect(-0.1, 1.5, -1.9, -1.15)
ZONES.append(("AMR_Navigation_Path", c, s, (0.60, 0.20, 0.90), 0.009))

c, s = rect(1.3, 4.6, -1.2, 1.2)
ZONES.append(("Loading_Working_Zone", c, s, (0.20, 0.80, 0.30), 0.010))

# Safety Working Zone: 꽉 찬 사각형이 아니라 테두리(프레임) 4조각
SAFETY_OUTER = (0.8, 5.1, -1.9, 1.9)  # x0,x1,y0,y1
T = 0.35
ox0, ox1, oy0, oy1 = SAFETY_OUTER
c, s = rect(ox0, ox1, oy0, oy0 + T)
ZONES.append(("Safety_Working_Zone_Bottom", c, s, (0.95, 0.85, 0.10), 0.006))
c, s = rect(ox0, ox1, oy1 - T, oy1)
ZONES.append(("Safety_Working_Zone_Top", c, s, (0.95, 0.85, 0.10), 0.006))
c, s = rect(ox0, ox0 + T, oy0 + T, oy1 - T)
ZONES.append(("Safety_Working_Zone_Left", c, s, (0.95, 0.85, 0.10), 0.006))
c, s = rect(ox1 - T, ox1, oy0 + T, oy1 - T)
ZONES.append(("Safety_Working_Zone_Right", c, s, (0.95, 0.85, 0.10), 0.006))

c, s = rect(4.6, 13.4, -2.1, 2.1)
ZONES.append(("Vehicle_Parking_Slot", c, s, (0.45, 0.45, 0.60), 0.005))

c, s = rect(5.15, 6.38, -0.85, 0.85)
ZONES.append(("Trunk_Occupancy_Region", c, s, (0.90, 0.20, 0.20), 0.011))


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

# 기존 Zone_* 전부 삭제 후 재생성
removed = 0
for prim in list(stage.Traverse()):
    path = prim.GetPath().pathString
    if path.startswith("/World/Zone_"):
        stage.RemovePrim(path)
        removed += 1
print(f"[삭제] 기존 구역 {removed}개", flush=True)

for name, center_xy, size_xy, color, z in ZONES:
    add_zone_marker(stage, f"/World/Zone_{name}", center_xy, size_xy, color, z)

world.reset()
for _ in range(10):
    world.step(render=True)

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


snapshot(eye=[5.0, 0.0, 13.0], target=[5.0, 0.0, 0.0], fname="_verify_zones_top.png")
snapshot(eye=[1.0, -7.5, 4.5], target=[5.5, 0.0, 0.3], fname="_verify_zones_wide.png")

print("\n[안내] 재생 버튼은 이미 눌린 상태로 검증 완료. 창을 계속 열어두고 확인하거나 종료하세요.")
print("[안내] 창을 닫으면 결과가 자동 저장됩니다.\n", flush=True)

while simulation_app.is_running():
    simulation_app.update()
    time.sleep(0.01)

omni.usd.get_context().save_as_stage(SCENE_USD)
print(f"\n[저장 완료] {SCENE_USD}", flush=True)

simulation_app.close()
