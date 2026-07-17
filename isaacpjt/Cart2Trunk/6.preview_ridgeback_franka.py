"""
저장된 씬(cart2trunk_scene.usd)에 RidgebackFranka를 얹어서 눈으로 확인해보는 미리보기 스크립트.

RidgebackFranka는 Isaac Sim 5.1 표준 에셋 CDN에서 가져온 것 (인터넷 필요):
  Isaac/Robots/Clearpath/RidgebackFranka/ridgeback_franka.usd
Robot Waiting Zone 위치(-0.6, -2.65)에 배치했다.

주의: 이 스크립트는 확인용이라 cart2trunk_scene.usd를 덮어쓰지 않는다 (창을 닫아도 저장 안 함).
마음에 들어서 정식으로 씬에 편입하고 싶으면 따로 요청하면 된다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import time

import omni.usd
from pxr import UsdGeom, Gf
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path

_THIS_DIR = Path(__file__).resolve().parent
SCENE_USD = str(_THIS_DIR / "cart2trunk_scene.usd")

ROBOT_POS = (-0.6, -2.65, 0.0)  # Robot Waiting Zone 중심

omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    simulation_app.update()

world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

root = get_assets_root_path()
if root is None:
    print("[실패] get_assets_root_path()가 None -> 인터넷/CDN 접근 불가, RidgebackFranka를 못 불러옴", flush=True)
else:
    robot_url = root + "/Isaac/Robots/Clearpath/RidgebackFranka/ridgeback_franka.usd"
    xform = UsdGeom.Xform.Define(stage, "/World/Preview_RidgebackFranka")
    ok = xform.GetPrim().GetReferences().AddReference(robot_url)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*ROBOT_POS))
    print(f"[로드] {robot_url} -> {'성공' if ok else '실패'}, 위치={ROBOT_POS} (Robot Waiting Zone)", flush=True)

world.reset()
set_camera_view(eye=[1.5, -6.0, 2.5], target=[-0.6, -2.65, 0.5])

print("\n[안내] Robot Waiting Zone(노란 구역) 자리에 RidgebackFranka를 얹어뒀습니다.", flush=True)
print("[안내] 재생(▶) 버튼으로 물리 확인 가능. 이 스크립트는 저장하지 않으니 마음껏 만져보세요.\n", flush=True)

while simulation_app.is_running():
    simulation_app.update()
    time.sleep(0.01)

simulation_app.close()
