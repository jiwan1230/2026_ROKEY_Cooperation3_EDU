from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import time

import omni.usd
from isaacsim.core.api import World

_THIS_DIR = Path(__file__).resolve().parent
SCENE_USD = str(_THIS_DIR / "cart2trunk_scene.usd")

omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    simulation_app.update()

print(f"\n[열림] {SCENE_USD}", flush=True)
print("[안내] 재생(▶) 버튼을 눌러 물리 시뮬레이션을 확인하세요\n", flush=True)

world = World(stage_units_in_meters=1.0)

while simulation_app.is_running():
    world.step(render=True)
    time.sleep(0.01)

simulation_app.close()
