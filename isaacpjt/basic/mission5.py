from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})     # 1. Application

import numpy as np
import time
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World(stage_units_in_meters=1.0)                # 2. World
stage = omni.usd.get_context().get_stage()              # 3. Stage

cube_prim = DynamicCuboid(                              # 4. Prim
    prim_path="/World/RedCube",
    name="red_cube",
    position=np.array([0.0, 0.0, 0.5]),
    scale=np.array([0.15, 0.15, 0.15]),
    color=np.array([1.0, 0.0, 0.0]),
)

world.scene.add_default_ground_plane()                  # 5. Scene
world.scene.add(cube_prim)

world.reset()

TELEPORT_STEP = 300
TELEPORT_HEIGHT = 1.0

step_count = 0
was_playing = False
teleported = False

while simulation_app.is_running():                      # 6. Simulation
    world.step(render=True)
    time.sleep(0.01)

    is_playing = world.is_playing()

    # GUI에서 Stop -> Play로 바뀐 시점(엣지)을 감지해서 처음부터 다시 시작
    if is_playing and not was_playing:
        step_count = 0
        teleported = False
        print("[리셋] Play 시작 -> step_count = 0")

    # Stop 상태에서는 while 루프 자체는 계속 돌지만 카운트/텔레포트는 멈춘다
    if is_playing:
        step_count += 1

        if step_count % 100 == 0:
            print(f"step: {step_count}")

        if step_count == TELEPORT_STEP and not teleported:
            position, orientation = cube_prim.get_world_pose()
            cube_prim.set_world_pose(
                position=np.array([position[0], position[1], TELEPORT_HEIGHT]),
                orientation=orientation,
            )
            teleported = True
            print(f"[텔레포트] step {step_count} -> 큐브가 {TELEPORT_HEIGHT}m 높이로 이동")

    was_playing = is_playing

simulation_app.close()
