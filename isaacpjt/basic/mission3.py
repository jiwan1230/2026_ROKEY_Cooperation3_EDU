from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid

# World 생성
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

# 바닥 추가
world.scene.add_default_ground_plane()

# 큰 큐브 (고정)
large_cube = FixedCuboid(
    prim_path="/World/LargeCube",
    name="large_cube",
    position=np.array([0.0, 0.0, 0.15]),
    scale=np.array([0.3, 0.3, 0.3]),
    color=np.array([1.0, 0.0, 0.0]),  # 빨간색
)

# 작은 큐브 (동적)
small_cube = DynamicCuboid(
    prim_path="/World/SmallCube",
    name="small_cube",
    position=np.array([0.0, 0.0, 0.8]),
    scale=np.array([0.1, 0.1, 0.1]),
    color=np.array([0.0, 1.0, 0.0]),  # 초록색
)

# Scene에 추가
world.scene.add(large_cube)
world.scene.add(small_cube)

# 시뮬레이션 초기화
world.reset()

# 시뮬레이션 실행
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()