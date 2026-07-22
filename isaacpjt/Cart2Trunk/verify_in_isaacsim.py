"""
verify_in_isaacsim.py — 우리 Planner가 계산한 PlacementPlan을 저장된 씬에
실제로 스폰해서 스크린샷으로 확인한다. (지완/GPU 노트북에서 실행)
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import UsdGeom, UsdPhysics, Gf
from isaacsim.core.api import World

_THIS_DIR = Path(__file__).resolve().parent
SCENE_USD = str(_THIS_DIR / "cart2trunk_scene.usd")

# ---- 1. 우리 Planner 실행 결과 (08_unloadable_reason.py의 generate_loading_plan) ----
# REAL_TRUNK_OFFSET = (3.11, -0.56, 1.03) 를 더해서 로컬 좌표 -> 실제 씬 월드 좌표로 변환
OFFSET = (3.11, -0.56, 1.03)
PLACEMENT_RESULT = [
    # (박스 id, 로컬 x, y, z, 크기 w,d,h, 색상)
    ("Medium", 0.0, 0.0, 0.0, 0.4, 0.3, 0.25, (0.2, 0.8, 0.3)),
    ("Small",  0.0, 0.3, 0.0, 0.3, 0.2, 0.15, (0.4, 0.3, 0.9)),
]
UNLOADED = [("Large", 0.5, 0.35, 0.30)]  # 참고용 - 스폰 안 함


def spawn_placed_box(stage, box_id, x, y, z, w, d, h, color):
    world_x = x + OFFSET[0]
    world_y = y + OFFSET[1]
    world_z = z + OFFSET[2]

    prim_path = f"/World/PlannerResult_{box_id}"
    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube.GetSizeAttr().Set(1.0)
    xform = UsdGeom.Xformable(cube)
    xform.ClearXformOpOrder()
    # 중심 좌표로 변환 (박스 코너 좌표 + 절반 크기)
    xform.AddTranslateOp().Set(Gf.Vec3d(world_x + w/2, world_y + d/2, world_z + h/2))
    xform.AddScaleOp().Set(Gf.Vec3f(w, d, h))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    print(f"[SPAWN] {box_id} -> world({world_x:.3f}, {world_y:.3f}, {world_z:.3f})", flush=True)


if __name__ == "__main__":
    omni.usd.get_context().open_stage(SCENE_USD)
    stage = omni.usd.get_context().get_stage()

    world = World()
    world.reset()

    for box_id, x, y, z, w, d, h, color in PLACEMENT_RESULT:
        spawn_placed_box(stage, box_id, x, y, z, w, d, h, color)

    for _ in range(60):
        simulation_app.update()

    vp_util.get_active_viewport().set_active_camera_hidden = False
    vp_util.capture_viewport_to_file(vp_util.get_active_viewport(), str(_THIS_DIR / "_verify_planner_result.png"))
    for _ in range(30):
        simulation_app.update()

    print("완료: _verify_planner_result.png 로 실제 트렁크 안에 잘 들어갔는지 확인하세요.")
    # simulation_app.close()
