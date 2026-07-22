"""
sketch_placement_test_scenario4_full_trunk.py
[테스트 4/4] 완전히 꽉 찬 트렁크 - ⑧(미적재 사유 분류)이 서로 다른 이유를
정확히 구분하는지 확인.

트렁크를 큰 장애물 2개로 대부분 채워서 남는 공간을 L자 모양의 좁은 틈만 남긴다.
카트 박스 4개를 순서대로 시도하면서 성공 1개 + 서로 다른 미적재 사유 3가지
(SIZE_EXCEEDS_TRUNK / INSUFFICIENT_REMAINING_VOLUME / NO_VALID_CANDIDATE_POSITION)를
전부 나오게 한다.
"""
import sys, pathlib
from importlib import import_module

ALGORISM_DIR = pathlib.Path("/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU/isaacpjt/Cart2Trunk/algorism")
sys.path.insert(0, str(ALGORISM_DIR))
sys.path.insert(0, str(ALGORISM_DIR / "local_test_data"))

m02 = import_module("02_trunk_space_state")
m03 = import_module("03_extreme_point_candidates")
m07 = import_module("07_placement_plan")
m08 = import_module("08_unloadable_reason")
viz = import_module("_viz_helpers")

Trunk = m02.Trunk
Box = m03.Box
PlacedBox = m03.PlacedBox
ExtremePointState = m03.ExtremePointState
place_one_box = m07.place_one_box
classify_unloadable_reason = m08.classify_unloadable_reason
SceneBox = viz.SceneBox
draw_scene = viz.draw_scene

TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT = 0.60, 0.73, 0.50
trunk = Trunk(width=TRUNK_WIDTH, depth=TRUNK_DEPTH, height=TRUNK_HEIGHT)

# 트렁크 대부분을 채우는 큰 장애물 2개 - 남는 공간은 L자 모양의 좁은 틈뿐.
obstacle_1 = PlacedBox(box=Box("Obstacle_1", width=0.30, depth=0.73, height=0.30), x=0.00, y=0.00, z=0.0)
obstacle_2 = PlacedBox(box=Box("Obstacle_2", width=0.20, depth=0.50, height=0.30), x=0.30, y=0.00, z=0.0)
fixed_obstacles = [obstacle_1, obstacle_2]

cart_boxes = [
    Box("Small_Fit", width=0.15, depth=0.15, height=0.15),
    Box("Shape_Blocked", width=0.25, depth=0.25, height=0.15),
    Box("Volume_Squeeze", width=0.55, depth=0.50, height=0.47),
    Box("Too_Big_Overall", width=0.70, depth=0.30, height=0.15),
]

print("트렁크:", trunk, f"(부피 {trunk.width*trunk.depth*trunk.height*1000:.1f}L)")
for obs in fixed_obstacles:
    print(f"  {obs.box.id}: x[{obs.x:.3f},{obs.x+obs.box.width:.3f}] "
          f"y[{obs.y:.3f},{obs.y+obs.box.depth:.3f}] z[{obs.z:.3f},{obs.z+obs.box.height:.3f}]  "
          f"(부피 {obs.box.volume*1000:.1f}L)")

COLORS = {"Obstacle_1": "#e53935", "Obstacle_2": "#e53935", "Small_Fit": "#43a047",
          "Shape_Blocked": "#fb8c00", "Volume_Squeeze": "#8e24aa", "Too_Big_Overall": "#6d4c41"}
fixed_scene = [SceneBox(o.box.id, o.x, o.y, o.z, o.box.width, o.box.depth, o.box.height, COLORS[o.box.id])
               for o in fixed_obstacles]
waiting_scene = [SceneBox(b.id, 0, 0, 0, b.width, b.depth, b.height, COLORS[b.id]) for b in cart_boxes]

draw_scene(
    TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT,
    fixed_obstacles=fixed_scene, placed_boxes=[], waiting_boxes=waiting_scene,
    title="[시나리오 4] BEFORE - 트렁크가 장애물 2개로 대부분 참, 카트 박스 4개 대기",
    out_path=str(ALGORISM_DIR / "local_test_data" / "sketch_scenario4_before.png"),
)

state = ExtremePointState()
for obs in fixed_obstacles:
    state.register_placement(obs)

print("\n[1단계] 카트 박스 4개를 순서대로 시도")
results = {}
for box in cart_boxes:
    plan = place_one_box(box, trunk, state, order=len(results) + 1, allow_stacking=True)
    if plan is not None:
        results[box.id] = ("성공", plan)
        x, y, z = plan.position
        print(f"  {box.id}: 성공 x[{x:.3f},{x+box.width:.3f}] y[{y:.3f},{y+box.depth:.3f}] z[{z:.3f},{z+box.height:.3f}]")
    else:
        reason = classify_unloadable_reason(box, trunk, state)
        results[box.id] = (reason.value, None)
        print(f"  {box.id}: 미적재 - {reason.value}")

placed_scene, waiting_after = [], []
for b in cart_boxes:
    status, plan = results[b.id]
    if plan is not None:
        x, y, z = plan.position
        placed_scene.append(SceneBox(b.id, x, y, z, b.width, b.depth, b.height, COLORS[b.id], dashed=(z > 1e-9)))
    else:
        waiting_after.append(SceneBox(f"{b.id}({status})", 0, 0, 0, b.width, b.depth, b.height, COLORS[b.id]))

draw_scene(
    TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT,
    fixed_obstacles=fixed_scene, placed_boxes=placed_scene, waiting_boxes=waiting_after,
    title="[시나리오 4] AFTER - 성공/미적재 사유별 결과",
    out_path=str(ALGORISM_DIR / "local_test_data" / "sketch_scenario4_after.png"),
)

print("\n=== 요약 ===")
reasons_seen = {v[0] for v in results.values()}
for bid, (status, _) in results.items():
    print(f"  {bid}: {status}")
print(f"\n서로 다른 미적재 사유 {len(reasons_seen - {'성공'})}가지 확인됨: {sorted(reasons_seen - {'성공'})}")
