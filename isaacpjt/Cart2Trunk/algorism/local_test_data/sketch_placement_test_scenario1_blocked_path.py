"""
sketch_placement_test_scenario1_blocked_path.py
[테스트 1/4] ⑯ 접근 경로 확인이 실제로 후보를 거부하는 상황.

지금까지 만든 시나리오들은 전부 우연히 "경계값이라 통과"했다 (예: 장애물 높이가
착지 높이와 정확히 같았던 경우). 이번엔 일부러 착지 높이보다 훨씬 높이 솟은
장애물을 입구 쪽에 놓아서, ⑯이 실제로 후보를 거부하는 걸 보여준다.

구성: 이미 트렁크에 확정으로 들어있는 것 - 차 바퀴 2개, 아주 높은 장애물
(Tall_Blocker, 높이 0.40m) 하나, 그 뒤에 놓인 받침 박스(Base_Box, 높이 0.15m) 하나.
카트에는 박스 하나(Cart_Item) - Base_Box 위에 쌓으려고 시도한다.
"""
import sys, pathlib
from importlib import import_module

ALGORISM_DIR = pathlib.Path("/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU/isaacpjt/Cart2Trunk/algorism")
sys.path.insert(0, str(ALGORISM_DIR))
sys.path.insert(0, str(ALGORISM_DIR / "local_test_data"))

m02 = import_module("02_trunk_space_state")
m03 = import_module("03_extreme_point_candidates")
m05 = import_module("05_candidate_scoring")
m07 = import_module("07_placement_plan")
m13 = import_module("13_support_check")
m15 = import_module("15_overhead_clearance_check")
viz = import_module("_viz_helpers")

Trunk = m02.Trunk
Box = m03.Box
PlacedBox = m03.PlacedBox
ExtremePointState = m03.ExtremePointState
generate_wall_flush_candidates = m03.generate_wall_flush_candidates
generate_box_flush_candidates = m03.generate_box_flush_candidates
place_one_box = m07.place_one_box
PlacementPlan = m07.PlacementPlan
score_candidate = m05.score_candidate
is_candidate_valid_with_stacking = m13.is_candidate_valid_with_stacking
has_overhead_clearance = m15.has_overhead_clearance
has_clear_approach_path = m15.has_clear_approach_path
SceneBox = viz.SceneBox
draw_scene = viz.draw_scene

TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT = 0.60, 0.73, 0.50
trunk = Trunk(width=TRUNK_WIDTH, depth=TRUNK_DEPTH, height=TRUNK_HEIGHT)

wheel_front = PlacedBox(box=Box("Wheel_Front", width=0.16, depth=0.15, height=0.20), x=0.44, y=0.00, z=0.0)
wheel_rear = PlacedBox(box=Box("Wheel_Rear", width=0.16, depth=0.21, height=0.20), x=0.44, y=0.52, z=0.0)

# 입구 쪽(x 작음)에 아주 높이 솟은 장애물 - 트렁크 높이(0.50m)의 80%나 됨
tall_blocker = PlacedBox(box=Box("Tall_Blocker", width=0.15, depth=0.30, height=0.40), x=0.10, y=0.20, z=0.0)
# 그 뒤(더 깊은 곳, 같은 y 폭)에 놓인 받침 박스 - 카트 박스가 쌓일 예정지
base_box = PlacedBox(box=Box("Base_Box", width=0.20, depth=0.20, height=0.15), x=0.30, y=0.25, z=0.0)

fixed_obstacles = [wheel_front, wheel_rear, tall_blocker, base_box]

cart_item = Box("Cart_Item", width=0.15, depth=0.15, height=0.12)

print("트렁크:", trunk)
print("고정 배치(알고리즘 미적용):")
for obs in fixed_obstacles:
    print(f"  {obs.box.id}: x[{obs.x:.3f},{obs.x+obs.box.width:.3f}] "
          f"y[{obs.y:.3f},{obs.y+obs.box.depth:.3f}] z[{obs.z:.3f},{obs.z+obs.box.height:.3f}]")

state = ExtremePointState()
for obs in fixed_obstacles:
    state.register_placement(obs)

# ---- BEFORE 그림: Cart_Item은 아직 카트 밖(트렁크 왼쪽)에 대기 중 ----
COLORS = {"Wheel_Front": "#424242", "Wheel_Rear": "#424242", "Tall_Blocker": "#e53935",
          "Base_Box": "#8d6e63", "Cart_Item": "#1e88e5"}
fixed_scene = [SceneBox(o.box.id, o.x, o.y, o.z, o.box.width, o.box.depth, o.box.height, COLORS[o.box.id])
               for o in fixed_obstacles]
waiting_scene = [SceneBox(cart_item.id, 0, 0, 0, cart_item.width, cart_item.depth, cart_item.height, COLORS["Cart_Item"])]

draw_scene(
    TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT,
    fixed_obstacles=fixed_scene, placed_boxes=[], waiting_boxes=waiting_scene,
    title="[시나리오 1] BEFORE - Cart_Item은 아직 카트에 있음, Tall_Blocker가 접근로를 막고 있음",
    out_path=str(ALGORISM_DIR / "local_test_data" / "sketch_scenario1_before.png"),
)

# ---------------------------------------------------------------------------
# [1단계] Base_Box 위에 쌓기를 직접 시도 - has_clear_approach_path만 따로 검사해서
# "왜 막히는지"를 명시적으로 보여준다.
# ---------------------------------------------------------------------------
target_x, target_y, target_z = base_box.x, base_box.y, base_box.box.height  # Base_Box 바로 위
print(f"\n[1단계] Base_Box 위 (x={target_x}, y={target_y}, z={target_z})에 Cart_Item 쌓기 시도")

ok_support = is_candidate_valid_with_stacking(target_x, target_y, target_z, cart_item, trunk, state.placed, allow_stacking=True)
ok_overhead = has_overhead_clearance(target_z, cart_item, trunk)
ok_path = has_clear_approach_path(target_x, target_y, target_z, cart_item, trunk, state.placed)
print(f"  받침 확인(⑬): {'통과' if ok_support else '거부'}")
print(f"  최종 자리 천장 여유(⑮): {'통과' if ok_overhead else '거부'} "
      f"(자리 자체는 여유 {TRUNK_HEIGHT - (target_z + cart_item.height):.2f}m >= 0.20m)")
print(f"  접근 경로 확인(⑯): {'통과' if ok_path else '거부'}")
if not ok_path:
    peak = max(target_z, tall_blocker.z_range[1])
    print(f"    -> Tall_Blocker(높이 {tall_blocker.box.height:.2f}m)가 입구 쪽에서 같은 y폭을 막고 있어서,")
    print(f"       실제로 넘어야 하는 높이는 {peak:.2f}m. 그 기준 남는 여유 = "
          f"{TRUNK_HEIGHT - (peak + cart_item.height):.2f}m < 0.20m -> 거부")

# ---------------------------------------------------------------------------
# [2단계] 예상대로 쌓기가 막히면, 일반 place_one_box(allow_stacking=True)로
# 대안 자리를 찾아본다 (⑯이 그냥 실패로 끝내는 게 아니라 안전한 대안으로
# 유도하는지까지 확인).
# ---------------------------------------------------------------------------
print("\n[2단계] 쌓기가 막혔으니 일반 배치(바닥 포함, allow_stacking=True)로 재시도")
plan = place_one_box(cart_item, trunk, state, order=1, allow_stacking=True)
if plan is None:
    print("  Cart_Item: 배치 불가 (대안 자리도 없음)")
    placed_scene = []
else:
    x, y, z = plan.position
    layer = "2층(쌓임)" if z > 1e-9 else "1층(바닥)"
    print(f"  Cart_Item: x[{x:.3f},{x+cart_item.width:.3f}] y[{y:.3f},{y+cart_item.depth:.3f}] "
          f"z[{z:.3f},{z+cart_item.height:.3f}]  [{layer}]  (Base_Box 위가 아니라 다른 자리로 우회함)")
    placed_scene = [SceneBox(cart_item.id, x, y, z, cart_item.width, cart_item.depth, cart_item.height,
                              COLORS["Cart_Item"], dashed=(z > 1e-9))]

draw_scene(
    TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT,
    fixed_obstacles=fixed_scene, placed_boxes=placed_scene, waiting_boxes=[],
    title="[시나리오 1] AFTER - Base_Box 위 쌓기는 ⑯에 의해 거부되고, 안전한 다른 자리로 배치됨",
    out_path=str(ALGORISM_DIR / "local_test_data" / "sketch_scenario1_after.png"),
)

print("\n=== 요약 ===")
print(f"Base_Box 위 쌓기 시도: {'통과' if ok_path else '거부됨 (⑯ 접근 경로 확인)'}")
print(f"최종 배치: {'성공' if plan is not None else '미적재'}")
