"""
sketch_placement_test_scenario3_needs_rotation.py
[테스트 3/4] 회전하면 들어가지만, 지금 알고리즘은 회전을 안 하는 상황.

Wide_Box는 지금 방향(가로 0.65m)으로는 트렁크 폭(0.60m)보다 커서 애초에 안 들어감
(SIZE_EXCEEDS_TRUNK, 위치와 무관하게 불가능). 근데 90도 돌려서 가로/세로를
바꾸면(0.30 x 0.65) 트렁크에 들어간다 - 03_extreme_point_candidates.py의
fits_dims()에 이미 명시된 "회전 미고려(MVP 범위)" 한계를 실제 숫자로 보여준다.
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
fits_dims = m03.fits_dims
place_one_box = m07.place_one_box
classify_unloadable_reason = m08.classify_unloadable_reason
UnloadableReason = m08.UnloadableReason
SceneBox = viz.SceneBox
draw_scene = viz.draw_scene

TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT = 0.60, 0.73, 0.50
trunk = Trunk(width=TRUNK_WIDTH, depth=TRUNK_DEPTH, height=TRUNK_HEIGHT)

wheel_front = PlacedBox(box=Box("Wheel_Front", width=0.16, depth=0.15, height=0.20), x=0.44, y=0.00, z=0.0)
wheel_rear = PlacedBox(box=Box("Wheel_Rear", width=0.16, depth=0.21, height=0.20), x=0.44, y=0.52, z=0.0)
fixed_obstacles = [wheel_front, wheel_rear]

# 지금 방향: 가로(width) 0.65m > 트렁크 폭 0.60m -> 어디에 놓든 애초에 불가능.
wide_box = Box("Wide_Box", width=0.65, depth=0.30, height=0.15)
# 90도 돌리면(가로/세로 교환): 0.30 x 0.65 -> 트렁크(0.60 x 0.73)에 들어감.
rotated_box = Box("Wide_Box(회전됨)", width=wide_box.depth, depth=wide_box.width, height=wide_box.height)

print("트렁크:", trunk)
print(f"Wide_Box 현재 방향: 가로{wide_box.width}m x 세로{wide_box.depth}m "
      f"(트렁크 가로 {TRUNK_WIDTH}m보다 {'큼 - 못 들어감' if wide_box.width > TRUNK_WIDTH else '작음'})")
print(f"90도 돌리면: 가로{rotated_box.width}m x 세로{rotated_box.depth}m "
      f"(트렁크 안에 {'들어감' if fits_dims(rotated_box, trunk) else '그래도 안 들어감'})")

COLORS = {"Wheel_Front": "#424242", "Wheel_Rear": "#424242", "Wide_Box": "#c62828", "Wide_Box(회전됨)": "#9e9e9e"}
fixed_scene = [SceneBox(o.box.id, o.x, o.y, o.z, o.box.width, o.box.depth, o.box.height, COLORS[o.box.id])
               for o in fixed_obstacles]
waiting_scene = [SceneBox(wide_box.id, 0, 0, 0, wide_box.width, wide_box.depth, wide_box.height, COLORS["Wide_Box"])]

draw_scene(
    TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT,
    fixed_obstacles=fixed_scene, placed_boxes=[], waiting_boxes=waiting_scene,
    title="[시나리오 3] BEFORE - Wide_Box(가로 0.65m)가 트렁크 폭(0.60m)보다 큼",
    out_path=str(ALGORISM_DIR / "local_test_data" / "sketch_scenario3_before.png"),
)

# ---------------------------------------------------------------------------
# [1단계] 실제 알고리즘으로 시도 (회전 없음, 있는 그대로)
# ---------------------------------------------------------------------------
state = ExtremePointState()
for obs in fixed_obstacles:
    state.register_placement(obs)

print("\n[1단계] 지금 방향 그대로 배치 시도 (회전 미지원)")
plan = place_one_box(wide_box, trunk, state, order=1, allow_stacking=True)
reason = None if plan is not None else classify_unloadable_reason(wide_box, trunk, state)
if plan is None:
    print(f"  Wide_Box: 배치 불가 - 사유: {reason.value}")
else:
    print(f"  Wide_Box: 배치 성공 (예상과 다름) - {plan.position}")

# ---------------------------------------------------------------------------
# [2단계] "만약 회전을 지원했다면" - 가상으로 rotated_box를 시도해서 참고용으로 표시.
# 지금 알고리즘 코드는 전혀 안 건드리고, 그냥 회전된 치수로 별도 시도만 해봄.
# ---------------------------------------------------------------------------
print("\n[2단계] (참고용, 지금은 미지원) 회전됐다면 어디 들어갔을지 시험 배치")
state2 = ExtremePointState()
for obs in fixed_obstacles:
    state2.register_placement(obs)
rotated_plan = place_one_box(rotated_box, trunk, state2, order=1, allow_stacking=True)
if rotated_plan is not None:
    x, y, z = rotated_plan.position
    print(f"  Wide_Box(회전됨): x[{x:.3f},{x+rotated_box.width:.3f}] y[{y:.3f},{y+rotated_box.depth:.3f}] "
          f"z[{z:.3f},{z+rotated_box.height:.3f}]  (참고용 - 지금 알고리즘은 이 시도를 안 함)")
else:
    print("  Wide_Box(회전됨): 그래도 자리 없음")

placed_scene = []
waiting_after = []
if plan is None:
    # 실제 결과: 미적재라서 여전히 카트(대기) 자리에 남음
    waiting_after.append(SceneBox(wide_box.id, 0, 0, 0, wide_box.width, wide_box.depth, wide_box.height, COLORS["Wide_Box"]))
if rotated_plan is not None:
    x, y, z = rotated_plan.position
    placed_scene.append(SceneBox("Wide_Box(회전하면 여기)", x, y, z, rotated_box.width, rotated_box.depth,
                                  rotated_box.height, COLORS["Wide_Box(회전됨)"], dashed=True))

draw_scene(
    TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT,
    fixed_obstacles=fixed_scene, placed_boxes=placed_scene, waiting_boxes=waiting_after,
    title=f"[시나리오 3] AFTER - 실제 결과: 미적재({reason.value if reason else '?'}) "
          f"/ 회색 점선=회전했다면 들어갔을 자리(참고용, 미지원)",
    out_path=str(ALGORISM_DIR / "local_test_data" / "sketch_scenario3_after.png"),
)

print("\n=== 요약 ===")
print(f"실제 배치 결과: {'성공' if plan is not None else f'미적재 ({reason.value})'}")
print(f"회전 지원했다면: {'들어갔을 것' if rotated_plan is not None else '그래도 못 들어감'}")
