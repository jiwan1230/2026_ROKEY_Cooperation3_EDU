"""
sketch_placement_test_scenario3_needs_rotation.py
[테스트 3/4] 회전(⑱)이 필요한 박스 - 이제는 자동으로 90도 돌려서 넣는다.

Wide_Box는 지금 방향(가로 0.65m)으로는 트렁크 폭(0.60m)보다 커서 정자세로는
어디에도 못 들어간다. 예전에는 여기서 그냥 미적재 처리됐지만(19번 라운드에서
확인한 한계), ⑱ 회전 지원을 추가한 뒤로는 place_one_box()가 정자세 실패 시
자동으로 90도 돌린 자세(가로/세로 교환, 높이는 그대로)를 한 번 더 시도한다.
이 스크립트는 그걸 실제 숫자로 보여준다 - 호출은 이제 딱 한 번(place_one_box)
이면 충분하고, 별도로 "회전판을 만들어서 따로 시도"할 필요가 없다.
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
m18 = import_module("18_rotation")
viz = import_module("_viz_helpers")

Trunk = m02.Trunk
Box = m03.Box
PlacedBox = m03.PlacedBox
ExtremePointState = m03.ExtremePointState
fits_dims = m03.fits_dims
place_one_box = m07.place_one_box
classify_unloadable_reason = m08.classify_unloadable_reason
UnloadableReason = m08.UnloadableReason
rotate_box = m18.rotate_box
SceneBox = viz.SceneBox
draw_scene = viz.draw_scene

TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT = 0.60, 0.73, 0.50
trunk = Trunk(width=TRUNK_WIDTH, depth=TRUNK_DEPTH, height=TRUNK_HEIGHT)

wheel_front = PlacedBox(box=Box("Wheel_Front", width=0.16, depth=0.15, height=0.20), x=0.44, y=0.00, z=0.0)
wheel_rear = PlacedBox(box=Box("Wheel_Rear", width=0.16, depth=0.21, height=0.20), x=0.44, y=0.52, z=0.0)
fixed_obstacles = [wheel_front, wheel_rear]

# 정자세: 가로(width) 0.65m > 트렁크 폭 0.60m -> 정자세로는 어디에도 못 들어감.
wide_box = Box("Wide_Box", width=0.65, depth=0.30, height=0.15)
rotated_preview = rotate_box(wide_box)  # 화면 출력/그림용 미리보기일 뿐, 실제 시도는 place_one_box 내부에서 자동으로 함

print("트렁크:", trunk)
print(f"Wide_Box 정자세: 가로{wide_box.width}m x 세로{wide_box.depth}m "
      f"(트렁크 가로 {TRUNK_WIDTH}m보다 {'큼 - 정자세로는 못 들어감' if wide_box.width > TRUNK_WIDTH else '작음'})")
print(f"90도 돌리면: 가로{rotated_preview.width}m x 세로{rotated_preview.depth}m "
      f"(트렁크 안에 {'들어감' if fits_dims(rotated_preview, trunk) else '그래도 안 들어감'})")

COLORS = {"Wheel_Front": "#424242", "Wheel_Rear": "#424242", "Wide_Box": "#c62828"}
fixed_scene = [SceneBox(o.box.id, o.x, o.y, o.z, o.box.width, o.box.depth, o.box.height, COLORS[o.box.id])
               for o in fixed_obstacles]
waiting_scene = [SceneBox(wide_box.id, 0, 0, 0, wide_box.width, wide_box.depth, wide_box.height, COLORS["Wide_Box"])]

draw_scene(
    TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT,
    fixed_obstacles=fixed_scene, placed_boxes=[], waiting_boxes=waiting_scene,
    title="[시나리오 3] BEFORE - Wide_Box(가로 0.65m)가 정자세로는 트렁크 폭(0.60m)보다 큼",
    out_path=str(ALGORISM_DIR / "local_test_data" / "sketch_scenario3_before.png"),
)

# ---------------------------------------------------------------------------
# [1단계] place_one_box() 딱 한 번 호출 - 내부에서 정자세 먼저 시도하고, 안
# 되면 ⑱이 자동으로 90도 돌린 자세를 다시 시도한다. 호출하는 쪽은 회전 여부를
# 신경 쓸 필요 없이 결과(plan.rotated)만 보면 된다.
# ---------------------------------------------------------------------------
state = ExtremePointState()
for obs in fixed_obstacles:
    state.register_placement(obs)

print("\n[1단계] place_one_box() 호출 (내부에서 정자세 -> 실패 시 자동 회전 시도)")
plan = place_one_box(wide_box, trunk, state, order=1, allow_stacking=True)
if plan is None:
    reason = classify_unloadable_reason(wide_box, trunk, state)
    print(f"  Wide_Box: 배치 불가 - 사유: {reason.value}")
else:
    x, y, z = plan.position
    w, d, h = plan.dimensions
    print(f"  Wide_Box: 배치 성공 x[{x:.3f},{x+w:.3f}] y[{y:.3f},{y+d:.3f}] z[{z:.3f},{z+h:.3f}]")
    print(f"  회전 여부: {'90도 회전됨 (가로/세로 교환)' if plan.rotated else '정자세 그대로'}")

placed_scene = []
waiting_after = []
if plan is None:
    waiting_after.append(SceneBox(wide_box.id, 0, 0, 0, wide_box.width, wide_box.depth, wide_box.height, COLORS["Wide_Box"]))
else:
    x, y, z = plan.position
    w, d, h = plan.dimensions
    label = f"{wide_box.id} (회전됨)" if plan.rotated else wide_box.id
    placed_scene.append(SceneBox(label, x, y, z, w, d, h, COLORS["Wide_Box"], dashed=plan.rotated))

draw_scene(
    TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT,
    fixed_obstacles=fixed_scene, placed_boxes=placed_scene, waiting_boxes=waiting_after,
    title=("[시나리오 3] AFTER - ⑱ 자동 회전으로 배치 성공 (점선 테두리=90도 회전됨)"
           if plan is not None else "[시나리오 3] AFTER - 그래도 미적재"),
    out_path=str(ALGORISM_DIR / "local_test_data" / "sketch_scenario3_after.png"),
)

print("\n=== 요약 ===")
print(f"실제 배치 결과: {'성공' if plan is not None else '미적재'}"
      + (f" (회전됨, dims={plan.dimensions})" if plan is not None and plan.rotated else ""))
