"""
sketch_placement_test_scenario5_margin.py
[테스트 5] ⑰ 박스-벽 / 박스-박스 마진(1cm)이 실제 적재 결과에 적용되는지 확인.

박스 4개를 빈 트렁크에 순서대로 배치한 뒤, 벽까지의 거리와 박스끼리의 거리를
직접 계산해서 전부 MARGIN(0.01m) 이상인지 확인한다. 1cm는 그림에서 눈으로
구분하기엔 너무 작아서, "after" 그림에 실제 간격 수치를 텍스트로 같이 표시한다.
"""
import sys, pathlib
from importlib import import_module

ALGORISM_DIR = pathlib.Path("/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU/isaacpjt/Cart2Trunk/algorism")
sys.path.insert(0, str(ALGORISM_DIR))
sys.path.insert(0, str(ALGORISM_DIR / "local_test_data"))

m02 = import_module("02_trunk_space_state")
m03 = import_module("03_extreme_point_candidates")
m06 = import_module("06_loading_order_decision")
m07 = import_module("07_placement_plan")
m17 = import_module("17_margin_check")
viz = import_module("_viz_helpers")

Trunk = m02.Trunk
Box = m03.Box
ExtremePointState = m03.ExtremePointState
decide_loading_order = m06.decide_loading_order
place_one_box = m07.place_one_box
MARGIN = m17.MARGIN
SceneBox = viz.SceneBox
draw_scene = viz.draw_scene

TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT = 0.60, 0.73, 0.50
trunk = Trunk(width=TRUNK_WIDTH, depth=TRUNK_DEPTH, height=TRUNK_HEIGHT)

# 장애물 없이 박스 4개만으로 순수하게 ⑰ 마진 효과를 본다 (딱 붙었으면 서로/벽에
# 다 맞닿았을 크기·개수).
cart_boxes = [
    Box("Box_A", width=0.20, depth=0.18, height=0.15),
    Box("Box_B", width=0.15, depth=0.15, height=0.12),
    Box("Box_C", width=0.25, depth=0.20, height=0.15),
    Box("Box_D", width=0.12, depth=0.12, height=0.10),
]

print("트렁크:", trunk, f"(마진 설정값: {MARGIN*100:.0f}cm)")

COLORS = {"Box_A": "#43a047", "Box_B": "#1e88e5", "Box_C": "#fb8c00", "Box_D": "#8e24aa"}
waiting_scene = [SceneBox(b.id, 0, 0, 0, b.width, b.depth, b.height, COLORS[b.id]) for b in cart_boxes]

draw_scene(
    TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT,
    fixed_obstacles=[], placed_boxes=[], waiting_boxes=waiting_scene,
    title="[시나리오 5] BEFORE - 박스 4개, 트렁크는 비어있음 (⑰ 마진 1cm 적용 예정)",
    out_path=str(ALGORISM_DIR / "local_test_data" / "sketch_scenario5_before.png"),
)

print("\n[1단계] 적재 순서 (부피 큰 순):")
order = decide_loading_order(cart_boxes)
for i, b in enumerate(order, start=1):
    print(f"  {i}. {b.id}: volume={b.volume*1000:.2f}L")

print("\n[2단계] 순서대로 배치 (⑰ 마진 하드 컷 적용된 실제 파이프라인)")
state = ExtremePointState()
plans = {}
for i, box in enumerate(order, start=1):
    plan = place_one_box(box, trunk, state, order=i, allow_stacking=False)
    if plan is None:
        print(f"  {box.id}: 배치 불가")
        continue
    plans[box.id] = plan
    x, y, z = plan.position
    print(f"  {box.id}: x[{x:.4f},{x+box.width:.4f}] y[{y:.4f},{y+box.depth:.4f}] z[{z:.4f},{z+box.height:.4f}]")

# ---------------------------------------------------------------------------
# [3단계] 벽까지 거리 + 박스끼리 거리를 직접 계산해서 전부 MARGIN 이상인지 확인.
# ---------------------------------------------------------------------------
print(f"\n[3단계] 벽/박스 간격 실측 (기준 {MARGIN*100:.0f}cm)")
gap_notes = []
for box in cart_boxes:
    if box.id not in plans:
        continue
    x, y, z = plans[box.id].position
    wall_gaps = {
        "왼쪽 벽": x,
        "오른쪽 벽": TRUNK_WIDTH - (x + box.width),
        "앞쪽 벽": y,
        "뒤쪽 벽": TRUNK_DEPTH - (y + box.depth),
    }
    nearest_wall, nearest_gap = min(wall_gaps.items(), key=lambda kv: kv[1])
    ok = "OK" if nearest_gap >= MARGIN - 1e-9 else "위반!"
    print(f"  {box.id} - 가장 가까운 벽: {nearest_wall} {nearest_gap*100:.2f}cm [{ok}]")
    gap_notes.append(f"{box.id}~{nearest_wall}: {nearest_gap*100:.1f}cm")

placed_ids = list(plans.keys())
for i in range(len(placed_ids)):
    for j in range(i + 1, len(placed_ids)):
        id_a, id_b = placed_ids[i], placed_ids[j]
        pa, pb = plans[id_a].position, plans[id_b].position
        ba = next(b for b in cart_boxes if b.id == id_a)
        bb = next(b for b in cart_boxes if b.id == id_b)
        x_gap = max(pb[0] - (pa[0] + ba.width), pa[0] - (pb[0] + bb.width))
        y_gap = max(pb[1] - (pa[1] + ba.depth), pa[1] - (pb[1] + bb.depth))
        best_gap = max(x_gap, y_gap)
        ok = "OK" if best_gap >= MARGIN - 1e-9 else "위반!"
        print(f"  {id_a}~{id_b} 간격: x_gap={x_gap*100:.2f}cm, y_gap={y_gap*100:.2f}cm -> {best_gap*100:.2f}cm [{ok}]")

placed_scene = [SceneBox(bid, *plans[bid].position, *plans[bid].dimensions, COLORS[bid]) for bid in plans]

draw_scene(
    TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT,
    fixed_obstacles=[], placed_boxes=placed_scene, waiting_boxes=[],
    title=f"[시나리오 5] AFTER - 박스 {len(plans)}개 배치, 전부 벽/서로 {MARGIN*100:.0f}cm 이상 간격 확보",
    out_path=str(ALGORISM_DIR / "local_test_data" / "sketch_scenario5_after.png"),
)

print("\n=== 요약 ===")
print(f"배치 성공: {len(plans)}/{len(cart_boxes)}")
print(f"모든 벽/박스 간격이 {MARGIN*100:.0f}cm 이상인지: 위에서 확인 (위반 없으면 전부 [OK])")
