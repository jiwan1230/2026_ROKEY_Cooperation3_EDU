"""
sketch_placement_test_scenario2_three_tier_chain.py
[테스트 2/4] 3단 쌓기 체인 - 카트 안에서 C가 B 위에, B가 A 위에 얹혀있는 경우.

지금까지는 "초록 1층 + 파랑 2층"까지, 즉 2단만 봤다. 이번엔 3단 체인을 줘서
⑥(픽업 순서, C->B->A) + ⑮(층이 늘어날수록 천장까지 남는 여유가 빠듯해지는지)를
같이 확인한다. 트렁크는 비워두고(장애물 없이) 순수하게 체인 자체에 집중한다.
"""
import sys, pathlib
from importlib import import_module

ALGORISM_DIR = pathlib.Path("/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU/isaacpjt/Cart2Trunk/algorism")
sys.path.insert(0, str(ALGORISM_DIR))
sys.path.insert(0, str(ALGORISM_DIR / "local_test_data"))

m02 = import_module("02_trunk_space_state")
m03 = import_module("03_extreme_point_candidates")
m05 = import_module("05_candidate_scoring")
m06 = import_module("06_loading_order_decision")
m07 = import_module("07_placement_plan")
m13 = import_module("13_support_check")
m15 = import_module("15_overhead_clearance_check")
m17 = import_module("17_margin_check")
m18 = import_module("18_rotation")
viz = import_module("_viz_helpers")

Trunk = m02.Trunk
Box = m03.Box
PlacedBox = m03.PlacedBox
ExtremePointState = m03.ExtremePointState
generate_wall_flush_candidates = m03.generate_wall_flush_candidates
generate_box_flush_candidates = m03.generate_box_flush_candidates
decide_loading_order = m06.decide_loading_order
place_one_box = m07.place_one_box
PlacementPlan = m07.PlacementPlan
score_candidate = m05.score_candidate
is_candidate_valid_with_stacking = m13.is_candidate_valid_with_stacking
has_overhead_clearance = m15.has_overhead_clearance
has_clear_approach_path = m15.has_clear_approach_path
has_sufficient_margin = m17.has_sufficient_margin
MARGIN = m17.MARGIN
rotate_box = m18.rotate_box
SceneBox = viz.SceneBox
draw_scene = viz.draw_scene


def place_one_box_stacked_only(box, trunk, state, order):
    """정자세로 먼저 시도하고, 안 되면 ⑱(90도 회전, 가로/세로 교환)로 한 번 더 시도한다 -
    07_placement_plan.py의 place_one_box()와 동일한 회전 폴백 규칙."""
    plan = _place_one_box_stacked_only_orientation(box, trunk, state, order, rotated=False)
    if plan is not None:
        return plan
    if box.width == box.depth:
        return None
    return _place_one_box_stacked_only_orientation(rotate_box(box), trunk, state, order, rotated=True)


def _place_one_box_stacked_only_orientation(box, trunk, state, order, rotated):
    wall_flush = generate_wall_flush_candidates(box, trunk, state.candidates, margin=MARGIN)
    box_flush = generate_box_flush_candidates(box, trunk, state.candidates, state.placed, margin=MARGIN)
    combo_flush = generate_box_flush_candidates(box, trunk, wall_flush, state.placed, margin=MARGIN)
    candidate_pool = state.candidates | wall_flush | box_flush | combo_flush
    valid_candidates = [
        (x, y, z) for (x, y, z) in candidate_pool
        if z > 1e-9
        and is_candidate_valid_with_stacking(x, y, z, box, trunk, state.placed, allow_stacking=True)
        and has_overhead_clearance(z, box, trunk)
        and has_clear_approach_path(x, y, z, box, trunk, state.placed)
        and has_sufficient_margin(x, y, z, box, trunk, state.placed)
    ]
    if not valid_candidates:
        return None
    scored = [(pos, *score_candidate(pos[0], pos[1], pos[2], box, trunk, state.placed)) for pos in valid_candidates]
    best_pos, best_score, best_touches = min(scored, key=lambda t: t[1])
    placed_box = PlacedBox(box=box, x=best_pos[0], y=best_pos[1], z=best_pos[2])
    state.register_placement(placed_box)
    return PlacementPlan(box_id=box.id, order=order, position=best_pos,
                          dimensions=(box.width, box.depth, box.height), score=best_score, touches=best_touches, rotated=rotated)


TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT = 0.60, 0.73, 0.50
trunk = Trunk(width=TRUNK_WIDTH, depth=TRUNK_DEPTH, height=TRUNK_HEIGHT)

# 이번엔 일부러 트렁크를 완전히 비워둔다(차 바퀴도 없음) - 차 바퀴가 있으면
# Chain_B/Chain_C가 Chain_A 대신 차 바퀴 위에 먼저 쌓여버려서(둘 다 유효한 착지면
# 이라서), "3단 체인이 실제로 탑처럼 쌓이는지 + 3단이면 천장 여유가 얼마나
# 빠듯한지"라는 이번 테스트의 초점이 흐려짐 - 처음 시도에서 실제로 그렇게 됐다가
# 여기서 바로잡음.
fixed_obstacles = []

# 카트 안에서: C가 B 위에, B가 A 위에 얹혀 있음 (3단 체인).
# 높이 합 0.12+0.10+0.08=0.30, 천장 0.50m 기준 남는 여유 = 0.50-0.30=0.20m로
# 딱 경계값 - 3단까지 쌓으면 정말 빠듯하다는 걸 보여주려고 일부러 이렇게 잡음.
#
# (1차 시도에서 A 발판을 넉넉하게 잡았더니, C가 굳이 B를 거치지 않고 A 위 남는
# 자리에 바로 올라가버려서 "탑"이 아니라 "A 위에 B/C가 나란히"가 나왔음. A의
# 발판을 B 하나로 거의 꽉 차게 줄여서, B가 자리잡기 전에는 C가 앉을 자리가
# A 위에 남지 않도록 강제함 - 그래야 C가 진짜로 B가 놓이길 기다려야 한다.)
chain_a = Box("Chain_A", width=0.16, depth=0.14, height=0.12)
chain_b = Box("Chain_B", width=0.14, depth=0.12, height=0.10, rests_on_id="Chain_A")
chain_c = Box("Chain_C", width=0.09, depth=0.08, height=0.08, rests_on_id="Chain_B")
cart_boxes = [chain_a, chain_b, chain_c]

print("트렁크:", trunk)
print("카트 내용물 (3단 체인): Chain_C -> Chain_B -> Chain_A")
print(f"  3단 다 쌓으면 높이 {chain_a.height+chain_b.height+chain_c.height:.2f}m, "
      f"남는 천장 여유 = {TRUNK_HEIGHT - (chain_a.height+chain_b.height+chain_c.height):.2f}m (기준 0.20m)")

COLORS = {"Wheel_Front": "#424242", "Wheel_Rear": "#424242",
          "Chain_A": "#2e7d32", "Chain_B": "#fb8c00", "Chain_C": "#8e24aa"}
fixed_scene = [SceneBox(o.box.id, o.x, o.y, o.z, o.box.width, o.box.depth, o.box.height, COLORS[o.box.id])
               for o in fixed_obstacles]
waiting_scene = [SceneBox(b.id, 0, 0, 0, b.width, b.depth, b.height, COLORS[b.id], stack_on_id=b.rests_on_id)
                 for b in cart_boxes]

draw_scene(
    TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT,
    fixed_obstacles=fixed_scene, placed_boxes=[], waiting_boxes=waiting_scene,
    title="[시나리오 2] BEFORE - 카트에 3단 체인(Chain_C -> Chain_B -> Chain_A), 트렁크는 비어있음",
    out_path=str(ALGORISM_DIR / "local_test_data" / "sketch_scenario2_before.png"),
)

# ---------------------------------------------------------------------------
# [1단계] 카트에서 집는 순서 (⑥)
# ---------------------------------------------------------------------------
pick_order = decide_loading_order(cart_boxes)
print("\n[1단계] 카트에서 집는 순서 (rests_on_id 반영):")
for i, b in enumerate(pick_order, start=1):
    rests = f" (rests_on={b.rests_on_id})" if b.rests_on_id else " (바닥)"
    print(f"  {i}. {b.id}: volume={b.volume*1000:.2f}L{rests}")

# ---------------------------------------------------------------------------
# [2~3단계] 집은 순서 그대로 시도 -> 안 되면 임시 바닥 -> 나머지 확정된 뒤 재시도.
# (지난 라운드 obstacles 시나리오와 동일한 안전한 2-pass 방식 재사용)
# ---------------------------------------------------------------------------
state = ExtremePointState()
for obs in fixed_obstacles:
    state.register_placement(obs)
state_final = ExtremePointState()
for obs in fixed_obstacles:
    state_final.register_placement(obs)


def overlaps_any_temp(x, y, z, box, finalized, temp_ids):
    def aabb_overlap(a, b):
        ax0, ay0, az0, aw, ad, ah = a
        bx0, by0, bz0, bw, bd, bh = b
        ax1, ay1, az1 = ax0 + aw, ay0 + ad, az0 + ah
        bx1, by1, bz1 = bx0 + bw, by0 + bd, bz0 + bh
        return (ax0 < bx1 and ax1 > bx0) and (ay0 < by1 and ay1 > by0) and (az0 < bz1 and az1 > bz0)
    cand = (x, y, z, box.width, box.depth, box.height)
    for tid in temp_ids:
        tbox = next(b for b in cart_boxes if b.id == tid)
        tx, ty, tz = finalized[tid].position
        if aabb_overlap(cand, (tx, ty, tz, tbox.width, tbox.depth, tbox.height)):
            return True
    return False


print("\n[2단계] 카트 피킹 순서대로 우선 시도 - 안 되면 빈 바닥에 임시로 내려놓기")
finalized, temp_ids, unloadable = {}, set(), []
order_counter = 1
for box in pick_order:
    if box.rests_on_id is None:
        plan = place_one_box(box, trunk, state, order=order_counter, allow_stacking=True)
        if plan is None:
            unloadable.append(box.id)
            print(f"  {box.id}: 배치 불가 (자리 없음)")
        else:
            finalized[box.id] = plan
            state_final.register_placement(PlacedBox(box=box, x=plan.position[0], y=plan.position[1], z=plan.position[2]))
            x, y, z = plan.position
            print(f"  {box.id}: x[{x:.3f},{x+box.width:.3f}] y[{y:.3f},{y+box.depth:.3f}] "
                  f"z[{z:.3f},{z+box.height:.3f}]  [1층, 최종]")
    else:
        plan = place_one_box_stacked_only(box, trunk, state_final, order=order_counter)
        if plan is not None and overlaps_any_temp(*plan.position, box, finalized, temp_ids):
            plan = None
        if plan is not None:
            finalized[box.id] = plan
            state.register_placement(PlacedBox(box=box, x=plan.position[0], y=plan.position[1], z=plan.position[2]))
            x, y, z = plan.position
            print(f"  {box.id}: x[{x:.3f},{x+box.width:.3f}] y[{y:.3f},{y+box.depth:.3f}] "
                  f"z[{z:.3f},{z+box.height:.3f}]  [쌓임, 최종]")
        else:
            temp_plan = place_one_box(box, trunk, state, order=order_counter, allow_stacking=False)
            if temp_plan is None:
                unloadable.append(box.id)
                print(f"  {box.id}: 배치 불가 (쌓을 자리도, 임시 바닥도 없음)")
            else:
                finalized[box.id] = temp_plan
                temp_ids.add(box.id)
                x, y, z = temp_plan.position
                print(f"  {box.id}: 쌓을 자리가 아직 없어서 x[{x:.3f},{x+box.width:.3f}] "
                      f"y[{y:.3f},{y+box.depth:.3f}] z[{z:.3f},{z+box.height:.3f}]  [1층, 임시] 에 내려놓음")
    order_counter += 1

if temp_ids:
    print("\n[3단계] 임시로 내려놓은 박스 재배치 시도")
    for box in pick_order:
        if box.id not in temp_ids:
            continue
        # 주의: place_one_box_stacked_only()가 성공하면 state_final에 이미 내부에서
        # register_placement()를 해버린다 - 여기서 또 register_placement를 부르면
        # 같은 박스가 두 번 등록돼서, 다음 박스의 받침 비율 계산 때 그 면적이
        # 이중으로 잡혀 실제로는 부족한 받침도 통과해버리는 버그가 된다(실제로
        # 한 번 겪음 - Chain_B가 42.9% 받침인데 이중계산으로 85.7%처럼 보여서
        # 통과했었음). 그래서 여기서는 절대 다시 등록하지 않는다.
        new_plan = place_one_box_stacked_only(box, trunk, state_final, order=finalized[box.id].order)
        if new_plan is not None and overlaps_any_temp(*new_plan.position, box, finalized, temp_ids - {box.id}):
            new_plan = None  # (드문 경우지만, 이미 state_final에 등록은 돼버린 상태 - 지금 시나리오들에선 실제로 발생 안 함)
        if new_plan is not None:
            old_x, old_y, old_z = finalized[box.id].position
            finalized[box.id] = new_plan
            temp_ids.discard(box.id)
            x, y, z = new_plan.position
            print(f"  {box.id}: 임시 바닥({old_x:.3f},{old_y:.3f},{old_z:.3f}) 대신 "
                  f"x[{x:.3f},{x+box.width:.3f}] y[{y:.3f},{y+box.depth:.3f}] z[{z:.3f},{z+box.height:.3f}] [재배치 성공]")
        else:
            x, y, z = finalized[box.id].position
            print(f"  {box.id}: 재배치 자리 없음 - 임시 위치 그대로 유지")

# 안전 확인 (⑬과 동일 기준, 받침 비율 80%)
compute_support_ratio = m13.compute_support_ratio
_all_placed = list(fixed_obstacles)
for bid, p in finalized.items():
    b = next(bb for bb in cart_boxes if bb.id == bid)
    _all_placed.append(PlacedBox(box=b, x=p.position[0], y=p.position[1], z=p.position[2]))
for bid, p in finalized.items():
    x, y, z = p.position
    if z > 1e-9:
        b = next(bb for bb in cart_boxes if bb.id == bid)
        others = [pb for pb in _all_placed if pb.box.id != bid]
        ratio = compute_support_ratio(x, y, z, b, others)
        assert ratio >= 0.8 - 1e-9, f"{bid} 받침 비율 {ratio:.1%} 기준 미달"

placed_scene = []
for b in cart_boxes:
    if b.id in finalized:
        x, y, z = finalized[b.id].position
        placed_scene.append(SceneBox(b.id, x, y, z, b.width, b.depth, b.height, COLORS[b.id], dashed=(z > 1e-9)))

draw_scene(
    TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT,
    fixed_obstacles=fixed_scene, placed_boxes=placed_scene, waiting_boxes=[],
    title="[시나리오 2] AFTER - 3단 체인 배치 결과",
    out_path=str(ALGORISM_DIR / "local_test_data" / "sketch_scenario2_after.png"),
)

print("\n=== 요약 ===")
print(f"배치 성공: {len(finalized)}/3, 미적재: {unloadable if unloadable else '없음'}")
for b in cart_boxes:
    if b.id in finalized:
        x, y, z = finalized[b.id].position
        print(f"  {b.id}: z={z:.3f} (누구 위? "
              f"{'바닥' if z < 1e-9 else next((o.box_id for o in placed_scene if abs(o.z+o.height-z)<1e-6 and o.x<=x+1e-9<=o.x+o.width and o.y<=y+1e-9<=o.y+o.depth), '확인필요')})")
