import sys, pathlib, types
from importlib import import_module

ALGORISM_DIR = pathlib.Path("/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU/isaacpjt/Cart2Trunk/algorism")
sys.path.insert(0, str(ALGORISM_DIR))

m02 = import_module("02_trunk_space_state")
m03 = import_module("03_extreme_point_candidates")
m05 = import_module("05_candidate_scoring")
m06 = import_module("06_loading_order_decision")
m07 = import_module("07_placement_plan")
m13 = import_module("13_support_check")
m15 = import_module("15_overhead_clearance_check")
m17 = import_module("17_margin_check")

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


def place_one_box_stacked_only(box, trunk, state, order):
    """z=0(바닥) 후보는 빼고 z>0(쌓기)만 본다 - 07_placement_plan.py와 같은 검증
    (③ 벽/박스 밀착 후보, ⑮ 상단 여유 공간, ⑯ 접근 경로 확인)을 그대로 적용."""
    candidate_pool = (
        state.candidates
        | generate_wall_flush_candidates(box, trunk, state.candidates)
        | generate_box_flush_candidates(box, trunk, state.candidates, state.placed)
    )
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
                          dimensions=(box.width, box.depth, box.height),
                          score=best_score, touches=best_touches)


# ---------------------------------------------------------------------------
# 새 손그림 시나리오: 트렁크에 (기존 화물 대신) 빨간 장애물 박스 2개 + 차 바퀴 2개가
# 이미 고정으로 있고, 카트에는 초록 박스 1개(1층) 위에 파란 박스 2개(2층, 초록 위에
# 얹힘)가 있다. 장애물/바퀴는 차 바퀴와 동일하게 알고리즘 미적용 고정값으로 등록하고,
# 카트 박스 3개만 ⑥ 픽업 순서(rests_on_id) + 배치 알고리즘을 적용한다.
# ---------------------------------------------------------------------------

TRUNK_WIDTH = 0.60
TRUNK_DEPTH = 0.73
TRUNK_HEIGHT = 0.50

ROBOT_TO_TRUNK_GAP = 0.30
ROBOT_Y = TRUNK_DEPTH / 2

trunk = Trunk(width=TRUNK_WIDTH, depth=TRUNK_DEPTH, height=TRUNK_HEIGHT)

wheel_front = PlacedBox(box=Box("Wheel_Front", width=0.16, depth=0.15, height=0.20),
                         x=0.44, y=0.00, z=0.0)
wheel_rear = PlacedBox(box=Box("Wheel_Rear", width=0.16, depth=0.21, height=0.20),
                        x=0.44, y=0.52, z=0.0)

# 손그림(2차, 장애물 3개로 늘어남)의 빨간 "장애물" 구도 - 입구 쪽 상단에 하나,
# 두 바퀴 사이 틈 근처에 제일 큰 것 하나, 안쪽 하단에 하나. 손그림이 정밀 치수는
# 아니라 픽셀을 완전히 역산할 순 없어서 구도(위/중간/아래 3개, 바퀴와 안 겹침)만
# 맞추고 좌표는 손으로 잡았다.
obstacle_1 = PlacedBox(box=Box("Obstacle_1", width=0.20, depth=0.23, height=0.20),
                        x=0.22, y=0.00, z=0.0)
obstacle_2 = PlacedBox(box=Box("Obstacle_2", width=0.20, depth=0.22, height=0.20),
                        x=0.38, y=0.28, z=0.0)
obstacle_3 = PlacedBox(box=Box("Obstacle_3", width=0.20, depth=0.22, height=0.20),
                        x=0.14, y=0.50, z=0.0)
fixed_obstacles = [wheel_front, wheel_rear, obstacle_1, obstacle_2, obstacle_3]

print("트렁크:", trunk)
print("고정 장애물(알고리즘 미적용):")
for obs in fixed_obstacles:
    print(f"  {obs.box.id}: x[{obs.x:.3f},{obs.x+obs.box.width:.3f}] "
          f"y[{obs.y:.3f},{obs.y+obs.box.depth:.3f}] z[{obs.z:.3f},{obs.z+obs.box.height:.3f}]")

# state: 실제로 지금 트렁크를 물리적으로 차지하고 있는 모든 것(임시+최종 다 포함) -
# 새 후보가 겹치면 안 되는 대상은 항상 이걸 기준으로 본다.
state = ExtremePointState()
for obs in fixed_obstacles:
    state.register_placement(obs)

# state_final: "다른 박스가 그 위에 안심하고 쌓여도 되는" 확정 배치만 담는다.
# 임시로 내려놓은 박스는 나중에 다른 곳으로 옮겨질 수 있으므로, 절대 여기 안 넣는다
# (안 그러면 다른 박스가 임시 박스 위에 쌓였다가, 그 임시 박스가 나중에 옮겨지면서
# 붕 뜬 채로 남는 물리적으로 불가능한 결과가 나온다).
state_final = ExtremePointState()
for obs in fixed_obstacles:
    state_final.register_placement(obs)


def aabb_overlap(a, b):
    ax0, ay0, az0, aw, ad, ah = a
    bx0, by0, bz0, bw, bd, bh = b
    ax1, ay1, az1 = ax0 + aw, ay0 + ad, az0 + ah
    bx1, by1, bz1 = bx0 + bw, by0 + bd, bz0 + bh
    return (ax0 < bx1 and ax1 > bx0) and (ay0 < by1 and ay1 > by0) and (az0 < bz1 and az1 > bz0)


def overlaps_any_temp(x, y, z, box, finalized, temp_ids):
    """state_final은 임시 박스를 모르므로, 그쪽 기준으로 찾은 후보가 실제로는
    임시 박스와 겹치지 않는지 따로 수동 확인한다."""
    cand = (x, y, z, box.width, box.depth, box.height)
    for tid in temp_ids:
        tbox = next(b for b in cart_boxes if b.id == tid)
        tx, ty, tz = finalized[tid].position
        if aabb_overlap(cand, (tx, ty, tz, tbox.width, tbox.depth, tbox.height)):
            return True
    return False


# 카트 박스 3개 - 초록 1개(1층) 위에 파랑 2개(2층, 초록 위에 얹힘 - rests_on_id).
# 2차 손그림에서 Cart_Blue2가 정사각형이 아니라 옆으로 긴 납작한 막대 모양으로
# 바뀐 걸 반영 (Cart_Blue1은 이전과 동일한 작은 정사각형 모양 유지).
cart_boxes = [
    Box("Cart_Green", width=0.20, depth=0.18, height=0.15),
    Box("Cart_Blue1", width=0.09, depth=0.10, height=0.12, rests_on_id="Cart_Green"),
    Box("Cart_Blue2", width=0.16, depth=0.06, height=0.10, rests_on_id="Cart_Green"),
]

# ---------------------------------------------------------------------------
# [1단계] 카트에서 실제로 집는 순서 (⑥ 픽업 순서 제약, rests_on_id 반영)
# ---------------------------------------------------------------------------
cart_pick_order = decide_loading_order(cart_boxes)
print("\n[1단계] 카트에서 집는 순서 (rests_on_id 반영):")
for i, b in enumerate(cart_pick_order, start=1):
    rests = f" (rests_on={b.rests_on_id})" if b.rests_on_id else " (바닥)"
    print(f"  {i}. {b.id}: volume={b.volume*1000:.2f}L{rests}")

# ---------------------------------------------------------------------------
# [2단계] 카트 피킹 순서대로 우선 시도 - 쌓을 자리가 아직 없으면(예: 그 위에
# 얹혀야 할 Cart_Green이 아직 트렁크에 없어서) 완전히 포기하지 않고, 일단 빈
# 바닥에 임시로 내려놓는다("로봇이 잠깐 다른 빈 자리에 내려뒀다가 나중에 다시
# 집어서 옮긴다"는 걸 흉내). 최종 배치가 아니라 미정 상태로 표시(temp_ids).
# 쌓기 시도는 반드시 state_final 기준으로만 한다 - 그래야 임시 박스 위에
# 잘못 쌓이는 일이 없다.
# ---------------------------------------------------------------------------
print("\n[2단계] 카트 피킹 순서대로 우선 시도 - 안 되면 빈 바닥에 임시로 내려놓기")
finalized = {}   # box_id -> PlacementPlan
temp_ids = set()  # 임시 바닥 배치라서 나중에 재배치를 시도해야 하는 박스 id
unloadable = []
order_counter = 1
for box in cart_pick_order:
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
                  f"z[{z:.3f},{z+box.height:.3f}]  [1층, 최종]  점수={plan.score:.4f} 접촉면={plan.touches}/6")
    else:
        plan = place_one_box_stacked_only(box, trunk, state_final, order=order_counter)
        if plan is not None and overlaps_any_temp(*plan.position, box, finalized, temp_ids):
            plan = None  # state_final은 임시 박스를 몰라서 골랐을 수도 있는 잘못된 후보 - 폐기
        if plan is not None:
            finalized[box.id] = plan
            state.register_placement(PlacedBox(box=box, x=plan.position[0], y=plan.position[1], z=plan.position[2]))
            x, y, z = plan.position
            print(f"  {box.id}: x[{x:.3f},{x+box.width:.3f}] y[{y:.3f},{y+box.depth:.3f}] "
                  f"z[{z:.3f},{z+box.height:.3f}]  [2층, 최종]  점수={plan.score:.4f} 접촉면={plan.touches}/6")
        else:
            temp_plan = place_one_box(box, trunk, state, order=order_counter, allow_stacking=False)
            if temp_plan is None:
                unloadable.append(box.id)
                print(f"  {box.id}: 배치 불가 (2층 자리도 없고 임시로 내려놓을 바닥도 없음)")
            else:
                finalized[box.id] = temp_plan
                temp_ids.add(box.id)
                x, y, z = temp_plan.position
                print(f"  {box.id}: 2층 자리가 아직 없어서 x[{x:.3f},{x+box.width:.3f}] "
                      f"y[{y:.3f},{y+box.depth:.3f}] z[{z:.3f},{z+box.height:.3f}]  [1층, 임시] 에 내려놓음")
    order_counter += 1

# ---------------------------------------------------------------------------
# [3단계] 임시로 내려놓은 박스가 있으면, Cart_Green 등 나머지가 자리 잡은 뒤
# state_final(확정만 있는 상태) 기준으로 다시 원래 목적지(2층)로 재배치를
# 시도한다. 성공하면 임시 바닥 자리는 버리고 2층으로 옮기고(그리고 state_final에도
# 등록해서 다음 임시 박스가 그 위에 쌓이는 것도 허용), 실패하면 임시 위치를
# 그대로 최종으로 쓴다.
# ---------------------------------------------------------------------------
if temp_ids:
    print("\n[3단계] 임시로 내려놓은 박스 재배치 시도 (이제 Cart_Green이 자리를 잡았는지 확인)")
    for box in cart_pick_order:
        if box.id not in temp_ids:
            continue
        # place_one_box_stacked_only()가 성공하면 state_final에 이미 내부에서
        # register_placement()를 해버리므로, 여기서 또 등록하면 같은 박스가 두 번
        # 잡혀 다음 박스의 받침 비율 계산이 부풀려지는 버그가 된다 (시나리오2에서
        # 실제로 겪음) - 그래서 여기서는 다시 등록하지 않는다.
        new_plan = place_one_box_stacked_only(box, trunk, state_final, order=finalized[box.id].order)
        if new_plan is not None and overlaps_any_temp(*new_plan.position, box, finalized, temp_ids - {box.id}):
            new_plan = None
        if new_plan is not None:
            old_x, old_y, old_z = finalized[box.id].position
            finalized[box.id] = new_plan
            temp_ids.discard(box.id)
            x, y, z = new_plan.position
            print(f"  {box.id}: 임시 바닥({old_x:.3f},{old_y:.3f},{old_z:.3f}) 대신 "
                  f"x[{x:.3f},{x+box.width:.3f}] y[{y:.3f},{y+box.depth:.3f}] z[{z:.3f},{z+box.height:.3f}] "
                  f"[2층, 재배치 성공]으로 옮김")
        else:
            x, y, z = finalized[box.id].position
            print(f"  {box.id}: 재배치할 2층 자리 없음 - 임시 바닥 위치 "
                  f"x[{x:.3f},{x+box.width:.3f}] y[{y:.3f},{y+box.depth:.3f}] z[{z:.3f},{z+box.height:.3f}] 그대로 유지")

plans = [finalized[box.id] for box in cart_boxes if box.id in finalized]

# ---------------------------------------------------------------------------
# 안전 확인: 2층(z>0)에 최종 배치된 박스는 전부, ⑬(13_support_check.py)이 쓰는
# 것과 똑같은 기준(받침 비율 80% 이상)으로 다시 검증한다 - 임시 박스가 나중에
# 옮겨지면서 그 위에 얹혀 있던 게 붕 뜨는 사고가 없는지 마지막으로 한 번 더
# 확인하는 것. (100% 완전 지지를 요구하면 ⑬의 실제 기준보다 엄격해서 거짓
# 경보가 남 - 반드시 같은 min_support_ratio=0.8 기준을 그대로 재사용해야 함.)
# ---------------------------------------------------------------------------
compute_support_ratio = m13.compute_support_ratio

_all_placed = list(fixed_obstacles)
for p in plans:
    b = next(bb for bb in cart_boxes if bb.id == p.box_id)
    _all_placed.append(PlacedBox(box=b, x=p.position[0], y=p.position[1], z=p.position[2]))

for p in plans:
    x, y, z = p.position
    if z > 1e-9:
        b = next(bb for bb in cart_boxes if bb.id == p.box_id)
        others = [pb for pb in _all_placed if pb.box.id != p.box_id]
        ratio = compute_support_ratio(x, y, z, b, others)
        assert ratio >= 0.8 - 1e-9, \
            f"물리적으로 불가능: {p.box_id}가 z={z:.3f}에 있는데 받침 비율이 {ratio:.1%}로 기준(80%) 미달"
print(f"\n[안전 확인] 2층 배치 {sum(1 for p in plans if p.position[2] > 1e-9)}개 전부 받침 비율 80% 이상 확인됨")

# ---- 시각화 ----
_local_site = next(p for p in sys.path if p.endswith("site-packages") and "/.local/" in p)
_pkg = types.ModuleType("mpl_toolkits")
_pkg.__path__ = [str(pathlib.Path(_local_site) / "mpl_toolkits")]
sys.modules["mpl_toolkits"] = _pkg

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def cuboid_faces(x0, y0, z0, dx, dy, dz):
    x1, y1, z1 = x0 + dx, y0 + dy, z0 + dz
    v = [
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),
    ]
    return [
        [v[0], v[1], v[2], v[3]], [v[4], v[5], v[6], v[7]],
        [v[0], v[1], v[5], v[4]], [v[2], v[3], v[7], v[6]],
        [v[1], v[2], v[6], v[5]], [v[0], v[3], v[7], v[4]],
    ]


def draw_cuboid(ax, x0, y0, z0, dx, dy, dz, facecolor, edgecolor, alpha=0.85, linewidth=1.0):
    coll = Poly3DCollection(cuboid_faces(x0, y0, z0, dx, dy, dz), facecolor=facecolor,
                             edgecolor=edgecolor, alpha=alpha, linewidths=linewidth)
    ax.add_collection3d(coll)


def draw_wireframe_box(ax, x0, y0, z0, dx, dy, dz, color, label=None):
    x1, y1, z1 = x0 + dx, y0 + dy, z0 + dz
    v = {
        "000": (x0, y0, z0), "100": (x1, y0, z0), "110": (x1, y1, z0), "010": (x0, y1, z0),
        "001": (x0, y0, z1), "101": (x1, y0, z1), "111": (x1, y1, z1), "011": (x0, y1, z1),
    }
    edges = [
        ("000", "100"), ("100", "110"), ("110", "010"), ("010", "000"),
        ("001", "101"), ("101", "111"), ("111", "011"), ("011", "001"),
        ("000", "001"), ("100", "101"), ("110", "111"), ("010", "011"),
    ]
    for a, b in edges:
        xs, ys, zs = zip(v[a], v[b])
        ax.plot3D(xs, ys, zs, color=color, linewidth=2)
    if label:
        ax.plot([], [], color=color, linewidth=2, label=label)


COLORS = {"Cart_Green": "#43a047", "Cart_Blue1": "#0d47a1", "Cart_Blue2": "#1e88e5"}

G = ROBOT_TO_TRUNK_GAP
fig = plt.figure(figsize=(13, 7))

ax3d = fig.add_subplot(1, 2, 1, projection="3d")
ax3d.scatter([0], [ROBOT_Y], [0], color="crimson", s=80, label="로봇 (base 원점)")
ax3d.quiver(0, ROBOT_Y, 0, G, 0, 0, color="crimson", linewidth=2, arrow_length_ratio=0.15)
draw_wireframe_box(ax3d, G, 0, 0, TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT, color="red", label="trunk")
for obs in [wheel_front, wheel_rear]:
    draw_cuboid(ax3d, G + obs.x, obs.y, obs.z, obs.box.width, obs.box.depth, obs.box.height,
                facecolor="black", edgecolor="black", alpha=0.6)
ax3d.plot([], [], color="black", linewidth=2, label="차 바퀴")
for obs in [obstacle_1, obstacle_2, obstacle_3]:
    draw_cuboid(ax3d, G + obs.x, obs.y, obs.z, obs.box.width, obs.box.depth, obs.box.height,
                facecolor="#e53935", edgecolor="#b71c1c", alpha=0.5)
ax3d.plot([], [], color="#e53935", linewidth=2, label="장애물(고정)")
for plan in plans:
    x, y, z = plan.position
    box = next(b for b in cart_boxes if b.id == plan.box_id)
    draw_cuboid(ax3d, G + x, y, z, box.width, box.depth, box.height,
                facecolor=COLORS[plan.box_id], edgecolor=COLORS[plan.box_id])

ax3d.set_xlim(0, G + TRUNK_WIDTH)
ax3d.set_ylim(0, TRUNK_DEPTH)
ax3d.set_zlim(0, TRUNK_HEIGHT)
ax3d.set_xlabel("x (m) - 로봇 원점 기준, 로봇이 접근하는 방향")
ax3d.set_ylabel("y (m) - 좌우(입구와 무관)")
ax3d.set_zlabel("z (m, height)")
ax3d.set_title("3D 아이소메트릭 - 장애물 3개 + 카트 박스 3개 적재")
ax3d.legend(loc="upper left", fontsize=7)
ax3d.set_box_aspect((G + TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT))

ax2d = fig.add_subplot(1, 2, 2)
ax2d.scatter([0], [ROBOT_Y], color="crimson", s=100, zorder=5, label="로봇 (base 원점)")
ax2d.annotate("", xy=(G, ROBOT_Y), xytext=(0, ROBOT_Y),
              arrowprops=dict(arrowstyle="->", color="crimson", linewidth=2))
ax2d.text(G / 2, ROBOT_Y + 0.02, "접근 방향(+x, 고정)", color="crimson", fontsize=8, ha="center")
ax2d.add_patch(Rectangle((G, 0), TRUNK_WIDTH, TRUNK_DEPTH, fill=False, edgecolor="red", linewidth=2, label="trunk"))
for obs in [wheel_front, wheel_rear]:
    ax2d.add_patch(Rectangle((G + obs.x, obs.y), obs.box.width, obs.box.depth, facecolor="black",
                              edgecolor="black", alpha=0.6))
for obs in [obstacle_1, obstacle_2, obstacle_3]:
    ax2d.add_patch(Rectangle((G + obs.x, obs.y), obs.box.width, obs.box.depth, facecolor="#e53935",
                              edgecolor="#b71c1c", alpha=0.5))
    ax2d.text(G + obs.x + obs.box.width / 2, obs.y + obs.box.depth / 2, obs.box.id,
              fontsize=6, ha="center", va="center", color="white", weight="bold")

for plan in plans:
    x, y, z = plan.position
    box = next(b for b in cart_boxes if b.id == plan.box_id)
    color = COLORS[plan.box_id]
    is_stacked = z > 1e-9
    ax2d.add_patch(Rectangle((G + x, y), box.width, box.depth, facecolor=color,
                              edgecolor=("navy" if is_stacked else color),
                              alpha=0.9, linewidth=(2.2 if is_stacked else 1.3),
                              linestyle=("--" if is_stacked else "-")))
    ax2d.text(G + x + box.width / 2, y + box.depth / 2, plan.box_id,
              fontsize=6, ha="center", va="center", color="white", weight="bold")

ax2d.set_xlim(-0.05, G + TRUNK_WIDTH + 0.05)
ax2d.set_ylim(-0.05, TRUNK_DEPTH + 0.05)
ax2d.set_xlabel("x (m) - 로봇 원점 기준, 로봇이 접근하는 방향")
ax2d.set_ylabel("y (m) - 좌우(입구와 무관)")
ax2d.set_title("top-down - 빨강(반투명)=고정 장애물, 점선=2층")
ax2d.legend(loc="upper left", fontsize=7)
ax2d.set_aspect("equal")

plt.tight_layout()
out_path = str(ALGORISM_DIR / "local_test_data" / "sketch_placement_obstacles_result.png")
plt.savefig(out_path, dpi=130)
print("\n그래프 저장:", out_path)

print("\n=== 요약 ===")
print(f"카트 박스 3개 중 {len(plans)}개 배치 성공")
print(f"미적재: {unloadable if unloadable else '없음'}")
