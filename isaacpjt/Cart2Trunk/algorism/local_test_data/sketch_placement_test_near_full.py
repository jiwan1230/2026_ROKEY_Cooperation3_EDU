import sys, pathlib, types
from importlib import import_module

ALGORISM_DIR = pathlib.Path("/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU/isaacpjt/Cart2Trunk/algorism")
sys.path.insert(0, str(ALGORISM_DIR))

m02 = import_module("02_trunk_space_state")
m03 = import_module("03_extreme_point_candidates")
m05 = import_module("05_candidate_scoring")
m06 = import_module("06_loading_order_decision")
m07 = import_module("07_placement_plan")
m08 = import_module("08_unloadable_reason")
m13 = import_module("13_support_check")

Trunk = m02.Trunk
Box = m03.Box
PlacedBox = m03.PlacedBox
ExtremePointState = m03.ExtremePointState
generate_wall_flush_candidates = m03.generate_wall_flush_candidates
decide_loading_order = m06.decide_loading_order
place_one_box = m07.place_one_box
PlacementPlan = m07.PlacementPlan
score_candidate = m05.score_candidate
is_candidate_valid_with_stacking = m13.is_candidate_valid_with_stacking
classify_unloadable_reason = m08.classify_unloadable_reason


def place_one_box_stacked_only(box, trunk, state, order):
    """z=0(바닥) 후보는 빼고 z>0(쌓기)만 본다 - '2층에 놓는다'는 지정을 그대로 반영."""
    candidate_pool = state.candidates | generate_wall_flush_candidates(box, trunk, state.candidates)
    valid_candidates = [
        (x, y, z) for (x, y, z) in candidate_pool
        if z > 1e-9 and is_candidate_valid_with_stacking(x, y, z, box, trunk, state.placed, allow_stacking=True)
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
# 이번 시나리오: 이전 두 번과 다르게, 트렁크에 이미 큰 초록 박스 5개가 "확정된 채로"
# 꽉 차있는 상태(재배치 불가 - 이미 로봇이 놓고 간 것들)에서, 카트에서 새로 온 박스
# 3개(초록 1층 1개 + 파랑 2층 2개, 지난번과 같은 크기)가 그 좁아진 공간에도 잘
# 들어가는지 스트레스 테스트한다. 그래서 이번엔 "전부 합쳐서 재계산"이 아니라
# 지난번 첫 시도처럼 "기존 상태 고정 + 신규만 새로 배치" 방식으로 진행한다.
# ---------------------------------------------------------------------------

TRUNK_WIDTH = 0.60
TRUNK_DEPTH = 0.73
TRUNK_HEIGHT = 0.40

ROBOT_TO_TRUNK_GAP = 0.30
ROBOT_Y = TRUNK_DEPTH / 2

trunk = Trunk(width=TRUNK_WIDTH, depth=TRUNK_DEPTH, height=TRUNK_HEIGHT)

wheel_front = PlacedBox(box=Box("Wheel_Front", width=0.16, depth=0.15, height=0.20),
                         x=0.44, y=0.00, z=0.0)
wheel_rear = PlacedBox(box=Box("Wheel_Rear", width=0.16, depth=0.21, height=0.20),
                        x=0.44, y=0.52, z=0.0)
obstacles = [wheel_front, wheel_rear]

# 이미 확정된 상태 - 손그림처럼 큰 초록 박스 5개가 트렁크를 거의 채우고 있음
filled_boxes = [
    Box("F_Tall", width=0.11, depth=0.15, height=0.15),
    Box("F_Small", width=0.06, depth=0.08, height=0.15),
    Box("F_BigLeft", width=0.18, depth=0.21, height=0.15),
    Box("F_BigRight", width=0.22, depth=0.30, height=0.15),   # 손그림에서 보라 테두리로 강조된 제일 큰 박스
    Box("F_Wide", width=0.28, depth=0.05, height=0.15),
]

# 카트에서 새로 온 것 - 지난번과 같은 크기(초록 1개 1층 + 파랑 2개 2층)
cart_green_box = Box("Cart_Green", width=0.20, depth=0.18, height=0.15)
cart_blue_boxes = [
    Box("Cart_Blue1", width=0.09, depth=0.10, height=0.12),
    Box("Cart_Blue2", width=0.12, depth=0.14, height=0.12),
]

print("트렁크:", trunk)

state = ExtremePointState()
for obs in obstacles:
    state.register_placement(obs)

# ---- 1단계: 이미 확정된 5개를 먼저 채워서 "거의 가득 찬" 상태를 만든다 ----
order_counter = 1
filled_plans, filled_unloadable = [], []
for box in decide_loading_order(filled_boxes):
    plan = place_one_box(box, trunk, state, order=order_counter, allow_stacking=False)
    order_counter += 1
    if plan is None:
        filled_unloadable.append(box.id)
    else:
        filled_plans.append(plan)

used_floor_area = sum(p.box.width * p.box.depth for p in state.placed if p.z < 1e-9 and p.box.id not in ("Wheel_Front", "Wheel_Rear"))
print(f"기존 확정 배치: {len(filled_plans)}/{len(filled_boxes)}개 성공, 미적재: {filled_unloadable}")
print(f"바닥 사용률: {used_floor_area / (TRUNK_WIDTH * TRUNK_DEPTH):.1%}")

# ---- 2단계: 카트의 초록(1층) - 이미 확정된 상태에 자리가 남아있는지 시험 ----
new_plans, new_unloadable = [], []

plan = place_one_box(cart_green_box, trunk, state, order=order_counter, allow_stacking=False)
order_counter += 1
if plan is None:
    reason = classify_unloadable_reason(cart_green_box, trunk, state)
    new_unloadable.append((cart_green_box.id, reason.value))
    print(f"{cart_green_box.id}: 배치 불가 - 사유: {reason.value}")
else:
    new_plans.append(plan)
    x, y, z = plan.position
    print(f"{cart_green_box.id}(1층): x[{x:.3f},{x+cart_green_box.width:.3f}] "
          f"y[{y:.3f},{y+cart_green_box.depth:.3f}] z[{z:.3f},{z+cart_green_box.height:.3f}]  "
          f"점수={plan.score:.4f} 접촉면={plan.touches}/6")

# ---- 3단계: 카트의 파랑(2층) - 초록 위/장애물 위 등 쌓을 자리가 있는지 시험 ----
for box in decide_loading_order(cart_blue_boxes):
    plan = place_one_box_stacked_only(box, trunk, state, order=order_counter)
    order_counter += 1
    if plan is None:
        new_unloadable.append((box.id, "NO_VALID_STACK_POSITION"))
        print(f"{box.id}: 배치 불가 (쌓을 자리 없음)")
    else:
        new_plans.append(plan)
        x, y, z = plan.position
        layer = "1층(바닥)" if z < 1e-9 else "2층(쌓임)"
        print(f"{box.id}[{layer}]: x[{x:.3f},{x+box.width:.3f}] y[{y:.3f},{y+box.depth:.3f}] "
              f"z[{z:.3f},{z+box.height:.3f}]  점수={plan.score:.4f} 접촉면={plan.touches}/6")

all_plans = filled_plans + new_plans
all_boxes = filled_boxes + [cart_green_box] + cart_blue_boxes

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


NEW_IDS = {cart_green_box.id} | {b.id for b in cart_blue_boxes}
NEW_COLORS = {cart_green_box.id: "#ffb300", cart_blue_boxes[0].id: "#e53935", cart_blue_boxes[1].id: "#8e24aa"}


def color_for(box_id):
    return NEW_COLORS[box_id] if box_id in NEW_IDS else "#66bb6a"  # 기존 확정 5개는 연두색 고정


G = ROBOT_TO_TRUNK_GAP

fig = plt.figure(figsize=(13, 7))

# ---- 왼쪽: 3D 아이소메트릭 ----
ax3d = fig.add_subplot(1, 2, 1, projection="3d")
ax3d.scatter([0], [ROBOT_Y], [0], color="crimson", s=80, label="로봇 (base 원점)")
ax3d.quiver(0, ROBOT_Y, 0, G, 0, 0, color="crimson", linewidth=2, arrow_length_ratio=0.15)
draw_wireframe_box(ax3d, G, 0, 0, TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT, color="red", label="trunk")
for i, obs in enumerate(obstacles):
    draw_cuboid(ax3d, G + obs.x, obs.y, obs.z, obs.box.width, obs.box.depth, obs.box.height,
                facecolor="black", edgecolor="black", alpha=0.6)
ax3d.plot([], [], color="black", linewidth=2, label="차 바퀴(휠하우스)")
ax3d.plot([], [], color="#66bb6a", linewidth=2, label="기존 확정 5개(재배치 불가)")
for plan in all_plans:
    x, y, z = plan.position
    box = next(b for b in all_boxes if b.id == plan.box_id)
    color = color_for(plan.box_id)
    draw_cuboid(ax3d, G + x, y, z, box.width, box.depth, box.height, facecolor=color, edgecolor=color)
for box_id, reason in new_unloadable:
    ax3d.plot([], [], color="none", label=f"[미적재] {box_id}: {reason}")

ax3d.set_xlim(0, G + TRUNK_WIDTH)
ax3d.set_ylim(0, TRUNK_DEPTH)
ax3d.set_zlim(0, TRUNK_HEIGHT)
ax3d.set_xlabel("x (m) - 로봇 원점 기준, 로봇이 접근하는 방향")
ax3d.set_ylabel("y (m) - 좌우(입구와 무관)")
ax3d.set_zlabel("z (m, height)")
ax3d.set_title("3D 아이소메트릭 - 거의 꽉 찬 트렁크 + 카트 신규 적재 시험")
ax3d.legend(loc="upper left", fontsize=6)
ax3d.set_box_aspect((G + TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT))

# ---- 오른쪽: 2D 탑다운 ----
ax2d = fig.add_subplot(1, 2, 2)
ax2d.scatter([0], [ROBOT_Y], color="crimson", s=100, zorder=5, label="로봇 (base 원점)")
ax2d.annotate("", xy=(G, ROBOT_Y), xytext=(0, ROBOT_Y),
              arrowprops=dict(arrowstyle="->", color="crimson", linewidth=2))
ax2d.text(G / 2, ROBOT_Y + 0.02, "접근 방향(+x, 고정)", color="crimson", fontsize=8, ha="center")

ax2d.add_patch(Rectangle((G, 0), TRUNK_WIDTH, TRUNK_DEPTH, fill=False, edgecolor="red", linewidth=2, label="trunk"))
for i, obs in enumerate(obstacles):
    ax2d.add_patch(Rectangle((G + obs.x, obs.y), obs.box.width, obs.box.depth, facecolor="black",
                              edgecolor="black", alpha=0.6, label="차 바퀴(휠하우스)" if i == 0 else None))

for plan in all_plans:
    x, y, z = plan.position
    box = next(b for b in all_boxes if b.id == plan.box_id)
    color = color_for(plan.box_id)
    is_stacked = z > 1e-9
    is_new = plan.box_id in NEW_IDS
    ax2d.add_patch(Rectangle((G + x, y), box.width, box.depth, facecolor=color,
                              edgecolor=("navy" if is_stacked else ("black" if is_new else color)),
                              alpha=0.85, linewidth=(2.2 if is_new else 1.0),
                              linestyle=("--" if is_stacked else "-")))
    ax2d.text(G + x + box.width / 2, y + box.depth / 2, plan.box_id,
              fontsize=6, ha="center", va="center", color="white" if is_new else "black", weight="bold")

ax2d.set_xlim(-0.05, G + TRUNK_WIDTH + 0.05)
ax2d.set_ylim(-0.05, TRUNK_DEPTH + 0.05)
ax2d.set_xlabel("x (m) - 로봇 원점 기준, 로봇이 접근하는 방향")
ax2d.set_ylabel("y (m) - 좌우(입구와 무관)")
unloadable_note = (", ".join(f"{bid}:{r}" for bid, r in new_unloadable)) if new_unloadable else "없음"
ax2d.set_title(f"top-down - 초록=기존 확정 / 굵은 테두리=카트 신규 / 미적재: {unloadable_note}")
ax2d.legend(loc="upper left", fontsize=7)
ax2d.set_aspect("equal")

plt.tight_layout()
out_path = str(ALGORISM_DIR / "local_test_data" / "sketch_placement_near_full_result.png")
plt.savefig(out_path, dpi=130)
print("\n그래프 저장:", out_path)

print("\n=== 요약 ===")
print(f"기존 확정 배치: {len(filled_plans)}/{len(filled_boxes)}개")
print(f"카트 신규 배치: {len(new_plans)}/{1 + len(cart_blue_boxes)}개 성공")
print(f"카트 신규 미적재: {new_unloadable if new_unloadable else '없음'}")
