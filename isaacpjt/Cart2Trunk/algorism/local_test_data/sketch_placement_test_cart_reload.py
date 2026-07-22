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
# 정정: 지난번 트렁크 상태를 "확정된 것"으로 두고 그 위에 추가하는 게 아니라,
# 지난번에 쓴 박스 7개(1층 초록 5개 + 2층 파랑 2개)와 이번에 카트에서 새로 온
# 박스 3개(1층 초록 1개 + 2층 파랑 2개)를 전부 한 배치로 합쳐서, 빈 트렁크부터
# 다시 최적 배치를 계산한다 - "새롭게 조합해서 적재".
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

# 1층(바닥) 후보 전부 - 지난번 초록 5개 + 카트에서 새로 온 초록 1개
green_boxes = [
    Box("G_Tall", width=0.08, depth=0.17, height=0.15),
    Box("G_SmallTop", width=0.07, depth=0.05, height=0.15),
    Box("G_Mid", width=0.14, depth=0.16, height=0.15),
    Box("G_SmallRight", width=0.07, depth=0.06, height=0.15),
    Box("G_Wide", width=0.30, depth=0.07, height=0.15),
    Box("Cart_Green", width=0.20, depth=0.18, height=0.15),
]

# 2층(쌓기) 후보 전부 - 지난번 파랑 2개 + 카트에서 새로 온 파랑 2개
blue_boxes = [
    Box("B_Mid", width=0.10, depth=0.09, height=0.12),
    Box("B_Small", width=0.07, depth=0.07, height=0.12),
    Box("Cart_Blue1", width=0.09, depth=0.10, height=0.12),
    Box("Cart_Blue2", width=0.12, depth=0.14, height=0.12),
]

print("트렁크:", trunk)
print(f"1층 후보 {len(green_boxes)}개(초록), 2층 후보 {len(blue_boxes)}개(파랑) - 전부 합쳐서 빈 트렁크부터 재계산")

state = ExtremePointState()
for obs in obstacles:
    state.register_placement(obs)

# ---- 1단계: 초록 전부(1층) - 부피 큰 순으로 한 배치에서 같이 결정 ----
green_order = decide_loading_order(green_boxes)
print("\n1층(초록) 적재 순서(부피 큰 순):", [b.id for b in green_order])

green_plans, unloadable = [], []
order_counter = 1
for box in green_order:
    plan = place_one_box(box, trunk, state, order=order_counter, allow_stacking=False)
    order_counter += 1
    if plan is None:
        unloadable.append(box.id)
        print(f"{box.id}: 배치 불가 (자리 없음)")
    else:
        green_plans.append(plan)
        x, y, z = plan.position
        print(f"{box.id}: x[{x:.3f},{x+box.width:.3f}] y[{y:.3f},{y+box.depth:.3f}] "
              f"z[{z:.3f},{z+box.height:.3f}]  점수={plan.score:.4f} 접촉면={plan.touches}/6")

# ---- 2단계: 파랑 전부(2층) - 마찬가지로 한 배치에서 같이 결정 ----
blue_order = decide_loading_order(blue_boxes)
print("\n2층(파랑) 적재 순서(부피 큰 순):", [b.id for b in blue_order])

blue_plans = []
for box in blue_order:
    plan = place_one_box_stacked_only(box, trunk, state, order=order_counter)
    order_counter += 1
    if plan is None:
        unloadable.append(box.id)
        print(f"{box.id}: 배치 불가 (자리 없음)")
    else:
        blue_plans.append(plan)
        x, y, z = plan.position
        print(f"{box.id}: x[{x:.3f},{x+box.width:.3f}] y[{y:.3f},{y+box.depth:.3f}] "
              f"z[{z:.3f},{z+box.height:.3f}]  점수={plan.score:.4f} 접촉면={plan.touches}/6")

all_plans = green_plans + blue_plans
all_boxes = green_boxes + blue_boxes

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


GREEN_SHADES = ["#1b5e20", "#2e7d32", "#43a047", "#66bb6a", "#81c784", "#a5d6a7"]
BLUE_SHADES = ["#0d47a1", "#1565c0", "#1e88e5", "#64b5f6"]
COLORS = {b.id: GREEN_SHADES[i % len(GREEN_SHADES)] for i, b in enumerate(green_boxes)}
COLORS.update({b.id: BLUE_SHADES[i % len(BLUE_SHADES)] for i, b in enumerate(blue_boxes)})

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
for plan in all_plans:
    x, y, z = plan.position
    box = next(b for b in all_boxes if b.id == plan.box_id)
    color = COLORS[plan.box_id]
    draw_cuboid(ax3d, G + x, y, z, box.width, box.depth, box.height, facecolor=color, edgecolor=color)

ax3d.set_xlim(0, G + TRUNK_WIDTH)
ax3d.set_ylim(0, TRUNK_DEPTH)
ax3d.set_zlim(0, TRUNK_HEIGHT)
ax3d.set_xlabel("x (m) - 로봇 원점 기준, 로봇이 접근하는 방향")
ax3d.set_ylabel("y (m) - 좌우(입구와 무관)")
ax3d.set_zlabel("z (m, height)")
ax3d.set_title("3D 아이소메트릭 - 카트+트렁크 통합 재적재 (10개 전부 새로 계산)")
ax3d.legend(loc="upper left", fontsize=7)
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
    color = COLORS[plan.box_id]
    is_stacked = z > 1e-9
    ax2d.add_patch(Rectangle((G + x, y), box.width, box.depth, facecolor=color,
                              edgecolor=("navy" if is_stacked else color),
                              alpha=0.85, linewidth=(2.2 if is_stacked else 1.3),
                              linestyle=("--" if is_stacked else "-")))
    ax2d.text(G + x + box.width / 2, y + box.depth / 2, plan.box_id,
              fontsize=6, ha="center", va="center", color="white", weight="bold")

ax2d.set_xlim(-0.05, G + TRUNK_WIDTH + 0.05)
ax2d.set_ylim(-0.05, TRUNK_DEPTH + 0.05)
ax2d.set_xlabel("x (m) - 로봇 원점 기준, 로봇이 접근하는 방향")
ax2d.set_ylabel("y (m) - 좌우(입구와 무관)")
ax2d.set_title("top-down - 점선 굵은 테두리 = 2층(쌓인 박스)")
ax2d.legend(loc="upper left", fontsize=7)
ax2d.set_aspect("equal")

plt.tight_layout()
out_path = str(ALGORISM_DIR / "local_test_data" / "sketch_placement_cart_reload_result.png")
plt.savefig(out_path, dpi=130)
print("\n그래프 저장:", out_path)

print("\n=== 요약 ===")
print(f"1층(초록) 배치: {len(green_plans)}/{len(green_boxes)}개 성공")
print(f"2층(파랑) 배치: {len(blue_plans)}/{len(blue_boxes)}개 성공")
print(f"미적재: {unloadable if unloadable else '없음'}")
