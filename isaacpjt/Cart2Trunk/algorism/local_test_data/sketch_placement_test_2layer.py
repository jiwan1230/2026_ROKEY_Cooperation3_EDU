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
    """
    place_one_box()와 같은 로직이지만 z=0(바닥) 후보는 아예 제외하고 z>0(다른 박스
    위에 쌓기)만 본다. 파이프라인(⑦) 자체는 안 건드리고 이 스크립트에서만 쓰는
    버전 - "파란 박스는 무조건 2층에 놓는다"는 이번 데모 시나리오의 지시를 그대로
    반영하기 위함이다 (원래 place_one_box는 바닥에 자리가 남아있으면 높이 우선
    원칙 때문에 바닥을 먼저 고르는데, 지금은 그 선택지를 아예 없애고 싶은 것).
    나머지(③ 벽/박스 밀착 후보 보강, ⑮ 상단 여유 공간, ⑯ 접근 경로 확인)는 실제
    07_placement_plan.py와 동일하게 맞춰서, 이 데모가 지금 진짜 알고리즘과
    어긋나지 않도록 함.
    """
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
# 이전 손그림(sketch_placement_test.py)과 같은 트렁크/차 바퀴 - 이번엔 초록 박스
# 5개(1층)에 파란 박스 2개(2층, 초록 위에 쌓기)가 추가된 새 손그림을 그대로 반영.
# 크기/위치는 그림 픽셀 비율을 이전과 같은 스케일(트렁크 폭 0.6m 기준)로 추정.
# ---------------------------------------------------------------------------

TRUNK_WIDTH = 0.60   # x축
TRUNK_DEPTH = 0.73   # y축
TRUNK_HEIGHT = 0.50  # ⑮ 상단 여유 공간(0.2m) 도입 후, 실제 스캔 데이터(~0.52m)에 맞춰 상향

ROBOT_TO_TRUNK_GAP = 0.30
ROBOT_Y = TRUNK_DEPTH / 2

trunk = Trunk(width=TRUNK_WIDTH, depth=TRUNK_DEPTH, height=TRUNK_HEIGHT)

wheel_front = PlacedBox(box=Box("Wheel_Front", width=0.16, depth=0.15, height=0.20),
                         x=0.44, y=0.00, z=0.0)
wheel_rear = PlacedBox(box=Box("Wheel_Rear", width=0.16, depth=0.21, height=0.20),
                        x=0.44, y=0.52, z=0.0)
obstacles = [wheel_front, wheel_rear]

# 1층(바닥) - 초록 박스 5개
green_boxes = [
    Box("G_Tall", width=0.08, depth=0.17, height=0.15),
    Box("G_SmallTop", width=0.07, depth=0.05, height=0.15),
    Box("G_Mid", width=0.14, depth=0.16, height=0.15),
    Box("G_SmallRight", width=0.07, depth=0.06, height=0.15),
    Box("G_Wide", width=0.30, depth=0.07, height=0.15),
]

# 2층(초록 위에 쌓기) - 파란 박스 2개, 1층보다 낮은 높이로 추정
blue_boxes = [
    Box("B_Mid", width=0.10, depth=0.09, height=0.12),
    Box("B_Small", width=0.07, depth=0.07, height=0.12),
]

print("트렁크:", trunk)
print("장애물(차 바퀴) 2개:", [(o.box.id, o.x, o.y, o.box.width, o.box.depth) for o in obstacles])

state = ExtremePointState()
for obs in obstacles:
    state.register_placement(obs)

# ---- 1단계: 초록 박스(1층) 배치 - allow_stacking=False로 바닥에만 ----
green_order = decide_loading_order(green_boxes)
print("\n1층(초록) 적재 순서(부피 큰 순):", [b.id for b in green_order])

green_plans, green_unloadable = [], []
order_counter = 1
for box in green_order:
    plan = place_one_box(box, trunk, state, order=order_counter, allow_stacking=False)
    order_counter += 1
    if plan is None:
        green_unloadable.append(box.id)
        print(f"{box.id}: 배치 불가 (자리 없음)")
    else:
        green_plans.append(plan)
        x, y, z = plan.position
        print(f"{box.id}: 배치 -> x[{x:.3f},{x+box.width:.3f}] y[{y:.3f},{y+box.depth:.3f}] "
              f"z[{z:.3f},{z+box.height:.3f}]  점수={plan.score:.4f} 접촉면={plan.touches}/6")

# ---- 2단계: 파란 박스(2층) 배치 - allow_stacking=True로 초록 위에도 놓을 수 있게 ----
blue_order = decide_loading_order(blue_boxes)
print("\n2층(파랑) 적재 순서(부피 큰 순):", [b.id for b in blue_order])

blue_plans, blue_unloadable = [], []
for box in blue_order:
    plan = place_one_box_stacked_only(box, trunk, state, order=order_counter)
    order_counter += 1
    if plan is None:
        blue_unloadable.append(box.id)
        print(f"{box.id}: 배치 불가 (자리 없음)")
    else:
        blue_plans.append(plan)
        x, y, z = plan.position
        layer = "1층(바닥)" if z < 1e-9 else "2층(쌓임)"
        print(f"{box.id}: 배치 -> x[{x:.3f},{x+box.width:.3f}] y[{y:.3f},{y+box.depth:.3f}] "
              f"z[{z:.3f},{z+box.height:.3f}]  [{layer}]  점수={plan.score:.4f} 접촉면={plan.touches}/6")

all_plans = green_plans + blue_plans

# ---- 시각화 (이전 스크립트와 같은 3D+2D 방식 재사용) ----
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


def draw_cuboid(ax, x0, y0, z0, dx, dy, dz, facecolor, edgecolor, alpha=0.6, label=None):
    coll = Poly3DCollection(cuboid_faces(x0, y0, z0, dx, dy, dz), facecolor=facecolor,
                             edgecolor=edgecolor, alpha=alpha, linewidths=1.0)
    ax.add_collection3d(coll)
    if label:
        ax.plot([], [], color=edgecolor, label=label)


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


GREEN_SHADES = ["#2e7d32", "#43a047", "#66bb6a", "#81c784", "#a5d6a7"]
BLUE_SHADES = ["#1565c0", "#42a5f5"]
GREEN_COLORS = {b.id: GREEN_SHADES[i % len(GREEN_SHADES)] for i, b in enumerate(green_boxes)}
BLUE_COLORS = {b.id: BLUE_SHADES[i % len(BLUE_SHADES)] for i, b in enumerate(blue_boxes)}
ALL_COLORS = {**GREEN_COLORS, **BLUE_COLORS}

G = ROBOT_TO_TRUNK_GAP

fig = plt.figure(figsize=(13, 7))

# ---- 왼쪽: 3D 아이소메트릭 ----
ax3d = fig.add_subplot(1, 2, 1, projection="3d")
ax3d.scatter([0], [ROBOT_Y], [0], color="crimson", s=80, label="로봇 (base 원점)")
ax3d.quiver(0, ROBOT_Y, 0, G, 0, 0, color="crimson", linewidth=2, arrow_length_ratio=0.15)
draw_wireframe_box(ax3d, G, 0, 0, TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT, color="red", label="trunk (그림 추정)")
for i, obs in enumerate(obstacles):
    draw_cuboid(ax3d, G + obs.x, obs.y, obs.z, obs.box.width, obs.box.depth, obs.box.height,
                facecolor="black", edgecolor="black", alpha=0.6,
                label="차 바퀴(휠하우스)" if i == 0 else None)
for plan in all_plans:
    x, y, z = plan.position
    box = next(b for b in green_boxes + blue_boxes if b.id == plan.box_id)
    color = ALL_COLORS[plan.box_id]
    draw_cuboid(ax3d, G + x, y, z, box.width, box.depth, box.height,
                facecolor=color, edgecolor=color, alpha=0.85)

ax3d.set_xlim(0, G + TRUNK_WIDTH)
ax3d.set_ylim(0, TRUNK_DEPTH)
ax3d.set_zlim(0, TRUNK_HEIGHT)
ax3d.set_xlabel("x (m) - 로봇 원점 기준, 로봇이 접근하는 방향")
ax3d.set_ylabel("y (m) - 좌우(입구와 무관)")
ax3d.set_zlabel("z (m, height)")
ax3d.set_title("3D 아이소메트릭 - 1층(초록)+2층(파랑) 적재")
ax3d.legend(loc="upper left", fontsize=7)
ax3d.set_box_aspect((G + TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT))

# ---- 오른쪽: 2D 탑다운 ----
ax2d = fig.add_subplot(1, 2, 2)
ax2d.scatter([0], [ROBOT_Y], color="crimson", s=100, zorder=5, label="로봇 (base 원점)")
ax2d.annotate("", xy=(G, ROBOT_Y), xytext=(0, ROBOT_Y),
              arrowprops=dict(arrowstyle="->", color="crimson", linewidth=2))
ax2d.text(G / 2, ROBOT_Y + 0.02, "접근 방향(+x, 고정)", color="crimson", fontsize=8, ha="center")

ax2d.add_patch(Rectangle((G, 0), TRUNK_WIDTH, TRUNK_DEPTH, fill=False, edgecolor="red",
                          linewidth=2, label="trunk (그림 추정)"))
for i, obs in enumerate(obstacles):
    ax2d.add_patch(Rectangle((G + obs.x, obs.y), obs.box.width, obs.box.depth, facecolor="black",
                              edgecolor="black", alpha=0.6, label="차 바퀴(휠하우스)" if i == 0 else None))

# 1층 박스는 실선, 2층(z>0)에 쌓인 박스는 굵은 파란 테두리 점선으로 구분해서 top-down에서도 층을 알아볼 수 있게 함
for plan in all_plans:
    x, y, z = plan.position
    box = next(b for b in green_boxes + blue_boxes if b.id == plan.box_id)
    color = ALL_COLORS[plan.box_id]
    is_stacked = z > 1e-9
    ax2d.add_patch(Rectangle((G + x, y), box.width, box.depth, facecolor=color,
                              edgecolor=("navy" if is_stacked else color),
                              alpha=(0.9 if is_stacked else 0.7),
                              linewidth=(2.5 if is_stacked else 1.5),
                              linestyle=("--" if is_stacked else "-")))
    ax2d.text(G + x + box.width / 2, y + box.depth / 2, plan.box_id,
              fontsize=6, ha="center", va="center", color="white" if is_stacked else "black")

ax2d.set_xlim(-0.05, G + TRUNK_WIDTH + 0.05)
ax2d.set_ylim(-0.05, TRUNK_DEPTH + 0.05)
ax2d.set_xlabel("x (m) - 로봇 원점 기준, 로봇이 접근하는 방향")
ax2d.set_ylabel("y (m) - 좌우(입구와 무관)")
ax2d.set_title("top-down - 점선 굵은 테두리 = 2층(쌓인 박스)")
ax2d.legend(loc="upper left", fontsize=7)
ax2d.set_aspect("equal")

plt.tight_layout()
out_path = str(ALGORISM_DIR / "local_test_data" / "sketch_placement_2layer_result.png")
plt.savefig(out_path, dpi=130)
print("\n그래프 저장:", out_path)

print("\n=== 요약 ===")
print(f"1층(초록) 배치: {len(green_plans)}/{len(green_boxes)}개 성공, 미적재: {green_unloadable}")
print(f"2층(파랑) 배치: {len(blue_plans)}/{len(blue_boxes)}개 성공, 미적재: {blue_unloadable}")
for plan in blue_plans:
    layer = "1층(바닥)" if plan.position[2] < 1e-9 else "2층(쌓임)"
    print(f"  {plan.box_id}: {layer}")
