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
m18 = import_module("18_rotation")

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
    """
    z=0(바닥) 후보는 빼고 z>0(쌓기)만 본다 - '2층에 놓는다'는 지정을 그대로 반영.
    나머지(③ 벽/박스 밀착 후보 보강, ⑮ 상단 여유 공간, ⑯ 접근 경로 확인)는 실제
    07_placement_plan.py와 동일하게 맞춰서, 이 데모가 지금 진짜 알고리즘과
    어긋나지 않도록 함.
    """
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
                          dimensions=(box.width, box.depth, box.height),
                          score=best_score, touches=best_touches, rotated=rotated)


# ---------------------------------------------------------------------------
# 정정(2차): "기존 7개도 전부 카트로 다시 빼서 10개를 빈 트렁크부터 새로 계산"하는
# 방식은, 로봇이 안 움직여도 되는 박스까지 전부 빼고 다시 넣는 걸 전제로 해서
# 비현실적인 작업량을 만든다는 지적을 받아 수정함.
#
# 실제로는: 지난번에 실은 7개(1층 초록 5 + 2층 파랑 2)는 "이미 트렁크 안, 그
# 위치 그대로 고정"으로 두고 - 지금 로봇으로 기존 적재물을 밀거나 빼는 건 아직
# 어려우니 건드리지 않는다 - 카트에서 새로 온 3개(Cart_Green/Blue1/Blue2)만
# **카트 피킹 순서 그대로(⑥, rests_on_id 반영)** 남는 공간에 최선을 다해
# 추가 배치한다. 즉 "집는 순서 == 놓는 순서"(집자마자 바로 놓는 1패스 동작)이고,
# 기존 7개는 아예 다시 계산하지 않는다.
# ---------------------------------------------------------------------------

TRUNK_WIDTH = 0.60
TRUNK_DEPTH = 0.73
TRUNK_HEIGHT = 0.50  # ⑮ 상단 여유 공간(0.2m) 도입 후, 실제 스캔 데이터(~0.52m)에 맞춰 상향

ROBOT_TO_TRUNK_GAP = 0.30
ROBOT_Y = TRUNK_DEPTH / 2

trunk = Trunk(width=TRUNK_WIDTH, depth=TRUNK_DEPTH, height=TRUNK_HEIGHT)

wheel_front = PlacedBox(box=Box("Wheel_Front", width=0.16, depth=0.15, height=0.20),
                         x=0.44, y=0.00, z=0.0)
wheel_rear = PlacedBox(box=Box("Wheel_Rear", width=0.16, depth=0.21, height=0.20),
                        x=0.44, y=0.52, z=0.0)
obstacles = [wheel_front, wheel_rear]

# 지난번에 이미 트렁크에 실어놓은 7개 - 손그림 그대로 "이미 고정된 상태"다.
# 이번 재적재 세션에서 우리 알고리즘이 얘네 자리를 "결정"하는 게 아니라, 이미
# 정해진 위치를 그대로 사실로 받아들인다 - 그래서 decide_loading_order/
# place_one_box를 아예 안 거치고, 차 바퀴(Wheel_Front/Rear)와 똑같이 좌표를
# 고정값으로 바로 등록한다.
#
# 좌표는 알고리즘이 계산한 값이 아니라, 사용자가 준 손그림(트렁크 top-down)의
# 배치를 그대로 참고해서 다시 잡음: 입구 쪽(작은 x)에 키 큰 박스(G_Tall)+작은
# 사각형(G_SmallTop/G_SmallRight)이 위쪽에 모여있고, 그 아래 넓은 박스(G_Wide)가
# 가로로 걸쳐 있고, 바퀴 사이 틈(차 바퀴 두 개 사이, x가 깊은 쪽)에 큰 박스
# (G_Mid)가 끼워져 있는 구도. 손그림이 정밀 치수가 아니라 픽셀 단위로 정확히
# 역산하는 건 신뢰할 수 없어서, 이 구도(어느 박스가 어디 근처에 있는지)만
# 최대한 맞추고 실제 미터 값은 서로 안 겹치게 손으로 잡았다.
old_green_boxes = [
    Box("G_Tall", width=0.08, depth=0.17, height=0.15),
    Box("G_SmallTop", width=0.07, depth=0.05, height=0.15),
    Box("G_Mid", width=0.14, depth=0.16, height=0.15),
    Box("G_SmallRight", width=0.07, depth=0.06, height=0.15),
    Box("G_Wide", width=0.30, depth=0.07, height=0.15),
]
old_blue_boxes = [
    Box("B_Mid", width=0.10, depth=0.09, height=0.12),
    Box("B_Small", width=0.07, depth=0.07, height=0.12),
]
_DEFAULT_OLD_FIXED_POSITIONS = {
    # 입구 쪽(x 작음) 상단 클러스터: 키 큰 박스 + 작은 사각형 둘
    "G_Tall": (0.03, 0.03, 0.000),
    "G_SmallTop": (0.14, 0.05, 0.000),
    "G_SmallRight": (0.14, 0.12, 0.000),
    # 상단 클러스터 아래, 가로로 넓게 걸쳐진 박스
    "G_Wide": (0.03, 0.23, 0.000),
    # 차 바퀴 두 개 사이 틈(깊은 쪽, x 큼)에 끼워진 큰 박스
    "G_Mid": (0.44, 0.26, 0.000),
    # 2층(쌓기) - G_Mid 위에 나란히
    "B_Mid": (0.44, 0.26, 0.150),
    "B_Small": (0.44, 0.35, 0.150),
}

# interactive_cart_reload_editor.py에서 "저장" 버튼으로 내보낸 좌표가 있으면
# 그걸 우선 쓴다 - 사용자가 직접 슬라이더로 맞춘 값을 그대로 반영하기 위함.
# 없으면(아직 저장 안 했으면) 위 기본값을 그대로 쓴다.
# 편집기에서 이름/치수/색상까지 넣어 직접 추가한 박스가 저장 파일에 같이 들어있어도,
# 여기서는 old_green_boxes/old_blue_boxes에 정의된 7개의 자리(x,y,z)만 골라 쓴다 -
# 이 파이프라인 스크립트는 그 7개 Box 정의(치수 고정)만 다루도록 만들어져 있어서,
# 편집기에서 즉석으로 만든 새 박스는 시각화 실험용으로만 남고 여기엔 반영되지 않는다.
_SAVED_POSITIONS_PATH = ALGORISM_DIR / "local_test_data" / "cart_reload_fixed_positions.json"
_KNOWN_OLD_IDS = {b.id for b in old_green_boxes + old_blue_boxes}
if _SAVED_POSITIONS_PATH.exists():
    import json as _json
    _saved = _json.loads(_SAVED_POSITIONS_PATH.read_text())
    OLD_FIXED_POSITIONS = {
        k: (v["x"], v["y"], v["z"]) for k, v in _saved.items() if k in _KNOWN_OLD_IDS
    }
    missing = _KNOWN_OLD_IDS - OLD_FIXED_POSITIONS.keys()
    if missing:
        for box_id in missing:
            OLD_FIXED_POSITIONS[box_id] = _DEFAULT_OLD_FIXED_POSITIONS[box_id]
    print(f"[알림] {_SAVED_POSITIONS_PATH.name}에서 저장된 좌표를 불러왔습니다 (인터랙티브 편집기에서 저장한 값).")
else:
    OLD_FIXED_POSITIONS = _DEFAULT_OLD_FIXED_POSITIONS

# 카트에서 새로 온 3개 - 손그림 그대로 Cart_Blue1/Cart_Blue2가 카트 안에서
# Cart_Green 위에 얹혀 있다(rests_on_id) - ⑥ 픽업 순서 제약이 실제로 걸리는 대상.
new_cart_boxes = [
    Box("Cart_Green", width=0.20, depth=0.18, height=0.15),
    Box("Cart_Blue1", width=0.09, depth=0.10, height=0.12, rests_on_id="Cart_Green"),
    Box("Cart_Blue2", width=0.12, depth=0.14, height=0.12, rests_on_id="Cart_Green"),
]

print("트렁크:", trunk)
print(f"기존 적재(고정) {len(old_green_boxes) + len(old_blue_boxes)}개, "
      f"카트에서 새로 온 박스 {len(new_cart_boxes)}개")

state = ExtremePointState()
for obs in obstacles:
    state.register_placement(obs)

# ---------------------------------------------------------------------------
# [1단계] 기존 7개를 "주어진 사실"로 그대로 등록 - 우리 알고리즘이 이 자리를
# 정하는 게 아니라, 차 바퀴처럼 이미 고정된 장애물/적재물로 취급한다. 그래서
# decide_loading_order나 place_one_box를 아예 호출하지 않고, 고정 좌표를
# 바로 state에 등록만 한다 (order 번호도 부여 안 함 - 이번 세션의 로봇 동작이
# 아니므로).
# ---------------------------------------------------------------------------
print("\n[1단계] 기존 적재 상태 등록 (지난번에 이미 실어놓은 7개, 고정값 그대로 - 알고리즘 미적용)")

old_plans, unloadable = [], []
order_counter = 1
for box in old_green_boxes + old_blue_boxes:
    x, y, z = OLD_FIXED_POSITIONS[box.id]
    state.register_placement(PlacedBox(box=box, x=x, y=y, z=z))
    old_plans.append(PlacementPlan(box_id=box.id, order=None, position=(x, y, z),
                                    dimensions=(box.width, box.depth, box.height),
                                    score=None, touches=None))
    print(f"  {box.id}: x[{x:.3f},{x+box.width:.3f}] y[{y:.3f},{y+box.depth:.3f}] "
          f"z[{z:.3f},{z+box.height:.3f}]  (기존, 고정 - 알고리즘 미적용)")

# ---------------------------------------------------------------------------
# [2단계] 카트에서 실제로 집는 순서 (⑥ 픽업 순서 제약, rests_on_id 반영).
# Cart_Blue1/Blue2가 Cart_Green 위에 얹혀 있으므로 Green을 집으려면 그 전에
# 위에 있는 둘을 먼저 치워야 한다.
# ---------------------------------------------------------------------------
cart_pick_order = decide_loading_order(new_cart_boxes)
print("\n[2단계] 카트에서 집는 순서 (rests_on_id 반영):")
for i, b in enumerate(cart_pick_order, start=1):
    rests = f" (rests_on={b.rests_on_id})" if b.rests_on_id else " (바닥)"
    print(f"  {i}. {b.id}: volume={b.volume*1000:.2f}L{rests}")
old_style_order = sorted(new_cart_boxes, key=lambda b: b.volume, reverse=True)
print("  참고 - 수정 전(순수 부피순)이었다면:", [b.id for b in old_style_order],
      "← Cart_Green이 맨 아래인데도 1번으로 나와 물리적으로 불가능했음")

# ---------------------------------------------------------------------------
# [3단계] 카트에서 집은 순서 그대로, 기존 7개를 안 건드리고 남는 공간에 즉시
# 배치한다("집자마자 바로 놓는다" - 별도 스테이징 없음, 기존 배치 재계산 없음).
# Cart_Green도 floor 전용이 아니라 이미 채워진 기존 바닥 위 2층 자리를 써도
# 되므로, 목적지(1층/2층)는 place_one_box가 자유롭게 고르게 둔다(allow_stacking=True).
# 단, Cart_Blue1/2는 손그림상 항상 2층이라는 지정을 유지하기 위해 stacked-only로 둔다.
# ---------------------------------------------------------------------------
print("\n[3단계] 카트 피킹 순서 그대로 즉시 배치 (기존 7개는 그대로 둔 채 남는 공간에서 최선)")
new_plans = []
for box in cart_pick_order:
    if box.rests_on_id is not None:
        plan = place_one_box_stacked_only(box, trunk, state, order=order_counter)
    else:
        plan = place_one_box(box, trunk, state, order=order_counter, allow_stacking=True)
    order_counter += 1
    if plan is None:
        unloadable.append(box.id)
        print(f"  {box.id}: 배치 불가 (기존 7개를 안 건드리는 한 남는 공간 없음)")
    else:
        new_plans.append(plan)
        x, y, z = plan.position
        layer = "2층" if z > 1e-9 else "1층"
        print(f"  {box.id}: x[{x:.3f},{x+box.width:.3f}] y[{y:.3f},{y+box.depth:.3f}] "
              f"z[{z:.3f},{z+box.height:.3f}]  [{layer}]  점수={plan.score:.4f} 접촉면={plan.touches}/6")

all_plans = old_plans + new_plans
all_boxes = old_green_boxes + old_blue_boxes + new_cart_boxes
green_boxes = old_green_boxes  # 시각화 색상 배정용 (아래 코드가 참조)
blue_boxes = old_blue_boxes + new_cart_boxes

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
ax3d.set_title("3D 아이소메트릭 - 기존 7개 고정 + 카트 신규 3개만 추가 배치")
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

new_ids = {b.id for b in new_cart_boxes}
for plan in all_plans:
    x, y, z = plan.position
    box = next(b for b in all_boxes if b.id == plan.box_id)
    color = COLORS[plan.box_id]
    is_stacked = z > 1e-9
    is_new = plan.box_id in new_ids
    ax2d.add_patch(Rectangle((G + x, y), box.width, box.depth, facecolor=color,
                              edgecolor=("navy" if is_stacked else color),
                              alpha=0.85, linewidth=(2.8 if is_new else 1.3),
                              linestyle=("--" if is_stacked else ("-." if is_new else "-"))))
    ax2d.text(G + x + box.width / 2, y + box.depth / 2, plan.box_id,
              fontsize=6, ha="center", va="center", color="white", weight="bold")

ax2d.set_xlim(-0.05, G + TRUNK_WIDTH + 0.05)
ax2d.set_ylim(-0.05, TRUNK_DEPTH + 0.05)
ax2d.set_xlabel("x (m) - 로봇 원점 기준, 로봇이 접근하는 방향")
ax2d.set_ylabel("y (m) - 좌우(입구와 무관)")
ax2d.set_title("top-down - 점선=2층, 굵은 테두리=이번에 새로 넣은 카트 박스 3개")
ax2d.legend(loc="upper left", fontsize=7)
ax2d.set_aspect("equal")

plt.tight_layout()
out_path = str(ALGORISM_DIR / "local_test_data" / "sketch_placement_cart_reload_result.png")
plt.savefig(out_path, dpi=130)
print("\n그래프 저장:", out_path)

print("\n=== 요약 ===")
print(f"기존(고정) 7개: {len(old_plans)}/7개 위치 재현 성공")
print(f"신규(카트) 3개: {len(new_plans)}/3개 남는 공간에 배치 성공")
print(f"미적재: {unloadable if unloadable else '없음'}")

# ---------------------------------------------------------------------------
# 전체 흐름 표: [카트 피킹 순서] -> [트렁크에 실제로 놓은 순서/위치].
# 기존 7개는 "고정(안 움직임)"으로 표시해서, 신규 3개만 이번 세션에서 로봇이
# 실제로 수행하는 동작이라는 걸 명확히 한다.
# ---------------------------------------------------------------------------
print("\n=== 전체 흐름: 카트 피킹 순서 vs 트렁크 적재 순서 ===")
pick_rank = {b.id: i for i, b in enumerate(cart_pick_order, start=1)}
plan_by_id = {p.box_id: p for p in all_plans}
header = f"{'box_id':<12}{'카트 피킹 순서':<16}{'트렁크 적재 순서':<18}{'층':<6}{'위치(x,y,z)'}"
print(header)
print("-" * len(header))
for b in all_boxes:
    pick = str(pick_rank[b.id]) if b.id in pick_rank else "-(기존,고정)"
    plan = plan_by_id.get(b.id)
    if plan is None:
        print(f"{b.id:<12}{pick:<16}{'미적재':<18}{'-':<6}-")
        continue
    layer = "2층" if plan.position[2] > 1e-9 else "1층"
    x, y, z = plan.position
    order_label = str(plan.order) if b.id in new_ids else "고정(알고리즘 미적용)"
    print(f"{b.id:<12}{pick:<16}{order_label:<18}{layer:<6}({x:.3f}, {y:.3f}, {z:.3f})")
