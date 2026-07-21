import sys, pathlib, types
from importlib import import_module

ALGORISM_DIR = pathlib.Path("/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU/isaacpjt/Cart2Trunk/algorism")
sys.path.insert(0, str(ALGORISM_DIR))

m02 = import_module("02_trunk_space_state")
m03 = import_module("03_extreme_point_candidates")
m06 = import_module("06_loading_order_decision")
m07 = import_module("07_placement_plan")

Trunk = m02.Trunk
Box = m03.Box
PlacedBox = m03.PlacedBox
ExtremePointState = m03.ExtremePointState
decide_loading_order = m06.decide_loading_order
place_one_box = m07.place_one_box

# ---------------------------------------------------------------------------
# 사용자가 손그림으로 그린 트렁크를 픽셀 비율 그대로 추정해서 실측 단위(m)로 변환.
# 트렁크 사각형 픽셀 크기(가로 650 x 세로 790, displayed 2000폭 기준)를 기준으로
# 가로(width, x축)=0.60m 로 잡고 같은 스케일(0.60/650 m/px)로 나머지를 다 환산함.
# -> 세로(depth, y축) = 790 * (0.60/650) = 0.729m
# 검정 네모(차 바퀴/휠하우스) 2개는 오른쪽 벽에 붙어서 위/아래 구석에 있는 고정 장애물로,
# 초록 사각형 3개는 "이런 크기의 박스가 있다"는 뜻으로 보고 크기만 가져오고
# 배치 위치는 우리 알고리즘이 직접 고르게 한다 (그림 속 위치는 참고용으로만 겹쳐 그림).
# ---------------------------------------------------------------------------

TRUNK_WIDTH = 0.60   # x축
TRUNK_DEPTH = 0.73   # y축
TRUNK_HEIGHT = 0.40

# 로봇은 M0609 base 좌표계 원점(0,0,0)에 고정되어 있고, 트렁크는 그 원점에서 x축
# 방향으로 조금 떨어진 곳에 있다 (실제 데이터의 offset과 같은 개념 - 로봇과 트렁크가
# 같은 자리에 있는 게 아니라, 로봇이 팔을 뻗어야 하는 만큼 떨어져 있다는 걸 명시하기
# 위해 임의의 간격을 둠). 그래프의 (0,0) = 로봇, 트렁크는 거기서 오른쪽(+x)으로
# ROBOT_TO_TRUNK_GAP만큼 떨어진 자리에 그린다 - 로봇은 항상 이 +x 방향으로만 접근한다.
ROBOT_TO_TRUNK_GAP = 0.30

trunk = Trunk(width=TRUNK_WIDTH, depth=TRUNK_DEPTH, height=TRUNK_HEIGHT)

wheel_front = PlacedBox(box=Box("Wheel_Front", width=0.16, depth=0.15, height=0.20),
                         x=0.44, y=0.00, z=0.0)
wheel_rear = PlacedBox(box=Box("Wheel_Rear", width=0.16, depth=0.21, height=0.20),
                        x=0.44, y=0.52, z=0.0)
obstacles = [wheel_front, wheel_rear]

# 그림 속 초록 박스 3개 - 크기만 추정 (모양: 세로긴것/중간것/가로긴것)
green_boxes = [
    Box("Green_Tall", width=0.07, depth=0.16, height=0.15),
    Box("Green_Mid", width=0.14, depth=0.16, height=0.15),
    Box("Green_Wide", width=0.28, depth=0.07, height=0.15),
]

# 그림 속에 실제로 그려져 있던 대략 위치 (참고/비교용, 알고리즘 입력으로는 안 씀)
sketch_positions = {
    "Green_Tall": (0.10, 0.10),
    "Green_Mid": (0.17, 0.30),
    "Green_Wide": (0.09, 0.54),
}

print("트렁크:", trunk)
print("장애물(차 바퀴) 2개:", [(o.box.id, o.x, o.y, o.box.width, o.box.depth) for o in obstacles])

state = ExtremePointState()
for obs in obstacles:
    state.register_placement(obs)

order = decide_loading_order(green_boxes)
print("\n적재 순서(부피 큰 순):", [b.id for b in order])

plans = []
for i, box in enumerate(order, start=1):
    plan = place_one_box(box, trunk, state, order=i)
    if plan is None:
        print(f"{box.id}: 배치 불가 (자리 없음)")
    else:
        plans.append(plan)
        x, y, z = plan.position
        print(f"{box.id}: 배치 -> x[{x:.3f},{x+box.width:.3f}] y[{y:.3f},{y+box.depth:.3f}] "
              f"z[{z:.3f},{z+box.height:.3f}]  점수={plan.score:.4f} 접촉면={plan.touches}/6")

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


def draw_cuboid(ax, x0, y0, z0, dx, dy, dz, facecolor, edgecolor, alpha=0.5, label=None):
    coll = Poly3DCollection(cuboid_faces(x0, y0, z0, dx, dy, dz), facecolor=facecolor,
                             edgecolor=edgecolor, alpha=alpha, linewidths=1.2)
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


RESULT_COLORS = {"Green_Tall": "orange", "Green_Mid": "purple", "Green_Wide": "deeppink"}
G = ROBOT_TO_TRUNK_GAP  # 로컬 좌표를 그래프에 그릴 때 전부 이만큼 +x로 밀어서 그림 (로컬->로봇 base 환산)
# 로봇은 트렁크 왼쪽 "중앙"(y = 트렁크 깊이의 절반)에서 접근한다 (사용자 확인 - 왼쪽
# 아래 구석이 아니라 왼쪽 중앙). 로봇 자신의 y좌표만 이 값이고, 트렁크/장애물/배치
# 결과의 y좌표는 전부 로컬 좌표 그대로(0~TRUNK_DEPTH)라 이 상수와 별개다.
ROBOT_Y = TRUNK_DEPTH / 2

fig = plt.figure(figsize=(13, 7))

# ---- 왼쪽: 3D 아이소메트릭 ----
ax3d = fig.add_subplot(1, 2, 1, projection="3d")
# 로봇 자신은 원점(0, ROBOT_Y, 0)에 점으로 표시 - 트렁크와 겹치지 않게 확실히 띄워서 그린다
ax3d.scatter([0], [ROBOT_Y], [0], color="crimson", s=80, label="로봇 (base 원점)")
# 로봇 -> 트렁크 방향으로 화살표 (로봇은 항상 이 +x 방향으로만 접근)
ax3d.quiver(0, ROBOT_Y, 0, G, 0, 0, color="crimson", linewidth=2, arrow_length_ratio=0.15)
draw_wireframe_box(ax3d, G, 0, 0, TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT, color="red", label="trunk (그림 추정)")
for i, obs in enumerate(obstacles):
    draw_cuboid(ax3d, G + obs.x, obs.y, obs.z, obs.box.width, obs.box.depth, obs.box.height,
                facecolor="black", edgecolor="black", alpha=0.6,
                label="차 바퀴(휠하우스)" if i == 0 else None)
for i, plan in enumerate(plans):
    x, y, z = plan.position
    box = next(b for b in order if b.id == plan.box_id)
    color = RESULT_COLORS[plan.box_id]
    draw_cuboid(ax3d, G + x, y, z, box.width, box.depth, box.height,
                facecolor=color, edgecolor=color, alpha=0.75, label=f"{plan.box_id} (알고리즘 배치)")

ax3d.set_xlim(0, G + TRUNK_WIDTH)
ax3d.set_ylim(0, TRUNK_DEPTH)
ax3d.set_zlim(0, TRUNK_HEIGHT)
ax3d.set_xlabel("x (m) - 로봇 원점 기준, 로봇이 접근하는 방향")
ax3d.set_ylabel("y (m) - 좌우(입구와 무관)")
ax3d.set_zlabel("z (m, height)")
ax3d.set_title("3D 아이소메트릭 - 손그림 기반 테스트 (원점=로봇)")
ax3d.legend(loc="upper left", fontsize=7)
ax3d.set_box_aspect((G + TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT))

# ---- 오른쪽: 2D 탑다운 (그림 속 원래 위치도 점선으로 같이 표시) ----
ax2d = fig.add_subplot(1, 2, 2)
# 로봇 위치(원점) + 접근 방향 화살표 - 손그림의 "로봇 자리. 여기가 원점, 왼쪽 중앙에서
# 온다" 표기와 같은 구도 (y=0이 아니라 트렁크 깊이의 절반 높이에서 접근)
ax2d.scatter([0], [ROBOT_Y], color="crimson", s=100, zorder=5, label="로봇 (base 원점)")
ax2d.annotate("", xy=(G, ROBOT_Y), xytext=(0, ROBOT_Y),
              arrowprops=dict(arrowstyle="->", color="crimson", linewidth=2))
ax2d.text(G / 2, ROBOT_Y + 0.02, "접근 방향(+x, 고정)", color="crimson", fontsize=8, ha="center")

ax2d.add_patch(Rectangle((G, 0), TRUNK_WIDTH, TRUNK_DEPTH, fill=False, edgecolor="red",
                          linewidth=2, label="trunk (그림 추정)"))
for i, obs in enumerate(obstacles):
    ax2d.add_patch(Rectangle((G + obs.x, obs.y), obs.box.width, obs.box.depth, facecolor="black",
                              edgecolor="black", alpha=0.6, label="차 바퀴(휠하우스)" if i == 0 else None))
for i, plan in enumerate(plans):
    x, y, z = plan.position
    box = next(b for b in order if b.id == plan.box_id)
    color = RESULT_COLORS[plan.box_id]
    ax2d.add_patch(Rectangle((G + x, y), box.width, box.depth, facecolor=color, edgecolor=color,
                              alpha=0.7, linewidth=2, label=f"{plan.box_id} (알고리즘 배치)"))
for i, (box_id, (sx, sy)) in enumerate(sketch_positions.items()):
    box = next(b for b in green_boxes if b.id == box_id)
    ax2d.add_patch(Rectangle((G + sx, sy), box.width, box.depth, fill=False, edgecolor="green",
                              linewidth=2, linestyle="--",
                              label="그림 속 원래 위치(참고)" if i == 0 else None))

ax2d.set_xlim(-0.05, G + TRUNK_WIDTH + 0.05)
ax2d.set_ylim(-0.05, TRUNK_DEPTH + 0.05)
ax2d.set_xlabel("x (m) - 로봇 원점 기준, 로봇이 접근하는 방향")
ax2d.set_ylabel("y (m) - 좌우(입구와 무관)")
ax2d.set_title("top-down - 손그림 기반 테스트 (원점=로봇)")
ax2d.legend(loc="upper left", fontsize=7)
ax2d.set_aspect("equal")

plt.tight_layout()
out_path = str(ALGORISM_DIR / "local_test_data" / "sketch_placement_result.png")
plt.savefig(out_path, dpi=130)
print("\n그래프 저장:", out_path)
