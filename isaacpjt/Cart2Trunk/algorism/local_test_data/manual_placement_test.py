import sys, pathlib
from importlib import import_module

ALGORISM_DIR = pathlib.Path("/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU/isaacpjt/Cart2Trunk/algorism")
sys.path.insert(0, str(ALGORISM_DIR))

m02 = import_module("02_trunk_space_state")
m03 = import_module("03_extreme_point_candidates")
m07 = import_module("07_placement_plan")

Box = m03.Box
ExtremePointState = m03.ExtremePointState
place_one_box = m07.place_one_box
local_to_base_frame = m02.local_to_base_frame

RUN = "run_20260720_200104"
json_path = str(ALGORISM_DIR.parents[3] / RUN / "pointcloud" / "trunk_map.json")

world_map = m02.load_trunk_from_world_map(json_path)
trunk, offset = world_map.to_bounding_trunk()
obstacles = m02.load_obstacles_from_world_map(json_path, offset)

print("트렁크(로컬):", trunk)
print("오프셋(base frame 원점):", offset)
print("장애물 개수:", len(obstacles))

state = ExtremePointState()
for obs in obstacles:
    state.register_placement(obs)

small_box = Box("Small", width=0.30, depth=0.20, height=0.15)  # PDF 예시와 동일 규격
plan = place_one_box(small_box, trunk, state, order=1)

if plan is None:
    print("\n결과: 배치 불가 (자리 없음)")
else:
    lx, ly, lz = plan.position
    bx, by, bz = local_to_base_frame(lx, ly, lz, offset)
    print(f"\n로컬 좌표: ({lx:.4f}, {ly:.4f}, {lz:.4f})")
    print(f"base frame 좌표 (PDF 이미지와 비교 가능): ({bx:.4f}, {by:.4f}, {bz:.4f})")
    print(f"박스 영역 (base frame): x[{bx:.3f},{bx+small_box.width:.3f}] y[{by:.3f},{by+small_box.depth:.3f}] z[{bz:.3f},{bz+small_box.height:.3f}]")
    print(f"점수: {plan.score:.4f}  접촉면: {plan.touches}/6")

# ---- 시각화 준비 ----
# 이 환경은 pip matplotlib(~/.local, 최신)와 apt matplotlib(/usr/lib/python3/dist-packages, 구버전)이
# 같이 깔려있다. mpl_toolkits는 .local 쪽엔 __init__.py가 없는 네임스페이스 패키지라서,
# import 시 뒤쪽 경로의 구버전(정규 패키지, __init__.py 있음)이 항상 이겨버려 3D(Axes3D)가 깨진다.
# site-packages는 건드리지 않고, 이 스크립트 안에서만 'mpl_toolkits'를 .local 위치로 미리
# sys.modules에 등록해서 우회한다 (dist-packages는 cycler 등 다른 의존성 때문에 sys.path에 남겨둠).
import types
_local_site = next(p for p in sys.path if p.endswith("site-packages") and "/.local/" in p)
_pkg = types.ModuleType("mpl_toolkits")
_pkg.__path__ = [str(pathlib.Path(_local_site) / "mpl_toolkits")]
sys.modules["mpl_toolkits"] = _pkg

import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"  # 한글 라벨 깨짐 방지 (이 환경엔 KR 변형이 폰트매니저에 안 잡혀서 JP로 대체 - 한글 글리프는 포함됨)
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def cuboid_faces(x0, y0, z0, dx, dy, dz):
    """AABB 박스 하나의 6개 면을 Poly3DCollection에 넣을 정점 리스트로 반환."""
    x1, y1, z1 = x0 + dx, y0 + dy, z0 + dz
    v = [
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),  # 바닥
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),  # 천장
    ]
    return [
        [v[0], v[1], v[2], v[3]],  # 바닥
        [v[4], v[5], v[6], v[7]],  # 천장
        [v[0], v[1], v[5], v[4]],  # 앞
        [v[2], v[3], v[7], v[6]],  # 뒤
        [v[1], v[2], v[6], v[5]],  # 오른쪽
        [v[0], v[3], v[7], v[4]],  # 왼쪽
    ]


def draw_cuboid(ax, x0, y0, z0, dx, dy, dz, facecolor, edgecolor, alpha=0.35, label=None):
    coll = Poly3DCollection(cuboid_faces(x0, y0, z0, dx, dy, dz), facecolor=facecolor,
                             edgecolor=edgecolor, alpha=alpha, linewidths=1.2)
    ax.add_collection3d(coll)
    if label:
        # 범례에 3D collection이 바로 안 잡혀서, 빈 라인으로 프록시 핸들 하나 추가
        ax.plot([], [], color=edgecolor, label=label)


def draw_wireframe_box(ax, x0, y0, z0, dx, dy, dz, color, label=None):
    """면을 안 채우고 테두리 12개 선만 그린다 (트렁크 경계용 - PDF의 빨간 테두리 스타일)."""
    x1, y1, z1 = x0 + dx, y0 + dy, z0 + dz
    v = {
        "000": (x0, y0, z0), "100": (x1, y0, z0), "110": (x1, y1, z0), "010": (x0, y1, z0),
        "001": (x0, y0, z1), "101": (x1, y0, z1), "111": (x1, y1, z1), "011": (x0, y1, z1),
    }
    edges = [
        ("000", "100"), ("100", "110"), ("110", "010"), ("010", "000"),  # 바닥 4변
        ("001", "101"), ("101", "111"), ("111", "011"), ("011", "001"),  # 천장 4변
        ("000", "001"), ("100", "101"), ("110", "111"), ("010", "011"),  # 기둥 4개
    ]
    for a, b in edges:
        xs, ys, zs = zip(v[a], v[b])
        ax.plot3D(xs, ys, zs, color=color, linewidth=2)
    if label:
        ax.plot([], [], color=color, linewidth=2, label=label)


tx0, ty0, tz0 = offset
tx1, ty1, tz1 = tx0 + trunk.width, ty0 + trunk.depth, tz0 + trunk.height

fig = plt.figure(figsize=(13, 7))

# ---- 왼쪽: 3D 아이소메트릭 (PDF 1페이지 스타일) ----
ax3d = fig.add_subplot(1, 2, 1, projection="3d")
draw_wireframe_box(ax3d, tx0, ty0, tz0, trunk.width, trunk.depth, trunk.height,
                    color="red", label="trunk (실측)")

for i, obs in enumerate(obstacles):
    obx, oby, obz = local_to_base_frame(obs.x, obs.y, obs.z, offset)
    draw_cuboid(ax3d, obx, oby, obz, obs.box.width, obs.box.depth, obs.box.height,
                facecolor="steelblue", edgecolor="blue",
                label="obstacle (점유 공간)" if i == 0 else None)

if plan is not None:
    lx, ly, lz = plan.position
    bx, by, bz = local_to_base_frame(lx, ly, lz, offset)
    draw_cuboid(ax3d, bx, by, bz, small_box.width, small_box.depth, small_box.height,
                facecolor="orange", edgecolor="darkorange", alpha=0.75, label="우리 알고리즘 배치 결과")

ax3d.set_xlim(tx0, tx1)
ax3d.set_ylim(ty0, ty1)
ax3d.set_zlim(tz0, tz0 + max(trunk.height, 0.3))
ax3d.set_xlabel("x (base, m, +deep)")
ax3d.set_ylabel("y (base, m)")
ax3d.set_zlabel("z (base, m, +up)")
ax3d.set_title(f"3D 아이소메트릭 - {RUN}")
ax3d.legend(loc="upper left", fontsize=8)
ax3d.set_box_aspect((trunk.width, trunk.depth, max(trunk.height, 0.3)))

# ---- 오른쪽: 2D 탑다운 (기존 그대로) ----
ax2d = fig.add_subplot(1, 2, 2)
ax2d.add_patch(Rectangle((tx0, ty0), trunk.width, trunk.depth, fill=False, edgecolor="red",
                          linewidth=2, label="trunk solid (실측)"))
for i, obs in enumerate(obstacles):
    obx, oby, _ = local_to_base_frame(obs.x, obs.y, obs.z, offset)
    ax2d.add_patch(Rectangle((obx, oby), obs.box.width, obs.box.depth, facecolor="steelblue",
                              edgecolor="blue", alpha=0.5, label="obstacle (점유 공간)" if i == 0 else None))
if plan is not None:
    ax2d.add_patch(Rectangle((bx, by), small_box.width, small_box.depth, facecolor="orange",
                              edgecolor="darkorange", alpha=0.7, linewidth=2, label="우리 알고리즘 배치 결과"))
ax2d.add_patch(Rectangle((0.72, 0.03), 0.30, 0.30, fill=False, edgecolor="green", linewidth=2,
                          linestyle="--", label="PDF 초록 박스 (참고, 팀 스크립트 결과)"))

pad = 0.15
ax2d.set_xlim(tx0 - pad, tx1 + pad)
ax2d.set_ylim(ty0 - pad, ty1 + pad)
ax2d.set_xlabel("x (base, m, +deep)")
ax2d.set_ylabel("y (base, m)")
ax2d.set_title(f"top-down - {RUN}")
ax2d.legend(loc="upper left", fontsize=8)
ax2d.set_aspect("equal")

plt.tight_layout()
out_path = str(ALGORISM_DIR / "local_test_data" / "our_algorithm_placement_200104.png")
plt.savefig(out_path, dpi=130)
print("\n그래프 저장:", out_path)
