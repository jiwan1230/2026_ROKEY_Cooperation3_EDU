"""
_viz_helpers.py
여러 손그림 데모 스크립트가 거의 똑같이 복붙해서 쓰던 3D+top-down 시각화 코드를
한 곳으로 모은 공용 헬퍼. 새 시나리오 스크립트는 이 파일의 draw_scene()만 호출하면
같은 스타일의 그림을 얻는다.

비포/애프터 한 쌍을 만들려면 같은 trunk/obstacles로 draw_scene()을 두 번 호출하면
된다 - "before"는 placed=[]에 아직 안 놓인 박스를 waiting=[]에 넣고, "after"는
placed=[실제 배치 결과]로 채우고 waiting=[]로 비운다.
"""
import sys
import pathlib
import types

ALGORISM_DIR = pathlib.Path("/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU/isaacpjt/Cart2Trunk/algorism")

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

ROBOT_TO_TRUNK_GAP = 0.30


def _cuboid_faces(x0, y0, z0, dx, dy, dz):
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


def _draw_cuboid(ax, x0, y0, z0, dx, dy, dz, facecolor, edgecolor, alpha=0.85, linewidth=1.0, linestyle="-"):
    coll = Poly3DCollection(_cuboid_faces(x0, y0, z0, dx, dy, dz), facecolor=facecolor,
                             edgecolor=edgecolor, alpha=alpha, linewidths=linewidth, linestyle=linestyle)
    ax.add_collection3d(coll)


def _draw_wireframe_box(ax, x0, y0, z0, dx, dy, dz, color, label=None):
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


class SceneBox:
    """draw_scene()에 넘길 박스 하나 - 위치/치수/색/표시 방식을 담는다."""

    def __init__(self, box_id, x, y, z, width, depth, height, color, dashed=False, stack_on_id=None):
        self.box_id = box_id
        self.x, self.y, self.z = x, y, z
        self.width, self.depth, self.height = width, depth, height
        self.color = color
        self.dashed = dashed  # True면 2층(쌓인) 박스처럼 굵은 점선 테두리
        # waiting_boxes 전용 - 카트 안에서 이 박스가 어떤 박스 위에 얹혀 있는지
        # (다른 waiting box의 box_id). draw_scene()이 "대기 중" 칸을 그릴 때
        # 나란히 눕혀놓지 않고 실제로 쌓인 모양 그대로 그리는 데 쓴다.
        self.stack_on_id = stack_on_id


def _resolve_waiting_layout(waiting_boxes):
    """
    대기 중인(카트) 박스들의 배치를 계산한다. stack_on_id가 없는 박스는 새
    "칸"(컬럼)의 바닥이 되고, stack_on_id가 있는 박스는 그 박스 바로 위에
    (같은 칸, z만 그 박스 높이만큼 더해서) 쌓인다 - 그래야 카트 안에서 실제로
    얹혀있는 모양이 "나란히 따로" 대신 "쌓인 채로" 그려진다.
    반환: box_id -> (col_index, z, level) 딕셔너리.
    """
    by_id = {wb.box_id: wb for wb in waiting_boxes}
    col_of, z_of, level_of = {}, {}, {}
    resolved = set()

    col_counter = 0
    for wb in waiting_boxes:
        if wb.stack_on_id is None:
            col_of[wb.box_id] = col_counter
            z_of[wb.box_id] = 0.0
            level_of[wb.box_id] = 0
            resolved.add(wb.box_id)
            col_counter += 1

    remaining = [wb for wb in waiting_boxes if wb.box_id not in resolved]
    guard = 0
    while remaining and guard <= len(waiting_boxes):
        guard += 1
        for wb in list(remaining):
            if wb.stack_on_id in resolved:
                base = by_id[wb.stack_on_id]
                col_of[wb.box_id] = col_of[wb.stack_on_id]
                z_of[wb.box_id] = z_of[wb.stack_on_id] + base.height
                level_of[wb.box_id] = level_of[wb.stack_on_id] + 1
                resolved.add(wb.box_id)
                remaining.remove(wb)
    for wb in remaining:  # stack_on_id가 목록에 없는 등 못 푼 경우 - 새 칸으로 안전하게 처리
        col_of[wb.box_id] = col_counter
        z_of[wb.box_id] = 0.0
        level_of[wb.box_id] = 0
        col_counter += 1

    return col_of, z_of, level_of


def draw_scene(
    trunk_width, trunk_depth, trunk_height,
    fixed_obstacles,     # List[SceneBox] - 차 바퀴/장애물/이미 확정된 짐 (항상 트렁크 안에 고정)
    placed_boxes,        # List[SceneBox] - 이번에 알고리즘이 트렁크 안에 배치한 결과
    waiting_boxes,       # List[SceneBox] - 아직 카트에 있어서 트렁크 밖(왼쪽)에 대기 중인 박스
    title, out_path,
    subtitle_3d="3D 아이소메트릭", subtitle_2d="top-down",
):
    """
    trunk + fixed_obstacles + placed_boxes(트렁크 안) + waiting_boxes(트렁크 왼쪽 밖,
    "아직 안 실음"을 표현)를 3D + top-down 2패널로 그려서 out_path에 저장한다.
    "before" 이미지는 placed_boxes=[], waiting_boxes에 카트 내용물을 넣어서 호출하고,
    "after" 이미지는 placed_boxes=결과, waiting_boxes=[]로 호출하면 짝이 맞는다.
    """
    G = ROBOT_TO_TRUNK_GAP
    robot_y = trunk_depth / 2

    fig = plt.figure(figsize=(14, 7.5))
    fig.suptitle(title, fontsize=13, weight="bold")

    # ---- 왼쪽: 3D 아이소메트릭 ----
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax3d.scatter([0], [robot_y], [0], color="crimson", s=80, label="로봇 (base 원점)")
    ax3d.quiver(0, robot_y, 0, G, 0, 0, color="crimson", linewidth=2, arrow_length_ratio=0.15)
    _draw_wireframe_box(ax3d, G, 0, 0, trunk_width, trunk_depth, trunk_height, color="red", label="trunk")

    for obs in fixed_obstacles:
        _draw_cuboid(ax3d, G + obs.x, obs.y, obs.z, obs.width, obs.depth, obs.height,
                     facecolor=obs.color, edgecolor=obs.color, alpha=0.55)
    for pb in placed_boxes:
        _draw_cuboid(ax3d, G + pb.x, pb.y, pb.z, pb.width, pb.depth, pb.height,
                     facecolor=pb.color, edgecolor=pb.color, alpha=0.9,
                     linewidth=(2.4 if pb.dashed else 1.0))
    # 대기 중인(아직 안 실은) 박스는 카트 자리(트렁크 왼쪽 바깥)에 흐릿하게 표시.
    # stack_on_id로 얹힌 관계가 있으면 나란히가 아니라 실제로 쌓인 모양으로 그린다.
    wx = -0.05
    col_of, z_of, _level_of = _resolve_waiting_layout(waiting_boxes)
    for wb in waiting_boxes:
        wy = 0.10 + col_of[wb.box_id] * 0.20
        _draw_cuboid(ax3d, wx - wb.width, wy, z_of[wb.box_id], wb.width, wb.depth, wb.height,
                     facecolor=wb.color, edgecolor=wb.color, alpha=0.4, linestyle="--")

    ax3d.set_xlim(wx - 0.3, G + trunk_width)
    ax3d.set_ylim(0, trunk_depth)
    ax3d.set_zlim(0, trunk_height)
    ax3d.set_xlabel("x (m)")
    ax3d.set_ylabel("y (m)")
    ax3d.set_zlabel("z (m)")
    ax3d.set_title(subtitle_3d, fontsize=10)
    ax3d.legend(loc="upper left", fontsize=7)
    ax3d.set_box_aspect((G + trunk_width - (wx - 0.3), trunk_depth, trunk_height))

    # ---- 오른쪽: 2D top-down ----
    ax2d = fig.add_subplot(1, 2, 2)
    ax2d.scatter([0], [robot_y], color="crimson", s=100, zorder=5, label="로봇 (base 원점)")
    ax2d.annotate("", xy=(G, robot_y), xytext=(0, robot_y),
                  arrowprops=dict(arrowstyle="->", color="crimson", linewidth=2))
    ax2d.text(G / 2, robot_y + 0.02, "접근 방향(+x)", color="crimson", fontsize=8, ha="center")
    ax2d.add_patch(Rectangle((G, 0), trunk_width, trunk_depth, fill=False, edgecolor="red", linewidth=2, label="trunk"))

    for obs in fixed_obstacles:
        ax2d.add_patch(Rectangle((G + obs.x, obs.y), obs.width, obs.depth, facecolor=obs.color,
                                  edgecolor=obs.color, alpha=0.55))
        ax2d.text(G + obs.x + obs.width / 2, obs.y + obs.depth / 2, obs.box_id,
                  fontsize=6, ha="center", va="center", color="black")
    for pb in placed_boxes:
        ax2d.add_patch(Rectangle((G + pb.x, pb.y), pb.width, pb.depth, facecolor=pb.color,
                                  edgecolor=("navy" if pb.dashed else pb.color),
                                  alpha=0.9, linewidth=(2.4 if pb.dashed else 1.3),
                                  linestyle=("--" if pb.dashed else "-")))
        ax2d.text(G + pb.x + pb.width / 2, pb.y + pb.depth / 2, pb.box_id,
                  fontsize=6, ha="center", va="center", color="white", weight="bold")
    # top-down은 z를 못 그리니, 쌓인 레벨마다 x로 살짝 밀어서 카드 겹치듯 보이게 한다
    # (레벨 0=제일 아래는 안 밀림, 레벨이 올라갈수록 조금씩 더 오른쪽으로).
    for wb in waiting_boxes:
        wy = 0.10 + col_of[wb.box_id] * 0.20
        level_shift = _level_of[wb.box_id] * 0.025
        ax2d.add_patch(Rectangle((wx - wb.width + level_shift, wy), wb.width, wb.depth, facecolor=wb.color,
                                  edgecolor=wb.color, alpha=0.4, linestyle="--", linewidth=1.5))
        ax2d.text(wx - wb.width / 2 + level_shift, wy + wb.depth / 2, wb.box_id, fontsize=6, ha="center", va="center")
    if waiting_boxes:
        ax2d.text(wx - 0.15, -0.06, "카트(대기 중)", fontsize=8, ha="center", color="dimgray")

    ax2d.set_xlim(wx - 0.35, G + trunk_width + 0.05)
    ax2d.set_ylim(-0.1, trunk_depth + 0.05)
    ax2d.set_xlabel("x (m)")
    ax2d.set_ylabel("y (m)")
    ax2d.set_title(subtitle_2d, fontsize=10)
    ax2d.legend(loc="upper left", fontsize=7)
    ax2d.set_aspect("equal")

    plt.tight_layout()
    plt.savefig(out_path, dpi=130)
    plt.close(fig)
    print("그래프 저장:", out_path)
