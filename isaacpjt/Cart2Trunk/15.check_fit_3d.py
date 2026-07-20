"""
15.check_fit_3d.py
trunk_map.json의 extreme point(AABB min/max corner) 기반으로 후보 박스가 트렁크 안에
들어갈 수 있는지 판정하고, 트렁크 + 기존 점유 공간(obstacle_N) + 후보 박스를 3D로 시각화한다.

Isaac Sim 불필요 (일반 python3 + numpy + matplotlib).

    python3 15.check_fit_3d.py                                        # 최신 run, 기본 후보 박스(입구쪽 자동 배치)
    python3 15.check_fit_3d.py results/run_YYYYMMDD_HHMMSS
    python3 15.check_fit_3d.py --size 0.30 0.20 0.15 --center 0.85 0.1
    python3 15.check_fit_3d.py --size 0.30 0.20 0.15 --center 1.3 0.1 --z 0.014

판정 로직 (전부 AABB min/max corner = extreme point 비교, 축정렬 박스 가정):
  1. 후보 박스가 트렁크 AABB(바닥~ceiling_limit, 좌우/안쪽 벽 안) 안에 완전히 포함되는가
  2. 각 obstacle_N의 AABB와 x/y/z 세 축 모두에서 겹치는가(한 축이라도 분리돼 있으면 그
     obstacle과는 비충돌)
둘 다 통과해야 "배치 가능"(초록), 하나라도 실패하면 "배치 불가"(빨강)이며 어떤 obstacle과
충돌했는지/트렁크 밖으로 나갔는지 콘솔에 출력한다. 물리적으로 떠 있는지(지지 여부)는 보지 않는
순수 기하학적 AABB 겹침 검사다.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (3d projection 등록용)
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.lines import Line2D

for _font in ("NanumSquareRound", "NanumGothic", "Noto Sans CJK KR", "Noto Sans CJK JP", "Noto Sans CJK SC"):
    if any(_font in f.name for f in matplotlib.font_manager.fontManager.ttflist):
        matplotlib.rcParams["font.family"] = _font
        break
matplotlib.rcParams["axes.unicode_minus"] = False

_THIS_DIR = Path(__file__).resolve().parent
RESULTS_DIR = _THIS_DIR / "results"

DEFAULT_BOX_SIZE = (0.30, 0.20, 0.15)  # (x=깊이,y=폭,z=높이) - 4.add_boxes_and_zones.py의 "Small" 규격


def resolve_trunk_map_path(arg: str | None) -> Path:
    if arg:
        p = Path(arg)
        if not p.is_absolute():
            p = _THIS_DIR / p
        if p.is_dir():
            p = p / "pointcloud" / "trunk_map.json"
        if not p.exists():
            raise SystemExit(f"[에러] {p}가 없습니다.")
        return p
    runs = sorted(RESULTS_DIR.glob("run_*"))
    for run in reversed(runs):
        cand = run / "pointcloud" / "trunk_map.json"
        if cand.exists():
            print(f"[자동 선택] 최신 trunk_map.json: {cand.relative_to(_THIS_DIR)}")
            return cand
    raise SystemExit(f"[에러] {RESULTS_DIR}에 trunk_map.json을 가진 run이 없습니다 (13.export_trunk_map.py 먼저 실행).")


def default_candidate_center(trunk_min: np.ndarray, trunk_max: np.ndarray, size: np.ndarray) -> tuple[float, float]:
    """기본 후보 위치: 입구(x_min) 쪽, y 중앙 - 별도 --center 없이 실행해도 바로 결과를 볼 수 있게."""
    cx = trunk_min[0] + size[0] / 2.0 + 0.05
    cy = (trunk_min[1] + trunk_max[1]) / 2.0
    return float(cx), float(cy)


# --------------------------------------------------------------------------- #
# AABB(extreme point) 겹침 판정
# --------------------------------------------------------------------------- #

def obstacle_aabb(obs: dict) -> tuple[np.ndarray, np.ndarray]:
    v = np.array(obs["vertices"])
    return v.min(axis=0), v.max(axis=0)


def aabbs_overlap(min_a, max_a, min_b, max_b) -> bool:
    return bool(np.all(min_a < max_b) and np.all(min_b < max_a))


def aabb_contains(outer_min, outer_max, inner_min, inner_max) -> bool:
    return bool(np.all(inner_min >= outer_min) and np.all(inner_max <= outer_max))


def check_fit(trunk_map: dict, box_min: np.ndarray, box_max: np.ndarray) -> dict:
    v = np.array(trunk_map["vertices"])
    trunk_min, trunk_max = v.min(axis=0), v.max(axis=0)
    within_trunk = aabb_contains(trunk_min, trunk_max, box_min, box_max)

    collisions = []
    for obs in trunk_map.get("obstacles", []):
        obs_min, obs_max = obstacle_aabb(obs)
        if aabbs_overlap(box_min, box_max, obs_min, obs_max):
            collisions.append(obs["name"])

    return {
        "fits": within_trunk and not collisions,
        "within_trunk": within_trunk,
        "collisions": collisions,
        "trunk_min": trunk_min,
        "trunk_max": trunk_max,
    }


# --------------------------------------------------------------------------- #
# 3D 시각화
# --------------------------------------------------------------------------- #

BOX_FACE_IDX = [
    [0, 1, 2, 3], [4, 5, 6, 7],  # bottom, top
    [0, 1, 5, 4], [3, 2, 6, 7],  # y_min면, y_max면
    [0, 3, 7, 4], [1, 2, 6, 5],  # x_min면, x_max면
]
BOX_EDGE_IDX = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4],
                [0, 4], [1, 5], [2, 6], [3, 7]]


def box_vertices_from_minmax(box_min, box_max):
    x0, y0, z0 = box_min
    x1, y1, z1 = box_max
    return np.array([
        [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
        [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
    ])


def draw_box_edges(ax, verts, color, lw=2, ls="-"):
    for a, b in BOX_EDGE_IDX:
        p0, p1 = verts[a], verts[b]
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], color=color, linewidth=lw, linestyle=ls)


def draw_box_faces(ax, verts, color, alpha=0.25):
    faces = [[verts[i] for i in idx] for idx in BOX_FACE_IDX]
    poly = Poly3DCollection(faces, facecolor=color, edgecolor=color, alpha=alpha, linewidths=1)
    ax.add_collection3d(poly)


def render_view(trunk_map, candidate_verts, fit_result, out_path, elev, azim, title_suffix):
    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")

    v = np.array(trunk_map["vertices"])
    for e in trunk_map["edges"]:
        a, b = e["v"]
        color = "red" if e["style"] == "solid" else "orange"
        ls = "-" if e["style"] == "solid" else "--"
        ax.plot([v[a][0], v[b][0]], [v[a][1], v[b][1]], [v[a][2], v[b][2]], color=color, linewidth=2, linestyle=ls)

    for obs in trunk_map.get("obstacles", []):
        ov = np.array(obs["vertices"])
        draw_box_edges(ax, ov, "dodgerblue", lw=1.5)
        draw_box_faces(ax, ov, "dodgerblue", alpha=0.25)

    cand_color = "limegreen" if fit_result["fits"] else "crimson"
    draw_box_edges(ax, candidate_verts, cand_color, lw=2.5)
    draw_box_faces(ax, candidate_verts, cand_color, alpha=0.35)

    ax.set_xlabel("x (base, m, +deep)")
    ax.set_ylabel("y (base, m)")
    ax.set_zlabel("z (base, m, +up)")
    ax.view_init(elev=elev, azim=azim)

    trunk_min, trunk_max = fit_result["trunk_min"], fit_result["trunk_max"]
    pad = 0.1
    ax.set_xlim(trunk_min[0] - pad, trunk_max[0] + pad)
    ax.set_ylim(trunk_min[1] - pad, trunk_max[1] + pad)
    ax.set_zlim(0, trunk_max[2] + pad)
    try:
        ax.set_box_aspect((
            trunk_max[0] - trunk_min[0] + 2 * pad,
            trunk_max[1] - trunk_min[1] + 2 * pad,
            trunk_max[2] + pad,
        ))
    except AttributeError:
        pass  # 오래된 matplotlib은 set_box_aspect가 없음 - 비율 안 맞아도 기능엔 지장 없음

    verdict = "FIT (배치 가능)" if fit_result["fits"] else "NO FIT (배치 불가)"
    legend = [
        Line2D([0], [0], color="red", lw=2, label="trunk solid (실측)"),
        Line2D([0], [0], color="orange", lw=2, linestyle="--", label="trunk dashed (설계한계)"),
        Line2D([0], [0], color="dodgerblue", lw=2, label="obstacle (점유 공간)"),
        Line2D([0], [0], color=cand_color, lw=2, label=f"candidate box - {verdict}"),
    ]
    ax.legend(handles=legend, loc="upper left", fontsize=8)
    ax.set_title(f"trunk fit-check ({title_suffix}) - {trunk_map['run_id']}")

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"[저장] {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", nargs="?", default=None,
                         help="results/run_YYYYMMDD_HHMMSS 경로 또는 trunk_map.json 직접 경로 (생략 시 최신 run 자동 선택)")
    parser.add_argument("--size", nargs=3, type=float, default=list(DEFAULT_BOX_SIZE), metavar=("L", "W", "H"),
                         help="후보 박스 크기 (x=깊이,y=폭,z=높이), 기본값 Small박스 0.30 0.20 0.15")
    parser.add_argument("--center", nargs=2, type=float, default=None, metavar=("X", "Y"),
                         help="후보 박스 중심 x,y (base frame). 생략 시 입구쪽 y중앙에 자동 배치")
    parser.add_argument("--z", type=float, default=None,
                         help="후보 박스 바닥면(z_min) 높이. 생략 시 트렁크 실측 바닥(floor_z)에 놓는다")
    args = parser.parse_args()

    trunk_map_path = resolve_trunk_map_path(args.run)
    trunk_map = json.loads(trunk_map_path.read_text())

    size = np.array(args.size, dtype=float)
    v = np.array(trunk_map["vertices"])
    trunk_min, trunk_max = v.min(axis=0), v.max(axis=0)
    floor_z = float(trunk_min[2])

    if args.center is not None:
        cx, cy = args.center
    else:
        cx, cy = default_candidate_center(trunk_min, trunk_max, size)
    z_min = args.z if args.z is not None else floor_z

    box_min = np.array([cx - size[0] / 2.0, cy - size[1] / 2.0, z_min])
    box_max = box_min + size
    print(f"[후보 박스] size={size.tolist()} center_xy=({cx:.3f},{cy:.3f}) "
          f"min={np.round(box_min, 3).tolist()} max={np.round(box_max, 3).tolist()}")

    fit_result = check_fit(trunk_map, box_min, box_max)
    if fit_result["fits"]:
        print(f"[결과] FIT - 트렁크 안에 들어갑니다 "
              f"(obstacle {len(trunk_map.get('obstacles', []))}개와 비충돌, 트렁크 AABB 내부)")
    else:
        reasons = []
        if not fit_result["within_trunk"]:
            reasons.append("트렁크 AABB를 벗어남")
        if fit_result["collisions"]:
            reasons.append(f"충돌: {', '.join(fit_result['collisions'])}")
        print(f"[결과] NO FIT - {' / '.join(reasons)}")

    candidate_verts = box_vertices_from_minmax(box_min, box_max)
    out_dir = trunk_map_path.parent
    render_view(trunk_map, candidate_verts, fit_result, out_dir / "fit_check_iso.png",
                elev=25, azim=-60, title_suffix="isometric")
    render_view(trunk_map, candidate_verts, fit_result, out_dir / "fit_check_top.png",
                elev=88, azim=-90, title_suffix="top-down")


if __name__ == "__main__":
    main()
