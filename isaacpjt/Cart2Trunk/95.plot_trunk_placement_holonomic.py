"""95.plot_trunk_placement_holonomic.py
카트 박스 스캔 -> 트렁크 스캔(장애물 자동 검출 포함) -> 적재 알고리즘 배치까지 전체 흐름을
한 그림에. 37.plot_crate_placement.py(크레이트 데모)와 동일한 구성/팔레트를 홀로노믹
카트->트렁크 시나리오 데이터로 재사용한다.

사용자 지적("우리 트렁크 스캔할 때 장애물을 인지하는 로직이 분명 있었거든") 확인 결과 -
trunk_map.json에는 실제로 "obstacles" 필드가 있고(90.export_trunk_map_holonomic.py의
grid 휴리스틱 + connected-component 검출, 13.py와 동일 원리), 이번 스캔에서 4개가
검출됐다 - 좌우로 거의 대칭인 obstacle_1/2(휠하우스로 추정, 높이 0.16~0.17m)와 더 작은
obstacle_3/4. 다만 이 프로젝트의 어떤 실행 코드(92/94번)도 이 obstacles 필드를 읽어서
쓰지 않는다 - 적재 알고리즘(19_run_full_pipeline_with_yaw.py)만 배치 계산에 이걸
참고했다(그래서 두 박스 다 장애물에서 딱 4cm 여유로 배치됨 - 실제 실행 중 흔들림엔
너무 빠듯한 여유였다는 게 최근 STAGE 3.3 부딪힘의 배경).

왼쪽: 카트에서 스캔한 박스들(all_boxes_corners_*.json, 카트 스캔 세션 자체의 base_link
     좌표계 - 트렁크 스캔과는 다른 시점/자세라 오른쪽과 좌표계가 다르다).
오른쪽: 트렁크 내부(trunk_map.json의 AABB + obstacles + 원본 point cloud 밀도) 위에
       algorism이 계산한 배치(placement_result.json)를 겹쳐 그린다 - 트렁크 스캔의
       base_link 좌표계 하나로 통일(trunk_map/placement_result가 이미 같은 프레임).

실행: perception/.venv 안에서 실행해야 함 (numpy<2 고정, 시스템 numpy 2.x와 ABI 충돌).
    source perception/.venv/bin/activate && python3 95.plot_trunk_placement_holonomic.py
"""
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
_KR_FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
fm.fontManager.addfont(_KR_FONT_PATH)
matplotlib.rcParams["font.family"] = fm.FontProperties(fname=_KR_FONT_PATH).get_name()
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D

# ---------------- 팔레트 (37.py와 동일, dataviz 스킬 references/palette.md) ----------------
SURFACE = "#fcfcfb"
INK_PRIMARY = "#0b0b0b"
INK_SECONDARY = "#52514e"
INK_MUTED = "#898781"
GRID = "#e1e0d9"
BASELINE = "#c3c2b7"
BLUE = "#2a78d6"     # 박스(스캔/배치)
SAND = "#c98a3a"     # 장애물(휠하우스 등, 자동 검출)
CLOUD = "#7a93a8"    # 원본 point cloud 밀도(서브샘플)

SCRIPT_DIR = Path(__file__).resolve().parent
OUT_DIR = SCRIPT_DIR / "results" / "holonomic_base"

TRUNK_MAP_JSON = OUT_DIR / "trunk_map.json"
TRUNK_META_JSON = OUT_DIR / "trunk_pointcloud_meta.json"
TRUNK_CLOUD_NPY = OUT_DIR / "trunk_pointcloud.npy"
PLACEMENT_JSON = OUT_DIR / "placement_result.json"

_vision_dir = Path.home() / "box_pointcloud"
_vision_files = sorted(_vision_dir.glob("all_boxes_corners_*.json"))
if not _vision_files:
    raise SystemExit(f"[에러] {_vision_dir}에 all_boxes_corners_*.json이 없습니다.")
VISION_JSON = _vision_files[-1]

print("[사용 데이터 경로]")
print(f"  카트 박스 스캔(vision)   : {VISION_JSON}")
print(f"  트렁크 맵(장애물 포함)   : {TRUNK_MAP_JSON}")
print(f"  트렁크 원본 point cloud  : {TRUNK_CLOUD_NPY}")
print(f"  트렁크 스캔 메타(base)   : {TRUNK_META_JSON}")
print(f"  적재 알고리즘 결과       : {PLACEMENT_JSON}")

with open(VISION_JSON) as f:
    vision_data = json.load(f)
with open(TRUNK_MAP_JSON) as f:
    trunk_map = json.load(f)
with open(TRUNK_META_JSON) as f:
    trunk_meta = json.load(f)
with open(PLACEMENT_JSON) as f:
    placement_data = json.load(f)

print(f"\n[트렁크맵] obstacles={len(trunk_map.get('obstacles', []))}개 검출됨 "
      f"(90.export_trunk_map_holonomic.py의 grid 휴리스틱 자동 검출)")
for obs in trunk_map.get("obstacles", []):
    zs = [v[2] for v in obs["vertices"]]
    print(f"  - {obs['name']}: 높이(z)={min(zs):.3f}~{max(zs):.3f}m ({obs.get('note', '')[:30]}...)")


def quat_wxyz_to_matrix(q):
    w, x, y, z = q
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array([
        [1 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1 - (xx + yy)],
    ])


BASE_POS = np.asarray(trunk_meta["base_pos"], dtype=np.float64)
BASE_QUAT = np.asarray(trunk_meta["base_quat"], dtype=np.float64)
R_BASE = quat_wxyz_to_matrix(BASE_QUAT)

# 89.py가 저장한 원본 point cloud는 world 좌표다 - trunk_map/placement_result와 같은
# base_link 좌표계로 되돌린다(정확히 반대 변환: base_row = (world_row - base_pos) @ R_base,
# world_row = base_row @ R_base.T + base_pos의 역연산).
cloud_world = np.load(TRUNK_CLOUD_NPY)
cloud_base = (cloud_world - BASE_POS) @ R_BASE
print(f"\n[point cloud] 원본 {len(cloud_base)}점(world) -> base_link 좌표로 역변환 완료")
_rng = np.random.default_rng(0)
if len(cloud_base) > 60000:
    idx = _rng.choice(len(cloud_base), size=60000, replace=False)
    cloud_base = cloud_base[idx]
    print(f"  -> 렌더링용 6만 점으로 서브샘플")

fig, (ax_scan, ax_trunk) = plt.subplots(1, 2, figsize=(14, 7), facecolor=SURFACE)

# ================= 왼쪽: 카트 박스 스캔 결과 =================
ax_scan.set_facecolor(SURFACE)
for box in vision_data["boxes"]:
    box_id = str(box["box_id"])
    xs = [c[0] for c in box["corners_m"]]
    ys = [c[1] for c in box["corners_m"]]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    ax_scan.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor=BLUE, edgecolor=BLUE,
                                 alpha=0.35, linewidth=2.2, zorder=3))
    ax_scan.text((x0 + x1) / 2, (y0 + y1) / 2, f"박스{box_id}", ha="center", va="center",
                 fontsize=9, color=INK_PRIMARY, fontweight="bold")

ax_scan.set_title("① 카트 박스 스캔 (run_scan_batch.py)", fontsize=12, color=INK_PRIMARY, loc="left", pad=12)
ax_scan.set_xlabel(f"x [m] ({VISION_JSON.name}의 base_link 기준)", color=INK_SECONDARY, fontsize=8)
ax_scan.set_ylabel("y [m]", color=INK_SECONDARY, fontsize=9)
ax_scan.set_aspect("equal")
ax_scan.margins(0.3)
ax_scan.grid(True, color=GRID, linewidth=0.7)
ax_scan.set_axisbelow(True)
for spine in ax_scan.spines.values():
    spine.set_color(BASELINE)
ax_scan.tick_params(colors=INK_MUTED, labelsize=8)

# ================= 오른쪽: 트렁크 내부(base_link 좌표계) =================
ax_trunk.set_facecolor(SURFACE)

# 원본 point cloud 밀도 - 휠하우스 부근에 실제로 점이 몰려있는지(=진짜 돌출부인지) 눈으로
# 바로 확인할 수 있게 은은하게 깔아준다.
ax_trunk.scatter(cloud_base[:, 0], cloud_base[:, 1], s=0.4, color=CLOUD, alpha=0.12,
                  linewidths=0, zorder=1)

trunk_xs = [v[0] for v in trunk_map["vertices"]]
trunk_ys = [v[1] for v in trunk_map["vertices"]]
tx0, tx1 = min(trunk_xs), max(trunk_xs)
ty0, ty1 = min(trunk_ys), max(trunk_ys)
ax_trunk.add_patch(Rectangle((tx0, ty0), tx1 - tx0, ty1 - ty0, facecolor="none",
                              edgecolor=INK_PRIMARY, linewidth=2.0, zorder=4))
ax_trunk.text(tx0, ty1 + 0.03, "트렁크 AABB(전체 스캔 경계)", fontsize=8.5, color=INK_SECONDARY)

# 장애물 - 90.export_trunk_map_holonomic.py의 grid 휴리스틱 자동 검출(휠하우스/기존 물건
# 구분 없이 바닥 위 점유 공간을 그대로 AABB로 표시).
for obs in trunk_map.get("obstacles", []):
    oxs = [v[0] for v in obs["vertices"]]
    oys = [v[1] for v in obs["vertices"]]
    ox0, ox1 = min(oxs), max(oxs)
    oy0, oy1 = min(oys), max(oys)
    ax_trunk.add_patch(Rectangle((ox0, oy0), ox1 - ox0, oy1 - oy0, facecolor=SAND,
                                  edgecolor=SAND, alpha=0.45, hatch="///", linewidth=1.3, zorder=2))
    ax_trunk.text((ox0 + ox1) / 2, oy1 + 0.008, obs["name"], ha="center", va="bottom",
                  fontsize=7, color=INK_SECONDARY)

# algorism이 실제로 계산한 배치(rotated=False 가정 - 축 정렬 사각형. wrist_yaw 회전은
# 미세 조정용이라 여기선 반영 안 함, 37.py와 동일 단순화).
for p in placement_data["placements"]:
    box_id = str(p["box_id"])
    x, y, _ = p["position_base_frame"]
    w, d, _ = p["dimensions"]
    ax_trunk.add_patch(Rectangle((x, y), w, d, facecolor=BLUE, edgecolor=BLUE, alpha=0.4,
                                  linewidth=2.2, zorder=5))
    ax_trunk.text(x + w / 2, y + d / 2, f"박스{box_id}", ha="center", va="center", fontsize=8.5,
                  color=INK_PRIMARY, fontweight="bold", zorder=6)

unloadable = placement_data.get("unloadable", [])
if unloadable:
    reasons = ", ".join(f"박스{u['box_id']}({u['reason']})" for u in unloadable)
    ax_trunk.text(tx0, ty0 - 0.05, f"미적재: {reasons}", fontsize=8.5, color=INK_MUTED, ha="left", va="top")

ax_trunk.set_title("② 트렁크 내부 + 장애물(자동 검출) + 배치 결과", fontsize=12, color=INK_PRIMARY, loc="left", pad=12)
ax_trunk.set_xlabel("x [m] (트렁크 스캔 base_link 기준)", color=INK_SECONDARY, fontsize=9)
ax_trunk.set_ylabel("y [m]", color=INK_SECONDARY, fontsize=9)
ax_trunk.set_aspect("equal")
ax_trunk.margins(0.1)
ax_trunk.grid(True, color=GRID, linewidth=0.7)
ax_trunk.set_axisbelow(True)
for spine in ax_trunk.spines.values():
    spine.set_color(BASELINE)
ax_trunk.tick_params(colors=INK_MUTED, labelsize=8)

legend_handles = [
    Rectangle((0, 0), 1, 1, facecolor=BLUE, edgecolor=BLUE, alpha=0.4, linewidth=2.2, label="박스(스캔/배치)"),
    Rectangle((0, 0), 1, 1, facecolor=SAND, edgecolor=SAND, alpha=0.45, hatch="///", linewidth=1.3,
              label="장애물(휠하우스 등, grid 휴리스틱 자동 검출)"),
    Line2D([0], [0], marker="o", color="none", markerfacecolor=CLOUD, markeredgecolor="none",
           markersize=6, alpha=0.6, label="원본 스캔 point cloud(서브샘플)"),
]
legend = fig.legend(handles=legend_handles, loc="lower center", ncol=1, frameon=False,
                     fontsize=9, labelcolor=INK_SECONDARY, bbox_to_anchor=(0.5, -0.06))

suptitle = fig.suptitle("Cart2Trunk: 카트 박스 스캔 → 트렁크 스캔(장애물 자동검출) → 적재 알고리즘 배치",
                         fontsize=13, color=INK_PRIMARY, y=1.02, fontweight="bold")

out_path = SCRIPT_DIR / "_trunk_placement_flow.png"
fig.savefig(out_path, dpi=150, facecolor=SURFACE, bbox_inches="tight",
            bbox_extra_artists=[legend, suptitle])
print(f"\n[SAVED] {out_path}")
