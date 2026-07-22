"""박스 스캔 검출 결과(테이블) vs 적재 알고리즘이 계산한 트렁크 내부 배치를
나란히 보여주는 2패널 그림. box_id=0(Medium)이 실제로 pick&place를 실행한 박스
(33.box_table_pick_to_trunk.py). placement_result.json은 reproject_trunk_map.py로
재투영한 trunk_map + 실제 라이브 박스 스캔 JSON을 14_run_full_pipeline.py(algorism/)에
넣어 얻은 결과 - results/run_20260722_box_pick_place/에 고정 저장해둔 스냅샷을 그대로 쓴다.

실행: perception/.venv 안에서 실행해야 함 (numpy<2 고정, 시스템 numpy 2.x와 ABI 충돌).
    source perception/.venv/bin/activate && python3 34.plot_placement_comparison.py
"""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
_KR_FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
fm.fontManager.addfont(_KR_FONT_PATH)
matplotlib.rcParams["font.family"] = fm.FontProperties(fname=_KR_FONT_PATH).get_name()
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# ---------------- 팔레트 (dataviz 스킬 references/palette.md) ----------------
SURFACE = "#fcfcfb"
INK_PRIMARY = "#0b0b0b"
INK_SECONDARY = "#52514e"
INK_MUTED = "#898781"
GRID = "#e1e0d9"
BASELINE = "#c3c2b7"
BLUE = "#2a78d6"     # 카테고리 슬롯1 - 실행된 박스
AQUA = "#1baf7a"      # 카테고리 슬롯3 - 실측이지만 미실행

SCRIPT_DIR = Path(__file__).resolve().parent
RUN_DIR = SCRIPT_DIR / "results" / "run_20260722_box_pick_place"

BOX_JSON = "/home/rokey/box_pointcloud/all_boxes_corners_20260722_001235_552930.json"
PLACEMENT_JSON = RUN_DIR / "placement_result.json"

# box_id별 분류: 실행됨 / 실측(미실행) / 오탐·저신뢰
EXECUTED_ID = "0"
REAL_NOT_EXECUTED_IDS = {"2"}          # Small, 검출 높이(0.11m)가 실제 규격과 일치
LOW_CONFIDENCE_IDS = {"1", "3", "4"}   # 1: 높이 이상(0.03m), 3/4: 테이블 빈 면 오탐(높이 0.40m)

BOX_LABELS = {
    "0": "박스0 (Medium)",
    "1": "박스1 (저신뢰)",
    "2": "박스2 (Small)",
    "3": "박스3 (오탐)",
    "4": "박스4 (오탐)",
}


def classify(box_id):
    if box_id == EXECUTED_ID:
        return "executed"
    if box_id in REAL_NOT_EXECUTED_IDS:
        return "real"
    return "low_confidence"


STYLE = {
    "executed": dict(facecolor=BLUE, edgecolor=BLUE, alpha=0.35, linewidth=2.2, linestyle="-", zorder=3),
    "real": dict(facecolor=AQUA, edgecolor=AQUA, alpha=0.30, linewidth=1.6, linestyle="-", zorder=3),
    # 오탐/저신뢰는 뒤로 물러나 보이게: 채움 없이 얇은 점선 테두리만 (박스4는 테이블 전체를
    # 덮을 만큼 커서 실제 박스들과 겹치므로, 채우면 다른 박스를 가려버린다).
    "low_confidence": dict(facecolor="none", edgecolor=INK_MUTED, alpha=0.8, linewidth=1.1, linestyle="--", zorder=1),
}

# 큰 오탐 박스(4는 테이블 전체를 덮음)는 중앙 라벨이 다른 박스 라벨과 겹치므로, 모서리 쪽에
# 배치할 (box_id -> (x_frac, y_frac, ha, va)) 오버라이드. 기본은 사각형 중앙.
LABEL_POS_OVERRIDE = {
    "4": (0.03, 0.95, "left", "top"),
}


def label_position(box_id, x0, y0, x1, y1):
    if box_id in LABEL_POS_OVERRIDE:
        xf, yf, ha, va = LABEL_POS_OVERRIDE[box_id]
        return x0 + xf * (x1 - x0), y0 + yf * (y1 - y0), ha, va
    return (x0 + x1) / 2, (y0 + y1) / 2, "center", "center"

with open(BOX_JSON) as f:
    box_data = json.load(f)
with open(PLACEMENT_JSON) as f:
    placement_data = json.load(f)

fig, (ax_scan, ax_trunk) = plt.subplots(1, 2, figsize=(13, 6), facecolor=SURFACE)

# ================= 왼쪽: 테이블 위 스캔 검출 결과 (box-scan 세션 base_link, x-y 평면) =================
ax_scan.set_facecolor(SURFACE)
for box in box_data["boxes"]:
    box_id = str(box["box_id"])
    corners = box["corners_m"]
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    style = STYLE[classify(box_id)]
    ax_scan.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, **style))
    lx, ly, ha, va = label_position(box_id, x0, y0, x1, y1)
    ax_scan.text(lx, ly, BOX_LABELS[box_id], ha=ha, va=va,
                 fontsize=9, color=INK_PRIMARY, fontweight="bold" if box_id == EXECUTED_ID else "normal")

ax_scan.set_title("① 박스 스캔 검출 결과 (테이블, 5개)", fontsize=12, color=INK_PRIMARY, loc="left", pad=12)
ax_scan.set_xlabel("x [m] (base_link 기준)", color=INK_SECONDARY, fontsize=9)
ax_scan.set_ylabel("y [m]", color=INK_SECONDARY, fontsize=9)
ax_scan.set_aspect("equal")
ax_scan.margins(0.15)
ax_scan.grid(True, color=GRID, linewidth=0.7)
ax_scan.set_axisbelow(True)
for spine in ax_scan.spines.values():
    spine.set_color(BASELINE)
ax_scan.tick_params(colors=INK_MUTED, labelsize=8)

# ================= 오른쪽: 적재 알고리즘 배치 결과 (트렁크 내부, 로컬 좌표) =================
ax_trunk.set_facecolor(SURFACE)
trunk_w = placement_data["trunk_local"]["width"]
trunk_d = placement_data["trunk_local"]["depth"]
ax_trunk.add_patch(Rectangle((0, 0), trunk_w, trunk_d, facecolor="none",
                              edgecolor=INK_PRIMARY, linewidth=2.0, zorder=3))
ax_trunk.text(0.0, trunk_d + 0.03, "트렁크 내부 경계", fontsize=9, color=INK_SECONDARY)

for p in placement_data["placements"]:
    box_id = str(p["box_id"])
    x, y, _ = p["position_local"]
    w, d, _ = p["dimensions"]
    style = STYLE[classify(box_id)]
    ax_trunk.add_patch(Rectangle((x, y), w, d, **style))
    ax_trunk.text(x + w / 2, y + d / 2, BOX_LABELS[box_id], ha="center", va="center",
                  fontsize=9, color=INK_PRIMARY, fontweight="bold" if box_id == EXECUTED_ID else "normal")

unloadable = placement_data.get("unloadable", [])
if unloadable:
    reasons = ", ".join(f"박스{u['box_id']}({u['reason']})" for u in unloadable)
    # transAxes(-0.10)로 두면 x축 라벨과 같은 줄에 겹쳤다 - 축 프레임 안쪽 데이터
    # 좌표(트렁크 바닥선 y=0 바로 아래 여백)에 둬서 xlabel과 분리한다.
    ax_trunk.text(0.05, -0.15, f"미적재: {reasons}",
                  fontsize=8.5, color=INK_MUTED, ha="left")

ax_trunk.set_title("② 적재 알고리즘 추천 배치 (트렁크 내부)", fontsize=12, color=INK_PRIMARY, loc="left", pad=12)
ax_trunk.set_xlabel("x [m] (트렁크 로컬)", color=INK_SECONDARY, fontsize=9)
ax_trunk.set_ylabel("y [m]", color=INK_SECONDARY, fontsize=9)
ax_trunk.set_aspect("equal")
ax_trunk.set_xlim(-0.1, trunk_w + 0.1)
ax_trunk.set_ylim(-0.2, trunk_d + 0.15)
ax_trunk.grid(True, color=GRID, linewidth=0.7)
ax_trunk.set_axisbelow(True)
for spine in ax_trunk.spines.values():
    spine.set_color(BASELINE)
ax_trunk.tick_params(colors=INK_MUTED, labelsize=8)

# ================= 범례 (양쪽 공용) =================
legend_handles = [
    Rectangle((0, 0), 1, 1, facecolor=BLUE, edgecolor=BLUE, alpha=0.35, linewidth=2.2, label="실행됨 - box_id=0 (Medium)"),
    Rectangle((0, 0), 1, 1, facecolor=AQUA, edgecolor=AQUA, alpha=0.30, linewidth=1.6, label="실측 박스, 미실행 - box_id=2 (Small)"),
    Rectangle((0, 0), 1, 1, facecolor="none", edgecolor=INK_MUTED, linewidth=1.1, linestyle="--", label="오탐/저신뢰 (배치 대상 아님)"),
]
fig.legend(handles=legend_handles, loc="lower center", ncol=3, frameon=False,
           fontsize=9, labelcolor=INK_SECONDARY, bbox_to_anchor=(0.5, -0.02))

fig.suptitle("Cart2Trunk: 박스 스캔 → 적재 알고리즘 배치 → 실제 실행(box_id=0)",
             fontsize=13, color=INK_PRIMARY, y=1.02, fontweight="bold")

fig.tight_layout(rect=[0, 0.04, 1, 1])
out_path = SCRIPT_DIR / "_pick_to_trunk_05_placement_comparison.png"
fig.savefig(out_path, dpi=150, facecolor=SURFACE, bbox_inches="tight")
print(f"[SAVED] {out_path}")
