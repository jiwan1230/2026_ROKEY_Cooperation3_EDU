"""37.plot_crate_placement.py
박스 스캔 -> 크레이트(트렁크) 스캔 -> 적재 알고리즘 배치까지 전체 흐름을 한 그림에.

results/crate_demo/{table_boxes_filtered,trunk_map,placement_result}.json 세 개를
읽어서 왼쪽엔 테이블 스캔 결과, 오른쪽엔 크레이트 내부(장애물로 등록된 더미 박스 2개 +
알고리즘이 실제로 계산한 배치)를 나란히 그린다. 36.py는 이제 배치에 성공한 박스
전부를 순서대로 옮기므로(예전엔 하나만 하드코딩해서 옮겼음), 기본적으로
placement_result.json의 모든 box_id를 "실행됨"으로 강조한다.

실행: perception/.venv 안에서 실행해야 함 (numpy<2 고정, 시스템 numpy 2.x와 ABI 충돌).
    source perception/.venv/bin/activate && python3 37.plot_crate_placement.py
"""
import json
import textwrap
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
BLUE = "#2a78d6"    # 카테고리 슬롯1 - 실제 실행된 박스(EXECUTED_IDS)
AQUA = "#1baf7a"    # 카테고리 슬롯3 - 알고리즘이 배치는 계산했지만 이번엔 안 옮긴 박스

SCRIPT_DIR = Path(__file__).resolve().parent
RUN_DIR = SCRIPT_DIR / "results" / "crate_demo"

TABLE_BOXES_JSON = RUN_DIR / "table_boxes_filtered.json"
TRUNK_MAP_JSON = RUN_DIR / "trunk_map.json"
PLACEMENT_JSON = RUN_DIR / "placement_result.json"

with open(TABLE_BOXES_JSON) as f:
    table_data = json.load(f)
with open(TRUNK_MAP_JSON) as f:
    trunk_map = json.load(f)
with open(PLACEMENT_JSON) as f:
    placement_data = json.load(f)

# 36.crate_pick_to_place.py는 placement_result.json에 배치 성공으로 나온 박스를
# 전부(하나가 아니라) 순서대로 집어서 옮긴다 - 기본값은 그 전부를 "실행됨"으로 표시.
# 특정 실행에서 일부만 흡착 실패 등으로 건너뛰었다면 이 set을 수동으로 좁혀서
# 실제 결과를 반영할 수 있다.
EXECUTED_IDS = {str(p["box_id"]) for p in placement_data["placements"]}

offset = placement_data["trunk_offset_base_frame"]


def to_local_xy(vertices):
    """base_link 좌표 -> 알고리즘 내부 로컬 좌표(트렁크 min corner가 원점) - 순수 평행이동
    (02_trunk_space_state.py의 local_to_base_frame과 정확히 반대 방향)."""
    xs = [v[0] - offset[0] for v in vertices]
    ys = [v[1] - offset[1] for v in vertices]
    return min(xs), max(xs), min(ys), max(ys)


fig, (ax_scan, ax_trunk) = plt.subplots(1, 2, figsize=(13, 6.5), facecolor=SURFACE)

# ================= 왼쪽: 테이블 스캔 결과 (base_link 기준, x-y 평면) =================
ax_scan.set_facecolor(SURFACE)
for box in table_data["boxes"]:
    box_id = str(box["box_id"])
    xs = [c[0] for c in box["corners_m"]]
    ys = [c[1] for c in box["corners_m"]]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    executed = box_id in EXECUTED_IDS
    style = dict(facecolor=BLUE, edgecolor=BLUE, alpha=0.35, linewidth=2.2) if executed \
        else dict(facecolor=AQUA, edgecolor=AQUA, alpha=0.30, linewidth=1.6)
    ax_scan.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, **style, zorder=3))
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    label = f"박스{box_id}" + (" (실행됨)" if executed else "")
    ax_scan.text(cx, cy, label, ha="center", va="center", fontsize=9,
                 color=INK_PRIMARY, fontweight="bold" if executed else "normal")

ax_scan.set_title("① 테이블 박스 스캔 (box_top_extractor.py)", fontsize=12, color=INK_PRIMARY, loc="left", pad=12)
ax_scan.set_xlabel("x [m] (base_link 기준)", color=INK_SECONDARY, fontsize=9)
ax_scan.set_ylabel("y [m]", color=INK_SECONDARY, fontsize=9)
ax_scan.set_aspect("equal")
ax_scan.margins(0.2)
ax_scan.grid(True, color=GRID, linewidth=0.7)
ax_scan.set_axisbelow(True)
for spine in ax_scan.spines.values():
    spine.set_color(BASELINE)
ax_scan.tick_params(colors=INK_MUTED, labelsize=8)

# ================= 오른쪽: 크레이트 내부 (알고리즘 로컬 좌표 - 트렁크 min corner가 원점) =================
ax_trunk.set_facecolor(SURFACE)
trunk_w = placement_data["trunk_local"]["width"]
trunk_d = placement_data["trunk_local"]["depth"]
ax_trunk.add_patch(Rectangle((0, 0), trunk_w, trunk_d, facecolor="none",
                              edgecolor=INK_PRIMARY, linewidth=2.0, zorder=4))
ax_trunk.text(0.0, trunk_d + 0.03, "크레이트 내부 경계", fontsize=9, color=INK_SECONDARY)

# 장애물(더미 박스, 실제로 크레이트 안을 스캔해서 얻음) - 알고리즘이 이 자리를 피해서 계산했다.
for obs in trunk_map["obstacles"]:
    x0, x1, y0, y1 = to_local_xy(obs["vertices"])
    ax_trunk.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor="none",
                                  edgecolor=INK_MUTED, hatch="///", linewidth=1.3, zorder=2))
    # 더미 박스 자체가 작아서(≈0.05x0.12) 라벨을 안에 넣으면 겹친다 - 오른쪽에 두면 박스2
    # 라벨과 겹치는 경우가 있어서, 트렁크 왼쪽 벽 쪽 여유 공간에 왼쪽 정렬로 붙인다.
    ax_trunk.text(x0 - 0.015, (y0 + y1) / 2, "장애물", ha="right", va="center",
                  fontsize=7.5, color=INK_SECONDARY)

# 알고리즘이 실제로 계산한 배치 - 실행된 박스만 강조.
for p in placement_data["placements"]:
    box_id = str(p["box_id"])
    x, y, _ = p["position_local"]
    w, d, _ = p["dimensions"]
    executed = box_id in EXECUTED_IDS
    style = dict(facecolor=BLUE, edgecolor=BLUE, alpha=0.35, linewidth=2.2) if executed \
        else dict(facecolor=AQUA, edgecolor=AQUA, alpha=0.30, linewidth=1.6)
    ax_trunk.add_patch(Rectangle((x, y), w, d, **style, zorder=3))
    label = f"박스{box_id}" + (" (실행됨)" if executed else "")
    ax_trunk.text(x + w / 2, y + d / 2, label, ha="center", va="center", fontsize=8.5,
                  color=INK_PRIMARY, fontweight="bold" if executed else "normal")

unloadable = placement_data.get("unloadable", [])
_unloadable_n_lines = 1
if unloadable:
    reasons = ", ".join(f"박스{u['box_id']}({u['reason']})" for u in unloadable)
    # 박스 개수가 늘면(44.py) 미적재 목록도 같이 길어져서 한 줄로는 이미지 폭을 넘어
    # 잘릴 수 있다(실측 확인) - 고정 너비로 줄바꿈해서 항상 캔버스 안에 들어오게 한다.
    wrapped = textwrap.fill(f"미적재: {reasons}", width=55)
    _unloadable_n_lines = wrapped.count("\n") + 1
    # "크레이트 내부 경계" 라벨과 같은 y에 두면 겹친다 - 그 위로 한 줄 더 띄운다.
    # va="bottom"이라 줄이 늘어날수록 텍스트 블록이 위로 자라므로, 아래 ylim도
    # _unloadable_n_lines에 맞춰 같이 늘려야 한다.
    ax_trunk.text(0.02, trunk_d + 0.09, wrapped, fontsize=8.5, color=INK_MUTED, ha="left", va="bottom")

ax_trunk.set_title("② 적재 알고리즘 배치 결과 (14_run_full_pipeline.py)", fontsize=12, color=INK_PRIMARY, loc="left", pad=12)
ax_trunk.set_xlabel("x [m] (트렁크 로컬 - min corner가 원점)", color=INK_SECONDARY, fontsize=9)
ax_trunk.set_ylabel("y [m]", color=INK_SECONDARY, fontsize=9)
ax_trunk.set_aspect("equal")
ax_trunk.set_xlim(-0.08, trunk_w + 0.08)
ax_trunk.set_ylim(-0.08, trunk_d + 0.15 + 0.05 * (_unloadable_n_lines - 1))
ax_trunk.grid(True, color=GRID, linewidth=0.7)
ax_trunk.set_axisbelow(True)
for spine in ax_trunk.spines.values():
    spine.set_color(BASELINE)
ax_trunk.tick_params(colors=INK_MUTED, labelsize=8)

# ================= 범례 (양쪽 공용) =================
legend_handles = [
    Rectangle((0, 0), 1, 1, facecolor=BLUE, edgecolor=BLUE, alpha=0.35, linewidth=2.2,
              label="실행됨 (36.py가 실제로 집어서 옮김)"),
    Rectangle((0, 0), 1, 1, facecolor=AQUA, edgecolor=AQUA, alpha=0.30, linewidth=1.6,
              label="알고리즘이 배치는 계산했지만 이번엔 안 옮긴 박스"),
    Rectangle((0, 0), 1, 1, facecolor="none", edgecolor=INK_MUTED, hatch="///", linewidth=1.3,
              label="장애물(더미 박스, 실측 스캔) - 알고리즘이 피해서 배치"),
]
legend = fig.legend(handles=legend_handles, loc="lower center", ncol=1, frameon=False,
                     fontsize=9, labelcolor=INK_SECONDARY, bbox_to_anchor=(0.5, -0.08))

suptitle = fig.suptitle(f"Cart2Trunk: 박스 스캔 → 크레이트 스캔(장애물) → 적재 알고리즘 배치 → 실제 실행({len(EXECUTED_IDS)}개 전부)",
                        fontsize=13, color=INK_PRIMARY, y=1.02, fontweight="bold")

# tight_layout()과 savefig(bbox_inches="tight")를 같이 쓰면 서로 다른 여백 계산이
# 충돌한다 - 특히 "미적재" 목록/범례처럼 실행마다 길이가 달라지는 축 밖 텍스트가 있으면
# 그 크기 차이에 따라 두 서브플롯이 겹쳐 보이는 등 예측 불가능하게 깨질 수 있다(실측
# 확인: 박스 개수가 늘어 미적재 목록이 길어진 실행에서 실제로 깨짐). tight_layout()은
# 빼고, 축 밖에 있는 legend/suptitle을 bbox_extra_artists로 명시해서 savefig의
# bbox_inches="tight" 한 번으로만 전체 여백을 계산하게 한다.
out_path = SCRIPT_DIR / "_crate_05_placement_flow.png"
fig.savefig(out_path, dpi=150, facecolor=SURFACE, bbox_inches="tight",
            bbox_extra_artists=[legend, suptitle])
print(f"[SAVED] {out_path}")

# ================= 이번 실행 결과를 날짜별 폴더에 보관 (35/36.py와 동일) =================
import shutil
from datetime import datetime

_run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
_archive_dir = RUN_DIR / "runs" / f"{_run_stamp}_37plot"
_archive_dir.mkdir(parents=True, exist_ok=True)
shutil.copy2(out_path, _archive_dir / f"{_run_stamp}_{out_path.name}")
shutil.copy2(PLACEMENT_JSON, _archive_dir / f"{_run_stamp}_{PLACEMENT_JSON.name}")
print(f"[보관] {_archive_dir} 에 2개 파일 복사", flush=True)
