"""
rotation_angle_analysis.py
"⑱ 회전을 90도가 아니라 임의 각도로도 할 수 있는가?"에 대한 답을 실제 계산 +
그림으로 검증한다.

[결론 먼저]
불가능한 게 아니라 "해도 손해"다. 사각형 박스를 사각형 트렁크에 넣을 때는
0도/90도가 항상 최적이고, 그 사이 각도로 돌리면 박스를 감싸는 축정렬
바운딩박스(실제로 다른 박스/벽과 충돌 계산에 쓰이는 영역)가 항상 더 커져서
공간을 더 낭비한다 - 45도에서 최대로 나쁘고(최대 2배 가까이), 45도를 넘어가면
다시 줄어들어 90도에서 원래 크기로 돌아온다. 즉 0/90만 시도하는 지금 방식이
이미 "낭비가 없는" 유이한 두 각도를 정확히 고르고 있다 - 임의 각도를 추가로
시도하게 만드는 건 코드도 훨씬 복잡해지고(축정렬 사각형 겹침 판정 -> 회전된
사각형 겹침 판정으로 충돌 검사 엔진 전체를 다시 짜야 함) 결과도 더 나빠지는,
들이는 노력 대비 얻는 게 없는 방향이다.

[왜 이런 일이 생기는가 - 기하학]
박스(가로 W, 세로 D)를 각도 θ만큼 돌리면, 그 박스를 딱 감싸는 축정렬
바운딩박스의 가로/세로는:
    bbox_width  = W*cos(θ) + D*sin(θ)
    bbox_depth  = W*sin(θ) + D*cos(θ)
θ=0(또는 90)일 때는 이 값이 정확히 W,D(또는 D,W)로, 낭비가 0이다. 그 사이
각도에서는 cos+sin 조합이 항상 1보다 커져서 바운딩박스가 원래 박스보다
커진다 - 네 귀퉁이에 삼각형 모양의 "죽은 공간"이 생기는 것과 같다. 트렁크
안 충돌/겹침 판정(④)은 전부 이 축정렬 바운딩박스 기준으로 하기 때문에, 이
낭비는 실제로 다른 박스가 못 들어가는 진짜 손해로 이어진다.
"""

import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
_NOTO_CJK = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
fm.fontManager.addfont(_NOTO_CJK)
matplotlib.rcParams["font.family"] = fm.FontProperties(fname=_NOTO_CJK).get_name()
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np


def bounding_footprint(width: float, depth: float, angle_deg: float):
    theta = math.radians(angle_deg)
    bbox_w = width * abs(math.cos(theta)) + depth * abs(math.sin(theta))
    bbox_d = width * abs(math.sin(theta)) + depth * abs(math.cos(theta))
    return bbox_w, bbox_d


def rotated_corners(width: float, depth: float, angle_deg: float, cx: float, cy: float):
    """박스 중심(cx,cy) 기준으로 각도만큼 돌린 4개 꼭짓점 좌표."""
    theta = math.radians(angle_deg)
    hw, hd = width / 2, depth / 2
    local = [(-hw, -hd), (hw, -hd), (hw, hd), (-hw, hd)]
    return [
        (cx + lx * math.cos(theta) - ly * math.sin(theta),
         cy + lx * math.sin(theta) + ly * math.cos(theta))
        for lx, ly in local
    ]


BOX_W, BOX_D = 0.40, 0.25  # 실제 데모에서 자주 쓰던 비대칭 박스 크기와 비슷하게

fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))

# ---- 왼쪽: 낭비 면적 vs 회전각 그래프 ----
angles = np.linspace(0, 90, 181)
areas = [bounding_footprint(BOX_W, BOX_D, a)[0] * bounding_footprint(BOX_W, BOX_D, a)[1] for a in angles]
original_area = BOX_W * BOX_D
waste_pct = [(a / original_area - 1) * 100 for a in areas]

ax = axes[0]
ax.plot(angles, waste_pct, color="#c0392b", linewidth=2.5)
ax.axvline(0, color="#27ae60", linestyle="--", alpha=0.6)
ax.axvline(90, color="#27ae60", linestyle="--", alpha=0.6)
ax.axvline(45, color="#7f8c8d", linestyle=":", alpha=0.6)
ax.scatter([0, 90], [0, 0], color="#27ae60", s=80, zorder=5, label="0°/90° (낭비 0%, 지금 방식)")
peak_idx = int(np.argmax(waste_pct))
ax.scatter([angles[peak_idx]], [waste_pct[peak_idx]], color="#c0392b", s=80, zorder=5,
           label=f"{angles[peak_idx]:.0f}° (낭비 최대 +{waste_pct[peak_idx]:.0f}%)")
ax.set_xlabel("회전 각도 (도)")
ax.set_ylabel("바운딩박스 낭비 면적 (%)")
ax.set_title(f"박스 {BOX_W}m x {BOX_D}m를 돌렸을 때\n충돌판정용 바운딩박스가 커지는 정도")
ax.legend(loc="upper center", fontsize=9)
ax.grid(alpha=0.3)

# ---- 오른쪽: 0°(정자세) vs 30° 회전 - 실제 낭비 공간 시각화 ----
ax2 = axes[1]
trunk_w, trunk_d = 1.0, 0.6
ax2.add_patch(patches.Rectangle((0, 0), trunk_w, trunk_d, fill=False, edgecolor="black", linewidth=2))

# 정자세 박스 (왼쪽에 배치, 여백 없이 딱 붙임 - 실제 낭비 0)
ax2.add_patch(patches.Rectangle((0.03, 0.03), BOX_W, BOX_D, facecolor="#2ecc71", alpha=0.6, edgecolor="#27ae60", linewidth=2))
ax2.text(0.03 + BOX_W / 2, 0.03 + BOX_D / 2, "0°\n(지금 방식)\n낭비 없음", ha="center", va="center", fontsize=9, weight="bold")

# 30도 회전 박스 (오른쪽에 배치) + 그 축정렬 바운딩박스(빨간 점선) 같이 표시
angle_demo = 30
cx, cy = 0.68, 0.28
corners = rotated_corners(BOX_W, BOX_D, angle_demo, cx, cy)
ax2.add_patch(patches.Polygon(corners, closed=True, facecolor="#e67e22", alpha=0.7, edgecolor="#d35400", linewidth=2))
bw, bd = bounding_footprint(BOX_W, BOX_D, angle_demo)
bbox_x, bbox_y = cx - bw / 2, cy - bd / 2
ax2.add_patch(patches.Rectangle((bbox_x, bbox_y), bw, bd, fill=False, edgecolor="#c0392b", linewidth=2, linestyle="--"))
waste_here = (bw * bd / original_area - 1) * 100
ax2.text(cx, cy - bd / 2 - 0.06, f"{angle_demo}° 회전\n(가상) 점선=실제 충돌판정 영역\n낭비 +{waste_here:.0f}%",
          ha="center", va="top", fontsize=9, weight="bold", color="#c0392b")

ax2.set_xlim(-0.05, trunk_w + 0.05)
ax2.set_ylim(-0.25, trunk_d + 0.05)
ax2.set_aspect("equal")
ax2.set_title("같은 박스, 같은 트렁크 단면\n30도로 돌리면 점선(=실제 못 쓰는 영역)이 훨씬 커짐")
ax2.axis("off")

plt.tight_layout()
out_path = "/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU/isaacpjt/Cart2Trunk/algorism/local_test_data/rotation_angle_analysis.png"
plt.savefig(out_path, dpi=130)
print(f"저장됨: {out_path}")
print(f"\n박스 {BOX_W}x{BOX_D}m 기준:")
print(f"  0도/90도: 낭비 0%")
print(f"  30도: 낭비 {(bounding_footprint(BOX_W,BOX_D,30)[0]*bounding_footprint(BOX_W,BOX_D,30)[1]/original_area-1)*100:.1f}%")
print(f"  45도(최악): 낭비 {(bounding_footprint(BOX_W,BOX_D,45)[0]*bounding_footprint(BOX_W,BOX_D,45)[1]/original_area-1)*100:.1f}%")
