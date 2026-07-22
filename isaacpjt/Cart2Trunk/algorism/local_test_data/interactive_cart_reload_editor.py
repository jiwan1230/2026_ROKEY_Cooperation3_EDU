"""
interactive_cart_reload_editor.py
카트 재적재 시나리오의 "기존 7개(고정)" 좌표를 직접 마우스/슬라이더로 조정하면서
3D+top-down으로 바로 확인할 수 있는 도구.

사용법: python3 interactive_cart_reload_editor.py
  1. 왼쪽 라디오버튼으로 조정할 박스를 고른다.
  2. X/Y/Z 슬라이더를 움직이면 3D(왼쪽)와 top-down(오른쪽)이 실시간으로 갱신된다.
  3. 다른 박스와 겹치면 그 박스 테두리가 빨간색으로 바뀌고 상단에 경고가 뜬다.
  4. 맨 아래 "이름/가로/세로/높이/색상"을 채우고 "박스 추가"를 누르면 새 박스가
     생겨서 (0,0,0)에 놓이고 라디오버튼에도 바로 추가된다. 색상은 비워두면
     자동으로 팔레트에서 하나 배정된다 (matplotlib이 이해하는 이름/hex 아무거나 가능,
     예: "orange", "#ff9800").
  5. "저장" 버튼을 누르면 현재 모든 박스(위치+치수+색상)를
     cart_reload_fixed_positions.json에 저장한다 - sketch_placement_test_cart_reload.py가
     이 파일이 있으면 자동으로 읽어서 기존 7개의 위치를 갱신한다 (직접 추가한 새
     박스는 이 편집기 안에서만 보이고, 메인 파이프라인 스크립트에는 반영되지 않음 -
     거긴 정해진 박스 7개 자리만 읽어가도록 만들어져 있음).
  6. "초기화" 버튼을 누르면 이 스크립트를 시작할 때의 기존 7개 좌표로 되돌린다
     (직접 추가한 박스는 삭제됨).
"""
import json
import sys
import pathlib
import types
from importlib import import_module

ALGORISM_DIR = pathlib.Path("/home/sunwook/cobot3_ws/src/2026_ROKEY_Cooperation3_EDU/isaacpjt/Cart2Trunk/algorism")
sys.path.insert(0, str(ALGORISM_DIR))

m02 = import_module("02_trunk_space_state")
m03 = import_module("03_extreme_point_candidates")
Trunk = m02.Trunk
Box = m03.Box
PlacedBox = m03.PlacedBox

# ---- mplot3d가 시스템 패키지와 충돌하는 환경 문제 우회 (기존 데모 스크립트와 동일) ----
_local_site = next(p for p in sys.path if p.endswith("site-packages") and "/.local/" in p)
_pkg = types.ModuleType("mpl_toolkits")
_pkg.__path__ = [str(pathlib.Path(_local_site) / "mpl_toolkits")]
sys.modules["mpl_toolkits"] = _pkg

import matplotlib
matplotlib.use("TkAgg")
matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.widgets import Slider, RadioButtons, Button, TextBox
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

TRUNK_WIDTH = 0.60
TRUNK_DEPTH = 0.73
TRUNK_HEIGHT = 0.50
OUT_JSON = ALGORISM_DIR / "local_test_data" / "cart_reload_fixed_positions.json"

trunk = Trunk(width=TRUNK_WIDTH, depth=TRUNK_DEPTH, height=TRUNK_HEIGHT)

WHEELS = [
    PlacedBox(box=Box("Wheel_Front", width=0.16, depth=0.15, height=0.20), x=0.44, y=0.00, z=0.0),
    PlacedBox(box=Box("Wheel_Rear", width=0.16, depth=0.21, height=0.20), x=0.44, y=0.52, z=0.0),
]

# 조정 대상 - 기존 7개(고정으로 취급되는 박스들). 여기 좌표가 조정용 시작값.
BOXES = [
    Box("G_Tall", width=0.08, depth=0.17, height=0.15),
    Box("G_SmallTop", width=0.07, depth=0.05, height=0.15),
    Box("G_SmallRight", width=0.07, depth=0.06, height=0.15),
    Box("G_Wide", width=0.30, depth=0.07, height=0.15),
    Box("G_Mid", width=0.14, depth=0.16, height=0.15),
    Box("B_Mid", width=0.10, depth=0.09, height=0.12),
    Box("B_Small", width=0.07, depth=0.07, height=0.12),
]
BOX_BY_ID = {b.id: b for b in BOXES}

INITIAL_POSITIONS = {
    "G_Tall": [0.03, 0.03, 0.000],
    "G_SmallTop": [0.14, 0.05, 0.000],
    "G_SmallRight": [0.14, 0.12, 0.000],
    "G_Wide": [0.03, 0.23, 0.000],
    "G_Mid": [0.44, 0.26, 0.000],
    "B_Mid": [0.44, 0.26, 0.150],
    "B_Small": [0.44, 0.35, 0.150],
}
positions = {k: list(v) for k, v in INITIAL_POSITIONS.items()}

GREEN_SHADES = ["#1b5e20", "#2e7d32", "#43a047", "#66bb6a", "#81c784"]
BLUE_SHADES = ["#0d47a1", "#1e88e5"]
_green_ids = [b.id for b in BOXES if b.id.startswith("G_")]
_blue_ids = [b.id for b in BOXES if b.id.startswith("B_")]
COLORS = {bid: GREEN_SHADES[i % len(GREEN_SHADES)] for i, bid in enumerate(_green_ids)}
COLORS.update({bid: BLUE_SHADES[i % len(BLUE_SHADES)] for i, bid in enumerate(_blue_ids)})

# 새 박스를 추가할 때 색상을 안 적으면 여기서 순서대로 하나씩 자동 배정한다.
AUTO_PALETTE = ["#e65100", "#6a1b9a", "#00838f", "#795548", "#ad1457", "#757575", "#f9a825", "#3949ab"]
_auto_color_idx = 0


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


def aabb_overlap(a, b):
    """a, b = (x, y, z, w, d, h). 세 축 전부 양의 길이로 겹쳐야 진짜 충돌."""
    ax0, ay0, az0, aw, ad, ah = a
    bx0, by0, bz0, bw, bd, bh = b
    ax1, ay1, az1 = ax0 + aw, ay0 + ad, az0 + ah
    bx1, by1, bz1 = bx0 + bw, by0 + bd, bz0 + bh
    return (ax0 < bx1 and ax1 > bx0) and (ay0 < by1 and ay1 > by0) and (az0 < bz1 and az1 > bz0)


def compute_overlaps():
    """겹치는 박스 id 쌍을 찾는다 (차 바퀴 포함)."""
    entries = []
    for b in BOXES:
        x, y, z = positions[b.id]
        entries.append((b.id, (x, y, z, b.width, b.depth, b.height)))
    for w in WHEELS:
        entries.append((w.box.id, (w.x, w.y, w.z, w.box.width, w.box.depth, w.box.height)))

    bad_ids = set()
    for i in range(len(entries)):
        for j in range(i + 1, len(entries)):
            id_a, box_a = entries[i]
            id_b, box_b = entries[j]
            if aabb_overlap(box_a, box_b):
                bad_ids.add(id_a)
                bad_ids.add(id_b)
    return bad_ids


def out_of_bounds(box_id):
    x, y, z = positions[box_id]
    b = BOX_BY_ID[box_id]
    return (x < -1e-9 or y < -1e-9 or z < -1e-9
            or x + b.width > TRUNK_WIDTH + 1e-9
            or y + b.depth > TRUNK_DEPTH + 1e-9
            or z + b.height > TRUNK_HEIGHT + 1e-9)


fig = plt.figure(figsize=(15, 9))
ax3d = fig.add_subplot(1, 2, 1, projection="3d")
ax2d = fig.add_subplot(1, 2, 2)
plt.subplots_adjust(bottom=0.40, left=0.05, right=0.98, top=0.92)

status_ax = fig.add_axes([0.05, 0.94, 0.9, 0.04])
status_ax.axis("off")
status_text = status_ax.text(0.5, 0.5, "", ha="center", va="center", fontsize=11, color="crimson")


def redraw():
    bad_ids = compute_overlaps()
    oob_ids = {b.id for b in BOXES if out_of_bounds(b.id)}

    warn_parts = []
    if bad_ids:
        warn_parts.append("겹침: " + ", ".join(sorted(bad_ids)))
    if oob_ids:
        warn_parts.append("트렁크 밖: " + ", ".join(sorted(oob_ids)))
    status_text.set_text(" / ".join(warn_parts) if warn_parts else "겹침/트렁크 이탈 없음")
    status_text.set_color("crimson" if warn_parts else "seagreen")

    ax3d.cla()
    ax3d.set_xlim(0, TRUNK_WIDTH)
    ax3d.set_ylim(0, TRUNK_DEPTH)
    ax3d.set_zlim(0, TRUNK_HEIGHT)
    ax3d.set_xlabel("x (m) - 로봇 접근 방향")
    ax3d.set_ylabel("y (m) - 좌우")
    ax3d.set_zlabel("z (m)")
    ax3d.set_title("3D 아이소메트릭 (드래그로 회전 가능)")
    ax3d.set_box_aspect((TRUNK_WIDTH, TRUNK_DEPTH, TRUNK_HEIGHT))

    ax2d.cla()
    ax2d.set_xlim(-0.02, TRUNK_WIDTH + 0.02)
    ax2d.set_ylim(-0.02, TRUNK_DEPTH + 0.02)
    ax2d.set_xlabel("x (m) - 로봇 접근 방향")
    ax2d.set_ylabel("y (m) - 좌우")
    ax2d.set_title("top-down")
    ax2d.set_aspect("equal")
    ax2d.add_patch(Rectangle((0, 0), TRUNK_WIDTH, TRUNK_DEPTH, fill=False, edgecolor="red", linewidth=2))

    for w in WHEELS:
        ax3d.add_collection3d(Poly3DCollection(
            cuboid_faces(w.x, w.y, w.z, w.box.width, w.box.depth, w.box.height),
            facecolor="black", edgecolor="black", alpha=0.6))
        ax2d.add_patch(Rectangle((w.x, w.y), w.box.width, w.box.depth, facecolor="black", alpha=0.6))

    selected_id = radio.value_selected
    for b in BOXES:
        x, y, z = positions[b.id]
        is_bad = b.id in bad_ids or b.id in oob_ids
        is_selected = b.id == selected_id
        edge = "red" if is_bad else ("yellow" if is_selected else COLORS[b.id])
        lw = 3.0 if (is_bad or is_selected) else 1.0

        ax3d.add_collection3d(Poly3DCollection(
            cuboid_faces(x, y, z, b.width, b.depth, b.height),
            facecolor=COLORS[b.id], edgecolor=edge, alpha=0.85, linewidths=lw))
        ax2d.add_patch(Rectangle((x, y), b.width, b.depth, facecolor=COLORS[b.id],
                                  edgecolor=edge, linewidth=lw, alpha=0.85))
        ax2d.text(x + b.width / 2, y + b.depth / 2, b.id, fontsize=7, ha="center", va="center",
                  color="white", weight="bold")

    fig.canvas.draw_idle()


# ---- 위젯: 박스 선택 라디오버튼 (새 박스 추가 시 다시 만들어야 해서 함수로 뺌) ----
radio_ax = fig.add_axes([0.03, 0.11, 0.13, 0.24])
radio_ax.set_title("박스 선택", fontsize=9)
radio = None  # rebuild_radio()가 채운다


def on_radio_change(label):
    global _updating_sliders
    _updating_sliders = True
    x, y, z = positions[label]
    slider_x.set_val(x)
    slider_y.set_val(y)
    slider_z.set_val(z)
    _updating_sliders = False
    redraw()


def rebuild_radio(active_id):
    """새 박스가 추가되면 라디오버튼 목록 자체를 다시 만들어야 한다(matplotlib에
    항목을 나중에 추가하는 공식 API가 없음)."""
    global radio
    radio_ax.clear()
    radio_ax.set_title("박스 선택", fontsize=9)
    labels = [b.id for b in BOXES]
    active_index = labels.index(active_id) if active_id in labels else 0
    radio = RadioButtons(radio_ax, labels, active=active_index)
    radio.on_clicked(on_radio_change)
    on_radio_change(labels[active_index])


# ---- 위젯: X/Y/Z 슬라이더 ----
slider_x_ax = fig.add_axes([0.22, 0.28, 0.55, 0.03])
slider_y_ax = fig.add_axes([0.22, 0.22, 0.55, 0.03])
slider_z_ax = fig.add_axes([0.22, 0.16, 0.55, 0.03])
slider_x = Slider(slider_x_ax, "X", 0.0, TRUNK_WIDTH, valinit=positions[BOXES[0].id][0])
slider_y = Slider(slider_y_ax, "Y", 0.0, TRUNK_DEPTH, valinit=positions[BOXES[0].id][1])
slider_z = Slider(slider_z_ax, "Z", 0.0, TRUNK_HEIGHT, valinit=positions[BOXES[0].id][2])

_updating_sliders = False  # 슬라이더 값을 코드에서 세팅할 때 콜백 재귀 방지용


def on_slider_change(_val):
    if _updating_sliders:
        return
    selected_id = radio.value_selected
    positions[selected_id] = [slider_x.val, slider_y.val, slider_z.val]
    redraw()


slider_x.on_changed(on_slider_change)
slider_y.on_changed(on_slider_change)
slider_z.on_changed(on_slider_change)

# ---- 위젯: 저장/초기화 버튼 ----
save_ax = fig.add_axes([0.82, 0.22, 0.12, 0.05])
save_button = Button(save_ax, "저장 (JSON)")

reset_ax = fig.add_axes([0.82, 0.15, 0.12, 0.05])
reset_button = Button(reset_ax, "초기화")


def on_save(_event):
    payload = {}
    for b in BOXES:
        x, y, z = positions[b.id]
        payload[b.id] = {
            "x": x, "y": y, "z": z,
            "width": b.width, "depth": b.depth, "height": b.height,
            "color": COLORS[b.id],
        }
    with open(OUT_JSON, "w") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    print(f"저장됨: {OUT_JSON}")
    for bid, (x, y, z) in positions.items():
        print(f"  {bid}: ({x:.3f}, {y:.3f}, {z:.3f})")
    status_text.set_text(f"저장됨 -> {OUT_JSON.name}")
    status_text.set_color("royalblue")
    fig.canvas.draw_idle()


def on_reset(_event):
    global positions, BOXES, BOX_BY_ID, COLORS, _auto_color_idx
    positions = {k: list(v) for k, v in INITIAL_POSITIONS.items()}
    BOXES = [Box(b.id, width=b.width, depth=b.depth, height=b.height) for b in _ORIGINAL_BOXES]
    BOX_BY_ID = {b.id: b for b in BOXES}
    COLORS = dict(_ORIGINAL_COLORS)
    _auto_color_idx = 0
    rebuild_radio(active_id=BOXES[0].id)


save_button.on_clicked(on_save)
reset_button.on_clicked(on_reset)

# ---- 위젯: 새 박스 추가 (맨 아래 줄) ----
_field_axes = {}


def _add_field(name, x, w, initial=""):
    label_ax = fig.add_axes([x, 0.075, w, 0.02])
    label_ax.axis("off")
    label_ax.text(0, 0.5, name, fontsize=8, va="center")
    box_ax = fig.add_axes([x, 0.02, w, 0.045])
    tb = TextBox(box_ax, "", initial=initial)
    _field_axes[name] = tb
    return tb


tb_id = _add_field("이름", 0.03, 0.14)
tb_w = _add_field("가로 W(m)", 0.19, 0.09, "0.10")
tb_d = _add_field("세로 D(m)", 0.29, 0.09, "0.10")
tb_h = _add_field("높이 H(m)", 0.39, 0.09, "0.10")
tb_color = _add_field("색상(비우면 자동)", 0.49, 0.15, "")

add_button_ax = fig.add_axes([0.66, 0.02, 0.12, 0.045])
add_button = Button(add_button_ax, "박스 추가")


def on_add_box(_event):
    global _auto_color_idx

    box_id = tb_id.text.strip()
    if not box_id:
        status_text.set_text("박스 추가 실패: 이름을 입력하세요")
        status_text.set_color("crimson")
        fig.canvas.draw_idle()
        return
    if box_id in BOX_BY_ID:
        status_text.set_text(f"박스 추가 실패: '{box_id}'는 이미 있는 이름입니다")
        status_text.set_color("crimson")
        fig.canvas.draw_idle()
        return

    try:
        w, d, h = float(tb_w.text), float(tb_d.text), float(tb_h.text)
        if w <= 0 or d <= 0 or h <= 0:
            raise ValueError("치수는 0보다 커야 함")
    except ValueError:
        status_text.set_text("박스 추가 실패: 가로/세로/높이를 양수로 입력하세요")
        status_text.set_color("crimson")
        fig.canvas.draw_idle()
        return

    color = tb_color.text.strip()
    if not color:
        color = AUTO_PALETTE[_auto_color_idx % len(AUTO_PALETTE)]
        _auto_color_idx += 1
    elif not mcolors.is_color_like(color):
        status_text.set_text(f"박스 추가 실패: 색상 '{color}'을 이해할 수 없습니다 (예: orange, #ff9800)")
        status_text.set_color("crimson")
        fig.canvas.draw_idle()
        return

    new_box = Box(box_id, width=w, depth=d, height=h)
    BOXES.append(new_box)
    BOX_BY_ID[box_id] = new_box
    COLORS[box_id] = color
    positions[box_id] = [0.0, 0.0, 0.0]

    rebuild_radio(active_id=box_id)
    tb_id.set_val("")
    status_text.set_text(f"'{box_id}' 추가됨 (색상={color}) - (0,0,0)에 놓였으니 슬라이더로 옮기세요")
    status_text.set_color("royalblue")


add_button.on_clicked(on_add_box)

_ORIGINAL_BOXES = list(BOXES)
_ORIGINAL_COLORS = dict(COLORS)

rebuild_radio(active_id=BOXES[0].id)
print("인터랙티브 편집기 시작 - 라디오버튼으로 박스를 고르고 슬라이더로 X/Y/Z를 조정하세요.")
print("맨 아래에서 새 박스를 이름/치수/색상과 함께 추가할 수 있습니다.")
print("저장 버튼을 누르면", OUT_JSON, "에 저장되고, 이 파일이 있으면 sketch_placement_test_cart_reload.py가 자동으로 읽습니다.")
plt.show()
