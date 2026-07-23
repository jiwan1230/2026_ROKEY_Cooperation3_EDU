"""
planner_gui.py
trunk_map_planner_node.py를 매번 터미널 명령어로 실행하는 대신, tkinter 창
하나로 "트렁크 스캔 파일 + 카트 박스(개수/프리셋/직접 편집) + 모드 + 마진"을
고르고 [실행] 누르면 비포/애프터 그림을 바로, 화면 크기에 맞춰 크게 볼 수
있게 만든 GUI.

ROS2 없이 파일 기반(--test-file과 같은 경로)으로만 동작한다 - trunk_map_
planner_node.py의 plan_from_trunk_map_data()/DEFAULT_MARGIN을 그대로 가져다
쓴다(로직 중복 없음, 이 GUI가 만드는 건 화면뿐).

[실행]
    cd isaacpjt/Cart2Trunk
    python3 planner_gui.py

[필요한 것] tkinter(보통 파이썬 기본 포함 - 없으면 `sudo apt install python3-tk`),
Pillow(`pip install Pillow` - 이미지 미리보기용). rclpy는 trunk_map_planner_
node.py를 import하기 위해 필요하지만 실제로 ROS2를 켜지는 않는다(이 GUI는
--test-file 경로만 씀).

[디자인 - 애플/iOS 느낌]
단일 파란 강조색(iOS 시스템 블루 #007AFF) + 옅은 회색 배경 + 여백 위주 구성,
알약(pill)/둥근 사각형 버튼과 세그먼트 컨트롤을 Canvas로 직접 그려서 tkinter
기본 위젯의 각진 느낌을 최대한 줄였다. 폰트는 Pretendard를 쓰고 싶었지만 이
컴퓨터엔 설치돼 있지 않아서(fc-list로 직접 확인함), 같은 계열 굵기 단계
(Regular/Medium/Bold 등)를 갖춘 Noto Sans CJK KR로 대체했다 - 두께로 위계를
만드는 원칙(제목=Bold, 라벨=Medium, 본문=Regular)은 그대로 유지.
"""

import json
import pathlib
import random
import sys
import tkinter as tk
from importlib import import_module
from tkinter import ttk, messagebox

from PIL import Image, ImageTk

_HERE = pathlib.Path(__file__).resolve().parent
_ALGORISM_DIR = _HERE / "algorism"
_LOCAL_TEST_DATA_DIR = _ALGORISM_DIR / "local_test_data"
for p in (str(_ALGORISM_DIR), str(_LOCAL_TEST_DATA_DIR), str(_HERE)):
    if p not in sys.path:
        sys.path.insert(0, p)

_planner = import_module("trunk_map_planner_node")
_viz = import_module("_viz_helpers")

plan_from_trunk_map_data = _planner.plan_from_trunk_map_data
_color_for_box_id = _planner._color_for_box_id
DEFAULT_MARGIN = _planner.DEFAULT_MARGIN
_DEFAULT_CART_BOXES = _planner._DEFAULT_CART_BOXES
SceneBox = _viz.SceneBox
draw_scene = _viz.draw_scene

_SRC_DIR = pathlib.Path("/home/sunwook/cobot3_ws/src")
_GUI_OUT_DIR = _LOCAL_TEST_DATA_DIR / "_gui_output"
_GUI_OUT_DIR.mkdir(exist_ok=True)


# ---------------------------------------------------------------------------
# 디자인 토큰 (iOS 느낌 - 단일 블루 강조색, 옅은 배경, 두께로 위계)
# ---------------------------------------------------------------------------

class Palette:
    canvas = "#F5F5F7"       # 앱 배경 (애플 "parchment")
    surface = "#FFFFFF"      # 카드 배경
    border = "#E5E5EA"       # 아주 옅은 구분선
    text_primary = "#1D1D1F"
    text_secondary = "#6E6E73"
    accent = "#007AFF"       # iOS 시스템 블루
    accent_pressed = "#0060DF"
    segment_bg = "#E9E9EB"
    success = "#34C759"
    danger = "#FF3B30"


_FONT_FAMILY = "Noto Sans CJK KR"  # Pretendard 미설치 - 굵기 단계가 있는 대체 폰트


class Font:
    title = (_FONT_FAMILY, 18, "bold")
    section = (_FONT_FAMILY, 11, "bold")
    label = (_FONT_FAMILY, 10)
    body = (_FONT_FAMILY, 10)
    button = (_FONT_FAMILY, 11, "bold")
    caption = (_FONT_FAMILY, 9)
    mono = ("monospace", 9)


def _rounded_rect_points(x1, y1, x2, y2, r):
    r = min(r, (x2 - x1) / 2, (y2 - y1) / 2)
    return [
        x1 + r, y1, x2 - r, y1, x2, y1, x2, y1 + r,
        x2, y2 - r, x2, y2, x2 - r, y2, x1 + r, y2,
        x1, y2, x1, y2 - r, x1, y1 + r, x1, y1,
    ]


class RoundedButton(tk.Canvas):
    """iOS 스타일 알약형 버튼 - ttk.Button은 각진 네이티브 테두리를 못 벗어나서
    Canvas에 직접 둥근 사각형+텍스트를 그리는 방식으로 대체."""

    def __init__(self, parent, text, command, bg=Palette.accent, fg="white",
                 font=Font.button, width=140, height=38, radius=19, **kwargs):
        super().__init__(parent, width=width, height=height, highlightthickness=0,
                          bg=parent["bg"], **kwargs)
        self._command = command
        self._bg, self._fg, self._text, self._font = bg, fg, text, font
        # 주의: self._w/self._h는 tkinter 내부에서 위젯 경로(pathname)로 이미 쓰는
        # 예약된 속성이라, 크기값을 거기 저장하면 내부 상태가 깨진다 - 다른 이름 사용.
        self._btn_w, self._btn_h, self._radius = width, height, radius
        self._draw(bg)
        self.bind("<Button-1>", self._on_press)
        self.bind("<ButtonRelease-1>", self._on_release)
        self.bind("<Enter>", lambda e: self.configure(cursor="hand2"))

    def _draw(self, fill):
        self.delete("all")
        self.create_polygon(_rounded_rect_points(1, 1, self._btn_w - 1, self._btn_h - 1, self._radius),
                             smooth=True, fill=fill, outline="")
        self.create_text(self._btn_w / 2, self._btn_h / 2, text=self._text, fill=self._fg, font=self._font)

    def _on_press(self, event):
        self._draw(Palette.accent_pressed if self._bg == Palette.accent else self._bg)

    def _on_release(self, event):
        self._draw(self._bg)
        if 0 <= event.x <= self._btn_w and 0 <= event.y <= self._btn_h:
            self._command()

    def set_enabled(self, enabled: bool):
        self.unbind("<Button-1>") if not enabled else None
        self._draw(self._bg if enabled else Palette.segment_bg)


class SegmentedControl(tk.Canvas):
    """iOS 세그먼트 컨트롤(둘 중 하나 고르는 알약형 토글) - 라디오버튼 대신."""

    def __init__(self, parent, options, variable, width=320, height=32, **kwargs):
        super().__init__(parent, width=width, height=height, highlightthickness=0,
                          bg=parent["bg"], **kwargs)
        self._segments = options  # [(label, value), ...]
        self._var = variable
        # self._w/self._h는 tkinter 내부 예약 속성이라 다른 이름 사용 (RoundedButton 참고)
        self._ctrl_w, self._ctrl_h = width, height
        self._seg_w = width / len(options)
        self.bind("<Button-1>", self._on_click)
        self._var.trace_add("write", lambda *_: self._draw())
        self._draw()

    def _draw(self):
        self.delete("all")
        self.create_polygon(_rounded_rect_points(0, 0, self._ctrl_w, self._ctrl_h, self._ctrl_h / 2),
                             smooth=True, fill=Palette.segment_bg, outline="")
        current = self._var.get()
        for i, (label, value) in enumerate(self._segments):
            x1, x2 = i * self._seg_w, (i + 1) * self._seg_w
            if value == current:
                pad = 3
                self.create_polygon(
                    _rounded_rect_points(x1 + pad, pad, x2 - pad, self._ctrl_h - pad, (self._ctrl_h - 2 * pad) / 2),
                    smooth=True, fill=Palette.surface, outline="",
                )
            fg = Palette.text_primary if value == current else Palette.text_secondary
            self.create_text((x1 + x2) / 2, self._ctrl_h / 2, text=label, fill=fg, font=Font.label)

    def _on_click(self, event):
        idx = min(int(event.x // self._seg_w), len(self._segments) - 1)
        self._var.set(self._segments[idx][1])


class Card(tk.Frame):
    """옅은 테두리 + 여백을 가진 카드형 컨테이너 (iOS의 "타일" 섹션 느낌)."""

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=Palette.surface, highlightbackground=Palette.border,
                          highlightthickness=1, bd=0, **kwargs)


# ---------------------------------------------------------------------------
# 데이터 탐색 헬퍼
# ---------------------------------------------------------------------------

def _discover_trunk_maps() -> list:
    """/home/sunwook/cobot3_ws/src/run_*/pointcloud/trunk_map.json 전부 찾기."""
    return sorted(_SRC_DIR.glob("run_*/pointcloud/trunk_map.json"))


def _discover_box_presets() -> dict:
    """local_test_data/example_cart_boxes_*.json 전부 찾아서 {표시이름: 경로} 딕셔너리로."""
    presets = {"기본값 (Large/Medium/Small)": None}
    for f in sorted(_LOCAL_TEST_DATA_DIR.glob("example_cart_boxes_*.json")):
        presets[f.stem.replace("example_cart_boxes_", "")] = f
    return presets


def _generate_random_boxes(count: int) -> list:
    """박스 개수만 정하면 임의의(그럴듯한 범위 안) 크기로 목록을 만들어준다."""
    rng = random.Random()
    boxes = []
    for i in range(count):
        w = round(rng.uniform(0.15, 0.45), 2)
        d = round(rng.uniform(0.15, 0.40), 2)
        h = round(rng.uniform(0.10, 0.30), 2)
        boxes.append({"id": f"Box{i + 1}", "width": w, "depth": d, "height": h})
    return boxes


# ---------------------------------------------------------------------------
# 메인 GUI
# ---------------------------------------------------------------------------

class PlannerGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Cart2Trunk — 적재 알고리즘 시뮬레이터")
        self.configure(bg=Palette.canvas)
        self.geometry("1700x1000")
        self.minsize(1100, 700)

        self._trunk_maps = _discover_trunk_maps()
        self._box_presets = _discover_box_presets()
        self._tk_images = {}       # PhotoImage 참조 유지용
        self._pil_originals = {}   # 원본 PIL 이미지 캐시 (창 크기 바뀔 때 재계산 없이 리사이즈만)
        self._resize_job = None

        self._build_header()
        self._build_controls()
        self._build_result_area()

        self.bind("<Configure>", self._on_window_resize)

    # ------------------------------------------------------------------ UI

    def _build_header(self):
        header = tk.Frame(self, bg=Palette.canvas)
        header.pack(side="top", fill="x", padx=28, pady=(22, 4))

        tk.Label(header, text="CART2TRUNK · PLANNER", font=Font.caption,
                 fg=Palette.accent, bg=Palette.canvas).pack(anchor="w")
        tk.Label(header, text="적재 알고리즘 비포/애프터 시뮬레이터", font=Font.title,
                 fg=Palette.text_primary, bg=Palette.canvas).pack(anchor="w", pady=(2, 0))

    def _build_controls(self):
        outer = tk.Frame(self, bg=Palette.canvas)
        outer.pack(side="top", fill="x", padx=28, pady=(14, 10))

        card = Card(outer)
        card.pack(fill="x")
        inner = tk.Frame(card, bg=Palette.surface, padx=20, pady=16)
        inner.pack(fill="x")

        # ---- 1행: 트렁크 스캔 파일 / 카트 박스 프리셋 / 박스 개수 자동생성 ----
        row1 = tk.Frame(inner, bg=Palette.surface)
        row1.pack(fill="x")

        self._field_label(row1, "트렁크 스캔 파일").grid(row=0, column=0, sticky="w")
        self.trunk_map_var = tk.StringVar()
        trunk_map_names = [str(p.parent.parent.name) for p in self._trunk_maps]
        self.trunk_map_combo = ttk.Combobox(row1, textvariable=self.trunk_map_var,
                                             values=trunk_map_names, width=26, state="readonly",
                                             font=Font.body)
        if trunk_map_names:
            self.trunk_map_combo.current(len(trunk_map_names) - 1)
        self.trunk_map_combo.grid(row=1, column=0, sticky="w", pady=(4, 0))

        self._field_label(row1, "카트 박스 프리셋").grid(row=0, column=1, sticky="w", padx=(28, 0))
        self.box_preset_var = tk.StringVar(value=next(iter(self._box_presets)))
        self.box_preset_combo = ttk.Combobox(row1, textvariable=self.box_preset_var,
                                              values=list(self._box_presets.keys()), width=24,
                                              state="readonly", font=Font.body)
        self.box_preset_combo.current(0)
        self.box_preset_combo.bind("<<ComboboxSelected>>", self._on_preset_selected)
        self.box_preset_combo.grid(row=1, column=1, sticky="w", padx=(28, 0), pady=(4, 0))

        self._field_label(row1, "박스 개수로 자동 생성").grid(row=0, column=2, sticky="w", padx=(28, 0))
        gen_frame = tk.Frame(row1, bg=Palette.surface)
        gen_frame.grid(row=1, column=2, sticky="w", padx=(28, 0), pady=(4, 0))
        self.box_count_var = tk.IntVar(value=6)
        self.box_count_spin = tk.Spinbox(gen_frame, from_=1, to=40, width=4,
                                          textvariable=self.box_count_var, font=Font.body,
                                          relief="solid", bd=1)
        self.box_count_spin.pack(side="left")
        RoundedButton(gen_frame, "자동 생성", self._on_generate_boxes,
                      bg=Palette.segment_bg, fg=Palette.text_primary,
                      width=100, height=30, radius=15).pack(side="left", padx=(10, 0))

        # ---- 2행: 적재 모드(세그먼트) / 마진 ----
        row2 = tk.Frame(inner, bg=Palette.surface)
        row2.pack(fill="x", pady=(16, 0))

        self._field_label(row2, "적재 모드").grid(row=0, column=0, sticky="w")
        self.mode_var = tk.StringVar(value="large_first")
        SegmentedControl(row2, [("큰 거 우선", "large_first"), ("개수 우선", "count_first")],
                          self.mode_var, width=240, height=34).grid(row=1, column=0, sticky="w", pady=(4, 0))

        self._field_label(row2, f"마진 (m · 기본 {DEFAULT_MARGIN})").grid(row=0, column=1, sticky="w", padx=(28, 0))
        self.margin_var = tk.StringVar(value="")
        margin_entry = tk.Entry(row2, textvariable=self.margin_var, width=10, font=Font.body,
                                 relief="solid", bd=1)
        margin_entry.grid(row=1, column=1, sticky="w", padx=(28, 0), pady=(4, 0), ipady=3)

        self.run_button = RoundedButton(row2, "▶  실행", self._run, width=140, height=38)
        self.run_button.grid(row=1, column=2, sticky="w", padx=(28, 0), pady=(4, 0))

        self.status_var = tk.StringVar(value="준비됨")
        tk.Label(row2, textvariable=self.status_var, font=Font.caption,
                 fg=Palette.text_secondary, bg=Palette.surface).grid(
            row=1, column=3, sticky="w", padx=(16, 0), pady=(4, 0)
        )

        # ---- 3행: 박스 목록 JSON (접이식 느낌으로 작게, 필요할 때만 손으로 수정) ----
        row3 = tk.Frame(inner, bg=Palette.surface)
        row3.pack(fill="x", pady=(16, 0))
        self._field_label(row3, "박스 목록 (JSON · 직접 수정 가능)").pack(anchor="w")
        self.box_text = tk.Text(row3, height=4, font=Font.mono, relief="solid", bd=1,
                                 wrap="none", padx=8, pady=6)
        self.box_text.pack(fill="x", pady=(4, 0))
        self._on_preset_selected()

    def _field_label(self, parent, text):
        return tk.Label(parent, text=text, font=Font.section, fg=Palette.text_secondary, bg=Palette.surface)

    def _build_result_area(self):
        outer = tk.Frame(self, bg=Palette.canvas)
        outer.pack(side="top", fill="both", expand=True, padx=28, pady=(0, 10))
        outer.columnconfigure(0, weight=1)
        outer.columnconfigure(1, weight=1)
        outer.rowconfigure(0, weight=1)

        self.before_card = Card(outer)
        self.before_card.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        tk.Label(self.before_card, text="BEFORE · 아직 안 실음", font=Font.section,
                 fg=Palette.text_secondary, bg=Palette.surface).pack(anchor="w", padx=16, pady=(14, 6))
        self.before_label = tk.Label(self.before_card, bg=Palette.surface)
        self.before_label.pack(fill="both", expand=True, padx=16, pady=(0, 16))

        self.after_card = Card(outer)
        self.after_card.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        tk.Label(self.after_card, text="AFTER · 배치 결과", font=Font.section,
                 fg=Palette.accent, bg=Palette.surface).pack(anchor="w", padx=16, pady=(14, 6))
        self.after_label = tk.Label(self.after_card, bg=Palette.surface)
        self.after_label.pack(fill="both", expand=True, padx=16, pady=(0, 16))

        log_outer = tk.Frame(self, bg=Palette.canvas)
        log_outer.pack(side="bottom", fill="x", padx=28, pady=(0, 22))
        log_card = Card(log_outer)
        log_card.pack(fill="x")
        tk.Label(log_card, text="결과 로그", font=Font.section, fg=Palette.text_secondary,
                 bg=Palette.surface).pack(anchor="w", padx=16, pady=(12, 4))
        self.log_text = tk.Text(log_card, height=7, font=Font.mono, relief="flat",
                                 bg=Palette.surface, padx=16, pady=4)
        self.log_text.pack(fill="x", padx=4, pady=(0, 12))

    # -------------------------------------------------------------- 동작

    def _on_preset_selected(self, event=None):
        preset_path = self._box_presets[self.box_preset_var.get()]
        boxes = json.loads(preset_path.read_text()) if preset_path else _DEFAULT_CART_BOXES
        self._set_box_json(boxes)

    def _on_generate_boxes(self):
        boxes = _generate_random_boxes(self.box_count_var.get())
        self._set_box_json(boxes)
        self.status_var.set(f"박스 {len(boxes)}개 자동 생성됨 - [실행]을 눌러 확인하세요")

    def _set_box_json(self, boxes):
        self.box_text.delete("1.0", "end")
        self.box_text.insert("1.0", json.dumps(boxes, ensure_ascii=False, indent=2))

    def _run(self):
        try:
            self._run_impl()
        except Exception as e:
            messagebox.showerror("오류", f"{type(e).__name__}: {e}")
            self.status_var.set(f"오류: {e}")

    def _run_impl(self):
        if not self._trunk_maps:
            messagebox.showerror("오류", f"{_SRC_DIR} 밑에서 run_*/pointcloud/trunk_map.json을 못 찾음")
            return

        run_name = self.trunk_map_var.get()
        trunk_map_path = next(p for p in self._trunk_maps if p.parent.parent.name == run_name)
        data = json.loads(trunk_map_path.read_text())

        cart_boxes_raw = json.loads(self.box_text.get("1.0", "end"))
        mode = self.mode_var.get()
        margin_str = self.margin_var.get().strip()
        margin = float(margin_str) if margin_str else None

        self.status_var.set("계산 중...")
        self.update_idletasks()

        plans, unloadable, trunk, obstacles = plan_from_trunk_map_data(
            data, cart_boxes_raw, mode=mode, margin=margin
        )
        effective_margin = margin if margin is not None else DEFAULT_MARGIN

        box_by_id = {b["id"]: b for b in cart_boxes_raw}
        fixed_obstacles = [
            SceneBox(o.box.id, o.x, o.y, o.z, o.box.width, o.box.depth, o.box.height, "#7f8c8d")
            for o in obstacles
        ]

        # ---- Before: 아무 것도 안 놓인 상태, 카트 박스 전부 대기 중 ----
        before_path = _GUI_OUT_DIR / "before.png"
        draw_scene(
            trunk.width, trunk.depth, trunk.height,
            fixed_obstacles=fixed_obstacles, placed_boxes=[],
            waiting_boxes=[
                SceneBox(b["id"], 0, 0, 0, b["width"], b["depth"], b["height"], _color_for_box_id(b["id"]))
                for b in cart_boxes_raw
            ],
            title=f"Before - run={run_name} ({len(cart_boxes_raw)}개 대기 중)",
            out_path=str(before_path),
        )

        # ---- After: 실제 배치 결과 ----
        after_path = _GUI_OUT_DIR / "after.png"
        draw_scene(
            trunk.width, trunk.depth, trunk.height,
            fixed_obstacles=fixed_obstacles,
            placed_boxes=[
                SceneBox(p.box_id, p.position[0], p.position[1], p.position[2],
                         p.dimensions[0], p.dimensions[1], p.dimensions[2], _color_for_box_id(p.box_id))
                for p in plans
            ],
            waiting_boxes=[
                SceneBox(u.box_id, 0, 0, 0, box_by_id[u.box_id]["width"], box_by_id[u.box_id]["depth"],
                          box_by_id[u.box_id]["height"], _color_for_box_id(u.box_id))
                for u in unloadable
            ],
            title=f"After - mode={mode}, margin={effective_margin:.2f}m ({len(plans)}/{len(cart_boxes_raw)}개 적재)",
            out_path=str(after_path),
        )

        self._pil_originals["before"] = Image.open(before_path).copy()
        self._pil_originals["after"] = Image.open(after_path).copy()
        self._render_images()

        log_lines = [f"[{run_name}] mode={mode}, margin={effective_margin:.2f}m -> {len(plans)}/{len(cart_boxes_raw)}개 배치"]
        for p in plans:
            log_lines.append(f"  PLACED {p.box_id}: pos=({p.position[0]:.2f},{p.position[1]:.2f},{p.position[2]:.2f}) rotated={p.rotated}")
        for u in unloadable:
            log_lines.append(f"  UNLOADABLE {u.box_id}: {u.reason.value}")
        self.log_text.delete("1.0", "end")
        self.log_text.insert("1.0", "\n".join(log_lines))

        self.status_var.set(f"완료 - {len(plans)}/{len(cart_boxes_raw)}개 배치")

    # ------------------------------------------------------- 이미지 크기 조절

    def _on_window_resize(self, event):
        if event.widget is not self:
            return
        # 드래그 중 매 픽셀마다 리사이즈하면 버벅이므로 120ms 정도 묶어서 마지막 것만 처리
        if self._resize_job is not None:
            self.after_cancel(self._resize_job)
        self._resize_job = self.after(120, self._render_images)

    def _render_images(self):
        if "before" not in self._pil_originals:
            return
        self._show_image(self.before_label, self._pil_originals["before"])
        self._show_image(self.after_label, self._pil_originals["after"])

    def _show_image(self, label: tk.Label, original: Image.Image):
        # 카드 폭에 맞춰 확대/축소 (원본 화질 유지 - 캐시된 원본에서 매번 다시 리사이즈)
        target_w = max(label.winfo_width() - 4, 200)
        if target_w <= 10:
            target_w = 600
        ratio = target_w / original.width
        target_h = int(original.height * ratio)
        img = original.resize((target_w, target_h), Image.LANCZOS)
        photo = ImageTk.PhotoImage(img)
        label.configure(image=photo)
        self._tk_images[str(label)] = photo  # 참조 유지 (안 하면 가비지 컬렉션으로 사라짐)


if __name__ == "__main__":
    app = PlannerGUI()
    app.mainloop()
