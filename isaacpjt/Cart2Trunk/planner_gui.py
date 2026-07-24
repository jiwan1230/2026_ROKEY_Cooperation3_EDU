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
import time
import tkinter as tk
from datetime import datetime
from importlib import import_module
from tkinter import messagebox

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
build_task_json = _planner.build_task_json
_send_task_to_msi2 = _planner._send_task_to_msi2
SceneBox = _viz.SceneBox
draw_scene = _viz.draw_scene
draw_side_view = _viz.draw_side_view

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
        self._enabled = True
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
            if not self._enabled:
                fg = Palette.border
            self.create_text((x1 + x2) / 2, self._ctrl_h / 2, text=label, fill=fg, font=Font.label)

    def _on_click(self, event):
        if not self._enabled:
            return
        idx = min(int(event.x // self._seg_w), len(self._segments) - 1)
        self._var.set(self._segments[idx][1])

    def set_enabled(self, enabled: bool):
        self._enabled = enabled
        self._draw()


class ToggleSwitch(tk.Canvas):
    """iOS 스타일 on/off 스위치 - tk.Checkbutton의 각진 네이티브 체크박스 대신."""

    def __init__(self, parent, variable, width=46, height=26, **kwargs):
        super().__init__(parent, width=width, height=height, highlightthickness=0,
                          bg=parent["bg"], **kwargs)
        self._var = variable
        self._sw_w, self._sw_h = width, height
        self._enabled = True
        self.bind("<Button-1>", self._on_click)
        self.bind("<Enter>", lambda e: self.configure(cursor="hand2"))
        self._var.trace_add("write", lambda *_: self._draw())
        self._draw()

    def _on_click(self, event):
        if not self._enabled:
            return
        self._var.set(not self._var.get())

    def _draw(self):
        self.delete("all")
        on = bool(self._var.get())
        track_fill = Palette.success if on else Palette.segment_bg
        if not self._enabled:
            track_fill = Palette.border
        self.create_polygon(_rounded_rect_points(0, 0, self._sw_w, self._sw_h, self._sw_h / 2),
                             smooth=True, fill=track_fill, outline="")
        r = self._sw_h / 2 - 2
        cx = self._sw_w - self._sw_h / 2 if on else self._sw_h / 2
        cy = self._sw_h / 2
        self.create_oval(cx - r, cy - r, cx + r, cy + r, fill="white", outline="")

    def set_enabled(self, enabled: bool):
        self._enabled = enabled
        self._draw()


class IOSDropdown(tk.Canvas):
    """iOS 스타일 드롭다운 - ttk.Combobox는 각 OS/테마의 네이티브 테두리·화살표·
    목록 팝업을 그대로 써서 나머지 iOS풍 위젯들과 톤이 안 맞았다. RoundedButton과
    같은 방식으로 알약형 버튼을 직접 그리고, 목록은 테두리 없는 Toplevel 팝업으로
    띄워서 색상·폰트·모서리 반경까지 전부 Palette/Font를 그대로 따르게 했다."""

    _ROW_HEIGHT = 34  # 팝업 목록 한 줄의 대략적인 픽셀 높이 (Label padx/pady 8 기준)

    def __init__(self, parent, values, variable, width=220, height=36, font=None,
                 on_select=None, max_visible_rows=6, **kwargs):
        super().__init__(parent, width=width, height=height, highlightthickness=0,
                          bg=parent["bg"], **kwargs)
        self._values = self._normalize(values)
        self._var = variable
        self._dd_w, self._dd_h = width, height
        self._font = font or Font.body
        self._on_select = on_select
        self._max_visible_rows = max_visible_rows  # 이보다 항목이 많으면 스크롤 목록으로 전환
        self._enabled = True
        self._popup = None
        self.bind("<Button-1>", self._on_click)
        self.bind("<Enter>", lambda e: self.configure(cursor="hand2" if self._enabled else "arrow"))
        self._var.trace_add("write", lambda *_: self._draw())
        self._draw()

    @staticmethod
    def _normalize(values):
        """값 목록을 (표시 라벨, 실제 값) 튜플 리스트로 통일한다 - 단순 문자열
        목록(트렁크맵/프리셋)과 (라벨, 값) 목록(예: 박스 선택에서 "1. Box3"처럼
        적재 순서 번호는 라벨에만 보여주고 값은 box_id로 유지)을 둘 다 지원."""
        return [v if isinstance(v, tuple) else (v, v) for v in values]

    def _draw(self):
        self.delete("all")
        fill = Palette.segment_bg if self._enabled else Palette.canvas
        self.create_polygon(_rounded_rect_points(1, 1, self._dd_w - 1, self._dd_h - 1, 10),
                             smooth=True, fill=fill, outline=Palette.border)
        current = self._var.get()
        current_label = next((label for label, value in self._values if value == current), current)
        text = current_label or "선택..."
        text_color = Palette.text_primary if self._enabled else Palette.text_secondary
        self.create_text(14, self._dd_h / 2, text=text, fill=text_color,
                          font=self._font, anchor="w", width=self._dd_w - 40)
        # 셰브런(펼침) 아이콘 - 네이티브 콤보박스 화살표 대신 작은 삼각형 직접 그림
        cx, cy = self._dd_w - 18, self._dd_h / 2
        chevron_color = Palette.text_secondary if self._enabled else Palette.border
        self.create_polygon(cx - 5, cy - 3, cx + 5, cy - 3, cx, cy + 3,
                             fill=chevron_color, outline="")

    def _on_click(self, event):
        if not self._enabled:
            return
        self._close_popup() if self._popup is not None else self._open_popup()

    def _open_popup(self):
        self._popup = tk.Toplevel(self)
        self._popup.overrideredirect(True)
        self._popup.attributes("-topmost", True)
        self._popup.configure(bg=Palette.border)

        row_width_chars = max((len(label) for label, _ in self._values), default=12) + 2
        canvas_width = max(self._dd_w, row_width_chars * 8)
        visible_rows = min(len(self._values), self._max_visible_rows)
        popup_h = visible_rows * self._ROW_HEIGHT + 2

        # 화면 아래로 넘치면(예: 창 아래쪽에 있는 드롭다운) 버튼 위쪽에 띄운다 -
        # 실제로 발견된 버그: 항상 아래쪽에만 띄워서 항목이 많으면 화면 밖으로
        # 잘려 나갔음.
        screen_h = self.winfo_screenheight()
        x = self.winfo_rootx()
        y_below = self.winfo_rooty() + self._dd_h + 4
        if y_below + popup_h > screen_h and self.winfo_rooty() - popup_h - 4 >= 0:
            y = self.winfo_rooty() - popup_h - 4  # 버튼 위쪽에 띄움
        else:
            y = y_below
        self._popup.geometry(f"+{x}+{y}")

        outer = tk.Frame(self._popup, bg=Palette.surface)
        outer.pack(padx=1, pady=1)

        needs_scroll = len(self._values) > self._max_visible_rows
        if needs_scroll:
            canvas = tk.Canvas(outer, width=canvas_width, height=popup_h,
                                bg=Palette.surface, highlightthickness=0)
            scrollbar = tk.Scrollbar(outer, orient="vertical", command=canvas.yview)
            canvas.configure(yscrollcommand=scrollbar.set)
            canvas.pack(side="left", fill="both", expand=True)
            scrollbar.pack(side="right", fill="y")
            list_frame = tk.Frame(canvas, bg=Palette.surface)
            canvas.create_window((0, 0), window=list_frame, anchor="nw", width=canvas_width)
            list_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
            canvas.bind("<Button-4>", lambda e: canvas.yview_scroll(-1, "units"))
            canvas.bind("<Button-5>", lambda e: canvas.yview_scroll(1, "units"))
            canvas.bind("<MouseWheel>", lambda e: canvas.yview_scroll(int(-e.delta / 120), "units"))
        else:
            list_frame = tk.Frame(outer, bg=Palette.surface)
            list_frame.pack()

        for label, value in self._values:
            row = tk.Label(list_frame, text=label, font=self._font, anchor="w",
                            bg=Palette.surface, fg=Palette.text_primary,
                            padx=14, pady=8, width=row_width_chars, cursor="hand2")
            row.pack(fill="x")
            row.bind("<Enter>", lambda e, r=row: r.configure(bg=Palette.segment_bg))
            row.bind("<Leave>", lambda e, r=row: r.configure(bg=Palette.surface))
            row.bind("<Button-1>", lambda e, v=value: self._select(v))

        self._popup.bind("<FocusOut>", lambda e: self._close_popup())
        self._popup.focus_force()

    def _select(self, value):
        self._var.set(value)
        self._close_popup()
        if self._on_select is not None:
            self._on_select()

    def _close_popup(self):
        if self._popup is not None:
            self._popup.destroy()
            self._popup = None

    def set_enabled(self, enabled: bool):
        self._enabled = enabled
        if not enabled:
            self._close_popup()
        self._draw()

    def set_values(self, values):
        """목록을 나중에(예: 계획 계산 후 박스 ID 목록으로) 바꿔 끼울 수 있게 - 생성
        시점엔 값을 몰라도 되는 드롭다운(예: 박스 상세정보 선택)에 씀."""
        self._values = self._normalize(values)
        self._close_popup()
        self._draw()  # 선택된 값의 라벨이 바뀌었을 수 있어 다시 그림


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

        # ---- 승인 워크플로우 상태 (HMI 8절 원칙: 승인 전엔 MSI2로 전달 안 함,
        # 실행 중 파라미터 변경 금지) ----
        # NOT_COMPUTED(계획 없음) -> COMPUTED(계산됨, 승인 대기) -> APPROVED(승인됨,
        # 파라미터 잠금 + MSI2 전송 가능). "계획 거부"는 항상 NOT_COMPUTED로 되돌린다.
        self._plan_state = "NOT_COMPUTED"
        self._last_plans = None
        self._last_trunk_map_id = None
        self._last_box_snapshot_id = None
        self._last_run_parameters = None
        self._pending_task = None
        # 계획 계산 시점의 파라미터 스냅샷 - 그 후 아무거나 하나라도 바뀌면 기존
        # 계획을 무효화한다(_on_param_changed). "재스캔 후 기존 계획 무효화"와
        # "Box Snapshot/Trunk Map ID 불일치 시 실행 차단"(HMI 8절 원칙 #2, #5)을
        # 하나의 메커니즘으로 일반화: 트렁크맵/박스목록을 포함해 뭐가 됐든 계산
        # 당시와 달라지면 그 계획은 더 이상 신뢰할 수 없다고 본다.
        self._last_computed_snapshot = None
        self._auto_recompute_job = None  # 슬라이더/토글 변경 후 디바운싱된 자동 재계산 예약용

        self._build_header()
        # 이 환경의 Tk/Tcl이 위젯을 아주 빠르게 대량 생성하면(특히 Canvas 기반
        # 커스텀 위젯이 많아진 뒤로) 간헐적으로 세그폴트가 났다 - _build_header()
        # 직후 idle task를 한 번 비워주면(pending 이벤트 큐 flush) 재현 안 됨을
        # 직접 여러 번 재현/수정해서 확인함. 완전한 원인 규명은 아니지만 안전한
        # 완화책이라 남겨둠.
        self.update_idletasks()
        self._build_controls()
        self._build_result_area()

        self.bind("<Configure>", self._on_window_resize)

    # ------------------------------------------------------------------ UI

    def _build_header(self):
        header = tk.Frame(self, bg=Palette.canvas)
        header.pack(side="top", fill="x", padx=28, pady=(22, 4))

        title_col = tk.Frame(header, bg=Palette.canvas)
        title_col.pack(side="left")
        tk.Label(title_col, text="CART2TRUNK · PLANNER", font=Font.caption,
                 fg=Palette.accent, bg=Palette.canvas).pack(anchor="w")
        tk.Label(title_col, text="적재 알고리즘 비포/애프터 시뮬레이터", font=Font.title,
                 fg=Palette.text_primary, bg=Palette.canvas).pack(anchor="w", pady=(2, 0))

        # HMI 8절 원칙 #6: "Emergency Stop 버튼은 모든 화면에서 접근할 수 있어야
        # 한다" - _set_params_enabled의 잠금 대상에 절대 포함시키지 않는다(항상
        # 클릭 가능). ⚠️ 이 GUI는 로봇에 직접 연결되지 않은 로컬 시뮬레이터라, 이
        # 버튼이 실제로 멈출 수 있는 건 "이 화면이 만드는 승인/전송"뿐이다 - 실제
        # 로봇 모터 정지는 MSI2/하드웨어 E-Stop 담당(_on_emergency_stop 참고).
        self.estop_button = RoundedButton(header, "🛑 EMERGENCY STOP", self._on_emergency_stop,
                                           bg=Palette.danger, width=210, height=44, radius=10)
        self.estop_button.pack(side="right", anchor="ne")

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
        if trunk_map_names:
            self.trunk_map_var.set(trunk_map_names[-1])
        self.trunk_map_dropdown = IOSDropdown(row1, trunk_map_names, self.trunk_map_var, width=230, height=36)
        self.trunk_map_dropdown.grid(row=1, column=0, sticky="w", pady=(4, 0))

        self._field_label(row1, "카트 박스 프리셋").grid(row=0, column=1, sticky="w", padx=(28, 0))
        self.box_preset_var = tk.StringVar(value=next(iter(self._box_presets)))
        self.box_preset_dropdown = IOSDropdown(row1, list(self._box_presets.keys()), self.box_preset_var,
                                                width=210, height=36, on_select=self._on_preset_selected)
        self.box_preset_dropdown.grid(row=1, column=1, sticky="w", padx=(28, 0), pady=(4, 0))

        self._field_label(row1, "박스 개수로 자동 생성").grid(row=0, column=2, sticky="w", padx=(28, 0))
        gen_frame = tk.Frame(row1, bg=Palette.surface)
        gen_frame.grid(row=1, column=2, sticky="w", padx=(28, 0), pady=(4, 0))
        self.box_count_var = tk.IntVar(value=6)
        self.box_count_spin = tk.Spinbox(gen_frame, from_=1, to=40, width=4,
                                          textvariable=self.box_count_var, font=Font.body,
                                          relief="solid", bd=1)
        self.box_count_spin.pack(side="left")
        self.gen_button = RoundedButton(gen_frame, "자동 생성", self._on_generate_boxes,
                                         bg=Palette.segment_bg, fg=Palette.text_primary,
                                         width=100, height=30, radius=15)
        self.gen_button.pack(side="left", padx=(10, 0))

        # ---- 2행: 적재 모드(세그먼트) / 쌓기 / 실행 ----
        row2 = tk.Frame(inner, bg=Palette.surface)
        row2.pack(fill="x", pady=(16, 0))

        self._field_label(row2, "적재 모드").grid(row=0, column=0, sticky="w")
        self.mode_var = tk.StringVar(value="large_first")
        self.mode_control = SegmentedControl(
            row2, [("큰 거 우선", "large_first"), ("개수 우선", "count_first")],
            self.mode_var, width=240, height=34)
        self.mode_control.grid(row=1, column=0, sticky="w", pady=(4, 0))

        self._field_label(row2, "2층↑ 쌓기 허용").grid(row=0, column=1, sticky="w", padx=(28, 0))
        stacking_frame = tk.Frame(row2, bg=Palette.surface)
        stacking_frame.grid(row=1, column=1, sticky="w", padx=(28, 0), pady=(4, 0))
        self.stacking_var = tk.BooleanVar(value=False)
        self.stacking_switch = ToggleSwitch(stacking_frame, self.stacking_var)
        self.stacking_switch.pack(side="left")

        self.run_button = RoundedButton(row2, "① 계획 계산", self._run, width=140, height=38)
        self.run_button.grid(row=1, column=2, sticky="w", padx=(28, 0), pady=(4, 0))

        # "계획 다시 계산"은 ①과 같은 동작(self._run) - 파라미터를 바꾼 뒤 ①을
        # 다시 누르는 것 자체가 "다시 계산"이라, 별도 로직 없이 문서가 요구하는
        # 버튼만 하나 더 둔다(사용자가 "다시 계산"이라는 라벨을 찾기 쉽게).
        self.recompute_button = RoundedButton(row2, "다시 계산", self._run,
                                               bg=Palette.segment_bg, fg=Palette.text_primary,
                                               width=100, height=38, radius=19)
        self.recompute_button.grid(row=1, column=3, sticky="w", padx=(10, 0), pady=(4, 0))

        self.reset_button = RoundedButton(row2, "기본값 복원", self._on_reset_defaults,
                                           bg=Palette.segment_bg, fg=Palette.text_primary,
                                           width=110, height=38, radius=19)
        self.reset_button.grid(row=1, column=4, sticky="w", padx=(10, 0), pady=(4, 0))

        self.status_var = tk.StringVar(value="준비됨")
        tk.Label(row2, textvariable=self.status_var, font=Font.caption,
                 fg=Palette.text_secondary, bg=Palette.surface).grid(
            row=1, column=5, sticky="w", padx=(16, 0), pady=(4, 0)
        )

        # ---- 2-b행: 안전 마진 4종 ("HMI 화면 설계 가이드라인" 문서 4절) - 전부
        # 비워두면 각자 기본값(margin=17_margin_check.MARGIN, ceiling=15_overhead_
        # clearance_check.OVERHEAD_CLEARANCE)을 그대로 쓴다. ----
        row2b = tk.Frame(inner, bg=Palette.surface)
        row2b.pack(fill="x", pady=(16, 0))

        margin_fields = [
            (f"박스 간격 (m · 기본 {DEFAULT_MARGIN})", "margin_var"),
            ("벽면 간격 (m · 기본 박스간격과 동일)", "wall_margin_var"),
            ("천장 여유 (m · 기본 0.20)", "ceiling_margin_var"),
            ("장애물 간격 (m · 기본 박스간격과 동일)", "obstacle_margin_var"),
            ("입구 여유 거리 (m · 기본 벽면간격과 동일)", "entrance_margin_var"),
        ]
        self.margin_entries = []
        for col, (label, var_name) in enumerate(margin_fields):
            self._field_label(row2b, label).grid(row=0, column=col, sticky="w", padx=(0 if col == 0 else 28, 0))
            var = tk.StringVar(value="")
            setattr(self, var_name, var)
            entry = tk.Entry(row2b, textvariable=var, width=10, font=Font.body, relief="solid", bd=1)
            entry.grid(row=1, column=col, sticky="w", padx=(0 if col == 0 else 28, 0), pady=(4, 0), ipady=3)
            self.margin_entries.append(entry)

        # ---- 2-c행: 적재 우선순위 슬라이더 2축 + 회전 허용 토글 ----
        row2c = tk.Frame(inner, bg=Palette.surface)
        row2c.pack(fill="x", pady=(16, 0))

        self._field_label(row2c, "입구 우선 ↔ 깊은 위치 우선").grid(row=0, column=0, sticky="w")
        self.entrance_pref_var = tk.DoubleVar(value=1.0)
        self.entrance_pref_scale = tk.Scale(
            row2c, from_=-1.0, to=1.0, resolution=0.1, orient="horizontal", length=200,
            variable=self.entrance_pref_var, showvalue=True, font=Font.caption,
            bg=Palette.surface, highlightthickness=0, troughcolor=Palette.segment_bg)
        self.entrance_pref_scale.grid(row=1, column=0, sticky="w", pady=(4, 0))

        self._field_label(row2c, "공간활용률 우선 ↔ 안정성 우선(접촉면 가중치)").grid(
            row=0, column=1, sticky="w", padx=(28, 0))
        self.contact_pref_var = tk.DoubleVar(value=1.0)
        self.contact_pref_scale = tk.Scale(
            row2c, from_=0.0, to=2.0, resolution=0.1, orient="horizontal", length=200,
            variable=self.contact_pref_var, showvalue=True, font=Font.caption,
            bg=Palette.surface, highlightthickness=0, troughcolor=Palette.segment_bg)
        self.contact_pref_scale.grid(row=1, column=1, sticky="w", padx=(28, 0), pady=(4, 0))

        self._field_label(row2c, "바닥부터 채우기 강도").grid(row=0, column=2, sticky="w", padx=(28, 0))
        self.height_pref_var = tk.DoubleVar(value=1.0)
        self.height_pref_scale = tk.Scale(
            row2c, from_=0.0, to=2.0, resolution=0.1, orient="horizontal", length=200,
            variable=self.height_pref_var, showvalue=True, font=Font.caption,
            bg=Palette.surface, highlightthickness=0, troughcolor=Palette.segment_bg)
        self.height_pref_scale.grid(row=1, column=2, sticky="w", padx=(28, 0), pady=(4, 0))

        self._field_label(row2c, "박스 90도 회전 허용").grid(row=0, column=3, sticky="w", padx=(28, 0))
        rotation_frame = tk.Frame(row2c, bg=Palette.surface)
        rotation_frame.grid(row=1, column=3, sticky="w", padx=(28, 0), pady=(4, 0))
        self.allow_rotation_var = tk.BooleanVar(value=True)
        self.rotation_switch = ToggleSwitch(rotation_frame, self.allow_rotation_var)
        self.rotation_switch.pack(side="left")

        self._field_label(row2c, "적재 순서 고정(박스 목록 순서 그대로)").grid(row=0, column=4, sticky="w", padx=(28, 0))
        fixed_order_frame = tk.Frame(row2c, bg=Palette.surface)
        fixed_order_frame.grid(row=1, column=4, sticky="w", padx=(28, 0), pady=(4, 0))
        self.fixed_order_var = tk.BooleanVar(value=False)
        self.fixed_order_switch = ToggleSwitch(fixed_order_frame, self.fixed_order_var)
        self.fixed_order_switch.pack(side="left")

        # ---- 3행: 박스 목록 JSON (접이식 느낌으로 작게, 필요할 때만 손으로 수정) ----
        row3 = tk.Frame(inner, bg=Palette.surface)
        row3.pack(fill="x", pady=(16, 0))
        self._field_label(row3, "박스 목록 (JSON · 직접 수정 가능)").pack(anchor="w")
        self.box_text = tk.Text(row3, height=4, font=Font.mono, relief="solid", bd=1,
                                 wrap="none", padx=8, pady=6)
        self.box_text.pack(fill="x", pady=(4, 0))
        self.box_text.bind("<<Modified>>", self._on_box_text_modified)
        self._on_preset_selected()

        # ---- 파라미터 변경 감지 배선 - 값이 하나라도 바뀌면 계산 당시 스냅샷과
        # 달라지므로 _on_param_changed가 기존 계획을 무효화한다. ----
        for var in (self.trunk_map_var, self.box_preset_var, self.mode_var, self.margin_var,
                    self.wall_margin_var, self.ceiling_margin_var, self.obstacle_margin_var,
                    self.entrance_margin_var, self.entrance_pref_var, self.contact_pref_var,
                    self.height_pref_var, self.stacking_var, self.allow_rotation_var,
                    self.fixed_order_var):
            var.trace_add("write", self._on_param_changed)

        # ---- 4행: 승인 워크플로우 - "HMI 화면 설계 가이드라인" 4절이 요구하는
        # 계획계산(위 ①) -> 현재계획승인 -> 계획거부/승인및실행 흐름. 승인되면
        # 파라미터가 잠기고(_set_params_enabled), 승인 전엔 MSI2로 아무것도
        # 나가지 않는다(_send_task_to_msi2가 approved=False를 거부).
        row4 = tk.Frame(inner, bg=Palette.surface)
        row4.pack(fill="x", pady=(16, 0))

        self.approve_button = RoundedButton(row4, "② 현재 계획 승인", self._on_approve,
                                             bg=Palette.success, width=160, height=36)
        self.approve_button.pack(side="left")
        self.reject_button = RoundedButton(row4, "계획 거부", self._on_reject,
                                            bg=Palette.danger, width=110, height=36)
        self.reject_button.pack(side="left", padx=(10, 0))
        self.send_button = RoundedButton(row4, "③ 승인 및 실행(MSI2로)", self._on_send,
                                          bg=Palette.accent, width=190, height=36)
        self.send_button.pack(side="left", padx=(10, 0))

        self.plan_state_var = tk.StringVar(value="")
        tk.Label(row4, textvariable=self.plan_state_var, font=Font.caption,
                 fg=Palette.text_secondary, bg=Palette.surface).pack(side="left", padx=(16, 0))

        self._set_plan_state("NOT_COMPUTED")

    def _field_label(self, parent, text):
        return tk.Label(parent, text=text, font=Font.section, fg=Palette.text_secondary, bg=Palette.surface)

    def _build_result_area(self):
        # ---- 계획 요약 카드 - "총 입력/적재가능/불가능/공간활용률/계산시간/
        # 전체점수"를 한눈에. 값은 _run_impl()이 계산 직후 채운다. ----
        summary_outer = tk.Frame(self, bg=Palette.canvas)
        summary_outer.pack(side="top", fill="x", padx=28, pady=(0, 10))
        summary_card = Card(summary_outer)
        summary_card.pack(fill="x")
        summary_inner = tk.Frame(summary_card, bg=Palette.surface, padx=20, pady=12)
        summary_inner.pack(fill="x")
        tk.Label(summary_inner, text="계획 요약", font=Font.section, fg=Palette.text_secondary,
                 bg=Palette.surface).grid(row=0, column=0, columnspan=6, sticky="w", pady=(0, 6))

        self.summary_vars = {}
        summary_fields = [
            ("총 입력 박스 수", "total"), ("적재 가능", "placed"), ("적재 불가능", "unplaced"),
            ("공간 활용률", "utilization"), ("계산 시간", "calc_time"),
            ("평균 배치 점수(낮을수록 좋음)", "avg_score"),
        ]
        for col, (label, key) in enumerate(summary_fields):
            box = tk.Frame(summary_inner, bg=Palette.surface)
            box.grid(row=1, column=col, sticky="w", padx=(0 if col == 0 else 24, 0))
            tk.Label(box, text=label, font=Font.caption, fg=Palette.text_secondary,
                     bg=Palette.surface).pack(anchor="w")
            var = tk.StringVar(value="-")
            self.summary_vars[key] = var
            tk.Label(box, textvariable=var, font=Font.section, fg=Palette.text_primary,
                     bg=Palette.surface).pack(anchor="w")

        outer = tk.Frame(self, bg=Palette.canvas)
        outer.pack(side="top", fill="both", expand=True, padx=28, pady=(0, 10))
        outer.columnconfigure(0, weight=1)
        outer.columnconfigure(1, weight=1)
        outer.columnconfigure(2, weight=1)
        outer.rowconfigure(0, weight=1)

        self.before_card = Card(outer)
        self.before_card.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        tk.Label(self.before_card, text="BEFORE · 아직 안 실음", font=Font.section,
                 fg=Palette.text_secondary, bg=Palette.surface).pack(anchor="w", padx=16, pady=(14, 6))
        self.before_label = tk.Label(self.before_card, bg=Palette.surface)
        self.before_label.pack(fill="both", expand=True, padx=16, pady=(0, 16))

        self.after_card = Card(outer)
        self.after_card.grid(row=0, column=1, sticky="nsew", padx=6)
        tk.Label(self.after_card, text="AFTER · 배치 결과", font=Font.section,
                 fg=Palette.accent, bg=Palette.surface).pack(anchor="w", padx=16, pady=(14, 6))
        self.after_label = tk.Label(self.after_card, bg=Palette.surface)
        self.after_label.pack(fill="both", expand=True, padx=16, pady=(0, 8))

        # ---- 박스 선택 -> 상세정보(적재순서/Target Pose/Yaw/Score/선정사유) ----
        detail_frame = tk.Frame(self.after_card, bg=Palette.surface)
        detail_frame.pack(fill="x", padx=16, pady=(0, 14))
        tk.Label(detail_frame, text="박스 선택", font=Font.caption, fg=Palette.text_secondary,
                 bg=Palette.surface).pack(side="left")
        self.box_select_var = tk.StringVar(value="")
        self.box_select_dropdown = IOSDropdown(detail_frame, [], self.box_select_var, width=130, height=30,
                                                font=Font.caption, on_select=self._on_box_selected)
        self.box_select_dropdown.pack(side="left", padx=(8, 0))
        self.box_detail_var = tk.StringVar(value="계획 계산 후 박스를 선택하면 상세정보가 표시됩니다")
        tk.Label(detail_frame, textvariable=self.box_detail_var, font=Font.caption,
                 fg=Palette.text_primary, bg=Palette.surface, anchor="w", justify="left",
                 wraplength=460).pack(side="left", padx=(14, 0), fill="x", expand=True)

        self.side_card = Card(outer)
        self.side_card.grid(row=0, column=2, sticky="nsew", padx=(6, 0))
        tk.Label(self.side_card, text="SIDE VIEW · 측면도(높이 확인용)", font=Font.section,
                 fg=Palette.text_secondary, bg=Palette.surface).pack(anchor="w", padx=16, pady=(14, 6))
        self.side_label = tk.Label(self.side_card, bg=Palette.surface)
        self.side_label.pack(fill="both", expand=True, padx=16, pady=(0, 16))

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

    def _show_error(self, code: str, cause: str, action: str):
        """"HMI 핵심 동작 원칙" 7번: 오류 코드뿐 아니라 해결 방법도 함께 표시한다."""
        messagebox.showerror(f"오류: {code}", f"원인:\n{cause}\n\n권장 조치:\n{action}")

    def _run(self):
        try:
            self._run_impl()
        except json.JSONDecodeError as e:
            self._show_error(
                "BOX_JSON_INVALID", f"박스 목록 JSON 형식이 올바르지 않습니다: {e}",
                "박스 목록 텍스트 상자의 JSON 문법(쉼표, 중괄호, 따옴표)을 확인한 뒤 다시 계산하세요.",
            )
            self.status_var.set("오류: 박스 JSON 형식 오류")
        except Exception as e:
            self._show_error(
                type(e).__name__, str(e),
                "입력값(트렁크 스캔 파일, 박스 목록, 마진/우선순위 파라미터)을 확인한 뒤 다시 시도하세요.",
            )
            self.status_var.set(f"오류: {e}")

    def _run_impl(self):
        if not self._trunk_maps:
            self._show_error(
                "TRUNK_MAP_NOT_FOUND",
                f"{_SRC_DIR} 밑에서 run_*/pointcloud/trunk_map.json을 하나도 찾지 못했습니다.",
                "트렁크 스캔이 완료된 run_* 폴더가 있는지 확인하거나, 준형님께 trunk_map.json 생성을 요청하세요.",
            )
            return

        run_name = self.trunk_map_var.get()
        trunk_map_path = next(p for p in self._trunk_maps if p.parent.parent.name == run_name)
        data = json.loads(trunk_map_path.read_text())

        cart_boxes_raw = json.loads(self.box_text.get("1.0", "end"))
        mode = self.mode_var.get()

        def _parse_optional_float(var):
            s = var.get().strip()
            return float(s) if s else None

        margin = _parse_optional_float(self.margin_var)
        wall_margin = _parse_optional_float(self.wall_margin_var)
        obstacle_margin = _parse_optional_float(self.obstacle_margin_var)
        ceiling_margin = _parse_optional_float(self.ceiling_margin_var)
        entrance_margin = _parse_optional_float(self.entrance_margin_var)
        entrance_preference = self.entrance_pref_var.get()
        contact_preference = self.contact_pref_var.get()
        height_preference = self.height_pref_var.get()
        allow_stacking = self.stacking_var.get()
        allow_rotation = self.allow_rotation_var.get()
        # "적재 순서 고정"이 켜져 있으면, 박스 목록 JSON에 적힌 순서를 그대로 fixed_order로
        # 씀 - 사용자가 순서를 바꾸고 싶으면 JSON에서 박스를 재배열하면 된다.
        fixed_order = [b["id"] for b in cart_boxes_raw] if self.fixed_order_var.get() else None

        self.status_var.set("계산 중...")
        self.update_idletasks()

        t0 = time.perf_counter()
        plans, unloadable, trunk, obstacles = plan_from_trunk_map_data(
            data, cart_boxes_raw, mode=mode, margin=margin, allow_stacking=allow_stacking,
            allow_rotation=allow_rotation, wall_margin=wall_margin, obstacle_margin=obstacle_margin,
            ceiling_margin=ceiling_margin, entrance_margin=entrance_margin,
            entrance_preference=entrance_preference, contact_preference=contact_preference,
            height_preference=height_preference, fixed_order=fixed_order,
        )
        calc_time_sec = time.perf_counter() - t0
        effective_margin = margin if margin is not None else DEFAULT_MARGIN
        self._last_plans = plans  # box 상세정보 갱신(_on_box_selected)이 최신 결과를 보게 미리 반영

        box_by_id = {b["id"]: b for b in cart_boxes_raw}
        fixed_obstacles = [
            SceneBox(o.box.id, o.x, o.y, o.z, o.box.width, o.box.depth, o.box.height, "#7f8c8d")
            for o in obstacles
        ]
        placed_scene_boxes = [
            SceneBox(p.box_id, p.position[0], p.position[1], p.position[2],
                     p.dimensions[0], p.dimensions[1], p.dimensions[2], _color_for_box_id(p.box_id),
                     dashed=(p.position[2] > 1e-6))
            for p in plans
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
            placed_boxes=placed_scene_boxes,
            waiting_boxes=[
                SceneBox(u.box_id, 0, 0, 0, box_by_id[u.box_id]["width"], box_by_id[u.box_id]["depth"],
                          box_by_id[u.box_id]["height"], _color_for_box_id(u.box_id))
                for u in unloadable
            ],
            title=f"After - mode={mode}, margin={effective_margin:.2f}m, "
                  f"쌓기={'허용' if allow_stacking else '1층전용'} ({len(plans)}/{len(cart_boxes_raw)}개 적재)",
            out_path=str(after_path),
        )

        # ---- Side View: x-z 평면(높이 확인용) ----
        side_path = _GUI_OUT_DIR / "side.png"
        draw_side_view(
            trunk.width, trunk.height, fixed_obstacles=fixed_obstacles, placed_boxes=placed_scene_boxes,
            title=f"Side View - {len(plans)}/{len(cart_boxes_raw)}개 적재, 최고층 z="
                  f"{max((p.position[2] + p.dimensions[2] for p in plans), default=0.0):.2f}m",
            out_path=str(side_path),
        )

        self._pil_originals["before"] = Image.open(before_path).copy()
        self._pil_originals["after"] = Image.open(after_path).copy()
        self._pil_originals["side"] = Image.open(side_path).copy()
        self._render_images()

        # ---- 계획 요약 카드 갱신 ----
        placed_volume = sum(p.dimensions[0] * p.dimensions[1] * p.dimensions[2] for p in plans)
        trunk_volume = trunk.width * trunk.depth * trunk.height
        utilization_pct = (placed_volume / trunk_volume * 100) if trunk_volume > 1e-9 else 0.0
        avg_score = (sum(p.score for p in plans) / len(plans)) if plans else 0.0
        self.summary_vars["total"].set(str(len(cart_boxes_raw)))
        self.summary_vars["placed"].set(str(len(plans)))
        self.summary_vars["unplaced"].set(str(len(unloadable)))
        self.summary_vars["utilization"].set(f"{utilization_pct:.1f}%")
        self.summary_vars["calc_time"].set(f"{calc_time_sec * 1000:.0f}ms")
        self.summary_vars["avg_score"].set(f"{avg_score:.3f}")

        # ---- 박스 선택 드롭다운 갱신 - p.order로 명시적으로 정렬하고, 라벨에
        # 순번을 보여줘서 "적재 순서가 맞는지" 눈으로 바로 확인할 수 있게 한다.
        # p.order는 실제로 이미 놓인 카트 박스 개수 다음부터 매겨지고(09_rescan_
        # replan._run_strategy) 트렁크 장애물은 순번에서 빠지므로, 첫 카트
        # 박스는 항상 order=1부터 시작한다 - 그대로 보여줘도 헷갈리지 않음.
        plans_by_order = sorted(plans, key=lambda p: p.order)
        self.box_select_dropdown.set_values([(f"{p.order}. {p.box_id}", p.box_id) for p in plans_by_order])
        self.box_select_var.set(plans_by_order[0].box_id if plans_by_order else "")
        self._on_box_selected()

        log_lines = [f"[{run_name}] mode={mode}, margin={effective_margin:.2f}m, "
                     f"쌓기={'허용' if allow_stacking else '1층전용'}, "
                     f"회전={'허용' if allow_rotation else '비허용'}, "
                     f"입구/깊이축={entrance_preference:+.1f}, 접촉면가중치={contact_preference:.1f}, "
                     f"바닥우선강도={height_preference:.1f}, "
                     f"순서고정={'예' if fixed_order else '아니오'} "
                     f"-> {len(plans)}/{len(cart_boxes_raw)}개 배치"]
        for p in plans:
            log_lines.append(f"  PLACED {p.box_id}: pos=({p.position[0]:.2f},{p.position[1]:.2f},{p.position[2]:.2f}) rotated={p.rotated}")
        for u in unloadable:
            log_lines.append(f"  UNLOADABLE {u.box_id}: {u.reason.value}")
        self.log_text.delete("1.0", "end")
        self.log_text.insert("1.0", "\n".join(log_lines))

        self.status_var.set(f"완료 - {len(plans)}/{len(cart_boxes_raw)}개 배치")

        # ---- 승인 워크플로우용 메타데이터 저장 (② 현재 계획 승인이 이걸로 Task
        # JSON을 만든다) - self._last_plans는 위에서 이미 반영함 ----
        self._last_trunk_map_id = data.get("run_id", run_name)  # trunk_map.json 실제 필드(run_id) 재사용
        # ⚠️ box_snapshot_id는 아직 placeholder다 - 이 GUI의 박스 입력은 실제
        # box_scan.json(①.load_box_snapshot_from_json)이 아니라 프리셋/수동 JSON을
        # 쓰고 있어서 진짜 snapshot_id가 없다. 실제 비전 연동 시 box_scan.json을
        # 선택하는 UI(트렁크 스캔 파일 선택과 같은 방식)로 교체하면서 여기도
        # snapshot.snapshot_id로 바꿔야 한다.
        self._last_box_snapshot_id = f"manual_input:{self.box_preset_var.get()}"
        self._last_run_parameters = {
            "mode": mode,
            "margin": effective_margin,
            "wall_margin": wall_margin,
            "obstacle_margin": obstacle_margin,
            "ceiling_margin": ceiling_margin,
            "entrance_margin": entrance_margin,
            "allow_stacking": allow_stacking,
            "allow_rotation": allow_rotation,
            "entrance_preference": entrance_preference,
            "contact_preference": contact_preference,
            "height_preference": height_preference,
            "fixed_order": fixed_order,
        }
        self._pending_task = None
        self._last_computed_snapshot = self._current_param_snapshot()
        self._set_plan_state("COMPUTED")

    # ------------------------------------------------------ 승인 워크플로우

    def _set_plan_state(self, state: str):
        """NOT_COMPUTED -> COMPUTED -> APPROVED. "계획 거부"는 항상 NOT_COMPUTED로
        되돌린다(재계산부터 다시 하도록 강제 - 승인 없이 애매하게 남는 상태를 없앰)."""
        self._plan_state = state
        labels = {
            "NOT_COMPUTED": "①을 먼저 눌러 계획을 계산하세요",
            "COMPUTED": "계획 계산됨 - ②로 승인하거나 거부하세요",
            "APPROVED": "승인됨 - 파라미터 잠김 - ③으로 MSI2에 보낼 수 있습니다",
        }
        self.plan_state_var.set(labels[state])
        self.approve_button.set_enabled(state == "COMPUTED")
        self.reject_button.set_enabled(state in ("COMPUTED", "APPROVED"))
        self.send_button.set_enabled(state == "APPROVED")
        # HMI 8절 원칙 #4: 실행 중(=승인 후) 파라미터 변경 금지 - 승인되면 잠그고,
        # 거부/재계산 전까지는 안 풀린다.
        self._set_params_enabled(state != "APPROVED")

    def _set_params_enabled(self, enabled: bool):
        entry_state = "normal" if enabled else "disabled"
        scale_state = "normal" if enabled else "disabled"
        self.trunk_map_dropdown.set_enabled(enabled)
        self.box_preset_dropdown.set_enabled(enabled)
        self.box_count_spin.configure(state=entry_state)
        for entry in self.margin_entries:
            entry.configure(state=entry_state)
        self.entrance_pref_scale.configure(state=scale_state)
        self.contact_pref_scale.configure(state=scale_state)
        self.height_pref_scale.configure(state=scale_state)
        self.box_text.configure(state=entry_state)
        self.gen_button.set_enabled(enabled)
        self.mode_control.set_enabled(enabled)
        self.stacking_switch.set_enabled(enabled)
        self.rotation_switch.set_enabled(enabled)
        self.fixed_order_switch.set_enabled(enabled)
        self.run_button.set_enabled(enabled)
        self.recompute_button.set_enabled(enabled)
        self.reset_button.set_enabled(enabled)

    def _append_log(self, text: str):
        self.log_text.insert("end", "\n" + text)
        self.log_text.see("end")

    def _on_approve(self):
        if self._plan_state != "COMPUTED" or self._last_plans is None:
            return
        plan_id = f"load_plan_{datetime.now():%Y%m%d_%H%M%S}"
        task = build_task_json(
            plan_id=plan_id,
            box_snapshot_id=self._last_box_snapshot_id,
            trunk_map_id=self._last_trunk_map_id,
            parameters=self._last_run_parameters,
            plans=self._last_plans,
            approved=True,
        )
        self._pending_task = task
        self._append_log(
            f"[승인] plan_id={plan_id} (box_snapshot_id={self._last_box_snapshot_id} - "
            f"아직 placeholder, 실제 비전 스냅샷 연동 전)"
        )
        self._set_plan_state("APPROVED")

    def _on_reject(self):
        self._pending_task = None
        self._append_log("[거부] 계획을 거부했습니다 - 파라미터를 조정하고 ①부터 다시 계산하세요.")
        self._set_plan_state("NOT_COMPUTED")

    def _on_send(self):
        if self._plan_state != "APPROVED" or self._pending_task is None:
            return
        try:
            out_path = _send_task_to_msi2(self._pending_task)
        except Exception as e:
            self._show_error(
                type(e).__name__, str(e),
                "승인된 계획이 approved=True인지, 저장 경로에 쓰기 권한이 있는지 확인하세요.",
            )
            return
        self._append_log(
            f"[승인 및 실행] MSI2 실제 전송 경로 미확정(TODO - 지완 확인 필요) - "
            f"로컬에만 저장됨: {out_path}\n  실제 로봇 동작은 시작되지 않습니다."
        )
        self.status_var.set("승인된 계획을 로컬에 저장함 (MSI2 실전송 경로 확정 대기)")

    # ---------------------------------------------- 파라미터 변경 감지·무효화

    def _current_param_snapshot(self) -> dict:
        """지금 화면에 있는 파라미터 전부를 스냅샷으로 - _on_param_changed가 이걸
        계산 시점 스냅샷과 비교해서 계획이 여전히 유효한지 판단한다."""
        return {
            "trunk_map": self.trunk_map_var.get(),
            "box_preset": self.box_preset_var.get(),
            "box_text": self.box_text.get("1.0", "end"),
            "mode": self.mode_var.get(),
            "margin": self.margin_var.get(),
            "wall_margin": self.wall_margin_var.get(),
            "obstacle_margin": self.obstacle_margin_var.get(),
            "ceiling_margin": self.ceiling_margin_var.get(),
            "entrance_margin": self.entrance_margin_var.get(),
            "entrance_preference": self.entrance_pref_var.get(),
            "contact_preference": self.contact_pref_var.get(),
            "height_preference": self.height_pref_var.get(),
            "allow_stacking": self.stacking_var.get(),
            "allow_rotation": self.allow_rotation_var.get(),
            "fixed_order": self.fixed_order_var.get(),
        }

    def _invalidate_plan_if_stale(self) -> bool:
        """트렁크맵/박스목록/모드/마진/우선순위/쌓기/회전/순서고정 중 뭐든
        하나라도 계산 시점과 달라지면 기존 계획을 무효화한다("HMI 핵심 동작
        원칙" #2, #5를 하나로 일반화 - box_snapshot_id/trunk_map_id가 계산 때와
        달라진 것도, 재스캔으로 트렁크가 바뀐 것도 결국 "계산 당시 입력과
        지금이 다르다"는 같은 문제라서 한 메커니즘으로 다룬다). 실제로
        무효화가 일어났으면 True."""
        if self._plan_state == "NOT_COMPUTED" or self._last_computed_snapshot is None:
            return False
        if self._current_param_snapshot() == self._last_computed_snapshot:
            return False
        was_approved = self._plan_state == "APPROVED"
        self._pending_task = None
        self._set_plan_state("NOT_COMPUTED")
        suffix = " (승인도 함께 취소됨)" if was_approved else ""
        self._append_log(f"[무효화] 파라미터가 변경되어 기존 계획을 무효화했습니다{suffix}.")
        return True

    def _on_param_changed(self, *_args):
        """슬라이더/토글/드롭다운처럼 항상 유효한 값만 나오는 파라미터가
        바뀌면, 무효화뿐 아니라 디바운싱된 자동 재계산까지 예약한다(진짜
        "즉시 재계산" - 매번 버튼을 누를 필요 없음)."""
        if self._invalidate_plan_if_stale():
            self.status_var.set("⚠️ 파라미터 변경됨 - 잠시 후 자동 재계산...")
            self._schedule_auto_recompute()

    def _on_box_text_modified(self, event=None):
        """박스 목록 JSON은 타이핑 도중 문법이 잠깐 깨진 상태를 거칠 수 있어서
        (예: 여는 중괄호만 친 순간), 자동 재계산은 하지 않고 무효화까지만 한다
        - "다시 계산" 버튼을 직접 눌러야 반영된다."""
        if self.box_text.edit_modified():
            if self._invalidate_plan_if_stale():
                self.status_var.set("⚠️ 박스 목록이 변경됨 - '다시 계산'을 눌러주세요")
            self.box_text.edit_modified(False)  # Text 위젯의 modified 플래그는 수동으로 꺼줘야 계속 감지됨

    def _schedule_auto_recompute(self):
        """슬라이더를 드래그하는 동안 매 픽셀마다 재계산하면 버벅이므로,
        마지막 변경 후 400ms 동안 추가 변경이 없을 때만 실제로 재계산한다
        (_on_window_resize의 디바운싱과 같은 방식)."""
        if self._auto_recompute_job is not None:
            self.after_cancel(self._auto_recompute_job)
        self._auto_recompute_job = self.after(400, self._run)

    def _on_reset_defaults(self):
        """전략 파라미터만 기본값으로 되돌린다 (트렁크맵/박스목록 선택은 사용자
        입력 데이터라 안 건드림)."""
        self.mode_var.set("large_first")
        self.margin_var.set("")
        self.wall_margin_var.set("")
        self.obstacle_margin_var.set("")
        self.ceiling_margin_var.set("")
        self.entrance_margin_var.set("")
        self.entrance_pref_var.set(1.0)
        self.contact_pref_var.set(1.0)
        self.height_pref_var.set(1.0)
        self.stacking_var.set(False)
        self.allow_rotation_var.set(True)
        self.fixed_order_var.set(False)
        self._append_log("[기본값 복원] 적재 전략 파라미터를 기본값으로 되돌렸습니다.")

    def _on_box_selected(self, *_args):
        box_id = self.box_select_var.get()
        plan = next((p for p in (self._last_plans or []) if p.box_id == box_id), None)
        if plan is None:
            self.box_detail_var.set("계획 계산 후 박스를 선택하면 상세정보가 표시됩니다")
            return
        reason = (f"접촉면 {plan.touches}/6개, "
                  f"{'90도 회전됨' if plan.rotated else '정자세'}, "
                  f"점수 {plan.score:.3f}(낮을수록 좋은 자리)")
        self.box_detail_var.set(
            f"{plan.box_id} · 적재순서 {plan.order} · "
            f"Target=({plan.position[0]:.2f}, {plan.position[1]:.2f}, {plan.position[2]:.2f})m · "
            f"Yaw={plan.target_yaw:.2f}rad\n선정 사유: {reason}"
        )

    def _on_emergency_stop(self):
        """HMI 8절 원칙 #6: 모든 화면에서 접근 가능해야 함 - _set_params_enabled의
        잠금 대상에 이 버튼은 절대 포함시키지 않는다. ⚠️ 이 GUI는 로봇에 직접
        연결되지 않은 로컬 시뮬레이터라, 실제로 멈출 수 있는 건 "이 화면이 만드는
        승인/전송"뿐이다 - 실제 로봇 정지는 MSI2/하드웨어 E-Stop 담당."""
        self._pending_task = None
        self._set_plan_state("NOT_COMPUTED")
        self._append_log(
            "[EMERGENCY STOP] 승인/전송을 즉시 취소했습니다. 이 버튼은 이 화면(Lenovo "
            "Planning HMI)의 승인·전송 게이트만 잠급니다 - 실제 로봇 모터 정지는 "
            "MSI2/하드웨어 E-Stop 담당입니다."
        )
        self.status_var.set("🛑 EMERGENCY STOP - 승인/전송 차단됨")
        messagebox.showwarning(
            "Emergency Stop",
            "이 화면에서 만든 승인/전송이 즉시 취소되었습니다.\n\n"
            "⚠️ 이 버튼은 Lenovo Planning HMI의 승인 게이트만 제어합니다 - "
            "실제 로봇 정지는 MSI2/하드웨어 비상정지가 담당합니다.",
        )

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
        if "side" in self._pil_originals:
            self._show_image(self.side_label, self._pil_originals["side"])

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
