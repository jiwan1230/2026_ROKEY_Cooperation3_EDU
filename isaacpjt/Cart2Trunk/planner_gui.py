"""
planner_gui.py
trunk_map_planner_node.py를 매번 터미널 명령어로 실행하는 대신, tkinter 창
하나로 "트렁크 스캔 파일 + 카트 박스 + 모드 + 마진"을 고르고 [실행] 누르면
비포/애프터 그림을 바로 볼 수 있게 만든 GUI.

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
"""

import json
import pathlib
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


def _discover_trunk_maps() -> list:
    """/home/sunwook/cobot3_ws/src/run_*/pointcloud/trunk_map.json 전부 찾기."""
    return sorted(_SRC_DIR.glob("run_*/pointcloud/trunk_map.json"))


def _discover_box_presets() -> dict:
    """local_test_data/example_cart_boxes_*.json 전부 찾아서 {표시이름: 경로} 딕셔너리로."""
    presets = {"기본값 (Large/Medium/Small)": None}
    for f in sorted(_LOCAL_TEST_DATA_DIR.glob("example_cart_boxes_*.json")):
        presets[f.stem.replace("example_cart_boxes_", "")] = f
    return presets


class PlannerGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Cart2Trunk 적재 알고리즘 - 비포/애프터 시뮬레이터")
        self.geometry("1500x900")

        self._trunk_maps = _discover_trunk_maps()
        self._box_presets = _discover_box_presets()
        self._tk_images = {}  # PhotoImage 참조 유지용 (안 하면 가비지 컬렉션으로 사라짐)

        self._build_controls()
        self._build_result_area()

    # ------------------------------------------------------------------ UI

    def _build_controls(self):
        frame = ttk.Frame(self, padding=10)
        frame.pack(side="top", fill="x")

        ttk.Label(frame, text="트렁크 스캔 파일:").grid(row=0, column=0, sticky="w")
        self.trunk_map_var = tk.StringVar()
        trunk_map_names = [str(p.parent.parent.name) for p in self._trunk_maps]
        self.trunk_map_combo = ttk.Combobox(frame, textvariable=self.trunk_map_var,
                                             values=trunk_map_names, width=30, state="readonly")
        if trunk_map_names:
            self.trunk_map_combo.current(len(trunk_map_names) - 1)  # 최신 run 기본 선택
        self.trunk_map_combo.grid(row=0, column=1, sticky="w", padx=5)

        ttk.Label(frame, text="카트 박스 목록:").grid(row=0, column=2, sticky="w", padx=(20, 0))
        self.box_preset_var = tk.StringVar(value=next(iter(self._box_presets)))
        self.box_preset_combo = ttk.Combobox(frame, textvariable=self.box_preset_var,
                                              values=list(self._box_presets.keys()), width=25, state="readonly")
        self.box_preset_combo.current(0)
        self.box_preset_combo.bind("<<ComboboxSelected>>", self._on_preset_selected)
        self.box_preset_combo.grid(row=0, column=3, sticky="w", padx=5)

        ttk.Label(frame, text="적재 모드:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        self.mode_var = tk.StringVar(value="large_first")
        mode_frame = ttk.Frame(frame)
        ttk.Radiobutton(mode_frame, text="큰 거 우선 (large_first)", variable=self.mode_var,
                         value="large_first").pack(side="left")
        ttk.Radiobutton(mode_frame, text="개수 우선 (count_first)", variable=self.mode_var,
                         value="count_first").pack(side="left", padx=(10, 0))
        mode_frame.grid(row=1, column=1, columnspan=2, sticky="w", pady=(8, 0))

        ttk.Label(frame, text=f"마진(m, 기본 {DEFAULT_MARGIN}):").grid(row=1, column=3, sticky="w", pady=(8, 0))
        self.margin_var = tk.StringVar(value="")
        ttk.Entry(frame, textvariable=self.margin_var, width=8).grid(row=1, column=4, sticky="w", padx=5, pady=(8, 0))

        ttk.Label(frame, text="박스 목록 (JSON, 직접 수정 가능):").grid(row=2, column=0, sticky="nw", pady=(8, 0))
        self.box_text = tk.Text(frame, width=90, height=5)
        self.box_text.grid(row=2, column=1, columnspan=4, sticky="w", pady=(8, 0))
        self._on_preset_selected()

        run_button = ttk.Button(frame, text="▶ 실행", command=self._run)
        run_button.grid(row=3, column=0, pady=10, sticky="w")

        self.status_var = tk.StringVar(value="준비됨")
        ttk.Label(frame, textvariable=self.status_var, foreground="#2c3e50").grid(
            row=3, column=1, columnspan=4, sticky="w", pady=10
        )

    def _build_result_area(self):
        frame = ttk.Frame(self, padding=10)
        frame.pack(side="top", fill="both", expand=True)

        before_frame = ttk.LabelFrame(frame, text="Before (아직 안 실음)")
        before_frame.pack(side="left", fill="both", expand=True, padx=5)
        self.before_label = ttk.Label(before_frame)
        self.before_label.pack(fill="both", expand=True)

        after_frame = ttk.LabelFrame(frame, text="After (배치 결과)")
        after_frame.pack(side="left", fill="both", expand=True, padx=5)
        self.after_label = ttk.Label(after_frame)
        self.after_label.pack(fill="both", expand=True)

        log_frame = ttk.LabelFrame(self, text="결과 로그", padding=5)
        log_frame.pack(side="bottom", fill="x", padx=10, pady=(0, 10))
        self.log_text = tk.Text(log_frame, height=8, font=("monospace", 9))
        self.log_text.pack(fill="x")

    # -------------------------------------------------------------- 동작

    def _on_preset_selected(self, event=None):
        preset_path = self._box_presets[self.box_preset_var.get()]
        boxes = json.loads(preset_path.read_text()) if preset_path else _DEFAULT_CART_BOXES
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

        self._show_image(self.before_label, before_path)
        self._show_image(self.after_label, after_path)

        log_lines = [f"[{run_name}] mode={mode}, margin={effective_margin:.2f}m -> {len(plans)}/{len(cart_boxes_raw)}개 배치"]
        for p in plans:
            log_lines.append(f"  PLACED {p.box_id}: pos=({p.position[0]:.2f},{p.position[1]:.2f},{p.position[2]:.2f}) rotated={p.rotated}")
        for u in unloadable:
            log_lines.append(f"  UNLOADABLE {u.box_id}: {u.reason.value}")
        self.log_text.delete("1.0", "end")
        self.log_text.insert("1.0", "\n".join(log_lines))

        self.status_var.set(f"완료 - {len(plans)}/{len(cart_boxes_raw)}개 배치")

    def _show_image(self, label: ttk.Label, path: pathlib.Path):
        img = Image.open(path)
        max_w = 700
        ratio = max_w / img.width
        img = img.resize((max_w, int(img.height * ratio)))
        photo = ImageTk.PhotoImage(img)
        label.configure(image=photo)
        self._tk_images[str(label)] = photo  # 참조 유지


if __name__ == "__main__":
    app = PlannerGUI()
    app.mainloop()
