"""
sketch_placement_test_loading_mode_comparison.py
"큰 거 우선"(large_first, 기본값) vs "개수 우선"(count_first) 적재 모드를
같은 트렁크/같은 박스 목록으로 나란히 검증하고 비포/애프터 그림으로 남긴다.

트렁크를 빠듯하게(0.6x0.4x0.45) 잡고 소형 6개+대형 2개를 섞어서, 두 모드가
실제로 다른 결과(적재 개수)를 내는 걸 보여준다 - 이미 회귀 테스트로 확인된
수치(4/8 -> 7/8)를 여기서 그림으로 다시 검증한다.
"""
import sys
import pathlib
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from _viz_helpers import SceneBox, draw_scene

_ALGORISM_DIR = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_ALGORISM_DIR))
m02 = import_module("02_trunk_space_state")
m03 = import_module("03_extreme_point_candidates")
m08 = import_module("08_unloadable_reason")

Trunk = m02.Trunk
Box = m03.Box
generate_loading_plan = m08.generate_loading_plan

TRUNK = Trunk(width=0.6, depth=0.4, height=0.45)  # ⑮ 상단 여유(0.2m) 감안
SMALL_COLOR = "#3498db"
BIG_COLOR = "#e67e22"

boxes = (
    [Box(f"소형{i+1}", 0.1, 0.1, 0.1) for i in range(6)]
    + [Box(f"대형{i+1}", 0.3, 0.2, 0.2) for i in range(2)]
)
color_of = {b.id: (SMALL_COLOR if b.id.startswith("소형") else BIG_COLOR) for b in boxes}


def _waiting_scene_boxes():
    return [SceneBox(b.id, 0, 0, 0, b.width, b.depth, b.height, color_of[b.id]) for b in boxes]


def _placed_scene_boxes(plans):
    return [
        SceneBox(p.box_id, p.position[0], p.position[1], p.position[2],
                 p.dimensions[0], p.dimensions[1], p.dimensions[2], color_of[p.box_id])
        for p in plans
    ]


# ---- before (공통 - 아직 아무것도 안 실은 상태, 소형6+대형2 전부 대기 중) ----
draw_scene(
    TRUNK.width, TRUNK.depth, TRUNK.height,
    fixed_obstacles=[], placed_boxes=[], waiting_boxes=_waiting_scene_boxes(),
    title="적재 모드 비교 - Before (소형 6개 + 대형 2개, 아직 안 실음)",
    out_path=str(pathlib.Path(__file__).parent / "sketch_loading_mode_before.png"),
)

# ---- after: large_first(기본값) ----
plans_large_first, unloadable_large_first = generate_loading_plan(boxes, TRUNK, mode="large_first")
draw_scene(
    TRUNK.width, TRUNK.depth, TRUNK.height,
    fixed_obstacles=[], placed_boxes=_placed_scene_boxes(plans_large_first),
    waiting_boxes=[SceneBox(u.box_id, 0, 0, 0, next(b for b in boxes if b.id == u.box_id).width,
                             next(b for b in boxes if b.id == u.box_id).depth,
                             next(b for b in boxes if b.id == u.box_id).height, color_of[u.box_id])
                   for u in unloadable_large_first],
    title=f"After - large_first(기본, 큰 거 우선) : {len(plans_large_first)}/{len(boxes)}개 적재",
    out_path=str(pathlib.Path(__file__).parent / "sketch_loading_mode_large_first_after.png"),
)

# ---- after: count_first ----
plans_count_first, unloadable_count_first = generate_loading_plan(boxes, TRUNK, mode="count_first")
draw_scene(
    TRUNK.width, TRUNK.depth, TRUNK.height,
    fixed_obstacles=[], placed_boxes=_placed_scene_boxes(plans_count_first),
    waiting_boxes=[SceneBox(u.box_id, 0, 0, 0, next(b for b in boxes if b.id == u.box_id).width,
                             next(b for b in boxes if b.id == u.box_id).depth,
                             next(b for b in boxes if b.id == u.box_id).height, color_of[u.box_id])
                   for u in unloadable_count_first],
    title=f"After - count_first(개수 우선) : {len(plans_count_first)}/{len(boxes)}개 적재",
    out_path=str(pathlib.Path(__file__).parent / "sketch_loading_mode_count_first_after.png"),
)

print(f"\n=== 요약 ===")
print(f"large_first: {len(plans_large_first)}/{len(boxes)}개 적재, 미적재: {[u.box_id for u in unloadable_large_first]}")
print(f"count_first: {len(plans_count_first)}/{len(boxes)}개 적재, 미적재: {[u.box_id for u in unloadable_count_first]}")
