"""
93.probe_trunk_entrance_geometry.py

92.trunk_place_holonomic.py의 STAGE 2->3 전환에서 계속 충돌이 나서, "박스+그리퍼가 물리적으로
트렁크 입구를 안 부딪히고 지나갈 공간이 실제로 있는지"를 직접 raycast로 측정해서 확인한다.
로봇/홀로노믹 베이스는 전혀 스폰하지 않고 차량 메시만 92.py와 똑같은 CAR_POS/CAR_EXTRA_SCALE/
CAR_ROT_Z로 로드한 뒤, 입구 근방 (x, y) 그리드에서 위->아래(천장/뚜껑면), 아래->위(바닥/문턱면)
raycast를 쏴서 실측 수직 개구부 높이를 뽑는다. 92.py의 TRUNK_FLOOR_Z/TRUNK_WALL_TOP은 트렁크
"안쪽"에서 실측된 값이라, 입구 자체의 좁아지는 지점(문턱/뚜껑 프레임)의 진짜 여유 공간과는
다를 수 있다는 게 지금까지 반복된 버그의 근본 원인이었다(TRUNK_ENTRANCE_X 자체도 그래서
TRUNK_X_MIN에서 따로 분리해야 했음) - 이번엔 그 부분을 추측 대신 직접 재본다.
"""

from isaacsim import SimulationApp

import os

HEADLESS = os.environ.get("HEADLESS", "1") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

import numpy as np
import omni.usd
from omni.physx import get_physx_scene_query_interface
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf

from isaacsim.core.api import World
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent

# ---------------- 92.py와 완전히 동일 - 차량 실측 상수 ----------------
CAR_USD = str(_THIS_DIR / "assets/Lexus_IS300_Trunk_Open_No_More_Hell_Room.usdz")
CAR_POS = (5.0, 0.0, 0.0)
CAR_EXTRA_SCALE = 0.50
CAR_ROT_Z = 0.0
TRUNK_X_MIN, TRUNK_X_MAX = 3.11, 3.68
TRUNK_Y_MIN, TRUNK_Y_MAX = -0.56, 0.56
TRUNK_FLOOR_Z = 0.44
TRUNK_WALL_TOP = 1.28
TRUNK_ENTRANCE_X = TRUNK_X_MIN - 0.15
SDF_RESOLUTION = 256

# 박스/그리퍼 실측 상수(92.py와 동일) - 여기와 비교할 "실제로 필요한 수직 공간".
TEST_BOX_SIZE = (0.135, 0.177, 0.106)
TIP_LOCAL_OFFSET_Z = 0.0188


def add_asset(stage, prim_path, usd_path, position, extra_scale, target_mpu, target_up, rot_z=0.0):
    src_stage = Usd.Stage.Open(usd_path)
    src_mpu = UsdGeom.GetStageMetersPerUnit(src_stage)
    src_up = UsdGeom.GetStageUpAxis(src_stage)
    scale = (src_mpu / target_mpu if target_mpu else src_mpu) * extra_scale
    xform = UsdGeom.Xform.Define(stage, prim_path)
    prim = xform.GetPrim()
    prim.GetReferences().AddReference(usd_path)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(position)
    if rot_z:
        xform.AddRotateZOp().Set(rot_z)
    if src_up == UsdGeom.Tokens.y and target_up == UsdGeom.Tokens.z:
        xform.AddRotateXOp().Set(90.0)
    xform.AddScaleOp().Set((scale, scale, scale))
    return xform


def add_sdf_collision(stage, root_prim_path, sdf_resolution=SDF_RESOLUTION):
    root_prim = stage.GetPrimAtPath(root_prim_path)
    n = 0
    for prim in Usd.PrimRange(root_prim):
        if prim.GetTypeName() == "Mesh":
            UsdPhysics.CollisionAPI.Apply(prim)
            mc = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mc.CreateApproximationAttr().Set("sdf")
            sdf_api = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
            sdf_api.CreateSdfResolutionAttr().Set(sdf_resolution)
            n += 1
    print(f"[SDF] {root_prim_path}: {n} mesh", flush=True)


world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
target_mpu = UsdGeom.GetStageMetersPerUnit(stage)
target_up = UsdGeom.GetStageUpAxis(stage)

add_asset(stage, "/World/Vehicle", CAR_USD, Gf.Vec3d(*CAR_POS), CAR_EXTRA_SCALE, target_mpu, target_up, rot_z=CAR_ROT_Z)
add_sdf_collision(stage, "/World/Vehicle")

world.reset()
# SDF 베이크가 끝날 시간을 준다(92.py 본 파이프라인도 실제 충돌이 걸리기 전까지 이 정도
# 스텝은 항상 지나가므로 동일 조건).
for _ in range(60):
    world.step(render=False)


def raycast_down(x, y, z_start=2.5, max_dist=4.0):
    hit = get_physx_scene_query_interface().raycast_closest(
        carb_vec(x, y, z_start), carb_vec(0.0, 0.0, -1.0), max_dist)
    if hit["hit"]:
        return float(hit["position"][2])
    return None


def raycast_up(x, y, z_start=-0.5, max_dist=4.0):
    hit = get_physx_scene_query_interface().raycast_closest(
        carb_vec(x, y, z_start), carb_vec(0.0, 0.0, 1.0), max_dist)
    if hit["hit"]:
        return float(hit["position"][2])
    return None


def carb_vec(x, y, z):
    return Gf.Vec3f(float(x), float(y), float(z))


# 입구 근방(TRUNK_ENTRANCE_X 조금 앞 ~ TRUNK_X_MIN 조금 뒤)과 박스 폭을 덮는 Y 범위를 그리드로 스캔.
xs = np.arange(TRUNK_ENTRANCE_X - 0.05, TRUNK_X_MIN + 0.30 + 1e-9, 0.02)
half_box_y = max(TEST_BOX_SIZE[0], TEST_BOX_SIZE[1]) / 2.0
ys = np.arange(-half_box_y - 0.05, half_box_y + 0.05 + 1e-9, 0.02)

print(f"\n[스캔 범위] x=[{xs[0]:.3f},{xs[-1]:.3f}] (TRUNK_ENTRANCE_X={TRUNK_ENTRANCE_X:.3f}, "
      f"TRUNK_X_MIN={TRUNK_X_MIN:.3f}) y=[{ys[0]:.3f},{ys[-1]:.3f}]", flush=True)

worst_opening = None
worst_xy = None
worst_floor = None
worst_ceiling = None
per_x_min_opening = []

for x in xs:
    row_min = None
    for y in ys:
        ceiling_z = raycast_down(x, y)
        floor_z = raycast_up(x, y)
        if ceiling_z is None or floor_z is None:
            continue
        opening = ceiling_z - floor_z
        if row_min is None or opening < row_min:
            row_min = opening
        if worst_opening is None or opening < worst_opening:
            worst_opening = opening
            worst_xy = (float(x), float(y))
            worst_floor = floor_z
            worst_ceiling = ceiling_z
    per_x_min_opening.append((float(x), row_min))

print("\n[x별 최소 수직 개구부(그 x에서 y를 훑었을 때 가장 좁은 지점)]", flush=True)
for x, opening in per_x_min_opening:
    tag = " <-- entrance" if abs(x - TRUNK_ENTRANCE_X) < 0.011 else (" <-- X_MIN" if abs(x - TRUNK_X_MIN) < 0.011 else "")
    if opening is None:
        print(f"  x={x:.3f}: (raycast 실패 - 열린 공간이거나 메시 누락){tag}", flush=True)
    else:
        print(f"  x={x:.3f}: opening={opening:.3f}m{tag}", flush=True)

print(f"\n[최악 지점] xy={worst_xy} floor_z={worst_floor:.3f} ceiling_z={worst_ceiling:.3f} "
      f"opening={worst_opening:.3f}m", flush=True)

box_h = TEST_BOX_SIZE[2]
required_bare = box_h
required_with_margin = box_h + 0.03 + 0.03  # 위/아래 각 3cm 안전마진(그리퍼 팁 오프셋 포함 여유)
print(f"\n[필요 공간] 박스 높이={box_h:.3f}m, 위아래 3cm씩 마진 포함 필요치={required_with_margin:.3f}m "
      f"(그리퍼 팁 오프셋 {TIP_LOCAL_OFFSET_Z:.4f}m은 이미 박스 상단=팁 위치 기준이라 별도 가산 불필요)", flush=True)

if worst_opening is None:
    print("\n[결론] 유효한 raycast 결과가 없습니다 - 스캔 범위/차량 위치를 재확인하세요.", flush=True)
elif worst_opening >= required_with_margin:
    print(f"\n[결론] 물리적으로 여유 있음 - 최악 지점에서도 {worst_opening:.3f}m 확보되어 "
          f"필요치({required_with_margin:.3f}m)를 넘습니다. 지금 충돌은 공간 부족이 아니라 "
          "접근 경로/자세(팔 자세, 진입 높이 목표값) 쪽 문제일 가능성이 높습니다.", flush=True)
else:
    print(f"\n[결론] 공간이 빠듯하거나 부족함 - 최악 지점 개구부({worst_opening:.3f}m)가 "
          f"필요치({required_with_margin:.3f}m)보다 작습니다. 박스를 눕히거나 더 얇은 방향으로 "
          "돌리거나, 그리퍼 진입 높이/자세를 다시 설계해야 할 수 있습니다.", flush=True)

simulation_app.close()
