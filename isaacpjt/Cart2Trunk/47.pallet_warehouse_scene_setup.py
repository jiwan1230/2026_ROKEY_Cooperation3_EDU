"""
47.pallet_warehouse_scene_setup.py

시나리오 2(창고 파레트 적재)용 Isaac Sim 씬을 새로 만든다. 원 노트 기준 요구사항:
"입구 개념 없이 사방에서 접근 가능한 평바닥 적재 구역, 공간 활용률(밀도) 최대화, 순서 무관."

크레이트(35.crate_scan_setup.py)와의 핵심 차이
----
크레이트는 벽 4장으로 막힌 컨테이너라 SDF 콜리전이 필요했지만, 파레트 적재는 사방이
뚫린 평바닥이라 벽 자체가 없다 - 파레트 상판을 단순 convexHull 콜리전으로 처리하는 것으로
충분하다(1.load_assets_check.py의 add_rigid_physics()와 동일한 근사 방식, SDF 불필요).

이번 스크립트의 범위
----
이 파일은 "씬 제작"(환경/에셋/박스 배치 + 물리 검증 + 스크린샷)까지만 한다.
32/35.py에 있는 RMPflow 스캔 수렴, ROS2 카메라 브리지, perception 프로세스 연동은
이번 범위 밖이다 - 그건 이 씬이 시각적으로 검증된 뒤 다음 단계(33/36.py 대응 스크립트)에서
따로 진행한다(4.add_boxes_and_zones.py -> 5.fix_zone_layout.py -> (한참 뒤) 32.py 순서로
이 프로젝트가 실제로 밟았던 단계와 동일한 순서).

에셋 (사용자가 ~/Downloads/에 이미 받아둔 것, readme.txt 추천 조합 기준)
----
- 배경: 창고 파래트 적재공간/Low_Poly_Warehouse.usdz (시각용, 물리 콜리전 없음 -
  구조를 모르는 임포트 모델과 로봇/박스가 부딪힐 위험을 원천 차단)
- 파레트: 창고 파래트 적재공간/PALLET_LOW_POLY.usdz (박스가 실제로 얹히는 대상,
  convexHull 콜리전 적용)
- 실제 배치 좌표/스케일은 첫 실행 스크린샷으로 확인 후 다음 반복에서 조정 예정
  (1.load_assets_check.py 때도 카트/차량 배치를 여러 번 스크린샷으로 고쳤던 것과 동일한 흐름).
"""

from isaacsim import SimulationApp

import os

HEADLESS = os.environ.get("HEADLESS", "1") == "1"
_sim_app_config = {"headless": HEADLESS}
if not HEADLESS:
    _sim_app_config.update({"width": 640, "height": 480})
simulation_app = SimulationApp(_sim_app_config)

import time
from pathlib import Path

import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, Gf
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view

_THIS_DIR = Path(__file__).resolve().parent
_DOWNLOADS_DIR = Path.home() / "Downloads" / "창고 파래트 적재공간"

WAREHOUSE_USD = str(_DOWNLOADS_DIR / "Low_Poly_Warehouse.usdz")
PALLET_USD = str(_DOWNLOADS_DIR / "PALLET_LOW_POLY.usdz")

# ---- 배치 파라미터 ----
# WAREHOUSE_POS: 1차 실행에서 (0,0,0)+자동보정(x0.01)으로 넣었더니 미니어처 크기가 나와서
# scale_override=1.0(무보정)으로 바꿈 - 원본 로컬 원점이 bbox 중심이 아니라서(비대칭),
# RotateX(90) 이후 로컬 footprint가 x:[-15.12,10.99] y:[-46.86,6.64] z:[0.72,23.40] 범위로
# 나온다(2차 실행 raw bbox 실측 기반 계산). 이 footprint 중심이 파레트(2,0) 근처에 오고
# 바닥(z=0.72)이 그라운드플레인과 맞닿도록 translate를 역산한 값.
WAREHOUSE_POS = (4.0, 20.0, -0.7)
PALLET_POS = (2.0, 0.0, 0.0)

# ---- 박스 3종 (4.add_boxes_and_zones.py와 동일 규격 - 프로젝트 전체에서 이미 쓰는 값,
# 시나리오가 바뀌어도 "박스 자체 규격"은 그대로 유지해 알고리즘 쪽 가정과 어긋나지 않게 함) ----
BOXES = [
    # name,    size(L,W,H),         color(RGB),          drop_xy(파레트 중심 기준 상대offset)
    ("Small",  (0.30, 0.20, 0.15),  (0.85, 0.25, 0.20),  (-0.25, -0.15)),
    ("Medium", (0.40, 0.30, 0.25),  (0.25, 0.65, 0.30),  (0.05, 0.05)),
    ("Large",  (0.50, 0.35, 0.30),  (0.20, 0.35, 0.85),  (0.25, -0.05)),
]
BOX_MASS_KG = {"Small": 1.0, "Medium": 2.0, "Large": 3.5}
PALLET_DROP_Z = 1.2  # 파레트 상판보다 충분히 높은 곳에서 떨어뜨려 실제로 얹히는지 확인


def inspect_source(usd_path):
    src_stage = Usd.Stage.Open(usd_path)
    mpu = UsdGeom.GetStageMetersPerUnit(src_stage)
    up = UsdGeom.GetStageUpAxis(src_stage)
    print(f"[SOURCE] {Path(usd_path).name}: metersPerUnit={mpu}  upAxis={up}", flush=True)
    return mpu, up


def add_asset(stage, target_mpu, target_up, prim_path, usd_path, position, rot_z=0.0, scale_override=None):
    """1.load_assets_check.py의 add_asset()과 동일한 단위/축 보정 패턴.

    scale_override: 첫 실행에서 Low_Poly_Warehouse.usdz가 metersPerUnit=0.01을 곧이곧대로
    적용하면 파레트보다도 작은 미니어처(0.26x0.53x0.23m)로 로드되는 걸 발견했다 - 원본
    좌표(raw local bbox, 소스 좌표계 기준)를 직접 열어보니 x폭 26.1 / y(height) 22.67 /
    z(depth) 53.5 단위였는데, 이걸 1유닛=1cm로 해석하면 미니어처가, 1유닛=1m로 해석하면
    26x53m 바닥면적에 23m 높이라는 대형 창고 건물로는 아주 그럴듯한 크기가 나온다.
    파레트 쪽은 metersPerUnit=0.01 그대로가 실제 유로파레트 규격(1.2x0.8x0.144m)과 잘
    맞았으므로(=이 보정이 맞았음을 확인) 같은 메타데이터라도 에셋마다 모델링 스케일 관례가
    다를 수 있다는 뜻 - 창고에는 이 파라미터로 수동 보정값(1.0, 즉 무보정)을 강제한다."""
    src_mpu, src_up = inspect_source(usd_path)
    scale = scale_override if scale_override is not None else (src_mpu / target_mpu if target_mpu else src_mpu)

    xform = UsdGeom.Xform.Define(stage, prim_path)
    prim = xform.GetPrim()
    ok = prim.GetReferences().AddReference(usd_path)

    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    if rot_z:
        xform.AddRotateZOp().Set(rot_z)
    if src_up == UsdGeom.Tokens.y and target_up == UsdGeom.Tokens.z:
        xform.AddRotateXOp().Set(90.0)
    xform.AddScaleOp().Set((scale, scale, scale))

    print(f"[REF] {prim_path} AddReference={ok}  scale={scale:.5f}  rotZ={rot_z}", flush=True)
    return xform


def describe_bbox(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    bbox = bbox_cache.ComputeWorldBound(prim)
    rng = bbox.ComputeAlignedRange()
    print(f"[BBOX] {prim_path} min={rng.GetMin()} max={rng.GetMax()}", flush=True)
    return rng


def add_mesh_collision_convexhull(stage, root_prim_path):
    """1.py의 add_rigid_physics()에서 물리(RigidBody/Mass) 부분만 뺀 버전 - 파레트는
    카트/차량처럼 스스로 움직이지 않는 고정 지지대라 RigidBodyAPI 없이 정적 콜리전만
    필요하다(3.fix_container_collision.py의 '카트/차량은 완전 정적' 관례와 동일)."""
    root_prim = stage.GetPrimAtPath(root_prim_path)
    mesh_count = 0
    for prim in Usd.PrimRange(root_prim):
        if prim.GetTypeName() == "Mesh":
            UsdPhysics.CollisionAPI.Apply(prim)
            mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mesh_collision.CreateApproximationAttr().Set("convexHull")
            mesh_count += 1
    print(f"[PHYSICS] {root_prim_path} 정적 convexHull 콜리전 {mesh_count}개 메쉬에 적용", flush=True)
    return mesh_count


def add_dynamic_box(stage, prim_path, center, size, color, mass_kg):
    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube.GetSizeAttr().Set(1.0)
    xform = UsdGeom.Xformable(cube)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*center))
    xform.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(mass_kg)
    print(f"[BOX] {prim_path} size={size} mass={mass_kg}kg drop_center={center}", flush=True)
    return prim


def add_zone_marker(stage, prim_path, center_xy, size_xy, color, z):
    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube.GetSizeAttr().Set(1.0)
    xform = UsdGeom.Xformable(cube)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(center_xy[0], center_xy[1], z))
    xform.AddScaleOp().Set(Gf.Vec3f(size_xy[0], size_xy[1], 0.012))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    print(f"[ZONE] {prim_path} center={center_xy} size={size_xy}", flush=True)
    return cube.GetPrim()


# ================= 씬 구성 =================
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

target_mpu = UsdGeom.GetStageMetersPerUnit(stage)
target_up = UsdGeom.GetStageUpAxis(stage)
print(f"[STAGE] metersPerUnit={target_mpu}  upAxis={target_up}", flush=True)

add_asset(stage, target_mpu, target_up, "/World/Warehouse", WAREHOUSE_USD, WAREHOUSE_POS, scale_override=1.0)
add_asset(stage, target_mpu, target_up, "/World/Pallet", PALLET_USD, PALLET_POS)

for _ in range(30):
    simulation_app.update()

describe_bbox(stage, "/World/Warehouse")
pallet_bbox = describe_bbox(stage, "/World/Pallet")

add_mesh_collision_convexhull(stage, "/World/Pallet")
# 창고 배경은 의도적으로 콜리전을 안 붙인다(순수 시각용 - 구조를 모르는 임포트 모델이
# 로봇/박스와 부딪혀 스폰 즉시 밀려나는 사고를 원천 차단, HANDOFF.md 5-1절에서 겪었던
# "단일 메쉬에 잘못된 콜리전을 붙여 물체가 튕겨나가는" 문제의 재발 방지).

world.reset()
for _ in range(10):
    world.step(render=True)

viewport = vp_util.get_active_viewport()


def snapshot(eye, target, fname):
    set_camera_view(eye=eye, target=target)
    for _ in range(20):
        world.step(render=True)
    out = str(_THIS_DIR / fname)
    vp_util.capture_viewport_to_file(viewport, out)
    for _ in range(20):
        world.step(render=True)
    print(f"[SCREENSHOT] {out}", flush=True)


# 조기 레이아웃 확인 - 박스를 넣기 전에 창고/파레트 배치·스케일이 말이 되는지 먼저 본다
# (35.py의 "_verify_crate_scan_00_layout.png" 조기 확인 관례와 동일한 이유: 겹침/스케일
# 문제를 뒤에 가서 발견하면 낭비가 크다).
snapshot(eye=[PALLET_POS[0] - 3.0, PALLET_POS[1] - 3.0, 3.0], target=[PALLET_POS[0], PALLET_POS[1], 0.3],
         fname="_verify_pallet_00_layout.png")

pallet_top_z = float(pallet_bbox.GetMax()[2])
print(f"[PALLET] 측정된 상판 z={pallet_top_z:.4f}m (박스 드롭 기준)", flush=True)

for name, size, color, (dx, dy) in BOXES:
    center = (PALLET_POS[0] + dx, PALLET_POS[1] + dy, PALLET_DROP_Z)
    add_dynamic_box(stage, f"/World/Box_{name}", center, size, color, BOX_MASS_KG[name])
    for _ in range(100):
        world.step(render=True)
    print(f"[낙하 완료] Box_{name}", flush=True)

# 파레트 적재 구역 표시 (알고리즘의 "Trunk" 바운딩 영역과 대응 - 벽이 없는 개방형이라
# 물리 벽 대신 시각 마커로만 경계를 보여준다)
add_zone_marker(stage, "/World/Zone_PalletLoadingArea", (PALLET_POS[0], PALLET_POS[1]),
                 (1.3, 0.9), (0.20, 0.80, 0.30), 0.015)

world.reset()
for _ in range(10):
    world.step(render=True)

xform_cache = UsdGeom.XformCache()
for name, *_ in BOXES:
    prim = stage.GetPrimAtPath(f"/World/Box_{name}")
    pos = xform_cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    print(f"[RESULT] Box_{name} final_pos=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})", flush=True)

snapshot(eye=[PALLET_POS[0] - 1.5, PALLET_POS[1] - 1.8, 1.6], target=[PALLET_POS[0], PALLET_POS[1], 0.3],
         fname="_verify_pallet_01_boxes_close.png")
snapshot(eye=[PALLET_POS[0] - 0.2, PALLET_POS[1] - 0.2, 5.0], target=[PALLET_POS[0], PALLET_POS[1], 0.0],
         fname="_verify_pallet_02_top.png")

scene_path = str(_THIS_DIR / "pallet_warehouse_scene.usd")
omni.usd.get_context().save_as_stage(scene_path)
print(f"[저장] {scene_path}", flush=True)

print("\n[완료] 47.pallet_warehouse_scene_setup.py 끝.\n", flush=True)

if HEADLESS:
    simulation_app.close()
else:
    print("[안내] 창을 직접 둘러보세요 - 닫으면 스크립트가 종료됩니다.\n", flush=True)
    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.01)
    simulation_app.close()
