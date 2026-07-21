"""
31.collect_mobile_manipulator_lift.py
28번 스크립트에서 검증 완료된 "Nova Carter + M0609(+VGP20+카메라) + 승강 리프트" 조합을
독립된 재사용 가능한 자기완결(self-contained) USD 에셋으로 저장한다.

[먼저 알아둘 것 - 뭐가 저장되고 뭐가 안 되는지]
저장되는 것: Nova Carter 참조, M0609(+VGP20+카메라) 참조, chassis_link<->base_link
FilteredPairsAPI 충돌 필터링, base_link의 독립 ArticulationRootAPI, 시각용 텔레스코핑
리프트 원기둥 - 즉 "조립된 하드웨어 구성"까지는 전부 USD 구조로 남는다.
저장 안 되는 것: 리프트를 실제로 올리고 내리는 동작(set_lift_height 등 매 프레임 텔레포트
로직), 마운트 XY 오프셋 계산, PICK/PLACE 시퀀스 - 이런 "제어"는 항상 그것대로 파이썬
스크립트가 맡아야 한다(관절 각도 명령과 똑같은 영역). 이 스크립트가 저장하는 건 "차체
위에 리프트+팔이 이미 조립되어 있는 상태"까지고, 그걸 움직이는 코드는 28번 스크립트처럼
따로 필요하다.

절차: (1) 28번의 build_mobile_manipulator_with_lift()를 그대로 재사용해 로봇을 조립하고
원점에 둔 채로 스테이지를 로컬 usd 파일로 export, (2) 10번 스크립트(M0609+카메라 collect)
와 동일한 omni.kit.usd.collect.Collector로 그 파일을 self-contained 폴더로 묶는다
(Nova Carter의 nucleus 원격 에셋을 로컬로 끌어와 복사).
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import asyncio
import time
from pathlib import Path

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import get_assets_root_path

_THIS_DIR = Path(__file__).resolve().parent
M0609_DIR = _THIS_DIR.parent / "M0609"
M0609_USD = str(M0609_DIR / "Collected_m0609_vgp20_camera" / "m0609_vgp20_camera.usd")

STAGING_DIR = _THIS_DIR / "_staging_mobile_manipulator_lift"
STAGING_USD = str(STAGING_DIR / "mobile_manipulator_lift.usd")
TARGET_DIR = str(_THIS_DIR / "Collected_mobile_manipulator_lift")
FINAL_USD_NAME = "mobile_manipulator_lift.usd"

LIFT_MIN = 0.42
LIFT_COLUMN_RADIUS = 0.06
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e8, 1e4, 1e8


def add_drive_stiffness(stage, root_path):
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        for dof_type in ["angular", "linear"]:
            drive = UsdPhysics.DriveAPI.Get(prim, dof_type)
            if drive:
                drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                drive.GetDampingAttr().Set(DRIVE_DAMPING)
                drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                n += 1
    return n


def build_mobile_manipulator_with_lift(stage, start_xy):
    """28번 스크립트와 동일 - Nova Carter chassis_link와 M0609 base_link를 독립
    articulation 두 개로 분리 + 충돌 필터링 + 시각용 리프트 원기둥."""
    root = get_assets_root_path()
    carter_url = root + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"

    carter_path = "/World/MobileManipulator/NovaCarter"
    carter_xform = UsdGeom.Xform.Define(stage, carter_path)
    carter_xform.GetPrim().GetReferences().AddReference(carter_url)
    carter_xform.ClearXformOpOrder()
    carter_xform.AddTranslateOp().Set(Gf.Vec3d(start_xy[0], start_xy[1], 0.0))
    chassis_link_path = f"{carter_path}/chassis_link"

    m0609_path = "/World/MobileManipulator/M0609"
    m0609_xform = UsdGeom.Xform.Define(stage, m0609_path)
    m0609_xform.GetPrim().GetReferences().AddReference(M0609_USD, "/World/m0609")
    m0609_xform.ClearXformOpOrder()
    m0609_xform.AddTranslateOp().Set(Gf.Vec3d(start_xy[0], start_xy[1], LIFT_MIN))

    for _ in range(20):
        simulation_app.update()

    base_link_path = f"{m0609_path}/base_link"
    old_root_joint_path = f"{m0609_path}/root_joint"
    if stage.GetPrimAtPath(old_root_joint_path).IsValid():
        stage.RemovePrim(old_root_joint_path)

    base_link_prim = stage.GetPrimAtPath(base_link_path)
    UsdPhysics.ArticulationRootAPI.Apply(base_link_prim)

    chassis_link_prim = stage.GetPrimAtPath(chassis_link_path)
    filt_chassis = UsdPhysics.FilteredPairsAPI.Apply(chassis_link_prim)
    filt_chassis.CreateFilteredPairsRel().AddTarget(Sdf.Path(base_link_path))
    filt_base = UsdPhysics.FilteredPairsAPI.Apply(base_link_prim)
    filt_base.CreateFilteredPairsRel().AddTarget(Sdf.Path(chassis_link_path))
    print(f"[필터] {chassis_link_path} <-> {base_link_path} 충돌 필터링 적용", flush=True)

    lift_column_path = "/World/MobileManipulator/LiftColumnVisual"
    lift_column = UsdGeom.Cylinder.Define(stage, lift_column_path)
    lift_column.CreateRadiusAttr().Set(LIFT_COLUMN_RADIUS)
    lift_column.CreateHeightAttr().Set(1.0)
    lift_column.CreateAxisAttr("Z")
    lift_column.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.45, 0.1)])
    lift_column_xform = UsdGeom.Xformable(lift_column)
    lift_column_xform.ClearXformOpOrder()
    lift_column_xform.AddTranslateOp().Set(Gf.Vec3d(start_xy[0], start_xy[1], LIFT_MIN / 2.0))
    lift_column_xform.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, LIFT_MIN))

    stray_prim = stage.GetPrimAtPath(f"{m0609_path}/onrobot_rg2ft/world")
    if stray_prim.IsValid() and stray_prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
        stray_prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)

    n = add_drive_stiffness(stage, m0609_path)
    print(f"[DRIVE] M0609={n}개 조인트 강성 적용 (NovaCarter는 원래 값 그대로 유지)", flush=True)

    return carter_path, chassis_link_path, m0609_path, base_link_path


# ================= 1단계: 조립 + 로컬 usd로 export =================
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

build_mobile_manipulator_with_lift(stage, (0.0, 0.0))

STAGING_DIR.mkdir(parents=True, exist_ok=True)
stage.GetRootLayer().Export(STAGING_USD)
print(f"[EXPORT] {STAGING_USD}", flush=True)

# ================= 2단계: Collect로 self-contained 폴더로 묶기 (10번 스크립트와 동일 패턴) =================
enable_extension("omni.kit.usd.collect")
simulation_app.update()

from omni.kit.usd.collect.collector import Collector  # noqa: E402

GRID_ENV_PREFIX = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Environments/Grid/"

collector = Collector(
    STAGING_USD,
    TARGET_DIR,
    usd_only=False,
    flat_collection=False,
    skip_existing=False,
    exclusion_rules={GRID_ENV_PREFIX: None},
)


def on_progress(current, total):
    print(f"[진행] {current}/{total}", flush=True)


async def _run_collect():
    return await collector.collect(progress_callback=on_progress)


print(f"[COLLECT] {STAGING_USD} -> {TARGET_DIR}", flush=True)
task = asyncio.ensure_future(_run_collect())
last_report = time.time()
while not task.done():
    simulation_app.update()
    if time.time() - last_report > 5.0:
        last_report = time.time()
        print("[대기] collect 진행 중...", flush=True)
success, root_usd = task.result()
print(f"[COLLECT 결과] success={success} status={collector.get_status()} root_usd={root_usd}", flush=True)
assert success, "Collect 실패"

root_path = Path(root_usd)
print(f"[확인] 저장된 루트 usd 파일명: {root_path.name} (기대값: {FINAL_USD_NAME})", flush=True)
print(f"\n[안내] 완료 - 재사용 시 {TARGET_DIR}/{root_path.name}을 AddReference 하면 됨.\n", flush=True)

simulation_app.close()
