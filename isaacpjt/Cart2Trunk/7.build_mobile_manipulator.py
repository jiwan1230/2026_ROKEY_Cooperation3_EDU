"""
Nova Carter(AMR 베이스) + M0609(팔+그리퍼)를 결합해서 Final_Mobile_Manipulator를 만든다.
(RidgebackFranka는 폐기 — 팔이 이미 하나의 스키마로 융합돼 있어서 M0609로 교체하려면
재조립 수술이 필요했음. Nova Carter는 베이스 전용 에셋이라 그럴 필요 없음.)

구조:
  /World/MobileManipulator/NovaCarter  <- Isaac Sim 표준 에셋 (실제 차동구동 물리, ArticulationRoot)
  /World/MobileManipulator/M0609       <- isaacpjt/M0609의 기존 그리퍼 포함 팔, chassis_link 위에 배치

결합 방법: M0609의 root_joint(원래 "world에 고정"용 FixedJoint + ArticulationRoot)에서
ArticulationRootAPI를 떼어내고, body0을 Nova Carter의 chassis_link로 재지정해서 하나의
articulation으로 합친다 (한 물리 트리에 ArticulationRoot는 하나만 있어야 함).

Robot Waiting Zone(-0.6, -2.65) 위치에 배치하고 물리 검증 + 스크린샷 확인 후 저장한다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import time

import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path

_THIS_DIR = Path(__file__).resolve().parent
SCENE_USD = str(_THIS_DIR / "cart2trunk_scene.usd")
M0609_USD = str(_THIS_DIR.parent / "M0609" / "Collected_m0609_camera" / "m0609_camera.usd")

ROBOT_XY = (-0.6, -2.65)  # Robot Waiting Zone 중심
CARTER_Z = 0.0
MOUNT_Z = 0.42  # Nova Carter chassis_link 윗면 추정 높이 (스크린샷으로 검증 완료)
FACE_ROT_Z = 37.0  # Robot Waiting Zone -> Loading Working Zone 방향을 바라보도록 회전

omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    simulation_app.update()

world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

root = get_assets_root_path()
carter_url = root + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"

# ---- Nova Carter 배치 ----
carter_path = "/World/MobileManipulator/NovaCarter"
carter_xform = UsdGeom.Xform.Define(stage, carter_path)
ok1 = carter_xform.GetPrim().GetReferences().AddReference(carter_url)
carter_xform.ClearXformOpOrder()
carter_xform.AddTranslateOp().Set(Gf.Vec3d(ROBOT_XY[0], ROBOT_XY[1], CARTER_Z))
carter_xform.AddRotateZOp().Set(FACE_ROT_Z)
print(f"[NOVA-CARTER] ref={ok1} pos=({ROBOT_XY[0]},{ROBOT_XY[1]},{CARTER_Z}) rotZ={FACE_ROT_Z}", flush=True)

# ---- M0609 배치 (Nova Carter chassis 위, 같은 XY/회전, Z만 위로 - 한 몸처럼 같이 회전) ----
m0609_path = "/World/MobileManipulator/M0609"
m0609_xform = UsdGeom.Xform.Define(stage, m0609_path)
ok2 = m0609_xform.GetPrim().GetReferences().AddReference(M0609_USD, "/World/m0609")
m0609_xform.ClearXformOpOrder()
m0609_xform.AddTranslateOp().Set(Gf.Vec3d(ROBOT_XY[0], ROBOT_XY[1], CARTER_Z + MOUNT_Z))
m0609_xform.AddRotateZOp().Set(FACE_ROT_Z)
print(f"[M0609] ref={ok2} pos=({ROBOT_XY[0]},{ROBOT_XY[1]},{CARTER_Z + MOUNT_Z}) rotZ={FACE_ROT_Z}", flush=True)

for _ in range(20):
    simulation_app.update()

# ---- 두 articulation을 하나로 합치기 ----
chassis_link_path = f"{carter_path}/chassis_link"
root_joint_path = f"{m0609_path}/root_joint"

chassis_prim = stage.GetPrimAtPath(chassis_link_path)
root_joint_prim = stage.GetPrimAtPath(root_joint_path)
print(f"[확인] chassis_link 존재={chassis_prim.IsValid()}  root_joint 존재={root_joint_prim.IsValid()}", flush=True)

if root_joint_prim.IsValid():
    joint = UsdPhysics.Joint(root_joint_prim)
    body0_targets = list(joint.GetBody0Rel().GetTargets())
    body1_targets = list(joint.GetBody1Rel().GetTargets())
    print(f"[root_joint 기존] body0={body0_targets}  body1={body1_targets}", flush=True)

    # M0609 쪽 articulation root를 제거하고 Nova Carter의 chassis_link에 고정
    root_joint_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    root_joint_prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
    joint.GetBody0Rel().SetTargets([Sdf.Path(chassis_link_path)])
    # 버그였던 부분: body0을 chassis_link로 바꿔도 localPos0(조인트가 chassis_link 로컬 좌표계
    # 안에서 어디를 앵커로 삼을지)는 그대로 (0,0,0)이라, base_link가 chassis_link 원점(거의
    # 바닥 높이)에 그대로 붙어버렸었다. chassis_link 로컬 +Z 방향으로 MOUNT_Z만큼 띄워줘야
    # base_link가 실제로 캐리어 "위"에 올라간다.
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, MOUNT_Z))
    print(f"[root_joint 수정] ArticulationRootAPI 제거, body0 -> {chassis_link_path}, "
          f"localPos0 -> (0,0,{MOUNT_Z})", flush=True)

for _ in range(10):
    simulation_app.update()

# ---- 조인트 드라이브 강성 설정 ----
# 기존 M0609 스크립트들(4.pick_place.py 등)에 있던 필수 단계: USD에 저장된 기본 드라이브
# 게인이 너무 약해서(또는 0이라서) 이 단계 없이는 팔이 중력에 못 이기고 축 늘어진다.
DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8
drive_count = 0
for prim in Usd.PrimRange(stage.GetPrimAtPath(m0609_path)):
    for dof_type in ["angular", "linear"]:
        drive = UsdPhysics.DriveAPI.Get(prim, dof_type)
        if drive:
            drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
            drive.GetDampingAttr().Set(DRIVE_DAMPING)
            drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
            drive_count += 1
print(f"[DRIVE] {drive_count}개 조인트에 강성 적용 (stiffness={DRIVE_STIFFNESS}, damping={DRIVE_DAMPING})", flush=True)

try:
    world.reset()
    for _ in range(120):
        world.step(render=True)
    print("[물리] world.reset() + 120 step 성공, 에러 없음", flush=True)
except Exception as e:
    print(f"[물리-에러] {e}", flush=True)

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


snapshot(eye=[1.8, -5.5, 2.2], target=[ROBOT_XY[0], ROBOT_XY[1], 0.6], fname="_verify_mobile_manip_wide.png")
snapshot(eye=[0.2, -3.3, 1.3], target=[ROBOT_XY[0], ROBOT_XY[1], 0.7], fname="_verify_mobile_manip_close.png")

print("\n[안내] 재생 버튼은 이미 눌린 상태로 검증 완료. 창을 계속 열어두고 확인하거나 종료하세요.")
print("[안내] 창을 닫으면 결과가 자동 저장됩니다.\n", flush=True)

while simulation_app.is_running():
    simulation_app.update()
    time.sleep(0.01)

omni.usd.get_context().save_as_stage(SCENE_USD)
print(f"\n[저장 완료] {SCENE_USD}", flush=True)

simulation_app.close()
