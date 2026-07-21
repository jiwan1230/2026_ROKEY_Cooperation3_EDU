"""
22.lift_stage4_full_combo.py
리프트 검증 4단계(최종): Nova Carter + 프리즘 리프트 조인트 + M0609(+VGP20) 전부 결합해서
리프트가 정상 동작하는지 확인한다.

1~3단계 전부 성공(박스2개 / 고정앵커+M0609 / NovaCarter+더미박스) - 각 조합은 문제없었다.
18번 스크립트(전부 합친 첫 시도)에서 실패한 원인은 아직 특정 못했지만, 여기서는 1~3단계에서
검증된 설정(넉넉한 limit 범위, 경계값 근처 회피)을 그대로 재사용해서 전부 합쳐본다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path

import numpy as np
import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, UsdLux, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import quat_to_euler_angles
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

_THIS_DIR = Path(__file__).resolve().parent
M0609_DIR = _THIS_DIR.parent / "M0609"
M0609_USD = str(M0609_DIR / "Collected_m0609_vgp20_camera" / "m0609_vgp20_camera.usd")

# 가설: LIFT_MIN=0.20에서 얼어붙은 건, chassis_link 자기 원점이 지면 근처라(기존
# MOUNT_Z=0.42가 "섀시 윗면까지의 오프셋"이었던 사실 참고) 0.20에서는 M0609 base_link가
# Nova Carter 섀시 몸체 "안쪽"에 파묻혀 시작해서 극심한 충돌 반발력이 리프트 드라이브
# (강성 1e8)를 압도했을 가능성 - 안전했던(섀시 위에 얹힌) 원래 MOUNT_Z=0.42부터 시작해서
# 확인한다.
LIFT_MIN, LIFT_MAX = 0.42, 1.20
LIFT_TEST_HEIGHTS = [0.42, 0.60, 0.80]
FACE_ROT_Z = 0.0
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


world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

area_light = UsdLux.SphereLight.Define(stage, "/World/VerifyAreaLight")
area_light.CreateRadiusAttr().Set(0.5)
area_light.CreateIntensityAttr().Set(30000)
UsdGeom.Xformable(area_light).AddTranslateOp().Set(Gf.Vec3d(0.5, 0.5, 2.0))

root = get_assets_root_path()
carter_url = root + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
carter_path = "/World/MobileManipulator/NovaCarter"
carter_xform = UsdGeom.Xform.Define(stage, carter_path)
carter_xform.GetPrim().GetReferences().AddReference(carter_url)
carter_xform.ClearXformOpOrder()
carter_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
carter_xform.AddRotateZOp().Set(FACE_ROT_Z)
chassis_link_path = f"{carter_path}/chassis_link"

m0609_path = "/World/MobileManipulator/M0609"
m0609_xform = UsdGeom.Xform.Define(stage, m0609_path)
m0609_xform.GetPrim().GetReferences().AddReference(M0609_USD, "/World/m0609")
m0609_xform.ClearXformOpOrder()
m0609_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, LIFT_MIN))
m0609_xform.AddRotateZOp().Set(FACE_ROT_Z)

for _ in range(20):
    simulation_app.update()

base_link_path = f"{m0609_path}/base_link"
old_root_joint_path = f"{m0609_path}/root_joint"
if stage.GetPrimAtPath(old_root_joint_path).IsValid():
    stage.RemovePrim(old_root_joint_path)
    print(f"[제거] {old_root_joint_path}", flush=True)

# 실험: chassis_link(루트)가 바퀴/캐스터 + lift_joint 두 갈래로 직접 갈라지는 지점에 프리즘
# 조인트를 바로 놓으면 안 움직였다(위 두 번 실패 확인). chassis_link의 직계 자식은 "바퀴/캐스터
# + FixedJoint 하나"로 단순하게 유지하고, 실제 프리즘 조인트는 그 FixedJoint 다음(중간 캐리지
# 바디)에 두어 분기점에서 한 단계 떨어뜨려본다 - 2단계(고정앵커+M0609)와 동일한 위상 구조.
carriage_path = "/World/MobileManipulator/LiftCarriage"
carriage_cube = UsdGeom.Cube.Define(stage, carriage_path)
carriage_cube.GetSizeAttr().Set(1.0)
carriage_xform = UsdGeom.Xformable(carriage_cube)
carriage_xform.ClearXformOpOrder()
carriage_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
carriage_xform.AddScaleOp().Set(Gf.Vec3f(0.15, 0.15, 0.05))
carriage_cube.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.2, 0.2)])
carriage_prim = carriage_cube.GetPrim()
UsdPhysics.RigidBodyAPI.Apply(carriage_prim)
UsdPhysics.MassAPI.Apply(carriage_prim).CreateMassAttr().Set(2.0)
for _ in range(5):
    simulation_app.update()

carriage_fix = UsdPhysics.FixedJoint.Define(stage, f"{carriage_path}/mount_joint")
carriage_fix.CreateBody0Rel().SetTargets([Sdf.Path(chassis_link_path)])
carriage_fix.CreateBody1Rel().SetTargets([Sdf.Path(carriage_path)])
carriage_fix.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
carriage_fix.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
print(f"[생성] {carriage_path} (chassis_link에 FixedJoint로 고정된 중간 캐리지)", flush=True)

lift_joint_path = f"{carriage_path}/lift_joint"
lift_joint = UsdPhysics.PrismaticJoint.Define(stage, lift_joint_path)
lift_joint.CreateAxisAttr("Z")
lift_joint.CreateBody0Rel().SetTargets([Sdf.Path(carriage_path)])
lift_joint.CreateBody1Rel().SetTargets([Sdf.Path(base_link_path)])
lift_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
lift_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
lift_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
lift_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
lift_joint.CreateLowerLimitAttr().Set(LIFT_MIN)
lift_joint.CreateUpperLimitAttr().Set(LIFT_MAX)
drive = UsdPhysics.DriveAPI.Apply(lift_joint.GetPrim(), "linear")
drive.CreateTypeAttr().Set("force")
drive.CreateStiffnessAttr().Set(DRIVE_STIFFNESS)
drive.CreateDampingAttr().Set(DRIVE_DAMPING)
drive.CreateMaxForceAttr().Set(DRIVE_MAX_FORCE)
print(f"[생성] {lift_joint_path} limit=[{LIFT_MIN},{LIFT_MAX}] body0={carriage_path} body1={base_link_path}", flush=True)

stray_prim = stage.GetPrimAtPath(f"{m0609_path}/onrobot_rg2ft/world")
if stray_prim.IsValid() and stray_prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
    stray_prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)

n = add_drive_stiffness(stage, m0609_path)
n2 = add_drive_stiffness(stage, carter_path)
n3 = add_drive_stiffness(stage, carriage_path)  # lift_joint는 이제 carriage 밑에 있음
print(f"[DRIVE] M0609={n}개, NovaCarter={n2}개, LiftCarriage(리프트)={n3}개 조인트 강성 적용", flush=True)

# 진단: chassis_link의 PhysxArticulationAPI solver iteration count가 Nova Carter 단독
# DOF 수 기준으로 낮게 잡혀있어서, M0609 6-DOF가 추가된 지금은 매 스텝 안에 리프트까지
# 드라이브 힘이 제대로 전파가 안 될 가능성 - 조회 후 넉넉하게 올려본다.
chassis_prim_diag = stage.GetPrimAtPath(chassis_link_path)
if chassis_prim_diag.HasAPI(PhysxSchema.PhysxArticulationAPI):
    art_api = PhysxSchema.PhysxArticulationAPI(chassis_prim_diag)
    print(f"[진단] PhysxArticulationAPI solverPositionIterationCount="
          f"{art_api.GetSolverPositionIterationCountAttr().Get()} "
          f"solverVelocityIterationCount={art_api.GetSolverVelocityIterationCountAttr().Get()} "
          f"sleepThreshold={art_api.GetSleepThresholdAttr().Get()} "
          f"stabilizationThreshold={art_api.GetStabilizationThresholdAttr().Get()}", flush=True)
    art_api.GetSolverPositionIterationCountAttr().Set(64)
    art_api.GetSolverVelocityIterationCountAttr().Set(64)
    art_api.GetSleepThresholdAttr().Set(0.0)
    print("[진단] solver iteration count -> 64, sleepThreshold -> 0으로 상향", flush=True)
else:
    print(f"[진단] {chassis_link_path}에 PhysxArticulationAPI 없음(!)", flush=True)

ee_path = f"{m0609_path}/link_6"
gripper = SurfaceGripper(end_effector_prim_path=ee_path, surface_gripper_path="")
robot = SingleManipulator(prim_path=chassis_link_path, end_effector_prim_path=ee_path, name="full_combo_test", gripper=gripper)

world.reset()
robot.initialize(physics_sim_view=world.physics_sim_view)
print(f"[안정화] dof_names={robot.dof_names}", flush=True)
idx_lift = robot.dof_names.index("lift_joint")

init_joints = np.zeros(robot.num_dof)
init_joints[idx_lift] = LIFT_MIN
if "joint_3" in robot.dof_names:
    init_joints[robot.dof_names.index("joint_3")] = np.pi / 2
if "joint_5" in robot.dof_names:
    init_joints[robot.dof_names.index("joint_5")] = np.pi / 2
robot.set_joint_positions(init_joints)
for _ in range(60):
    world.step(render=True)


def chassis_tilt_deg():
    _, quat = robot.get_world_pose()
    roll, pitch, _ = quat_to_euler_angles(quat)
    return np.degrees(roll), np.degrees(pitch)


viewport = vp_util.get_active_viewport()


def snapshot(fname):
    set_camera_view(eye=[1.8, 1.8, 1.3], target=[0.0, 0.0, 0.6])
    for _ in range(10):
        world.step(render=True)
    out = str(_THIS_DIR / fname)
    vp_util.capture_viewport_to_file(viewport, out)
    for _ in range(5):
        world.step(render=True)
    print(f"[SCREENSHOT] {out}", flush=True)


def base_link_world_z():
    prim = stage.GetPrimAtPath(base_link_path)
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return float(mat.ExtractTranslation()[2])


roll0, pitch0 = chassis_tilt_deg()
print(f"[시작] base_link world z={base_link_world_z():.4f} chassis roll={roll0:.2f}deg pitch={pitch0:.2f}deg", flush=True)
snapshot("_stage4_00_start.png")

for i, h in enumerate(LIFT_TEST_HEIGHTS):
    for _ in range(150):
        robot.apply_action(ArticulationAction(joint_positions=[h], joint_indices=[idx_lift]))
        world.step(render=True)
    actual = robot.get_joint_positions()[idx_lift]
    bz = base_link_world_z()
    roll, pitch = chassis_tilt_deg()
    print(f"[테스트 {i}] target={h:.3f} joint_position={actual:.4f} base_link_world_z={bz:.4f} "
          f"chassis roll={roll:.2f}deg pitch={pitch:.2f}deg", flush=True)
    snapshot(f"_stage4_{i+1:02d}_h{h:.2f}.png")

print("\n[안내] 4단계(전부 결합) 검증 완료.\n", flush=True)
simulation_app.close()
