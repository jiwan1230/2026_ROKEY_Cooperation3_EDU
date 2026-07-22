"""
24.lift_stage6_dual_articulation.py
5단계(kinematic 캐리지)에서도 여전히 얼어붙었는데, 그때 로그를 보면 결정적 단서가 있었다:
robot.dof_names에 M0609 관절이 단 하나도 없었다 (['joint_caster_base', ...]만 나오고
joint_1~6이 전부 빠짐). 즉 kinematic 캐리지에 FixedJoint로 매달리는 순간 base_link 이하
체인이 "chassis_link의 articulation"에서 완전히 빠져나갔을 뿐 아니라, 자기 자신의
ArticulationRoot도 없어서 joint_1~6 자체가 articulation으로 시뮬레이션되지 않았다
(2단계에서는 anchor_prim에 ArticulationRootAPI를 명시적으로 붙였었는데, 5단계에서는
kinematic 캐리지에는 그걸 붙이지 않았다 - kinematic body는 관례상 articulation root가
될 필요가 없다고 가정했었는데, 그러면 base_link 체인이 뿌리를 잃는다).

해법: base_link 자체에 ArticulationRootAPI를 직접 적용해서 M0609(+VGP20) 체인을
Nova Carter와 완전히 독립된 자기 자신의 articulation으로 만든다. Nova Carter(바퀴/캐스터)는
원래대로 chassis_link가 자기 articulation root. 두 articulation을 kinematic 캐리지로
물리적으로만 연결(매 프레임 위치 동기화)하고, 로봇 핸들도 두 개(carter_robot, m0609_robot)로
분리해서 각각 독립적으로 제어한다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path

import numpy as np
import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, UsdLux, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim, SingleArticulation
from isaacsim.core.utils.rotations import quat_to_euler_angles
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

_THIS_DIR = Path(__file__).resolve().parent
M0609_DIR = _THIS_DIR.parent / "M0609"
M0609_USD = str(M0609_DIR / "Collected_m0609_vgp20_camera" / "m0609_vgp20_camera.usd")

LIFT_MIN, LIFT_MAX = 0.42, 1.20
LIFT_TEST_HEIGHTS = [0.42, 0.60, 0.80, 0.60]
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

# ---- 핵심 수정: base_link를 M0609 체인 자신의 독립 ArticulationRoot로 만든다 ----
base_link_prim = stage.GetPrimAtPath(base_link_path)
UsdPhysics.ArticulationRootAPI.Apply(base_link_prim)
print(f"[적용] {base_link_path}에 ArticulationRootAPI (M0609를 독립 articulation으로)", flush=True)

# ---- kinematic 캐리지: 매 프레임 Xform을 직접 설정, FixedJoint로 base_link를 붙잡음 ----
carriage_path = "/World/MobileManipulator/LiftCarriage"
carriage_cube = UsdGeom.Cube.Define(stage, carriage_path)
carriage_cube.GetSizeAttr().Set(1.0)
carriage_xform_api = UsdGeom.Xformable(carriage_cube)
carriage_xform_api.ClearXformOpOrder()
carriage_translate_op = carriage_xform_api.AddTranslateOp()
carriage_translate_op.Set(Gf.Vec3d(0.0, 0.0, LIFT_MIN))
carriage_xform_api.AddScaleOp().Set(Gf.Vec3f(0.15, 0.15, 0.05))
carriage_cube.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.2, 0.2)])
carriage_prim = carriage_cube.GetPrim()
UsdPhysics.RigidBodyAPI.Apply(carriage_prim)
UsdPhysics.RigidBodyAPI(carriage_prim).CreateKinematicEnabledAttr().Set(True)
print(f"[생성] {carriage_path} kinematic rigid body", flush=True)

fix_joint = UsdPhysics.FixedJoint.Define(stage, f"{carriage_path}/carry_joint")
fix_joint.CreateBody0Rel().SetTargets([Sdf.Path(carriage_path)])
fix_joint.CreateBody1Rel().SetTargets([Sdf.Path(base_link_path)])
fix_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
fix_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
print(f"[생성] {carriage_path}/carry_joint (FixedJoint, carriage<->base_link)", flush=True)

stray_prim = stage.GetPrimAtPath(f"{m0609_path}/onrobot_rg2ft/world")
if stray_prim.IsValid() and stray_prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
    stray_prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)

n = add_drive_stiffness(stage, m0609_path)
n2 = add_drive_stiffness(stage, carter_path)
print(f"[DRIVE] M0609={n}개, NovaCarter={n2}개 조인트 강성 적용", flush=True)

ee_path = f"{m0609_path}/link_6"
gripper = SurfaceGripper(end_effector_prim_path=ee_path, surface_gripper_path="")
m0609_robot = SingleManipulator(prim_path=base_link_path, end_effector_prim_path=ee_path, name="m0609_arm", gripper=gripper)
carter_robot = SingleArticulation(prim_path=chassis_link_path, name="carter_base")

world.reset()
carter_robot.initialize(physics_sim_view=world.physics_sim_view)
m0609_robot.initialize(physics_sim_view=world.physics_sim_view)
print(f"[안정화] carter dof_names={carter_robot.dof_names}", flush=True)
print(f"[안정화] m0609 dof_names={m0609_robot.dof_names}", flush=True)

init_joints = np.zeros(m0609_robot.num_dof)
if "joint_3" in m0609_robot.dof_names:
    init_joints[m0609_robot.dof_names.index("joint_3")] = np.pi / 2
if "joint_5" in m0609_robot.dof_names:
    init_joints[m0609_robot.dof_names.index("joint_5")] = np.pi / 2
m0609_robot.set_joint_positions(init_joints)

carriage_xform_prim = SingleRigidPrim(carriage_path)


def set_lift_height(h):
    """chassis_link의 현재 월드포즈를 읽어, 그 위 h(월드 Z 오프셋)에 캐리지를 kinematic으로 이동."""
    chassis_pos, chassis_quat = carter_robot.get_world_pose()
    target_pos = np.array([chassis_pos[0], chassis_pos[1], chassis_pos[2] + h])
    carriage_xform_prim.set_world_pose(position=target_pos, orientation=chassis_quat)


set_lift_height(LIFT_MIN)
for _ in range(60):
    world.step(render=True)


def chassis_tilt_deg():
    _, quat = carter_robot.get_world_pose()
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
snapshot("_stage6_00_start.png")

for i, h in enumerate(LIFT_TEST_HEIGHTS):
    for _ in range(120):
        set_lift_height(h)
        world.step(render=True)
    bz = base_link_world_z()
    roll, pitch = chassis_tilt_deg()
    print(f"[테스트 {i}] target_h={h:.3f} base_link_world_z={bz:.4f} "
          f"(기대≈{h:.3f}) chassis roll={roll:.2f}deg pitch={pitch:.2f}deg", flush=True)
    snapshot(f"_stage6_{i+1:02d}_h{h:.2f}.png")

print("\n[안내] 6단계(base_link 독립 articulation + kinematic 리프트) 검증 완료.\n", flush=True)
simulation_app.close()
