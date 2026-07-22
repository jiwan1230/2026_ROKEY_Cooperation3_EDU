"""
25.lift_stage7_fixedjoint_teleport.py
6단계까지 프리즘 조인트(힘 기반 드라이브)도, kinematic 캐리지(매 프레임 위치 강제)도 전부
Nova Carter+M0609 풀 콤보에서 얼어붙었다(kinematic은 딱 한 번만 살짝 움직이고 그 뒤로 고정).
반면 9/16번 스크립트에서 검증된 "FixedJoint로 chassis_link<->base_link를 고정 결합해서 하나의
큰 articulation으로 합치는" 패턴은 완벽하게 동작한다(4.4mm 정확도, 바퀴/팔 전부 정상).

계획서(nova-cart-twinkling-mochi.md)를 다시 보면 리프트는 "태스크 단계별로 동적으로" 바뀌면
되지, 매 프레임 부드럽게 움직일 필요는 없다(올리기->이동->내리기->집기->올리기, 이산적 단계
전환). 그래서 접근을 바꾼다: 검증된 FixedJoint 병합 articulation 패턴을 그대로 쓰되, "리프트를
움직인다"는 것을 물리 드라이브가 아니라 - 몇 스텝 멈추고 - FixedJoint의 localPos 오프셋 자체를
USD로 직접 수정(순간이동)하는 것으로 구현한다. 물리적으로 부드럽게 올라가는 모습은 없지만
(대신 시각적으로는 몇 프레임 안에 뛰어오르듯 보일 것), PhysX 드라이브 버그를 완전히 우회한다.
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

# ---- 9/16번에서 검증된 병합 articulation 패턴: FixedJoint로 chassis_link<->base_link 고정 ----
mount_joint_path = f"{chassis_link_path}/lift_mount_joint"
mount_joint = UsdPhysics.FixedJoint.Define(stage, mount_joint_path)
mount_joint.CreateBody0Rel().SetTargets([Sdf.Path(chassis_link_path)])
mount_joint.CreateBody1Rel().SetTargets([Sdf.Path(base_link_path)])
mount_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, LIFT_MIN))
mount_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
mount_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
mount_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
print(f"[생성] {mount_joint_path} (FixedJoint, chassis_link<->base_link, localPos0.z={LIFT_MIN})", flush=True)

stray_prim = stage.GetPrimAtPath(f"{m0609_path}/onrobot_rg2ft/world")
if stray_prim.IsValid() and stray_prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
    stray_prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)

n = add_drive_stiffness(stage, m0609_path)
n2 = add_drive_stiffness(stage, carter_path)
print(f"[DRIVE] M0609={n}개, NovaCarter={n2}개 조인트 강성 적용", flush=True)

ee_path = f"{m0609_path}/link_6"
gripper = SurfaceGripper(end_effector_prim_path=ee_path, surface_gripper_path="")
robot = SingleManipulator(prim_path=chassis_link_path, end_effector_prim_path=ee_path, name="teleport_lift_test", gripper=gripper)

world.reset()
robot.initialize(physics_sim_view=world.physics_sim_view)
print(f"[안정화] dof_names={robot.dof_names}", flush=True)

init_joints = np.zeros(robot.num_dof)
if "joint_3" in robot.dof_names:
    init_joints[robot.dof_names.index("joint_3")] = np.pi / 2
if "joint_5" in robot.dof_names:
    init_joints[robot.dof_names.index("joint_5")] = np.pi / 2
robot.set_joint_positions(init_joints)
for _ in range(60):
    world.step(render=True)


def set_lift_height_teleport(h):
    """FixedJoint의 localPos0.z를 직접 갱신 - 물리 드라이브가 아니라 USD 오프셋 순간이동."""
    mount_joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, float(h)))


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
snapshot("_stage7_00_start.png")

for i, h in enumerate(LIFT_TEST_HEIGHTS):
    set_lift_height_teleport(h)
    for _ in range(90):
        world.step(render=True)
    bz = base_link_world_z()
    roll, pitch = chassis_tilt_deg()
    print(f"[테스트 {i}] target_h={h:.3f} base_link_world_z={bz:.4f} "
          f"(기대≈{h:.3f}) chassis roll={roll:.2f}deg pitch={pitch:.2f}deg", flush=True)
    snapshot(f"_stage7_{i+1:02d}_h{h:.2f}.png")

print("\n[안내] 7단계(FixedJoint 오프셋 순간이동 리프트) 검증 완료.\n", flush=True)
simulation_app.close()
