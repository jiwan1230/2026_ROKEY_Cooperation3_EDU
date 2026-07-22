"""
26.lift_stage8_direct_teleport.py
6/7단계까지: 프리즘 드라이브, kinematic 캐리지+FixedJoint, FixedJoint 오프셋 직접 수정까지
전부 Nova Carter(실제)+M0609(실제) 조합에서 얼어붙었다(항상 world.reset() 시점에 설정된
초기값에 고정, 매 프레임 명령을 줘도 반응 없음) - 조인트 매커니즘 자체가 의심된다.

이번 8단계는 PhysX 조인트/제약을 아예 쓰지 않는다: 6단계에서 이미 base_link에
ArticulationRootAPI를 직접 적용해 M0609를 Nova Carter와 완전히 독립된 자기 articulation으로
분리하는 것까지는 성공했다(dof_names가 정확히 joint_1~6만 보여줌). 거기서 한 단계 더 나아가서
캐리지/FixedJoint 없이, M0609 자신의 articulation 루트(m0609_robot)에 매 프레임
set_world_pose()를 직접 호출해서 Nova Carter의 chassis 포즈+리프트 오프셋을 따라가게 만든다.
조인트 제약 솔버를 전혀 거치지 않는 가장 단순한 텔레포트라서, 이것도 안 되면 PhysX 조인트
문제가 아니라 이 두 에셋을 한 world에 같이 올리는 것 자체에 근본적인 문제가 있다는 뜻이다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path

import numpy as np
import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, UsdLux, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
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

# ---- M0609를 자기 자신의 독립 articulation으로 만든다 (Nova Carter와 물리적 조인트 없음) ----
base_link_prim = stage.GetPrimAtPath(base_link_path)
UsdPhysics.ArticulationRootAPI.Apply(base_link_prim)
print(f"[적용] {base_link_path}에 ArticulationRootAPI (독립 articulation, 조인트로 연결 안 함)", flush=True)

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


def set_lift_height(h):
    """chassis_link의 현재 월드포즈를 읽어, m0609_robot(독립 articulation) 자체를 그 위 h로 직접 텔레포트.
    자유 부유 상태라 중력을 받으므로, 텔레포트할 때마다 속도도 0으로 눌러줘야 프레임 사이에
    중력으로 누적된 속도가 다음 텔레포트 목표를 지나쳐 드리프트하는 것을 막을 수 있다."""
    chassis_pos, chassis_quat = carter_robot.get_world_pose()
    target_pos = np.array([chassis_pos[0], chassis_pos[1], chassis_pos[2] + h])
    m0609_robot.set_world_pose(position=target_pos, orientation=chassis_quat)
    m0609_robot.set_linear_velocity(np.zeros(3))
    m0609_robot.set_angular_velocity(np.zeros(3))


for _ in range(60):
    set_lift_height(LIFT_MIN)
    world.step(render=True)


def chassis_tilt_deg():
    _, quat = carter_robot.get_world_pose()
    roll, pitch, _ = quat_to_euler_angles(quat)
    return np.degrees(roll), np.degrees(pitch)


viewport = vp_util.get_active_viewport()


def snapshot(fname, hold_h=None):
    set_camera_view(eye=[1.8, 1.8, 1.3], target=[0.0, 0.0, 0.6])
    for _ in range(10):
        if hold_h is not None:
            set_lift_height(hold_h)
        world.step(render=True)
    out = str(_THIS_DIR / fname)
    vp_util.capture_viewport_to_file(viewport, out)
    for _ in range(5):
        if hold_h is not None:
            set_lift_height(hold_h)
        world.step(render=True)
    print(f"[SCREENSHOT] {out}", flush=True)


def base_link_world_z():
    prim = stage.GetPrimAtPath(base_link_path)
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return float(mat.ExtractTranslation()[2])


def base_link_world_z_tensor():
    pos, _ = m0609_robot.get_world_pose()
    return float(pos[2])


roll0, pitch0 = chassis_tilt_deg()
print(f"[시작] base_link world z={base_link_world_z():.4f} chassis roll={roll0:.2f}deg pitch={pitch0:.2f}deg", flush=True)
snapshot("_stage8_00_start.png", hold_h=LIFT_MIN)

for i, h in enumerate(LIFT_TEST_HEIGHTS):
    for _ in range(90):
        set_lift_height(h)
        world.step(render=True)
    bz = base_link_world_z()
    bz_t = base_link_world_z_tensor()
    roll, pitch = chassis_tilt_deg()
    print(f"[테스트 {i}] target_h={h:.3f} base_link_world_z(usd)={bz:.4f} "
          f"base_link_world_z(tensor)={bz_t:.4f} (기대≈{h:.3f}) chassis roll={roll:.2f}deg pitch={pitch:.2f}deg", flush=True)
    snapshot(f"_stage8_{i+1:02d}_h{h:.2f}.png", hold_h=h)

print("\n[안내] 8단계(조인트 없이 m0609_robot 직접 텔레포트) 검증 완료.\n", flush=True)
simulation_app.close()
