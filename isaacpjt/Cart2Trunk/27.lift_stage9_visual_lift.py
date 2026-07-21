"""
27.lift_stage9_visual_lift.py
8단계에서 처음으로 리프트 텔레포트가 정확히 동작했다(목표 대비 3mm 이내, 왕복 가역적) -
Nova Carter chassis_link와 M0609 base_link를 물리 조인트로 연결하지 않고 완전히 독립된
articulation 두 개로 분리한 뒤, 매 프레임 m0609_robot.set_world_pose()로 직접 텔레포트하고
속도를 0으로 눌러주는 방식이다.

다만 부작용이 있었다: 두 바디가 이제 물리적으로 완전히 독립이라 마운트 지점에서 콜리전
메시가 서로 겹치면서(충돌 필터링이 없어서) 첫 프레임에 튕기는 충돌이 발생해 섀시가 19도
기울어졌다. 사용자 지적: (1) 콜리전 겹침 때문에 튕기는 게 맞다 - 필터링 필요, (2) 지금까지
스크립트에는 리프트가 눈에 보이는 형상(예: 위아래로 늘어나는 원기둥)으로 전혀 표현되지
않았다 - 실제 구동을 생각하면 시각적으로도 리프트가 있어야 한다.

이번 버전에서 추가/수정:
1. chassis_link<->base_link 사이에 UsdPhysics.FilteredPairsAPI로 충돌 필터링 (튕김 방지)
2. 섀시 마운트점부터 base_link 바닥까지 매 프레임 높이를 갱신하는 시각용 원기둥
   (콜리전 없음 - 순수 시각 요소, 물리에 전혀 관여하지 않음) 추가
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

# ---- M0609를 자기 자신의 독립 articulation으로 만든다 (8단계에서 검증된 방식) ----
base_link_prim = stage.GetPrimAtPath(base_link_path)
UsdPhysics.ArticulationRootAPI.Apply(base_link_prim)
print(f"[적용] {base_link_path}에 ArticulationRootAPI (독립 articulation)", flush=True)

# ---- 충돌 필터링: 마운트 지점에서 chassis_link<->base_link 콜리전 겹침으로 인한 튕김 방지 ----
chassis_link_prim = stage.GetPrimAtPath(chassis_link_path)
filt_chassis = UsdPhysics.FilteredPairsAPI.Apply(chassis_link_prim)
filt_chassis.CreateFilteredPairsRel().AddTarget(Sdf.Path(base_link_path))
filt_base = UsdPhysics.FilteredPairsAPI.Apply(base_link_prim)
filt_base.CreateFilteredPairsRel().AddTarget(Sdf.Path(chassis_link_path))
print(f"[필터] {chassis_link_path} <-> {base_link_path} 충돌 필터링 적용", flush=True)

# ---- 시각용 리프트 원기둥: 콜리전 없음, 순수 표시용. 매 프레임 마운트점~base_link 바닥 사이를 채운다 ----
lift_column_path = "/World/MobileManipulator/LiftColumnVisual"
lift_column = UsdGeom.Cylinder.Define(stage, lift_column_path)
lift_column.CreateRadiusAttr().Set(LIFT_COLUMN_RADIUS)
lift_column.CreateHeightAttr().Set(1.0)
lift_column.CreateAxisAttr("Z")
lift_column.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.45, 0.1)])
lift_column_xform = UsdGeom.Xformable(lift_column)
lift_column_xform.ClearXformOpOrder()
lift_column_translate_op = lift_column_xform.AddTranslateOp()
lift_column_scale_op = lift_column_xform.AddScaleOp()
lift_column_scale_op.Set(Gf.Vec3f(1.0, 1.0, 1.0))
print(f"[생성] {lift_column_path} (시각용 텔레스코핑 원기둥, 콜리전 없음)", flush=True)

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
    """chassis_link 위 h 위치로 m0609_robot을 텔레포트하고, 마운트점~base_link 바닥까지
    보이는 원기둥(LiftColumnVisual)의 위치/스케일도 같이 갱신한다."""
    chassis_pos, chassis_quat = carter_robot.get_world_pose()
    target_pos = np.array([chassis_pos[0], chassis_pos[1], chassis_pos[2] + h])
    m0609_robot.set_world_pose(position=target_pos, orientation=chassis_quat)
    m0609_robot.set_linear_velocity(np.zeros(3))
    m0609_robot.set_angular_velocity(np.zeros(3))

    # chassis_link의 원점(바퀴 축 높이 근처)이 아니라, 차체 윗면(=LIFT_MIN에서 팔이 얹히는
    # 기준면)부터 위로만 뻗어나가야 "바닥/바퀴 밑에서 튀어나오는" 것처럼 안 보인다.
    # 원기둥은 기본 height=1(중심 기준 -0.5~+0.5)이므로, 뻗어난 길이(h-LIFT_MIN)만큼
    # z축 스케일을 주고, 중심 위치를 (차체 윗면 + 뻗은길이/2)로 놓는다.
    column_base_z = chassis_pos[2] + LIFT_MIN
    column_len = max(float(h) - LIFT_MIN, 0.001)
    lift_column_scale_op.Set(Gf.Vec3f(1.0, 1.0, column_len))
    lift_column_translate_op.Set(Gf.Vec3d(float(chassis_pos[0]), float(chassis_pos[1]), float(column_base_z + column_len / 2.0)))


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


def base_link_world_z_tensor():
    pos, _ = m0609_robot.get_world_pose()
    return float(pos[2])


for _ in range(60):
    set_lift_height(LIFT_MIN)
    world.step(render=True)

roll0, pitch0 = chassis_tilt_deg()
print(f"[시작] base_link world z={base_link_world_z_tensor():.4f} chassis roll={roll0:.2f}deg pitch={pitch0:.2f}deg", flush=True)
snapshot("_stage9_00_start.png", hold_h=LIFT_MIN)

for i, h in enumerate(LIFT_TEST_HEIGHTS):
    for _ in range(90):
        set_lift_height(h)
        world.step(render=True)
    bz_t = base_link_world_z_tensor()
    roll, pitch = chassis_tilt_deg()
    print(f"[테스트 {i}] target_h={h:.3f} base_link_world_z(tensor)={bz_t:.4f} "
          f"(기대≈{h:.3f}) chassis roll={roll:.2f}deg pitch={pitch:.2f}deg", flush=True)
    snapshot(f"_stage9_{i+1:02d}_h{h:.2f}.png", hold_h=h)

print("\n[안내] 9단계(충돌 필터링 + 시각용 리프트 원기둥) 검증 완료.\n", flush=True)
simulation_app.close()
