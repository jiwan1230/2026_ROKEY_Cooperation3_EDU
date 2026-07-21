"""
18.lift_joint_diagnostic.py
Nova Carter + M0609(+VGP20) 사이의 고정 마운트(root_joint, FixedJoint)를 Z축 프리즘 조인트로
바꿔서, 실제로 명령한 높이까지 잘 움직이고 그 자리에서 잘 버티는지만 검증하는 최소 스크립트.
카트/차량/주행/PLACE는 전혀 없음 - 리프트 자체의 동작만 확인.

배경: PICK이 계속 실패한 근본 원인은 "M0609 마운트 높이가 고정돼있어서, 목표 높이가
팔 자신의 base_link 자기충돌 회피 구간과 겹치면 절대 못 닿는다"는 것이었다
(m0609_description.yaml의 collision sphere로 확인: relative z 약 -0.08~0.34는 회피 구간).
사용자 제안: 이 마운트를 상하로 움직이는 리프트로 만들어서, 태스크 단계마다 리프트 높이
자체를 바꿔가며 목표를 항상 "팔이 편하게 뻗을 수 있는 상대 높이"에 오도록 만든다.

이 스크립트가 확인할 것:
  1. Nova Carter 현재 질량(질량 보강 계획을 위한 기준값)
  2. 프리즘 조인트가 명령한 높이(LIFT_MIN~LIFT_MAX)까지 실제로 움직이는지
  3. 움직이는 동안/도달 후 팔이나 로봇이 불안정해지지 않는지(스크린샷으로 육안 확인)
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

ROBOT_XY = (0.0, 0.0)
FACE_ROT_Z = 0.0
LIFT_MIN = 0.42   # 기존 고정 마운트 높이(섀시 윗면, 최저 높이)
LIFT_MAX = 0.95   # 카트 최고높이(1.03) 근처까지 커버할 여유(진단 후 확정)
LIFT_TEST_HEIGHTS = [0.42, 0.60, 0.80, 0.95, 0.60]
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e8, 1e4, 1e8
LIFT_DRIVE_STIFFNESS, LIFT_DRIVE_DAMPING, LIFT_DRIVE_MAX_FORCE = 1e9, 1e6, 1e9


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


def build_mobile_manipulator_with_lift(stage):
    root = get_assets_root_path()
    carter_url = root + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"

    carter_path = "/World/MobileManipulator/NovaCarter"
    carter_xform = UsdGeom.Xform.Define(stage, carter_path)
    carter_xform.GetPrim().GetReferences().AddReference(carter_url)
    carter_xform.ClearXformOpOrder()
    carter_xform.AddTranslateOp().Set(Gf.Vec3d(ROBOT_XY[0], ROBOT_XY[1], 0.0))
    carter_xform.AddRotateZOp().Set(FACE_ROT_Z)

    m0609_path = "/World/MobileManipulator/M0609"
    m0609_xform = UsdGeom.Xform.Define(stage, m0609_path)
    m0609_xform.GetPrim().GetReferences().AddReference(M0609_USD, "/World/m0609")
    m0609_xform.ClearXformOpOrder()
    m0609_xform.AddTranslateOp().Set(Gf.Vec3d(ROBOT_XY[0], ROBOT_XY[1], LIFT_MIN))
    m0609_xform.AddRotateZOp().Set(FACE_ROT_Z)

    for _ in range(20):
        simulation_app.update()

    chassis_link_path = f"{carter_path}/chassis_link"
    base_link_path = f"{m0609_path}/base_link"

    # --- Nova Carter 현재 질량 조회(질량 보강 계획의 기준값) ---
    chassis_prim = stage.GetPrimAtPath(chassis_link_path)
    mass_api = UsdPhysics.MassAPI(chassis_prim) if chassis_prim.HasAPI(UsdPhysics.MassAPI) else None
    if mass_api:
        print(f"[질량 조회] {chassis_link_path} mass={mass_api.GetMassAttr().Get()} "
              f"(0/미설정이면 밀도 기반 자동계산)", flush=True)
    else:
        print(f"[질량 조회] {chassis_link_path}에 MassAPI 없음 - 밀도 기반 자동계산 중일 가능성", flush=True)
    # Nova Carter 전체 서브트리에서 MassAPI 있는 다른 prim도 훑어본다(질량이 어디 실려있는지 확인).
    for prim in Usd.PrimRange(stage.GetPrimAtPath(carter_path)):
        if prim.HasAPI(UsdPhysics.MassAPI):
            m = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
            print(f"[질량 조회] {prim.GetPath()} mass={m}", flush=True)

    # --- 기존 root_joint(FixedJoint) 제거하고 Z축 PrismaticJoint로 교체 ---
    root_joint_path = f"{m0609_path}/root_joint"
    root_joint_prim = stage.GetPrimAtPath(root_joint_path)
    if root_joint_prim.IsValid():
        stage.RemovePrim(root_joint_path)
        print(f"[제거] 기존 {root_joint_path}(FixedJoint)", flush=True)

    lift_joint_path = f"{m0609_path}/lift_joint"
    lift_joint = UsdPhysics.PrismaticJoint.Define(stage, lift_joint_path)
    lift_joint.CreateAxisAttr("Z")
    lift_joint.CreateBody0Rel().SetTargets([Sdf.Path(chassis_link_path)])
    lift_joint.CreateBody1Rel().SetTargets([Sdf.Path(base_link_path)])
    lift_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    lift_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    lift_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    lift_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    lift_joint.CreateLowerLimitAttr().Set(LIFT_MIN)
    lift_joint.CreateUpperLimitAttr().Set(LIFT_MAX)
    drive = UsdPhysics.DriveAPI.Apply(lift_joint.GetPrim(), "linear")
    drive.CreateTypeAttr().Set("force")
    drive.CreateStiffnessAttr().Set(LIFT_DRIVE_STIFFNESS)
    drive.CreateDampingAttr().Set(LIFT_DRIVE_DAMPING)
    drive.CreateMaxForceAttr().Set(LIFT_DRIVE_MAX_FORCE)
    print(f"[생성] {lift_joint_path} PrismaticJoint(Z), limit=[{LIFT_MIN},{LIFT_MAX}]", flush=True)

    # RG2FT 병합 때와 동일하게, M0609 서브트리 안에 남아있을 수 있는 stray articulation 흔적 제거.
    stray_prim = stage.GetPrimAtPath(f"{m0609_path}/onrobot_rg2ft/world")
    if stray_prim.IsValid() and stray_prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
        stray_prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)

    n = add_drive_stiffness(stage, m0609_path)
    print(f"[DRIVE] {n}개 조인트(리프트 포함) 강성 재설정", flush=True)

    return carter_path, chassis_link_path, m0609_path


# ================= 씬 구성 =================
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

area_light = UsdLux.SphereLight.Define(stage, "/World/VerifyAreaLight")
area_light.CreateRadiusAttr().Set(0.5)
area_light.CreateIntensityAttr().Set(30000)
UsdGeom.Xformable(area_light).AddTranslateOp().Set(Gf.Vec3d(0.5, 0.5, 2.0))

carter_path, chassis_link_path, m0609_path = build_mobile_manipulator_with_lift(stage)
ee_path = f"{m0609_path}/link_6"

# 리프트만 테스트하는 단계라 그리퍼는 최소 구성(SurfaceGripper 상속, 실제 흡착 로직은 안 씀).
gripper = SurfaceGripper(end_effector_prim_path=ee_path, surface_gripper_path="")
robot = SingleManipulator(
    prim_path=chassis_link_path,
    end_effector_prim_path=ee_path,
    name="mobile_manipulator",
    gripper=gripper,
)

world.reset()
robot.initialize(physics_sim_view=world.physics_sim_view)

init_joints = np.zeros(robot.num_dof)
idx_lift = robot.dof_names.index("lift_joint")
idx_j3 = robot.dof_names.index("joint_3")
idx_j5 = robot.dof_names.index("joint_5")
init_joints[idx_lift] = LIFT_MIN
init_joints[idx_j3] = np.pi / 2
init_joints[idx_j5] = np.pi / 2
robot.set_joint_positions(init_joints)
for _ in range(60):
    world.step(render=True)
print(f"\n[안정화 완료] lift_joint idx={idx_lift}, dof_names={robot.dof_names}\n", flush=True)

viewport = vp_util.get_active_viewport()


def snapshot(eye, target, fname):
    set_camera_view(eye=eye, target=target)
    for _ in range(15):
        world.step(render=True)
    out = str(_THIS_DIR / fname)
    vp_util.capture_viewport_to_file(viewport, out)
    for _ in range(5):
        world.step(render=True)
    print(f"[SCREENSHOT] {out}", flush=True)


def chassis_tilt_deg():
    _, quat = robot.get_world_pose()
    roll, pitch, _ = quat_to_euler_angles(quat)
    return np.degrees(roll), np.degrees(pitch)


snapshot(eye=[1.5, 1.5, 1.2], target=[0.0, 0.0, 0.6], fname="_lift_diag_00_start.png")
roll0, pitch0 = chassis_tilt_deg()
print(f"[기울기] 시작 roll={roll0:.2f}deg pitch={pitch0:.2f}deg", flush=True)

lift_joint_prim = stage.GetPrimAtPath(f"{m0609_path}/lift_joint")
lift_drive = UsdPhysics.DriveAPI(lift_joint_prim, "linear")
print(f"[진단] lift_joint 존재={lift_joint_prim.IsValid()} "
      f"lowerLimit={UsdPhysics.PrismaticJoint(lift_joint_prim).GetLowerLimitAttr().Get()} "
      f"upperLimit={UsdPhysics.PrismaticJoint(lift_joint_prim).GetUpperLimitAttr().Get()} "
      f"axis={UsdPhysics.PrismaticJoint(lift_joint_prim).GetAxisAttr().Get()} "
      f"stiffness={lift_drive.GetStiffnessAttr().Get()} damping={lift_drive.GetDampingAttr().Get()} "
      f"maxForce={lift_drive.GetMaxForceAttr().Get()} targetPos(초기)={lift_drive.GetTargetPositionAttr().Get()}",
      flush=True)

# USD 드라이브 target을 직접 Set()해도 실제로 안 움직였다(joint state가 0.420 고정) - 즉
# 명령 전달 문제가 아니라 조인트 자체가 물리적으로 안 움직이는 것으로 보임. 바퀴에서는
# 확실히 되는 velocity 제어로 같은 조인트를 테스트해서, position 드라이브만의 문제인지
# 아니면 이 조인트 자체가 아예 안 움직이는지 구분한다.
print("\n[진단] velocity 제어로 테스트 (바퀴에서 검증된 방식과 동일)\n", flush=True)
for _ in range(150):
    robot.apply_action(ArticulationAction(joint_velocities=[0.3], joint_indices=[idx_lift]))
    world.step(render=True)
vel_test_pos = robot.get_joint_positions()[idx_lift]
print(f"[진단] velocity=0.3으로 150스텝 후 lift position={vel_test_pos:.4f} (0.420에서 변화 있어야 함)", flush=True)
robot.apply_action(ArticulationAction(joint_velocities=[0.0], joint_indices=[idx_lift]))
for _ in range(30):
    world.step(render=True)

for i, h in enumerate(LIFT_TEST_HEIGHTS):
    # apply_action(joint_positions=...)이 이 조인트엔 안 먹혔으므로(targetPos(USD)가 계속 0.0),
    # USD 드라이브 attribute를 직접 Set()해서 조인트/드라이브 설정 자체는 정상인지 먼저 확인.
    lift_drive.GetTargetPositionAttr().Set(float(h))
    for step_i in range(150):
        robot.apply_action(ArticulationAction(joint_positions=[h], joint_indices=[idx_lift]))
        world.step(render=True)
        if step_i == 0:
            print(f"[진단] targetPos(USD, 직접Set 이후)={lift_drive.GetTargetPositionAttr().Get()}", flush=True)
    actual_positions = robot.get_joint_positions()
    actual_lift = actual_positions[idx_lift]
    base_pos, _ = robot.end_effector.get_world_pose()  # link_6 world pos (참고용)
    roll, pitch = chassis_tilt_deg()
    print(f"[리프트 {i}] target={h:.3f} actual={actual_lift:.3f} err={abs(actual_lift-h):.4f} "
          f"link6_world_z={base_pos[2]:.3f} chassis_roll={roll:.2f}deg pitch={pitch:.2f}deg", flush=True)
    snapshot(eye=[1.5, 1.5, 1.2], target=[0.0, 0.0, h], fname=f"_lift_diag_{i+1:02d}_h{h:.2f}.png")

print("\n[안내] 리프트 진단 완료.\n", flush=True)
simulation_app.close()
