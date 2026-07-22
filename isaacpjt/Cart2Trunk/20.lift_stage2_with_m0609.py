"""
20.lift_stage2_with_m0609.py
리프트 검증 2단계: 단순 고정 앵커(Nova Carter 아님) 위에 프리즘 리프트 조인트로 M0609
(+VGP20) 전체를 얹어서, 리프트를 명령하면 M0609 전체가 같이 움직이는지 확인한다.

1단계(19번, 박스 2개)는 성공(0.0->0.2->0.4까지 정상 추종) - 순수 프리즘 조인트+드라이브
자체는 문제없음을 확인했다. 다만 상한선(0.6)에 닿았을 때 NaN 폭발이 있었으므로, 여기서는
한계값에 여유를 넉넉히 두고 절대 한계 근처까지 안 간다.

이 단계에서 M0609(6-DOF 팔 전체가 매달린 상태)를 얹었을 때도 여전히 리프트가 잘 움직이면,
18번 스크립트(Nova Carter+M0609 풀 콤보)에서 실패했던 원인은 Nova Carter 쪽에 있다는 뜻이고,
여기서도 안 움직이면 M0609(무거운 다관절 체인)를 매달았을 때 생기는 문제라는 뜻이다.
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
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

_THIS_DIR = Path(__file__).resolve().parent
M0609_DIR = _THIS_DIR.parent / "M0609"
M0609_USD = str(M0609_DIR / "Collected_m0609_vgp20_camera" / "m0609_vgp20_camera.usd")

# 1단계에서 상한 근처(0.6=한계 그 자체)에서 NaN 폭발이 있었으므로 한계에 여유를 크게 둔다.
LIFT_MIN, LIFT_MAX = 0.20, 1.20
LIFT_TEST_HEIGHTS = [0.42, 0.60, 0.80]  # 한계(0.20~1.20) 안쪽에서만, 경계 근처는 피함
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

# ---- 고정 앵커: Nova Carter 대신 쓰는 단순 정적 박스(articulation root) ----
anchor_path = "/World/LiftAnchor"
anchor_cube = UsdGeom.Cube.Define(stage, anchor_path)
anchor_cube.GetSizeAttr().Set(1.0)
anchor_xform = UsdGeom.Xformable(anchor_cube)
anchor_xform.ClearXformOpOrder()
anchor_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.15))
anchor_xform.AddScaleOp().Set(Gf.Vec3f(0.4, 0.4, 0.3))
anchor_cube.CreateDisplayColorAttr([Gf.Vec3f(0.3, 0.3, 0.3)])
anchor_prim = anchor_cube.GetPrim()
UsdPhysics.RigidBodyAPI.Apply(anchor_prim)
UsdPhysics.CollisionAPI.Apply(anchor_prim)
UsdPhysics.MassAPI.Apply(anchor_prim).CreateMassAttr().Set(50.0)
anchor_fix = UsdPhysics.FixedJoint.Define(stage, f"{anchor_path}/world_anchor")
anchor_fix.CreateBody1Rel().SetTargets([Sdf.Path(anchor_path)])
UsdPhysics.ArticulationRootAPI.Apply(anchor_prim)

# ---- M0609(+VGP20) 참조 ----
m0609_path = "/World/M0609"
m0609_xform = UsdGeom.Xform.Define(stage, m0609_path)
m0609_xform.GetPrim().GetReferences().AddReference(M0609_USD, "/World/m0609")
m0609_xform.ClearXformOpOrder()
m0609_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, LIFT_MIN))

for _ in range(20):
    simulation_app.update()

base_link_path = f"{m0609_path}/base_link"
old_root_joint_path = f"{m0609_path}/root_joint"
if stage.GetPrimAtPath(old_root_joint_path).IsValid():
    stage.RemovePrim(old_root_joint_path)
    print(f"[제거] {old_root_joint_path}", flush=True)

lift_joint_path = f"{anchor_path}/lift_joint"
lift_joint = UsdPhysics.PrismaticJoint.Define(stage, lift_joint_path)
lift_joint.CreateAxisAttr("Z")
lift_joint.CreateBody0Rel().SetTargets([Sdf.Path(anchor_path)])
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
print(f"[생성] {lift_joint_path} limit=[{LIFT_MIN},{LIFT_MAX}]", flush=True)

stray_prim = stage.GetPrimAtPath(f"{m0609_path}/onrobot_rg2ft/world")
if stray_prim.IsValid() and stray_prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
    stray_prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)

n = add_drive_stiffness(stage, m0609_path)
print(f"[DRIVE] {n}개 M0609 조인트 강성 적용", flush=True)

ee_path = f"{m0609_path}/link_6"
gripper = SurfaceGripper(end_effector_prim_path=ee_path, surface_gripper_path="")
robot = SingleManipulator(prim_path=anchor_path, end_effector_prim_path=ee_path, name="lift_m0609_test", gripper=gripper)

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


print(f"[시작] base_link world z={base_link_world_z():.4f}", flush=True)
snapshot("_stage2_00_start.png")

for i, h in enumerate(LIFT_TEST_HEIGHTS):
    for _ in range(150):
        robot.apply_action(ArticulationAction(joint_positions=[h], joint_indices=[idx_lift]))
        world.step(render=True)
    actual = robot.get_joint_positions()[idx_lift]
    bz = base_link_world_z()
    print(f"[테스트 {i}] target={h:.3f} joint_position={actual:.4f} base_link_world_z={bz:.4f}", flush=True)
    snapshot(f"_stage2_{i+1:02d}_h{h:.2f}.png")

print("\n[안내] 2단계(리프트+M0609) 검증 완료.\n", flush=True)
simulation_app.close()
