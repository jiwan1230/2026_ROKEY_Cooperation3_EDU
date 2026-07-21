"""
21.lift_stage3_with_novacarter.py
리프트 검증 3단계: 실제 Nova Carter 섀시 위에 프리즘 리프트 조인트를 붙이되, M0609는
아직 얹지 않고 리프트 자체(위에 간단한 더미 박스만)만 잘 움직이는지 확인한다.

1단계(박스 2개)·2단계(고정 앵커+M0609)는 둘 다 성공 - 리프트 메커니즘 자체도, M0609의
무거운 다관절 체인을 매다는 것도 문제없었다. 남은 용의자는 Nova Carter 쪽(바퀴/캐스터가
이미 있는 실제 articulation, chassis_link의 ArticulationRootAPI 등)이다.
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
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.prims import SingleArticulation

_THIS_DIR = Path(__file__).resolve().parent

LIFT_MIN, LIFT_MAX = 0.20, 1.20
LIFT_TEST_HEIGHTS = [0.42, 0.60, 0.80]
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

# ---- Nova Carter ----
root = get_assets_root_path()
carter_url = root + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
carter_path = "/World/NovaCarter"
carter_xform = UsdGeom.Xform.Define(stage, carter_path)
carter_xform.GetPrim().GetReferences().AddReference(carter_url)
carter_xform.ClearXformOpOrder()
carter_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
chassis_link_path = f"{carter_path}/chassis_link"

# ---- 더미 플랫폼: M0609 대신 쓰는 단순 박스(리프트 위에서 잘 움직이는지만 확인) ----
platform_path = "/World/DummyPlatform"
platform_cube = UsdGeom.Cube.Define(stage, platform_path)
platform_cube.GetSizeAttr().Set(1.0)
platform_xform = UsdGeom.Xformable(platform_cube)
platform_xform.ClearXformOpOrder()
platform_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, LIFT_MIN))
platform_xform.AddScaleOp().Set(Gf.Vec3f(0.3, 0.3, 0.15))
platform_cube.CreateDisplayColorAttr([Gf.Vec3f(0.9, 0.2, 0.1)])
platform_prim = platform_cube.GetPrim()
UsdPhysics.RigidBodyAPI.Apply(platform_prim)
UsdPhysics.CollisionAPI.Apply(platform_prim)
UsdPhysics.MassAPI.Apply(platform_prim).CreateMassAttr().Set(20.0)  # M0609+VGP20 대략 무게 근사

for _ in range(20):
    simulation_app.update()

lift_joint_path = f"{chassis_link_path}/lift_joint"
lift_joint = UsdPhysics.PrismaticJoint.Define(stage, lift_joint_path)
lift_joint.CreateAxisAttr("Z")
lift_joint.CreateBody0Rel().SetTargets([Sdf.Path(chassis_link_path)])
lift_joint.CreateBody1Rel().SetTargets([Sdf.Path(platform_path)])
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
print(f"[생성] {lift_joint_path} limit=[{LIFT_MIN},{LIFT_MAX}] body0={chassis_link_path} body1={platform_path}", flush=True)

n = add_drive_stiffness(stage, carter_path)
print(f"[DRIVE] {n}개 조인트(Nova Carter 서브트리 - 바퀴/캐스터/리프트) 강성 적용", flush=True)

world.reset()
robot = SingleArticulation(prim_path=chassis_link_path, name="carter_lift_test")
robot.initialize(physics_sim_view=world.physics_sim_view)
print(f"[안정화] dof_names={robot.dof_names}", flush=True)
idx_lift = robot.dof_names.index("lift_joint")

init_joints = np.zeros(robot.num_dof)
init_joints[idx_lift] = LIFT_MIN
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


def platform_world_z():
    mat = UsdGeom.Xformable(platform_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return float(mat.ExtractTranslation()[2])


print(f"[시작] platform world z={platform_world_z():.4f}", flush=True)
snapshot("_stage3_00_start.png")

for i, h in enumerate(LIFT_TEST_HEIGHTS):
    for _ in range(150):
        robot.apply_action(ArticulationAction(joint_positions=[h], joint_indices=[idx_lift]))
        world.step(render=True)
    actual = robot.get_joint_positions()[idx_lift]
    pz = platform_world_z()
    print(f"[테스트 {i}] target={h:.3f} joint_position={actual:.4f} platform_world_z={pz:.4f}", flush=True)
    snapshot(f"_stage3_{i+1:02d}_h{h:.2f}.png")

print("\n[안내] 3단계(Nova Carter+리프트) 검증 완료.\n", flush=True)
simulation_app.close()
