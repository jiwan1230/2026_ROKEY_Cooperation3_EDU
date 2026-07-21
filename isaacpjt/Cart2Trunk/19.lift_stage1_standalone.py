"""
19.lift_stage1_standalone.py
리프트 검증 1단계: Nova Carter도 M0609도 전혀 없이, 단순 박스 2개만으로 Z축 프리즘
조인트+포지션 드라이브가 이 Isaac Sim 환경에서 애초에 정상 동작하는지부터 확인한다.

사용자 지적: "한 번에 다 하려고 하지 말고 단계를 쪼개라" - 18번 스크립트에서 Nova Carter+
M0609+프리즘 조인트를 한번에 합쳐서 테스트했다가 전혀 안 움직이는 문제를 만났는데, 그게
조인트/드라이브 설정 자체의 문제인지 Nova Carter/M0609 결합 과정에서 생긴 문제인지 구분이
안 됐다. 여기서는 가장 단순한 2-body 케이스로 프리즘 조인트 자체의 동작만 확인한다.

구조: /World/Base(바닥에 고정, ArticulationRoot) -- PrismaticJoint(Z) --> /World/Platform(위아래로 움직여야 함)
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path

import numpy as np
import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view

_THIS_DIR = Path(__file__).resolve().parent

LIFT_MIN = 0.0
LIFT_MAX = 0.6
TEST_HEIGHTS = [0.0, 0.2, 0.4, 0.6, 0.3]
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e8, 1e4, 1e8

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

area_light = UsdLux.SphereLight.Define(stage, "/World/VerifyAreaLight")
area_light.CreateRadiusAttr().Set(0.5)
area_light.CreateIntensityAttr().Set(30000)
UsdGeom.Xformable(area_light).AddTranslateOp().Set(Gf.Vec3d(0.5, 0.5, 2.0))

# ---- Base: 바닥에 고정된 작은 박스, articulation root ----
base_path = "/World/LiftBase"
base_cube = UsdGeom.Cube.Define(stage, base_path)
base_cube.GetSizeAttr().Set(1.0)
base_xform = UsdGeom.Xformable(base_cube)
base_xform.ClearXformOpOrder()
base_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.15))
base_xform.AddScaleOp().Set(Gf.Vec3f(0.3, 0.3, 0.3))
base_cube.CreateDisplayColorAttr([Gf.Vec3f(0.3, 0.3, 0.3)])
base_prim = base_cube.GetPrim()
UsdPhysics.RigidBodyAPI.Apply(base_prim)
UsdPhysics.CollisionAPI.Apply(base_prim)
UsdPhysics.MassAPI.Apply(base_prim).CreateMassAttr().Set(50.0)

# world에 고정하는 FixedJoint (바닥에 못박기)
anchor_joint = UsdPhysics.FixedJoint.Define(stage, f"{base_path}/anchor_joint")
anchor_joint.CreateBody1Rel().SetTargets([Sdf.Path(base_path)])
anchor_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
UsdPhysics.ArticulationRootAPI.Apply(base_prim)

# ---- Platform: base 위에서 Z로 움직이는 박스 ----
platform_path = "/World/LiftPlatform"
platform_cube = UsdGeom.Cube.Define(stage, platform_path)
platform_cube.GetSizeAttr().Set(1.0)
platform_xform = UsdGeom.Xformable(platform_cube)
platform_xform.ClearXformOpOrder()
platform_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.3 + LIFT_MIN))
platform_xform.AddScaleOp().Set(Gf.Vec3f(0.25, 0.25, 0.1))
platform_cube.CreateDisplayColorAttr([Gf.Vec3f(0.9, 0.2, 0.1)])
platform_prim = platform_cube.GetPrim()
UsdPhysics.RigidBodyAPI.Apply(platform_prim)
UsdPhysics.CollisionAPI.Apply(platform_prim)
UsdPhysics.MassAPI.Apply(platform_prim).CreateMassAttr().Set(5.0)

for _ in range(10):
    simulation_app.update()

# ---- 프리즘 조인트(Z축)로 base<->platform 연결 ----
lift_joint_path = f"{base_path}/lift_joint"
lift_joint = UsdPhysics.PrismaticJoint.Define(stage, lift_joint_path)
lift_joint.CreateAxisAttr("Z")
lift_joint.CreateBody0Rel().SetTargets([Sdf.Path(base_path)])
lift_joint.CreateBody1Rel().SetTargets([Sdf.Path(platform_path)])
lift_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.15))  # base 원점 기준 base 윗면
lift_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
lift_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))  # platform 자기 원점
lift_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
lift_joint.CreateLowerLimitAttr().Set(LIFT_MIN)
lift_joint.CreateUpperLimitAttr().Set(LIFT_MAX)
drive = UsdPhysics.DriveAPI.Apply(lift_joint.GetPrim(), "linear")
drive.CreateTypeAttr().Set("force")
drive.CreateStiffnessAttr().Set(DRIVE_STIFFNESS)
drive.CreateDampingAttr().Set(DRIVE_DAMPING)
drive.CreateMaxForceAttr().Set(DRIVE_MAX_FORCE)
print(f"[생성] {lift_joint_path} limit=[{LIFT_MIN},{LIFT_MAX}]", flush=True)

world.reset()
robot = SingleArticulation(prim_path=base_path, name="lift_test")
robot.initialize(physics_sim_view=world.physics_sim_view)
print(f"[안정화] dof_names={robot.dof_names}", flush=True)
idx_lift = robot.dof_names.index("lift_joint")

for _ in range(60):
    world.step(render=True)

viewport = vp_util.get_active_viewport()


def snapshot(fname):
    set_camera_view(eye=[1.2, 1.2, 1.0], target=[0.0, 0.0, 0.4])
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


print(f"[시작] platform world z={platform_world_z():.4f} (기대값: base윗면 0.3+{LIFT_MIN}=0.3)", flush=True)
snapshot("_stage1_00_start.png")

for i, h in enumerate(TEST_HEIGHTS):
    for _ in range(120):
        robot.apply_action(ArticulationAction(joint_positions=[h], joint_indices=[idx_lift]))
        world.step(render=True)
    actual = robot.get_joint_positions()[idx_lift]
    wz = platform_world_z()
    print(f"[테스트 {i}] target={h:.3f} joint_position={actual:.4f} platform_world_z={wz:.4f} "
          f"(기대 world_z ≈ 0.3+{h:.3f}={0.3+h:.3f})", flush=True)
    snapshot(f"_stage1_{i+1:02d}_h{h:.2f}.png")

print("\n[안내] 1단계(순수 프리즘 조인트) 검증 완료.\n", flush=True)
simulation_app.close()
