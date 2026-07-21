"""
29.carry_pose_calibration.py
사용자 지정 시퀀스의 "⑥ 카트 밖에서 팔을 안전 운반 자세로 접는다" 단계에 쓸
CARRY_JOINT_POSITIONS 상수를 찾기 위한 보정 스크립트.

카트/트렁크/전체 PICK 시퀀스는 필요 없다 - "이미 박스를 흡착한 채 카트 밖에서 정지한
상태"만 재현하면 되므로, Nova Carter+M0609(+리프트)만 놓고 박스를 그리퍼에 FixedJoint로
바로 붙인 뒤(진짜 흡착 판정 로직 없이), 후보 조인트 벡터 하나를 적용해서:
  - 박스 world AABB, NovaCarter 전체 world AABB, 리프트 컬럼 world AABB가 서로 겹치는지
    (지금은 서로 콜리전이 꺼져 있어서 겹쳐도 안 튕기므로 직접 계산해야 한다)
  - 박스 중심과 Carter 중심의 XY 거리
  - 스크린샷
을 출력한다. 이 세션 내내 해온 방식대로 CANDIDATE 값을 바꿔가며 재실행 -> 확인 -> 조정을
반복해서 좋은 값을 찾은 뒤, 28번 스크립트의 CARRY_JOINT_POSITIONS로 그대로 옮긴다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path

import numpy as np
import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, UsdLux, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

_THIS_DIR = Path(__file__).resolve().parent
M0609_DIR = _THIS_DIR.parent / "M0609"
M0609_USD = str(M0609_DIR / "Collected_m0609_vgp20_camera" / "m0609_vgp20_camera.usd")

LIFT_MIN, LIFT_MAX = 0.42, 1.20
LIFT_TRANSIT_H = 0.80  # 카트를 막 빠져나온 시점의 리프트 높이(28번 스크립트와 동일 값)
CUBE_SIZE = 0.05
LIFT_COLUMN_RADIUS = 0.06
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e8, 1e4, 1e8

# ---- 후보 운반 자세: 이 세션 내내 써온 "시드 자세"(joint_3=90, joint_5=90)를 1차 후보로
# 쓴다 - 지금까지 스크린샷들에서 이 자세가 이미 팔이 위로 접혀서 차체 위에 컴팩트하게
# 얹힌 모양으로 여러 번 확인됐다(9단계 등). 안 좋으면 여기 값만 바꿔서 재실행한다.
CANDIDATE_NAME = "v2_more_elbow"
CARRY_JOINT_POSITIONS = np.array([0.0, 0.0, np.radians(150.0), 0.0, np.pi / 2, 0.0])


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


def bbox_of(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    bbox = bbox_cache.ComputeWorldBound(prim)
    rng = bbox.ComputeAlignedRange()
    return np.array(rng.GetMin()), np.array(rng.GetMax())


def aabbs_overlap(min_a, max_a, min_b, max_b):
    return bool(np.all(min_a <= max_b) and np.all(min_b <= max_a))


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
chassis_link_path = f"{carter_path}/chassis_link"

m0609_path = "/World/MobileManipulator/M0609"
m0609_xform = UsdGeom.Xform.Define(stage, m0609_path)
m0609_xform.GetPrim().GetReferences().AddReference(M0609_USD, "/World/m0609")
m0609_xform.ClearXformOpOrder()
m0609_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, LIFT_TRANSIT_H))

for _ in range(20):
    simulation_app.update()

base_link_path = f"{m0609_path}/base_link"
old_root_joint_path = f"{m0609_path}/root_joint"
if stage.GetPrimAtPath(old_root_joint_path).IsValid():
    stage.RemovePrim(old_root_joint_path)

base_link_prim = stage.GetPrimAtPath(base_link_path)
UsdPhysics.ArticulationRootAPI.Apply(base_link_prim)

chassis_link_prim = stage.GetPrimAtPath(chassis_link_path)
filt_chassis = UsdPhysics.FilteredPairsAPI.Apply(chassis_link_prim)
filt_chassis.CreateFilteredPairsRel().AddTarget(Sdf.Path(base_link_path))
filt_base = UsdPhysics.FilteredPairsAPI.Apply(base_link_prim)
filt_base.CreateFilteredPairsRel().AddTarget(Sdf.Path(chassis_link_path))

lift_column_path = "/World/MobileManipulator/LiftColumnVisual"
lift_column = UsdGeom.Cylinder.Define(stage, lift_column_path)
lift_column.CreateRadiusAttr().Set(LIFT_COLUMN_RADIUS)
lift_column.CreateHeightAttr().Set(1.0)
lift_column.CreateAxisAttr("Z")
lift_column.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.45, 0.1)])
lift_column_xform = UsdGeom.Xformable(lift_column)
lift_column_xform.ClearXformOpOrder()
lift_translate_op = lift_column_xform.AddTranslateOp()
lift_scale_op = lift_column_xform.AddScaleOp()
lift_scale_op.Set(Gf.Vec3f(1.0, 1.0, 0.001))

stray_prim = stage.GetPrimAtPath(f"{m0609_path}/onrobot_rg2ft/world")
if stray_prim.IsValid() and stray_prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
    stray_prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)

n = add_drive_stiffness(stage, m0609_path)
print(f"[DRIVE] M0609={n}개 조인트 강성 적용", flush=True)

ee_path = f"{m0609_path}/link_6"
gripper_body_path = f"{m0609_path}/vgp20"
gripper = SurfaceGripper(end_effector_prim_path=ee_path, surface_gripper_path="")
m0609_robot = SingleManipulator(prim_path=base_link_path, end_effector_prim_path=ee_path, name="m0609_arm", gripper=gripper)
carter_robot = SingleArticulation(prim_path=chassis_link_path, name="carter_base")

world.reset()
carter_robot.initialize(physics_sim_view=world.physics_sim_view)
m0609_robot.initialize(physics_sim_view=world.physics_sim_view)
print(f"[안정화] m0609 dof_names={m0609_robot.dof_names}", flush=True)


def set_lift_height(h):
    chassis_pos, chassis_quat = carter_robot.get_world_pose()
    target_pos = np.array([chassis_pos[0], chassis_pos[1], chassis_pos[2] + h])
    m0609_robot.set_world_pose(position=target_pos, orientation=chassis_quat)
    m0609_robot.set_linear_velocity(np.zeros(3))
    m0609_robot.set_angular_velocity(np.zeros(3))
    column_base_z = chassis_pos[2] + LIFT_MIN
    column_len = max(float(h) - LIFT_MIN, 0.001)
    lift_scale_op.Set(Gf.Vec3f(1.0, 1.0, column_len))
    lift_translate_op.Set(Gf.Vec3d(float(chassis_pos[0]), float(chassis_pos[1]), float(column_base_z + column_len / 2.0)))


for _ in range(60):
    set_lift_height(LIFT_TRANSIT_H)
    world.step(render=True)

# ---- 후보 운반 자세 적용 ----
m0609_robot.set_joint_positions(CARRY_JOINT_POSITIONS)
for _ in range(60):
    set_lift_height(LIFT_TRANSIT_H)
    world.step(render=True)

# ---- 박스를 그리퍼 팁에 바로 FixedJoint로 붙인다 (실제 흡착 판정 로직은 여기선 불필요) ----
gripper_mat = UsdGeom.Xformable(stage.GetPrimAtPath(gripper_body_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
tip_local_offset = Gf.Vec3d(0.0, 0.0, 0.121)
tip_world = gripper_mat.Transform(tip_local_offset)
box = DynamicCuboid(
    prim_path="/World/CarryTestBox",
    name="carry_test_box",
    position=np.array([tip_world[0], tip_world[1], tip_world[2] - CUBE_SIZE / 2.0]),
    scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]),
    color=np.array([1.0, 0.15, 0.0]),
    mass=0.05,
)
for _ in range(10):
    set_lift_height(LIFT_TRANSIT_H)
    world.step(render=True)

box_pos_now, _ = box.get_world_pose()
attach_joint = UsdPhysics.FixedJoint.Define(stage, f"{gripper_body_path}/carry_test_attach")
attach_joint.CreateBody0Rel().SetTargets([Sdf.Path(gripper_body_path)])
attach_joint.CreateBody1Rel().SetTargets([Sdf.Path("/World/CarryTestBox")])
attach_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(tip_local_offset))
attach_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
print(f"[박스 부착] {gripper_body_path} <-fixed-> /World/CarryTestBox", flush=True)

for _ in range(60):
    set_lift_height(LIFT_TRANSIT_H)
    world.step(render=True)

# ---- 측정 ----
box_min, box_max = bbox_of(stage, "/World/CarryTestBox")
carter_min, carter_max = bbox_of(stage, carter_path)
lift_min, lift_max = bbox_of(stage, lift_column_path)
box_vs_carter = aabbs_overlap(box_min, box_max, carter_min, carter_max)
box_vs_lift = aabbs_overlap(box_min, box_max, lift_min, lift_max)

box_pos, _ = box.get_world_pose()
carter_pos, _ = carter_robot.get_world_pose()
xy_dist = float(np.linalg.norm(np.array(box_pos[:2]) - np.array(carter_pos[:2])))

print(f"\n[측정 결과] candidate={CANDIDATE_NAME} joints(deg)={np.degrees(CARRY_JOINT_POSITIONS)}", flush=True)
print(f"  박스 AABB: min={box_min} max={box_max}", flush=True)
print(f"  Carter AABB: min={carter_min} max={carter_max}", flush=True)
print(f"  리프트컬럼 AABB: min={lift_min} max={lift_max}", flush=True)
print(f"  박스<->Carter 겹침: {box_vs_carter}", flush=True)
print(f"  박스<->리프트컬럼 겹침: {box_vs_lift}", flush=True)
print(f"  박스-Carter 중심 XY거리: {xy_dist:.4f}m", flush=True)
print(f"  박스 중심 world pos: {np.round(box_pos,3)}, Carter 중심 world pos: {np.round(carter_pos,3)}", flush=True)

viewport = vp_util.get_active_viewport()


def snapshot(fname, eye, target):
    set_camera_view(eye=eye, target=target)
    for _ in range(10):
        set_lift_height(LIFT_TRANSIT_H)
        world.step(render=True)
    out = str(_THIS_DIR / fname)
    vp_util.capture_viewport_to_file(viewport, out)
    for _ in range(5):
        set_lift_height(LIFT_TRANSIT_H)
        world.step(render=True)
    print(f"[SCREENSHOT] {out}", flush=True)


snapshot(f"_carry_pose_{CANDIDATE_NAME}_front.png", eye=[1.8, 1.8, 1.3], target=[0.0, 0.0, 0.6])
snapshot(f"_carry_pose_{CANDIDATE_NAME}_side.png", eye=[0.1, 2.2, 1.0], target=[0.0, 0.0, 0.6])
snapshot(f"_carry_pose_{CANDIDATE_NAME}_top.png", eye=[0.1, 0.1, 2.8], target=[0.0, 0.0, 0.6])

print("\n[안내] 운반 자세 보정 완료.\n", flush=True)
simulation_app.close()
