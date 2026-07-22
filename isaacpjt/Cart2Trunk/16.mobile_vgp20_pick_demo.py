"""
Nova Carter + M0609 + VGP20(흡착 그리퍼 + 옆면 RealSense 카메라) 결합 1차 검증.

9.simple_mobile_pick_demo.py(Nova Carter 주행 + M0609/RG2FT pick&place 결합 검증)와
M0609/8.vgp20_pick_place.py(VGP20 흡착 그리퍼 DynamicSuctionGripper + RMPflow pick&place,
고정 베이스)를 합친 것 - 그리퍼만 RG2FT에서 VGP20(+옆면 카메라)로 바꾸고, 나머지 두 조합
방식(Nova Carter 위 M0609 articulation 병합, 이동 후 RMPflow 베이스 포즈 보정)은 그대로 재사용한다.

범위(1단계 검증): 실제 카트/트렁크 환경이 아니라 9번 스크립트와 동일하게 단순화된 환경에서
"이동 -> 흡착 pick&place"가 병합 articulation + VGP20 조합에서도 정상 동작하는지만 확인한다.
실제 카트→트렁크 배치는 이 검증 통과 후 다음 단계에서 진행한다.

[핵심 차이점 - RG2FT -> VGP20]
  - 그리퍼: ParallelGripper(손가락 열고 닫기) -> DynamicSuctionGripper(근접하면 런타임에
    FixedJoint 생성/삭제, M0609/8.vgp20_pick_place.py에서 그대로 가져옴).
  - 그리퍼 바디 경로: onrobot_rg2ft -> vgp20 (link_6에 FixedJoint로 부착된 별도 강체).
  - EE_OFFSET: RG2FT는 고정값(0,0,0.2). VGP20은 흡착팁이 vgp20 로컬 +Z로 0.121m 떨어져
    있으므로 EE_OFFSET.z = TIP_LOCAL_OFFSET.z + CUBE_SIZE/2 + gap 로 계산해야 한다
    (8.vgp20_pick_place.py와 동일 공식).
  - 드라이브 damping: 1e4 -> 1e6 (흡착된 큐브를 매달고 바닥 근처에서 진동하는 문제,
    8번 스크립트에서 이미 겪고 고친 값을 그대로 사용).
  - 카메라: RSD455를 그리퍼 옆면에 부착(9.attach_vgp20_camera.py에서 빌드+검증), RigidBodyAPI가
    vgp20 밑에 중첩되지 않도록 미리 제거해뒀음 - 병합 articulation에서 문제 없어야 함.

[9번 스크립트에서 검증된 "도달 가능 영역" 그대로 재사용]
  9번 스크립트의 reach sweep 결과: link_6의 팔 베이스 기준 상대 목표 높이가 0.30m(박스)/
  0.45m(리트랙트)일 때 오차 0에 가깝게 정확히 도달했다(베이스 높이 근처는 자기충돌로 거부).
  VGP20은 EE_OFFSET.z가 다르므로(0.149 vs RG2FT의 0.2), 같은 link_6 목표 높이(0.30/0.45)가
  나오도록 큐브 배치 높이(BOX_REL_Z)를 역산해서 맞춘다: BOX_REL_Z = 0.30 - EE_OFFSET.z.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import sys

import numpy as np
import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid, FixedCuboid
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import quat_to_euler_angles
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
import isaacsim.robot.manipulators.controllers as manipulators_controllers

_THIS_DIR = Path(__file__).resolve().parent
M0609_DIR = _THIS_DIR.parent / "M0609"
RMPFLOW_DIR = str(M0609_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController  # noqa: E402

M0609_USD = str(M0609_DIR / "Collected_m0609_vgp20_camera" / "m0609_vgp20_camera.usd")
M0609_URDF_PATH = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH = str(M0609_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(M0609_DIR / "rmpflow/m0609_rmpflow_common.yaml")

# ---------------- 설정값 ----------------
ROBOT_START_XY = (0.0, 0.0)
FACE_ROT_Z = 0.0
MOUNT_Z = 0.42
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e8, 1e6, 1e8  # damping 1e6: 흡착 큐브 흔들림 방지(8번 스크립트와 동일)

DRIVE_DISTANCE = 0.70
WHEEL_SPEED = 5.0
MAX_DRIVE_STEPS = 1200

EE_LINK_NAME = "link_6"
GRIPPER_BODY_NAME = "vgp20"

TIP_LOCAL_OFFSET = (0.0, 0.0, 0.121)  # vgp20 로컬 +Z 기준 흡착팁(컵 어레이) 위치
GRASP_RADIUS = 0.075

CUBE_SIZE = 0.05
CUBE_MASS = 0.05
VISUAL_HANG_GAP = 0.003
EE_OFFSET = np.array([0.0, 0.0, TIP_LOCAL_OFFSET[2] + CUBE_SIZE / 2.0 + VISUAL_HANG_GAP])  # link_6 -> 목표높이 오프셋

# 9번 스크립트 reach sweep 결과 재사용: link_6 상대 목표 높이 0.30(박스)/0.45(리트랙트)가 검증된
# 도달 가능 지점 - EE_OFFSET.z가 RG2FT(0.2)와 다르므로 그만큼 박스 배치 높이를 역산해서 맞춘다.
LINK6_REL_HEIGHT_PICK = 0.30
LINK6_REL_HEIGHT_RETRACT = 0.45
PICK_LOCAL_XY = (0.35, 0.0)
PLACE_LOCAL_XY = (0.35, 0.30)
BOX_REL_Z = LINK6_REL_HEIGHT_PICK - float(EE_OFFSET[2])  # 큐브 중심의 팔 베이스 기준 상대 높이
RETRACT_REL_Z = LINK6_REL_HEIGHT_RETRACT

CUBE_STATIC, CUBE_DYNAMIC = 1.2, 1.0

PLACE_CLEARANCE = 0.015  # 8번 스크립트와 동일: 내려놓기 직전 살짝 띄워서 흔들림 방지
EVENTS_DT = [0.008, 0.002, 0.05, 0.05, 0.0025, 0.002, 0.001, 0.2, 0.008, 0.08]


# ╔══════════════════════════════════════════════════════════════╗
# ║  DynamicSuctionGripper (M0609/8.vgp20_pick_place.py 그대로)      ║
# ╚══════════════════════════════════════════════════════════════╝
class DynamicSuctionGripper(SurfaceGripper):
    """근접하면 FixedJoint를 런타임에 생성/삭제해서 흡착을 흉내내는 그리퍼."""

    def __init__(self, end_effector_prim_path: str, gripper_body_path: str, target_prim_path: str,
                 tip_local_offset=(0.0, 0.0, 0.0), grasp_radius: float = 0.03):
        SurfaceGripper.__init__(self, end_effector_prim_path=end_effector_prim_path, surface_gripper_path="")
        self._gripper_body_path = gripper_body_path
        self._target_prim_path = target_prim_path
        self._tip_local_offset = Gf.Vec3d(*tip_local_offset)
        self._grasp_radius = grasp_radius
        self._joint_path = f"{gripper_body_path}/suction_attach_joint"
        self._attached = False

    def close(self) -> None:
        if self._attached:
            return
        stage = omni.usd.get_context().get_stage()
        target_prim = stage.GetPrimAtPath(self._target_prim_path)
        if not target_prim.IsValid():
            return
        gripper_mat = UsdGeom.Xformable(stage.GetPrimAtPath(self._gripper_body_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        target_mat = UsdGeom.Xformable(target_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        target_pos = target_mat.ExtractTranslation()
        tip_world = gripper_mat.Transform(self._tip_local_offset)
        dist = (tip_world - target_pos).GetLength()
        if dist > self._grasp_radius:
            return
        rel_local = target_mat.GetInverse().Transform(tip_world)
        gripper_rot = gripper_mat.ExtractRotationQuat()
        target_rot = target_mat.ExtractRotationQuat()
        local_rot1 = target_rot.GetInverse() * gripper_rot
        joint = UsdPhysics.FixedJoint.Define(stage, self._joint_path)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(self._gripper_body_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(self._target_prim_path)])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(self._tip_local_offset))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(rel_local))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(local_rot1))
        self._attached = True
        print(f"  [흡착] dist={dist:.4f}m <= {self._grasp_radius}m -> {self._joint_path} 생성", flush=True)

    def open(self) -> None:
        if self._attached:
            stage = omni.usd.get_context().get_stage()
            if stage.GetPrimAtPath(self._joint_path).IsValid():
                stage.RemovePrim(self._joint_path)
            print(f"  [해제] {self._joint_path} 삭제", flush=True)
        self._attached = False

    def is_closed(self) -> bool:
        return self._attached

    def is_open(self) -> bool:
        return not self._attached


# ╔══════════════════════════════════════════════════════════════╗
# ║  Nova Carter + M0609(+VGP20+카메라) 결합 (9번 스크립트와 동일 방식)  ║
# ╚══════════════════════════════════════════════════════════════╝
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


def build_mobile_manipulator(stage):
    root = get_assets_root_path()
    carter_url = root + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"

    carter_path = "/World/MobileManipulator/NovaCarter"
    carter_xform = UsdGeom.Xform.Define(stage, carter_path)
    carter_xform.GetPrim().GetReferences().AddReference(carter_url)
    carter_xform.ClearXformOpOrder()
    carter_xform.AddTranslateOp().Set(Gf.Vec3d(ROBOT_START_XY[0], ROBOT_START_XY[1], 0.0))
    carter_xform.AddRotateZOp().Set(FACE_ROT_Z)

    m0609_path = "/World/MobileManipulator/M0609"
    m0609_xform = UsdGeom.Xform.Define(stage, m0609_path)
    m0609_xform.GetPrim().GetReferences().AddReference(M0609_USD, "/World/m0609")
    m0609_xform.ClearXformOpOrder()
    m0609_xform.AddTranslateOp().Set(Gf.Vec3d(ROBOT_START_XY[0], ROBOT_START_XY[1], MOUNT_Z))
    m0609_xform.AddRotateZOp().Set(FACE_ROT_Z)

    for _ in range(20):
        simulation_app.update()

    chassis_link_path = f"{carter_path}/chassis_link"
    root_joint_prim = stage.GetPrimAtPath(f"{m0609_path}/root_joint")
    joint = UsdPhysics.Joint(root_joint_prim)
    root_joint_prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    root_joint_prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
    joint.GetBody0Rel().SetTargets([Sdf.Path(chassis_link_path)])
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, MOUNT_Z))
    print(f"[root_joint] body0 -> {chassis_link_path}, localPos0 -> (0,0,{MOUNT_Z})", flush=True)

    n = add_drive_stiffness(stage, m0609_path)
    print(f"[DRIVE] {n}개 조인트 강성 재설정", flush=True)

    return carter_path, chassis_link_path, m0609_path


# ================= 씬 구성 =================
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

carter_path, chassis_link_path, m0609_path = build_mobile_manipulator(stage)
gripper_body_path = f"{m0609_path}/{GRIPPER_BODY_NAME}"
ee_path = f"{m0609_path}/{EE_LINK_NAME}"

gripper = DynamicSuctionGripper(
    end_effector_prim_path=ee_path,
    gripper_body_path=gripper_body_path,
    target_prim_path="/World/PickBox",
    tip_local_offset=TIP_LOCAL_OFFSET,
    grasp_radius=GRASP_RADIUS,
)
robot = SingleManipulator(
    prim_path=chassis_link_path,
    end_effector_prim_path=ee_path,
    name="mobile_manipulator",
    gripper=gripper,
)

world.reset()
robot.initialize(physics_sim_view=world.physics_sim_view)
robot.gripper.initialize(physics_sim_view=world.physics_sim_view, articulation_num_dofs=robot.num_dof)
robot.set_joint_positions(np.zeros(robot.num_dof))
for _ in range(30):
    world.step(render=True)
print("\n[안정화 완료] 팔이 정상 기본 자세로 정지\n", flush=True)

# ================= 1. Nova Carter 이동 검증 =================
idx_left = robot.dof_names.index("joint_wheel_left")
idx_right = robot.dof_names.index("joint_wheel_right")
start_pos, _ = robot.get_world_pose()
start_x = float(start_pos[0])
print(f"[주행 시작] start=({start_pos[0]:.3f},{start_pos[1]:.3f})", flush=True)

wheel_vel = WHEEL_SPEED
robot.apply_action(ArticulationAction(joint_velocities=[wheel_vel, wheel_vel], joint_indices=[idx_left, idx_right]))
for _ in range(30):
    world.step(render=True)
cur_pos, _ = robot.get_world_pose()
if (cur_pos[0] - start_x) < 0.02:
    wheel_vel = -wheel_vel
    print(f"[주행] 부호 반전 -> wheel_vel={wheel_vel}", flush=True)
    robot.apply_action(ArticulationAction(joint_velocities=[wheel_vel, wheel_vel], joint_indices=[idx_left, idx_right]))

step_count = 0
while step_count < MAX_DRIVE_STEPS:
    world.step(render=True)
    step_count += 1
    cur_pos, _ = robot.get_world_pose()
    if abs(cur_pos[0] - start_x) >= DRIVE_DISTANCE:
        break
else:
    print("[경고] MAX_DRIVE_STEPS 안에 목표 거리 도달 못 함 - 현재 위치에서 강제 정지", flush=True)

robot.apply_action(ArticulationAction(joint_velocities=[0.0, 0.0], joint_indices=[idx_left, idx_right]))
for _ in range(60):
    world.step(render=True)

chassis_pos, chassis_quat = robot.get_world_pose()
print(
    f"[주행 완료] {step_count}스텝 이동, 최종 위치=({chassis_pos[0]:.3f},{chassis_pos[1]:.3f}), "
    f"이동거리={chassis_pos[0]-start_x:.3f}m",
    flush=True,
)

# ================= 2. 정지 위치 기준으로 스탠드 + 박스 배치 =================
yaw = float(quat_to_euler_angles(chassis_quat)[2])
base_pos = np.array([chassis_pos[0], chassis_pos[1], chassis_pos[2] + MOUNT_Z])
base_quat = chassis_quat


def local_to_world_xy(local_xy):
    dx, dy = local_xy
    wx = base_pos[0] + dx * np.cos(yaw) - dy * np.sin(yaw)
    wy = base_pos[1] + dx * np.sin(yaw) + dy * np.cos(yaw)
    return wx, wy


pick_x, pick_y = local_to_world_xy(PICK_LOCAL_XY)
place_x, place_y = local_to_world_xy(PLACE_LOCAL_XY)
box_world_z = float(base_pos[2]) + BOX_REL_Z
stand_top_z = box_world_z - CUBE_SIZE / 2.0
print(f"[배치] 팔 베이스 실측 위치=({base_pos[0]:.3f},{base_pos[1]:.3f},{base_pos[2]:.3f}) yaw={np.degrees(yaw):.1f}deg", flush=True)
print(f"[배치] pick=({pick_x:.3f},{pick_y:.3f}) place=({place_x:.3f},{place_y:.3f}) stand_top_z={stand_top_z:.3f} BOX_REL_Z={BOX_REL_Z:.3f}", flush=True)

margin = 0.15
xs, ys = [pick_x, place_x], [pick_y, place_y]
stand_cx, stand_cy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
stand_sx, stand_sy = (max(xs) - min(xs)) + 2 * margin, (max(ys) - min(ys)) + 2 * margin

stand = FixedCuboid(
    prim_path="/World/CartHeightStand",
    name="cart_height_stand",
    position=np.array([stand_cx, stand_cy, stand_top_z / 2.0]),
    scale=np.array([stand_sx, stand_sy, stand_top_z]),
    color=np.array([0.55, 0.45, 0.35]),
)

box_material = PhysicsMaterial(
    prim_path="/World/Physics_Materials/box_material",
    static_friction=CUBE_STATIC,
    dynamic_friction=CUBE_DYNAMIC,
    restitution=0.0,
)
box = DynamicCuboid(
    prim_path="/World/PickBox",
    name="pick_box",
    position=np.array([pick_x, pick_y, stand_top_z + CUBE_SIZE / 2.0]),
    scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]),
    color=np.array([1.0, 0.15, 0.0]),
    mass=CUBE_MASS,
    physics_material=box_material,
)
goal_marker = VisualCuboid(
    prim_path="/World/GoalMarker",
    name="goal_marker",
    position=np.array([place_x, place_y, stand_top_z + 0.002]),
    scale=np.array([CUBE_SIZE * 1.3, CUBE_SIZE * 1.3, 0.002]),
    color=np.array([0.1, 0.9, 0.2]),
)

for _ in range(40):
    world.step(render=True)

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


snapshot(
    eye=[base_pos[0] - 0.8, base_pos[1] - 1.2, base_pos[2] + 1.0],
    target=[pick_x, pick_y, stand_top_z],
    fname="_verify_vgp20_mobile_pick_before.png",
)

# ================= 3. RMPflow pick&place 컨트롤러 구성 =================
GOAL_POS = np.array([place_x, place_y, box_world_z])
EE_INITIAL_HEIGHT = float(base_pos[2]) + RETRACT_REL_Z

controller = manipulators_controllers.PickPlaceController(
    name="mobile_vgp20_pick_place_controller",
    cspace_controller=RMPFlowController(
        name="mobile_vgp20_cspace_controller",
        robot_articulation=robot,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=M0609_DESCRIPTION_PATH,
        rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
    ),
    gripper=robot.gripper,
    end_effector_initial_height=EE_INITIAL_HEIGHT,
    events_dt=EVENTS_DT,
)

# --- RMPflow 베이스 포즈 보정 (9번 스크립트와 동일 이유: articulation 루트=chassis_link) ---
cspace = controller._cspace_controller
cspace._default_position = base_pos
cspace._default_orientation = base_quat
cspace.rmp_flow.set_robot_base_pose(robot_position=base_pos, robot_orientation=base_quat)
print(f"[RMPflow 보정] base_pose -> pos={base_pos}, quat={base_quat}", flush=True)
ee_pos0, _ = robot.end_effector.get_world_pose()
print(f"[EE 초기 위치] {ee_pos0}  (pick target={pick_x:.3f},{pick_y:.3f},{stand_top_z + CUBE_SIZE/2:.3f})", flush=True)

# ================= 4. Pick & Place 실행 =================
print("\n[Pick & Place 시작]\n", flush=True)
last_event = -1
lifted_shot_done = False
step_i = 0
max_steps = 3000
while not controller.is_done() and step_i < max_steps:
    world.step(render=True)
    step_i += 1
    cube_pos, _ = box.get_world_pose()
    current_joints = robot.get_joint_positions()
    actions = controller.forward(
        picking_position=cube_pos,
        placing_position=GOAL_POS + np.array([0.0, 0.0, PLACE_CLEARANCE]),
        current_joint_positions=current_joints,
        end_effector_offset=EE_OFFSET,
    )
    robot.apply_action(actions)

    ev = controller.get_current_event()
    if ev != last_event:
        ee_pos, _ = robot.end_effector.get_world_pose()
        dist = np.linalg.norm(ee_pos - cube_pos)
        print(
            f"[event {last_event} -> {ev}] step={step_i} cube_pos=({cube_pos[0]:.3f},{cube_pos[1]:.3f},{cube_pos[2]:.3f}) "
            f"ee_pos=({ee_pos[0]:.3f},{ee_pos[1]:.3f},{ee_pos[2]:.3f}) ee-cube_dist={dist:.3f}",
            flush=True,
        )
        last_event = ev
    elif step_i % 60 == 0:
        ee_pos, _ = robot.end_effector.get_world_pose()
        dist = np.linalg.norm(ee_pos - cube_pos)
        print(f"[진행] step={step_i} event={ev} ee_pos=({ee_pos[0]:.3f},{ee_pos[1]:.3f},{ee_pos[2]:.3f}) ee-cube_dist={dist:.3f}", flush=True)
    if ev >= 4 and not lifted_shot_done:
        snapshot(
            eye=[base_pos[0] - 1.0, base_pos[1] - 1.0, base_pos[2] + 1.2],
            target=[pick_x, pick_y, stand_top_z],
            fname="_verify_vgp20_mobile_pick_lifted.png",
        )
        lifted_shot_done = True

if step_i >= max_steps:
    print(f"\n[경고] max_steps({max_steps})에 도달 - controller.is_done()={controller.is_done()}", flush=True)

final_box_pos, _ = box.get_world_pose()
print(f"\n[완료] 최종 박스 위치=({final_box_pos[0]:.3f},{final_box_pos[1]:.3f},{final_box_pos[2]:.3f})", flush=True)
print(f"[완료] 목표 위치=({GOAL_POS[0]:.3f},{GOAL_POS[1]:.3f},{GOAL_POS[2]:.3f})", flush=True)
error = np.linalg.norm(final_box_pos - GOAL_POS)
print(f"[완료] 목표와의 거리 오차={error:.4f}m -> {'PASS' if error < 0.05 else 'FAIL'}", flush=True)

snapshot(
    eye=[base_pos[0] - 0.8, base_pos[1] - 1.2, base_pos[2] + 1.0],
    target=[place_x, place_y, stand_top_z],
    fname="_verify_vgp20_mobile_pick_after.png",
)

print("\n[안내] 검증 완료.\n", flush=True)
simulation_app.close()
