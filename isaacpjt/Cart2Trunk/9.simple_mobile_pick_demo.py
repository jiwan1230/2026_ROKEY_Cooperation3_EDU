"""
간단한 검증용 예제 스크립트.

기존 cart2trunk_scene.usd(카트/차량/8개 구역 등 전체 환경)는 쓰지 않고, 이미 성공적으로 결합한
Nova Carter + M0609(7/8번 스크립트에서 검증한 방식)만 새로 가져와서 딱 두 가지만 검증한다:
  1. Nova Carter 실제 차동구동 바퀴를 코드로 굴려서 전진 이동시키기
  2. 이동해서 도착한 지점에서, 팔이 실제로 도달 가능한 높이(카트 정도 높이)의 스탠드 위에 놓인
     작은 박스를 M0609 + RMPflow pick&place 컨트롤러로 집어 올려 옆으로 옮기기

박스는 그리퍼가 쥘 수 있는 크기(5cm 정육면체)에 가벼운 질량(0.1kg)으로 설정한다.

핵심 이슈: M0609/rmpflow 의 PickPlaceController는 로봇이 "고정 베이스"라고 가정하고
robot_articulation.get_world_pose()로 베이스 포즈를 한 번 캡처해서 RMPflow에 넘겨준다.
그런데 지금 병합된 articulation의 실제 루트(ArticulationRootAPI를 가진 prim)는
M0609의 base_link가 아니라 Nova Carter의 chassis_link이므로, 그 값을 그대로 쓰면 팔의
진짜 베이스 좌표(chassis보다 MOUNT_Z=0.42m 위)가 아니라 chassis 좌표를 베이스로 착각해서
IK 타겟이 전부 어긋난다. -> 로봇이 정지한 뒤 "chassis_pos + (0,0,MOUNT_Z)"로 실제 팔 베이스
포즈를 계산해서 RMPflow에 강제로 다시 넣어준다 (아래 "RMPflow 베이스 포즈 보정" 부분 참고).
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
from isaacsim.core.prims import SingleGeometryPrim
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import quat_to_euler_angles
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

_THIS_DIR = Path(__file__).resolve().parent
M0609_DIR = _THIS_DIR.parent / "M0609"
RMPFLOW_DIR = str(M0609_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)
from m0609_pick_place_controller import PickPlaceController  # noqa: E402

M0609_USD = str(M0609_DIR / "Collected_m0609_camera" / "m0609_camera.usd")
M0609_URDF_PATH = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH = str(M0609_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(M0609_DIR / "rmpflow/m0609_rmpflow_common.yaml")

# ---------------- 설정값 ----------------
ROBOT_START_XY = (0.0, 0.0)
FACE_ROT_Z = 0.0  # +X 방향을 보고 시작 -> 직진만으로 접근 가능하게
MOUNT_Z = 0.42  # Nova Carter 위 M0609 장착 높이 (7/8번 스크립트에서 검증된 값)
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e8, 1e4, 1e8

DRIVE_DISTANCE = 0.70  # 목표 전진 거리(m). 대략치이며, 실제 정지 위치는 실측해서 이후 배치에 반영
WHEEL_SPEED = 5.0  # rad/s
MAX_DRIVE_STEPS = 1200  # 안전장치: 이 스텝 안에 목표 거리에 못 미쳐도 강제 정지

# 팔 베이스(m0609 base_link) 기준 로컬 오프셋.
# 처음에 고정 베이스 데모(M0609/4.pick_place.py)의 CUBE_INIT_POS(z가 베이스 높이 근처, 거의 0)를
# 그대로 재사용했더니 실패했다 - 별도 sweep 스크립트(reach_sweep.py)로 RMPflow가 실제로 도달 가능한
# 영역을 격자 탐색해본 결과, 베이스 높이 근처(relative z<=0.15)는 항상 실패하고 relative z>=0.30인
# 지점은 오차 0에 가깝게 정확히 도달함 - 아마 base_link 자체의 자기충돌 콜리전 스피어 때문에 팔이
# 자기 베이스 높이 근처로는 내려오길 거부하는 것으로 보임. 아래 값들은 그 sweep 결과에서 검증된
# "확실히 도달 가능한" 좌표를 그대로 사용한다 (PickPlaceController가 모든 phase에 EE_OFFSET(+0.2m)을
# 더해서 넘기므로, 실제 목표 오프셋 이전 높이는 이 값에서 0.2를 뺀 값으로 역산해서 사용).
PICK_LOCAL_XY = (0.35, 0.0)
PLACE_LOCAL_XY = (0.35, 0.30)
BOX_REL_Z = 0.10  # 팔 베이스 기준 박스 높이 (+ EE_OFFSET 0.2 = 0.30, sweep에서 dist=0.000 확인됨)
RETRACT_REL_Z = 0.25  # 팔 베이스 기준 들어올림/리트랙트 높이 (+ EE_OFFSET 0.2 = 0.45, sweep에서 dist=0.000 확인됨)

BOX_SIZE = 0.05  # 5cm 정육면체 - 그리퍼가 쥘 수 있는 크기
BOX_MASS = 0.1  # kg, 가볍게
CUBE_STATIC, CUBE_DYNAMIC = 1.2, 1.0
FINGER_STATIC, FINGER_DYNAMIC = 1.8, 1.4

EE_LINK_NAME = "link_6"
GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]
GRIPPER_OPEN = [0.0, 0.0]
GRIPPER_CLOSE = [0.5, 0.5]
GRIPPER_DELTA = [-0.5, -0.5]
EE_OFFSET = np.array([0.0, 0.0, 0.2])

EVENTS_DT = [0.008, 0.005, 0.02, 0.1, 0.0025, 0.01, 0.0025, 1, 0.008, 0.08]


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
    """7/8번 스크립트와 동일한 방식으로 Nova Carter + M0609를 하나의 articulation으로 결합."""
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

    stray_prim = stage.GetPrimAtPath(f"{m0609_path}/onrobot_rg2ft/world")
    if stray_prim.IsValid() and stray_prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
        stray_prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)

    n = add_drive_stiffness(stage, m0609_path)
    print(f"[DRIVE] {n}개 조인트 강성 재설정", flush=True)

    return carter_path, chassis_link_path, m0609_path


# ================= 씬 구성 =================
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

carter_path, chassis_link_path, m0609_path = build_mobile_manipulator(stage)

gripper = ParallelGripper(
    end_effector_prim_path=f"{m0609_path}/{EE_LINK_NAME}",
    joint_prim_names=GRIPPER_JOINTS,
    joint_opened_positions=np.array(GRIPPER_OPEN),
    joint_closed_positions=np.array(GRIPPER_CLOSE),
    action_deltas=np.array(GRIPPER_DELTA),
)
robot = SingleManipulator(
    prim_path=chassis_link_path,
    end_effector_prim_path=f"{m0609_path}/{EE_LINK_NAME}",
    name="mobile_manipulator",
    gripper=gripper,
)

world.reset()
robot.initialize(physics_sim_view=world.physics_sim_view)
robot.set_joint_positions(np.zeros(robot.num_dof))
for _ in range(30):
    world.step(render=True)
print("\n[안정화 완료] 팔이 정상 기본 자세(0관절, 팔꿈치 접힌 형태)로 정지\n", flush=True)

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
base_quat = chassis_quat  # root_joint에 회전 오프셋을 준 적 없으므로 base_link 자세 = chassis 자세


def local_to_world_xy(local_xy):
    dx, dy = local_xy
    wx = base_pos[0] + dx * np.cos(yaw) - dy * np.sin(yaw)
    wy = base_pos[1] + dx * np.sin(yaw) + dy * np.cos(yaw)
    return wx, wy


pick_x, pick_y = local_to_world_xy(PICK_LOCAL_XY)
place_x, place_y = local_to_world_xy(PLACE_LOCAL_XY)
box_world_z = float(base_pos[2]) + BOX_REL_Z
stand_top_z = box_world_z - BOX_SIZE / 2.0
print(f"[배치] 팔 베이스 실측 위치=({base_pos[0]:.3f},{base_pos[1]:.3f},{base_pos[2]:.3f}) yaw={np.degrees(yaw):.1f}deg", flush=True)
print(f"[배치] pick=({pick_x:.3f},{pick_y:.3f}) place=({place_x:.3f},{place_y:.3f}) stand_top_z={stand_top_z:.3f}", flush=True)

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
    position=np.array([pick_x, pick_y, stand_top_z + BOX_SIZE / 2.0]),
    scale=np.array([BOX_SIZE, BOX_SIZE, BOX_SIZE]),
    color=np.array([0.1, 0.3, 0.9]),
    mass=BOX_MASS,
    physics_material=box_material,
)
goal_marker = VisualCuboid(
    prim_path="/World/GoalMarker",
    name="goal_marker",
    position=np.array([place_x, place_y, stand_top_z + 0.002]),
    scale=np.array([BOX_SIZE * 1.3, BOX_SIZE * 1.3, 0.002]),
    color=np.array([0.1, 0.9, 0.2]),
)

finger_material = PhysicsMaterial(
    prim_path="/World/Physics_Materials/finger_material",
    static_friction=FINGER_STATIC,
    dynamic_friction=FINGER_DYNAMIC,
    restitution=0.0,
)
for link_name in ["left_inner_finger", "right_inner_finger"]:
    link_path = f"{m0609_path}/onrobot_rg2ft/{link_name}"
    SingleGeometryPrim(prim_path=link_path, name=f"{link_name}_geom").apply_physics_material(finger_material)

for _ in range(40):
    world.step(render=True)

viewport = vp_util.get_active_viewport()


def snapshot(eye, target, fname):
    set_camera_view(eye=eye, target=target)
    for _ in range(10):
        world.step(render=True)
    out = str(_THIS_DIR / fname)
    vp_util.capture_viewport_to_file(viewport, out)
    for _ in range(5):
        world.step(render=True)
    print(f"[SCREENSHOT] {out}", flush=True)


snapshot(
    eye=[base_pos[0] - 0.8, base_pos[1] - 1.2, base_pos[2] + 1.0],
    target=[pick_x, pick_y, stand_top_z],
    fname="_verify_simple_pick_before.png",
)

# ================= 3. RMPflow pick&place 컨트롤러 구성 =================
GOAL_POS = np.array([place_x, place_y, box_world_z])
EE_INITIAL_HEIGHT = float(base_pos[2]) + RETRACT_REL_Z

controller = PickPlaceController(
    name="mobile_pick_place_controller",
    gripper=robot.gripper,
    robot_articulation=robot,
    end_effector_initial_height=EE_INITIAL_HEIGHT,
    events_dt=EVENTS_DT,
    urdf_path=M0609_URDF_PATH,
    robot_description_path=M0609_DESCRIPTION_PATH,
    rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
    end_effector_frame_name=EE_LINK_NAME,
)

# --- RMPflow 베이스 포즈 보정 ---
# robot_articulation(=chassis_link)의 get_world_pose()로 자동 캡처된 값은 chassis 좌표라서
# 실제 팔 베이스(base_pos, base_quat)로 강제로 다시 넣어준다. reset() 시에도 이 값이 그대로
# 재사용되도록 _default_position/_orientation 자체를 덮어쓴다.
cspace = controller._cspace_controller
cspace._default_position = base_pos
cspace._default_orientation = base_quat
cspace.rmp_flow.set_robot_base_pose(robot_position=base_pos, robot_orientation=base_quat)
print(f"[RMPflow 보정] base_pose -> pos={base_pos}, quat={base_quat}", flush=True)
print(f"[RMPflow 보정 확인] cspace._default_position={cspace._default_position}", flush=True)
ee_pos0, _ = robot.end_effector.get_world_pose()
print(f"[EE 초기 위치] {ee_pos0}  (pick target={pick_x:.3f},{pick_y:.3f},{stand_top_z + BOX_SIZE/2:.3f})", flush=True)

# ================= 4. Pick & Place 실행 =================
print("\n[Pick & Place 시작]\n", flush=True)
last_event = -1
lifted_shot_done = False
step_i = 0
while not controller.is_done():
    world.step(render=True)
    step_i += 1
    cube_pos, _ = box.get_world_pose()
    current_joints = robot.get_joint_positions()
    actions = controller.forward(
        picking_position=cube_pos,
        placing_position=GOAL_POS,
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
            fname="_verify_simple_pick_lifted.png",
        )
        lifted_shot_done = True

final_box_pos, _ = box.get_world_pose()
print(f"\n[완료] 최종 박스 위치=({final_box_pos[0]:.3f},{final_box_pos[1]:.3f},{final_box_pos[2]:.3f})", flush=True)
print(f"[완료] 목표 위치=({GOAL_POS[0]:.3f},{GOAL_POS[1]:.3f},{GOAL_POS[2]:.3f})", flush=True)
error = np.linalg.norm(final_box_pos - GOAL_POS)
print(f"[완료] 목표와의 거리 오차={error:.4f}m -> {'PASS' if error < 0.05 else 'FAIL'}", flush=True)

snapshot(
    eye=[base_pos[0] - 0.8, base_pos[1] - 1.2, base_pos[2] + 1.0],
    target=[place_x, place_y, stand_top_z],
    fname="_verify_simple_pick_after.png",
)

print("\n[안내] 검증 완료.\n", flush=True)
simulation_app.close()
