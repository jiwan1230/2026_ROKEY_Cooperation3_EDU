"""
10번 스크립트의 자동 lookat 계산은 (1) 팔이 트렁크 뚜껑에 너무 가까워 IK가 억지로 비틀리는 문제,
(2) camera_forward 방향 계산 자체에 부호 버그(alignment=-1.0, 즉 의도한 반대 방향을 봄)가 있어서
결과가 계속 트렁크 내부가 아닌 다른 곳을 보고 있었다.

자동 각도 계산으로 계속 씨름하는 대신, 사용자가 Isaac Sim GUI에서 직접 조인트를 조작해서
"카메라가 트렁크 내부를 보는" 좋은 자세를 찾도록 한다. 이 스크립트는:
  1. 씬(차량/트렁크 + Nova Carter+M0609+카메라)을 구성하고
  2. 팔을 낮고 안전한 기본 자세(뚜껑과 충분히 떨어진 위치, 특별한 회전 없이 거의 정면)로 이동시킨 뒤
  3. RMPflow 명령을 멈추고 계속 실행 상태로 대기한다.

사용자가 Property 패널에서 각 조인트의 Drive Target Position 슬라이더를 움직여 자세를 잡으면
(드라이브 강성이 매우 높게 설정되어 있어 슬라이더를 바꾸면 그 즉시 그 각도로 움직임),
이 스크립트는 0.5초마다 현재 EE/카메라 world pose + 전체 조인트 각도를 콘솔에 출력하고,
3초마다 3인칭 스크린샷 + 카메라 RGB 프레임을 같은 파일명으로 덮어써서 저장한다.
좋은 자세를 찾으면 터미널에 그 시점의 pose/조인트값이 남아있으니 그걸 다음 스크립트에 반영하면 된다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import sys
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, UsdLux, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.utils.rotations import gf_quat_to_np_array, lookat_to_quatf
from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.sensors.camera import Camera

_THIS_DIR = Path(__file__).resolve().parent
M0609_DIR = _THIS_DIR.parent / "M0609"
RMPFLOW_DIR = str(M0609_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController  # noqa: E402

CAR_USD = str(_THIS_DIR / "assets/Lexus_IS300_Trunk_Open_No_More_Hell_Room.usdz")
M0609_USD = str(M0609_DIR / "Collected_m0609_camera" / "m0609_camera.usd")
M0609_URDF_PATH = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH = str(M0609_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(M0609_DIR / "rmpflow/m0609_rmpflow_common.yaml")

CAR_POS = (5.0, 0.0, 0.0)
CAR_EXTRA_SCALE = 0.50
CAR_ROT_Z = 0.0
TRUNK_X_MIN, TRUNK_X_MAX = 3.11, 3.68
TRUNK_Y_MIN, TRUNK_Y_MAX = -0.56, 0.56
TRUNK_FLOOR_Z = 1.03
TRUNK_WALL_TOP = 1.28
SDF_RESOLUTION = 256

ROBOT_XY = (TRUNK_X_MIN - 0.85, -0.15)
FACE_ROT_Z = 0.0
MOUNT_Z = 0.42
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e8, 1e4, 1e8

EE_LINK_NAME = "link_6"
GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]
GRIPPER_OPEN = [0.0, 0.0]
GRIPPER_CLOSE = [0.5, 0.5]
GRIPPER_DELTA = [-0.5, -0.5]
DEPTH_CAMERA_NAME_HINT = "Depth"

# "기본 자세" 목표: 낮고(트렁크 바닥과 비슷한 높이), 뚜껑에서 충분히 떨어진 위치.
# 방향은 로봇 베이스와 동일(=거의 정면, 특이한 회전 없음) - 사용자가 여기서부터 손목만 조절하면 됨.
BASIC_TARGET_LOCAL = (0.40, 0.15, 0.55)  # base_link 기준 상대 (dx, dy, dz)
BASIC_STEPS = 200


def add_asset_scaled(stage, prim_path, usd_path, position, extra_scale, target_mpu, target_up, rot_z=0.0):
    src_stage = Usd.Stage.Open(usd_path)
    src_mpu = UsdGeom.GetStageMetersPerUnit(src_stage)
    src_up = UsdGeom.GetStageUpAxis(src_stage)
    scale = (src_mpu / target_mpu if target_mpu else src_mpu) * extra_scale
    xform = UsdGeom.Xform.Define(stage, prim_path)
    prim = xform.GetPrim()
    prim.GetReferences().AddReference(usd_path)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(position)
    if rot_z:
        xform.AddRotateZOp().Set(rot_z)
    if src_up == UsdGeom.Tokens.y and target_up == UsdGeom.Tokens.z:
        xform.AddRotateXOp().Set(90.0)
    xform.AddScaleOp().Set((scale, scale, scale))
    return xform


def add_sdf_collision(stage, root_prim_path, sdf_resolution=SDF_RESOLUTION):
    root_prim = stage.GetPrimAtPath(root_prim_path)
    n = 0
    for prim in Usd.PrimRange(root_prim):
        if prim.GetTypeName() == "Mesh":
            UsdPhysics.CollisionAPI.Apply(prim)
            mc = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mc.CreateApproximationAttr().Set("sdf")
            sdf_api = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
            sdf_api.CreateSdfResolutionAttr().Set(sdf_resolution)
            n += 1
    print(f"[SDF] {root_prim_path}: {n} mesh", flush=True)


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
    carter_xform.AddTranslateOp().Set(Gf.Vec3d(ROBOT_XY[0], ROBOT_XY[1], 0.0))
    carter_xform.AddRotateZOp().Set(FACE_ROT_Z)

    m0609_path = "/World/MobileManipulator/M0609"
    m0609_xform = UsdGeom.Xform.Define(stage, m0609_path)
    m0609_xform.GetPrim().GetReferences().AddReference(M0609_USD, "/World/m0609")
    m0609_xform.ClearXformOpOrder()
    m0609_xform.AddTranslateOp().Set(Gf.Vec3d(ROBOT_XY[0], ROBOT_XY[1], MOUNT_Z))
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


def find_camera_prim_path(stage, root_path, name_hint):
    root_prim = stage.GetPrimAtPath(root_path)
    candidates = []
    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdGeom.Camera):
            candidates.append(str(prim.GetPath()))
    for c in candidates:
        if name_hint.lower() in c.lower():
            return c, candidates
    return (candidates[0] if candidates else None), candidates


# ================= 씬 구성 =================
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
target_mpu = UsdGeom.GetStageMetersPerUnit(stage)
target_up = UsdGeom.GetStageUpAxis(stage)

add_asset_scaled(stage, "/World/Vehicle", CAR_USD, CAR_POS, CAR_EXTRA_SCALE, target_mpu, target_up, rot_z=CAR_ROT_Z)
for _ in range(20):
    simulation_app.update()
add_sdf_collision(stage, "/World/Vehicle")

# 트렁크 내부는 빛이 거의 안 들어가 RGB 디버그가 새까맣게 나온다. 뚜껑 위쪽에 놓은 조명은 열린
# 뚜껑 패널에 가려져 효과가 없었으므로, 아예 캐비티 내부(바닥 바로 위)에 조명을 둔다.
trunk_light = UsdLux.SphereLight.Define(stage, "/World/TrunkAreaLight")
trunk_light.CreateRadiusAttr(0.08)
trunk_light.CreateIntensityAttr(400000)
trunk_center_xy = ((TRUNK_X_MIN + TRUNK_X_MAX) / 2, (TRUNK_Y_MIN + TRUNK_Y_MAX) / 2)
UsdGeom.Xformable(trunk_light).AddTranslateOp().Set(Gf.Vec3d(trunk_center_xy[0], trunk_center_xy[1], TRUNK_FLOOR_Z + 0.2))

carter_path, chassis_link_path, m0609_path = build_mobile_manipulator(stage)

camera_prim_path, all_cameras = find_camera_prim_path(stage, m0609_path, DEPTH_CAMERA_NAME_HINT)
print(f"[CAMERA] 사용할 depth 카메라: {camera_prim_path}", flush=True)
color_prim_path = next((c for c in all_cameras if "color" in c.lower()), None)

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
print("[안정화 완료] 팔 기본 자세", flush=True)

camera = Camera(prim_path=camera_prim_path, resolution=(640, 480))
camera.initialize()
camera.add_distance_to_image_plane_to_frame()
camera.add_pointcloud_to_frame()
rgb_camera = None
if color_prim_path:
    rgb_camera = Camera(prim_path=color_prim_path, resolution=(640, 480))
    rgb_camera.initialize()
    rgb_camera.add_rgb_to_frame()
for _ in range(10):
    world.step(render=True)

chassis_pos, chassis_quat = robot.get_world_pose()
base_pos = np.array([chassis_pos[0], chassis_pos[1], chassis_pos[2] + MOUNT_Z])
base_quat = chassis_quat
print(f"[팔 베이스 실측] pos={base_pos} quat={base_quat}", flush=True)

# ================= 링크6 <-> 카메라 상대 회전 오프셋 측정 (마운트 고정이므로 자세와 무관하게 불변) =================
link6_pos0, link6_quat0 = robot.end_effector.get_world_pose()
cam_pos0, cam_quat0 = camera.get_world_pose()
R_link6_0 = quats_to_rot_matrices(np.array([link6_quat0]))[0]
R_cam_0 = quats_to_rot_matrices(np.array([cam_quat0]))[0]
R_offset = R_link6_0.T @ R_cam_0
cam_local_pos_offset = R_link6_0.T @ (np.array(cam_pos0) - np.array(link6_pos0))
print(f"[오프셋] R_offset(link6->camera)=\n{R_offset}\ncamera pos offset in link6 frame={cam_local_pos_offset}", flush=True)


def lookat_to_link6_target(eye, look_at, up=(0.0, 0.0, 1.0)):
    """카메라가 eye에서 look_at을 보게 하는 link6 목표 자세를 계산.
    (10번 스크립트에서 lookat_to_quatf(eye, look_at, ...)를 그대로 쓰면 카메라가 정확히 반대
    방향을 보는 부호 문제가 있었음 - alignment 진단으로 확인됨. eye 기준으로 look_at을 반대편으로
    미러링해서 넘기면 보정됨: lookat_to_quatf가 만드는 '+Z가 향하는 방향'이 아니라 카메라의 실제
    forward(-Z)가 look_at을 향하게 하기 위함.)"""
    eye_np = np.array(eye, dtype=float)
    look_at_np = np.array(look_at, dtype=float)
    mirrored_target = eye_np - (look_at_np - eye_np)
    q_cam = lookat_to_quatf(Gf.Vec3f(*eye_np.tolist()), Gf.Vec3f(*mirrored_target.tolist()), Gf.Vec3f(*up))
    q_cam_np = gf_quat_to_np_array(q_cam)
    R_cam_target = quats_to_rot_matrices(np.array([q_cam_np]))[0]
    R_link6_target = R_cam_target @ R_offset.T
    q_link6_target = rot_matrices_to_quats(np.array([R_link6_target]))[0]
    link6_target_pos = eye_np - R_link6_target @ cam_local_pos_offset
    return link6_target_pos, q_link6_target


def camera_alignment_check(look_at):
    """실제로 카메라가 look_at을 보고 있는지 진단 (1.0에 가까울수록 정확히 봄)."""
    cam_pos_now, cam_quat_now = camera.get_world_pose()
    R_cam_now = quats_to_rot_matrices(np.array([cam_quat_now]))[0]
    forward_now = R_cam_now @ np.array([0.0, 0.0, -1.0])
    to_target = np.array(look_at) - np.array(cam_pos_now)
    to_target_dir = to_target / (np.linalg.norm(to_target) + 1e-9)
    return float(np.dot(forward_now, to_target_dir)), cam_pos_now, cam_quat_now

# ================= RMPflow로 "기본 자세"까지만 이동 (그 이후엔 손을 뗀다) =================
controller = RMPFlowController(
    name="trunk_scan_controller",
    robot_articulation=robot,
    urdf_path=M0609_URDF_PATH,
    robot_description_path=M0609_DESCRIPTION_PATH,
    rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
    end_effector_frame_name=EE_LINK_NAME,
)
controller._default_position = base_pos
controller._default_orientation = base_quat
controller.rmp_flow.set_robot_base_pose(robot_position=base_pos, robot_orientation=base_quat)

# 앵커(고정할 한 점) 위치는 기존과 동일하게 base 기준 상대좌표로 정하고,
# 방향은 "그리퍼(카메라)가 차량/트렁크 내부를 바라보게" lookat으로 계산한다 (base_quat 그대로 쓰면
# 위를 보는 문제가 있었음 - 사용자가 스크린샷으로 확인).
anchor_pos = base_pos + np.array(BASIC_TARGET_LOCAL)
trunk_center = np.array([(TRUNK_X_MIN + TRUNK_X_MAX) / 2, (TRUNK_Y_MIN + TRUNK_Y_MAX) / 2, TRUNK_FLOOR_Z])
target_pos, target_quat = lookat_to_link6_target(anchor_pos, trunk_center)
print(f"[기본 자세 목표] link6_pos={target_pos} (anchor={anchor_pos}, look_at=trunk_center={trunk_center})", flush=True)

for step_i in range(BASIC_STEPS):
    actions = controller.forward(target_end_effector_position=target_pos, target_end_effector_orientation=target_quat)
    robot.apply_action(actions)
    world.step(render=True)

ee_pos, ee_quat = robot.end_effector.get_world_pose()
err = np.linalg.norm(np.array(ee_pos) - target_pos)
alignment, cam_pos_now, cam_quat_now = camera_alignment_check(trunk_center)
R_cam_now = quats_to_rot_matrices(np.array([cam_quat_now]))[0]
cam_up_now = R_cam_now @ np.array([0.0, 1.0, 0.0])
print(
    f"[기본 자세 도달] ee_pos={ee_pos} err={err:.4f}m alignment(cos)={alignment:.3f} "
    f"camera_up={np.round(cam_up_now,3)} (world_up=[0,0,1]과 비교, roll 진단용)",
    flush=True,
)

# 사용자 참고 사진과 비슷한 구도(측면 뒤쪽에서 로봇+차량 전체를 넓게)로 스크린샷 촬영 - 기존
# 스크립트의 근접/위쪽 시점 대신, 자세 자체를 오해 없이 비교하기 위함.
set_camera_view(eye=[base_pos[0] - 2.2, base_pos[1] - 3.2, base_pos[2] + 1.6], target=[(base_pos[0] + CAR_POS[0]) / 2, 0.0, 1.0])
for _ in range(20):
    world.step(render=True)
vp_util.capture_viewport_to_file(vp_util.get_active_viewport(), str(_THIS_DIR / "_pose_check_reference_angle.png"))
for _ in range(5):
    world.step(render=True)
print("[참고 구도 스크린샷] _pose_check_reference_angle.png", flush=True)

DO_SWEEP = False
if not DO_SWEEP:
    print("\n[진단 모드] 자세 하나만 확인하고 스윕은 건너뜁니다. 결과 보고 다음 단계 결정.\n", flush=True)

# ================= 앵커 위치는 고정, look_at만 바꿔가며 트렁크 내부 여러 각도 스캔 =================
# target_end_effector_position은 매 waypoint 동일(anchor_pos 기준으로 계산)하게 유지하고,
# target_end_effector_orientation만 바뀌므로 팔은 그리퍼 끝단을 축으로 방향만 바꾼다.
SWEEP_WAYPOINTS = [
    ("trunk_center", trunk_center),
    ("trunk_near_floor", np.array([TRUNK_X_MIN + 0.15, 0.0, TRUNK_FLOOR_Z])),
    ("trunk_far_floor", np.array([TRUNK_X_MAX - 0.1, 0.0, TRUNK_FLOOR_Z])),
    ("trunk_left", np.array([trunk_center[0], TRUNK_Y_MIN + 0.15, TRUNK_FLOOR_Z])),
    ("trunk_right", np.array([trunk_center[0], TRUNK_Y_MAX - 0.15, TRUNK_FLOOR_Z])),
]

for name, look_at in (SWEEP_WAYPOINTS if DO_SWEEP else []):
    t_pos, t_quat = lookat_to_link6_target(anchor_pos, look_at)
    for _ in range(120):
        actions = controller.forward(target_end_effector_position=t_pos, target_end_effector_orientation=t_quat)
        robot.apply_action(actions)
        world.step(render=True)
    ee_pos_s, _ = robot.end_effector.get_world_pose()
    err_s = np.linalg.norm(np.array(ee_pos_s) - t_pos)
    alignment, cam_pos_s, cam_quat_s = camera_alignment_check(look_at)
    print(
        f"[스윕:{name}] look_at={look_at} ee_pos={np.round(ee_pos_s,3)} err={err_s:.4f}m "
        f"cam_pos={np.round(cam_pos_s,3)} alignment(cos)={alignment:.3f}",
        flush=True,
    )
    if rgb_camera is not None:
        rgb = rgb_camera.get_rgba()[:, :, :3]
        plt.imsave(str(_THIS_DIR / f"_sweep_{name}.png"), rgb)
    set_camera_view(
        eye=[base_pos[0] - 1.2, base_pos[1] - 1.5, base_pos[2] + 1.3],
        target=[float(cam_pos_s[0]), float(cam_pos_s[1]), float(cam_pos_s[2])],
    )
    for _ in range(10):
        world.step(render=True)
    vp_util.capture_viewport_to_file(vp_util.get_active_viewport(), str(_THIS_DIR / f"_sweep_wide_{name}.png"))
    for _ in range(5):
        world.step(render=True)

print("\n[스윕 완료] 이제부터 RMPflow 명령을 멈춥니다. Property 패널에서 각 조인트(Joint) 프림을 선택해\n"
      "  'Drive' 항목의 Target Position 슬라이더를 움직이면 그 즉시 그 각도로 이동합니다\n"
      "  (드라이브 강성이 매우 높게 설정되어 있음).\n", flush=True)

# ================= 대기 루프: 주기적으로 pose 출력 + 스크린샷 갱신 =================
viewport = vp_util.get_active_viewport()
last_print = 0.0
last_shot = 0.0
step_i = 0
while simulation_app.is_running():
    world.step(render=True)
    step_i += 1
    now = time.time()

    if now - last_print > 0.5:
        last_print = now
        joint_pos = robot.get_joint_positions()
        ee_pos, ee_quat = robot.end_effector.get_world_pose()
        cam_pos, cam_quat = camera.get_world_pose()
        print(
            f"[상태] ee_pos={np.round(ee_pos,3)} cam_pos={np.round(cam_pos,3)} cam_quat={np.round(cam_quat,3)} "
            f"joints={np.round(joint_pos,3)}",
            flush=True,
        )

    if now - last_shot > 3.0:
        last_shot = now
        set_camera_view(
            eye=[base_pos[0] - 1.2, base_pos[1] - 1.5, base_pos[2] + 1.3],
            target=[float(ee_pos[0]), float(ee_pos[1]), float(ee_pos[2])],
        )
        world.step(render=True)
        vp_util.capture_viewport_to_file(viewport, str(_THIS_DIR / "_live_wide.png"))
        if rgb_camera is not None:
            rgb = rgb_camera.get_rgba()[:, :, :3]
            plt.imsave(str(_THIS_DIR / "_live_rgb.png"), rgb)

simulation_app.close()
