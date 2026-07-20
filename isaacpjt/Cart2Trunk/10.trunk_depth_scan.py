"""
모바일 매니퓰레이터(Nova Carter + M0609, 손목에 RealSense D455)와 차량(트렁크)만 있는
standalone 씬. 그리퍼에 달린 Depth 카메라로 트렁크 내부를 여러 waypoint에서 스캔해서
하나의 3D 포인트클라우드(트렁크 내부 공간 맵)로 합치는 것까지가 이번 스크립트의 목표.
박스 적재(Extreme Point) 알고리즘은 다음 단계.

씬 구성은 8번 스크립트에서 검증된 실측 비율(차량 0.50배 축소, 트렁크 좌표)을 그대로 재사용하고,
로봇 결합은 9번 스크립트의 build_mobile_manipulator()를 그대로 재사용한다.

Depth 카메라는 isaacsim.sensors.camera.Camera로 기존 프림(Camera_*Depth*)을 감싸서 쓰고,
Camera.get_pointcloud(world_frame=True)로 카메라 pose/intrinsic을 자동 반영한 월드좌표
포인트클라우드를 얻는다 (직접 역투영 계산 불필요).

waypoint의 목표 자세는 "카메라가 특정 지점을 바라보게" 하는 lookat 방식으로 정한다:
링크6(EE)과 카메라 프림의 상대 회전 오프셋(고정 마운트이므로 불변)을 로봇이 정지한 임의 자세에서
한 번 측정해 두고, lookat_to_quatf(eye, target, up)으로 계산한 "카메라가 가져야 할 world 자세"를
역산해서 링크6에 넘길 목표 자세로 변환한다.
"""

from isaacsim import SimulationApp

HEADLESS = False  # 2차: 실제로 트렁크 안쪽을 보는지 RGB/스크린샷으로 육안 확인

simulation_app = SimulationApp({"headless": HEADLESS})

from pathlib import Path
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf

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

# ---- 차량/트렁크 (8번 스크립트에서 실측 검증된 값 그대로 재사용) ----
CAR_POS = (5.0, 0.0, 0.0)
CAR_EXTRA_SCALE = 0.50
CAR_ROT_Z = 0.0
TRUNK_X_MIN, TRUNK_X_MAX = 3.11, 3.68
TRUNK_Y_MIN, TRUNK_Y_MAX = -0.56, 0.56
TRUNK_FLOOR_Z = 1.03
TRUNK_WALL_TOP = 1.28
SDF_RESOLUTION = 256

# ---- 로봇 배치 ----
# 트렁크 앞 범퍼 쪽에 바짝 붙여서, 팔이 대부분의 리치를 dz(높이차)에 쓸 수 있게 한다.
ROBOT_XY = (TRUNK_X_MIN - 0.55, -0.15)
FACE_ROT_Z = 0.0
MOUNT_Z = 0.42
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e8, 1e4, 1e8

EE_LINK_NAME = "link_6"
GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]
GRIPPER_OPEN = [0.0, 0.0]
GRIPPER_CLOSE = [0.5, 0.5]
GRIPPER_DELTA = [-0.5, -0.5]
DEPTH_CAMERA_NAME_HINT = "Depth"

WAYPOINT_STEPS = 200
SETTLE_STEPS = 30


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
    """7/8/9번 스크립트와 동일한 방식으로 Nova Carter + M0609를 하나의 articulation으로 결합."""
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

carter_path, chassis_link_path, m0609_path = build_mobile_manipulator(stage)

camera_prim_path, all_cameras = find_camera_prim_path(stage, m0609_path, DEPTH_CAMERA_NAME_HINT)
print(f"[CAMERA] 발견된 카메라 프림들: {all_cameras}", flush=True)
print(f"[CAMERA] 사용할 depth 카메라: {camera_prim_path}", flush=True)
if camera_prim_path is None:
    raise RuntimeError("M0609 asset 안에서 Camera 프림을 찾지 못함")

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
for _ in range(SETTLE_STEPS):
    world.step(render=True)
print("[안정화 완료] 팔 기본 자세", flush=True)

viewport0 = vp_util.get_active_viewport()
set_camera_view(eye=[ROBOT_XY[0] - 1.5, ROBOT_XY[1] - 2.0, 2.0], target=[CAR_POS[0], CAR_POS[1], 1.0])
for _ in range(20):
    world.step(render=True)
vp_util.capture_viewport_to_file(viewport0, str(_THIS_DIR / "_debug_overview.png"))
for _ in range(5):
    world.step(render=True)
print("[전체 개요 스크린샷] _debug_overview.png", flush=True)

# ================= 카메라 초기화 =================
camera = Camera(prim_path=camera_prim_path, resolution=(640, 480))
camera.initialize()
camera.add_distance_to_image_plane_to_frame()
camera.add_pointcloud_to_frame()
for _ in range(10):
    world.step(render=True)

# 디버그용: 트렁크 옆을 보는 RGB 카메라(Color 프림)도 같이 확인 (Depth 프림 자체는 흑백 시각화가 애매하므로)
color_prim_path = None
for c in all_cameras:
    if "color" in c.lower():
        color_prim_path = c
        break
rgb_camera = None
if color_prim_path:
    rgb_camera = Camera(prim_path=color_prim_path, resolution=(640, 480))
    rgb_camera.initialize()
    rgb_camera.add_rgb_to_frame()
    for _ in range(10):
        world.step(render=True)

# ================= 링크6 <-> 카메라 상대 회전 오프셋 측정 =================
chassis_pos, chassis_quat = robot.get_world_pose()
base_pos = np.array([chassis_pos[0], chassis_pos[1], chassis_pos[2] + MOUNT_Z])
base_quat = chassis_quat
print(f"[팔 베이스 실측] pos={base_pos} quat={base_quat}", flush=True)

link6_pos, link6_quat = robot.end_effector.get_world_pose()
cam_pos, cam_quat = camera.get_world_pose()
print(f"[link6] pos={link6_pos} quat={link6_quat}", flush=True)
print(f"[camera] pos={cam_pos} quat={cam_quat}", flush=True)

R_link6 = quats_to_rot_matrices(np.array([link6_quat]))[0]
R_cam = quats_to_rot_matrices(np.array([cam_quat]))[0]
R_offset = R_link6.T @ R_cam  # link6 로컬 프레임에서 본 카메라의 고정 회전 (마운트가 단단하므로 불변)
cam_local_pos_offset = R_link6.T @ (np.array(cam_pos) - np.array(link6_pos))
print(f"[오프셋] R_offset(link6->camera)=\n{R_offset}", flush=True)
print(f"[오프셋] camera position offset in link6 frame = {cam_local_pos_offset}", flush=True)


def camera_lookat_to_link6_target(eye, look_at, up=(0.0, 0.0, 1.0)):
    """카메라가 eye에서 look_at을 보게 하는 world 자세를 계산하고, 그걸 만들기 위한
    link6 목표 자세(및 근사 목표 위치)로 역산한다."""
    q_cam = lookat_to_quatf(Gf.Vec3f(*[float(v) for v in eye]), Gf.Vec3f(*[float(v) for v in look_at]), Gf.Vec3f(*up))
    q_cam_np = gf_quat_to_np_array(q_cam)
    R_cam_target = quats_to_rot_matrices(np.array([q_cam_np]))[0]
    R_link6_target = R_cam_target @ R_offset.T
    q_link6_target = rot_matrices_to_quats(np.array([R_link6_target]))[0]
    # 위치: 카메라를 정확히 eye에 두려면 link6_pos = eye - R_link6_target @ cam_local_pos_offset
    link6_target_pos = np.array(eye) - R_link6_target @ cam_local_pos_offset
    return link6_target_pos, q_link6_target


# ================= RMPflow 컨트롤러 구성 (저수준, PickPlace phase 없이 직접 forward) =================
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
print(f"[RMPflow 보정] base_pose -> pos={base_pos}, quat={base_quat}", flush=True)

# ================= 스캔 waypoint 정의 =================
trunk_center = np.array([(TRUNK_X_MIN + TRUNK_X_MAX) / 2, (TRUNK_Y_MIN + TRUNK_Y_MAX) / 2, TRUNK_FLOOR_Z])
# 2차 시도(eye를 트렁크 입구 바로 앞까지 붙임)는 팔꿈치/손목이 열린 트렁크 뚜껑에 거의 부딪히는
# 자세가 되면서 IK가 억지로 비틀려 카메라가 내부가 아니라 옆을 보는 문제가 있었다 (사용자가
# 3인칭 스크린샷에서 직접 확인).
# -> eye(카메라 위치)는 뚜껑과 여유를 두고 "고정된 한 점"으로 두고, look_at만 바꿔가며
#    팬/틸트 하듯 방향만 스윕한다. 위치가 거의 고정이므로 waypoint마다 IK가 크게 재수렴할
#    필요가 없어 뚜껑 충돌로 인한 자세 뒤틀림 위험이 줄어든다.
EYE = (2.95, 0.0, 1.20)
WAYPOINTS = [
    ("forward_center", EYE, (trunk_center[0], 0.0, TRUNK_FLOOR_Z)),
    ("steep_down", EYE, (TRUNK_X_MIN + 0.10, 0.0, TRUNK_FLOOR_Z)),
    ("left", EYE, (trunk_center[0], TRUNK_Y_MIN + 0.15, TRUNK_FLOOR_Z)),
    ("right", EYE, (trunk_center[0], TRUNK_Y_MAX - 0.15, TRUNK_FLOOR_Z)),
]

captured_clouds = []

for name, eye, look_at in WAYPOINTS:
    target_pos, target_quat = camera_lookat_to_link6_target(eye, look_at)
    rel = target_pos - base_pos
    print(
        f"\n[waypoint:{name}] eye={eye} look_at={look_at} link6_target_pos={target_pos} "
        f"(base 기준 상대={rel}, |rel|={np.linalg.norm(rel):.3f}m)",
        flush=True,
    )
    for step_i in range(WAYPOINT_STEPS):
        current_joints = robot.get_joint_positions()
        actions = controller.forward(
            target_end_effector_position=target_pos,
            target_end_effector_orientation=target_quat,
        )
        robot.apply_action(actions)
        world.step(render=True)
        if step_i % 50 == 0 or step_i == WAYPOINT_STEPS - 1:
            ee_pos, _ = robot.end_effector.get_world_pose()
            err = np.linalg.norm(np.array(ee_pos) - target_pos)
            print(f"  [step {step_i}] ee_pos={ee_pos} target_dist_err={err:.4f}m", flush=True)

    ee_pos, ee_quat = robot.end_effector.get_world_pose()
    err = np.linalg.norm(np.array(ee_pos) - target_pos)
    cam_pos_now, cam_quat_now = camera.get_world_pose()
    print(f"[waypoint:{name}] 최종 ee_pos={ee_pos} err={err:.4f}m camera_pos={cam_pos_now}", flush=True)

    # 진단: 카메라가 실제로 look_at 방향을 보고 있는지 확인 (카메라 로컬 -Z가 forward라는 가정 검증)
    R_cam_now = quats_to_rot_matrices(np.array([cam_quat_now]))[0]
    forward_now = R_cam_now @ np.array([0.0, 0.0, -1.0])
    to_target = np.array(look_at) - np.array(cam_pos_now)
    to_target_dir = to_target / (np.linalg.norm(to_target) + 1e-9)
    alignment = float(np.dot(forward_now, to_target_dir))
    print(
        f"[waypoint:{name}] 진단: camera_forward={forward_now}, to_look_at_dir={to_target_dir}, "
        f"alignment(cos)={alignment:.3f} (1.0=완벽히 look_at을 향함)",
        flush=True,
    )

    for _ in range(10):
        world.step(render=True)
    pcd = camera.get_pointcloud(world_frame=True)
    n_pts = 0 if pcd is None else len(pcd)
    print(f"[waypoint:{name}] pointcloud 점 개수={n_pts}", flush=True)
    if n_pts > 0:
        captured_clouds.append(np.asarray(pcd))

    if rgb_camera is not None:
        rgb = rgb_camera.get_rgba()[:, :, :3]
        plt.imsave(str(_THIS_DIR / f"_debug_rgb_{name}.png"), rgb)
        print(f"[waypoint:{name}] RGB 저장 -> _debug_rgb_{name}.png", flush=True)

    viewport = vp_util.get_active_viewport()
    set_camera_view(eye=[base_pos[0] - 1.2, base_pos[1] - 1.5, base_pos[2] + 1.3], target=[eye[0], eye[1], eye[2]])
    for _ in range(10):
        world.step(render=True)
    vp_util.capture_viewport_to_file(viewport, str(_THIS_DIR / f"_debug_wide_{name}.png"))
    for _ in range(5):
        world.step(render=True)
    print(f"[waypoint:{name}] 3인칭 스크린샷 저장 -> _debug_wide_{name}.png", flush=True)

print("\n[스캔 완료]", flush=True)
if captured_clouds:
    merged = np.concatenate(captured_clouds, axis=0)
    print(f"[병합] 전체 포인트 개수={len(merged)}", flush=True)
    out_path = _THIS_DIR / "_trunk_pointcloud.npy"
    np.save(out_path, merged)
    print(f"[저장] {out_path}", flush=True)
else:
    print("[경고] 캡처된 포인트가 없음", flush=True)

simulation_app.close()
