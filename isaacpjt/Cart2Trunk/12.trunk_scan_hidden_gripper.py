"""
트렁크 Depth Camera 저위치·깊은 곳 주시 고정점 스캔 버전 (그리퍼 시야 가림 제거판).

11.trunk_scan_deep_anchor_sweep.py 기반. 두 가지가 추가됨:
1. 그리퍼 손가락/너클 메쉬를 렌더링에서만 숨김(MakeInvisible) - 물리/조인트/IK는 그대로 두고
   카메라가 자기 손가락을 찍어서 시야를 가리는 문제만 제거.
2. 결과물(포인트클라우드/스윕 스크린샷/라이브 스냅샷)을 실행 시각별 run 폴더
   (results/run_YYYYMMDD_HHMMSS/)에 정리해서 저장 - 루트에 파일이 흩어지지 않게 함.

기본 자세:
1. link_6(그리퍼 끝점)을 트렁크 입구의 낮은 위치에 고정한다.
2. 카메라는 트렁크 내부 가장 깊은 쪽의 중앙을 바라본다.
3. 스캔 중에는 그리퍼 끝점 위치를 유지하고 orientation만 바꾼다.
4. 깊은 곳 중앙을 기준으로 좌측, 우측, 바닥, 천장 방향을 차례로 촬영한다.

좌표/카메라 교정:
- Camera.get_world_pose(camera_axes="usd")를 사용한다.
- USD Camera의 +Y를 영상 위쪽, -Z를 시선 방향으로 일관되게 해석한다.
- Depth Camera 자체의 RGB 영상을 저장하여 실제 제어 카메라 시야를 확인한다.

처음에는 DO_SWEEP=False로 기본 자세를 확인하고, alignment/upright와 충돌 여부가 정상이면 True로 바꾼다.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
from datetime import datetime
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

# ================= 결과물 저장 폴더 (실행 시각별로 정리) =================
RUN_DIR = _THIS_DIR / "results" / f"run_{datetime.now():%Y%m%d_%H%M%S}"
POINTCLOUD_DIR = RUN_DIR / "pointcloud"
SWEEP_DIR = RUN_DIR / "sweep"
LIVE_DIR = RUN_DIR / "live"
POSE_CHECK_DIR = RUN_DIR / "pose_check"
for _d in (POINTCLOUD_DIR, SWEEP_DIR, LIVE_DIR, POSE_CHECK_DIR):
    _d.mkdir(parents=True, exist_ok=True)
print(f"[결과 폴더] {RUN_DIR}", flush=True)

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
# 1.03은 사실 입구 쪽의 얕은 턱(선반)이었다 - 위에서 떨어뜨린 박스가 거기 걸려서 진짜 바닥인 줄
# 착각했었음. 턱보다 낮은 위치(z=0.95)에서 박스를 스폰해 다시 낙하시켜보니 z=0.43~0.44에서
# 진짜 바닥을 찾음 (물리 낙하 테스트 + 스크린샷으로 확인, 사용자가 직접 지적해서 재검증함).
TRUNK_FLOOR_Z = 0.44
TRUNK_WALL_TOP = 1.28  # 입구 턱 높이(1.03) 위쪽 - 재검증 필요할 수 있음, 일단 유지
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

# 고정점 기준. "link6"는 손목 플랜지 자체를 고정 (실제 그리퍼 끝단과 약 14cm 오프셋 있음).
# "camera"는 카메라 optical center를 고정. "gripper_tip"은 그리퍼 손가락 실제 끝단을 고정한다 -
# ANCHOR_HEIGHT_ABOVE_FLOOR=0이면 그리퍼 손가락이 정확히 바닥 높이에 오도록 손가락 오프셋을
# 역산해서 보정한다 (link6 기준일 때 "0.01을 줘도 안 내려가 보이던" 문제의 근본 수정).
ANCHOR_MODE = "gripper_tip"
CAMERA_AXES = "usd"  # USD Camera: +Y up, -Z forward
WORLD_UP = (0.0, 0.0, 1.0)

# ---------------- 트렁크 스캔 기준점 (그리퍼 손가락 실제 끝단 기준) ----------------
# 그리퍼 끝단을 트렁크 입구보다 약간 바깥쪽, 좌우 중앙, 바닥보다 ANCHOR_HEIGHT_ABOVE_FLOOR만큼
# 위에 둔다. 0이면 그리퍼 끝단이 바닥 높이와 정확히 같다. 바닥을 뚫고 들어가면 음수는 피할 것.
# 사용자 요청: 바닥(0.44)까지 안 가도 되고, 바닥~턱(1.03) 사이 z=0.67 정도면 충분.
# ANCHOR_HEIGHT_ABOVE_FLOOR = 0.67 - TRUNK_FLOOR_Z(0.44) = 0.23
ANCHOR_OUTSIDE_OFFSET = 0.20
ANCHOR_HEIGHT_ABOVE_FLOOR = 0.33
ANCHOR_Y = 0.0

# 기본 시선은 트렁크 가장 깊은 벽에서 8cm 앞, 바닥보다 8cm 위를 향한다.
# 더 아래를 보게 하려면 DEEP_CENTER_HEIGHT를 낮춘다.
DEEP_WALL_MARGIN = 0.08
DEEP_CENTER_HEIGHT = 0.4

# 좌우/상하 스캔 시 경계면과 너무 가깝지 않도록 둔 여유 거리.
SIDE_MARGIN = 0.10
FLOOR_MARGIN = 0.02
CEILING_MARGIN = 0.05

BASIC_STEPS = 240
SWEEP_STEPS = 150
DO_SWEEP = True  # 기본 자세와 충돌 여부 확인 완료 -> 스윕 진행


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

    # RSD455 카메라 asset에 딸려오는 IMU 센서(우리는 안 씀)가 매 스텝 rigid body velocity를
    # 조회하다가 병합된 19-DOF articulation에서 "expected 6, received 12 shape(2,6)" 텐서
    # 에러를 유발한다 (격리 테스트로 확인함). 안 쓰는 센서이므로 비활성화.
    imu_prim = stage.GetPrimAtPath(
        f"{m0609_path}/onrobot_rg2ft/angle_bracket/realsense_d455/RSD455/Imu_Sensor"
    )
    if imu_prim.IsValid():
        imu_prim.SetActive(False)
        print("[IMU] RSD455 Imu_Sensor 비활성화 (velocity tensor 에러 원인)", flush=True)

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


def hide_gripper_visuals(stage, m0609_path):
    """그리퍼(onrobot_rg2ft) 전체를 렌더링에서만 숨긴다 (물리/조인트/IK는 그대로 유지).
    손가락만 숨기면 손등/팔레트(gripper_body)·퀵체인저·브라켓 외피가 여전히 남아 카메라 시야를
    가린다. link_6이 로봇의 실질적인 끝단인 것처럼 보이도록 onrobot_rg2ft 서브트리 전체를
    MakeInvisible() 한 번으로 끈다 - USD visibility는 자식에게 상속되므로 그 안의 손가락/너클/
    브라켓/카메라 하우징 메쉬까지 전부 한 번에 가려진다.

    카메라(Camera_Pseudo_Depth 등)는 Mesh가 아니라 UsdGeom.Camera 프림이라, 조상의 visibility를
    꺼도 촬영 기능 자체는 영향 없다 - 오직 "보이는 지오메트리"만 사라진다.
    """
    gripper_root = stage.GetPrimAtPath(f"{m0609_path}/onrobot_rg2ft")
    UsdGeom.Imageable(gripper_root).MakeInvisible()
    print(f"[VISIBILITY] {gripper_root.GetPath()} 전체 숨김 (link_6이 사실상 끝단, 물리는 그대로)", flush=True)


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
hide_gripper_visuals(stage, m0609_path)

camera_prim_path, all_cameras = find_camera_prim_path(stage, m0609_path, DEPTH_CAMERA_NAME_HINT)
print(f"[CAMERA 후보] {all_cameras}", flush=True)
if camera_prim_path is None:
    raise RuntimeError("M0609 하위에서 UsdGeom.Camera prim을 찾지 못했습니다.")
print(f"[CAMERA] 제어/스캔에 사용할 depth 카메라: {camera_prim_path}", flush=True)
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
camera.add_rgb_to_frame()  # 제어 중인 바로 그 Depth Camera 시야를 RGB로 디버그
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
cam_pos0, cam_quat0 = camera.get_world_pose(camera_axes=CAMERA_AXES)
R_link6_0 = quats_to_rot_matrices(np.array([link6_quat0]))[0]
R_cam_0 = quats_to_rot_matrices(np.array([cam_quat0]))[0]
R_offset = R_link6_0.T @ R_cam_0
cam_local_pos_offset = R_link6_0.T @ (np.array(cam_pos0) - np.array(link6_pos0))
print(f"[오프셋] R_offset(link6->camera)=\n{R_offset}\ncamera pos offset in link6 frame={cam_local_pos_offset}", flush=True)

# ================= 링크6 <-> 그리퍼 손가락(실제 끝단) 오프셋 측정 =================
# link6(손목 플랜지)는 실제로 눈에 보이는 그리퍼 끝단이 아니다 - 그 사이에 고정된 물리적
# 오프셋이 있어서, ANCHOR_HEIGHT_ABOVE_FLOOR를 0에 가깝게 줘도 link6는 바닥 근처로 가지만
# 실제 그리퍼는 여전히 떠 보이는 문제가 있었다. 카메라 오프셋과 동일한 방식으로
# left/right_inner_finger 링크(그리퍼 손가락 실물)의 world pose를 측정해서 link6 로컬 프레임
# 기준 오프셋을 구하고, 이후 anchor_mode="gripper_tip"에서 이 오프셋을 보정에 사용한다.
xform_cache_tip = UsdGeom.XformCache()
_finger_paths = [f"{m0609_path}/onrobot_rg2ft/left_inner_finger", f"{m0609_path}/onrobot_rg2ft/right_inner_finger"]
_finger_world_positions = []
for _p in _finger_paths:
    _prim = stage.GetPrimAtPath(_p)
    _mat = xform_cache_tip.GetLocalToWorldTransform(_prim)
    _pos = _mat.ExtractTranslation()
    _finger_world_positions.append(np.array([_pos[0], _pos[1], _pos[2]]))
gripper_tip_world0 = np.mean(_finger_world_positions, axis=0)
gripper_tip_local_offset = R_link6_0.T @ (gripper_tip_world0 - np.array(link6_pos0))
print(f"[오프셋] gripper tip(손가락 중심) pos offset in link6 frame={gripper_tip_local_offset}", flush=True)


def _normalize(v, eps=1e-9):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < eps:
        raise ValueError(f"영벡터는 방향으로 사용할 수 없습니다: {v}")
    return v / n


def make_usd_camera_rotation(eye, look_at, up_ref=WORLD_UP):
    """USD Camera 축(+Y up, -Z forward)에 맞는 world rotation matrix를 직접 구성한다.

    기존 코드의 핵심 문제는 Camera.get_world_pose() 기본값인 'world' 축(+X forward, +Z up)을
    받아 놓고, 이를 USD 카메라 축(-Z forward, +Y up)처럼 해석한 것이다. 이 함수는 축 규약을
    명시적으로 고정해 카메라가 옆을 보거나 90도 누워 버리는 문제를 제거한다.
    """
    eye = np.asarray(eye, dtype=float)
    look_at = np.asarray(look_at, dtype=float)
    forward = _normalize(look_at - eye)
    up_ref = _normalize(up_ref)

    # 시선과 up이 거의 평행하면 외적이 불안정하므로 대체 up 축 사용.
    if abs(float(np.dot(forward, up_ref))) > 0.97:
        alt = np.array([0.0, 1.0, 0.0])
        if abs(float(np.dot(forward, alt))) > 0.97:
            alt = np.array([1.0, 0.0, 0.0])
        up_ref = alt

    # USD camera local axes: +X right, +Y up, +Z backward, therefore -Z forward.
    right = _normalize(np.cross(forward, up_ref))
    backward = -forward
    camera_up = _normalize(np.cross(backward, right))
    R_cam_target = np.column_stack((right, camera_up, backward))

    # 수치오차/축 구성 오류 진단.
    det = float(np.linalg.det(R_cam_target))
    if det < 0.99:
        raise RuntimeError(f"카메라 회전행렬이 우수좌표계가 아닙니다. det={det:.6f}")
    return R_cam_target


def lookat_to_link6_target(anchor_world, look_at, up=WORLD_UP, anchor_mode=ANCHOR_MODE):
    """고정점 모드에 따라 link6 목표 위치/자세를 계산한다.

    anchor_mode='link6': link6(그리퍼 끝점) 위치를 완전히 고정하고 팔 자세만 변경.
    anchor_mode='camera': 카메라 optical center를 완전히 고정하고 link6 위치를 오프셋만큼 보정.
    """
    anchor_world = np.asarray(anchor_world, dtype=float)
    look_at = np.asarray(look_at, dtype=float)

    if anchor_mode == "camera":
        camera_eye = anchor_world
        R_cam_target = make_usd_camera_rotation(camera_eye, look_at, up)
        R_link6_target = R_cam_target @ R_offset.T
        link6_target_pos = camera_eye - R_link6_target @ cam_local_pos_offset

    elif anchor_mode == "link6":
        link6_target_pos = anchor_world
        # 카메라 위치는 손목 회전에 따라 조금 변하므로 4회 반복해 실제 camera eye를 반영한다.
        R_link6_target = R_link6_0.copy()
        for _ in range(4):
            camera_eye = link6_target_pos + R_link6_target @ cam_local_pos_offset
            R_cam_target = make_usd_camera_rotation(camera_eye, look_at, up)
            R_link6_target = R_cam_target @ R_offset.T

    elif anchor_mode == "gripper_tip":
        # 그리퍼 손가락 실제 끝단(anchor_world)을 고정하고, 카메라는 여전히 look_at을 보게 한다.
        # link6은 손가락 끝단이 아니므로, 손가락 오프셋만큼 역산해서 link6 목표 위치를 구한다.
        tip_world = anchor_world
        R_link6_target = R_link6_0.copy()
        for _ in range(4):
            camera_eye = tip_world + R_link6_target @ (cam_local_pos_offset - gripper_tip_local_offset)
            R_cam_target = make_usd_camera_rotation(camera_eye, look_at, up)
            R_link6_target = R_cam_target @ R_offset.T
        link6_target_pos = tip_world - R_link6_target @ gripper_tip_local_offset
    else:
        raise ValueError(f"지원하지 않는 ANCHOR_MODE={anchor_mode!r}")

    q_link6_target = rot_matrices_to_quats(np.array([R_link6_target]))[0]
    return link6_target_pos, q_link6_target


def camera_alignment_check(look_at):
    """실제 Depth Camera의 시선과 수평 상태를 진단한다."""
    cam_pos_now, cam_quat_now = camera.get_world_pose(camera_axes=CAMERA_AXES)
    R_cam_now = quats_to_rot_matrices(np.array([cam_quat_now]))[0]
    forward_now = R_cam_now @ np.array([0.0, 0.0, -1.0])
    up_now = R_cam_now @ np.array([0.0, 1.0, 0.0])
    to_target_dir = _normalize(np.asarray(look_at, dtype=float) - np.asarray(cam_pos_now, dtype=float))
    alignment = float(np.dot(forward_now, to_target_dir))
    upright = float(np.dot(up_now, np.array(WORLD_UP)))
    return alignment, upright, cam_pos_now, cam_quat_now


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

# 그리퍼 끝점은 트렁크 입구의 낮은 위치에 고정한다.
# 로봇 베이스 상대 좌표가 아니라 트렁크의 실제 월드 좌표를 사용하므로 조절이 직관적이다.
anchor_pos = np.array([
    TRUNK_X_MIN - ANCHOR_OUTSIDE_OFFSET,
    ANCHOR_Y,
    TRUNK_FLOOR_Z + ANCHOR_HEIGHT_ABOVE_FLOOR,
], dtype=float)

# 기본 시선은 트렁크 가장 깊은 내부의 낮은 중앙 지점이다.
deep_x = TRUNK_X_MAX - DEEP_WALL_MARGIN
deep_center = np.array([
    deep_x,
    0.0,
    TRUNK_FLOOR_Z + DEEP_CENTER_HEIGHT,
], dtype=float)

target_pos, target_quat = lookat_to_link6_target(anchor_pos, deep_center, anchor_mode=ANCHOR_MODE)
print(
    f"[기본 자세 목표] link6_pos={np.round(target_pos,3)} "
    f"anchor={np.round(anchor_pos,3)} look_at=deep_center={np.round(deep_center,3)}",
    flush=True,
)

for step_i in range(BASIC_STEPS):
    actions = controller.forward(target_end_effector_position=target_pos, target_end_effector_orientation=target_quat)
    robot.apply_action(actions)
    world.step(render=True)

ee_pos, ee_quat = robot.end_effector.get_world_pose()
err = np.linalg.norm(np.array(ee_pos) - target_pos)
alignment, upright, cam_pos_now, cam_quat_now = camera_alignment_check(deep_center)
R_link6_now = quats_to_rot_matrices(np.array([ee_quat]))[0]
gripper_tip_now = np.array(ee_pos) + R_link6_now @ gripper_tip_local_offset
tip_err = np.linalg.norm(gripper_tip_now - anchor_pos) if ANCHOR_MODE == "gripper_tip" else float("nan")
print(
    f"[기본 자세 도달] mode={ANCHOR_MODE} ee_pos(link6)={np.round(ee_pos,3)} err(link6)={err:.4f}m "
    f"gripper_tip={np.round(gripper_tip_now,3)} tip_err={tip_err:.4f}m "
    f"alignment={alignment:.3f} upright={upright:.3f} cam_pos={np.round(cam_pos_now,3)}",
    flush=True,
)
print(
    f"[높이 확인] 그리퍼 끝단 Z={gripper_tip_now[2]:.4f} vs 트렁크 바닥 Z={TRUNK_FLOOR_Z:.4f} "
    f"(차이={gripper_tip_now[2]-TRUNK_FLOOR_Z:+.4f}m, 목표={ANCHOR_HEIGHT_ABOVE_FLOOR:+.4f}m)",
    flush=True,
)
pose_is_valid = alignment >= 0.95 and upright >= 0.80
if not pose_is_valid:
    print("[경고] alignment 또는 upright가 낮아 자동 스윕을 차단합니다. 카메라 prim/마운트 축을 확인하세요.", flush=True)

# 사용자 참고 사진과 비슷한 구도(측면 뒤쪽에서 로봇+차량 전체를 넓게)로 스크린샷 촬영 - 기존
# 스크립트의 근접/위쪽 시점 대신, 자세 자체를 오해 없이 비교하기 위함.
set_camera_view(eye=[base_pos[0] - 2.2, base_pos[1] - 3.2, base_pos[2] + 1.6], target=[(base_pos[0] + CAR_POS[0]) / 2, 0.0, 1.0])
for _ in range(20):
    world.step(render=True)
vp_util.capture_viewport_to_file(vp_util.get_active_viewport(), str(POSE_CHECK_DIR / "pose_check_reference_angle.png"))
for _ in range(5):
    world.step(render=True)
print("[참고 구도 스크린샷] _pose_check_reference_angle.png", flush=True)

RUN_SWEEP = DO_SWEEP and pose_is_valid
if not RUN_SWEEP:
    print("\n[진단 모드] 깊은 곳 기본 자세만 확인하고 스윕은 건너뜁니다. alignment/upright, 충돌 여부와 _live_rgb.png를 확인하세요.\n", flush=True)

# ================= 앵커 위치는 고정, look_at만 바꿔가며 트렁크 내부 여러 각도 스캔 =================
# ANCHOR_MODE="link6"이면 target_end_effector_position은 모든 waypoint에서 완전히 동일하다.
# 방향만 바뀌므로 팔의 다른 관절이 움직이면서 그리퍼 끝점 기준으로 시야를 회전시킨다.
SWEEP_WAYPOINTS = [
    # 깊은 곳 중앙을 기본으로 보고, 같은 깊이 평면에서 좌/우/아래/위를 훑는다.
    ("deep_center", deep_center),
    (
        "deep_left",
        np.array([deep_x, TRUNK_Y_MIN + SIDE_MARGIN, TRUNK_FLOOR_Z + DEEP_CENTER_HEIGHT]),
    ),
    (
        "deep_right",
        np.array([deep_x, TRUNK_Y_MAX - SIDE_MARGIN, TRUNK_FLOOR_Z + DEEP_CENTER_HEIGHT]),
    ),
    (
        "deep_floor",
        np.array([deep_x, 0.0, TRUNK_FLOOR_Z + FLOOR_MARGIN]),
    ),
    (
        "deep_ceiling",
        np.array([deep_x, 0.0, TRUNK_WALL_TOP - CEILING_MARGIN]),
    ),
]

captured_clouds = []
scan_meta = []

for name, look_at in (SWEEP_WAYPOINTS if RUN_SWEEP else []):
    t_pos, t_quat = lookat_to_link6_target(anchor_pos, look_at, anchor_mode=ANCHOR_MODE)
    for _ in range(SWEEP_STEPS):
        actions = controller.forward(target_end_effector_position=t_pos, target_end_effector_orientation=t_quat)
        robot.apply_action(actions)
        world.step(render=True)
    ee_pos_s, _ = robot.end_effector.get_world_pose()
    err_s = np.linalg.norm(np.array(ee_pos_s) - t_pos)
    alignment, upright, cam_pos_s, cam_quat_s = camera_alignment_check(look_at)
    print(
        f"[스윕:{name}] look_at={look_at} ee_pos={np.round(ee_pos_s,3)} err={err_s:.4f}m "
        f"cam_pos={np.round(cam_pos_s,3)} alignment={alignment:.3f} upright={upright:.3f}",
        flush=True,
    )
    rgb = camera.get_rgba()[:, :, :3]
    plt.imsave(str(SWEEP_DIR / f"sweep_{name}.png"), rgb)

    # 이 waypoint에서 depth pointcloud 캡처 (world_frame=True라 카메라 pose/intrinsic이 자동 반영됨).
    for _ in range(5):
        world.step(render=True)
    pcd = camera.get_pointcloud(world_frame=True)
    n_pts = 0 if pcd is None else len(pcd)
    print(f"[스윕:{name}] pointcloud 점 개수={n_pts}", flush=True)
    if n_pts > 0:
        captured_clouds.append(np.asarray(pcd))
        scan_meta.append({"name": name, "look_at": look_at.tolist(), "cam_pos": np.asarray(cam_pos_s).tolist(),
                           "cam_quat": np.asarray(cam_quat_s).tolist(), "n_points": int(n_pts)})

    set_camera_view(
        eye=[base_pos[0] - 1.2, base_pos[1] - 1.5, base_pos[2] + 1.3],
        target=[float(cam_pos_s[0]), float(cam_pos_s[1]), float(cam_pos_s[2])],
    )
    for _ in range(10):
        world.step(render=True)
    vp_util.capture_viewport_to_file(vp_util.get_active_viewport(), str(SWEEP_DIR / f"sweep_wide_{name}.png"))
    for _ in range(5):
        world.step(render=True)

if RUN_SWEEP:
    if captured_clouds:
        merged = np.concatenate(captured_clouds, axis=0)
        out_path = POINTCLOUD_DIR / "trunk_pointcloud.npy"
        np.save(out_path, merged)
        print(f"\n[병합] waypoint {len(captured_clouds)}개, 전체 포인트 개수={len(merged)}", flush=True)
        print(f"[저장] {out_path}", flush=True)

        import json
        meta_path = POINTCLOUD_DIR / "trunk_pointcloud_meta.json"
        with open(meta_path, "w") as f:
            json.dump(
                {
                    "trunk_bounds": {
                        "x": [TRUNK_X_MIN, TRUNK_X_MAX],
                        "y": [TRUNK_Y_MIN, TRUNK_Y_MAX],
                        "floor_z": TRUNK_FLOOR_Z,
                        "wall_top_z": TRUNK_WALL_TOP,
                    },
                    "anchor_pos": anchor_pos.tolist(),
                    "base_pos": np.asarray(base_pos).tolist(),
                    "base_quat": np.asarray(base_quat).tolist(),
                    "waypoints": scan_meta,
                },
                f,
                indent=2,
            )
        print(f"[저장] {meta_path}", flush=True)
    else:
        print("\n[경고] 캡처된 포인트가 없음", flush=True)

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
        cam_pos, cam_quat = camera.get_world_pose(camera_axes=CAMERA_AXES)
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
        vp_util.capture_viewport_to_file(viewport, str(LIVE_DIR / "live_wide.png"))
        rgb = camera.get_rgba()[:, :, :3]
        plt.imsave(str(LIVE_DIR / "live_rgb.png"), rgb)

simulation_app.close()
