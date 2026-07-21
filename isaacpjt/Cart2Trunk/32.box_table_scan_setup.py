"""
32.box_table_scan_setup.py
박스 스캔 전용 씬 구성 + 손목 카메라 고정 스캔 자세 + base_link->camera 변환 측정.

배경
----
box_top_extractor.py(perception/)는 depth로 박스 8꼭짓점을 뽑아서 JSON으로 저장하는데,
14_run_full_pipeline.py(algorism/)의 load_boxes_from_vision_json()이 coordinate_frame이
정확히 "m0609_base_link"가 아니면 바로 에러를 낸다. 지금까지 box_top_extractor.py는
box_scan_test_scene.usd(카메라+박스만 있는 독립 테스트 씬)로 검증했는데, 거기엔 로봇이
아예 없어서 base_link 좌표계로 변환할 방법이 없었다.

이 스크립트가 하는 일
----
1. 차량/트렁크(8/28번 스크립트 실측값)는 그대로 두고, 카트 "만" 테이블로 교체한다
   (place 목표인 트렁크는 실제 시나리오에 필요하므로 유지 - 처음엔 안 써도 된다고
   잘못 판단했다가 사용자가 바로잡음). 테이블 위에 축소된 박스 3개를 올린다.
2. Nova Carter + M0609(흡착 그리퍼 vgp20 + 카메라 포함, 9.simple_mobile_pick_demo.py와 동일
   결합 방식이지만 그리퍼는 28.cart_to_trunk_pick_place_lift.py의 vgp20 버전)를 구성한다.
   평행 그리퍼(onrobot_rg2ft)는 이 프로젝트에서 5cm 테스트 큐브 이상은 집어본 적이 없고
   손가락 벌어지는 폭이 지금 박스 크기(0.20~0.35m)엔 물리적으로 안 맞는다 - 나중에 실제
   pick&place를 붙일 때 그리퍼를 바꾸면 이 스크립트가 만든 카메라 마운트 보정이 무효가
   되므로, 스캔 단계부터 최종적으로 쓸 그리퍼(vgp20, 16~28번에서 검증된 패턴)로 맞춘다.
3. M0609 손목 depth 카메라를 찾아서 ROS2 카메라 브리지(OmniGraph)를 연결한다
   (M0609/6.pick_place_color_ros.py에 이미 검증된 ROS2CameraHelper 패턴을 depth로 확장 +
   camera_info는 별도 노드 ROS2CameraInfoHelper - ROS2CameraHelper의 type 값이 아니다).
4. RMPflow로 팔을 "테이블을 내려다보는" 고정 자세로 이동시킨다
   (12.trunk_scan_hidden_gripper.py의 lookat_to_link6_target 재사용, anchor_mode="camera" 고정).
5. 수렴 후 base_link 프림의 실측 world transform(XformCache)과 카메라 world pose를 각각
   측정해서 T(base_link->camera)를 계산하고 perception/base_to_camera_transform.json으로 저장한다.
   box_top_extractor.py는 이 파일을 읽어서 카메라 좌표계 결과를 base_link 좌표계로 변환한다.
6. 검증 스크린샷 저장 + 씬을 box_table_scan_scene.usd로 저장한다.

주의: 이 변환은 "이 스크립트가 만든 고정 자세"에서만 유효하다. 팔이 이후 다른 자세로
움직이면(예: 실제 pick&place 단계) 재측정이 필요하다 - 이번 작업 범위 밖.
"""

from isaacsim import SimulationApp

HEADLESS = True  # 1회 빌드+측정만 하고 종료 (스크린샷/USD/JSON 산출물만 필요)
simulation_app = SimulationApp({"headless": HEADLESS})

import json
import sys
from pathlib import Path

import numpy as np
import omni.usd
import omni.graph.core as og
import omni.kit.viewport.utility as vp_util
from isaacsim.core.utils.extensions import enable_extension
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf

enable_extension("isaacsim.ros2.bridge")

from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices, rot_matrices_to_quats
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.sensors.camera import Camera

_THIS_DIR = Path(__file__).resolve().parent
M0609_DIR = _THIS_DIR.parent / "M0609"
PERCEPTION_DIR = _THIS_DIR / "perception"
RMPFLOW_DIR = str(M0609_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController  # noqa: E402

# ================= USD 경로 =================
CAR_USD = str(_THIS_DIR / "assets/Lexus_IS300_Trunk_Open_No_More_Hell_Room.usdz")
# 28.cart_to_trunk_pick_place_lift.py와 동일한 vgp20(흡착 그리퍼)+카메라 버전 - 평행
# 그리퍼(Collected_m0609_camera)는 이 크기 박스를 못 집으므로 스캔 단계부터 맞춘다.
M0609_USD = str(M0609_DIR / "Collected_m0609_vgp20_camera" / "m0609_vgp20_camera.usd")
M0609_URDF_PATH = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH = str(M0609_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(M0609_DIR / "rmpflow/m0609_rmpflow_common.yaml")

# ================= 차량/트렁크 (8/28번 스크립트 실측값 그대로 재사용) =================
CAR_POS = (5.0, 0.0, 0.0)
CAR_EXTRA_SCALE = 0.50
CAR_ROT_Z = 0.0
SDF_RESOLUTION = 256

# ================= 테이블 (카트 자리를 대체) =================
CART_POS = (0.0, 0.0, 0.0)
# 처음엔 28.py의 CART_BASKET_FLOOR_Z(0.68)를 그대로 썼는데, 카메라(월드 z~1.12)와
# 큰 박스 윗면(0.68+0.22=0.90) 사이 여유가 0.22m밖에 안 남아서 라이브 테스트에서 큰 박스가
# 화면에 잘려 보였다(사용자 지적). 테이블을 낮추고, 카메라는 EYE_HEIGHT_ABOVE_TABLE을 늘려서
# 최대한 원래 절대 높이 근처를 유지하도록 함 - 결과적으로 카메라<->박스 윗면 여유가 커진다.
TABLE_TOP_Z = 0.40
TABLE_SIZE = (0.8, 0.6, TABLE_TOP_Z - 0.05)  # (x, y, height) - 박스 3개를 겹치지 않게 배치할 여유

# ================= 박스 3개 (표준 규격의 ~65~70% 축소판, 트렁크에 여유 있게 들어가도록) =================
# 표준 규격은 8.rescale_and_rebuild.py의 BOXES 참고. 실측 트렁크 내부(m0609_base_link 기준,
# results/run_20260720_200104/pointcloud/trunk_map.json): x span 0.83m, y span 1.23m,
# z span(바닥~ceiling_limit) 0.52m. 표준 규격도 개별로는 들어가지만 3개 동시 배치 + 장애물
# 5개까지 감안해 여유를 크게 뒀다.
# xy_offset은 8.rescale_and_rebuild.py의 BOXES(Medium/Large가 둘 다 (0,0))와 다르게 골랐다 -
# 거긴 "카트에 쏟아부은 계단식 더미" 연출이 의도였지만, 여기선 depth RANSAC이 박스 3개를
# 각각 독립된 윗면으로 분리 인식해야 하므로 겹치면 안 된다. 서로 다른 두 박스가 겹치지
# 않으려면 x축 또는 y축 중 하나에서라도 (반폭 합 + 여유)만큼 떨어져 있으면 된다 - 아래 값은
# 세 쌍 전부 그 조건을 만족하도록 계산한 것.
TABLE_BOXES = [
    ("Small", (0.20, 0.15, 0.12), (0.85, 0.25, 0.20), (-0.22, -0.12)),
    ("Medium", (0.28, 0.20, 0.18), (0.25, 0.65, 0.30), (0.22, -0.12)),
    ("Large", (0.35, 0.25, 0.22), (0.20, 0.35, 0.85), (0.0, 0.15)),
]
BOX_MASS_KG = {"Small": 1.0, "Medium": 2.0, "Large": 3.5}
TABLE_DROP_Z = TABLE_TOP_Z + 0.5

# ================= 로봇 (9.simple_mobile_pick_demo.py와 동일 결합 방식) =================
ROBOT_START_XY = (0.0, -0.55)  # 테이블(0,0) 앞쪽 - 9.py에서 검증된 ~0.35m급 reach 범위 안
FACE_ROT_Z = 90.0  # 테이블(+Y) 쪽을 보고 시작
MOUNT_Z = 0.42
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e8, 1e4, 1e8
EE_LINK_NAME = "link_6"
GRIPPER_BODY_NAME = "vgp20"  # 28.py와 동일 - 흡착 패드가 붙은 바디 프림 이름
TIP_LOCAL_OFFSET = (0.0, 0.0, 0.121)  # 28.py와 동일 (흡착 팁의 gripper body 로컬 오프셋)
GRASP_RADIUS = 0.10  # 28.py와 동일. 이 스크립트는 close()를 호출하지 않으므로 실사용은 안 함
DEPTH_CAMERA_NAME_HINT = "Depth"
CAMERA_AXES = "usd"  # USD Camera: +Y up, -Z forward
WORLD_UP = (0.0, 0.0, 1.0)
BASIC_STEPS = 350

# ================= 스캔 자세: 테이블을 내려다보는 고정 시점 =================
# 정확히 수직으로 내려다보면 forward·up_ref가 -1이 되어 make_usd_camera_rotation의 대체-up
# 분기를 타므로, 약간 로봇 쪽으로 기울여서(순수 수직이 아니게) 자연스러운 오블리크 뷰를 만든다.
# 그래도 박스 윗면 법선과의 정렬은 CAMERA_FACING_NORMAL_DOT_MIN(0.82, box_top_extractor.py)을
# 여유있게 만족한다 (수직에서 ~15-20도 정도).
EYE_HEIGHT_ABOVE_TABLE = 0.85  # 테이블을 낮춘 만큼(0.28m) 늘려서 카메라 절대 높이를 비슷하게 유지
SCAN_EYE = np.array([CART_POS[0], CART_POS[1] - 0.15, TABLE_TOP_Z + EYE_HEIGHT_ABOVE_TABLE])
SCAN_LOOK_AT = np.array([CART_POS[0], CART_POS[1], TABLE_TOP_Z])

# ================= ROS2 카메라 토픽 (box_top_extractor.py가 구독하는 것과 정확히 일치) =================
DEPTH_TOPIC = "/camera/depth"
CAMERA_INFO_TOPIC = "/camera/camera_info"
CAMERA_FRAME_ID = "m0609_depth_camera_optical_frame"
CAMERA_WIDTH, CAMERA_HEIGHT = 640, 480


# ================= DynamicSuctionGripper (28.cart_to_trunk_pick_place_lift.py와 동일) =================
# 이 스크립트는 흡착(close())을 실제로 호출하지 않는다 - SingleManipulator 생성자가
# gripper 객체를 요구해서 vgp20 그리퍼 형식을 맞추기 위해서만 만든다.
class DynamicSuctionGripper(SurfaceGripper):
    def __init__(self, end_effector_prim_path, gripper_body_path, target_prim_path,
                 tip_local_offset=(0.0, 0.0, 0.0), grasp_radius=0.03):
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

    def open(self) -> None:
        if self._attached:
            stage = omni.usd.get_context().get_stage()
            if stage.GetPrimAtPath(self._joint_path).IsValid():
                stage.RemovePrim(self._joint_path)
        self._attached = False

    def is_closed(self) -> bool:
        return self._attached

    def is_open(self) -> bool:
        return not self._attached


# ================= 헬퍼 (기존 스크립트에서 그대로 재사용) =================
def add_asset(stage, prim_path, usd_path, position, extra_scale, target_mpu, target_up, rot_z=0.0):
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


def add_dynamic_box(stage, prim_path, center, size, color, mass_kg):
    cube = UsdGeom.Cube.Define(stage, prim_path)
    cube.GetSizeAttr().Set(1.0)
    xform = UsdGeom.Xformable(cube)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*center))
    xform.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(mass_kg)
    print(f"[BOX] {prim_path} center={center} size={size}", flush=True)


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
    """9.simple_mobile_pick_demo.py와 동일한 방식으로 Nova Carter + M0609를 결합."""
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

    imu_prim = stage.GetPrimAtPath(f"{m0609_path}/onrobot_rg2ft/angle_bracket/realsense_d455/RSD455/Imu_Sensor")
    if imu_prim.IsValid():
        imu_prim.SetActive(False)
        print("[IMU] RSD455 Imu_Sensor 비활성화 (velocity tensor 에러 원인, 12.py와 동일 이유)", flush=True)

    n = add_drive_stiffness(stage, m0609_path)
    print(f"[DRIVE] {n}개 조인트 강성 재설정", flush=True)

    return carter_path, chassis_link_path, m0609_path


def bake_joint_drive_targets(stage, robot):
    """robot.set_joint_positions()는 "지금 이 순간 물리 상태"만 순간이동시키는 것이지,
    USD에 저장되는 UsdPhysics.DriveAPI의 targetPosition(관절 서보 목표각)은 안 건드린다
    (Isaac Sim 소스 docstring: "This method will immediately set (teleport)... Use the
    apply_action method to control robot joints"도 같은 경고). 그래서 그 상태로 저장하면
    joint_1~6의 targetPosition이 여전히 0으로 남아있고, 씬을 다시 열어 Play를 누르는 순간
    아주 높은 강성(1e8)이 전부 0으로 잡아당겨서 "모든 관절이 0도" 자세로 튄다 - 실제로
    사용자가 겪은 버그. box_table_scan_scene.usd를 pxr로 직접 열어서 6개 관절의
    drive_target이 전부 0.0인 것으로 확인했다. 여기서 현재 각도를 targetPosition에
    직접 구워넣어야 재실행 후에도 이 자세가 유지된다."""
    dof_names = robot.dof_names
    joint_positions_rad = robot.get_joint_positions()
    angle_by_name = dict(zip(dof_names, joint_positions_rad))

    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/MobileManipulator/M0609")):
        name = prim.GetName()
        if name not in angle_by_name:
            continue
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            continue
        degrees = float(np.degrees(angle_by_name[name]))
        drive.GetTargetPositionAttr().Set(degrees)
        n += 1
    print(f"[DRIVE TARGET] {n}개 관절의 targetPosition을 현재 각도로 고정", flush=True)


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


def _normalize(v, eps=1e-9):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < eps:
        raise ValueError(f"영벡터는 방향으로 사용할 수 없습니다: {v}")
    return v / n


def make_usd_camera_rotation(eye, look_at, up_ref=WORLD_UP):
    """USD Camera 축(+Y up, -Z forward)에 맞는 world rotation matrix (12.py와 동일)."""
    eye = np.asarray(eye, dtype=float)
    look_at = np.asarray(look_at, dtype=float)
    forward = _normalize(look_at - eye)
    up_ref = _normalize(up_ref)

    if abs(float(np.dot(forward, up_ref))) > 0.97:
        alt = np.array([0.0, 1.0, 0.0])
        if abs(float(np.dot(forward, alt))) > 0.97:
            alt = np.array([1.0, 0.0, 0.0])
        up_ref = alt

    right = _normalize(np.cross(forward, up_ref))
    backward = -forward
    camera_up = _normalize(np.cross(backward, right))
    R_cam_target = np.column_stack((right, camera_up, backward))

    det = float(np.linalg.det(R_cam_target))
    if det < 0.99:
        raise RuntimeError(f"카메라 회전행렬이 우수좌표계가 아닙니다. det={det:.6f}")
    return R_cam_target


def quat_wxyz_to_matrix(q) -> np.ndarray:
    """Isaac Sim 관례(w, x, y, z) 쿼터니언 -> 3x3 회전행렬 (13.export_trunk_map.py와 동일)."""
    w, x, y, z = q
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array([
        [1 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1 - (xx + yy)],
    ])


def setup_ros2_camera_bridge(camera_prim_path):
    """M0609/6.pick_place_color_ros.py에서 이미 검증된 ROS2CameraHelper 패턴을 depth로 확장.
    camera_info는 ROS2CameraHelper의 "type" 값이 아니라 별도 노드
    (isaacsim.ros2.bridge.ROS2CameraInfoHelper, renderProductPath만 받고 type 입력이
    없음)라서 처음엔 ROS2CameraHelper에 type="camera_info"를 잘못 줘서 매 프레임
    "type is not supported" 에러가 났다 - Isaac Sim 소스(OgnROS2CameraHelper.py의
    sensor_type 분기: rgb/depth/depth_pcl/instance_segmentation/semantic_segmentation/
    bbox_2d_tight/bbox_2d_loose/bbox_3d만 유효, camera_info 없음)로 실제 스키마 확인 후 수정.
    box_top_extractor.py가 구독하는 토픽과 정확히 일치시킨다
    (RGB는 box_top_extractor.py가 안 쓰므로 생략)."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/ROS2_Box_Scan_Camera_Graph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "DepthPublish.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "DepthPublish.inputs:renderProductPath"),
                ("CreateRenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", camera_prim_path),
                ("CreateRenderProduct.inputs:width", CAMERA_WIDTH),
                ("CreateRenderProduct.inputs:height", CAMERA_HEIGHT),
                ("DepthPublish.inputs:type", "depth"),
                ("DepthPublish.inputs:topicName", DEPTH_TOPIC),
                ("DepthPublish.inputs:frameId", CAMERA_FRAME_ID),
                ("DepthPublish.inputs:resetSimulationTimeOnStop", True),
                ("CameraInfoPublish.inputs:topicName", CAMERA_INFO_TOPIC),
                ("CameraInfoPublish.inputs:frameId", CAMERA_FRAME_ID),
                ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
            ],
        },
    )
    for _ in range(5):
        simulation_app.update()
    print(f"[ROS2] depth->{DEPTH_TOPIC}, camera_info->{CAMERA_INFO_TOPIC} 퍼블리시 그래프 생성 완료", flush=True)


# ================= 씬 구성 =================
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()
target_mpu = UsdGeom.GetStageMetersPerUnit(stage)
target_up = UsdGeom.GetStageUpAxis(stage)

add_asset(stage, "/World/Vehicle", CAR_USD, CAR_POS, CAR_EXTRA_SCALE, target_mpu, target_up, rot_z=CAR_ROT_Z)
for _ in range(20):
    simulation_app.update()
add_sdf_collision(stage, "/World/Vehicle")

table = FixedCuboid(
    prim_path="/World/BoxScanTable",
    name="box_scan_table",
    position=np.array([CART_POS[0], CART_POS[1], TABLE_TOP_Z - TABLE_SIZE[2] / 2.0]),
    scale=np.array(TABLE_SIZE),
    color=np.array([0.55, 0.40, 0.25]),
)

for name, size, color, (dx, dy) in TABLE_BOXES:
    add_dynamic_box(
        stage,
        f"/World/Box_{name}",
        (CART_POS[0] + dx, CART_POS[1] + dy, TABLE_DROP_Z),
        size,
        color,
        BOX_MASS_KG[name],
    )
    for _ in range(100):
        world.step(render=True)
    print(f"[낙하 완료] Box_{name}", flush=True)

carter_path, chassis_link_path, m0609_path = build_mobile_manipulator(stage)

camera_prim_path, all_cameras = find_camera_prim_path(stage, m0609_path, DEPTH_CAMERA_NAME_HINT)
print(f"[CAMERA 후보] {all_cameras}", flush=True)
if camera_prim_path is None:
    raise RuntimeError("M0609 하위에서 UsdGeom.Camera prim을 찾지 못했습니다.")
print(f"[CAMERA] 스캔에 사용할 depth 카메라: {camera_prim_path}", flush=True)

gripper = DynamicSuctionGripper(
    end_effector_prim_path=f"{m0609_path}/{EE_LINK_NAME}",
    gripper_body_path=f"{m0609_path}/{GRIPPER_BODY_NAME}",
    target_prim_path="/World/Box_Small",  # 이 스크립트는 close()를 안 부르므로 형식상 값
    tip_local_offset=TIP_LOCAL_OFFSET,
    grasp_radius=GRASP_RADIUS,
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

camera = Camera(prim_path=camera_prim_path, resolution=(CAMERA_WIDTH, CAMERA_HEIGHT))
camera.initialize()
camera.add_distance_to_image_plane_to_frame()
camera.add_rgb_to_frame()  # 실제 스캔 카메라 시야 확인용 (12.py와 동일 디버그 목적)
for _ in range(10):
    world.step(render=True)

# ================= base_link 실측 (근사값과 교차검증) =================
chassis_pos, chassis_quat = robot.get_world_pose()
base_pos_approx = np.array([chassis_pos[0], chassis_pos[1], chassis_pos[2] + MOUNT_Z])
base_quat_approx = np.asarray(chassis_quat)

xform_cache = UsdGeom.XformCache()
base_link_prim = stage.GetPrimAtPath(f"{m0609_path}/base_link")
base_mat = xform_cache.GetLocalToWorldTransform(base_link_prim)
base_pos = np.array(base_mat.ExtractTranslation())
base_quat_gf = base_mat.ExtractRotation().GetQuat()
base_quat = np.array([base_quat_gf.GetReal(), *base_quat_gf.GetImaginary()])

pos_diff = float(np.linalg.norm(base_pos - base_pos_approx))
print(
    f"[base_link 실측] pos={np.round(base_pos, 4)} quat={np.round(base_quat, 4)} "
    f"(근사값과의 차이={pos_diff:.5f}m - 0에 가까워야 정상)",
    flush=True,
)

# ================= link6 <-> camera 상대 오프셋 측정 (12.py와 동일) =================
link6_pos0, link6_quat0 = robot.end_effector.get_world_pose()
cam_pos0, cam_quat0 = camera.get_world_pose(camera_axes=CAMERA_AXES)
R_link6_0 = quats_to_rot_matrices(np.array([link6_quat0]))[0]
R_cam_0 = quats_to_rot_matrices(np.array([cam_quat0]))[0]
R_offset = R_link6_0.T @ R_cam_0
cam_local_pos_offset = R_link6_0.T @ (np.array(cam_pos0) - np.array(link6_pos0))
print(
    f"[오프셋] R_offset(link6->camera)=\n{R_offset}\ncamera pos offset in link6 frame={cam_local_pos_offset}",
    flush=True,
)


def lookat_to_link6_target(anchor_world, look_at, up=WORLD_UP):
    """anchor_mode='camera' 고정 버전 (12.py의 분기 중 하나만 필요하므로 단순화)."""
    camera_eye = np.asarray(anchor_world, dtype=float)
    look_at = np.asarray(look_at, dtype=float)
    R_cam_target = make_usd_camera_rotation(camera_eye, look_at, up)
    R_link6_target = R_cam_target @ R_offset.T
    link6_target_pos = camera_eye - R_link6_target @ cam_local_pos_offset
    q_link6_target = rot_matrices_to_quats(np.array([R_link6_target]))[0]
    return link6_target_pos, q_link6_target


# ================= RMPflow로 스캔 자세까지 이동 =================
controller = RMPFlowController(
    name="box_table_scan_controller",
    robot_articulation=robot,
    urdf_path=M0609_URDF_PATH,
    robot_description_path=M0609_DESCRIPTION_PATH,
    rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
    end_effector_frame_name=EE_LINK_NAME,
)
controller._default_position = base_pos
controller._default_orientation = base_quat
controller.rmp_flow.set_robot_base_pose(robot_position=base_pos, robot_orientation=base_quat)

target_pos, target_quat = lookat_to_link6_target(SCAN_EYE, SCAN_LOOK_AT)
print(
    f"[스캔 자세 목표] link6_pos={np.round(target_pos, 3)} eye={np.round(SCAN_EYE, 3)} "
    f"look_at={np.round(SCAN_LOOK_AT, 3)}",
    flush=True,
)

for _ in range(BASIC_STEPS):
    actions = controller.forward(target_end_effector_position=target_pos, target_end_effector_orientation=target_quat)
    robot.apply_action(actions)
    world.step(render=True)

# RMPflow는 매 스텝 실시간으로 관절을 서보하는 것이라, 수렴한 각도가 USD에 "고정 자세"로
# 저장되는 게 아니다. set_joint_positions()는 "현재 물리 상태"만 순간이동시킬 뿐 DriveAPI의
# targetPosition(관절 서보 목표각)은 그대로 둔다 - 이 상태로 save_as_stage()하면 씬을 다시
# 열어 Play를 눌렀을 때 아주 높은 강성(1e8)이 여전히 0인 targetPosition으로 전부 잡아당겨서
# "모든 관절이 0도" 자세로 튄다(사용자가 실제로 겪은 버그, box_table_scan_scene.usd를
# pxr로 직접 열어서 drive_target=0.0 확인함). set_joint_positions()로 즉시 상태를 맞추고,
# bake_joint_drive_targets()로 서보 목표각 자체도 다시 못박아야 재실행 후에도 유지된다.
converged_joint_positions = robot.get_joint_positions()
robot.set_joint_positions(converged_joint_positions)
for _ in range(10):
    world.step(render=True)
bake_joint_drive_targets(stage, robot)
for _ in range(10):
    world.step(render=True)

ee_pos, ee_quat = robot.end_effector.get_world_pose()
err = np.linalg.norm(np.array(ee_pos) - target_pos)
cam_pos_final, cam_quat_final = camera.get_world_pose(camera_axes=CAMERA_AXES)
R_cam_final = quats_to_rot_matrices(np.array([cam_quat_final]))[0]
forward_final = R_cam_final @ np.array([0.0, 0.0, -1.0])
to_target_dir = _normalize(SCAN_LOOK_AT - np.asarray(cam_pos_final))
alignment = float(np.dot(forward_final, to_target_dir))
print(
    f"[스캔 자세 도달] ee_pos(link6)={np.round(ee_pos, 3)} err={err:.4f}m "
    f"cam_pos={np.round(cam_pos_final, 3)} alignment={alignment:.3f} (1.0이면 완벽히 정면)",
    flush=True,
)
if err > 0.05:
    print("[경고] IK 수렴 오차가 5cm를 넘습니다 - ROBOT_START_XY/SCAN_EYE 재조정이 필요할 수 있습니다.", flush=True)

# ================= T(base_link -> camera) 계산 및 저장 =================
# R_cam_final은 camera.get_world_pose(camera_axes="usd")로 얻은 것이라 USD 카메라 축
# (+X right, +Y up, +Z backward, 즉 -Z가 forward)이다. 그런데 box_top_extractor.py의
# depth_to_points()가 만드는 점은 표준 ROS optical 축(+X right, +Y down, +Z forward,
# depth가 그대로 +Z)이다 - 두 축 규약이 Y/Z 부호가 반대라서, 이 보정 없이 바로 R_cam_final을
# 썼더니 박스 "윗면"이 지지면(테이블)보다 오히려 낮은 z로 나오는 뒤집힌 결과가 나왔다(실제
# 라이브 테스트에서 발견). optical -> USD 카메라 축 변환은 X축 기준 180도 회전
# diag(1,-1,-1) 하나로 충분하다 (자기 자신이 역변환이기도 함).
OPTICAL_TO_USD_CAMERA_AXES = np.diag([1.0, -1.0, -1.0])
R_base = quat_wxyz_to_matrix(base_quat)
R_base_to_cam = R_base.T @ R_cam_final @ OPTICAL_TO_USD_CAMERA_AXES
t_base_to_cam = R_base.T @ (np.array(cam_pos_final) - base_pos)

PERCEPTION_DIR.mkdir(parents=True, exist_ok=True)
transform_path = PERCEPTION_DIR / "base_to_camera_transform.json"
transform_payload = {
    "R": R_base_to_cam.tolist(),
    "t": t_base_to_cam.tolist(),
    "note": (
        "32.box_table_scan_setup.py가 만든 고정 스캔 자세 전용. "
        "팔이 이 자세를 벗어나면 무효 - 재측정 필요."
    ),
    "measured_base_pos": base_pos.tolist(),
    "measured_base_quat": base_quat.tolist(),
    "measured_camera_pos": np.asarray(cam_pos_final).tolist(),
    "measured_camera_quat": np.asarray(cam_quat_final).tolist(),
    "ik_convergence_error_m": float(err),
    "camera_alignment": float(alignment),
}
transform_path.write_text(json.dumps(transform_payload, indent=2))
print(f"[저장] {transform_path}", flush=True)

# ================= ROS2 카메라 브리지 연결 =================
setup_ros2_camera_bridge(camera_prim_path)

# ================= 검증 스크린샷 =================
viewport = vp_util.get_active_viewport()
set_camera_view(
    eye=[ROBOT_START_XY[0] - 1.0, ROBOT_START_XY[1] - 1.3, TABLE_TOP_Z + 1.0],
    target=[CART_POS[0], CART_POS[1], TABLE_TOP_Z],
)
for _ in range(20):
    world.step(render=True)
screenshot_path = str(_THIS_DIR / "_verify_box_table_scan.png")
vp_util.capture_viewport_to_file(viewport, screenshot_path)
for _ in range(5):
    world.step(render=True)
print(f"[SCREENSHOT] {screenshot_path}", flush=True)

# 외부 시점 스크린샷과 별개로, 실제 스캔 카메라 자신이 보는 화면을 저장한다 -
# 박스가 화면 안에 다 들어오는지(FOV 클리핑 여부)는 이 이미지로만 확인 가능하다.
camera_view_path = str(_THIS_DIR / "_verify_box_table_scan_camera_view.png")
rgb = camera.get_rgba()[:, :, :3]
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.imsave(camera_view_path, rgb)
print(f"[SCREENSHOT] {camera_view_path} (스캔 카메라 자체 시야)", flush=True)

# ================= 씬 저장 =================
# NOTE(3.fix_container_collision.py와 동일 주의): save_as_stage()는 스테이지 재로드 이벤트를
# 일으켜 World 싱글턴을 무효화한다. 그래서 world.step()은 전부 이 위에서 끝내고, 저장은
# 스크립트의 마지막 동작으로만 한다.
scene_path = str(_THIS_DIR / "box_table_scan_scene.usd")
omni.usd.get_context().save_as_stage(scene_path)
print(f"[저장] {scene_path}", flush=True)

print("\n[완료] 이 씬을 Isaac Sim GUI로 열고 Play를 누르면 ROS2 카메라 토픽이 즉시 나옵니다.\n", flush=True)
simulation_app.close()
