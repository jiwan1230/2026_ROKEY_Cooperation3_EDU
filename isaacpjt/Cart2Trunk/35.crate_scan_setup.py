"""
35.crate_scan_setup.py
트렁크(차량) 대신 오픈-탑 크레이트 + 더미 장애물 박스 2개로 최소 환경을 구성하고,
테이블(대상 박스)과 크레이트(장애물) 양쪽을 실제로 스캔한다.

배경 (32/33.py에서 뭘 바꿨나)
----
33.box_table_pick_to_trunk.py까지의 데모는 실제 차량+트렁크 모델을 목표 지점으로
썼는데, 사용자가 두 가지를 지적했다:
1. 트렁크 문(천장) 높이 제한 때문에 접근 waypoint(entrance/interior)를 따로 만들어야
   했고, Nova Carter가 차체에 부딪히는 문제 때문에 충돌을 임시로 꺼서 통과시키는
   비정상적인 타협(UsdPhysics.FilteredPairsAPI)까지 해야 했다.
2. 트렁크가 비어있는 채로 박스 하나만 넣어봤을 뿐이라, 적재 알고리즘이 "장애물을
   피해서 빈 자리를 찾는" 능력을 전혀 보여주지 못했다.

그래서 차량/트렁크를 완전히 들어내고, 안쪽이 파인 오픈-탑 크레이트(벽 4장 + 바닥,
뚜껑 없음 - 그리퍼가 위에서 부딪힐 게 없음)로 교체한다. 크레이트 안에는 더미 박스
2개를 미리 놓아서, 테이블에서 새로 인식한 박스를 그 사이 빈 자리에 알고리즘이
어떻게 배치하는지 시각적으로 잘 보이게 한다.

장애물 회피는 이미 algorism/02_trunk_space_state.py의 load_obstacles_from_world_map()
+ 14_run_full_pipeline.py의 ExtremePointState.register_placement()로 완전히 구현돼
있다 - 이 스크립트는 trunk_map.json의 obstacles 필드만 채우면 되고, 알고리즘 코드는
하나도 안 건드린다.

이 스크립트가 하는 일
----
1. 차량/트렁크 없이: 크레이트(더미 박스 2개 포함) + 테이블(박스 3개, 32.py와 동일) +
   Nova Carter+M0609(vgp20)를 구성한다. 로봇 베이스 위치는 32.py와 동일 -
   테이블과 크레이트 둘 다 로봇 베이스 기준 검증된 리치 범위(~0.3-0.5m) 안에 들어오도록
   배치해서, 베이스를 단 한 번도 옮기지 않고 팔 회전만으로 양쪽에 닿는다(33.py의
   텔레포트+충돌해제 편법이 필요 없다).
2. RMPflow로 팔을 테이블 스캔 자세로 수렴시키고, base_to_camera_transform.json을
   측정+저장한 뒤, 마커 파일이 나타날 때까지 world.step()을 반복하며 대기한다 -
   그 사이 별도 프로세스(perception/run_scan_once.py)가 실제 depth 프레임을 받아
   box_top_extractor.py의 검출 로직을 그대로 돌리고 저장한다(테이블 위 대상 박스들).
3. 팔을 크레이트 스캔 자세로 재수렴시키고, transform.json을 다시 측정+덮어쓰기한 뒤
   같은 방식으로 대기 -> 더미 박스 2개가 검출된다(크레이트 안 장애물).
4. 크레이트 자체의 8개 코너(우리가 만든 값이므로 하드코딩, world -> base_link 변환만
   적용)와 방금 스캔된 더미 박스 코너(obstacles)를 합쳐 trunk_map.json을 직접 작성한다
   - RANSAC으로 크레이트 벽을 다시 스캔할 필요가 없다(우리가 만든 도형이라 이미 정확히
     아는 값을 스캔으로 재발견할 이유가 없음).
5. 검증 스크린샷(외부 시점 2장 + 카메라 자체 시점 2장) 저장 + 씬을
   crate_scan_scene.usd로 저장한다.
"""

from isaacsim import SimulationApp

import os

# 기본은 GUI로 직접 보이게 - 스캔 자세로 수렴해서 마커를 기다리는 동안 로봇이 멈춰있는
# 것도 직접 볼 수 있다. 자동 검증용으로 헤드리스가 필요하면 HEADLESS=1로 실행:
#   HEADLESS=1 isaac_python 35.crate_scan_setup.py
HEADLESS = os.environ.get("HEADLESS", "0") == "1"
# GUI 렌더링은 헤드리스보다 훨씬 느리다(36.py에서 실측: 234초 vs 20~30초) - 해상도를
# 낮춰서 물리 진행과 화면이 크게 벌어지지 않게 한다. 헤드리스 스크린샷 품질에는 영향 없음.
_sim_app_config = {"headless": HEADLESS}
if not HEADLESS:
    _sim_app_config.update({"width": 640, "height": 480})
simulation_app = SimulationApp(_sim_app_config)

import json
import subprocess
import sys
import time
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
RESULTS_DIR = _THIS_DIR / "results" / "crate_demo"
RMPFLOW_DIR = str(M0609_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController  # noqa: E402

# ================= USD 경로 =================
M0609_USD = str(M0609_DIR / "Collected_m0609_vgp20_camera" / "m0609_vgp20_camera.usd")
M0609_URDF_PATH = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH = str(M0609_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(M0609_DIR / "rmpflow/m0609_rmpflow_common.yaml")

# ================= 테이블 (32.py와 동일 - 대상 박스 스캔용, 그대로 재사용) =================
CART_POS = (0.0, 0.0, 0.0)
TABLE_TOP_Z = 0.40
TABLE_SIZE = (0.8, 0.6, TABLE_TOP_Z - 0.05)

TABLE_BOXES = [
    ("Small", (0.20, 0.15, 0.12), (0.85, 0.25, 0.20), (-0.22, -0.12)),
    ("Medium", (0.28, 0.20, 0.18), (0.25, 0.65, 0.30), (0.22, -0.12)),
    ("Large", (0.35, 0.25, 0.22), (0.20, 0.35, 0.85), (0.0, 0.15)),
]
BOX_MASS_KG = {"Small": 1.0, "Medium": 2.0, "Large": 3.5}
TABLE_DROP_Z = TABLE_TOP_Z + 0.5

# ================= 로봇 (32.py와 동일 결합 방식/위치 - 베이스는 이번에도 안 움직인다) =================
ROBOT_START_XY = (0.0, -0.55)
FACE_ROT_Z = 90.0  # 테이블(+Y) 쪽을 보고 시작
MOUNT_Z = 0.42
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 1e8, 1e4, 1e8
EE_LINK_NAME = "link_6"
GRIPPER_BODY_NAME = "vgp20"
TIP_LOCAL_OFFSET = (0.0, 0.0, 0.121)
GRASP_RADIUS = 0.10
DEPTH_CAMERA_NAME_HINT = "Depth"
CAMERA_AXES = "usd"
WORLD_UP = (0.0, 0.0, 1.0)
BASIC_STEPS = 350

# ================= 크레이트 (트렁크 대체 - 오픈 탑, 뚜껑 없음) =================
# 1차: 로봇 뒤쪽(-Y, CRATE_CENTER_XY=(0,-1.0))에 뒀다가 섀시-크레이트 벽 충돌(스폰 시
# PhysX가 밀어내며 섀시가 기울어짐, base_link 위치 오차 0.087m로 실측 확인)로 실패.
# 2차: 간격만 0.63m로 늘렸으나(0,-1.4) - 이번엔 36.py의 PLACE 단계에서 리포지셔닝한
# 섀시가 "로봇 뒤쪽" 접근이라 크레이트 벽에 다시 너무 가까워져(짧은 벽이지만 그 아래
# 바닥 지지대는 테이블처럼 두께가 있어 챠시와 겹침, 스크린샷으로 재확인) 또 충돌했다.
# 최종: 테이블과 "같은 방식"(로봇이 그 앞(-Y)에 서서 접근)으로 크레이트를 테이블
# 옆(+X)에 둔다 - 이건 테이블-로봇 배치(간격 0.25m)가 이미 충돌 없이 검증된 패턴이라
# 그대로 재사용하는 것. 36.py도 리포지셔닝 시 로봇을 크레이트 앞(y=-0.55, x만 이동)에
# 세워서 같은 안전한 접근 형태를 그대로 따라간다.
CRATE_CENTER_XY = (0.9, 0.0)
# 크레이트를 내려다보려고 카메라 lookat 자세를 (0.9,...)로 요청했더니 IK가 나쁜 로컬해로
# 수렴했다(err=0.345m, alignment=0.295 - box_top_extractor.py의 CAMERA_FACING_NORMAL_DOT_MIN
# =0.82에 한참 못 미쳐서 평면 후보가 0개 나옴, 실측 확인). 섀시가 그대로 (0,-0.55)에 있는
# 채로 0.9m 옆을 내려다보려니 팔이 옆으로 뻗어야 해서 무리였던 것 - 테이블 스캔(섀시
# 정면, 0.55m 거리)과 똑같이 좋은 조건을 만들려고, 크레이트 스캔 때만 섀시를 크레이트
# 앞(같은 y오프셋 0.55)으로 잠깐 옮긴다(회전 없이 x만 이동, 스캔 끝나면 원위치로 복귀 -
# 36.py가 테이블 위치에서 PICK을 시작한다는 가정이 그대로 유지되게).
CRATE_ROBOT_XY = (CRATE_CENTER_XY[0], ROBOT_START_XY[1])
# 처음엔 (0.65,0.30) - 알고리즘이 찾은 배치 자리가 벽에서 5cm도 안 떨어져서, 팔의
# 통상 오차(~0.13-0.2m)만으로 박스가 벽 너머로 떨어졌다(실측 확인). 목표 좌표를
# 손으로 당겨봤다가 RMPflow가 오히려 크게 발산해서(교훈: 미세 목표 변경에 지역해가
# 불안정하게 반응할 수 있음) 목표는 그대로 두고 벽 자체의 여유를 넓힌다.
CRATE_INNER_SIZE = (0.85, 0.40)  # (x폭, y깊이) - 깊이도 넓혀서 더미/목표 박스 사이 여유를 더 준다
CRATE_WALL_HEIGHT = 0.15  # 뚜껑 없음 - 그리퍼가 위에서 접근할 때 부딪힐 게 없는 낮은 벽 (물리 지오메트리용)
CRATE_WALL_THICKNESS = 0.02
# trunk_map.json에 적는 "높이 한계"는 벽의 실제 물리 높이(0.15m)가 아니라 별도의 넉넉한
# 가상값을 쓴다 - 안 그러면 02_trunk_space_state.py의 Trunk.height가 0.15m로 잡혀서
# 알고리즘이 SIZE_EXCEEDS_TRUNK로 거부한다(실제로 겪음: Medium 박스 높이 0.17m > 0.15m).
# 트렁크 문이 닫히는 진짜 천장이 없는 오픈-탑 크레이트의 핵심 취지가 "높이 제한 없음"이므로,
# 물리 벽 높이와 알고리즘이 보는 높이 한계를 의도적으로 분리한다.
TRUNK_MAP_VIRTUAL_HEIGHT = 1.0
# 사용자 지적: PICK 자세(테이블 위, ee 높이~0.78, 매달린 박스 바닥~0.57)를 그대로 안고
# 섀시만 크레이트 앞으로 텔레포트하면, 그 순간 팔/박스가 크레이트 벽 꼭대기(당시
# 0.40+0.15=0.55)에 거의 닿을 듯 말듯한 높이라 실행마다 충돌 여부가 오락가락했을
# 가능성이 높다(더 많은 안정화 스텝/새 controller/joint pin 다 안 먹혔던 것과 부합 -
# RMPflow 문제가 아니라 물리적으로 막혀있었을 수 있음). 크레이트를 훨씬 낮춰서
# 텔레포트 직후 팔이 지나는 높이보다 벽이 한참 아래 있게 한다.
CRATE_FLOOR_TOP_Z = 0.15
CRATE_WALL_COLOR = (0.35, 0.35, 0.38)
CRATE_FLOOR_COLOR = (0.30, 0.30, 0.33)

# 더미 장애물 박스 2개 - 크레이트 양쪽 벽에 붙여서 가운데에 대상 박스가 들어갈 틈을 남긴다.
# 원래는 x만 벽에 붙이고 y는 중앙(CRATE_CENTER_XY[1])이었는데, 그러면 목표 박스가 극점
# 알고리즘 특성상 더미 바로 옆에 딱 붙어서 배치되는 경우가 많아(장애물 회피는 맞지만
# 여유가 거의 없음) - 실측된 obstacle 크기가 실제보다 작게 잡히는 경우(스캔 오차)
# 이 "딱 붙임"이 실제로는 겹침이 될 위험이 있다. x/y 둘 다 벽에 붙여서 실제 모서리에
# 박아두면, 가운데(y 반대쪽 절반)가 어느 장애물과도 안 가까운 확실한 여유 공간이 된다.
DUMMY_BOX_SIZE = (0.15, 0.15, 0.15)
# 처음엔 DUMMY_MARGIN=0.01로 x/y 모서리에 동시에 박아봤다가 실측 실패 - 벽 두께
# (CRATE_WALL_THICKNESS=0.02)까지 감안하면 사실상 벽과 겹치는 위치라 스폰 즉시
# PhysX가 밀어냈고(스크린샷: 한쪽은 찌그러져 바닥에 눌리고 한쪽은 벽 위로 튕겨나감),
# 그 결과 depth 스캔이 박스 대신 엉뚱한 z(-1.1~-1.7m)의 거대한 "floor" 오탐만 잡았다
# (섀시-크레이트 벽 충돌 때와 같은 종류의 실수 - 모서리에 박아둘수록 여유를 더 줘야
# 하는데 거꾸로 더 줄였음). 모서리 방향으로 밀되 벽에서 확실히 떨어지도록 여유를 키운다.
DUMMY_MARGIN = 0.05  # 벽 안쪽면에서 더미 박스 중심까지의 여유
_dummy_half_w = DUMMY_BOX_SIZE[0] / 2.0
_dummy_half_d = DUMMY_BOX_SIZE[1] / 2.0
_crate_x_min = CRATE_CENTER_XY[0] - CRATE_INNER_SIZE[0] / 2.0
_crate_x_max = CRATE_CENTER_XY[0] + CRATE_INNER_SIZE[0] / 2.0
_crate_y_min = CRATE_CENTER_XY[1] - CRATE_INNER_SIZE[1] / 2.0
_dummy_corner_y = _crate_y_min + DUMMY_MARGIN + _dummy_half_d  # 둘 다 같은(y_min) 쪽 모서리에
DUMMY_BOXES = [
    ("DummyA", (_crate_x_min + DUMMY_MARGIN + _dummy_half_w, _dummy_corner_y, CRATE_FLOOR_TOP_Z + DUMMY_BOX_SIZE[2] / 2.0)),
    ("DummyB", (_crate_x_max - DUMMY_MARGIN - _dummy_half_w, _dummy_corner_y, CRATE_FLOOR_TOP_Z + DUMMY_BOX_SIZE[2] / 2.0)),
]
DUMMY_MASS_KG = 1.0

# ================= 스캔 자세 1: 테이블을 내려다보는 고정 시점 (32.py와 동일) =================
EYE_HEIGHT_ABOVE_TABLE = 0.85
# 사용자가 실측/실험으로 찾은 값: 박스가 쌓인 공간을 스캔할 때 수직(정면 위)에서
# 21도 기울여서 볼 때 검출 정확도가 가장 높다. 높이(EYE_HEIGHT_ABOVE_TABLE)는
# 그대로 두고, 그 높이에서 21도가 나오도록 카메라의 수평 오프셋만 계산한다
# (기존엔 오프셋 0.15m 고정값이라 실제로는 약 10도였음 - atan(0.15/0.85)).
SCAN_TILT_FROM_VERTICAL_DEG = 21.0
_scan_horizontal_offset = EYE_HEIGHT_ABOVE_TABLE * np.tan(np.radians(SCAN_TILT_FROM_VERTICAL_DEG))
SCAN_EYE = np.array([CART_POS[0], CART_POS[1] - _scan_horizontal_offset, TABLE_TOP_Z + EYE_HEIGHT_ABOVE_TABLE])
SCAN_LOOK_AT = np.array([CART_POS[0], CART_POS[1], TABLE_TOP_Z])

# ================= 스캔 자세 2: 크레이트를 내려다보는 고정 시점 =================
# 크레이트가 이제 테이블과 같은 y밴드(로봇이 -y쪽에서 접근)에 있으므로, 테이블 스캔과
# 동일하게 eye를 -y로 살짝 기울여 완전 수직을 피한다.
# 처음엔 0.65(테이블의 0.85보다 낮게)로 했다가 IK가 나쁜 로컬해로 수렴해서 카메라가
# 더미 박스 하나에 거의 닿을 만큼 가깝게 내려가버렸다(스크린샷으로 확인, top
# candidates=0 - 너무 가까워서 평면 검출 자체가 실패). 테이블과 같은 높이로 맞춘다.
CRATE_EYE_HEIGHT_ABOVE_FLOOR = 0.85
CRATE_SCAN_EYE = np.array([CRATE_CENTER_XY[0], CRATE_CENTER_XY[1] - 0.15, CRATE_FLOOR_TOP_Z + CRATE_EYE_HEIGHT_ABOVE_FLOOR])
CRATE_SCAN_LOOK_AT = np.array([CRATE_CENTER_XY[0], CRATE_CENTER_XY[1], CRATE_FLOOR_TOP_Z])

# ================= ROS2 카메라 토픽 (box_top_extractor.py가 구독하는 것과 정확히 일치) =================
DEPTH_TOPIC = "/camera/depth"
CAMERA_INFO_TOPIC = "/camera/camera_info"
CAMERA_FRAME_ID = "m0609_depth_camera_optical_frame"
CAMERA_WIDTH, CAMERA_HEIGHT = 640, 480

# ================= 마커 파일 핸드셰이크 (perception/run_scan_once.py와 짝, 테이블 스캔 전용 -
# 크레이트 스캔은 get_pointcloud() 동기 호출로 바뀌면서 더 이상 마커가 필요 없다) =================
SCAN_MARKER_DIR = Path("/tmp/claude-1000/-home-rokey-cobot3-ws/1a920049-f3c7-4d48-90f4-62c8e0fe71a3/scratchpad")
TABLE_SCAN_MARKER = SCAN_MARKER_DIR / "scan_table.done"
MAX_HOLD_STEPS = 6000  # 안전 타임아웃 (약 100초 상당, world.step 기준)


# ================= DynamicSuctionGripper (32/33.py와 동일) =================
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


# ================= 헬퍼 (32.py에서 재사용) =================
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


def build_open_crate(center_xy, floor_top_z, inner_size, wall_height, wall_thickness,
                      virtual_height=None):
    """바닥(FixedCuboid, 테이블과 동일 패턴의 지지대) + 벽 4장(FixedCuboid, 뚜껑 없음).
    반환값: trunk_map.json에 쓸 8개 코너(월드 좌표, bottom loop 0-3 + top loop 4-7,
    관례와 동일한 순서) - 벽/더미 박스는 물리 오브젝트라 스캔해서 얻지만, 이 8개 코너는
    우리가 만든 값이라 스캔 없이 바로 계산한다.
    virtual_height가 주어지면 top loop(z_max)는 실제 벽 높이(wall_height) 대신 이 값을
    쓴다 - 오픈-탑이라 진짜 천장이 없는데, 알고리즘(Trunk.height)에는 물리 벽 높이가 아니라
    "높이 제한 없음"에 해당하는 넉넉한 값을 줘야 하기 때문(실제 벽은 그리퍼 충돌 방지용으로만
    낮게 유지)."""
    cx, cy = center_xy
    iw, idepth = inner_size
    x_min, x_max = cx - iw / 2.0, cx + iw / 2.0
    y_min, y_max = cy - idepth / 2.0, cy + idepth / 2.0
    z_min = floor_top_z
    z_max = floor_top_z + (virtual_height if virtual_height is not None else wall_height)

    outer_w = iw + 2 * wall_thickness
    outer_d = idepth + 2 * wall_thickness

    FixedCuboid(
        prim_path="/World/Crate/Floor",
        name="crate_floor",
        position=np.array([cx, cy, floor_top_z / 2.0]),
        scale=np.array([outer_w, outer_d, floor_top_z]),
        color=np.array(CRATE_FLOOR_COLOR),
    )

    wall_center_z = floor_top_z + wall_height / 2.0
    FixedCuboid(
        prim_path="/World/Crate/WallXMin", name="crate_wall_x_min",
        position=np.array([x_min - wall_thickness / 2.0, cy, wall_center_z]),
        scale=np.array([wall_thickness, outer_d, wall_height]), color=np.array(CRATE_WALL_COLOR),
    )
    FixedCuboid(
        prim_path="/World/Crate/WallXMax", name="crate_wall_x_max",
        position=np.array([x_max + wall_thickness / 2.0, cy, wall_center_z]),
        scale=np.array([wall_thickness, outer_d, wall_height]), color=np.array(CRATE_WALL_COLOR),
    )
    FixedCuboid(
        prim_path="/World/Crate/WallYMin", name="crate_wall_y_min",
        position=np.array([cx, y_min - wall_thickness / 2.0, wall_center_z]),
        scale=np.array([iw, wall_thickness, wall_height]), color=np.array(CRATE_WALL_COLOR),
    )
    FixedCuboid(
        prim_path="/World/Crate/WallYMax", name="crate_wall_y_max",
        position=np.array([cx, y_max + wall_thickness / 2.0, wall_center_z]),
        scale=np.array([iw, wall_thickness, wall_height]), color=np.array(CRATE_WALL_COLOR),
    )
    print(f"[CRATE] center={center_xy} inner={inner_size} wall_h={wall_height} "
          f"floor_top_z={floor_top_z}", flush=True)

    vertices_world = [
        (x_min, y_min, z_min), (x_max, y_min, z_min), (x_max, y_max, z_min), (x_min, y_max, z_min),
        (x_min, y_min, z_max), (x_max, y_min, z_max), (x_max, y_max, z_max), (x_min, y_max, z_max),
    ]
    return vertices_world


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
    """32.box_table_scan_setup.py와 동일한 방식으로 Nova Carter + M0609를 결합."""
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
        print("[IMU] RSD455 Imu_Sensor 비활성화 (velocity tensor 에러 원인)", flush=True)

    n = add_drive_stiffness(stage, m0609_path)
    print(f"[DRIVE] {n}개 조인트 강성 재설정", flush=True)

    return carter_path, chassis_link_path, m0609_path


def bake_joint_drive_targets(stage, robot):
    """set_joint_positions()는 물리 상태만 순간이동시키고 UsdPhysics.DriveAPI의
    targetPosition은 그대로 두므로(32.py에서 실제로 겪은 버그), 여기서 서보 목표각
    자체를 현재 각도로 다시 못박는다 - 재실행 후에도 이 자세가 유지되게 하려면 필수."""
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
    """USD Camera 축(+Y up, -Z forward)에 맞는 world rotation matrix (12/32.py와 동일)."""
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
    """Isaac Sim 관례(w, x, y, z) 쿼터니언 -> 3x3 회전행렬."""
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
    """32.py에서 검증된 패턴 그대로 - depth + camera_info 퍼블리시."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/ROS2_Crate_Scan_Camera_Graph", "evaluator_name": "execution"},
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


def hold_until_marker(world, marker_path: Path, label: str):
    """마커 파일이 생길 때까지 world.step()을 반복한다 - 그 사이 별도 프로세스
    (run_scan_once.py)가 이 씬이 지금 퍼블리시하는 실제 depth 프레임을 받아서 스캔한다."""
    if marker_path.exists():
        marker_path.unlink()
    print(f"[HOLD] {label} - 마커 대기 시작: {marker_path}", flush=True)
    n = 0
    while not marker_path.exists():
        world.step(render=True)
        n += 1
        if n % 300 == 0:
            print(f"[HOLD] {label} - 대기 중... ({n} steps)", flush=True)
        if n >= MAX_HOLD_STEPS:
            raise RuntimeError(f"[HOLD] {label} - 마커가 {MAX_HOLD_STEPS} step 동안 나타나지 않았습니다 "
                                f"(run_scan_once.py를 이 시점에 실행했는지 확인).")
    print(f"[HOLD] {label} - 마커 확인, 진행 ({n} steps 대기)", flush=True)


# ================= 씬 구성 =================
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()

crate_vertices_world = build_open_crate(CRATE_CENTER_XY, CRATE_FLOOR_TOP_Z, CRATE_INNER_SIZE,
                                         CRATE_WALL_HEIGHT, CRATE_WALL_THICKNESS,
                                         virtual_height=TRUNK_MAP_VIRTUAL_HEIGHT)
for _ in range(20):
    simulation_app.update()

for name, center in DUMMY_BOXES:
    add_dynamic_box(stage, f"/World/Crate/{name}", center, DUMMY_BOX_SIZE, (0.6, 0.2, 0.2), DUMMY_MASS_KG)
    for _ in range(100):
        world.step(render=True)
    print(f"[낙하 완료] {name}", flush=True)

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
    target_prim_path="/World/Box_Small",
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

# 크레이트/테이블 배치가 섀시와 물리적으로 겹치지 않는지 RMPflow 수렴 전에 먼저 눈으로
# 확인한다 - 겹치면 스폰 즉시 PhysX가 섀시를 밀어내서 이후 모든 IK가 깨지는데, 그걸
# 스캔 자세 수렴까지 다 돌고 나서야 알아채면 GPU 시간이 아깝다(실제로 한 번 겪음).
_early_viewport = vp_util.get_active_viewport()
set_camera_view(eye=[0.4, -2.0, 1.8], target=[0.5, -0.3, 0.4])
for _ in range(10):
    world.step(render=True)
vp_util.capture_viewport_to_file(_early_viewport, str(_THIS_DIR / "_verify_crate_scan_00_layout.png"))
for _ in range(5):
    world.step(render=True)
print("[SCREENSHOT] _verify_crate_scan_00_layout.png (배치 충돌 여부 조기 확인용)", flush=True)

camera = Camera(prim_path=camera_prim_path, resolution=(CAMERA_WIDTH, CAMERA_HEIGHT))
camera.initialize()
camera.add_distance_to_image_plane_to_frame()
camera.add_rgb_to_frame()
for _ in range(10):
    world.step(render=True)

# ================= base_link 실측 (베이스는 이후 절대 안 움직이므로 한 번만 측정) =================
chassis_pos, chassis_quat = robot.get_world_pose()
base_pos_approx = np.array([chassis_pos[0], chassis_pos[1], chassis_pos[2] + MOUNT_Z])

xform_cache = UsdGeom.XformCache()
base_link_prim = stage.GetPrimAtPath(f"{m0609_path}/base_link")
base_mat = xform_cache.GetLocalToWorldTransform(base_link_prim)
base_pos = np.array(base_mat.ExtractTranslation())
base_quat_gf = base_mat.ExtractRotation().GetQuat()
base_quat = np.array([base_quat_gf.GetReal(), *base_quat_gf.GetImaginary()])
R_base = quat_wxyz_to_matrix(base_quat)

pos_diff = float(np.linalg.norm(base_pos - base_pos_approx))
print(
    f"[base_link 실측] pos={np.round(base_pos, 4)} quat={np.round(base_quat, 4)} "
    f"(근사값과의 차이={pos_diff:.5f}m)",
    flush=True,
)

# 크레이트 8개 코너를 base_link 좌표계로 변환 (스캔 없이, 우리가 만든 값이므로 계산만).
crate_vertices_base = [
    (R_base.T @ (np.array(v) - base_pos)).tolist() for v in crate_vertices_world
]

# ================= link6 <-> camera 상대 오프셋 측정 (32.py와 동일, 팔 자세와 무관한 고정 기구 관계) =================
link6_pos0, link6_quat0 = robot.end_effector.get_world_pose()
cam_pos0, cam_quat0 = camera.get_world_pose(camera_axes=CAMERA_AXES)
R_link6_0 = quats_to_rot_matrices(np.array([link6_quat0]))[0]
R_cam_0 = quats_to_rot_matrices(np.array([cam_quat0]))[0]
R_offset = R_link6_0.T @ R_cam_0
cam_local_pos_offset = R_link6_0.T @ (np.array(cam_pos0) - np.array(link6_pos0))


def lookat_to_link6_target(anchor_world, look_at, up=WORLD_UP):
    camera_eye = np.asarray(anchor_world, dtype=float)
    look_at = np.asarray(look_at, dtype=float)
    R_cam_target = make_usd_camera_rotation(camera_eye, look_at, up)
    R_link6_target = R_cam_target @ R_offset.T
    link6_target_pos = camera_eye - R_link6_target @ cam_local_pos_offset
    q_link6_target = rot_matrices_to_quats(np.array([R_link6_target]))[0]
    return link6_target_pos, q_link6_target


OPTICAL_TO_USD_CAMERA_AXES = np.diag([1.0, -1.0, -1.0])


def measure_base_link():
    """base_link의 실측 world pos/quat/회전행렬을 다시 잰다 - 섀시를 옮긴 뒤에는
    반드시 다시 불러야 한다(안 그러면 이전 위치 기준 좌표를 계속 쓰게 됨)."""
    xc = UsdGeom.XformCache()
    mat = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(f"{m0609_path}/base_link"))
    pos = np.array(mat.ExtractTranslation())
    quat_gf = mat.ExtractRotation().GetQuat()
    quat = np.array([quat_gf.GetReal(), *quat_gf.GetImaginary()])
    return pos, quat, quat_wxyz_to_matrix(quat)


def reposition_chassis(controller, xy, label):
    """섀시를 회전 없이 xy로만 옮기고(고정된 FACE_ROT_Z 유지) RMPflow의 base pose와
    base_link 실측값을 갱신한다. 크레이트 스캔 전용 위치로 옮겼다가 테이블 스캔 때
    쓰던 원래 위치로 되돌아오는 데 재사용한다."""
    chassis_pos_before, chassis_quat_before = robot.get_world_pose()
    chassis_target_pos = np.array([xy[0], xy[1], float(chassis_pos_before[2])])
    robot.set_world_pose(position=chassis_target_pos, orientation=chassis_quat_before)
    robot.set_linear_velocity(np.zeros(3))
    robot.set_angular_velocity(np.zeros(3))
    for _ in range(30):
        world.step(render=True)
    pos, quat, R = measure_base_link()
    controller._default_position = pos
    controller._default_orientation = quat
    controller.rmp_flow.set_robot_base_pose(robot_position=pos, robot_orientation=quat)
    print(f"[리포지셔닝] {label}: chassis -> {np.round(chassis_target_pos, 3)}, base_link={np.round(pos, 3)}", flush=True)
    return pos, quat, R


def converge_to_pose(controller, eye, look_at, label, cur_base_pos, cur_base_quat, cur_R_base):
    target_pos, target_quat = lookat_to_link6_target(eye, look_at)
    print(f"[{label} 목표] link6_pos={np.round(target_pos, 3)} eye={np.round(eye, 3)} look_at={np.round(look_at, 3)}", flush=True)
    for _ in range(BASIC_STEPS):
        actions = controller.forward(target_end_effector_position=target_pos, target_end_effector_orientation=target_quat)
        robot.apply_action(actions)
        world.step(render=True)

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
    to_target_dir = _normalize(np.asarray(look_at) - np.asarray(cam_pos_final))
    alignment = float(np.dot(forward_final, to_target_dir))
    print(
        f"[{label} 도달] ee_pos={np.round(ee_pos, 3)} err={err:.4f}m "
        f"cam_pos={np.round(cam_pos_final, 3)} alignment={alignment:.3f}",
        flush=True,
    )
    if err > 0.05:
        print(f"[경고] {label} IK 수렴 오차가 5cm를 넘습니다.", flush=True)

    R_base_to_cam = cur_R_base.T @ R_cam_final @ OPTICAL_TO_USD_CAMERA_AXES
    t_base_to_cam = cur_R_base.T @ (np.array(cam_pos_final) - cur_base_pos)

    PERCEPTION_DIR.mkdir(parents=True, exist_ok=True)
    transform_path = PERCEPTION_DIR / "base_to_camera_transform.json"
    transform_payload = {
        "R": R_base_to_cam.tolist(),
        "t": t_base_to_cam.tolist(),
        "note": f"35.crate_scan_setup.py의 '{label}' 스캔 자세 전용. 팔이 이 자세를 벗어나면 무효.",
        "measured_base_pos": cur_base_pos.tolist(),
        "measured_base_quat": cur_base_quat.tolist(),
        "measured_camera_pos": np.asarray(cam_pos_final).tolist(),
        "measured_camera_quat": np.asarray(cam_quat_final).tolist(),
        "ik_convergence_error_m": float(err),
        "camera_alignment": float(alignment),
    }
    transform_path.write_text(json.dumps(transform_payload, indent=2))
    print(f"[저장] {transform_path} ({label})", flush=True)
    return err, alignment


# ================= RMPflow 컨트롤러 =================
controller = RMPFlowController(
    name="crate_scan_controller",
    robot_articulation=robot,
    urdf_path=M0609_URDF_PATH,
    robot_description_path=M0609_DESCRIPTION_PATH,
    rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
    end_effector_frame_name=EE_LINK_NAME,
)
controller._default_position = base_pos
controller._default_orientation = base_quat
controller.rmp_flow.set_robot_base_pose(robot_position=base_pos, robot_orientation=base_quat)

# ================= ROS2 카메라 브리지 (포즈와 무관, 한 번만 연결) =================
setup_ros2_camera_bridge(camera_prim_path)

# ================= 스캔 자세 1: 테이블 =================
converge_to_pose(controller, SCAN_EYE, SCAN_LOOK_AT, "테이블 스캔", base_pos, base_quat, R_base)

SAVE_DIRECTORY = Path.home() / "box_pointcloud"
existing_jsons_before_table = set(SAVE_DIRECTORY.glob("all_boxes_corners_*.json")) if SAVE_DIRECTORY.exists() else set()

viewport = vp_util.get_active_viewport()
set_camera_view(
    eye=[ROBOT_START_XY[0] - 1.0, ROBOT_START_XY[1] - 1.3, TABLE_TOP_Z + 1.0],
    target=[CART_POS[0], CART_POS[1], TABLE_TOP_Z],
)
for _ in range(20):
    world.step(render=True)
vp_util.capture_viewport_to_file(viewport, str(_THIS_DIR / "_verify_crate_scan_table_view.png"))
print(f"[SCREENSHOT] _verify_crate_scan_table_view.png", flush=True)

print("\n[대기] 지금 별도 터미널에서 다음을 실행하세요 (venv/ROS2 환경 설정 포함):\n"
      f"  source {PERCEPTION_DIR / '.venv/bin/activate'}\n"
      f"  source /opt/ros/humble/setup.bash\n"
      f"  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n"
      f"  cd {PERCEPTION_DIR}\n"
      f"  DISPLAY=:1 python3 run_scan_once.py --marker {TABLE_SCAN_MARKER}\n", flush=True)
hold_until_marker(world, TABLE_SCAN_MARKER, "테이블 스캔")

table_jsons_after = set(SAVE_DIRECTORY.glob("all_boxes_corners_*.json"))
new_table_jsons = table_jsons_after - existing_jsons_before_table
if not new_table_jsons:
    raise RuntimeError("테이블 스캔 후 새 all_boxes_corners_*.json이 없습니다.")
table_boxes_json_path = max(new_table_jsons, key=lambda p: p.stat().st_mtime)
print(f"[테이블 스캔 결과] {table_boxes_json_path}", flush=True)

# 14_run_full_pipeline.py는 --boxes로 받은 모든 항목을 "배치할 박스"로 취급하고
# support_type을 전혀 걸러내지 않는다 - 병합/오탐(빈 테이블 면 전체를 하나로 묶은
# 거대 후보 등)이 실제 박스보다 커서 decide_loading_order(부피 큰 순)에서 먼저
# 배치되면, 진짜 목표가 자리를 뺏겨 NO_VALID_CANDIDATE_POSITION으로 튕겨나가는 걸
# 실제로 겪었다 - 그래서 필터링된 사본을 미리 저장해둔다.
#
# 처음엔 support_type=="box_top"만 채택했다(오탐=floor라는 가정) - 그런데
# box_top_extractor.py 코드를 직접 보면 support_type은 "이 박스 아래 받침을 어떻게
# 찾았나"를 나타낼 뿐이다: 다른 검출 후보가 바로 밑에서 매칭되면 "box_top", 못
# 찾아서 별도 바닥 RANSAC으로 폴백하면 "floor" - 테이블에 그냥 놓인(안 쌓인) 박스는
# 원래 "floor"가 정상이고, "box_top"은 오히려 "밑에 우연히 평평한 빈 테이블 조각이
# 후보로 잡혀서 매칭된" 우연에 가깝다. 실측 확인: Large(카메라에서 가장 먼 박스)는
# 매 스캔마다 예외 없이 정확하게(footprint 0.247~0.249x0.348~0.349, 실제
# 0.35x0.25와 근접) 검출되지만 항상 support_type=="floor"라 이 필터가 매번
# 버리고 있었다 - "floor=오탐"은 옛날에 겪은 특정 사례(테이블 전체를 뒤덮은 거대
# 병합 후보가 우연히 floor였던 것)를 일반화한 잘못된 가정이었다.
#
# support_type 대신, 실제 문제였던 "터무니없이 큰/작은 병합·조각 오탐"을 직접
# 걸러내는 크기 기반 필터로 교체한다 - 이 데모 테이블 위 실제 박스는 TABLE_BOXES에
# 정의된 3종류뿐이므로, 후보의 footprint(정렬된 w,d)가 그중 하나와
# TABLE_BOX_SIZE_TOLERANCE_M 이내로 맞는지 확인한다(크레이트 더미 박스를 크기로
# 걸렀던 것과 같은 패턴).
TABLE_BOX_SIZE_TOLERANCE_M = 0.05
_known_table_footprints = [tuple(sorted((w, d))) for _, (w, d, _h), _color, _off in TABLE_BOXES]


def _matches_known_table_box(box):
    xs = [c[0] for c in box["corners_m"]]
    ys = [c[1] for c in box["corners_m"]]
    footprint = tuple(sorted((max(xs) - min(xs), max(ys) - min(ys))))
    return any(
        abs(footprint[0] - known[0]) <= TABLE_BOX_SIZE_TOLERANCE_M
        and abs(footprint[1] - known[1]) <= TABLE_BOX_SIZE_TOLERANCE_M
        for known in _known_table_footprints
    )


def _table_box_center_xy(box):
    xs = [c[0] for c in box["corners_m"]]
    ys = [c[1] for c in box["corners_m"]]
    return (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0


table_boxes_data = json.loads(table_boxes_json_path.read_text())
_table_total = len(table_boxes_data["boxes"])
_size_ok_table_boxes = [b for b in table_boxes_data["boxes"] if _matches_known_table_box(b)]

# 크기 필터를 통과해도 같은 물리적 박스가 후보 2개(support_type이 다르거나 경계가
# 살짝 다른)로 겹쳐 잡히는 경우가 있을 수 있다(크레이트 더미에서 실제로 겪은 것과
# 같은 종류) - 중심이 가까우면 하나만(먼저 만난 것) 채택한다.
TABLE_BOX_DEDUP_RADIUS_M = 0.10
table_real_boxes = []
_kept_centers = []
for b in _size_ok_table_boxes:
    cx, cy = _table_box_center_xy(b)
    if any(((cx - kx) ** 2 + (cy - ky) ** 2) ** 0.5 < TABLE_BOX_DEDUP_RADIUS_M for kx, ky in _kept_centers):
        continue
    table_real_boxes.append(b)
    _kept_centers.append((cx, cy))

table_boxes_data["boxes"] = table_real_boxes
table_boxes_data["box_count"] = len(table_real_boxes)
table_boxes_filtered_path = RESULTS_DIR / "table_boxes_filtered.json"
RESULTS_DIR.mkdir(parents=True, exist_ok=True)
table_boxes_filtered_path.write_text(json.dumps(table_boxes_data, indent=2))
print(f"[저장] {table_boxes_filtered_path} (알려진 박스 크기 매칭 {len(_size_ok_table_boxes)}/{_table_total}개, "
      f"중복 제거 후 {len(table_real_boxes)}개 유지)", flush=True)

# ================= 스캔 자세 2: 크레이트 (포인트클라우드 기반 점유영역 검출) =================
# box_top_extractor.py(RANSAC 개별 박스 검출)는 애초에 "테이블 위에 흩어진 낱개 박스들을
# 찾아라"는 문제를 풀도록 만들어졌지, "빈 컨테이너 안에 뭐가 얼마나 차 있나"를 묻는 문제용이
# 아니다 - 그래서 크레이트에 그대로 쓰면 더미 박스를 "빈 크레이트라는 큰 박스 안에 든 작은
# 박스"처럼 잘못 엮어 인식하는 사례가 나왔다. 실제 트렁크(13.export_trunk_map.py)를 스캔할
# 때도 같은 이유로 박스 검출이 아니라 포인트클라우드를 그리드로 나눠 바닥보다 솟은 곳
# (occupied region)을 찾는 방식을 썼다 - 크레이트도 결국 "컨테이너 내부에서 바닥 기준 돌출부
# 찾기"라는 같은 문제이므로 그 방식을 그대로 가져온다. 크레이트는 우리가 만든 도형이라 벽/바닥
# 경계(x_min/x_max/y_min/y_max/floor_z)를 스캔으로 추정할 필요가 없다(13.py는 실제 트렁크라
# percentile로 추정해야 했음) - 알려진 상수를 그대로 aabb로 쓴다.
#
# get_pointcloud()는 카메라 자체 렌더 파이프라인(camera.add_distance_to_image_plane_to_frame()로
# 이미 켜둔 depth 애노테이터)에서 바로 world 좌표 포인트클라우드를 뽑는 Isaac Sim 네이티브
# API라, box_top_extractor.py처럼 별도 프로세스(run_scan_once.py)/ROS2/마커 파일 핸드셰이크가
# 전혀 필요 없다 - 같은 프로세스 안에서 동기 호출로 끝난다(테이블 스캔은 낱개 박스 식별이 진짜
# 필요하므로 box_top_extractor.py/마커 방식을 그대로 유지한다 - 위 테이블 스캔 섹션 참고).
crate_base_pos, crate_base_quat, crate_R_base = reposition_chassis(controller, CRATE_ROBOT_XY, "크레이트 스캔용")
converge_to_pose(controller, CRATE_SCAN_EYE, CRATE_SCAN_LOOK_AT, "크레이트 스캔", crate_base_pos, crate_base_quat, crate_R_base)

set_camera_view(
    eye=[CRATE_CENTER_XY[0] - 1.0, CRATE_CENTER_XY[1] - 1.0, CRATE_FLOOR_TOP_Z + 1.0],
    target=[CRATE_CENTER_XY[0], CRATE_CENTER_XY[1], CRATE_FLOOR_TOP_Z],
)
for _ in range(20):
    world.step(render=True)
vp_util.capture_viewport_to_file(viewport, str(_THIS_DIR / "_verify_crate_scan_crate_view.png"))
print(f"[SCREENSHOT] _verify_crate_scan_crate_view.png", flush=True)

for _ in range(20):
    world.step(render=True)
pts_world = np.asarray(camera.get_pointcloud(world_frame=True))
print(f"[크레이트 스캔] 포인트클라우드 {len(pts_world)}개 포인트 획득", flush=True)

# ================= trunk_map.json 조립 (크레이트 vertices 하드코딩 + 더미 박스 obstacles) =================
CRATE_OCCUPIED_CELL = 0.03           # grid cell 크기 (m) - 13.export_trunk_map.py의 OCCUPIED_CELL과 동일값
CRATE_OCCUPIED_EDGE_MARGIN = 0.03    # 벽 근처 cell 제외 (크레이트가 작아서 13.py의 0.05보다 살짝 좁힘)
CRATE_OCCUPIED_BUMP_THRESHOLD = 0.03  # local floor 기준값보다 이 높이(m) 이상 솟은 cell만 돌출부 후보
CRATE_OCCUPIED_MIN_CELLS = 6         # 덩어리로 인정할 최소 cell 개수 (노이즈 방지)
CRATE_OCCUPIED_MAX_AREA_FRACTION = 0.25   # 덩어리 하나가 interior의 이 비율을 넘으면 바닥 오검출로 보고 제외
CRATE_OCCUPIED_MAX_Y_SPAN_FRACTION = 0.5  # 덩어리가 y폭 전체의 이 비율을 넘게 가로지르면 제외
CRATE_OCCUPIED_MAX_HEIGHT_ABOVE_FLOOR = 0.35  # 이보다 높은 점(카메라/팔 자체 등)은 애초에 후보에서 제외


def detect_crate_obstacles_from_pointcloud(pts_world):
    """13.export_trunk_map.py의 detect_occupied_regions()와 같은 그리드-돌출(occupied-region)
    방식. 실제 트렁크와 달리 크레이트는 우리가 만든 도형이라 x/y/floor 경계를 percentile로 추정할
    필요 없이 CRATE_CENTER_XY/CRATE_INNER_SIZE/CRATE_FLOOR_TOP_Z를 그대로 aabb로 쓴다."""
    from scipy import ndimage

    x_min = CRATE_CENTER_XY[0] - CRATE_INNER_SIZE[0] / 2.0
    x_max = CRATE_CENTER_XY[0] + CRATE_INNER_SIZE[0] / 2.0
    y_min = CRATE_CENTER_XY[1] - CRATE_INNER_SIZE[1] / 2.0
    y_max = CRATE_CENTER_XY[1] + CRATE_INNER_SIZE[1] / 2.0
    floor_z = CRATE_FLOOR_TOP_Z

    inside = (
        (pts_world[:, 0] >= x_min) & (pts_world[:, 0] <= x_max)
        & (pts_world[:, 1] >= y_min) & (pts_world[:, 1] <= y_max)
        & (pts_world[:, 2] >= floor_z - 0.02)
        & (pts_world[:, 2] < floor_z + CRATE_OCCUPIED_MAX_HEIGHT_ABOVE_FLOOR)
    )
    band_pts = pts_world[inside]
    print(f"[점유영역] 크레이트 내부/높이 범위로 자른 후 후보점: {len(pts_world)} -> {len(band_pts)}", flush=True)

    nx = max(1, int((x_max - x_min) / CRATE_OCCUPIED_CELL))
    ny = max(1, int((y_max - y_min) / CRATE_OCCUPIED_CELL))
    x_edges = np.linspace(x_min, x_max, nx + 1)
    y_edges = np.linspace(y_min, y_max, ny + 1)

    ix = np.clip(np.digitize(band_pts[:, 0], x_edges) - 1, 0, nx - 1)
    iy = np.clip(np.digitize(band_pts[:, 1], y_edges) - 1, 0, ny - 1)

    local_min_z = np.full((nx, ny), np.nan)
    for cx in range(nx):
        for cy in range(ny):
            sel = band_pts[(ix == cx) & (iy == cy), 2]
            if len(sel) >= 3:
                local_min_z[cx, cy] = np.percentile(sel, 5)

    edge_margin_cells_x = max(1, int(CRATE_OCCUPIED_EDGE_MARGIN / CRATE_OCCUPIED_CELL))
    edge_margin_cells_y = max(1, int(CRATE_OCCUPIED_EDGE_MARGIN / CRATE_OCCUPIED_CELL))
    interior = np.zeros_like(local_min_z, dtype=bool)
    interior[edge_margin_cells_x:nx - edge_margin_cells_x, edge_margin_cells_y:ny - edge_margin_cells_y] = True
    interior &= ~np.isnan(local_min_z)
    n_interior = int(np.count_nonzero(interior))
    if n_interior == 0:
        print("[점유영역] interior에 유효 데이터 없음 -> 검출 안 함", flush=True)
        return []

    floor_ref = float(np.median(local_min_z[interior]))
    bump_mask = interior & (local_min_z > (floor_ref + CRATE_OCCUPIED_BUMP_THRESHOLD))
    print(f"[점유영역] floor_ref(local median)={floor_ref:.3f} (설계값 floor_z={floor_z:.3f}) "
          f"bump 후보 {int(np.count_nonzero(bump_mask))}/{n_interior} cell", flush=True)

    labeled, n_blobs = ndimage.label(bump_mask, structure=np.ones((3, 3)))
    y_span_total = y_max - y_min
    blobs = []
    for blob_id in range(1, n_blobs + 1):
        blob = labeled == blob_id
        n_cells = int(np.count_nonzero(blob))
        area_fraction = n_cells / n_interior
        if n_cells < CRATE_OCCUPIED_MIN_CELLS:
            continue
        if area_fraction > CRATE_OCCUPIED_MAX_AREA_FRACTION:
            print(f"[점유영역] 덩어리(cell {n_cells}개)가 interior의 {area_fraction:.0%} 차지 "
                  f"(> {CRATE_OCCUPIED_MAX_AREA_FRACTION:.0%}) - 바닥 전체 오검출 가능성 -> 제외", flush=True)
            continue

        cx_idx, cy_idx = np.nonzero(blob)
        bx_min, bx_max = x_edges[cx_idx.min()], x_edges[cx_idx.max() + 1]
        by_min, by_max = y_edges[cy_idx.min()], y_edges[cy_idx.max() + 1]
        if (by_max - by_min) > CRATE_OCCUPIED_MAX_Y_SPAN_FRACTION * y_span_total:
            print(f"[점유영역] 덩어리(cell {n_cells}개, y=[{by_min:.3f},{by_max:.3f}])가 폭 전체의 "
                  f"{(by_max - by_min) / y_span_total:.0%} 가로질러 제외 (문턱/선반 등으로 추정)", flush=True)
            continue

        top_z = float(np.nanpercentile(local_min_z[blob], 90))
        blobs.append({"x_min": float(bx_min), "x_max": float(bx_max),
                      "y_min": float(by_min), "y_max": float(by_max),
                      "z_min": floor_z, "z_max": top_z, "n_cells": n_cells})

    blobs.sort(key=lambda b: -b["n_cells"])
    if not blobs:
        print("[점유영역] 조건을 만족하는 덩어리 없음 -> 검출 안 함", flush=True)
    for i, b in enumerate(blobs, start=1):
        print(f"[점유영역] dummy_box_{i}: x=[{b['x_min']:.3f},{b['x_max']:.3f}] "
              f"y=[{b['y_min']:.3f},{b['y_max']:.3f}] z=[{b['z_min']:.3f},{b['z_max']:.3f}] "
              f"(cell {b['n_cells']}개)", flush=True)
    return blobs


crate_blobs = detect_crate_obstacles_from_pointcloud(pts_world)


def _aabb_world_to_base_corners(b):
    """world AABB(x_min/x_max/y_min/y_max/z_min/z_max) -> 8코너 -> 원래(테이블쪽) base_link
    좌표. crate_vertices_base(위 655-658행)와 정확히 같은 world->base 변환 - get_pointcloud()가
    이미 world 좌표를 주므로, 예전처럼 임시(크레이트쪽) base_link 프레임을 거쳐 재투영할 필요가
    없다(그 프레임 자체가 애초에 box_top_extractor.py의 base_link-상대 corners_m 출력 때문에
    필요했던 것)."""
    corners_world = [
        (x, y, z)
        for x in (b["x_min"], b["x_max"])
        for y in (b["y_min"], b["y_max"])
        for z in (b["z_min"], b["z_max"])
    ]
    return [(R_base.T @ (np.array(c) - base_pos)).tolist() for c in corners_world]


obstacles = [
    {"name": f"dummy_box_{i}", "vertices": _aabb_world_to_base_corners(b)}
    for i, b in enumerate(crate_blobs, start=1)
]
print(f"[장애물] 크레이트 포인트클라우드에서 {len(obstacles)}개 점유영역을 장애물로 채택", flush=True)

trunk_map = {
    "frame": "m0609_base_link (crate vertices는 하드코딩, obstacles는 실측 스캔)",
    "vertices": crate_vertices_base,
    "edges": [
        {"v": [0, 1]}, {"v": [1, 2]}, {"v": [2, 3]}, {"v": [3, 0]},
        {"v": [4, 5]}, {"v": [5, 6]}, {"v": [6, 7]}, {"v": [7, 4]},
        {"v": [0, 4]}, {"v": [1, 5]}, {"v": [2, 6]}, {"v": [3, 7]},
    ],
    "obstacles": obstacles,
}
RESULTS_DIR.mkdir(parents=True, exist_ok=True)
trunk_map_path = RESULTS_DIR / "trunk_map.json"
trunk_map_path.write_text(json.dumps(trunk_map, indent=2))
print(f"[저장] {trunk_map_path}", flush=True)
print(f"[저장] 테이블 박스 JSON 경로 참고용: {table_boxes_json_path}", flush=True)

# ================= 섀시를 원래(테이블쪽) 위치로 복귀 =================
# 36.py는 이 씬을 열었을 때 섀시가 원래 자리(ROBOT_START_XY)에 있다고 가정하고 PICK을
# 시작한다 - 크레이트 스캔용으로 옮겼던 걸 저장 전에 되돌린다.
reposition_chassis(controller, ROBOT_START_XY, "테이블쪽 원위치 복귀")

# ================= 씬 저장 =================
scene_path = str(_THIS_DIR / "crate_scan_scene.usd")
omni.usd.get_context().save_as_stage(scene_path)
print(f"[저장] {scene_path}", flush=True)

print("\n[완료] 35.crate_scan_setup.py 끝.\n", flush=True)

if HEADLESS:
    simulation_app.close()
else:
    print("[안내] 창을 직접 둘러보세요 - 닫으면 스크립트가 종료됩니다.\n", flush=True)
    while simulation_app.is_running():
        world.step(render=True)
    simulation_app.close()
