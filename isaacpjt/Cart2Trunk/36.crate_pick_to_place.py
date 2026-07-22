"""
36.crate_pick_to_place.py
테이블에서 박스들을 하나씩 집어서, 크레이트 안 적재 알고리즘이 계산한 자리에
내려놓는다 - 33.box_table_pick_to_trunk.py의 트렁크 버전을 대체.

33.py와 달라진 점
----
1. 차량/트렁크가 없다 - 35.crate_scan_setup.py가 만든 오픈-탑 크레이트(뚜껑 없음)가
   목표다. 그래서 33.py에 있던 "chassis_link <-> /World/Vehicle 충돌 임시 해제"
   (UsdPhysics.FilteredPairsAPI) 코드가 통째로 필요 없다 - 통과시킬 차체가 없다.
2. 크레이트 안에 더미 박스 2개가 미리 놓여 있고(35.py가 실제로 스캔해서
   obstacles로 등록), 적재 알고리즘이 그 사이 빈 자리를 실제로 찾아서 배치했다
   (results/crate_demo/placement_result.json).
3. 트렁크 문(천장) 높이 제한이 없다(오픈-탑이라 물리적으로 막는 게 없음) - 그래서
   PLACE가 33.py의 wp_entrance->wp_interior 2단 접근 없이 hover->하강->release
   단일 경로로 단순화된다(PICK과 대칭 구조).
4. 목표가 테이블 스캔 위치에서 팔만으로 안 닿을 만큼 멀어서, "회전 없는 순수 이동"
   (리포지셔닝)이 필요하다 - 갈 길에 장애물이 없으므로 충돌 해제 없이 정직하게 이동.

[대대적 개편 - 특정 박스 하드코딩 제거]
이전 버전은 "지금 데모가 어떤 박스를 옮기는가"(Small? Medium? Large?)가 바뀔
때마다 BOX_PICK_PRIM_PATH/GRASP_RADIUS/PLACE_WORLD_XY/PLACE_DIMENSIONS/
box_top_z의 절반 높이 근사값까지 사람이 손으로 다시 계산해서 넣어야 했다 -
Medium->Small->Large->Small로 바꿀 때마다 매번 이런 상수를 새로 박아 넣는 건
지속 가능하지 않다는 지적(사용자)에 따라, 이 스크립트는 이제:

  - results/crate_demo/table_boxes_filtered.json(박스 스캔 결과)과
    results/crate_demo/placement_result.json(적재 알고리즘이 실제로 배치에
    성공한 박스들, 이미 알고리즘이 결정한 순서)을 그대로 읽어서,
  - "적재에 성공한 모든 박스"를 알고리즘이 정한 순서 그대로 하나씩 집어서
    옮긴다 - 특정 박스 이름이나 개수를 코드에 박아두지 않는다.
  - 각 박스의 PICK/PLACE 좌표, grasp 판정 반경은 전부 스캔/배치 결과에서
    실행 시점에 계산한다(8코너 회전으로 base_link->world 축 스왑까지 자동 보정).
  - placement_result.json의 box_id가 어떤 물리 USD 프림(Box_Small/Medium/Large)인지는
    이름표가 아니라 "스캔된 테이블 위 위치와 가장 가까운 실측 위치의 프림"으로
    매칭한다 - 스캔이 매번 box_id를 카메라 거리순으로 새로 매기므로(Q4, 재스캔해도
    같은 id가 보장 안 됨) 이름 매칭보다 위치 매칭이 안전하다.

[물리 충돌 재현 - Large를 실제로 옮기려다 발견한 문제]
Large(테이블 위에서 로봇 기준 가장 먼 자리)를 집으러 팔이 쭉 뻗을 때, 그 경로가
더 가까운 Small/Medium을 스치는 물리 충돌이 실측으로 확인됐다(PLACE 단계
cushion/hover/release 오차가 0.32~0.35m로 튀고 크레이트 밖에 떨어짐 - RMPflow
발산처럼 보였지만 실제 원인은 팔이 다른 박스와 부딪힌 것으로 추정). 35.py에서
테이블을 낮추고(0.40->0.34) 박스 크기/서로간 거리를 0.65배로 줄여서 팔이 스치는
부피와 스윙 거리를 함께 줄였다 - 이 스크립트가 다루는 "여러 박스를 순서대로
옮기는" 시나리오 자체가 이 문제를 훨씬 자주 노출시키므로(모든 박스를 매번
왕복해서 집어야 함) 반드시 같이 검증해야 한다.

[동작 재설계 - 안전 운반 자세 + PLACE 순수 Z축 하강]
사용자가 GUI 실행(화면 녹화 포함)으로 직접 두 가지 문제를 잡아냄:
1. 흡착 직후 살짝만 든 상태(pick_hover 높이, 옆 박스들과 가까움)에서 곧장
   텔레포트하면 문제가 생긴다 - 텔레포트는 순간 이동이라 스치며 지나가는 구간이
   없고, 텔레포트 *직전*의 팔 자세가 곧 *직후*의 자세(섀시만 바뀐 채 관절 각도는
   유지됨)와 완전히 같다. 그래서 텔레포트 전에 항상: 박스와 함께 안전 높이까지
   완전히 들어올리고, 섀시 쪽으로 당겨서 컴팩트한 "안전 운반 자세"를 만든 뒤에만
   텔레포트한다(SAFE_TRANSIT_Z, CARRY_OFFSET_FROM_CHASSIS 참고).
2. PLACE 접근이 대각선(x/y/z 동시 변화)으로 목표까지 가면서 크레이트 안 기존
   더미 장애물과 부딪혀 "터졌다"(그립 폭발과 같은 종류의 PhysX 충돌 반응). PLACE는
   이제 안전 높이에서 목표 xy 위까지 순수 수평 이동만 먼저 하고, 그 다음부터는
   xy를 전혀 안 바꾸고 z만 내리는 순수 수직 하강으로 hover->release까지
   진행한다 - 장애물을 옆에서 스치는 경로 자체가 없어진다.
"""

from isaacsim import SimulationApp

import os

# 기본은 GUI로 직접 보이게 - 자동 검증용으로 헤드리스가 필요하면 HEADLESS=1로 실행:
#   HEADLESS=1 isaac_python 36.crate_pick_to_place.py
HEADLESS = os.environ.get("HEADLESS", "0") == "1"
# GUI로 실제 렌더링하면서 ~2000 스텝을 다 돌리면 실측 234초(헤드리스는 20~30초) - 화면
# 해상도를 낮춰서 렌더링 부하를 줄인다. 헤드리스 스크린샷 품질에는 영향 없음(기본 해상도 유지).
_sim_app_config = {"headless": HEADLESS}
if not HEADLESS:
    _sim_app_config.update({"width": 640, "height": 480})
simulation_app = SimulationApp(_sim_app_config)

import json
from pathlib import Path
import sys

import numpy as np
import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.core.prims import SingleRigidPrim

_THIS_DIR = Path(__file__).resolve().parent
M0609_DIR = _THIS_DIR.parent / "M0609"
RMPFLOW_DIR = str(M0609_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController  # noqa: E402

SCENE_USD = str(_THIS_DIR / "crate_scan_scene.usd")
RESULTS_DIR = _THIS_DIR / "results" / "crate_demo"
TABLE_BOXES_JSON = RESULTS_DIR / "table_boxes_filtered.json"
PLACEMENT_JSON = RESULTS_DIR / "placement_result.json"

M0609_URDF_PATH = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH = str(M0609_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(M0609_DIR / "rmpflow/m0609_rmpflow_common.yaml")

M0609_PATH = "/World/MobileManipulator/M0609"
CHASSIS_PATH = "/World/MobileManipulator/NovaCarter/chassis_link"
EE_LINK_NAME = "link_6"
GRIPPER_BODY_NAME = "vgp20"
MOUNT_Z = 0.42  # 35.py와 동일 - base_link 실측 시 chassis_pos에 더할 근사 오프셋
TIP_LOCAL_OFFSET = (0.0, 0.0, 0.121)

# DynamicSuctionGripper.close()의 판정 거리(dist)는 대략 "박스 반높이"로 수렴한다
# (석션 팁은 항상 박스 윗면에 오도록 설계했는데, 박스 중심 거리는 그 팁에서 반높이만큼
# 떨어져 있으므로). 예전엔 GRASP_RADIUS를 박스마다 손으로 다시 맞췄다(0.10이었다가
# Large의 반높이 0.11m를 못 넘어서 흡착이 30번 조용히 실패한 적이 있었음, 0.15로
# 올려서 해결) - 이제는 각 박스의 스캔된 실제 높이에서 half_height를 실행 시점에
# 계산하고, 여기에 여유만 더한다.
GRASP_RADIUS_MARGIN = 0.03

# pick_grasp_pos을 box_top_z + TIP_LOCAL_OFFSET[2]로(팁이 정확히 박스 표면에 닿는
# 지점) 잡으면 여유가 0이다 - 사용자가 화면 녹화(260722_1526)로 실제로 잡아냄:
# 이 목표로 하강할 때 팁(또는 그리퍼 몸체)이 박스 강체와 살짝 겹치는 프레임이
# 생기고, 그 겹침을 PhysX가 관절 드라이브의 강한 힘(DRIVE_MAX_FORCE=1e8)으로
# 순식간에 밀어내면서 박스가 폭발적으로 튕겨나갔다(Large가 테이블에서 크레이트
# 근처까지 한 프레임 새 날아감, 영상으로 확인). GRASP_RADIUS_MARGIN(0.03) 여유가
# 있으므로, 팁을 표면에 딱 붙이는 대신 살짝 띄워서 겹침 자체가 안 생기게 한다.
GRASP_STANDOFF = 0.01

DOWN_QUAT = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))
WAYPOINT_STEPS = 150

# 35.py와 동일 - 로봇 베이스는 테이블 스캔 때 이 자리에 있었고, 크레이트 PLACE
# 사이사이 매번 여기로 되돌아온다(테이블에서 다음 박스를 집어야 하므로).
ROBOT_START_XY = (0.0, -0.55)
# 리포지셔닝 목표: 35.py 스캔 때 이미 검증된 위치(충돌 없음 + IK 수렴 양호,
# alignment=0.996) 그대로 재사용한다. RMPflow는 특정 목표 좌표 자체에 예측 불가능
# 하게 민감한 것으로 실측 확인됐으므로(비율을 바꿔 역산해도 오히려 더 발산), 섀시는
# 안전이 검증된 위치에 고정한다.
CHASSIS_TARGET_XY = (0.9, -0.55)

PICK_HOVER_HEIGHT_ABOVE_BOX = 0.20
RELEASE_CLEARANCE_ABOVE_FLOOR = 0.02  # release 시점에 박스 바닥과 크레이트 바닥 사이 남길 여유

# place_hover -> place_release 하강(보통 0.15m)을 move_link6 한 번에 통째로 주면
# RMPflow가 갑자기 0.15m 떨어진 목표를 향해 큰 보정 동작을 만들어낼 수 있다 - 이
# 프로젝트에서 반복 확인된 "큰 점프 한 번보다 작은 점프 여러 번이 더 안정적으로
# 수렴한다" 패턴(PICK/PLACE의 SAFE_TRANSIT_Z 단계적 설계와 동일한 이유)을 마지막
# 하강 구간에도 그대로 적용한다. 특히 벽 두 개에 동시에 가까운 코너 자리(Small)는
# 여유가 좁아서, 큰 점프의 초기 오버슈트가 그대로 충돌로 이어지기 쉽다.
PLACE_DESCENT_SUBSTEPS = 3

# 실측으로 확인된 문제: 박스를 순서대로 여러 개 옮길 때, 한 박스(성공이든 실패든)를
# 다루고 난 뒤 곧장 다음 박스의 pick_hover로, 또는 텔레포트 직후 곧장 place_hover로
# (대각선으로) 점프시키면 RMPflow가 예측 불가능하게 발산하거나 기존 장애물과
# 충돌해서 "터진다"(사용자가 GUI 실행/화면 녹화로 직접 확인) - 모든 "박스를 들고
# 이동"하는 구간에서 XY 변경과 Z 변경을 분리해서, 항상 이 안전 높이를 거쳐가게
# 한다: 수직으로만 여기까지 오르내리고, 이 높이에서만 수평 이동한다. 관측된 hover
# 높이가 최대 약 0.68m였으므로 0.85m면 모든 박스/장애물 위를 확실히 지나간다
# (관련 근거: base_link 기준 상대 z=0.43m, 이 프로젝트에서 이미 여러 번 검증된 안전권).
SAFE_TRANSIT_Z = 0.85

# 텔레포트 직전에 항상 만드는 "안전 운반 자세" - 섀시 기준 상대 오프셋(dx, dy),
# SAFE_TRANSIT_Z 높이에서 적용한다. 텔레포트는 순간 이동이라 실제로 뭔가를 스치며
# 지나가는 구간이 없으므로, 텔레포트 *직전*의 팔 자세가 곧 텔레포트 *직후*의
# 자세(섀시만 바뀐 채 관절 각도는 그대로 유지됨)와 똑같다 - 살짝 든 낮고 옆으로
# 뻗은 자세 그대로 텔레포트하면 새 위치에서도 똑같이 위험한 자세로 도착한다
# (사용자 지적, GUI 실행으로 직접 확인). 텔레포트 전에 항상 박스를 몸 쪽으로
# 당겨서 컴팩트하게 만든다 - 이 오프셋은 PICK 접근에서 이미 반복적으로
# err<0.01m로 수렴이 검증된 값을 그대로 재사용한다. 섀시 기준 상대값으로 정의해서,
# 텔레포트 후(관절 각도 유지)에도 자동으로 "새 섀시 위치 기준 같은 상대 자세"가
# 되므로 추가 이동 없이 바로 다음 단계(PLACE 접근/PICK 접근)를 시작할 수 있다.
CARRY_OFFSET_FROM_CHASSIS = (0.22, 0.43)

# 35.py가 테이블에 실제로 만드는 물리 박스 3개(TABLE_BOXES) - 어떤 placement_result.json
# box_id가 이 중 어느 프림인지는 이름이 아니라 스캔 위치 매칭으로 실행 시점에 정한다
# (match_physical_prim 참고, Q4: 재스캔해도 box_id 지속성이 보장 안 됨).
CANDIDATE_BOX_PRIM_PATHS = ["/World/Box_Small", "/World/Box_Medium", "/World/Box_Large"]


def quat_wxyz_to_matrix(q) -> np.ndarray:
    """Isaac Sim 관례(w, x, y, z) 쿼터니언 -> 3x3 회전행렬. 35.py와 동일."""
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


def world_aabb_from_base_corners(corners_base, base_pos, R_base):
    """box_top_extractor.py 스타일 입력(base_link 좌표계의 명시적 8꼭짓점, 회전
    없이 그대로 옴)을 world AABB로 변환. -> (center_xyz, dims_xyz, min_xyz)"""
    pts_base = np.asarray(corners_base, dtype=np.float64)
    pts_world = pts_base @ R_base.T + base_pos
    mn = pts_world.min(axis=0)
    mx = pts_world.max(axis=0)
    return (mn + mx) / 2.0, mx - mn, mn


def world_aabb_from_base_pos_dims(pos_base, dims_base, base_pos, R_base):
    """placement_result.json 스타일 입력(base_link 좌표계의 min corner + 별도
    dims, 아직 축이 안 바뀐 상태)을 8코너 회전으로 world AABB로 변환 - base_link가
    world 기준 ~90도 돌아가 있어서 dims의 x/y가 world에서 서로 바뀌므로, 8개
    꼭짓점을 전부 개별 회전한 뒤 min/max를 다시 잡아야 한다(축 스왑 버그 재발 방지,
    Round 6/8에서 실측으로 확인된 필수 절차). -> (center_xyz, dims_xyz, min_xyz)"""
    pos_base = np.asarray(pos_base, dtype=np.float64)
    dims = np.asarray(dims_base, dtype=np.float64)
    corners_world = []
    for dx in (0.0, dims[0]):
        for dy in (0.0, dims[1]):
            for dz in (0.0, dims[2]):
                corners_world.append(R_base @ (pos_base + np.array([dx, dy, dz])) + base_pos)
    corners_world = np.array(corners_world)
    mn = corners_world.min(axis=0)
    mx = corners_world.max(axis=0)
    return (mn + mx) / 2.0, mx - mn, mn


class DynamicSuctionGripper(SurfaceGripper):
    """32/33.py의 DynamicSuctionGripper와 같은 흡착 로직 - target_prim_path/
    grasp_radius를 생성자에서 한 번 고정하는 대신 set_target()으로 박스마다
    바꿀 수 있게 한 것만 다르다(여러 박스를 순서대로 옮기려면 필요)."""

    def __init__(self, end_effector_prim_path, gripper_body_path,
                 tip_local_offset=(0.0, 0.0, 0.0)):
        SurfaceGripper.__init__(self, end_effector_prim_path=end_effector_prim_path, surface_gripper_path="")
        self._gripper_body_path = gripper_body_path
        self._tip_local_offset = Gf.Vec3d(*tip_local_offset)
        self._joint_path = f"{gripper_body_path}/suction_attach_joint"
        self._attached = False
        self._target_prim_path = None
        self._grasp_radius = 0.0

    def set_target(self, target_prim_path, grasp_radius):
        self._target_prim_path = target_prim_path
        self._grasp_radius = grasp_radius

    def close(self) -> None:
        if self._attached:
            return
        if self._target_prim_path is None:
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
            print(f"  [흡착 실패] dist={dist:.4f}m > grasp_radius={self._grasp_radius}m", flush=True)
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


def snapshot(world, viewport, eye, target, fname, robot=None, chassis_pin=None):
    set_camera_view(eye=eye, target=target)
    for _ in range(15):
        world.step(render=True)
        if chassis_pin is not None:
            robot.set_world_pose(position=chassis_pin[0], orientation=chassis_pin[1])
            robot.set_linear_velocity(np.zeros(3))
            robot.set_angular_velocity(np.zeros(3))
    out = str(_THIS_DIR / fname)
    vp_util.capture_viewport_to_file(viewport, out)
    # capture_viewport_to_file은 비동기(캡처 요청 후 몇 프레임 뒤에야 실제 파일이
    # 쓰인다) - 보통은 그 다음 snapshot() 호출의 앞쪽 스텝들이 우연히 이 flush를
    # 대신 채워줘서 문제가 안 보였는데, 스크립트에서 마지막으로 호출되는 snapshot
    # (다음 snapshot 호출이 없는 경우, 예: 전체 완료 후 마지막 전경 사진)은 그
    # 여유가 없어서 파일이 끝내 갱신되지 않고 예전 파일이 그대로 아카이빙되는
    # 버그를 실측으로 확인했다(mtime이 같은 실행의 다른 스크린샷보다 몇 분이나
    # 앞서 있었음). 모든 호출에 동일하게 넉넉한 flush 여유를 준다.
    for _ in range(30):
        world.step(render=True)
        if chassis_pin is not None:
            robot.set_world_pose(position=chassis_pin[0], orientation=chassis_pin[1])
            robot.set_linear_velocity(np.zeros(3))
            robot.set_angular_velocity(np.zeros(3))
    print(f"[SCREENSHOT] {out}", flush=True)


def move_link6(world, controller, robot, target_pos, steps=WAYPOINT_STEPS, hold_gripper_closed=False,
               label="", chassis_pin=None):
    """17/33.py와 동일. GUI로 실제 렌더링하면 스텝당 시간이 헤드리스보다 훨씬 길어서
    (실측 234초/2000스텝 vs 헤드리스 20~30초) 웨이포인트 하나가 몇십 초씩 걸릴 수
    있다 - 중간 진행 로그가 없으면 "멈췄나?" 싶어진다. 100스텝마다 한 번씩 찍는다.

    [섀시 드리프트 방지] 3박스를 순서대로 옮기다가, 리포지셔닝 직후엔 웨이포인트가
    거의 완벽하게 수렴하는데(err<0.02m) 그 직후 다음 웨이포인트부터 갑자기 크게
    발산하는 패턴이 반복 확인됐다. 원인을 실측으로 추적한 결과 - RMPflow 발산이
    아니라 섀시 자체가 팔의 반작용력으로 물리적으로 밀리고 있었다(팔 관절 드라이브가
    DRIVE_MAX_FORCE=1e8로 매우 강해서, 그 반작용이 안 눌린 섀시를 밀어낼 수 있다).
    1차로 매 스텝 속도만 0으로 누르는 시도를 했으나 부족했다(그래도 섀시가 최대
    0.27~0.29m 밀림, 실측 확인 - 힘이 매 프레임 짧게라도 가속시킨 뒤에야 속도가
    지워지니 위치 자체는 조금씩 계속 새어나갈 수 있다). 매 스텝 world/orientation을
    통째로 다시 못박는 게 유일하게 확실한 방법이라, chassis_pin=(position, quat)이
    주어지면 매 스텝 끝에 robot.set_world_pose()로 강제 재고정한다."""
    for i in range(steps):
        actions = controller.forward(
            target_end_effector_position=np.array(target_pos, dtype=float),
            target_end_effector_orientation=DOWN_QUAT,
        )
        robot.apply_action(actions)
        if hold_gripper_closed:
            robot.gripper.close()
        world.step(render=True)
        if chassis_pin is not None:
            robot.set_world_pose(position=chassis_pin[0], orientation=chassis_pin[1])
        robot.set_linear_velocity(np.zeros(3))
        robot.set_angular_velocity(np.zeros(3))
        if (i + 1) % 100 == 0:
            print(f"  [진행{' ' + label if label else ''}] {i + 1}/{steps} 스텝", flush=True)
    ee_pos, _ = robot.end_effector.get_world_pose()
    err = np.linalg.norm(np.array(ee_pos) - np.array(target_pos))
    print(f"[웨이포인트{' ' + label if label else ''}] target={np.round(target_pos, 3)} "
          f"ee={np.round(ee_pos, 3)} err={err:.4f}m", flush=True)
    return ee_pos, err


def make_controller(name, robot, base_pos, base_quat):
    controller = RMPFlowController(
        name=name,
        robot_articulation=robot,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=M0609_DESCRIPTION_PATH,
        rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
    )
    controller._default_position = base_pos
    controller._default_orientation = base_quat
    controller.rmp_flow.set_robot_base_pose(robot_position=base_pos, robot_orientation=base_quat)
    return controller


def reposition_chassis(world, robot, target_xy, label):
    """PICK<->PLACE 사이 섀시 이동 (회전 없음, 갈 길에 장애물이 없어 충돌 해제 불필요).
    텔레포트마다 RMPflowController를 새로 만들어야 한다 - set_robot_base_pose()만
    다시 불러서 기존 컨트롤러를 재사용하면 내부에 캐시된 장애물 모델이 예전 base
    pose 기준으로 남아서 발산한다는 게 이미 실측으로 확인됐다(Round 1)."""
    chassis_pos_before, chassis_quat_before = robot.get_world_pose()
    chassis_target_pos = np.array([target_xy[0], target_xy[1], float(chassis_pos_before[2])])
    pre_teleport_joint_positions = robot.get_joint_positions()

    robot.set_world_pose(position=chassis_target_pos, orientation=chassis_quat_before)
    # 텔레포트 직후 관절 각도가 물리 솔버에 의해 미세하게 흔들려서 결국 완전히 다른
    # 자세로 "settle"되는 현상을 실측으로 확인했다 - 텔레포트 직전 관절 각도를 그대로
    # 다시 못박아서 물리 상태와 관절 각도가 어긋나지 않게 한다.
    robot.set_joint_positions(pre_teleport_joint_positions)
    robot.set_linear_velocity(np.zeros(3))
    robot.set_angular_velocity(np.zeros(3))
    for _ in range(30):
        world.step(render=True)
        # move_link6()의 chassis_pin과 같은 이유 - 속도만 누르는 것으로는 부족해서
        # (팔 관절 정착 중 반작용으로 여전히 밀릴 수 있음) 위치/자세를 통째로 재고정한다.
        robot.set_world_pose(position=chassis_target_pos, orientation=chassis_quat_before)
        robot.set_linear_velocity(np.zeros(3))
        robot.set_angular_velocity(np.zeros(3))

    new_base_pos = chassis_target_pos + np.array([0.0, 0.0, MOUNT_Z])
    new_controller = make_controller(f"crate_pick_to_place_{label}", robot, new_base_pos, chassis_quat_before)
    print(f"[리포지셔닝:{label}] chassis {np.round(chassis_pos_before, 3)} -> "
          f"{np.round(chassis_target_pos, 3)}", flush=True)
    return new_controller, chassis_target_pos


def get_world_pos(prim):
    """XformCache 캐시 문제(33.py에서 실제로 겪음) 회피 - 매번 직접 계산."""
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array(mat.ExtractTranslation())


def match_physical_prim(stage, scan_center_world_xy, available_paths):
    """placement_result.json의 box_id가 참조하는(table_boxes_filtered.json의)
    스캔된 테이블 위 world 위치와 가장 가까운 실제 USD 박스 프림을 찾는다 - 이름
    (Small/Medium/Large) 매칭이 아니라 위치 매칭이라, 스캔이 매번 box_id를 카메라
    거리순으로 새로 매기는 것과 무관하게 안전하다."""
    best_path, best_dist = None, None
    for path in available_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        pos = get_world_pos(prim)
        dist = float(np.linalg.norm(pos[:2] - np.asarray(scan_center_world_xy)))
        if best_dist is None or dist < best_dist:
            best_path, best_dist = path, dist
    return best_path, best_dist


# ================= 결과 JSON 로드 =================
table_data = json.loads(TABLE_BOXES_JSON.read_text())
placement_data = json.loads(PLACEMENT_JSON.read_text())
scan_by_box_id = {str(b["box_id"]): b for b in table_data["boxes"]}
placements = placement_data["placements"]
print(f"[로드] {TABLE_BOXES_JSON.name}: 스캔된 박스 {len(table_data['boxes'])}개, "
      f"{PLACEMENT_JSON.name}: 배치 성공 {len(placements)}개(미적재 {len(placement_data.get('unloadable', []))}개)",
      flush=True)

# ================= 씬 열기 =================
omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    simulation_app.update()

world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

gripper = DynamicSuctionGripper(
    end_effector_prim_path=f"{M0609_PATH}/{EE_LINK_NAME}",
    gripper_body_path=f"{M0609_PATH}/{GRIPPER_BODY_NAME}",
    tip_local_offset=TIP_LOCAL_OFFSET,
)
robot = SingleManipulator(
    prim_path=CHASSIS_PATH,
    end_effector_prim_path=f"{M0609_PATH}/{EE_LINK_NAME}",
    name="mobile_manipulator",
    gripper=gripper,
)
world.reset()
robot.initialize(physics_sim_view=world.physics_sim_view)
for _ in range(20):
    world.step(render=True)

# ================= base_link 실측 (35.py와 동일 방식 - 하드코딩 대신 씬 로드
# 직후 직접 측정. 씬을 재생성해도 값이 저절로 최신으로 유지된다) =================
chassis_pos, chassis_quat = robot.get_world_pose()
xform_cache = UsdGeom.XformCache()
base_link_prim = stage.GetPrimAtPath(f"{M0609_PATH}/base_link")
base_mat = xform_cache.GetLocalToWorldTransform(base_link_prim)
BASE_POS = np.array(base_mat.ExtractTranslation())
base_quat_gf = base_mat.ExtractRotation().GetQuat()
BASE_QUAT = np.array([base_quat_gf.GetReal(), *base_quat_gf.GetImaginary()])
R_BASE = quat_wxyz_to_matrix(BASE_QUAT)
print(f"[base_link 실측] pos={np.round(BASE_POS, 4)} quat={np.round(BASE_QUAT, 4)}", flush=True)

# move_link6()에 매 스텝 넘겨서 섀시를 강제 재고정하는 데 쓰는 (position, quat) -
# reposition_chassis()는 회전 없이 순수 이동만 하므로 quat은 스크립트 내내 그대로,
# position만 리포지셔닝마다 갱신한다(아래에서 chassis_pin = (target_pos, chassis_quat)로 갱신).
chassis_pin = (chassis_pos, chassis_quat)

controller = make_controller("crate_pick_to_place_table", robot, BASE_POS, BASE_QUAT)

viewport = vp_util.get_active_viewport()
snapshot(
    world, viewport,
    eye=[BASE_POS[0] - 1.0, BASE_POS[1] - 1.3, 1.4],
    target=[0.0, 0.0, 0.68],
    fname="_crate_00_start.png",
    robot=robot, chassis_pin=chassis_pin,
)

# ================= 0. 목표 위치 미리보기 (pick 전, 비교/설명용 - 배치될 박스 전부) =================
preview_prims = []
for i, placement in enumerate(placements):
    center, dims, mn = world_aabb_from_base_pos_dims(
        placement["position_base_frame"], placement["dimensions"], BASE_POS, R_BASE)
    prim = VisualCuboid(
        prim_path=f"/World/PlaceTargetPreview_{i}",
        name=f"place_target_preview_{i}",
        position=np.array([center[0], center[1], mn[2] + dims[2] / 2.0]),
        scale=np.array(dims),
        color=np.array([1.0, 0.0, 1.0]),  # 마젠타 - 실제 박스와 명확히 구분되는 미리보기 색
    )
    preview_prims.append(prim)
for _ in range(10):
    world.step(render=True)
    robot.set_world_pose(position=chassis_pin[0], orientation=chassis_pin[1])
    robot.set_linear_velocity(np.zeros(3))
    robot.set_angular_velocity(np.zeros(3))
snapshot(
    world, viewport,
    eye=[CHASSIS_TARGET_XY[0] - 1.0, CHASSIS_TARGET_XY[1] - 1.3, 1.5],
    target=[CHASSIS_TARGET_XY[0], CHASSIS_TARGET_XY[1], BASE_POS[2] - MOUNT_Z + 0.15],
    fname="_crate_00b_target_preview.png",
    robot=robot, chassis_pin=chassis_pin,
)
for p in preview_prims:
    p.set_visibility(False)
print(f"[미리보기] 적재 알고리즘 목표 위치/크기 마커 {len(preview_prims)}개 저장 완료, 이후 숨김 처리", flush=True)

# ================= 박스마다 PICK -> 리포지셔닝 -> PLACE -> (다음 박스면) 리포지셔닝 =================
used_prim_paths = set()
print(f"\n[계획] 적재 알고리즘이 배치에 성공한 박스 {len(placements)}개를 순서대로 집어서 옮긴다", flush=True)

for idx, placement in enumerate(placements):
    box_id = str(placement["box_id"])
    scan_entry = scan_by_box_id.get(box_id)
    if scan_entry is None:
        print(f"[경고] box_id={box_id}가 테이블 스캔 결과에 없음 - 건너뜀", flush=True)
        continue

    scan_center, scan_dims, _ = world_aabb_from_base_corners(scan_entry["corners_m"], BASE_POS, R_BASE)
    available = [p for p in CANDIDATE_BOX_PRIM_PATHS if p not in used_prim_paths]
    prim_path, match_dist = match_physical_prim(stage, scan_center[:2], available)
    if prim_path is None:
        print(f"[경고] box_id={box_id}에 매칭되는 물리 프림을 못 찾음 - 건너뜀", flush=True)
        continue
    used_prim_paths.add(prim_path)
    box_prim = stage.GetPrimAtPath(prim_path)
    print(f"\n===== [{idx + 1}/{len(placements)}] box_id={box_id} -> {prim_path} "
          f"(스캔 위치와의 실측 거리={match_dist:.3f}m) =====", flush=True)

    # ---- 1. PICK ----
    box_pos = get_world_pos(box_prim)
    half_height = float(scan_dims[2]) / 2.0
    box_top_z = float(box_pos[2]) + half_height
    grasp_radius = half_height + GRASP_RADIUS_MARGIN
    gripper.set_target(prim_path, grasp_radius)

    print(f"[PICK] {prim_path} 실측 world pos={np.round(box_pos, 3)}, "
          f"half_height={half_height:.3f}, grasp_radius={grasp_radius:.3f}", flush=True)

    pick_hover_pos = (box_pos[0], box_pos[1], box_top_z + PICK_HOVER_HEIGHT_ABOVE_BOX)
    pick_grasp_pos = (box_pos[0], box_pos[1], box_top_z + TIP_LOCAL_OFFSET[2] + GRASP_STANDOFF)

    # 안전 후퇴 -> 안전 높이에서 수평 접근 -> hover -> grasp (SAFE_TRANSIT_Z 설명
    # 참고) - 지금 팔이 어디 있든(이전 박스 성공/실패와 무관하게) 먼저 그 자리에서
    # 수직으로만 안전 높이까지 올라간 다음에야 새 박스 xy로 수평 이동한다.
    current_ee_pos, _ = robot.end_effector.get_world_pose()
    lift_pos = (float(current_ee_pos[0]), float(current_ee_pos[1]), SAFE_TRANSIT_Z)
    approach_pos = (box_pos[0], box_pos[1], SAFE_TRANSIT_Z)

    move_link6(world, controller, robot, lift_pos, steps=200, label=f"pick_lift[{idx}]", chassis_pin=chassis_pin)
    move_link6(world, controller, robot, approach_pos, steps=200, label=f"pick_approach[{idx}]", chassis_pin=chassis_pin)
    move_link6(world, controller, robot, pick_hover_pos, steps=200, label=f"pick_hover[{idx}]", chassis_pin=chassis_pin)
    move_link6(world, controller, robot, pick_grasp_pos, steps=200, label=f"pick_grasp[{idx}]", chassis_pin=chassis_pin)
    for _ in range(30):
        robot.gripper.close()
        world.step(render=True)
        robot.set_world_pose(position=chassis_pin[0], orientation=chassis_pin[1])
        robot.set_linear_velocity(np.zeros(3))
        robot.set_angular_velocity(np.zeros(3))

    if not robot.gripper.is_closed():
        print(f"[경고] box_id={box_id} 흡착 실패 - grasp_radius/스캔 높이 재확인 필요, 이 박스는 건너뜀", flush=True)
        # 실패해도 다음 박스로 넘어가기 전에 안전 높이로 먼저 올라온다 - 여기서
        # pick_hover(박스 바로 위, 낮은 높이)로만 돌아가면 다음 박스가 또 이 낮은
        # 자리에서 곧장 옆으로 점프하게 되어 같은 발산이 재현된다(실측 확인).
        move_link6(world, controller, robot, (box_pos[0], box_pos[1], SAFE_TRANSIT_Z),
                   steps=150, label=f"pick_실패_후퇴[{idx}]", chassis_pin=chassis_pin)
        continue

    # ---- 흡착 성공: 텔레포트 전에 "안전 운반 자세" 만들기 ----
    # 사용자가 GUI 실행을 직접 보고 지적함: 흡착 직후 살짝만 든 상태(pick_hover
    # 높이, 아직 다른 박스들 옆)에서 곧장 리포지셔닝(텔레포트)하면 문제가 생긴다 -
    # 텔레포트는 순간 이동이라 실제로 뭔가를 "스치며 지나가는" 구간이 없으므로,
    # 텔레포트 *직전*의 팔 자세가 곧 텔레포트 *직후*의 자세(섀시만 바뀐 채 관절
    # 각도는 그대로 유지됨)와 똑같다. 살짝 든 낮고 옆으로 뻗은 자세 그대로
    # 텔레포트하면, 새 위치에서도 똑같이 낮고 옆으로 뻗은(=위험한) 자세로
    # 도착해버린다. 그래서 텔레포트 전에: (1) 박스와 함께 안전 높이까지 수직으로
    # 완전히 들어올리고, (2) 섀시 쪽으로 팔을 당겨서 몸에 붙인 컴팩트한 "운반 자세"를
    # 만든 다음에야 텔레포트한다 - 이 운반 자세는 섀시 기준 상대 오프셋으로 정의해서
    # (CARRY_OFFSET_FROM_CHASSIS), 텔레포트 후에도(관절 각도가 그대로 유지되므로)
    # 자동으로 "새 섀시 위치 기준 같은 상대 자세"가 된다 - 추가 이동 없이 바로
    # PLACE 접근을 시작할 수 있다.
    post_grasp_lift_pos = (box_pos[0], box_pos[1], SAFE_TRANSIT_Z)
    move_link6(world, controller, robot, post_grasp_lift_pos, steps=200, hold_gripper_closed=True,
               label=f"pick_post_lift[{idx}]", chassis_pin=chassis_pin)

    carry_pos = (
        float(chassis_pin[0][0]) + CARRY_OFFSET_FROM_CHASSIS[0],
        float(chassis_pin[0][1]) + CARRY_OFFSET_FROM_CHASSIS[1],
        SAFE_TRANSIT_Z,
    )
    move_link6(world, controller, robot, carry_pos, steps=200, hold_gripper_closed=True,
               label=f"pick_carry[{idx}]", chassis_pin=chassis_pin)

    if idx == 0:
        snapshot(
            world, viewport,
            eye=[BASE_POS[0] - 1.0, BASE_POS[1] - 1.3, 1.4],
            target=[box_pos[0], box_pos[1], box_top_z],
            fname="_crate_01_picked.png",
            robot=robot, chassis_pin=chassis_pin,
)

    # ---- 2. 리포지셔닝: 테이블 -> 크레이트 (팔은 이미 안전 운반 자세) ----
    controller, chassis_target_pos = reposition_chassis(world, robot, CHASSIS_TARGET_XY, f"to_crate[{idx}]")
    chassis_pin = (chassis_target_pos, chassis_quat)

    if idx == 0:
        snapshot(
            world, viewport,
            eye=[chassis_target_pos[0] - 1.0, chassis_target_pos[1] - 1.3, 1.5],
            target=[CHASSIS_TARGET_XY[0], CHASSIS_TARGET_XY[1], BASE_POS[2] - MOUNT_Z],
            fname="_crate_02_repositioned.png",
            robot=robot, chassis_pin=chassis_pin,
)

    # ---- 3. PLACE (안전 높이에서 수평 접근 -> 이후로는 순수 Z축 하강만) ----
    # 사용자가 GUI 실행을 직접 보고 지적함: 텔레포트 직후 자세(image #21)에서 목표
    # 위치로 대각선(x/y/z 동시 변화) 이동을 하면, 그 대각선 경로가 크레이트 안에
    # 이미 놓인 더미 장애물과 부딪히며 "터지는"(그립 폭발과 같은 종류의 PhysX 충돌
    # 반응) 문제가 생긴다. 대신: 팔은 이미 안전 운반 자세(안전 높이, 섀시 근처)로
    # 도착해 있으므로, 먼저 그 안전 높이를 유지한 채 목표 xy 바로 위까지 순수하게
    # 수평으로만 이동하고, 그 다음부터는 xy를 전혀 안 바꾸고 z만 내리는 순수 수직
    # 하강으로 hover -> release까지 진행한다 - 장애물을 옆에서 스치는 경로 자체가
    # 없어진다.
    place_center, place_dims, place_min = world_aabb_from_base_pos_dims(
        placement["position_base_frame"], placement["dimensions"], BASE_POS, R_BASE)
    place_world_xy = (float(place_center[0]), float(place_center[1]))
    crate_floor_world_z = float(place_min[2])
    place_release_z = (
        crate_floor_world_z + RELEASE_CLEARANCE_ABOVE_FLOOR
        + float(place_dims[2]) + TIP_LOCAL_OFFSET[2]
    )
    place_hover_z = place_release_z + 0.15

    place_approach_pos = (place_world_xy[0], place_world_xy[1], SAFE_TRANSIT_Z)
    place_hover_pos = (place_world_xy[0], place_world_xy[1], place_hover_z)
    place_release_pos = (place_world_xy[0], place_world_xy[1], place_release_z)

    move_link6(world, controller, robot, place_approach_pos, steps=300, hold_gripper_closed=True, label=f"place_approach[{idx}]", chassis_pin=chassis_pin)
    move_link6(world, controller, robot, place_hover_pos, steps=300, hold_gripper_closed=True, label=f"place_hover[{idx}]", chassis_pin=chassis_pin)

    if idx == 0:
        snapshot(
            world, viewport,
            eye=[chassis_target_pos[0] - 1.0, chassis_target_pos[1] - 1.3, 1.5],
            target=[place_world_xy[0], place_world_xy[1], crate_floor_world_z],
            fname="_crate_03_approaching.png",
            robot=robot, chassis_pin=chassis_pin,
)

    descent_span = place_hover_z - place_release_z
    substep_moves = max(300 // PLACE_DESCENT_SUBSTEPS, 1)
    for sub_i in range(1, PLACE_DESCENT_SUBSTEPS + 1):
        sub_z = place_hover_z - descent_span * sub_i / PLACE_DESCENT_SUBSTEPS
        sub_pos = (place_world_xy[0], place_world_xy[1], sub_z)
        move_link6(
            world, controller, robot, sub_pos, steps=substep_moves, hold_gripper_closed=True,
            label=f"place_release_sub{sub_i}/{PLACE_DESCENT_SUBSTEPS}[{idx}]", chassis_pin=chassis_pin,
        )

    robot.gripper.open()

    # 진짜 원인을 찾음: release 직전 박스는 그리퍼와 완전히 같은 속도(거의 0)로 300스텝
    # 동안 붙어있었다 - 조인트를 떼자마자 중력이 붙긴 했지만, 그 몇 프레임 사이 속도가
    # PhysX의 sleep 임계값 아래로 유지되면서 "정지 상태"로 오판돼 그대로 잠들어버린
    # 것으로 보인다. SingleRigidPrim으로 직접 작은 속도를 줘서 강제로 깨운다.
    box_rigid_prim = SingleRigidPrim(prim_path)
    box_rigid_prim.initialize(physics_sim_view=world.physics_sim_view)
    box_rigid_prim.set_linear_velocity(np.array([0.0, 0.0, -0.3]))
    print(f"[낙하 유도] {prim_path}에 -z 속도 부여해서 sleep 상태 강제 해제", flush=True)

    for i in range(180):
        world.step(render=True)
        robot.set_world_pose(position=chassis_pin[0], orientation=chassis_pin[1])
        robot.set_linear_velocity(np.zeros(3))
        robot.set_angular_velocity(np.zeros(3))
        if (i + 1) % 30 == 0:
            _z = get_world_pos(box_prim)[2]
            print(f"  [낙하 확인] {i + 1}/180 스텝, 박스 z={_z:.3f}", flush=True)

    final_box_pos = get_world_pos(box_prim)
    err_xy = np.linalg.norm(final_box_pos[:2] - np.array(place_world_xy))
    print(
        f"\n[완료 {idx + 1}/{len(placements)}] {prim_path} 최종 world 위치={np.round(final_box_pos, 3)}, "
        f"목표 xy={np.round(place_world_xy, 3)}, xy 오차={err_xy:.4f}m",
        flush=True,
    )

    # 후퇴도 PLACE와 대칭 - 순수 Z축으로 hover, 그 다음 안전 높이까지 올라간 뒤에야
    # 다음 박스가 있으면 섀시 쪽으로 당겨서 다시 안전 운반 자세를 만든다(위 PICK쪽
    # 설명과 동일한 이유 - 다음 리포지셔닝 전에 항상 컴팩트한 자세로 만들어둔다).
    move_link6(world, controller, robot, place_hover_pos, steps=200, label=f"place_retract_hover[{idx}]", chassis_pin=chassis_pin)
    place_retract_safe_pos = (place_world_xy[0], place_world_xy[1], SAFE_TRANSIT_Z)
    move_link6(world, controller, robot, place_retract_safe_pos, steps=200, label=f"place_retract_safe[{idx}]", chassis_pin=chassis_pin)

    snapshot(
        world, viewport,
        eye=[chassis_target_pos[0] - 1.0, chassis_target_pos[1] - 1.3, 1.5],
        target=[place_world_xy[0], place_world_xy[1], crate_floor_world_z],
        fname=f"_crate_04_placed_box{idx + 1}.png",
        robot=robot, chassis_pin=chassis_pin,
)
    if idx == len(placements) - 1:
        snapshot(
            world, viewport,
            eye=[chassis_target_pos[0] - 1.0, chassis_target_pos[1] - 1.3, 1.5],
            target=[place_world_xy[0], place_world_xy[1], crate_floor_world_z],
            fname="_crate_04_placed.png",
            robot=robot, chassis_pin=chassis_pin,
)

    # ---- 4. 다음 박스가 남아 있으면 테이블로 복귀 (역시 텔레포트 전에 안전 운반 자세) ----
    if idx < len(placements) - 1:
        return_carry_pos = (
            float(chassis_pin[0][0]) + CARRY_OFFSET_FROM_CHASSIS[0],
            float(chassis_pin[0][1]) + CARRY_OFFSET_FROM_CHASSIS[1],
            SAFE_TRANSIT_Z,
        )
        move_link6(world, controller, robot, return_carry_pos, steps=200, label=f"place_carry_before_return[{idx}]", chassis_pin=chassis_pin)

        controller, chassis_target_pos = reposition_chassis(world, robot, ROBOT_START_XY, f"to_table[{idx}]")
        chassis_pin = (chassis_target_pos, chassis_quat)

print(f"\n[안내] 전체 검증 완료 - 배치 시도 {len(placements)}개.\n", flush=True)

# ================= 이번 실행 결과를 날짜별 폴더에 보관 (35.py와 동일 - 매번 덮어써지는 문제 해결) =================
import shutil
from datetime import datetime

_run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
_archive_dir = RESULTS_DIR / "runs" / f"{_run_stamp}_36pickplace"
_archive_dir.mkdir(parents=True, exist_ok=True)
# 폴더 이름에 이미 타임스탬프가 있지만, 파일만 다른 곳으로 꺼내 보면 어느 실행인지
# 구분이 안 된다(특히 idx==0 스냅샷은 PICK이 정밀해진 뒤로 여러 실행에서 내용까지
# 완전히 동일한 경우가 실제로 있었다 - 버그가 아니라 결정론적으로 재현된 것) -
# 파일명 자체에도 타임스탬프를 접두어로 붙인다.
_src_names = [_f.name for _f in _THIS_DIR.glob("_crate_0*.png")]
_archived = []
for _name in _src_names:
    _archived_name = f"{_run_stamp}_{_name}"
    shutil.copy2(_THIS_DIR / _name, _archive_dir / _archived_name)
    _archived.append(_archived_name)
print(f"[보관] {_archive_dir} 에 {len(_archived)}개 파일 복사: {sorted(_archived)}", flush=True)

if HEADLESS:
    simulation_app.close()
else:
    print("[안내] 창을 직접 둘러보세요 - 닫으면 스크립트가 종료됩니다.\n", flush=True)
    while simulation_app.is_running():
        world.step(render=True)
    simulation_app.close()
