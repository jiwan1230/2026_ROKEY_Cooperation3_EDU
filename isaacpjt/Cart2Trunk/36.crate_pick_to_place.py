"""
36.crate_pick_to_place.py
테이블에서 박스 1개를 집어서, 크레이트 안 두 더미 박스 사이 빈 자리(적재 알고리즘이
계산한 좌표)에 내려놓는다 - 33.box_table_pick_to_trunk.py의 트렁크 버전을 대체.

33.py와 달라진 점
----
1. 차량/트렁크가 없다 - 35.crate_scan_setup.py가 만든 오픈-탑 크레이트(뚜껑 없음)가
   목표다. 그래서 33.py에 있던 "chassis_link <-> /World/Vehicle 충돌 임시 해제"
   (UsdPhysics.FilteredPairsAPI) 코드가 통째로 필요 없다 - 통과시킬 차체가 없다.
2. 크레이트 안에 더미 박스 2개가 미리 놓여 있고(35.py가 실제로 스캔해서
   obstacles로 등록), 적재 알고리즘이 그 사이 빈 자리를 실제로 찾아서 배치했다
   (results/crate_demo/placement_result.json) - 33.py는 트렁크가 빈 채로 박스
   하나만 넣어봐서 "장애물 회피"를 전혀 보여주지 못했는데, 이번엔 그게 보인다.
3. 트렁크 문(천장) 높이 제한이 없다(오픈-탑이라 물리적으로 막는 게 없음) - 그래서
   PLACE가 33.py의 wp_entrance->wp_interior 2단 접근 없이 hover->하강->release
   단일 경로로 단순화된다(PICK과 대칭 구조).
4. 목표(world (0.27, -1.41) 근방)가 테이블 스캔 위치(로봇 베이스 (0,-0.55))에서
   반경 약 0.9m로 너무 멀어서 팔만으로는 안 닿는다 - 승인된 계획의 대체안대로
   "회전 없는 순수 이동"만 추가한다(33.py의 "차체를 통과하는 텔레포트"와 달리, 이번엔
   갈 길에 아무 장애물도 없으므로 충돌 해제가 필요 없는 정직한 리포지셔닝이다).

좌표 근거
----
5. 테이블 박스 검출 필터를 수정해서(support_type 기반 -> 알려진 크기 매칭 기반)
   Large가 매번 제대로 잡히게 되자, 적재 알고리즘의 "부피 큰 순" 배치 정책상
   Large가 먼저 크레이트의 좋은 자리를 차지하고 Medium이 NO_VALID_CANDIDATE_POSITION
   으로 밀려나는 상황이 실제로 나왔다(오탐이 아니라 진짜 3개가 경쟁한 정상 결과) -
   사용자 선택에 따라 데모가 실제로 집어서 옮기는 박스를 Medium에서 Small로 바꿨다
   (Small은 Large/Medium과 경쟁해도 배치에 성공함, 실측 확인).

(35.crate_scan_setup.py가 실측한 base_link pose + algorism/14_run_full_pipeline.py를
results/crate_demo/trunk_map.json + 테이블 스캔 박스 JSON으로 돌려서 얻음. trunk_map.json의
vertices 상단(z_max)은 크레이트 벽의 실제 물리 높이(0.15m)가 아니라 별도의 넉넉한 가상
높이(1.0m)를 썼다 - 안 그러면 Trunk.height=0.15m로 잡혀 박스 높이가 SIZE_EXCEEDS_TRUNK로
거부된다(35.py 1차 실행에서 실제로 겪음). 오픈-탑이라 진짜 천장이 없다는 취지를
알고리즘에도 반영한 것.
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

M0609_URDF_PATH = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH = str(M0609_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(M0609_DIR / "rmpflow/m0609_rmpflow_common.yaml")

M0609_PATH = "/World/MobileManipulator/M0609"
CHASSIS_PATH = "/World/MobileManipulator/NovaCarter/chassis_link"
EE_LINK_NAME = "link_6"
GRIPPER_BODY_NAME = "vgp20"
TIP_LOCAL_OFFSET = (0.0, 0.0, 0.121)
# DynamicSuctionGripper.close()의 판정 거리(dist)는 대략 "박스 반높이"로 수렴한다
# (석션 팁은 항상 박스 윗면에 오도록 설계했는데, 박스 중심 거리는 그 팁에서 반높이만큼
# 떨어져 있으므로). 0.10은 Small(반높이 0.06)/Medium(반높이 0.09) 기준으로는 충분했지만
# Large(반높이 0.11)에서는 dist=0.1108m > 0.1m로 매번 흡착이 조용히 실패했다(실측
# 확인 - 로그에 "[흡착 실패]"만 30번 찍히고 예외 없이 넘어가서 눈치채기 쉽지 않았다) -
# 박스가 테이블에 그대로 남은 채 팔만 빈손으로 크레이트까지 이동했고, 그 뒤 벌어진
# RMPflow 발산은 이 실패의 부수적 증상이었을 뿐 원인이 아니었다. 지금 테이블에 있는
# 가장 큰 박스(Large, 0.22m 높이)까지 감안해서 여유 있게 올린다.
GRASP_RADIUS = 0.15

DOWN_QUAT = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))
WAYPOINT_STEPS = 150

# 35.crate_scan_setup.py가 실측해서 저장한 이 세션의 base_link world pose (양쪽 스캔
# 자세 모두 동일값 - 팔만 움직였고 섀시는 안 움직였으므로).
BASE_POS = np.array([3.2465904951095585e-06, -0.550336480140686, 0.41999951004981995])
BASE_QUAT = np.array([0.7071318116571836, 0.0005542394938275026, -0.0005510263145295675, 0.7070813179055245])

# results/crate_demo/placement_result.json box_id=2(Small)의 base_frame 좌표(8코너
# 회전 방식, world axis-swap 버그 회피)를 world로 변환해서 얻은 실제 알고리즘 계산값.
#
# [지난 실수] Medium을 데모 대상으로 쓰던 라운드에서 "알고리즘 원본 좌표가 벽에
# 거의 붙어서 안전 여유가 필요하다"는 이유로, 두 더미 사이 gap의 기하학적 중심
# (0.9, 0.0)을 대신 썼다. 이후 데모 대상을 Small로 바꾸면서 "이 좌표는 특정 박스
# 결과가 아니라 컨테이너 형상 자체에서 나온 값이라 그대로 둬도 된다"고 판단했는데
# - 이게 틀렸다. 알고리즘이 Small에 대해 실제로 계산한 위치(world center
# ≈(0.692,0.127))와 (0.9,0.0)은 약 0.24m나 떨어져 있어서, 시뮬 결과(프리뷰/실제
# 배치)가 placement_flow 그림(37.py, 알고리즘의 진짜 답을 그림)과 전혀 다른 자리에
# 있는 것처럼 보이는 원인이 됐다(사용자 지적으로 발견). "박스마다 다시 계산 안 해도
# 되는 안전한 대체 좌표"라는 편의를 위해 데모의 핵심 목적(알고리즘이 계산한 자리를
# 실제로 실행해서 보여주는 것)을 놓친 것 - 앞으로 데모 대상 박스를 바꿀 때마다
# PLACE_WORLD_XY/PLACE_DIMENSIONS는 반드시 그 박스의 placement_result.json 항목에서
# 새로 계산해야 한다.
#
# [알고리즘 자신의 답도 벽에 거의 닿아있다] Small의 원본 AABB는 y=[0.0516,0.2016]
# - 크레이트 벽(y_max=0.2)을 오차 0의 이상적인 상황에서도 1.6mm 넘는다. DummyA
# (y_max=0.015)와는 3.7cm 여유. 이 자리(가용 y폭 0.185m)는 Small의 y방향 폭
# (0.150m)에 비해 여유가 3.5cm밖에 없다 - 극점 알고리즘이 매 라운드 반복해서
# 보여준 "경계에 딱 붙는" 특성(설계 성질, 버그 아님)이 이번에도 그대로 나타난 것.
# 인위적인 안전 마진 없이 알고리즘의 실제 계산값을 그대로 쓰고, 실측으로 얼마나
# 벗어나는지 확인한다 - 벗어나면 그때 이 tight한 슬롯 자체의 한계로 보고 판단.
#
# [Large 실험 결과 - 실패, Small로 복귀] Large(box_id=3)도 실제로 집어서 넣을 수
# 있는지 검증해봤다 - PICK 자체는 GRASP_RADIUS를 늘린 뒤 성공했지만(아래 참고),
# PLACE 단계(cushion/hover/release)의 RMPflow 수렴 오차가 0.32~0.35m로 Small 때
# (~0.05m)보다 훨씬 커졌고, 최종적으로 박스가 크레이트 안이 아니라 벽/테이블
# 경계 위에 걸쳐 놓였다(실측 확인, 최종 오차 0.30m). Large의 원본 AABB
# (x=[0.7935,1.1429], y=[-0.0453,0.2016])가 DummyB(x_min=1.143)와 0.1mm,
# y_max벽(0.2)과는 -1.6mm로 Small보다 더 타이트했던 것과 더해, 훨씬 큰/무거운
# 박스를 들고 있을 때 이 웨이포인트 체인(cushion/hover) 자체가 Round 1에서 겪은
# "들고 있는 물체가 크레이트 구조물과 부딪혀 RMPflow가 흔들리는" 것과 같은 종류의
# 문제를 다시 일으키는 것으로 보인다(추가 조사 필요, 아직 안 함) - 이 자리는
# 지금 정밀도로는 안정적으로 실행 불가능하다고 판단, Small로 되돌린다.
#
# [PLACE_DIMENSIONS 축 스왑 버그 발견 - 사용자가 타겟 프리뷰 큐브를 보고 지적함]
# PLACE_WORLD_XY(중심점)는 8코너 회전으로 base_link->world 축 스왑(R_base가 z축
# 기준 ~90도 회전이라 base_link의 x/y가 world에서 서로 바뀜)을 제대로 보정해서
# 계산했는데, PLACE_DIMENSIONS는 그 보정 없이 placement_result.json의 원본
# dimensions(base_link 순서, x/y가 아직 안 바뀐 상태)를 그대로 복사해 넣고 있었다
# - 그 결과 타겟 프리뷰(마젠타 큐브)의 x폭/y깊이가 실제와 뒤바뀐 채로 그려져서,
# 정사각형이 아닌 박스는 "90도 돌아간 것처럼" 보이고 그만큼 여유가 없는 축으로
# 튀어나와 벽을 뚫는 것처럼 보였다(Large 실험 때 특히 심하게 나타남 - 원래 폭
# 0.35 vs 원래 깊이 0.25 차이가 커서 훨씬 눈에 띄었다).
# 영향 범위 확인: PLACE_DIMENSIONS[0]/[1](폭/깊이)은 코드 전체에서 이 프리뷰
# 큐브의 scale 한 곳에만 쓰인다 - 실제 PICK/PLACE 실행은 PLACE_WORLD_XY(점 하나)와
# 물리 박스 자신의 실제 치수로만 움직이므로 이 버그의 영향을 받지 않았다(Small
# 실측 결과가 계속 정상이었던 이유). PLACE_DIMENSIONS[2](높이)는 release 높이
# 계산에 쓰이는데 축 스왑은 z축과 무관해서 원래도 문제없었다.
# 수정: 8코너 회전으로 x/y를 포함해 전부 다시 계산(placement_result.json이
# 갱신됐을 수도 있어 좌표도 같이 새로 뽑음).
PLACE_WORLD_XY = (0.6937756394655847, 0.1261962655879676)
PLACE_DIMENSIONS = (0.19957022142237035, 0.15083803392570694, 0.1128347975432542)  # world (x폭, y깊이, 높이)
CRATE_FLOOR_WORLD_Z = 0.14976559773383802  # 크레이트 바닥 (35.py의 CRATE_FLOOR_TOP_Z와 동일, 낮춘 값)
# 기존엔 BASE_POS[2]+0.30(~0.72m)에서 그냥 놓아서 박스가 바닥(~0.15-0.24m)까지
# 0.48m를 자유낙하했다 - "내려놓기"가 아니라 "던지기"에 가까웠고, release 시점의
# 수평 오차(~0.10m)가 그대로 최종 위치 오차로 남았다. PICK이 pick_grasp_pos =
# box_top_z + TIP_LOCAL_OFFSET[2]로 박스 윗면에 흡착 팁을 정확히 맞춰서
# err~0.001-0.002m까지 수렴하는 것과 같은 방식으로, PLACE도 박스 바닥이 크레이트
# 바닥 바로 위(RELEASE_CLEARANCE_ABOVE_FLOOR)에 오도록 link6 목표 높이를 역산해서
# 실제로 "내려놓는다" - 흡착 팁이 박스 윗면을 붙잡고 있으므로 박스는 팁 아래로
# PLACE_DIMENSIONS[2](박스 높이)만큼 매달려 있다:
#   box_bottom = link6_z - TIP_LOCAL_OFFSET[2] - PLACE_DIMENSIONS[2]
# 이걸 CRATE_FLOOR_WORLD_Z + RELEASE_CLEARANCE_ABOVE_FLOOR로 맞추도록 link6_z를 구한다.
RELEASE_CLEARANCE_ABOVE_FLOOR = 0.02  # release 시점에 박스 바닥과 바닥 사이 남길 여유
PLACE_RELEASE_WORLD_Z = (
    CRATE_FLOOR_WORLD_Z + RELEASE_CLEARANCE_ABOVE_FLOOR
    + PLACE_DIMENSIONS[2] + TIP_LOCAL_OFFSET[2]
)
PLACE_HOVER_Z = PLACE_RELEASE_WORLD_Z + 0.15

# 리포지셔닝 목표: 35.py 스캔 때 이미 검증된 위치(충돌 없음 + IK 수렴 양호,
# alignment=0.996) 그대로 재사용한다. (dx,dy) 비율을 바꿔가며 역산해봤지만
# (0.272,0.644)도 오히려 더 심하게 발산해서(139m) "비율" 가설은 기각 - RMPflow는
# 특정 목표 좌표 자체에 예측 불가능하게 민감한 것으로 보인다. 그래서 섀시는 안전이
# 검증된 위치에 고정하고, PLACE_WORLD_XY 쪽을 안정적으로 수렴했던 값 근처로 잡는다.
CHASSIS_TARGET_XY = (0.9, -0.55)

BOX_PICK_PRIM_PATH = "/World/Box_Small"
PICK_HOVER_HEIGHT_ABOVE_BOX = 0.20


class DynamicSuctionGripper(SurfaceGripper):
    """32/33.py와 완전히 동일."""

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


def snapshot(world, viewport, eye, target, fname):
    set_camera_view(eye=eye, target=target)
    for _ in range(15):
        world.step(render=True)
    out = str(_THIS_DIR / fname)
    vp_util.capture_viewport_to_file(viewport, out)
    for _ in range(5):
        world.step(render=True)
    print(f"[SCREENSHOT] {out}", flush=True)


def move_link6(world, controller, robot, target_pos, steps=WAYPOINT_STEPS, hold_gripper_closed=False, label=""):
    """17/33.py와 동일. GUI로 실제 렌더링하면 스텝당 시간이 헤드리스보다 훨씬 길어서
    (실측 234초/2000스텝 vs 헤드리스 20~30초) 웨이포인트 하나가 몇십 초씩 걸릴 수
    있다 - 중간 진행 로그가 없으면 "멈췄나?" 싶어진다. 100스텝마다 한 번씩 찍는다."""
    for i in range(steps):
        actions = controller.forward(
            target_end_effector_position=np.array(target_pos, dtype=float),
            target_end_effector_orientation=DOWN_QUAT,
        )
        robot.apply_action(actions)
        if hold_gripper_closed:
            robot.gripper.close()
        world.step(render=True)
        if (i + 1) % 100 == 0:
            print(f"  [진행{' ' + label if label else ''}] {i + 1}/{steps} 스텝", flush=True)
    ee_pos, _ = robot.end_effector.get_world_pose()
    err = np.linalg.norm(np.array(ee_pos) - np.array(target_pos))
    print(f"[웨이포인트{' ' + label if label else ''}] target={np.round(target_pos, 3)} "
          f"ee={np.round(ee_pos, 3)} err={err:.4f}m", flush=True)
    return ee_pos, err


# ================= 씬 열기 (33.py와 동일 패턴) =================
omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    simulation_app.update()

world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

gripper = DynamicSuctionGripper(
    end_effector_prim_path=f"{M0609_PATH}/{EE_LINK_NAME}",
    gripper_body_path=f"{M0609_PATH}/{GRIPPER_BODY_NAME}",
    target_prim_path=BOX_PICK_PRIM_PATH,
    tip_local_offset=TIP_LOCAL_OFFSET,
    grasp_radius=GRASP_RADIUS,
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

controller = RMPFlowController(
    name="crate_pick_to_place_controller",
    robot_articulation=robot,
    urdf_path=M0609_URDF_PATH,
    robot_description_path=M0609_DESCRIPTION_PATH,
    rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
    end_effector_frame_name=EE_LINK_NAME,
)
controller._default_position = BASE_POS
controller._default_orientation = BASE_QUAT
controller.rmp_flow.set_robot_base_pose(robot_position=BASE_POS, robot_orientation=BASE_QUAT)

viewport = vp_util.get_active_viewport()
snapshot(
    world, viewport,
    eye=[BASE_POS[0] - 1.0, BASE_POS[1] - 1.3, 1.4],
    target=[0.0, 0.0, 0.68],
    fname="_crate_00_start.png",
)

# ================= 0. 목표 위치 미리보기 (pick 전, 비교/설명용) =================
target_marker = VisualCuboid(
    prim_path="/World/PlaceTargetPreview",
    name="place_target_preview",
    position=np.array([PLACE_WORLD_XY[0], PLACE_WORLD_XY[1], CRATE_FLOOR_WORLD_Z + PLACE_DIMENSIONS[2] / 2.0]),
    scale=np.array(PLACE_DIMENSIONS),
    color=np.array([1.0, 0.0, 1.0]),  # 마젠타 - 실제 박스(초록)와 명확히 구분되는 미리보기 색
)
for _ in range(10):
    world.step(render=True)
snapshot(
    world, viewport,
    eye=[CHASSIS_TARGET_XY[0] - 1.0, CHASSIS_TARGET_XY[1] - 1.3, 1.5],
    target=[PLACE_WORLD_XY[0], PLACE_WORLD_XY[1], CRATE_FLOOR_WORLD_Z],
    fname="_crate_00b_target_preview.png",
)
target_marker.set_visibility(False)
print("[미리보기] 적재 알고리즘 목표 위치/크기 마커 저장 완료, 이후 숨김 처리", flush=True)


def get_box_world_pos():
    """XformCache 캐시 문제(33.py에서 실제로 겪음) 회피 - 매번 직접 계산."""
    mat = UsdGeom.Xformable(box_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array(mat.ExtractTranslation())


# ================= 1. PICK =================
box_prim = stage.GetPrimAtPath(BOX_PICK_PRIM_PATH)
box_pos = get_box_world_pos()
box_top_z = float(box_pos[2]) + 0.06  # Small 박스 절반 높이(0.12/2) 근사

print(f"\n[PICK] Box_Small 실측 world pos={np.round(box_pos, 3)}", flush=True)

pick_hover_pos = (box_pos[0], box_pos[1], box_top_z + PICK_HOVER_HEIGHT_ABOVE_BOX)
pick_grasp_pos = (box_pos[0], box_pos[1], box_top_z + TIP_LOCAL_OFFSET[2])

move_link6(world, controller, robot, pick_hover_pos, steps=200, label="pick_hover")
move_link6(world, controller, robot, pick_grasp_pos, steps=200, label="pick_grasp")
for _ in range(30):
    robot.gripper.close()
    world.step(render=True)
if not robot.gripper.is_closed():
    print("[경고] 흡착 실패 - grasp_radius/높이 재조정 필요할 수 있음", flush=True)
move_link6(world, controller, robot, pick_hover_pos, steps=200, hold_gripper_closed=True, label="pick_hover_복귀")

snapshot(
    world, viewport,
    eye=[BASE_POS[0] - 1.0, BASE_POS[1] - 1.3, 1.4],
    target=[box_pos[0], box_pos[1], box_top_z],
    fname="_crate_01_picked.png",
)

# ================= 2. 리포지셔닝 (회전 없이 위치만 - 차체 같은 장애물이 없어 충돌 해제가 필요 없다) =================
chassis_pos_before, chassis_quat_before = robot.get_world_pose()
chassis_target_pos = np.array([CHASSIS_TARGET_XY[0], CHASSIS_TARGET_XY[1], float(chassis_pos_before[2])])
pre_teleport_joint_positions = robot.get_joint_positions()

robot.set_world_pose(position=chassis_target_pos, orientation=chassis_quat_before)
# 텔레포트 직후 관절 각도가 물리 솔버에 의해 미세하게 흔들려서 결국 완전히 다른
# 자세로 "settle"되는 현상을 실측으로 확인했다(팔이 목표와 무관하게 항상 같은 엉뚱한
# 자세에 멈춤 - 더 많은 스텝/새 controller/reset() 다 시도해도 그대로). 텔레포트
# 직전 관절 각도를 그대로 다시 못박아서 물리 상태와 관절 각도가 어긋나지 않게 한다.
robot.set_joint_positions(pre_teleport_joint_positions)
robot.set_linear_velocity(np.zeros(3))
robot.set_angular_velocity(np.zeros(3))
for _ in range(30):
    world.step(render=True)
    robot.set_linear_velocity(np.zeros(3))
    robot.set_angular_velocity(np.zeros(3))
settle_ee_pos, _ = robot.end_effector.get_world_pose()
print(f"[안정화] 텔레포트 후 30스텝, ee_pos={np.round(settle_ee_pos, 3)}", flush=True)

new_base_pos = chassis_target_pos + np.array([0.0, 0.0, 0.42])
# 안정화 스텝을 늘려도(150스텝) 팔이 목표 쪽으로 전혀 움직이지 않았다(600스텝을 더
# 줘도 err가 그대로) - "느리게 수렴 중"이 아니라 RMPflow가 그 자리에서 정지한 것.
# 기존 controller 인스턴스를 재사용하며 set_robot_base_pose()만 다시 부르는 방식은
# PICK에서 이미 검증된 것과 완전히 같은 상대 오프셋(dx=0.22,dy=0.43)조차 못 움직였다 -
# set_robot_base_pose()가 타겟 좌표계는 갱신해도 RMPflow 내부에 캐시된 장애물/충돌
# 모델(테이블, 바닥 등)은 예전 base pose 기준으로 남아있어서, 새 위치에서는 전혀
# 다른(잘못된) 곳에 장애물이 있는 것처럼 보여 움직임을 막았을 가능성이 높다.
# controller.reset()으로도 안 고쳐졌으므로(139m 발산), 아예 새 컨트롤러 인스턴스를
# 만들어 생성자가 지금 섀시 위치를 처음부터 다시 읽게 한다.
controller = RMPFlowController(
    name="crate_pick_to_place_controller_place_phase",
    robot_articulation=robot,
    urdf_path=M0609_URDF_PATH,
    robot_description_path=M0609_DESCRIPTION_PATH,
    rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
    end_effector_frame_name=EE_LINK_NAME,
)
controller._default_position = new_base_pos
controller._default_orientation = chassis_quat_before
controller.rmp_flow.set_robot_base_pose(robot_position=new_base_pos, robot_orientation=chassis_quat_before)
print(
    f"\n[리포지셔닝] chassis {np.round(chassis_pos_before, 3)} -> {np.round(chassis_target_pos, 3)}, "
    f"목표까지 남은 수평거리={np.linalg.norm(chassis_target_pos[:2] - np.array(PLACE_WORLD_XY)):.3f}m",
    flush=True,
)

snapshot(
    world, viewport,
    eye=[chassis_target_pos[0] - 1.0, chassis_target_pos[1] - 1.3, 1.5],
    target=[PLACE_WORLD_XY[0], PLACE_WORLD_XY[1], CRATE_FLOOR_WORLD_Z],
    fname="_crate_02_repositioned.png",
)

# ================= 3. PLACE (완충 웨이포인트 -> hover -> 하강 -> release) =================
# 리포지셔닝 직후 팔은 "테이블에서 pick_hover 하던 자세"를 그대로 강체 이동만 해 온
# 상태라, 거기서 곧장 place_hover(새 목표)로 한 번에 점프시켰더니 RMPflow가 예측
# 불가능하게 완전히 발산했다(같은 좌표인데도 어떤 실행은 err 0.13m, 어떤 실행은
# 130m+ - 목표 좌표 자체보다 "한 번에 너무 큰 점프"가 원인으로 보인다). PICK에서
# 이미 검증된 것과 같은 상대 오프셋(dx=0.22, dy=0.43 - 매번 err<0.002m로 수렴)을
# 새 섀시 위치 기준으로 다시 써서 완충 웨이포인트를 하나 넣는다 - 33.py의
# wp_entrance/wp_interior 다단계 접근과 같은 이유(RMPflow에게 작고 익숙한 점프만
# 준다), 천장 제약과는 무관하게 안정성 자체를 위해 필요했다.
_cushion_pos = (CHASSIS_TARGET_XY[0] + 0.22, CHASSIS_TARGET_XY[1] + 0.43, PLACE_HOVER_Z)
place_hover_pos = (PLACE_WORLD_XY[0], PLACE_WORLD_XY[1], PLACE_HOVER_Z)
place_release_pos = (PLACE_WORLD_XY[0], PLACE_WORLD_XY[1], PLACE_RELEASE_WORLD_Z)

move_link6(world, controller, robot, _cushion_pos, steps=300, hold_gripper_closed=True, label="place_cushion")
move_link6(world, controller, robot, place_hover_pos, steps=300, hold_gripper_closed=True, label="place_hover")
snapshot(
    world, viewport,
    eye=[chassis_target_pos[0] - 1.0, chassis_target_pos[1] - 1.3, 1.5],
    target=[PLACE_WORLD_XY[0], PLACE_WORLD_XY[1], CRATE_FLOOR_WORLD_Z],
    fname="_crate_03_approaching.png",
)
move_link6(world, controller, robot, place_release_pos, steps=300, hold_gripper_closed=True, label="place_release")

robot.gripper.open()

# 진짜 원인을 찾음: release 직전 박스는 그리퍼와 완전히 같은 속도(거의 0)로 300스텝
# 동안 붙어있었다 - 조인트를 떼자마자 중력이 붙긴 했지만(z 0.72->0.509로 실제 낙하),
# 그 몇 프레임 사이 속도가 PhysX의 sleep 임계값 아래로 유지되면서 "정지 상태"로
# 오판돼 그대로 잠들어버린 것으로 보인다 - 헤드리스로 180스텝(3초)을 더 줘봐도
# z=0.509에서 소수점까지 정확히 동일하게 멈춰있음을 실측 확인(사용자가 GUI에서 본
# "안 떨어짐"이 실제 버그였음, 렌더링 지연이 아니었음). SingleRigidPrim으로 직접
# 작은 속도를 줘서 강제로 깨운다.
box_rigid_prim = SingleRigidPrim(BOX_PICK_PRIM_PATH)
box_rigid_prim.initialize(physics_sim_view=world.physics_sim_view)
box_rigid_prim.set_linear_velocity(np.array([0.0, 0.0, -0.3]))
print(f"[낙하 유도] {BOX_PICK_PRIM_PATH}에 -z 속도 부여해서 sleep 상태 강제 해제", flush=True)

for i in range(180):
    world.step(render=True)
    if (i + 1) % 30 == 0:
        _z = get_box_world_pos()[2]
        print(f"  [낙하 확인] {i + 1}/180 스텝, 박스 z={_z:.3f}", flush=True)

final_box_pos = get_box_world_pos()
err_xy = np.linalg.norm(final_box_pos[:2] - np.array(PLACE_WORLD_XY))
print(
    f"\n[완료] 박스 최종 world 위치={np.round(final_box_pos, 3)}, "
    f"목표 xy={np.round(PLACE_WORLD_XY, 3)}, xy 오차={err_xy:.4f}m",
    flush=True,
)

move_link6(world, controller, robot, place_hover_pos, steps=200, label="place_retract")

snapshot(
    world, viewport,
    eye=[chassis_target_pos[0] - 1.0, chassis_target_pos[1] - 1.3, 1.5],
    target=[PLACE_WORLD_XY[0], PLACE_WORLD_XY[1], CRATE_FLOOR_WORLD_Z],
    fname="_crate_04_placed.png",
)

print("\n[안내] 검증 완료.\n", flush=True)

if HEADLESS:
    simulation_app.close()
else:
    print("[안내] 창을 직접 둘러보세요 - 닫으면 스크립트가 종료됩니다.\n", flush=True)
    while simulation_app.is_running():
        world.step(render=True)
    simulation_app.close()
