"""
33.box_table_pick_to_trunk.py
박스 1개를 테이블에서 집어서, 적재 알고리즘이 계산한 트렁크 내부 좌표에 내려놓는다.

배경
----
32.box_table_scan_setup.py + box_top_extractor.py + 14_run_full_pipeline.py 파이프라인이
검증 완료된 상태(720b78e 커밋). 이번엔 실제로 검출된 박스 중 하나(box_id=0, Medium -
검출 높이 0.17m가 실제 규격 0.18m-마진0.01m와 거의 정확히 일치해서 가장 확실한 진짜
검출)를 골라, 알고리즘이 제시한 트렁크 내부 위치로 실제로 옮긴다.

좌표계 정합 (사전 작업, 이 스크립트 밖에서 이미 완료)
----
14_run_full_pipeline.py는 트렁크와 박스가 같은 m0609_base_link 좌표계라고 전제하는데,
기존 trunk_map.json(results/run_20260720_200104/)은 12.trunk_scan_hidden_gripper.py
세션(base_pos~(2.259,-0.150,0.420), yaw~0)의 base_link이고, 박스 JSON은
32.box_table_scan_setup.py 세션(base_pos~(0,-0.55,0.42), yaw~90deg)의 base_link라
실제로는 다른 원점이었다. 스크래치패드에서 두 세션의 실측 base_pos/base_quat로
trunk_map을 박스-스캔 세션 기준으로 재투영한 뒤 파이프라인을 다시 돌려서 얻은 결과:

    box_id=0 position_base_frame = (0.9636, -3.3139, 0.0130)  [box-scan 세션 base_link 기준]
    -> world 좌표 = (3.314, 0.413, 0.435)  [트렁크 바닥, TRUNK_FLOOR_Z(0.44)와 거의 일치]

리치 문제
----
목표 z(0.435)는 base_link 기준 relative z≈0.01로, M0609가 항상 IK 실패하는 구간이다
([[cart2trunk_mobile_pick_demo]] 메모: relative z<=0.15 항상 실패, z>=0.30 잘 수렴).
17/28번 스크립트와 동일한 타협을 그대로 쓴다: 실제 바닥이 아니라 "바닥+0.30m 높이"에서
그리퍼를 열고 중력으로 나머지를 떨어뜨린다.

수평 거리 문제 (사용자 지적, 미리 알려진 리스크)
----
현재 로봇 위치(박스 스캔 자세)에서 목표까지 수평 거리는 약 3.45m - 당연히 팔이 안 닿는다.
Nova Carter를 거기까지 실제로 주행시키는 건(회전 함수가 이 프로젝트에 한 번도 검증된 적
없어서 리스크가 크다는 사용자 지적으로) 이번엔 하지 않는다. 대신:
  1. chassis_link <-> /World/Vehicle 충돌을 PhysxSchema.FilteredPairsAPI로 끈다
     (28.py가 chassis_link<->base_link에 쓰던 것과 동일 API, 대상만 다름).
  2. robot(=병합된 articulation의 root, chassis_link)을 9.py의 reach-sweep으로 검증된
     안전 오프셋(로컬 dx=0.35, dz는 그대로 유지)만큼 떨어진 목표 근처로 한 번에
     set_world_pose()로 텔레포트한다 - 28.py가 리프트 흉내낼 때 매 프레임 하는 것과
     동일한 API를 한 번만 쓰는 것.
  이 충돌 해제는 이번 실험 한정 타협이다 - 실제 하드웨어에는 적용 불가.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pathlib import Path
import sys

import numpy as np
import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

_THIS_DIR = Path(__file__).resolve().parent
M0609_DIR = _THIS_DIR.parent / "M0609"
RMPFLOW_DIR = str(M0609_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController  # noqa: E402

SCENE_USD = str(_THIS_DIR / "box_table_scan_scene.usd")

M0609_URDF_PATH = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH = str(M0609_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(M0609_DIR / "rmpflow/m0609_rmpflow_common.yaml")

M0609_PATH = "/World/MobileManipulator/M0609"
CHASSIS_PATH = "/World/MobileManipulator/NovaCarter/chassis_link"
EE_LINK_NAME = "link_6"
GRIPPER_BODY_NAME = "vgp20"
TIP_LOCAL_OFFSET = (0.0, 0.0, 0.121)
GRASP_RADIUS = 0.10

DOWN_QUAT = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))
WAYPOINT_STEPS = 150

# 32번 스크립트가 실측해서 저장한 이 세션의 base_link world pose.
BASE_POS = np.array([3.2465904951095585e-06, -0.550336480140686, 0.41999951004981995])
BASE_QUAT = np.array([0.7071318116571836, 0.0005542394938275026, -0.0005510263145295675, 0.7070813179055245])

# 좌표 재투영(스크래치패드) + 14_run_full_pipeline.py 재실행으로 얻은 box_id=0 목표.
PLACE_WORLD_XY = (3.31402086, 0.41301339)
TRUNK_FLOOR_WORLD_Z = 0.435  # 참고용 - 실제로는 안 씀(reach 문제로 못 내려감)
RELEASE_HEIGHT_ABOVE_BASE = 0.30  # [[cart2trunk_mobile_pick_demo]] 기준 안전 도달 높이
PLACE_RELEASE_WORLD_Z = float(BASE_POS[2]) + RELEASE_HEIGHT_ABOVE_BASE  # ~0.72, 17/28.py와 동일 타협

# 텔레포트 목표 계산: 9.py reach-sweep에서 검증된 로컬 오프셋(dx=0.35, dy=0)만큼 떨어진 지점에
# 로봇을 놓는다 - 즉 chassis를 목표보다 dx=0.35m 만큼 -X 쪽에, y는 목표와 동일하게 둔다.
REACH_DX = 0.35
CHASSIS_TARGET_XY = (PLACE_WORLD_XY[0] - REACH_DX, PLACE_WORLD_XY[1])
CHASSIS_TARGET_YAW_DEG = 0.0  # 텔레포트라 방향은 자유 - 세계 +X를 보게 (17/28.py와 동일 관례)

BOX_PICK_PRIM_PATH = "/World/Box_Medium"
PICK_HOVER_HEIGHT_ABOVE_BOX = 0.20


class DynamicSuctionGripper(SurfaceGripper):
    """28.cart_to_trunk_pick_place_lift.py와 완전히 동일."""

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
    """17.cart_to_trunk_pick_place.py와 동일."""
    for _ in range(steps):
        actions = controller.forward(
            target_end_effector_position=np.array(target_pos, dtype=float),
            target_end_effector_orientation=DOWN_QUAT,
        )
        robot.apply_action(actions)
        if hold_gripper_closed:
            robot.gripper.close()
        world.step(render=True)
    ee_pos, _ = robot.end_effector.get_world_pose()
    err = np.linalg.norm(np.array(ee_pos) - np.array(target_pos))
    print(f"[웨이포인트{' ' + label if label else ''}] target={np.round(target_pos, 3)} "
          f"ee={np.round(ee_pos, 3)} err={err:.4f}m", flush=True)
    return ee_pos, err


# ================= 씬 열기 (2.open_saved_scene.py 패턴) =================
omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    simulation_app.update()

world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

# ================= 차량 충돌 임시 해제 (이번 실험 한정 타협) =================
# world.reset()/robot.initialize() 이후(물리 시뮬레이션 시작 후)에 이 스키마를 추가하면
# PhysX 텐서 뷰가 무효화되어 "Simulation view object is invalidated" 에러가 난다(실측 확인) -
# 그래서 물리가 시작되기 전인 지금 미리 걸어둔다. 28.py의 chassis_link<->base_link 패턴과
# 동일 API(UsdPhysics.FilteredPairsAPI), 대상만 /World/Vehicle로 다르다.
vehicle_prim = stage.GetPrimAtPath("/World/Vehicle")
chassis_prim = stage.GetPrimAtPath(CHASSIS_PATH)
filt_chassis = UsdPhysics.FilteredPairsAPI.Apply(chassis_prim)
filt_chassis.CreateFilteredPairsRel().AddTarget(vehicle_prim.GetPath())
filt_vehicle = UsdPhysics.FilteredPairsAPI.Apply(vehicle_prim)
filt_vehicle.CreateFilteredPairsRel().AddTarget(chassis_prim.GetPath())
print(f"\n[충돌 해제] {CHASSIS_PATH} <-> /World/Vehicle (이번 실험 한정, 실제 로봇엔 적용 불가)", flush=True)

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
    name="pick_to_trunk_controller",
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
    fname="_pick_to_trunk_00_start.png",
)

def get_box_world_pos():
    """UsdGeom.XformCache는 물리 시뮬레이션이 갱신한 위치를 캐시가 낀 채로 오래된 값을
    돌려줄 수 있다(실측: 텔레포트+PLACE 이후에도 PICK 시점 위치를 그대로 반환하는 버그로
    확인됨) - DynamicSuctionGripper.close()가 이미 쓰고 있는, 캐시 없이 매번 직접 계산하는
    방식으로 통일한다."""
    mat = UsdGeom.Xformable(box_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array(mat.ExtractTranslation())


# ================= 1. PICK =================
box_prim = stage.GetPrimAtPath(BOX_PICK_PRIM_PATH)
box_pos = get_box_world_pos()
box_top_z = float(box_pos[2]) + 0.09  # Medium 박스 절반 높이(0.18/2) 근사 - 아래 log로 실측 보정 확인

print(f"\n[PICK] Box_Medium 실측 world pos={np.round(box_pos, 3)}", flush=True)

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
    fname="_pick_to_trunk_01_picked.png",
)

# ================= 2. 텔레포트 (충돌 해제는 이미 물리 시작 전에 적용해둠) =================
chassis_pos_before, _ = robot.get_world_pose()
chassis_target_pos = np.array([CHASSIS_TARGET_XY[0], CHASSIS_TARGET_XY[1], float(chassis_pos_before[2])])
chassis_target_quat = euler_angles_to_quat(np.array([0.0, 0.0, np.radians(CHASSIS_TARGET_YAW_DEG)]))

robot.set_world_pose(position=chassis_target_pos, orientation=chassis_target_quat)
robot.set_linear_velocity(np.zeros(3))
robot.set_angular_velocity(np.zeros(3))
for _ in range(30):
    world.step(render=True)

new_base_pos = chassis_target_pos + np.array([0.0, 0.0, 0.42])
controller._default_position = new_base_pos
controller._default_orientation = chassis_target_quat
controller.rmp_flow.set_robot_base_pose(robot_position=new_base_pos, robot_orientation=chassis_target_quat)
print(
    f"\n[텔레포트] chassis {np.round(chassis_pos_before, 3)} -> {np.round(chassis_target_pos, 3)}, "
    f"목표까지 남은 수평거리={np.linalg.norm(chassis_target_pos[:2] - np.array(PLACE_WORLD_XY)):.3f}m",
    flush=True,
)

snapshot(
    world, viewport,
    eye=[chassis_target_pos[0] - 1.2, chassis_target_pos[1] - 1.5, 1.6],
    target=[PLACE_WORLD_XY[0], PLACE_WORLD_XY[1], TRUNK_FLOOR_WORLD_Z],
    fname="_pick_to_trunk_02_teleported.png",
)

# ================= 3. PLACE =================
place_hover_pos = (PLACE_WORLD_XY[0], PLACE_WORLD_XY[1], PLACE_RELEASE_WORLD_Z + 0.15)
place_release_pos = (PLACE_WORLD_XY[0], PLACE_WORLD_XY[1], PLACE_RELEASE_WORLD_Z)

# move_link6_converge(잔차 보정 반복)는 시도해봤으나 오히려 발산했다 - 이 목표가 17.py의
# "체계적 방향성 편차" 케이스가 아니라 도달 가능 범위 경계에 가까운 경우라, aim을 더 밀수록
# 더 못 닿는 쪽으로 악화됐다(실측). 단발 IK + 충분한 스텝 수(그 자체로 err 0.16~0.20m,
# 박스가 트렁크 안에 실제로 안착하는 걸 스크린샷으로 확인함)가 더 나은 결과라 이걸 유지한다.
move_link6(world, controller, robot, place_hover_pos, steps=400, hold_gripper_closed=True, label="place_hover")
move_link6(world, controller, robot, place_release_pos, steps=300, hold_gripper_closed=True, label="place_release")

robot.gripper.open()
for _ in range(60):
    world.step(render=True)

final_box_pos = get_box_world_pos()
err_xy = np.linalg.norm(final_box_pos[:2] - np.array(PLACE_WORLD_XY))
print(
    f"\n[완료] 박스 최종 world 위치={np.round(final_box_pos, 3)}, "
    f"목표 xy={np.round(PLACE_WORLD_XY, 3)}, xy 오차={err_xy:.4f}m",
    flush=True,
)

move_link6(world, controller, robot, place_hover_pos, steps=150, label="place_retract")

snapshot(
    world, viewport,
    eye=[chassis_target_pos[0] - 1.2, chassis_target_pos[1] - 1.5, 1.6],
    target=[PLACE_WORLD_XY[0], PLACE_WORLD_XY[1], TRUNK_FLOOR_WORLD_Z],
    fname="_pick_to_trunk_03_placed.png",
)

print("\n[안내] 검증 완료.\n", flush=True)
simulation_app.close()
