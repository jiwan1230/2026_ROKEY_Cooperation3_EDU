"""
쇼핑카트 밑으로 들어가서 카트를 들어 올려 운반하는 저상형 로봇(현대위아 주차로봇 스타일) 에셋을 만든다.

1차 목표(이 스크립트): 정적 형상(본체+리프트판+바퀴4개) + 물리(RigidBody/Collision,
리프트판을 올리는 Prismatic Joint)를 갖춘 재사용 가능한 USD 에셋(cart_robot.usd)으로 저장.
바퀴 구동(실제 이동) 로직은 2차 목표 — 이 스크립트에서는 바퀴에 Drive를 달지 않고 자유 회전으로 둔다.

구조 (모두 /World/CartRobot 아래, 각각 독립된 Xform Rigid Body):
  base_link    <- 본체(저상 플레이트), ArticulationRoot
  lift_plate   <- 카트를 들어올리는 상판, base_link와 Prismatic Joint(Z축, 0~LIFT_RANGE)로 연결
  wheel_fl/fr/rl/rr <- 바퀴 4개, base_link와 Revolute Joint(Y축)로 연결 (아직 Drive 없음)
  sensor_mount <- 전방 센서(라이다/카메라) 장착 자리 표시용 큐브 (물리 없음, 순수 시각용)

치수는 1차 추정값이다 (참고: 현대위아 주차로봇 두께 약 110mm, 전방향 이동, 실제 쇼핑카트 폭 고려).
실제 카트 하부 간격에 맞는지는 이 프로젝트의 기존 방식대로(HANDOFF.md 5-1절 참고) Isaac Sim에서
Metal_Shopping_Cart.usdz와 겹쳐놓고 스크린샷으로 확인 후 조정이 필요하다 — 숫자만 믿지 말 것.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import time

import omni.usd
import omni.kit.viewport.utility as vp_util
from pxr import UsdGeom, UsdPhysics, Sdf, Gf
from isaacsim.core.api import World
from isaacsim.core.utils.viewports import set_camera_view

_THIS_DIR = Path(__file__).resolve().parent
OUT_USD = str(_THIS_DIR / "cart_robot.usd")

# ---- 치수 (미터 단위, 1차 추정값) ----
BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT = 0.85, 0.50, 0.10
LIFT_LENGTH, LIFT_WIDTH, LIFT_THICK = 0.75, 0.45, 0.02
LIFT_RANGE = 0.06                          # 리프트판이 올라가는 최대 높이
WHEEL_RADIUS, WHEEL_THICK = 0.06, 0.04
WHEEL_INSET_X = 0.12                       # 앞/뒤 모서리에서 안쪽으로 들어간 거리
WHEEL_Y_OFFSET = BASE_WIDTH / 2 + WHEEL_THICK / 2 + 0.01   # 본체 옆면보다 살짝 밖으로 나오게

BASE_MASS, LIFT_MASS, WHEEL_MASS = 40.0, 5.0, 1.0

ROBOT_PATH = "/World/CartRobot"

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = omni.usd.get_context().get_stage()


def add_rigid_box(name, size_xyz, world_pos, mass_kg, color, articulation_root=False):
    """ROBOT_PATH 아래 name Xform(RigidBody)을 만들고 그 안에 Cube 지오메트리(Collision)를 넣는다."""
    xform_path = f"{ROBOT_PATH}/{name}"
    xform = UsdGeom.Xform.Define(stage, xform_path)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*world_pos))

    cube = UsdGeom.Cube.Define(stage, f"{xform_path}/geom")
    cube.CreateSizeAttr().Set(1.0)
    UsdGeom.Xformable(cube).AddScaleOp().Set(Gf.Vec3f(*size_xyz))
    cube.CreateDisplayColorAttr().Set([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    rigid_prim = xform.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(rigid_prim)
    UsdPhysics.MassAPI.Apply(rigid_prim).CreateMassAttr().Set(mass_kg)
    if articulation_root:
        UsdPhysics.ArticulationRootAPI.Apply(rigid_prim)
    print(f"[BOX] {xform_path} size={size_xyz} pos={world_pos} mass={mass_kg}kg", flush=True)
    return xform_path


def add_rigid_wheel(name, world_pos, mass_kg, color=(0.05, 0.05, 0.05)):
    xform_path = f"{ROBOT_PATH}/{name}"
    xform = UsdGeom.Xform.Define(stage, xform_path)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*world_pos))

    cyl = UsdGeom.Cylinder.Define(stage, f"{xform_path}/geom")
    cyl.CreateRadiusAttr().Set(WHEEL_RADIUS)
    cyl.CreateHeightAttr().Set(WHEEL_THICK)
    cyl.CreateAxisAttr().Set("Y")
    cyl.CreateDisplayColorAttr().Set([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())

    rigid_prim = xform.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(rigid_prim)
    UsdPhysics.MassAPI.Apply(rigid_prim).CreateMassAttr().Set(mass_kg)
    print(f"[WHEEL] {xform_path} pos={world_pos} mass={mass_kg}kg", flush=True)
    return xform_path


def connect_joint(joint_type, joint_name, parent_path, parent_world_pos, child_path, child_world_pos, axis):
    """parent(=base_link, 회전 없음)와 child를 axis 방향 joint로 연결.
    parent/child 둘 다 회전이 없으므로 localPos0는 그냥 (child_world_pos - parent_world_pos)."""
    joint_path = f"{child_path}/{joint_name}"
    if joint_type == "prismatic":
        joint = UsdPhysics.PrismaticJoint.Define(stage, joint_path)
    else:
        joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
    joint.CreateAxisAttr().Set(axis)
    joint.GetBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.GetBody1Rel().SetTargets([Sdf.Path(child_path)])
    local0 = Gf.Vec3f(*(child_world_pos[i] - parent_world_pos[i] for i in range(3)))
    joint.CreateLocalPos0Attr().Set(local0)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    print(f"[JOINT] {joint_path} type={joint_type} axis={axis} localPos0={local0}", flush=True)
    return joint


# ---- base_link (본체, ArticulationRoot) ----
base_z = WHEEL_RADIUS + BASE_HEIGHT / 2
base_pos = (0.0, 0.0, base_z)
base_path = add_rigid_box("base_link", (BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT), base_pos,
                           BASE_MASS, color=(0.15, 0.15, 0.18), articulation_root=True)

# ---- lift_plate (base_link 위, Prismatic Joint Z축으로 연결) ----
lift_z = WHEEL_RADIUS + BASE_HEIGHT + LIFT_THICK / 2
lift_pos = (0.0, 0.0, lift_z)
lift_path = add_rigid_box("lift_plate", (LIFT_LENGTH, LIFT_WIDTH, LIFT_THICK), lift_pos,
                           LIFT_MASS, color=(0.6, 0.55, 0.05))
lift_joint = connect_joint("prismatic", "lift_joint", base_path, base_pos, lift_path, lift_pos, axis="Z")
lift_joint.CreateLowerLimitAttr().Set(0.0)
lift_joint.CreateUpperLimitAttr().Set(LIFT_RANGE)

lift_drive = UsdPhysics.DriveAPI.Apply(lift_joint.GetPrim(), "linear")
lift_drive.CreateTypeAttr().Set("force")
lift_drive.CreateStiffnessAttr().Set(5e4)
lift_drive.CreateDampingAttr().Set(5e3)
lift_drive.CreateMaxForceAttr().Set(2000.0)
lift_drive.CreateTargetPositionAttr().Set(0.0)   # 기본값: 접혀있는(내려간) 상태 유지
print(f"[DRIVE] lift_joint stiffness=5e4 damping=5e3 maxForce=2000N target=0.0(접힘)", flush=True)

# ---- 바퀴 4개 (base_link와 Revolute Joint Y축으로 연결, 아직 Drive 없음 = 자유 회전) ----
wheel_x = BASE_LENGTH / 2 - WHEEL_INSET_X
wheel_specs = {
    "wheel_fl": (wheel_x, WHEEL_Y_OFFSET),
    "wheel_fr": (wheel_x, -WHEEL_Y_OFFSET),
    "wheel_rl": (-wheel_x, WHEEL_Y_OFFSET),
    "wheel_rr": (-wheel_x, -WHEEL_Y_OFFSET),
}
for name, (wx, wy) in wheel_specs.items():
    wpos = (wx, wy, WHEEL_RADIUS)
    wpath = add_rigid_wheel(name, wpos, WHEEL_MASS)
    connect_joint("revolute", f"{name}_joint", base_path, base_pos, wpath, wpos, axis="Y")

# ---- 센서 마운트 (전방, 순수 시각용 - 물리 없음) ----
sensor_path = f"{base_path}/sensor_mount"
sensor_xform = UsdGeom.Xform.Define(stage, sensor_path)
sensor_xform.ClearXformOpOrder()
sensor_xform.AddTranslateOp().Set(Gf.Vec3d(BASE_LENGTH / 2 - 0.03, 0.0, BASE_HEIGHT / 2 + 0.02))
sensor_cube = UsdGeom.Cube.Define(stage, f"{sensor_path}/geom")
sensor_cube.CreateSizeAttr().Set(1.0)
UsdGeom.Xformable(sensor_cube).AddScaleOp().Set(Gf.Vec3f(0.03, 0.10, 0.03))
sensor_cube.CreateDisplayColorAttr().Set([Gf.Vec3f(0.8, 0.1, 0.1)])
print(f"[SENSOR] {sensor_path} (물리 없음, 자리 표시용)", flush=True)

for _ in range(20):
    simulation_app.update()

try:
    world.reset()
    for _ in range(120):
        world.step(render=True)
    print("[물리] world.reset() + 120 step 성공, 에러 없음", flush=True)
except Exception as e:
    print(f"[물리-에러] {e}", flush=True)

viewport = vp_util.get_active_viewport()


def snapshot(eye, target, fname):
    set_camera_view(eye=eye, target=target)
    for _ in range(20):
        world.step(render=True)
    out = str(_THIS_DIR / fname)
    vp_util.capture_viewport_to_file(viewport, out)
    for _ in range(20):
        world.step(render=True)
    print(f"[SCREENSHOT] {out}", flush=True)


snapshot(eye=[1.5, -1.5, 1.0], target=[0.0, 0.0, 0.15], fname="_verify_cart_robot_wide.png")
snapshot(eye=[0.6, -0.6, 0.4], target=[0.0, 0.0, 0.15], fname="_verify_cart_robot_close.png")

print("\n[안내] 재생 버튼은 이미 눌린 상태로 검증 완료. 카트(Metal_Shopping_Cart.usdz)와 겹쳐서")
print("[안내] 실제로 밑에 들어가는지 확인하려면 씬에 카트를 같이 로드해서 크기/위치를 비교할 것.")
print("[안내] 창을 닫으면 결과가 저장됩니다.\n", flush=True)

while simulation_app.is_running():
    simulation_app.update()
    time.sleep(0.01)

omni.usd.get_context().save_as_stage(OUT_USD)
print(f"\n[저장 완료] {OUT_USD}", flush=True)

simulation_app.close()

