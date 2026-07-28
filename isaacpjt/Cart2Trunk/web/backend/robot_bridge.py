"""
robot_bridge.py
routes/robot.py의 트렁크 스캔 라우트가 쓰는 어댑터. algorism_bridge.py와 같은
이유로(이 백엔드 venv에는 rclpy를 설치하지 않기로 함, algorism_bridge.py 최상단
docstring 참고) Flask 프로세스 안에서 직접 rclpy를 import하지 않는다. 대신
`ros2 run cart2trunk_bridge trunk_scan_client`를 서브프로세스로 실행해서, 그
프로세스가 자기만의 (시스템) ROS2 파이썬 환경 안에서 액션을 호출하고 청크를
모아 파일로 저장한 뒤 마지막 줄에 JSON 결과를 stdout으로 출력하게 한다. 여기서는
그 서브프로세스를 띄우고 마지막 줄만 파싱한다.

trunk_scan_action_server.py(Isaac Sim PC 쪽)와 trunk_scan_client(이 파일이 띄우는
쪽)는 isaacpjt/ros2_ws/src/cart2trunk_bridge/에 있다 - 89.trunk_scan_holonomic.py/
90.export_trunk_map_holonomic.py 원본은 그쪽에서도 서브프로세스로만 다루고
수정하지 않는다.
"""
import json
import os
import pathlib
import shlex
import subprocess

_HERE = pathlib.Path(__file__).resolve().parent
_ISAACPJT_DIR = _HERE.parents[2]  # backend -> web -> Cart2Trunk -> isaacpjt
_ROS2_WS_SETUP = os.environ.get(
    "CART2TRUNK_ROS2_WS_SETUP", str(_ISAACPJT_DIR / "ros2_ws" / "install" / "setup.bash"))
_ROS_SETUP = "/opt/ros/humble/setup.bash"

RECEIVED_SCANS_DIR = _HERE / "received_scans"
TRUNK_SCAN_TIMEOUT_SEC = 180
CART_SCAN_TIMEOUT_SEC = 180
# 2026-07-27 4박스 실측 기준 STAGE0~4 전체(박스당 수 분)가 15~20분 걸렸다 -
# trunk/cart_scan(고정 180초)보다 훨씬 넉넉하게 잡는다.
PICK_AND_PLACE_TIMEOUT_SEC = 1800
# pick_and_place_client가 Feedback을 실시간으로 append하는 파일 - POST
# /api/robot/pick-and-place가 15~20분 동안 블로킹돼있는 중에도 GET
# /api/robot/pick-and-place/progress로 이 파일을 폴링해서 진행 상황을
# 보여줄 수 있다(app.py가 threaded=True라 GET이 POST와 동시에 처리됨).
PICK_AND_PLACE_PROGRESS_FILE = RECEIVED_SCANS_DIR / "pick_and_place_progress.jsonl"


def _run_client_subprocess(cmd: str, label: str, timeout_sec: int) -> dict:
    """`cmd`(전체 셸 명령 문자열, 이미 source .../ros2 run ... 형태로 완성된 것)를
    서브프로세스로 실행하고, 마지막 stdout 줄을 JSON으로 파싱해서 반환한다.
    trunk_scan_client/cart_scan_client/pick_and_place_client/
    send_placement_plan_client 전부 이 함수를 거친다 - 다들 같은
    (success, message, ...) 계약으로 stdout에 JSON 한 줄을 찍기 때문."""
    try:
        proc = subprocess.run(
            ["bash", "-lc", cmd], capture_output=True, text=True, timeout=timeout_sec + 30)
    except subprocess.TimeoutExpired as e:
        raise RuntimeError(f"{label} 서브프로세스 타임아웃: {e}") from e

    stdout_lines = [line for line in proc.stdout.splitlines() if line.strip()]
    if not stdout_lines:
        raise RuntimeError(
            f"{label} 서브프로세스가 출력이 없습니다(returncode={proc.returncode}). "
            f"stderr 마지막 부분: {proc.stderr[-500:]}")

    # 클라이언트 스크립트 자신은 항상 stdout 마지막 줄에 JSON 한 줄을 찍지만,
    # (서비스에 연결 못 해 sys.exit(1)로 정상 종료하는 경우처럼) 비정상
    # 종료로 보이면 `ros2 run`이 그 뒤에 자기 트레일러 메시지("[ros2run]:
    # Process exited with failure N")를 stdout 마지막 줄로 덧붙인다 - 실측
    # 확인(send_placement_plan_client가 서비스 미연결로 _fail()을 정상 출력
    # 했는데도, 진짜 오류 메시지 대신 이 트레일러 때문에 "JSON 파싱 실패"로만
    # 보였다). 뒤에서부터 실제로 JSON으로 파싱되는 줄을 찾는다.
    result = None
    for line in reversed(stdout_lines):
        try:
            result = json.loads(line)
            break
        except json.JSONDecodeError:
            continue

    if result is None:
        raise RuntimeError(
            f"{label} 결과를 JSON으로 파싱할 수 없습니다: {stdout_lines[-1]!r}")

    if not result.get("success"):
        raise RuntimeError(result.get("message", f"{label} 실패(원인 불명)"))

    return result


def _run_scan_client(console_script: str, label: str, timeout_sec: int) -> dict:
    """`ros2 run cart2trunk_bridge <console_script> --output-dir ... --timeout-sec ...`를
    실행한다. trunk_scan_client/cart_scan_client 둘 다 이 함수를 거친다."""
    RECEIVED_SCANS_DIR.mkdir(parents=True, exist_ok=True)

    cmd = (
        f"source {shlex.quote(_ROS_SETUP)} && "
        f"source {shlex.quote(_ROS2_WS_SETUP)} && "
        f"ros2 run cart2trunk_bridge {console_script} "
        f"--output-dir {shlex.quote(str(RECEIVED_SCANS_DIR))} "
        f"--timeout-sec {timeout_sec}"
    )
    return _run_client_subprocess(cmd, label, timeout_sec)


def run_trunk_scan(timeout_sec: int = TRUNK_SCAN_TIMEOUT_SEC) -> dict:
    """`/cart2trunk/trunk_scan` 액션을 호출해서 89.py+90.py를 실행시키고, 청크로
    받은 PLY 2개(전처리+원본)를 RECEIVED_SCANS_DIR에 저장한다. 성공 시
    {"success": True, "filename", "path", "total_bytes", "total_chunks", "point_count",
    "raw_filename", "raw_path", "raw_total_bytes", "raw_point_count"}를 반환하고,
    실패 시 RuntimeError를 던진다."""
    return _run_scan_client("trunk_scan_client", "트렁크 스캔", timeout_sec)


def run_cart_scan(timeout_sec: int = CART_SCAN_TIMEOUT_SEC) -> dict:
    """`/cart2trunk/cart_scan` 액션을 호출해서 99.py+multiview_scan.py를 실행시키고,
    검출된 박스 JSON과 청크로 받은 PLY 2개(전처리+원본)를 RECEIVED_SCANS_DIR에
    저장한다. 성공 시 {"success": True, "box_count", "json_filename", "ply_filename",
    "ply_total_bytes", "ply_total_chunks", "raw_ply_filename", "raw_ply_total_bytes"}를
    반환하고, 실패 시 RuntimeError를 던진다."""
    return _run_scan_client("cart_scan_client", "카트 스캔", timeout_sec)


def run_pick_and_place(plan_id: str = "", timeout_sec: int = PICK_AND_PLACE_TIMEOUT_SEC) -> dict:
    """`/cart2trunk/pick_and_place` 액션을 호출한다. trunk_scan/cart_scan과 달리
    89.py/99.py를 새로 부팅하는 서브프로세스가 아니라, Isaac Sim PC에서 이미 계속
    떠있는 isaac_task_runner.py의 /isaac_task_runner/run_pick_and_place Trigger
    서비스를 pick_and_place_action_server.py가 대신 호출하는 방식이다(cart2trunk_bridge/
    pick_and_place_action_server.py 참고) - 받을 PLY가 없어서 --output-dir이 필요없다.
    성공 시 {"success": True, "message", "boxes_placed", "boxes_total"}를 반환하고,
    실패 시 RuntimeError를 던진다.

    호출 전에 PICK_AND_PLACE_PROGRESS_FILE을 비워서, read_pick_and_place_progress()가
    이전 실행의 이벤트를 이번 실행 것으로 착각하지 않게 한다."""
    RECEIVED_SCANS_DIR.mkdir(parents=True, exist_ok=True)
    PICK_AND_PLACE_PROGRESS_FILE.write_text("", encoding="utf-8")

    cmd = (
        f"source {shlex.quote(_ROS_SETUP)} && "
        f"source {shlex.quote(_ROS2_WS_SETUP)} && "
        f"ros2 run cart2trunk_bridge pick_and_place_client "
        f"--plan-id {shlex.quote(plan_id)} "
        f"--timeout-sec {timeout_sec} "
        f"--progress-file {shlex.quote(str(PICK_AND_PLACE_PROGRESS_FILE))}"
    )
    return _run_client_subprocess(cmd, "pick_and_place", timeout_sec)


def list_cart_scan_files() -> list:
    """RECEIVED_SCANS_DIR에 저장된 실제 카트 스캔 박스 목록(all_boxes_corners_*.json,
    run_cart_scan()이 저장한 것)의 파일명 목록을 오래된 순으로 반환한다 -
    list_trunk_maps()(algorism_bridge.py)와 같은 관례. "실시간 제어" 탭이 실제
    로봇으로 스캔한 박스 파일을 드롭다운으로 골라 불러올 때 쓴다."""
    if not RECEIVED_SCANS_DIR.exists():
        return []
    return sorted(p.name for p in RECEIVED_SCANS_DIR.glob("all_boxes_corners_*.json"))


def read_pick_and_place_progress() -> list:
    """PICK_AND_PLACE_PROGRESS_FILE(JSON Lines)을 읽어서 이벤트 리스트로
    반환한다. run_pick_and_place()가 아직 한 번도 안 불렸거나 파일이 없으면
    빈 리스트. 쓰는 중(pick_and_place_client가 계속 append)인 마지막 줄이
    잘려 있을 수 있어 그런 줄은 조용히 건너뛴다(다음 폴링에서 완성된 채로
    다시 읽힘)."""
    if not PICK_AND_PLACE_PROGRESS_FILE.exists():
        return []
    events = []
    for line in PICK_AND_PLACE_PROGRESS_FILE.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return events


def send_placement_plan(task_json: dict, timeout_sec: int = 30) -> dict:
    """`/cart2trunk/send_placement_plan` 서비스를 호출해서, 승인된 Task JSON
    (algorism/20_task_export.build_task_json() 출력)을 MSI2의
    results/holonomic_base/placement_result.json에 직접 써넣는다 -
    isaac_task_runner.py의 run_pick_and_place()가 다음 호출 때 이 파일을
    그대로 읽는다(20_task_export.py가 만드는 스키마와 완전히 동일). Isaac Sim
    세션과는 무관한 가벼운 서비스라, isaac_task_runner.py가 꺼져있어도
    먼저 계획만 받아둘 수 있다. 성공 시
    {"success": True, "message", "written_path"}를 반환하고, 실패 시
    (approved=False였거나, 서비스에 연결 못 했거나 등) RuntimeError를 던진다."""
    import tempfile

    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, encoding="utf-8") as f:
        json.dump(task_json, f, ensure_ascii=False)
        tmp_path = f.name

    try:
        cmd = (
            f"source {shlex.quote(_ROS_SETUP)} && "
            f"source {shlex.quote(_ROS2_WS_SETUP)} && "
            f"ros2 run cart2trunk_bridge send_placement_plan_client "
            f"--json-file {shlex.quote(tmp_path)} "
            f"--timeout-sec {timeout_sec}"
        )
        return _run_client_subprocess(cmd, "적재 계획 전송", timeout_sec)
    finally:
        pathlib.Path(tmp_path).unlink(missing_ok=True)
