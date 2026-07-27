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


def _run_scan_client(console_script: str, label: str, timeout_sec: int) -> dict:
    """`ros2 run cart2trunk_bridge <console_script> --output-dir ... --timeout-sec ...`를
    서브프로세스로 실행하고, 마지막 stdout 줄을 JSON으로 파싱해서 반환한다.
    trunk_scan_client/cart_scan_client 둘 다 이 함수를 거친다 - 둘 다 같은
    (success, message, ...) 계약으로 stdout에 JSON 한 줄을 찍기 때문."""
    RECEIVED_SCANS_DIR.mkdir(parents=True, exist_ok=True)

    cmd = (
        f"source {shlex.quote(_ROS_SETUP)} && "
        f"source {shlex.quote(_ROS2_WS_SETUP)} && "
        f"ros2 run cart2trunk_bridge {console_script} "
        f"--output-dir {shlex.quote(str(RECEIVED_SCANS_DIR))} "
        f"--timeout-sec {timeout_sec}"
    )

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

    try:
        result = json.loads(stdout_lines[-1])
    except json.JSONDecodeError as e:
        raise RuntimeError(
            f"{label} 결과를 JSON으로 파싱할 수 없습니다: {stdout_lines[-1]!r} ({e})") from e

    if not result.get("success"):
        raise RuntimeError(result.get("message", f"{label} 실패(원인 불명)"))

    return result


def run_trunk_scan(timeout_sec: int = TRUNK_SCAN_TIMEOUT_SEC) -> dict:
    """`/cart2trunk/trunk_scan` 액션을 호출해서 89.py+90.py를 실행시키고, 청크로
    받은 PLY를 RECEIVED_SCANS_DIR에 저장한다. 성공 시
    {"success": True, "filename", "path", "total_bytes", "total_chunks", "point_count"}를
    반환하고, 실패 시 RuntimeError를 던진다."""
    return _run_scan_client("trunk_scan_client", "트렁크 스캔", timeout_sec)


def run_cart_scan(timeout_sec: int = CART_SCAN_TIMEOUT_SEC) -> dict:
    """`/cart2trunk/cart_scan` 액션을 호출해서 99.py+multiview_scan.py를 실행시키고,
    검출된 박스 JSON과 청크로 받은 PLY를 RECEIVED_SCANS_DIR에 저장한다. 성공 시
    {"success": True, "box_count", "json_filename", "ply_filename",
    "ply_total_bytes", "ply_total_chunks"}를 반환하고, 실패 시 RuntimeError를 던진다."""
    return _run_scan_client("cart_scan_client", "카트 스캔", timeout_sec)
