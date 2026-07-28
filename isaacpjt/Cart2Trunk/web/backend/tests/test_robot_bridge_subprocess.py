import pathlib
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import robot_bridge


class _FakeCompletedProcess:
    def __init__(self, stdout="", stderr="", returncode=0):
        self.stdout = stdout
        self.stderr = stderr
        self.returncode = returncode


def _mock_run(monkeypatch, stdout, returncode=0, stderr=""):
    monkeypatch.setattr(
        robot_bridge.subprocess, "run",
        lambda *a, **kw: _FakeCompletedProcess(stdout=stdout, stderr=stderr, returncode=returncode))


def test_parses_json_from_last_line(monkeypatch):
    _mock_run(monkeypatch, '{"success": true, "message": "ok"}\n')
    result = robot_bridge._run_client_subprocess("echo test", "테스트", 30)
    assert result == {"success": True, "message": "ok"}


def test_ignores_ros2run_failure_trailer_after_valid_json(monkeypatch):
    # [회귀 테스트] send_placement_plan_client가 서비스 미연결로 _fail()을 통해
    # 정상적으로 JSON을 찍고 종료 코드 1로 끝나면, `ros2 run`이 그 뒤에 자기
    # 트레일러 메시지를 stdout 마지막 줄로 덧붙인다 - 그 트레일러가 아니라
    # 실제 스크립트가 찍은 JSON을 찾아서 파싱해야 한다(진짜 오류 메시지가
    # 사용자에게 보여야지, "JSON 파싱 실패"라는 무의미한 메시지가 보이면 안 됨).
    stdout = (
        '{"success": false, "message": "서비스(/cart2trunk/send_placement_plan)에 '
        '연결할 수 없습니다"}\n'
        "[ros2run]: Process exited with failure 1\n"
    )
    _mock_run(monkeypatch, stdout, returncode=1)
    try:
        robot_bridge._run_client_subprocess("echo test", "적재 계획 전송", 30)
        assert False, "RuntimeError를 기대했지만 발생하지 않음"
    except RuntimeError as e:
        assert "서비스" in str(e) and "연결할 수 없습니다" in str(e)
        assert "JSON으로 파싱할 수 없습니다" not in str(e)


def test_raises_with_message_when_success_false(monkeypatch):
    _mock_run(monkeypatch, '{"success": false, "message": "실패 이유"}\n')
    try:
        robot_bridge._run_client_subprocess("echo test", "테스트", 30)
        assert False, "RuntimeError를 기대했지만 발생하지 않음"
    except RuntimeError as e:
        assert str(e) == "실패 이유"


def test_raises_clear_error_when_no_line_is_valid_json(monkeypatch):
    _mock_run(monkeypatch, "완전히 무관한 로그 줄\n또 다른 줄\n", returncode=1)
    try:
        robot_bridge._run_client_subprocess("echo test", "테스트", 30)
        assert False, "RuntimeError를 기대했지만 발생하지 않음"
    except RuntimeError as e:
        assert "JSON으로 파싱할 수 없습니다" in str(e)


def test_raises_when_no_stdout_at_all(monkeypatch):
    _mock_run(monkeypatch, "", returncode=1, stderr="파이썬 트레이스백")
    try:
        robot_bridge._run_client_subprocess("echo test", "테스트", 30)
        assert False, "RuntimeError를 기대했지만 발생하지 않음"
    except RuntimeError as e:
        assert "출력이 없습니다" in str(e)


def test_raises_on_timeout(monkeypatch):
    def _raise(*a, **kw):
        raise subprocess.TimeoutExpired(cmd="echo test", timeout=30)
    monkeypatch.setattr(robot_bridge.subprocess, "run", _raise)
    try:
        robot_bridge._run_client_subprocess("echo test", "테스트", 30)
        assert False, "RuntimeError를 기대했지만 발생하지 않음"
    except RuntimeError as e:
        assert "타임아웃" in str(e)
