"""pipeline_runner.py

89.trunk_scan_holonomic.py / 90.export_trunk_map_holonomic.py를 서브프로세스로
실행하는 헬퍼와, 그 결과 PLY(double 정밀도)를 웹 뷰어 호환성을 위해 float32로
재인코딩하는 헬퍼. 두 원본 스크립트는 여기서 subprocess로만 다루며 파일 내용은
절대 읽거나 수정하지 않는다.
"""
import os
import struct
import subprocess
import time

import numpy as np

_PLY_TYPE_TO_NUMPY = {
    "char": "i1", "int8": "i1",
    "uchar": "u1", "uint8": "u1",
    "short": "i2", "int16": "i2",
    "ushort": "u2", "uint16": "u2",
    "int": "i4", "int32": "i4",
    "uint": "u4", "uint32": "u4",
    "float": "f4", "float32": "f4",
    "double": "f8", "float64": "f8",
}


class PipelineStageError(RuntimeError):
    """89.py/90.py 서브프로세스가 실패했을 때 발생."""


def run_stage(cmd, cwd, extra_env=None, timeout_sec=600, on_line=None):
    """`cmd`(리스트)를 서브프로세스로 실행하고 stdout/stderr를 한 줄씩 on_line()에
    넘긴다. timeout_sec를 넘기면 프로세스를 강제 종료하고 PipelineStageError를
    던진다. 정상 종료했지만 returncode != 0이어도 PipelineStageError를 던진다.
    성공하면 아무것도 반환하지 않는다(예외가 곧 실패 신호)."""
    env = dict(os.environ)
    if extra_env:
        env.update(extra_env)

    proc = subprocess.Popen(
        cmd, cwd=cwd, env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )
    start = time.monotonic()
    tail_lines = []
    try:
        while True:
            line = proc.stdout.readline()
            if line:
                tail_lines.append(line.rstrip("\n"))
                tail_lines[:] = tail_lines[-50:]
                if on_line:
                    on_line(line.rstrip("\n"))
            elif proc.poll() is not None:
                break
            if time.monotonic() - start > timeout_sec:
                proc.kill()
                proc.wait(timeout=10)
                raise PipelineStageError(
                    f"{' '.join(cmd)} 실행이 {timeout_sec}초를 초과해 강제 종료됨")
    finally:
        if proc.stdout:
            proc.stdout.close()

    if proc.returncode != 0:
        raise PipelineStageError(
            f"{' '.join(cmd)} 실패(returncode={proc.returncode}):\n" + "\n".join(tail_lines))


def _parse_ply_header(data: bytes):
    """(header_text, header_end_offset, format_line, elements) 반환.
    elements = [(name, count, [(prop_type, prop_name), ...]), ...]
    리스트 프로퍼티(예: face의 vertex_indices)는 지원하지 않는다 - 만나면
    ValueError."""
    end_marker = b"end_header\n"
    end_idx = data.find(end_marker)
    if end_idx == -1:
        raise ValueError("PLY end_header를 찾을 수 없음")
    header_end = end_idx + len(end_marker)
    header_text = data[:header_end].decode("ascii")
    lines = header_text.splitlines()

    format_line = next((l for l in lines if l.startswith("format")), None)
    if format_line is None or "binary_little_endian" not in format_line:
        raise ValueError(f"지원하지 않는 PLY format: {format_line!r}")

    elements = []
    cur_name, cur_count, cur_props = None, None, None
    for line in lines:
        if line.startswith("element"):
            if cur_name is not None:
                elements.append((cur_name, cur_count, cur_props))
            _, cur_name, count_str = line.split()
            cur_count, cur_props = int(count_str), []
        elif line.startswith("property"):
            parts = line.split()
            if parts[1] == "list":
                raise ValueError("list property(예: face)는 지원하지 않음")
            cur_props.append((parts[1], parts[2]))
    if cur_name is not None:
        elements.append((cur_name, cur_count, cur_props))

    return header_text, header_end, format_line, elements


def convert_ply_double_to_float32(data: bytes) -> bytes:
    """binary_little_endian PLY의 double(float64) 프로퍼티를 float32로 재인코딩한
    새 바이트열을 반환한다. 지원하지 않는 형태(list property 등)를 만나면 원본
    데이터를 그대로 반환한다(변환 실패가 전송 자체를 막지 않도록)."""
    try:
        header_text, header_end, _format_line, elements = _parse_ply_header(data)
    except ValueError:
        return data

    out_header_lines = []
    out_chunks = [None]  # placeholder, header가 먼저 채워짐
    offset = header_end
    body = bytearray()
    had_double = False

    for name, count, props in elements:
        np_dtype = np.dtype([(pname, _PLY_TYPE_TO_NUMPY[ptype]) for ptype, pname in props])
        itemsize = np_dtype.itemsize
        chunk = data[offset:offset + count * itemsize]
        offset += count * itemsize
        arr = np.frombuffer(chunk, dtype=np_dtype, count=count)

        new_fields = []
        for ptype, pname in props:
            if ptype in ("double", "float64"):
                new_fields.append((pname, "f4"))
                had_double = True
            else:
                new_fields.append((pname, _PLY_TYPE_TO_NUMPY[ptype]))
        new_dtype = np.dtype(new_fields)
        new_arr = np.zeros(count, dtype=new_dtype)
        for pname, _ in new_fields:
            new_arr[pname] = arr[pname]
        body += new_arr.tobytes()

    if not had_double:
        return data

    for line in header_text.splitlines(keepends=False):
        if line.startswith("property double") or line.startswith("property float64"):
            line = line.replace("double", "float", 1).replace("float64", "float", 1)
        out_header_lines.append(line)
    new_header = ("\n".join(out_header_lines) + "\n").encode("ascii")

    return new_header + bytes(body)


def parse_ply_vertex_count(data: bytes) -> int:
    """PLY 헤더의 `element vertex N`에서 N을 읽는다. 못 찾으면 -1."""
    end_idx = data.find(b"end_header\n")
    header = data[:end_idx if end_idx != -1 else len(data)].decode("ascii", errors="ignore")
    for line in header.splitlines():
        if line.startswith("element vertex"):
            return int(line.split()[-1])
    return -1


def write_binary_ply_from_xyz(points: np.ndarray) -> bytes:
    """Nx3 포인트 배열을 binary_little_endian PLY 바이트열로 직렬화한다.

    trunk_pointcloud.npy/merged_cart_scan.npy(89.py/99.py가 저장하는 미가공
    포인트클라우드, 둘 다 xyz float32뿐 - color/normal 없음)를 "원본" 뷰용
    PLY로 바꿀 때 쓴다. convert_ply_double_to_float32()의 반대 방향(이쪽은
    "PLY가 이미 있고 double->float32로 재인코딩") - 여기는 "PLY 자체가
    없고 raw numpy 배열에서 새로 만듦"이라는 차이가 있다."""
    points = np.ascontiguousarray(points, dtype=np.float32)
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {len(points)}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n"
    ).encode("ascii")
    return header + points.tobytes()
