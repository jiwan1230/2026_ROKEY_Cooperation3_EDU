"""
02_trunk_space_state.py
② 공간 상태 구성
==================
상태: 🔴 보류 — 팀원 데이터 필요 (준형의 트렁크 스캔 결과 대기)

[막힌 지점]
    "실제 스캔 결과로 점유 상태 재구성"이 완료 기준인데, 준형 파트가 나와야
    진짜 검증됨. 지금은 트렁크를 "가로×세로×높이 직육면체 하나 + 이미 놓인
    박스 목록"으로 단순화한 Trunk 클래스만 준비해뒀음.

[지금 확정된 부분]
    Trunk 자료구조 자체는 팀원 데이터와 무관하게 완성 상태.
    실제 스캔 결과가 오면 아래 생성자에 실측값만 넣으면 됨.

[참고 - 실제 저장소(8.rescale_and_rebuild.py)에서 확인한 트렁크 실측값]
    월드 절대좌표 기준:
        TRUNK_X_MIN, TRUNK_X_MAX = 3.11, 3.68
        TRUNK_Y_MIN, TRUNK_Y_MAX = -0.56, 0.56
        TRUNK_FLOOR_Z = 1.03
        TRUNK_WALL_TOP = 1.28
    로컬 치수로 환산하면: width=0.57m, depth=1.12m, height=0.25m
    (단, 이건 지완이 만든 시뮬레이션 씬의 값이라 "실제 스캔 데이터"는 아님 -
     준형의 진짜 스캔 파이프라인이 나오면 이 값을 대체해야 함)
"""

from dataclasses import dataclass, field
from typing import List


@dataclass
class Trunk:
    """
    트렁크 공간 (로컬 좌표계, 원점 (0,0,0) 기준).

    ⚠️ TODO (팀원 데이터 도착 시):
        준형의 실제 스캔 결과(Point Cloud / Occupancy Map)가 오면
        - 단순 직육면체 가정으로 충분한지 재검토
        - 이미 트렁크 안에 있는 물품이 있다면 occupied 리스트로 반영
    """
    width: float   # x축 (m)
    depth: float   # y축 (m)
    height: float  # z축 (m)


# ---- 실제 저장소(8.rescale_and_rebuild.py)에서 확인한 값 (임시 - 준형 스캔 데이터로 교체 예정) ----
WORLD_ORIGIN = (3.11, -0.56, 1.03)  # 로컬 (0,0,0)이 가리키는 월드 좌표 (X_MIN, Y_MIN, FLOOR_Z)

REAL_TRUNK = Trunk(
    width=3.68 - 3.11,   # 0.57
    depth=0.56 - (-0.56),  # 1.12
    height=1.28 - 1.03,   # 0.25
)


def local_to_world(x: float, y: float, z: float):
    """로컬 좌표(0,0,0 시작) -> 월드 절대좌표 변환. Q2 답변 오면 이 함수 사용처 확정."""
    ox, oy, oz = WORLD_ORIGIN
    return (x + ox, y + oy, z + oz)


# TODO: 팀원 스캔 데이터 연동 함수 (아직 미구현)
# def load_trunk_from_scan(scan_result) -> Trunk:
#     """준형의 Point Cloud / Occupancy Map 결과를 Trunk로 변환."""
#     raise NotImplementedError("트렁크 스캔 데이터 연동 방식 확정 후 구현 예정")
