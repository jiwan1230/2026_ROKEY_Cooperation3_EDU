"""
02_trunk_space_state.py
② 공간 상태 구성
==================
상태: 🟡 형식 확정, 실제 연동 로직은 다음 단계 — 준형/지완 답변 반영 (7/20)

[지완 답변 - 트렁크 스캔 데이터 형식]
Point Cloud를 모아서 만든 하나의 "3D World Map"을, Extreme Point 알고리즘에
맞게 가공해서 준다고 함 - 트렁크의 굴곡이나 꺾이는 지점을 점(vertex)과
선(edge)으로 표현한 뼈대(skeleton) 형태로 제공 예정 (스케치 이미지 참고:
빨간 선으로 그려진 트렁크 뼈대 + 그 안에 배치된 박스도 같은 점/선으로 표현).
박스를 하나 넣고 나면 그 박스도 같은 점/선 표현으로 "삽입"해서 트렁크
맵을 갱신하는 그림임 - 이건 정확히 ③의 register_placement()가 하는 일과
개념적으로 같음 (배치 후 상태 갱신).

[지완 답변 - 재스캔 트리거, ⑨와 직결]
박스를 하나 넣을 때마다 잘 들어갔는지 재스캔해서 트렁크 공간을 업데이트하고
다음 작업을 수행하는 방식. 즉 트리거는 "매 박스 배치 1회당 1번" - 별도
조건(위치 변화 임계값 등) 없이 고정 주기로 재스캔함.

[지금 상태]
"점/선으로 표현된 형태"라는 데이터 형식은 확정됐지만, 그 데이터를 실제로
파싱해서 우리 알고리즘이 쓰는 Trunk(현재는 단순 직육면체)로 변환하는
로직은 아직 못 만듦 - 실제 예시 데이터(좌표값이 담긴 실물)가 와야
정확한 파싱 로직을 짤 수 있음. 지금은 그 데이터를 받을 그릇(TrunkWorldMap)
만 미리 준비해두고, 기존 단순 직육면체 Trunk는 MVP fallback으로 유지.
"""

from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class Trunk:
    """
    MVP fallback: 트렁크를 단순 직육면체로 취급하는 현재 버전.
    실제 TrunkWorldMap 파싱 로직이 완성되기 전까지 ③~⑩ 전체가 이걸 사용.
    """
    width: float   # x축 (m)
    depth: float   # y축 (m)
    height: float  # z축 (m)


@dataclass
class TrunkWorldMap:
    """
    준형이 제공할 "점/선으로 표현된 트렁크 뼈대" 데이터를 담을 그릇.

    ⚠️ TODO: 아직 실제 좌표 예시를 못 받아서 vertices/edges의 정확한
    포맷(단위, 좌표계, 정렬 순서 등)은 확정 안 됨. 실제 데이터 오면:
      1. vertices/edges 파싱 로직 작성
      2. 이 굴곡진 형태를 Extreme Point가 쓸 수 있는 형태로 변환
         (단순 직육면체 이상으로 확장 - 오목한 부분/휠하우스 등 반영)
      3. insert_box()로 배치된 박스를 맵에 반영하는 로직 (준형 스케치의
         "삽입" 개념과 대응)
    """
    vertices: List[Tuple[float, float, float]] = field(default_factory=list)
    edges: List[Tuple[int, int]] = field(default_factory=list)  # vertices 인덱스 쌍

    def to_bounding_trunk(self) -> Trunk:
        """
        임시 어댑터: 정밀 파싱 로직이 완성되기 전까지, vertices의 최소/최대
        범위로 단순 직육면체(Trunk)를 근사해서 반환. 굴곡/오목 부분은
        무시됨 - 진짜 데이터로 검증 전까지의 잠정 처리.
        """
        if not self.vertices:
            raise ValueError("TrunkWorldMap에 vertices가 없음")
        xs = [v[0] for v in self.vertices]
        ys = [v[1] for v in self.vertices]
        zs = [v[2] for v in self.vertices]
        return Trunk(width=max(xs) - min(xs), depth=max(ys) - min(ys), height=max(zs) - min(zs))


# ---- 8.rescale_and_rebuild.py 기준 값 (현재 고정 차량 기준 유효, 정밀 World Map 데이터 예정) ----
WORLD_ORIGIN = (3.11, -0.56, 1.03)  # 로컬 (0,0,0)이 가리키는 월드 좌표 (X_MIN, Y_MIN, FLOOR_Z)

REAL_TRUNK = Trunk(
    width=3.68 - 3.11,     # 0.57
    depth=0.56 - (-0.56),  # 1.12
    height=1.28 - 1.03,    # 0.25
)


def local_to_world(x: float, y: float, z: float):
    """로컬 좌표(0,0,0 시작) -> 월드 절대좌표 변환."""
    ox, oy, oz = WORLD_ORIGIN
    return (x + ox, y + oy, z + oz)


# ---- 재스캔 트리거 확정 (지완 답변) ----
# "박스 하나 배치할 때마다 무조건 재스캔" - 조건부 트리거 아님.
RESCAN_TRIGGER_POLICY = "PER_PLACEMENT"  # 박스 1개 놓을 때마다 1회


# TODO: 준형의 실제 World Map 좌표 데이터 연동 (아직 미구현)
# def load_trunk_from_world_map(raw_scan_data) -> TrunkWorldMap:
#     """
#     준형이 줄 점/선 형식 데이터를 TrunkWorldMap으로 파싱.
#     실제 예시 데이터 도착 후 포맷에 맞춰 구현.
#     """
#     raise NotImplementedError("실제 World Map 데이터 샘플 도착 후 구현 예정")