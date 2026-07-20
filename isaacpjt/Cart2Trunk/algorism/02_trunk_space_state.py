"""
02_trunk_space_state.py
② 공간 상태 구성
==================
상태: 🟡 형식 확정, 실제 연동 로직은 다음 단계 — 팀 답변 반영 (7/20)

[좌표계 정리 - Q2 완전히 해소됨]
트렁크·카트·박스(Object3D) 전부 M0609(로봇팔) base 좌표계 하나로 통일해서
온다 (스케치 확인: 로봇이 원점, 트렁크/카트 점들이 그 원점 기준 벡터).
그래서 "누가 좌표를 변환해줄지" 같은 질문 자체가 필요 없어졌음 -
처음부터 끝까지 같은 좌표계.

[그런데 왜 여전히 오프셋 계산이 필요한가]
우리 극점(Extreme Point) 알고리즘(③)은 "박스가 놓일 자리를 (0,0,0)부터
계산"하는 내부 방식을 쓴다. 근데 로봇 원점 기준 트렁크 점들은 (0,0,0)이
아니라 로봇 위치에 따라 음수·양수가 섞인 값으로 온다 (예: 트렁크가
로봇보다 앞쪽·위쪽에 있으면 z는 양수, x는 로봇 기준 앞/뒤에 따라 음수일
수도 있음). 그래서 "트렁크 점들 중 가장 작은 좌표(min corner)"를 우리가
직접 계산해서, 그 지점을 (0,0,0)으로 잡고 나머지 계산은 전부 그 기준
로컬 좌표로 하는 것 - 이건 팀에게 요청할 변환이 아니라 이 파일 안에서
알아서 처리하는 내부 구현 디테일임.

나중에 배치 결과(PlacementPlan)를 로봇 제어(민결)에게 넘길 때는, 이 오프셋을
다시 더해서 M0609 base 좌표계로 되돌려줘야 함 (base_frame_to_local의 역변환).

[준형 답변 - 트렁크 스캔 데이터 형식]
Point Cloud를 모아 만든 "3D World Map"을, 트렁크의 굴곡/꺾이는 지점을
점(vertex)과 선(edge)으로 표현해서 제공 예정. 실제 좌표 샘플은 아직
진행 중 (팀에서 계속 작업 중).

[지완 답변 - 재스캔 트리거]
박스 1개 배치할 때마다 무조건 재스캔 ("PER_PLACEMENT" 고정 주기).
"""

from dataclasses import dataclass, field
from typing import List, Tuple


@dataclass
class Trunk:
    """
    내부 계산용 트렁크 표현 - 항상 (0,0,0)을 한쪽 코너로 하는 로컬 좌표계.
    실제 M0609 base 좌표계 데이터를 받으면 TrunkWorldMap.to_bounding_trunk()로
    변환해서 이 형태로 만든다.
    """
    width: float   # x축 (m)
    depth: float   # y축 (m)
    height: float  # z축 (m)


@dataclass
class TrunkWorldMap:
    """
    준형이 제공할 "M0609 base 좌표계 기준, 점/선으로 표현된 트렁크 뼈대" 데이터.

    ⚠️ TODO: 실제 좌표 샘플이 아직 도착 전 (준형 작업 진행 중). vertices의
    정확한 포맷(단위, 정렬 순서 등)은 실제 데이터 오면 확정.
    """
    vertices: List[Tuple[float, float, float]] = field(default_factory=list)  # M0609 base 좌표계 기준
    edges: List[Tuple[int, int]] = field(default_factory=list)  # vertices 인덱스 쌍

    def to_bounding_trunk(self) -> Tuple[Trunk, Tuple[float, float, float]]:
        """
        M0609 base 좌표계의 vertices를 받아서:
          1. 우리 알고리즘이 쓸 로컬 Trunk(0,0,0 코너 기준)로 변환
          2. 그 변환에 쓴 오프셋(= M0609 base 좌표계에서 로컬 원점이 실제로
             어디였는지)을 같이 반환 - 나중에 로봇에게 좌표를 돌려줄 때 필요

        굴곡/오목한 부분은 지금은 무시하고 최소/최대 범위로 근사 (정밀
        형태 반영은 실제 데이터로 검증하며 다음 단계에서 확장).
        """
        if not self.vertices:
            raise ValueError("TrunkWorldMap에 vertices가 없음")

        xs = [v[0] for v in self.vertices]
        ys = [v[1] for v in self.vertices]
        zs = [v[2] for v in self.vertices]

        offset = (min(xs), min(ys), min(zs))  # M0609 base 좌표계 기준 로컬 원점 위치
        trunk = Trunk(width=max(xs) - min(xs), depth=max(ys) - min(ys), height=max(zs) - min(zs))
        return trunk, offset


def local_to_base_frame(x: float, y: float, z: float, offset: Tuple[float, float, float]):
    """
    로컬 좌표(0,0,0 코너 기준) -> M0609 base 좌표계로 되돌리는 변환.
    PlacementPlan을 로봇 제어(민결)에게 넘길 때 이 함수로 되돌려서 전달해야 함.
    """
    ox, oy, oz = offset
    return (x + ox, y + oy, z + oz)


# ---- 8.rescale_and_rebuild.py 시뮬레이션 씬 기준 값 (MVP fallback, 실제 World Map 데이터 예정) ----
# 참고: 이 값은 준형의 실제 M0609 base 좌표계 스캔 데이터가 아니라, 지완이 만든
# 시뮬레이션 씬에서 뽑은 값이라 임시로만 사용.
REAL_TRUNK = Trunk(
    width=3.68 - 3.11,     # 0.57
    depth=0.56 - (-0.56),  # 1.12
    height=1.28 - 1.03,    # 0.25
)
REAL_TRUNK_OFFSET = (3.11, -0.56, 1.03)  # 위 REAL_TRUNK를 만들 때 쓴 로컬 원점 오프셋 (임시값)


# ---- 재스캔 트리거 확정 (지완 답변) ----
RESCAN_TRIGGER_POLICY = "PER_PLACEMENT"  # 박스 1개 놓을 때마다 1회


# TODO: 준형의 실제 World Map 좌표 데이터 연동 (샘플 데이터 도착 대기 중)
# def load_trunk_from_world_map(raw_scan_data) -> TrunkWorldMap:
#     """준형이 줄 점/선 형식 데이터를 TrunkWorldMap으로 파싱. 실제 샘플 도착 후 구현."""
#     raise NotImplementedError("실제 World Map 데이터 샘플 도착 후 구현 예정")


if __name__ == "__main__":
    # 데모: M0609 base 좌표계 기준(음수 포함) 트렁크 점들이 왔을 때 오프셋 계산 확인
    demo_map = TrunkWorldMap(vertices=[
        (-0.3, -0.5, 0.9), (0.27, -0.5, 0.9), (-0.3, 0.5, 0.9), (0.27, 0.5, 0.9),
        (-0.3, -0.5, 1.15), (0.27, -0.5, 1.15), (-0.3, 0.5, 1.15), (0.27, 0.5, 1.15),
    ])
    trunk, offset = demo_map.to_bounding_trunk()
    print("로컬 Trunk:", trunk)
    print("M0609 base 기준 오프셋:", offset)
    print("로컬 (0.1,0.1,0.1)을 base frame으로 되돌리면:", local_to_base_frame(0.1, 0.1, 0.1, offset))