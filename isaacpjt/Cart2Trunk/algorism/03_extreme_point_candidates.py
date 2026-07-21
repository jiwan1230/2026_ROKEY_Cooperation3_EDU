"""
03_extreme_point_candidates.py
③ Extreme Point 후보 생성
===========================
상태: 🟢 완료·확정

[극점(Extreme Point) 알고리즘이란]
박스를 하나 놓을 때마다, 그 박스의 오른쪽 끝(x+width)/안쪽 끝(y+depth)/
윗쪽 끝(z+height) 세 지점을 "다음 박스를 놓을 수도 있는 후보 자리"로
새로 추가한다. 이사할 때 짐을 하나 넣으면 "이 짐 옆, 이 짐 뒤, 이 짐 위"에
다음 짐을 놓을 수 있겠다고 자연스럽게 후보를 떠올리는 것과 같다.

[왜 층(layer) 쌓기 대신 이 방식을 골랐나]
층 쌓기는 "지금까지 몇 층째인지"같은 절차적 상태를 기억해야 하는데,
극점 방식은 "지금 놓여있는 박스들 + 후보 좌표 집합"만 있으면 언제든
그 상태를 재구성할 수 있다. 이게 트렁크를 매번 재스캔해서 상태를
갱신하는 워크플로우(⑨)와 궁합이 맞는 이유다.
"""

from dataclasses import dataclass, field
from typing import List, Set, Tuple


@dataclass(frozen=True)
class Box:
    """적재 대상 박스 (기획안 9.6절 규격 기준)."""
    id: str
    width: float   # x축 (m)
    depth: float   # y축 (m)
    height: float  # z축 (m)
    mass_kg: float = 0.0
    is_fragile: bool = False

    @property
    def volume(self) -> float:
        return self.width * self.depth * self.height


@dataclass
class PlacedBox:
    """트렁크 안에 실제로 배치된 박스 (좌표 포함)."""
    box: Box
    x: float
    y: float
    z: float

    @property
    def x_range(self) -> Tuple[float, float]:
        return (self.x, self.x + self.box.width)

    @property
    def y_range(self) -> Tuple[float, float]:
        return (self.y, self.y + self.box.depth)

    @property
    def z_range(self) -> Tuple[float, float]:
        return (self.z, self.z + self.box.height)


@dataclass
class ExtremePointState:
    """
    현재까지 배치된 박스들 + 다음 박스를 놓을 수 있는 후보 좌표 집합.
    이 두 가지만 있으면 트렁크 내부 상태를 완전히 재구성할 수 있다.
    """
    placed: List[PlacedBox] = field(default_factory=list)
    candidates: Set[Tuple[float, float, float]] = field(
        default_factory=lambda: {(0.0, 0.0, 0.0)}
    )

    def _slide_to_wall_or_obstacle(self, point: Tuple[float, float, float], slide_axis: int) -> float:
        """
        point를 slide_axis 방향으로 0쪽(벽 쪽)으로 밀었을 때, 가장 먼저 부딪히는
        지점을 반환한다. 그 축 위에서 다른 두 좌표를 가로막는 placed 박스가 없으면
        벽(0.0)까지, 있으면 그 중 point에 가장 가까운 박스의 먼 쪽 면에서 멈춘다.
        """
        coords = list(point)
        target = coords[slide_axis]
        other_axes = [i for i in range(3) if i != slide_axis]

        best = 0.0
        for p in self.placed:
            p_ranges = (p.x_range, p.y_range, p.z_range)
            # 미는 축이 아닌 나머지 두 축에서, point가 이 박스의 범위 안에 걸치는지 확인
            if all(p_ranges[i][0] - 1e-9 <= coords[i] <= p_ranges[i][1] + 1e-9 for i in other_axes):
                far_face = p_ranges[slide_axis][1]
                if far_face <= target + 1e-9 and far_face > best:
                    best = far_face
        return best

    def register_placement(self, placed_box: PlacedBox) -> None:
        """
        박스를 하나 배치한 뒤, 그 박스 기준으로 새 후보 3개(x/y/z축 방향)를 추가한다.
        사용된 후보 좌표는 집합에서 제거한다.

        여기에 더해, 그 3개 모서리 각각을 "자신을 만든 축이 아닌 나머지 두 축" 방향으로
        벽 또는 다른 장애물에 부딪힐 때까지 밀어서 생기는 자리도 추가로 후보에 넣는다.
        장애물 여러 개가 서로 다른 위치에 독립적으로 놓여 있으면, "각 박스 자기 모서리
        3개"만으로는 그 장애물들 사이에 생기는 틈을 못 잡는 경우가 있어서다.
        """
        b = placed_box.box
        used = (placed_box.x, placed_box.y, placed_box.z)
        self.candidates.discard(used)  # 방금 쓴 자리는 더 이상 후보가 아니므로 제거
        self.placed.append(placed_box)  # 배치 완료된 박스 목록에 추가

        # 새로 놓인 박스의 오른쪽(x+width) / 안쪽(y+depth) / 위쪽(z+height) 끝점을
        # 다음 박스가 놓일 수도 있는 새 후보로 추가한다 (③ 극점 알고리즘의 핵심 동작)
        raw_corners = [
            (placed_box.x + b.width, placed_box.y, placed_box.z),   # x축으로 만든 모서리
            (placed_box.x, placed_box.y + b.depth, placed_box.z),   # y축으로 만든 모서리
            (placed_box.x, placed_box.y, placed_box.z + b.height),  # z축으로 만든 모서리
        ]
        for corner in raw_corners:
            self.candidates.add(corner)

        # 각 모서리를 "자신을 만든 축"이 아닌 나머지 두 축 방향으로 밀어서 추가 후보 생성
        for axis_built, corner in enumerate(raw_corners):
            for slide_axis in range(3):
                if slide_axis == axis_built:
                    continue
                slid_value = self._slide_to_wall_or_obstacle(corner, slide_axis)
                if abs(slid_value - corner[slide_axis]) < 1e-9:
                    continue  # 이미 벽/장애물에 붙어 있어서 밀어도 그대로면 새 정보 없음
                slid_corner = list(corner)
                slid_corner[slide_axis] = slid_value
                self.candidates.add(tuple(slid_corner))


def fits_dims(box: Box, trunk) -> bool:
    """박스 자체 크기가 트렁크보다 큰지 여부 (회전은 고려하지 않음 - MVP 범위)."""
    return box.width <= trunk.width and box.depth <= trunk.depth and box.height <= trunk.height


if __name__ == "__main__":
    # 간단 데모: 박스 하나 놓으면 후보가 어떻게 늘어나는지 확인
    state = ExtremePointState()
    print("초기 후보:", state.candidates)

    box = Box("B1", width=0.4, depth=0.3, height=0.25)
    placed = PlacedBox(box=box, x=0.0, y=0.0, z=0.0)
    state.register_placement(placed)
    print("박스 1개 배치 후 후보:", state.candidates)
