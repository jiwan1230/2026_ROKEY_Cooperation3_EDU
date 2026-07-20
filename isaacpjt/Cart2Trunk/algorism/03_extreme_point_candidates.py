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

    def register_placement(self, placed_box: PlacedBox) -> None:
        """
        박스를 하나 배치한 뒤, 그 박스 기준으로 새 후보 3개(x/y/z축 방향)를 추가한다.
        사용된 후보 좌표는 집합에서 제거한다.
        """
        b = placed_box.box
        used = (placed_box.x, placed_box.y, placed_box.z)
        self.candidates.discard(used)
        self.placed.append(placed_box)

        self.candidates.add((placed_box.x + b.width, placed_box.y, placed_box.z))
        self.candidates.add((placed_box.x, placed_box.y + b.depth, placed_box.z))
        self.candidates.add((placed_box.x, placed_box.y, placed_box.z + b.height))


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
