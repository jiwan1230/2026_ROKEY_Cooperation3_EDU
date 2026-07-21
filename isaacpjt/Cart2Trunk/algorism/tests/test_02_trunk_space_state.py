"""
test_02_trunk_space_state.py
② to_bounding_trunk()이 "입구 방향"을 로봇 base 원점 기준으로 올바르게 추정하는지 검증.

배경: 지금까지 알고리즘은 트렁크의 어느 쪽이 입구인지 전혀 모르고 무조건 로컬 (0,0,0)부터
채워나갔다. 그런데 실제 트렁크 데이터에서 로컬 (0,0,0) 코너가 "우연히" 로봇(=입구) 쪽인지
반대쪽인지는 스캔마다 다를 수 있다 (to_bounding_trunk이 그냥 min(x,y,z) 코너를 원점으로
잡기 때문). 로봇 base 좌표계에서는 로봇 팔 자신이 원점(0,0,0)이므로, 트렁크의 두 변(로컬
0쪽 / 로컬 max쪽) 중 그 원점에 더 가까운 쪽이 곧 로봇이 접근하는 입구 쪽이라고 볼 수 있다.
이 테스트는 그 추정 로직이 양쪽 케이스(입구가 로컬 원점 쪽/반대쪽) 모두에서 맞는지 확인한다.
"""
import sys, pathlib
from importlib import import_module

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))  # tests/ -> algorism/
_m02 = import_module("02_trunk_space_state")

TrunkWorldMap = _m02.TrunkWorldMap


def test_entrance_near_local_origin_when_trunk_is_positive_offset_from_robot():
    """
    트렁크 전체가 로봇 원점(0,0) 기준 x:[0.5,1.0], y:[0.5,1.0] 범위에 있으면,
    로컬 (0,0,0) 코너(=base 기준 (0.5,0.5,..))가 반대쪽 코너(=base 기준 (1.0,1.0,..))보다
    로봇에 더 가깝다 -> 입구는 로컬 원점 쪽(entrance_near_x/y=True)이어야 한다.
    """
    world_map = TrunkWorldMap(vertices=[
        (0.5, 0.5, 0.0), (1.0, 0.5, 0.0), (0.5, 1.0, 0.0), (1.0, 1.0, 0.3),
    ])
    trunk, offset = world_map.to_bounding_trunk()

    assert trunk.entrance_near_x is True
    assert trunk.entrance_near_y is True


def test_entrance_near_far_corner_when_trunk_is_negative_offset_from_robot():
    """
    트렁크 전체가 로봇 원점 기준 x:[-1.0,-0.5], y:[-1.0,-0.5] 범위(둘 다 음수)에 있으면,
    로컬 max쪽 코너(=base 기준 (-0.5,-0.5,..))가 로컬 원점 코너(=base 기준 (-1.0,-1.0,..))보다
    로봇에 더 가깝다 -> 입구는 로컬 원점 반대쪽(entrance_near_x/y=False)이어야 한다.
    """
    world_map = TrunkWorldMap(vertices=[
        (-1.0, -1.0, 0.0), (-0.5, -1.0, 0.0), (-1.0, -0.5, 0.0), (-0.5, -0.5, 0.3),
    ])
    trunk, offset = world_map.to_bounding_trunk()

    assert trunk.entrance_near_x is False
    assert trunk.entrance_near_y is False
