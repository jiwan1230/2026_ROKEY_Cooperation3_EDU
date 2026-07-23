"""
trunk_map_planner_node.py
Planner(적재 알고리즘, algorism/) 쪽 ROS2 진입점.

TRUNK_MAP_ROS2_HANDOFF.md 8절 액션 아이템(선욱 담당) 구현: "/cart2trunk/trunk_map"
토픽(std_msgs/String, trunk_map.json을 그대로 JSON 직렬화한 payload)을 구독해서,
algorism/02_trunk_space_state.py의 기존 파서(load_trunk_from_world_map,
load_obstacles_from_world_map)에 dict로 바로 넘긴다 - HANDOFF.md 5절이 명시한 대로
이 파서들은 이미 dict를 받게 되어 있어서 파서 자체는 한 줄도 안 고쳤다(7.2절 스케치
그대로). 이어서 09_rescan_replan.py의 "보수적 가정"(재스캔마다 트렁크를 처음부터
새로 인식) 재계획 로직으로 06~08 파이프라인을 그대로 돌린다.

[QoS - HANDOFF.md 7.1절 MVP안 그대로]
reliable + TRANSIENT_LOCAL(durability) + depth=1. "래치드 토픽"처럼 동작해서, 이
노드가 지완 쪽 퍼블리셔(13.export_trunk_map.py)보다 늦게 켜져도 구독하는 순간
마지막 trunk_map을 바로 받는다. depth=1인 이유: trunk_map은 "diff"가 아니라 항상
"현재 전체 상태"를 통째로 다시 보내는 것으로 설계됐다(HANDOFF.md 6-3항 재스캔
정책과 일치) - 과거 값은 의미가 없다.

[아직 빠진 것 - 알고 있는 한계]
이 노드는 "트렁크 안에 뭐가 있는지"(trunk_map = 바닥/벽/기존 장애물)만 받는다.
"카트에 실제로 뭐가 남아있는지"(적재 대상 박스 목록)는 이 HANDOFF의 범위 밖이라
비전 쪽 박스 검출 결과를 아직 구독하지 않는다 - 지금은 ROS2 파라미터
(cart_boxes_json)로 임시 입력받는다. 비전 박스 검출 토픽이 확정되면 그 구독
콜백을 추가하고 이 파라미터는 fallback(토픽 데이터가 아직 없을 때 데모/디버그용)
으로만 남기면 된다.

[실행]
    python3 trunk_map_planner_node.py
    (선택) --ros-args -p cart_boxes_json:='[{"id":"A","width":0.3,"depth":0.2,"height":0.15}]'
                       -p loading_mode:=count_first -p margin:=0.05

파일 기반으로 먼저 확인하려면(HANDOFF.md 8절 "공통" 액션 아이템):
    python3 trunk_map_planner_node.py --test-file <trunk_map.json 경로> [--mode count_first] [--margin 0.05] [--log-level DEBUG]

[적재 모드 / 마진 선택]
loading_mode 파라미터로 "large_first"(기본값, 큰 것부터+입구 접근성 우선)와
"count_first"(작은 것부터+공간 재사용 우선, 최대한 많은 개수 담기)를 고를 수
있다. margin 파라미터(미지정 시 -1 = 17_margin_check.MARGIN 기본값 사용)로
벽/박스 최소 간격도 조절 가능 (예: 냉동 물류는 냉기 순환용으로 크게).

[판단 로그]
알고리즘이 왜 이 자리를 골랐는지(⑦), 왜 이 박스는 못 실었는지(⑧⑨)는 표준
logging 모듈로 남는다 - 기본은 INFO(박스별 시도/결과만), --log-level DEBUG로
올리면 후보 개수·회전 재시도 같은 내부 판단 과정까지 다 보인다.
"""

import argparse
import json
import logging
import pathlib
import sys
from importlib import import_module

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

_ALGORISM_DIR = str(pathlib.Path(__file__).resolve().parent / "algorism")
if _ALGORISM_DIR not in sys.path:
    sys.path.insert(0, _ALGORISM_DIR)

_m02 = import_module("02_trunk_space_state")
_m03 = import_module("03_extreme_point_candidates")
_m09 = import_module("09_rescan_replan")

load_trunk_from_world_map = _m02.load_trunk_from_world_map
load_obstacles_from_world_map = _m02.load_obstacles_from_world_map
Box = _m03.Box
replan_after_rescan = _m09.replan_after_rescan

TRUNK_MAP_TOPIC = "/cart2trunk/trunk_map"

# 비전 쪽 카트 박스 검출 토픽이 아직 없어서 임시로 쓰는 기본값 - 실제 통합 때는
# ROS2 파라미터(cart_boxes_json)로 덮어쓰거나, 박스 검출 토픽 구독으로 교체한다.
_DEFAULT_CART_BOXES = [
    {"id": "Large", "width": 0.50, "depth": 0.35, "height": 0.30},
    {"id": "Medium", "width": 0.40, "depth": 0.30, "height": 0.25},
    {"id": "Small", "width": 0.30, "depth": 0.20, "height": 0.15},
]


def plan_from_trunk_map_data(
    data: dict, cart_boxes_raw: list, mode: str = "large_first", margin=None,
) -> tuple:
    """
    trunk_map.json(dict)과 카트 박스 목록(dict 리스트)을 받아 (plans, unloadable)을
    반환한다. ROS2 콜백과 --test-file 경로 둘 다 이 함수 하나로 수렴시켜서, 파싱
    이후 로직이 토픽으로 받았든 파일로 받았든 완전히 같게 동작함을 보장한다.
    """
    world_map = load_trunk_from_world_map(data)  # 이미 dict 지원 (HANDOFF.md 5절)
    trunk, offset = world_map.to_bounding_trunk()
    obstacles = load_obstacles_from_world_map(data, offset)
    cart_boxes = [Box(**b) for b in cart_boxes_raw]
    return replan_after_rescan(cart_boxes, trunk, obstacles, mode=mode, margin=margin)


def _log_plan_result(log, data: dict, plans, unloadable, cart_box_count: int) -> None:
    log(
        f"trunk_map 수신 (run_id={data.get('run_id', '?')}) "
        f"-> 배치 {len(plans)}/{cart_box_count}개 성공"
    )
    for p in plans:
        log(f"  PLACED {p.box_id}: pos={p.position} rotated={p.rotated}")
    for u in unloadable:
        log(f"  UNLOADABLE {u.box_id}: {u.reason.value}")


class TrunkMapPlannerNode(Node):
    def __init__(self):
        super().__init__("trunk_map_planner_node")

        self.declare_parameter("cart_boxes_json", "")
        cart_boxes_param = self.get_parameter("cart_boxes_json").value
        self._cart_boxes_raw = (
            json.loads(cart_boxes_param) if cart_boxes_param else _DEFAULT_CART_BOXES
        )
        if not cart_boxes_param:
            self.get_logger().warn(
                "cart_boxes_json 파라미터가 비어 있어서 기본 테스트 박스 3개를 사용합니다 "
                "- 실제 비전 박스 검출 연동 전까지의 임시값입니다."
            )

        self.declare_parameter("loading_mode", "large_first")
        self._loading_mode = self.get_parameter("loading_mode").value
        if self._loading_mode not in ("large_first", "count_first"):
            self.get_logger().warn(
                f"loading_mode='{self._loading_mode}'는 알 수 없는 값 - "
                f"'large_first'로 진행합니다 (large_first/count_first 중 하나여야 함)"
            )
            self._loading_mode = "large_first"

        # -1.0 = "지정 안 함" 센티널 (실제 마진 값은 항상 양수라 안전하게 구분됨) ->
        # None으로 변환해서 17_margin_check.MARGIN 기본값을 그대로 쓰게 한다.
        self.declare_parameter("margin", -1.0)
        margin_param = self.get_parameter("margin").value
        self._margin = margin_param if margin_param >= 0.0 else None

        self.get_logger().info(
            f"적재 정책: loading_mode={self._loading_mode}, "
            f"margin={'기본값' if self._margin is None else self._margin}"
        )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscription = self.create_subscription(
            String, TRUNK_MAP_TOPIC, self._on_trunk_map, qos
        )
        self.get_logger().info(f"{TRUNK_MAP_TOPIC} 구독 시작 (QoS: reliable + transient_local)")

    def _on_trunk_map(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"trunk_map JSON 파싱 실패: {e}")
            return

        plans, unloadable = plan_from_trunk_map_data(
            data, self._cart_boxes_raw, mode=self._loading_mode, margin=self._margin
        )
        _log_plan_result(self.get_logger().info, data, plans, unloadable, len(self._cart_boxes_raw))


def _run_test_file(path: str, mode: str, margin) -> None:
    """ROS2 없이 파일 기반으로 파이프라인만 먼저 확인 (HANDOFF.md 8절 공통 액션 아이템)."""
    data = json.loads(pathlib.Path(path).read_text())
    plans, unloadable = plan_from_trunk_map_data(data, _DEFAULT_CART_BOXES, mode=mode, margin=margin)
    _log_plan_result(print, data, plans, unloadable, len(_DEFAULT_CART_BOXES))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--test-file", help="trunk_map.json 경로 - 지정하면 ROS2 없이 그 파일로 1회만 실행")
    parser.add_argument("--mode", default="large_first", choices=["large_first", "count_first"],
                         help="--test-file과 함께 쓰는 적재 모드 (기본: large_first)")
    parser.add_argument("--margin", type=float, default=None,
                         help="--test-file과 함께 쓰는 마진(m) - 생략하면 기본값(17_margin_check.MARGIN)")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING"],
                         help="판단 로그 상세도 - DEBUG면 후보 개수/회전 재시도까지 다 보임")
    args, ros_args = parser.parse_known_args()

    logging.basicConfig(level=getattr(logging, args.log_level), format="[%(name)s] %(message)s")

    if args.test_file:
        _run_test_file(args.test_file, args.mode, args.margin)
        return

    rclpy.init(args=ros_args)
    node = TrunkMapPlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
