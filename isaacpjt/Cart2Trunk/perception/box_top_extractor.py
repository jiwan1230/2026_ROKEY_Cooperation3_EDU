#!/usr/bin/env python3
"""
실행 방법
----
Isaac Sim은 isaac_sim 자체 python(rclpy/cv_bridge 없음)으로 돌리고,
이 노드는 시스템 python + 이 폴더의 venv로 별도 터미널에서 돌린다.

1) Isaac Sim에서 box_scan_test_scene.usd를 열어 재생한다
   (ROS2 bridge가 /camera/depth, /camera/camera_info를 publish해야 한다).
2) 이 노드는 별도 터미널에서:
     source /opt/ros/humble/setup.bash
     source isaacpjt/Cart2Trunk/perception/.venv/bin/activate   (최초 1회: 아래 venv 생성 참고)
     python3 isaacpjt/Cart2Trunk/perception/box_top_extractor.py
3) 화면에서 S: 현재 프레임의 전체 박스 8꼭짓점+복원 표면을
   ~/box_pointcloud/all_boxes_corners_*.json, all_boxes_completed_*.ply로 저장. Q: 종료.

venv 최초 생성 (1회, apt의 rclpy/cv_bridge를 --system-site-packages로 그대로 쓰되
pip numpy 2.x가 cv_bridge를 깨는 문제는 venv 안에 numpy<2를 별도로 고정해서 피한다):
     python3 -m venv --system-site-packages isaacpjt/Cart2Trunk/perception/.venv
     source isaacpjt/Cart2Trunk/perception/.venv/bin/activate
     pip install -r isaacpjt/Cart2Trunk/perception/requirements.txt

순수 Depth 기반 박스 윗면 검출 + 8개 꼭짓점 추출 (다중 박스 지원).

가정
----
카메라가 박스 적재물을 위쪽에서 바라본다.

방법
----
1. Depth Map을 XYZ Point Cloud로 변환한다.
2. Open3D RANSAC으로 평면들을 반복 검출한다.
3. 카메라를 향하는 평면만 박스 윗면 후보로 사용한다.
4. 같은 평면의 떨어진 영역을 DBSCAN으로 분리한다.
5. 직사각형에 가깝고 크기가 충분한 후보만 남긴다.
6. 각 윗면 후보마다 아래 방향으로 ray를 쏴서, 법선이 평행한 다른 박스
   윗면(또는 없으면 바닥)을 지지면으로 찾는다.
7. 윗면 4점 + 지지면 교점 4점 = 8개 꼭짓점을 모든 박스에 대해 계산한다.
8. S 입력 시 전체 박스의 8개 꼭짓점을 JSON으로, 복원된 표면 Point Cloud를
   PLY로 저장한다.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
import os
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import open3d as o3d
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import Pose, PoseArray, Point
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


# ============================================================
# ROS 2 토픽
# ============================================================

DEPTH_TOPIC = "/camera/depth"
CAMERA_INFO_TOPIC = "/camera/camera_info"

SCENE_CLOUD_TOPIC = "/depth/scene_cloud"
TOPMOST_BOX_CLOUD_TOPIC = "/depth/topmost_box_surface_cloud"
TOPMOST_BOX_COMPLETED_CLOUD_TOPIC = "/depth/topmost_box_completed_cloud"
BOX_CORNERS_TOPIC = "/depth/topmost_box_corners"
BOX_MARKERS_TOPIC = "/depth/topmost_box_markers"
DEBUG_IMAGE_TOPIC = "/depth/topmost_box_debug"


# ============================================================
# Depth 설정
# ============================================================

MIN_DEPTH_M = 0.10
MAX_DEPTH_M = 5.00


def _parse_image_roi() -> tuple[float, float, float, float]:
    raw = os.environ.get("CART2TRUNK_IMAGE_ROI", "0.15,0.15,0.85,0.85")
    try:
        values = [float(item) for item in raw.split(",")]
    except ValueError:
        values = [0.15, 0.15, 0.85, 0.85]

    if len(values) != 4:
        values = [0.15, 0.15, 0.85, 0.85]

    return tuple(values)  # type: ignore[return-value]


# ROI를 중앙부로 좁혀 배경 오탐을 줄이고, 박스 윗면이 더 잘 맞도록 한다.
# 기본값은 새 Vision 패스에서 검증한 값(0.15, 0.15, 0.85, 0.85)이다.
IMAGE_ROI = _parse_image_roi()


def _parse_roi_base_bounds(env_name: str, default: str) -> Optional[np.ndarray]:
    """"x_min,y_min,z_min" 또는 "x_max,y_max,z_max" 형태의 3-tuple을 읽는다.
    빈 문자열("")을 넣으면 그 축의 제한을 끈다(None 반환 = 크롭 비활성화)."""
    raw = os.environ.get(env_name, default)
    if raw.strip() == "":
        return None
    try:
        values = [float(item) for item in raw.split(",")]
    except ValueError:
        values = [float(item) for item in default.split(",")]
    if len(values) != 3:
        values = [float(item) for item in default.split(",")]
    return np.asarray(values, dtype=np.float64)


# ============================================================
# 3D ROI (base_link 좌표계, 카트 내부 볼륨으로 제한)
# ============================================================
# 88.cart_scan_holonomic.py 실측(2026-07-24) - 카트 옆면 스캔에서 진짜 박스 2개 외에
# 카트 자체(와이어 메쉬, 손잡이, 바퀴)가 5개의 가짜 "박스"로 오검출됐다. 오검출된 것들은
# 전부 "카트 테두리 높이 -> 실제 바닥까지 거리"가 거의 동일(~0.43m)한 값으로 나왔는데,
# 이는 카트 구조물의 여러 지점이 윗면 후보로 잘못 잡히고 그 아래 진짜 바닥까지
# ray-cast되어 생긴 특징이다(RANSAC/서포트 탐지 이후에는 걸러내기 어렵다). 그래서 PDF
# 요구 파이프라인 순서("Depth -> Point Cloud 변환 -> ROI 적용 -> ...")대로, RANSAC 전에
# base_link 좌표계에서 "카트 바스켓 내부"로 point cloud 자체를 먼저 크롭한다.
# 기본값은 88.py 실측 데이터(진짜 박스 중심 x=-0.11~0.17, y=0.63~0.66, z=-0.06~-0.05)에
# 여유를 두고 넉넉하게 잡은 값 - 실제 카트/스캔 자세에 맞게 환경변수로 조정한다.
ROI_BASE_MIN = _parse_roi_base_bounds("CART2TRUNK_ROI_BASE_MIN", "-0.35,0.40,-0.15")
ROI_BASE_MAX = _parse_roi_base_bounds("CART2TRUNK_ROI_BASE_MAX", "0.35,0.90,0.15")


# ============================================================
# Open3D 전처리
# ============================================================

VOXEL_SIZE_M = 0.005
OUTLIER_NB_NEIGHBORS = 20
OUTLIER_STD_RATIO = 2.0


# ============================================================
# RANSAC 평면 검출
# ============================================================

# 40번(적층/조밀 배치) 실측: RGB 프레임으로 가림/ROI 크롭이 아님을 확인한 뒤에도
# Medium/Large의 윗면이 fill_ratio 0.52~0.59(새 0.60 임계값 바로 아래)로 자꾸
# 기각되는 걸 디버그 로그로 추적한 결과, 실제로는 가려진 게 아니라 depth 노이즈가
# 이 6mm 임계값을 국소적으로 넘는 부분에서 RANSAC이 같은 물리적 평면을 서로 다른
# 반복(iteration)으로 쪼개버리고 있었다 - 쪼개진 조각은 DBSCAN도 따로 돌아서
# 완전한 직사각형으로 합쳐지지 못하고 낮은 fill_ratio로 기각된다. 소량만 늘려서
# (0.006->0.008) 분할 자체를 줄인다 - 원래 오탐 억제 목적(옆면/다른 높이 표면 분리)은
# TOP_NORMAL_CONSISTENCY_DOT_MIN 등 다른 필터가 계속 담당하므로 과도하게 풀 필요는 없다.
#
# 44번(박스 개수 가변화) 실측: 지터로 기존 "Large"보다 더 큰 박스(0.2554x0.1799)가
# 나온 실행에서, 그 윗면이 다시 두 조각(footprint 0.2554x0.0979와 0.2824x0.1573)으로
# 쪼개져 둘 다 알려진 박스 크기 허용치(0.025m)를 살짝 못 미치는 차이(0.027m)로
# 기각됐다 - 같은 평면 분할 문제가 "더 큰 평평한 표면일수록" 8mm로도 마진이
# 빠듯하다는 뜻이므로, 같은 원칙으로 한 번 더 소량만 늘린다(0.008->0.010).
PLANE_DISTANCE_THRESHOLD_M = float(
    os.environ.get("CART2TRUNK_PLANE_DISTANCE_THRESHOLD_M", "0.010")
)
PLANE_RANSAC_N = 3
PLANE_RANSAC_ITERATIONS = 500

MIN_PLANE_POINTS = 150
MAX_PLANES = 12

# 카메라 optical frame의 전방축은 +Z.
#
# 1차 필터:
# 카메라를 어느 정도 바라보는 평면만 "대표 윗면 법선" 후보로 사용한다.
# 기존 0.82보다 완화하되, 이 값만으로 최종 윗면을 결정하지 않는다.
CAMERA_FACING_NORMAL_DOT_MIN = 0.70

# 2차 필터:
# 먼저 검출된 평면들 중 대표 윗면 법선을 정한 뒤,
# 대표 법선과 거의 평행한 평면만 실제 박스 윗면 후보로 사용한다.
#
# 0.94는 약 20도 이내의 법선 차이만 허용한다.
# 따라서 카메라가 조금 기울어져도 윗면은 유지하고,
# 윗면과 거의 직각인 옆면(또는 다른 높이의 박스/테이블면)은 제외할 수 있다.
TOP_NORMAL_CONSISTENCY_DOT_MIN = 0.94

# 대표 법선 선택 시 평면 Point 수와 카메라 정면성을 함께 반영한다.
REFERENCE_NORMAL_ALIGNMENT_POWER = 2.0


# ============================================================
# 평면 내부의 박스 윗면 분리
# ============================================================

DBSCAN_EPS_M = 0.025
DBSCAN_MIN_POINTS = 15
MIN_CLUSTER_POINTS = 70

MIN_BOX_SIDE_M = 0.04
MAX_BOX_SIDE_M = 1.50

# minAreaRect 면적 중 실제 Point가 차지하는 비율.
# 너무 낮으면 긴 띠, 불규칙 배경일 가능성이 높다.
# 40번 Vision 패스가 처음엔 0.60으로 강하게 걸렀으나, 실측(RGB로 가림/ROI 크롭이
# 아님을 확인)+디버그 로그로 Medium/Large의 정상적인(가려지지 않은) 윗면이 depth
# 노이즈발 RANSAC 평면 분할 때문에 fill_ratio 0.52~0.59로 떨어져 매번 기각되는 걸
# 확인했다 - 그 범위를 포괄하도록 0.50으로 낮춘다(PLANE_DISTANCE_THRESHOLD_M 소폭
# 완화와 함께 적용 - 분할 자체도 줄이고, 그래도 남는 약간의 분할은 여기서 흡수).
MIN_RECTANGULAR_FILL_RATIO = float(
    os.environ.get("CART2TRUNK_MIN_RECTANGULAR_FILL_RATIO", "0.50")
)

# 너무 가느다란 평면 제외
MAX_SIDE_ASPECT_RATIO = 8.0

# select_support_candidate/detect_floor_boundary가 왜 거부했는지 코너별 수치를
# 자세히 찍어본다 (적층 시나리오에서 윗면은 잡히는데 지지면 매칭에서 탈락하는
# 원인을 실측으로 확인하기 위한 진단용 - 평소엔 꺼둔다).
DEBUG_SUPPORT = os.environ.get("CART2TRUNK_DEBUG_SUPPORT", "0") == "1"


# ============================================================
# 8개 꼭짓점 계산
# ============================================================

# 최상단 윗면과 아래 후보 사이의 최소 Depth 차이
MIN_SUPPORT_DEPTH_GAP_M = 0.025

# 두 평면 법선이 평행하다고 판단할 최소 내적 절댓값
SUPPORT_NORMAL_DOT_MIN = 0.90

# 아래 후보가 최상단 후보 주변에 있다고 판단하기 위한 영상 확장량
SUPPORT_SEARCH_DILATION_PX = 45

# 확장된 최상단 영역과 아래 후보 영역의 최소 겹침 비율
MIN_SUPPORT_OVERLAP_RATIO = 0.02

# 실제 지지면보다 이만큼 위쪽에 아랫면 꼭짓점을 생성한다.
# 접촉 경계 노이즈를 피하기 위한 값이다.
BOTTOM_CUT_MARGIN_M = 0.010

# 초록색 윗면 꼭짓점에서 아래 방향으로 ray를 내릴 때의 설정
MIN_RAY_DISTANCE_M = 0.020
MAX_RAY_DISTANCE_M = 1.00

# 네 꼭짓점에서 계산된 경계 깊이가 지나치게 다르면 제외
MAX_RAY_DISTANCE_SPREAD_M = 0.060

# ray 교점과 후보 평면의 실제 Point Cloud 사이 허용 거리
BOUNDARY_HIT_DISTANCE_M = 0.040

# 네 ray 중 최소 몇 개가 후보 평면 근처를 지나야 하는지
MIN_BOUNDARY_RAY_HITS = 3

# 아래 박스 윗면 후보가 없을 때 바닥을 찾기 위한 설정
FLOOR_RANSAC_MAX_PLANES = 8
FLOOR_MIN_POINTS = 300
FLOOR_DISTANCE_THRESHOLD_M = 0.012
FLOOR_RANSAC_ITERATIONS = 1200
FLOOR_NORMAL_DOT_MIN = 0.94
FLOOR_MIN_AREA_M2 = 0.12
# 40번(적층 시나리오) 실측: 스택된 박스(Large 밑, Small 위) 주변은 다른 박스들과
# 더 촘촘하게 배치되고 위쪽 박스에 가려져서, Large 테두리의 실제 지지면(테이블)은
# 법선/높이 전부 정확히 검출되는데도 그 코너 근처에 "실제로 관측된" 테이블 포인트가
# 0.12m보다 훨씬 멀리(0.16~0.25m) 떨어져 있어 매번 기각되는 것을 CART2TRUNK_DEBUG_SUPPORT=1
# 진단 로그로 직접 확인했다(no lower boundary or floor). 원래 0.12m는 박스가 서로
# 떨어져 있던 이전 배치용 값 - 지지 평면 자체(법선/높이/기울기)는 이미 검증되므로,
# 그 평면 위의 관측 포인트가 조금 멀리 있어도(가려짐) 받아들이도록 완화한다.
FLOOR_BOUNDARY_HIT_DISTANCE_M = float(
    os.environ.get("CART2TRUNK_FLOOR_BOUNDARY_HIT_DISTANCE_M", "0.30")
)

# 현실적인 박스 높이 범위
MIN_BOX_HEIGHT_M = 0.035
MAX_BOX_HEIGHT_M = 1.00

# 초록색 윗면 footprint에서 옆면까지 포함하기 위한 여유
OBJECT_FOOTPRINT_MARGIN_M = 0.025

# 8개 꼭짓점이 만드는 육면체의 각 면 바깥쪽으로 허용할 오차
# Depth 노이즈 때문에 실제 표면 Point가 경계 밖으로 약간 벗어나는 것을 허용한다.
BOX_HULL_TOLERANCE_M = 0.010

# 8개 꼭짓점으로 보이지 않는 면까지 복원할 때의 Point 간격
COMPLETED_SURFACE_POINT_SPACING_M = 0.005

# 중복 Point를 정리할 Voxel 크기
COMPLETED_SURFACE_VOXEL_SIZE_M = 0.003

# 윗면보다 위로 튀는 작은 Depth 노이즈 허용값
TOP_PLANE_TOLERANCE_M = 0.012

# 경계 절단 후 Point Cloud 정리
OBJECT_VOXEL_SIZE_M = 0.004
OBJECT_OUTLIER_NB_NEIGHBORS = 20
OBJECT_OUTLIER_STD_RATIO = 2.0
MIN_OBJECT_POINTS = 80

# Marker 크기
CORNER_MARKER_SCALE_M = 0.018
EDGE_MARKER_SCALE_M = 0.006


# ============================================================
# 출력
# ============================================================

SCENE_PUBLISH_STRIDE = 3
TOP_PUBLISH_STRIDE = 1

SAVE_DIRECTORY = Path.home() / "box_pointcloud"
SAVE_DIRECTORY.mkdir(parents=True, exist_ok=True)

LOG_INTERVAL_FRAMES = 10

# 저장 좌표계. 14_run_full_pipeline.py(algorism/)의 load_boxes_from_vision_json()이
# 정확히 이 문자열이 아니면 바로 에러를 낸다 (팀 계약).
OUTPUT_FRAME = "m0609_base_link"

# 32.box_table_scan_setup.py가 고정 스캔 자세에서 측정해서 저장한 카메라->base_link 변환.
# 팔이 그 스크립트가 만든 자세를 벗어나면 이 값은 더 이상 유효하지 않다 - 재측정 필요.
BASE_TO_CAMERA_TRANSFORM_PATH = Path(__file__).resolve().parent / "base_to_camera_transform.json"


def load_base_to_camera_transform() -> tuple[np.ndarray, np.ndarray]:
    """base_link->camera 변환(R, t)을 읽는다. 카메라 좌표계 점 p_cam을
    p_base = p_cam @ R.T + t 로 base_link 좌표계로 옮기는 데 쓴다."""
    if not BASE_TO_CAMERA_TRANSFORM_PATH.exists():
        raise RuntimeError(
            f"{BASE_TO_CAMERA_TRANSFORM_PATH} 가 없습니다. "
            "먼저 32.box_table_scan_setup.py를 실행해서 고정 스캔 자세의 "
            "base_link->camera 변환을 만들어야 합니다 "
            f"(저장 좌표계가 반드시 {OUTPUT_FRAME!r}이어야 하기 때문)."
        )
    data = json.loads(BASE_TO_CAMERA_TRANSFORM_PATH.read_text())
    R = np.asarray(data["R"], dtype=np.float64)
    t = np.asarray(data["t"], dtype=np.float64)
    return R, t


@dataclass
class PlaneClusterCandidate:
    candidate_id: int
    points: np.ndarray

    normal: np.ndarray
    median_depth: float

    width: float
    height: float
    area: float
    fill_ratio: float

    center: np.ndarray
    plane_d: float
    corners_3d: np.ndarray
    pixel_polygon: Optional[np.ndarray]


class DepthTopmostBoxExtractor(Node):
    def __init__(self) -> None:
        super().__init__("depth_topmost_box_extractor")

        # 좌표계 변환은 시작 시점에 미리 읽어서, 못 찾으면 스캔을 아예 시작하지 않고
        # 바로 종료한다 (카메라 좌표계로 조용히 저장해버리는 것을 막기 위함).
        self._base_R, self._base_t = load_base_to_camera_transform()
        self.get_logger().info(
            f"[좌표 변환] {BASE_TO_CAMERA_TRANSFORM_PATH} 로드 완료 "
            f"(저장 좌표계: {OUTPUT_FRAME})"
        )

        self.bridge = CvBridge()
        self.frame_count = 0

        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None

        # depth_to_points() 직후, 전처리 전의 raw point cloud(카메라 좌표계) - 여러
        # 프레임을 합쳐서 한 번만 검출하는 누적 모드(run_scan_once_accumulate.py)가
        # 프레임마다 이걸 읽어서 쌓는다. 매 프레임 그냥 덮어써질 뿐이라 기존 단일
        # 프레임 검출 동작에는 전혀 영향 없다.
        self.latest_raw_scene_points = np.empty((0, 3), dtype=np.float32)
        self.latest_header = None  # 누적 모드가 합친 point cloud를 퍼블리시할 때 쓸 (frame_id, stamp)

        # 현재 프레임에서 복원에 성공한 모든 박스 결과
        # 각 원소:
        # {
        #   "box_id": int,
        #   "top": PlaneClusterCandidate,
        #   "support": PlaneClusterCandidate,
        #   "support_type": "box_top" | "floor",
        #   "corners": (8, 3),
        #   "completed_points": (N, 3),
        # }
        self.latest_boxes = []

        # ROS 시각화용 통합 Point Cloud
        self.latest_all_top_points = np.empty(
            (0, 3),
            dtype=np.float32,
        )
        self.latest_all_completed_points = np.empty(
            (0, 3),
            dtype=np.float32,
        )

        output_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.scene_cloud_publisher = self.create_publisher(
            PointCloud2,
            SCENE_CLOUD_TOPIC,
            output_qos,
        )

        self.top_cloud_publisher = self.create_publisher(
            PointCloud2,
            TOPMOST_BOX_CLOUD_TOPIC,
            output_qos,
        )

        self.completed_cloud_publisher = self.create_publisher(
            PointCloud2,
            TOPMOST_BOX_COMPLETED_CLOUD_TOPIC,
            output_qos,
        )

        self.corners_publisher = self.create_publisher(
            PoseArray,
            BOX_CORNERS_TOPIC,
            output_qos,
        )

        self.markers_publisher = self.create_publisher(
            MarkerArray,
            BOX_MARKERS_TOPIC,
            output_qos,
        )

        self.debug_image_publisher = self.create_publisher(
            Image,
            DEBUG_IMAGE_TOPIC,
            output_qos,
        )

        self.depth_subscription = self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self.depth_callback,
            qos_profile_sensor_data,
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Open3D version: {o3d.__version__}"
        )
        self.get_logger().info(
            f"Top surface cloud: {TOPMOST_BOX_CLOUD_TOPIC}"
        )
        self.get_logger().info(
            f"Completed box cloud: {TOPMOST_BOX_COMPLETED_CLOUD_TOPIC}"
        )
        self.get_logger().info(
            f"8 corners: {BOX_CORNERS_TOPIC}"
        )
        self.get_logger().info(
            f"corner markers: {BOX_MARKERS_TOPIC}"
        )
        self.get_logger().info(
            "Processing: every detected box-top candidate"
        )
        self.get_logger().info(
            "Normal filter: "
            f"camera>={CAMERA_FACING_NORMAL_DOT_MIN:.2f}, "
            f"reference parallel>={TOP_NORMAL_CONSISTENCY_DOT_MIN:.2f}"
        )
        self.get_logger().info(
            "S: save one combined JSON and one combined PLY, Q: quit"
        )

    # ========================================================
    # ROS callbacks
    # ========================================================

    def camera_info_callback(
        self,
        msg: CameraInfo,
    ) -> None:
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def process_scene_cloud(self, scene_pcd, header, depth_shape=(0, 0)):
        """전처리된 point cloud 하나에서 박스 후보를 검출하고, 각 후보의 받침면/
        8꼭짓점까지 복원해서 self.latest_boxes/latest_all_top_points/
        latest_all_completed_points를 채우고 관련 토픽을 퍼블리시한다.

        depth_callback(프레임 하나)과 누적 모드(run_scan_once_accumulate.py, 여러
        프레임을 합친 point cloud 하나)가 공유하는 핵심 로직 - 원래 depth_callback
        안에 그대로 있던 코드를 옮긴 것뿐, 로직은 전혀 안 바뀌었다.
        `depth_shape`는 select_support_candidate()의 형식상 인자일 뿐 내부에서
        안 쓰이므로(`del image_shape`) 아무 값이나 넘겨도 무방하다.

        반환값: candidates (호출부가 디버그 이미지를 그릴 때 필요해서 그대로 돌려줌).
        """
        processed_points = np.asarray(
            scene_pcd.points,
            dtype=np.float32,
        )

        if len(processed_points) < MIN_PLANE_POINTS:
            self.latest_boxes = []
            self.latest_all_top_points = np.empty((0, 3), dtype=np.float32)
            self.latest_all_completed_points = np.empty((0, 3), dtype=np.float32)
            return []

        self.publish_cloud(
            processed_points[::SCENE_PUBLISH_STRIDE],
            header,
            self.scene_cloud_publisher,
        )

        candidates = self.detect_box_top_candidates(
            scene_pcd
        )

        # 카메라에 가까운 순서로 ID를 안정적으로 부여한다.
        ordered_candidates = sorted(
            candidates,
            key=lambda candidate: (
                candidate.median_depth,
                candidate.candidate_id,
            ),
        )

        current_boxes = []
        all_top_points = []
        all_completed_points = []

        for top_candidate in ordered_candidates:
            support = self.select_support_candidate(
                top_candidate,
                candidates,
                depth_shape,
            )
            support_type = "box_top"

            # 아래쪽 박스 윗면이 없으면 바닥을 FIRST BOUNDARY로 사용
            if support is None:
                support = self.detect_floor_boundary(
                    top_candidate,
                    scene_pcd,
                )
                support_type = "floor"

            if support is None:
                self.get_logger().warning(
                    "Candidate "
                    f"{top_candidate.candidate_id}: "
                    "no lower boundary or floor. "
                    f"footprint=({top_candidate.width:.3f},{top_candidate.height:.3f}) "
                    f"center={np.round(top_candidate.center, 3).tolist()} "
                    f"fill_ratio={top_candidate.fill_ratio:.3f}"
                )
                continue

            corners = self.compute_box_corners(
                top_candidate,
                support,
            )

            if corners is None or np.asarray(corners).shape != (8, 3):
                continue

            corners = np.asarray(
                corners,
                dtype=np.float32,
            )

            completed_points = (
                self.generate_completed_box_surface(
                    corners
                )
            )

            if len(completed_points) == 0:
                continue

            box_id = len(current_boxes)

            current_boxes.append(
                {
                    "box_id": box_id,
                    "top": top_candidate,
                    "support": support,
                    "support_type": support_type,
                    "corners": corners,
                    "completed_points": completed_points.astype(
                        np.float32
                    ),
                }
            )

            all_top_points.append(
                top_candidate.points.astype(
                    np.float32
                )
            )
            all_completed_points.append(
                completed_points.astype(
                    np.float32
                )
            )

        self.latest_boxes = current_boxes

        if all_top_points:
            self.latest_all_top_points = np.vstack(
                all_top_points
            ).astype(np.float32)

            self.publish_cloud(
                self.latest_all_top_points[
                    ::TOP_PUBLISH_STRIDE
                ],
                header,
                self.top_cloud_publisher,
            )
        else:
            self.latest_all_top_points = np.empty(
                (0, 3),
                dtype=np.float32,
            )

        if all_completed_points:
            self.latest_all_completed_points = np.vstack(
                all_completed_points
            ).astype(np.float32)

            self.publish_cloud(
                self.latest_all_completed_points,
                header,
                self.completed_cloud_publisher,
            )

            self.publish_all_box_corners(
                self.latest_boxes,
                header,
            )
        else:
            self.latest_all_completed_points = np.empty(
                (0, 3),
                dtype=np.float32,
            )
            self.publish_delete_markers(
                header
            )

        return candidates

    def depth_callback(
        self,
        msg: Image,
    ) -> None:
        if not self.camera_intrinsics_ready():
            return

        depth = self.convert_depth_message(msg)
        if depth is None:
            return

        self.frame_count += 1

        scene_points = self.depth_to_points(
            depth
        )
        scene_points = self.filter_points_by_base_roi(
            scene_points
        )
        self.latest_raw_scene_points = scene_points
        self.latest_header = msg.header

        if len(scene_points) < MIN_PLANE_POINTS:
            return

        scene_pcd = self.preprocess_cloud(
            scene_points
        )

        candidates = self.process_scene_cloud(
            scene_pcd, msg.header, depth.shape
        )

        debug_image = self.create_all_boxes_debug_image(
            depth=depth,
            candidates=candidates,
            boxes=self.latest_boxes,
        )

        self.publish_debug_image_direct(
            debug_image,
            msg.header,
        )

        cv2.imshow(
            "Depth All Boxes Extraction",
            debug_image,
        )

        key = cv2.waitKey(1) & 0xFF

        if key in (ord("s"), ord("S")):
            self.save_current_cloud()

        elif key in (ord("q"), ord("Q")):
            rclpy.shutdown()

        if self.frame_count % LOG_INTERVAL_FRAMES == 0:
            self.get_logger().info(
                f"top candidates={len(candidates)}, "
                f"reconstructed boxes={len(self.latest_boxes)}"
            )

    # ========================================================
    # Depth → Point Cloud
    # ========================================================

    def convert_depth_message(
        self,
        msg: Image,
    ) -> Optional[np.ndarray]:
        try:
            depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )
            depth = np.asarray(depth)

            if msg.encoding == "16UC1":
                return (
                    depth.astype(np.float32)
                    / 1000.0
                )

            return depth.astype(np.float32)

        except Exception as error:
            self.get_logger().error(
                f"Depth conversion failed: {error}"
            )
            return None

    def depth_to_points(
        self,
        depth: np.ndarray,
    ) -> np.ndarray:
        image_height, image_width = depth.shape

        left, top, right, bottom = IMAGE_ROI

        x1 = int(
            np.clip(
                left * image_width,
                0,
                image_width - 1,
            )
        )
        y1 = int(
            np.clip(
                top * image_height,
                0,
                image_height - 1,
            )
        )
        x2 = int(
            np.clip(
                right * image_width,
                x1 + 1,
                image_width,
            )
        )
        y2 = int(
            np.clip(
                bottom * image_height,
                y1 + 1,
                image_height,
            )
        )

        roi_depth = depth[y1:y2, x1:x2]

        local_v, local_u = np.indices(
            roi_depth.shape,
            dtype=np.float32,
        )

        u = local_u + float(x1)
        v = local_v + float(y1)

        valid = (
            np.isfinite(roi_depth)
            & (roi_depth >= MIN_DEPTH_M)
            & (roi_depth <= MAX_DEPTH_M)
        )

        z = roi_depth[valid]

        if len(z) == 0:
            return np.empty(
                (0, 3),
                dtype=np.float64,
            )

        u = u[valid]
        v = v[valid]

        x = (
            (u - self.cx)
            * z
            / self.fx
        )
        y = (
            (v - self.cy)
            * z
            / self.fy
        )

        return np.column_stack(
            (x, y, z)
        ).astype(np.float64)

    def filter_points_by_base_roi(
        self,
        points_cam: np.ndarray,
    ) -> np.ndarray:
        """카메라 좌표계 점들을 base_link 좌표계로 옮겨서 ROI_BASE_MIN/MAX 밖의 점을
        RANSAC 전에 잘라낸다(카트 와이어 메쉬/손잡이/바퀴 오검출 방지, PDF 요구 파이프라인의
        "ROI 적용" 단계). ROI_BASE_MIN 또는 MAX가 None이면(환경변수로 끈 경우) 그대로 반환."""
        if ROI_BASE_MIN is None or ROI_BASE_MAX is None:
            return points_cam
        if len(points_cam) == 0:
            return points_cam

        points_base = points_cam @ self._base_R.T + self._base_t
        inside = np.all(
            (points_base >= ROI_BASE_MIN) & (points_base <= ROI_BASE_MAX),
            axis=1,
        )
        return points_cam[inside]

    @staticmethod
    def preprocess_cloud(
        points: np.ndarray,
    ) -> o3d.geometry.PointCloud:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(
            points
        )

        pcd = pcd.voxel_down_sample(
            voxel_size=VOXEL_SIZE_M
        )

        if len(pcd.points) >= OUTLIER_NB_NEIGHBORS:
            pcd, _ = pcd.remove_statistical_outlier(
                nb_neighbors=OUTLIER_NB_NEIGHBORS,
                std_ratio=OUTLIER_STD_RATIO,
            )

        return pcd

    # ========================================================
    # 평면 검출 → 박스 윗면 후보
    # ========================================================

    def detect_box_top_candidates(
        self,
        scene_pcd: o3d.geometry.PointCloud,
    ) -> list[PlaneClusterCandidate]:
        """
        1. RANSAC으로 여러 평면을 먼저 검출한다.
        2. 카메라를 어느 정도 바라보는 평면들 중 대표 윗면 법선을 구한다.
        3. 대표 법선과 거의 평행한 평면만 박스 윗면 후보로 사용한다.
        4. 각 평면 내부를 DBSCAN으로 분리하고 직사각형 조건을 검사한다.

        이 방식은 CAMERA_FACING_NORMAL_DOT_MIN을 완화해도
        옆면/다른 높이의 표면까지 윗면으로 섞여 들어오는 문제를 줄이기 위한 것이다.
        """
        remaining = o3d.geometry.PointCloud(
            scene_pcd
        )

        camera_forward = np.array(
            [0.0, 0.0, 1.0],
            dtype=np.float64,
        )

        detected_planes = []

        # --------------------------------------------------------
        # 1단계: RANSAC으로 평면들을 먼저 모두 수집
        # --------------------------------------------------------
        for plane_index in range(MAX_PLANES):
            if len(remaining.points) < MIN_PLANE_POINTS:
                break

            plane_model, inliers = remaining.segment_plane(
                distance_threshold=PLANE_DISTANCE_THRESHOLD_M,
                ransac_n=PLANE_RANSAC_N,
                num_iterations=PLANE_RANSAC_ITERATIONS,
            )

            if len(inliers) < MIN_PLANE_POINTS:
                break

            plane_cloud = remaining.select_by_index(
                inliers
            )

            remaining = remaining.select_by_index(
                inliers,
                invert=True,
            )

            normal = np.asarray(
                plane_model[:3],
                dtype=np.float64,
            )

            normal_norm = float(
                np.linalg.norm(normal)
            )

            if normal_norm < 1e-8:
                continue

            normal /= normal_norm

            # 법선 방향의 부호를 카메라 전방(+Z) 쪽으로 통일한다.
            if float(
                np.dot(
                    normal,
                    camera_forward,
                )
            ) < 0.0:
                normal = -normal

            camera_alignment = abs(
                float(
                    np.dot(
                        normal,
                        camera_forward,
                    )
                )
            )

            detected_planes.append(
                {
                    "plane_index": plane_index,
                    "cloud": plane_cloud,
                    "normal": normal,
                    "camera_alignment": camera_alignment,
                    "point_count": len(inliers),
                }
            )

        if not detected_planes:
            return []

        # --------------------------------------------------------
        # 2단계: 대표 윗면 법선 선택
        #
        # 카메라를 어느 정도 바라보는 평면만 대상으로 하고,
        # Point 수가 많고 카메라 정면성이 높은 평면을 선택한다.
        # 바닥이 선택되더라도 박스 윗면과 평행하므로 문제없다.
        # --------------------------------------------------------
        reference_pool = [
            plane
            for plane in detected_planes
            if (
                plane["camera_alignment"]
                >= CAMERA_FACING_NORMAL_DOT_MIN
            )
        ]

        if not reference_pool:
            if self.frame_count % LOG_INTERVAL_FRAMES == 0:
                best_alignment = max(
                    plane["camera_alignment"]
                    for plane in detected_planes
                )
                self.get_logger().warning(
                    "No plane passed camera-facing filter. "
                    f"best alignment={best_alignment:.3f}, "
                    f"threshold={CAMERA_FACING_NORMAL_DOT_MIN:.3f}"
                )
            return []

        reference_plane = max(
            reference_pool,
            key=lambda plane: (
                float(plane["point_count"])
                * (
                    float(
                        plane["camera_alignment"]
                    )
                    ** REFERENCE_NORMAL_ALIGNMENT_POWER
                )
            ),
        )

        reference_normal = np.asarray(
            reference_plane["normal"],
            dtype=np.float64,
        )

        candidates: list[PlaneClusterCandidate] = []
        candidate_id = 0

        # --------------------------------------------------------
        # 3단계: 대표 법선과 평행한 평면만 DBSCAN/사각형 검사
        # --------------------------------------------------------
        for plane in detected_planes:
            normal = np.asarray(
                plane["normal"],
                dtype=np.float64,
            )

            normal_consistency = abs(
                float(
                    np.dot(
                        normal,
                        reference_normal,
                    )
                )
            )

            if (
                normal_consistency
                < TOP_NORMAL_CONSISTENCY_DOT_MIN
            ):
                if DEBUG_SUPPORT:
                    pts = np.asarray(plane["cloud"].points)
                    self.get_logger().info(
                        f"[DEBUG plane] plane_idx={plane['plane_index']} "
                        f"points={plane['point_count']} center={np.round(pts.mean(axis=0), 3).tolist()}: "
                        f"normal_consistency={normal_consistency:.3f} < {TOP_NORMAL_CONSISTENCY_DOT_MIN} -> reject whole plane"
                    )
                continue

            plane_cloud = plane["cloud"]

            labels = np.asarray(
                plane_cloud.cluster_dbscan(
                    eps=DBSCAN_EPS_M,
                    min_points=DBSCAN_MIN_POINTS,
                    print_progress=False,
                )
            )

            if labels.size == 0:
                continue

            for label in np.unique(labels):
                if label < 0:
                    continue

                cluster_indices = np.flatnonzero(
                    labels == label
                )

                if len(cluster_indices) < MIN_CLUSTER_POINTS:
                    if DEBUG_SUPPORT and len(cluster_indices) >= 10:
                        cluster_pts = np.asarray(plane_cloud.points)[cluster_indices]
                        self.get_logger().info(
                            f"[DEBUG cluster] plane_idx={plane['plane_index']} label={label}: "
                            f"points={len(cluster_indices)} < MIN_CLUSTER_POINTS({MIN_CLUSTER_POINTS}) "
                            f"center={np.round(cluster_pts.mean(axis=0), 3).tolist()} -> drop"
                        )
                    continue

                cluster = plane_cloud.select_by_index(
                    cluster_indices.tolist()
                )

                candidate = self.make_candidate(
                    candidate_id=candidate_id,
                    cluster=cluster,
                    plane_normal=normal,
                    debug_plane_index=plane["plane_index"],
                    debug_label=int(label),
                )

                if candidate is None:
                    continue

                candidates.append(candidate)
                candidate_id += 1

        if self.frame_count % LOG_INTERVAL_FRAMES == 0:
            self.get_logger().info(
                "normal filter: "
                f"planes={len(detected_planes)}, "
                f"reference_alignment="
                f"{reference_plane['camera_alignment']:.3f}, "
                f"top_parallel_threshold="
                f"{TOP_NORMAL_CONSISTENCY_DOT_MIN:.3f}, "
                f"candidates={len(candidates)}"
            )

        return candidates

    def make_candidate(
        self,
        candidate_id: int,
        cluster: o3d.geometry.PointCloud,
        plane_normal: np.ndarray,
        debug_plane_index: Optional[int] = None,
        debug_label: Optional[int] = None,
    ) -> Optional[PlaneClusterCandidate]:
        points = np.asarray(
            cluster.points,
            dtype=np.float64,
        )

        if len(points) < MIN_CLUSTER_POINTS:
            return None

        center = points.mean(axis=0)

        axis_u, axis_v = self.make_plane_basis(
            plane_normal
        )

        relative = points - center

        local_u = relative @ axis_u
        local_v = relative @ axis_v

        points_2d = np.column_stack(
            (local_u, local_v)
        ).astype(np.float32)

        rectangle = cv2.minAreaRect(
            points_2d
        )

        rect_center, rect_size, _ = rectangle
        side_a = float(rect_size[0])
        side_b = float(rect_size[1])

        if side_a <= 0.0 or side_b <= 0.0:
            return None

        width = max(side_a, side_b)
        height = min(side_a, side_b)

        if not (
            MIN_BOX_SIDE_M
            <= width
            <= MAX_BOX_SIDE_M
        ):
            if DEBUG_SUPPORT:
                self.get_logger().info(
                    f"[DEBUG make_candidate] plane_idx={debug_plane_index} label={debug_label} "
                    f"points={len(points)} center={np.round(center, 3).tolist()}: "
                    f"width={width:.3f} out of [{MIN_BOX_SIDE_M},{MAX_BOX_SIDE_M}] -> reject"
                )
            return None

        if not (
            MIN_BOX_SIDE_M
            <= height
            <= MAX_BOX_SIDE_M
        ):
            if DEBUG_SUPPORT:
                self.get_logger().info(
                    f"[DEBUG make_candidate] plane_idx={debug_plane_index} label={debug_label} "
                    f"points={len(points)} center={np.round(center, 3).tolist()}: "
                    f"height={height:.3f} out of [{MIN_BOX_SIDE_M},{MAX_BOX_SIDE_M}] -> reject"
                )
            return None

        aspect_ratio = width / max(height, 1e-8)

        if aspect_ratio > MAX_SIDE_ASPECT_RATIO:
            if DEBUG_SUPPORT:
                self.get_logger().info(
                    f"[DEBUG make_candidate] plane_idx={debug_plane_index} label={debug_label} "
                    f"points={len(points)} center={np.round(center, 3).tolist()}: "
                    f"aspect_ratio={aspect_ratio:.2f} > {MAX_SIDE_ASPECT_RATIO} "
                    f"(width={width:.3f}, height={height:.3f}) -> reject"
                )
            return None

        rectangle_area = width * height

        # 2D convex hull 면적을 실제 관측 면적으로 사용
        hull = cv2.convexHull(points_2d)
        observed_area = float(
            cv2.contourArea(hull)
        )

        fill_ratio = (
            observed_area
            / max(rectangle_area, 1e-8)
        )

        if fill_ratio < MIN_RECTANGULAR_FILL_RATIO:
            if DEBUG_SUPPORT:
                base_center = center @ self._base_R.T + self._base_t
                self.get_logger().info(
                    f"[DEBUG make_candidate] plane_idx={debug_plane_index} label={debug_label} "
                    f"points={len(points)} center_cam={np.round(center, 3).tolist()} "
                    f"center_base={np.round(base_center, 3).tolist()} "
                    f"footprint=({width:.3f},{height:.3f}): "
                    f"fill_ratio={fill_ratio:.3f} < {MIN_RECTANGULAR_FILL_RATIO} -> reject"
                )
            return None

        rectangle_points_2d = cv2.boxPoints(
            rectangle
        ).astype(np.float64)

        corners_3d = (
            center[None, :]
            + rectangle_points_2d[:, 0:1]
            * axis_u[None, :]
            + rectangle_points_2d[:, 1:2]
            * axis_v[None, :]
        )

        pixel_polygon = self.project_points(
            corners_3d
        )

        median_depth = float(
            np.median(points[:, 2])
        )

        plane_d = -float(
            np.dot(plane_normal, center)
        )

        return PlaneClusterCandidate(
            candidate_id=candidate_id,
            points=points.astype(np.float32),
            normal=plane_normal.copy(),
            median_depth=median_depth,
            width=width,
            height=height,
            area=rectangle_area,
            fill_ratio=fill_ratio,
            center=center.astype(np.float64),
            plane_d=plane_d,
            corners_3d=corners_3d.astype(np.float64),
            pixel_polygon=pixel_polygon,
        )

    # ========================================================
    # 아래 지지 평면 선택 및 8개 꼭짓점 계산
    # ========================================================

    def detect_floor_boundary(
        self,
        top: PlaneClusterCandidate,
        scene_pcd: o3d.geometry.PointCloud,
    ) -> Optional[PlaneClusterCandidate]:
        """
        아래쪽 박스 윗면 후보가 없을 때 전체 장면에서 바닥을 찾는다.

        바닥 조건:
          1. 초록색 윗면과 거의 평행
          2. 초록색 네 꼭짓점의 아래 방향 ray와 교차
          3. 충분히 넓고 Point가 많은 평면
          4. 조건을 만족하는 평면 중 가장 먼저 만나는 평면
        """
        remaining = o3d.geometry.PointCloud(
            scene_pcd
        )

        top_corners = self.order_rectangle_corners(
            top.corners_3d.copy()
        )
        down_direction = self.get_down_direction(
            top
        )

        floor_candidates = []

        if DEBUG_SUPPORT:
            self.get_logger().info(
                f"[DEBUG floor] top={top.candidate_id} "
                f"footprint=({top.width:.3f},{top.height:.3f}) "
                f"corners_z={np.round(top_corners[:, 2], 3).tolist()} "
                f"scene_points={len(remaining.points)}"
            )

        for plane_index in range(
            FLOOR_RANSAC_MAX_PLANES
        ):
            if len(remaining.points) < FLOOR_MIN_POINTS:
                if DEBUG_SUPPORT:
                    self.get_logger().info(
                        f"[DEBUG floor] top={top.candidate_id} plane_index={plane_index}: "
                        f"remaining points {len(remaining.points)} < FLOOR_MIN_POINTS, stop"
                    )
                break

            plane_model, inliers = remaining.segment_plane(
                distance_threshold=(
                    FLOOR_DISTANCE_THRESHOLD_M
                ),
                ransac_n=3,
                num_iterations=(
                    FLOOR_RANSAC_ITERATIONS
                ),
            )

            if len(inliers) < FLOOR_MIN_POINTS:
                break

            plane_cloud = remaining.select_by_index(
                inliers
            )
            remaining = remaining.select_by_index(
                inliers,
                invert=True,
            )

            points = np.asarray(
                plane_cloud.points,
                dtype=np.float64,
            )

            if len(points) < FLOOR_MIN_POINTS:
                continue

            normal = np.asarray(
                plane_model[:3],
                dtype=np.float64,
            )
            normal_norm = float(
                np.linalg.norm(normal)
            )

            if normal_norm < 1e-8:
                continue

            normal /= normal_norm
            plane_d = float(
                plane_model[3]
                / normal_norm
            )

            parallel_score = abs(
                float(
                    np.dot(
                        normal,
                        top.normal,
                    )
                )
            )

            if parallel_score < FLOOR_NORMAL_DOT_MIN:
                if DEBUG_SUPPORT:
                    self.get_logger().info(
                        f"[DEBUG floor] top={top.candidate_id} plane_index={plane_index} "
                        f"points={len(points)}: parallel_score={parallel_score:.3f} "
                        f"< {FLOOR_NORMAL_DOT_MIN} -> reject"
                    )
                continue

            denominator = float(
                np.dot(
                    normal,
                    down_direction,
                )
            )

            if abs(denominator) < 1e-6:
                continue

            ray_distances = -(
                top_corners @ normal
                + plane_d
            ) / denominator

            if np.any(
                ~np.isfinite(ray_distances)
            ):
                continue

            if np.any(
                ray_distances < MIN_RAY_DISTANCE_M
            ):
                if DEBUG_SUPPORT:
                    self.get_logger().info(
                        f"[DEBUG floor] top={top.candidate_id} plane_index={plane_index}: "
                        f"ray_distances={np.round(ray_distances, 3).tolist()} "
                        f"has value < MIN_RAY_DISTANCE_M({MIN_RAY_DISTANCE_M}) -> reject"
                    )
                continue

            median_distance = float(
                np.median(ray_distances)
            )

            if not (
                MIN_RAY_DISTANCE_M
                <= median_distance
                <= MAX_RAY_DISTANCE_M
            ):
                if DEBUG_SUPPORT:
                    self.get_logger().info(
                        f"[DEBUG floor] top={top.candidate_id} plane_index={plane_index}: "
                        f"median_distance={median_distance:.3f} out of range -> reject"
                    )
                continue

            spread = float(
                np.max(ray_distances)
                - np.min(ray_distances)
            )

            if spread > MAX_RAY_DISTANCE_SPREAD_M:
                if DEBUG_SUPPORT:
                    self.get_logger().info(
                        f"[DEBUG floor] top={top.candidate_id} plane_index={plane_index}: "
                        f"spread={spread:.3f} > MAX_RAY_DISTANCE_SPREAD_M({MAX_RAY_DISTANCE_SPREAD_M}) "
                        f"ray_distances={np.round(ray_distances, 3).tolist()} -> reject"
                    )
                continue

            # 평면의 실제 넓이를 로컬 2D convex hull로 계산
            center = points.mean(axis=0)
            axis_u, axis_v = self.make_plane_basis(
                normal
            )
            relative = points - center
            local_points = np.column_stack(
                (
                    relative @ axis_u,
                    relative @ axis_v,
                )
            ).astype(np.float32)

            hull = cv2.convexHull(
                local_points
            )
            observed_area = float(
                cv2.contourArea(hull)
            )

            if observed_area < FLOOR_MIN_AREA_M2:
                if DEBUG_SUPPORT:
                    self.get_logger().info(
                        f"[DEBUG floor] top={top.candidate_id} plane_index={plane_index}: "
                        f"observed_area={observed_area:.3f} < FLOOR_MIN_AREA_M2({FLOOR_MIN_AREA_M2}) -> reject"
                    )
                continue

            intersections = (
                top_corners
                + ray_distances[:, None]
                * down_direction[None, :]
            )

            nearest_distances = []

            for intersection in intersections:
                distances = np.linalg.norm(
                    points
                    - intersection[None, :],
                    axis=1,
                )
                nearest_distances.append(
                    float(np.min(distances))
                )

            nearest_distances = np.asarray(
                nearest_distances,
                dtype=np.float64,
            )

            hit_count = int(
                np.count_nonzero(
                    nearest_distances
                    <= FLOOR_BOUNDARY_HIT_DISTANCE_M
                )
            )

            if hit_count < MIN_BOUNDARY_RAY_HITS:
                if DEBUG_SUPPORT:
                    self.get_logger().info(
                        f"[DEBUG floor] top={top.candidate_id} plane_index={plane_index}: "
                        f"nearest_distances={np.round(nearest_distances, 3).tolist()} "
                        f"hit_count={hit_count} < MIN_BOUNDARY_RAY_HITS({MIN_BOUNDARY_RAY_HITS}) -> reject"
                    )
                continue

            if DEBUG_SUPPORT:
                self.get_logger().info(
                    f"[DEBUG floor] top={top.candidate_id} plane_index={plane_index}: ACCEPTED as floor "
                    f"(median_distance={median_distance:.3f}, spread={spread:.3f}, hit_count={hit_count})"
                )

            # 디버그 및 dataclass 호환용 바닥 사각형
            rectangle = cv2.minAreaRect(
                local_points
            )
            rectangle_points_2d = cv2.boxPoints(
                rectangle
            ).astype(np.float64)

            corners_3d = (
                center[None, :]
                + rectangle_points_2d[:, 0:1]
                * axis_u[None, :]
                + rectangle_points_2d[:, 1:2]
                * axis_v[None, :]
            )

            rect_size = rectangle[1]
            width = float(
                max(rect_size)
            )
            height = float(
                min(rect_size)
            )
            rectangle_area = max(
                width * height,
                1e-8,
            )
            fill_ratio = (
                observed_area
                / rectangle_area
            )

            pixel_polygon = self.project_points(
                corners_3d
            )

            floor_candidate = PlaneClusterCandidate(
                candidate_id=-(plane_index + 1),
                points=points,
                normal=normal,
                median_depth=float(
                    np.median(points[:, 2])
                ),
                width=width,
                height=height,
                area=observed_area,
                fill_ratio=fill_ratio,
                center=center,
                plane_d=plane_d,
                corners_3d=corners_3d,
                pixel_polygon=pixel_polygon,
            )

            floor_candidates.append(
                (
                    median_distance,
                    -observed_area,
                    -hit_count,
                    floor_candidate,
                )
            )

        if not floor_candidates:
            return None

        floor_candidates.sort(
            key=lambda item: item[:3]
        )

        return floor_candidates[0][3]

    def select_support_candidate(
        self,
        top: PlaneClusterCandidate,
        candidates: list[PlaneClusterCandidate],
        image_shape: tuple[int, int],
    ) -> Optional[PlaneClusterCandidate]:
        """
        초록색 윗면의 네 꼭짓점에서 아래 방향으로 ray를 내리고,
        네 ray가 가장 먼저 만나는 평행 후보 평면을 경계면으로 고른다.
        """
        del image_shape

        top_corners = self.order_rectangle_corners(
            top.corners_3d.copy()
        )
        down_direction = self.get_down_direction(
            top
        )

        ranked = []

        for candidate in candidates:
            if candidate.candidate_id == top.candidate_id:
                continue

            normal_dot = abs(
                float(
                    np.dot(
                        top.normal,
                        candidate.normal,
                    )
                )
            )
            if normal_dot < SUPPORT_NORMAL_DOT_MIN:
                continue

            candidate_normal = candidate.normal.astype(
                np.float64
            )
            candidate_normal /= max(
                np.linalg.norm(candidate_normal),
                1e-8,
            )

            denominator = float(
                np.dot(
                    candidate_normal,
                    down_direction,
                )
            )
            if abs(denominator) < 1e-6:
                continue

            ray_distances = -(
                top_corners @ candidate_normal
                + float(candidate.plane_d)
            ) / denominator

            if np.any(~np.isfinite(ray_distances)):
                continue

            if np.any(
                ray_distances < MIN_RAY_DISTANCE_M
            ):
                continue

            median_distance = float(
                np.median(ray_distances)
            )

            if not (
                MIN_RAY_DISTANCE_M
                <= median_distance
                <= MAX_RAY_DISTANCE_M
            ):
                continue

            spread = float(
                np.max(ray_distances)
                - np.min(ray_distances)
            )

            if spread > MAX_RAY_DISTANCE_SPREAD_M:
                continue

            intersections = (
                top_corners
                + ray_distances[:, None]
                * down_direction[None, :]
            )

            candidate_points = np.asarray(
                candidate.points,
                dtype=np.float64,
            )

            if len(candidate_points) == 0:
                continue

            nearest_distances = []

            for intersection in intersections:
                distances = np.linalg.norm(
                    candidate_points
                    - intersection[None, :],
                    axis=1,
                )
                nearest_distances.append(
                    float(np.min(distances))
                )

            nearest_distances = np.asarray(
                nearest_distances,
                dtype=np.float64,
            )

            hit_count = int(
                np.count_nonzero(
                    nearest_distances
                    <= BOUNDARY_HIT_DISTANCE_M
                )
            )

            if hit_count < MIN_BOUNDARY_RAY_HITS:
                continue

            ranked.append(
                (
                    median_distance,
                    -hit_count,
                    spread,
                    float(np.min(nearest_distances)),
                    candidate,
                )
            )

        if not ranked:
            if DEBUG_SUPPORT:
                self.get_logger().info(
                    f"[DEBUG box_top support] top={top.candidate_id}: "
                    f"no other top candidate qualifies as support -> falling back to floor"
                )
            return None

        ranked.sort(
            key=lambda item: item[:4]
        )
        if DEBUG_SUPPORT:
            self.get_logger().info(
                f"[DEBUG box_top support] top={top.candidate_id}: "
                f"ACCEPTED support candidate={ranked[0][4].candidate_id} "
                f"(median_distance={ranked[0][0]:.3f})"
            )
        return ranked[0][4]

    @staticmethod
    def get_down_direction(
        top: PlaneClusterCandidate,
    ) -> np.ndarray:
        """
        윗면 법선 두 방향 중 카메라에서 멀어지는 방향을 아래로 사용한다.
        optical frame의 +Z는 카메라 전방이다.
        """
        direction = top.normal.astype(
            np.float64
        )
        direction /= max(
            np.linalg.norm(direction),
            1e-8,
        )

        camera_forward = np.array(
            [0.0, 0.0, 1.0],
            dtype=np.float64,
        )

        if float(
            np.dot(
                direction,
                camera_forward,
            )
        ) < 0.0:
            direction = -direction

        return direction

    @staticmethod
    def order_rectangle_corners(
        corners: np.ndarray,
    ) -> np.ndarray:
        """4개 꼭짓점을 둘레 순서로 정렬한다."""
        center = corners.mean(axis=0)

        edge_1 = corners[1] - corners[0]
        edge_2 = corners[2] - corners[0]
        normal = np.cross(edge_1, edge_2)
        normal_norm = np.linalg.norm(normal)

        if normal_norm < 1e-8:
            return corners

        normal /= normal_norm
        axis_u = edge_1 / max(
            np.linalg.norm(edge_1),
            1e-8,
        )
        axis_v = np.cross(normal, axis_u)
        axis_v /= max(
            np.linalg.norm(axis_v),
            1e-8,
        )

        relative = corners - center
        angles = np.arctan2(
            relative @ axis_v,
            relative @ axis_u,
        )
        return corners[np.argsort(angles)]

    def compute_box_corners(
        self,
        top: PlaneClusterCandidate,
        support: PlaneClusterCandidate,
    ) -> Optional[np.ndarray]:
        """
        초록색 윗면 네 꼭짓점에서 아래 방향 ray를 내리고,
        선택된 경계 평면과 만나는 네 교점으로 아래 꼭짓점을 만든다.
        """
        top_corners = self.order_rectangle_corners(
            top.corners_3d.copy()
        )

        down_direction = self.get_down_direction(
            top
        )

        support_normal = support.normal.astype(
            np.float64
        )
        support_normal /= max(
            np.linalg.norm(support_normal),
            1e-8,
        )
        support_d = float(
            support.plane_d
        )

        denominator = float(
            np.dot(
                support_normal,
                down_direction,
            )
        )
        if abs(denominator) < 1e-6:
            return None

        ray_distances = -(
            top_corners @ support_normal
            + support_d
        ) / denominator

        if np.any(~np.isfinite(ray_distances)):
            return None

        if np.any(
            ray_distances < MIN_RAY_DISTANCE_M
        ):
            return None

        median_height = float(
            np.median(ray_distances)
        )

        if not (
            MIN_BOX_HEIGHT_M
            <= median_height
            <= MAX_BOX_HEIGHT_M
        ):
            return None

        spread = float(
            np.max(ray_distances)
            - np.min(ray_distances)
        )
        if spread > MAX_RAY_DISTANCE_SPREAD_M:
            return None

        effective_distances = (
            ray_distances
            - BOTTOM_CUT_MARGIN_M
        )

        if np.any(effective_distances <= 0.0):
            return None

        bottom_corners = (
            top_corners
            + effective_distances[:, None]
            * down_direction[None, :]
        )

        return np.vstack(
            (top_corners, bottom_corners)
        ).astype(np.float64)

    def generate_completed_box_surface(
        self,
        corners: np.ndarray,
    ) -> np.ndarray:
        """
        8개 꼭짓점으로 직육면체의 여섯 면을 샘플링한다.

        이 Point들은 Depth 카메라가 실제로 측정한 Point가 아니라,
        검출된 8개 꼭짓점으로부터 기하학적으로 복원한 Point다.

        corner 순서:
          0,1,2,3: 초록색 윗면
          4,5,6,7: FIRST BOUNDARY 아랫면
        """
        corners = np.asarray(
            corners,
            dtype=np.float64,
        )

        if corners.shape != (8, 3):
            return np.empty(
                (0, 3),
                dtype=np.float32,
            )

        faces = (
            (0, 1, 2, 3),  # top
            (4, 5, 6, 7),  # bottom
            (0, 1, 5, 4),
            (1, 2, 6, 5),
            (2, 3, 7, 6),
            (3, 0, 4, 7),
        )

        sampled_faces = []

        for i0, i1, i2, i3 in faces:
            face_points = self.sample_quad_surface(
                corners[i0],
                corners[i1],
                corners[i2],
                corners[i3],
                COMPLETED_SURFACE_POINT_SPACING_M,
            )

            if len(face_points) > 0:
                sampled_faces.append(
                    face_points
                )

        if not sampled_faces:
            return np.empty(
                (0, 3),
                dtype=np.float32,
            )

        points = np.vstack(
            sampled_faces
        )

        pcd = o3d.geometry.PointCloud()
        pcd.points = (
            o3d.utility.Vector3dVector(
                points
            )
        )

        pcd = pcd.voxel_down_sample(
            voxel_size=(
                COMPLETED_SURFACE_VOXEL_SIZE_M
            )
        )

        return np.asarray(
            pcd.points,
            dtype=np.float32,
        )

    @staticmethod
    def sample_quad_surface(
        corner_00: np.ndarray,
        corner_10: np.ndarray,
        corner_11: np.ndarray,
        corner_01: np.ndarray,
        spacing: float,
    ) -> np.ndarray:
        """
        네 꼭짓점으로 정의된 사각형 면을 bilinear interpolation으로
        균일하게 샘플링한다.
        """
        edge_u_length = max(
            float(
                np.linalg.norm(
                    corner_10 - corner_00
                )
            ),
            float(
                np.linalg.norm(
                    corner_11 - corner_01
                )
            ),
        )

        edge_v_length = max(
            float(
                np.linalg.norm(
                    corner_01 - corner_00
                )
            ),
            float(
                np.linalg.norm(
                    corner_11 - corner_10
                )
            ),
        )

        if (
            edge_u_length < 1e-8
            or edge_v_length < 1e-8
        ):
            return np.empty(
                (0, 3),
                dtype=np.float64,
            )

        count_u = max(
            2,
            int(
                np.ceil(
                    edge_u_length / spacing
                )
            ) + 1,
        )

        count_v = max(
            2,
            int(
                np.ceil(
                    edge_v_length / spacing
                )
            ) + 1,
        )

        u_values = np.linspace(
            0.0,
            1.0,
            count_u,
            dtype=np.float64,
        )
        v_values = np.linspace(
            0.0,
            1.0,
            count_v,
            dtype=np.float64,
        )

        uu, vv = np.meshgrid(
            u_values,
            v_values,
        )

        uu = uu.reshape(-1, 1)
        vv = vv.reshape(-1, 1)

        points = (
            (1.0 - uu)
            * (1.0 - vv)
            * corner_00[None, :]
            + uu
            * (1.0 - vv)
            * corner_10[None, :]
            + uu
            * vv
            * corner_11[None, :]
            + (1.0 - uu)
            * vv
            * corner_01[None, :]
        )

        return points.astype(
            np.float64
        )

    def publish_all_box_corners(
        self,
        boxes: list,
        source_header,
    ) -> None:
        """
        모든 박스의 꼭짓점을 하나의 PoseArray와 MarkerArray로 발행한다.
        PoseArray에는 BOX 0의 8점, BOX 1의 8점 순서로 들어간다.
        """
        pose_array = PoseArray()
        pose_array.header = source_header

        marker_array = MarkerArray()

        delete = Marker()
        delete.header = source_header
        delete.action = Marker.DELETEALL
        marker_array.markers.append(delete)

        edges = (
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        )

        for box in boxes:
            box_id = int(box["box_id"])
            corners = np.asarray(
                box["corners"],
                dtype=np.float32,
            )

            for corner in corners:
                pose = Pose()
                pose.position.x = float(corner[0])
                pose.position.y = float(corner[1])
                pose.position.z = float(corner[2])
                pose.orientation.w = 1.0
                pose_array.poses.append(pose)

            spheres = Marker()
            spheres.header = source_header
            spheres.ns = f"box_{box_id}_corners"
            spheres.id = box_id * 2
            spheres.type = Marker.SPHERE_LIST
            spheres.action = Marker.ADD
            spheres.pose.orientation.w = 1.0
            spheres.scale.x = CORNER_MARKER_SCALE_M
            spheres.scale.y = CORNER_MARKER_SCALE_M
            spheres.scale.z = CORNER_MARKER_SCALE_M
            spheres.color.r = 1.0
            spheres.color.g = 0.1
            spheres.color.b = 0.1
            spheres.color.a = 1.0

            for corner in corners:
                point = Point()
                point.x = float(corner[0])
                point.y = float(corner[1])
                point.z = float(corner[2])
                spheres.points.append(point)

            marker_array.markers.append(spheres)

            lines = Marker()
            lines.header = source_header
            lines.ns = f"box_{box_id}_edges"
            lines.id = box_id * 2 + 1
            lines.type = Marker.LINE_LIST
            lines.action = Marker.ADD
            lines.pose.orientation.w = 1.0
            lines.scale.x = EDGE_MARKER_SCALE_M
            lines.color.r = 0.1
            lines.color.g = 1.0
            lines.color.b = 0.1
            lines.color.a = 1.0

            for start_index, end_index in edges:
                for corner_index in (
                    start_index,
                    end_index,
                ):
                    point = Point()
                    point.x = float(
                        corners[corner_index, 0]
                    )
                    point.y = float(
                        corners[corner_index, 1]
                    )
                    point.z = float(
                        corners[corner_index, 2]
                    )
                    lines.points.append(point)

            marker_array.markers.append(lines)

        self.corners_publisher.publish(
            pose_array
        )
        self.markers_publisher.publish(
            marker_array
        )

    def publish_delete_markers(
        self,
        source_header,
    ) -> None:
        delete = Marker()
        delete.header = source_header
        delete.action = Marker.DELETEALL
        array = MarkerArray()
        array.markers.append(delete)
        self.markers_publisher.publish(array)

    # ========================================================
    # 화면 표시
    # ========================================================

    def create_debug_image(
        self,
        depth: np.ndarray,
        candidates: list[PlaneClusterCandidate],
    ) -> np.ndarray:
        valid = (
            np.isfinite(depth)
            & (depth >= MIN_DEPTH_M)
            & (depth <= MAX_DEPTH_M)
        )

        normalized = np.zeros_like(
            depth,
            dtype=np.uint8,
        )

        if np.any(valid):
            values = depth[valid]

            low = float(
                np.percentile(values, 2.0)
            )
            high = float(
                np.percentile(values, 98.0)
            )

            if high > low:
                # 가까운 영역이 밝게 보이도록 반전
                normalized[valid] = np.clip(
                    (
                        high - depth[valid]
                    )
                    / (high - low)
                    * 255.0,
                    0,
                    255,
                ).astype(np.uint8)

        image = cv2.applyColorMap(
            normalized,
            cv2.COLORMAP_JET,
        )
        image[~valid] = (0, 0, 0)

        for candidate in candidates:
            if candidate.pixel_polygon is None:
                continue

            color = (0, 165, 255)

            polygon = candidate.pixel_polygon.astype(
                np.int32
            )

            cv2.polylines(
                image,
                [polygon],
                True,
                color,
                1,
            )

            label = (
                f"ID={candidate.candidate_id} "
                f"Z={candidate.median_depth:.3f} "
                f"{candidate.width:.2f}x"
                f"{candidate.height:.2f}"
            )

            cv2.putText(
                image,
                label,
                tuple(polygon[0]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                color,
                2,
            )

        # 요약 텍스트는 create_all_boxes_debug_image가 같은 자리에
        # 더 큰 배경 사각형으로 덮어 그리므로 여기서는 그리지 않는다.
        return image

    def create_all_boxes_debug_image(
        self,
        depth: np.ndarray,
        candidates: list[PlaneClusterCandidate],
        boxes: list,
    ) -> np.ndarray:
        """
        모든 검출 윗면은 초록색, 각 FIRST BOUNDARY는 굵은 주황색으로 표시.
        """
        image = self.create_debug_image(
            depth=depth,
            candidates=candidates,
        )

        for box in boxes:
            box_id = int(box["box_id"])
            top = box["top"]
            corners = np.asarray(
                box["corners"],
                dtype=np.float32,
            )

            if top.pixel_polygon is not None:
                top_polygon = np.asarray(
                    top.pixel_polygon,
                    dtype=np.int32,
                )

                cv2.polylines(
                    image,
                    [top_polygon],
                    True,
                    (0, 255, 0),
                    3,
                )
                cv2.putText(
                    image,
                    f"BOX {box_id} TOP",
                    tuple(top_polygon[0]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2,
                )

            boundary_polygon = self.project_points(
                corners[4:8]
            )

            if boundary_polygon is not None:
                boundary_polygon = np.round(
                    boundary_polygon
                ).astype(np.int32)

                cv2.polylines(
                    image,
                    [boundary_polygon],
                    True,
                    (0, 165, 255),
                    3,
                )

                support_label = (
                    "FLOOR"
                    if box["support_type"] == "floor"
                    else "BOX TOP"
                )

                cv2.putText(
                    image,
                    f"BOX {box_id} BOUNDARY={support_label}",
                    tuple(boundary_polygon[0]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.48,
                    (0, 165, 255),
                    2,
                )

        cv2.rectangle(
            image,
            (10, 10),
            (760, 62),
            (0, 0, 0),
            -1,
        )
        cv2.putText(
            image,
            (
                f"TOP CANDIDATES={len(candidates)} | "
                f"RECONSTRUCTED BOXES={len(boxes)}"
            ),
            (20, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            (255, 255, 255),
            2,
        )

        return image

    def publish_debug_image_direct(
        self,
        image: np.ndarray,
        source_header,
    ) -> None:
        image = np.ascontiguousarray(
            image,
            dtype=np.uint8,
        )

        height, width = image.shape[:2]

        message = Image()
        message.header = source_header
        message.height = int(height)
        message.width = int(width)
        message.encoding = "bgr8"
        message.is_bigendian = False
        message.step = int(width * 3)
        message.data = image.tobytes()

        self.debug_image_publisher.publish(
            message
        )

    # ========================================================
    # PointCloud2 및 저장
    # ========================================================

    @staticmethod
    def publish_cloud(
        points: np.ndarray,
        source_header,
        publisher,
    ) -> None:
        if len(points) == 0:
            return

        header = Header()
        header.stamp = source_header.stamp
        header.frame_id = source_header.frame_id

        message = (
            point_cloud2.create_cloud_xyz32(
                header,
                points.astype(
                    np.float32
                ).tolist(),
            )
        )

        publisher.publish(message)

    def save_current_cloud(self) -> None:
        """
        S 키를 누르면 전체 박스 결과를 두 파일에 통합 저장한다.

        1) all_boxes_corners_*.json
           - 모든 박스의 8개 꼭짓점
           - 통합 PLY에서 각 박스가 차지하는 Point 범위

        2) all_boxes_completed_*.ply
           - 모든 박스의 복원 Point Cloud를 하나로 합친 파일
        """
        if not self.latest_boxes:
            self.get_logger().warning(
                "저장할 복원 박스 정보가 없습니다."
            )
            return

        timestamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S_%f"
        )

        all_completed_points = []
        boxes_payload = []
        point_offset = 0

        for box in self.latest_boxes:
            box_id = int(box["box_id"])

            corners_cam = np.asarray(
                box["corners"],
                dtype=np.float64,
            )
            completed_points_cam = np.asarray(
                box["completed_points"],
                dtype=np.float64,
            )

            # 카메라 좌표계 -> m0609_base_link 좌표계 (32.box_table_scan_setup.py가
            # 고정 스캔 자세에서 측정한 변환). RANSAC/DBSCAN 등 검출 로직 자체는 계속
            # 카메라 좌표계로 돌고, 저장 직전 이 한 곳에서만 변환한다.
            corners = (
                corners_cam @ self._base_R.T + self._base_t
            ).astype(np.float32)
            completed_points = (
                completed_points_cam @ self._base_R.T + self._base_t
            ).astype(np.float32)

            if corners.shape != (8, 3):
                self.get_logger().warning(
                    f"BOX {box_id}: invalid corners shape "
                    f"{corners.shape}"
                )
                continue

            if len(completed_points) == 0:
                self.get_logger().warning(
                    f"BOX {box_id}: empty completed cloud"
                )
                continue

            point_start_index = point_offset
            point_count = int(
                len(completed_points)
            )
            point_end_index = (
                point_start_index
                + point_count
                - 1
            )

            all_completed_points.append(
                completed_points
            )

            top_candidate = box["top"]
            support_candidate = box["support"]

            boxes_payload.append(
                {
                    "box_id": box_id,
                    "support_type": box["support_type"],
                    "top_candidate_id": int(
                        top_candidate.candidate_id
                    ),
                    "support_candidate_id": int(
                        support_candidate.candidate_id
                    ),
                    "corner_order": [
                        "top_0",
                        "top_1",
                        "top_2",
                        "top_3",
                        "bottom_0",
                        "bottom_1",
                        "bottom_2",
                        "bottom_3",
                    ],
                    "corners_m": corners.tolist(),
                    "bottom_cut_margin_m": (
                        BOTTOM_CUT_MARGIN_M
                    ),
                    "completed_point_count": (
                        point_count
                    ),
                    "ply_point_start_index": (
                        point_start_index
                    ),
                    "ply_point_end_index": (
                        point_end_index
                    ),
                }
            )

            point_offset += point_count

        if not all_completed_points:
            self.get_logger().warning(
                "유효한 박스 Point Cloud가 없습니다."
            )
            return

        merged_points = np.vstack(
            all_completed_points
        ).astype(np.float64)

        ply_path = (
            SAVE_DIRECTORY
            / f"all_boxes_completed_{timestamp}.ply"
        )

        merged_pcd = o3d.geometry.PointCloud()
        merged_pcd.points = (
            o3d.utility.Vector3dVector(
                merged_points
            )
        )

        ply_success = o3d.io.write_point_cloud(
            str(ply_path),
            merged_pcd,
            write_ascii=True,
        )

        json_path = (
            SAVE_DIRECTORY
            / f"all_boxes_corners_{timestamp}.json"
        )

        payload = {
            "coordinate_frame": OUTPUT_FRAME,
            "unit": "meter",
            "box_count": len(boxes_payload),
            "completed_ply_file": ply_path.name,
            "total_completed_point_count": int(
                len(merged_points)
            ),
            "boxes": boxes_payload,
        }

        json_path.write_text(
            json.dumps(
                payload,
                indent=2,
            ),
            encoding="utf-8",
        )

        if ply_success:
            self.get_logger().info(
                f"Saved {len(boxes_payload)} boxes: "
                f"{json_path.name}, {ply_path.name}"
            )
        else:
            self.get_logger().error(
                "통합 PLY 저장에 실패했습니다."
            )

    # ========================================================
    # 보조 함수
    # ========================================================

    def camera_intrinsics_ready(self) -> bool:
        return all(
            value is not None
            for value in (
                self.fx,
                self.fy,
                self.cx,
                self.cy,
            )
        )

    @staticmethod
    def make_plane_basis(
        normal: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        normal = (
            normal
            / np.linalg.norm(normal)
        )

        reference = np.array(
            [1.0, 0.0, 0.0],
            dtype=np.float64,
        )

        if abs(
            float(
                np.dot(reference, normal)
            )
        ) > 0.9:
            reference = np.array(
                [0.0, 1.0, 0.0],
                dtype=np.float64,
            )

        axis_u = (
            reference
            - np.dot(reference, normal)
            * normal
        )
        axis_u /= np.linalg.norm(axis_u)

        axis_v = np.cross(
            normal,
            axis_u,
        )
        axis_v /= np.linalg.norm(axis_v)

        return axis_u, axis_v

    def project_points(
        self,
        points: np.ndarray,
    ) -> Optional[np.ndarray]:
        z = points[:, 2]

        if np.any(z <= 1e-6):
            return None

        u = (
            self.fx
            * points[:, 0]
            / z
            + self.cx
        )

        v = (
            self.fy
            * points[:, 1]
            / z
            + self.cy
        )

        return np.column_stack(
            (u, v)
        ).astype(np.float32)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None

    try:
        node = DepthTopmostBoxExtractor()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as error:
        print(f"\nProgram error: {error}\n")
        raise

    finally:
        cv2.destroyAllWindows()

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
