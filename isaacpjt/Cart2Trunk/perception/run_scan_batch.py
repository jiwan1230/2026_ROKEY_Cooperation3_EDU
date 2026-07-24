#!/usr/bin/env python3
"""
run_scan_batch.py
35.crate_scan_setup.py가 다중 시점을 base_link 좌표계로 합쳐 저장한 point cloud
파일(--input, .npy)을 읽어 box_geometry.py 기반 검출을 한 번 실행하고, 완료되면
--marker 경로에 마커 파일을 만든다.

run_scan_once.py(단일 고정 자세, ROS2 depth 토픽 실시간 구독)와 짝이 되는 자리지만
이쪽은 이미 다 모아진 point cloud 하나를 오프라인으로 처리하므로 rclpy/cv_bridge/ROS2
토픽 구독이 전혀 필요 없다 - 실제 작업은 multiview_scan.py에 있고 이 파일은 그 CLI를
그대로 실행하는 진입점이다.

사용법:
    python3 run_scan_batch.py --input <merged_cloud.npy> --marker <marker경로>
"""

from multiview_scan import main

if __name__ == "__main__":
    main()
