#!/bin/bash
# Isaac Sim PC(MSI2)에서 trunk_scan_action_server를 띄우는 헬퍼 - 터미널에
# 긴 명령어를 여러 줄로 붙여넣다가 줄바꿈 지점에서 끊기는 문제를 피하기 위해
# 스크립트 파일로 만들었다. 사용법: bash run_trunk_scan_server.sh
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run cart2trunk_bridge trunk_scan_action_server \
  --ros-args --params-file src/cart2trunk_bridge/config/trunk_scan_server.params.yaml
