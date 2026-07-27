"""trunk_scan_server.launch.py - Isaac Sim PC에서 trunk_scan_action_server를 파라미터
파일과 함께 띄우는 런치 파일. 기본 파라미터 파일은 이 패키지에 포함된 예시(config/
trunk_scan_server.params.yaml)이며, 실제 배포 머신에서는 `params_file` 런치 인자로
그 머신 전용 파일을 넘기면 된다."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("cart2trunk_bridge"),
        "config", "trunk_scan_server.params.yaml")

    params_file_arg = DeclareLaunchArgument(
        "params_file", default_value=default_params,
        description="trunk_scan_action_server 파라미터 YAML 경로 (머신별로 다름)")

    node = Node(
        package="cart2trunk_bridge",
        executable="trunk_scan_action_server",
        name="trunk_scan_action_server",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_file_arg, node])
