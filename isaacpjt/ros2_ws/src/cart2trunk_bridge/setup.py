from setuptools import find_packages, setup

package_name = "cart2trunk_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/trunk_scan_server.launch.py"]),
        ("share/" + package_name + "/config", [
            "config/trunk_scan_server.params.yaml",
            "config/cart_scan_server.params.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rokey",
    maintainer_email="hbm06047@gmail.com",
    description="Cart2Trunk 트렁크 스캔 ROS2 액션 서버/클라이언트 노드",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "trunk_scan_action_server = cart2trunk_bridge.trunk_scan_action_server:main",
            "trunk_scan_client = cart2trunk_bridge.trunk_scan_action_client:main",
            "cart_scan_action_server = cart2trunk_bridge.cart_scan_action_server:main",
            "cart_scan_client = cart2trunk_bridge.cart_scan_action_client:main",
            "pick_and_place_action_server = cart2trunk_bridge.pick_and_place_action_server:main",
            "pick_and_place_client = cart2trunk_bridge.pick_and_place_action_client:main",
        ],
    },
)
