cmake_minimum_required(VERSION 3.8)
project(cart2trunk_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/TrunkScan.action"
  "action/CartScan.action"
)

ament_package()
