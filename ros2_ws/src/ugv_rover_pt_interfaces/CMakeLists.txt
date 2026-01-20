cmake_minimum_required(VERSION 3.8)
project(ugv_rover_pt_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ScanAndOffload.action"
)

ament_package()
