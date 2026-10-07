cmake_minimum_required(VERSION 3.8)
project(thesis_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
    "msg/PoseKeypoint.msg"
    "msg/PoseResult.msg"
    "msg/PoseKeypoint3D.msg"
    "msg/PoseResult3D.msg"
    "msg/MultiplePoseResult3D.msg"
    "msg/TrackedHuman.msg"
    "msg/TrackedHumans.msg"
  DEPENDENCIES
    std_msgs
)

ament_package()
