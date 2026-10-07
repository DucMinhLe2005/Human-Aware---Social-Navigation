cmake_minimum_required(VERSION 3.8)
project(social_nav_safety_cpp)

# colcon does not set a build type, which would mean -O0. This package exists
# to save CPU, so default to Release.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Kieu build (mac dinh Release)" FORCE)
  message(STATUS "social_nav_safety_cpp: CMAKE_BUILD_TYPE not set -> using Release")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Core library: no rclcpp dependency, so gtest runs without a ROS graph.
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME}_core SHARED
  src/lidar_safety.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(lidar_safety_node1 src/lidar_safety_node.cpp)
target_link_libraries(lidar_safety_node1 ${PROJECT_NAME}_core)
ament_target_dependencies(lidar_safety_node1
  rclcpp
  geometry_msgs
  sensor_msgs
)

install(
  TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Executables must be in lib/<package>/ for `ros2 run` and launch to find them.
install(
  TARGETS lidar_safety_node1
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  # No file in the workspace has a copyright header, and Python lint would only
  # touch the developer script under tools/.
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_flake8
    ament_cmake_pep257
    ament_cmake_cpplint
  )
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS -legal/copyright)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_lidar_safety test/test_lidar_safety.cpp)
  target_link_libraries(test_lidar_safety ${PROJECT_NAME}_core)
  target_include_directories(test_lidar_safety PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  # Golden data exported from the Python implementation -- see tools/dump_safety_golden.py
  target_compile_definitions(test_lidar_safety PRIVATE
    GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/golden")
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(rclcpp geometry_msgs sensor_msgs)

ament_package()
