cmake_minimum_required(VERSION 3.8)
project(social_nav_tracking_cpp)

# colcon does not set a build type, which means no optimisation flags. This
# package exists to save CPU, so default to Release unless chosen otherwise.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Kieu build (mac dinh Release)" FORCE)
  message(STATUS "social_nav_tracking_cpp: CMAKE_BUILD_TYPE not set -> using Release")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(thesis_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------
# Core library: no rclcpp dependency. The whole algorithm lives here and the
# node is a thin wrapper, so gtest runs in milliseconds without a ROS graph.
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME}_core SHARED
  src/numpy_compat.cpp
  src/assignment.cpp
  src/distance_transform.cpp
  src/lidar_clustering.cpp
  src/human_tracking.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen)

# Node: thin wrapper around the core library.
add_executable(human_tracker_node1 src/human_tracker_node.cpp)
target_link_libraries(human_tracker_node1 ${PROJECT_NAME}_core)
ament_target_dependencies(human_tracker_node1
  rclcpp
  thesis_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(
  TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Executables must be in lib/<package>/ for `ros2 run` and launch to find them.
install(
  TARGETS human_tracker_node1
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  # Two linters are disabled on purpose:
  #  - copyright: no file in this workspace has a copyright header yet; this is a
  #    repository-wide decision.
  #  - flake8/pep257: they would only check tools/dump_golden.py, a developer
  #    script that is not installed and does not run on the robot.
  # cpplint/uncrustify/cppcheck stay enabled for the C++ code that runs on the robot.
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_flake8
    ament_cmake_pep257
    ament_cmake_cpplint
  )
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # cpplint runs separately (not through lint_auto) only to disable the single
  # copyright check while keeping every other check.
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS -legal/copyright)
  find_package(ament_cmake_gtest REQUIRED)

  set(social_nav_tracking_cpp_tests
    test_numpy_compat
    test_assignment
    test_distance_transform
    test_clustering
    test_golden_replay
    test_lidar_only_decay
  )
  foreach(test_name ${social_nav_tracking_cpp_tests})
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME}_core)
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
    # Golden data exported from the Python/numpy/scipy implementation -- see tools/dump_golden.py
    target_compile_definitions(${test_name} PRIVATE
      GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/golden")
  endforeach()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(
  rclcpp thesis_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs Eigen3)

ament_package()
