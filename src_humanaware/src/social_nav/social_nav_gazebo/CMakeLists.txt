cmake_minimum_required(VERSION 3.8)
project(social_nav_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

add_library(social_nav_gazebo SHARED
  src/moving_collision_proxy_plugin1.cpp
)
target_link_libraries(social_nav_gazebo
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-math7::gz-math7
)

# Separate library: GZ_ADD_PLUGIN may appear only once per .so (duplicate
# GzPluginHook at link time). The SDF loads filename="social_nav_ground_truth".
add_library(social_nav_ground_truth SHARED
  src/ground_truth_pose_publisher1.cpp
)
target_link_libraries(social_nav_ground_truth
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

install(
  TARGETS social_nav_gazebo social_nav_ground_truth
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Metrics node (Python) -- ros2 run social_nav_gazebo social_nav_metrics
install(
  PROGRAMS scripts/social_nav_metrics.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME social_nav_metrics
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/social_nav_gazebo.dsv.in"
)

ament_package()
