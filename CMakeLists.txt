cmake_minimum_required(VERSION 3.8)
project(aatb_crowd_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aatb_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Main executable
add_executable(crowd_tracker_node
  src/crowd_tracker_node.cpp
  src/background_model.cpp
  src/cluster_extractor.cpp
  src/tracker.cpp
)

ament_target_dependencies(crowd_tracker_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  builtin_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  aatb_msgs
)

install(TARGETS
  crowd_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()