cmake_minimum_required(VERSION 3.8)
project(rviz_marker_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(rviz_marker_controller
  src/rviz_marker_node.cpp
  src/rviz_marker_main.cpp
)

ament_target_dependencies(rviz_marker_controller
  rclcpp
  visualization_msgs
  interactive_markers
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  rviz_marker_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()