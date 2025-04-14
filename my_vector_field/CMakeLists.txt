cmake_minimum_required(VERSION 3.8)
project(my_vector_field)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(static_vector_field_node
  src/static_vector_field_node.cpp
  src/static_vector_field_main.cpp
)

add_executable(odom_node
  src/odom_publisher.cpp
)

ament_target_dependencies(odom_node
  rclcpp
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(static_vector_field_node
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  visualization_msgs
  tf2
  Eigen3
)

install(TARGETS
  odom_node
  static_vector_field_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
