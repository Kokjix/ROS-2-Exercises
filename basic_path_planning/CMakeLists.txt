cmake_minimum_required(VERSION 3.8)
project(basic_path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(my_path_planning_node
  src/my_path_planning_main.cpp
  src/my_path_planning_node.cpp
)

ament_target_dependencies(my_path_planning_node
  rclcpp
  geometry_msgs
  turtlesim
)

install(TARGETS 
  my_path_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()