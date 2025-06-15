cmake_minimum_required(VERSION 3.8)
project(assign_goals)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

add_compile_options(-Wall -O3 -g)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(quadrotor_msgs REQUIRED)
find_package(traj_utils REQUIRED)
find_package(uav_utils REQUIRED)

# Include directories
include_directories(
  include
  SYSTEM
  ${EIGEN3_INCLUDE_DIR}
)

# Executable
add_executable(assign_goals_node
  src/assign_goals_node.cpp
)

# Link dependencies
ament_target_dependencies(assign_goals_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  quadrotor_msgs
  traj_utils
  uav_utils
)

# Install
install(TARGETS assign_goals_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()
