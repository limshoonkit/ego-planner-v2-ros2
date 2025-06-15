cmake_minimum_required(VERSION 3.8)
project(moving_obstacles)
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
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(traj_opt REQUIRED)
find_package(traj_utils REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Executable
add_executable(${PROJECT_NAME} src/moving_obstacles.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  traj_opt
  traj_utils
)

target_link_libraries(${PROJECT_NAME}
  traj_utils::traj_utils_lib
)

# Installation
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Export
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  traj_opt
  traj_utils
)

ament_package()