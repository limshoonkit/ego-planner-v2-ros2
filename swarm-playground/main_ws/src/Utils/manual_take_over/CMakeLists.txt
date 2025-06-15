cmake_minimum_required(VERSION 3.10)
project(manual_take_over)

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
find_package(Eigen3 REQUIRED)

find_package(quadrotor_msgs REQUIRED)

# Include directories
include_directories(
  include
  SYSTEM
  ${EIGEN3_INCLUDE_DIR}
)

# Node 1: manual_take_over
add_executable(manual_take_over
  src/manual_take_over.cpp
)
ament_target_dependencies(manual_take_over
  rclcpp
  std_msgs
  sensor_msgs
  quadrotor_msgs
)

# Node 2: manual_take_over_station
add_executable(manual_take_over_station
  src/ground_station.cpp
)
ament_target_dependencies(manual_take_over_station
  rclcpp
  std_msgs
  sensor_msgs
  quadrotor_msgs
)

# Install executables
install(TARGETS
  manual_take_over
  manual_take_over_station
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()