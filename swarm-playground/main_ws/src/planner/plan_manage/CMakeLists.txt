cmake_minimum_required(VERSION 3.8)
project(ego_planner)

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
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Custom packages
find_package(quadrotor_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(traj_utils REQUIRED)
find_package(traj_opt REQUIRED)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(SYSTEM
  include
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# === ego_planner_node ===
add_executable(ego_planner_node
  src/ego_planner_node.cpp 
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
)

target_include_directories(ego_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(ego_planner_node
  rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs
  cv_bridge plan_env path_searching traj_opt traj_utils
  quadrotor_msgs Eigen3 PCL
)

# === traj_server ===
add_executable(traj_server src/traj_server.cpp)

target_include_directories(traj_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(traj_server
  rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs
  cv_bridge plan_env path_searching traj_opt traj_utils
  quadrotor_msgs Eigen3 PCL
)

# === Installation ===
install(TARGETS
  ego_planner_node
  traj_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# === Export for downstream packages ===
ament_export_include_directories(include)
ament_export_libraries()
ament_export_dependencies(
  rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs
  cv_bridge plan_env path_searching traj_opt traj_utils
  quadrotor_msgs Eigen3 PCL
)

ament_package()