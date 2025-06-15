cmake_minimum_required(VERSION 3.8)
project(swarm_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

add_compile_options(-Wall -O3 -g)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(traj_utils REQUIRED)
find_package(traj_opt REQUIRED) # for traj2odom_node
find_package(Boost REQUIRED COMPONENTS thread)

set(ENABLE_TCP false) # requires zmq, zmqpp 

set(DEPENDENCIES 
  rclcpp
  rmw
  rosidl_typesupport_cpp  
  rcutils
  std_msgs
  sensor_msgs
  quadrotor_msgs
  nav_msgs
  geometry_msgs
  traj_utils
  traj_opt
  Eigen3
  Boost
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  SYSTEM
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(bridge_node_udp
  src/bridge_node_udp.cpp 
  )
  ament_target_dependencies(bridge_node_udp ${DEPENDENCIES})

if(ENABLE_TCP)

  add_executable(bridge_node_tcp
    src/bridge_node_tcp.cpp 
    )

  ament_target_dependencies(bridge_node_tcp  ${DEPENDENCIES} zmq zmqpp)

endif(ENABLE_TCP)

add_executable(traj2odom_node
  src/traj2odom_node.cpp 
  )

 ament_target_dependencies(traj2odom_node ${DEPENDENCIES})


# Install targets
install(
  TARGETS bridge_node_udp traj2odom_node
  DESTINATION lib/${PROJECT_NAME}
)

IF(ENABLE_TCP)
  install(
    TARGETS bridge_node_tcp
    DESTINATION lib/${PROJECT_NAME}
  )
ENDIF(ENABLE_TCP)

# 安装 launch 文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
