cmake_minimum_required(VERSION 3.8)
project(traj_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

add_compile_options(-Wall -O3 -g)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
# find_package(PCL 1.7 REQUIRED) #to help find Eigen


rosidl_generate_interfaces(traj_utils
  "msg/PolyTraj.msg"
  "msg/DataDisp.msg"
  "msg/MINCOTraj.msg"

  DEPENDENCIES std_msgs geometry_msgs rclcpp builtin_interfaces
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
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

ament_export_include_directories(include)

link_directories(${PCL_LIBRARY_DIRS})

add_library(traj_utils_lib SHARED
  src/planning_visualization.cpp
)

ament_target_dependencies(traj_utils_lib
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  Eigen3
  PCL
)

install(TARGETS traj_utils_lib
  EXPORT export_traj_utils_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}
)

ament_export_targets(export_traj_utils_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs geometry_msgs visualization_msgs Eigen3 PCL)

ament_package()
