cmake_minimum_required(VERSION 3.8)
project(traj_opt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

add_compile_options(-Wall -O3 -g)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(plan_env REQUIRED)
find_package(traj_utils REQUIRED)
find_package(path_searching REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

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

add_library( traj_opt 
    src/poly_traj_optimizer.cpp 
    )

ament_target_dependencies(traj_opt 
                          std_msgs visualization_msgs 
                          cv_bridge path_searching
                          rclcpp Eigen3 nav_msgs
                          PCL pcl_conversions 
                          plan_env traj_utils)

                          
install(TARGETS traj_opt
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_libraries(traj_opt)
ament_export_include_directories(include)

ament_package()