cmake_minimum_required(VERSION 3.22.1)
project(map_processor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Find PCL
find_package(PCL 1.10 REQUIRED COMPONENTS Common Io KdTree Filters)

# Add executable
add_executable(map_processor_node src/map_processor_node.cpp)

target_link_libraries(map_processor_node ${PCL_LIBRARIES})

# Add PCL include directories
target_include_directories(map_processor_node PUBLIC ${PCL_INCLUDE_DIRS})

# Link ROS 2 dependencies
ament_target_dependencies(map_processor_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

# Install executable
install(TARGETS map_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch maps config
  DESTINATION share/${PROJECT_NAME}/
)

# Check and lint
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()