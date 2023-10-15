cmake_minimum_required(VERSION 3.5)
project(livox_to_pointcloud2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

include_directories(include)

add_executable(livox_to_pointcloud2_node src/livox_to_pointcloud2.cpp)
ament_target_dependencies(livox_to_pointcloud2_node rclcpp sensor_msgs livox_ros_driver2)

install(TARGETS
  livox_to_pointcloud2_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

