cmake_minimum_required(VERSION 2.8.3)
project(livox_to_pointcloud2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  livox_ros_driver2
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Declare things to be passed to dependent projects
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES livox_to_pointcloud2
#  CATKIN_DEPENDS roscpp sensor_msgs livox_ros_driver
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/livox_to_pointcloud2.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

