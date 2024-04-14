cmake_minimum_required(VERSION 3.5)
project(livox_to_pointcloud2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# Declare executables
ament_auto_add_executable(livox_to_pointcloud2_node src/livox_to_pointcloud2_node.cpp src/livox_to_pointcloud2.cpp)

# Install rules
ament_auto_package(INSTALL_TO_SHARE
  launch
)