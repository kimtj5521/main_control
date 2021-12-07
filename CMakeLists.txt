cmake_minimum_required(VERSION 3.0.2)
project(main_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  LIBRARIES main_control
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(main_control_src
  src/main.cpp
  src/main.h
)

add_executable(main_control ${main_control_src})
target_link_libraries(main_control ${catkin_LIBRARIES})