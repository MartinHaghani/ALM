cmake_minimum_required(VERSION 3.0.2)
project(mower_hardware)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mower_msgs
  roscpp
  xesc_driver
  xesc_interface
  xesc_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mower_hardware
  src/mower_hardware.cpp
)

add_dependencies(mower_hardware ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(mower_hardware ${catkin_LIBRARIES})
