cmake_minimum_required (VERSION 2.8.8 FATAL_ERROR)

project (vl53l0x_ros)

### Setup compiler
# Set warning flags
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wformat=2 -Wall -Wextra")
# Set optimalization flags
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
# Set C++ standard
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  std_msgs
  message_runtime
  message_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(FILES
UInt16Array.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

add_library(vl53l0x src/I2Cdev.cpp src/VL53L0X.cpp)

add_executable(single_vl53l0x src/ros_single_tof.cpp)
target_link_libraries(single_vl53l0x vl53l0x ${catkin_LIBRARIES})

add_executable(multiple_vl53l0x src/ros_multiple_tof.cpp)
target_link_libraries(multiple_vl53l0x vl53l0x ${catkin_LIBRARIES})

### Examples
# add_subdirectory (examples)