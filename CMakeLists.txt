cmake_minimum_required (VERSION 2.8.8 FATAL_ERROR)

project(VL53L0X_linux)

# Set this accordingly
set (WIRING_PI_DIR "/path/to/wiringPi")

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -O2 -Wformat=2 -Wall -Wextra")

set (VL53L0X_SOURCE_FILES
	VL53L0X.cpp
)

# Include directories are relative to source dir
set (WIRING_PI_HEADER_DIR "${WIRING_PI_DIR}/wiringPi")
# Linking directories are relative to build dir
set (WIRING_PI_SHARED_OBJ "${WIRING_PI_DIR}/build/wiringPi/libwiringPi.so.2")

add_library (vl53l0x SHARED ${VL53L0X_SOURCE_FILES})
target_include_directories (vl53l0x PUBLIC ${WIRING_PI_HEADER_DIR})
target_link_libraries (vl53l0x LINK_PUBLIC ${WIRING_PI_SHARED_OBJ})
