cmake_minimum_required (VERSION 2.8.8 FATAL_ERROR)

# Set WiringPi directory
if (DEFINED WIRINGPI_DIR)
	message ("Using provided WiringPi directory: ${WIRINGPI_DIR}")
elseif (DEFINED ENV{WIRINGPI_DIR})
	set (WIRINGPI_DIR $ENV{WIRINGPI_DIR})
	message ("Using provided WiringPi directory: ${WIRINGPI_DIR}")
else ()
	set (WIRINGPI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../wiringPi")
	message ("Using default WiringPi directory: ${WIRINGPI_DIR}")
endif ()

# Set compiler - not recommended way but the cleanest one
set (CMAKE_C_COMPILER "gcc-6")
set (CMAKE_CXX_COMPILER "g++-6")

# Set warning flags
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wformat=2 -Wall -Wextra")
# Set optimalization flags
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# Set source files
set (VL53L0X_SOURCE_FILES
	VL53L0X.cpp
)

# Include directories are relative to source dir
set (WIRING_PI_HEADER_DIR "${WIRINGPI_DIR}/wiringPi")
# Linking directories are relative to build dir
set (WIRING_PI_SHARED_OBJ "${WIRINGPI_DIR}/build/wiringPi/libwiringPi.so.2")

project(VL53L0X_linux)
add_library (vl53l0x SHARED ${VL53L0X_SOURCE_FILES})
target_include_directories (vl53l0x PUBLIC ${WIRING_PI_HEADER_DIR})
target_link_libraries (vl53l0x LINK_PUBLIC ${WIRING_PI_SHARED_OBJ})
