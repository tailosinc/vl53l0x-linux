cmake_minimum_required (VERSION 2.8.8 FATAL_ERROR)

project (VL53L0X_linux)

### Setup compiler
# Try to use GCC/G++ 6
include (cmake_modules/SetGCC6.cmake)
# Set warning flags
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wformat=2 -Wall -Wextra")
# Set optimalization flags
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

### Main library target
# Set source files
set (VL53L0X_SOURCE_FILES
	VL53L0X.cpp
)

set (LIB_VERSION_MAJOR 0)
set (LIB_VERSION_MINOR 1)
set (LIB_VERSION_PATCH 3)
set (LIB_VERSION ${LIB_VERSION_MAJOR}.${LIB_VERSION_MINOR}.${LIB_VERSION_PATCH})

# Set up target
add_library (vl53l0x SHARED ${VL53L0X_SOURCE_FILES})

if (DEFINED ENV{WIRINGPI_DIR})
	set (WIRINGPI_DIR $ENV{WIRINGPI_DIR})
endif ()

### WiringPi
# Set WiringPi directory: from cmake -D, from ENV and default
if (DEFINED WIRINGPI_DIR)
	message (STATUS "Using provided WiringPi directory: ${WIRINGPI_DIR}")
	# Include directories are relative to source dir
	set (WIRINGPI_INCLUDE_DIR "${WIRINGPI_DIR}/wiringPi")
	# Linking directories are relative to build dir
	set (WIRINGPI_LIBRARIES "${WIRINGPI_DIR}/build/wiringPi/libwiringPi.so.2")
else ()
	message (STATUS "Using system WiringPi library")
	include (cmake_modules/FindwiringPi.cmake)
	if (NOT WIRINGPI_FOUND)
		message (FATAL_ERROR "WiringPi library not found. Install it or provide via $WIRINGPI_DIR.")
	endif ()
endif ()

# Add WiringPi dependency
target_include_directories (vl53l0x PUBLIC ${WIRINGPI_INCLUDE_DIR})
target_link_libraries (vl53l0x LINK_PUBLIC ${WIRINGPI_LIBRARIES})

### Examples
add_subdirectory (examples)

### Package generation
include (cmake_modules/CPack.cmake)
