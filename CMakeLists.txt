cmake_minimum_required (VERSION 2.8.8 FATAL_ERROR)

project (VL53L0X_linux)
include (GenerateExportHeader)
include (CMakePackageConfigHelpers)

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
set (LIB_VERSION_PATCH 4)
set (LIB_VERSION ${LIB_VERSION_MAJOR}.${LIB_VERSION_MINOR}.${LIB_VERSION_PATCH})

# Set up target
add_library (vl53l0x SHARED ${VL53L0X_SOURCE_FILES})
generate_export_header (vl53l0x)
set_target_properties (vl53l0x PROPERTIES
	VERSION ${LIB_VERSION}
	SOVERSION ${LIB_VERSION_MAJOR}
)

### WiringPi
if (DEFINED ENV{WIRINGPI_DIR})
	set (WIRINGPI_DIR $ENV{WIRINGPI_DIR})
endif ()

# Set WiringPi directory: from cmake -D, from ENV and default
if (DEFINED WIRINGPI_DIR)
	message (STATUS "Using provided WiringPi directory: ${WIRINGPI_DIR}")
	# Include directories are relative to source dir
	set (wiringPi_INCLUDE_DIR "${WIRINGPI_DIR}/wiringPi")
	# Linking directories are relative to build dir
	set (wiringPi_LIBRARIES "${WIRINGPI_DIR}/build/wiringPi/libwiringPi.so.2")

	target_include_directories (vl53l0x PUBLIC ${wiringPi_INCLUDE_DIR})
	target_link_libraries (vl53l0x LINK_PUBLIC ${wiringPi_LIBRARIES})
else ()
	message (STATUS "Using system WiringPi library")
	find_package (wiringPi REQUIRED)

	# Add WiringPi dependency
	# target_include_directories (vl53l0x PUBLIC wiringPi)
	target_link_libraries (vl53l0x LINK_PUBLIC wiringPi)
endif ()

### Install target
set (CMAKE_INSTALL_PREFIX /usr/)
install (TARGETS vl53l0x
	EXPORT vl53l0xTargets
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	INCLUDES DESTINATION include
)
install (FILES "VL53L0X.h"
	DESTINATION include
)

### Exports
write_basic_package_version_file (
  "${CMAKE_CURRENT_BINARY_DIR}/vl53l0x/vl53l0xConfigVersion.cmake"
  VERSION ${LIB_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_file (cmake_modules/vl53l0xConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/vl53l0x/vl53l0xConfig.cmake"
  COPYONLY
)

install (EXPORT vl53l0xTargets
  DESTINATION lib/cmake/vl53l0x
)
install (FILES
  "${CMAKE_CURRENT_BINARY_DIR}/vl53l0x/vl53l0xConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/vl53l0x/vl53l0xConfigVersion.cmake"
  DESTINATION lib/cmake/vl53l0x
)

### Examples
add_subdirectory (examples)

### Package generation
include (cmake_modules/CPack.cmake)
