cmake_minimum_required (VERSION 2.8.8 FATAL_ERROR)

if (NOT WIRING_PI_HEADER_DIR OR NOT WIRING_PI_SHARED_OBJ)
	message (FATAL_ERROR "This CMakeLists is not intended for standalone usage. Use one from main directory instead.")
endif ()

# Examples
set (VL53L0X_EXAMPLES
	single
	singleMultipleSensors
	continuousMultipleSensors
)

foreach (example ${VL53L0X_EXAMPLES})
	add_executable (${example} EXCLUDE_FROM_ALL ${example}.cpp)
	target_include_directories (${example} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../ ${WIRING_PI_HEADER_DIR})
	target_link_libraries (${example} LINK_PUBLIC vl53l0x ${WIRING_PI_SHARED_OBJ})
endforeach (example)

# Examples make target (`make examples` to build)
add_custom_target (examples DEPENDS ${VL53L0X_EXAMPLES})
