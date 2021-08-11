cmake_minimum_required (VERSION 2.8.8 FATAL_ERROR)

# Examples
set (VL53L0X_TESTS
	hello_sensor
	read_sensor
)

foreach (test ${VL53L0X_TESTS})
	add_executable (${test}
		${test}.cpp
		../src/VL53L0X.cpp
		../src/I2Cdev.cpp
	)
	target_link_libraries(${example} ${catkin_LIBRARIES} -lwiringPi)
	add_definitions(-std=c++0x -L${WIRING_PI_WS}/wiringPi)
	target_include_directories (${test} PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/../
		../include
	)
endforeach (test)
