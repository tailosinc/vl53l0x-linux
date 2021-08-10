cmake_minimum_required (VERSION 2.8.8 FATAL_ERROR)

# Examples
set (VL53L0X_EXAMPLES
	single
	singleMinimal
	continuousMultipleSensors
)

foreach (example ${VL53L0X_EXAMPLES})
	add_executable (${example}
		${example}.cpp
		../src/VL53L0X.cpp
		../src/I2Cdev.cpp
	)
	target_link_libraries(${example} ${catkin_LIBRARIES} -lwiringPi -lm -lrt -lcrypt)
	add_definitions(-std=c++0x -L${WIRING_PI_WS}/wiringPi)
	target_include_directories (${example} PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/../
		../include
	)
endforeach (example)
