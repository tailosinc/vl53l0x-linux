#include <iostream>
#include "VL53L0X.h"

int main() {
	VL53L0X sensor;
	sensor.initialize();
	sensor.setTimeout(200);

	for (int i = 0; i < 10; ++i) {
		sensor.readRangeSingleMillimeters();
		if (sensor.timeoutOccurred()) 
    {
			std::cout << "Sensor timed out!" << std::endl;
		} 
    else 
    {
			std::cout << "Hello world!" << std::endl;
      break;
		}
	}

	return 0;
}
