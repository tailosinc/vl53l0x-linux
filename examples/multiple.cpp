#include <VL53L0X.h>
#include <wiringPi.h>

#include <iostream>
#include <unistd.h>

int main() {
	// Initialize GPIO connectivity
	wiringPiSetup();

	// Create sensor objects
	VL53L0X* sensors[3];
	// GPIO pin 0 - GPIOX.BIT19 - Pin number 11
	sensors[0] = new VL53L0X(0);
	// GPIO pin 2 - GPIOX.BIT11 - Pin number 13
	sensors[1] = new VL53L0X(2);
	// GPIO pin 3 - GPIOX.BIT9 - Pin number 15
	sensors[2] = new VL53L0X(3);

	// Power off all sensors (before initialization!)
	sensors[0]->powerOff();
	sensors[1]->powerOff();
	sensors[2]->powerOff();

	// Initialize sensors and set their addresses
	// Sensor 0 (no address change, power off)
	sensors[0]->init();
	sensors[0]->powerOff();
	// Sensor 1 (change address, power off)
	sensors[1]->init();
	sensors[1]->setAddress(VL53L0X_ADDRESS_DEFAULT + 1);
	sensors[1]->powerOff();
	// Sensor 2 (change address, no power off)
	sensors[2]->init();
	sensors[2]->setAddress(VL53L0X_ADDRESS_DEFAULT + 2);
	// Power on sensors 0 and 1
	sensors[0]->powerOn();
	sensors[1]->powerOn();

	// 1000 readings for every sensor every half second
	for (int i = 0; i < 1000; ++i) {
		std::cout << i << ":" << std::endl;
		for (int i = 0; i < 3; ++i) {
			uint16_t distance = sensors[i]->readRangeSingleMillimeters();
			std::cout << "\tSensor" << i << ": ";
			if (sensors[i]->timeoutOccurred()) {
				std::cout << "TIMEOUT";
			} else {
				std::cout << distance;
			}
			std::cout << std::endl;
		}
		usleep(500*1000);
	}

	// Clean-up
	delete sensors[0];
	delete sensors[1];
	delete sensors[2];
	return 0;
}
