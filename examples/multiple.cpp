#include <VL53L0X.h>
#include <wiringPi.h>

#include <iostream>
#include <unistd.h>

int main() {
	// Initialize GPIO connectivity
	wiringPiSetup();

	// Ensure software shutdown
	digitalWrite(0, LOW);
	digitalWrite(2, LOW);
	digitalWrite(3, LOW);

	// Create sensor objects
	VL53L0X* sensors[3];
	if (argc > 1) {
		// GPIO pin 0 - GPIOX.BIT19 - Pin number 11
		sensors[0] = new VL53L0X(0, VL53L0X_ADDRESS_DEFAULT);
		// GPIO pin 2 - GPIOX.BIT11 - Pin number 13
		sensors[1] = new VL53L0X(2, VL53L0X_ADDRESS_DEFAULT + 8);
		// GPIO pin 3 - GPIOX.BIT9 - Pin number 15
		sensors[2] = new VL53L0X(3, VL53L0X_ADDRESS_DEFAULT + 16);
	} else {
		sensors[0] = new VL53L0X(0);
		sensors[1] = new VL53L0X(2);
		sensors[2] = new VL53L0X(3);
	}

	std::cout << "Sensors created...\n";

	// Initialize sensors and set their addresses
	// Sensor 0 (no address change, power off)
	sensors[0]->init();
	sensors[0]->setTimeout(200);
	sensors[0]->powerOff();
	std::cout << "Sensor 0 initialized\n";
	// Sensor 1 (change address, power off)
	sensors[1]->init();
	sensors[1]->setTimeout(200);
	sensors[1]->setAddress(VL53L0X_ADDRESS_DEFAULT + 1);
	sensors[1]->powerOff();
	std::cout << "Sensor 1 initialized\n";
	// Sensor 2 (change address, no power off)
	sensors[2]->init();
	sensors[2]->setTimeout(200);
	sensors[2]->setAddress(VL53L0X_ADDRESS_DEFAULT + 2);
	std::cout << "Sensor 2 initialized\n";
	// Power on sensors 0 and 1
	sensors[0]->powerOn();
	sensors[1]->powerOn();

	std::cout << "Sensors initialized and powered on\n";

	// 1000 readings for every sensor every half second
	for (int i = 0; i < 1000; ++i) {
		std::cout << i << ":" << std::endl;
		for (int i = 0; i < 3; ++i) {
			uint16_t distance;
			try {
				distance = sensors[i]->readRangeSingleMillimeters();
			} catch (std::string err) {
				std::cerr << err;
				return 1;
			}
			std::cout << "\tSensor" << i << ": ";
			if (sensors[i]->timeoutOccurred()) {
				std::cout << "TIMEOUT\n";
			} else {
				std::cout << distance << std::endl;
			}
		}
		usleep(500*1000);
	}

	// Clean-up
	delete sensors[0];
	delete sensors[1];
	delete sensors[2];
	return 0;
}
