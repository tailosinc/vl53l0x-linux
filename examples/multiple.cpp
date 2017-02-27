#include <VL53L0X.h>
#include <wiringPi.h>

#include <csignal>
#include <iostream>
#include <unistd.h>

volatile sig_atomic_t exitFlag = 0;
void sigintHandler(int) {
	exitFlag = 1;
}

int main() {
	// Configuration constants
	// Number of sensors. If changed, make sure to adjust pins and addresses accordingly (ie to match size).
	const int SENSOR_COUNT = 6;
	// GPIO pins to use for sensors' XSHUT. As exported by WiringPi.
	const uint8_t pins[SENSOR_COUNT] = { 0, 1, 2, 3, 4, 5 };
	// Sensors' addresses that will be set and used. These have to be unique.
	const uint8_t addresses[SENSOR_COUNT] = {
		VL53L0X_ADDRESS_DEFAULT + 2,
		VL53L0X_ADDRESS_DEFAULT + 4,
		VL53L0X_ADDRESS_DEFAULT + 6,
		VL53L0X_ADDRESS_DEFAULT + 10,
		VL53L0X_ADDRESS_DEFAULT + 12,
		VL53L0X_ADDRESS_DEFAULT + 14
	};

	// Register SIGINT handler
	signal(SIGINT, sigintHandler);

	// Initialize GPIO connectivity
	wiringPiSetup();

	// Create sensor objects' array
	VL53L0X* sensors[SENSOR_COUNT];

	// Create sensors (and ensure GPIO pin mode)
	for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i) {
		pinMode(pins[i], OUTPUT);
		sensors[i] = new VL53L0X(pins[i]);
		sensors[i]->powerOff();
	}
	if (exitFlag) {
		return 0;
	}

	// For each sensor: create object, init the sensor (ensures power on), set timeout and address
	// Note: don't power off - it will reset the address to default!
	for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i) {
		// Create...
		// ...init...
		sensors[i]->init();
		// ...set timeout...
		sensors[i]->setTimeout(200);
		// ...and set address
		sensors[i]->setAddress(addresses[i]);
		// Also, notify user.
		std::cout << "Sensor " << i << " initialized\n";
	}

	// 1000 readings for every sensor every half second
	for (int j = 0; !exitFlag && j < 1000; ++j) {
		usleep(500*1000);
		std::cout << "Reading " << j << ":" << std::endl;
		for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i) {
			uint16_t distance;
			try {
				distance = sensors[i]->readRangeSingleMillimeters();
			} catch (std::string & err) {
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
	}

	// Clean-up: delete objects, set GPIO/XSHUT pins to low.
	for (int i = 0; i < SENSOR_COUNT; ++i) {
		delete sensors[i];
		digitalWrite(pins[i], LOW);
	}
	return 0;
}
