#include <VL53L0X.h>
#include <wiringPi.h>

#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <unistd.h>

volatile sig_atomic_t exitFlag = 0;
void sigintHandler(int) {
	exitFlag = 1;
}

// chrt -r 1 ./yourprogram

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

	// See multiple.cpp for notes on what's happening here
	for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i) {
		sensors[i]->init();
		sensors[i]->setTimeout(200);
		sensors[i]->setMeasurementTimingBudget(20000);
		sensors[i]->setAddress(addresses[i]);
		std::cout << "Sensor " << i << " initialized\n";
	}

	// Start continuous measurement
	for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i) {
		sensors[i]->startContinuous();
	}

	std::chrono::nanoseconds totalDuration(0);
	std::chrono::nanoseconds maxDuration(0);
	std::chrono::nanoseconds minDuration(std::chrono::seconds(10));
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

	// 1000 readings for every sensor every half second
	int j = 0;
	for (; !exitFlag && j < 1000; ++j) {
		std::cout << "\r" << std::setw(3) << std::setfill('0') << j << " | ";
		for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i) {
			uint16_t distance;
			try {
				distance = sensors[i]->readRangeContinuousMillimeters();
			} catch (std::string & err) {
				std::cerr << err;
				return 1;
			}
			if (sensors[i]->timeoutOccurred()) {
				std::cout << "tout | ";
			} else {
				std::cout << std::setw(4) << distance << " | ";
			}
		}
		std::cout << std::flush;

		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
		std::chrono::nanoseconds duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t2);
		t1 = t2;
		totalDuration += duration;
		if (duration > maxDuration) {
			maxDuration = duration;
		}
		if (duration < minDuration) {
			minDuration = duration;
		}
	}

	unsigned int averageDuration = totalDuration.count()/j;

	std::cout << "\nMax duration: " << maxDuration.count() << std::endl;
	std::cout << "Min duration: " << minDuration.count() << std::endl;
	std::cout << "Avg duration: " << averageDuration << std::endl;

	// Clean-up: delete objects, set GPIO/XSHUT pins to low.
	for (int i = 0; i < SENSOR_COUNT; ++i) {
		sensors[i]->stopContinuous();
		delete sensors[i];
		digitalWrite(pins[i], LOW);
	}
	return 0;
}
