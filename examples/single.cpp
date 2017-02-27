/* This example shows how to get single-shot range
 measurements from the VL53L0X. The sensor can optionally be
 configured with different ranging profiles, as described in
 the VL53L0X API user manual, to get better performance for
 a certain application. This code is based on "Single.ino"
 example from vl53l0x-arduino library, which in turn is based
 on the four "SingleRanging" examples in the VL53L0X API.

 The range readings are in units of mm. */

#include <iostream>
#include <wiringPi.h>
#include <VL53L0X.h>

// Uncomment to enable long range measurements
// #define LONG_RANGE
// Uncomment ONE to enable high speed or high accuracy measurements
// #define HIGH_SPEED
// #define HIGH_ACCURACY

int main() {
	wiringPiSetup();

	VL53L0X sensor;
	try {
		sensor.init();
		sensor.setTimeout(500);
	} catch (std::string & err) {
		std::cerr << err;
		return 1;
	}

	#ifdef LONG_RANGE
		// Lower the return signal rate limit (default is 0.25 MCPS)
		sensor.setSignalRateLimit(0.1);
		// Increase laser pulse periods (defaults are 14 and 10 PCLKs)
		sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
		sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
	#endif

	#if defined HIGH_SPEED
		// Reduce timing budget to 20 ms (default is about 33 ms)
		sensor.setMeasurementTimingBudget(20000);
	#elif defined HIGH_ACCURACY
		// Increase timing budget to 200 ms
		sensor.setMeasurementTimingBudget(200000);
	#endif

	// 5000 readings
	for (int i = 0; i < 1000; ++i) {
		std::cout << i << ": ";
		uint16_t distance = sensor.readRangeSingleMillimeters();
		if (sensor.timeoutOccurred()) {
			std::cout << "TIMEOUT\n";
		} else {
			std::cout << distance << std::endl;
		}
		usleep(500*1000);
	}

	return 0;
}
