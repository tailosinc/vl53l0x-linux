#include "VL53L0X.h"

#include <chrono>
#include <csignal>
#include <exception>
#include <iomanip>
#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include "vl53l0x_ros/Int32Array.h"

// SIGINT (CTRL-C) exit flag and signal handler
volatile sig_atomic_t exitFlag = 0;
void sigintHandler(int) 
{
	exitFlag = 1;
}

int main(int argc, char** argv) 
{
  //These can be read as a ROS param eventually

	// Configuration constants
	// Number of sensors. If changed, make sure to adjust pins and addresses accordingly (ie to match size).
	const int SENSOR_COUNT = 2;
	// GPIO pins to use for sensors' XSHUT. As exported by RPi BCM. BCM is the protocol followed by the 
	// /sys/class/gpio files that are used for GPIO control here. Run `gpio readall` for the pinout mapping.
	const uint8_t pins[SENSOR_COUNT] = { 17, 18 };
	// Sensors' addresses that will be set and used. These have to be unique.
	const uint8_t addresses[SENSOR_COUNT] = {
		VL53L0X_ADDRESS_DEFAULT + 2,
		VL53L0X_ADDRESS_DEFAULT + 4
		// VL53L0X_ADDRESS_DEFAULT + 6,
		// VL53L0X_ADDRESS_DEFAULT + 10,
		// VL53L0X_ADDRESS_DEFAULT + 12,
		// VL53L0X_ADDRESS_DEFAULT + 14
	};

	// Register SIGINT handler
	signal(SIGINT, sigintHandler);

	// Create sensor objects' array
	VL53L0X* sensors[SENSOR_COUNT];

	// Create sensors (and ensure GPIO pin mode)
	for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i) 
  {
		sensors[i] = new VL53L0X(pins[i]);
		sensors[i]->powerOff();
	}
	usleep(10000);
	// Just a check for an early CTRL-C
	if (exitFlag) 
  {
		return 0;
	}

	// For each sensor: create object, init the sensor (ensures power on), set timeout and address
	// Note: don't power off - it will reset the address to default!
	for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i) 
  {
		try {
			// Initialize...
			sensors[i]->initialize();
			// ...set measurement timeout...
			usleep(10000);
			sensors[i]->setTimeout(200);
			// ...set the lowest possible timing budget (high speed mode)...
			sensors[i]->setMeasurementTimingBudget(20000);
			// ...and set I2C address...
			sensors[i]->setAddress(addresses[i]);
			usleep(10000);
			// ...also, notify user.
			std::cout << "Sensor " << i << " initialized, real time budget: " << sensors[i]->getMeasurementTimingBudget() << std::endl;
		} 
    catch (const std::exception & error) 
    {
			std::cerr << "Error initializing sensor " << i << " with reason:" << std::endl << error.what() << std::endl;
			return 1;
		}
	}

	// Start continuous back-to-back measurement
	for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i) 
  {
		try 
    {
			sensors[i]->startContinuous();
		} 
    catch (const std::exception & error) 
    {
			std::cerr << "Error starting continuous read mode for sensor " << i << " with reason:" << std::endl << error.what() << std::endl;
			return 2;
		}
	}

	// We need that variable after the for loop
	int j = 0;
	// Also, set fill and width options for cout so that measurements are aligned
	std::cout << std::setw(4) << std::setfill('0');


  ros::init(argc, argv, "multiple_tof");
  ros::NodeHandle nh;
  ros::Rate rate(10);
  ros::Publisher tof_range_pub = nh.advertise<vl53l0x_ros::Int32Array>("/tof_range", 10);
	// Take the measurements!
  vl53l0x_ros::Int32Array range_array;
	range_array.ranges.resize(SENSOR_COUNT);
  std_msgs::Int32 distance;
  while (ros::ok())
  {
		for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i) 
    {
      distance.data = 8096;
			try {
				// Read the range. Note that it's a blocking call
				distance.data = sensors[i]->readRangeContinuousMillimeters();
			} 
      catch (const std::exception & error) 
      {
				std::cerr << std::endl << "Error getting measurement from sensor " << i << " with reason:" << std::endl << error.what() << std::endl;
				// You may want to bail out here, depending on your application - error means issues on I2C bus read/write.
				// return 3;
				distance.data = 8096;
			}

			// Check for timeout
			if (sensors[i]->timeoutOccurred()) 
      {
				std::cout << "\rToF reading timeout occured for sensor " << i  << std::endl;
			} 

      range_array.ranges[i] = distance;
		}

    tof_range_pub.publish(range_array);
		
		rate.sleep();
	}

	// Clean-up: delete objects, set GPIO/XSHUT pins to low.
	for (int i = 0; i < SENSOR_COUNT; ++i) {
		sensors[i]->stopContinuous();
		delete sensors[i];
	}

	return 0;
}
