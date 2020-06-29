#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include "VL53L0X.h"

void signal_handler(int s)
{
  printf("Caught signal %d\n",s);
  exit(1); 
}

int main()
{
  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = signal_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);

  VL53L0X sensor;
	sensor.initialize();
	sensor.setTimeout(200);

	while (true) 
  {
		uint16_t distance = sensor.readRangeSingleMillimeters();
		if (sensor.timeoutOccurred()) 
    {
			std::cout << "Sensor timed out!" << std::endl;
		} 
    else 
    {
			std::cout << "Measured distance of " << distance << std::endl;
		}
	}

	return 0;  
}