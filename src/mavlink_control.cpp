#include "mavlink_control.h"

int main(void){
	autopilot_intialize();

	#ifdef STM32F4

	// initialise LIS3DSH accelerometer
	lis3dsh_init();

	#endif

	serial_start();

	while (1) {
		commands();
	}

	return 0;
	}

// Scheduler
void commands(void){
		 operation(10);
		// operation_extended(10);
		// square_operation(7);
		// circle_operation(5);
		// automatic_takeoff(10);
		// flight_control_sequence(10);
		}
