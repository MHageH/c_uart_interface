#include "mavlink_control.h"

mavlink_set_position_target_local_ned_t initial_position;

int value_mg_x, value_mg_y, value_mg_z;

// Scheduler related
int lock_read_messages = 0;
bool lock_ = false;

int Program_counter = 0; 

float seconds = 0;

int main(void){
	autopilot_intialize();

	#ifdef STM32F4
	// initialise LIS3DSH 
	lis3dsh_init();

	#endif

	serial_start();
	autopilot_start();

	while (1) {
		commands();
	}

	return 0;
	}

// Scheduler
void commands(void){
	//
		operation(3);
	}
void operation (float timer){
	#ifndef STM32F4
	    time_t end;
	    time_t begin =  time(NULL);
	#endif 

	read_messages_helper();

	global_read_messages();
	autopilot_write_helper();

	mavlink_set_position_target_local_ned_t set_point;
	mavlink_set_position_target_local_ned_t ip = initial_position;

		#ifdef STM32F4

		value_mg_x = ((lis3dsh_read_reg(ADD_REG_OUT_X_H) << 8) | lis3dsh_read_reg(ADD_REG_OUT_X_L));
		value_mg_y = ((lis3dsh_read_reg(ADD_REG_OUT_Y_H) << 8) | lis3dsh_read_reg(ADD_REG_OUT_Y_L));
		value_mg_z = ((lis3dsh_read_reg(ADD_REG_OUT_Z_H) << 8) | lis3dsh_read_reg(ADD_REG_OUT_Z_L));

		/* transform X value from two's complement to 16-bit int */
		value_mg_x = two_compl_to_int16(value_mg_x);
		/* convert X absolute value to mg value */
		value_mg_x = value_mg_x * SENS_2G_RANGE_MG_PER_DIGIT;

		char x_acc[sizeof(value_mg_x)];
		send_string("X acceleration value :");
		send_string(custom_itoa(value_mg_x, x_acc));
		send_string("\r\n");

		/* transform Y value from two's complement to 16-bit int */
		value_mg_y = two_compl_to_int16(value_mg_y);
		/* convert Y absolute value to mg value */
		value_mg_y = value_mg_y * SENS_2G_RANGE_MG_PER_DIGIT;

		char y_acc[sizeof(value_mg_y)];
		send_string("Y acceleration value :");
		send_string(custom_itoa(value_mg_y, y_acc));
		send_string("\r\n");

		/* transform Z value from two's complement to 16-bit int */
		value_mg_z = two_compl_to_int16(value_mg_z);
		/* convert Z absolute value to mg value */
		value_mg_z = value_mg_z * SENS_2G_RANGE_MG_PER_DIGIT;

		char z_acc[sizeof(value_mg_z)];
		send_string("Z acceleration value :");
		send_string(custom_itoa(value_mg_z, z_acc));
		send_string("\r\n");

		#endif

	switch(Program_counter){
			case 0 : 
				enable_offboard_control();
				//enable_offboard_control();
				#ifdef STM32F4	
					gpio_toggle(GPIOD, GPIO13); 
				#endif 
				break;
			case 1 :
					set__( 1 , 0, - 2.5, set_point); break;
			case 2 :
					#ifdef STM32F4
						set__( 1 + value_mg_y/100 , value_mg_x/100, - 5, set_point); break; 
					#else 
						set__( 5 , 0, - 5, set_point); break;
					#endif
			/* OLD  ::
			case 3 :
					set__( -2 , -2.5, - 2.5, set_point); break;
			case 4 :
					set__( -2 , 2.5, - 2.5, set_point); break; 
			case 5 :
					set__( -2 , 2.5, - 1000, set_point); break; 
			*/
			default : break;
		}
		#ifndef STM32F4
			end =  time(NULL);
		if ((end - begin) == timer){
				Program_counter++;
				end = 0;
			}

		#else 
			
		if (seconds == timer){
			Program_counter++;
			seconds = 0;
		}
		#endif
		// OLD :: if (Program_counter == 0 || Program_counter == 6) { Program_counter = 1;}
		if (Program_counter == 0 || Program_counter == 3) { Program_counter = 2;}
	}

// Function Helpers
void read_messages_helper(void){
	if(lock_read_messages == 0){
			read_messages();
	}
	lock_read_messages = 1;
	}
void autopilot_write_helper(void){
	if (lock_ == false) {
			autopilot_write();		
	}
	lock_ = true;
	}