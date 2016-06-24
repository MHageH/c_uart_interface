#include <commands.h>

int arm_lock = 0;
int offboard_control_lock = 0;

int value_mg_x, value_mg_y, value_mg_z;

// Scheduler related
bool lock_ = false;
int Program_counter = 0; 

volatile float seconds = 0;
volatile float omega = 0;

#ifndef STM32F4
	time_t end;
	time_t begin =  time(NULL);
#endif

void operation (float timer){
	read_messages();
	autopilot_start();
	autopilot_write_helper();

	mavlink_set_position_target_local_ned_t set_point;

		#ifdef STM32F4

		value_mg_x = ((lis3dsh_read_reg(ADD_REG_OUT_X_H) << 8) | lis3dsh_read_reg(ADD_REG_OUT_X_L));
		value_mg_y = ((lis3dsh_read_reg(ADD_REG_OUT_Y_H) << 8) | lis3dsh_read_reg(ADD_REG_OUT_Y_L));
		value_mg_z = ((lis3dsh_read_reg(ADD_REG_OUT_Z_H) << 8) | lis3dsh_read_reg(ADD_REG_OUT_Z_L));

		// transform X value from two's complement to 16-bit int 
		value_mg_x = two_compl_to_int16(value_mg_x);
		// convert X absolute value to mg value 
		value_mg_x = value_mg_x * SENS_2G_RANGE_MG_PER_DIGIT;

		// transform Y value from two's complement to 16-bit int 
		value_mg_y = two_compl_to_int16(value_mg_y);
		// convert Y absolute value to mg value 
		value_mg_y = value_mg_y * SENS_2G_RANGE_MG_PER_DIGIT;

		// transform Z value from two's complement to 16-bit int 
		value_mg_z = two_compl_to_int16(value_mg_z);
		// convert Z absolute value to mg value 
		value_mg_z = value_mg_z * SENS_2G_RANGE_MG_PER_DIGIT;

		#endif

	switch(Program_counter){
			case 0 :
					arm_sequence(); break;
			case 1 :
					offboard_control_sequence(); break;
			case 2 :
					#ifndef STM32F4
						printf("Set point 1 \n");
					#endif

					set_velocity( - 0.25 , - 0.0 , - 0.50 , set_point);
					set_yaw (ip.yaw, set_point);

					#ifdef STM32F4

						if (offboard_control_lock == 0){
							control_status = false;
							enable_offboard_control();
							offboard_control_lock = 1;
						}		

					#endif 

					set__( - 1.0 + ip.x , ip.y , ip.z - 1, set_point); break;
			case 3 :
					#ifdef STM32F4
						set_yaw (ip.yaw, set_point);

						set__( - 1.0 - (float) (value_mg_y/500) + ip.x , ip.y - (float)(value_mg_x/500), ip.z - 2.0, set_point); break; 
					#else 
						set_yaw (ip.yaw, set_point);

						set__( 1.0 + ip.x , ip.y, ip.z - 2, set_point); break;
					#endif
			case 4 : 
					#ifndef STM32F4
					printf("Current Initial position : x = %f , y = %f , z = %f\n", ip.x, ip.y, ip.z);
					printf("Current set point : x = %f, y = %f z=%f\n", ip.x, ip.y, ip.z - 0.25);
					
					// printf("Set point 3\n");
					#endif
					set_yaw (ip.yaw, set_point);

					set__( - 1.0 - (float) (value_mg_y/500) + ip.x , ip.y - (float)(value_mg_x/500), ip.z - 1.0, set_point); break;
			case 5 : 
					disable_offboard_control_sequence(); 
					Program_counter = 6;
					break;
			case 6 : 
					#ifndef STM32F4
						printf("Ideling..\n");
					#else 
						__asm__("NOP");
					#endif
						break;
						Program_counter = 7;
			case 7 :
					disarm_sequence();
					Program_counter = 8;
					break;

			default : break;
		}
		program_counter_sequence(timer);

			if (Program_counter == 8) { Program_counter = 8;}
	}
void operation_extended (float timer){
	read_messages();
	autopilot_start();
	autopilot_write_helper();

	mavlink_set_position_target_local_ned_t set_point;

		#ifdef STM32F4

		value_mg_x = ((lis3dsh_read_reg(ADD_REG_OUT_X_H) << 8) | lis3dsh_read_reg(ADD_REG_OUT_X_L));
		value_mg_y = ((lis3dsh_read_reg(ADD_REG_OUT_Y_H) << 8) | lis3dsh_read_reg(ADD_REG_OUT_Y_L));
		value_mg_z = ((lis3dsh_read_reg(ADD_REG_OUT_Z_H) << 8) | lis3dsh_read_reg(ADD_REG_OUT_Z_L));

		// transform X value from two's complement to 16-bit int 
		value_mg_x = two_compl_to_int16(value_mg_x);
		// convert X absolute value to mg value 
		value_mg_x = value_mg_x * SENS_2G_RANGE_MG_PER_DIGIT;

		// transform Y value from two's complement to 16-bit int 
		value_mg_y = two_compl_to_int16(value_mg_y);
		// convert Y absolute value to mg value 
		value_mg_y = value_mg_y * SENS_2G_RANGE_MG_PER_DIGIT;

		// transform Z value from two's complement to 16-bit int 
		value_mg_z = two_compl_to_int16(value_mg_z);
		// convert Z absolute value to mg value 
		value_mg_z = value_mg_z * SENS_2G_RANGE_MG_PER_DIGIT;

		#endif

	switch(Program_counter){
			case 0 :
					arm_sequence(); break;
			case 1 :
					offboard_control_sequence(); break;
			case 2 :
					#ifndef STM32F4
						printf("Set point 1 \n");
					#endif

					set_velocity( - 0.25 , - 0.0 , - 0.50 , set_point);
					set_yaw (ip.yaw, set_point);

					#ifdef STM32F4

						if (offboard_control_lock == 0){
							control_status = false;
							enable_offboard_control();
							offboard_control_lock = 1;
						}		

					#endif 

					set__( 1.0 + ip.x , ip.y , ip.z - 1, set_point); break;
			case 3 :
					#ifdef STM32F4
						set__( 1.0 + (float) (value_mg_y/100) + ip.x , ip.y + (float)(value_mg_x/100), ip.z - 2.0, set_point); break; 
					#else 
						set__( 1.0 + ip.x , ip.y, ip.z - 2, set_point); break;
					#endif

			default : break;
		}

		program_counter_sequence(timer);

			if (Program_counter == 4) { Program_counter = 3;}
	}
void square_operation (float timer){
	read_messages();
	autopilot_start();
	autopilot_write_helper();

	mavlink_set_position_target_local_ned_t set_point;


	switch(Program_counter){
			case 0 :
					arm_sequence(); break;
			case 1 :
					offboard_control_sequence(); break;
			case 2 :
					set_velocity( - 0.25 , - 0.25 , - 0.25 , set_point);

					set__(ip.x ,ip.x , ip.z - 2.0, set_point);

					#ifdef STM32F4

						if (offboard_control_lock == 0){
							control_status = false;
							enable_offboard_control();
							offboard_control_lock = 1;
						}		

					#endif 

					break;
			case 3 :
					set__(ip.x + 1.0, ip.x, ip.z - 2.0, set_point); break;
			case 4 :
					set__(ip.x + 1.0, ip.x + 1.0 , ip.z - 2.0, set_point); break;
			case 5 :
					set__(ip.x  ,ip.x + 1.0 , ip.z  -2.0, set_point); break; 
			case 6 :
					set__(ip.x ,ip.x  , ip.z  -1.0, set_point); break;
			case 7 : 
					#ifndef STM32F4
						printf("Disabled offboard control\n");
					#endif

					disable_offboard_control(); 
					Program_counter = 8; 
					break;
			case 8 : 
					#ifndef STM32F4
						printf("Ideling..\n");
					#else 
						__asm__("NOP");
					#endif
						break;
			case 9 :
					#ifndef STM32F4
						printf("Disarmed \n");
					#endif

					#ifdef STM32F4
						autopilot_write();
					#endif 

					autopilot_disarm();

					Program_counter = 10;
					break;

			default : break;
		}

		#ifndef STM32F4
			end =  time(NULL);

		if ((end - begin) >= timer){
				begin = time(NULL);
				printf("Operation : %d \n", Program_counter);
				Program_counter++;
			}

		#else 
			
		if (seconds >= timer){
			Program_counter++;
			seconds = 0;
		}

		#endif

		if (Program_counter == 10) { Program_counter = 10;}
	}
void circle_operation (float timer){
	read_messages();
	autopilot_start();
	autopilot_write_helper();

	mavlink_set_position_target_local_ned_t set_point;

	switch(Program_counter){
			case 0 :
					arm_sequence(); break;
			case 1 :
					offboard_control_sequence(); break;
			case 2 :
					set_circle(ip.x + 1, ip.y + (float) omega, ip.z - 1.0 , set_point);
					omega++;
					if (omega > 360) omega = 0;
					break;
			default : break;
		}

		#ifndef STM32F4

		program_counter_sequence(timer);

		#endif
		if (Program_counter == 0 || Program_counter == 3) { Program_counter = 2;}
	}
void automatic_takeoff (float timer){
	read_messages();
	autopilot_start();
	autopilot_write_helper();

	mavlink_set_position_target_local_ned_t set_point;

	switch(Program_counter){
			case 0 :
					arm_sequence(); break;
			case 1 :
					offboard_control_sequence(); break;
			case 2 :
					#ifndef STM32F4
					printf("Current Initial position : x = %f , y = %f , z = %f\n", ip.x, ip.y, ip.z);
					printf("Current set point : x = %f, y = %f z=%f\n", ip.x, ip.y, ip.z - 1);

					#endif

					set_yaw (ip.yaw, set_point);
					set_velocity( - 0.25 , - 0.0 , - 0.50 , set_point);

					#ifdef STM32F4

						if (offboard_control_lock == 0){
							control_status = false;
							enable_offboard_control();
							offboard_control_lock = 1;
						}		

					#endif 

					set__( ip.x , ip.y , ip.z - 2.00, set_point); break;
			case 3 : 
					#ifndef STM32F4
					printf("Current set point : x = %f, y = %f z=%f\n", ip.x, ip.y, ip.z - 0.25);

					#endif
					set_yaw (ip.yaw, set_point);
					set__( ip.x , ip.y , ip.z - 2.00 , set_point); break;
			case 4 : 
					#ifndef STM32F4
					printf("Current set point : x = %f, y = %f z=%f\n", ip.x, ip.y, ip.z - 0.25);

					#endif

					set_yaw (ip.yaw, set_point);
					set__( ip.x , ip.y , ip.z - 1.00 , set_point); break;
			case 5 : 
					#ifndef STM32F4
					printf("Current set point : x = %f, y = %f z=%f\n", ip.x, ip.y, ip.z - 0.25);
					
					#endif

					set_yaw (ip.yaw, set_point);
					set__( ip.x, ip.y , ip.z - 0.50 , set_point); break;
			case 6 : 
					disable_offboard_control_sequence();
					Program_counter = 7; 
					break;
			case 7 : 
					#ifndef STM32F4
						printf("Ideling..\n");
					#else 
						__asm__("NOP");
					#endif
						break;
						Program_counter = 8;
			case 8 :
					disarm_sequence();
					Program_counter = 9;
					break;

			default : break;
		}
		
		program_counter_sequence(timer);

			if (Program_counter == 9) { Program_counter = 9;}
	}
void flight_control_sequence (float timer){
	read_messages();
	autopilot_start();
	autopilot_write_helper();

	mavlink_set_position_target_local_ned_t set_point;

	switch(Program_counter){
			case 0 :
					arm_sequence(); break;
			case 1 :
					offboard_control_sequence(); break;
			case 2 :
					#ifndef STM32F4
					printf("Current Initial position : x = %f , y = %f , z = %f\n", ip.x, ip.y, ip.z);
					printf("Current set point : x = %f, y = %f z=%f\n", ip.x, ip.y, ip.z - 1);

					// printf("Set point 1\n");
					#endif

					set_velocity( - 0.25 , - 0.0 , - 0.50 , set_point);

					#ifdef STM32F4

						if (offboard_control_lock == 0){
							control_status = false;
							enable_offboard_control();
							offboard_control_lock = 1;
						}		

					#endif 

					set__( ip.x , ip.y , ip.z - 2.00, set_point); break;
			case 3 : 
					#ifndef STM32F4
					printf("Current Initial position : x = %f , y = %f , z = %f\n", ip.x, ip.y, ip.z);
					printf("Current set point : x = %f, y = %f z=%f\n", ip.x, ip.y, ip.z - 0.25);

					#endif
					set__( ip.x - 2.00 , ip.y , ip.z -2.00 , set_point); break;
			case 4 : 
					#ifndef STM32F4
					printf("Current Initial position : x = %f , y = %f , z = %f\n", ip.x, ip.y, ip.z);
					printf("Current set point : x = %f, y = %f z=%f\n", ip.x, ip.y, ip.z - 0.25);

					#endif
					set__( ip.x , ip.y , ip.z - 2.00 , set_point); break;

			case 5 : 
					#ifndef STM32F4
					printf("Current Initial position : x = %f , y = %f , z = %f\n", ip.x, ip.y, ip.z);
					printf("Current set point : x = %f, y = %f z=%f\n", ip.x, ip.y, ip.z - 0.25);
					
					#endif

					set__( ip.x , ip.y - 2.00 , ip.z - 2.00, set_point); break;
			case 6 : 
					#ifndef STM32F4
					printf("Current Initial position : x = %f , y = %f , z = %f\n", ip.x, ip.y, ip.z);
					printf("Current set point : x = %f, y = %f z=%f\n", ip.x, ip.y, ip.z - 0.25);

					#endif
					set__( ip.x , ip.y + 0.00 , ip.z - 2.00 , set_point); break;

			case 7 : 
					#ifndef STM32F4
					printf("Current Initial position : x = %f , y = %f , z = %f\n", ip.x, ip.y, ip.z);
					printf("Current set point : x = %f, y = %f z=%f\n", ip.x, ip.y, ip.z - 0.25);
					
					#endif

					set__( ip.x + 0.00 , ip.y , ip.z - 0.50 , set_point); break;
			case 8 : 
					disable_offboard_control_sequence();

					Program_counter = 9; 
					break;
			case 9 : 
					#ifndef STM32F4
						printf("Ideling..\n");
					#else 
						__asm__("NOP");
					#endif
						break;
					Program_counter = 10;
			case 10 :
					disarm_sequence();
					Program_counter = 11;
					break;

			default : break;
		}
		
		program_counter_sequence(timer);

			if (Program_counter == 11) { Program_counter = 11;}
	}

// Flight functions 
void arm_sequence (void){
	#ifndef STM32F4

		if (current_messages.heartbeat.base_mode != ARMED_BASE_MODE && arm_lock == 0){
			printf("Arming\n");
			autopilot_arm();
			arm_lock = 1;
		}

		usleep(100);

	#else 

		autopilot_write();
		autopilot_arm();

	#endif
	}
void offboard_control_sequence(void){
	#ifndef STM32F4
		printf("Enable offboard control\n");
	#endif

	autopilot_write();
	enable_offboard_control();
	autopilot_write();

	if (current_messages.heartbeat.base_mode != OFFBOARD_CONTROL_BASE_MODE){
		control_status = false;
		autopilot_write();
		enable_offboard_control();
		autopilot_write();
	}		
		
	#ifndef STM32F4
		usleep(100);
	#endif

	Program_counter = 2;
	}
void disable_offboard_control_sequence(void){
	#ifndef STM32F4
		printf("Disabled offboard control\n");
	#endif

	disable_offboard_control(); 
	} 
void disarm_sequence (void){
	#ifndef STM32F4
		printf("Disarmed \n");
	#else 
		autopilot_write();
	#endif

	autopilot_disarm();

	if (current_messages.heartbeat.base_mode == ARMED_BASE_MODE){
		autopilot_disarm();
		}

	}
void program_counter_sequence(float timer){
	#ifndef STM32F4

		end =  time(NULL);
		
		if ((end - begin) >= timer){
			begin = time(NULL);
			printf("Operation : %d \n", Program_counter);
			Program_counter++;
		}

	#else 
			
		if (seconds >= timer){
			Program_counter++;
			seconds = 0;
		}

	#endif
	}

void autopilot_write_helper(void){
	if (lock_ == false) {
			autopilot_write();		
		}
		lock_ = true;
	}


