/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *   
 *   Porting to STM32F4-discovery done by : 
 * 			 Mohamed Hage Hassan, <mohamed.hagehassan@yahoo.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "serial_port.h"


mavlink_status_t status;
uint8_t          msgReceived = false;

volatile uint32_t ticks_sec = 0;
volatile float seconds = 0;

Mavlink_Messages current_messages;
mavlink_set_position_target_local_ned_t current_setpoint;
mavlink_set_position_target_local_ned_t initial_position;

char control_status;

int system_id;
int autopilot_id;
int companion_id;
bool time_to_exit;

int lock_read_messages = 0;

int Program_counter = 0; // Program counter for the scheduler
bool lock_ = false;

void serial_start(void){
    //rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
  	 rcc_periph_clock_enable (RCC_GPIOA);
  	 rcc_periph_clock_enable (RCC_GPIOD);
  	 rcc_periph_clock_enable (RCC_GPIOC);
  	 //rcc_periph_clock_enable (RCC_USART1);
  	 rcc_periph_clock_enable (RCC_USART3);

  	 // Button 
  	 gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
  	 gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);

  	 // LEDs
  	 gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);

  	 //initialisation de port série
  	 nvic_enable_irq (NVIC_USART3_IRQ);	//uart interrupt
  	 nvic_set_priority(NVIC_USART3_IRQ, 2);
  	 gpio_mode_setup (GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);	//GPD8 : Tx send from STM32 to ext
  	 gpio_mode_setup (GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);	//GPD9 : Rx recieve from ext to STM32
  	 gpio_set_output_options (GPIOD, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO9);
  	 gpio_set_af (GPIOD, GPIO_AF7, GPIO8);
  	 gpio_set_af (GPIOD, GPIO_AF7, GPIO9);
  	 // Setup USART3 parameters. 
  	 usart_set_baudrate (USART3, 57600);
  	 usart_set_databits (USART3, 8);
  	 usart_set_stopbits (USART3, USART_STOPBITS_1);
  	 usart_set_mode (USART3, USART_MODE_TX_RX);
  	 usart_set_parity (USART3, USART_PARITY_NONE);
  	 usart_set_flow_control (USART3, USART_FLOWCONTROL_NONE);
  	 // Enable USART3 Receive interrupt. 
  	 usart_enable_rx_interrupt (USART3);
  	 // Finally enable the USART. 
  	 usart_enable (USART3);

  	 // MOD :: Clock Initialisation
  	 // Enable Timer 2 clock
	 rcc_periph_clock_enable (RCC_TIM2);
	 // Enable Timer 2 interrupt
	 nvic_enable_irq(NVIC_TIM2_IRQ);
	 nvic_set_priority(NVIC_TIM2_IRQ, 1);
	 // Reset Timer 2 peripheral
	 timer_reset(TIM2);
	 // Timer global mode : No divider, Alignement edge,  Direction up
	 timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	 //prescaler
	 timer_set_prescaler(TIM2, 64); // 255 for 1 Hz LED ON OR OFF : 127 for 1 on ON/OFF
	 //Enable preload
	 timer_disable_preload(TIM2);
	 //Continous mode
	 timer_continuous_mode(TIM2);
	 // Period 
	 timer_set_period(TIM2, 65535); // ( (16 MHz) ?? / (65535 * 255) ~ 1 Hz for LED_ON : 0.5 for LED_ON/OFF :: use prescaler at 127 for F_LED_ON/OFF = 1 Hz )
	 // ARR preload enable
	 timer_disable_preload(TIM2);
	 // Counter enable
	 timer_enable_counter(TIM2);
	 // Enable communication interrupt
	 timer_enable_irq(TIM2, TIM_DIER_CC1IE);

		return;
	}

void usart3_isr(void){

		}

void tim2_isr (void) { // ISR : Interrupt Service routine
	ticks_sec++;
	seconds = seconds + 0.25;

		if (timer_get_flag(TIM2, TIM_SR_CC1IF)){

			commands();
			timer_clear_flag(TIM2, TIM_SR_CC1IF);
		}
	}

struct timeval {
		time_t      tv_sec;     /* seconds */
		suseconds_t tv_usec;    /* microseconds */
		};
struct timezone {
		int tz_minuteswest;     /* minutes west of Greenwich */
		int tz_dsttime;         /* type of DST correction */
		};
int get_time_sec(struct timeval *tv, struct timezone *tz){
		uint32_t tick = ticks_sec;      /* ms */ // MOD :: each 250 ms
		
		tv->tv_sec = tick / 4; // MOD :: OLD : 1000
		tv->tv_usec = (tick % 250000) * 1000;
		
		return 0;
		}

// To Mod 
// Initialisation

void autopilot_intialize(void){
	// initialize attributes

	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	system_id    = 1; // system id
	autopilot_id = 1; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	}
void autopilot_start(void){

	read_messages();

	while ( not current_messages.sysid ){
		if ( time_to_exit )
			return;
		// usleep(500000); // check at 2Hz
	}

	if ( not system_id ){
		system_id = current_messages.sysid;
	}

	if ( not autopilot_id ){
		autopilot_id = current_messages.compid;
	}

	// Wait for initial position ned
	while ( not ( current_messages.time_stamps.local_position_ned && current_messages.time_stamps.attitude)){
		if ( time_to_exit )
			return;
		//usleep(500000);
	}

	// copy initial position ned
	Mavlink_Messages local_data = current_messages;
	initial_position.x        = local_data.local_position_ned.x;
	initial_position.y        = local_data.local_position_ned.y;
	initial_position.z        = local_data.local_position_ned.z;
	initial_position.vx       = local_data.local_position_ned.vx;
	initial_position.vy       = local_data.local_position_ned.vy;
	initial_position.vz       = local_data.local_position_ned.vz;
	initial_position.yaw      = local_data.attitude.yaw;
	initial_position.yaw_rate = local_data.attitude.yawspeed;

	//autopilot_write();
	return;
	}

// READ
void read_messages(void){
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_all ){ // and !time_to_exit
		//   READ MESSAGE
		mavlink_message_t message;

		success = usart_read_message(message);

		//   HANDLE MESSAGE
		if( success ){
			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;


			// Handle Message ID
			switch (message.msgid){
				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					gpio_toggle(GPIOD, GPIO12);
					//printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;	
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));

					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					gpio_toggle(GPIOD, GPIO14);
					//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					//printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
				}

				default:
				{
					// printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				} // MAVLINK_MESSAGES
			} // end: switch msgid
		} // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
		//				this_timestamps.battery_status             &&
		//				this_timestamps.radio_status               &&
						this_timestamps.local_position_ned         
		//				this_timestamps.global_position_int        &&
		//				this_timestamps.position_target_local_ned  &&
		//				this_timestamps.position_target_global_int &&
		//				this_timestamps.highres_imu                &&
		//				this_timestamps.attitude                   &&
			// :: MOD : disabled for fast debugging
			//	this_timestamps.sys_status
				;		
	} // end: while not received all

	
	return;
	}
// Mod
void global_read_messages(void){
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_all ){ // and !time_to_exit
		//   READ MESSAGE
		mavlink_message_t message;

		success = usart_read_message(message);

		if(success){
			switch (message.msgid){
				case MAVLINK_MSG_ID_HEARTBEAT:{
					gpio_toggle(GPIOD, GPIO12);
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;	
					break;
					}

				case MAVLINK_MSG_ID_SYS_STATUS:{
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
					}

				case MAVLINK_MSG_ID_BATTERY_STATUS:{
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
					}

				case MAVLINK_MSG_ID_RADIO_STATUS:{
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
					}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:{
					gpio_toggle(GPIOD, GPIO14);
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					break;
					}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:{
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					break;
					}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:{
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					break;
					}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:{
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
					}

				case MAVLINK_MSG_ID_HIGHRES_IMU: {
					gpio_toggle(GPIOA, GPIO6);
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
					}

				case MAVLINK_MSG_ID_ATTITUDE: {
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
					}

				default: break;
				 // MAVLINK_MESSAGES
			} // end: switch msgid
		} // end: if read message

			// Check for receipt of all items
		received_all =
		//		this_timestamps.heartbeat                  &&
		//				this_timestamps.battery_status             &&
		//				this_timestamps.radio_status               &&
		//				this_timestamps.local_position_ned         
		//				this_timestamps.global_position_int        &&
		//				this_timestamps.position_target_local_ned  &&
		//				this_timestamps.position_target_global_int &&
						this_timestamps.highres_imu               // &&
		//				this_timestamps.attitude                   &&
			// :: MOD : disabled for fast debugging
			//	this_timestamps.sys_status
				;		
		} // end: while not received all
	return;
	}
//
int usart_read_message(mavlink_message_t &message){

   	msgReceived = mavlink_parse_char(MAVLINK_COMM_1, usart_recv_blocking(USART3), &message, &status);

	return msgReceived;
	}

// Write
void autopilot_write(void){
	// signal startup
	// prepare an initial setpoint, just stay put
	mavlink_set_position_target_local_ned_t set_point;
	set_point.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY & MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	set_point.coordinate_frame = MAV_FRAME_LOCAL_NED;
	set_point.vx       = 0.0;
	set_point.vy       = 0.0;
	set_point.vz       = 0.0;
	set_point.yaw_rate = 0.0;
	
	// set position target
	current_setpoint = set_point;

	autopilot_write_setpoint();

		return;
	}
void autopilot_write_setpoint(void){
	//   PACK PAYLOAD

	// pull from position target
	mavlink_set_position_target_local_ned_t set_point = current_setpoint;

	// double check some system parameters
	if ( not set_point.time_boot_ms )
		set_point.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	set_point.target_system    = system_id;
	set_point.target_component = autopilot_id;

	//   ENCODE
	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &set_point);


	//   WRITE
	autopilot_write_message(message);
		//gpio_toggle(GPIOD, GPIO15);	
	return;
	}
void autopilot_write_message(mavlink_message_t message){
	// do the write
	serial_write_message(message);

	// Done!
	return;
	}
int serial_write_message(const mavlink_message_t &message){
		volatile char buff[300];
		
		int bytesWritten;
		
		unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buff, &message);

		usart_enable_tx_interrupt(USART3);

		for (int i = 0; i < sizeof(buff); i++) {
                usart_send_blocking(USART3, buff[i]);
                gpio_toggle(GPIOD, GPIO15);
		}
		
		len = 0;

		usart_disable_tx_interrupt(USART3);

		bytesWritten = len;

		return bytesWritten;
		}

void autopilot_update_setpoint(mavlink_set_position_target_local_ned_t setpoint){
	// update_setpoint
	current_setpoint = setpoint;
	}
void commands(void){
	//
		operation(5);
	}

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

void operation (float timer){
	read_messages_helper();

	global_read_messages();
	autopilot_write_helper();

	mavlink_set_position_target_local_ned_t set_point;
	mavlink_set_position_target_local_ned_t ip = initial_position;

	switch(Program_counter){
			case 0 : 
				enable_offboard_control();
				//enable_offboard_control();	
				gpio_toggle(GPIOD, GPIO13); 
				break;
			case 1 :
					set__( 1 , 0, - 2.5, set_point); break;
			case 2 :
					set__( -2 , 0, - 2.5, set_point); break; 
			case 3 :
					set__( -2 , -2.5, - 2.5, set_point); break;
			case 4 :
					set__( -2 , 2.5, - 2.5, set_point); break; 
			case 5 :
					set__( -2 , 2.5, - 1000, set_point); break; 
			default : break;
		}
		if (seconds == timer){
			gpio_toggle(GPIOA, GPIO6);
			Program_counter++;
			seconds = 0;
		}
		if (Program_counter == 0 || Program_counter == 6) { Program_counter = 1;}
	}

uint64_t get_time_usec(void){
	static struct timeval _time_stamp;
	get_time_sec(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
	}

// Offboard Control

void enable_offboard_control(void){
	// Should only send this command once
	if ( control_status == false ){
		//   TOGGLE OFF-BOARD MODE

		// Sends the command to go off-board
		int success = toggle_offboard_control( true );
		// Check the command was written
		if ( success )
			control_status = true;
	
	} // end: if not offboard_status
	}
void disable_offboard_control(void){
	// Should only send this command once
	if ( control_status == true ){
		
		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_offboard_control( false );

		// Check the command was written
		if ( success )
			control_status = false;
		
	} // end: if offboard_status
	}
int toggle_offboard_control( bool flag ){
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	//com.param1           = (int32_t) flag; // flag >0.5 => start, <0.5 => stop
	com.param1 			 = 1;

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = serial_write_message(message);

	// Done!
	return len;
	}

// Control
void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &set_position){
	set_position.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;
	set_position.coordinate_frame = MAV_FRAME_LOCAL_NED;

	set_position.x   = x; set_position.y   = y; set_position.z   = z;
	
	}
void set__(float x, float y, float z, mavlink_set_position_target_local_ned_t &final_set_point){
				set_position( x , y  , z, final_set_point);
				autopilot_update_setpoint(final_set_point);
				autopilot_write_setpoint();
				gpio_toggle(GPIOD, GPIO13);
					}


