
/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
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

/**
 * @file interface.h
 *
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Modifications made by :
 * Mohamed Hage Hassan, <mohamed.hagehassan@yahoo.com>
 *
 */

#ifndef _INTERFACE_H_
#define _INTERFACE_H_

#include <serial_port.h>

                                                // bit number  876543210987654321
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111


//   Data Structures
struct Time_Stamps{
	Time_Stamps(){
		reset_timestamps();
	}

	uint64_t heartbeat;
	uint64_t sys_status;
	uint64_t battery_status;
	uint64_t radio_status;
	uint64_t local_position_ned;
	uint64_t global_position_int;
	uint64_t position_target_local_ned;
	uint64_t position_target_global_int;
	uint64_t highres_imu;
	uint64_t attitude;
	uint64_t command_ack;

	void reset_timestamps(){
		heartbeat = 0;
		sys_status = 0;
		battery_status = 0;
		radio_status = 0;
		local_position_ned = 0;
		global_position_int = 0;
		position_target_local_ned = 0;
		position_target_global_int = 0;
		highres_imu = 0;
		attitude = 0;
		command_ack = 0;
	}
	};
struct Mavlink_Messages {

	int sysid;
	int compid;

	// Heartbeat
	mavlink_heartbeat_t heartbeat;

	// System Status
	mavlink_sys_status_t sys_status;

	// Battery Status
	mavlink_battery_status_t battery_status;

	// Radio Status
	mavlink_radio_status_t radio_status;

	// Local Position
	mavlink_local_position_ned_t local_position_ned;

	// Global Position
	mavlink_global_position_int_t global_position_int;

	// Local Position Target
	mavlink_position_target_local_ned_t position_target_local_ned;

	// Global Position Target
	mavlink_position_target_global_int_t position_target_global_int;

	// HiRes IMU
	mavlink_highres_imu_t highres_imu;

	// Attitude
	mavlink_attitude_t attitude;

	// System Parameters?

	// Command acknowledgement
	mavlink_command_ack_t command_ack;

	// Time Stamps
	Time_Stamps time_stamps;

	void reset_timestamps(){
		time_stamps.reset_timestamps();
	}
	};

// Initialisation
void autopilot_intialize(void);
void autopilot_start(void);

//	READ 
//void global_read_messages(void);
void read_messages(void);

// Write
void autopilot_write(void);
void autopilot_write_message(mavlink_message_t message);
void autopilot_write_setpoint(void);
void autopilot_update_setpoint(mavlink_set_position_target_local_ned_t setpoint);

// Offboard Control

void disable_offboard_control(void);
void enable_offboard_control(void);
int toggle_offboard_control( bool flag );

// Arm/Disarm Control

void autopilot_arm (void);
void autopilot_disarm(void);
int toggle_arm_disarm(bool flag);

// MAVLink messages acknowledgement
int check_offboard_control(void);
int check_arm_disarm(void);
int check_message (uint16_t COMMAND_ID);

// Control

// Mathematical approximations
#include <mfunctions.h>

void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp);
void set__(float x, float y, float z, mavlink_set_position_target_local_ned_t &set_point);
void set_velocity(float vx, float vy, float va, mavlink_set_position_target_local_ned_t &set_point);
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp);
void position_and_speed_set(float x, float y, float z ,float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &final_set_point);


void set_circle (float R, float theta, float z, mavlink_set_position_target_local_ned_t &set_point);

uint64_t get_time_usec(void);

#endif
