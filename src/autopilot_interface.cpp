/*
 * autopilot_interface.hpp
 *
 * Author:	Yevhenii Kovryzhenko, Department of Aerospace Engineering, Auburn University.
 * Contact: yzk0058@auburn.edu
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL I
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Last Edit:  11/11/2022 (MM/DD/YYYY)
 *
 * Functions for sending and receiving commands to an autopilot via MAVlink
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <iostream>
#include <string>
#include <sstream>

#include <math.h>
#include "autopilot_interface.hpp"
#include "thread_defs.hpp"
#include "tools.hpp"
#include <common/mavlink.h>
#include <development/development.h>
#include "nonBlocking.hpp"

// terminal emulator control sequences
#define WRAP_DISABLE	"\033[?7l"
#define WRAP_ENABLE		"\033[?7h"

//other defines
#define PRINTF_DATA_WIDTH 11
#define PRINT_HEADER(in) __print_header(#in, PRINTF_DATA_WIDTH)
#define PRINT_DATA(in) __print_data_float(in, PRINTF_DATA_WIDTH)

//#define DEBUG


void __copy_data(mavlink_vision_position_estimate_t& buff_out, mocap_data_t& buff_in)
{
	buff_out.usec = buff_in.time_us;
	buff_out.x = buff_in.x;
	buff_out.y = buff_in.y;
	buff_out.z = buff_in.z;
	buff_out.roll = buff_in.roll;
	buff_out.pitch = buff_in.pitch;
	buff_out.yaw = buff_in.yaw;
	return;
}

bool __is_vision_data_same(mavlink_vision_position_estimate_t& in1, mavlink_vision_position_estimate_t& in2)
{
	if (in1.usec == in2.usec) return true;
	if (in1.x == in2.x) return true;
	if (in1.y == in2.y) return true;
	if (in1.z == in2.z) return true;
	if (in1.yaw == in2.yaw) return true;
	if (in1.roll == in2.roll) return true;
	if (in1.pitch == in2.pitch) return true;
	return false;
}

bool __update_from_mocap(mavlink_vision_position_estimate_t& buff_out, int mocap_ID, mocap_node_t& node, bool& _time_to_exit)
{
	bool data_is_good = false;

	//get the data from mocap
	mocap_data_t tmp;
	node.get_data(tmp, mocap_ID);
	if (tmp.trackingValid) //check if tracking was done properly
	{
		mavlink_vision_position_estimate_t tmp_2;
		__copy_data(tmp_2, tmp);
		if (!__is_vision_data_same(tmp_2, buff_out))
		{
			buff_out.usec = tmp_2.usec;
			buff_out.x = tmp_2.x;
			buff_out.y = tmp_2.y;
			buff_out.z = tmp_2.z;
			buff_out.roll = tmp_2.roll;
			buff_out.pitch = tmp_2.pitch;
			buff_out.yaw = tmp_2.yaw;
			data_is_good = true;
		}
#ifdef DEBUG
		else
		{
			printf("WARNING in __update_from_mocap: no new update\n");
		}
#endif // DEBUG		
	}
	return data_is_good;
}


// ----------------------------------------------------------------------------------
//   Setpoint Helper Functions
// ----------------------------------------------------------------------------------

// choose one of the next three

/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.x   = x;
	sp.y   = y;
	sp.z   = z;

#ifdef DEBUG
	printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);
#endif // DEBUG
}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;

#ifdef DEBUG
	printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);
#endif // DEBUG
}

/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
void set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{

	// NOT IMPLEMENTED

	fprintf(stderr,"set_acceleration doesn't work yet \n");
	throw 1;


	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.afx  = ax;
	sp.afy  = ay;
	sp.afz  = az;
}

// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;

	sp.yaw  = yaw;

#ifdef DEBUG
	printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);
#endif // DEBUG
}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;

#ifdef DEBUG
	printf("POSITION SETPOINT YAW RATE = %.4f \n", sp.yaw_rate);
#endif // DEBUG
}


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::Autopilot_Interface(Generic_Port* target_port_, settings_t& settings_)
{
	init(target_port_, settings_);
}
Autopilot_Interface::Autopilot_Interface(Generic_Port* target_port_, Generic_Port* relay_port_, settings_t& settings_)
{
	init(target_port_, relay_port_, settings_);
}
Autopilot_Interface::Autopilot_Interface(Generic_Port* target_port_, Generic_Port* relay_port_, Generic_Port* relay_port2_, settings_t& settings_)
{
	init(target_port_, relay_port_, relay_port2_, settings_);
}

void Autopilot_Interface::init(Generic_Port* target_port_, Generic_Port* relay_port_, Generic_Port* relay_port2_, settings_t& settings_)
{
	// initialize attributes
	write_count = 0;
	relay_write_count = 0;

	reading_status = 0;     // whether the read thread is running
	writing_status = 0;     // whether the write thread is running
	relay_status = 0;		// whether the relay thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit = false;  // flag to signal thread exit

	read_tid.init(READ_THREAD_PRI, READ_THREAD_TYPE);
	write_tid.init(WRITE_THREAD_PRI, WRITE_THREAD_TYPE);
	vision_position_estimate_write_tid.init(VPE_THREAD_PRI, VPE_THREAD_TYPE);
	relay_tid.init(RELAY_THREAD_PRI, RELAY_THREAD_TYPE);
	nonblock_io_tid.init(NONBLOCK_IO_THREAD_PRI, NONBLOCK_IO_THREAD_TYPE);
	printf_tid.init(PRINTF_THREAD_PRI, PRINTF_THREAD_TYPE);
	sys_tid.init(SYS_THREAD_PRI, SYS_THREAD_TYPE);

	system_id = -1; // system id
	autopilot_id = -1; // autopilot component id
	companion_id = 0; // companion computer component id.

	current_RX_messages.sysid = system_id;
	current_RX_messages.compid = autopilot_id;	

	settings = settings_;

	target_port = target_port_;
	relay_port = relay_port_;
	relay_port2 = relay_port2_;
	return;
}

void Autopilot_Interface::init(Generic_Port* target_port_, settings_t& settings_)
{
	settings_.enable_relay2 = false;
	settings_.enable_relay = false;
	init(target_port_, relay_port, relay_port2, settings_);
	return;
}

void Autopilot_Interface::init(Generic_Port* target_port_,  Generic_Port* relay_port_, settings_t& settings_)
{
	settings_.enable_relay2 = false;
	init(target_port_, relay_port_, relay_port2, settings_);
	return;
}

Autopilot_Interface::~Autopilot_Interface(){}


// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void Autopilot_Interface::update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	std::lock_guard<std::mutex> lock(current_setpoint.mutex);
	current_setpoint.data = setpoint;	
}
void Autopilot_Interface::get_setpoint(mavlink_set_position_target_local_ned_t& setpoint)
{
	std::lock_guard<std::mutex> lock(current_setpoint.mutex);
	setpoint = current_setpoint.data;
}

// ------------------------------------------------------------------------------
//   Update Non-blocking interface
// ------------------------------------------------------------------------------
void Autopilot_Interface::update_nonblock_io(void)
{
	non_blocking_client_update(this);   
}

//#define DEBUG
// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void Autopilot_Interface::read_messages(void)
{
	bool success;               // receive success flag	

	// ----------------------------------------------------------------------
	//   READ MESSAGE
	// ----------------------------------------------------------------------
	mavlink_message_t message;
	success = target_port->read_message(message, MAVLINK_COMM_0);

	// ----------------------------------------------------------------------
	//   HANDLE MESSAGE
	// ----------------------------------------------------------------------
	if (success)
	{
#ifdef DEBUG
printf("DEBUG: received good message from target!\n");
#endif
		if (settings.enable_relay) relay_message(message); //relay the message

		// Store message sysid and compid.
		// Note this doesn't handle multiple message sources.
		if (current_RX_messages.sysid == -1 || current_RX_messages.compid == -1) 
		{
			switch (static_cast<MAV_COMPONENT>(message.compid))
			{
			case MAV_COMP_ID_TELEMETRY_RADIO:
			{
				//static bool already_printed__ = false;
				//if (!already_printed__) 
				//{
					printf("Got message from telemetry module, waiting for autopilot...\n");
				//	already_printed__ = true;
				//}
				return;
			}
			case MAV_COMP_ID_AUTOPILOT1:
			{
				printf("Got message from autopilot\n");
				current_RX_messages.sysid = message.sysid;
				current_RX_messages.compid = message.compid;
				break;
			}
			default:
				printf("Got message from someone else with comp_id %i\n", message.compid);
				current_RX_messages.sysid = message.sysid;
				current_RX_messages.compid = message.compid;
				break;
			}
		}
		else if (current_RX_messages.sysid != message.sysid) return;

		// Handle Message ID
		switch (message.msgid)
		{

		case MAVLINK_MSG_ID_HEARTBEAT:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_HEARTBEAT\n");
#endif // DEBUG
			std::lock_guard<std::mutex> lock(current_RX_messages.heartbeat.mutex);
			mavlink_msg_heartbeat_decode(&message, &(current_RX_messages.heartbeat.data));
			time_stamps_old.heartbeat = current_RX_messages.time_stamps.heartbeat;
			current_RX_messages.time_stamps.heartbeat = get_time_usec();
			break;
		}

		case MAVLINK_MSG_ID_SYS_STATUS:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_SYS_STATUS\n");
#endif // DEBUG
			std::lock_guard<std::mutex> lock(current_RX_messages.sys_status.mutex);
			mavlink_msg_sys_status_decode(&message, &(current_RX_messages.sys_status.data));
			time_stamps_old.sys_status = current_RX_messages.time_stamps.sys_status;
			current_RX_messages.time_stamps.sys_status = get_time_usec();
			break;
		}

		case MAVLINK_MSG_ID_BATTERY_STATUS:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
#endif // DEBUG
			std::lock_guard<std::mutex> lock(current_RX_messages.battery_status.mutex);
			mavlink_msg_battery_status_decode(&message, &(current_RX_messages.battery_status.data));
			time_stamps_old.battery_status = current_RX_messages.time_stamps.battery_status;
			current_RX_messages.time_stamps.battery_status = get_time_usec();
			break;
		}

		case MAVLINK_MSG_ID_RADIO_STATUS:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
#endif // DEBUG
			std::lock_guard<std::mutex> lock(current_RX_messages.radio_status.mutex);
			mavlink_msg_radio_status_decode(&message, &(current_RX_messages.radio_status.data));
			time_stamps_old.radio_status = current_RX_messages.time_stamps.radio_status;
			current_RX_messages.time_stamps.radio_status = get_time_usec();
			break;
		}

		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
#endif // DEBUG
			std::lock_guard<std::mutex> lock(current_RX_messages.local_position_ned.mutex);
			mavlink_msg_local_position_ned_decode(&message, &(current_RX_messages.local_position_ned.data));
			time_stamps_old.local_position_ned = current_RX_messages.time_stamps.local_position_ned;
			current_RX_messages.time_stamps.local_position_ned = get_time_usec();
			break;
		}

		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
#endif // DEBUG
			std::lock_guard<std::mutex> lock(current_RX_messages.global_position_int.mutex);
			mavlink_msg_global_position_int_decode(&message, &(current_RX_messages.global_position_int.data));
			time_stamps_old.global_position_int = current_RX_messages.time_stamps.global_position_int;
			current_RX_messages.time_stamps.global_position_int = get_time_usec();
			break;
		}

		case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
#endif // DEBUG
			std::lock_guard<std::mutex> lock(current_RX_messages.position_target_local_ned.mutex);
			mavlink_msg_position_target_local_ned_decode(&message, &(current_RX_messages.position_target_local_ned.data));
			time_stamps_old.position_target_local_ned = current_RX_messages.time_stamps.position_target_local_ned;
			current_RX_messages.time_stamps.position_target_local_ned = get_time_usec();
			break;
		}

		case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
#endif // DEBUG
			std::lock_guard<std::mutex> lock(current_RX_messages.position_target_global_int.mutex);
			mavlink_msg_position_target_global_int_decode(&message, &(current_RX_messages.position_target_global_int.data));
			time_stamps_old.position_target_global_int = current_RX_messages.time_stamps.position_target_global_int;
			current_RX_messages.time_stamps.position_target_global_int = get_time_usec();
			break;
		}

		case MAVLINK_MSG_ID_HIGHRES_IMU:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
#endif // DEBUG
			std::lock_guard<std::mutex> lock(current_RX_messages.highres_imu.mutex);
			mavlink_msg_highres_imu_decode(&message, &(current_RX_messages.highres_imu.data));
			time_stamps_old.highres_imu = current_RX_messages.time_stamps.highres_imu;
			current_RX_messages.time_stamps.highres_imu = get_time_usec();
			break;
		}

		case MAVLINK_MSG_ID_ATTITUDE:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_ATTITUDE\n");
#endif // DEBUG
			std::lock_guard<std::mutex> lock(current_RX_messages.attitude.mutex);
			mavlink_msg_attitude_decode(&message, &(current_RX_messages.attitude.data));
			time_stamps_old.attitude = current_RX_messages.time_stamps.attitude;
			current_RX_messages.time_stamps.attitude = get_time_usec();
			break;
		}

		case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE\n");
#endif // DEBUG
			std::lock_guard<std::mutex> lock(current_RX_messages.vision_position_estimate.mutex);
			mavlink_msg_vision_position_estimate_decode(&message, &(current_RX_messages.vision_position_estimate.data));
			time_stamps_old.vision_position_estimate = current_RX_messages.time_stamps.vision_position_estimate;
			current_RX_messages.time_stamps.vision_position_estimate = get_time_usec();
			break;
		}

		case MAVLINK_MSG_ID_ODOMETRY:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_ODOMETRY\n");
#endif // DEBUG
			std::lock_guard<std::mutex> lock(current_RX_messages.odometry.mutex);
			mavlink_msg_odometry_decode(&message, &(current_RX_messages.odometry.data));
			time_stamps_old.odometry = current_RX_messages.time_stamps.odometry;
			current_RX_messages.time_stamps.odometry = get_time_usec();
			break;
		}
		case MAVLINK_MSG_ID_ALTITUDE:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_ALTITUDE\n");
#endif // DEBUG
			std::lock_guard<std::mutex> lock(current_RX_messages.altitude.mutex);
			mavlink_msg_altitude_decode(&message, &(current_RX_messages.altitude.data));
			time_stamps_old.altitude = current_RX_messages.time_stamps.altitude;
			current_RX_messages.time_stamps.altitude = get_time_usec();

			break;
		}

		case MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_TIMESYNC:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_TIMESYNC\n");
#endif // DEBUG
			
			break;
		}

		case MAVLINK_MSG_ID_ESTIMATOR_STATUS:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_ESTIMATOR_STATUS\n");
#endif // DEBUG
			std::lock_guard<std::mutex> lock(current_RX_messages.estimator_status.mutex);
			mavlink_msg_estimator_status_decode(&message, &(current_RX_messages.estimator_status.data));
			time_stamps_old.estimator_status = current_RX_messages.time_stamps.estimator_status;
			current_RX_messages.time_stamps.estimator_status = get_time_usec();

			break;
		}

		case MAVLINK_MSG_ID_PING:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_PING\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_GPS_RAW_INT:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_GPS_RAW_INT\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_ATTITUDE_QUATERNION\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_PARAM_REQUEST_LIST\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_PARAM_EXT_SET:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_PARAM_EXT_SET\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_PARAM_EXT_VALUE:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_PARAM_EXT_VALUE\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_COMMAND_INT:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_COMMAND_INT\n");
#endif // DEBUG
			std::lock_guard<std::mutex> lock(current_RX_messages.command_int.mutex);
			mavlink_msg_command_int_decode(&message, &(current_RX_messages.command_int.data));
			time_stamps_old.command_int = current_RX_messages.time_stamps.command_int;
			current_RX_messages.time_stamps.command_int = get_time_usec();
			break;
		}

		case MAVLINK_MSG_ID_COMMAND_LONG:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_COMMAND_LONG\n");
#endif // DEBUG
			std::lock_guard<std::mutex> lock(current_RX_messages.command_long.mutex);
			mavlink_msg_command_long_decode(&message, &(current_RX_messages.command_long.data));
			time_stamps_old.command_long = current_RX_messages.time_stamps.command_long;
			current_RX_messages.time_stamps.command_long = get_time_usec();
			break;
		}

		case MAVLINK_MSG_ID_COMMAND_ACK:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_COMMAND_ACK\n");
#endif // DEBUG
			std::lock_guard<std::mutex> lock(current_RX_messages.command_ack.mutex);
			mavlink_msg_command_ack_decode(&message, &(current_RX_messages.command_ack.data));
			time_stamps_old.command_ack = current_RX_messages.time_stamps.command_ack;
			current_RX_messages.time_stamps.command_ack = get_time_usec();
			break;
		}

		case MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS:
		{
			#ifdef DEBUG
			printf("MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS\n");
			#endif
			break;
		}

		case MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY:
		{
			#ifdef DEBUG
			printf("MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY\n");
			#endif
			break;
		}

		default:
		{
#ifdef DEBUG
			printf("Warning, did not handle message id %i\n", message.msgid);
#endif // DEBUG
			break;
		}


		} // end: switch msgid

	} // end: if read message

	return;
}
//#define DEBUG
// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void Autopilot_Interface::relay_read(void)
{
	bool success = false, success2 = false;               // receive success flag
	// ----------------------------------------------------------------------
	//   READ MESSAGE
	// ----------------------------------------------------------------------

	mavlink_message_t message, message2;
	success = relay_port->read_message(message, MAVLINK_COMM_1);
	if (settings.enable_relay2)
	{
		success2 = relay_port2->read_message(message2, MAVLINK_COMM_1);

		if (success2)
		{
			//printf("Received msg to relay2, back to target..\n");
			write_message(message2);
		}
	}
	

	// ----------------------------------------------------------------------
	//   HANDLE MESSAGE
	// ----------------------------------------------------------------------
	if (success)
	{
		write_message(message);

#ifdef DEBUG
		printf("Received msg to relay:\t");
		// Store message sysid and compid.
		// Note this doesn't handle multiple message sources.
		current_RX_messages.sysid = message.sysid;
		current_RX_messages.compid = message.compid;

		// Handle Message ID
		switch (message.msgid)
		{

		case MAVLINK_MSG_ID_HEARTBEAT:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_HEARTBEAT\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_SYS_STATUS:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_SYS_STATUS\n");
#endif // DEBUG
			break;
		}

case MAVLINK_MSG_ID_MANUAL_CONTROL:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_MANUAL_CONTROL\n");
			mavlink_manual_control_t manual_control{};
			mavlink_msg_manual_control_decode(&message, &manual_control);
			printf("MSG: [%i, %i, %i, %i]\n",manual_control.x, manual_control.y, manual_control.z, manual_control.r);
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_BATTERY_STATUS:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_RADIO_STATUS:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_HIGHRES_IMU:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_ATTITUDE:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_ATTITUDE\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_ODOMETRY:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_ODOMETRY\n");
#endif // DEBUG
			break;
		}
		case MAVLINK_MSG_ID_ALTITUDE:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_ALTITUDE\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_TIMESYNC:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_TIMESYNC\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_ESTIMATOR_STATUS:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_ESTIMATOR_STATUS\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_PING:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_PING\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_GPS_RAW_INT:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_GPS_RAW_INT\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_ATTITUDE_QUATERNION\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_PARAM_REQUEST_LIST\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_PARAM_EXT_SET:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_PARAM_EXT_SET\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_PARAM_EXT_VALUE:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_PARAM_EXT_VALUE\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_COMMAND_INT:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_COMMAND_INT\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_COMMAND_LONG:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_COMMAND_LONG\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_COMMAND_ACK:
		{
#ifdef DEBUG
			printf("MAVLINK_MSG_ID_COMMAND_ACK\n");
#endif // DEBUG
			break;
		}

		case MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS:
		{
			#ifdef DEBUG
			printf("MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS\n");
			#endif
			break;
		}

		case MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY:
		{
			#ifdef DEBUG
			printf("MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY\n");
			#endif
			break;
		}

		default:
		{
#ifdef DEBUG
			printf("WARNING: did not handle message id %i\n", message.msgid);
#endif // DEBUG
			break;
		}
		}
#endif // DEBUG
	}
	return;
}

//#define DEBUG
// ------------------------------------------------------------------------------
//   Relay Write
// ------------------------------------------------------------------------------
int Autopilot_Interface::relay_message(mavlink_message_t& message)
{
#ifdef DEBUG
printf("DEBUG: trying to relay message...\n");
#endif
	// do the write
	int len = relay_port->write_message(message);
	if (settings.enable_relay2) relay_port2->write_message(message);

	// book keep
	relay_write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int Autopilot_Interface::write_message(mavlink_message_t message)
{
	// do the write
	int len = target_port->write_message(message);
	

	// book keep
	write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write Vision Position Estimate Message
// ------------------------------------------------------------------------------
void Autopilot_Interface::write_vision_position_estimate()
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------

	// pull from position target
	mavlink_vision_position_estimate_t vpe;
	{
		std::lock_guard<std::mutex> lock(current_vision_position_estimate.mutex);
		vpe = current_vision_position_estimate.data;		
	}

	// double check some system parameters
	if (not vpe.usec)
		vpe.usec = get_time_usec();


	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_vision_position_estimate_encode(system_id, companion_id, &message, &vpe);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if (len <= 0)
		fprintf(stderr, "WARNING: could not send VISION_POSITION_ESTIMATE \n");

	return;
}

// ------------------------------------------------------------------------------
//  SYS Write Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::sys_write(void)
{
	int len = 0;
	mavlink_message_t msg;

	/*Send Heartbeat */
	mavlink_msg_heartbeat_pack(254, 190, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
	len = write_message(msg);
	// check the write
	if (len <= 0) fprintf(stderr, "WARNING: could not send HEARTBEAT\n");
	
	/* Send Status */
	mavlink_msg_sys_status_pack(254, 190, &msg,\
		0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	len = write_message(msg);
	// check the write
	if (len <= 0) fprintf(stderr, "WARNING: could not send sys_status\n");

	return;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void Autopilot_Interface::write_setpoint(void)
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------
	//if (current_setpoint.time_ms_old == current_setpoint.data.time_boot_ms) return;
	//if (!current_setpoint.new_data_available) return;
	//current_setpoint.new_data_available = false;

	// pull from position target
	mavlink_set_position_target_local_ned_t sp;
	{
		std::lock_guard<std::mutex> lock(current_setpoint.mutex);
		current_setpoint.time_ms_old = current_setpoint.data.time_boot_ms;
		current_setpoint.data.time_boot_ms = get_time_usec() / 1000;
		sp = current_setpoint.data;		
	}

	// double check some system parameters
	//if ( not sp.time_boot_ms )
	//	sp.time_boot_ms = (uint32_t)(get_time_usec() / 1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;

	//printf("\nWriting new setpoint\n");

	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------
	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);

	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");

	return;
}

// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
char Autopilot_Interface::enable_offboard_control()
{
	// Should only send this command once
	if (control_status == false)
	{
#ifdef DEBUG
		printf("ENABLE OFFBOARD MODE\n");
#endif // DEBUG	

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board

		// Check the command was written
		if (toggle_offboard_control(true) > 0)
			control_status = true;
		else
		{
			fprintf(stderr, "Error: off-board mode not set, could not write message\n");
			return -1;
		}
#ifdef DEBUG
		printf("\n");
#endif // DEBUG
		return 1;

	} // end: if not offboard_status
	else return 0;
}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
char Autopilot_Interface::disable_offboard_control()
{

	// Should only send this command once
	if (control_status == true)
	{
#ifdef DEBUG
		printf("DISABLE OFFBOARD MODE\n");
#endif // DEBUG	

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		// Check the command was written
		if (toggle_offboard_control(false) > 0)
			control_status = false;
		else
		{
			fprintf(stderr, "Error: off-board mode not set, could not write message\n");
			return -1;
		}
#ifdef DEBUG
		printf("\n");
#endif // DEBUG
		return 1;

	} // end: if offboard_status
	else return 0;
}

// ------------------------------------------------------------------------------
//   Arm
// ------------------------------------------------------------------------------
char Autopilot_Interface::arm_disarm( bool flag )
{
#ifdef DEBUG
	if (flag)
	{
		printf("ARM ROTORS\n");
	}
	else
	{
		printf("DISARM ROTORS\n");
	}
#endif // DEBUG	

	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_COMPONENT_ARM_DISARM;
	com.confirmation     = 0;
	com.param1           = (float) flag;
	com.param2           = 21196;
	
	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = write_message(message);
	if (len < 1)
	{
		printf("ERROR in arm_disarm: failed to write message\n");
		return -1;
	}
	
	
	for (int i = 0; i < 50; i++)
	{
		usleep(2E5); //wait

#ifdef DEBUG
		printf("arm_disarm: Trying to check if armed for %i-th time\n", i);
#endif // DEBUG


		//check if confimation has been received
		{
			std::lock_guard<std::mutex> lock(current_RX_messages.command_ack.mutex);
			if (current_RX_messages.command_ack.data.command == MAV_CMD_COMPONENT_ARM_DISARM &&\
				current_RX_messages.command_ack.data.target_system == system_id)
			{
				if (current_RX_messages.command_ack.data.progress == 0)//success
				{
					current_RX_messages.command_ack.data.progress = 10;
					if (flag)printf("WARNING: ARMED!\n");
					else printf("WARNING: DISARMED!\n");
					return 1;
				} 
				else
				{
					if (flag)printf("WARNING: ARMING DENINED\n");
					else printf("WARNING: DISARMING DENINED\n");
					return -1;
				}
			}
		}
		
		com.confirmation++; //increment confirmation id

		// Encode
		mavlink_message_t message;
		mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

		// Send the message
		len = write_message(message);
		if (len < 1)
		{
			printf("ERROR in arm_disarm: failed to write message\n");
			return -1;
		}
	}
	// Done!
	printf("ERROR in arm_disarmed: timeout\n");
	return -1;
}

// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
char Autopilot_Interface::toggle_offboard_control( bool flag )
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command = MAV_CMD_DO_SET_MODE;// MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = 0;
	//com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop
	com.param1 = 1; //use custom PX4 mode
	if (flag)
	{
		com.param2 = 6; //switch to offboard;
	}
	else
	{
		com.param2 = 3; //switch to position hold;
	}

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = write_message(message);
	if (len < 1)
	{
		printf("ERROR in toggle_offboard_control: failed to write message\n");
		return -1;
	}

	for (int i = 0; i < 50; i++)
	{
		usleep(2E5); //wait

#ifdef DEBUG
		printf("arm_disarm: Trying to check if offboard has been switched for %i-th time\n", i);
#endif // DEBUG


		//check if confimation has been received
		{
			std::lock_guard<std::mutex> lock(current_RX_messages.command_ack.mutex);
			if (current_RX_messages.command_ack.data.command == MAV_CMD_DO_SET_MODE && \
				current_RX_messages.command_ack.data.target_system == system_id)
			{
				if (current_RX_messages.command_ack.data.progress == 0)//success
				{
					current_RX_messages.command_ack.data.progress = 10;
					if (flag)printf("WARNING: OFFBOARD MODE ON!\n");
					else printf("WARNING: OFFBOARD MODE OFF!\n");
					return 1;
				}
				else
				{
					printf("WARNING: OFFBOARD DENINED\n");
					return -1;
				}
			}
		}

		com.confirmation++; //increment confirmation id

		// Encode
		mavlink_message_t message;
		mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

		// Send the message
		len = write_message(message);
		if (len < 1)
		{
			printf("ERROR in toggle_offboard_control: failed to write message\n");
			return -1;
		}
	}


	// Done!
	printf("ERROR in toggle_offboard_control: timeout\n");
	return -1;
}

//#define DEBUG
// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void Autopilot_Interface::start(void)
{
	int result;
#ifdef DEBUG
	printf("trying to start autopilot interface...\n");
#endif // DEBUG


	if (settings.enable_target)
	{
		// --------------------------------------------------------------------------
		//   CHECK PORT
		// --------------------------------------------------------------------------

#ifdef DEBUG
		printf("Checking if target_port is running...\n");
#endif // DEBUG
		if (!target_port->is_running()) // PORT_OPEN
		{
			fprintf(stderr, "ERROR: target_port not open\n");
			throw 1;
		}
#ifdef DEBUG
		printf("Good.\n");
#endif // DEBUG

		// Start SYS thread:
#ifdef DEBUG
		printf("Starting SYS thread...\n");
#endif // DEBUG
		result = sys_tid.start(&start_autopilot_interface_sys_thread, this);
		if (result) throw result;
#ifdef DEBUG
		printf("SYS thread started!\n");
#endif // DEBUG		
	}


	if (settings.enable_relay2)
	{
		// --------------------------------------------------------------------------
		//   CHECK PORT
		// --------------------------------------------------------------------------
		if (!settings.enable_relay)
		{
			fprintf(stderr, "ERROR: relay2 is enabled while there is no relay\n");
			throw 1;
		}

#ifdef DEBUG
		printf("Checking if relay_port2 is running...\n");
#endif // DEBUG
		if (!relay_port2->is_running()) // PORT_OPEN
		{
			fprintf(stderr, "ERROR: relay_port2 not open\n");
			throw 1;
		}
#ifdef DEBUG
		printf("Good.\n");
#endif // DEBUG
	}

	if (settings.enable_relay)
	{
		// --------------------------------------------------------------------------
		//   CHECK PORT
		// --------------------------------------------------------------------------
		if (!settings.enable_target)
		{
			fprintf(stderr, "ERROR: relay is enabled while there is no target\n");
			throw 1;
		}

#ifdef DEBUG
		printf("Checking if relay_port is running...\n");
#endif // DEBUG
		if (!relay_port->is_running()) // PORT_OPEN
		{
			fprintf(stderr, "ERROR: relay_port not open\n");
			throw 1;
		}
#ifdef DEBUG
		printf("Good.\n");
#endif // DEBUG

		// Start Relay thread:
#ifdef DEBUG
		printf("Starting Relay thread...\n");
#endif // DEBUG
		result = sys_tid.start(&start_autopilot_interface_relay_thread, this);
		if (result) throw result;
#ifdef DEBUG
		printf("Relay thread started!\n");
#endif // DEBUG	
	}


#ifdef DEBUG
	printf("Check if mocap is enabled\n");
#endif // DEBUG
	if (settings.enable_mocap)
	{
#ifdef DEBUG
		printf("mocap is enabled, starting...\n");
#endif // DEBUG
		if (settings.mocap_YUP2NED) mocap.togle_YUP2NED(true);
		else if (settings.mocap_ZUP2NED) mocap.togle_ZUP2NED(true);
		// --------------------------------------------------------------------------
		//   MOCAP READ THREAD
		// --------------------------------------------------------------------------
#ifdef DEBUG
		printf("START MOCAP READ THREAD \n");
#endif // DEBUG
		result = mocap.start(settings.mocap_ip);
		if (result) throw result;

		// now we're reading messages from mocap
#ifdef DEBUG
		printf("\n");
#endif // DEBUG
	}
#ifdef DEBUG
	printf("Check if telemetry is enabled\n");
#endif // DEBUG
	if (settings.enable_telemetry)
	{
#ifdef DEBUG
		printf("Telemetry is enabled, starting...\n");
#endif // DEBUG
		// --------------------------------------------------------------------------
		//   READ THREAD
		// --------------------------------------------------------------------------
#ifdef DEBUG
		printf("START READ THREAD \n");
#endif // DEBUG
		result = read_tid.start(&start_autopilot_interface_read_thread, this);
		if (result) throw result;

		// now we're reading messages
#ifdef DEBUG
		printf("\n");
#endif // DEBUG
	}

	// --------------------------------------------------------------------------
	//   WRITE VISION POSITION ESTIMATE THREAD
	// --------------------------------------------------------------------------
	if (settings.enable_vpe)
	{
#ifdef DEBUG
		printf("START VISION POSITION ESTIMATE WRITE THREAD \n");
#endif // DEBUG

		result = vision_position_estimate_write_tid.start(&start_autopilot_interface_write_vision_position_estimate_thread, this);
		if (result) throw result;

#ifdef DEBUG
		printf("\n");
#endif // DEBUG
	}

#ifdef DEBUG
	printf("Check if telemetry is enabled\n");
#endif // DEBUG
	if (settings.enable_telemetry)
	{
		// --------------------------------------------------------------------------
		//   CHECK FOR MESSAGES
		// --------------------------------------------------------------------------

#ifdef DEBUG
		printf("CHECK FOR MESSAGES\n");
#endif // DEBUG
		if (current_RX_messages.sysid == -1)
		{
			printf("Telemetry enabled, waiting for connection with the vehicle...\n");
			while (current_RX_messages.sysid == -1)
			{
				if (time_to_exit) return;
				usleep(500000); // check at 2Hz
			}
		}

#ifdef DEBUG	

		// now we know autopilot is sending messages
		printf("\n");
#endif // DEBUG

		// --------------------------------------------------------------------------
		//   GET SYSTEM and COMPONENT IDs
		// --------------------------------------------------------------------------

		// This comes from the heartbeat, which in theory should only come from
		// the autopilot we're directly connected to it.  If there is more than one
		// vehicle then we can't expect to discover id's like this.
		// In which case set the id's manually.

		// System ID
		if (system_id == -1)
		{
			system_id = current_RX_messages.sysid;
			printf("GOT VEHICLE SYSTEM ID: %i\n", system_id);
		}

		// Component ID
		if (autopilot_id == -1)
		{
			autopilot_id = current_RX_messages.compid;
			printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
			printf("\n");
		}

		if (system_id < 255)
		{
			// --------------------------------------------------------------------------
			//   GET INITIAL POSITION
			// --------------------------------------------------------------------------

			// Wait for initial position ned
			if (settings.enable_print) printf("Waiting for telemetry...\n");
			if (settings.enable_control)
			{
				printf("Attitude Update..."); fflush(stdout);
				usleep(1E5);
				while (not (current_RX_messages.time_stamps.attitude && (current_RX_messages.estimator_status.data.flags & ESTIMATOR_ATTITUDE) == ESTIMATOR_ATTITUDE))
				{
#ifdef DEBUG
					printf("flag = %i\n", current_RX_messages.estimator_status.data.flags);
#endif // DEBUG
					if (time_to_exit)
						return;
					usleep(1E5);
				}
				printf("\tOK\n");			
				
				printf("Horizontal Velocity..."); fflush(stdout);
				usleep(1E5);
				while (not ((current_RX_messages.estimator_status.data.flags & ESTIMATOR_VELOCITY_HORIZ) == ESTIMATOR_VELOCITY_HORIZ))
				{
#ifdef DEBUG
					printf("flag = %i\n", current_RX_messages.estimator_status.data.flags);
#endif // DEBUG

					if (time_to_exit)
						return;
					usleep(1E5);
				}
				printf("\tOK\n");

				printf("Vertical Velocity..."); fflush(stdout);
				usleep(1E5);
				while (not ((current_RX_messages.estimator_status.data.flags & ESTIMATOR_VELOCITY_VERT) == ESTIMATOR_VELOCITY_VERT))
				{
					if (time_to_exit)
						return;
					usleep(1E5);
				}
				printf("\tOK\n");

				printf("Horizontal Position..."); fflush(stdout);
				usleep(1E5);
				while (not ((\
					(current_RX_messages.estimator_status.data.flags & ESTIMATOR_POS_HORIZ_REL) == ESTIMATOR_POS_HORIZ_REL\
					) && (\
					(current_RX_messages.estimator_status.data.flags & ESTIMATOR_POS_HORIZ_ABS) == ESTIMATOR_POS_HORIZ_ABS\
					)))
				{
#ifdef DEBUG
					printf("flag = %i\n", current_RX_messages.estimator_status.data.flags);
#endif // DEBUG
					if (time_to_exit)
						return;
					usleep(1E5);
				}
				printf("\tOK\n");

				printf("Vertical Position..."); fflush(stdout);
				usleep(1E5);
				while (not ((\
					(current_RX_messages.estimator_status.data.flags & ESTIMATOR_POS_VERT_ABS) == ESTIMATOR_POS_VERT_ABS\
					) && (\
						(current_RX_messages.estimator_status.data.flags & ESTIMATOR_POS_VERT_AGL) == ESTIMATOR_POS_VERT_AGL\
						)))
				{
#ifdef DEBUG
					printf("flag = %i\n", current_RX_messages.estimator_status.data.flags);
#endif // DEBUG
					if (time_to_exit)
						return;
					usleep(1E5);
				}
				printf("\tOK\n");

				printf("Horizontal Position Prediction..."); fflush(stdout);
				usleep(1E5);
				while (not ((\
					(current_RX_messages.estimator_status.data.flags & ESTIMATOR_PRED_POS_HORIZ_REL) == ESTIMATOR_PRED_POS_HORIZ_REL\
					) && (\
						(current_RX_messages.estimator_status.data.flags & ESTIMATOR_PRED_POS_HORIZ_ABS) == ESTIMATOR_PRED_POS_HORIZ_ABS\
						)))
				{
#ifdef DEBUG
					printf("flag = %i\n", current_RX_messages.estimator_status.data.flags);
#endif // DEBUG
					if (time_to_exit)
						return;
					usleep(1E5);
				}
				printf("\tOK\n");				

				printf("Initial Position and Velocity..."); fflush(stdout);
				usleep(1E5);
				while (not (current_RX_messages.time_stamps.local_position_ned &&
					current_RX_messages.time_stamps.attitude))
				{
#ifdef DEBUG
					printf("flag = %i\n", current_RX_messages.estimator_status.data.flags);
#endif // DEBUG
					if (time_to_exit)
						return;
					usleep(1E5);
				}
				printf("\tOK\n");

				printf("All good! Receiving initial state...\t %5.1f%s\r",0.0,"%");
				fflush(stdout);
				uint wait_time_us = 10E6;
				uint N_datas = 100;
				mavlink_set_position_target_local_ned_t tmp_rx_pos;
				memset(&tmp_rx_pos, 0, sizeof(tmp_rx_pos));
				for (int i = 0; i < N_datas; i++)
				{
					{
						std::lock_guard<std::mutex> lock(current_RX_messages.local_position_ned.mutex);
						tmp_rx_pos.x += current_RX_messages.local_position_ned.data.x / N_datas;
						tmp_rx_pos.y += current_RX_messages.local_position_ned.data.y / N_datas;
						tmp_rx_pos.z += current_RX_messages.local_position_ned.data.z / N_datas;
						tmp_rx_pos.vx += current_RX_messages.local_position_ned.data.vx / N_datas;
						tmp_rx_pos.vy += current_RX_messages.local_position_ned.data.vy / N_datas;
						tmp_rx_pos.vz += current_RX_messages.local_position_ned.data.vz / N_datas;
						tmp_rx_pos.yaw += current_RX_messages.attitude.data.yaw / N_datas;
						tmp_rx_pos.yaw_rate += current_RX_messages.attitude.data.yawspeed / N_datas;
					}
					printf("All good! Receiving initial state...\t %5.1f%s\r", (double)(i + 1) / (double)N_datas * 100, "%");
					fflush(stdout);
					usleep(wait_time_us / N_datas);
				}
				printf("\nDone!\n");
				initial_position = tmp_rx_pos;

#ifdef DEBUG
				printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
				printf("INITIAL VELOCITY XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.vx, initial_position.vy, initial_position.vz);
				printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
				printf("\n");
#endif // DEBUG	
			}
		}
	}

#ifdef DEBUG
	printf("Check if control is enabled\n");
#endif // DEBUG
	if (settings.enable_control)
	{
		// --------------------------------------------------------------------------
		//   WRITE THREAD
		// --------------------------------------------------------------------------
#ifdef DEBUG
		printf("START WRITE THREAD \n");
#endif // DEBUG

		result = write_tid.start(&start_autopilot_interface_write_thread, this);
		if (result) throw result;

		// now we're streaming setpoint commands
#ifdef DEBUG
		printf("\n");
#endif // DEBUG
	}

#ifdef DEBUG
	printf("Check if printing is enabled\n");
#endif // DEBUG
	if (settings.enable_print)
	{
#ifdef DEBUG
		printf("START PRINTF THREAD \n");
#endif // DEBUG

		result = printf_tid.start(&start_autopilot_interface_printf_thread, this);
		if (result) throw result;

		// now we're streaming setpoint commands
#ifdef DEBUG
		printf("\n");
#endif // DEBUG
	}

	result = nonblock_io_tid.start(&start_autopilot_interface_nonblock_io_thread, this);
	if (result) throw result;

#ifdef DEBUG
	printf("Start routine is completed!\n");
#endif // DEBUG

	// Done!
	return;
}

// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void Autopilot_Interface::stop(void)
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
#ifdef DEBUG
	printf("CLOSE THREADS\n");
#endif // DEBUG

	// signal exit
	time_to_exit = true;

	// wait for exit
	printf_tid.stop(PRINTF_THREAD_TOUT);
	vision_position_estimate_write_tid.stop(VPE_THREAD_TOUT);
	read_tid.stop(READ_THREAD_TOUT);
	write_tid.stop(WRITE_THREAD_TOUT);
	sys_tid.stop(SYS_THREAD_TOUT);
	relay_tid.stop(RELAY_THREAD_TOUT);
	nonblock_io_tid.stop(NONBLOCK_IO_THREAD_TOUT);

#ifdef DEBUG
	printf("Trying to close mocap THREADS\n");
#endif // DEBUG
	mocap.stop();

	// now the read and write threads are closed
#ifdef DEBUG
	printf("All threads are closed\n");
#endif // DEBUG

	// still need to close the target_port separately
	return;
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::start_read_thread(void)
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"ERROR in start_read_thread: thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::start_write_thread(void)
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"ERROR in start_write_thread: thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}

// ------------------------------------------------------------------------------
//   Non-blocking Input-Output Thread (20Hz)
// ------------------------------------------------------------------------------
void Autopilot_Interface::start_nonblock_io_thread(void)
{
	if (not nonblock_io_status == false)
	{
		fprintf(stderr, "ERROR in start_nonblock_io_thread: thread already running\n");
		return;
	}

	else
	{
		nonblock_io_thread();
		return;
	}
}

// ------------------------------------------------------------------------------
//   Sys Thread (1Hz)
// ------------------------------------------------------------------------------
void Autopilot_Interface::start_sys_thread(void)
{
	if (not sys_status == false)
	{
		fprintf(stderr, "ERROR in start_sys_thread: thread already running\n");
		return;
	}

	else
	{
		sys_thread();
		return;
	}

}

// ------------------------------------------------------------------------------
//   Printf Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::start_printf_thread(void)
{
	if (not printf_status == false)
	{
		fprintf(stderr, "ERROR in start_printf_thread: thread already running\n");
		return;
	}

	else
	{
		printf_thread();
		return;
	}

}

// ------------------------------------------------------------------------------
//   Vision Position Estimate Write Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::start_vision_position_estimate_write_thread(void)
{
	if (not vision_position_writing_status == false)
	{
		fprintf(stderr, "ERROR in start_vision_position_estimate_write_thread: thread already running\n");
		return;
	}

	else
	{
		vision_position_estimate_write_thread();
		return;
	}

}

// ------------------------------------------------------------------------------
//   Relay Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::start_relay_thread(void)
{

	if (relay_status != 0)
	{
		fprintf(stderr, "ERROR in start_relay_thread: thread already running\n");
		return;
	}
	else
	{
		relay_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void Autopilot_Interface::handle_quit( int sig )
{
	if (settings.enable_control) disable_offboard_control();	

	try {
		stop();
	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}
	return;
}


// ------------------------------------------------------------------------------
//   Printf
// ------------------------------------------------------------------------------

void __print_header(const char* in, int size)
{
	char tmp[100];
	sprintf(tmp, " %s%i%s |", "%", size, "s");
	printf(tmp,in);
}
void __print_data_float(double in, int size)
{
	char tmp[100];
	sprintf(tmp, " %s+%i.2%s |", "%", size, "f");
	printf(tmp, in);
}
void Autopilot_Interface::print_header(void)
{
	printf("\n| ");
	if (settings.enable_mocap && settings.print_mocap)
	{
		PRINT_HEADER(mocap.Hz);
		PRINT_HEADER(mocap.roll);
		PRINT_HEADER(mocap.pitch);
		PRINT_HEADER(mocap.yaw);
		PRINT_HEADER(mocap.x);
		PRINT_HEADER(mocap.y);
		PRINT_HEADER(mocap.z);
	}
	if (settings.enable_vpe && settings.print_vpe)
	{
		PRINT_HEADER(vpe.Hz);
		PRINT_HEADER(vpe.roll);
		PRINT_HEADER(vpe.pitch);
		PRINT_HEADER(vpe.yaw);
		PRINT_HEADER(vpe.x);
		PRINT_HEADER(vpe.y);
		PRINT_HEADER(vpe.z);
	}
	if (settings.enable_control && settings.print_control)
	{
		PRINT_HEADER(sp.Hz);
		PRINT_HEADER(sp.x);
		PRINT_HEADER(sp.y);
		PRINT_HEADER(sp.z);
		PRINT_HEADER(sp.vx);
		PRINT_HEADER(sp.vy);
		PRINT_HEADER(sp.vz);
		PRINT_HEADER(sp.yaw);
		PRINT_HEADER(sp.yaw_rate);
	}
	if (settings.enable_telemetry && settings.print_telemetry)
	{
		PRINT_HEADER(att.Hz);
		PRINT_HEADER(att.roll);
		PRINT_HEADER(att.pitch);
		PRINT_HEADER(att.yaw);
		PRINT_HEADER(lp.Hz);
		PRINT_HEADER(lp.x);
		PRINT_HEADER(lp.y);
		PRINT_HEADER(lp.z);
	}

	printf("\n");
	fflush(stdout);
	return;
}
void Autopilot_Interface::print_data(void)
{
	// make a local copy of all the data
	mavlink_vision_position_estimate_t vpe;
	mavlink_set_position_target_local_ned_t sp;
	//Mavlink_Messages mav;
	Time_Stamps mav_time_stamps_old;
	uint64_t sp_time_ms_old;
	uint64_t vpe_time_us_old;
	mocap_data_t mc;
	if (settings.enable_mocap && settings.print_mocap)
	{
		mocap.get_data(mc, settings.mocap_ID);
	}
	if (settings.enable_vpe && settings.print_vpe)
	{
		{
			std::lock_guard<std::mutex> lock(current_vision_position_estimate.mutex);
			vpe = current_vision_position_estimate.data;
			vpe_time_us_old = current_vision_position_estimate.time_us_old;
		}

	}
	if (settings.enable_control && settings.print_control)
	{
		{
			std::lock_guard<std::mutex> lock(current_setpoint.mutex);
			sp = current_setpoint.data;
			sp_time_ms_old = current_setpoint.time_ms_old;
		}
	}
	if (settings.enable_telemetry && settings.print_telemetry)
	{
		
		mav_time_stamps_old = time_stamps_old;
	}

	//print data
	printf("\r| ");
	if (settings.enable_mocap && settings.print_mocap)
	{
		PRINT_DATA(1.0E6 / (mc.time_us - mc.time_us_old));
		PRINT_DATA(mc.roll);
		PRINT_DATA(mc.pitch);
		PRINT_DATA(mc.yaw);
		PRINT_DATA(mc.x);
		PRINT_DATA(mc.y);
		PRINT_DATA(mc.z);
	}
	if (settings.enable_vpe && settings.print_vpe)
	{
		PRINT_DATA(1.0E6 / (vpe.usec - vpe_time_us_old));
		PRINT_DATA(vpe.roll);
		PRINT_DATA(vpe.pitch);
		PRINT_DATA(vpe.yaw);
		PRINT_DATA(vpe.x);
		PRINT_DATA(vpe.y);
		PRINT_DATA(vpe.z);
	}
	if (settings.enable_control && settings.print_control)
	{
		//if ((sp.type_mask & MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION) == MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION)
		PRINT_DATA((1.0E3 / (sp.time_boot_ms - sp_time_ms_old)));
		PRINT_DATA(sp.x);
		PRINT_DATA(sp.y);
		PRINT_DATA(sp.z);
		PRINT_DATA(sp.vx);
		PRINT_DATA(sp.vy);
		PRINT_DATA(sp.vz);
		PRINT_DATA(sp.yaw);
		PRINT_DATA(sp.yaw_rate);
	}
	if (settings.enable_telemetry && settings.print_telemetry)
	{
		std::lock_guard<std::mutex> lock(current_RX_messages.attitude.mutex);
		PRINT_DATA((1.0E6 / (current_RX_messages.time_stamps.attitude - mav_time_stamps_old.attitude)));
		PRINT_DATA(current_RX_messages.attitude.data.roll);
		PRINT_DATA(current_RX_messages.attitude.data.pitch);
		PRINT_DATA(current_RX_messages.attitude.data.yaw);
		std::lock_guard<std::mutex> lock2(current_RX_messages.local_position_ned.mutex);
		PRINT_DATA((1.0E6 / (current_RX_messages.time_stamps.local_position_ned - mav_time_stamps_old.local_position_ned)));
		PRINT_DATA(current_RX_messages.local_position_ned.data.x);
		PRINT_DATA(current_RX_messages.local_position_ned.data.y);
		PRINT_DATA(current_RX_messages.local_position_ned.data.z);

	}
	fflush(stdout);
	
	return;
}


// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::read_thread(void)
{
	reading_status = true;
	while ( ! time_to_exit )
	{
		read_messages();
		usleep(1E6 / READ_THREAD_HZ); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}

// ------------------------------------------------------------------------------
//   vision_position_estimate (MOCAP) Write Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::vision_position_estimate_write_thread(void)
{
	// signal startup
	vision_position_writing_status = 2;

	// prepare an initial setpoint, just stay put
	mavlink_vision_position_estimate_t vpe;
	for (int i = 0; i < 21; i++) vpe.covariance[i] = 0;

	//get the data from mocap
	while (!__update_from_mocap(vpe, settings.mocap_ID, mocap, time_to_exit));
	// set vision position estimate
	{
		std::lock_guard<std::mutex> lock(current_vision_position_estimate.mutex);
		current_vision_position_estimate.time_us_old = get_time_usec();
		current_vision_position_estimate.data = vpe;
	}

	// write a message and signal writing
	write_vision_position_estimate();
	vision_position_writing_status = true;
	usleep(20000);   // Stream at 50Hz

	//The messages should be streamed at between 30Hz(if containing covariances) and 50 Hz.
	//If the message rate is too low, EKF2 will not fuse the external vision messages.
	bool tmp = false;
	uint64_t tmp_time_old;
	while (!time_to_exit)
	{		
		tmp = false;
		{
			std::lock_guard<std::mutex> lock(current_vision_position_estimate.mutex);
			tmp_time_old = current_vision_position_estimate.data.usec;
			tmp = __update_from_mocap(current_vision_position_estimate.data, settings.mocap_ID, mocap, time_to_exit);
			if (tmp) current_vision_position_estimate.time_us_old = tmp_time_old;
		}
		if (tmp)
		{
			write_vision_position_estimate();
			usleep(1E6 / VPE_THREAD_HZ);   // Stream at 50Hz
		}
		else usleep(100);
	}

	// signal end
	vision_position_writing_status = false;

	return;

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::write_thread(void)
{
	// signal startup
	writing_status = 2;

	// prepare an initial setpoint, just stay put
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx       = 0.0;
	sp.vy       = 0.0;
	sp.vz       = 0.0;
	sp.yaw_rate = 0.0;

	// set position target
	{
		std::lock_guard<std::mutex> lock(current_setpoint.mutex);
		current_setpoint.data = sp;
	}

	// write a message and signal writing
	write_setpoint();
	writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( !time_to_exit )
	{		
		write_setpoint();
		usleep(1E6 / WRITE_THREAD_HZ);   // Stream at 4Hz
	}

	// signal end
	writing_status = false;

	return;

}

// ------------------------------------------------------------------------------
//   Non-blocking Input-Output Thread (20Hz)
// ------------------------------------------------------------------------------
void Autopilot_Interface::nonblock_io_thread(void)
{
	nonblock_io_status = true;
	while (!time_to_exit)
	{
		update_nonblock_io();
		usleep(1E6 / NONBLOCK_IO_THREAD_HZ);   // Stream at 20Hz
	}

	// signal end
	nonblock_io_status = false;

	return;

}

// ------------------------------------------------------------------------------
//   sys Thread (1Hz)
// ------------------------------------------------------------------------------
void Autopilot_Interface::sys_thread(void)
{
	// signal startup
	sys_status = 2;

	// write a message and signal writing
	sys_write();
	sys_status = true;
	usleep(1E6);   // Stream at 1Hz

	while (!time_to_exit)
	{
		sys_write();
		usleep(1E6 / SYS_THREAD_HZ);   // Stream at 1Hz
	}

	// signal end
	sys_status = false;

	return;

}


// ------------------------------------------------------------------------------
//   Printf Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::printf_thread(void)
{
	// signal startup
	printf_status = 2;

	// put linewrap back on
	printf(WRAP_DISABLE);
	print_header(); //stat by printing a header
	
	printf_status = true;
	while (!time_to_exit)
	{		
		print_data();
		usleep(1E6 / PRINTF_THREAD_HZ);   // Print at 20Hz
	}
	printf("\n");

	// put linewrap back on
	printf(WRAP_ENABLE);
	printf_status = false;
	return;

}

// ------------------------------------------------------------------------------
//   Relay Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::relay_thread(void)
{
	relay_status = true;
	while (!time_to_exit)
	{
		relay_read(); //also writes the messages to target
		usleep(1E6 / RELAY_THREAD_HZ);
	}

	relay_status = false;

	return;
}

// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void* start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}

void* start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}

void* start_autopilot_interface_write_vision_position_estimate_thread(void* args)
{
	// takes an autopilot object argument
	Autopilot_Interface* autopilot_interface = (Autopilot_Interface*)args;

	// run the object's read thread
	autopilot_interface->start_vision_position_estimate_write_thread();

	// done!
	return NULL;
}

void* start_autopilot_interface_sys_thread(void* args)
{
	// takes an autopilot object argument
	Autopilot_Interface* autopilot_interface = (Autopilot_Interface*)args;

	// run the object's read thread
	autopilot_interface->start_sys_thread();

	// done!
	return NULL;
}

void* start_autopilot_interface_nonblock_io_thread(void* args)
{
	// takes an autopilot object argument
	Autopilot_Interface* autopilot_interface = (Autopilot_Interface*)args;

	// run the object's printf thread
	autopilot_interface->start_nonblock_io_thread();

	// done!
	return NULL;
}


void* start_autopilot_interface_printf_thread(void* args)
{
	// takes an autopilot object argument
	Autopilot_Interface* autopilot_interface = (Autopilot_Interface*)args;

	// run the object's printf thread
	autopilot_interface->start_printf_thread();

	// done!
	return NULL;
}

void* start_autopilot_interface_relay_thread(void* args)
{
	// takes an autopilot object argument
	Autopilot_Interface* autopilot_interface = (Autopilot_Interface*)args;

	// run the object's read thread
	autopilot_interface->start_relay_thread();

	// done!
	return NULL;
}
