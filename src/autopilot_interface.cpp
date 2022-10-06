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
 * Last Edit:  10/05/2022 (MM/DD/YYYY)
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_interface.hpp"
#include "thread_defs.hpp"

// terminal emulator control sequences
#define WRAP_DISABLE	"\033[?7l"
#define WRAP_ENABLE		"\033[?7h"

//other defines
#define PRINTF_DATA_WIDTH 11
#define PRINT_HEADER(in) __print_header(#in, PRINTF_DATA_WIDTH)
#define PRINT_DATA(in) __print_data_float(in, PRINTF_DATA_WIDTH)

//#define DEBUG

static uint64_t __get_dt_us(uint64_t last_time)
{
	return get_time_usec() - last_time;
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
Autopilot_Interface::Autopilot_Interface(Generic_Port *port_)
{
	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	// init threads
	read_tid.init(READ_THREAD_PRI, READ_THREAD_TYPE);
	write_tid.init(WRITE_THREAD_PRI, WRITE_THREAD_TYPE);
	vision_position_estimate_write_tid.init(VPE_THREAD_PRI, VPE_THREAD_TYPE);
	printf_tid.init(PRINTF_THREAD_PRI, PRINTF_THREAD_TYPE);

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	port = port_; // port management object
}
Autopilot_Interface::Autopilot_Interface(Generic_Port* port_, std::string ip_addr_mocap_, int mocap_ID_)
{
	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit = false;  // flag to signal thread exit

	read_tid.init(READ_THREAD_PRI, READ_THREAD_TYPE);
	write_tid.init(WRITE_THREAD_PRI, WRITE_THREAD_TYPE);
	vision_position_estimate_write_tid.init(VPE_THREAD_PRI, VPE_THREAD_TYPE);
	printf_tid.init(PRINTF_THREAD_PRI, PRINTF_THREAD_TYPE);

	system_id = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid = system_id;
	current_messages.compid = autopilot_id;

	port = port_; // port management object
	mocap_ID = mocap_ID_;
	ip_addr_mocap = ip_addr_mocap_;
}

Autopilot_Interface::~Autopilot_Interface(){}

void Autopilot_Interface::enable_control(void)
{
	enable_control_fl = true;
	return;
}
void Autopilot_Interface::enable_telemetry(void)
{
	enable_telemetry_fl = true;
	return;
}
void Autopilot_Interface::enable_mocap(void)
{
	enable_mocap_fl = true;
	return;
}
void Autopilot_Interface::toggle_mocap_YUP2NED(bool in)
{
	mocap.togle_YUP2NED(in);
	return;
}
void Autopilot_Interface::toggle_mocap_ZUP2NED(bool in)
{
	mocap.togle_ZUP2NED(in);
	return;
}
void Autopilot_Interface::enable_vpe(void)
{
	enable_vpe_fl = true;
	return;
}
void Autopilot_Interface::enable_print_mocap(void)
{
	enable_printf_fl = true;
	printf_mocap_fl = true;
	return;
}
void Autopilot_Interface::enable_print_vpe(void)
{
	enable_printf_fl = true;
	printf_vpe_fl = true;
	return;
}
void Autopilot_Interface::enable_print_control(void)
{
	enable_printf_fl = true;
	printf_control_fl = true;
	return;
}
void Autopilot_Interface::enable_print_telemetry(void)
{
	enable_printf_fl = true;
	printf_telemetry_fl = true;
	return;
}

// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void Autopilot_Interface::update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	std::lock_guard<std::mutex> lock(current_setpoint.mutex);
	current_setpoint.data = setpoint;
}

// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void Autopilot_Interface::read_messages(void)
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	//while ( !received_all and !time_to_exit )
	//{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
#ifdef DEBUG
					printf("MAVLINK_MSG_ID_HEARTBEAT\n");
#endif // DEBUG
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					time_stamps_old.heartbeat = current_messages.time_stamps.heartbeat;
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
#ifdef DEBUG
					printf("MAVLINK_MSG_ID_SYS_STATUS\n");
#endif // DEBUG
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					time_stamps_old.sys_status = current_messages.time_stamps.sys_status;
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
#ifdef DEBUG
					printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
#endif // DEBUG
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					time_stamps_old.battery_status = current_messages.time_stamps.battery_status;
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
#ifdef DEBUG
					printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
#endif // DEBUG
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					time_stamps_old.radio_status = current_messages.time_stamps.radio_status;
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
#ifdef DEBUG
					printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
#endif // DEBUG
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					time_stamps_old.local_position_ned = current_messages.time_stamps.local_position_ned;
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
#ifdef DEBUG
					printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
#endif // DEBUG
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					time_stamps_old.global_position_int = current_messages.time_stamps.global_position_int;
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
#ifdef DEBUG
					printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
#endif // DEBUG
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					time_stamps_old.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
#ifdef DEBUG
					printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
#endif // DEBUG
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					time_stamps_old.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
#ifdef DEBUG
					printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
#endif // DEBUG
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					time_stamps_old.highres_imu = current_messages.time_stamps.highres_imu;
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
#ifdef DEBUG
					printf("MAVLINK_MSG_ID_ATTITUDE\n");
#endif // DEBUG
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					time_stamps_old.attitude = current_messages.time_stamps.attitude;
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
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

		// Check for receipt of all items
		//received_all =
		//		this_timestamps.heartbeat                  &&
//				this_timestamps.battery_status             &&
//				this_timestamps.radio_status               &&
//				this_timestamps.local_position_ned         &&
//				this_timestamps.global_position_int        &&
//				this_timestamps.position_target_local_ned  &&
//				this_timestamps.position_target_global_int &&
//				this_timestamps.highres_imu                &&
//				this_timestamps.attitude                   &&
		//		this_timestamps.sys_status
		//		;
		
		// give the write thread time to use the port
		//if ( writing_status > false ) {
		//	usleep(100); // look for components of batches at 10kHz
		//}

	//} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int Autopilot_Interface::write_message(mavlink_message_t message)
{
	// do the write
	int len = port->write_message(message);

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
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void Autopilot_Interface::write_setpoint(void)
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------


	// pull from position target
	mavlink_set_position_target_local_ned_t sp;
	{
		std::lock_guard<std::mutex> lock(current_setpoint.mutex);
		sp = current_setpoint.data;
		current_setpoint.time_ms_old = current_setpoint.data.time_boot_ms;
	}

	// double check some system parameters
	if ( not sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;


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
void Autopilot_Interface::enable_offboard_control()
{
	// Should only send this command once
	if ( control_status == false )
	{
#ifdef DEBUG
		printf("ENABLE OFFBOARD MODE\n");
#endif // DEBUG	

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard_control( true );

		// Check the command was written
		if ( success )
			control_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}
#ifdef DEBUG
		printf("\n");
#endif // DEBUG

	} // end: if not offboard_status

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::disable_offboard_control()
{

	// Should only send this command once
	if ( control_status == true )
	{
#ifdef DEBUG
		printf("DISABLE OFFBOARD MODE\n");
#endif // DEBUG	

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_offboard_control( false );

		// Check the command was written
		if ( success )
			control_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}
#ifdef DEBUG
		printf("\n");
#endif // DEBUG

	} // end: if offboard_status

}

// ------------------------------------------------------------------------------
//   Arm
// ------------------------------------------------------------------------------
int
Autopilot_Interface::arm_disarm( bool flag )
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
	com.confirmation     = true;
	com.param1           = (float) flag;
	com.param2           = 21196;

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = port->write_message(message);

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
Autopilot_Interface::toggle_offboard_control( bool flag )
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = true;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = port->write_message(message);

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void Autopilot_Interface::start(void)
{
	int result;
#ifdef DEBUG
	printf("trying to start autopilot interface...\n");
#endif // DEBUG


	if (enable_telemetry_fl)
	{
		// --------------------------------------------------------------------------
		//   CHECK PORT
		// --------------------------------------------------------------------------

#ifdef DEBUG
		printf("Checking if port is running...\n");
#endif // DEBUG
		if (!port->is_running()) // PORT_OPEN
		{
			fprintf(stderr, "ERROR: port not open\n");
			throw 1;
		}
#ifdef DEBUG
		printf("Good.\n");
#endif // DEBUG
	}

#ifdef DEBUG
	printf("Check if mocap is enabled\n");
#endif // DEBUG
	if (enable_mocap_fl)
	{
#ifdef DEBUG
		printf("mocap is enabled, starting...\n");
#endif // DEBUG
		// --------------------------------------------------------------------------
		//   MOCAP READ THREAD
		// --------------------------------------------------------------------------
#ifdef DEBUG
		printf("START MOCAP READ THREAD \n");
#endif // DEBUG
		result = mocap.start(ip_addr_mocap);
		if (result) throw result;

		// now we're reading messages from mocap
#ifdef DEBUG
		printf("\n");
#endif // DEBUG
	}
#ifdef DEBUG
	printf("Check if telemetry is enabled\n");
#endif // DEBUG
	if (enable_telemetry_fl)
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
	if (enable_vpe_fl)
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
	if (enable_telemetry_fl)
	{
		// --------------------------------------------------------------------------
		//   CHECK FOR MESSAGES
		// --------------------------------------------------------------------------

#ifdef DEBUG
		printf("CHECK FOR MESSAGES\n");
#endif // DEBUG
		printf("Telemetry enabled, waiting for connection with the vehicle...\n");
		while (not current_messages.sysid)
		{
			if (time_to_exit)
				return;
			usleep(500000); // check at 2Hz
		}

#ifdef DEBUG
		printf("Found\n");

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
		if (not system_id)
		{
			system_id = current_messages.sysid;
			printf("GOT VEHICLE SYSTEM ID: %i\n", system_id);
		}

		// Component ID
		if (not autopilot_id)
		{
			autopilot_id = current_messages.compid;
			printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
			printf("\n");
		}

		if (system_id < 255)
		{
			// --------------------------------------------------------------------------
			//   GET INITIAL POSITION
			// --------------------------------------------------------------------------

			// Wait for initial position ned
			printf("Waiting for telemetry...\n");
			while (not (current_messages.time_stamps.local_position_ned &&
				current_messages.time_stamps.attitude))
			{
				if (time_to_exit)
					return;
				usleep(500000);
			}
			printf("Got initial atittude and position!\n");

			// copy initial position ned
			Mavlink_Messages local_data = current_messages;
			initial_position.x = local_data.local_position_ned.x;
			initial_position.y = local_data.local_position_ned.y;
			initial_position.z = local_data.local_position_ned.z;
			initial_position.vx = local_data.local_position_ned.vx;
			initial_position.vy = local_data.local_position_ned.vy;
			initial_position.vz = local_data.local_position_ned.vz;
			initial_position.yaw = local_data.attitude.yaw;
			initial_position.yaw_rate = local_data.attitude.yawspeed;
		}
#ifdef DEBUG
		printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
		printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
		printf("\n");
#endif // DEBUG	
	}

#ifdef DEBUG
	printf("Check if control is enabled\n");
#endif // DEBUG
	if (enable_control_fl)
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
	if (enable_printf_fl)
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

#ifdef DEBUG
	printf("Trying to close mocap THREADS\n");
#endif // DEBUG
	mocap.stop();

	// now the read and write threads are closed
#ifdef DEBUG
	printf("All threads are closed\n");
#endif // DEBUG

	// still need to close the port separately
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
//   Quit Handler
// ------------------------------------------------------------------------------
void Autopilot_Interface::handle_quit( int sig )
{

	disable_offboard_control();

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}
	return;

}
void Autopilot_Interface::handle_quit_no_control(int sig)
{

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr, "Warning, could not stop autopilot interface\n");
	}

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
	if (enable_mocap_fl && printf_mocap_fl)
	{
		PRINT_HEADER(mocap.Hz);
		PRINT_HEADER(mocap.roll);
		PRINT_HEADER(mocap.pitch);
		PRINT_HEADER(mocap.yaw);
		PRINT_HEADER(mocap.x);
		PRINT_HEADER(mocap.y);
		PRINT_HEADER(mocap.z);
	}
	if (enable_vpe_fl && printf_vpe_fl)
	{
		PRINT_HEADER(vpe.Hz);
		PRINT_HEADER(vpe.roll);
		PRINT_HEADER(vpe.pitch);
		PRINT_HEADER(vpe.yaw);
		PRINT_HEADER(vpe.x);
		PRINT_HEADER(vpe.y);
		PRINT_HEADER(vpe.z);
	}
	if (enable_control_fl && printf_control_fl)
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
	if (enable_telemetry_fl && printf_telemetry_fl)
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
	Mavlink_Messages mav;
	Time_Stamps mav_time_stamps_old;
	uint64_t sp_time_ms_old;
	uint64_t vpe_time_us_old;
	mocap_data_t mc;
	if (enable_mocap_fl && printf_mocap_fl)
	{
		mocap.get_data(mc, mocap_ID);
	}
	if (enable_vpe_fl && printf_vpe_fl)
	{
		{
			std::lock_guard<std::mutex> lock(current_vision_position_estimate.mutex);
			vpe = current_vision_position_estimate.data;
			vpe_time_us_old = current_vision_position_estimate.time_us_old;
		}

	}
	if (enable_control_fl && printf_control_fl)
	{
		{
			std::lock_guard<std::mutex> lock(current_setpoint.mutex);
			sp = current_setpoint.data;
			sp_time_ms_old = current_setpoint.time_ms_old;
		}
	}
	if (enable_telemetry_fl && printf_telemetry_fl)
	{
		mav = current_messages;
		mav_time_stamps_old = time_stamps_old;
	}

	//print data
	printf("\r| ");
	if (enable_mocap_fl && printf_mocap_fl)
	{
		PRINT_DATA(1.0E6 / (mc.time_us - mc.time_us_old));
		PRINT_DATA(mc.roll);
		PRINT_DATA(mc.pitch);
		PRINT_DATA(mc.yaw);
		PRINT_DATA(mc.x);
		PRINT_DATA(mc.y);
		PRINT_DATA(mc.z);
	}
	if (enable_vpe_fl && printf_vpe_fl)
	{
		PRINT_DATA(1.0E6 / (vpe.usec - vpe_time_us_old));
		PRINT_DATA(vpe.roll);
		PRINT_DATA(vpe.pitch);
		PRINT_DATA(vpe.yaw);
		PRINT_DATA(vpe.x);
		PRINT_DATA(vpe.y);
		PRINT_DATA(vpe.z);
	}
	if (enable_control_fl && printf_control_fl)
	{
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
	if (enable_telemetry_fl && printf_telemetry_fl)
	{
		PRINT_DATA((1.0E6 / (mav.time_stamps.attitude - mav_time_stamps_old.attitude)));
		PRINT_DATA(mav.attitude.roll);
		PRINT_DATA(mav.attitude.pitch);
		PRINT_DATA(mav.attitude.yaw);
		PRINT_DATA((1.0E6 / (mav.time_stamps.local_position_ned - mav_time_stamps_old.local_position_ned)));
		PRINT_DATA(mav.local_position_ned.x);
		PRINT_DATA(mav.local_position_ned.y);
		PRINT_DATA(mav.local_position_ned.z);

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
	uint64_t tmp_time_us = get_time_usec();
	while ( ! time_to_exit )
	{
		read_messages();
		usleep(1E6 / READ_THREAD_HZ); // Read batches at 10Hz
		tmp_time_us = get_time_usec();
	}

	reading_status = false;

	return;
}



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

int __quaternion_multiply_array(double a[4], double b[4], double c[4])
{
	//if (unlikely(a == NULL || b == NULL || c == NULL)) {
	//	fprintf(stderr, "ERROR: in rc_quaternion_multiply_array, received NULL pointer\n");
	//	return -1;
	//}

	int i, j;
	double tmp[4][4];
	// construct tmp matrix
	tmp[0][0] = a[0];
	tmp[0][1] = -a[1];
	tmp[0][2] = -a[2];
	tmp[0][3] = -a[3];
	tmp[1][0] = a[1];
	tmp[1][1] = a[0];
	tmp[1][2] = -a[3];
	tmp[1][3] = a[2];
	tmp[2][0] = a[2];
	tmp[2][1] = a[3];
	tmp[2][2] = a[0];
	tmp[2][3] = -a[1];
	tmp[3][0] = a[3];
	tmp[3][1] = -a[2];
	tmp[3][2] = a[1];
	tmp[3][3] = a[0];
	// multiply
	for (i = 0; i < 4; i++) {
		c[i] = 0.0;
		for (j = 0; j < 4; j++) c[i] += tmp[i][j] * b[j];
	}
	return 0;
}

int __quaternion_rotate_array(double p[4], double q[4])
{
	double conj[4], tmp[4];
	//if (unlikely(p == NULL || q == NULL)) {
	//	fprintf(stderr, "ERROR: in rc_quaternion_rotate_array, received NULL pointer\n");
	//	return -1;
	//}
	// make a conjugate of q
	conj[0] = q[0];
	conj[1] = -q[1];
	conj[2] = -q[2];
	conj[3] = -q[3];
	// multiply tmp=pq*
	__quaternion_multiply_array(p, conj, tmp);
	// multiply p'=q*tmp
	__quaternion_multiply_array(q, tmp, p);
	return 0;
}

int __quaternion_rotate_vector_array(double v[3], double q[4])
{
	double vq[4];
	//if (unlikely(v == NULL || q == NULL)) {
	//	fprintf(stderr, "ERROR: in rc_quaternion_rotate_vector_array, received NULL pointer\n");
	//	return -1;
	//}
	// duplicate v into a quaternion with 0 real part
	vq[0] = 0.0;
	vq[1] = v[0];
	vq[2] = v[1];
	vq[3] = v[2];
	// rotate quaternion vector
	__quaternion_rotate_array(vq, q);
	// populate v with result
	v[0] = vq[1];
	v[1] = vq[2];
	v[2] = vq[3];
	return 0;
}

bool __update_from_mocap(mavlink_vision_position_estimate_t& buff_out, int mocap_ID, mocap_node_t& node)
{
	bool data_is_good = false;
	
	//get the data from mocap
	mocap_data_t tmp;	
	node.get_data(tmp, mocap_ID);
	if (tmp.trackingValid) //check if tracking was done properly
	{
		mavlink_vision_position_estimate_t tmp_2;
		__copy_data(tmp_2, tmp);
		double tmp_q[4] = { tmp.qw, tmp.qx, tmp.qy, tmp.qz };
		double tmp_pos[3] = { tmp_2.x, tmp_2.y, tmp_2.z };
		__quaternion_rotate_vector_array(tmp_pos, tmp_q);
		tmp_2.x = tmp_pos[0];
		tmp_2.y = tmp_pos[1];
		tmp_2.z = tmp_pos[2];
		if (!__is_vision_data_same(tmp_2, buff_out))
		{
			//__copy_data(buff_out, tmp);
			buff_out = tmp_2;
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

// ------------------------------------------------------------------------------
//   vision_position_estimate (MOCAP) Write Thread
// ------------------------------------------------------------------------------
void Autopilot_Interface::vision_position_estimate_write_thread(void)
{
	// signal startup
	vision_position_writing_status = 2;

	// prepare an initial setpoint, just stay put
	mavlink_vision_position_estimate_t vpe;
	vpe.covariance[0] = NAN;

	//get the data from mocap
	while (!__update_from_mocap(vpe, mocap_ID, mocap));
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
	while (!time_to_exit)
	{
		uint64_t tmp_time_old;
		tmp = false;
		{
			std::lock_guard<std::mutex> lock(current_vision_position_estimate.mutex);
			tmp_time_old = current_vision_position_estimate.data.usec;
			tmp = __update_from_mocap(current_vision_position_estimate.data, mocap_ID, mocap);
			if (tmp) current_vision_position_estimate.time_us_old = tmp_time_old;
		}
		if (tmp)
		{			
			write_vision_position_estimate();
			usleep(1E6 / VPE_THREAD_HZ);   // Stream at 50Hz
		}		
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
	uint64_t time_tmp = get_time_usec();
	while ( !time_to_exit )
	{		
		write_setpoint();
		usleep(1E6 / WRITE_THREAD_HZ);   // Stream at 4Hz
		time_tmp = get_time_usec();
	}

	// signal end
	writing_status = false;

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
	uint64_t tmp_time_us = get_time_usec();
	while (!time_to_exit)
	{		
		print_data();
		usleep(1E6 / PRINTF_THREAD_HZ);   // Print at 20Hz
		tmp_time_us = get_time_usec();
	}
	printf("\n");

	// put linewrap back on
	printf(WRAP_ENABLE);
	printf_status = false;
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


void* start_autopilot_interface_printf_thread(void* args)
{
	// takes an autopilot object argument
	Autopilot_Interface* autopilot_interface = (Autopilot_Interface*)args;

	// run the object's printf thread
	autopilot_interface->start_printf_thread();

	// done!
	return NULL;
}


