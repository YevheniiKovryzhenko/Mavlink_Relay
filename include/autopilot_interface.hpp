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
 * Last Edit:  10/14/2022 (MM/DD/YYYY)
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 */


#ifndef AUTOPILOT_INTERFACE_HPP
#define AUTOPILOT_INTERFACE_HPP

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "generic_port.h"
#include "mocap_node.hpp"

#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h>  // UNIX standard function definitions
#include <mutex>
#include "thread_gen.hpp"
#include <common/mavlink.h>
#include "settings.hpp"

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

/**
 * Defines for mavlink_set_position_target_local_ned_t.type_mask
 *
 * Bitmask to indicate which dimensions should be ignored by the vehicle
 *
 * a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
 * the setpoint dimensions should be ignored.
 *
 * If bit 10 is set the floats afx afy afz should be interpreted as force
 * instead of acceleration.
 *
 * Mapping:
 * bit 1: x,
 * bit 2: y,
 * bit 3: z,
 * bit 4: vx,
 * bit 5: vy,
 * bit 6: vz,
 * bit 7: ax,
 * bit 8: ay,
 * bit 9: az,
 * bit 10: is force setpoint,
 * bit 11: yaw,
 * bit 12: yaw rate
 * remaining bits unused
 *
 * Combine bitmasks with bitwise &
 *
 * Example for position and yaw angle:
 * uint16_t type_mask =
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
 */

                                                // bit number  876543210987654321
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111

#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF      0x1000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND         0x2000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LOITER       0x3000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_IDLE         0x4000

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------


// helper functions
void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp);
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp);
void set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp);
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp);
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp);

void* start_autopilot_interface_read_thread(void *args);
void* start_autopilot_interface_write_thread(void *args);
void* start_autopilot_interface_write_vision_position_estimate_thread(void* args);
void* start_autopilot_interface_printf_thread(void* args);
void* start_autopilot_interface_nonblock_io_thread(void* args);
void* start_autopilot_interface_sys_thread(void* args);
void* start_autopilot_interface_relay_thread(void* args);

// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

struct Time_Stamps
{
	Time_Stamps()
	{
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
	uint64_t vision_position_estimate;
	uint64_t odometry;
	uint64_t altitude;
	uint64_t estimator_status;
	uint64_t command_int;
	uint64_t command_long;
	uint64_t command_ack;

	void reset_timestamps()
	{
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
		vision_position_estimate = 0;
		odometry = 0;
		altitude = 0;
		estimator_status = 0;
		command_int = 0;
		command_long = 0;
		command_ack = 0;
	}

};


// Struct containing information on the MAV we are currently connected to

struct Mavlink_Messages {	

	int sysid = -1;
	int compid = -1;

	// Heartbeat
	struct {
		std::mutex mutex;
		mavlink_heartbeat_t data;
	}heartbeat;

	// System Status
	struct {
		std::mutex mutex;
		mavlink_sys_status_t data;
	}sys_status;

	// Battery Status
	struct {
		std::mutex mutex;
		mavlink_battery_status_t data;
	}battery_status;

	// Radio Status
	struct {
		std::mutex mutex;
		mavlink_radio_status_t data;
	}radio_status;

	// Local Position
	struct {
		std::mutex mutex;
		mavlink_local_position_ned_t data;
	}local_position_ned;

	// Global Position
	struct {
		std::mutex mutex;
		mavlink_global_position_int_t data;
	}global_position_int;

	// Local Position Target
	struct {
		std::mutex mutex;
		mavlink_position_target_local_ned_t data;
	}position_target_local_ned;

	// Global Position Target
	struct {
		std::mutex mutex;
		mavlink_position_target_global_int_t data;
	}position_target_global_int;

	// HiRes IMU
	struct {
		std::mutex mutex;
		mavlink_highres_imu_t data;
	}highres_imu;

	// Attitude
	struct {
		std::mutex mutex;
		mavlink_attitude_t data;
	}attitude;

	// Vision position and attitude estimate
	struct {
		std::mutex mutex;
		mavlink_vision_position_estimate_t data;
	}vision_position_estimate;

	// Odometry
	struct {
		std::mutex mutex;
		mavlink_odometry_t data;
	}odometry;

	// Altitude
	struct {
		std::mutex mutex;
		mavlink_altitude_t data;
	}altitude;
	
	// Estimator status
	struct {
		std::mutex mutex;
		mavlink_estimator_status_t data;
	}estimator_status;

	// Message for encoding a command
	struct {
		std::mutex mutex;
		mavlink_command_int_t data;
	}command_int;

	// Message for encoding a command
	struct {
		std::mutex mutex;
		mavlink_command_long_t data;
	}command_long;

	// Command acknowledgement
	struct {
		std::mutex mutex;
		mavlink_command_ack_t data;
	}command_ack;

	// System Parameters?


	// Time Stamps
	Time_Stamps time_stamps;

	void reset_timestamps()
	{
		time_stamps.reset_timestamps();
	}

};


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------
/*
 * Autopilot Interface Class
 *
 * This starts two threads for read and write over MAVlink. The read thread
 * listens for any MAVlink message and pushes it to the current_RX_messages
 * attribute.  The write thread at the moment only streams a position target
 * in the local NED frame (mavlink_set_position_target_local_ned_t), which
 * is changed by using the method update_setpoint().  Sending these messages
 * are only half the requirement to get response from the autopilot, a signal
 * to enter "offboard_control" mode is sent by using the enable_offboard_control()
 * method.  Signal the exit of this mode with disable_offboard_control().  It's
 * important that one way or another this program signals offboard mode exit,
 * otherwise the vehicle will go into failsafe.
 */
class Autopilot_Interface
{

public:

	Autopilot_Interface();
	Autopilot_Interface(Generic_Port* target_port_, settings_t& settings_);
	Autopilot_Interface(Generic_Port* target_port_, Generic_Port* relay_port_, settings_t& settings_);
	Autopilot_Interface(Generic_Port* target_port_, Generic_Port* relay_port_, Generic_Port* relay_port2_, settings_t& settings_);
	~Autopilot_Interface();	

	char printf_status;
	char vision_position_writing_status;
	char sys_status;
	char nonblock_io_status;
	char reading_status;
	char relay_status;
	char writing_status;
	char control_status;
    uint64_t write_count;
	uint64_t relay_write_count;

    int system_id;
	int autopilot_id;
	int companion_id;

	Mavlink_Messages current_RX_messages;
	
	
	//Mavlink_Messages current_TX_messages;
	mavlink_set_position_target_local_ned_t initial_position;

	void get_setpoint(mavlink_set_position_target_local_ned_t& setpoint);
	void update_setpoint(mavlink_set_position_target_local_ned_t setpoint);
	void read_messages();
	int write_message(mavlink_message_t message);
	int relay_message(mavlink_message_t& message);

	char arm_disarm( bool flag );
	char enable_offboard_control(void);
	char disable_offboard_control(void);

	void start(void);
	void stop(void);

	void start_vision_position_estimate_write_thread(void);
	void start_read_thread(void);
	void start_write_thread(void);
	void start_printf_thread(void);
	void start_nonblock_io_thread(void);
	void start_sys_thread(void);
	void start_relay_thread(void);

	void handle_quit( int sig );

	settings_t settings;

private:
	void init(Generic_Port* target_port_, settings_t& settings_);
	void init(Generic_Port* target_port_, Generic_Port* relay_port_ , settings_t& settings_);
	void init(Generic_Port* target_port_, Generic_Port* relay_port_ , Generic_Port* relay_port2_, settings_t& settings_);

	Time_Stamps time_stamps_old;

	mocap_node_t mocap;

	Generic_Port *target_port, *relay_port, *relay_port2;

	bool time_to_exit;

	thread_gen_t read_tid;
	thread_gen_t write_tid;
	thread_gen_t vision_position_estimate_write_tid;
	thread_gen_t printf_tid;
	thread_gen_t sys_tid;
	thread_gen_t nonblock_io_tid;
	thread_gen_t relay_tid;

	struct {
		std::mutex mutex;
		mavlink_set_position_target_local_ned_t data;
		uint64_t time_ms_old;
		bool new_data_available = false;
	} current_setpoint;

	struct {
		std::mutex mutex;
		mavlink_vision_position_estimate_t data;
		uint64_t time_us_old;
	} current_vision_position_estimate;

	void read_thread();
	void write_thread(void);
	void vision_position_estimate_write_thread(void);
	void printf_thread(void);
	void nonblock_io_thread(void);
	void sys_thread(void);
	void relay_thread(void);

	void update_nonblock_io(void);
	char toggle_offboard_control( bool flag );
	void write_setpoint();
	void write_vision_position_estimate();
	void sys_write(void);
	void relay_read(void);	
	void print_header(void);
	void print_data(void);
};



#endif // AUTOPILOT_INTERFACE_HPP


