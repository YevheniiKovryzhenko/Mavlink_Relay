/*
 * mavlink_control.cpp
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
 * Last Edit:  10/24/2022 (MM/DD/YYYY)
 *
 * This process connects an external MAVLink UART device to send and receive data.
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.hpp"
#include "thread_defs.hpp"
//#define DEBUG

bool termination_requested = false;


// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int top (int argc, char **argv)
{

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	// Default input arguments
	settings_t settings;
	settings.enable_target = false;
#ifdef __APPLE__
	settings.target_uart_name = (char*)"/dev/tty.usbmodem1";
	settings.relay_uart_name = (char*)"/dev/tty.usbmodem1";
#else
	settings.target_uart_name = (char*)"/dev/ttyUSB0";
	settings.relay_uart_name = (char*)"/dev/ttyUSB0";
#endif
	settings.target_use_uart = false;
	settings.target_baudrate = 57600;

	settings.target_use_udp = false;
	settings.target_ip = (char*)"127.0.0.1";
	settings.target_port = 14550;
	settings.target_bind_port = 14555;

	settings.enable_relay = false;
#ifdef __APPLE__
	settings.relay_uart_name = (char*)"/dev/tty.usbmodem1";
#else
	settings.relay_uart_name = (char*)"/dev/ttyUSB0";
#endif

	settings.relay_use_uart = false;
	settings.relay_baudrate = 57600;

	settings.relay_use_udp = false;
	settings.relay_ip = (char*)"127.0.0.1";
	settings.relay_port = 14550;
	settings.relay_bind_port = 14555;


	settings.autotakeoff = false;

	settings.enable_mocap = false;
	settings.mocap_YUP2NED = false;
	settings.mocap_ZUP2NED = false;
	settings.mocap_ip = (char*)"127.0.0.1";
	settings.mocap_ID = 1;

	settings.enable_control = false;
	settings.enable_telemetry = true;

	settings.print_control = false;
	settings.print_vpe = false;
	settings.print_telemetry = false;

	// do the parse, will throw an int if it fails
	if (parse_commandline(argc, argv, settings) < 0)
	{
		fprintf(stderr, "ERROR in top: failed to parse compandline arguments\n");
		return -1;
	}

	// check input flags:
	if (settings.enable_target)
	{
		settings.enable_vpe = settings.enable_mocap;
	}
	else
	{
		settings.enable_relay = false;
		settings.enable_control = false;
		settings.enable_vpe = false;
		settings.enable_telemetry = false;
	}

	// --------------------------------------------------------------------------
	//   PORT and THREAD STARTUP
	// --------------------------------------------------------------------------

	/*
	 * Instantiate a generic port object
	 *
	 * This object handles the opening and closing of the offboard computer's
	 * port over which it will communicate to an autopilot.  It has
	 * methods to read and write a mavlink_message_t object.  To help with read
	 * and write in the context of pthreading, it gaurds port operations with a
	 * pthread mutex lock. It can be a serial or an UDP port.
	 *
	 */
	Generic_Port *target_port, *relay_port;

	if (settings.enable_target)
	{
		if (settings.target_use_udp)
		{
			target_port = new UDP_Port(settings.target_ip, settings.target_port, settings.target_bind_port);
		}
		else
		{
			target_port = new Serial_Port(settings.target_uart_name, settings.target_baudrate);
		}
	}

	if (settings.enable_relay)
	{
		if (settings.relay_use_udp)
		{
			relay_port = new UDP_Port(settings.relay_ip, settings.relay_port, settings.relay_bind_port);
		}
		else
		{
			relay_port = new Serial_Port(settings.relay_uart_name, settings.relay_baudrate);
		}
	}

	/*
	* Instantiate an autopilot interface object
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
	*
	*/
	Autopilot_Interface autopilot_interface(target_port, relay_port, settings);


	/*
	* Setup interrupt signal handler
	*
	* Responds to early exits signaled with Ctrl-C.  The handler will command
	* to exit offboard mode if required, and close threads and the port.
	* The handler in this example needs references to the above objects.
	*
	*/
	if (settings.enable_target) target_port_quit = target_port;
	if (settings.enable_relay) relay_port_quit = relay_port;
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT, quit_handler);

#ifdef DEBUG
	printf("quit handler started\n");
#endif // DEBUG

	if (settings.enable_target)
	{
		/*
		* Start the target port
		*/
#ifdef DEBUG
		printf("Starting target port...\n");
#endif // DEBUG
		target_port->start();
	}

	if (settings.enable_relay)
	{
		/*
		* Start the relay port
		*/
#ifdef DEBUG
		printf("Starting relay port...\n");
#endif // DEBUG
		relay_port->start();
	}

#ifdef DEBUG
	printf("Starting autopilot interface...\n");
#endif // DEBUG
	autopilot_interface.start();

	if (settings.enable_target && settings.enable_control)
	{
#ifdef DEBUG
		printf("Running commands port...\n");
#endif // DEBUG
		// --------------------------------------------------------------------------
		//   RUN COMMANDS
		// --------------------------------------------------------------------------
		usleep(5E5);
		/*
		* Now we can implement the algorithm we want on top of the autopilot interface
		*/
		commands(autopilot_interface, settings.autotakeoff);
	}

	while (!termination_requested)
	{
		usleep(100000); //wait till user termination
	}

	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------

	/*
	* Now that we are done we can stop the threads and close the port
	*/
#ifdef DEBUG
	printf("Terminating...\n");
#endif // DEBUG
	autopilot_interface.stop();
	if (settings.enable_target)
	{
		target_port->stop();
		delete target_port;
	}
	if (settings.enable_relay)
	{
		relay_port->stop();
		delete relay_port;
	}
	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------

	// woot!
	return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void commands(Autopilot_Interface &api, bool autotakeoff)
{		
	// initialize command data strtuctures
	mavlink_set_position_target_local_ned_t sp;	
	mavlink_set_position_target_local_ned_t ip = api.initial_position;

	//zero-out velocity terms:
	//Mavlink_Messages local_data = current_RX_messages;
	ip.vx = 0.0;
	ip.vy = 0.0;
	ip.vz = 0.0;
	ip.yaw_rate = 0.0;

	api.update_setpoint(ip);
	api.get_setpoint(sp);
	//for (int i = 0; i < 100; i++)
	//{
		//api.update_setpoint(sp);
	//	usleep(1E6/WRITE_THREAD_HZ);
	//}

	// now the autopilot is accepting setpoint commands

	//if(autotakeoff)
	//{
		// arm autopilot
	if (api.arm_disarm(true) < 0)
	{
		printf("ERROR in commands: failed to arm the system\n");
		return;
	}

	//sleep(3);
	// --------------------------------------------------------------------------
	//   START OFFBOARD MODE
	// --------------------------------------------------------------------------

	if (api.enable_offboard_control() < 0)
	{
		printf("ERROR in commands: failed to put the system into offboard mode\n");
		return;
	}

	//}

		//return;
	
	// --------------------------------------------------------------------------
	//   SEND OFFBOARD COMMANDS
	// --------------------------------------------------------------------------
#ifdef DEBUG
	printf("SEND OFFBOARD COMMANDS\n");
#endif // DEBUG

	

	// autopilot_interface.h provides some helper functions to build the command




	// Example 1 - Fly up by to 2m
	set_position( ip.x ,       // [m]
			 	  ip.y ,       // [m]
				  ip.z - 1.0 , // [m]
				  sp         );

	if(autotakeoff)
	{
		sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF;
	}

	// SEND THE COMMAND
	api.update_setpoint(sp);
	// NOW pixhawk will try to move

	// Wait for 8 seconds, check position
	for (int i=0; i < 20; i++)
	{
		//mavlink_local_position_ned_t pos = api.current_RX_messages.local_position_ned;
#ifdef DEBUG
		//printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
#endif // DEBUG		
		sleep(1);
	}


	// Example 2 - Set Velocity
	//set_velocity( -1.0       , // [m/s]
	//			  -1.0       , // [m/s]
	//			   0.0       , // [m/s]
	//			   sp        );

	// Example 2.1 - Append Yaw Command
	//set_yaw( ip.yaw + 90.0/180.0*M_PI, // [rad]
	//		 sp     );

	// SEND THE COMMAND
	//api.update_setpoint(sp);
	// NOW pixhawk will try to move

	// Wait for 4 seconds, check position
	//for (int i=0; i < 4; i++)
	//{
		//mavlink_local_position_ned_t pos = api.current_RX_messages.local_position_ned;
#ifdef DEBUG
		//printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
#endif // DEBUG		
	//	sleep(1);
	//}

	//if(autotakeoff)
	//{
		// Example 3 - Land using fixed velocity
		set_velocity(  0.0       , // [m/s]
					   0.0       , // [m/s]
					   1.0       , // [m/s]
					   sp        );

		sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND;

		// SEND THE COMMAND
		api.update_setpoint(sp);
		// NOW pixhawk will try to move

		// Wait for 8 seconds, check position
		for (int i=0; i < 8; i++)
		{
			//mavlink_local_position_ned_t pos = api.current_RX_messages.local_position_ned;
#ifdef DEBUG
			//printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
#endif // DEBUG			
			sleep(1);
		}
#ifdef DEBUG
		printf("\n");
#endif // DEBUG	

		// disarm autopilot
		api.arm_disarm(false);
	//}

	// --------------------------------------------------------------------------
	//   STOP OFFBOARD MODE
	// --------------------------------------------------------------------------

	api.disable_offboard_control();

	// now pixhawk isn't listening to setpoint commands


	// --------------------------------------------------------------------------
	//   GET A MESSAGE
	// --------------------------------------------------------------------------
#ifdef DEBUG
	printf("READ SOME MESSAGES \n");
#endif // DEBUG	


	
#ifdef DEBUG
	// local position in ned frame
	mavlink_local_position_ned_t pos;
	{
		std::lock_guard<std::mutex> lock(api.current_RX_messages.local_position_ned.mutex);
		pos = api.current_RX_messages.local_position_ned.data;
	}
	printf("Got message LOCAL_POSITION_NED (spec: https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED)\n");
	printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z);
#endif // DEBUG	

	
#ifdef DEBUG
	// hires imu
	mavlink_highres_imu_t imu;
	{
		std::lock_guard<std::mutex> lock(api.current_RX_messages.highres_imu.mutex);
		imu = api.current_RX_messages.highres_imu.data;
	}
	printf("Got message HIGHRES_IMU (spec: https://mavlink.io/en/messages/common.html#HIGHRES_IMU)\n");
	printf("    ap time:     %lu \n", imu.time_usec);
	printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc, imu.yacc, imu.zacc);
	printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
	printf("    mag  (NED):  % f % f % f (Ga)\n", imu.xmag, imu.ymag, imu.zmag);
	printf("    baro:        %f (mBar) \n", imu.abs_pressure);
	printf("    altitude:    %f (m) \n", imu.pressure_alt);
	printf("    temperature: %f C \n", imu.temperature);

	printf("\n");
#endif // DEBUG

	// --------------------------------------------------------------------------
	//   END OF COMMANDS
	// --------------------------------------------------------------------------

	return;

}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// return -1 if could not parse something
char parse_commandline(int argc, char **argv, settings_t& settings)
{

	// string for command line usage
	const char* commandline_usage = \
		"\n-----------------------------------------------------------------------------------------------------------------------------------------------------\n"
		"						Usage of mavlink_control"
		"\n-----------------------------------------------------------------------------------------------------------------------------------------------------\n"
		"shortcut		full flag   					meaning								default value\n\n"
		"-h 			--help 	    					prints this message\n\n"
		"-et			--enable_target					enable communication with target (uses default udp)		false\n"
		"-td			--target_device					specify target device name					/dev/ttyUSB0\n"
		"-tb			--target_baudrate				specify target baudrate						57600\n"
		"-tu			--target_udp_ip					specify target upd address					127.0.0.1\n"
		"-tp			--target_port					specify target udp port						14550\n"
		"-tbp			--target_bind_port				specify target bind udp port    				14555\n\n"
		"-er			--enable_relay					enable relay communication with target (uses default udp)	false\n"
		"-rd			--relay_device					specify relay device name					/dev/ttyUSB0\n"
		"-rb			--relay_baudrate				specify relay baudrate						57600\n"
		"-ru			--relay_udp_ip					specify relay upd address					127.0.0.1\n"
		"-rp			--relay_port					specify relay udp port						14550\n"
		"-rbp			--relay_bind_port				specify relay bind udp port					14555\n\n"
		"-em			--enable_mocap					enable mocap (uses default ip)  				false\n"
		"-m 			--mocap_ip					specify mocap interface						127.0.0.1\n"
		"-mI			--mocap_ID  					specify frame ID from mocap					1\n"
		"-mY			--mocap_YUP2NED					rotate from Y-Up to NED 					false\n"
		"-mZ			--mocap_ZUP2NED					rotate from Z-Up to NED 					false\n\n"
		"-c 			--enable_control				enable control algorithm					false\n"
		"-t 			--disable_telem					disable telemetry send/receive					false\n\n"		
		"-pc			--print_control					print setpoints to console					false\n"
		"-pm			--print_mocap					print mocap tracking						false\n"
		"-pv			--print_vpe 					print vision position estimate					false\n"
		"-pt			--print_telemetry				print data from the vehicle					false"
		"\n-----------------------------------------------------------------------------------------------------------------------------------------------------\n";
		
	// Read input arguments
	for (int i = 1; i < argc; i++) {

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			return -1;
		}

		// target enable target
		if (strcmp(argv[i], "-et") == 0 || strcmp(argv[i], "--enable_target") == 0) {
			settings.enable_target = true;
			settings.target_use_udp = true;
		}

		// target UART device ID
		if (strcmp(argv[i], "-td") == 0 || strcmp(argv[i], "--target_device") == 0) {
			if (argc > i + 1) {
				i++;
				settings.enable_target = true;
				settings.target_use_uart = true;
				settings.target_uart_name = argv[i];
				settings.target_use_udp = false;
			} else {
				printf("%s\n",commandline_usage);
				return -1;
			}
		}

		// target Baud rate
		if (strcmp(argv[i], "-tb") == 0 || strcmp(argv[i], "--target_baud") == 0) {
			if (argc > i + 1) {
				i++;
				settings.enable_target = true;
				settings.target_use_uart = true;
				settings.target_baudrate = atoi(argv[i]);
				settings.target_use_udp = false;
			} else {
				printf("%s\n",commandline_usage);
				return -1;
			}
		}

		// target UDP ip
		if (strcmp(argv[i], "-tu") == 0 || strcmp(argv[i], "--target_udp_ip") == 0) {
			if (argc > i + 1) {
				i++;
				settings.enable_target = true;
				settings.target_ip = argv[i];
				settings.target_use_udp = true;
				settings.target_use_uart = false;
			} else {
				printf("%s\n",commandline_usage);
				return -1;
			}
		}

		// target UDP port
		if (strcmp(argv[i], "-tp") == 0 || strcmp(argv[i], "--target_port") == 0) {
			if (argc > i + 1) {
				i++;
				settings.enable_target = true;
				settings.target_port = atoi(argv[i]);
				settings.target_use_udp = true;
				settings.target_use_uart = false;
			} else {
				printf("%s\n",commandline_usage);
				return -1;
			}
		}
		
		// target UDP bind port
		if (strcmp(argv[i], "-tbp") == 0 || strcmp(argv[i], "--target_bind_port") == 0) {
			if (argc > i + 1) {
				i++;
				settings.enable_target = true;
				settings.target_bind_port = atoi(argv[i]);
				settings.target_use_udp = true;
				settings.target_use_uart = false;
			}
			else {
				printf("%s\n", commandline_usage);
				return -1;
			}
		}

		// relay enable target
		if (strcmp(argv[i], "-er") == 0 || strcmp(argv[i], "--enable_relay") == 0) {
			settings.enable_relay = true;
			settings.relay_use_udp = true;
		}

		// relay UART device ID
		if (strcmp(argv[i], "-rd") == 0 || strcmp(argv[i], "--relay_device") == 0) {
			if (argc > i + 1) {
				i++;
				settings.enable_relay = true;
				settings.relay_use_uart = true;
				settings.relay_uart_name = argv[i];
				settings.relay_use_udp = false;
			}
			else {
				printf("%s\n", commandline_usage);
				return -1;
			}
		}

		// relay Baud rate
		if (strcmp(argv[i], "-rb") == 0 || strcmp(argv[i], "--relay_baud") == 0) {
			if (argc > i + 1) {
				i++;
				settings.enable_relay = true;
				settings.relay_use_uart = true;
				settings.relay_baudrate = atoi(argv[i]);
				settings.relay_use_udp = false;
			}
			else {
				printf("%s\n", commandline_usage);
				return -1;
			}
		}

		// relay UDP ip
		if (strcmp(argv[i], "-ru") == 0 || strcmp(argv[i], "--relay_udp_ip") == 0) {
			if (argc > i + 1) {
				i++;
				settings.enable_relay = true;
				settings.relay_ip = argv[i];
				settings.relay_use_udp = true;
				settings.relay_use_uart = false;
			}
			else {
				printf("%s\n", commandline_usage);
				return -1;
			}
		}

		// relay UDP port
		if (strcmp(argv[i], "-rp") == 0 || strcmp(argv[i], "--relay_port") == 0) {
			if (argc > i + 1) {
				i++;
				settings.enable_relay = true;
				settings.relay_port = atoi(argv[i]);
				settings.relay_use_udp = true;
				settings.relay_use_uart = false;
			}
			else {
				printf("%s\n", commandline_usage);
				return -1;
			}
		}

		// relay UDP bind port
		if (strcmp(argv[i], "-rb") == 0 || strcmp(argv[i], "--relay_bind_port") == 0) {
			if (argc > i + 1) {
				i++;
				settings.enable_relay = true;
				settings.relay_bind_port = atoi(argv[i]);
				settings.relay_use_udp = true;
				settings.relay_use_uart = false;
			}
			else {
				printf("%s\n", commandline_usage);
				return -1;
			}
		}

		// mocap
		if (strcmp(argv[i], "-em") == 0 || strcmp(argv[i], "--enable_mocap") == 0) {
			settings.enable_mocap = true;
		}
		if (strcmp(argv[i], "-m") == 0 || strcmp(argv[i], "--mocap_ip") == 0) {
			if (argc > i + 1) {
				i++;
				settings.mocap_ip = argv[i];
				settings.enable_mocap = true;
			}
			else {
				printf("%s\n", commandline_usage);
				return -1;
			}
		}
		if (strcmp(argv[i], "-mI") == 0 || strcmp(argv[i], "--mocap_ID") == 0) {
			if (argc > i + 1) {
				i++;
				settings.mocap_ID = atoi(argv[i]);
				settings.enable_mocap = true;
			}
			else {
				printf("%s\n", commandline_usage);
				return -1;
			}
		}
		if (strcmp(argv[i], "-mY") == 0 || strcmp(argv[i], "--mocap_YUP2NED") == 0) {
			settings.mocap_YUP2NED = true;
			if (settings.mocap_ZUP2NED)
			{
				fprintf(stderr, "ERROR can only specify one rotation at a time\n");
				printf("%s\n", commandline_usage);
				return -1;
			}
		}
		if (strcmp(argv[i], "-mZ") == 0 || strcmp(argv[i], "--mocap_ZUP2NED") == 0) {
			settings.mocap_ZUP2NED = true;
			if (settings.mocap_YUP2NED)
			{
				fprintf(stderr, "ERROR can only specify one rotation at a time\n");
				printf("%s\n", commandline_usage);
				return -1;
			}
		}

		//control
		if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--enable_control") == 0) {
			settings.enable_control = true;
		}
		// Autotakeoff
		if (strcmp(argv[i], "-a") == 0 || strcmp(argv[i], "--autotakeoff") == 0) {
			settings.autotakeoff = true;
		}

		//telemetry
		if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--disable_telem") == 0) {
			settings.enable_telemetry = false;
		}

		//printf settings
		if (strcmp(argv[i], "-pm") == 0 || strcmp(argv[i], "--print_mocap") == 0) {
			settings.print_mocap = true;
			settings.enable_print = true;
		}
		if (strcmp(argv[i], "-pv") == 0 || strcmp(argv[i], "--print_vpe") == 0) {
			settings.print_vpe = true;
			settings.enable_print = true;
		}
		if (strcmp(argv[i], "-pc") == 0 || strcmp(argv[i], "--print_control") == 0) {
			settings.print_control = true;
			settings.enable_print = true;
		}
		if (strcmp(argv[i], "-pt") == 0 || strcmp(argv[i], "--print_telemetry") == 0) {
			settings.print_telemetry = true;
			settings.enable_print = true;
		}
	}
	// end: for each input argument

	// Done!
	return 0;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void quit_handler( int sig )
{
	printf("\n");
	printf("WARNING: Detected ctrl+c, terminating the program...\n");
	printf("\n");
	termination_requested = true;

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// port
	if (target_port_quit != nullptr)
	{
		try {
			target_port_quit->stop();
		}
		catch (int error) {}
		delete target_port_quit;
	}

	if (relay_port_quit != nullptr)
	{
		try {
			relay_port_quit->stop();
		}
		catch (int error) {}

		delete relay_port_quit;
	}	
	// end program here
	exit(0);
}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}

}


