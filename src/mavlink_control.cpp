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
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"
//#define DEBUG

bool termination_requested = false;


// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	// Default input arguments
#ifdef __APPLE__
	char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
	char *uart_name = (char*)"/dev/ttyUSB0";
#endif
	bool use_uart = false;
	int baudrate = 57600;

	bool use_udp = false;
	char *udp_ip = (char*)"127.0.0.1";
	int udp_port = 14540;
	bool autotakeoff = false;

	bool enable_mocap = false;
	char* mocap_ip = (char*)"127.0.0.1";
	int mocap_ID = 1;

	bool enable_control = false;

	bool print_control = false;
	bool print_vpe = false;
	bool print_telemetry = false;

	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, use_uart, uart_name, baudrate, use_udp, udp_ip, udp_port, autotakeoff,
		enable_mocap, mocap_ip, mocap_ID, enable_control, print_control, print_telemetry, print_vpe);

	bool use_uart_or_udp = use_uart || use_udp;
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
	Generic_Port *port;

	if (use_udp)
	{
		port = new UDP_Port(udp_ip, udp_port);
	}
	else
	{
		port = new Serial_Port(uart_name, baudrate);
	}

	/*
	* Instantiate an autopilot interface object
	*
	* This starts two threads for read and write over MAVlink. The read thread
	* listens for any MAVlink message and pushes it to the current_messages
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
	Autopilot_Interface autopilot_interface(port, mocap_ip, mocap_ID);	

	if (enable_control)
	{
		autopilot_interface.enable_control();

		/*
		 * Setup interrupt signal handler
		 *
		 * Responds to early exits signaled with Ctrl-C.  The handler will command
		 * to exit offboard mode if required, and close threads and the port.
		 * The handler in this example needs references to the above objects.
		 *
		 */
		port_quit = port;
		autopilot_interface_quit = &autopilot_interface;
		signal(SIGINT, quit_handler);
	}
	else
	{
		/*
		 * Setup interrupt signal handler
		 *
		 * Responds to early exits signaled with Ctrl-C.  The handler will command
		 * to exit offboard mode if required, and close threads and the port.
		 * The handler in this example needs references to the above objects.
		 *
		 */
		port_quit = port;
		autopilot_interface_quit = &autopilot_interface;
		signal(SIGINT, quit_handler_no_control);

	}
#ifdef DEBUG
	printf("quit handler started\n");
#endif // DEBUG


	if (enable_mocap) //for now don't do control
	{
#ifdef DEBUG
		printf("Enabling mocap...\n");
#endif // DEBUG
		autopilot_interface.enable_vpe();
		if (print_vpe) autopilot_interface.enable_print_vpe();
	}

	if (enable_control)
	{
#ifdef DEBUG
		printf("Enabling control...\n");
#endif // DEBUG
		autopilot_interface.enable_control();
		if (print_control) autopilot_interface.enable_print_control();
	}

#ifdef DEBUG
	printf("Enabling telemetry...\n");
#endif // DEBUG
	autopilot_interface.enable_telemetry();
	if (print_telemetry) autopilot_interface.enable_print_telemetry();

#ifdef DEBUG
	printf("Enabling printing...\n");
#endif // DEBUG

	/*
	* Start the port and autopilot_interface
	* This is where the port is opened, and read and write threads are started.
	*/
#ifdef DEBUG
	printf("Starting port...\n");
#endif // DEBUG
	port->start();
#ifdef DEBUG
	printf("Starting autopilot interface...\n");
#endif // DEBUG
	autopilot_interface.start();

	if (enable_control)
	{
#ifdef DEBUG
		printf("Running commands port...\n");
#endif // DEBUG
		// --------------------------------------------------------------------------
		//   RUN COMMANDS
		// --------------------------------------------------------------------------

		/*
		* Now we can implement the algorithm we want on top of the autopilot interface
		*/
		commands(autopilot_interface, autotakeoff);
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
	port->stop();

	delete port;

	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------

	// woot!
	return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands(Autopilot_Interface &api, bool autotakeoff)
{

	// --------------------------------------------------------------------------
	//   START OFFBOARD MODE
	// --------------------------------------------------------------------------

	api.enable_offboard_control();
	usleep(100); // give some time to let it sink in

	// now the autopilot is accepting setpoint commands

	if(autotakeoff)
	{
		// arm autopilot
		api.arm_disarm(true);
		usleep(100); // give some time to let it sink in
	}

	// --------------------------------------------------------------------------
	//   SEND OFFBOARD COMMANDS
	// --------------------------------------------------------------------------
#ifdef DEBUG
	printf("SEND OFFBOARD COMMANDS\n");
#endif // DEBUG

	// initialize command data strtuctures
	mavlink_set_position_target_local_ned_t sp;
	mavlink_set_position_target_local_ned_t ip = api.initial_position;

	// autopilot_interface.h provides some helper functions to build the command




	// Example 1 - Fly up by to 2m
	set_position( ip.x ,       // [m]
			 	  ip.y ,       // [m]
				  ip.z - 2.0 , // [m]
				  sp         );

	if(autotakeoff)
	{
		sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF;
	}

	// SEND THE COMMAND
	api.update_setpoint(sp);
	// NOW pixhawk will try to move

	// Wait for 8 seconds, check position
	for (int i=0; i < 8; i++)
	{
		mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
#ifdef DEBUG
		printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
#endif // DEBUG		
		sleep(1);
	}


	// Example 2 - Set Velocity
	set_velocity( -1.0       , // [m/s]
				  -1.0       , // [m/s]
				   0.0       , // [m/s]
				   sp        );

	// Example 2.1 - Append Yaw Command
	set_yaw( ip.yaw + 90.0/180.0*M_PI, // [rad]
			 sp     );

	// SEND THE COMMAND
	api.update_setpoint(sp);
	// NOW pixhawk will try to move

	// Wait for 4 seconds, check position
	for (int i=0; i < 4; i++)
	{
		mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
#ifdef DEBUG
		printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
#endif // DEBUG		
		sleep(1);
	}

	if(autotakeoff)
	{
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
			mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
#ifdef DEBUG
			printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
#endif // DEBUG			
			sleep(1);
		}
#ifdef DEBUG
		printf("\n");
#endif // DEBUG	

		// disarm autopilot
		api.arm_disarm(false);
		usleep(100); // give some time to let it sink in
	}

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

	// copy current messages
	Mavlink_Messages messages = api.current_messages;

	// local position in ned frame
	mavlink_local_position_ned_t pos = messages.local_position_ned;
#ifdef DEBUG
	printf("Got message LOCAL_POSITION_NED (spec: https://mavlink.io/en/messages/common.html#LOCAL_POSITION_NED)\n");
	printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z);
#endif // DEBUG	

	// hires imu
	mavlink_highres_imu_t imu = messages.highres_imu;
#ifdef DEBUG
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
// throws EXIT_FAILURE if could not open the port
void parse_commandline(int argc, char **argv, bool use_uart, char *&uart_name, int &baudrate,\
		bool &use_udp, char *&udp_ip, int &udp_port, bool &autotakeoff,\
	bool &enable_mocap, char *&mocap_ip, int& mocap_ID, bool& enable_control,\
	bool& print_control, bool& print_telemetry, bool& print_vpe)
{

	// string for command line usage
	const char* commandline_usage = \
		"usage of mavlink_control:\n"
		"-h				help			prints this message)\n"
		"-d				device			specify device name,		/dev/ttyUSB0\n"
		"-b				baudrate		specify baudrate,			57600\n"
		"-u				udp_ip			specify upd address,		127.0.0.1\n"
		"-p				port			specify udp port,			14540\n"
		"-c				enable_control	enable control algorithm\n"
		"-m				mocap_ip		specify mocap interface		127.0.0.1\n"
		"-mI			mocap_ID		specify frame ID from mocap	1\n"
		"-pc			print_control	print setpoints to console\n"
		"-pm			print_mocap		print mocap tracking\n"
		"-pt			print_telemetry print data from the vehicle\n";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				i++;
				use_uart = true;
				uart_name = argv[i];
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				i++;
				baudrate = atoi(argv[i]);
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// UDP ip
		if (strcmp(argv[i], "-u") == 0 || strcmp(argv[i], "--udp_ip") == 0) {
			if (argc > i + 1) {
				i++;
				udp_ip = argv[i];
				use_udp = true;
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// UDP port
		if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--port") == 0) {
			if (argc > i + 1) {
				i++;
				udp_port = atoi(argv[i]);
			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Autotakeoff
		if (strcmp(argv[i], "-a") == 0 || strcmp(argv[i], "--autotakeoff") == 0) {
			autotakeoff = true;
		}

		// mocap
		if (strcmp(argv[i], "-m") == 0 || strcmp(argv[i], "--mocap_ip") == 0) {
			if (argc > i + 1) {
				i++;
				mocap_ip = argv[i];
				enable_mocap = true;
			}
			else {
				printf("%s\n", commandline_usage);
				throw EXIT_FAILURE;
			}
		}
		if (strcmp(argv[i], "-mI") == 0 || strcmp(argv[i], "--mocap_ID") == 0) {
			if (argc > i + 1) {
				i++;
				mocap_ID = atoi(argv[i]);
				enable_mocap = true;
			}
			else {
				printf("%s\n", commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--enable_control") == 0) {
			enable_control = true;
		}

		//printf settings
		if (strcmp(argv[i], "-pm") == 0 || strcmp(argv[i], "--print_mocap") == 0) {
			print_vpe = true;
		}
		if (strcmp(argv[i], "-pc") == 0 || strcmp(argv[i], "--print_control") == 0) {
			print_control = true;
		}
		if (strcmp(argv[i], "-pt") == 0 || strcmp(argv[i], "--print_telemetry") == 0) {
			print_telemetry = true;
		}
	}
	// end: for each input argument

	// Done!
	return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void quit_handler( int sig )
{
#ifdef DEBUG
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");
#endif // DEBUG	
	termination_requested = true;

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// port
	try {
		port_quit->stop();
	}
	catch (int error){}

	// end program here
	exit(0);

}
void quit_handler_no_control(int sig)
{
#ifdef DEBUG
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");
#endif // DEBUG
	termination_requested = true;

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit_no_control(sig);
	}
	catch (int error) {}

	// port
	try {
		port_quit->stop();
	}
	catch (int error) {}

	// end program here
	exit(0);

}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
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


