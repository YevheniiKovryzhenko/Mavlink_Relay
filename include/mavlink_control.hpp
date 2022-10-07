/*
 * mavlink_control.hpp
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
 * Last Edit:  10/07/2022 (MM/DD/YYYY)
 *
 * This process connects an external MAVLink UART device to send and receive data.
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

using std::string;
using namespace std;

#include <common/mavlink.h>

#include "autopilot_interface.hpp"
#include "serial_port.hpp"
#include "udp_port.h"

typedef struct settings_t
{
	bool use_uart;
	char* uart_name;
	int baudrate;

	bool use_udp;
	char* target_ip;
	int target_port;
	int local_port;
	
	bool enable_mocap;
	bool mocap_YUP2NED;
	bool mocap_ZUP2NED;
	char* mocap_ip;
	int mocap_ID;
	
	bool enable_control;
	bool autotakeoff;

	bool enable_telemetry;

	bool print_mocap;
	bool print_control;
	bool print_telemetry;
	bool print_vpe;
}settings_t;

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

int main(int argc, char **argv);
int top(int argc, char **argv);

void commands(Autopilot_Interface &autopilot_interface, bool autotakeoff);
void parse_commandline(int argc, char** argv, settings_t& settings);

// quit handler
Autopilot_Interface *autopilot_interface_quit;
Generic_Port *port_quit;
void quit_handler( int sig );

