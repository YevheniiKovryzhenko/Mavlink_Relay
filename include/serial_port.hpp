/*
 * serial_port.hpp
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
 * Functions for opening, closing, reading and writing via serial ports.
 */

#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

//#include <cstdlib>
//#include <stdio.h>   // Standard input/output definitions
//#include <unistd.h>  // UNIX standard function definitions
//#include <fcntl.h>   // File control definitions
//#include <termios.h> // POSIX terminal control definitions
//#include <pthread.h> // This uses POSIX Threads
//#include <signal.h>

#include <common/mavlink.h>

#include "generic_port.h"
#include "serialib.h"



// ----------------------------------------------------------------------------------
//   Serial Port Manager Class
// ----------------------------------------------------------------------------------
/*
 * Serial Port Class
 *
 * This object handles the opening and closing of the offboard computer's
 * serial port over which we'll communicate.  It also has methods to write
 * a byte stream buffer.  MAVlink is not used in this object yet, it's just
 * a serialization interface.  To help with read and write pthreading, it
 * gaurds any port operation with a pthread mutex.
 */
class Serial_Port: public Generic_Port
{

public:

	Serial_Port();
	Serial_Port(const char *uart_name_, int baudrate_);
	virtual ~Serial_Port();

	char read_message(mavlink_message_t &message, mavlink_channel_t mavlink_channel_);
	int write_message(const mavlink_message_t &message);

	bool is_running(){
		return is_open;
	}
	char start();
	void stop();
	int bytes_available(void);

private:
	serialib serial;

	//int  fd;
	mavlink_status_t lastStatus;
	pthread_mutex_t  lock;

	void initialize_defaults();

	bool debug;
	const char *uart_name;
	int  baudrate;
	bool is_open;

	int  _read_port(uint8_t &cp);
	int _write_port(char *buf, unsigned len);

};



#endif // SERIAL_PORT_H_


