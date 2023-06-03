/*
 * udp_port.hpp
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
 * Functions for opening, closing, reading and writing via UDP ports.
 */

#ifndef UDP_PORT_H_
#define UDP_PORT_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <cstdlib>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <pthread.h> // This uses POSIX Threads
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdbool.h>

#include <common/mavlink.h>
#include "generic_port.h"

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------
//   UDP Port Manager Class
// ----------------------------------------------------------------------------------
/*
 * UDP Port Class
 *
 * This object handles the opening and closing of the offboard computer's
 * UDP port over which we'll communicate.  It also has methods to write
 * a byte stream buffer.  MAVlink is not used in this object yet, it's just
 * a serialization interface.  To help with read and write pthreading, it
 * gaurds any port operation with a pthread mutex.
 */
class UDP_Port: public Generic_Port
{

public:

	UDP_Port();
	UDP_Port(const char *target_ip_, int targetPort_, int bind_port_);
	virtual ~UDP_Port();

	char read_message(mavlink_message_t &message, mavlink_channel_t mavlink_channel_);
	int write_message(const mavlink_message_t &message);

	bool is_running(){
		return is_open;
	}
	char start();
	void stop();
	int bytes_available(void);

private:

	mavlink_status_t lastStatus;
	pthread_mutex_t  lock;

	void initialize_defaults();

	const static int BUFF_LEN = 2041;
	uint8_t buff[BUFF_LEN];
	int buff_ptr;
	int buff_len;
	char target_ip[16] = "127.0.0.1";
	char local_ip[16] = "0.0.0.0";
	
	int sock;
	bool is_open;

	struct sockaddr_in target_addr;
	struct sockaddr_in bind_addr;
	uint16_t bind_port;
	uint16_t target_port;
	socklen_t fromlen = sizeof(target_addr);
	int bytes_sent;
	

	int  _read_port(uint8_t &cp);
	//int _read_bind_port(uint8_t& cp);
	int _write_port(char *buf, unsigned len);

};



#endif // UDP_PORT_H_


