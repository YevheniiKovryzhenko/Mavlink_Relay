/*
 * udp_port.cpp
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


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "udp_port.h"
#include <sys/ioctl.h>


// ----------------------------------------------------------------------------------
//   UDP Port Manager Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
UDP_Port::UDP_Port(const char *target_ip_, int targetPort_, int bind_port_)
{
	initialize_defaults();
	target_ip = target_ip_;
	target_port = targetPort_;
	bind_port = bind_port_;
	is_open = false;
}

UDP_Port::UDP_Port()
{
	initialize_defaults();
}

UDP_Port::~UDP_Port()
{
	// destroy mutex
	pthread_mutex_destroy(&lock);
}

void UDP_Port::initialize_defaults()
{
	// Initialize attributes
	target_ip = "127.0.0.1";
	is_open = false;
	sock = -1;
	
	bind_port = 14551; //QGC socket port
	target_port = 14550; //mavlink port

	// Start mutex
	int result = pthread_mutex_init(&lock, NULL);
	if ( result != 0 )
	{
		printf("ERROR in initialize_defaults: mutex init failed\n");
		throw 1;
	}
}


// ------------------------------------------------------------------------------
//   Read from UDP
// ------------------------------------------------------------------------------
int UDP_Port::bytes_available(void)
{
	int count;
	ioctl(sock, FIONREAD, &count);
	return count;
}

char UDP_Port::read_message(mavlink_message_t& message, mavlink_channel_t mavlink_channel_)
{	
	uint8_t          msgReceived = false;

	while (!msgReceived && bytes_available() > 0)
	{
		uint8_t          cp;
		mavlink_status_t status;

		// --------------------------------------------------------------------------
		//   READ FROM PORT
		// --------------------------------------------------------------------------

		// this function locks the port during read
		int result = _read_port(cp);


		// --------------------------------------------------------------------------
		//   PARSE MESSAGE
		// --------------------------------------------------------------------------
		if (result > 0)
		{
			// the parsing
			msgReceived = mavlink_parse_char(mavlink_channel_, cp, &message, &status);
#ifdef DEBUG
			// check for dropped packets
			if ((lastStatus.packet_rx_drop_count != status.packet_rx_drop_count))
			{
				printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
				unsigned char v = cp;
				fprintf(stderr, "%02x ", v);
			}
#endif // DEBUG		
			lastStatus = status;
		}

		// Couldn't read from port
		else
		{
			fprintf(stderr, "ERROR: Could not read, res = %d, errno = %d : %m\n", result, errno);
			break;
		}

		// --------------------------------------------------------------------------
		//   DEBUGGING REPORTS
		// --------------------------------------------------------------------------
#ifdef DEBUG
		if (msgReceived)
		{
			// Report info
			printf("Received message from UDP with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);

			fprintf(stderr, "Received UDP data: ");
			unsigned int i;
			uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

			// check message is write length
			unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);

			// message length error
			if (messageLength > MAVLINK_MAX_PACKET_LEN)
			{
				fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
			}

			// print out the buffer
			else
			{
				for (i = 0; i < messageLength; i++)
				{
					unsigned char v = buffer[i];
					fprintf(stderr, "%02x ", v);
				}
				fprintf(stderr, "\n");
			}
		}
#endif // DEBUG	
	}
	// Done!
	return msgReceived;
}

// ------------------------------------------------------------------------------
//   Write to UDP
// ------------------------------------------------------------------------------
int UDP_Port::write_message(const mavlink_message_t& message)
{
	char buf[300];

	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	// Write buffer to UDP port, locks port while writing
	int bytesWritten = _write_port(buf, len);
	if (bytesWritten < 0) {
		fprintf(stderr, "ERROR: Could not write, res = %d, errno = %d : %m\n", bytesWritten, errno);
	}

	return bytesWritten;
}


// ------------------------------------------------------------------------------
//   Open UDP Port
// ------------------------------------------------------------------------------
char UDP_Port::start()
{
	// --------------------------------------------------------------------------
	//   OPEN PORT
	// --------------------------------------------------------------------------

	/* Create socket */
	sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0)
	{
		fprintf(stderr, "ERROR: failed to create a socket\n");
		return -1;
	}

	memset(&bind_addr, 0, sizeof(sockaddr_in));
	bind_addr.sin_family = AF_INET;
	bind_addr.sin_addr.s_addr = inet_addr(target_ip); //INADDR_ANY;
	bind_addr.sin_port = htons(bind_port);
	
	/* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */
	if (-1 == bind(sock, (struct sockaddr*)&bind_addr, sizeof(struct sockaddr)))
	{
		fprintf(stderr, "ERROR in start: failed to bind to socket %i\n", bind_port);
		close(sock);
		return -1;
	}
	/* Attempt to make it non blocking */
#if (defined __QNX__) | (defined __QNXNTO__)
	if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
#else
	if (fcntl(sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
#endif
	{
		fprintf(stderr, "ERROR in start: failed to set the socket as nonblocking %s\n", strerror(errno));
		close(sock);
		return -1;
	}
	memset(&target_addr, 0, sizeof(sockaddr_in));
	target_addr.sin_family = AF_INET;
	target_addr.sin_addr.s_addr = inet_addr(target_ip);
	target_addr.sin_port = htons(target_port);

	// --------------------------------------------------------------------------
	//   CONNECTED!
	// --------------------------------------------------------------------------
	printf("Binded to %i\n", bind_port);
	printf("Listening to %s:%i\n", target_ip, target_port);
	lastStatus.packet_rx_drop_count = 0;

	is_open = true;

	printf("\n");

	return 0;

}


// ------------------------------------------------------------------------------
//   Close UDP Port
// ------------------------------------------------------------------------------
void UDP_Port::stop()
{
#ifdef DEBUG
	printf("CLOSE PORT\n");
#endif // DEBUG

	int result = close(sock);
	sock = -1;

	if ( result )
	{
		fprintf(stderr,"ERROR in stop: failed to close socket (%i)\n", result );
	}

	is_open = false;

#ifdef DEBUG
	printf("\n");
#endif // DEBUG
}

// ------------------------------------------------------------------------------
//   Read Port with Lock
// ------------------------------------------------------------------------------
int UDP_Port::_read_port(uint8_t& cp)
{
	int result = -1;	
	// Lock
	pthread_mutex_lock(&lock);
		
	if (buff_ptr < buff_len) {
		cp = buff[buff_ptr];
		buff_ptr++;
		result = 1;
	}
	else {
		result = recvfrom(sock, (void*)buff, BUFF_LEN, 0, (struct sockaddr*)&target_addr, &fromlen);
		if (result > 0) {
			buff_len = result;
			buff_ptr = 0;
			cp = buff[buff_ptr];
			buff_ptr++;
		}
	}

	// Unlock
	pthread_mutex_unlock(&lock);

	return result;
}


// ------------------------------------------------------------------------------
//   Write Port with Lock
// ------------------------------------------------------------------------------
int UDP_Port::_write_port(char* buf, unsigned len)
{
	// Lock
	pthread_mutex_lock(&lock);

	// Write packet via UDP link
	int bytesWritten = sendto(sock, buf, len, 0, (struct sockaddr*)&target_addr, sizeof(struct sockaddr_in));

	// Unlock
	pthread_mutex_unlock(&lock);

	return bytesWritten;
}


