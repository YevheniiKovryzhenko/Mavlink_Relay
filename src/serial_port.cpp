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
 * @file serial_port.cpp
 *
 * @brief Serial interface functions
 *
 * Functions for opening, closing, reading and writing via serial ports
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "serial_port.h"

//#define DEBUG
// ----------------------------------------------------------------------------------
//   Serial Port Manager Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Serial_Port::Serial_Port(const char *uart_name_ , int baudrate_)
{
	initialize_defaults();
	uart_name = uart_name_;
	baudrate  = baudrate_;
}

Serial_Port::
Serial_Port()
{
	initialize_defaults();
}

Serial_Port::
~Serial_Port()
{
	// destroy mutex
	pthread_mutex_destroy(&lock);
}

void
Serial_Port::
initialize_defaults()
{
	// Initialize attributes
	debug  = false;
	//fd     = -1;
	is_open = false;

	uart_name = (char*)"/dev/ttyUSB0";
	baudrate  = 57600;

	// Start mutex
	int result = pthread_mutex_init(&lock, NULL);
	if ( result != 0 )
	{
		printf("\n mutex init failed\n");
		throw 1;
	}
}


// ------------------------------------------------------------------------------
//   Read from Serial
// ------------------------------------------------------------------------------
int Serial_Port::read_message(mavlink_message_t &message)
{
	uint8_t          cp;
	mavlink_status_t status;
	uint8_t          msgReceived = false;

	// --------------------------------------------------------------------------
	//   READ FROM PORT
	// --------------------------------------------------------------------------

	// this function locks the port during read
	if (serial.available())
	{
		int result = _read_port(cp);


		// --------------------------------------------------------------------------
		//   PARSE MESSAGE
		// --------------------------------------------------------------------------
		if (result > 0)
		{
			// the parsing
			msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);

			// check for dropped packets
			if ((lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug)
			{
				printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
				unsigned char v = cp;
				fprintf(stderr, "%02x ", v);
			}
			lastStatus = status;
		}

		// Couldn't read from port
		else
		{
			fprintf(stderr, "ERROR: Could not read from %s\n", uart_name);
		}
#ifdef DEBUG
		// --------------------------------------------------------------------------
		//   DEBUGGING REPORTS
		// --------------------------------------------------------------------------
		if (msgReceived)
		{
			// Report info
			printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);

			fprintf(stderr, "Received serial data: ");
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
//   Write to Serial
// ------------------------------------------------------------------------------
int
Serial_Port::
write_message(const mavlink_message_t &message)
{
	char buf[300];

	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	// Write buffer to serial port, locks port while writing
	int bytesWritten = _write_port(buf,len);

	return bytesWritten;
}


// ------------------------------------------------------------------------------
//   Open Serial Port
// ------------------------------------------------------------------------------
/**
 * throws EXIT_FAILURE if could not open the port
 */
void Serial_Port::start()
{

	// --------------------------------------------------------------------------
	//   OPEN PORT
	// --------------------------------------------------------------------------
#ifdef DEBUG
	printf("OPEN PORT\n");
#endif // DEBUG

	// --------------------------------------------------------------------------
	//   SETUP PORT
	// --------------------------------------------------------------------------
	if (serial.openDevice(uart_name, baudrate) < 0)
	{
		printf("failure, could not configure port.\n");
		throw EXIT_FAILURE;
	}
	serial.flushReceiver();

	// --------------------------------------------------------------------------
	//   CONNECTED!
	// --------------------------------------------------------------------------
	printf("Connected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
	lastStatus.packet_rx_drop_count = 0;

	is_open = true;

	printf("\n");

	return;

}


// ------------------------------------------------------------------------------
//   Close Serial Port
// ------------------------------------------------------------------------------
void Serial_Port::stop()
{
#ifdef DEBUG
	printf("CLOSE PORT\n");
#endif // DEBUG	

	//int result = close(fd);
	if (is_open)
	{
		if (serial.isDeviceOpen()) serial.closeDevice();
		is_open = false;
	}
	else
	{
		return;
	}

}

// ------------------------------------------------------------------------------
//   Read Port with Lock
// ------------------------------------------------------------------------------
int Serial_Port::_read_port(uint8_t &cp)
{
	// Lock
	pthread_mutex_lock(&lock);

	//int result = read(fd, &cp, 1);
	
	int result = serial.readBytes(&cp, 1);
	// Unlock
	pthread_mutex_unlock(&lock);

	return result;
}


// ------------------------------------------------------------------------------
//   Write Port with Lock
// ------------------------------------------------------------------------------
int Serial_Port::_write_port(char *buf, unsigned len)
{

	// Lock
	pthread_mutex_lock(&lock);

	// Write packet via serial link
	//const int bytesWritten = static_cast<int>(write(fd, buf, len));
	serial.writeBytes(buf, len);
	serial.sync();
	// Wait until all data has been written
	//tcdrain(fd);

	// Unlock
	pthread_mutex_unlock(&lock);


	return len;
}


