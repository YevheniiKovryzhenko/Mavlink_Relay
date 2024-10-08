/*
 * serial_port.cpp
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
 * Last Edit:  06/03/2022 (MM/DD/YYYY)
 *
 * Functions for opening, closing, reading and writing via serial ports.
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "serial_port.hpp"

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

Serial_Port::Serial_Port()
{
	initialize_defaults();
}

Serial_Port::~Serial_Port()
{
	// destroy mutex
	pthread_mutex_destroy(&lock);
}

void Serial_Port::initialize_defaults()
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
int Serial_Port::bytes_available(void)
{
	return serial.available();
}
char Serial_Port::read_message(mavlink_message_t &message, mavlink_channel_t mavlink_channel_)
{	
	uint8_t          msgReceived = false;

	// --------------------------------------------------------------------------
	//   READ FROM PORT
	// --------------------------------------------------------------------------

	// this function locks the port during read
	while (!msgReceived && serial.available() > 0)
	{
		uint8_t          cp;
		mavlink_status_t status;
		int result = _read_port(cp);


		// --------------------------------------------------------------------------
		//   PARSE MESSAGE
		// --------------------------------------------------------------------------
		if (result > 0)
		{
			// the parsing
			msgReceived = mavlink_parse_char(mavlink_channel_, cp, &message, &status);

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
int Serial_Port::write_message(const mavlink_message_t &message)
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
char Serial_Port::start()
{

	// --------------------------------------------------------------------------
	//   OPEN PORT
	// --------------------------------------------------------------------------
	printf("Configuring Serial connection...\t");

	// --------------------------------------------------------------------------
	//   SETUP PORT
	// --------------------------------------------------------------------------
	if (serial.openDevice(uart_name, baudrate, \
		SERIAL_DATABITS_8, SERIAL_PARITY_NONE, SERIAL_STOPBITS_1) < 0)
	{
		fprintf(stderr, "\nERROR in start: failed to configure port %s with baudrate %i.\n", uart_name, baudrate);
		return -1;
	}
	serial.flushReceiver();
	printf("\tOK!\n");
	// --------------------------------------------------------------------------
	//   CONNECTED!
	// --------------------------------------------------------------------------
	printf("Connected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
	lastStatus.packet_rx_drop_count = 0;

	is_open = true;

	return 0;

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
	serial.writeBytes(buf, len);
	serial.sync();

	// Unlock
	pthread_mutex_unlock(&lock);


	return len;
}


