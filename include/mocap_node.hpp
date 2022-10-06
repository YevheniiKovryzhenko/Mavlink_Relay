/*
 * mocap_node.hpp
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
 * Functions to start and stop the optitrack mocap thread.
 */

#ifndef MOCAP_NODE_HPP
#define MOCAP_NODE_HPP
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h> // for socklen_t
#include <sys/socket.h> // for sockaddr
#include <mutex>
#include "optitrack.hpp"
#include "thread_gen.hpp"

uint64_t get_time_usec();

class mocap_data_t:public optitrack_message_t
{
public:
	uint64_t time_us_old;
	uint64_t time_us;
	double roll;
	double pitch;
	double yaw;
private:
};


class mocap_node_t
{
public:
	char reading_status;

	bool YUP2END;
	bool ZUP2NED;

	char start(std::string ip_addr);
	char stop(void);
	void togle_YUP2NED(bool in);
	void togle_ZUP2NED(bool in);
	char get_data(mocap_data_t& buff, int ID);

	void start_read_thread(void);

	mocap_node_t();
	~mocap_node_t();
private:
	uint64_t time_us;
	uint64_t time_us_old;
	pthread_mutex_t  lock;
	thread_gen_t thread;
	bool time_to_exit;

	const static int BUFF_LEN = 20000;
	char buff[BUFF_LEN];

	SOCKET dataSocket = 0;
	socklen_t ADDRLEN = sizeof(sockaddr);	
	std::vector<optitrack_message_t> incomingMessages;	

	
	char init(std::string ip_addr);	
	void read_messages(void);
	char march(void);
	void read_thread(void);
};

#endif  //MOCAP_NODE_HPP