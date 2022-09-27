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

	bool YUP2END = true;

	char start(std::string ip_addr);
	char stop(void);
	char get_data(mocap_data_t& buff, int ID);

	void start_read_thread(void);

	mocap_node_t();
	~mocap_node_t();
private:
	uint64_t time_us;
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