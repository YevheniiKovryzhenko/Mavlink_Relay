#ifndef MOCAP_NODE_HPP
#define MOCAP_NODE_HPP
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h> // for socklen_t
#include <sys/socket.h> // for sockaddr
#include "optitrack.hpp"

class mocap_data_t:public optitrack_message_t
{
public:

private:
};


class mocap_node_t
{
public:
	char start(std::string interface);
	char stop(void);
	char march(void);

	mocap_node_t();
	~mocap_node_t();
private:
	bool initialized = false;
	pthread_mutex_t  lock;

	const static int BUFF_LEN = 20000;
	char buff[BUFF_LEN];

	SOCKET dataSocket = 0;
	socklen_t ADDRLEN = sizeof(sockaddr);	
	std::vector<optitrack_message_t> incomingMessages;

};

#endif  //MOCAP_NODE_HPP