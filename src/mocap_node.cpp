#include "mocap_node.hpp"

#include <getopt.h>
#include <iostream>
#include <limits>
#include <string.h>

#include <vector>

#include <errno.h>  //Errors for read/write
#include <stdio.h>
#include <unistd.h>  // read / write / sleep

#include <cstdlib>
#include <cstring>

#include <netinet/in.h>
#include <sys/socket.h>
#include <cmath>
#include <fstream>  // std::ifstream
#include <iostream>

// Below for PRId64
#include <inttypes.h>
#include <cinttypes>

#include <stdint.h>

#include "optitrack_channels.h"
#include "optitrack.hpp"

char mocap_node_t::start(std::string ip_addr)
{
    dataSocket = 0;

    // Start mutex
    if (pthread_mutex_init(&lock, NULL) != 0)
    {
        fprintf(stderr,"ERROR in start: failed to initialize mutex\n");
        return -1;
    }

    if (ip_addr.length() == 0) {
        ip_addr = guess_optitrack_network_interface();
    }
    // If there's still no interface, we have a problem
    if (ip_addr.length() == 0) {
        fprintf(stderr,
            "[optitrack_driver] error could not determine network ip address for "
            "receiving multicast packets.\n");
        return -1;
    }

    dataSocket = create_optitrack_data_socket(ip_addr, PORT_DATA);

    if (dataSocket == -1) {
        fprintf(stderr,
            "ERROR in open: error failed to create socket for ip address "
            "%s:%d\n",
            ip_addr.c_str(), PORT_DATA);
        return -1;
    }
    else {
        fprintf(stderr,
            "ERROR in open: successfully created socket for ip address "
            "%s:%d\n",
            ip_addr.c_str(), PORT_DATA);
    }
    return 0;
}

char mocap_node_t::stop(void)
{
    return 0;
}

char mocap_node_t::march(void)
{
    // Lock
    pthread_mutex_lock(&lock);

    // Block until we receive a datagram from the network
    sockaddr_in incomingAddress;
    recvfrom(dataSocket, buff, BUFF_LEN, 0,
        (sockaddr*)&incomingAddress, &ADDRLEN);
    incomingMessages =
        parse_optitrack_packet_into_messages(buff, BUFF_LEN);

    // Unlock
    pthread_mutex_unlock(&lock);
    return 0;
}

mocap_node_t::
mocap_node_t()
{
    dataSocket = 0;
    if (pthread_mutex_init(&lock, NULL) != 0)
    {
        fprintf(stderr, "ERROR in start: failed to initialize mutex\n");
        throw 1;
    }
}

mocap_node_t::
~mocap_node_t()
{
    // destroy mutex
    pthread_mutex_destroy(&lock);
}
