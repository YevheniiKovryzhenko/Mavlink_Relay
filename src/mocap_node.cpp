/*
 * mocap_node.cpp
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
#include <sys/time.h>

#include "optitrack_channels.h"
#include "optitrack.hpp"
#include "thread_defs.hpp"

//#define DEBUG

static uint64_t __get_dt_us(uint64_t last_time)
{
    return get_time_usec() - last_time;
}

void __copy_data(mocap_data_t& buff_out, mocap_data_t& buff_in)
{
    buff_out.id = buff_in.id;
    buff_out.trackingValid = buff_in.trackingValid;
    buff_out.qw = buff_in.qw;
    buff_out.qx = buff_in.qx;
    buff_out.qy = buff_in.qy;
    buff_out.qz = buff_in.qz;
    buff_out.x = buff_in.x;
    buff_out.y = buff_in.y;
    buff_out.z = buff_in.z;
    buff_out.roll = buff_in.roll;
    buff_out.pitch = buff_in.pitch;
    buff_out.yaw = buff_in.yaw;
    return;
}
void __copy_data(optitrack_message_t& buff_out, mocap_data_t& buff_in)
{
    buff_out.id = buff_in.id;
    buff_out.trackingValid = buff_in.trackingValid;
    buff_out.qw = buff_in.qw;
    buff_out.qx = buff_in.qx;
    buff_out.qy = buff_in.qy;
    buff_out.qz = buff_in.qz;
    buff_out.x = buff_in.x;
    buff_out.y = buff_in.y;
    buff_out.z = buff_in.z;
    return;
}
void __copy_data(optitrack_message_t& buff_out, optitrack_message_t& buff_in)
{
    buff_out.id = buff_in.id;
    buff_out.trackingValid = buff_in.trackingValid;
    buff_out.qw = buff_in.qw;
    buff_out.qx = buff_in.qx;
    buff_out.qy = buff_in.qy;
    buff_out.qz = buff_in.qz;
    buff_out.x = buff_in.x;
    buff_out.y = buff_in.y;
    buff_out.z = buff_in.z;
    return;
}
void __copy_data(mocap_data_t& buff_out, optitrack_message_t& buff_in)
{
    buff_out.id = buff_in.id;
    buff_out.trackingValid = buff_in.trackingValid;
    buff_out.qw = buff_in.qw;
    buff_out.qx = buff_in.qx;
    buff_out.qy = buff_in.qy;
    buff_out.qz = buff_in.qz;
    buff_out.x = buff_in.x;
    buff_out.y = buff_in.y;
    buff_out.z = buff_in.z;
    toEulerAngle(buff_in, buff_out.roll, buff_out.pitch, buff_out.yaw);
    return;
}

void __rotate_YUP2NED(optitrack_message_t& buff_in)
{
    optitrack_message_t tmp = buff_in;

    //buff_in.x = tmp.x;
    buff_in.y = tmp.z;
    buff_in.z = -tmp.y;

    //buff_in.qw = tmp.qw;
    //buff_in.qx = tmp.qx;
    buff_in.qy = tmp.qz;
    buff_in.qz = -tmp.qy;
    return;
}

void __rotate_ZUP2NED(optitrack_message_t& buff_in)
{
    optitrack_message_t tmp = buff_in;

    //buff_in.x = tmp.x;
    buff_in.y = -tmp.y;
    buff_in.z = -tmp.z;

    //buff_in.qw = tmp.qw;
    //buff_in.qx = tmp.qx;
    buff_in.qy = -tmp.qy;
    buff_in.qz = -tmp.qz;
    return;
}

// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t get_time_usec()
{
    static struct timeval _time_stamp;
    gettimeofday(&_time_stamp, NULL);
    return _time_stamp.tv_sec * 1000000 + _time_stamp.tv_usec;
}

/* check if MOCAP data is new */
bool is_mocap_data_same(mocap_data_t& data_new, mocap_data_t& data_old)
{
    if (data_new.pitch != data_old.pitch) return false;
    if (data_new.roll != data_old.roll) return false;
    if (data_new.yaw != data_old.yaw) return false;
    if (data_new.x != data_old.x) return false;
    if (data_new.y != data_old.y) return false;
    if (data_new.z != data_old.z) return false;
    return true;
}


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void* start_mocap_read_thread(void* args)
{
    // takes an autopilot object argument
    mocap_node_t* autopilot_interface = (mocap_node_t*)args;

    // run the object's read thread
    autopilot_interface->start_read_thread();

    // done!
    return NULL;
}

// ------------------------------------------------------------------------------
//   Start MOCAP Thread
// ------------------------------------------------------------------------------
char mocap_node_t::start(std::string ip_addr)
{
    if (init(ip_addr) < 0)
    {
        printf("ERROR in start: failed to initialize ip\n");
        stop();
        return -1;
    }
    if (thread.init(MOCAP_THREAD_PRI, MOCAP_THREAD_TYPE))
    {
        printf("ERROR in start: failed to initialize thread\n");
        stop();
        return -1;
    }
    if (thread.start(&start_mocap_read_thread, this))
    {
        printf("ERROR in start: failed to start thread\n");
        stop();
        return -1;
    }
    
    return 0;
}

// ------------------------------------------------------------------------------
//   Stop MOCAP Thread
// ------------------------------------------------------------------------------
char mocap_node_t::stop(void)
{
#ifdef DEBUG
    printf("Terminating mocap node thread\n");
#endif // DEBUG

    // signal exit
    time_to_exit = true;

    // wait for exit
    thread.stop(MOCAP_THREAD_TOUT);
    //pthread_join(read_tid, NULL);

    // destroy mutex
    pthread_mutex_destroy(&lock);
#ifdef DEBUG
    printf("Done terminating mocap node thread\n");
#endif // DEBUG
    return 0;
}

// ------------------------------------------------------------------------------
//   Start Mocap Read Thread
// ------------------------------------------------------------------------------
void mocap_node_t::start_read_thread()
{
    if (reading_status != 0)
    {
        fprintf(stderr, "ERROR in start_read_thread: mocap reading thread is already running\n");
        return;
    }
    else
    {
        read_thread();
        return;
    }

}

// ------------------------------------------------------------------------------
//   Mocap Read Thread
// ------------------------------------------------------------------------------
void mocap_node_t::read_thread()
{
    reading_status = true;

    while (!time_to_exit)
    {
        read_messages();         
        usleep(1.0E6 / MOCAP_THREAD_HZ); // Read batches at 100Hz
    }

    reading_status = false;

    return;
}

// ------------------------------------------------------------------------------
//   Initialize MOCAP Node
// ------------------------------------------------------------------------------
char mocap_node_t::init(std::string ip_addr)
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
            "Successfully created socket for ip address "
            "%s:%d\n",
            ip_addr.c_str(), PORT_DATA);
    }
    time_to_exit = false;
    return 0;
}

// ------------------------------------------------------------------------------
//   March MOCAP Node
// ------------------------------------------------------------------------------
char mocap_node_t::march(void)
{
    // Block until we receive a datagram from the network
    sockaddr_in incomingAddress;
    recvfrom(dataSocket, buff, BUFF_LEN, 0,
        (sockaddr*)&incomingAddress, &ADDRLEN);

    // Lock
    pthread_mutex_lock(&lock);    
    incomingMessages =
        parse_optitrack_packet_into_messages(buff, BUFF_LEN);
    time_us_old = time_us;
    time_us = get_time_usec();
    // Unlock
    pthread_mutex_unlock(&lock);
    return 0;
}

// ------------------------------------------------------------------------------
//   Reads most recent data
// ------------------------------------------------------------------------------
char mocap_node_t::get_data(mocap_data_t& buff, int ID)
{
    // Lock
    pthread_mutex_lock(&lock);

    size_t tmp = incomingMessages.size();
    if (tmp < 1)
    {
#ifdef DEBUG
        printf("WARNING in get_data: have not received new data (vector is empty)\n");
#endif // DEBUG       

        // Unlock
        pthread_mutex_unlock(&lock);
        return 0;
    }
    for (auto& msg : incomingMessages)
    {
        if (msg.id == ID)
        {
            optitrack_message_t tmp_msg = msg;
            if (YUP2END) __rotate_YUP2NED(tmp_msg);
            else if (ZUP2NED) __rotate_ZUP2NED(tmp_msg);
            __copy_data(buff, tmp_msg);
            buff.time_us_old = time_us_old;
            buff.time_us = time_us;

            // Unlock
            pthread_mutex_unlock(&lock);
            return 0;
        }
    }
    // Unlock
    pthread_mutex_unlock(&lock);
    printf("WARNING in get_data: no rigid body matching ID=%i\n", ID);
    return -1;
}


/* toggle coordinate frame rotation */
void mocap_node_t::togle_YUP2NED(bool in)
{
    if (in)
    {
        ZUP2NED = false;
        YUP2END = true;
    }
    else
    {
        YUP2END = false;
    }
    return;
}
void mocap_node_t::togle_ZUP2NED(bool in)
{
    if (in)
    {
        ZUP2NED = true;
        YUP2END = false;
    }
    else
    {
        ZUP2NED = false;
    }
    return;
}


// ------------------------------------------------------------------------------
//   Read MOCAP Messages
// ------------------------------------------------------------------------------
void mocap_node_t::read_messages()
{
    march();
    return;
}


mocap_node_t::mocap_node_t()
{
    dataSocket = 0;
    if (pthread_mutex_init(&lock, NULL) != 0)
    {
        fprintf(stderr, "ERROR in start: failed to initialize mutex\n");
        throw 1;
    }
}

mocap_node_t::~mocap_node_t()
{
    stop();
}