/*
 * thread_gen.cpp
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
 * Last Edit:  09/27/2020 (MM/DD/YYYY)
 *
 * Summary :
 * 
 *
 */
#include "thread_gen.hpp"
#include <stdio.h>
#include <sys/time.h>
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h>  // UNIX standard function definitions
#include <cstdint>
#include <iostream>

int thread_gen_t::init(int PRIORITY, thread_policy POLICY)
{    
    if (PRIORITY > 99 || PRIORITY < 0)
    {
        printf("ERROR in init: PRIORITY must be between 0 and 99\n");
        return -1;
    }
    priority = PRIORITY;
    policy = POLICY;

    initialized = true;
    started = false;

    return 0;
}

int thread_gen_t::start(void* (*func)(void*))
{
    if (!initialized)
    {
        printf("ERROR in start: thread not initialized\n");
        return -1;
    }
    if (gen_pthread_create(&thread, func, NULL,
        policy, priority) == -1) {
        printf("ERROR in start: failed to start thread\n");
        return -1;
    }
    usleep(50000);
    started = true;
    return 0;
}
int thread_gen_t::start(void* (*func)(void*), void* __arg)
{
	if (!initialized)
	{
		printf("ERROR in start: thread not initialized\n");
		return -1;
	}
	if (gen_pthread_create(&thread, func, __arg,
		policy, priority) == -1) {
		printf("ERROR in start: failed to start thread\n");
		return -1;
	}
	usleep(50000);
	started = true;
	return 0;
}
int thread_gen_t::stop(float TOUT)
{
    int ret = 0;
    if (started) {
        // wait for the thread to exit
        ret = gen_pthread_timed_join(thread, NULL, TOUT);
        if (ret == 1) fprintf(stderr, "WARNING in stop: exit timeout\n");
        else if (ret == -1) fprintf(stderr, "ERROR in stop: failed to join thread\n");
    }
    started = false;
    return ret;
}

bool thread_gen_t::is_started(void) const
{
    return started;
}


int thread_gen_t::gen_pthread_timed_join(pthread_t thread, void** retval, float timeout_sec) {
	struct timespec thread_timeout;
	clock_gettime(CLOCK_REALTIME, &thread_timeout);
	uint64_t timeout_ns = timeout_sec * 1000000000;
	thread_timeout.tv_sec += timeout_ns / 1000000000;
	thread_timeout.tv_nsec += timeout_ns % 1000000000;
	if (thread_timeout.tv_nsec > 1000000000) {
		thread_timeout.tv_sec += 1;
		thread_timeout.tv_nsec -= 1000000000;
	}

	errno = pthread_timedjoin_np(thread, retval, &thread_timeout);
	// if no error, return 0
	if (errno == 0) return 0;
	// in case of timeout, return 1
	if (errno == ETIMEDOUT) return 1;
	// otherwise print error message
	perror("ERROR: in gen_pthread_timed_join: ");
	return -1;
}


int thread_gen_t::gen_pthread_create(pthread_t* thread, void* (*func)(void*), void* arg, int policy, int priority)
{
	pthread_attr_t pthread_attr;
	struct sched_param pthread_param;

	// sanity checks
	if (policy != SCHED_FIFO && policy != SCHED_RR && policy != SCHED_OTHER) {
		fprintf(stderr, "ERROR in rc_pthread_create: policy must be SCHED_FIFO, SCHED_RR, or SCHED_OTHER\n");
		return -1;
	}
	if (thread == NULL || func == NULL) {
		fprintf(stderr, "ERROR in rc_pthread_create: received NULL pointer\n");
		return -1;
	}

	// necessary attribute initialization
	pthread_attr_init(&pthread_attr);

	// if user is requesting anything other than inherited policy 0 and
	// priority 0, make sure we have permission to do explicit scheduling
	if (priority != 0 || policy != SCHED_OTHER) {
		// print warning if no permissions
		errno = pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
		if (errno) {
			perror("ERROR: pthread_attr_setinheritsched: ");
			return -1;
		}
	}

	// set scheduling policy
	const int max_pri = sched_get_priority_max(policy);
	const int min_pri = sched_get_priority_min(policy);
	if (priority > max_pri || priority < min_pri) {
		fprintf(stderr, "ERROR in rc_pthread_create, priority must be between %d & %d\n", min_pri, max_pri);
		return -1;
	}

	// set policy to attributes
	errno = pthread_attr_setschedpolicy(&pthread_attr, policy);
	if (errno) {
		perror("ERROR: pthread_attr_setschedpolicy");
		return -1;
	}

	// set priority in attributes
	pthread_param.sched_priority = priority;
	errno = pthread_attr_setschedparam(&pthread_attr, &pthread_param);
	if (errno) {
		perror("ERROR: pthread_attr_setschedparam");
		return -1;
	}

	// create the thread
	errno = pthread_create(thread, &pthread_attr, func, arg);
	if (errno == EPERM) {
		fprintf(stderr, "WARNING: in gen_pthread_create, insufficient privileges to set scheduling policy\n");
		fprintf(stderr, "starting thread with inherited scheduling policy instead\n");
		fprintf(stderr, "to silence this warning, call with policy=SCHED_OTHER & priority=0\n");
		policy = SCHED_OTHER;
		priority = 0;
		errno = pthread_create(thread, NULL, func, arg);
		if (errno != 0) {
			perror("ERROR: in gen_pthread_create ");
			pthread_attr_destroy(&pthread_attr);
			return -1;
		}
	}
	else if (errno) {
		perror("ERROR: in gen_pthread_create: ");
		pthread_attr_destroy(&pthread_attr);
		return -1;
	}

	// check if it worked
	int policy_new;
	struct sched_param params_new;
	// get parameters from pthread_t
	errno = pthread_getschedparam(*thread, &policy_new, &params_new);
	if (errno) {
		perror("ERROR: pthread_getschedparam");
		return -1;
	}

	// print warnings if there is a mismatch
	if (policy_new != policy) {
		fprintf(stderr, "WARNING in gen_pthread_create, policy actually got set to %d, expected %d\n", \
			policy_new, policy);
	}
	if (params_new.sched_priority != priority) {
		fprintf(stderr, "WARNING in gen_pthread_create, priority actually got set to %d, expected %d\n", \
			params_new.sched_priority, priority);
	}

	// destroy the attributes object
	if (pthread_attr_destroy(&pthread_attr)) {
		fprintf(stderr, "WARNING, failed to destroy pthread_attr\n");
	}

	return 0;
}