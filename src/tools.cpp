/*
 * tools.cpp
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
 * Last Edit:  10/07/2022 (MM/DD/YYYY)
 *
 */

#include "tools.hpp"


#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdbool.h> /* required for the definition of bool in C99 */
#endif


// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------

/* QNX timer version */
#if (defined __QNX__) | (defined __QNXNTO__)
uint64_t get_time_usec(void)
{

	struct timespec time;

	clock_gettime(CLOCK_REALTIME, &time);

	return (uint64_t)time.tv_sec * 1000000 + time.tv_nsec / 1000;
}
#else
uint64_t get_time_usec(void)
{

	struct timeval tv;

	gettimeofday(&tv, NULL);

	return ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
}
#endif

uint64_t get_dt_us(uint64_t last_time)
{
	return get_time_usec() - last_time;
}

double get_dt_s(uint64_t last_time)
{
	return (double)get_dt_us(last_time) / 1000000.0;
}