/*
 * trajectories_common.hpp
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
 * Last Edit:  10/26/2020 (MM/DD/YYYY)
 *
 * Summary :
 * Contains all the automated trajectory guidance and related functionality
 *
 */

#ifndef TRAJECTORIES_COMMON_HPP
#define TRAJECTORIES_COMMON_HPP

#include <stdint.h>

class cubic_guide_t
{
private:
	bool started;
	bool completed;
	double xi, xf, xdoti, xdotf;
	uint64_t time_us;
	float tt;
public:
	void set(double new_xi, double new_xf, double new_xdoti, double new_xdotf, float new_tt);
	bool march(double& current_pos);
	void restart(void);
};

/*
class circ_guide_t
{
private:
	bool started;
	bool completed;
	bool dir_yaw_cw;
	double T, R, omega, tt, x_i, y_i, yaw_i;
	uint64_t time_us;
public:
	void set(double T, double R, double new_tt, bool dir_yaw_cw, double x_i, double y_i, double yaw_i);
	bool march(double& X, double& Y, double& Yaw);
	void restart(void);
};
*/

typedef struct square_settings_t
{
	double X_length_m;
	double X_time_s;
	double Y_time_s;
	double Y_length_m;
	double waypoint_delay_s;
}square_settings_t;

typedef struct guidance_settings_t
{
	bool enable_square;
	square_settings_t square_settings;
}guidance_settings_t;


class square_guide_t
{
	/* -------------- AUTONOMOUS XY TRACKING-----------//
	This algorithm performs simple square trajectory using cubic polynomials.
	The basic logic is as the following:
	1. Assume AUTONOMOUS has just been activated in air
	Assuming Hover state, execute the trajectory
		1.1 using initial time and position apply dv
		1.f stabilize to hover (might need to write a separate funtion for this)
	*/
	bool en_fl;
	bool finished_fl;
	square_settings_t settings;
	uint64_t start_time_us;

	cubic_guide_t x_guide;
	cubic_guide_t y_guide;

	uint32_t waypoint = 0;
	
public:
	char march(double* xy_out);
	void reset(void);
	char start(square_settings_t square_settings_);

	bool is_complete(void);
};

class setpoint_guidance_t
{
private:
	bool en_fl;								// initialization flag
	double XYZ[3];

	int march_XY(void);

	guidance_settings_t settings;
	square_guide_t square;
public:

	/* Initialization, call at the start of the program */
	int start(guidance_settings_t guidance_settings_, double X0, double Y0, double Z0);

	/* 
	Marching the auto guidance algorithms 
	Call this function at each itteration before feedback.
	Start deactivated, use flags to start a certain algorithm.
	Each algorithm dectivates once finished
	*/
	int march(void);

	/*
	* External reset of all the autonoumous functions, 
	* can be use to stop any autonous algorithm and/or
	* in emergency.
	*/
	void reset(void);

	void get_XYZ(double* XYZ_out);

	bool is_square_complete(void);
};
extern setpoint_guidance_t setpoint_guidance;


#endif /* TRAJECTORIES_COMMON_H */