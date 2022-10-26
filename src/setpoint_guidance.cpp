/*
 * trajectories_common.cpp
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
 * Last Edit:  10/26/2022 (MM/DD/YYYY)
 *
 * Summary :
 * Contains all the automated trajectory guidance and related functionality.
 * This establishes framework of how to start/end automated guidance and safely
 * mannage setpoints.
 *
 */

#include <math.h>
#include <stdio.h>

#include "setpoint_guidance.hpp"
#include "tools.hpp"

 // preposessor macros
#ifndef unlikely
#define unlikely(x)	__builtin_expect (!!(x), 0)
#endif // !unlikely

#ifndef likely
#define likely(x)	__builtin_expect (!!(x), 1)
#endif // !likely

setpoint_guidance_t setpoint_guidance{};

inline double cubicPol(double xi, double xf, double xdoti, double xdotf, float tt_s, double dt)
{
    return (2 * xi - 2 * xf + tt_s * (xdotf + xdoti)) / (pow(tt_s, 3)) * pow(dt, 3) - \
        (3 * xi - 3 * xf + tt_s * (xdotf + 2 * xdoti)) / (pow(tt_s, 2)) * pow(dt, 2) + \
        xdoti * dt + xi;
    /* Cubic BVP:
    initial position and velocity is 0
    final position is xf and velocity is 0
    tt_s is total time in seconds
    */
    // double dX = (-2*xf/(tt_s*tt_s*tt_s)) * dt*dt*dt + 3*xf/(tt_s*tt_s) * dt*dt;
}


void cubic_guide_t::set(double new_xi, double new_xf, double new_xdoti, double new_xdotf, float new_tt)
{
    xi = new_xi;
    xf = new_xf;
    xdoti = new_xdoti;
    xdotf = new_xdotf;
    tt = new_tt;
    started = false;
    completed = false;
    return;
}

void cubic_guide_t::restart(void)
{
    started     = false;
    completed   = false;
}

bool cubic_guide_t::march(double &current_pos)
{
    if (completed)
    {
        printf("WARNING in march: trying to march cubic guide when completed\n");
        return true;
    }
    if (!started)
    {        
        time_us = get_time_usec();
        started = true;
    }


    if (get_dt_s(time_us) >= tt)
    {
        return true;
    }
    else
    {
        // use cubic function and time to find the current position:
        current_pos =
            cubicPol(xi, xf, xdoti, xdotf, tt, get_dt_s(time_us));
        return false;
    }
    return false;
}

char square_guide_t::march(double* xy_out)
{
    /* -------------- AUTONOMOUS XY TRACKING-----------//
    This algorithm performs simple square trajectory using cubic polynomials.
    The basic logic is as the following:
    1. Assume AUTONOMOUS has just been activated in air
    Assuming Hover state, execute the trajectory
        1.1 using initial time and position apply dv
        1.f stabilize to hover (might need to write a separate function for this)
    */
    if (!en_fl || finished_fl) return 0;


    bool tmp1, tmp2;

    // begin main sequence -- use waypoints for transition
    switch (waypoint)
    {
    case 0:
        x_guide.set(0, settings.X_length_m / 2.0, 0,
            0, settings.X_time_s / 2.0);
        y_guide.set(0, settings.Y_length_m / 2.0, 0,
            0, settings.Y_time_s / 2.0);
        waypoint++;
        xy_out[0] = 0;
        xy_out[1] = 0;
        return 0;

    case 1:
        // move to first corner
        tmp1 = y_guide.march(xy_out[0]); // always march both
        tmp2 = x_guide.march(xy_out[1]);
        if (tmp1 && tmp2)
        {
            waypoint++;
            start_time_us = get_time_usec();
        }
        return 0;

    case 2:
        if (get_dt_s(start_time_us) > settings.waypoint_delay_s)
        {
            x_guide.set(settings.X_length_m / 2.0, -settings.X_length_m / 2.0, 0,
                0, settings.X_time_s);
            waypoint++;
        }
        else
        {
            //keep current position:
            xy_out[0] = settings.X_length_m / 2.0;
            xy_out[1] = settings.Y_length_m / 2.0;
        }
        return 0;

    case 3:
        // move to corner 2:
        if (x_guide.march(xy_out[0]))
        {
            waypoint++;
            start_time_us = get_time_usec();
        }
        return 0;

    case 4:
        if (get_dt_s(start_time_us) > settings.waypoint_delay_s)
        {
            y_guide.set(settings.Y_length_m / 2.0, -settings.Y_length_m / 2.0, 0,
                0, settings.Y_time_s);
            waypoint++;
        }
        else
        {
            //keep current position:
            xy_out[0] = -settings.X_length_m / 2.0;
            xy_out[1] = settings.Y_length_m / 2.0;
        }
        return 0;

    case 5:
        // move to corner 3:
        if (y_guide.march(xy_out[1]))
        {
            waypoint++;
            start_time_us = get_time_usec();
        }
        return 0;

    case 6:
        if (get_dt_s(start_time_us) > settings.waypoint_delay_s)
        {
            x_guide.set(-settings.X_length_m / 2.0, settings.X_length_m / 2.0, 0,
                0, settings.X_time_s);
            waypoint++;
        }
        else
        {
            //keep current position:
            xy_out[0] = -settings.X_length_m / 2.0;
            xy_out[1] = -settings.Y_length_m / 2.0;
        }
        return 0;

    case 7:
        // move to corner 4:
        if (x_guide.march(xy_out[0]))
        {
            waypoint++;
            start_time_us = get_time_usec();
        }
        return 0;

    case 8:
        if (get_dt_s(start_time_us) > settings.waypoint_delay_s)
        {
            y_guide.set(-settings.Y_length_m / 2.0, settings.Y_length_m / 2.0, 0,
                0, settings.Y_time_s);
            waypoint++;
        }
        else
        {
            //keep current position:
            xy_out[0] = settings.X_length_m / 2.0;
            xy_out[1] = -settings.Y_length_m / 2.0;
        }
        return 0;

    case 9:
        // move back to corner 1:
        if (y_guide.march(xy_out[1]))
        {
            start_time_us = get_time_usec();
            waypoint++;
        }
        return 0;

    case 10:
        if (get_dt_s(start_time_us) > settings.waypoint_delay_s)
        {
            y_guide.set(xy_out[1], 0, 0,
                0, settings.Y_time_s / 2.0);
            x_guide.set(xy_out[0], 0, 0,
                0, settings.X_time_s / 2.0);
            waypoint++;
        }
        return 0;

    case 11:
        // move back to the origin:
        tmp1 = y_guide.march(xy_out[1]);
        tmp2 = x_guide.march(xy_out[0]);
        if (tmp1 && tmp2)
        {
            waypoint++;
            start_time_us = get_time_usec();
        }
        return 0;

    case 12:
        if (get_dt_s(start_time_us) > settings.waypoint_delay_s)
        {
            finished_fl = true;
            en_fl = false;
            return 1;
        }
        else
        {
            //keep current position:
            xy_out[0] = 0;
            xy_out[1] = 0;
        }
        return 0;
        //---------End of trajectory-------------//
    default:
        en_fl = false;
        fprintf(stderr,"ERROR in march: unknown waypoint\n");
        return -1;
    }

    return -1;
}

void square_guide_t::reset(void)
{
    en_fl = false;
    finished_fl = false;
    settings.X_length_m = 1.0;
    settings.X_time_s = 1.0;
    settings.Y_time_s = 1.0;
    settings.Y_length_m = 1.0;
    settings.waypoint_delay_s = 1.0;
    waypoint = 0;
    return;
}

char square_guide_t::start(square_settings_t square_settings_) //intended to allow running at each itter.
{
    if (finished_fl)
    {
        en_fl = false;
        return 0;
    }
    else
    {
        reset();
        settings = square_settings_;
        start_time_us = get_time_usec();
        en_fl = true;
        return 1; //started
    }

    return -1;
}

bool square_guide_t::is_complete(void) //intended to allow running at each itter.
{
    return finished_fl;
}

/*
void circ_guide_t::set(double new_T, double new_R, double new_tt, bool new_dir_yaw_cw,\
    double new_x_i, double new_y_i, double new_yaw_i)
{
    x_i = new_x_i;
    y_i = new_y_i;
    yaw_i = new_yaw_i;
    tt = new_tt;
    dir_yaw_cw = new_dir_yaw_cw;
    T = new_T;
    R = new_R;

    omega = 2.0 * M_PI / T; //angular velocity in a turn

    started = false;
    completed = false;
    return;
}

void circ_guide_t::restart(void)
{
    started = false;
    completed = false;
}

bool circ_guide_t::march(double& X, double& Y, double &Yaw)
{
    if (completed)
    {
        printf("\nWARNING in march: trying to march circ guide when completed");
        return true;
    }
    if (!started)
    {
        time_us = get_time_usec();
        started = true;
    }

    double tmp = get_dt_s(time_us);
    if (tmp >= tt)
    {
        return true;
    }
    else
    {
        X = R * sin(omega * tmp) + x_i;
        Y = R * cos(omega * tmp) + y_i;
        if (dir_yaw_cw)
        {
            Yaw = -omega * tmp + yaw_i;
        }
        else
        {
            Yaw = omega * tmp + yaw_i;
        }
        return false;
    }
    return false;
}
*/


//----------------------------------XY------------------------------------------//
/*
* Horizontal position setpoint management and guidance general functions.
* Intended to be independent of Z guidance. Each XY algorithm must have
* a way to start and stop itself and will use the below format. Note,
* only one XY guidance algorithm should be running at once to avoid overwrites
* of commands. Moreover, each time XY marching should be properly finished
* by setting st_XY = true.
*/
int setpoint_guidance_t::march_XY(void)
{

    if (!en_fl) return 0;

    //run XY guidance algorithms if enabled (only 1 at a time)
    if (settings.enable_square)
    {
        if (unlikely(square.march(XYZ) < 0))
        {
            fprintf(stderr, "ERROR in march_XY: failed to march square\n");
            return -1;
        }
    }

    return 0;
}

/*
int setpoint_guidance_t::march_circ(void)
{
    if (unlikely(!XY_initialized))
    {
        printf("\nERROR in march_circ: XY guidance not initialized");
        return -1;
    }
    if (unlikely(!en_XY))
    {
        printf("\nERROR in march_circ: XY guidance not enabled");
        return -1;
    }
    if (unlikely(!en_circ))
    {
        printf("\nWARNING in march_circ: trying to do_square while not enabled");
        return 0;
    }


    switch (waypoint)
    {
    case 0:
        if (get_dt_s(start_time_us) > XY_start_delay_s)
        {
            waypoint = 1;
            break;
        }
        else
        {
            //keep current position:
            setpoint.XY.x.value.set(X_initial);
            setpoint.XY.y.value.set(Y_initial);
            setpoint.ATT.z.value.set(Yaw_initial);
            break;
        }
    case 1:
        // move to corner edge of the circle (theta = 0):
        if (y_guide.march(*setpoint.XY.y.value.get_pt()))
        {
            waypoint = 2;
            start_time_us = get_time_usec();
            break;
        }
        else break;
    case 2:
        if (get_dt_s(start_time_us) > settings.waypoint_delay_s)
        {
            waypoint = 3;
            break;
        }
        else
        {
            //keep current position:
            setpoint.XY.x.value.set(X_initial);
            setpoint.XY.y.value.set(Y_initial + turn_radius);
            setpoint.ATT.z.value.set(Yaw_initial);
            break;
        }
    case 3:
        // move arround the circle untill time is up
        if (circ_guide.march(*setpoint.XY.x.value.get_pt(), *setpoint.XY.y.value.get_pt(), *setpoint.ATT.z.value.get_pt()))
        {
            waypoint = 4;
            start_time_us = get_time_usec();
            break;
        }
        else break;
    case 4:
        if (get_dt_s(start_time_us) > XY_start_delay_s)
        {
            y_guide.set(setpoint.XY.y.value.get(), Y_initial, 0,
                0, TWO_PI * turn_radius * turn_radius / turn_period);
            x_guide.set(setpoint.XY.x.value.get(), X_initial, 0,
                0, TWO_PI * turn_radius * turn_radius / turn_period);
            waypoint = 5;
            break;
        }
        else
        {
            //keep current position:
            // we don't know where we are (depends on tt)
            // assume we reached our setpoint and want to go back
            break;
        }
    case 5:
        // move back from edge of the circle to the center:
        if (y_guide.march(*setpoint.XY.y.value.get_pt()) && x_guide.march(*setpoint.XY.x.value.get_pt()))
        {
            waypoint = 6;
            start_time_us = get_time_usec();
            break;
        }
        else break;
    case 6:
        if (get_dt_s(start_time_us) > settings.waypoint_delay_s)
        {
            st_XY = true;
            circ_finished = true;
            en_circ = false;
            return 0;
        }
        else
        {
            //keep current position:
            setpoint.XY.x.value.set(X_initial);
            setpoint.XY.y.value.set(Y_initial);
            break;
        }
        //---------End of trajectory-------------//
    }
    return 0;
}

void setpoint_guidance_t::init_circ(void)
{
    en_circ = false;
    reset_circ();
    return;
}

void setpoint_guidance_t::reset_circ(void)
{
    turn_radius = settings.turn_radius;
    turn_period = settings.turn_period;
    turn_time_s = settings.turn_time_s;
    turn_dir    = settings.turn_dir_yaw_cw;
    return;
}

void setpoint_guidance_t::start_circ(void) //intended to allow running at each itter.
{
    if (!en_XY && !en_circ && !circ_finished)
    {
        en_XY = true; // allow XY guidance
        en_circ = true; // start circ guidance
        setpoint.XY.x.value.set(state_estimate.get_X()); //zero out the error
        setpoint.XY.y.value.set(state_estimate.get_Y()); //zero out the error
        setpoint.ATT.z.value.set(state_estimate.get_continuous_heading()); //zero out the error

        circ_guide.set(turn_period, turn_radius, turn_time_s, turn_dir, \
            X_initial, Y_initial, Yaw_initial);
        y_guide.set(Y_initial, turn_radius + Y_initial, 0,
            0, TWO_PI * turn_radius * turn_radius / turn_period);

        printf("\nStarting circular trajectory algorithm....");
    }//otherwise do nothing

    return;
}
*/

/*
* Use this function to restart takeoff in case of failure. Should be triggered using
* radio button or one-time command signal.
*/
/*
int setpoint_guidance_t::restart_circ(void) //intended as a trigger (do not run at each itter.)
{
    if (en_XY)
    {
        printf("\n Error in restart_circ: XY guidance is already tunning, must finish or stop the current algorithm");
        return -1;
    }
    reset_XY();
    circ_finished = false;
    start_circ();
    return 0;
}
*/

int setpoint_guidance_t::start(guidance_settings_t guidance_settings_, double X0, double Y0, double Z0)
{
    if (unlikely(en_fl))
    {
        fprintf(stderr,"ERROR in start: already started\n");
        return -1;
    }

    reset();

    settings = guidance_settings_;
    XYZ[0] = X0;
    XYZ[1] = Y0;
    XYZ[2] = Z0;

    if (unlikely(settings.enable_square && square.start(settings.square_settings) < 0))
    {
        fprintf(stderr,"ERROR in start: failed to start square guidance\n");
        return -1;
    }    

    en_fl = true;

    return 0;
}


int setpoint_guidance_t::march(void)
{
    if (!en_fl) return 0;

    if (settings.enable_square)
    {
        if (unlikely(square.march(XYZ) < 0))
        {
            fprintf(stderr, "ERROR in march: failed to march square\n");
            en_fl = false;
            return -1;
        }
    }

    return 0;
}


void setpoint_guidance_t::reset(void)
{
    en_fl = false;
    for (int i = 0; i < 3; i++) XYZ[i] = 0;
    square.reset();
    return;
}

void setpoint_guidance_t::get_XYZ(double* XYZ_out)
{
    for (int i = 0; i < 0; i++) XYZ_out[i] = XYZ[i];
    return;
}

bool setpoint_guidance_t::is_square_complete(void)
{
    return square.is_complete();
}