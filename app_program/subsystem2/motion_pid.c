/*
 *	motion_pid.c
 *	Subsystem 2 -- PID speed controller and unit conversions.
 *
 *	The RP2040 has no FPU; float maths is done in software (~100 cycles
 *	per operation), which is far below 1% of a 10 ms period here.
 */

#include "motion_pid.h"
#include "subsystem2.h"	/* S2_PERIOD_MS */
#include "s2_config.h"

LOCAL float clampf(float v, float lim)
{
	if (v > lim)  return lim;
	if (v < -lim) return -lim;
	return v;
}

EXPORT void pid_init(PID_CTRL *pid, float kff, float kp, float ki,
		     float kd, float i_limit, float out_limit)
{
	pid->kff       = kff;
	pid->kp        = kp;
	pid->ki        = ki;
	pid->kd        = kd;
	pid->i_limit   = i_limit;
	pid->out_limit = out_limit;
	pid_reset(pid);
}

EXPORT void pid_reset(PID_CTRL *pid)
{
	pid->integral = 0.0f;
	pid->prev_err = 0.0f;
	pid->first    = TRUE;
}

EXPORT float pid_update(PID_CTRL *pid, float target, float measured,
			float dt_s)
{
	float	err = target - measured;
	float	deriv = 0.0f;
	float	out;

	if (dt_s <= 0.0f) {
		dt_s = (float)S2_PERIOD_MS / 1000.0f;
	}

	/* Integral, clamped so it cannot wind up while the wheel is held */
	pid->integral = clampf(pid->integral + pid->ki * err * dt_s,
			       pid->i_limit);

	if (!pid->first) {
		deriv = (err - pid->prev_err) / dt_s;
	}
	pid->prev_err = err;
	pid->first    = FALSE;

	out = pid->kff * target + pid->kp * err + pid->integral
	    + pid->kd * deriv;
	return clampf(out, pid->out_limit);
}

EXPORT INT motion_ramp(INT cur, INT target, INT step)
{
	if (target > cur + step) return cur + step;
	if (target < cur - step) return cur - step;
	return target;
}

EXPORT float motion_ticks_to_mm(UW ticks)
{
	return (float)ticks * S2_MM_PER_TICK;
}

EXPORT float motion_turn_arc_mm(INT angle_deg)
{
	INT	a = (angle_deg < 0) ? -angle_deg : angle_deg;

	/* Each wheel moves along a circle of diameter = track width */
	return (S2_PI * S2_TRACK_MM) * (float)a / 360.0f;
}
