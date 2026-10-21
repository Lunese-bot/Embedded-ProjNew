/*
 *	motion_pid.h
 *	Subsystem 2 -- PID speed controller and unit conversions.
 *
 *	Pure functions with no hardware or kernel calls, so they can be
 *	unit-tested on the target (subsystem2_test.c) and reused per wheel.
 */

#ifndef MOTION_PID_H
#define MOTION_PID_H

#include "../app.h"

typedef struct {
	float	kff;		/* feed-forward, output per unit target */
	float	kp;
	float	ki;		/* per second */
	float	kd;		/* seconds */
	float	i_limit;	/* |integral term| clamp (anti-windup) */
	float	out_limit;	/* |output| clamp */
	float	integral;	/* state: integral term, already * ki */
	float	prev_err;	/* state: previous error, for D */
	BOOL	first;		/* state: no previous error yet */
} PID_CTRL;

IMPORT void	pid_init(PID_CTRL *pid, float kff, float kp, float ki,
			 float kd, float i_limit, float out_limit);
IMPORT void	pid_reset(PID_CTRL *pid);

/* One controller step.  dt_s = time since the previous step, seconds.
   Returns the output, clamped to +-out_limit. */
IMPORT float	pid_update(PID_CTRL *pid, float target, float measured,
			   float dt_s);

/* Move cur towards target by at most step (set-point ramp) */
IMPORT INT	motion_ramp(INT cur, INT target, INT step);

/* Encoder ticks -> millimetres of wheel travel */
IMPORT float	motion_ticks_to_mm(UW ticks);

/* Arc each wheel travels when pivoting on the spot by angle_deg */
IMPORT float	motion_turn_arc_mm(INT angle_deg);

#endif /* MOTION_PID_H */
