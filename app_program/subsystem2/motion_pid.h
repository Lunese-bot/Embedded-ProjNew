/*
 *      motion_pid.h
 *      Subsystem 2 -- PID speed controller and unit conversions.
 *
 *      Pure functions with no hardware or kernel calls, so they can be
 *      unit-tested on the target (subsystem2_test.c) and reused per wheel.
 */

#ifndef MOTION_PID_H
#define MOTION_PID_H

#include "../app.h"

typedef struct
{
    float32_t kff; /* feed-forward, output per unit target */
    float32_t kp;
    float32_t ki;        /* per second */
    float32_t kd;        /* seconds */
    float32_t i_limit;   /* |integral term| clamp (anti-windup) */
    float32_t out_limit; /* |output| clamp */
    float32_t integral;  /* state: integral term, already * ki */
    float32_t prev_err;  /* state: previous error, for D */
    bool      first;     /* state: no previous error yet */
} pid_ctrl_t;

extern void pid_init(pid_ctrl_t * p_pid, float32_t kff, float32_t gain_p,
                     float32_t gain_i, float32_t gain_d, float32_t i_limit,
                     float32_t out_limit);
extern void pid_reset(pid_ctrl_t * p_pid);

/* One controller step.  dt_s = time since the previous step, seconds.
   Returns the output, clamped to +-out_limit. */
extern float32_t pid_update(pid_ctrl_t * p_pid, float32_t target,
                            float32_t measured, float32_t dt_s);

/* Move cur towards target by at most step (set-point ramp) */
extern int32_t motion_ramp(int32_t cur, int32_t target, int32_t step);

/* Encoder ticks -> millimetres of wheel travel */
extern float32_t motion_ticks_to_mm(uint32_t ticks);

/* Arc each wheel travels when pivoting on the spot by angle_deg */
extern float32_t motion_turn_arc_mm(int32_t angle_deg);

#endif /* MOTION_PID_H */

/*** end of file ***/
