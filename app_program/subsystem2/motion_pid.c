/*
 *      motion_pid.c
 *      Subsystem 2 -- PID speed controller and unit conversions.
 *
 *      The RP2040 has no FPU; float maths is done in software (~100 cycles
 *      per operation), which is far below 1% of a 10 ms period here.
 */

#include "motion_pid.h"
#include "subsystem2.h" /* S2_PERIOD_MS */
#include "s2_config.h"

static float32_t clampf(float32_t value, float32_t lim)
{
    if (value > lim)
    {
        return lim;
    }
    if (value < -lim)
    {
        return -lim;
    }
    return value;
}

void pid_init(pid_ctrl_t * p_pid, float32_t kff, float32_t gain_p,
              float32_t gain_i, float32_t gain_d, float32_t i_limit,
              float32_t out_limit)
{
    p_pid->kff       = kff;
    p_pid->kp        = gain_p;
    p_pid->ki        = gain_i;
    p_pid->kd        = gain_d;
    p_pid->i_limit   = i_limit;
    p_pid->out_limit = out_limit;
    pid_reset(p_pid);
}

void pid_reset(pid_ctrl_t * p_pid)
{
    p_pid->integral = 0.0f;
    p_pid->prev_err = 0.0f;
    p_pid->first    = true;
}

float32_t pid_update(pid_ctrl_t * p_pid, float32_t target, float32_t measured,
                     float32_t dt_s)
{
    float32_t err   = target - measured;
    float32_t deriv = 0.0f;
    float32_t out;

    if (dt_s <= 0.0f)
    {
        dt_s = (float32_t)S2_PERIOD_MS / (float32_t)S2_MS_PER_S;
    }

    /* Integral, clamped so it cannot wind up while the wheel is held */
    p_pid->integral =
        clampf(p_pid->integral + (p_pid->ki * err * dt_s), p_pid->i_limit);

    if (!p_pid->first)
    {
        deriv = (err - p_pid->prev_err) / dt_s;
    }
    p_pid->prev_err = err;
    p_pid->first    = false;

    out = (p_pid->kff * target) + (p_pid->kp * err) + p_pid->integral +
          (p_pid->kd * deriv);

    /* BARR-C 5.4: never pass on an INFINITY or NAN (e.g. from a bad
       measurement): restart the controller and output nothing.
       __builtin_isfinite() is what <math.h>'s isfinite() expands to;
       <math.h> itself clashes with the kernel's size_t typedef. */
    if (!__builtin_isfinite(out))
    {
        pid_reset(p_pid);
        out = 0.0f;
    }
    return clampf(out, p_pid->out_limit);
}

int32_t motion_ramp(int32_t cur, int32_t target, int32_t step)
{
    if (target > (cur + step))
    {
        return cur + step;
    }
    if (target < (cur - step))
    {
        return cur - step;
    }
    return target;
}

float32_t motion_ticks_to_mm(uint32_t ticks)
{
    return (float32_t)ticks * S2_MM_PER_TICK;
}

float32_t motion_turn_arc_mm(int32_t angle_deg)
{
    int32_t abs_deg = (angle_deg < 0) ? -angle_deg : angle_deg;

    /* Each wheel moves along a circle of diameter = track width */
    return ((S2_PI * S2_TRACK_MM) * (float32_t)abs_deg) / 360.0f;
}

/*** end of file ***/
