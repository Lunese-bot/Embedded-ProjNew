/*
 *      subsystem2_test.c
 *      On-target tests for subsystem 2.  Built only with: make TEST=1
 *      Record results in testing/subsystem2/TEST_LOG.md.
 *
 *      These tests never make the wheels turn: they check the maths and
 *      the kernel interfaces, and only send STOP / SAFE_STOP / RESET.
 *      Driving tests (speed, distance, turn accuracy) are MANUAL tests,
 *      listed in TEST_LOG.md.
 */

#include "subsystem2.h"
#include "s2_config.h"
#include "motion_pid.h"
#include "../test_report.h"

#if APP_TEST

#define S2_TEST_HOLD_STEPS 1000 /* 1000 x 10 ms = 10 s */

static int32_t absdiff(int32_t lhs, int32_t rhs)
{
    return (lhs > rhs) ? (lhs - rhs) : (rhs - lhs);
}

/* S2-U04: 10 s of a wheel held still.  True if the output never exceeds
   S2_DUTY_MAX and the integral stays within S2_PID_I_LIMIT. */
static bool pid_stays_in_limits(void)
{
    pid_ctrl_t pid;
    float32_t  out;
    int32_t    idx;
    bool       b_ok = true;

    pid_init(&pid, S2_PID_KFF, S2_PID_KP, S2_PID_KI, S2_PID_KD, S2_PID_I_LIMIT,
             (float32_t)S2_DUTY_MAX);
    for (idx = 0; idx < S2_TEST_HOLD_STEPS; idx++)
    {
        out = pid_update(&pid, 400.0f, 0.0f, 0.01f);
        if (out > (float32_t)S2_DUTY_MAX)
        {
            b_ok = false;
        }
    }
    return (b_ok && (pid.integral <= S2_PID_I_LIMIT));
}

/* S2-U05: with the wheel too slow, the output rises over time (I term) */
static bool pid_integral_rises(void)
{
    pid_ctrl_t pid;
    float32_t  out;
    float32_t  prev;

    pid_init(&pid, S2_PID_KFF, S2_PID_KP, S2_PID_KI, 0.0f, S2_PID_I_LIMIT,
             (float32_t)S2_DUTY_MAX);
    prev = pid_update(&pid, 100.0f, 50.0f, 0.01f);
    out  = pid_update(&pid, 100.0f, 50.0f, 0.01f);
    return (out > prev);
}

/* S2-U06: the control loop is running at its period.  Count only a
   500 ms window: overruns from boot (USB connect, the burst of test
   output before this point) are not the loop's fault. */
static bool loop_runs_on_time(void)
{
    motion_status_t status;
    uint32_t        loops0;
    uint32_t        ovr0;
    uint32_t        loops;
    bool            b_ok;

    tk_dly_tsk(300); /* let the console drain the test output */
    b_ok   = (E_OK == s2_get_status(&status));
    loops0 = status.loop_count;
    ovr0   = status.overruns;
    tk_dly_tsk(500);
    b_ok  = b_ok && (E_OK == s2_get_status(&status));
    loops = status.loop_count - loops0;

    /* The counts are far below INT32_MAX, so the casts keep the value */
    tm_printf((uint8_t *)"[TEST] S2-U06 info: %d loops in 500 ms, %d overruns"
                         " (%d since boot)\n",
              (int32_t)loops, (int32_t)(status.overruns - ovr0),
              (int32_t)status.overruns);
    return (b_ok && (loops >= 45U) && (loops <= 55U) &&
            (status.overruns == ovr0));
}

void subsystem2_test(void)
{
    motion_status_t status;
    bool            b_ok;
    UINT            ptn;

    TEST_INIT();

    /* S2-U01: one wheel turn of ticks = one wheel circumference.
       Casts: values are a few hundred mm, well inside int32_t. */
    TEST_CHECK("S2-U01",
               absdiff((int32_t)motion_ticks_to_mm(S2_TICKS_PER_REV),
                       (int32_t)(S2_PI * S2_WHEEL_DIAM_MM)) <= 1,
               "ticks_to_mm: one revolution = pi * wheel diameter");

    /* S2-U02: a 360 deg pivot moves each wheel pi * track */
    TEST_CHECK("S2-U02",
               (absdiff((int32_t)motion_turn_arc_mm(360),
                        (int32_t)(S2_PI * S2_TRACK_MM)) <= 1) &&
                   ((int32_t)motion_turn_arc_mm(-90) ==
                    (int32_t)motion_turn_arc_mm(90)),
               "turn arc: 360 deg = pi * track, left = right");

    /* S2-U03: ramp moves by at most one step and never overshoots */
    TEST_CHECK("S2-U03",
               (8 == motion_ramp(0, 100, 8)) &&
                   (100 == motion_ramp(95, 100, 8)) &&
                   (-8 == motion_ramp(0, -100, 8)),
               "set-point ramp limited to one step");

    /* S2-U04: output is clamped and the integral cannot wind up */
    TEST_CHECK("S2-U04", pid_stays_in_limits(),
               "PID output and integral stay within limits");

    /* S2-U05: at the target, output equals the feed-forward plus
       integral; below it the integral raises the output */
    TEST_CHECK("S2-U05", pid_integral_rises(),
               "PID integral increases output while below target");

    TEST_CHECK("S2-U06", loop_runs_on_time(),
               "motion_task runs every 10 ms, no overruns");

    /* S2-U07: STOP command is accepted and the robot is stopped */
    TEST_CHECK("S2-U07", E_OK == s2_cmd_stop(),
               "IF-M1 message buffer accepts a command");
    tk_dly_tsk(50);
    s2_get_status(&status);
    TEST_CHECK("S2-U08",
               (MSTATE_IDLE == status.state) && (0 == status.duty_l) &&
                   (0 == status.duty_r),
               "after STOP: state IDLE, both duties 0");

    /* S2-U09: emergency stop latches; DRIVE is refused until RESET */
    tk_clr_flg(g_flg_motion, ~MFLG_SAFE_STOPPED);
    s2_emergency_stop();
    b_ok = (E_OK == tk_wai_flg(g_flg_motion, MFLG_SAFE_STOPPED,
                               TWF_ORW | TWF_BITCLR, &ptn, 100));
    s2_cmd_drive(100, 0); /* must be ignored */
    tk_dly_tsk(50);
    s2_get_status(&status);
    TEST_CHECK("S2-U09",
               b_ok && (MSTATE_SAFE_STOP == status.state) &&
                   (0 == status.duty_l) && (0 == status.duty_r),
               "emergency stop within 100 ms, latched, DRIVE ignored");

    /* S2-U10: RESET leaves Safe Stop */
    s2_cmd_reset();
    tk_dly_tsk(50);
    s2_get_status(&status);
    TEST_CHECK("S2-U10", MSTATE_IDLE == status.state, "RESET returns to IDLE");

    TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */

/*** end of file ***/
