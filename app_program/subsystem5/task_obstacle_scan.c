/*
 *      task_obstacle_scan.c
 *      Subsystem 5 -- Obstacle Detection & Scanning task.
 *
 *      Owns: ultrasonic sensor (GP16/GP17) and scan servo (GP12, PWM 6).
 *
 *      Modes (Statechart 6.3):
 *        MONITOR  servo straight ahead, one ping every 20 ms.  S5_DETECT_COUNT
 *                 pings in a row under S5_DETECT_MM -> OBSTACLE_DETECTED
 *                 to the controller, then HOLD.
 *        HOLD     wait for the avoid task: SCAN_REQ (scan) or RESUME.
 *        SCAN     coarse scan, profile, fine scan around the edges,
 *                 profile again, hand it over (S5_FLG_PROFILE), HOLD.
 *      Monitoring pauses while S5_FLG_ENABLED is clear.
 */

#include "s5_local.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"

#define SCAN_MAX_SAMPLES 40
#define SCAN_READS       3 /* median of 3 per angle */
#define SCAN_READ_GAP_MS 15

typedef enum
{
    MODE_MONITOR,
    MODE_HOLD
} scan_mode_t;

static uint32_t g_silent_in_scan; /* consecutive no-echo during scans */

/* Turn the sensor to deg gently: small steps instead of one jump, so
   the servo cannot fling the sensor off its mount, then let it settle. */
static void point_and_wait(int32_t deg)
{
    int32_t now   = servo_get_deg();
    int32_t steps = 0;

    if (deg > SERVO_MAX_DEG)
    {
        deg = SERVO_MAX_DEG;
    }
    if (deg < -SERVO_MAX_DEG)
    {
        deg = -SERVO_MAX_DEG;
    }
    while (now != deg)
    {
        if ((deg - now) > S5_SERVO_STEP_DEG)
        {
            now += S5_SERVO_STEP_DEG;
        }
        else if ((now - deg) > S5_SERVO_STEP_DEG)
        {
            now -= S5_SERVO_STEP_DEG;
        }
        else
        {
            now = deg;
        }
        servo_set_deg(now);
        tk_dly_tsk(S5_SERVO_STEP_MS);
        steps++;
    }
    /* a single small step barely shakes the mount: settle less */
    tk_dly_tsk((steps <= 1) ? S5_SERVO_SETTLE_SHORT_MS : S5_SERVO_SETTLE_MS);
}

/* Sweep the given angles, one median reading each. */
static int32_t sweep(const int8_t * p_deg, int32_t count, s5_sample_t * p_out)
{
    int32_t  idx;
    uint16_t dist_mm;

    for (idx = 0; idx < count; idx++)
    {
        point_and_wait(p_deg[idx]);
        dist_mm = us_read_median_mm(SCAN_READS, SCAN_READ_GAP_MS);
        s5_note_ping(dist_mm);
        g_silent_in_scan =
            (US_NO_ECHO == dist_mm) ? (g_silent_in_scan + 1U) : 0U;
        p_out[idx].deg = p_deg[idx];
        p_out[idx].mm  = dist_mm;
    }
    return count;
}

static void print_profile(const char * p_tag, const s5_profile_t * p_profile)
{
    tm_printf((uint8_t *)"s5: %s d=%dmm edges R%d..L%d deg w=%dmm "
                         "clrL=%d clrR=%d side=%d\n",
              p_tag, p_profile->distance_mm, p_profile->right_edge_deg,
              p_profile->left_edge_deg, p_profile->width_mm,
              p_profile->clear_left_mm, p_profile->clear_right_mm,
              p_profile->bypass_side);
}

/* Coarse then fine scan.  Leaves the profile in the status snapshot
   (valid = 0 if nothing usable was found). */
static void do_scan(void)
{
    int8_t       ang[SCAN_MAX_SAMPLES];
    s5_sample_t  smp[SCAN_MAX_SAMPLES];
    s5_profile_t prof;
    s5_profile_t coarse;
    int32_t      count;
    s5_evt_t     evt = { 0 };

    g_silent_in_scan = 0;

    /* 1. coarse */
    s5_set_state(S5_ST_COARSE_SCAN);
    count = s5_coarse_angles(ang, SCAN_MAX_SAMPLES);
    count = sweep(ang, count, smp);
    if (!s5_build_profile(smp, count, &prof))
    {
        tm_printf((uint8_t *)"s5: coarse scan found nothing\n");
    }
    else
    {
        print_profile("coarse", &prof);
        coarse = prof;

        /* 2. fine, around the edges seen by the coarse scan */
        s5_set_state(S5_ST_FINE_SCAN);
        count = s5_fine_angles(&prof, ang, SCAN_MAX_SAMPLES);
        count = sweep(ang, count, smp);
        if (s5_build_profile(smp, count, &prof))
        {
            print_profile("fine", &prof);
        }
        else
        {
            /* moved away or a bad read: keep the coarse profile */
            tm_printf((uint8_t *)"s5: fine scan lost it, using coarse\n");
            prof = coarse;
        }
    }

    if (g_silent_in_scan >= S5_ECHO_FAULT_COUNT)
    {
        s5_set_fault(S5_FAULT_ECHO_TIMEOUT);
        evt.type = S5_EVT_FAULT;
        evt.code = S5_FAULT_ECHO_TIMEOUT;
        s5_send_evt(&evt);
    }

    point_and_wait(0);
    s5_store_profile(&prof);
}

/* An obstacle has been seen S5_DETECT_COUNT times in a row */
static void report_obstacle(uint16_t dist_mm)
{
    s5_evt_t evt = { 0 };

    s5_set_state(S5_ST_DETECTED);
    s5_count_obstacle();
    tm_printf((uint8_t *)"s5: obstacle at %d mm\n", dist_mm);

    evt.type        = S5_EVT_OBSTACLE_DETECTED;
    evt.distance_mm = dist_mm;
    /* Safety event: if the VC queue is full, stop the car anyway, then
       HOLD as normal (Tribunal 1 Version C, latched stop path). */
    (void)s5_send_critical(&evt);
#if S5_BENCH_MODE
    { /* play the controller: start avoiding at once */
        s5_cmd_t cmd = { S5_CMD_START_AVOID, 0, 0 };

        tk_snd_mbf(g_mbf_s5_cmd, &cmd, sizeof(cmd), TMO_POL);
    }
#endif
}

/* One MONITOR period: ping straight ahead, count near readings.
   Returns the next mode (HOLD once an obstacle is reported). */
static scan_mode_t monitor_step(int32_t * p_near_count)
{
    SYSTIM      start_time;
    SYSTIM      end_time;
    int32_t     used;
    uint16_t    dist_mm;
    scan_mode_t mode = MODE_MONITOR;

    tk_get_tim(&start_time);
    dist_mm = us_read_mm();
    s5_note_ping(dist_mm);

    if ((US_NO_ECHO != dist_mm) && (dist_mm < S5_DETECT_MM))
    {
        (*p_near_count)++;
    }
    else
    {
        *p_near_count = 0;
    }

    if (*p_near_count >= S5_DETECT_COUNT)
    {
        *p_near_count = 0;
        mode          = MODE_HOLD;
        report_obstacle(dist_mm);
    }
    else
    {
        /* keep the 20 ms period (NFR2).  Cast: the difference of two
           wrapping millisecond counters is a small signed value. */
        tk_get_tim(&end_time);
        used = (int32_t)(end_time.lo - start_time.lo);
        if (used < S5_PERIOD_MS)
        {
            tk_dly_tsk(S5_PERIOD_MS - used);
        }
    }
    return mode;
}

void s5_scan_task(int32_t stacd, void * p_exinf)
{
    scan_mode_t mode       = MODE_MONITOR;
    int32_t     near_count = 0;
    UINT        flg;

    (void)stacd;
    (void)p_exinf;

    app_check(us_init(), "s5 us_init");
    app_check(servo_init(), "s5 servo_init");
    tm_printf((uint8_t *)"s5: scan task %d up (bench mode %d)\n", tk_get_tid(),
              S5_BENCH_MODE);

    for (;;)
    {
        /* Requests from the avoid task come first, in any mode */
        if (E_OK == tk_wai_flg(g_flg_s5, S5_FLG_SCAN_REQ, TWF_ORW | TWF_BITCLR,
                               &flg, TMO_POL))
        {
            do_scan();
            tk_set_flg(g_flg_s5, S5_FLG_PROFILE);
            mode = MODE_HOLD;
        }
        else
        {
            if (E_OK == tk_wai_flg(g_flg_s5, S5_FLG_RESUME,
                                   TWF_ORW | TWF_BITCLR, &flg, TMO_POL))
            {
                point_and_wait(0);
                near_count = 0;
                mode       = MODE_MONITOR;
            }

            if (MODE_HOLD == mode)
            {
                /* sleep until the avoid task wants something */
                tk_wai_flg(g_flg_s5, S5_FLG_SCAN_REQ | S5_FLG_RESUME, TWF_ORW,
                           &flg, TMO_FEVR);
            }
            /* MONITOR, but only while the controller allows it */
            else if (E_OK != tk_wai_flg(g_flg_s5, S5_FLG_ENABLED, TWF_ORW, &flg,
                                        TMO_POL))
            {
                near_count = 0;
                servo_off();
                tk_wai_flg(g_flg_s5, S5_FLG_ENABLED | S5_FLG_SCAN_REQ, TWF_ORW,
                           &flg, TMO_FEVR);
                point_and_wait(0); /* ramp back to centre, no jump */
            }
            else
            {
                mode = monitor_step(&near_count);
            }
        }
    }
}

/*** end of file ***/
