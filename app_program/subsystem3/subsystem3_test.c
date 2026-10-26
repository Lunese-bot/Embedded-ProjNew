/*
 *      subsystem3_test.c
 *      On-target tests for subsystem 3.  Built only with: make TEST=1
 *      Record unit results in testing/subsystem3/TEST_LOG.md.
 *
 *      S3-U01..U05 need no IR hardware: they feed synthetic data to the
 *      algorithms and to barcode_task.  S3-U06 needs the robot on the
 *      track (manual).  The same algorithm checks also run on a PC:
 *      testing/subsystem3/host/.
 *
 *      NOTE: S3-U05 reads g_mbf_s3_events itself.  If the Vehicle Controller
 *      is also reading it in the test build, it may take the event first;
 *      run this test before the controller starts, or disable it.
 */

#include "subsystem3.h"
#include "../subsystem6/subsystem6.h" /* g_tid_vc: paused during S3-U05 */
#include "code39.h"
#include "line_algo.h"
#include "../test_report.h"

#if APP_TEST

#define S3_TEST_POLL_MS    50  /* wait_event() polling step */
#define S3_TEST_LIVE_LINES 10  /* 10 lines ...                    */
#define S3_TEST_LIVE_MS    200 /* ... 200 ms apart = 2 s of status */

static void drain_events(void)
{
    s3_event_t evt;

    while (tk_rcv_mbf(g_mbf_s3_events, &evt, TMO_POL) > 0)
    {
        /* discard */
    }
}

/* Wait up to tmo_ms for an event of the given type */
static bool wait_event(uint8_t type, s3_event_t * p_evt, int32_t tmo_ms)
{
    bool b_found = false;

    while ((!b_found) && (tmo_ms > 0))
    {
        if ((int32_t)sizeof(s3_event_t) ==
            tk_rcv_mbf(g_mbf_s3_events, p_evt, S3_TEST_POLL_MS))
        {
            b_found = (p_evt->type == type);
        }
        else
        {
            tmo_ms -= S3_TEST_POLL_MS;
        }
    }
    return b_found;
}

/* Reverse the order of count widths (a barcode read backwards) */
static void reverse_widths(uint32_t * p_widths, uint32_t count)
{
    uint32_t idx;
    uint32_t width;

    for (idx = 0; idx < (count / 2); idx++)
    {
        width                     = p_widths[idx];
        p_widths[idx]             = p_widths[count - 1 - idx];
        p_widths[count - 1 - idx] = width;
    }
}

/* S3-U03: a line under the right sensor only gives position +1000 and
   a steering request to the right */
static bool line_on_right_ok(void)
{
    line_params_t  params;
    line_tracker_t tracker;
    uint16_t       right_only[S3_IR_COUNT] = { 0, 0, 1000 };

    line_params_default(&params);
    params.line_mask = S3_MASK_L | S3_MASK_C | S3_MASK_R;
    line_tracker_reset(&tracker);
    (void)line_tracker_step(&tracker, &params, right_only, false);
    return ((1000 == tracker.position) && (tracker.steering > 0) &&
            (1 == tracker.last_side));
}

/* S3-U04 information: line task counters and kernel state */
static void print_line_task_info(uint32_t samples)
{
    T_RTSK task_ref;
    T_RCYC cyc_ref;
    ER     ercd_a;
    ER     ercd_b;

    ercd_a = tk_ref_tsk(g_tid_s3_line, &task_ref);
    ercd_b = tk_ref_cyc(g_cyc_s3_line, &cyc_ref);

    /* Casts: counters and kernel codes fit int32_t, printed with %d/%x */
    tm_printf((uint8_t *)"[TEST] S3-U04 info: %d samples in 500 ms, loops %d,"
                         " overruns %d, read errors %d, task stat 0x%x (er %d),"
                         " cyc stat 0x%x (er %d)\n",
              (int32_t)samples, (int32_t)g_s3_line_wakeups,
              (int32_t)g_s3_line_overruns, (int32_t)g_s3_line_read_errors,
              (int32_t)task_ref.tskstat, (int32_t)ercd_a,
              (int32_t)cyc_ref.cycstat, (int32_t)ercd_b);
}

/* S3-U05: feed the runs of barcode "*L*" into g_mbf_s3_bc_runs and wait
   for BARCODE_START, then BARCODE_DECODED(NAV_LEFT), on IF-32 */
static void feed_barcode(bool * p_b_start, bool * p_b_decoded)
{
    uint32_t    widths[S3_BC_MAX_ELEMENTS];
    s3_bc_run_t run;
    s3_event_t  evt;
    uint32_t    count;
    uint32_t    idx;

    /* The Vehicle Controller also reads g_mbf_s3_events (vc_bridge_s3.c)
       and would take these events first: pause it for this test. */
    (void)tk_sus_tsk(g_tid_vc);
    drain_events();
    count        = code39_encode("L", 20000, 50000, widths, S3_BC_MAX_ELEMENTS);
    run.black    = 0;
    run.width_us = 500000; /* leading quiet zone */
    tk_snd_mbf(g_mbf_s3_bc_runs, &run, sizeof(run), TMO_FEVR);
    for (idx = 0; idx < count; idx++)
    {
        run.black    = (0U == (idx % 2U)) ? 1U : 0U;
        run.width_us = widths[idx];
        tk_snd_mbf(g_mbf_s3_bc_runs, &run, sizeof(run), TMO_FEVR);
    }
    *p_b_start   = wait_event(S3_EVT_BARCODE_START, &evt, 500);
    *p_b_decoded = wait_event(S3_EVT_BARCODE_DECODED, &evt, 1000) &&
                   (NAV_LEFT == evt.nav) && ('L' == evt.text[0]);
    drain_events();
    (void)tk_rsm_tsk(g_tid_vc);
}

/* S3-U06 (manual, on the track): see TEST_LOG.md.  Print the live
   status for 2 s so a person can move the robot over the line. */
static void print_live_status(void)
{
    s3_line_status_t status;
    uint32_t         idx;

    for (idx = 0; idx < S3_TEST_LIVE_LINES; idx++)
    {
        s3_get_line_status(&status);
        tm_printf(
            (uint8_t *)"[S3] state=%d pos=%d steer=%d mask=%x raw=%d,%d,%d\n",
            status.line_state, status.position, status.steering,
            status.black_mask, status.raw[0], status.raw[1], status.raw[2]);
        tk_dly_tsk(S3_TEST_LIVE_MS);
    }
}

void subsystem3_test(void)
{
    uint32_t         widths[S3_BC_MAX_ELEMENTS];
    char             text[S3_BC_TEXT_MAX];
    bool             b_rev = false;
    code39_params_t  prm   = { S3_BC_MIN_RATIO_X10 };
    code39_result_t  result;
    s3_line_status_t st1;
    s3_line_status_t st2;
    uint32_t         count;
    uint32_t         samples;
    bool             b_start;
    bool             b_decoded;

    TEST_INIT();

    /* S3-U01: Code 39 "*L*" decodes, forwards and backwards */
    count  = code39_encode("L", 20, 50, widths, S3_BC_MAX_ELEMENTS);
    result = code39_decode(widths, count, &prm, text, sizeof(text), &b_rev);
    TEST_CHECK("S3-U01",
               (29U == count) && (CODE39_OK == result) && ('L' == text[0]) &&
                   ('\0' == text[1]) && (!b_rev),
               "Code 39 *L* decodes forwards");
    reverse_widths(widths, count);
    result = code39_decode(widths, count, &prm, text, sizeof(text), &b_rev);
    TEST_CHECK("S3-U01b", (CODE39_OK == result) && ('L' == text[0]) && b_rev,
               "Code 39 *L* decodes backwards");

    /* S3-U02: a corrupted barcode is rejected */
    count      = code39_encode("R", 20, 50, widths, S3_BC_MAX_ELEMENTS);
    widths[12] = 50; /* a 4th wide element in the data character */
    result     = code39_decode(widths, count, &prm, text, sizeof(text), &b_rev);
    TEST_CHECK("S3-U02", CODE39_OK != result, "corrupted barcode rejected");

    /* S3-U03: line position and steering sign */
    TEST_CHECK("S3-U03", line_on_right_ok(),
               "line on the right -> position +1000, steer right");

    /* S3-U04: the line task runs every 10 ms (seq advances ~50 in 500 ms) */
    s3_get_line_status(&st1);
    tk_dly_tsk(500);
    s3_get_line_status(&st2);
    samples = st2.seq - st1.seq;
    print_line_task_info(samples);
    TEST_CHECK("S3-U04", (samples >= 45U) && (samples <= 55U),
               "line task samples every 10 ms");

    /* S3-U05: barcode_task decodes runs and reports them on IF-32 */
    feed_barcode(&b_start, &b_decoded);
    TEST_CHECK("S3-U05a", b_start, "BARCODE_START event sent");
    TEST_CHECK("S3-U05b", b_decoded, "BARCODE_DECODED event with NAV_LEFT");

    print_live_status();

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */

/*** end of file ***/
