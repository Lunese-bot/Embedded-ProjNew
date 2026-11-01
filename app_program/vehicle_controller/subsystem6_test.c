/*
 *      subsystem6_test.c
 *      On-target tests for subsystem 6.  Built only with: make TEST=1
 *      Record results in testing/subsystem6/TEST_LOG.md.  IDs: S6-U01, ...
 *
 *      The state-machine tests drive a PRIVATE vc_sm_t, so the live
 *      controller and the motors are not touched.  The same scenarios run
 *      on a PC with host_test/ (no Pico needed).
 */

#include "subsystem6.h"
#include "vc_sm.h"
#include "../test_report.h"

#if APP_TEST

static vc_sm_t      g_t_sm;
static vc_actions_t g_t_out;
static uint32_t     g_t_now;

static void feed(vc_source_t src, vc_event_type_t type, int32_t param1,
                 int32_t param2)
{
    vc_event_t event = { 0 };

    event.src  = (uint8_t)src;
    event.type = (uint8_t)type;
    event.p1   = param1;
    event.p2   = param2;
    vc_sm_handle(&g_t_sm, &event, g_t_now, &g_t_out);
}

static void advance(uint32_t delta_ms) /* ticks every 20 ms for ms */
{
    uint32_t done;

    for (done = 0; done < delta_ms; done += 20)
    { /* wrap-safe */
        g_t_now += 20;
        feed(VC_SRC_VC, VC_EV_TICK, 0, 0);
    }
}

static bool has_move(vc_move_t move)
{
    int32_t idx;

    for (idx = 0; idx < g_t_out.n; idx++)
    {
        if ((VC_ACT_MOVE == g_t_out.a[idx].kind) &&
            (g_t_out.a[idx].p1 == (int32_t)move))
        {
            return true;
        }
    }
    return false;
}

static void start_mission(void)
{
    g_t_now = 1000;
    vc_sm_init(&g_t_sm, g_t_now);
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
    feed(VC_SRC_LINE, VC_EV_READY, 0, 0);
    feed(VC_SRC_IMU, VC_EV_READY, 0, 0);
    feed(VC_SRC_OBST, VC_EV_READY, 0, 0);
}

void subsystem6_test(void)
{
    vc_status_t status;
    TEST_INIT();

    /* --- live objects --- */
    TEST_CHECK("S6-U01",
               (g_tid_vc > 0) && (g_mbf_vc_events > 0) &&
                   (g_mtx_vc_status > 0) && (g_flg_vc_state > 0),
               "kernel objects and task created");
    TEST_CHECK("S6-U02",
               (E_OK == vc_get_status(&status)) && (VC_ST_IDLE == status.state),
               "live controller waits in IDLE");
    TEST_CHECK("S6-U03",
               E_OK ==
                   vc_post_event(VC_SRC_COMMS, VC_EV_WIFI_UP, 0, 0, TMO_POL),
               "vc_post_event accepted");

    /* --- state machine (private instance) --- */
    start_mission();
    TEST_CHECK("S6-U04",
               (VC_ST_FOLLOW_LINE == g_t_sm.st.state) &&
                   has_move(VC_MOVE_FORWARD),
               "start + all READY -> FOLLOW_LINE, drive forward");

    g_t_now = 1000;
    vc_sm_init(&g_t_sm, g_t_now);
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    feed(VC_SRC_IMU, VC_EV_READY, 0, 0);
    advance(VC_SELF_CHECK_MS + 40);
    TEST_CHECK("S6-U05", VC_ST_FAULT == g_t_sm.st.state,
               "self check: motion/line missing -> FAULT");

    start_mission();
    feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
    feed(VC_SRC_LINE, VC_EV_BARCODE_DECODED, NAV_LEFT, 0);
    TEST_CHECK("S6-U06",
               (VC_ST_EXECUTE_ACTION == g_t_sm.st.state) &&
                   has_move(VC_MOVE_TURN_LEFT),
               "barcode LEFT -> EXECUTE_ACTION, turn left");
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 0, 0);
    TEST_CHECK("S6-U07", VC_ST_FOLLOW_LINE == g_t_sm.st.state,
               "manoeuvre done -> FOLLOW_LINE");

    start_mission();
    feed(VC_SRC_LINE, VC_EV_LINE_LOST, 0, 0);
    advance(VC_LINE_ATTEMPT_MS * (VC_LINE_MAX_ATTEMPTS + 1));
    TEST_CHECK("S6-U08",
               (VC_ST_FAULT == g_t_sm.st.state) &&
                   (VC_FC_LINE_RECOVERY == g_t_sm.st.last_fault_code),
               "line not found within limits -> FAULT");

    start_mission();
    feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
    feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_STEER, 300);
    TEST_CHECK("S6-U09",
               (VC_ST_AVOID_OBSTACLE == g_t_sm.st.state) && (0 == g_t_out.n) &&
                   (1 == g_t_sm.st.moves_rejected),
               "AVOID: move request from S3 rejected");
    feed(VC_SRC_OBST, VC_EV_MOVE_REQUEST, VC_MOVE_TURN_RIGHT, 45);
    TEST_CHECK("S6-U10", has_move(VC_MOVE_TURN_RIGHT),
               "AVOID: move request from S5 forwarded");

    start_mission();
    feed(VC_SRC_COMMS, VC_EV_WIFI_DOWN, 0, 0);
    TEST_CHECK("S6-U11", VC_ST_FOLLOW_LINE == g_t_sm.st.state,
               "WiFi down is non-critical: keep following");

    feed(VC_SRC_IMU, VC_EV_FAULT, VC_FAULT_CRITICAL, 0x4001);
    TEST_CHECK("S6-U12",
               (VC_ST_FAULT == g_t_sm.st.state) && has_move(VC_MOVE_SAFE_STOP),
               "critical fault -> FAULT, motors stopped");
    feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
    TEST_CHECK("S6-U13", VC_ST_IDLE == g_t_sm.st.state,
               "reset from FAULT -> IDLE");

    start_mission();
    feed(VC_SRC_IMU, VC_EV_HUMP_START, 0, 0);
    feed(VC_SRC_LINE, VC_EV_LINE_LOST, 0, 0);
    feed(VC_SRC_IMU, VC_EV_HUMP_CLEARED, 42, 0);
    TEST_CHECK("S6-U14",
               (VC_ST_FOLLOW_LINE == g_t_sm.st.state) &&
                   (42 == g_t_sm.st.max_hump_um),
               "hump: line loss ignored, peak recorded");

    TEST_SUMMARY("subsystem6");
}

#endif /* APP_TEST */

/*** end of file ***/
