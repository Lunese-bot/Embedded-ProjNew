/*
 *      test_vc_sm.c
 *      PC unit tests for the Vehicle Controller state machine (vc_sm.c).
 *      No Pico, no kernel.  From this folder:
 *
 *          make            # builds and runs, prints PASS/FAIL per check
 *
 *      IDs S6-H01 ... are recorded in testing/subsystem6/TEST_LOG.md.
 */

#include <stdio.h>
#include <string.h>
#include "../vc_sm.h"

static vc_sm_t      g_sm;
static vc_actions_t g_actions;
static uint32_t     g_now_ms;
static int32_t      g_passed;
static int32_t      g_failed;

#define CHECK(id, cond, desc)                                            \
    do                                                                   \
    {                                                                    \
        if (cond)                                                        \
        {                                                                \
            g_passed++;                                                  \
            printf("[TEST] %s PASS  %s\n", id, desc);                    \
        }                                                                \
        else                                                             \
        {                                                                \
            g_failed++;                                                  \
            printf("[TEST] %s FAIL  %s   (%s:%d)\n", id, desc, __FILE__, \
                   __LINE__);                                            \
        }                                                                \
    } while (0)

static void feed(vc_source_t src, vc_event_type_t type, int32_t param1,
                 int32_t param2)
{
    vc_event_t event = { 0 };

    event.src  = (uint8_t)src;
    event.type = (uint8_t)type;
    event.p1   = param1;
    event.p2   = param2;
    vc_sm_handle(&g_sm, &event, g_now_ms, &g_actions);
}

static void feed_barcode(const char *text)
{
    vc_event_t event = { 0 };
    size_t i;

    event.src = VC_SRC_LINE;
    event.type = VC_EV_BARCODE_DECODED;
    for (i = 0u; (i < sizeof(event.barcode_text) - 1u) && (text[i] != '\0'); i++)
    {
        event.barcode_text[i] = text[i];
    }
    event.barcode_text[i] = '\0';
    vc_sm_handle(&g_sm, &event, g_now_ms, &g_actions);
}

#define HOST_TICK_MS 20U /* simulated controller tick */

static void advance(uint32_t delta_ms)
{
    /* g_now_ms may wrap around: the state machine must cope */
    for (uint32_t done = 0; done < delta_ms; done += HOST_TICK_MS)
    {
        g_now_ms += HOST_TICK_MS;
        feed(VC_SRC_VC, VC_EV_TICK, 0, 0);
    }
}

static int32_t has_move(vc_move_t move, int32_t value, int32_t check_value)
{
    int32_t found = 0;

    for (int32_t idx = 0; idx < g_actions.n; idx++)
    {
        if ((VC_ACT_MOVE == g_actions.a[idx].kind) &&
            (g_actions.a[idx].p1 == (int32_t)move) &&
            ((0 == check_value) || (g_actions.a[idx].p2 == value)))
        {
            found = 1;
        }
    }
    return found;
}

static int32_t has_request(vc_source_t target, vc_request_t req)
{
    int32_t found = 0;

    for (int32_t idx = 0; idx < g_actions.n; idx++)
    {
        if ((VC_ACT_REQUEST == g_actions.a[idx].kind) &&
            (g_actions.a[idx].target == target) &&
            (g_actions.a[idx].p1 == (int32_t)req))
        {
            found = 1;
        }
    }
    return found;
}

static void start_mission(void)
{
    g_now_ms = 0xFFFFF000u; /* close to wrap-around on purpose */
    vc_sm_init(&g_sm, g_now_ms);
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
    feed(VC_SRC_LINE, VC_EV_READY, 0, 0);
    feed(VC_SRC_IMU, VC_EV_READY, 0, 0);
    feed(VC_SRC_OBST, VC_EV_READY, 0, 0);
}

/* start-up */
static void test_startup(void)
{
    g_now_ms = 0;
    vc_sm_init(&g_sm, g_now_ms);
    feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_FORWARD, 200);
    CHECK("S6-H01", (VC_ST_IDLE == g_sm.st.state) && (0 == g_actions.n),
          "IDLE ignores move requests");
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    CHECK("S6-H02",
          (VC_ST_MISSION_START == g_sm.st.state) &&
              has_request(VC_SRC_NONE, VC_REQ_SELF_CHECK),
          "start -> MISSION_START, self check requested");

    start_mission();
    CHECK("S6-H03",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
              has_move(VC_MOVE_FORWARD, VC_SPEED_CRUISE_MMS, 1) &&
              has_request(VC_SRC_LINE, VC_REQ_LINE_FOLLOW),
          "all READY -> FOLLOW_LINE at cruise speed");

    g_now_ms = 0;
    vc_sm_init(&g_sm, g_now_ms);
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
    feed(VC_SRC_LINE, VC_EV_READY, 0, 0);
    advance(VC_SELF_CHECK_MS + 20);
    CHECK("S6-H04",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
              (VC_FC_SELF_CHECK == g_sm.st.last_fault_code),
          "optional IMU/obstacle missing -> run degraded, reported");

    g_now_ms = 0;
    vc_sm_init(&g_sm, g_now_ms);
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    feed(VC_SRC_IMU, VC_EV_READY, 0, 0);
    advance(VC_SELF_CHECK_MS + 20);
    CHECK("S6-H05", VC_ST_FAULT == g_sm.st.state,
          "motion/line missing -> FAULT");
}

/* arbitration */
static void test_arbitration(void)
{
    start_mission();
    feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_STEER, -300);
    CHECK("S6-H06", has_move(VC_MOVE_STEER, -300, 1),
          "FOLLOW_LINE: S3 steering forwarded");
    feed(VC_SRC_OBST, VC_EV_MOVE_REQUEST, VC_MOVE_REVERSE, 100);
    CHECK("S6-H07", (0 == g_actions.n) && (1 == g_sm.st.moves_rejected),
          "FOLLOW_LINE: S5 move rejected");
    feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, 99, 0);
    CHECK("S6-H08", 0 == g_actions.n, "invalid move code rejected");
    feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_STOP_CONFIRM, 0);
    CHECK("S6-H08a", 0 == g_actions.n,
          "internal stop-confirm handshake cannot be requested by a subsystem");
}

/* barcode and junction stop */
static void test_barcode(void)
{
    start_mission();
    feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
    CHECK("S6-H09",
          (VC_ST_DECODE_BARCODE == g_sm.st.state) &&
              has_move(VC_MOVE_SET_SPEED, VC_SPEED_SLOW_MMS, 1),
          "barcode start -> DECODE_BARCODE, slow");
    feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_FORWARD, 300);
    CHECK("S6-H10", has_move(VC_MOVE_FORWARD, VC_SPEED_SLOW_MMS, 1),
          "DECODE: S3 speed capped to slow");
    feed(VC_SRC_LINE, VC_EV_LINE_LOST, 0, 0);
    CHECK("S6-H11", VC_ST_DECODE_BARCODE == g_sm.st.state,
          "DECODE: line loss ignored");

    feed_barcode("RIGHT");
    CHECK("S6-H12",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
              (1 == g_sm.st.barcodes_read) && (NAV_NONE == g_sm.pending_nav) &&
              (0 == strcmp("RIGHT", g_sm.pending_barcode)),
          "full barcode text buffered; no unconfirmed mapping is assumed");

    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    CHECK("S6-H13",
          (VC_ST_JUNCTION_STOP == g_sm.st.state) &&
              has_move(VC_MOVE_STOP_CONFIRM, 0, 1) &&
              !has_move(VC_MOVE_TURN_RIGHT, VC_TURN_DEG, 1),
          "junction issues STOP and waits; no turn is issued yet");
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    CHECK("S6-H13b",
          (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
              has_move(VC_MOVE_DRIVE_DIST, VC_JUNCTION_CLEAR_MM, 1),
          "after S2 confirms stopped, unknown barcode defaults to straight clear");
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("S6-H13a", (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
          (NAV_NONE == g_sm.pending_nav) && ('\0' == g_sm.pending_barcode[0]),
          "junction action complete clears buffered barcode");

    /* Full-string rules are exercised with test-only mappings. */
    feed_barcode("TESTLEFT");
    CHECK("S6-H13c", NAV_LEFT == g_sm.pending_nav,
          "full barcode string maps to LEFT; matching is not first-character based");
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    CHECK("S6-H13d", (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
          has_move(VC_MOVE_TURN_LEFT, VC_TURN_DEG, 1),
          "configured full-string rule executes only after stop confirmation");
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);

    /* A junction without any barcode follows the same safe straight path. */
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    CHECK("S6-H14", VC_ST_JUNCTION_STOP == g_sm.st.state,
          "junction without barcode still stops first");
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    CHECK("S6-H14a", (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
          has_move(VC_MOVE_DRIVE_DIST, VC_JUNCTION_CLEAR_MM, 1),
          "no barcode -> move straight far enough to clear junction");
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("S6-H14b", (VC_ST_FOLLOW_LINE == g_sm.st.state),
          "resume line following after junction clear completes");

    /* A stop that never completes must fail safe rather than turning. */
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    advance(VC_JUNCTION_STOP_MS + 20);
    CHECK("S6-H14c", (VC_ST_FAULT == g_sm.st.state) &&
          (VC_FC_JUNCTION_STOP == g_sm.st.last_fault_code),
          "stop confirmation timeout -> critical safe stop");

    start_mission();
    feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
    feed(VC_SRC_LINE, VC_EV_BARCODE_FAILED, 0, 0);
    feed(VC_SRC_LINE, VC_EV_BARCODE_FAILED, 0, 0);
    CHECK("S6-H15",
          (VC_ST_DECODE_BARCODE == g_sm.st.state) &&
              has_request(VC_SRC_LINE, VC_REQ_BARCODE_READ),
          "barcode retry within limit");
    feed(VC_SRC_LINE, VC_EV_BARCODE_FAILED, 0, 0);
    CHECK("S6-H16",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
              (VC_FC_BARCODE == g_sm.st.last_fault_code) &&
              (VC_FAULT_NON_CRITICAL == g_sm.st.last_fault_sev),
          "retries used up -> keep following, non-critical report");

    feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
    advance(VC_DECODE_MS * (VC_BARCODE_MAX_RETRY + 1) + 40);
    CHECK("S6-H17", VC_ST_FOLLOW_LINE == g_sm.st.state,
          "barcode read timeouts count as failures");

    /* An unconfigured barcode must not cause a U-turn. */
    feed_barcode("UTURN");
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    CHECK("S6-H18", (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
          !has_move(VC_MOVE_U_TURN, 0, 0) &&
          has_move(VC_MOVE_DRIVE_DIST, VC_JUNCTION_CLEAR_MM, 1),
          "unconfigured UTURN text defaults to straight, not U-turn");
    advance(VC_EXECUTE_MS + 20);
    CHECK("S6-H18a", VC_ST_LOST_LINE == g_sm.st.state,
          "junction clearing timeout -> LOST_LINE recovery");
}

/* line loss */
static void test_line_loss(void)
{
    start_mission();
    feed(VC_SRC_LINE, VC_EV_LINE_LOST, 0, 0);
    CHECK("S6-H19",
          (VC_ST_LOST_LINE == g_sm.st.state) && has_move(VC_MOVE_STOP, 0, 0) &&
              has_request(VC_SRC_LINE, VC_REQ_LINE_SEARCH),
          "line lost -> stop, search");
    advance(VC_LINE_ATTEMPT_MS);
    feed(VC_SRC_LINE, VC_EV_LINE_FOUND, 0, 0);
    CHECK("S6-H20", VC_ST_FOLLOW_LINE == g_sm.st.state,
          "line found -> FOLLOW_LINE");
    feed(VC_SRC_LINE, VC_EV_LINE_LOST, 0, 0);
    advance(VC_LINE_ATTEMPT_MS * VC_LINE_MAX_ATTEMPTS + 40);
    CHECK("S6-H21",
          (VC_ST_FAULT == g_sm.st.state) &&
              (VC_FC_LINE_RECOVERY == g_sm.st.last_fault_code),
          "line recovery bounded -> FAULT");
}

/* obstacle */
static void test_obstacle(void)
{
    start_mission();
    feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
    CHECK("S6-H22",
          (VC_ST_AVOID_OBSTACLE == g_sm.st.state) &&
              has_request(VC_SRC_OBST, VC_REQ_OBST_AVOID),
          "obstacle -> AVOID");
    feed(VC_SRC_LINE, VC_EV_LINE_LOST, 0, 0);
    feed(VC_SRC_OBST, VC_EV_MOVE_REQUEST, VC_MOVE_TURN_LEFT, 30);
    CHECK("S6-H23",
          (VC_ST_AVOID_OBSTACLE == g_sm.st.state) &&
              has_move(VC_MOVE_TURN_LEFT, 30, 1),
          "AVOID: line loss ignored, S5 moves forwarded");
    feed(VC_SRC_OBST, VC_EV_AVOID_DONE, 0, 0);
    CHECK("S6-H24", VC_ST_FOLLOW_LINE == g_sm.st.state,
          "avoid done -> FOLLOW_LINE");
    feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
    feed(VC_SRC_OBST, VC_EV_AVOID_FAILED, 0, 0);
    CHECK("S6-H35",
          (VC_ST_LOST_LINE == g_sm.st.state) &&
              (VC_FC_AVOID_FAILED == g_sm.st.last_fault_code) &&
              (VC_FAULT_RECOVERABLE == g_sm.st.last_fault_sev),
          "avoid failed -> bounded line search (recoverable)");
    feed(VC_SRC_LINE, VC_EV_LINE_FOUND, 0, 0);
    feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
    advance(VC_AVOID_MS + 20);
    CHECK("S6-H25", VC_ST_FAULT == g_sm.st.state, "avoid timeout -> FAULT");
}

/* terrain */
static void test_terrain(void)
{
    start_mission();
    feed(VC_SRC_IMU, VC_EV_HUMP_START, 0, 0);
    feed(VC_SRC_IMU, VC_EV_HUMP_CLEARED, 35, 80);
    feed(VC_SRC_IMU, VC_EV_HUMP_START, 0, 0);
    feed(VC_SRC_IMU, VC_EV_HUMP_CLEARED, 20, 50);
    CHECK("S6-H26",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) && (35 == g_sm.st.max_hump_um),
          "highest hump kept");
    feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
    feed(VC_SRC_IMU, VC_EV_HUMP_CLEARED, 51, 0);
    CHECK("S6-H36",
          (VC_ST_DECODE_BARCODE == g_sm.st.state) &&
              (51 == g_sm.st.max_hump_um),
          "hump crossed while reading a barcode is still recorded");
}

/* faults */
static void test_faults(void)
{
    start_mission();
    feed(VC_SRC_COMMS, VC_EV_WIFI_DOWN, 0, 0);
    CHECK("S6-H27", (VC_ST_FOLLOW_LINE == g_sm.st.state) && !g_sm.st.wifi_up,
          "WiFi down: keep navigating");
    feed(VC_SRC_LINE, VC_EV_FAULT, VC_FAULT_RECOVERABLE, 0x3001);
    CHECK("S6-H28", VC_ST_LOST_LINE == g_sm.st.state,
          "recoverable line fault -> LOST_LINE");
    feed(VC_SRC_IMU, VC_EV_IMPACT, 0, 0);
    CHECK("S6-H29",
          (VC_ST_FAULT == g_sm.st.state) && has_move(VC_MOVE_SAFE_STOP, 0, 0) &&
              has_request(VC_SRC_NONE, VC_REQ_STOP_ALL),
          "impact -> FAULT, stop all");
    feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_FORWARD, 100);
    CHECK("S6-H30", 0 == g_actions.n, "FAULT: no motion accepted");
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    CHECK("S6-H31", VC_ST_FAULT == g_sm.st.state,
          "FAULT: start ignored until reset");
    feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
    CHECK("S6-H32",
          (VC_ST_IDLE == g_sm.st.state) && has_move(VC_MOVE_RESET, 0, 0),
          "reset -> IDLE, motion latch cleared");

    start_mission();
    feed(VC_SRC_COMMS, VC_EV_CMD_STOP, 0, 0);
    CHECK("S6-H33",
          (VC_ST_FAULT == g_sm.st.state) &&
              (VC_FC_OPERATOR_STOP == g_sm.st.last_fault_code),
          "remote stop -> FAULT");

    start_mission();
    feed(VC_SRC_LINE, VC_EV_END_OF_COURSE, 0, 0);
    CHECK("S6-H34",
          (VC_ST_MISSION_COMPLETE == g_sm.st.state) &&
              has_move(VC_MOVE_STOP, 0, 0),
          "end of course -> MISSION_COMPLETE");
}

/* line search pattern and ownership */
static void test_line_search(void)
{
    start_mission();
    feed(VC_SRC_LINE, VC_EV_LINE_LOST, +1, 0); /* last seen on the right */
    CHECK("S6-H37", has_move(VC_MOVE_TURN_RIGHT, VC_SEARCH_TURN_DEG, 1),
          "search turns towards the side the line was last seen");
    feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_STEER, 500);
    CHECK("S6-H38", 0 == g_actions.n,
          "LOST_LINE: S3 steering rejected while searching");
    advance(VC_LINE_ATTEMPT_MS);
    CHECK("S6-H39", has_move(VC_MOVE_TURN_LEFT, 2 * VC_SEARCH_TURN_DEG, 1),
          "second attempt turns twice as far the other way");
    feed(VC_SRC_LINE, VC_EV_LINE_FOUND, 0, 0);
    feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_RESET, 0);
    CHECK("S6-H40", 0 == g_actions.n, "subsystems may not request RESET");
    feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
    CHECK("S6-H41", has_request(VC_SRC_LINE, VC_REQ_BARCODE_OFF),
          "AVOID: barcode reading switched off");
}

int main(void)
{
    test_startup();
    test_arbitration();
    test_barcode();
    test_line_loss();
    test_obstacle();
    test_terrain();
    test_faults();
    test_line_search();

    printf("[TEST] vehicle_controller host SUMMARY %d passed, %d failed\n", g_passed,
           g_failed);
    return (0 != g_failed) ? 1 : 0;
}

/*** end of file ***/
