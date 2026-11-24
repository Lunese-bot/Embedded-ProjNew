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
#include "../vc_barcode_rules.h"
#include "../vc_barcode_config.h"

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

/* Counts every MOVE action issued since tally_clear(), by move type. */
static int32_t g_tally[32];

static void tally_clear(void)
{
    memset(g_tally, 0, sizeof(g_tally));
}

static void tally_add(void)
{
    for (int32_t idx = 0; idx < g_actions.n; idx++)
    {
        if ((VC_ACT_MOVE == g_actions.a[idx].kind) &&
            (g_actions.a[idx].p1 >= 0) && (g_actions.a[idx].p1 < 32))
        {
            g_tally[g_actions.a[idx].p1]++;
        }
    }
}

static void feed(vc_source_t src, vc_event_type_t type, int32_t param1,
                 int32_t param2)
{
    vc_event_t event = { 0 };

    event.src  = (uint8_t)src;
    event.type = (uint8_t)type;
    event.p1   = param1;
    event.p2   = param2;
    vc_sm_handle(&g_sm, &event, g_now_ms, &g_actions);
    tally_add();
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
    tally_add();
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
    feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
    CHECK("S6-SAFE01", (VC_ST_IDLE == g_sm.st.state) &&
              has_move(VC_MOVE_RESET, 0, 0),
          "reset in IDLE is forwarded to motion control");
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

    vc_sm_init(&g_sm, g_now_ms);
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_FAULT, VC_FAULT_CRITICAL, 0x2011);
    CHECK("S6-SAFE02", (VC_ST_FAULT == g_sm.st.state) &&
              has_move(VC_MOVE_SAFE_STOP, 0, 0),
          "safe stop after motion READY faults the mission");
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
/* LEFT / RIGHT at a junction: with VC_JUNCTION_ADVANCE_MM set, the robot
   first drives that far (axle over the crossing) and turns when S2 reports
   it done.  Feeds that MANOEUVRE_DONE so the next check sees the turn. */
static void junction_advance(void)
{
#if VC_JUNCTION_ADVANCE_MM > 0
    CHECK("S6-H45", (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
          has_move(VC_MOVE_DRIVE_DIST, VC_JUNCTION_ADVANCE_MM, 1) &&
          !has_move(VC_MOVE_TURN_LEFT, VC_TURN_CMD_DEG, 1) &&
          !has_move(VC_MOVE_TURN_RIGHT, VC_TURN_CMD_DEG, 1),
          "turn at a junction starts with the advance, no turn yet");
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
#endif
}

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
              !has_move(VC_MOVE_TURN_RIGHT, VC_TURN_CMD_DEG, 1),
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
    junction_advance();
    CHECK("S6-H13d", (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
          has_move(VC_MOVE_TURN_LEFT, VC_TURN_CMD_DEG, 1),
          "configured full-string rule executes only after stop confirmation");
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);

    /* Dashboard "At every junction: Right" (vc_bridge_s3.c): an action-only
       BARCODE_DECODED (p1 = nav, no text) just before the JUNCTION. */
    feed(VC_SRC_LINE, VC_EV_BARCODE_DECODED, (int32_t)NAV_RIGHT, 0);
    CHECK("S6-H13e", (VC_ST_FOLLOW_LINE == g_sm.st.state) && (NAV_RIGHT == g_sm.pending_nav),
          "action-only event sets RIGHT and keeps following");
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    junction_advance();
    CHECK("S6-H13f", (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
          has_move(VC_MOVE_TURN_RIGHT, VC_TURN_CMD_DEG, 1),
          "junction after it: stop, then turn RIGHT");
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("S6-H13g", (VC_ST_FOLLOW_LINE == g_sm.st.state) && (NAV_NONE == g_sm.pending_nav),
          "after the turn: follow the line again, action cleared");

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
    {
        int32_t k;
        bool    b_ok = (VC_ST_DECODE_BARCODE == g_sm.st.state);
        for (k = 0; k < VC_BARCODE_MAX_RETRY; k++)
        {
            feed(VC_SRC_LINE, VC_EV_BARCODE_FAILED, 0, 0);
            b_ok = b_ok && (VC_ST_DECODE_BARCODE == g_sm.st.state) &&
                   has_request(VC_SRC_LINE, VC_REQ_BARCODE_READ);
        }
        CHECK("S6-H15", b_ok, "barcode retry within limit");
    }
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

/* review 2026-11-10: junction reached while still in DECODE_BARCODE */
/* Junction turn ends on the line (VC_TURN_END_ON_LINE) */
static void line_at(uint32_t odo)
{
    feed(VC_SRC_LINE, VC_EV_LINE_FOUND, 0, (int32_t)odo);
}

static void test_turn_on_line(void)
{
#if VC_TURN_END_ON_LINE
    const uint32_t odo0 = 100000u;
    const uint32_t gate = VC_PIVOT_COUNTS(VC_TURN_GATE_DEG);

    CHECK("S6-H50", (gate > 400u) && (gate < 520u) &&
          (VC_PIVOT_COUNTS(90) > 650u) && (VC_PIVOT_COUNTS(90) < 740u),
          "pivot counts: 60 deg ~461, 90 deg ~692 (track 130, 692 ticks/rev)");

    start_mission();
    feed(VC_SRC_VC, VC_EV_TICK, 0, (int32_t)odo0);
    feed(VC_SRC_LINE, VC_EV_BARCODE_DECODED, (int32_t)NAV_LEFT, 0);
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    junction_advance();
    CHECK("S6-H52", (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
          has_move(VC_MOVE_TURN_LEFT, VC_TURN_MAX_DEG, 1) &&
          (gate == g_sm.turn_gate),
          "LEFT asks S2 for the max angle and watches for the line");
    line_at(odo0 + 10u); /* sensors still on the crossing as the turn starts */
    CHECK("S6-H51", VC_ST_EXECUTE_ACTION == g_sm.st.state,
          "line seen at the start of the turn ends nothing");
    line_at(g_sm.turn_odo0 + gate / 2u);
    CHECK("S6-H53", VC_ST_EXECUTE_ACTION == g_sm.st.state,
          "line before the gate (the arm it came from) is ignored");
    line_at(g_sm.turn_odo0 + gate + 5u);
    CHECK("S6-H54", (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
          (NAV_NONE == g_sm.pending_nav) && (0u == g_sm.turn_gate) &&
          has_move(VC_MOVE_FORWARD, VC_SPEED_CRUISE_MMS, 1),
          "line after the gate ends the turn: follow the new arm");

    /* no line by the max angle: S2's MANOEUVRE_DONE ends it as before */
    feed(VC_SRC_LINE, VC_EV_BARCODE_DECODED, (int32_t)NAV_RIGHT, 0);
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    junction_advance();
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("S6-H55", (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
          (0u == g_sm.turn_gate),
          "no line seen: turn ends at the max angle, follow as before");

    /* U-turn: the side arm at ~90 deg must not end it */
    feed(VC_SRC_LINE, VC_EV_BARCODE_DECODED, (int32_t)NAV_U_TURN, 0);
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    CHECK("S6-H56", (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
          has_move(VC_MOVE_U_TURN, 0, 0) &&
          (VC_PIVOT_COUNTS(VC_UTURN_GATE_DEG) == g_sm.turn_gate),
          "U-turn watches with the 150 deg gate");
    line_at(g_sm.turn_odo0 + VC_PIVOT_COUNTS(90));
    CHECK("S6-H57", VC_ST_EXECUTE_ACTION == g_sm.st.state,
          "U-turn: side arm at 90 deg ignored");
    line_at(g_sm.turn_odo0 + VC_PIVOT_COUNTS(170));
    CHECK("S6-H58", VC_ST_FOLLOW_LINE == g_sm.st.state,
          "U-turn: line at 170 deg ends it");

    /* STRAIGHT: no watch, LINE_FOUND does not cut the clearing drive */
    feed(VC_SRC_LINE, VC_EV_BARCODE_DECODED, (int32_t)NAV_STRAIGHT, 0);
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    line_at(g_sm.odo + 5000u);
    CHECK("S6-H59", (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
          (0u == g_sm.turn_gate),
          "STRAIGHT: line events do not end the clearing drive");
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
#endif
}

static void test_junction_in_decode(void)
{
    start_mission();
    feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    CHECK("S6-H42", (VC_ST_JUNCTION_STOP == g_sm.st.state) &&
          has_move(VC_MOVE_STOP_CONFIRM, 0, 1),
          "junction during DECODE_BARCODE is not dropped: stop first");
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("S6-H43", VC_ST_FOLLOW_LINE == g_sm.st.state,
          "then straight across and follow again");

    /* one failed read: no waiting for a retry that cannot happen */
    feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
    feed(VC_SRC_LINE, VC_EV_BARCODE_FAILED, 0, 0);
    CHECK("S6-H44", (VC_BARCODE_MAX_RETRY > 0) ||
          (VC_ST_FOLLOW_LINE == g_sm.st.state),
          "failed read returns to FOLLOW_LINE at once (retry 0)");
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
    bool b_early_cruise = false;

    start_mission();
    feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
    CHECK("S6-H22",
          (VC_ST_AVOID_OBSTACLE == g_sm.st.state) &&
              has_move(VC_MOVE_STOP_CONFIRM, 0, 1) &&
              !has_request(VC_SRC_OBST, VC_REQ_OBST_AVOID),
          "obstacle stops before scan starts");
    feed(VC_SRC_OBST, VC_EV_MOVE_REQUEST, VC_MOVE_TURN_LEFT, 30);
    CHECK("S6-OBS01", !has_move(VC_MOVE_TURN_LEFT, 30, 1),
          "no bypass movement before physical stop");
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    CHECK("S6-OBS02", has_request(VC_SRC_OBST, VC_REQ_OBST_AVOID),
          "physical stop confirmation starts scan");
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    CHECK("S6-OBS03", !has_request(VC_SRC_OBST, VC_REQ_OBST_AVOID),
          "duplicate stop indication does not restart scan");
    feed(VC_SRC_LINE, VC_EV_LINE_LOST, 0, 0);
    feed(VC_SRC_OBST, VC_EV_MOVE_REQUEST, VC_MOVE_TURN_LEFT, 30);
    CHECK("S6-H23",
          (VC_ST_AVOID_OBSTACLE == g_sm.st.state) &&
              has_move(VC_MOVE_TURN_LEFT, 30, 1),
          "AVOID: line loss ignored, S5 moves forwarded");
    feed(VC_SRC_OBST, VC_EV_MOVE_REQUEST, VC_MOVE_STOP, 0);
    CHECK("S6-OBS05", has_move(VC_MOVE_STOP_CONFIRM, 0, 1),
          "S5 stop for a line sighting waits for physical confirmation");
    feed(VC_SRC_OBST, VC_EV_AVOID_DONE, 0, 0);
    CHECK("S6-H24", (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
              has_move(VC_MOVE_SET_SPEED, VC_SPEED_SLOW_MMS, 1) &&
              !has_move(VC_MOVE_FORWARD, 0, 0),
          "confirmed line reacquisition waits for steering at slow speed");
    for (uint32_t waited = 0; waited < 1000; waited += HOST_TICK_MS)
    {
        g_now_ms += HOST_TICK_MS;
        feed(VC_SRC_VC, VC_EV_TICK, 0, 0);
        b_early_cruise |= has_move(VC_MOVE_SET_SPEED, VC_SPEED_CRUISE_MMS, 1);
    }
    feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_STEER, 100);
    CHECK("S6-OBS08",
          !b_early_cruise && has_move(VC_MOVE_STEER, 100, 1) &&
              !has_move(VC_MOVE_SET_SPEED, VC_SPEED_CRUISE_MMS, 1),
          "the first fresh steering command drives at the slow speed");
    for (uint32_t sample = 2; sample < VC_REJOIN_STEER_SAMPLES; sample++)
    {
        feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_STEER, 0);
        b_early_cruise |= has_move(VC_MOVE_SET_SPEED, VC_SPEED_CRUISE_MMS, 1);
    }
    feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_STEER, 0);
    CHECK("S6-OBS09",
          !b_early_cruise && has_move(VC_MOVE_SET_SPEED, VC_SPEED_CRUISE_MMS, 1),
          "cruise resumes on the expected steering sample");
    feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    feed(VC_SRC_OBST, VC_EV_AVOID_FAILED, 0, 0);
    CHECK("S6-H35",
          (VC_ST_FAULT == g_sm.st.state) &&
              (VC_FC_AVOID_FAILED == g_sm.st.last_fault_code) &&
              (VC_FAULT_CRITICAL == g_sm.st.last_fault_sev) &&
              has_move(VC_MOVE_SAFE_STOP, 0, 1) &&
              !has_request(VC_SRC_LINE, VC_REQ_LINE_SEARCH),
          "exhausted obstacle recovery latches a stop");
    start_mission();
    feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
    advance(VC_JUNCTION_STOP_MS);
    CHECK("S6-OBS04", VC_ST_FAULT == g_sm.st.state && has_move(VC_MOVE_SAFE_STOP, 0, 1),
          "missing stop confirmation faults instead of moving");
    start_mission();
    feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
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
    feed(VC_SRC_IMU, VC_EV_FAULT, VC_FAULT_CRITICAL, 0x4005); /* overturn */
    CHECK("S6-H29",
          (VC_ST_FAULT == g_sm.st.state) && has_move(VC_MOVE_SAFE_STOP, 0, 0) &&
              has_request(VC_SRC_NONE, VC_REQ_STOP_ALL) &&
              (0x4005 == g_sm.st.last_fault_code),
          "overturn (critical 0x4005) -> FAULT, stop all");
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


/* lift ends the run (professor's IMU change; collision detection removed) */
#define LIFT_CODE 0x4005
static void feed_lift(void)
{
    feed(VC_SRC_IMU, VC_EV_LIFTED, 1, LIFT_CODE);
}

static int32_t any_move(void)
{
    int32_t found = 0;

    for (int32_t idx = 0; idx < g_actions.n; idx++)
    {
        if (VC_ACT_MOVE == g_actions.a[idx].kind)
        {
            found = 1;
        }
    }
    return found;
}

static int32_t lift_ends_run_in(vc_state_t target)
{
    start_mission();
    switch (target)
    {
        case VC_ST_DECODE_BARCODE:
            feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
            break;
        case VC_ST_JUNCTION_STOP:
            feed_barcode("TESTLEFT");
            feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
            break;
        case VC_ST_LOST_LINE:
            feed(VC_SRC_LINE, VC_EV_LINE_LOST, 0, 0);
            break;
        case VC_ST_AVOID_OBSTACLE:
            feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
            break;
        case VC_ST_TERRAIN_CHECK:
            feed(VC_SRC_IMU, VC_EV_HUMP_START, 0, 0);
            break;
        default:
            break;
    }
    if (target != g_sm.st.state)
    {
        return 0;
    }
    feed_lift();
    return (VC_ST_MISSION_COMPLETE == g_sm.st.state) &&
           has_move(VC_MOVE_STOP, 0, 0) &&
           has_request(VC_SRC_NONE, VC_REQ_STOP_ALL) &&
           (NAV_NONE == g_sm.pending_nav);
}

static void test_lift(void)
{
    start_mission();
    CHECK("VC-L01", VC_ST_FOLLOW_LINE == g_sm.st.state, "mission running");
    feed_lift();
    CHECK("VC-L02",
          (VC_ST_MISSION_COMPLETE == g_sm.st.state) &&
              has_move(VC_MOVE_STOP, 0, 0) &&
              has_request(VC_SRC_NONE, VC_REQ_STOP_ALL) &&
              (VC_FC_NONE == g_sm.st.last_fault_code),
          "lift in FOLLOW_LINE -> MISSION_COMPLETE, stop all, no fault");
    CHECK("VC-L03", lift_ends_run_in(VC_ST_DECODE_BARCODE),
          "lift while reading a barcode ends the run");
    CHECK("VC-L04", lift_ends_run_in(VC_ST_JUNCTION_STOP),
          "lift at a junction ends the run, pending turn dropped");
    CHECK("VC-L05", lift_ends_run_in(VC_ST_LOST_LINE),
          "lift while searching for the line ends the run");
    CHECK("VC-L06", lift_ends_run_in(VC_ST_AVOID_OBSTACLE),
          "lift while avoiding an obstacle ends the run");
    CHECK("VC-L07", lift_ends_run_in(VC_ST_TERRAIN_CHECK),
          "lift on a hump ends the run");

    /* after the lift: motors stay off until Reset then Start */
    start_mission();
    feed_lift();
    feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_FORWARD, 100);
    CHECK("VC-L08", !any_move(), "after lift: set-down line follow request refused");
    feed(VC_SRC_LINE, VC_EV_LINE_FOUND, 0, 0);
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
    feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
    feed(VC_SRC_IMU, VC_EV_HUMP_START, 0, 0);
    advance(5000u);
    CHECK("VC-L09", VC_ST_MISSION_COMPLETE == g_sm.st.state,
          "after lift: line, junction, obstacle, hump events and time change nothing");
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    CHECK("VC-L10", (VC_ST_MISSION_COMPLETE == g_sm.st.state) && !any_move(),
          "after lift: nothing but Start or Reset leaves MISSION_COMPLETE");
    feed_lift();
    CHECK("VC-L11", (VC_ST_MISSION_COMPLETE == g_sm.st.state) && (0 == g_actions.n),
          "second lift while complete: no change, no action");
    feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
    CHECK("VC-L12", (VC_ST_IDLE == g_sm.st.state) && has_move(VC_MOVE_RESET, 0, 0),
          "Reset after lift -> IDLE, motion latch cleared");
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    CHECK("VC-L13", VC_ST_MISSION_START == g_sm.st.state,
          "Start after Reset begins a new mission");

    /* Start straight after a lift (e.g. MQTT start): Reset then Start */
    start_mission();
    feed_lift();
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    CHECK("VC-S01",
          (VC_ST_MISSION_START == g_sm.st.state) &&
              has_move(VC_MOVE_RESET, 0, 0) &&
              has_request(VC_SRC_NONE, VC_REQ_SELF_CHECK),
          "lift then Start: S2 latch reset, new mission starts (self check)");
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
    feed(VC_SRC_LINE, VC_EV_READY, 0, 0);
    feed(VC_SRC_IMU, VC_EV_READY, 0, 0);
    feed(VC_SRC_OBST, VC_EV_READY, 0, 0);
    CHECK("VC-S02", VC_ST_FOLLOW_LINE == g_sm.st.state,
          "lift then Start: mission runs again once subsystems are ready");
    feed_lift();
    feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
    CHECK("VC-S03", VC_ST_IDLE == g_sm.st.state,
          "lift then Reset still works");
    feed_lift();
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    feed_lift();
    CHECK("VC-S04", VC_ST_FAULT == g_sm.st.state,
          "lift again during the new self check -> FAULT 0x4005");
    /* overturn FAULT stays Reset-first */
    start_mission();
    feed(VC_SRC_IMU, VC_EV_FAULT, VC_FAULT_CRITICAL, 0x4005);
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    CHECK("VC-S05", VC_ST_FAULT == g_sm.st.state,
          "overturn FAULT: Start ignored, Reset first");

    /* lift outside a running mission */
    g_now_ms = 0u;
    vc_sm_init(&g_sm, g_now_ms);
    feed_lift();
    CHECK("VC-L14",
          (VC_ST_IDLE == g_sm.st.state) && !any_move() &&
              (LIFT_CODE == g_sm.st.last_fault_code) &&
              (VC_FAULT_CRITICAL == g_sm.st.last_fault_sev),
          "lift in IDLE: reported critical 0x4005, nothing starts");
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    feed_lift();
    CHECK("VC-L15",
          (VC_ST_FAULT == g_sm.st.state) &&
              (LIFT_CODE == g_sm.st.last_fault_code),
          "lift during self check (mission not running) -> FAULT 0x4005");
    feed_lift();
    CHECK("VC-L16", (VC_ST_FAULT == g_sm.st.state) && (0 == g_actions.n),
          "lift while already in FAULT: stays FAULT, no action");
}

/* letter configuration (vc_barcode_config.h) */
static void test_letter_config(void)
{
    static const navigation_command_t expect[VC_BARCODE_LETTER_COUNT] = {
#define VC_EXPECT(letter, nav) nav,
        VC_BARCODE_LETTER_TABLE(VC_EXPECT)
#undef VC_EXPECT
    };
    int32_t all_match = 1;
    int32_t not_none = 0;
    char    text[2] = { 'A', '\0' };

    for (uint32_t i = 0u; i < VC_BARCODE_LETTER_COUNT; i++)
    {
        text[0] = (char)('A' + (int)i);
        if (vc_barcode_lookup(text) != expect[i])
        {
            all_match = 0;
        }
        if (NAV_NONE != expect[i])
        {
            not_none++;
        }
    }
    CHECK("VC-C01", all_match,
          "every letter A..Z returns the value set in vc_barcode_config.h");
    CHECK("VC-C02",
          (NAV_LEFT == vc_barcode_lookup("H")) &&
              (NAV_RIGHT == vc_barcode_lookup("Z")) && (2 == not_none),
          "as shipped: H = LEFT, Z = RIGHT, every other letter NONE "
          "(when the team changes a row, update this check with it)");
    CHECK("VC-C03",
          (NAV_NONE == vc_barcode_lookup("")) &&
              (NAV_NONE == vc_barcode_lookup("z")) &&
              (NAV_NONE == vc_barcode_lookup("AA")) &&
              (NAV_NONE == vc_barcode_lookup("ZZ")) &&
              (NAV_NONE == vc_barcode_lookup("1")) &&
              (NAV_NONE == vc_barcode_lookup((const char *)0)),
          "empty, lower case, longer and unlisted text return NONE");
    start_mission();
    feed_barcode("Z");
    CHECK("VC-C04", NAV_RIGHT == g_sm.pending_nav,
          "decoded Z becomes a pending RIGHT turn");
}

/* ---- barcode letter -> junction navigation, end to end ---------------- */

/* A junction: S3 reports it, S2 confirms the stop, the axle advance (if any)
   completes.  Returns with the controller in EXECUTE_ACTION. */
static void junction_to_execute(void)
{
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    if (0u != g_sm.exec_phase)
    {
        feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0); /* advance done */
    }
}

static int32_t turns_issued(void)
{
    return g_tally[VC_MOVE_TURN_LEFT] + g_tally[VC_MOVE_TURN_RIGHT] +
           g_tally[VC_MOVE_U_TURN];
}

/* One decoded letter, then one junction.  The letter alone must start no
   motion; the junction issues `move` exactly once; S2's completion clears the
   instruction; a second junction without a new barcode does not repeat it. */
static void check_letter_maneuver(const char *id, const char *letter,
                                  navigation_command_t nav, vc_move_t move,
                                  const char *what)
{
    char name[16];
    char desc[112];

#define NAV_CHECK(sfx, cond, text)                                     \
    do                                                                 \
    {                                                                  \
        snprintf(name, sizeof(name), "%s%s", id, sfx);                 \
        snprintf(desc, sizeof(desc), "%s: %s", what, text);            \
        CHECK(name, cond, desc);                                       \
    } while (0)

    start_mission();
    tally_clear();
    feed_barcode(letter);
    NAV_CHECK("a", (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
                       (nav == g_sm.pending_nav) && (0 == turns_issued()) &&
                       (0 == g_tally[VC_MOVE_DRIVE_DIST]) &&
                       (0 == g_tally[VC_MOVE_STOP_CONFIRM]),
              "decoding alone stores the instruction and starts no motion");

    tally_clear();
    junction_to_execute();
    NAV_CHECK("b", (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
                       (1 == g_tally[VC_MOVE_STOP_CONFIRM]) &&
                       (1 == g_tally[move]) &&
                       ((VC_MOVE_DRIVE_DIST == move) ? (0 == turns_issued())
                                                     : (1 == turns_issued())),
              "stops at the junction, then issues the maneuver exactly once");
    if (VC_MOVE_U_TURN == move)
    {
        NAV_CHECK("c", has_move(VC_MOVE_U_TURN, 0, 0) &&
                           (VC_PIVOT_COUNTS(VC_UTURN_GATE_DEG) == g_sm.turn_gate),
                  "U-turn request is VC_MOVE_U_TURN (bridge sends 180 deg: S6-U01)");
    }
    else if (VC_MOVE_DRIVE_DIST == move)
    {
        NAV_CHECK("c", has_move(VC_MOVE_DRIVE_DIST, VC_JUNCTION_CLEAR_MM, 1),
                  "straight drives the clearing distance");
    }
    else
    {
        NAV_CHECK("c", has_move(move, VC_TURN_CMD_DEG, 1),
                  "turn request carries the encoder turn angle");
    }

    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    NAV_CHECK("d", (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
                       (NAV_NONE == g_sm.pending_nav) &&
                       ('\0' == g_sm.pending_barcode[0]),
              "completion clears the instruction and resumes line following");

    tally_clear();
    junction_to_execute();
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    NAV_CHECK("e", (VC_ST_FOLLOW_LINE == g_sm.st.state) && (0 == turns_issued()),
              "second junction without a new barcode makes no turn");
#undef NAV_CHECK
}

static void test_letter_navigation(void)
{
    /* Z is the production default (right). Left, U-turn and straight use the
       test-only mappings so the team's production table stays as set. */
    check_letter_maneuver("NAV-01", "TESTLEFT", NAV_LEFT, VC_MOVE_TURN_LEFT, "LEFT");
    check_letter_maneuver("NAV-02", "Z", NAV_RIGHT, VC_MOVE_TURN_RIGHT, "Z = RIGHT");
    check_letter_maneuver("NAV-03", "TESTUTURN", NAV_U_TURN, VC_MOVE_U_TURN, "U_TURN");
    check_letter_maneuver("NAV-04", "TESTSTRAIGHT", NAV_STRAIGHT,
                          VC_MOVE_DRIVE_DIST, "STRAIGHT");

    /* A: successful decode, goes straight through its NAV_NONE fallback */
    start_mission();
    tally_clear();
    feed_barcode("A");
    CHECK("NAV-05a", (1u == g_sm.st.barcodes_read) &&
                         (0 == strcmp("A", g_sm.st.last_barcode)) &&
                         (NAV_NONE == g_sm.pending_nav),
          "A is recorded as a successful decode (count 1, last = A)");
    junction_to_execute();
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("NAV-05b", (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
                         (0 == turns_issued()) && (1 == g_tally[VC_MOVE_DRIVE_DIST]) &&
                         (0 == strcmp("A", g_sm.st.last_barcode)),
          "A drives straight over the junction, no turn, still recorded as A");

    /* a missed read and a wrong letter are distinguishable from A */
    start_mission();
    feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
    feed(VC_SRC_LINE, VC_EV_BARCODE_FAILED, 0, 0);
    CHECK("NAV-06a", (0u == g_sm.st.barcodes_read) &&
                         ('\0' == g_sm.st.last_barcode[0]) &&
                         (NAV_NONE == g_sm.pending_nav) &&
                         (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
                         (VC_FC_BARCODE == g_sm.st.last_fault_code) &&
                         (VC_FAULT_NON_CRITICAL == g_sm.st.last_fault_sev),
          "a missed read records no letter (count 0) and reports VC_FC_BARCODE, "
          "non-critical; the run goes on");
    start_mission();
    feed_barcode("T"); /* T is what a backwards Z looks like */
    CHECK("NAV-06b", (1u == g_sm.st.barcodes_read) &&
                         (0 == strcmp("T", g_sm.st.last_barcode)) &&
                         (0 != strcmp("A", g_sm.st.last_barcode)) &&
                         (0 != strcmp("Z", g_sm.st.last_barcode)),
          "a wrong letter (T) is recorded as T, not as A or Z");

    /* Z followed by two junctions: only the first turns */
    start_mission();
    tally_clear();
    feed_barcode("Z");
    junction_to_execute();
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    junction_to_execute();
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("NAV-07", (1 == g_tally[VC_MOVE_TURN_RIGHT]) && (1 == turns_issued()) &&
                        (VC_ST_FOLLOW_LINE == g_sm.st.state),
          "Z then two junctions: exactly one right turn, the second goes straight");

    /* Z, then an operator stop (FAULT), Reset, Start: no stale right turn */
    start_mission();
    feed_barcode("Z");
    feed(VC_SRC_COMMS, VC_EV_CMD_STOP, 0, 0);
    feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
    CHECK("NAV-08a", (VC_ST_IDLE == g_sm.st.state) &&
                         (NAV_NONE == g_sm.pending_nav) &&
                         ('\0' == g_sm.pending_barcode[0]),
          "Z then stop then Reset: pending instruction cleared");
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
    feed(VC_SRC_LINE, VC_EV_READY, 0, 0);
    feed(VC_SRC_IMU, VC_EV_READY, 0, 0);
    feed(VC_SRC_OBST, VC_EV_READY, 0, 0);
    tally_clear();
    junction_to_execute();
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("NAV-08b", (VC_ST_FOLLOW_LINE == g_sm.st.state) && (0 == turns_issued()),
          "after Reset and Start the first junction goes straight");

    /* Z, then overturn (critical fault), Reset: same */
    start_mission();
    feed_barcode("Z");
    feed(VC_SRC_IMU, VC_EV_FAULT, VC_FAULT_CRITICAL, 0x4005);
    feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
    CHECK("NAV-09", (VC_ST_IDLE == g_sm.st.state) && (NAV_NONE == g_sm.pending_nav),
          "Z then overturn fault then Reset: pending instruction cleared");

    /* Z, then lift (end of run): no turn, instruction cleared */
    start_mission();
    feed_barcode("Z");
    feed_lift();
    CHECK("NAV-10", (VC_ST_MISSION_COMPLETE == g_sm.st.state) &&
                        (NAV_NONE == g_sm.pending_nav),
          "Z then lift: run ends, pending instruction cleared");

    /* lift at the junction wins over the pending turn, no turn issued */
    start_mission();
    feed_barcode("Z");
    tally_clear();
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed_lift();
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    CHECK("NAV-11", (VC_ST_MISSION_COMPLETE == g_sm.st.state) &&
                        (0 == turns_issued()),
          "lift between junction and turn: stop wins, no turn is issued");

    /* Z, lift (run ends), then Start without Reset: Start goes via IDLE,
       so the old Z must not turn at the first junction of the new run */
    start_mission();
    feed_barcode("Z");
    feed_lift();
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
    feed(VC_SRC_LINE, VC_EV_READY, 0, 0);
    feed(VC_SRC_IMU, VC_EV_READY, 0, 0);
    feed(VC_SRC_OBST, VC_EV_READY, 0, 0);
    tally_clear();
    junction_to_execute();
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("NAV-12", (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
                        (0 == turns_issued()) &&
                        (1 == g_tally[VC_MOVE_DRIVE_DIST]),
          "Z, lift, Start: the first junction of the new run goes straight");

    /* Reset while the run is still going is ignored (Reset only acts in
       IDLE, FAULT and MISSION_COMPLETE), so the pending Z still turns */
    start_mission();
    feed_barcode("Z");
    feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
    CHECK("NAV-13", (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
                        (NAV_RIGHT == g_sm.pending_nav),
          "Reset during a running mission changes nothing (existing rule)");

    /* The letter decides, not S3's legacy nav byte (S3_BC_CHAR_* are all
       off, but a stale p1 must not override the table either) */
    {
        vc_event_t event = { 0 };

        start_mission();
        event.src  = VC_SRC_LINE;
        event.type = VC_EV_BARCODE_DECODED;
        event.p1   = (int32_t)NAV_LEFT;
        event.barcode_text[0] = 'Z';
        vc_sm_handle(&g_sm, &event, g_now_ms, &g_actions);
        CHECK("NAV-14", (NAV_RIGHT == g_sm.pending_nav) &&
                            (0 == strcmp("Z", g_sm.st.last_barcode)),
              "decoded text Z with legacy p1 = LEFT: the table (RIGHT) wins");
    }

#if VC_TURN_END_ON_LINE
    /* Z ends on the new arm (LINE_FOUND past the gate) instead of S2's
       MANOEUVRE_DONE.  S2's late DONE then arrives in FOLLOW_LINE: it must
       not start anything, and the next junction goes straight. */
    {
        const uint32_t odo0 = 200000u;

        start_mission();
        feed(VC_SRC_VC, VC_EV_TICK, 0, (int32_t)odo0);
        feed_barcode("Z");
        tally_clear();
        junction_to_execute();
        line_at(g_sm.turn_odo0 + g_sm.turn_gate + 5u);
        CHECK("NAV-15a", (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
                             (NAV_NONE == g_sm.pending_nav) &&
                             (1 == g_tally[VC_MOVE_TURN_RIGHT]),
              "Z turn ended on the line: instruction cleared, following");
        tally_clear();
        feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
        CHECK("NAV-15b", (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
                             (0 == g_actions.n),
              "late S2 MANOEUVRE_DONE after a line-ended turn does nothing");
        junction_to_execute();
        feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
        CHECK("NAV-15c", (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
                             (0 == turns_issued()),
              "next junction after a line-ended Z goes straight");
    }
#endif
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

static void test_pause_resume(void)
{
    start_mission();
    feed(VC_SRC_COMMS, VC_EV_CMD_PAUSE, 0, 0);
    CHECK("S6-PA01",
          g_sm.st.paused && !g_sm.pause_stopped &&
              has_move(VC_MOVE_STOP_CONFIRM, 0, 0),
          "pause in FOLLOW_LINE requests a confirmed stop");
    feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_STEER, 200);
    CHECK("S6-PA02", g_sm.st.paused && (0 == g_actions.n),
          "paused controller rejects steering");
    feed(VC_SRC_COMMS, VC_EV_CMD_RESUME, 0, 0);
    CHECK("S6-PA03", g_sm.st.paused && (0 == g_actions.n),
          "resume before stop confirmation does not move");
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    feed(VC_SRC_COMMS, VC_EV_CMD_RESUME, 0, 0);
    CHECK("S6-PA04",
          !g_sm.st.paused && has_move(VC_MOVE_FORWARD, VC_SPEED_CRUISE_MMS, 1),
          "resume after confirmed stop restarts line following");

    start_mission();
    feed(VC_SRC_LINE, VC_EV_LINE_LOST, 1, 0);
    feed(VC_SRC_COMMS, VC_EV_CMD_PAUSE, 0, 0);
    CHECK("S6-PA05", !g_sm.st.paused && (VC_ST_LOST_LINE == g_sm.st.state) &&
          (0 == g_actions.n), "pause rejected during line recovery");

    start_mission();
    feed(VC_SRC_COMMS, VC_EV_CMD_PAUSE, 0, 0);
    feed(VC_SRC_OBST, VC_EV_OBSTACLE, 100, 0);
    CHECK("S6-PA06", (VC_ST_FAULT == g_sm.st.state) && !g_sm.st.paused &&
          has_move(VC_MOVE_SAFE_STOP, 0, 0),
          "obstacle while paused enters safe fault without avoidance movement");

    start_mission();
    feed(VC_SRC_COMMS, VC_EV_CMD_PAUSE, 0, 0);
    g_now_ms += VC_JUNCTION_STOP_MS;
    feed(VC_SRC_VC, VC_EV_TICK, 0, 0);
    CHECK("S6-PA07", (VC_ST_FAULT == g_sm.st.state) &&
          has_move(VC_MOVE_SAFE_STOP, 0, 0),
          "missing stop confirmation enters safe fault");

    start_mission();
    feed(VC_SRC_COMMS, VC_EV_CMD_PAUSE, 0, 0);
    feed(VC_SRC_IMU, VC_EV_LIFTED, 0, 0);
    feed(VC_SRC_COMMS, VC_EV_CMD_RESUME, 0, 0);
    CHECK("S6-PA08",
          (VC_ST_MISSION_COMPLETE == g_sm.st.state) && !g_sm.st.paused &&
              (0 == g_actions.n),
          "lift while paused ends the run; resume cannot restart it");

    start_mission();
    feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    feed(VC_SRC_OBST, VC_EV_AVOID_DONE, 0, 0);
    feed(VC_SRC_COMMS, VC_EV_CMD_PAUSE, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    feed(VC_SRC_COMMS, VC_EV_CMD_RESUME, 0, 0);
    CHECK("S6-PA09",
          !g_sm.st.paused && (0u != g_sm.rejoin_steer_left) &&
          has_move(VC_MOVE_SET_SPEED, VC_SPEED_SLOW_MMS, 1) &&
          !has_move(VC_MOVE_FORWARD, 0, 0) &&
          !has_move(VC_MOVE_SET_SPEED, VC_SPEED_CRUISE_MMS, 1),
          "resume during post-bypass handoff waits for slow fresh steering");
}

int main(void)
{
    test_startup();
    test_arbitration();
    test_barcode();
    test_junction_in_decode();
    test_turn_on_line();
    test_line_loss();
    test_obstacle();
    test_terrain();
    test_faults();
    test_lift();
    test_letter_config();
    test_letter_navigation();
    test_line_search();
    test_pause_resume();

    printf("[TEST] vehicle_controller host SUMMARY %d passed, %d failed\n", g_passed,
           g_failed);
    return (0 != g_failed) ? 1 : 0;
}

/*** end of file ***/
