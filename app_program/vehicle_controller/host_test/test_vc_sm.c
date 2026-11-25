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
#include "../vc_barcode_map.h"

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
static int32_t g_overflows; /* action lists that did not fit: a bug */

static void tally_clear(void)
{
    memset(g_tally, 0, sizeof(g_tally));
}

static void tally_add(void)
{
    if (g_actions.overflow)
    {
        g_overflows++;
    }
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

static void feed_barcode(const char * p_text)
{
    vc_event_t event = { 0 };
    size_t     idx2;

    event.src  = VC_SRC_LINE;
    event.type = VC_EV_BARCODE_DECODED;
    for (idx2 = 0u;
         (idx2 < sizeof(event.barcode_text) - 1u) && (p_text[idx2] != '\0');
         idx2++)
    {
        event.barcode_text[idx2] = p_text[idx2];
    }
    event.barcode_text[idx2] = '\0';
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
    CHECK("S6-SAFE01",
          (VC_ST_IDLE == g_sm.st.state) && has_move(VC_MOVE_RESET, 0, 0),
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
    CHECK("S6-SAFE02",
          (VC_ST_FAULT == g_sm.st.state) && has_move(VC_MOVE_SAFE_STOP, 0, 0),
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
    CHECK("S6-H45",
          (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
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
          "after S2 confirms stopped, unknown barcode defaults to straight "
          "clear");
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("S6-H13a",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
              (NAV_NONE == g_sm.pending_nav) &&
              ('\0' == g_sm.pending_barcode[0]),
          "junction action complete clears buffered barcode");

    /* Full-string rules are exercised with test-only mappings. */
    feed_barcode("TESTLEFT");
    CHECK("S6-H13c", NAV_LEFT == g_sm.pending_nav,
          "full barcode string maps to LEFT; matching is not first-character "
          "based");
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    junction_advance();
    CHECK("S6-H13d",
          (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
              has_move(VC_MOVE_TURN_LEFT, VC_TURN_CMD_DEG, 1),
          "configured full-string rule executes only after stop confirmation");
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);

    /* Dashboard "At every junction: Right" (vc_bridge_s3.c): an action-only
       BARCODE_DECODED (p1 = nav, no text) just before the JUNCTION. */
    feed(VC_SRC_LINE, VC_EV_BARCODE_DECODED, (int32_t)NAV_RIGHT, 0);
    CHECK("S6-H13e",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
              (NAV_RIGHT == g_sm.pending_nav),
          "action-only event sets RIGHT and keeps following");
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    junction_advance();
    CHECK("S6-H13f",
          (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
              has_move(VC_MOVE_TURN_RIGHT, VC_TURN_CMD_DEG, 1),
          "junction after it: stop, then turn RIGHT");
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("S6-H13g",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
              (NAV_NONE == g_sm.pending_nav),
          "after the turn: follow the line again, action cleared");

    /* A junction without any barcode follows the same safe straight path. */
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    CHECK("S6-H14", VC_ST_JUNCTION_STOP == g_sm.st.state,
          "junction without barcode still stops first");
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    CHECK("S6-H14a",
          (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
              has_move(VC_MOVE_DRIVE_DIST, VC_JUNCTION_CLEAR_MM, 1),
          "no barcode -> move straight far enough to clear junction");
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("S6-H14b", (VC_ST_FOLLOW_LINE == g_sm.st.state),
          "resume line following after junction clear completes");

    /* A stop that never completes must fail safe rather than turning. */
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    advance(VC_JUNCTION_STOP_MS + 20);
    CHECK("S6-H14c",
          (VC_ST_FAULT == g_sm.st.state) &&
              (VC_FC_JUNCTION_STOP == g_sm.st.last_fault_code),
          "stop confirmation timeout -> critical safe stop");

    start_mission();
    feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
    {
        int32_t kdx;
        bool    b_ok = (VC_ST_DECODE_BARCODE == g_sm.st.state);
        for (kdx = 0; kdx < VC_BARCODE_MAX_RETRY; kdx++)
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
    CHECK("S6-H18",
          (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
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

    CHECK("S6-H50",
          (gate > 400u) && (gate < 520u) && (VC_PIVOT_COUNTS(90) > 650u) &&
              (VC_PIVOT_COUNTS(90) < 740u),
          "pivot counts: 60 deg ~461, 90 deg ~692 (track 130, 692 ticks/rev)");

    start_mission();
    feed(VC_SRC_VC, VC_EV_TICK, 0, (int32_t)odo0);
    feed(VC_SRC_LINE, VC_EV_BARCODE_DECODED, (int32_t)NAV_LEFT, 0);
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    junction_advance();
    CHECK("S6-H52",
          (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
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
    CHECK("S6-H54",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
              (NAV_NONE == g_sm.pending_nav) && (0u == g_sm.turn_gate) &&
              has_move(VC_MOVE_FORWARD, VC_SPEED_CRUISE_MMS, 1),
          "line after the gate ends the turn: follow the new arm");

    /* no line by the max angle: S2's MANOEUVRE_DONE ends it as before */
    feed(VC_SRC_LINE, VC_EV_BARCODE_DECODED, (int32_t)NAV_RIGHT, 0);
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    junction_advance();
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("S6-H55",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) && (0u == g_sm.turn_gate),
          "no line seen: turn ends at the max angle, follow as before");

    /* U-turn: stop, then the fast pivot (the rest: test_uturn) */
    tally_clear();
    feed(VC_SRC_LINE, VC_EV_BARCODE_DECODED, (int32_t)NAV_U_TURN, 0);
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    CHECK("S6-H56",
          (VC_ST_JUNCTION_STOP == g_sm.st.state) &&
              (0 == g_tally[VC_MOVE_U_TURN]),
          "U-turn: the robot stops at the junction before it pivots");
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    CHECK("S6-H57",
          (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
              has_move(VC_MOVE_U_TURN, VC_UTURN_FAST_DEG, 1) &&
              (1 == g_tally[VC_MOVE_U_TURN]) &&
              (0 == g_tally[VC_MOVE_DRIVE_DIST]) && (0u == g_sm.turn_gate) &&
              (VC_EXEC_UTURN == g_sm.exec_phase) &&
              has_request(VC_SRC_LINE, VC_REQ_LINE_WATCH),
          "U-turn: fast pivot in place to VC_UTURN_FAST_DEG (no advance), line "
          "watch on");
    {
        uint32_t u0_val = g_sm.odo; /* odometer as the pivot starts */

        line_at(u0_val + VC_PIVOT_COUNTS(90));
        line_at(u0_val + VC_PIVOT_COUNTS(170));
    }
    CHECK("S6-H58",
          (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
              (VC_EXEC_UTURN == g_sm.exec_phase) &&
              (NAV_U_TURN == g_sm.pending_nav),
          "U-turn: LINE_FOUND at 90 and 170 deg does not end the pivot");
    feed(VC_SRC_COMMS, VC_EV_CMD_STOP, 0, 0);
    feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
    start_mission();

    /* STRAIGHT: no watch, LINE_FOUND does not cut the clearing drive */
    feed(VC_SRC_LINE, VC_EV_BARCODE_DECODED, (int32_t)NAV_STRAIGHT, 0);
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    line_at(g_sm.odo + 5000u);
    CHECK("S6-H59",
          (VC_ST_EXECUTE_ACTION == g_sm.st.state) && (0u == g_sm.turn_gate),
          "STRAIGHT: line events do not end the clearing drive");
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
#endif
}

static void test_junction_in_decode(void)
{
    start_mission();
    feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    CHECK("S6-H42",
          (VC_ST_JUNCTION_STOP == g_sm.st.state) &&
              has_move(VC_MOVE_STOP_CONFIRM, 0, 1),
          "junction during DECODE_BARCODE is not dropped: stop first");
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("S6-H43", VC_ST_FOLLOW_LINE == g_sm.st.state,
          "then straight across and follow again");

    /* one failed read: no waiting for a retry that cannot happen */
    feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
    feed(VC_SRC_LINE, VC_EV_BARCODE_FAILED, 0, 0);
    CHECK("S6-H44",
          (VC_BARCODE_MAX_RETRY > 0) || (VC_ST_FOLLOW_LINE == g_sm.st.state),
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
    CHECK("S6-H24",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
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
          !b_early_cruise &&
              has_move(VC_MOVE_SET_SPEED, VC_SPEED_CRUISE_MMS, 1),
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
    CHECK("S6-OBS04",
          VC_ST_FAULT == g_sm.st.state && has_move(VC_MOVE_SAFE_STOP, 0, 1),
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
              (LIFT_CODE == g_sm.st.last_fault_code) &&
              (VC_SRC_IMU == g_sm.st.last_fault_src) &&
              (VC_FAULT_NON_CRITICAL == g_sm.st.last_fault_sev),
          "lift in FOLLOW_LINE -> MISSION_COMPLETE, stop all, cause IMU 0x4005 "
          "non-critical");
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
    CHECK("VC-L08", !any_move(),
          "after lift: set-down line follow request refused");
    feed(VC_SRC_LINE, VC_EV_LINE_FOUND, 0, 0);
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
    feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
    feed(VC_SRC_IMU, VC_EV_HUMP_START, 0, 0);
    advance(5000u);
    CHECK("VC-L09", VC_ST_MISSION_COMPLETE == g_sm.st.state,
          "after lift: line, junction, obstacle, hump events and time change "
          "nothing");
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    CHECK("VC-L10", (VC_ST_MISSION_COMPLETE == g_sm.st.state) && !any_move(),
          "after lift: nothing but Start or Reset leaves MISSION_COMPLETE");
    feed_lift();
    CHECK("VC-L11",
          (VC_ST_MISSION_COMPLETE == g_sm.st.state) && (0 == g_actions.n),
          "second lift while complete: no change, no action");
    feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
    CHECK("VC-L12",
          (VC_ST_IDLE == g_sm.st.state) && has_move(VC_MOVE_RESET, 0, 0),
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
    CHECK("VC-S03", VC_ST_IDLE == g_sm.st.state, "lift then Reset still works");
    feed_lift();
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    feed_lift();
    CHECK("VC-S04", VC_ST_FAULT == g_sm.st.state,
          "lift again during the new self check -> FAULT 0x4005");
    /* Start in IDLE clears S2's latch first (lift while idle, 2026-11-24) */
    g_now_ms = 0u;
    vc_sm_init(&g_sm, g_now_ms);
    feed_lift(); /* robot picked up and placed on the track while idle */
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    {
        int32_t reset_at = -1;
        int32_t check_at = -1;

        for (int32_t idx = 0; idx < g_actions.n; idx++)
        {
            if ((VC_ACT_MOVE == g_actions.a[idx].kind) &&
                (g_actions.a[idx].p1 == (int32_t)VC_MOVE_RESET) &&
                (reset_at < 0))
            {
                reset_at = idx;
            }
            if ((VC_ACT_REQUEST == g_actions.a[idx].kind) &&
                (g_actions.a[idx].p1 == (int32_t)VC_REQ_SELF_CHECK))
            {
                check_at = idx;
            }
        }
        CHECK("VC-S06",
              (VC_ST_MISSION_START == g_sm.st.state) && (reset_at >= 0) &&
                  (check_at > reset_at),
              "Start in IDLE: S2 latch reset is sent before the self check");
    }
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
    feed(VC_SRC_LINE, VC_EV_READY, 0, 0);
    advance(VC_SELF_CHECK_MS);
    CHECK("VC-S07", VC_ST_FOLLOW_LINE == g_sm.st.state,
          "Start in IDLE after an idle lift: mission runs once S2 is ready");
    g_now_ms = 0u;
    vc_sm_init(&g_sm, g_now_ms);
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    advance(VC_SELF_CHECK_MS + 100u);
    CHECK("VC-S08",
          (VC_ST_FAULT == g_sm.st.state) &&
              (VC_FC_SELF_CHECK == g_sm.st.last_fault_code),
          "S2 still not ready after Start (robot still lifted): FAULT 0x0002 "
          "as before");

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
    int32_t not_none  = 0;
    char    text[2]   = { 'A', '\0' };

    for (uint32_t idx2 = 0u; idx2 < VC_BARCODE_LETTER_COUNT; idx2++)
    {
        text[0] = (char)('A' + (int)idx2);
        /* the active map holds the four actions: a NONE row reads STRAIGHT */
        if ((vc_bcmap_wire_from_nav(vc_barcode_lookup(text)) !=
             vc_bcmap_wire_from_nav(expect[idx2])) ||
            (NAV_NONE == vc_barcode_lookup(text)))
        {
            all_match = 0;
        }
        if (NAV_NONE != expect[idx2])
        {
            not_none++;
        }
    }
    CHECK("VC-C01", all_match,
          "factory map: every letter A..Z returns the action set in "
          "vc_barcode_config.h (NAV_NONE rows as STRAIGHT)");
    CHECK("VC-C02",
          (NAV_LEFT == vc_barcode_lookup("H")) &&
              (NAV_RIGHT == vc_barcode_lookup("Z")) && (2 == not_none),
          "as shipped: H = LEFT, Z = RIGHT, every other letter NONE (straight) "
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

/* ------------------------------------------------------------------ */
/*  U-turn: fast pivot, slow approach, line-confirmed end               */
/* ------------------------------------------------------------------ */

static void junction_to_execute(void);

/* Simulated S2 odometer during a U-turn: L + R counts, both wheels add */
static uint32_t g_ut_odo;
static uint32_t g_ut_sample_ms; /* time stamp of the next S3 sample */

/* heading in S2 counts since the U-turn began (the controller's own view) */
#define UT_DEG(d) ((int32_t)VC_PIVOT_COUNTS(d))

static int32_t count_moves(vc_move_t move, int32_t value)
{
    int32_t count = 0;

    for (int32_t idx = 0; idx < g_actions.n; idx++)
    {
        if ((VC_ACT_MOVE == g_actions.a[idx].kind) &&
            (g_actions.a[idx].p1 == (int32_t)move) &&
            (g_actions.a[idx].p2 == value))
        {
            count++;
        }
    }
    return count;
}

/* Traces since the last ut_log_clear(), so a step's lines can be checked
   after several events */
static int32_t g_ut_log[64][2];
static int32_t g_ut_log_n;

static void ut_log_clear(void)
{
    g_ut_log_n = 0;
}

static void ut_log_add(void)
{
    for (int32_t idx = 0; idx < g_actions.n; idx++)
    {
        if ((VC_ACT_TRACE == g_actions.a[idx].kind) && (g_ut_log_n < 64))
        {
            g_ut_log[g_ut_log_n][0] = g_actions.a[idx].p1;
            g_ut_log[g_ut_log_n][1] = g_actions.a[idx].rsv;
            g_ut_log_n++;
        }
    }
}

static int32_t logged(vc_ut_trace_t what, int32_t detail, int32_t check)
{
    for (int32_t idx = 0; idx < g_ut_log_n; idx++)
    {
        if ((g_ut_log[idx][0] == (int32_t)what) &&
            ((0 == check) || (g_ut_log[idx][1] == detail)))
        {
            return 1;
        }
    }
    return 0;
}

/* S2 has turned `counts` further (either way: the encoders only count) */
static void ut_turn(uint32_t counts)
{
    g_ut_odo += counts;
}

/* One fresh S3 sample (C / R bits) at the current odometer */
static void ut_sample_now(int32_t mask)
{
    vc_event_t event = { 0 };

    g_now_ms += 10u;
    g_ut_sample_ms = g_now_ms;
    event.src      = VC_SRC_LINE;
    event.type     = VC_EV_LINE_SAMPLE;
    event.p1       = mask;
    event.p2       = (int32_t)g_ut_odo;
    event.time_ms  = g_ut_sample_ms;
    vc_sm_handle(&g_sm, &event, g_now_ms, &g_actions);
    tally_add();
    ut_log_add();
}

/* A sample taken `age_ms` before now (S3 published it before the step) */
static void ut_sample_old(int32_t mask, uint32_t age_ms)
{
    vc_event_t event = { 0 };

    event.src     = VC_SRC_LINE;
    event.type    = VC_EV_LINE_SAMPLE;
    event.p1      = mask;
    event.p2      = (int32_t)g_ut_odo;
    event.time_ms = g_now_ms - age_ms;
    vc_sm_handle(&g_sm, &event, g_now_ms, &g_actions);
    tally_add();
    ut_log_add();
}

/* Turn on in `step`-count samples showing `mask` until `counts` more */
static void ut_turn_seeing(uint32_t counts, uint32_t step, int32_t mask)
{
    for (uint32_t done = 0u; done < counts; done += step)
    {
        ut_turn(step);
        ut_sample_now(mask);
    }
}

static void ut_tick(void)
{
    g_now_ms += HOST_TICK_MS;
    feed(VC_SRC_VC, VC_EV_TICK, 0, (int32_t)g_ut_odo);
    ut_log_add();
}

static void ut_motion(vc_event_type_t type)
{
    feed(VC_SRC_MOTION, type, 1, (int32_t)g_ut_odo);
    ut_log_add();
}

/* Mission running, U-turn barcode read, junction, stopped: the fast pivot
   has just been asked for.  Odometer starts at a high value on purpose. */
static void ut_begin(void)
{
    start_mission();
    g_ut_odo = 50000u;
    feed(VC_SRC_VC, VC_EV_TICK, 0, (int32_t)g_ut_odo);
    feed(VC_SRC_LINE, VC_EV_BARCODE_DECODED, (int32_t)NAV_U_TURN, 0);
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    tally_clear();
    ut_log_clear();
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    ut_log_add();
}

/* the fast pivot to VC_UTURN_FAST_DEG, then S2 says done */
static void ut_fast(void)
{
    ut_turn_seeing((uint32_t)UT_DEG(VC_UTURN_FAST_DEG), 8u, 0);
    ut_motion(VC_EV_MANOEUVRE_DONE);
}

/* A clean turn: the line comes under C, the robot brakes, C lets it go
   while braking and R never sees it (the tape ends between them) */
static void uturn_clean(void)
{
    ut_fast();
    ut_turn_seeing((uint32_t)UT_DEG(30), 8u, 0); /* white, approaching */
    ut_turn_seeing(16u, 8u, VC_LINE_C);          /* C on the tape */
    ut_turn_seeing(24u, 8u, VC_LINE_C);          /* braking, still on C */
    ut_turn_seeing(16u, 8u, 0);                  /* C lets it go */
    ut_motion(VC_EV_MOTION_STOPPED);
    for (int32_t count = 0; count < VC_UTURN_VERIFY_SAMPLES; count++)
    {
        ut_sample_now(0); /* both white, between C and R */
    }
}

static void test_uturn(void)
{
    int32_t h_val;

    /* settings */
    CHECK("UT-01",
          (VC_UTURN_FAST_DEG >= 90) && (VC_UTURN_FAST_DEG < VC_UTURN_CMD_DEG) &&
              (VC_UTURN_SLOW_MMS < 120) && (VC_TURN_GATE_DEG == 60) &&
              (VC_TURN_MAX_DEG == 135) && (VC_TURN_CMD_DEG == 135),
          "U-turn settings: fast part past 90 deg, short of the expected "
          "heading, slow pivot slower than the turn speed; LEFT/RIGHT "
          "tuning unchanged");

    /* 1. clean turn: no correction */
    ut_begin();
    CHECK("UT-02",
          has_move(VC_MOVE_U_TURN, VC_UTURN_FAST_DEG, 1) &&
              logged(VC_UT_START, VC_UTURN_FAST_DEG, 1),
          "start: fast pivot asked for VC_UTURN_FAST_DEG and logged");
    ut_fast();
    CHECK("UT-03",
          (VC_EXEC_UT_SWEEP == g_sm.exec_phase) &&
              has_move(VC_MOVE_PIVOT, VC_UTURN_SLOW_MMS, 1) &&
              logged(VC_UT_FAST_DONE, 0, 0) &&
              logged(VC_UT_SWEEP, VC_UT_APPROACH, 1),
          "fast pivot done: slow pivot left at VC_UTURN_SLOW_MMS (approach)");
    tally_clear();
    for (int32_t count = 0; count < 10; count++)
    {
        ut_tick();
    }
    CHECK("UT-04",
          (g_tally[VC_MOVE_PIVOT] >= 1) && (g_tally[VC_MOVE_PIVOT] <= 3) &&
              (VC_EXEC_UT_SWEEP == g_sm.exec_phase),
          "slow pivot is refreshed about every VC_UTURN_REFRESH_MS (S2 drive "
          "timeout)");
    ut_turn_seeing((uint32_t)UT_DEG(25), 8u, 0);
    tally_clear();
    ut_log_clear();
    ut_turn_seeing(8u, 8u, VC_LINE_C);
    CHECK("UT-05",
          (VC_EXEC_UT_SWEEP == g_sm.exec_phase) &&
              (0 == g_tally[VC_MOVE_STOP_CONFIRM]),
          "one black sample is not yet a sighting");
    h_val = g_sm.ut.heading + 8;
    ut_turn_seeing(8u, 8u, VC_LINE_C);
    CHECK(
        "UT-06",
        (VC_EXEC_UT_BRAKE == g_sm.exec_phase) &&
            (1 == g_tally[VC_MOVE_STOP_CONFIRM]) &&
            logged(VC_UT_SIGHT, VC_LINE_C, 1) && logged(VC_UT_BRAKE, 1, 1) &&
            (VC_LINE_C == g_sm.ut.sight) && (h_val == g_sm.ut.sight_h),
        "two black samples on C: sighting logged with its heading, stop asked");
    ut_turn_seeing(24u, 8u, VC_LINE_C);
    ut_turn_seeing(16u, 8u, 0);
    ut_log_clear();
    ut_motion(VC_EV_MOTION_STOPPED);
    CHECK("UT-07",
          (VC_EXEC_UT_VERIFY == g_sm.exec_phase) &&
              logged(VC_UT_SETTLED, 40, 1),
          "S2 confirms the stop: settled counts after the request logged (40)");
    tally_clear();
    ut_sample_now(0);
    ut_sample_now(0);
    CHECK("UT-08", VC_EXEC_UT_VERIFY == g_sm.exec_phase,
          "alignment needs VC_UTURN_VERIFY_SAMPLES fresh samples");
    ut_sample_now(0);
    CHECK("UT-09",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
              logged(VC_UT_VERDICT, VC_UT_V_BETWEEN, 1) &&
              logged(VC_UT_DONE, 0, 0) && (0 == g_tally[VC_MOVE_PIVOT]) &&
              (0 == g_tally[VC_MOVE_DRIVE_DIST]) &&
              has_move(VC_MOVE_FORWARD, VC_SPEED_SLOW_MMS, 1) &&
              has_move(VC_MOVE_SET_SPEED, VC_SPEED_SLOW_MMS, 1) &&
              !has_move(VC_MOVE_FORWARD, VC_SPEED_CRUISE_MMS, 1) &&
              (VC_REJOIN_STEER_SAMPLES == g_sm.rejoin_steer_left) &&
              (NAV_NONE == g_sm.pending_nav) &&
              has_request(VC_SRC_LINE, VC_REQ_LINE_FOLLOW),
          "clean turn: C let the tape go, R never saw it = between C and R; "
          "no correction, no straight drive, slow line following");
    tally_clear();
    for (int32_t count = 0; count < VC_REJOIN_STEER_SAMPLES; count++)
    {
        feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_STEER, 0);
    }
    CHECK("UT-10",
          has_move(VC_MOVE_SET_SPEED, VC_SPEED_CRUISE_MMS, 1) &&
              (0u == g_sm.rejoin_steer_left),
          "after VC_REJOIN_STEER_SAMPLES fresh steering samples: cruise");
    tally_clear();
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    CHECK("UT-11",
          (0 == g_tally[VC_MOVE_U_TURN]) &&
              has_move(VC_MOVE_DRIVE_DIST, VC_JUNCTION_CLEAR_MM, 1),
          "the U-turn is used once: the next junction goes straight");

    /* 2. early sightings (the junction, its 90 deg side arm) are ignored */
    ut_begin();
    tally_clear();
    ut_turn_seeing((uint32_t)UT_DEG(10), 8u, VC_LINE_C | VC_LINE_R); /* crossing
                                                                      */
    ut_turn_seeing((uint32_t)UT_DEG(55), 8u, 0);
    ut_turn_seeing((uint32_t)UT_DEG(20), 8u, VC_LINE_C); /* side arm */
    feed(VC_SRC_LINE, VC_EV_LINE_FOUND, 0, (int32_t)g_ut_odo);
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    CHECK("UT-12",
          (VC_EXEC_UTURN == g_sm.exec_phase) &&
              (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
              (0 == g_tally[VC_MOVE_STOP_CONFIRM]) &&
              (0 == g_tally[VC_MOVE_PIVOT]) &&
              logged(VC_UT_EARLY, VC_LINE_C | VC_LINE_R, 1) &&
              (0u == g_sm.ut.sight),
          "line on the crossing and on the 90 deg arm during the fast pivot: "
          "logged once, ignored, nothing stops or ends the U-turn");

    /* 3. overshoot: the line passes C and R while braking -> pivot back */
    ut_begin();
    ut_fast();
    ut_turn_seeing((uint32_t)UT_DEG(30), 8u, 0);
    ut_turn_seeing(16u, 8u, VC_LINE_C); /* sighting, stop asked */
    h_val = g_sm.ut.sight_h;
    ut_turn_seeing(48u, 8u, 0);         /* between, still turning */
    ut_turn_seeing(48u, 8u, VC_LINE_R); /* under R */
    ut_turn_seeing(40u, 8u, 0);         /* out past R */
    ut_motion(VC_EV_MOTION_STOPPED);
    tally_clear();
    ut_log_clear();
    ut_sample_now(0);
    ut_sample_now(0);
    ut_sample_now(0);
    CHECK("UT-13",
          (VC_EXEC_UT_SWEEP == g_sm.exec_phase) &&
              logged(VC_UT_VERDICT, VC_UT_V_PASSED, 1) &&
              logged(VC_UT_SWEEP, -(int32_t)VC_UT_CORRECT, 1) &&
              has_move(VC_MOVE_PIVOT, -VC_UTURN_SLOW_MMS, 1) &&
              (g_sm.ut.bound == h_val - UT_DEG(VC_UTURN_CORRECT_DEG)) &&
              (VC_ST_EXECUTE_ACTION == g_sm.st.state),
          "overshoot: R let the line go = turned past it; pivot back slowly "
          "(right), bounded at the sighting minus VC_UTURN_CORRECT_DEG");
    tally_clear();
    ut_log_clear();
    ut_motion(VC_EV_MANOEUVRE_DONE); /* late completion of the fast pivot */
    CHECK("UT-14",
          (VC_EXEC_UT_SWEEP == g_sm.exec_phase) &&
              (VC_ST_EXECUTE_ACTION == g_sm.st.state) && (0 == g_actions.n),
          "a late MANOEUVRE_DONE does not finish the corrective pivot");
    ut_turn_seeing(40u, 8u, 0);
    tally_clear();
    ut_turn_seeing(16u, 8u, VC_LINE_R); /* the line comes back under R */
    CHECK("UT-15",
          (VC_EXEC_UT_BRAKE == g_sm.exec_phase) &&
              (1 == g_tally[VC_MOVE_STOP_CONFIRM]) &&
              (VC_LINE_R == g_sm.ut.sight) && (g_sm.ut.heading > g_sm.ut.bound),
          "correction: R sees the line, stop asked, heading counted backwards");
    ut_motion(VC_EV_MOTION_STOPPED);
    tally_clear();
    ut_sample_now(VC_LINE_R);
    ut_sample_now(VC_LINE_R);
    ut_sample_now(VC_LINE_R);
    CHECK("UT-16",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
              has_move(VC_MOVE_FORWARD, VC_SPEED_SLOW_MMS, 1) &&
              (NAV_NONE == g_sm.pending_nav),
          "R on the tape after the correction: aligned, slow line following");

    /* 4. undershoot: the fast pivot stops short; the approach finds the
          line ahead, past the expected heading, without turning back */
    ut_begin();
    ut_fast();
    tally_clear();
    ut_turn_seeing((uint32_t)UT_DEG(VC_UTURN_APPROACH_DEG + 8), 8u, 0);
    CHECK("UT-17", VC_EXEC_UT_SWEEP == g_sm.exec_phase,
          "approach goes on past the expected heading (inside the sector)");
    ut_turn_seeing(16u, 8u, VC_LINE_C);
    ut_turn_seeing(16u, 8u, VC_LINE_C);
    ut_motion(VC_EV_MOTION_STOPPED);
    ut_sample_now(VC_LINE_C);
    ut_sample_now(VC_LINE_C);
    ut_sample_now(VC_LINE_C);
    CHECK("UT-18",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
              (0 == count_moves(VC_MOVE_PIVOT, -VC_UTURN_SLOW_MMS)),
          "undershoot: line found ahead under C, aligned, never pivoted back");

    /* 5. stale samples and brief noise do not confirm alignment */
    ut_begin();
    ut_fast();
    ut_turn_seeing((uint32_t)UT_DEG(20), 8u, 0);
    tally_clear();
    ut_sample_old(VC_LINE_C, 1000u); /* before the sweep began */
    ut_sample_old(VC_LINE_C, 990u);
    ut_turn(8u);
    ut_sample_now(VC_LINE_C);
    ut_turn(8u);
    ut_sample_now(0);
    ut_turn(8u);
    ut_sample_now(VC_LINE_R);
    ut_turn(8u);
    ut_sample_now(0);
    CHECK("UT-19",
          (VC_EXEC_UT_SWEEP == g_sm.exec_phase) &&
              (0 == g_tally[VC_MOVE_STOP_CONFIRM]) && (0u == g_sm.ut.sight),
          "samples older than the sweep, and single black blips, are no "
          "sighting");
    ut_turn_seeing(16u, 8u, VC_LINE_C);
    ut_turn_seeing(16u, 8u, VC_LINE_C);
    ut_motion(VC_EV_MOTION_STOPPED);
    tally_clear();
    ut_sample_old(VC_LINE_C, 30u); /* taken before the wheels stopped */
    ut_sample_old(VC_LINE_C, 20u);
    ut_sample_old(VC_LINE_C, 10u);
    CHECK("UT-20", VC_EXEC_UT_VERIFY == g_sm.exec_phase,
          "samples from before the stop confirmation do not confirm alignment");
    ut_sample_now(VC_LINE_C);
    ut_sample_now(VC_LINE_C);
    ut_sample_now(0); /* C goes white: still deciding (unsure) */
    ut_sample_now(VC_LINE_C);
    ut_sample_now(0);
    CHECK("UT-21",
          (VC_EXEC_UT_VERIFY == g_sm.exec_phase) &&
              (VC_ST_EXECUTE_ACTION == g_sm.st.state),
          "flickering fresh samples do not confirm alignment");
    ut_sample_now(VC_LINE_C);
    ut_sample_now(VC_LINE_C);
    ut_sample_now(VC_LINE_C);
    CHECK("UT-22", VC_ST_FOLLOW_LINE == g_sm.st.state,
          "three agreeing fresh samples do");

    /* 6. missing tape: approach to the sector edge, search back, then a
          bounded stop with line recovery failure */
    ut_begin();
    ut_fast();
    ut_log_clear();
    ut_turn_seeing(
        (uint32_t)UT_DEG(VC_UTURN_APPROACH_DEG + VC_UTURN_SECTOR_DEG) + 16u, 8u,
        0);
    CHECK("UT-23",
          (VC_EXEC_UT_BRAKE == g_sm.exec_phase) && logged(VC_UT_BRAKE, 2, 1),
          "no line by VC_UTURN_CMD_DEG + VC_UTURN_SECTOR_DEG: stop (bound)");
    ut_motion(VC_EV_MOTION_STOPPED);
    tally_clear();
    ut_sample_now(0);
    ut_sample_now(0);
    ut_sample_now(0);
    CHECK("UT-24",
          (VC_EXEC_UT_SWEEP == g_sm.exec_phase) &&
              has_move(VC_MOVE_PIVOT, -VC_UTURN_SLOW_MMS, 1) &&
              logged(VC_UT_VERDICT, VC_UT_V_NONE, 1) &&
              logged(VC_UT_SWEEP, -(int32_t)VC_UT_SEARCH, 1) &&
              (g_sm.ut.bound ==
               UT_DEG(VC_UTURN_CMD_DEG) - UT_DEG(VC_UTURN_SECTOR_DEG)),
          "then search back across the expected heading to its other side");
    {
        int32_t guard = 0;

        /* nothing anywhere: every sweep runs to its bound */
        while ((VC_ST_EXECUTE_ACTION == g_sm.st.state) && (guard < 2000))
        {
            guard++;
            if (VC_EXEC_UT_SWEEP == g_sm.exec_phase)
            {
                ut_turn(8u);
                ut_sample_now(0);
            }
            else if (VC_EXEC_UT_BRAKE == g_sm.exec_phase)
            {
                ut_motion(VC_EV_MOTION_STOPPED);
            }
            else
            {
                ut_sample_now(0);
            }
        }
    }
    CHECK("UT-25",
          (VC_ST_FAULT == g_sm.st.state) &&
              (VC_FC_LINE_RECOVERY == g_sm.st.last_fault_code) &&
              (VC_UTURN_MAX_SWEEPS == g_sm.ut.sweeps) &&
              has_move(VC_MOVE_SAFE_STOP, 0, 0) && logged(VC_UT_FAIL, 1, 1) &&
              (NAV_NONE == g_sm.pending_nav),
          "missing tape: bounded sweeps, then FAULT line recovery 0x0003, "
          "motors stopped");
    tally_clear();
    for (int32_t count = 0; count < 20; count++)
    {
        ut_tick();
    }
    CHECK("UT-26", 0 == g_tally[VC_MOVE_PIVOT], "no pivot after the failure");

    /* 7. time bound: S2 never finishes the fast pivot */
    ut_begin();
    ut_log_clear();
    for (uint32_t t_val = 0u; t_val <= VC_UTURN_MS; t_val += HOST_TICK_MS)
    {
        ut_tick();
    }
    CHECK("UT-27",
          (VC_ST_FAULT == g_sm.st.state) &&
              (VC_FC_LINE_RECOVERY == g_sm.st.last_fault_code) &&
              logged(VC_UT_FAIL, 4, 1),
          "a U-turn that never ends stops at VC_UTURN_MS with line recovery "
          "failure");

    /* 8. S2 never confirms the stop */
    ut_begin();
    ut_fast();
    ut_turn_seeing(16u, 8u, VC_LINE_C);
    ut_log_clear();
    for (uint32_t t_val = 0u; t_val <= VC_UTURN_SETTLE_MS;
         t_val += HOST_TICK_MS)
    {
        ut_tick();
    }
    CHECK("UT-28", (VC_ST_FAULT == g_sm.st.state) && logged(VC_UT_FAIL, 2, 1),
          "no stop confirmation within VC_UTURN_SETTLE_MS: FAULT, not a "
          "drive-off");

    /* 9. Stop, lift and obstacle interrupt the recovery */
    ut_begin();
    ut_fast();
    tally_clear();
    feed(VC_SRC_COMMS, VC_EV_CMD_STOP, 0, 0);
    CHECK("UT-29",
          (VC_ST_FAULT == g_sm.st.state) &&
              (VC_FC_OPERATOR_STOP == g_sm.st.last_fault_code) &&
              has_move(VC_MOVE_SAFE_STOP, 0, 0),
          "Stop during the slow approach: FAULT operator stop, safe stop");
    tally_clear();
    ut_tick();
    ut_turn_seeing(16u, 8u, VC_LINE_C);
    CHECK("UT-30",
          (0 == g_tally[VC_MOVE_PIVOT]) && (0 == g_tally[VC_MOVE_STOP_CONFIRM]),
          "after Stop: no pivot refresh, line samples do nothing");

    ut_begin();
    ut_fast();
    ut_turn_seeing((uint32_t)UT_DEG(30), 8u, 0);
    ut_turn_seeing(16u, 8u, VC_LINE_C);
    ut_turn_seeing(40u, 8u, VC_LINE_R);
    ut_turn_seeing(40u, 8u, 0);
    ut_motion(VC_EV_MOTION_STOPPED);
    ut_sample_now(0);
    ut_sample_now(0);
    ut_sample_now(0); /* correcting now */
    tally_clear();
    feed(VC_SRC_IMU, VC_EV_LIFTED, 1, VC_FC_IMU_SAFETY);
    CHECK("UT-31",
          (VC_ST_MISSION_COMPLETE == g_sm.st.state) &&
              has_move(VC_MOVE_STOP, 0, 0),
          "lift during the correction: MISSION_COMPLETE, motors stopped");
    tally_clear();
    ut_tick();
    ut_tick();
    ut_tick();
    ut_tick();
    ut_tick();
    ut_tick();
    CHECK("UT-32", 0 == g_tally[VC_MOVE_PIVOT],
          "after a lift: no pivot refresh");

    ut_begin();
    ut_fast();
    tally_clear();
    feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
    CHECK("UT-33",
          (VC_ST_AVOID_OBSTACLE == g_sm.st.state) &&
              has_move(VC_MOVE_STOP_CONFIRM, 0, 0) &&
              (NAV_NONE == g_sm.pending_nav),
          "obstacle during the U-turn: avoidance takes over, U-turn dropped");
    tally_clear();
    ut_tick();
    CHECK("UT-34", 0 == g_tally[VC_MOVE_PIVOT],
          "after the obstacle: no pivot refresh");

    /* 10. only the controller moves the robot during the U-turn */
    ut_begin();
    ut_fast();
    tally_clear();
    {
        uint32_t rejected = g_sm.st.moves_rejected;

        feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_STEER, 500);
        feed(VC_SRC_OBST, VC_EV_MOVE_REQUEST, VC_MOVE_PIVOT, 60);
        CHECK("UT-35",
              (0 == g_tally[VC_MOVE_STEER]) && (0 == g_tally[VC_MOVE_PIVOT]) &&
                  (rejected + 2u == g_sm.st.moves_rejected),
              "steering or pivot requests from S3 / S5 are refused during the "
              "U-turn");
    }

    /* 11. LEFT / RIGHT are unchanged: no line watch, no slow pivot */
    start_mission();
    tally_clear();
    feed(VC_SRC_LINE, VC_EV_BARCODE_DECODED, (int32_t)NAV_LEFT, 0);
    junction_to_execute();
    CHECK("UT-36",
          has_move(VC_MOVE_TURN_LEFT, VC_TURN_CMD_DEG, 1) &&
              !has_request(VC_SRC_LINE, VC_REQ_LINE_WATCH) &&
              (0 == g_tally[VC_MOVE_PIVOT]) && (0 == g_tally[VC_MOVE_U_TURN]),
          "LEFT: same turn request as before, no U-turn watch or pivot");
    CHECK("UT-37",
          (0 == strcmp("LINE_SAMPLE", vc_event_name(VC_EV_LINE_SAMPLE))) &&
              (0 == strcmp("CMD_CONFIG", vc_event_name(VC_EV_CMD_CONFIG))),
          "event names cover the new LINE_SAMPLE (tables size-checked at "
          "compile time)");
    start_mission();
    {
        uint32_t rejected = g_sm.st.moves_rejected;

        tally_clear();
        feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_PIVOT, 60);
        CHECK("UT-38",
              (0 == g_tally[VC_MOVE_PIVOT]) &&
                  (rejected + 1u == g_sm.st.moves_rejected),
              "PIVOT is the controller's own: refused even from the motion "
              "owner");
    }
    CHECK("UT-39", 0 == g_overflows,
          "no action list overflowed in any test (VC_MAX_ACTIONS)");
}

/* A junction: S3 reports it, S2 confirms the stop, the axle advance (if any)
   completes.  Returns with the controller in EXECUTE_ACTION. */
static void junction_to_execute(void)
{
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    if (VC_EXEC_ADVANCE == g_sm.exec_phase)
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
static void check_letter_maneuver(const char * p_obj_id, const char * p_letter,
                                  navigation_command_t nav, vc_move_t move,
                                  const char * p_what)
{
    char name[16];
    char desc[112];

#define NAV_CHECK(sfx, cond, text)                            \
    do                                                        \
    {                                                         \
        snprintf(name, sizeof(name), "%s%s", p_obj_id, sfx);  \
        snprintf(desc, sizeof(desc), "%s: %s", p_what, text); \
        CHECK(name, cond, desc);                              \
    } while (0)

    start_mission();
    tally_clear();
    feed_barcode(p_letter);
    NAV_CHECK("a",
              (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
                  (nav == g_sm.pending_nav) && (0 == turns_issued()) &&
                  (0 == g_tally[VC_MOVE_DRIVE_DIST]) &&
                  (0 == g_tally[VC_MOVE_STOP_CONFIRM]),
              "decoding alone stores the instruction and starts no motion");

    tally_clear();
    junction_to_execute();
    NAV_CHECK("b",
              (VC_ST_EXECUTE_ACTION == g_sm.st.state) &&
                  (1 == g_tally[VC_MOVE_STOP_CONFIRM]) &&
                  (1 == g_tally[move]) &&
                  ((VC_MOVE_DRIVE_DIST == move) ? (0 == turns_issued())
                                                : (1 == turns_issued())),
              "stops at the junction, then issues the maneuver exactly once");
    if (VC_MOVE_U_TURN == move)
    {
        NAV_CHECK("c",
                  has_move(VC_MOVE_U_TURN, VC_UTURN_FAST_DEG, 1) &&
                      (0u == g_sm.turn_gate),
                  "U-turn request is VC_MOVE_U_TURN, fast part (S6-U01)");
        uturn_clean(); /* aligned on the return line, slow follow */
        NAV_CHECK("u",
                  (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
                      (0 == g_tally[VC_MOVE_DRIVE_DIST]),
                  "then the line, not a straight drive, ends it");
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

    if (VC_MOVE_U_TURN != move)
    {
        feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    }
    NAV_CHECK("d",
              (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
                  (NAV_NONE == g_sm.pending_nav) &&
                  ('\0' == g_sm.pending_barcode[0]),
              "completion clears the instruction and resumes line following");

    tally_clear();
    junction_to_execute();
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    NAV_CHECK("e",
              (VC_ST_FOLLOW_LINE == g_sm.st.state) && (0 == turns_issued()),
              "second junction without a new barcode makes no turn");
#undef NAV_CHECK
}

static void test_letter_navigation(void)
{
    /* Z is the production default (right). Left, U-turn and straight use the
       test-only mappings so the team's production table stays as set. */
    check_letter_maneuver("NAV-01", "TESTLEFT", NAV_LEFT, VC_MOVE_TURN_LEFT,
                          "LEFT");
    check_letter_maneuver("NAV-02", "Z", NAV_RIGHT, VC_MOVE_TURN_RIGHT,
                          "Z = RIGHT");
    check_letter_maneuver("NAV-03", "TESTUTURN", NAV_U_TURN, VC_MOVE_U_TURN,
                          "U_TURN");
    check_letter_maneuver("NAV-04", "TESTSTRAIGHT", NAV_STRAIGHT,
                          VC_MOVE_DRIVE_DIST, "STRAIGHT");

    /* A: successful decode, its factory row (NONE) is the action STRAIGHT */
    start_mission();
    tally_clear();
    feed_barcode("A");
    CHECK("NAV-05a",
          (1u == g_sm.st.barcodes_read) &&
              (0 == strcmp("A", g_sm.st.last_barcode)) &&
              (NAV_STRAIGHT == g_sm.pending_nav),
          "A is recorded as a successful decode (count 1, last = A) and "
          "its assigned action is STRAIGHT");
    junction_to_execute();
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    CHECK("NAV-05b",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) && (0 == turns_issued()) &&
              (1 == g_tally[VC_MOVE_DRIVE_DIST]) &&
              (0 == strcmp("A", g_sm.st.last_barcode)),
          "A drives straight over the junction, no turn, still recorded as A");

    /* a missed read and a wrong letter are distinguishable from A */
    start_mission();
    feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
    feed(VC_SRC_LINE, VC_EV_BARCODE_FAILED, 0, 0);
    CHECK(
        "NAV-06a",
        (0u == g_sm.st.barcodes_read) && ('\0' == g_sm.st.last_barcode[0]) &&
            (NAV_NONE == g_sm.pending_nav) &&
            (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
            (VC_FC_BARCODE == g_sm.st.last_fault_code) &&
            (VC_FAULT_NON_CRITICAL == g_sm.st.last_fault_sev),
        "a missed read records no letter (count 0) and reports VC_FC_BARCODE, "
        "non-critical; the run goes on");
    start_mission();
    feed_barcode("T"); /* T is what a backwards Z looks like */
    CHECK("NAV-06b",
          (1u == g_sm.st.barcodes_read) &&
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
    CHECK("NAV-07",
          (1 == g_tally[VC_MOVE_TURN_RIGHT]) && (1 == turns_issued()) &&
              (VC_ST_FOLLOW_LINE == g_sm.st.state),
          "Z then two junctions: exactly one right turn, the second goes "
          "straight");

    /* Z, then an operator stop (FAULT), Reset, Start: no stale right turn */
    start_mission();
    feed_barcode("Z");
    feed(VC_SRC_COMMS, VC_EV_CMD_STOP, 0, 0);
    feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
    CHECK("NAV-08a",
          (VC_ST_IDLE == g_sm.st.state) && (NAV_NONE == g_sm.pending_nav) &&
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
    CHECK("NAV-08b",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) && (0 == turns_issued()),
          "after Reset and Start the first junction goes straight");

    /* Z, then overturn (critical fault), Reset: same */
    start_mission();
    feed_barcode("Z");
    feed(VC_SRC_IMU, VC_EV_FAULT, VC_FAULT_CRITICAL, 0x4005);
    feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
    CHECK("NAV-09",
          (VC_ST_IDLE == g_sm.st.state) && (NAV_NONE == g_sm.pending_nav),
          "Z then overturn fault then Reset: pending instruction cleared");

    /* Z, then lift (end of run): no turn, instruction cleared */
    start_mission();
    feed_barcode("Z");
    feed_lift();
    CHECK("NAV-10",
          (VC_ST_MISSION_COMPLETE == g_sm.st.state) &&
              (NAV_NONE == g_sm.pending_nav),
          "Z then lift: run ends, pending instruction cleared");

    /* lift at the junction wins over the pending turn, no turn issued */
    start_mission();
    feed_barcode("Z");
    tally_clear();
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed_lift();
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    CHECK("NAV-11",
          (VC_ST_MISSION_COMPLETE == g_sm.st.state) && (0 == turns_issued()),
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
    CHECK("NAV-12",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) && (0 == turns_issued()) &&
              (1 == g_tally[VC_MOVE_DRIVE_DIST]),
          "Z, lift, Start: the first junction of the new run goes straight");

    /* Reset while the run is still going is ignored (Reset only acts in
       IDLE, FAULT and MISSION_COMPLETE), so the pending Z still turns */
    start_mission();
    feed_barcode("Z");
    feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
    CHECK("NAV-13",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
              (NAV_RIGHT == g_sm.pending_nav),
          "Reset during a running mission changes nothing (existing rule)");

    /* The letter decides, not S3's legacy nav byte (S3_BC_CHAR_* are all
       off, but a stale p1 must not override the table either) */
    {
        vc_event_t event = { 0 };

        start_mission();
        event.src             = VC_SRC_LINE;
        event.type            = VC_EV_BARCODE_DECODED;
        event.p1              = (int32_t)NAV_LEFT;
        event.barcode_text[0] = 'Z';
        vc_sm_handle(&g_sm, &event, g_now_ms, &g_actions);
        CHECK("NAV-14",
              (NAV_RIGHT == g_sm.pending_nav) &&
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
        CHECK("NAV-15a",
              (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
                  (NAV_NONE == g_sm.pending_nav) &&
                  (1 == g_tally[VC_MOVE_TURN_RIGHT]),
              "Z turn ended on the line: instruction cleared, following");
        tally_clear();
        feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
        CHECK("NAV-15b",
              (VC_ST_FOLLOW_LINE == g_sm.st.state) && (0 == g_actions.n),
              "late S2 MANOEUVRE_DONE after a line-ended turn does nothing");
        junction_to_execute();
        feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
        CHECK("NAV-15c",
              (VC_ST_FOLLOW_LINE == g_sm.st.state) && (0 == turns_issued()),
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
    CHECK("S6-PA05",
          !g_sm.st.paused && (VC_ST_LOST_LINE == g_sm.st.state) &&
              (0 == g_actions.n),
          "pause rejected during line recovery");

    start_mission();
    feed(VC_SRC_COMMS, VC_EV_CMD_PAUSE, 0, 0);
    feed(VC_SRC_OBST, VC_EV_OBSTACLE, 100, 0);
    CHECK("S6-PA06",
          (VC_ST_FAULT == g_sm.st.state) && !g_sm.st.paused &&
              has_move(VC_MOVE_SAFE_STOP, 0, 0),
          "obstacle while paused enters safe fault without avoidance movement");

    start_mission();
    feed(VC_SRC_COMMS, VC_EV_CMD_PAUSE, 0, 0);
    g_now_ms += VC_JUNCTION_STOP_MS;
    feed(VC_SRC_VC, VC_EV_TICK, 0, 0);
    CHECK("S6-PA07",
          (VC_ST_FAULT == g_sm.st.state) && has_move(VC_MOVE_SAFE_STOP, 0, 0),
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

/* ------------------------------------------------------------------ */
/*  Stop cause: lift / overturn / S2 safe stop in either order          */
/*  (owner's reproduced defects, 2026-11-24).  last_fault_* is what     */
/*  S1 telemetry sends as fault_code (task_telemetry.c).                */
/* ------------------------------------------------------------------ */
#define SAFE_STOP_CODE 0x2011

static void feed_cause(vc_source_t src, vc_event_type_t type, int32_t param1,
                       int32_t param2, int16_t cause)
{
    vc_event_t event = { 0 };

    event.src   = (uint8_t)src;
    event.type  = (uint8_t)type;
    event.p1    = param1;
    event.p2    = param2;
    event.cause = cause;
    vc_sm_handle(&g_sm, &event, g_now_ms, &g_actions);
    tally_add();
}

/* S2's safe-stop report as vc_bridge_s2.c makes it */
static void feed_s2_safe_stop(int16_t cause)
{
    feed_cause(VC_SRC_MOTION, VC_EV_FAULT, VC_FAULT_CRITICAL, SAFE_STOP_CODE,
               cause);
}

static void feed_overturn(void)
{
    feed(VC_SRC_IMU, VC_EV_FAULT, VC_FAULT_CRITICAL, LIFT_CODE);
}

static int32_t has_report(vc_source_t src, vc_fault_severity_t sev,
                          int32_t code)
{
    int32_t found = 0;

    for (int32_t idx = 0; idx < g_actions.n; idx++)
    {
        if ((VC_ACT_REPORT == g_actions.a[idx].kind) &&
            (g_actions.a[idx].target == src) &&
            (g_actions.a[idx].p1 == (int32_t)sev) &&
            (g_actions.a[idx].p2 == code))
        {
            found = 1;
        }
    }
    return found;
}

static int32_t cause_is(vc_source_t src, vc_fault_severity_t sev, uint32_t code)
{
    return (code == g_sm.st.last_fault_code) &&
           ((uint8_t)src == g_sm.st.last_fault_src) &&
           (sev == g_sm.st.last_fault_sev);
}

static int32_t ended_as_lift(void)
{
    return (VC_ST_MISSION_COMPLETE == g_sm.st.state) &&
           cause_is(VC_SRC_IMU, VC_FAULT_NON_CRITICAL, LIFT_CODE);
}

static int32_t ended_as_overturn(void)
{
    return (VC_ST_FAULT == g_sm.st.state) &&
           cause_is(VC_SRC_IMU, VC_FAULT_CRITICAL, LIFT_CODE);
}

static void test_stop_cause(void)
{
    /* 1. S2 0x2011 (raised by the lift interlock) first, then S4 LIFTED */
    start_mission();
    tally_clear();
    feed_s2_safe_stop(VC_CAUSE_IMU_LIFT);
    CHECK("VC-C10",
          ended_as_lift() && has_move(VC_MOVE_STOP, 0, 0) &&
              has_request(VC_SRC_NONE, VC_REQ_STOP_ALL) &&
              has_report(VC_SRC_MOTION, VC_FAULT_CRITICAL, SAFE_STOP_CODE),
          "lift: S2 0x2011 first -> MISSION_COMPLETE at once, motors stopped, "
          "0x2011 still reported");
    feed_lift();
    CHECK("VC-C11", ended_as_lift() && (0 == g_actions.n),
          "lift: S4 LIFTED after S2's report changes nothing");
    CHECK("VC-C12",
          (0 == g_tally[VC_MOVE_FORWARD]) && (0 == g_tally[VC_MOVE_STEER]) &&
              (0 == g_tally[VC_MOVE_RESET]),
          "lift (S2 first): no drive or latch reset issued");

    /* 1b. S4's typed event dropped (nonblocking send): S2's report alone */
    start_mission();
    feed_s2_safe_stop(VC_CAUSE_IMU_LIFT);
    advance(5000u);
    CHECK("VC-C13", ended_as_lift(),
          "lift: S4 event dropped -> still MISSION_COMPLETE, cause IMU 0x4005");

    /* 2. S4 LIFTED first, then S2 0x2011 (with or without the IMU cause) */
    start_mission();
    feed_lift();
    feed_s2_safe_stop(VC_CAUSE_IMU_LIFT);
    CHECK("VC-C14",
          ended_as_lift() &&
              has_report(VC_SRC_MOTION, VC_FAULT_CRITICAL, SAFE_STOP_CODE) &&
              (1u <= g_sm.st.reports_after_end),
          "lift: LIFTED then S2 0x2011 -> MISSION_COMPLETE, cause stays IMU "
          "0x4005, 0x2011 counted");
    feed_s2_safe_stop(VC_CAUSE_NONE);
    CHECK("VC-C15", ended_as_lift() && !any_move(),
          "lift: a later plain 0x2011 does not replace the lift cause");

    /* 3. overturn: S4 0x4005 first, then S2 0x2011 */
    start_mission();
    tally_clear();
    feed_overturn();
    CHECK("VC-C16", ended_as_overturn() && (1 == g_tally[VC_MOVE_SAFE_STOP]),
          "overturn: S4 0x4005 -> FAULT, safe stop");
    feed_s2_safe_stop(VC_CAUSE_IMU_OVERTURN);
    CHECK("VC-C17",
          ended_as_overturn() &&
              has_report(VC_SRC_MOTION, VC_FAULT_CRITICAL, SAFE_STOP_CODE),
          "overturn: later S2 0x2011 reported but cause stays IMU 0x4005");
    feed_s2_safe_stop(VC_CAUSE_NONE);
    CHECK("VC-C18", ended_as_overturn(),
          "overturn: a later plain 0x2011 does not replace IMU 0x4005");

    /* 3b. overturn: S2 0x2011 first, then S4 0x4005; and S4 event dropped */
    start_mission();
    tally_clear();
    feed_s2_safe_stop(VC_CAUSE_IMU_OVERTURN);
    CHECK("VC-C19",
          ended_as_overturn() && (1 == g_tally[VC_MOVE_SAFE_STOP]) &&
              has_report(VC_SRC_MOTION, VC_FAULT_CRITICAL, SAFE_STOP_CODE),
          "overturn: S2 0x2011 first -> FAULT IMU 0x4005 at once, safe stop");
    feed_overturn();
    CHECK("VC-C20", ended_as_overturn(),
          "overturn: S4 0x4005 after S2's report: still FAULT IMU 0x4005");
    feed_lift();
    CHECK("VC-C21", ended_as_overturn(),
          "overturn: a lift report after the overturn never completes the "
          "mission");
    start_mission();
    feed_s2_safe_stop(VC_CAUSE_IMU_UNKNOWN);
    CHECK("VC-C22", ended_as_overturn(),
          "IMU interlock of unknown kind -> FAULT IMU 0x4005 (fail safe, not a "
          "lift)");

    /* 4. genuine S2 safe stop: no IMU cause */
    start_mission();
    tally_clear();
    feed_s2_safe_stop(VC_CAUSE_NONE);
    CHECK("VC-C23",
          (VC_ST_FAULT == g_sm.st.state) &&
              cause_is(VC_SRC_MOTION, VC_FAULT_CRITICAL, SAFE_STOP_CODE) &&
              (1 == g_tally[VC_MOVE_SAFE_STOP]),
          "standalone S2 0x2011 -> FAULT, critical 0x2011 from S2");
    feed_lift();
    CHECK("VC-C24",
          (VC_ST_FAULT == g_sm.st.state) &&
              cause_is(VC_SRC_MOTION, VC_FAULT_CRITICAL, SAFE_STOP_CODE),
          "unrelated lift after a genuine 0x2011: stays FAULT 0x2011");
    feed_s2_safe_stop(VC_CAUSE_IMU_LIFT);
    CHECK("VC-C25",
          (VC_ST_FAULT == g_sm.st.state) &&
              cause_is(VC_SRC_MOTION, VC_FAULT_CRITICAL, SAFE_STOP_CODE),
          "lift interlock after a genuine 0x2011: stays FAULT 0x2011");

    /* 5. other genuine fault, then lift */
    start_mission();
    feed(VC_SRC_OBST, VC_EV_AVOID_FAILED, 0, 0);
    feed(VC_SRC_LINE, VC_EV_FAULT, VC_FAULT_CRITICAL, 0x3001);
    feed_lift();
    feed_s2_safe_stop(VC_CAUSE_IMU_LIFT);
    CHECK(
        "VC-C26",
        (VC_ST_FAULT == g_sm.st.state) &&
            cause_is(VC_SRC_LINE, VC_FAULT_CRITICAL, 0x3001),
        "critical S3 fault, then lift + 0x2011: stays FAULT with the S3 cause");

    /* 6. a non-critical warning never becomes the run-ending cause */
    start_mission();
    feed(VC_SRC_COMMS, VC_EV_WIFI_DOWN, 0, 0);
    feed_lift();
    CHECK("VC-C27", ended_as_lift(),
          "WiFi warning then lift: cause is the lift, not 0x1000");
    start_mission();
    feed(VC_SRC_IMU, VC_EV_FAULT, VC_FAULT_NON_CRITICAL, 0x4002);
    feed_overturn();
    CHECK("VC-C28", ended_as_overturn(),
          "IMU non-critical 0x4002 then overturn: cause is 0x4005");
    start_mission();
    feed(VC_SRC_COMMS, VC_EV_WIFI_DOWN, 0, 0);
    feed(VC_SRC_LINE, VC_EV_END_OF_COURSE, 0, 0);
    CHECK("VC-C29",
          (VC_ST_MISSION_COMPLETE == g_sm.st.state) &&
              (VC_FC_NONE == g_sm.st.last_fault_code),
          "WiFi warning then end of course: complete with no fault code");
    feed(VC_SRC_COMMS, VC_EV_WIFI_DOWN, 0, 0);
    CHECK("VC-C30",
          (VC_FC_NONE == g_sm.st.last_fault_code) &&
              has_report(VC_SRC_COMMS, VC_FAULT_NON_CRITICAL, VC_FC_WIFI_DOWN),
          "warning after the run ended: still reported, cause unchanged");

    /* 7. repeated safe-stop notifications (one per S2 stop generation) */
    start_mission();
    feed_overturn();
    feed_s2_safe_stop(VC_CAUSE_IMU_OVERTURN);
    feed_s2_safe_stop(VC_CAUSE_IMU_OVERTURN);
    feed_s2_safe_stop(VC_CAUSE_NONE);
    CHECK("VC-C31", ended_as_overturn() && (3u <= g_sm.st.reports_after_end),
          "three more safe-stop reports after an overturn: cause unchanged, "
          "all counted");

    /* 8. Reset then a new run: the next stop gets its own cause */
    feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
    CHECK("VC-C32",
          (VC_ST_IDLE == g_sm.st.state) && has_move(VC_MOVE_RESET, 0, 0),
          "Reset after an overturn -> IDLE, S2 latch reset");
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
    feed(VC_SRC_LINE, VC_EV_READY, 0, 0);
    feed(VC_SRC_IMU, VC_EV_READY, 0, 0);
    feed(VC_SRC_OBST, VC_EV_READY, 0, 0);
    CHECK("VC-C33",
          (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
              (0u == g_sm.st.reports_after_end),
          "new run after Reset: following the line, after-end count cleared");
    feed_s2_safe_stop(VC_CAUSE_IMU_LIFT);
    CHECK("VC-C34", ended_as_lift(),
          "new run: a lift now ends it as MISSION_COMPLETE (old overturn cause "
          "gone)");
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
    feed(VC_SRC_LINE, VC_EV_READY, 0, 0);
    feed(VC_SRC_IMU, VC_EV_READY, 0, 0);
    feed(VC_SRC_OBST, VC_EV_READY, 0, 0);
    feed_s2_safe_stop(VC_CAUSE_NONE);
    CHECK("VC-C35",
          (VC_ST_FAULT == g_sm.st.state) &&
              cause_is(VC_SRC_MOTION, VC_FAULT_CRITICAL, SAFE_STOP_CODE),
          "Start after a lift, then a genuine 0x2011 -> FAULT 0x2011");

    /* 9. pre-start: the IMU-caused safe stop is reported, starts nothing */
    g_now_ms = 0u;
    vc_sm_init(&g_sm, g_now_ms);
    feed_s2_safe_stop(VC_CAUSE_IMU_LIFT);
    CHECK(
        "VC-C36",
        (VC_ST_IDLE == g_sm.st.state) && !any_move() &&
            cause_is(VC_SRC_IMU, VC_FAULT_CRITICAL, LIFT_CODE),
        "lift interlock in IDLE: critical IMU 0x4005 reported, nothing moves");
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    feed_s2_safe_stop(VC_CAUSE_IMU_LIFT);
    CHECK("VC-C37",
          (VC_ST_FAULT == g_sm.st.state) &&
              cause_is(VC_SRC_IMU, VC_FAULT_CRITICAL, LIFT_CODE),
          "lift interlock during self check -> FAULT IMU 0x4005 (as LIFTED "
          "there)");

    /* 10. the cause belongs to S2's safe stop only */
    start_mission();
    feed_cause(VC_SRC_MOTION, VC_EV_FAULT, VC_FAULT_CRITICAL, 0x2010,
               VC_CAUSE_IMU_LIFT);
    CHECK("VC-C38",
          (VC_ST_FAULT == g_sm.st.state) &&
              cause_is(VC_SRC_MOTION, VC_FAULT_CRITICAL, 0x2010),
          "a cause on another S2 code (0x2010) is ignored: FAULT 0x2010");

    /* 11. paused robot lifted */
    start_mission();
    feed(VC_SRC_COMMS, VC_EV_CMD_PAUSE, 0, 0);
    feed_s2_safe_stop(VC_CAUSE_IMU_LIFT);
    CHECK("VC-C39", ended_as_lift(),
          "lift while paused (S2 report first) -> MISSION_COMPLETE");
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
    test_stop_cause();
    test_uturn();

    printf("[TEST] vehicle_controller host SUMMARY %d passed, %d failed\n",
           g_passed, g_failed);
    return (0 != g_failed) ? 1 : 0;
}

/*** end of file ***/
