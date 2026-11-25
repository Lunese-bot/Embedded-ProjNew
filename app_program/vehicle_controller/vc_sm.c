/*
 *      vc_sm.c
 *      Subsystem 6 -- Vehicle Controller mission state machine.
 *
 *      Implements the "Vehicle controller" statechart of the design review
 *      (sections 5 and 6.1) and the fault rules of FR8 / NFR5 / NFR8:
 *
 *        IDLE --start--> MISSION_START --all READY--> FOLLOW_LINE (hub)
 *        FOLLOW_LINE --line_lost-------> LOST_LINE      --found--> FOLLOW_LINE
 *                    --obstacle--------> AVOID_OBSTACLE --done---> FOLLOW_LINE
 *                    --barcode_start---> DECODE_BARCODE --decoded->
 * FOLLOW_LINE (buffer full barcode text)
 *                    --junction--------> JUNCTION_STOP --S2 stopped->
 * EXECUTE_ACTION --manoeuvre_done--> FOLLOW_LINE
 *                    --hump_start------> TERRAIN_CHECK  --cleared-> FOLLOW_LINE
 *                    --end_of_course / lifted---> MISSION_COMPLETE
 *        any active state --critical fault / stop--> FAULT --reset-->
 * IDLE
 *
 *      Line-loss is ignored in DECODE_BARCODE, EXECUTE_ACTION, TERRAIN_CHECK
 *      (design review table) and AVOID_OBSTACLE (the robot leaves the line
 *      on purpose).  Every non-hub state has a time limit, so no recovery
 *      can run forever (NFR5).
 *
 *      No kernel calls in this file.
 */

#include "vc_sm.h"
#include "vc_barcode_rules.h"
#include "vc_barcode_map.h"

/* ------------------------------------------------------------------ */
/*  Names, for console and telemetry                                   */
/* ------------------------------------------------------------------ */

static const char * const g_state_names[] = {
    "IDLE",          "MISSION_START",  "FOLLOW_LINE",     "JUNCTION_STOP",
    "LOST_LINE",     "AVOID_OBSTACLE", "DECODE_BARCODE",  "EXECUTE_ACTION",
    "TERRAIN_CHECK", "FAULT",          "MISSION_COMPLETE"
};
_Static_assert(VC_ST_COUNT == sizeof(g_state_names) / sizeof(g_state_names[0]),
               "one name per vc_state_t");

static const char * const g_event_names[] = {
    "NONE",           "READY",           "MOVE_REQUEST",   "FAULT",
    "CMD_START",      "CMD_STOP",        "CMD_RESET",      "CMD_PAUSE",
    "CMD_RESUME",     "WIFI_DOWN",       "WIFI_UP",        "MANOEUVRE_DONE",
    "MOTION_STOPPED", "LINE_LOST",       "LINE_FOUND",     "JUNCTION",
    "BARCODE_START",  "BARCODE_DECODED", "BARCODE_FAILED", "END_OF_COURSE",
    "HUMP_START",     "HUMP_CLEARED",    "LIFTED",         "OBSTACLE",
    "AVOID_DONE",     "AVOID_FAILED",    "TICK",           "CMD_CONFIG",
    "LINE_SAMPLE"
};
/* A new event needs its name here too (a missing one was a null name). */
_Static_assert(VC_EV_COUNT == sizeof(g_event_names) / sizeof(g_event_names[0]),
               "one name per vc_event_type_t");

const char * vc_state_name(vc_state_t state)
{
    return ((uint32_t)state < (uint32_t)VC_ST_COUNT) ? g_state_names[state]
                                                     : "?";
}

const char * vc_event_name(vc_event_type_t type)
{
    return ((uint32_t)type < (uint32_t)VC_EV_COUNT) ? g_event_names[type] : "?";
}

/* ------------------------------------------------------------------ */
/*  Helpers                                                            */
/* ------------------------------------------------------------------ */

bool vc_sm_is_active(vc_state_t state)
{
    return (state >= VC_ST_MISSION_START) && (state <= VC_ST_TERRAIN_CHECK);
}

vc_source_t vc_sm_motion_owner(vc_state_t state)
{
    vc_source_t owner;

    switch (state)
    {
        case VC_ST_FOLLOW_LINE:
        case VC_ST_DECODE_BARCODE: /* S3 keeps steering, speed capped */
        case VC_ST_TERRAIN_CHECK:  /* S3 keeps steering, speed capped */
            owner = VC_SRC_LINE;
            break;
        case VC_ST_AVOID_OBSTACLE:
            owner = VC_SRC_OBST;
            break;
        default:
            /* LOST_LINE: the controller runs the search itself */
            owner = VC_SRC_NONE;
            break; /* controller's own moves only */
    }
    return owner;
}

static vc_action_t * emit(vc_actions_t * p_out, vc_action_kind_t kind,
                          vc_source_t target, int32_t param1, int32_t param2)
{
    vc_action_t * p_action;

    if (p_out->n >= VC_MAX_ACTIONS)
    {
        p_out->overflow = true;
        return (vc_action_t *)0; /* <stddef.h> clashes with the kernel's size_t
                                  */
    }
    p_action         = &p_out->a[p_out->n++];
    p_action->kind   = (uint8_t)kind;
    p_action->target = (uint8_t)target;
    p_action->rsv    = 0;
    p_action->p1     = param1;
    p_action->p2     = param2;
    return p_action;
}

static void move(vc_actions_t * p_out, vc_move_t move_cmd, int32_t value)
{
    (void)emit(p_out, VC_ACT_MOVE, VC_SRC_MOTION, (int32_t)move_cmd, value);
}

static void request(vc_actions_t * p_out, vc_source_t target, vc_request_t req,
                    int32_t arg)
{
    (void)emit(p_out, VC_ACT_REQUEST, target, (int32_t)req, arg);
}

static bool run_ended(vc_state_t state)
{
    return (VC_ST_FAULT == state) || (VC_ST_MISSION_COMPLETE == state);
}

static void set_cause(vc_sm_t * p_sm, vc_source_t src, vc_fault_severity_t sev,
                      uint32_t code)
{
    p_sm->st.last_fault_code = code;
    p_sm->st.last_fault_sev  = sev;
    p_sm->st.last_fault_src  = (uint8_t)src;
}

/* Every report is sent.  Once the run has ended, the cause that ended it
   stays in last_fault_* (a safe stop that follows an overturn, or a fault
   after a lift, must not replace it); later reports are only counted. */
static void report(vc_sm_t * p_sm, vc_actions_t * p_out, vc_source_t src,
                   vc_fault_severity_t sev, uint32_t code)
{
    if (run_ended(p_sm->st.state))
    {
        p_sm->st.reports_after_end++;
    }
    else
    {
        set_cause(p_sm, src, sev, code);
    }
    (void)emit(p_out, VC_ACT_REPORT, src, (int32_t)sev, (int32_t)code);
}

static uint32_t elapsed(uint32_t now_ms, uint32_t since_ms)
{
    return now_ms - since_ms; /* unsigned: wraps correctly */
}

/* ------------------------------------------------------------------ */
/*  State entry actions                                                */
/* ------------------------------------------------------------------ */

static void enter(vc_sm_t * p_sm, vc_state_t next, uint32_t now_ms,
                  vc_actions_t * p_out);

/* LOST_LINE search pattern, one step per attempt (bounded, NFR5):
     1. turn towards the side the line was last seen
     2. turn twice as far the other way
     3. reverse a little, back over where the line was   */
static void search_step(const vc_sm_t * p_sm, vc_actions_t * p_out)
{
    bool b_left_first = (p_sm->lost_side <= 0); /* unknown: try left */

    switch (p_sm->attempts)
    {
        case 1:
            move(p_out, b_left_first ? VC_MOVE_TURN_LEFT : VC_MOVE_TURN_RIGHT,
                 VC_SEARCH_TURN_DEG);
            break;
        case 2:
            move(p_out, b_left_first ? VC_MOVE_TURN_RIGHT : VC_MOVE_TURN_LEFT,
                 2 * VC_SEARCH_TURN_DEG);
            break;
        default:
            move(p_out, VC_MOVE_REVERSE, VC_SEARCH_BACK_MM);
            break;
    }
    request(p_out, VC_SRC_LINE, VC_REQ_LINE_SEARCH, (int32_t)p_sm->attempts);
}

/* Start a junction turn and, with VC_TURN_END_ON_LINE, watch for the new
   arm once the robot has turned gate_deg (on_execute_action). */
static void start_turn(vc_sm_t * p_sm, vc_move_t move_cmd, int32_t value,
                       uint32_t gate_deg, vc_actions_t * p_out)
{
    move(p_out, move_cmd, value);
    p_sm->turn_odo0 = p_sm->odo;
    p_sm->turn_gate =
        (0 != VC_TURN_END_ON_LINE) ? VC_PIVOT_COUNTS(gate_deg) : 0u;
}

/* Junction evidence for S1 (VC_ACT_JUNCTION), from the pending action as
   it stands now. */
static void junction_note(const vc_sm_t * p_sm, vc_actions_t * p_out,
                          vc_junction_note_t note)
{
    const char * p_text = p_sm->pending_barcode;
    int32_t      letter = 0;
    int32_t      code   = vc_bcmap_wire_from_nav(p_sm->pending_nav);

    if ((p_text[0] >= 'A') && (p_text[0] <= 'Z') && ('\0' == p_text[1]))
    {
        letter = (int32_t)p_text[0];
    }
    if (code < 0)
    {
        code = 0;
    }
    (void)emit(p_out, VC_ACT_JUNCTION, VC_SRC_VC, (int32_t)note,
               (letter << 8) | code);
}

/* Cancel a pending junction action inside a run, and say so when there was
   one. */
static void drop_pending(vc_sm_t * p_sm, vc_actions_t * p_out)
{
    if ((NAV_NONE != p_sm->pending_nav) || ('\0' != p_sm->pending_barcode[0]))
    {
        junction_note(p_sm, p_out, VC_JX_DROPPED);
    }
    p_sm->pending_nav        = NAV_NONE;
    p_sm->pending_barcode[0] = '\0';
}

static void ut_start(vc_sm_t * p_sm, uint32_t now_ms, vc_actions_t * p_out);

static void enter_execute(vc_sm_t * p_sm, uint32_t now_ms, vc_actions_t * p_out)
{
    p_sm->exec_phase = VC_EXEC_TURN;
    p_sm->turn_gate  = 0u;
    switch (p_sm->pending_nav)
    {
        case NAV_LEFT:
        case NAV_RIGHT:
            if (VC_JUNCTION_ADVANCE_MM > 0)
            {
                /* Put the axle over the crossing first; the turn follows
                   when S2 reports this move done (on_execute_action). */
                p_sm->exec_phase = VC_EXEC_ADVANCE;
                move(p_out, VC_MOVE_DRIVE_DIST, VC_JUNCTION_ADVANCE_MM);
            }
            else
            {
                start_turn(p_sm,
                           (NAV_LEFT == p_sm->pending_nav) ? VC_MOVE_TURN_LEFT
                                                           : VC_MOVE_TURN_RIGHT,
                           VC_TURN_CMD_DEG, VC_TURN_GATE_DEG, p_out);
            }
            break;
        case NAV_U_TURN:
            /* fast pivot, slow approach, line-confirmed end (vc_sm.h).
               turn_gate stays 0: LINE_FOUND never ends a U-turn. */
            ut_start(p_sm, now_ms, p_out);
            break;
        case NAV_STRAIGHT:
        case NAV_NONE:
        default:
            /* Clear the crossing before handing motion back to S3.  This
               distance is a track-tuning value, not a universal constant. */
            move(p_out, VC_MOVE_DRIVE_DIST, VC_JUNCTION_CLEAR_MM);
            break;
    }
}

static void enter(vc_sm_t * p_sm, vc_state_t next, uint32_t now_ms,
                  vc_actions_t * p_out)
{
    vc_state_t prev = p_sm->st.state;

    p_sm->st.prev_state     = prev;
    p_sm->st.state          = next;
    p_sm->st.paused         = false;
    p_sm->pause_stopped     = false;
    p_sm->st.state_since_ms = now_ms;
    p_sm->attempt_since_ms  = now_ms;
    (void)emit(p_out, VC_ACT_STATE, VC_SRC_VC, (int32_t)next, (int32_t)prev);

    switch (next)
    {
        case VC_ST_IDLE:
            /* Reset: a barcode read before the stop must not act later. */
            p_sm->pending_nav        = NAV_NONE;
            p_sm->pending_barcode[0] = '\0';
            if ((VC_ST_FAULT == prev) || (VC_ST_MISSION_COMPLETE == prev))
            {
                /* clear S2's latch (a lift latches it too) */
                move(p_out, VC_MOVE_RESET, 0);
            }
            move(p_out, VC_MOVE_STOP, 0);
            break;

        case VC_ST_MISSION_START:
            move(p_out, VC_MOVE_STOP, 0);
            p_sm->st.reports_after_end = 0;
            p_sm->ready_mask           = 0;
            p_sm->st.mission_start_ms  = now_ms;
            p_sm->st.max_hump_um       = 0;
            p_sm->st.barcodes_read     = 0;
            p_sm->st.last_nav          = NAV_NONE;
            p_sm->st.last_barcode[0]   = '\0';
            request(p_out, VC_SRC_NONE, VC_REQ_SELF_CHECK, 0);
            break;

        case VC_ST_FOLLOW_LINE:
            p_sm->retries = 0;
            p_sm->rejoin_steer_left =
                ((VC_ST_AVOID_OBSTACLE == prev) || p_sm->ut.handover)
                    ? VC_REJOIN_STEER_SAMPLES
                    : 0u;
            request(p_out, VC_SRC_OBST, VC_REQ_OBST_MONITOR, 0);
            request(p_out, VC_SRC_LINE, VC_REQ_LINE_FOLLOW, 0);
            if (p_sm->ut.handover)
            {
                /* U-turn aligned on the line: slow line following for
                   VC_REJOIN_STEER_SAMPLES fresh steering samples, then
                   cruise (move_request()) */
                p_sm->ut.handover = false;
                move(p_out, VC_MOVE_SET_SPEED, VC_SPEED_SLOW_MMS);
                move(p_out, VC_MOVE_FORWARD, VC_SPEED_SLOW_MMS);
            }
            else if (0u != p_sm->rejoin_steer_left)
            {
                /* S5 finished at rest. The first fresh steering sample
                   starts the car moving. */
                move(p_out, VC_MOVE_SET_SPEED, VC_SPEED_SLOW_MMS);
            }
            else
            {
                move(p_out, VC_MOVE_SET_SPEED, VC_SPEED_CRUISE_MMS);
                move(p_out, VC_MOVE_FORWARD, VC_SPEED_CRUISE_MMS);
            }
            break;

        case VC_ST_JUNCTION_STOP:
            /* Do not execute a turn yet. S2 reports MOTION_STOPPED only
               after both wheels have physically reached the stop threshold. */
            p_sm->junction_stopped = 0u;
            move(p_out, VC_MOVE_STOP_CONFIRM, 0);
            break;

        case VC_ST_LOST_LINE:
            move(p_out, VC_MOVE_STOP, 0);
            move(p_out, VC_MOVE_SET_SPEED, VC_SPEED_SLOW_MMS);
            p_sm->attempts = 1;
            search_step(p_sm, p_out);
            break;

        case VC_ST_AVOID_OBSTACLE:
            p_sm->avoid_wait_stop = true;
            move(p_out, VC_MOVE_STOP_CONFIRM, 0);
            request(p_out, VC_SRC_LINE, VC_REQ_BARCODE_OFF, 0);
            break;

        case VC_ST_DECODE_BARCODE:
            move(p_out, VC_MOVE_SET_SPEED, VC_SPEED_SLOW_MMS);
            request(p_out, VC_SRC_LINE, VC_REQ_BARCODE_READ, 0);
            break;

        case VC_ST_EXECUTE_ACTION:
            enter_execute(p_sm, now_ms, p_out);
            break;

        case VC_ST_TERRAIN_CHECK:
            move(p_out, VC_MOVE_SET_SPEED, VC_SPEED_SLOW_MMS);
            request(p_out, VC_SRC_IMU, VC_REQ_TERRAIN_MEASURE, 0);
            break;

        case VC_ST_FAULT:
            move(p_out, VC_MOVE_SAFE_STOP, 0); /* safe state first (NFR8) */
            request(p_out, VC_SRC_NONE, VC_REQ_STOP_ALL, 0);
            break;

        case VC_ST_MISSION_COMPLETE:
            move(p_out, VC_MOVE_STOP, 0);
            request(p_out, VC_SRC_NONE, VC_REQ_STOP_ALL, 0);
            /* Completion is not a fault: an earlier warning (WiFi, barcode)
               must not read as the reason the run ended.  A lift sets its
               own cause right after (lift()). */
            set_cause(p_sm, VC_SRC_NONE, VC_FAULT_NON_CRITICAL, VC_FC_NONE);
            break;

        default:
            break;
    }
}

static void go_fault(vc_sm_t * p_sm, vc_source_t src, uint32_t code,
                     uint32_t now_ms, vc_actions_t * p_out)
{
    report(p_sm, p_out, src, VC_FAULT_CRITICAL, code);
    if (p_sm->st.state != VC_ST_FAULT)
    {
        enter(p_sm, VC_ST_FAULT, now_ms, p_out);
    }
}

/* ------------------------------------------------------------------ */
/*  U-turn: fast pivot, slow approach, line-confirmed end (vc_sm.h)     */
/* ------------------------------------------------------------------ */

#define UT_C 0u /* index into vc_uturn_t's per-sensor arrays */
#define UT_R 1u

/* S2 counts (L + R) for `deg` S2 command degrees, signed */
static int32_t ut_counts(int32_t deg)
{
    /* Cast: a few hundred degrees is a few thousand counts */
    return (deg >= 0) ? (int32_t)VC_PIVOT_COUNTS(deg)
                      : -(int32_t)VC_PIVOT_COUNTS(-deg);
}

static bool ut_active(const vc_sm_t * p_sm)
{
    return (VC_ST_EXECUTE_ACTION == p_sm->st.state) &&
           (p_sm->exec_phase >= VC_EXEC_UTURN) &&
           (p_sm->exec_phase <= VC_EXEC_UT_VERIFY);
}

static void ut_trace(const vc_sm_t * p_sm, vc_actions_t * p_out,
                     vc_ut_trace_t what, int32_t detail)
{
    vc_action_t * p_action =
        emit(p_out, VC_ACT_TRACE, VC_SRC_VC, (int32_t)what, p_sm->ut.heading);

    if ((vc_action_t *)0 != p_action)
    {
        /* Cast: details are small (sensor, verdict, a few hundred counts) */
        p_action->rsv =
            (int16_t)((detail > 32767) ? 32767
                                       : ((detail < -32768) ? -32768 : detail));
    }
}

/* S2's odometer moved: the wheels turned that many counts the way the
   current pivot goes (they also coast that way after a stop request). */
static void ut_track_odo(vc_sm_t * p_sm, uint32_t odo_now)
{
    uint32_t delta = odo_now - p_sm->odo;

    if (ut_active(p_sm) && (delta < 0x80000000u))
    {
        /* Cast: delta < 2^31 */
        p_sm->ut.heading += (int32_t)p_sm->ut.dir * (int32_t)delta;
    }
}

static void ut_clear_edges(vc_sm_t * p_sm)
{
    uint32_t idx;

    for (idx = 0u; idx < 2u; idx++)
    {
        p_sm->ut.run_black[idx] = 0u;
        p_sm->ut.run_white[idx] = 0u;
        p_sm->ut.seen[idx]      = 0u;
        p_sm->ut.left[idx]      = 0u;
    }
    p_sm->ut.sight   = 0u;
    p_sm->ut.sight_h = 0;
}

/* One fresh sample into the sweep's edge history.  A sensor has SEEN the
   line after VC_UTURN_SIGHT_SAMPLES black samples in a row, and has LET IT
   GO after as many white ones since.  Returns true on the sweep's first
   credible sighting. */
static bool ut_edges(vc_sm_t * p_sm, int32_t mask)
{
    static const int32_t k_bit[2] = { VC_LINE_C, VC_LINE_R };
    bool                 b_first  = false;
    uint32_t             idx;

    for (idx = 0u; idx < 2u; idx++)
    {
        vc_uturn_t * p_ut = &p_sm->ut;

        if (0 != (mask & k_bit[idx]))
        {
            p_ut->run_white[idx] = 0u;
            if (p_ut->run_black[idx] < 255u)
            {
                p_ut->run_black[idx]++;
            }
            if (p_ut->run_black[idx] >= VC_UTURN_SIGHT_SAMPLES)
            {
                p_ut->seen[idx] = 1u;
                p_ut->left[idx] = 0u;
                if (0u == p_ut->sight)
                {
                    /* Cast: VC_LINE_C / VC_LINE_R */
                    p_ut->sight   = (uint8_t)k_bit[idx];
                    p_ut->sight_h = p_ut->heading;
                    b_first       = true;
                }
            }
        }
        else
        {
            p_ut->run_black[idx] = 0u;
            if (p_ut->run_white[idx] < 255u)
            {
                p_ut->run_white[idx]++;
            }
            if ((0u != p_ut->seen[idx]) &&
                (p_ut->run_white[idx] >= VC_UTURN_SIGHT_SAMPLES))
            {
                p_ut->left[idx] = 1u;
            }
        }
    }
    return b_first;
}

/* Where is the line, from this sample and the sweep's history?  The tape
   runs BETWEEN C and R when the robot follows it, so both white alone
   proves nothing (on the line, or off the track).  Pivoting left the line
   moves across the sensors from C to R, pivoting right from R to C.  So
   both white means between them only if the sensor the line reaches first
   has let it go and the other has not; if the other one let it go, the
   robot turned past the line. */
static vc_ut_verdict_t ut_verdict(const vc_sm_t * p_sm, int32_t mask)
{
    uint32_t        first = (p_sm->ut.dir > 0) ? UT_C : UT_R;
    uint32_t        other = (p_sm->ut.dir > 0) ? UT_R : UT_C;
    vc_ut_verdict_t value2;

    if (0 != (mask & (VC_LINE_C | VC_LINE_R)))
    {
        value2 = VC_UT_V_ON_LINE;
    }
    else if (0u != p_sm->ut.left[other])
    {
        value2 = VC_UT_V_PASSED;
    }
    else if (0u != p_sm->ut.left[first])
    {
        value2 = VC_UT_V_BETWEEN;
    }
    else if ((0u != p_sm->ut.seen[first]) || (0u != p_sm->ut.seen[other]))
    {
        value2 = VC_UT_V_UNSURE; /* a sensor is just going white: wait */
    }
    else
    {
        value2 = VC_UT_V_NONE;
    }
    return value2;
}

static void ut_start(vc_sm_t * p_sm, uint32_t now_ms, vc_actions_t * p_out)
{
    vc_uturn_t zero = { 0 };

    p_sm->ut          = zero;
    p_sm->ut.dir      = 1; /* S2 TURN +angle = left */
    p_sm->ut.t0_ms    = now_ms;
    p_sm->ut.phase_ms = now_ms;
    p_sm->exec_phase  = VC_EXEC_UTURN;
    move(p_out, VC_MOVE_U_TURN, VC_UTURN_FAST_DEG);
    request(p_out, VC_SRC_LINE, VC_REQ_LINE_WATCH, 1);
    ut_trace(p_sm, p_out, VC_UT_START, VC_UTURN_FAST_DEG);
}

static void ut_sweep(vc_sm_t * p_sm, vc_ut_sweep_t kind, int8_t dir,
                     int32_t bound, uint32_t now_ms, vc_actions_t * p_out)
{
    p_sm->ut.sweep    = (uint8_t)kind;
    p_sm->ut.dir      = dir;
    p_sm->ut.bound    = bound;
    p_sm->ut.phase_ms = now_ms;
    p_sm->ut.cmd_ms   = now_ms;
    p_sm->exec_phase  = VC_EXEC_UT_SWEEP;
    ut_clear_edges(p_sm);
    move(p_out, VC_MOVE_PIVOT, (int32_t)dir * VC_UTURN_SLOW_MMS);
    ut_trace(p_sm, p_out, VC_UT_SWEEP, (int32_t)kind * (int32_t)dir);
}

/* Ask S2 to stop; the wheels keep turning the same way until it confirms */
static void ut_brake(vc_sm_t * p_sm, int32_t reason, uint32_t now_ms,
                     vc_actions_t * p_out)
{
    p_sm->ut.stop_h   = p_sm->ut.heading;
    p_sm->ut.phase_ms = now_ms;
    p_sm->exec_phase  = VC_EXEC_UT_BRAKE;
    move(p_out, VC_MOVE_STOP_CONFIRM, 0);
    ut_trace(p_sm, p_out, VC_UT_BRAKE, reason);
}

static void ut_done(vc_sm_t * p_sm, uint32_t now_ms, vc_actions_t * p_out)
{
    ut_trace(p_sm, p_out, VC_UT_DONE, 0);
    request(p_out, VC_SRC_LINE, VC_REQ_LINE_WATCH, 0);
    junction_note(p_sm, p_out, VC_JX_COMPLETED);
    p_sm->pending_nav        = NAV_NONE; /* the U-turn is used up */
    p_sm->pending_barcode[0] = '\0';
    p_sm->ut.handover        = true;
    enter(p_sm, VC_ST_FOLLOW_LINE, now_ms, p_out);
}

static void ut_fail(vc_sm_t * p_sm, int32_t reason, uint32_t now_ms,
                    vc_actions_t * p_out)
{
    ut_trace(p_sm, p_out, VC_UT_FAIL, reason);
    request(p_out, VC_SRC_LINE, VC_REQ_LINE_WATCH, 0);
    p_sm->pending_nav        = NAV_NONE; /* never repeated after Reset */
    p_sm->pending_barcode[0] = '\0';
    go_fault(p_sm, VC_SRC_VC, VC_FC_LINE_RECOVERY, now_ms, p_out);
}

/* The wheels have stopped and the fresh samples agree */
static void ut_decide(vc_sm_t * p_sm, vc_ut_verdict_t value2, uint32_t now_ms,
                      vc_actions_t * p_out)
{
    int8_t back = (int8_t)(-p_sm->ut.dir);

    ut_trace(p_sm, p_out, VC_UT_VERDICT, (int32_t)value2);
    if ((VC_UT_V_ON_LINE == value2) || (VC_UT_V_BETWEEN == value2))
    {
        ut_done(p_sm, now_ms, p_out);
    }
    else if (p_sm->ut.sweeps >= VC_UTURN_MAX_SWEEPS)
    {
        ut_fail(p_sm, 1, now_ms, p_out);
    }
    else if (VC_UT_V_PASSED == value2)
    {
        /* turned past the line: slowly back toward where it was seen, at
           most VC_UTURN_CORRECT_DEG beyond that */
        p_sm->ut.sweeps++;
        ut_sweep(p_sm, VC_UT_CORRECT, back,
                 p_sm->ut.sight_h +
                     ((int32_t)back * ut_counts(VC_UTURN_CORRECT_DEG)),
                 now_ms, p_out);
    }
    else
    {
        /* no line in this sweep: the other way across the expected
           heading, up to VC_UTURN_SECTOR_DEG past it */
        p_sm->ut.sweeps++;
        ut_sweep(p_sm, VC_UT_SEARCH, back,
                 ut_counts(VC_UTURN_CMD_DEG) +
                     ((int32_t)back * ut_counts(VC_UTURN_SECTOR_DEG)),
                 now_ms, p_out);
    }
}

/* Slow sweep reached its encoder bound without a sighting */
static void ut_check_bound(vc_sm_t * p_sm, uint32_t now_ms,
                           vc_actions_t * p_out)
{
    if ((VC_EXEC_UT_SWEEP == p_sm->exec_phase) &&
        (((int32_t)p_sm->ut.dir * (p_sm->ut.heading - p_sm->ut.bound)) >= 0))
    {
        ut_brake(p_sm, 2, now_ms, p_out);
    }
}

static void ut_sample(vc_sm_t * p_sm, const vc_event_t * p_event,
                      uint32_t now_ms, vc_actions_t * p_out)
{
    int32_t mask = p_event->p1;

    /* taken before this step began (before the pivot, or while the wheels
       were still turning): stale, it proves nothing now */
    if ((int32_t)(p_event->time_ms - p_sm->ut.phase_ms) < 0)
    {
        return;
    }
    switch (p_sm->exec_phase)
    {
        case VC_EXEC_UTURN:
            /* the junction, its side arm: logged once, never acted on */
            if ((0 != (mask & (VC_LINE_C | VC_LINE_R))) &&
                (0u == p_sm->ut.early))
            {
                p_sm->ut.early = 1u;
                ut_trace(p_sm, p_out, VC_UT_EARLY, mask);
            }
            break;
        case VC_EXEC_UT_SWEEP:
            if ((VC_UT_APPROACH == p_sm->ut.sweep) &&
                (p_sm->ut.heading < ut_counts(VC_UTURN_FAST_DEG)))
            {
                break; /* not yet past the gate */
            }
            if (ut_edges(p_sm, mask))
            {
                ut_trace(p_sm, p_out, VC_UT_SIGHT, p_sm->ut.sight);
                ut_brake(p_sm, 1, now_ms, p_out);
            }
            else
            {
                ut_check_bound(p_sm, now_ms, p_out);
            }
            break;
        case VC_EXEC_UT_BRAKE:
            /* still turning: did the line pass under the sensors? */
            if (ut_edges(p_sm, mask))
            {
                ut_trace(p_sm, p_out, VC_UT_SIGHT, p_sm->ut.sight);
            }
            break;
        case VC_EXEC_UT_VERIFY:
        {
            vc_ut_verdict_t value2;
            bool            b_same;

            (void)ut_edges(p_sm, mask);
            value2 = ut_verdict(p_sm, mask);
            /* on the line and between C and R both mean aligned */
            b_same =
                (value2 == (vc_ut_verdict_t)p_sm->ut.verdict) ||
                (((VC_UT_V_ON_LINE == value2) || (VC_UT_V_BETWEEN == value2)) &&
                 ((VC_UT_V_ON_LINE == p_sm->ut.verdict) ||
                  (VC_UT_V_BETWEEN == p_sm->ut.verdict)));
            if (VC_UT_V_UNSURE == value2)
            {
                p_sm->ut.verdict_n = 0u;
            }
            else if (b_same && (0u != p_sm->ut.verdict_n))
            {
                p_sm->ut.verdict_n++;
            }
            else
            {
                p_sm->ut.verdict_n = 1u;
            }
            p_sm->ut.verdict = (uint8_t)value2;
            if (p_sm->ut.verdict_n >= VC_UTURN_VERIFY_SAMPLES)
            {
                ut_decide(p_sm, value2, now_ms, p_out);
            }
            break;
        }
        default:
            break;
    }
}

/* EXECUTE_ACTION while a U-turn runs (OBSTACLE is handled before) */
static void ut_handle(vc_sm_t * p_sm, const vc_event_t * p_event,
                      vc_event_type_t type, uint32_t now_ms,
                      vc_actions_t * p_out)
{
    if ((VC_EV_TICK == type) &&
        (elapsed(now_ms, p_sm->ut.t0_ms) >= VC_UTURN_MS))
    {
        ut_fail(p_sm, 4, now_ms, p_out);
    }
    else if ((VC_EV_MANOEUVRE_DONE == type) &&
             (VC_EXEC_UTURN == p_sm->exec_phase))
    {
        /* fast pivot done: the slow approach, same way */
        ut_trace(p_sm, p_out, VC_UT_FAST_DONE, 0);
        ut_sweep(p_sm, VC_UT_APPROACH, 1,
                 ut_counts(VC_UTURN_CMD_DEG + VC_UTURN_SECTOR_DEG), now_ms,
                 p_out);
    }
    else if ((VC_EV_MOTION_STOPPED == type) &&
             (VC_EXEC_UT_BRAKE == p_sm->exec_phase))
    {
        /* standing: how far did the wheels run on after the request? */
        ut_trace(p_sm, p_out, VC_UT_SETTLED,
                 (int32_t)p_sm->ut.dir * (p_sm->ut.heading - p_sm->ut.stop_h));
        p_sm->exec_phase   = VC_EXEC_UT_VERIFY;
        p_sm->ut.phase_ms  = now_ms;
        p_sm->ut.verdict   = (uint8_t)VC_UT_V_UNSURE;
        p_sm->ut.verdict_n = 0u;
    }
    else if (VC_EV_LINE_SAMPLE == type)
    {
        ut_sample(p_sm, p_event, now_ms, p_out);
    }
    else if (VC_EV_TICK == type)
    {
        uint32_t in_phase = elapsed(now_ms, p_sm->ut.phase_ms);

        if (VC_EXEC_UT_SWEEP == p_sm->exec_phase)
        {
            ut_check_bound(p_sm, now_ms, p_out);
            if ((VC_EXEC_UT_SWEEP == p_sm->exec_phase) &&
                (elapsed(now_ms, p_sm->ut.cmd_ms) >= VC_UTURN_REFRESH_MS))
            {
                p_sm->ut.cmd_ms = now_ms;
                move(p_out, VC_MOVE_PIVOT,
                     (int32_t)p_sm->ut.dir * VC_UTURN_SLOW_MMS);
            }
        }
        else if ((VC_EXEC_UT_BRAKE == p_sm->exec_phase) &&
                 (in_phase >= VC_UTURN_SETTLE_MS))
        {
            ut_fail(p_sm, 2, now_ms, p_out);
        }
        else if ((VC_EXEC_UT_VERIFY == p_sm->exec_phase) &&
                 (in_phase >= VC_UTURN_VERIFY_MS))
        {
            ut_fail(p_sm, 3, now_ms, p_out);
        }
        else
        {
            /* fast pivot running */
        }
    }
    else
    {
        /* a late MANOEUVRE_DONE (it can never end a slow sweep or a
           correction), LINE_FOUND, JUNCTION, barcodes: ignored */
    }
}

/* ------------------------------------------------------------------ */
/*  Events handled the same way in every state                         */
/*  Returns TRUE if the event is fully handled.                        */
/* ------------------------------------------------------------------ */

static int32_t clamp_slow(vc_move_t move_cmd, int32_t value)
{
    int32_t result = value;

    if (((VC_MOVE_FORWARD == move_cmd) || (VC_MOVE_SET_SPEED == move_cmd)) &&
        (value > VC_SPEED_SLOW_MMS))
    {
        result = VC_SPEED_SLOW_MMS;
    }
    return result;
}

/* MOVE_REQUEST from a subsystem: forward it only from the owner */
static void move_request(vc_sm_t * p_sm, const vc_event_t * p_event,
                         vc_actions_t * p_out)
{
    vc_state_t  state = p_sm->st.state;
    /* Cast: the src byte holds a vc_source_t value. */
    vc_source_t src   = (vc_source_t)p_event->src;

    /* Command arbitration: only the current owner may move us */
    if ((src != VC_SRC_NONE) && (src == vc_sm_motion_owner(state)) &&
        !((VC_ST_AVOID_OBSTACLE == state) && p_sm->avoid_wait_stop) &&
        (p_event->p1 >= 0) && (p_event->p1 < (int32_t)VC_MOVE_COUNT) &&
        (p_event->p1 != (int32_t)VC_MOVE_RESET) &&
        (p_event->p1 != (int32_t)VC_MOVE_STOP_CONFIRM) &&
        (p_event->p1 != (int32_t)VC_MOVE_PIVOT)) /* the U-turn's own */
    {
        /* Cast: p1 was range-checked against VC_MOVE_COUNT. */
        vc_move_t move_cmd = (vc_move_t)p_event->p1;
        int32_t   value    = p_event->p2;

        if ((VC_ST_DECODE_BARCODE == state) || (VC_ST_TERRAIN_CHECK == state))
        {
            value = clamp_slow(move_cmd, value);
        }
        if ((VC_SRC_OBST == src) && (VC_MOVE_STOP == move_cmd))
        {
            /* S5 waits for both wheels to stop before leaving a search leg. */
            move_cmd = VC_MOVE_STOP_CONFIRM;
        }
        if ((VC_ST_FOLLOW_LINE == state) && (VC_MOVE_STEER == move_cmd) &&
            (0u != p_sm->rejoin_steer_left))
        {
            p_sm->rejoin_steer_left--;
            if (0u == p_sm->rejoin_steer_left)
            {
                move(p_out, VC_MOVE_SET_SPEED, VC_SPEED_CRUISE_MMS);
            }
        }
        move(p_out, move_cmd, value);
    }
    else
    {
        p_sm->st.moves_rejected++;
    }
}

/* FAULT from a subsystem: act on its severity */
static void fault_event(vc_sm_t * p_sm, const vc_event_t * p_event,
                        uint32_t now_ms, vc_actions_t * p_out)
{
    vc_state_t  state = p_sm->st.state;
    /* Cast: the src byte holds a vc_source_t value. */
    vc_source_t src   = (vc_source_t)p_event->src;

    /* Casts: p1 holds a vc_fault_severity_t, p2 an unsigned fault code */
    if ((int32_t)VC_FAULT_CRITICAL == p_event->p1)
    {
        if (vc_sm_is_active(state))
        {
            go_fault(p_sm, src, (uint32_t)p_event->p2, now_ms, p_out);
        }
        else
        {
            report(p_sm, p_out, src, VC_FAULT_CRITICAL, (uint32_t)p_event->p2);
        }
    }
    else if ((int32_t)VC_FAULT_RECOVERABLE == p_event->p1)
    {
        report(p_sm, p_out, src, VC_FAULT_RECOVERABLE, (uint32_t)p_event->p2);
        /* A recoverable line fault starts bounded line recovery;
           every other subsystem runs its own bounded recovery and
           escalates with a CRITICAL fault if that fails. */
        if ((VC_SRC_LINE == src) && (VC_ST_FOLLOW_LINE == state))
        {
            p_sm->lost_side = 0;
            enter(p_sm, VC_ST_LOST_LINE, now_ms, p_out);
        }
    }
    else
    {
        report(p_sm, p_out, src, VC_FAULT_NON_CRITICAL, (uint32_t)p_event->p2);
    }
}

/* Lift = the end of the run (professor).  A lift while the mission is
   running goes to MISSION_COMPLETE with motors stopped; it stays there
   until Reset (then Start).  Before the mission has started (IDLE,
   MISSION_START) it is reported as a critical fault and starts nothing.
   After the run has ended (FAULT, MISSION_COMPLETE) the robot is already
   stopped and the cause that ended the run stays: a lift never turns a
   fault into a completed mission. */
static void lift(vc_sm_t * p_sm, vc_source_t src, uint32_t code,
                 uint32_t now_ms, vc_actions_t * p_out)
{
    vc_state_t state = p_sm->st.state;

    if (vc_sm_is_active(state) && (VC_ST_MISSION_START != state))
    {
        drop_pending(p_sm, p_out);
        enter(p_sm, VC_ST_MISSION_COMPLETE, now_ms, p_out);
        set_cause(p_sm, src, VC_FAULT_NON_CRITICAL, code);
    }
    else if (vc_sm_is_active(state))
    {
        go_fault(p_sm, src, code, now_ms, p_out);
    }
    else if (!run_ended(state))
    {
        report(p_sm, p_out, src, VC_FAULT_CRITICAL, code);
    }
    else
    {
        /* already stopped: nothing to do */
    }
}

/* S2 safe-stopped because S4's lift / overturn interlock latched it
   (vc_event_t.cause, set by vc_bridge_s2.c from S2's own record).  S2's
   report usually arrives before S4's typed event (S2 is read first) and
   S4's event may be dropped, so this one report must already end the run
   as the IMU stop: a lift as MISSION_COMPLETE, an overturn (or an IMU stop
   of unknown kind) as FAULT IMU 0x4005.  The 0x2011 report is still sent.
   Motors: S2 is already latched off; lift() / go_fault() stop them again.
   A safe stop WITHOUT an IMU cause stays a critical 0x2011 (fault_event). */
static void imu_safe_stop(vc_sm_t * p_sm, const vc_event_t * p_event,
                          uint32_t now_ms, vc_actions_t * p_out)
{
    report(p_sm, p_out, VC_SRC_MOTION, VC_FAULT_CRITICAL, VC_FC_S2_SAFE_STOP);
    if (VC_CAUSE_IMU_LIFT == p_event->cause)
    {
        lift(p_sm, VC_SRC_IMU, VC_FC_IMU_SAFETY, now_ms, p_out);
    }
    else if (vc_sm_is_active(p_sm->st.state))
    {
        go_fault(p_sm, VC_SRC_IMU, VC_FC_IMU_SAFETY, now_ms, p_out);
    }
    else
    {
        report(p_sm, p_out, VC_SRC_IMU, VC_FAULT_CRITICAL, VC_FC_IMU_SAFETY);
    }
}

static bool handle_common(vc_sm_t * p_sm, const vc_event_t * p_event,
                          uint32_t now_ms, vc_actions_t * p_out)
{
    vc_state_t  state    = p_sm->st.state;
    bool        b_active = vc_sm_is_active(state);
    /* Cast: the src byte holds a vc_source_t value. */
    vc_source_t src      = (vc_source_t)p_event->src;
    bool        b_handled;

    switch ((vc_event_type_t)p_event->type)
    {
        case VC_EV_MOVE_REQUEST:
            move_request(p_sm, p_event, p_out);
            b_handled = true;
            break;

        case VC_EV_FAULT:
            if ((VC_SRC_MOTION == src) &&
                ((int32_t)VC_FC_S2_SAFE_STOP == p_event->p2) &&
                (VC_CAUSE_NONE != p_event->cause))
            {
                imu_safe_stop(p_sm, p_event, now_ms, p_out);
            }
            else
            {
                fault_event(p_sm, p_event, now_ms, p_out);
            }
            b_handled = true;
            break;

        case VC_EV_LIFTED:
            lift(p_sm, src, (uint32_t)p_event->p2, now_ms, p_out);
            b_handled = true;
            break;

        case VC_EV_CMD_STOP:
            if (b_active)
            {
                go_fault(p_sm, src, VC_FC_OPERATOR_STOP, now_ms, p_out);
            }
            b_handled = true;
            break;

        case VC_EV_WIFI_DOWN:
            /* Non-critical (FR8): report, keep navigating */
            p_sm->st.wifi_up = false;
            report(p_sm, p_out, VC_SRC_COMMS, VC_FAULT_NON_CRITICAL,
                   VC_FC_WIFI_DOWN);
            b_handled = true;
            break;

        case VC_EV_WIFI_UP:
            p_sm->st.wifi_up = true;
            b_handled        = true;
            break;

        case VC_EV_CMD_CONFIG:
            /* Barcode map edits only while no mission is running (also
               paused: still active).  The state does not change. */
            (void)emit(p_out, VC_ACT_CONFIG, VC_SRC_COMMS, b_active ? 0 : 1,
                       p_event->p1);
            b_handled = true;
            break;

        default:
            b_handled = false;
            break;
    }
    return b_handled;
}

/* ------------------------------------------------------------------ */
/*  Per-state behaviour                                                */
/* ------------------------------------------------------------------ */

void vc_sm_init(vc_sm_t * p_sm, uint32_t now_ms)
{
    vc_sm_t zero = { 0 };

    *p_sm                   = zero;
    p_sm->st.state          = VC_ST_IDLE;
    p_sm->st.prev_state     = VC_ST_IDLE;
    p_sm->st.state_since_ms = now_ms;
    p_sm->st.last_nav       = NAV_NONE;
    p_sm->pending_nav       = NAV_NONE;
}

/* Per-state handlers.  Each one gets the same arguments: the machine,
   the event and its type, the time in the current state, the time now
   and the actions to fill in. */
static void on_mission_start(vc_sm_t * p_sm, const vc_event_t * p_event,
                             vc_event_type_t type, uint32_t in_state,
                             uint32_t now_ms, vc_actions_t * p_out)
{
    if (VC_EV_READY == type)
    {
        p_sm->ready_mask |= VC_READY_BIT(p_event->src);
        if (VC_READY_ALL == (p_sm->ready_mask & VC_READY_ALL))
        {
            enter(p_sm, VC_ST_FOLLOW_LINE, now_ms, p_out);
        }
    }
    else if ((VC_EV_TICK == type) && (in_state >= VC_SELF_CHECK_MS))
    {
        if (VC_READY_REQUIRED == (p_sm->ready_mask & VC_READY_REQUIRED))
        {
            /* optional subsystem missing: run degraded */
            report(p_sm, p_out, VC_SRC_VC, VC_FAULT_NON_CRITICAL,
                   VC_FC_SELF_CHECK);
            enter(p_sm, VC_ST_FOLLOW_LINE, now_ms, p_out);
        }
        else
        {
            go_fault(p_sm, VC_SRC_VC, VC_FC_SELF_CHECK, now_ms, p_out);
        }
    }
    else
    {
        /* keep waiting for READY */
    }
}

/* A decoded barcode is only a pending instruction.  Its action is held
   until the robot reaches the next junction. */
static void take_barcode(vc_sm_t * p_sm, const vc_event_t * p_event,
                         uint32_t now_ms, vc_actions_t * p_out)
{
    uint32_t idx;

    /* Store the complete decoded text. The configured rule table matches
       the whole string; an unknown string deliberately becomes NAV_NONE. */
    for (idx = 0u; idx < sizeof(p_sm->pending_barcode) - 1u; idx++)
    {
        p_sm->pending_barcode[idx] = p_event->barcode_text[idx];
        if ('\0' == p_event->barcode_text[idx])
        {
            break;
        }
    }
    p_sm->pending_barcode[sizeof(p_sm->pending_barcode) - 1u] = '\0';
    p_sm->pending_nav = vc_barcode_lookup(p_sm->pending_barcode);
    /* Backward-compatible event producers may still send only p1=nav.
       The real S3 bridge supplies barcode_text, so full-string rules win. */
    if (('\0' == p_sm->pending_barcode[0]) &&
        (p_event->p1 > (int32_t)NAV_NONE) &&
        (p_event->p1 <= (int32_t)NAV_U_TURN))
    {
        p_sm->pending_nav = (navigation_command_t)p_event->p1;
    }
    p_sm->st.last_nav = p_sm->pending_nav;
    junction_note(p_sm, p_out, VC_JX_ASSIGNED);
    for (idx = 0u; idx < sizeof(p_sm->st.last_barcode); idx++)
    {
        p_sm->st.last_barcode[idx] = p_sm->pending_barcode[idx];
        if ('\0' == p_sm->pending_barcode[idx])
        {
            break;
        }
    }
    p_sm->st.last_barcode[sizeof(p_sm->st.last_barcode) - 1u] = '\0';
    p_sm->st.barcodes_read++;
    enter(p_sm, VC_ST_FOLLOW_LINE, now_ms, p_out);
}

static void on_follow_line(vc_sm_t * p_sm, const vc_event_t * p_event,
                           vc_event_type_t type, uint32_t in_state,
                           uint32_t now_ms, vc_actions_t * p_out)
{
    (void)in_state;
    switch (type)
    {
        case VC_EV_OBSTACLE: /* safety first */
            enter(p_sm, VC_ST_AVOID_OBSTACLE, now_ms, p_out);
            break;
        case VC_EV_LINE_LOST:
            /* Cast: p1 is the last side seen, -1, 0 or +1. */
            p_sm->lost_side = (int8_t)p_event->p1;
            enter(p_sm, VC_ST_LOST_LINE, now_ms, p_out);
            break;
        case VC_EV_JUNCTION:
            /* Stop first. The dedicated state waits for S2's physical-stop
               confirmation before selecting and executing the buffered rule. */
            enter(p_sm, VC_ST_JUNCTION_STOP, now_ms, p_out);
            break;
        case VC_EV_BARCODE_START:
            enter(p_sm, VC_ST_DECODE_BARCODE, now_ms, p_out);
            break;
        case VC_EV_BARCODE_DECODED: /* S3 decoded without a START */
            take_barcode(p_sm, p_event, now_ms, p_out);
            break;
        case VC_EV_HUMP_START:
            enter(p_sm, VC_ST_TERRAIN_CHECK, now_ms, p_out);
            break;
        case VC_EV_END_OF_COURSE:
            enter(p_sm, VC_ST_MISSION_COMPLETE, now_ms, p_out);
            break;
        default:
            break; /* JUNCTION without a barcode: keep following */
    }
}

static void on_lost_line(vc_sm_t * p_sm, const vc_event_t * p_event,
                         vc_event_type_t type, uint32_t in_state,
                         uint32_t now_ms, vc_actions_t * p_out)
{
    (void)p_event;
    (void)in_state;
    if (VC_EV_LINE_FOUND == type)
    {
        enter(p_sm, VC_ST_FOLLOW_LINE, now_ms, p_out);
    }
    else if (VC_EV_OBSTACLE == type)
    {
        enter(p_sm, VC_ST_AVOID_OBSTACLE, now_ms, p_out);
    }
    else if ((VC_EV_TICK == type) &&
             (elapsed(now_ms, p_sm->attempt_since_ms) >= VC_LINE_ATTEMPT_MS))
    {
        p_sm->attempts++;
        if (p_sm->attempts > VC_LINE_MAX_ATTEMPTS)
        {
            go_fault(p_sm, VC_SRC_VC, VC_FC_LINE_RECOVERY, now_ms, p_out);
        }
        else
        {
            p_sm->attempt_since_ms = now_ms;
            search_step(p_sm, p_out);
        }
    }
    else
    {
        /* keep searching */
    }
}

static void on_avoid_obstacle(vc_sm_t * p_sm, const vc_event_t * p_event,
                              vc_event_type_t type, uint32_t in_state,
                              uint32_t now_ms, vc_actions_t * p_out)
{
    if (p_sm->avoid_wait_stop)
    {
        if ((VC_EV_MOTION_STOPPED == type) && (VC_SRC_MOTION == p_event->src))
        {
            p_sm->avoid_wait_stop = false;
            request(p_out, VC_SRC_OBST, VC_REQ_OBST_AVOID, 0);
        }
        else if ((VC_EV_TICK == type) && (in_state >= VC_JUNCTION_STOP_MS))
        {
            go_fault(p_sm, VC_SRC_MOTION, VC_FC_STATE_TIMEOUT, now_ms, p_out);
        }
        else
        {
            /* no other case needs any action */
        }
    }
    else if (VC_EV_AVOID_DONE == type)
    {
        enter(p_sm, VC_ST_FOLLOW_LINE, now_ms, p_out);
    }
    else if (VC_EV_AVOID_FAILED == type)
    {
        /* S5 has already searched around the obstacle. Stop here. */
        go_fault(p_sm, VC_SRC_OBST, VC_FC_AVOID_FAILED, now_ms, p_out);
    }
    else if ((VC_EV_TICK == type) && (in_state >= VC_AVOID_MS))
    {
        go_fault(p_sm, VC_SRC_VC, VC_FC_STATE_TIMEOUT, now_ms, p_out);
    }
    else
    {
        /* S5 is still working */
    }
}

/* DECODE_BARCODE: a failed read, or no result in time: retry or give up */
static void barcode_retry(vc_sm_t * p_sm, uint32_t now_ms, vc_actions_t * p_out)
{
    p_sm->retries++;
    if (p_sm->retries > VC_BARCODE_MAX_RETRY)
    {
        /* give up on this barcode, keep the mission going */
        report(p_sm, p_out, VC_SRC_LINE, VC_FAULT_NON_CRITICAL, VC_FC_BARCODE);
        enter(p_sm, VC_ST_FOLLOW_LINE, now_ms, p_out);
    }
    else
    {
        p_sm->attempt_since_ms = now_ms;
        /* Cast: the retry count is small. */
        request(p_out, VC_SRC_LINE, VC_REQ_BARCODE_READ,
                (int32_t)p_sm->retries);
    }
}

static void on_decode_barcode(vc_sm_t * p_sm, const vc_event_t * p_event,
                              vc_event_type_t type, uint32_t in_state,
                              uint32_t now_ms, vc_actions_t * p_out)
{
    (void)in_state;
    switch (type)
    {
        case VC_EV_OBSTACLE:
            enter(p_sm, VC_ST_AVOID_OBSTACLE, now_ms, p_out);
            break;
        case VC_EV_BARCODE_DECODED:
            take_barcode(p_sm, p_event, now_ms, p_out);
            break;
        case VC_EV_END_OF_COURSE:
            enter(p_sm, VC_ST_MISSION_COMPLETE, now_ms, p_out);
            break;
        case VC_EV_BARCODE_FAILED:
            barcode_retry(p_sm, now_ms, p_out);
            break;
        case VC_EV_JUNCTION:
            /* S3 reports no junctions while it is reading bars, so this is
               a real crossing reached after the read ended.  It used to be
               dropped here and the robot drove through the junction. */
            enter(p_sm, VC_ST_JUNCTION_STOP, now_ms, p_out);
            break;
        case VC_EV_TICK:
            if (elapsed(now_ms, p_sm->attempt_since_ms) >= VC_DECODE_MS)
            {
                barcode_retry(p_sm, now_ms, p_out);
            }
            break;
        default:
            break; /* LINE_LOST ignored while reading */
    }
}

static void on_junction_stop(vc_sm_t * p_sm, const vc_event_t * p_event,
                             vc_event_type_t type, uint32_t in_state,
                             uint32_t now_ms, vc_actions_t * p_out)
{
    (void)p_event;
    if ((VC_EV_MOTION_STOPPED == type) && (0u == p_sm->junction_stopped))
    {
        /* standing still: hold here for VC_JUNCTION_HOLD_MS, then act */
        p_sm->junction_stopped = 1u;
        p_sm->junction_stop_ms = now_ms;
    }
    if ((0u != p_sm->junction_stopped) &&
        ((VC_EV_MOTION_STOPPED == type) || (VC_EV_TICK == type)))
    {
        const uint32_t hold_ms = (uint32_t)VC_JUNCTION_HOLD_MS;

        if (elapsed(now_ms, p_sm->junction_stop_ms) >= hold_ms)
        {
            /* No mapping (including an unknown barcode) means continue
               straight. */
            if (NAV_NONE == p_sm->pending_nav)
            {
                p_sm->pending_nav = NAV_STRAIGHT;
            }
            p_sm->junction_stopped = 0u;
            enter(p_sm, VC_ST_EXECUTE_ACTION, now_ms, p_out);
        }
    }
    else if ((VC_EV_TICK == type) && (in_state >= VC_JUNCTION_STOP_MS))
    {
        go_fault(p_sm, VC_SRC_MOTION, VC_FC_JUNCTION_STOP, now_ms, p_out);
    }
    else if (VC_EV_END_OF_COURSE == type)
    {
        drop_pending(p_sm, p_out);
        enter(p_sm, VC_ST_MISSION_COMPLETE, now_ms, p_out);
    }
    else if (VC_EV_OBSTACLE == type)
    {
        /* Obstacle safety takes priority; discard the pending junction action.
         */
        drop_pending(p_sm, p_out);
        enter(p_sm, VC_ST_AVOID_OBSTACLE, now_ms, p_out);
    }
    else
    {
        /* no other case needs any action */
    }
}

static void on_execute_action(vc_sm_t * p_sm, const vc_event_t * p_event,
                              vc_event_type_t type, uint32_t in_state,
                              uint32_t now_ms, vc_actions_t * p_out)
{
    if (VC_EV_OBSTACLE == type)
    {
        /* obstacle safety first, also in the middle of a U-turn */
        drop_pending(p_sm, p_out);
        enter(p_sm, VC_ST_AVOID_OBSTACLE, now_ms, p_out);
    }
    else if (ut_active(p_sm))
    {
        ut_handle(p_sm, p_event, type, now_ms, p_out);
    }
    else if ((VC_EV_MANOEUVRE_DONE == type) &&
             (VC_EXEC_ADVANCE == p_sm->exec_phase))
    {
        /* the advance is done: the axle is over the crossing, now turn */
        p_sm->exec_phase = VC_EXEC_TURN;
        start_turn(p_sm,
                   (NAV_LEFT == p_sm->pending_nav) ? VC_MOVE_TURN_LEFT
                                                   : VC_MOVE_TURN_RIGHT,
                   VC_TURN_CMD_DEG, VC_TURN_GATE_DEG, p_out);
    }
    else if ((VC_EV_LINE_FOUND == type) && (VC_EXEC_TURN == p_sm->exec_phase) &&
             (0u != p_sm->turn_gate) &&
             ((p_sm->odo - p_sm->turn_odo0) >= p_sm->turn_gate))
    {
        /* past the gate and C or R is on the new arm: the turn is done,
           line following takes over (its FORWARD replaces S2's turn) */
        p_sm->turn_gate = 0u;
        junction_note(p_sm, p_out, VC_JX_COMPLETED);
        p_sm->pending_nav        = NAV_NONE;
        p_sm->pending_barcode[0] = '\0';
        enter(p_sm, VC_ST_FOLLOW_LINE, now_ms, p_out);
    }
    else if (VC_EV_MANOEUVRE_DONE == type)
    {
        p_sm->turn_gate = 0u;
        junction_note(p_sm, p_out, VC_JX_COMPLETED);
        p_sm->pending_nav        = NAV_NONE;
        p_sm->pending_barcode[0] = '\0';
        enter(p_sm, VC_ST_FOLLOW_LINE, now_ms, p_out);
    }
    else if ((VC_EV_TICK == type) && (in_state >= VC_EXECUTE_MS))
    {
        /* manoeuvre never finished: look for the line */
        report(p_sm, p_out, VC_SRC_MOTION, VC_FAULT_RECOVERABLE,
               VC_FC_STATE_TIMEOUT);
        drop_pending(p_sm, p_out);
        enter(p_sm, VC_ST_LOST_LINE, now_ms, p_out);
    }
    else
    {
        /* manoeuvre still running */
    }
}

static void on_terrain_check(vc_sm_t * p_sm, const vc_event_t * p_event,
                             vc_event_type_t type, uint32_t in_state,
                             uint32_t now_ms, vc_actions_t * p_out)
{
    (void)p_event;
    if (VC_EV_HUMP_CLEARED == type)
    {
        enter(p_sm, VC_ST_FOLLOW_LINE, now_ms, p_out);
    }
    else if (VC_EV_OBSTACLE == type)
    {
        enter(p_sm, VC_ST_AVOID_OBSTACLE, now_ms, p_out);
    }
    else if ((VC_EV_TICK == type) && (in_state >= VC_TERRAIN_MS))
    {
        report(p_sm, p_out, VC_SRC_IMU, VC_FAULT_RECOVERABLE,
               VC_FC_STATE_TIMEOUT);
        enter(p_sm, VC_ST_FOLLOW_LINE, now_ms, p_out);
    }
    else
    {
        /* still on the hump */
    }
}

void vc_sm_handle(vc_sm_t * p_sm, const vc_event_t * p_event, uint32_t now_ms,
                  vc_actions_t * p_out)
{
    /* Cast: the type byte holds a vc_event_type_t value. */
    vc_event_type_t type     = (vc_event_type_t)p_event->type;
    uint32_t        in_state = elapsed(now_ms, p_sm->st.state_since_ms);

    p_out->n        = 0;
    p_out->overflow = false;
    if (VC_EV_TICK != type)
    {
        p_sm->st.events_handled++;
    }
    /* S2 odometer (L + R counts) rides in p2 of TICK, of the LINE_FOUND /
       LINE_SAMPLE sent during a junction turn, and of S2's MANOEUVRE_DONE
       and MOTION_STOPPED; 0 = not given */
    if (((VC_EV_TICK == type) || (VC_EV_LINE_FOUND == type) ||
         (VC_EV_LINE_SAMPLE == type) ||
         (((VC_EV_MANOEUVRE_DONE == type) || (VC_EV_MOTION_STOPPED == type)) &&
          (VC_SRC_MOTION == p_event->src))) &&
        (0 != p_event->p2))
    {
        ut_track_odo(p_sm, (uint32_t)p_event->p2); /* Cast: counts, sent as
                                                      int32 */
        p_sm->odo = (uint32_t)p_event->p2;
    }

    /* a hump may also be crossed while reading a barcode or turning:
       always keep the highest one (FR5) */
    if ((VC_EV_HUMP_CLEARED == type) && (p_event->p1 > p_sm->st.max_hump_um))
    {
        p_sm->st.max_hump_um = p_event->p1;
    }

    if (VC_EV_CMD_PAUSE == type)
    {
        if ((VC_ST_FOLLOW_LINE == p_sm->st.state) && !p_sm->st.paused)
        {
            p_sm->st.paused      = true;
            p_sm->pause_stopped  = false;
            p_sm->pause_since_ms = now_ms;
            move(p_out, VC_MOVE_STOP_CONFIRM, 0);
        }
        return;
    }
    if (VC_EV_CMD_RESUME == type)
    {
        if (p_sm->st.paused && p_sm->pause_stopped)
        {
            p_sm->st.paused     = false;
            p_sm->pause_stopped = false;
            request(p_out, VC_SRC_LINE, VC_REQ_LINE_FOLLOW, 0);
            if (0u != p_sm->rejoin_steer_left)
            {
                move(p_out, VC_MOVE_SET_SPEED, VC_SPEED_SLOW_MMS);
            }
            else
            {
                move(p_out, VC_MOVE_SET_SPEED, VC_SPEED_CRUISE_MMS);
                move(p_out, VC_MOVE_FORWARD, VC_SPEED_CRUISE_MMS);
            }
        }
        return;
    }
    if (p_sm->st.paused)
    {
        if (VC_EV_MOTION_STOPPED == type)
        {
            p_sm->pause_stopped = true;
            return;
        }
        if ((VC_EV_TICK == type) && !p_sm->pause_stopped &&
            (elapsed(now_ms, p_sm->pause_since_ms) >= VC_JUNCTION_STOP_MS))
        {
            go_fault(p_sm, VC_SRC_MOTION, VC_FC_JUNCTION_STOP, now_ms, p_out);
            return;
        }
        if ((VC_EV_OBSTACLE == type) || (VC_EV_LINE_LOST == type) ||
            (VC_EV_JUNCTION == type) || (VC_EV_BARCODE_START == type) ||
            (VC_EV_BARCODE_DECODED == type) || (VC_EV_HUMP_START == type) ||
            (VC_EV_END_OF_COURSE == type) ||
            ((VC_EV_FAULT == type) &&
             ((int32_t)VC_FAULT_RECOVERABLE == p_event->p1)))
        {
            go_fault(p_sm, (vc_source_t)p_event->src, VC_FC_PAUSE_INTERRUPTED,
                     now_ms, p_out);
            return;
        }
        if ((VC_EV_MOVE_REQUEST == type) || (VC_EV_TICK == type))
        {
            return;
        }
        if (handle_common(p_sm, p_event, now_ms, p_out))
        {
            return;
        }
        return;
    }

    if (!handle_common(p_sm, p_event, now_ms, p_out))
    {
        switch (p_sm->st.state)
        {
            case VC_ST_IDLE:
                if (VC_EV_CMD_START == type)
                {
                    /* Start acts as Reset then Start, as it already does
                       after MISSION_COMPLETE: a lift while idle (e.g.
                       placing the robot on the track) leaves S2 latched in
                       SAFE_STOP, and without this reset S2 never reports
                       READY (self check 0x0002).  S2 refuses the reset
                       while S4's lift / overturn interlock is still set,
                       so a robot that is still lifted stays stopped. */
                    move(p_out, VC_MOVE_RESET, 0);
                    enter(p_sm, VC_ST_MISSION_START, now_ms, p_out);
                }
                else if (VC_EV_CMD_RESET == type)
                {
                    move(p_out, VC_MOVE_RESET, 0);
                }
                else
                {
                    /* no other case needs any action */
                }
                break;
            case VC_ST_MISSION_START:
                on_mission_start(p_sm, p_event, type, in_state, now_ms, p_out);
                break;
            case VC_ST_FOLLOW_LINE:
                on_follow_line(p_sm, p_event, type, in_state, now_ms, p_out);
                break;
            case VC_ST_JUNCTION_STOP:
                on_junction_stop(p_sm, p_event, type, in_state, now_ms, p_out);
                break;
            case VC_ST_LOST_LINE:
                on_lost_line(p_sm, p_event, type, in_state, now_ms, p_out);
                break;
            case VC_ST_AVOID_OBSTACLE:
                on_avoid_obstacle(p_sm, p_event, type, in_state, now_ms, p_out);
                break;
            case VC_ST_DECODE_BARCODE:
                on_decode_barcode(p_sm, p_event, type, in_state, now_ms, p_out);
                break;
            case VC_ST_EXECUTE_ACTION:
                on_execute_action(p_sm, p_event, type, in_state, now_ms, p_out);
                break;
            case VC_ST_TERRAIN_CHECK:
                on_terrain_check(p_sm, p_event, type, in_state, now_ms, p_out);
                break;
            case VC_ST_FAULT:
                /* stay stopped until Reset (overturn, operator stop, ...) */
                if (VC_EV_CMD_RESET == type)
                {
                    enter(p_sm, VC_ST_IDLE, now_ms, p_out);
                }
                break;
            case VC_ST_MISSION_COMPLETE:
                /* Motors stay off until an explicit Start or Reset (owner,
                   2026-11-14).  Start acts as Reset then Start: it clears
                   the S2 latch (via IDLE) and begins a new mission. */
                if (VC_EV_CMD_RESET == type)
                {
                    enter(p_sm, VC_ST_IDLE, now_ms, p_out);
                }
                else if (VC_EV_CMD_START == type)
                {
                    enter(p_sm, VC_ST_IDLE, now_ms, p_out);
                    enter(p_sm, VC_ST_MISSION_START, now_ms, p_out);
                }
                else
                {
                    /* everything else is ignored: stay stopped */
                }
                break;
            default:
                /* corrupt state: fail safe */
                go_fault(p_sm, VC_SRC_VC, VC_FC_STATE_TIMEOUT, now_ms, p_out);
                break;
        }
    }
}

/*** end of file ***/
