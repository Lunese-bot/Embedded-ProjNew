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
 * EXECUTE_ACTION
 *                                                       --manoeuvre_done-->
 * FOLLOW_LINE
 *                    --hump_start------> TERRAIN_CHECK  --cleared-> FOLLOW_LINE
 *                    --end_of_course---> MISSION_COMPLETE
 *        any active state --critical fault / impact / stop--> FAULT --reset-->
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

/* ------------------------------------------------------------------ */
/*  Names, for console and telemetry                                   */
/* ------------------------------------------------------------------ */

static const char * const g_state_names[VC_ST_COUNT] = {
    "IDLE",           "MISSION_START",   "FOLLOW_LINE",    "LOST_LINE",
    "AVOID_OBSTACLE", "DECODE_BARCODE",  "EXECUTE_ACTION", "TERRAIN_CHECK",
    "FAULT",          "MISSION_COMPLETE"
};

static const char * const g_event_names[VC_EV_COUNT] = {
    "NONE",          "READY",          "MOVE_REQUEST",    "FAULT",
    "CMD_START",     "CMD_STOP",       "CMD_RESET",       "WIFI_DOWN",
    "WIFI_UP",       "MANOEUVRE_DONE", "LINE_LOST",       "LINE_FOUND",
    "JUNCTION",      "BARCODE_START",  "BARCODE_DECODED", "BARCODE_FAILED",
    "END_OF_COURSE", "HUMP_START",     "HUMP_CLEARED",    "IMPACT",
    "OBSTACLE",      "AVOID_DONE",     "AVOID_FAILED",    "TICK"
};

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

static void emit(vc_actions_t * p_out, vc_action_kind_t kind,
                 vc_source_t target, int32_t param1, int32_t param2)
{
    vc_action_t * p_action;

    if (p_out->n >= VC_MAX_ACTIONS)
    {
        p_out->overflow = true;
        return;
    }
    p_action         = &p_out->a[p_out->n++];
    p_action->kind   = (uint8_t)kind;
    p_action->target = (uint8_t)target;
    p_action->rsv    = 0;
    p_action->p1     = param1;
    p_action->p2     = param2;
}

static void move(vc_actions_t * p_out, vc_move_t move_cmd, int32_t value)
{
    emit(p_out, VC_ACT_MOVE, VC_SRC_MOTION, (int32_t)move_cmd, value);
}

static void request(vc_actions_t * p_out, vc_source_t target, vc_request_t req,
                    int32_t arg)
{
    emit(p_out, VC_ACT_REQUEST, target, (int32_t)req, arg);
}

static void report(vc_sm_t * p_sm, vc_actions_t * p_out, vc_source_t src,
                   vc_fault_severity_t sev, uint32_t code)
{
    p_sm->st.last_fault_code = code;
    p_sm->st.last_fault_sev  = sev;
    p_sm->st.last_fault_src  = (uint8_t)src;
    emit(p_out, VC_ACT_REPORT, src, (int32_t)sev, (int32_t)code);
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

static void enter_execute(vc_sm_t * p_sm, uint32_t now_ms, vc_actions_t * p_out)
{
    switch (p_sm->pending_nav)
    {
        case NAV_LEFT:
            move(p_out, VC_MOVE_TURN_LEFT, VC_TURN_DEG);
            break;
        case NAV_RIGHT:
            move(p_out, VC_MOVE_TURN_RIGHT, VC_TURN_DEG);
            break;
        case NAV_U_TURN:
            move(p_out, VC_MOVE_U_TURN, 0);
            break;
        case NAV_STRAIGHT:
        case NAV_NONE:
        default:
            /* nothing to do: carry straight on along the line */
            enter(p_sm, VC_ST_FOLLOW_LINE, now_ms, p_out);
            break;
    }
}

static void enter(vc_sm_t * p_sm, vc_state_t next, uint32_t now_ms,
                  vc_actions_t * p_out)
{
    vc_state_t prev = p_sm->st.state;

    p_sm->st.prev_state     = prev;
    p_sm->st.state          = next;
    p_sm->st.state_since_ms = now_ms;
    p_sm->attempt_since_ms  = now_ms;
    emit(p_out, VC_ACT_STATE, VC_SRC_VC, (int32_t)next, (int32_t)prev);

    switch (next)
    {
        case VC_ST_IDLE:
            if (VC_ST_FAULT == prev)
            {
                move(p_out, VC_MOVE_RESET, 0); /* clear S2's latch */
            }
            move(p_out, VC_MOVE_STOP, 0);
            break;

        case VC_ST_MISSION_START:
            move(p_out, VC_MOVE_STOP, 0);
            p_sm->ready_mask          = 0;
            p_sm->st.mission_start_ms = now_ms;
            p_sm->st.max_hump_um      = 0;
            p_sm->st.barcodes_read    = 0;
            p_sm->st.last_nav         = NAV_NONE;
            request(p_out, VC_SRC_NONE, VC_REQ_SELF_CHECK, 0);
            break;

        case VC_ST_FOLLOW_LINE:
            p_sm->retries = 0;
            request(p_out, VC_SRC_OBST, VC_REQ_OBST_MONITOR, 0);
            request(p_out, VC_SRC_LINE, VC_REQ_LINE_FOLLOW, 0);
            move(p_out, VC_MOVE_SET_SPEED, VC_SPEED_CRUISE_MMS);
            move(p_out, VC_MOVE_FORWARD, VC_SPEED_CRUISE_MMS);
            break;

        case VC_ST_LOST_LINE:
            move(p_out, VC_MOVE_STOP, 0);
            move(p_out, VC_MOVE_SET_SPEED, VC_SPEED_SLOW_MMS);
            p_sm->attempts = 1;
            search_step(p_sm, p_out);
            break;

        case VC_ST_AVOID_OBSTACLE:
            move(p_out, VC_MOVE_STOP, 0);
            request(p_out, VC_SRC_LINE, VC_REQ_BARCODE_OFF, 0);
            request(p_out, VC_SRC_OBST, VC_REQ_OBST_AVOID, 0);
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
        (p_event->p1 >= 0) && (p_event->p1 < (int32_t)VC_MOVE_COUNT) &&
        (p_event->p1 != (int32_t)VC_MOVE_RESET))
    {
        /* Cast: p1 was range-checked against VC_MOVE_COUNT. */
        vc_move_t move_cmd = (vc_move_t)p_event->p1;
        int32_t   value    = p_event->p2;

        if ((VC_ST_DECODE_BARCODE == state) || (VC_ST_TERRAIN_CHECK == state))
        {
            value = clamp_slow(move_cmd, value);
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
            fault_event(p_sm, p_event, now_ms, p_out);
            b_handled = true;
            break;

        case VC_EV_IMPACT:
            if (b_active)
            {
                go_fault(p_sm, src, VC_FC_IMPACT, now_ms, p_out);
            }
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

/* A barcode was decoded: remember it and carry out its action */
static void take_barcode(vc_sm_t * p_sm, const vc_event_t * p_event,
                         uint32_t now_ms, vc_actions_t * p_out)
{
    /* Cast: p1 carries a navigation_command_t value. */
    p_sm->pending_nav = (navigation_command_t)p_event->p1;
    p_sm->st.last_nav = p_sm->pending_nav;
    p_sm->st.barcodes_read++;
    enter(p_sm, VC_ST_EXECUTE_ACTION, now_ms, p_out);
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
    (void)p_event;
    if (VC_EV_AVOID_DONE == type)
    {
        enter(p_sm, VC_ST_FOLLOW_LINE, now_ms, p_out);
    }
    else if (VC_EV_AVOID_FAILED == type)
    {
        /* recoverable (FR8): one more bounded search for the line
           with S3; LOST_LINE ends in FAULT if that fails too */
        report(p_sm, p_out, VC_SRC_OBST, VC_FAULT_RECOVERABLE,
               VC_FC_AVOID_FAILED);
        enter(p_sm, VC_ST_LOST_LINE, now_ms, p_out);
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

static void on_execute_action(vc_sm_t * p_sm, const vc_event_t * p_event,
                              vc_event_type_t type, uint32_t in_state,
                              uint32_t now_ms, vc_actions_t * p_out)
{
    (void)p_event;
    if (VC_EV_MANOEUVRE_DONE == type)
    {
        enter(p_sm, VC_ST_FOLLOW_LINE, now_ms, p_out);
    }
    else if (VC_EV_OBSTACLE == type)
    {
        enter(p_sm, VC_ST_AVOID_OBSTACLE, now_ms, p_out);
    }
    else if ((VC_EV_TICK == type) && (in_state >= VC_EXECUTE_MS))
    {
        /* manoeuvre never finished: look for the line */
        report(p_sm, p_out, VC_SRC_MOTION, VC_FAULT_RECOVERABLE,
               VC_FC_STATE_TIMEOUT);
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

    /* a hump may also be crossed while reading a barcode or turning:
       always keep the highest one (FR5) */
    if ((VC_EV_HUMP_CLEARED == type) && (p_event->p1 > p_sm->st.max_hump_um))
    {
        p_sm->st.max_hump_um = p_event->p1;
    }

    if (!handle_common(p_sm, p_event, now_ms, p_out))
    {
        switch (p_sm->st.state)
        {
            case VC_ST_IDLE:
                if (VC_EV_CMD_START == type)
                {
                    enter(p_sm, VC_ST_MISSION_START, now_ms, p_out);
                }
                break;
            case VC_ST_MISSION_START:
                on_mission_start(p_sm, p_event, type, in_state, now_ms, p_out);
                break;
            case VC_ST_FOLLOW_LINE:
                on_follow_line(p_sm, p_event, type, in_state, now_ms, p_out);
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
            case VC_ST_MISSION_COMPLETE:
                /* stay stopped until acknowledged */
                if (VC_EV_CMD_RESET == type)
                {
                    enter(p_sm, VC_ST_IDLE, now_ms, p_out);
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
