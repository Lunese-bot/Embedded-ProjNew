/*
 *      vc_bridge_s5.c
 *      Subsystem 6 -- adapter to subsystem 5's g_mbf_s5_cmd / g_mbf_s5_evt.
 *
 *      S5 -> controller
 *        OBSTACLE_DETECTED  -> VC_EV_OBSTACLE (p1 = distance mm)
 *        MOTION_REQ         -> VC_EV_MOVE_REQUEST(s); S5 waits for
 *                              MOTION_DONE with the same seq:
 *                                STOP          answered at once
 *                                TURN +/-deg   answered on MANOEUVRE_DONE
 *                                FORWARD       SET_SPEED + DRIVE_DIST, ditto
 *                                REVERSE       SET_SPEED + REVERSE, ditto
 *                              a refused request is answered with ok = 0
 *        LINE_REACQUIRED    -> VC_EV_AVOID_DONE
 *        AVOID_FAILED       -> VC_EV_AVOID_FAILED (bounded line search),
 *                              or a CRITICAL fault if no side was clear
 *        FAULT              -> VC_EV_FAULT CRITICAL, code 0x50nn: without
 *                              the sensor the robot cannot see obstacles
 *        PROFILE_READY      -> printed only (S1 can read s5_get_status)
 *
 *      controller -> S5
 *        SELF_CHECK  -> READY back to the controller if S5 answers status
 *        OBST_MONITOR-> S5_CMD_ENABLE
 *        OBST_AVOID  -> S5_CMD_START_AVOID
 *        STOP_ALL    -> S5_CMD_ABORT, then S5_CMD_DISABLE
 *        an OBSTACLE the controller does not act on -> S5_CMD_ABORT
 */

#include "vc_bridge.h"
#include "../subsystem5/subsystem5.h"

#ifdef S5_PRI_AVOID /* S5's real interface is present */

#define PENDING_MAX      3
#define S5_FC_FAULT_BASE 0x5000 /* + s5_fault_code_t */

static vc_event_t g_pending[PENDING_MAX]; /* converted, not yet handed out */
static int32_t    g_pending_n;
static bool       g_b_wait_done; /* an S5 motion is in progress */
static uint16_t   g_wait_seq;

static void push(vc_event_type_t type, int32_t param1, int32_t param2,
                 uint32_t time_ms)
{
    vc_event_t * p_event;

    if (g_pending_n < PENDING_MAX)
    {
        p_event = &g_pending[g_pending_n];
        g_pending_n++;
        p_event->src     = VC_SRC_OBST;
        p_event->type    = (uint8_t)type; /* Cast: enum fits 8 bits */
        p_event->rsv     = 0;
        p_event->p1      = param1;
        p_event->p2      = param2;
        p_event->time_ms = time_ms;
    }
}

static void send_cmd(s5_cmd_type_t type, uint8_t ok_flag, uint16_t seq)
{
    s5_cmd_t cmd;

    cmd.type = (uint8_t)type;
    cmd.ok   = ok_flag;
    cmd.seq  = seq;
    app_check(tk_snd_mbf(g_mbf_s5_cmd, &cmd, sizeof(cmd), TMO_POL),
              "vc->s5 cmd");
}

static void convert_motion(const s5_motion_req_t * p_req, uint32_t time_ms)
{
    switch (p_req->kind)
    {
        case S5_MOVE_STOP:
            push(VC_EV_MOVE_REQUEST, VC_MOVE_STOP, 0, time_ms);
            break;
        case S5_MOVE_TURN:
            if (p_req->angle_deg >= 0)
            {
                push(VC_EV_MOVE_REQUEST, VC_MOVE_TURN_LEFT, p_req->angle_deg,
                     time_ms);
            }
            else
            {
                push(VC_EV_MOVE_REQUEST, VC_MOVE_TURN_RIGHT, -p_req->angle_deg,
                     time_ms);
            }
            break;
        case S5_MOVE_FORWARD:
            push(VC_EV_MOVE_REQUEST, VC_MOVE_SET_SPEED, p_req->speed_mm_s,
                 time_ms);
            push(VC_EV_MOVE_REQUEST, VC_MOVE_DRIVE_DIST, p_req->distance_mm,
                 time_ms);
            break;
        case S5_MOVE_REVERSE:
            push(VC_EV_MOVE_REQUEST, VC_MOVE_SET_SPEED, p_req->speed_mm_s,
                 time_ms);
            push(VC_EV_MOVE_REQUEST, VC_MOVE_REVERSE, p_req->distance_mm,
                 time_ms);
            break;
        default:
            send_cmd(S5_CMD_MOTION_DONE, 0, p_req->seq); /* unknown: refuse */
            break;
    }
    if ((S5_MOVE_STOP == p_req->kind) || (S5_MOVE_TURN == p_req->kind) ||
        (S5_MOVE_FORWARD == p_req->kind) || (S5_MOVE_REVERSE == p_req->kind))
    {
        g_b_wait_done = true;
        g_wait_seq    = p_req->seq;
    }
}

/* One S5 event -> zero, one or two pending controller events */
static void convert_s5_event(const s5_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case S5_EVT_OBSTACLE_DETECTED:
            push(VC_EV_OBSTACLE, p_evt->distance_mm, 0, p_evt->time_ms);
            break;
        case S5_EVT_MOTION_REQ:
            convert_motion(&p_evt->u.motion, p_evt->time_ms);
            break;
        case S5_EVT_LINE_REACQUIRED:
            push(VC_EV_AVOID_DONE, 0, 0, p_evt->time_ms);
            break;
        case S5_EVT_AVOID_FAILED:
            if (S5_FAULT_BLOCKED == p_evt->code)
            {
                push(VC_EV_FAULT, VC_FAULT_CRITICAL,
                     S5_FC_FAULT_BASE + (int32_t)p_evt->code, p_evt->time_ms);
            }
            else
            {
                push(VC_EV_AVOID_FAILED, p_evt->code, 0, p_evt->time_ms);
            }
            break;
        case S5_EVT_FAULT:
            push(VC_EV_FAULT, VC_FAULT_CRITICAL,
                 S5_FC_FAULT_BASE + (int32_t)p_evt->code, p_evt->time_ms);
            break;
        case S5_EVT_PROFILE_READY:
            tm_printf((uint8_t *)"vc: s5 profile d=%d w=%d side=%d\n",
                      p_evt->u.profile.distance_mm, p_evt->u.profile.width_mm,
                      p_evt->u.profile.bypass_side);
            break;
        default:
            break;
    }
}

bool vc_bridge_s5_next(vc_event_t * p_event)
{
    s5_evt_t s5_evt;
    int32_t  len;
    int32_t  idx;
    bool     b_empty = false;
    bool     b_event = false;

    while ((0 == g_pending_n) && (!b_empty))
    {
        len = tk_rcv_mbf(g_mbf_s5_evt, &s5_evt, TMO_POL);
        /* Cast: sizeof of a small struct vs the signed receive size. */
        if (len < (int32_t)sizeof(s5_evt))
        {
            b_empty = true; /* empty (E_TMOUT) or bad size */
        }
        else
        {
            convert_s5_event(&s5_evt);
        }
    }

    if (g_pending_n > 0)
    {
        *p_event = g_pending[0];
        for (idx = 1; idx < g_pending_n; idx++)
        {
            g_pending[idx - 1] = g_pending[idx];
        }
        g_pending_n--;
        b_event = true;
    }
    return b_event;
}

void vc_bridge_s5_after(const vc_event_t * p_event, vc_state_t state_after,
                        bool b_rejected)
{
    if ((VC_EV_OBSTACLE == p_event->type) &&
        (VC_ST_AVOID_OBSTACLE != state_after))
    {
        send_cmd(S5_CMD_ABORT, 0, 0); /* we will not act on it */
    }
    else if ((VC_EV_MOVE_REQUEST != p_event->type) || (!g_b_wait_done))
    {
        /* not a motion request we are waiting on */
    }
    else if (b_rejected)
    {
        g_pending_n   = 0; /* drop the rest of this request */
        g_b_wait_done = false;
        send_cmd(S5_CMD_MOTION_DONE, 0, g_wait_seq);
    }
    else if ((int32_t)VC_MOVE_STOP == p_event->p1)
    {
        g_b_wait_done = false; /* a stop is done at once */
        send_cmd(S5_CMD_MOTION_DONE, 1, g_wait_seq);
    }
    else
    {
        /* MOTION_DONE is sent when S2 reports the manoeuvre */
    }
}

void vc_bridge_s5_manoeuvre_done(bool b_ok)
{
    if (g_b_wait_done)
    {
        g_b_wait_done = false;
        send_cmd(S5_CMD_MOTION_DONE, b_ok ? 1 : 0, g_wait_seq);
    }
}

ER vc_bridge_s5_request(vc_request_t req, int32_t arg)
{
    s5_status_t s5_st;

    (void)arg;
    switch (req)
    {
        case VC_REQ_SELF_CHECK:
            if ((E_OK == s5_get_status(&s5_st)) && (S5_ST_ERROR != s5_st.state))
            {
                push(VC_EV_READY, 0, 0, 0);
            }
            break;
        case VC_REQ_OBST_MONITOR:
            send_cmd(S5_CMD_ENABLE, 0, 0);
            break;
        case VC_REQ_OBST_AVOID:
            g_b_wait_done = false;
            send_cmd(S5_CMD_START_AVOID, 0, 0);
            break;
        case VC_REQ_STOP_ALL:
            g_pending_n   = 0;
            g_b_wait_done = false;
            send_cmd(S5_CMD_ABORT, 0, 0);
            send_cmd(S5_CMD_DISABLE, 0, 0);
            break;
        default:
            break;
    }
    return E_OK;
}

#else /* only the S5 template: S5 uses vc_post_event() directly */

bool vc_bridge_s5_next(vc_event_t * p_event)
{
    (void)p_event;
    return false;
}

void vc_bridge_s5_after(const vc_event_t * p_event, vc_state_t state_after,
                        bool b_rejected)
{
    (void)p_event;
    (void)state_after;
    (void)b_rejected;
}

void vc_bridge_s5_manoeuvre_done(bool b_ok)
{
    (void)b_ok;
}

ER vc_bridge_s5_request(vc_request_t req, int32_t arg)
{
    (void)req;
    (void)arg;
    return E_OK;
}

#endif /* S5_PRI_AVOID */

/*** end of file ***/
