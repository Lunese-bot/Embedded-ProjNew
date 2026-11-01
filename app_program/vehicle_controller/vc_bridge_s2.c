/*
 *      vc_bridge_s2.c
 *      Subsystem 6 -- adapter to subsystem 2 (motion control).
 *      S2's interface: ../subsystem2/subsystem2.h (IF-M1, IF-M2, IF-M3).
 *
 *      controller -> S2 (g_mbf_motion_cmd through S2's s2_cmd_xxx helpers)
 *        STOP          s2_cmd_stop()  (falls back to s2_emergency_stop()
 *                      if the queue is full: a stop is never dropped)
 *        FORWARD v     s2_cmd_drive(v, 0)
 *        STEER s       s2_cmd_drive(speed, steer): S3's -1000..+1000
 *                      (+ = right) becomes S2's wheel-speed difference
 *                      (+ = left), full scale = the current speed
 *        SET_SPEED v   remembered; re-sent at once while driving
 *        TURN_LEFT a   s2_cmd_turn(turn speed, +a)   (+ = left in S2)
 *        TURN_RIGHT a  s2_cmd_turn(turn speed, -a)
 *        U_TURN        s2_cmd_turn(turn speed, 180)
 *        DRIVE_DIST d  s2_cmd_drive_dist(speed, +d)
 *        REVERSE d     s2_cmd_drive_dist(speed, -d)
 *        SAFE_STOP     s2_emergency_stop()  (latched in S2)
 *        RESET         s2_cmd_reset()
 *      S2 stops by itself if DRIVE is not refreshed within S2_CMD_TIMEOUT_MS
 *      (500 ms).  In FOLLOW_LINE the steering from S3 refreshes it every
 *      10 ms; if that stops, S2 stops and reports it.
 *
 *      S2 -> controller (g_flg_motion, polled with TWF_BITCLR, never blocks)
 *        MFLG_CMD_DONE     -> VC_EV_MANOEUVRE_DONE (p1 = 1)
 *        MFLG_FAULT        -> CRITICAL fault 0x20nn (nn = motion_fault_t)
 *        MFLG_CMD_TIMEOUT  -> CRITICAL fault 0x2010 (steering stopped)
 *        SELF_CHECK        -> VC_EV_READY if S2 is not latched in a fault
 */

#include "vc_bridge.h"
#include "../subsystem2/subsystem2.h"

#ifdef S2_PRI_MOTION /* S2's real interface is present */

#define VC_TURN_SPEED_MMS   120 /* wheel speed for pivot turns */
#define S2_FC_TIMEOUT       0x2010
#define S2_FC_FAULT_BASE    0x2000U /* | motion_fault_t in the low byte */
#define S2_FC_FAULT_MASK    0xFFU
#define VC_STEER_FULL_SCALE 1000 /* S3 steering request -1000..+1000 */
#define VC_U_TURN_DEG       180

#define S2_EVENT_BITS (MFLG_CMD_DONE | MFLG_FAULT | MFLG_CMD_TIMEOUT)

static int32_t  g_speed     = 0;     /* current forward speed, mm/s */
static int32_t  g_steer     = 0;     /* last steering, S2 units */
static bool     g_b_driving = false; /* last command was a DRIVE */
static uint32_t g_got;               /* flag bits not yet handed out */
static bool     g_b_ready_pending;

ER vc_bridge_s2_move(vc_move_t move, int32_t value)
{
    ER ercd;

    switch (move)
    {
        case VC_MOVE_STOP:
            g_b_driving = false;
            ercd        = s2_cmd_stop();
            if (ercd < E_OK)
            {
                ercd = s2_emergency_stop(); /* never drop a stop */
            }
            break;
        case VC_MOVE_FORWARD:
            g_speed     = value;
            g_steer     = 0;
            g_b_driving = true;
            ercd        = s2_cmd_drive(g_speed, 0);
            break;
        case VC_MOVE_STEER:
            g_steer     = -((value * g_speed) / VC_STEER_FULL_SCALE);
            g_b_driving = true;
            ercd        = s2_cmd_drive(g_speed, g_steer);
            break;
        case VC_MOVE_SET_SPEED:
            g_speed = value;
            ercd    = g_b_driving ? s2_cmd_drive(g_speed, g_steer) : E_OK;
            break;
        case VC_MOVE_TURN_LEFT:
            g_b_driving = false;
            ercd        = s2_cmd_turn(VC_TURN_SPEED_MMS, value);
            break;
        case VC_MOVE_TURN_RIGHT:
            g_b_driving = false;
            ercd        = s2_cmd_turn(VC_TURN_SPEED_MMS, -value);
            break;
        case VC_MOVE_U_TURN:
            g_b_driving = false;
            ercd        = s2_cmd_turn(VC_TURN_SPEED_MMS, VC_U_TURN_DEG);
            break;
        case VC_MOVE_DRIVE_DIST:
            g_b_driving = false;
            ercd        = s2_cmd_drive_dist(g_speed, value);
            break;
        case VC_MOVE_REVERSE:
            g_b_driving = false;
            ercd        = s2_cmd_drive_dist(g_speed, -value);
            break;
        case VC_MOVE_SAFE_STOP:
            g_b_driving = false;
            ercd        = s2_emergency_stop();
            break;
        case VC_MOVE_RESET:
            g_b_driving = false;
            ercd        = s2_cmd_reset();
            break;
        default:
            ercd = E_PAR;
            break;
    }
    return ercd;
}

static void make(vc_event_t * p_event, vc_event_type_t type, int32_t param1,
                 int32_t param2)
{
    p_event->src     = VC_SRC_MOTION;
    p_event->type    = (uint8_t)type; /* Cast: enum fits 8 bits */
    p_event->rsv     = 0;
    p_event->p1      = param1;
    p_event->p2      = param2;
    p_event->time_ms = 0;
}

bool vc_bridge_s2_next(vc_event_t * p_event)
{
    UINT            ptn;
    motion_status_t motion_st;
    bool            b_event = true;

    if ((!g_b_ready_pending) && (0U == g_got) &&
        (E_OK == tk_wai_flg(g_flg_motion, S2_EVENT_BITS, TWF_ORW | TWF_BITCLR,
                            &ptn, TMO_POL)))
    {
        g_got = ptn & S2_EVENT_BITS;
    }

    if (g_b_ready_pending)
    {
        g_b_ready_pending = false;
        make(p_event, VC_EV_READY, 0, 0);
    }
    else if (0U != (g_got & MFLG_FAULT))
    {
        g_got &= ~MFLG_FAULT;
        motion_st.fault = 0;
        s2_get_status(&motion_st);
        /* Cast: mask the fault code as unsigned; the result is < 0x2100 */
        make(p_event, VC_EV_FAULT, VC_FAULT_CRITICAL,
             (int32_t)(S2_FC_FAULT_BASE |
                       ((uint32_t)motion_st.fault & S2_FC_FAULT_MASK)));
    }
    else if (0U != (g_got & MFLG_CMD_TIMEOUT))
    {
        g_got &= ~MFLG_CMD_TIMEOUT;
        make(p_event, VC_EV_FAULT, VC_FAULT_CRITICAL, S2_FC_TIMEOUT);
    }
    else if (0U != (g_got & MFLG_CMD_DONE))
    {
        g_got &= ~MFLG_CMD_DONE;
        make(p_event, VC_EV_MANOEUVRE_DONE, 1, 0);
    }
    else
    {
        b_event = false;
    }
    return b_event;
}

ER vc_bridge_s2_request(vc_request_t req, int32_t arg)
{
    motion_status_t motion_st;

    (void)arg;
    if ((VC_REQ_SELF_CHECK == req) && (E_OK == s2_get_status(&motion_st)) &&
        (MSTATE_FAULT != motion_st.state))
    {
        g_b_ready_pending = true;
    }
    return E_OK;
}

#else /* only the S2 example: log the command, S2 not bound yet */

ER vc_bridge_s2_move(vc_move_t move, int32_t value)
{
    (void)move;
    (void)value;
    return E_OK;
}

bool vc_bridge_s2_next(vc_event_t * p_event)
{
    (void)p_event;
    return false;
}

ER vc_bridge_s2_request(vc_request_t req, int32_t arg)
{
    (void)req;
    (void)arg;
    return E_OK;
}

#endif /* S2_PRI_MOTION */

/*** end of file ***/
