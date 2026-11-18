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
 *        MFLG_STOPPED      -> VC_EV_MOTION_STOPPED after a requested STOP
 *        MFLG_FAULT        -> CRITICAL fault 0x20nn (nn = motion_fault_t)
 *        MFLG_CMD_TIMEOUT  -> CRITICAL fault 0x2010 (steering stopped)
 *        SELF_CHECK        -> VC_EV_READY if S2 is not latched in a fault
 *      VC_MOVE_STOP_CONFIRM clears stale MFLG_STOPPED before issuing STOP,
 *      so the junction state advances only on its own stop confirmation.
 */

#include "vc_bridge.h"
#include "vc_sm.h" /* VC_CURVE_SLOW_PERMILLE */
#include "../subsystem2/subsystem2.h"
#include "../debugging/live_tune.h"

#ifdef S2_PRI_MOTION /* S2's real interface is present */

#define VC_TURN_SPEED_MMS   120 /* wheel speed for pivot turns */
#define S2_FC_TIMEOUT       0x2010
#define S2_FC_FAULT_BASE    0x2000U /* | motion_fault_t in the low byte */
#define S2_FC_FAULT_MASK    0xFFU
#define VC_STEER_FULL_SCALE 1000 /* S3 steering request -1000..+1000 */
#define VC_U_TURN_DEG       180

#define S2_EVENT_BITS (MFLG_CMD_DONE | MFLG_STOPPED | MFLG_FAULT | MFLG_CMD_TIMEOUT)

static int32_t  g_speed     = 0;     /* current forward speed, mm/s */

/* Speed actually sent to S2.  Normally g_speed; with the dashboard's live
   tuning on, scaled by g_lt_speed_pct so the mission speed can be tried
   without a rebuild. */
static int32_t sent_speed(void)
{
#if S2_WEBDEBUG
    return (g_speed * g_lt_speed_pct) / 100;
#else
    return g_speed;
#endif
}
/* Spin speed for the controller's turns (line search, barcode turns).  With
   the dashboard's live tuning on it follows the mission speed percent, so
   slowing the mission also slows the spinning; never below what a wheel
   can turn smoothly. */
#define VC_TURN_MIN_MMS 60
static int32_t sent_turn_speed(void)
{
#if S2_WEBDEBUG
    int32_t v = (VC_TURN_SPEED_MMS * g_lt_speed_pct) / 100;
    return (v < VC_TURN_MIN_MMS) ? VC_TURN_MIN_MMS : v;
#else
    return VC_TURN_SPEED_MMS;
#endif
}
static int32_t  g_steer     = 0;     /* last steering, S2 units */
static bool     g_b_driving = false; /* last command was a DRIVE */
static uint32_t g_got;               /* flag bits not yet handed out */
static bool     g_b_ready_pending;
static bool     g_b_waiting_stop;

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
        case VC_MOVE_STOP_CONFIRM:
            g_b_driving = false;
            /* Discard stale indications; only this explicit command arms
               the bridge to forward MFLG_STOPPED as a junction handshake. */
            (void)tk_clr_flg(g_flg_motion, ~MFLG_STOPPED);
            g_got &= ~MFLG_STOPPED;
            g_b_waiting_stop = true;
            ercd = s2_cmd_stop();
            if (ercd < E_OK)
            {
                ercd = s2_emergency_stop(); /* fail safe; timeout escalates */
            }
            break;
        case VC_MOVE_FORWARD:
            g_speed     = value;
            g_steer     = 0;
            g_b_driving = true;
            ercd        = s2_cmd_drive(sent_speed(), 0);
            break;
        case VC_MOVE_STEER:
#if S2_WEBDEBUG
            if (0 != g_lt_steer_inv)
            {
                value = -value; /* dashboard: swap left and right steering */
            }
#endif
            {
                /* slower in curves: the harder S3 steers, the lower the
                   forward speed (vc_sm.h VC_CURVE_SLOW_PERMILLE) */
                int32_t base = sent_speed();
                int32_t mag  = (value < 0) ? -value : value;
                int32_t fwd;
#if S2_WEBDEBUG
                int32_t slow = g_lt_curve;
#else
                int32_t slow = VC_CURVE_SLOW_PERMILLE;
#endif
                mag = (mag > VC_STEER_FULL_SCALE) ? VC_STEER_FULL_SCALE : mag;
                fwd = base - ((base * slow / 1000) * mag) / VC_STEER_FULL_SCALE;
                if ((fwd < VC_CURVE_MIN_MMS) && (base > VC_CURVE_MIN_MMS))
                {
                    fwd = VC_CURVE_MIN_MMS;
                }
                g_steer     = -((value * fwd) / VC_STEER_FULL_SCALE);
                g_b_driving = true;
                ercd        = s2_cmd_drive(fwd, g_steer);
            }
            break;
        case VC_MOVE_SET_SPEED:
            g_speed = value;
            ercd    = g_b_driving ? s2_cmd_drive(sent_speed(), g_steer) : E_OK;
            break;
        case VC_MOVE_TURN_LEFT:
            g_b_driving = false;
            ercd        = s2_cmd_turn(sent_turn_speed(), value);
            break;
        case VC_MOVE_TURN_RIGHT:
            g_b_driving = false;
            ercd        = s2_cmd_turn(sent_turn_speed(), -value);
            break;
        case VC_MOVE_U_TURN:
            g_b_driving = false;
            ercd        = s2_cmd_turn(sent_turn_speed(), VC_U_TURN_DEG);
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
    p_event->barcode_text[0] = '\0';
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
    else if (0U != (g_got & MFLG_STOPPED))
    {
        g_got &= ~MFLG_STOPPED;
        if (g_b_waiting_stop)
        {
            g_b_waiting_stop = false;
            make(p_event, VC_EV_MOTION_STOPPED, 0, 0);
        }
        else
        {
            b_event = false;
        }
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
