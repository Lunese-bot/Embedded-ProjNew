/*
 *      vc_bridge_s4.c
 *      Subsystem 6 -- adapter to subsystem 4 (IMU terrain monitoring).
 *      S4's interface: ../subsystem4/subsystem4.h (IF-S4-1, IF-S4-4).
 *
 *      S4 -> controller (g_mbf_imu_evt, imu_event_msg_t)
 *        CAL_DONE      -> VC_EV_READY            (self check passed)
 *        CAL_FAILED    -> non-critical fault 0x4004 (S4 retries on its own)
 *        HUMP_START    -> VC_EV_HUMP_START
 *        HUMP_END      -> VC_EV_HUMP_CLEARED, p1 = peak height (um)
 *        HUMP_ABORT    -> VC_EV_HUMP_CLEARED, p1 = 0
 *        IMPACT        -> VC_EV_IMPACT           (critical: safe stop)
 *        TILT_LIMIT    -> fault 0x4003 at S4's own severity
 *        FAULT         -> recoverable fault 0x4001 (S4 retrying, bounded)
 *        FAULT_FATAL   -> NON-CRITICAL fault 0x4002: the IMU is not needed to
 *                         follow the line, so the run continues without
 *                         hump data or impact detection.  TEAM DECISION:
 *                         change to VC_FAULT_CRITICAL for a safe stop.
 *        HUMP_PEAK, MOTION, RECOVERED -> not needed by the controller
 *
 *      controller -> S4
 *        SELF_CHECK -> s4_request_peak_reset() + s4_request_calibration()
 *                      (the robot is standing still in MISSION_START)
 */

#include "vc_bridge.h"
#include "../subsystem4/subsystem4.h"

#ifdef S4_PRI_IMU /* S4's real interface is present */

#define S4_FC_FAULT      0x4001
#define S4_FC_FATAL      0x4002
#define S4_FC_TILT       0x4003
#define S4_FC_CAL_FAILED 0x4004

static int32_t severity(uint8_t imu_sev)
{
    int32_t sev;

    switch (imu_sev)
    {
        case IMU_SEV_CRITICAL:
            sev = VC_FAULT_CRITICAL;
            break;
        case IMU_SEV_RECOVERABLE:
            sev = VC_FAULT_RECOVERABLE;
            break;
        default:
            sev = VC_FAULT_NON_CRITICAL;
            break;
    }
    return sev;
}

static void make(vc_event_t * p_event, vc_event_type_t type, int32_t param1,
                 int32_t param2, uint32_t time_ms)
{
    p_event->src     = VC_SRC_IMU;
    p_event->type    = (uint8_t)type; /* Cast: enum fits 8 bits */
    p_event->rsv     = 0;
    p_event->p1      = param1;
    p_event->p2      = param2;
    p_event->time_ms = time_ms;
}

/* One IMU event -> controller event.  Returns false for events the
   controller does not need. */
static bool map_imu_event(const imu_event_msg_t * p_msg, vc_event_t * p_event)
{
    bool b_event = true;

    switch (p_msg->type)
    {
        case IMU_EVT_CAL_DONE:
            make(p_event, VC_EV_READY, 0, 0, p_msg->time_ms);
            break;
        case IMU_EVT_CAL_FAILED:
            make(p_event, VC_EV_FAULT, VC_FAULT_NON_CRITICAL, S4_FC_CAL_FAILED,
                 p_msg->time_ms);
            break;
        case IMU_EVT_HUMP_START:
            make(p_event, VC_EV_HUMP_START, 0, 0, p_msg->time_ms);
            break;
        case IMU_EVT_HUMP_END:
            make(p_event, VC_EV_HUMP_CLEARED, p_msg->value, 0, p_msg->time_ms);
            break;
        case IMU_EVT_HUMP_ABORT:
            make(p_event, VC_EV_HUMP_CLEARED, 0, 0, p_msg->time_ms);
            break;
        case IMU_EVT_IMPACT:
            make(p_event, VC_EV_IMPACT, p_msg->value, 0, p_msg->time_ms);
            break;
        case IMU_EVT_TILT_LIMIT:
            make(p_event, VC_EV_FAULT, severity(p_msg->severity), S4_FC_TILT,
                 p_msg->time_ms);
            break;
        case IMU_EVT_FAULT:
            make(p_event, VC_EV_FAULT, VC_FAULT_RECOVERABLE, S4_FC_FAULT,
                 p_msg->time_ms);
            break;
        case IMU_EVT_FAULT_FATAL:
            make(p_event, VC_EV_FAULT, VC_FAULT_NON_CRITICAL, S4_FC_FATAL,
                 p_msg->time_ms);
            break;
        default:
            b_event = false; /* HUMP_PEAK, MOTION, RECOVERED: skip */
            break;
    }
    return b_event;
}

bool vc_bridge_s4_next(vc_event_t * p_event)
{
    imu_event_msg_t imu_msg;
    int32_t         len;
    bool            b_event = false;
    bool            b_empty = false;

    while ((!b_event) && (!b_empty))
    {
        len = tk_rcv_mbf(g_mbf_imu_evt, &imu_msg, TMO_POL);
        /* Cast: sizeof of a small struct vs the signed receive size. */
        if (len < (int32_t)sizeof(imu_msg))
        {
            b_empty = true; /* empty (E_TMOUT) or bad size */
        }
        else
        {
            b_event = map_imu_event(&imu_msg, p_event);
        }
    }
    return b_event;
}

ER vc_bridge_s4_request(vc_request_t req, int32_t arg)
{
    ER ercd = E_OK; /* S4 measures continuously: nothing else to ask */

    (void)arg;
    if (VC_REQ_SELF_CHECK == req)
    {
        app_check(s4_request_peak_reset(), "s4_request_peak_reset");
        ercd = s4_request_calibration(); /* answers CAL_DONE */
    }
    return ercd;
}

#else /* only the S4 example: S4 uses vc_post_event() directly */

bool vc_bridge_s4_next(vc_event_t * p_event)
{
    (void)p_event;
    return false;
}

ER vc_bridge_s4_request(vc_request_t req, int32_t arg)
{
    (void)req;
    (void)arg;
    return E_OK;
}

#endif /* S4_PRI_IMU */

/*** end of file ***/
