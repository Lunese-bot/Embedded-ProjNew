/*
 *      vc_bridge_s3.c
 *      Subsystem 6 -- adapter to subsystem 3 (IR line following + barcode).
 *      S3's interface: ../subsystem3/subsystem3.h (IF-31, IF-32, IF-33).
 *
 *      S3 -> controller
 *        g_mbf_s3_events (IF-32):
 *          LINE_LOST        -> VC_EV_LINE_LOST, p1 = last side
 *          LINE_FOUND       -> VC_EV_LINE_FOUND
 *          JUNCTION         -> VC_EV_JUNCTION
 *          BARCODE_START    -> VC_EV_BARCODE_START
 *          BARCODE_DECODED  -> VC_EV_BARCODE_DECODED, p1 = nav, p2 = 1 if read
 * backwards BARCODE_FAILED   -> VC_EV_BARCODE_FAILED, p1 = reason, p2 = S3
 * retries CAL_FAILED       -> non-critical fault 0x3002 (old calibration kept)
 *        s3_get_line_status() (IF-31): while S3 owns motion (FOLLOW_LINE,
 *          DECODE_BARCODE, TERRAIN_CHECK), each NEW sample's PD steering
 *          becomes a VC_EV_MOVE_REQUEST STEER.  S3 publishes it, it does not
 *          send it, so the controller reads it every tick (10 ms, NFR2).
 *
 *      controller -> S3 (s3_request, IF-33)
 *        SELF_CHECK     -> VC_EV_READY if S3 answers (and is calibrated in
 *                          ADC mode)
 *        LINE_FOLLOW    -> S3_REQ_BC_ENABLE
 *        BARCODE_READ n -> S3_REQ_BC_REARM for a retry (n > 0)
 *        BARCODE_OFF    -> S3_REQ_BC_DISABLE (while avoiding an obstacle)
 *        LINE_SEARCH    -> nothing: the controller turns the robot, S3 keeps
 *                          tracking and reports LINE_FOUND
 */

#include "vc_bridge.h"
#include "vc_sm.h"
#include "../subsystem3/subsystem3.h"

#ifdef S3_PRI_LINE /* S3's real interface is present */

#define S3_FC_CAL_FAILED 0x3002

static uint32_t g_last_seq;
static bool     g_b_ready_pending;

static void make(vc_event_t * p_event, vc_event_type_t type, int32_t param1,
                 int32_t param2, uint32_t time_ms)
{
    p_event->src     = VC_SRC_LINE;
    p_event->type    = (uint8_t)type; /* Cast: enum fits 8 bits */
    p_event->rsv     = 0;
    p_event->p1      = param1;
    p_event->p2      = param2;
    p_event->time_ms = time_ms;
}

/* One S3 event -> controller event.  Returns false for events the
   controller does not need (CAL_DONE). */
static bool map_s3_event(const s3_event_t * p_s3, vc_event_t * p_event)
{
    bool b_event = true;

    switch (p_s3->type)
    {
        case S3_EVT_LINE_LOST:
            make(p_event, VC_EV_LINE_LOST, p_s3->side, 0, p_s3->time_ms);
            break;
        case S3_EVT_LINE_FOUND:
            make(p_event, VC_EV_LINE_FOUND, 0, 0, p_s3->time_ms);
            break;
        case S3_EVT_JUNCTION:
            make(p_event, VC_EV_JUNCTION, 0, 0, p_s3->time_ms);
            break;
        case S3_EVT_BARCODE_START:
            make(p_event, VC_EV_BARCODE_START, 0, 0, p_s3->time_ms);
            break;
        case S3_EVT_BARCODE_DECODED:
            /* Cast: the console API takes uint8_t format strings. */
            tm_printf((uint8_t *)"vc: barcode \"%s\" nav %d\n", p_s3->text,
                      p_s3->nav);
            /* Cast: arg is 0 or 1 (read backwards). */
            make(p_event, VC_EV_BARCODE_DECODED, p_s3->nav, (int32_t)p_s3->arg,
                 p_s3->time_ms);
            break;
        case S3_EVT_BARCODE_FAILED:
            /* Cast: arg is a small retry count. */
            make(p_event, VC_EV_BARCODE_FAILED, p_s3->code, (int32_t)p_s3->arg,
                 p_s3->time_ms);
            break;
        case S3_EVT_CAL_FAILED:
            make(p_event, VC_EV_FAULT, VC_FAULT_NON_CRITICAL, S3_FC_CAL_FAILED,
                 p_s3->time_ms);
            break;
        default:
            b_event = false; /* CAL_DONE: nothing for the controller */
            break;
    }
    return b_event;
}

/* Read S3 events until one matters to the controller or none are left */
static bool next_s3_event(vc_event_t * p_event)
{
    s3_event_t s3_evt;
    int32_t    len;
    bool       b_event = false;
    bool       b_empty = false;

    while ((!b_event) && (!b_empty))
    {
        len = tk_rcv_mbf(g_mbf_s3_events, &s3_evt, TMO_POL);
        /* Cast: sizeof of a small struct vs the signed receive size. */
        if ((int32_t)sizeof(s3_evt) == len)
        {
            b_event = map_s3_event(&s3_evt, p_event);
        }
        else
        {
            b_empty = true;
        }
    }
    return b_event;
}

bool vc_bridge_s3_next(vc_event_t * p_event, vc_state_t state)
{
    s3_line_status_t line_st;
    bool             b_event = false;

    if (g_b_ready_pending)
    {
        g_b_ready_pending = false;
        make(p_event, VC_EV_READY, 0, 0, 0);
        b_event = true;
    }
    /* 1. events first: a LINE_LOST must beat the next steering value */
    else if (next_s3_event(p_event))
    {
        b_event = true;
    }
    /* 2. steering, once per new S3 sample, only while S3 owns motion */
    else if ((VC_SRC_LINE == vc_sm_motion_owner(state)) &&
             (E_OK == s3_get_line_status(&line_st)) &&
             (line_st.seq != g_last_seq) &&
             (LINE_STATE_FOLLOW == line_st.line_state))
    {
        g_last_seq = line_st.seq;
        make(p_event, VC_EV_MOVE_REQUEST, VC_MOVE_STEER, line_st.steering,
             line_st.time_ms);
        b_event = true;
    }
    else
    {
        /* nothing new */
    }
    return b_event;
}

ER vc_bridge_s3_request(vc_request_t req, int32_t arg)
{
    s3_line_status_t line_st;

    ER ercd = E_OK;

    switch (req)
    {
        case VC_REQ_SELF_CHECK:
            if ((E_OK == s3_get_line_status(&line_st)) &&
                ((line_st.calibrated) || (0 == S3_IR_USE_ADC)))
            {
                g_b_ready_pending = true;
            }
            break;
        case VC_REQ_LINE_FOLLOW:
            ercd = s3_request(S3_REQ_BC_ENABLE);
            break;
        case VC_REQ_BARCODE_READ:
            if (arg > 0)
            {
                ercd = s3_request(S3_REQ_BC_REARM);
            }
            break;
        case VC_REQ_BARCODE_OFF:
            ercd = s3_request(S3_REQ_BC_DISABLE);
            break;
        default:
            break;
    }
    return ercd;
}

#else /* only the S3 example: S3 uses vc_post_event() directly */

bool vc_bridge_s3_next(vc_event_t * p_event, vc_state_t state)
{
    (void)p_event;
    (void)state;
    return false;
}

ER vc_bridge_s3_request(vc_request_t req, int32_t arg)
{
    (void)req;
    (void)arg;
    return E_OK;
}

#endif /* S3_PRI_LINE */

/*** end of file ***/
