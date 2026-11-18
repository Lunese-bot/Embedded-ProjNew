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
 *          BARCODE_DECODED  -> VC_EV_BARCODE_DECODED, p1 = legacy nav,
 *                              barcode_text = complete decoded A-Z string,
 *                              p2 = 1 if read backwards BARCODE_FAILED   -> VC_EV_BARCODE_FAILED, p1 = reason, p2 = S3
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
#include "../debugging/live_tune.h"
#include "vc_sm.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem2/subsystem2.h" /* g_s2_odo_ticks */

#ifdef S3_PRI_LINE /* S3's real interface is present */

#define S3_FC_CAL_FAILED 0x3002

static uint32_t g_last_seq;
static uint32_t g_exec_seq; /* last S3 sample reported during a junction turn */
static bool     g_b_ready_pending;

#if S2_WEBDEBUG
/* Bench: "At every junction Left / Right / Straight" on the dashboard.  The
   junction is held back for one call and a BARCODE_DECODED carrying that
   action goes first, so the controller turns exactly as if a barcode had
   been read (same code path, no change to vc_sm.c). */
static bool       g_b_junction_held;
static vc_event_t g_junction_held;
#endif

/* Bench switch from the dashboard: 0 = the mission ignores barcodes (no
   start, decode or fail events, and S3 is kept from scanning), so the line
   follower can be tested on its own. */
#if S2_WEBDEBUG
#define BC_OFF (0 == g_lt_bc_en)
#define LOST_OFF (0 == g_lt_lost_en)
#else
#define BC_OFF 0
#define LOST_OFF 0
#endif

static void make(vc_event_t * p_event, vc_event_type_t type, int32_t param1,
                 int32_t param2, uint32_t time_ms)
{
    p_event->src     = VC_SRC_LINE;
    p_event->type    = (uint8_t)type; /* Cast: enum fits 8 bits */
    p_event->rsv     = 0;
    p_event->p1      = param1;
    p_event->p2      = param2;
    p_event->time_ms = time_ms;
    p_event->barcode_text[0] = '\0';
}

/* One S3 event -> controller event.  Returns false for events the
   controller does not need (CAL_DONE). */
static bool map_s3_event(const s3_event_t * p_s3, vc_event_t * p_event)
{
    bool b_event = true;

#if S2_WEBDEBUG
    /* dashboard: remember the last barcode result, even with barcodes
       switched off for the mission, so reading can be tested by hand */
    if (S3_EVT_BARCODE_DECODED == p_s3->type)
    {
        uint32_t idx;
        for (idx = 0u; (idx < (sizeof(g_lt_bc_text) - 1u)) &&
                       (idx < (sizeof(p_s3->text) - 1u)); idx++)
        {
            char ch = (char)p_s3->text[idx];
            if ((ch < 'A') || (ch > 'Z'))
            {
                break; /* A-Z only (also keeps the JSON safe) */
            }
            g_lt_bc_text[idx] = ch;
        }
        g_lt_bc_text[idx] = '\0';
    }
    else if (S3_EVT_BARCODE_FAILED == p_s3->type)
    {
        g_lt_bc_text[0] = '?'; /* last read failed */
        g_lt_bc_text[1] = '\0';
    }
    else
    {
        /* not a barcode result */
    }
    if ((S3_EVT_BARCODE_DECODED == p_s3->type) ||
        (S3_EVT_BARCODE_FAILED == p_s3->type))
    {
        /* history, newest first: "<text> " + old, cut to fit */
        char     tmp[sizeof(g_lt_bc_hist)];
        uint32_t n = 0u;
        uint32_t k;
        for (k = 0u; ('\0' != g_lt_bc_text[k]) && (n < (sizeof(tmp) - 2u)); k++)
        {
            tmp[n++] = g_lt_bc_text[k];
        }
        tmp[n++] = ' ';
        for (k = 0u; ('\0' != g_lt_bc_hist[k]) && (n < (sizeof(tmp) - 1u)); k++)
        {
            tmp[n++] = g_lt_bc_hist[k];
        }
        tmp[n] = '\0';
        for (k = 0u; k <= n; k++)
        {
            g_lt_bc_hist[k] = tmp[k];
        }
    }
#endif
    if (BC_OFF && ((S3_EVT_BARCODE_START == p_s3->type) ||
                   (S3_EVT_BARCODE_DECODED == p_s3->type) ||
                   (S3_EVT_BARCODE_FAILED == p_s3->type)))
    {
        return false; /* barcodes switched off from the dashboard */
    }
    if (LOST_OFF && (S3_EVT_LINE_LOST == p_s3->type))
    {
        return false; /* lost-line handling switched off from the dashboard */
    }
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
            /* Carry the complete text through the Vehicle Controller; action rules match the
               whole barcode rather than only S3's legacy first-letter nav. */
            {
                uint32_t idx;
                for (idx = 0u;
                     (idx < (sizeof(p_event->barcode_text) - 1u)) &&
                     (idx < (sizeof(p_s3->text) - 1u)) &&
                     ('\0' != (char)p_s3->text[idx]); idx++)
                {
                    p_event->barcode_text[idx] = (char)p_s3->text[idx];
                }
                p_event->barcode_text[idx] = '\0';
            }
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
#if S2_WEBDEBUG
    else if (g_b_junction_held)
    {
        g_b_junction_held = false;
        *p_event          = g_junction_held; /* after the forced action */
        b_event           = true;
    }
#endif
    /* 1. events first: a LINE_LOST must beat the next steering value */
    else if (next_s3_event(p_event))
    {
        b_event = true;
#if S2_WEBDEBUG
        if ((VC_EV_JUNCTION == p_event->type) &&
            (g_lt_next_nav > (int32_t)NAV_NONE) &&
            (g_lt_next_nav <= (int32_t)NAV_STRAIGHT))
        {
            /* send the dashboard's action first, the junction next call */
            g_junction_held   = *p_event;
            g_b_junction_held = true;
            make(p_event, VC_EV_BARCODE_DECODED, g_lt_next_nav, 0,
                 g_junction_held.time_ms);
        }
#endif
    }
    /* 1b. Controller is searching (LOST_LINE): with the straddle layout S3
       has no LOST state, so it never sends LINE_FOUND and the search could
       only end in a fault.  C or R on black = found (review 2026-11-10). */
    else if ((VC_ST_LOST_LINE == state) &&
             (E_OK == s3_get_line_status(&line_st)) &&
             (0u != (line_st.black_mask & (S3_MASK_C | S3_MASK_R))))
    {
        make(p_event, VC_EV_LINE_FOUND, 0, 0, line_st.time_ms);
        b_event = true;
    }
    /* 1c. Junction turn (EXECUTE_ACTION, VC_TURN_END_ON_LINE): C or R on
       black, once per new S3 sample, with S2's odometer in p2 so the
       controller can tell how far it has turned. */
    else if ((VC_ST_EXECUTE_ACTION == state) &&
             (E_OK == s3_get_line_status(&line_st)) &&
             (line_st.seq != g_exec_seq) &&
             (0u != (line_st.black_mask & (S3_MASK_C | S3_MASK_R))))
    {
        g_exec_seq = line_st.seq;
        /* Cast: the odometer is sent as int32 and read back as uint32 */
        make(p_event, VC_EV_LINE_FOUND, 0, (int32_t)g_s2_odo_ticks,
             line_st.time_ms);
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
            /* No calibration needed: analogue sensors use the white /
               black readings in s3_config.h until one is run. */
            if (E_OK == s3_get_line_status(&line_st))
            {
                g_b_ready_pending = true;
            }
            break;
        case VC_REQ_LINE_FOLLOW:
            ercd = BC_OFF ? s3_request(S3_REQ_BC_DISABLE)
                          : s3_request(S3_REQ_BC_ENABLE);
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
