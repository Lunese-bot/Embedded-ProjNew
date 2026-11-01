/*
 *      task_barcode.c
 *      Subsystem 3 -- the Barcode Detection task (Design Review section 3).
 *
 *      line_task turns the barcode sensor into runs ("black for 41 ms",
 *      "white for 18 ms", ...) and sends them through g_mbf_s3_bc_runs.
 *      This task collects the runs of one barcode and decodes them.
 *
 *      barcode_state_t (robot_types.h), Design Review 6.2:
 *
 *        IDLE   --bar after a long white gap--> DETECT (send BARCODE_START)
 *        DETECT --> READ    collect bar/space widths
 *        READ   --white gap > quiet zone, or no change for that long--> DECODE
 *        READ   --black run too long (a crossing line, not a bar)--> IDLE
 *        DECODE --ok--> COMPLETE (send BARCODE_DECODED + nav) --> IDLE
 *        DECODE --error--> RETRY (send BARCODE_FAILED + retries) --> IDLE
 *
 *      Widths are in microseconds, so the robot should hold a steady speed
 *      over the barcode (the controller slows down on BARCODE_START).
 *      Each Code 39 character is classified on its own, which tolerates a
 *      slow speed change.
 *
 *      APIs shown: tk_rcv_mbf with a timeout, tk_wai_flg (TMO_POL)
 */

#include "subsystem3.h"
#include "code39.h"

#define IDLE_POLL_MS 100   /* how often requests are checked when idle */
#define S3_US_PER_MS 1000U /* run widths are in microseconds */

static uint32_t        g_widths[S3_BC_MAX_ELEMENTS];
static uint32_t        g_n_widths;
static uint32_t        g_widest_us;
static uint32_t        g_retries;
static bool            g_b_enabled = true;
static barcode_state_t g_state     = BARCODE_STATE_IDLE;

static void set_state(barcode_state_t next)
{
    g_state = next;
    tk_loc_mtx(g_mtx_s3_status, TMO_FEVR);
    g_s3_status.bc_state   = next;
    g_s3_status.bc_enabled = g_b_enabled;
    tk_unl_mtx(g_mtx_s3_status);
}

static void go_idle(void)
{
    g_n_widths         = 0;
    g_widest_us        = 0;
    g_b_s3_bc_scanning = false;
    set_state(BARCODE_STATE_IDLE);
}

static void handle_requests(void)
{
    UINT ptn = 0;

    if (E_OK != tk_wai_flg(g_flg_s3_req, S3_FLG_BC_REQ, TWF_ORW | TWF_BITCLR,
                           &ptn, TMO_POL))
    {
        ptn = 0U; /* no request pending */
    }
    if (0U != (ptn & S3_REQ_BC_DISABLE))
    {
        g_b_enabled = false;
        go_idle();
    }
    if (0U != (ptn & S3_REQ_BC_ENABLE))
    {
        g_b_enabled = true;
        set_state(g_state);
    }
    if (0U != (ptn & S3_REQ_BC_REARM))
    {
        g_retries = 0;
        go_idle();
    }
}

/* Quiet zone: S3_BC_QUIET_FACTOR x widest element, at least the minimum */
static uint32_t quiet_us(void)
{
    uint32_t quiet     = g_widest_us * S3_BC_QUIET_FACTOR;
    uint32_t quiet_min = (uint32_t)S3_BC_QUIET_MIN_MS * S3_US_PER_MS;

    /* Cast above: the config value is a small positive int. */
    return (quiet < quiet_min) ? quiet_min : quiet;
}

static TMO wait_timeout(void)
{
    TMO tmo;

    /* Last element stored was a bar -> the sensor is on white now:
       end of barcode after a quiet zone.  Otherwise it is on black:
       give up after the longest allowed bar. */
    if (BARCODE_STATE_READ != g_state)
    {
        tmo = IDLE_POLL_MS;
    }
    else if (1U == (g_n_widths % 2U))
    {
        /* Cast: the quiet zone is a few hundred ms, well inside TMO. */
        tmo = (TMO)(quiet_us() / S3_US_PER_MS) + 1;
    }
    else
    {
        tmo = S3_BC_ELEMENT_MAX_MS;
    }
    return tmo;
}

static void decode(void)
{
    s3_event_t      evt = { 0 };
    code39_params_t prm;
    code39_result_t res;
    bool            b_reversed = false;

    set_state(BARCODE_STATE_DECODE);
    prm.min_ratio_x10 = S3_BC_MIN_RATIO_X10;

    if (g_n_widths < S3_BC_MIN_ELEMENTS)
    {
        res = CODE39_ERR_LENGTH;
    }
    else
    {
        /* Cast: the text buffer holds plain characters. */
        res = code39_decode(g_widths, g_n_widths, &prm, (char *)evt.text,
                            sizeof(evt.text), &b_reversed);
    }

    tk_loc_mtx(g_mtx_s3_status, TMO_FEVR);
    if (CODE39_OK == res)
    {
        g_s3_status.bc_ok++;
    }
    else
    {
        g_s3_status.bc_fail++;
    }
    tk_unl_mtx(g_mtx_s3_status);

    if (CODE39_OK == res)
    {
        set_state(BARCODE_STATE_COMPLETE);
        g_retries = 0;
        evt.type  = S3_EVT_BARCODE_DECODED;
        /* Cast: navigation_command_t values fit in the 8-bit field. */
        evt.nav   = (uint8_t)s3_text_to_nav(evt.text);
        evt.arg   = b_reversed ? 1U : 0U; /* 1 = read backwards */
    }
    else
    {
        set_state(BARCODE_STATE_RETRY);
        g_retries++;
        evt.type = S3_EVT_BARCODE_FAILED;
        evt.code = (uint8_t)res; /* Cast: code39_result_t fits 8 bits */
        evt.arg  = g_retries;    /* controller gives up after
                                    S3_BC_MAX_RETRIES */
    }
    s3_send_event(&evt);
    go_idle();
}

static void on_run(const s3_bc_run_t * p_run)
{
    s3_event_t evt = { 0 };

    switch (g_state)
    {
        case BARCODE_STATE_IDLE:
            /* A white run ended, so a bar has just begun.  It starts a
               barcode only after a long white gap (the quiet zone). */
            if ((0U == p_run->black) &&
                (p_run->width_us >=
                 ((uint32_t)S3_BC_QUIET_MIN_MS * S3_US_PER_MS)))
            {
                set_state(BARCODE_STATE_DETECT);
                g_b_s3_bc_scanning = true;
                g_n_widths         = 0;
                g_widest_us        = 0;
                evt.type           = S3_EVT_BARCODE_START;
                s3_send_event(&evt);
                set_state(BARCODE_STATE_READ);
            }
            break;

        case BARCODE_STATE_READ:
            if ((0U != p_run->black) &&
                (p_run->width_us >
                 ((uint32_t)S3_BC_ELEMENT_MAX_MS * S3_US_PER_MS)))
            {
                go_idle(); /* not a barcode bar */
            }
            else if ((0U == p_run->black) && (g_n_widths > 0U) &&
                     (p_run->width_us >= quiet_us()))
            {
                decode(); /* trailing quiet zone */
            }
            else
            {
                g_widths[g_n_widths] = p_run->width_us;
                g_n_widths++;
                if (p_run->width_us > g_widest_us)
                {
                    g_widest_us = p_run->width_us;
                }
                if (g_n_widths >= S3_BC_MAX_ELEMENTS)
                {
                    decode();
                }
            }
            break;

        default:
            go_idle();
            break;
    }
}

void barcode_task(int32_t stacd, void * p_exinf)
{
    s3_bc_run_t run;
    int32_t     msg_size;

    (void)stacd;
    (void)p_exinf;

    go_idle();

    for (;;)
    {
        handle_requests();

        msg_size = tk_rcv_mbf(g_mbf_s3_bc_runs, &run, wait_timeout());

        if (E_TMOUT == msg_size)
        {
            if (BARCODE_STATE_READ == g_state)
            {
                if (1U == (g_n_widths % 2U))
                {
                    decode(); /* quiet zone reached */
                }
                else
                {
                    go_idle(); /* black too long */
                }
            }
        }
        /* Cast: sizeof of a small struct, compared with the signed
           size returned by tk_rcv_mbf. */
        else if ((int32_t)sizeof(s3_bc_run_t) != msg_size)
        {
            app_check(msg_size, "barcode tk_rcv_mbf");
        }
        else if (g_b_enabled)
        {
            on_run(&run);
        }
        else
        {
            /* Disabled: drain the run and ignore it. */
        }
    }
}

/*** end of file ***/
