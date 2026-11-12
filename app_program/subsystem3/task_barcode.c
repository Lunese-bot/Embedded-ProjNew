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
#define DIST_POLL_MS 50    /* how often the white distance is checked */
#define DIST_LIVE_MIN 10u  /* counts in this read that prove the wheels turn */
#define S3_US_PER_MS 1000U /* run widths are in microseconds */

static uint32_t        g_widths[S3_BC_MAX_ELEMENTS];     /* by time, us */
static uint32_t        g_widths_pos[S3_BC_MAX_ELEMENTS]; /* by distance */
static uint32_t        g_total_pos; /* distance covered by the stored ones */
static uint32_t        g_last_run_pos; /* wheel counts when the last run came */
static uint32_t        g_white_ms;     /* time polled since then */

/* bench copy of the last read (subsystem3.h) */
volatile uint32_t g_s3_bc_dbg_seq;
volatile uint32_t g_s3_bc_dbg_n;
volatile uint16_t g_s3_ltrace_frz[S3_LTRACE_N];
volatile uint8_t  g_s3_ltrace_frz_fwd[S3_LTRACE_N];
volatile int8_t   g_s3_ltrace_frz_yaw[S3_LTRACE_N];
volatile uint32_t g_s3_ltrace_frz_n;
volatile uint32_t g_s3_bc_dbg_us[S3_BC_MAX_ELEMENTS];
volatile uint32_t g_s3_bc_dbg_pos[S3_BC_MAX_ELEMENTS];
volatile uint32_t g_s3_bc_dbg_unit;
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
    g_total_pos        = 0;
    g_b_s3_bc_scanning = false;
    g_b_s3_bc_bars     = false;
    set_state(BARCODE_STATE_IDLE);
}

/* A read that had started (BARCODE_START was sent) is given up: tell the
   controller, or it waits in DECODE_BARCODE for its whole time limit
   (slow speed, for seconds) for a result that never comes. */
static void abort_read(void)
{
    s3_event_t evt = { 0 };

    evt.type = S3_EVT_BARCODE_FAILED;
    evt.code = (uint8_t)CODE39_ERR_LENGTH; /* Cast: fits 8 bits */
    s3_send_event(&evt);
    go_idle();
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

/* Wheel counts now (0 when distance is not used) */
static uint32_t pos_now(void)
{
    return s3_bc_pos();
}

/* The wheels were seen turning during this read: its end is judged by
   distance (S3_BC_QUIET_COUNTS), not by time. */
static bool dist_live(void)
{
#if S3_BC_USE_DIST
    return (g_total_pos >= DIST_LIVE_MIN);
#else
    return false;
#endif
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
    else if ((1U == (g_n_widths % 2U)) && dist_live())
    {
        tmo = DIST_POLL_MS; /* keep checking how far the white goes on */
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

/* Decode the stored elements: by wheel distance first (independent of
   speed changes), then by time.  *p_unit = 1 if distance gave the result
   (or the error, when both failed and distance was tried), 0 = time. */
uint32_t g_s3_bc_partial; /* reads accepted by the one-'*' rule */

static code39_result_t try_decode(uint8_t * p_text, uint32_t text_size,
                                  bool * p_reversed, uint32_t * p_unit)
{
    code39_options_t opt;
    code39_result_t  res = CODE39_ERR_LENGTH;
    code39_result_t  res_t;

    opt.min_ratio_x10 = S3_BC_MIN_RATIO_X10;
    opt.letters_only  = (S3_BC_LETTERS_ONLY != 0);
    opt.max_chars     = (uint8_t)S3_BC_MAX_CHARS;
    *p_unit           = 0U;

    if (g_n_widths < S3_BC_MIN_ELEMENTS)
    {
        return CODE39_ERR_LENGTH;
    }
#if S3_BC_USE_DIST
    if (g_total_pos >= S3_BC_DIST_MIN)
    {
        *p_unit = 1U;
        /* Cast: the text buffer holds plain characters. */
        res = code39_decode_ex(g_widths_pos, g_n_widths, &opt,
                               (char *)p_text, text_size, p_reversed);
        if (CODE39_OK == res)
        {
            return res;
        }
    }
#endif
    /* Cast: the text buffer holds plain characters. */
    res_t = code39_decode_ex(g_widths, g_n_widths, &opt, (char *)p_text,
                             text_size, p_reversed);
    if ((CODE39_OK == res_t) || (0U == *p_unit))
    {
        *p_unit = 0U;
        res     = res_t;
    }
#if S3_BC_PARTIAL
    if (CODE39_OK != res)
    {
        /* one end distorted: accept the letter next to a clean '*' */
        uint32_t unit_full = *p_unit;
#if S3_BC_USE_DIST
        if (g_total_pos >= S3_BC_DIST_MIN)
        {
            if (CODE39_OK == code39_decode_partial(g_widths_pos, g_n_widths,
                                                   &opt, (char *)p_text,
                                                   text_size, p_reversed))
            {
                *p_unit = 1U;
                g_s3_bc_partial++;
                return CODE39_OK;
            }
        }
#endif
        if (CODE39_OK == code39_decode_partial(g_widths, g_n_widths, &opt,
                                               (char *)p_text, text_size,
                                               p_reversed))
        {
            *p_unit = 0U;
            g_s3_bc_partial++;
            return CODE39_OK;
        }
        *p_unit   = unit_full;
        p_text[0] = 0U;
    }
#endif
    return res; /* both failed: report the distance error */
}

static void decode(void)
{
    s3_event_t      evt = { 0 };
    code39_result_t res;
    bool            b_reversed = false;
    uint32_t        unit       = 0U;

    set_state(BARCODE_STATE_DECODE);
    res = try_decode(evt.text, sizeof(evt.text), &b_reversed, &unit);

    {
        uint32_t k;
        for (k = 0U; (k < g_n_widths) && (k < S3_BC_MAX_ELEMENTS); k++)
        {
            g_s3_bc_dbg_us[k]  = g_widths[k];
            g_s3_bc_dbg_pos[k] = g_widths_pos[k];
        }
        g_s3_bc_dbg_n    = k;
        g_s3_bc_dbg_unit = unit;

        /* freeze the L trace now, while the bars are still in the ring */
        {
            uint32_t idx   = g_s3_ltrace_idx;
            uint32_t count = (idx < S3_LTRACE_N) ? idx : S3_LTRACE_N;
            uint32_t start = idx - count;
            for (k = 0U; k < count; k++)
            {
                g_s3_ltrace_frz[k] = g_s3_ltrace[(start + k) % S3_LTRACE_N];
                g_s3_ltrace_frz_fwd[k] =
                    g_s3_ltrace_fwd[(start + k) % S3_LTRACE_N];
                g_s3_ltrace_frz_yaw[k] =
                    g_s3_ltrace_yaw[(start + k) % S3_LTRACE_N];
            }
            g_s3_ltrace_frz_n = count;
        }
        g_s3_bc_dbg_seq++;
    }

    tk_loc_mtx(g_mtx_s3_status, TMO_FEVR);
    /* Cast: element count is at most S3_BC_MAX_ELEMENTS, result fits 8 bits */
    g_s3_status.bc_last_elems = (uint16_t)g_n_widths;
    g_s3_status.bc_last_err   = (uint8_t)res;
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

/* S3_BC_EARLY_END: a bar has just ended and the elements so far make a
   whole number of characters (10 n - 1).  If they decode with '*' at both
   ends, the barcode is complete; a longer barcode would still be missing
   its stop '*' here, so it does not decode and the read goes on. */
static bool early_end_ok(void)
{
#if S3_BC_EARLY_END
    uint8_t  text[S3_BC_TEXT_MAX];
    bool     b_rev = false;
    uint32_t unit  = 0U;

    if ((g_n_widths < S3_BC_MIN_ELEMENTS) || (9U != (g_n_widths % 10U)))
    {
        return false;
    }
    return (CODE39_OK == try_decode(text, sizeof(text), &b_rev, &unit));
#else
    return false;
#endif
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
                 ((uint32_t)S3_BC_START_WHITE_MS * S3_US_PER_MS)))
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
                abort_read(); /* not a barcode bar */
            }
            else if ((0U == p_run->black) && (g_n_widths > 0U) &&
                     (dist_live()
                          ? (p_run->width_pos >= S3_BC_QUIET_COUNTS)
                          : (p_run->width_us >= quiet_us())))
            {
                decode(); /* trailing quiet zone */
            }
            else if (g_n_widths < S3_BC_MAX_ELEMENTS)
            {
                if ((0U != p_run->black) && (g_n_widths > 0U))
                {
                    /* the bar began later than first guessed: that piece
                       belongs to the space before it */
                    g_widths[g_n_widths - 1U]     += p_run->lead_us;
                    g_widths_pos[g_n_widths - 1U] += p_run->lead_pos;
                    g_total_pos += p_run->lead_pos;
                }
                g_widths[g_n_widths]     = p_run->width_us;
                g_widths_pos[g_n_widths] = p_run->width_pos;
                g_total_pos += p_run->width_pos;
                g_n_widths++;
                if (g_n_widths >= 2U)
                {
                    g_b_s3_bc_bars = true; /* bar + space: a barcode */
                }
                if (p_run->width_us > g_widest_us)
                {
                    g_widest_us = p_run->width_us;
                }
                if (g_n_widths >= S3_BC_MAX_ELEMENTS)
                {
                    decode();
                }
                else if (early_end_ok())
                {
                    decode(); /* stop '*' seen: no need to wait */
                }
                else
                {
                    /* keep reading */
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
                if ((1U == (g_n_widths % 2U)) && dist_live())
                {
                    /* white: has it gone on far enough (or has the robot
                       stood still long enough) to be the end? */
                    g_white_ms += DIST_POLL_MS;
                    if (((pos_now() - g_last_run_pos) >= S3_BC_QUIET_COUNTS) ||
                        (g_white_ms >= (uint32_t)S3_BC_QUIET_STOP_MS))
                    {
                        decode();
                    }
                }
                else if (1U == (g_n_widths % 2U))
                {
                    decode(); /* quiet zone reached */
                }
                else
                {
                    abort_read(); /* black too long */
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
            g_last_run_pos = pos_now();
            g_white_ms     = 0U;
            on_run(&run);
        }
        else
        {
            /* Disabled: drain the run and ignore it. */
        }
    }
}

/*** end of file ***/
