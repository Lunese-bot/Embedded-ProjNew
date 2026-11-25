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
#include "code39_geom.h"
#include "line_algo.h" /* bc_start_step, bc_first_bar_ok */

#if S3_BC_GEOM && (C39G_MAX_SAMPLES < S3_LTRACE_N)
/* shape_read passes the whole frozen trace */
#error "C39G_MAX_SAMPLES must be at least S3_LTRACE_N"
#endif

#define IDLE_POLL_MS  100   /* how often requests are checked when idle */
#define DIST_POLL_MS  50    /* how often the white distance is checked */
#define DIST_LIVE_MIN 10u   /* counts in this read that prove the wheels turn */
#define S3_US_PER_MS  1000U /* run widths are in microseconds */

static uint32_t   g_widths[S3_BC_MAX_ELEMENTS];     /* by time, us */
static uint32_t   g_widths_pos[S3_BC_MAX_ELEMENTS]; /* by distance */
static uint32_t   g_total_pos;         /* distance covered by the stored ones */
static uint32_t   g_last_run_pos;      /* wheel counts when the last run came */
static uint32_t   g_white_ms;          /* time polled since then */
volatile uint32_t g_s3_bc_relearn_seq; /* + 1 per rearm (subsystem3.h) */

/* bench copy of the last read (subsystem3.h) */
volatile uint32_t      g_s3_bc_dbg_seq;
volatile uint32_t      g_s3_bc_dbg_n;
volatile uint16_t      g_s3_ltrace_frz[S3_LTRACE_N];
volatile uint8_t       g_s3_ltrace_frz_fwd[S3_LTRACE_N];
volatile int8_t        g_s3_ltrace_frz_yaw[S3_LTRACE_N];
volatile uint32_t      g_s3_ltrace_frz_n;
volatile uint32_t      g_s3_bc_dbg_us[S3_BC_MAX_ELEMENTS];
volatile uint32_t      g_s3_bc_dbg_pos[S3_BC_MAX_ELEMENTS];
volatile uint32_t      g_s3_bc_dbg_unit;
volatile uint8_t       g_s3_bc_geom_res = 255U;
volatile char          g_s3_bc_geom_best;
volatile uint16_t      g_s3_bc_geom_err_x100;
volatile uint16_t      g_s3_bc_geom_margin_x100;
volatile uint8_t       g_s3_bc_geom_fit;
volatile uint32_t      g_s3_bc_geom_ok;
volatile uint16_t      g_s3_bc_geom_ms;
volatile uint8_t       g_s3_bc_geom_try;
volatile char          g_s3_bc_geom_first_best;
volatile uint32_t      g_s3_bc_geom_tie_ovr;
static uint32_t        g_read_start_idx; /* g_s3_ltrace_idx at the first bar */
static bc_start_t      g_start;          /* which run starts a read */
static uint32_t        g_pend_idx;       /* g_s3_ltrace_idx when it was armed */
static uint32_t        g_frz_idx;        /* g_s3_ltrace_idx when frozen */
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
    bc_start_reset(&g_start);
    g_n_widths         = 0;
    g_widest_us        = 0;
    g_total_pos        = 0;
    g_b_s3_bc_scanning = false;
    g_b_s3_bc_bars     = false;
    set_state(BARCODE_STATE_IDLE);
}

/* ---- per-read record (subsystem3.h s3_bc_rec_t) ---- */
volatile s3_bc_rec_t g_s3_bc_rec[S3_BC_REC_N];
volatile uint32_t    g_s3_bc_rec_seq;
volatile uint32_t    g_s3_bc_rec_gen;
volatile int16_t     g_s3_bc_speed_now;
static bool          g_b_rec_open; /* the newest record's read is open */

static void rec_start(void)
{
    SYSTIM                 now;
    uint32_t               seq   = g_s3_bc_rec_seq + 1U;
    volatile s3_bc_rec_t * p_val = &g_s3_bc_rec[seq % S3_BC_REC_N];

    tk_get_tim(&now);
    p_val->seq      = seq;
    p_val->t_ms     = now.lo;
    p_val->pos      = s3_bc_pos();
    p_val->len      = 0U;
    /* Casts: raw levels and margins fit 16 bits */
    p_val->white    = (int16_t)g_s3_bc_white;
    p_val->margin   = (int16_t)g_s3_bc_margin;
    p_val->n        = 0U;
    p_val->end      = S3_BC_END_OPEN;
    p_val->path     = 0U;
    p_val->letter   = 0;
    p_val->lock_why = 0U;
    p_val->lock_len = 0U;
    p_val->v_start  = g_s3_bc_speed_now;
    p_val->v_min    = g_s3_bc_speed_now;
    p_val->v_end    = 0;
    p_val->v_up     = 0;
    p_val->up_len   = 0U;
    g_s3_bc_rec_seq = seq;
    g_b_rec_open    = true;
    g_s3_bc_rec_gen++;
}

static void rec_end(uint8_t why, uint8_t path, char letter)
{
    volatile s3_bc_rec_t * p_val = &g_s3_bc_rec[g_s3_bc_rec_seq % S3_BC_REC_N];

    if (!g_b_rec_open)
    {
        return;
    }
    g_b_rec_open  = false;
    p_val->len    = s3_bc_pos() - p_val->pos;
    /* Cast: at most S3_BC_MAX_ELEMENTS */
    p_val->n      = (uint8_t)g_n_widths;
    p_val->end    = why;
    p_val->path   = path;
    p_val->letter = letter;
    p_val->v_end  = g_s3_bc_speed_now;
    g_s3_bc_rec_gen++;
}

/* A read that had started (BARCODE_START was sent) is given up: tell the
   controller, or it waits in DECODE_BARCODE for its whole time limit
   (slow speed, for seconds) for a result that never comes. */
static void abort_read(uint8_t why)
{
    s3_event_t evt = { 0 };

    rec_end(why, 0U, 0);

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
        rec_end(S3_BC_END_DISABLE, 0U, 0);
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
        g_s3_bc_relearn_seq++; /* line_task learns L's white again */
        rec_end(S3_BC_END_REARM, 0U, 0);
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
        res = code39_decode_ex(g_widths_pos, g_n_widths, &opt, (char *)p_text,
                               text_size, p_reversed);
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

#if S3_BC_GEOM
static SYSTIM g_geom_slice; /* when shape_read last let others run */

/* Called often by c39g_decode.  Sleeps (lets the tasks below
   S3_PRI_GEOM run) only after S3_BC_GEOM_SLICE_MS of running: one
   tk_dly_tsk(1) takes up to 2 kernel ticks (10-20 ms here), so sleeping
   on every call made a refused read take ~1.6 s (2026-11-12 logs). */
static void geom_yield(void)
{
    SYSTIM now;

    tk_get_tim(&now);
    if ((uint32_t)(now.lo - g_geom_slice.lo) >= S3_BC_GEOM_SLICE_MS)
    {
        (void)tk_dly_tsk(1);
        tk_get_tim(&g_geom_slice);
    }
}

/* S3_BC_GEOM: the element list did not decode; look for the wide bars of
   one letter in the frozen L trace, from just before this read's first
   bar.  true = a letter was found (p_text, *p_reversed filled). */
static bool shape_read(uint8_t * p_text, uint32_t text_size, bool * p_reversed)
{
    c39g_info_t   info;
    c39g_result_t r_val;
    uint32_t      count  = g_s3_ltrace_frz_n;
    uint32_t      back   = (g_frz_idx - g_read_start_idx) + S3_BC_GEOM_LEAD;
    uint32_t      off    = (back < count) ? (count - back) : 0U;
    bool          b_dist = false;
    SYSTIM        t0_val;
    SYSTIM        t1_val;

    tk_get_tim(&t0_val);
    g_geom_slice = t0_val;
    /* run below imu_task (18) while this long job runs, so the IMU keeps
       its 20 ms rhythm without this task having to sleep often.
       19 is in S4's band (18-19, TEAM.md) but S4 only uses 18 */
    (void)tk_chg_pri(TSK_SELF, S3_PRI_GEOM);
    /* the bars are over: let line_task look for junctions again (its
       after-read hold still covers the last bars) while this runs */
    g_b_s3_bc_bars = false;
#if S3_BC_USE_DIST
    {
        int32_t  dist = 0;
        uint32_t kdx;
        for (kdx = off; kdx < count; kdx++)
        {
            dist += (int32_t)g_s3_ltrace_frz_fwd[kdx] -
                    (int32_t)g_s3_ltrace_frz_yaw[kdx];
        }
        /* Cast: S3_BC_DIST_MIN is a small count */
        b_dist = (dist >= (int32_t)S3_BC_DIST_MIN);
    }
#endif
    /* Casts: the frozen copy is written only by this task, so reading it
       as plain (non-volatile) memory during the call is safe. */
    /* smallest hump from this read's noise margin (frozen while the read
       was open; 0 = none, then C39G_MIN_HUMP) */
    r_val = c39g_decode_hump((const uint16_t *)&g_s3_ltrace_frz[off],
                             (const uint8_t *)&g_s3_ltrace_frz_fwd[off],
                             (const int8_t *)&g_s3_ltrace_frz_yaw[off],
                             count - off, b_dist, c39g_min_hump(g_s3_bc_margin),
                             geom_yield, &info);
    (void)tk_chg_pri(TSK_SELF, S3_PRI_BARCODE);
    tk_get_tim(&t1_val);
    /* Cast: well under a minute */
    g_s3_bc_geom_ms          = (uint16_t)(t1_val.lo - t0_val.lo);
    /* Cast: c39g_result_t values fit 8 bits */
    g_s3_bc_geom_res         = (uint8_t)r_val;
    g_s3_bc_geom_best        = info.best;
    g_s3_bc_geom_err_x100    = info.err_x100;
    g_s3_bc_geom_margin_x100 = info.margin_x100;
    g_s3_bc_geom_fit         = info.fit_x100;
    g_s3_bc_geom_try         = info.try_n;
    g_s3_bc_geom_first_best  = info.first_best;
    if ((C39G_OK == r_val) && (2u == info.try_n) &&
        ((uint8_t)C39G_TIE == info.first_res))
    {
        g_s3_bc_geom_tie_ovr++; /* second try named a letter after a tie */
    }
    if ((C39G_OK == r_val) && (text_size >= 2U))
    {
        /* Cast: 'A'..'Z' */
        p_text[0]   = (uint8_t)info.letter;
        p_text[1]   = 0U;
        *p_reversed = info.b_reversed;
        g_s3_bc_geom_ok++;
        return true;
    }
    return false;
}
#endif

static void decode(uint8_t why)
{
    s3_event_t      evt = { 0 };
    code39_result_t res;
    bool            b_reversed = false;
    uint32_t        unit       = 0U;
    uint32_t        partial0   = g_s3_bc_partial;
    uint8_t         path       = 0U;

    set_state(BARCODE_STATE_DECODE);
    res = try_decode(evt.text, sizeof(evt.text), &b_reversed, &unit);
    if (CODE39_OK == res)
    {
        path = (partial0 != g_s3_bc_partial) ? ((0U != unit) ? 3U : 4U)
                                             : ((0U != unit) ? 1U : 2U);
    }

    {
        uint32_t kdx;
        for (kdx = 0U; (kdx < g_n_widths) && (kdx < S3_BC_MAX_ELEMENTS); kdx++)
        {
            g_s3_bc_dbg_us[kdx]  = g_widths[kdx];
            g_s3_bc_dbg_pos[kdx] = g_widths_pos[kdx];
        }
        g_s3_bc_dbg_n    = kdx;
        g_s3_bc_dbg_unit = unit;

        /* freeze the L trace now, while the bars are still in the ring */
        {
            uint32_t idx   = g_s3_ltrace_idx;
            uint32_t count = (idx < S3_LTRACE_N) ? idx : S3_LTRACE_N;
            g_frz_idx      = idx;
            uint32_t start = idx - count;
            for (kdx = 0U; kdx < count; kdx++)
            {
                g_s3_ltrace_frz[kdx] = g_s3_ltrace[(start + kdx) % S3_LTRACE_N];
                g_s3_ltrace_frz_fwd[kdx] =
                    g_s3_ltrace_fwd[(start + kdx) % S3_LTRACE_N];
                g_s3_ltrace_frz_yaw[kdx] =
                    g_s3_ltrace_yaw[(start + kdx) % S3_LTRACE_N];
            }
            g_s3_ltrace_frz_n = count;
        }
        g_s3_bc_geom_res = 255U; /* not tried */
        g_s3_bc_geom_try = 0U;
#if S3_BC_GEOM
        if ((CODE39_OK != res) && (g_n_widths >= S3_BC_GEOM_MIN_ELEMENTS) &&
            shape_read(evt.text, sizeof(evt.text), &b_reversed))
        {
            res              = CODE39_OK;
            g_s3_bc_dbg_unit = 2U; /* shape read */
            path             = 5U;
        }
#endif
        g_s3_bc_dbg_seq++;
    }
    /* Cast: the text is 'A'..'Z' (letters only) */
    rec_end(why, path, (CODE39_OK == res) ? (char)evt.text[0] : 0);

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
            /* A white run ended, so a bar has just begun.  It may start a
               barcode only after a long white gap (the quiet zone), and
               only once it has ended as a believable bar: a paper sheet
               boundary or the line is far wider than any printed bar and
               must not slow the robot or open a read (owner 2026-11-24). */
            {
                uint8_t state =
                    bc_start_step(&g_start, 0U != p_run->black, p_run->width_us,
                                  p_run->width_pos);
                if (BC_START_ARMED == state)
                {
                    g_pend_idx = g_s3_ltrace_idx;
                }
                else if (BC_START_READ == state)
                {
                    set_state(BARCODE_STATE_DETECT);
                    g_b_s3_bc_scanning = true;
                    g_read_start_idx   = g_pend_idx;
                    evt.type           = S3_EVT_BARCODE_START;
                    s3_send_event(&evt);
                    set_state(BARCODE_STATE_READ);
                    rec_start();
                    /* this bar is the first element */
                    g_widths[0]     = p_run->width_us;
                    g_widths_pos[0] = p_run->width_pos;
                    g_total_pos     = p_run->width_pos;
                    g_widest_us     = p_run->width_us;
                    g_n_widths      = 1U;
                }
                else
                {
                    /* not a start */
                }
            }
            break;

        case BARCODE_STATE_READ:
            if ((0U != p_run->black) &&
                (p_run->width_us >
                 ((uint32_t)S3_BC_ELEMENT_MAX_MS * S3_US_PER_MS)))
            {
                abort_read(S3_BC_END_LONG_BLACK); /* not a barcode bar */
            }
            else if ((0U == p_run->black) && (g_n_widths > 0U) &&
                     (dist_live() ? (p_run->width_pos >= S3_BC_QUIET_COUNTS)
                                  : (p_run->width_us >= quiet_us())))
            {
                decode(S3_BC_END_QUIET_RUN); /* trailing quiet zone */
            }
            else if (g_n_widths < S3_BC_MAX_ELEMENTS)
            {
                if ((0U != p_run->black) && (g_n_widths > 0U))
                {
                    /* the bar began later than first guessed: that piece
                       belongs to the space before it */
                    g_widths[g_n_widths - 1U] += p_run->lead_us;
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
                    decode(S3_BC_END_MAX_ELEMS);
                }
                else if (early_end_ok())
                {
                    decode(S3_BC_END_EARLY); /* stop '*' seen: no need to wait
                                              */
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
                    if ((pos_now() - g_last_run_pos) >= S3_BC_QUIET_COUNTS)
                    {
                        decode(S3_BC_END_QUIET_DIST);
                    }
                    else if (g_white_ms >= (uint32_t)S3_BC_QUIET_STOP_MS)
                    {
                        decode(S3_BC_END_STOPPED);
                    }
                    else
                    {
                        /* keep waiting */
                    }
                }
                else if (1U == (g_n_widths % 2U))
                {
                    decode(S3_BC_END_QUIET_TIME); /* quiet zone reached */
                }
                else
                {
                    abort_read(S3_BC_END_BLACK_WAIT); /* black too long */
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
