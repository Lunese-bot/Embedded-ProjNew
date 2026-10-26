/*
 *      task_line.c
 *      Subsystem 3 -- the Line Following task (Design Review section 3).
 *
 *      Every S3_LINE_PERIOD_MS (10 ms) the cyclic handler wakes this task,
 *      which:
 *        1. handles requests (calibrate, reset)            IF-33
 *        2. reads IR_L, IR_C, IR_R                          drv_ir.c
 *        3. updates line position, PD steering and state    line_algo.c
 *        4. reports line lost / found / junction            IF-32
 *        5. turns the barcode sensor into runs of black/white and hands
 *           them to barcode_task                            g_mbf_s3_bc_runs
 *        6. publishes the status snapshot                   IF-31
 *
 *      This task is the ONLY one that touches the IR pins and (in ADC
 *      mode) the ADC.
 *
 *      APIs shown: cyclic handler + tk_slp_tsk, tk_can_wup, tk_wai_flg
 *      (TMO_POL, TWF_BITCLR), tk_snd_mbf (TMO_POL), tk_loc_mtx / tk_unl_mtx
 */

#include "subsystem3.h"
#include "drv_ir.h"
#include "line_algo.h"

/* Line task state, private to this file */
static line_params_t  g_params;
static line_tracker_t g_tracker;
static line_cal_t     g_cal;      /* accepted calibration */
static line_cal_t     g_cal_work; /* calibration in progress */
static bool           g_b_calibrating;
static uint32_t       g_cal_start_ms;
static uint8_t        g_bc_level; /* barcode sensor: 1 = black */
static uint32_t       g_bc_run_start_us;
uint32_t              g_s3_line_overruns; /* periods missed (task too slow) */
uint32_t              g_s3_line_read_errors;
uint32_t              g_s3_line_wakeups; /* times the loop ran */

/* Cyclic handler: interrupt context, keep it this short */
void s3_line_cyclic_isr(void * p_exinf)
{
    (void)p_exinf;
    tk_wup_tsk(g_tid_s3_line);
}

static uint32_t now_ms(void)
{
    SYSTIM now;

    tk_get_tim(&now);
    return now.lo;
}

static void send_simple_event(uint8_t type, int8_t side)
{
    s3_event_t evt = { 0 };

    evt.type = type;
    evt.side = side;
    s3_send_event(&evt);
}

static void handle_requests(void)
{
    UINT ptn = 0;

    /* TMO_POL: never wait here.  TWF_BITCLR clears only our bits. */
    if (E_OK != tk_wai_flg(g_flg_s3_req, S3_FLG_LINE_REQ, TWF_ORW | TWF_BITCLR,
                           &ptn, TMO_POL))
    {
        ptn = 0U; /* no request pending */
    }

    if (0U != (ptn & S3_REQ_CALIBRATE))
    {
        line_cal_reset(&g_cal_work);
        g_b_calibrating = true;
        g_cal_start_ms  = now_ms();
        g_tracker.state = LINE_STATE_CALIBRATE;
    }
    if (0U != (ptn & S3_REQ_RESET_LINE))
    {
        line_tracker_reset(&g_tracker); /* -> SEARCH, last side 0 */
        g_b_calibrating = false;
    }
}

/* Step 1 of calibration: collect min/max until the time is up */
static void calibrate_step(const uint16_t p_raw[S3_IR_COUNT])
{
    line_cal_update(&g_cal_work, p_raw);

    if ((now_ms() - g_cal_start_ms) >= S3_CAL_DURATION_MS)
    {
        g_b_calibrating = false;
        /* Cast: the analog mask fits in 8 bits (one bit per sensor). */
        if (line_cal_finish(&g_cal_work, (uint8_t)DRV_IR_ANALOG_MASK,
                            S3_CAL_MIN_SPAN))
        {
            g_cal = g_cal_work;
            send_simple_event(S3_EVT_CAL_DONE, 0);
        }
        else
        {
            send_simple_event(S3_EVT_CAL_FAILED, 0); /* keep old cal */
        }
        line_tracker_reset(&g_tracker);
    }
}

/* Barcode sensor -> runs.  A run is sent when the level changes. */
static void barcode_runs_step(const uint16_t p_norm[S3_IR_COUNT], uint32_t t_us)
{
    uint8_t     level;
    s3_bc_run_t run;

    level = (p_norm[S3_BC_SENSOR] >= g_params.black_threshold) ? 1U : 0U;
    if (level != g_bc_level)
    {
        run.width_us = t_us - g_bc_run_start_us; /* wraps correctly */
        run.black    = g_bc_level;
        (void)tk_snd_mbf(g_mbf_s3_bc_runs, &run, sizeof(run), TMO_POL);

        g_bc_level        = level;
        g_bc_run_start_us = t_us;
    }
}

static void publish_status(const uint16_t p_raw[S3_IR_COUNT])
{
    int32_t idx;

    tk_loc_mtx(g_mtx_s3_status, TMO_FEVR);
    g_s3_status.seq++;
    g_s3_status.time_ms    = now_ms();
    g_s3_status.line_state = g_tracker.state;
    g_s3_status.position   = g_tracker.position;
    g_s3_status.error      = g_tracker.error;
    g_s3_status.steering   = g_tracker.steering;
    g_s3_status.last_side  = g_tracker.last_side;
    g_s3_status.black_mask = g_tracker.black_mask;
    g_s3_status.calibrated = g_cal.b_valid;
    for (idx = 0; idx < S3_IR_COUNT; idx++)
    {
        g_s3_status.raw[idx] = p_raw[idx];
    }
    tk_unl_mtx(g_mtx_s3_status);
}

/* One normal 10 ms step: track the line, report events, feed barcodes */
static void follow_step(const uint16_t p_raw[S3_IR_COUNT], uint32_t t_us)
{
    uint16_t   norm[S3_IR_COUNT];
    line_evt_t evt;

    /* Cast: the analog mask and the digital black level fit in 8 bits. */
    line_normalise(&g_cal, (uint8_t)DRV_IR_ANALOG_MASK,
                   (0 != S3_ADC_BLACK_IS_HIGH), (uint8_t)S3_IR_BLACK_LEVEL,
                   p_raw, norm);

    /* A barcode bar can look like a junction: no junctions
       while barcode_task is reading one */
    evt = line_tracker_step(&g_tracker, &g_params, norm, g_b_s3_bc_scanning);

    switch (evt)
    {
        case LINE_EVT_LOST:
            send_simple_event(S3_EVT_LINE_LOST, g_tracker.last_side);
            break;
        case LINE_EVT_FOUND:
            send_simple_event(S3_EVT_LINE_FOUND, g_tracker.last_side);
            break;
        case LINE_EVT_JUNCTION:
            send_simple_event(S3_EVT_JUNCTION, g_tracker.last_side);
            break;
        default:
            break;
    }

    g_s3_black_mask_live = g_tracker.black_mask;
    barcode_runs_step(norm, t_us);
    publish_status(p_raw);
}

void line_task(int32_t stacd, void * p_exinf)
{
    uint16_t raw[S3_IR_COUNT] = { 0 };
    uint32_t t_us;
    ER       ercd;

    (void)stacd;
    (void)p_exinf;

    line_params_default(&g_params);
    line_cal_reset(&g_cal);
    line_tracker_reset(&g_tracker);

    ercd = drv_ir_init();
    app_check(ercd, "drv_ir_init");

    g_bc_level        = 0;
    g_bc_run_start_us = drv_ir_time_us();

    for (;;)
    {
        tk_slp_tsk(TMO_FEVR); /* woken by s3_line_cyclic_isr */
        g_s3_line_wakeups++;
        if (tk_can_wup(TSK_SELF) > 0)
        {
            g_s3_line_overruns++; /* a period was missed */
        }

        handle_requests();

        t_us = drv_ir_time_us();
        if (drv_ir_read(raw) < E_OK)
        {
            g_s3_line_read_errors++; /* TODO: report S3 fault */
        }
        else if (g_b_calibrating)
        {
            calibrate_step(raw);
            publish_status(raw);
        }
        else
        {
            follow_step(raw, t_us);
        }
    }
}

/*** end of file ***/
