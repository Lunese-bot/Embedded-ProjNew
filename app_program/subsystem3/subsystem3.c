/*
 *      subsystem3.c
 *      Subsystem 3 -- IR Line Following and Barcode Navigation: creates its
 *      objects, starts its tasks, and implements the public calls of
 *      subsystem3.h (IF-31, IF-32, IF-33).
 *
 *      Kernel objects (TEAM.md section 6):
 *        2 tasks        line_task, barcode_task
 *        1 cyclic       g_cyc_s3_line      wakes line_task every 10 ms
 *        1 mutex        g_mtx_s3_status    guards g_s3_status (IF-31)
 *        2 msg buffers  g_mbf_s3_events    events to the controller (IF-32)
 *                       g_mbf_s3_bc_runs   barcode runs, line -> barcode task
 *        1 event flag   g_flg_s3_req       requests (IF-33)
 */

#include "subsystem3.h"

ID               g_mbf_s3_events  = E_NOEXS; /* IF-32 */
ID               g_mtx_s3_status  = E_NOEXS; /* IF-31 */
ID               g_mbf_s3_bc_runs = E_NOEXS;
ID               g_flg_s3_req     = E_NOEXS; /* IF-33 */
ID               g_tid_s3_line    = E_NOEXS;
ID               g_tid_s3_barcode = E_NOEXS;
ID               g_cyc_s3_line    = E_NOEXS;
s3_line_status_t g_s3_status;
volatile bool    g_b_s3_bc_scanning   = false;
volatile uint8_t g_s3_black_mask_live = 0;

#define S3_BC_RUN_QUEUE_LEN (S3_BC_MAX_ELEMENTS + 4)

ER subsystem3_init(void)
{
    T_CMTX cmtx = { 0 };
    T_CMBF cmbf = { 0 };
    T_CFLG cflg = { 0 };
    T_CCYC ccyc = { 0 };

    g_s3_status.line_state = LINE_STATE_INIT;
    g_s3_status.bc_state   = BARCODE_STATE_IDLE;
    g_s3_status.bc_enabled = true;

    /* Mutex with priority inheritance: the 10 ms line task must not be
       held up by a low-priority reader (e.g. telemetry) */
    cmtx.mtxatr     = TA_INHERIT;
    g_mtx_s3_status = tk_cre_mtx(&cmtx);
    if (g_mtx_s3_status < E_OK)
    {
        return g_mtx_s3_status;
    }

    /* IF-32: fixed-size events, copied into the buffer */
    cmbf.mbfatr     = TA_TFIFO;
    cmbf.bufsz      = S3_EVT_QUEUE_LEN * (sizeof(s3_event_t) + sizeof(int32_t));
    cmbf.maxmsz     = sizeof(s3_event_t);
    g_mbf_s3_events = tk_cre_mbf(&cmbf);
    if (g_mbf_s3_events < E_OK)
    {
        return g_mbf_s3_events;
    }

    /* Barcode runs: enough for one whole barcode plus margin */
    cmbf.bufsz  = S3_BC_RUN_QUEUE_LEN * (sizeof(s3_bc_run_t) + sizeof(int32_t));
    cmbf.maxmsz = sizeof(s3_bc_run_t);
    g_mbf_s3_bc_runs = tk_cre_mbf(&cmbf);
    if (g_mbf_s3_bc_runs < E_OK)
    {
        return g_mbf_s3_bc_runs;
    }

    /* IF-33: TA_WMUL so both tasks may wait on their own bits;
       each task clears only the bits it handles */
    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0;
    g_flg_s3_req = tk_cre_flg(&cflg);
    if (g_flg_s3_req < E_OK)
    {
        return g_flg_s3_req;
    }

    /* Cyclic handler: fixed 10 ms sampling, created stopped */
    ccyc.cycatr   = TA_HLNG;
    ccyc.cychdr   = (FP)s3_line_cyclic_isr; /* Cast: kernel handler type */
    ccyc.cyctim   = S3_LINE_PERIOD_MS;
    ccyc.cycphs   = S3_LINE_PERIOD_MS;
    g_cyc_s3_line = tk_cre_cyc(&ccyc);
    return (g_cyc_s3_line < E_OK) ? g_cyc_s3_line : E_OK;
}

ER subsystem3_start(void)
{
    /* Both on core 1 (TP_PRC2): line_task produces barcode runs that
       barcode_task consumes; keeping them on one core keeps their
       order simple.  Not core 0: the USB console (pri 3) and CYW43
       (pri 4) service tasks are pinned there, and on the bench they
       starved line_task (READY, 90 loops in ~15 s).  Core 1 holds only
       motion_task (14) and imu_task (18).  TEAM.md section 4. */
    g_tid_s3_barcode =
        app_start_task(barcode_task, S3_PRI_BARCODE, TP_PRC2, "barcode_task");
    if (g_tid_s3_barcode < E_OK)
    {
        return g_tid_s3_barcode;
    }

    g_tid_s3_line =
        app_start_task(line_task, S3_PRI_LINE, TP_PRC2, "line_task");
    if (g_tid_s3_line < E_OK)
    {
        return g_tid_s3_line;
    }

    return tk_sta_cyc(g_cyc_s3_line);
}

/* IF-31 */
ER s3_get_line_status(s3_line_status_t * p_out)
{
    ER ercd;

    if (NULL == p_out)
    {
        return E_PAR;
    }
    ercd = tk_loc_mtx(g_mtx_s3_status, TMO_FEVR);
    if (ercd < E_OK)
    {
        return ercd;
    }
    *p_out = g_s3_status;
    return tk_unl_mtx(g_mtx_s3_status);
}

/* IF-33 */
ER s3_request(uint32_t req)
{
    /* Cast: the request bits are unsigned flag-pattern bits. */
    if (0U != (req & ~(uint32_t)(S3_FLG_LINE_REQ | S3_FLG_BC_REQ)))
    {
        return E_PAR;
    }
    return tk_set_flg(g_flg_s3_req, req);
}

/* IF-32: never blocks.  A full buffer drops the event and counts it. */
void s3_send_event(s3_event_t * p_evt)
{
    SYSTIM now;
    ER     ercd;

    tk_get_tim(&now);
    p_evt->time_ms = now.lo;

    ercd = tk_snd_mbf(g_mbf_s3_events, p_evt, sizeof(s3_event_t), TMO_POL);
    if (E_OK != ercd)
    {
        tk_loc_mtx(g_mtx_s3_status, TMO_FEVR);
        g_s3_status.evt_dropped++;
        tk_unl_mtx(g_mtx_s3_status);
    }
}

/* Barcode text -> navigation command.  PLACEHOLDER mapping in
   s3_config.h; only the first character is used. */
navigation_command_t s3_text_to_nav(const uint8_t * p_text)
{
    navigation_command_t nav;

    switch (p_text[0])
    {
        case S3_BC_CHAR_LEFT:
            nav = NAV_LEFT;
            break;
        case S3_BC_CHAR_RIGHT:
            nav = NAV_RIGHT;
            break;
        case S3_BC_CHAR_STRAIGHT:
            nav = NAV_STRAIGHT;
            break;
        case S3_BC_CHAR_UTURN:
            nav = NAV_U_TURN;
            break;
        default:
            nav = NAV_NONE;
            break;
    }
    return nav;
}

/*** end of file ***/
