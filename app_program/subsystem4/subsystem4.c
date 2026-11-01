/*
 *      subsystem4.c
 *      Subsystem 4 -- IMU terrain monitoring: kernel objects, task start,
 *      and the public API other subsystems call.
 *
 *      Kernel objects (TEAM.md section 6): 1 task, 1 cyclic handler,
 *      1 mutex, 1 event flag, 1 message buffer.
 */

#include "subsystem4.h"

ID               g_mbf_imu_evt; /* IF-S4-1 */
ID               g_tid_imu;
ID               g_cyc_imu;
ID               g_mtx_imu;
ID               g_flg_imu_req;
imu_status_t     g_s4_status;
volatile int32_t g_s4_speed_mm_s = -1; /* unknown until S2 reports */

#define S4_STATUS_TMO_MS 50 /* s4_get_status(): longest wait for the mutex */

ER subsystem4_init(void)
{
    T_CMBF cmbf = { 0 };
    T_CMTX cmtx = { 0 };
    T_CFLG cflg = { 0 };
    T_CCYC ccyc = { 0 };

    /* Event queue to the Vehicle Controller.  Fixed-size messages. */
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = S4_EVT_QUEUE_LEN * (sizeof(imu_event_msg_t) + sizeof(int32_t));
    cmbf.maxmsz   = sizeof(imu_event_msg_t);
    g_mbf_imu_evt = tk_cre_mbf(&cmbf);
    if (g_mbf_imu_evt < E_OK)
    {
        return g_mbf_imu_evt;
    }

    /* Mutex for g_s4_status.  Priority inheritance, so a low-priority
       reader (telemetry) cannot hold up the IMU task for long. */
    cmtx.mtxatr = TA_INHERIT;
    g_mtx_imu   = tk_cre_mtx(&cmtx);
    if (g_mtx_imu < E_OK)
    {
        return g_mtx_imu;
    }

    /* Requests from other subsystems (calibrate, reset peaks) */
    cflg.flgatr   = TA_TFIFO | TA_WMUL;
    cflg.iflgptn  = 0;
    g_flg_imu_req = tk_cre_flg(&cflg);
    if (g_flg_imu_req < E_OK)
    {
        return g_flg_imu_req;
    }

    /* 20 ms period.  Created stopped; started once the task exists. */
    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP)imu_cyclic_isr; /* Cast: kernel handler type */
    ccyc.cyctim = S4_PERIOD_MS;
    ccyc.cycphs = S4_PERIOD_MS;
    g_cyc_imu   = tk_cre_cyc(&ccyc);
    return (g_cyc_imu < E_OK) ? g_cyc_imu : E_OK;
}

ER subsystem4_start(void)
{
    /* Core 1 (TP_PRC2): keeps it away from the USB and WiFi service
       tasks pinned to core 0.  Change by team agreement (TEAM.md 4). */
    g_tid_imu = app_start_task(imu_task, S4_PRI_IMU, TP_PRC2, "imu_task");
    if (g_tid_imu < E_OK)
    {
        return g_tid_imu;
    }
    return tk_sta_cyc(g_cyc_imu);
}

/* ---- public API ----------------------------------------------------- */

ER s4_get_status(imu_status_t * p_out)
{
    ER ercd;

    ercd = tk_loc_mtx(g_mtx_imu, S4_STATUS_TMO_MS); /* never block long */
    if (ercd < E_OK)
    {
        return ercd;
    }
    *p_out = g_s4_status;
    tk_unl_mtx(g_mtx_imu);
    return E_OK;
}

void s4_set_speed_mm_s(int32_t speed_mm_s)
{
    g_s4_speed_mm_s = speed_mm_s; /* one aligned word: atomic */
}

ER s4_request_calibration(void)
{
    return tk_set_flg(g_flg_imu_req, S4_REQ_CALIBRATE);
}

ER s4_request_peak_reset(void)
{
    return tk_set_flg(g_flg_imu_req, S4_REQ_PEAK_RESET);
}

/*** end of file ***/
