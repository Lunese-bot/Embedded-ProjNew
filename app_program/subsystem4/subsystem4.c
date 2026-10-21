/*
 *	subsystem4.c
 *	Subsystem 4 -- IMU terrain monitoring: kernel objects, task start,
 *	and the public API other subsystems call.
 *
 *	Kernel objects (TEAM.md section 6): 1 task, 1 cyclic handler,
 *	1 mutex, 1 event flag, 1 message buffer.
 */

#include "subsystem4.h"

EXPORT ID		mbf_imu_evt;	/* IF-S4-1 */
EXPORT ID		tid_imu;
EXPORT ID		cyc_imu;
EXPORT ID		mtx_imu;
EXPORT ID		flg_imu_req;
EXPORT IMU_STATUS	s4_status;
EXPORT volatile W	s4_speed_mm_s = -1;	/* unknown until S2 reports */

EXPORT ER subsystem4_init(void)
{
	T_CMBF	cmbf = {0};
	T_CMTX	cmtx = {0};
	T_CFLG	cflg = {0};
	T_CCYC	ccyc = {0};

	/* Event queue to the Vehicle Controller.  Fixed-size messages. */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = S4_EVT_QUEUE_LEN * (sizeof(IMU_EVENT_MSG) + sizeof(W));
	cmbf.maxmsz = sizeof(IMU_EVENT_MSG);
	mbf_imu_evt = tk_cre_mbf(&cmbf);
	if (mbf_imu_evt < E_OK) {
		return mbf_imu_evt;
	}

	/* Mutex for s4_status.  Priority inheritance, so a low-priority
	   reader (telemetry) cannot hold up the IMU task for long. */
	cmtx.mtxatr = TA_INHERIT;
	mtx_imu = tk_cre_mtx(&cmtx);
	if (mtx_imu < E_OK) {
		return mtx_imu;
	}

	/* Requests from other subsystems (calibrate, reset peaks) */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_imu_req = tk_cre_flg(&cflg);
	if (flg_imu_req < E_OK) {
		return flg_imu_req;
	}

	/* 20 ms period.  Created stopped; started once the task exists. */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)imu_cyclic_handler;
	ccyc.cyctim = S4_PERIOD_MS;
	ccyc.cycphs = S4_PERIOD_MS;
	cyc_imu = tk_cre_cyc(&ccyc);
	return (cyc_imu < E_OK) ? cyc_imu : E_OK;
}

EXPORT ER subsystem4_start(void)
{
	/* Core 1 (TP_PRC2): keeps it away from the USB and WiFi service
	   tasks pinned to core 0.  Change by team agreement (TEAM.md 4). */
	tid_imu = app_start_task(imu_task, S4_PRI_IMU, TP_PRC2, "imu_task");
	if (tid_imu < E_OK) {
		return tid_imu;
	}
	return tk_sta_cyc(cyc_imu);
}

/* ---- public API ----------------------------------------------------- */

EXPORT ER s4_get_status(IMU_STATUS *out)
{
	ER	er;

	er = tk_loc_mtx(mtx_imu, 50);		/* never block a caller long */
	if (er < E_OK) {
		return er;
	}
	*out = s4_status;
	tk_unl_mtx(mtx_imu);
	return E_OK;
}

EXPORT void s4_set_speed_mm_s(W speed_mm_s)
{
	s4_speed_mm_s = speed_mm_s;		/* one aligned word: atomic */
}

EXPORT ER s4_request_calibration(void)
{
	return tk_set_flg(flg_imu_req, S4_REQ_CALIBRATE);
}

EXPORT ER s4_request_peak_reset(void)
{
	return tk_set_flg(flg_imu_req, S4_REQ_PEAK_RESET);
}
