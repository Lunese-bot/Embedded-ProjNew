/*
 *	subsystem2.c
 *	Subsystem 2 -- Motion Control: creates its objects, starts its
 *	task, and provides the command/status helpers other tasks call.
 *
 *	APIs shown: tk_cre_mbf, tk_snd_mbf, tk_cre_mtx, tk_cre_flg,
 *	            tk_set_flg, tk_cre_cyc, tk_sta_cyc
 */

#include "subsystem2.h"
#include "s2_config.h"

EXPORT ID	mbf_motion_cmd;		/* IF-M1 */
EXPORT ID	mtx_motion;		/* IF-M2 */
EXPORT ID	flg_motion;		/* IF-M3 */
EXPORT ID	tid_motion;
EXPORT ID	cyc_motion;

#if S2_KEEP_EXAMPLE_IF
EXPORT ID	sem_slots;		/* example IF-02, see subsystem2.h */
EXPORT ID	mpf_samples;
#endif

#if S2_DEMO
LOCAL void	demo_task(INT stacd, void *exinf);
#endif

EXPORT ER subsystem2_init(void)
{
	T_CMBF	cmbf = {0};
	T_CMTX	cmtx = {0};
	T_CFLG	cflg = {0};
	T_CCYC	ccyc = {0};

	/* IF-M1: commands are copied into the buffer, so the Vehicle
	   Controller never shares memory with us.  Each message also
	   costs a small header inside the buffer, hence the + 8. */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = S2_CMD_QUEUE_LEN * (sizeof(MOTION_CMD) + 8);
	cmbf.maxmsz = sizeof(MOTION_CMD);
	mbf_motion_cmd = tk_cre_mbf(&cmbf);
	if (mbf_motion_cmd < E_OK) {
		return mbf_motion_cmd;
	}

	/* IF-M2: priority inheritance, so a low-priority reader (telemetry)
	   holding the lock cannot delay the control loop for long. */
	cmtx.mtxatr = TA_INHERIT;
	mtx_motion = tk_cre_mtx(&cmtx);
	if (mtx_motion < E_OK) {
		return mtx_motion;
	}

	/* IF-M3: several tasks may wait (VC, telemetry, demo). */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_motion = tk_cre_flg(&cflg);
	if (flg_motion < E_OK) {
		return flg_motion;
	}

	/* The control loop's clock.  Created stopped; started in
	   subsystem2_start() once the task it wakes exists. */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)motion_cyclic_handler;
	ccyc.cyctim = S2_PERIOD_MS;
	ccyc.cycphs = S2_PERIOD_MS;
	cyc_motion = tk_cre_cyc(&ccyc);
	if (cyc_motion < E_OK) {
		return cyc_motion;
	}

#if S2_KEEP_EXAMPLE_IF
	{
		/* Only so the example code in subsystems 3 and 4 still
		   builds and runs.  Nothing in subsystem 2 uses these. */
		T_CSEM	csem = {0};
		T_CMPF	cmpf = {0};

		csem.sematr  = TA_TFIFO;
		csem.isemcnt = S2_POOL_COUNT;
		csem.maxsem  = S2_POOL_COUNT;
		sem_slots = tk_cre_sem(&csem);
		cmpf.mpfatr = TA_TFIFO | TA_RNG3;
		cmpf.mpfcnt = S2_POOL_COUNT;
		cmpf.blfsz  = sizeof(SAMPLE_MSG);
		mpf_samples = tk_cre_mpf(&cmpf);
	}
#endif
	return E_OK;
}

EXPORT ER subsystem2_start(void)
{
	/* Pinned to core 1 so the 10 ms loop is not delayed by the USB
	   and WiFi service tasks, which run on core 0. */
	tid_motion = app_start_task(motion_task, S2_PRI_MOTION, TP_PRC2,
				    "motion_task");
	if (tid_motion < E_OK) {
		return tid_motion;
	}

#if S2_DEMO
	{
		ID tid = app_start_task(demo_task, S2_PRI_DEMO, 0,
					"s2_demo_task");
		if (tid < E_OK) {
			return tid;
		}
	}
#endif
	return tk_sta_cyc(cyc_motion);
}

/* ---------------------------------------------------------------------
 *  IF-M1 helpers.  These run in the CALLER's task.
 * --------------------------------------------------------------------- */
LOCAL UW s2_seq = 0;

EXPORT ER s2_send_cmd(MOTION_CMD *cmd)
{
	/* Only the Vehicle Controller sends commands (one task), so this
	   counter needs no lock.  s2_emergency_stop() from another task
	   may share a number; that is harmless, seq is only a label. */
	cmd->seq = ++s2_seq;
	return tk_snd_mbf(mbf_motion_cmd, cmd, sizeof(MOTION_CMD), TMO_POL);
}

LOCAL ER send_simple(INT type, INT speed, INT steer, INT dist, INT angle)
{
	MOTION_CMD	cmd = {0};

	cmd.type        = type;
	cmd.speed_mm_s  = speed;
	cmd.steer_mm_s  = steer;
	cmd.distance_mm = dist;
	cmd.angle_deg   = angle;
	return s2_send_cmd(&cmd);
}

EXPORT ER s2_cmd_stop(void)
{
	return send_simple(MOTION_STOP, 0, 0, 0, 0);
}

EXPORT ER s2_cmd_drive(INT speed_mm_s, INT steer_mm_s)
{
	return send_simple(MOTION_DRIVE, speed_mm_s, steer_mm_s, 0, 0);
}

EXPORT ER s2_cmd_drive_dist(INT speed_mm_s, INT distance_mm)
{
	return send_simple(MOTION_DRIVE_DIST, speed_mm_s, 0, distance_mm, 0);
}

EXPORT ER s2_cmd_turn(INT speed_mm_s, INT angle_deg)
{
	return send_simple(MOTION_TURN, speed_mm_s, 0, 0, angle_deg);
}

EXPORT ER s2_cmd_reset(void)
{
	return send_simple(MOTION_RESET, 0, 0, 0, 0);
}

EXPORT ER s2_emergency_stop(void)
{
	/* The flag works even if the command queue is full */
	ER er = tk_set_flg(flg_motion, MFLG_ESTOP_REQ);

	(void)send_simple(MOTION_SAFE_STOP, 0, 0, 0, 0);
	return er;
}

#if S2_DEMO
/* ---------------------------------------------------------------------
 *  Bench demo: stands in for the Vehicle Controller while it does not
 *  exist yet.  Build with S2_DEMO 1 in s2_config.h.
 * --------------------------------------------------------------------- */
LOCAL void demo_task(INT stacd, void *exinf)
{
	UINT	ptn;
	const UINT wait_bits = MFLG_CMD_DONE | MFLG_FAULT | MFLG_SAFE_STOPPED;

	(void)stacd; (void)exinf;
	tk_dly_tsk(2000);		/* time to put the robot down */

	while (1) {
		tm_printf((UB*)"s2 demo: forward 300 mm\n");
		s2_cmd_drive_dist(200, 300);
		tk_wai_flg(flg_motion, wait_bits, TWF_ORW | TWF_BITCLR,
			   &ptn, 10000);
		if (ptn & (MFLG_FAULT | MFLG_SAFE_STOPPED)) break;
		tk_dly_tsk(500);

		tm_printf((UB*)"s2 demo: turn right 90 deg\n");
		s2_cmd_turn(S2_TURN_SPEED_MM_S, -90);
		tk_wai_flg(flg_motion, wait_bits, TWF_ORW | TWF_BITCLR,
			   &ptn, 10000);
		if (ptn & (MFLG_FAULT | MFLG_SAFE_STOPPED)) break;
		tk_dly_tsk(1000);
	}
	tm_printf((UB*)"s2 demo: stopped by fault/safe stop\n");
	tk_exd_tsk();
}
#endif
