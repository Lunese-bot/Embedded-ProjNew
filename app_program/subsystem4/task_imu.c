/*
 *	task_imu.c
 *	Subsystem 4 -- imu_task (owning task of I2C0) and its cyclic handler.
 *
 *	Every 20 ms the cyclic handler wakes imu_task, which:
 *	  1. serves requests from other subsystems (calibrate, reset peaks)
 *	  2. reads the LSM303DLHC                     (drv_lsm303.c)
 *	  3. runs the terrain state machine            (imu_terrain.c)
 *	  4. publishes IMU_STATUS under mtx_imu        (IF-S4-2)
 *	  5. sends IMU_EVENT_MSG to mbf_imu_evt        (IF-S4-1)
 *
 *	Fault handling (FR8, NFR5, Statechart 4 "Error Handling"):
 *	  S4_I2C_MAX_CONSEC failed reads in a row -> ERROR + IMU_EVT_FAULT,
 *	  then re-init every S4_REINIT_EVERY periods, at most S4_REINIT_MAX
 *	  times -> IMU_EVT_FAULT_FATAL.  The Vehicle Controller decides
 *	  whether that means Safe Stop; this task never touches the motors.
 */

#include "subsystem4.h"
#include "drv_lsm303.h"

#define S4_VERBOSE		1	/* 1 = print a status line ... */
#define S4_PRINT_EVERY		50	/* ... every 50 periods (1 s) */

EXPORT UW	s4_evt_dropped;
EXPORT UW	s4_overruns;
EXPORT UW	s4_i2c_errors;

LOCAL imu_terrain_t	terrain;	/* only imu_task touches this */
LOCAL UH		evt_seq;

/*
 * Cyclic handler: interrupt context.  Short, never waits.
 */
EXPORT void imu_cyclic_handler(void *exinf)
{
	(void)exinf;
	tk_wup_tsk(tid_imu);
}

/* Sensor axes -> robot axes (subsystem4_cfg.h) */
LOCAL void remap(const LSM303_RAW *r, imu_sample_t *s)
{
	const H acc[3] = { r->ax_mg, r->ay_mg, r->az_mg };
	const H mag[3] = { r->mx, r->my, r->mz };

	s->ax_mg = (H)(S4_ACC_X_SIGN * acc[S4_ACC_X_SRC]);
	s->ay_mg = (H)(S4_ACC_Y_SIGN * acc[S4_ACC_Y_SRC]);
	s->az_mg = (H)(S4_ACC_Z_SIGN * acc[S4_ACC_Z_SRC]);
	s->mx    = (H)(S4_MAG_X_SIGN * mag[S4_ACC_X_SRC]);
	s->my    = (H)(S4_MAG_Y_SIGN * mag[S4_ACC_Y_SRC]);
	s->mz    = (H)(S4_MAG_Z_SIGN * mag[S4_ACC_Z_SRC]);
}

LOCAL void send_event(IMU_EVENT_TYPE type, IMU_SEVERITY sev, W value)
{
	IMU_EVENT_MSG	msg;
	SYSTIM		now;

	tk_get_tim(&now);
	msg.type     = (UB)type;
	msg.severity = (UB)sev;
	msg.seq      = evt_seq++;
	msg.value    = value;
	msg.time_ms  = now.lo;

	/* Never block the 20 ms loop: if the controller is not reading,
	   count the loss instead (the seq number shows the gap). */
	if (tk_snd_mbf(mbf_imu_evt, &msg, sizeof(msg), TMO_POL) < E_OK) {
		s4_evt_dropped++;
	}
#if S4_VERBOSE
	tm_printf((UB*)"imu: event %d sev %d value %d\n",
		  (INT)type, (INT)sev, (INT)value);
#endif
}

/* Turn the algorithm's IMU_EV_* bits into messages */
LOCAL void report(UW ev, const imu_status_t *st)
{
	W tilt;

	if (ev & IMU_EV_CAL_DONE)   send_event(IMU_EVT_CAL_DONE,   IMU_SEV_INFO, 0);
	if (ev & IMU_EV_CAL_FAILED) send_event(IMU_EVT_CAL_FAILED, IMU_SEV_NONCRITICAL, 0);
	if (ev & IMU_EV_HUMP_START) send_event(IMU_EVT_HUMP_START, IMU_SEV_INFO, 0);
	if (ev & IMU_EV_HUMP_PEAK)  send_event(IMU_EVT_HUMP_PEAK,  IMU_SEV_INFO, st->hump_height_um);
	if (ev & IMU_EV_HUMP_END)   send_event(IMU_EVT_HUMP_END,   IMU_SEV_INFO, st->last_peak_um);
	if (ev & IMU_EV_HUMP_ABORT) send_event(IMU_EVT_HUMP_ABORT, IMU_SEV_INFO, 0);
	if (ev & IMU_EV_IMPACT)     send_event(IMU_EVT_IMPACT,     IMU_SEV_CRITICAL, st->last_impact_mg);
	if (ev & IMU_EV_TILT_LIMIT) {
		tilt = st->pitch_cdeg;
		if (((st->roll_cdeg < 0) ? -st->roll_cdeg : st->roll_cdeg) >
		    ((tilt < 0) ? -tilt : tilt)) {
			tilt = st->roll_cdeg;
		}
		send_event(IMU_EVT_TILT_LIMIT, IMU_SEV_CRITICAL, tilt);
	}
	if (ev & IMU_EV_MOTION_CHANGE) send_event(IMU_EVT_MOTION, IMU_SEV_INFO, (W)st->motion);
}

LOCAL void publish(void)
{
	if (tk_loc_mtx(mtx_imu, TMO_FEVR) == E_OK) {
		s4_status = terrain.st;
		tk_unl_mtx(mtx_imu);
	}
}

EXPORT void imu_task(INT stacd, void *exinf)
{
	imu_cfg_t	cfg;
	imu_sample_t	sample;
	LSM303_RAW	raw;
	UINT		req;
	UW		ev;
	ER		er;
	INT		consec_err  = 0;
	INT		reinit_wait = 0;
	INT		reinit_cnt  = 0;
	BOOL		fatal       = FALSE;
	UW		ticks       = 0;

	(void)stacd; (void)exinf;

	imu_terrain_default_cfg(&cfg);
	imu_terrain_init(&terrain, &cfg);	/* starts in CALIBRATING */

	er = lsm303_init();
	if (er < E_OK) {
		tm_printf((UB*)"imu: LSM303 init failed (%d)\n", er);
		imu_terrain_set_fault(&terrain, 1);
		send_event(IMU_EVT_FAULT, IMU_SEV_RECOVERABLE, er);
	} else {
		tm_printf((UB*)"imu: LSM303 ready, keep the robot still to calibrate\n");
	}
	publish();

	while (1) {
		/* Wait for the cyclic handler.  A timeout means the handler
		   stopped, which should never happen: report and carry on. */
		er = tk_slp_tsk(S4_PERIOD_MS * 5);
		if (er == E_TMOUT) {
			tm_printf((UB*)"imu: no tick from cyc_imu\n");
		}
		/* Extra queued wake-ups = we overran a period.  Drop them so
		   the loop does not run back-to-back to "catch up". */
		if (tk_can_wup(TSK_SELF) > 0) {
			s4_overruns++;
		}
		ticks++;

		/* 1. Requests from other subsystems (never wait for them) */
		if (tk_wai_flg(flg_imu_req, S4_REQ_CALIBRATE | S4_REQ_PEAK_RESET,
			       TWF_ORW | TWF_CLR, &req, TMO_POL) == E_OK) {
			if (req & S4_REQ_CALIBRATE) {
				imu_terrain_recalibrate(&terrain);
			}
			if (req & S4_REQ_PEAK_RESET) {
				imu_terrain_reset_peaks(&terrain);
			}
		}

		/* 2a. In ERROR: bounded re-init attempts */
		if (terrain.st.state == IMU_ST_ERROR) {
			if (fatal || (++reinit_wait < S4_REINIT_EVERY)) {
				continue;
			}
			reinit_wait = 0;
			lsm303_close();
			er = lsm303_init();
			if (er == E_OK) {
				consec_err = 0;
				reinit_cnt = 0;
				imu_terrain_set_fault(&terrain, 0); /* recalibrates */
				send_event(IMU_EVT_RECOVERED, IMU_SEV_INFO, 0);
			} else if (++reinit_cnt >= S4_REINIT_MAX) {
				fatal = TRUE;
				send_event(IMU_EVT_FAULT_FATAL, IMU_SEV_RECOVERABLE, er);
			} else {
				/* try again in S4_REINIT_EVERY periods */
			}
			publish();
			continue;
		}

		/* 2b. Read the sensor */
		er = lsm303_read(&raw);
		if (er < E_OK) {
			s4_i2c_errors++;
			if (++consec_err >= S4_I2C_MAX_CONSEC) {
				imu_terrain_set_fault(&terrain, 1);
				send_event(IMU_EVT_FAULT, IMU_SEV_RECOVERABLE, er);
				publish();
			}
			continue;	/* skip one sample; filters ride through */
		}
		consec_err = 0;

		/* 3. Algorithm */
		remap(&raw, &sample);
		ev = imu_terrain_update(&terrain, &sample, s4_speed_mm_s);

		/* 4. Publish, 5. events */
		publish();
		if (ev != 0U) {
			report(ev, &terrain.st);
		}

#if S4_VERBOSE
		if ((ticks % S4_PRINT_EVERY) == 0U) {
			tm_printf((UB*)"imu: %s %s pitch=%d roll=%d hdg=%d rate=%d"
				  " h=%dum max=%dum humps=%d impacts=%d\n",
				  imu_state_name(terrain.st.state),
				  imu_motion_name(terrain.st.motion),
				  terrain.st.pitch_cdeg, terrain.st.roll_cdeg,
				  terrain.st.heading_cdeg,
				  (INT)terrain.st.turn_rate_cdeg_s,
				  (INT)terrain.st.hump_height_um,
				  (INT)terrain.st.max_peak_um,
				  terrain.st.hump_count, terrain.st.impact_count);
		}
#endif
	}
}
