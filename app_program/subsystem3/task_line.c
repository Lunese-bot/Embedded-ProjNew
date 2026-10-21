/*
 *	task_line.c
 *	Subsystem 3 -- the Line Following task (Design Review section 3).
 *
 *	Every S3_LINE_PERIOD_MS (10 ms) the cyclic handler wakes this task,
 *	which:
 *	  1. handles requests (calibrate, reset)            IF-33
 *	  2. reads IR_L, IR_C, IR_R                          drv_ir.c
 *	  3. updates line position, PD steering and state    line_algo.c
 *	  4. reports line lost / found / junction            IF-32
 *	  5. turns the barcode sensor into runs of black/white and hands
 *	     them to barcode_task                            mbf_s3_bc_runs
 *	  6. publishes the status snapshot                   IF-31
 *
 *	This task is the ONLY one that touches the IR pins and (in ADC
 *	mode) the ADC.
 *
 *	APIs shown: cyclic handler + tk_slp_tsk, tk_can_wup, tk_wai_flg
 *	(TMO_POL, TWF_BITCLR), tk_snd_mbf (TMO_POL), tk_loc_mtx / tk_unl_mtx
 */

#include "subsystem3.h"
#include "drv_ir.h"
#include "line_algo.h"

/* Line task state, private to this file */
LOCAL line_params_t	params;
LOCAL line_tracker_t	tracker;
LOCAL line_cal_t	cal;		/* accepted calibration */
LOCAL line_cal_t	cal_work;	/* calibration in progress */
LOCAL BOOL		calibrating;
LOCAL UW		cal_start_ms;
LOCAL UB		bc_level;	/* barcode sensor: 1 = black */
LOCAL UW		bc_run_start_us;
EXPORT UW		s3_line_overruns;	/* periods missed (task too slow) */
EXPORT UW		s3_line_read_errors;
EXPORT UW		s3_line_wakeups;	/* times the loop ran */
#define overruns	s3_line_overruns
#define read_errors	s3_line_read_errors

/* Cyclic handler: interrupt context, keep it this short */
EXPORT void s3_line_cyclic(void *exinf)
{
	(void)exinf;
	tk_wup_tsk(tid_s3_line);
}

LOCAL UW now_ms(void)
{
	SYSTIM	now;

	tk_get_tim(&now);
	return now.lo;
}

LOCAL void send_simple_event(UB type, B side)
{
	S3_EVENT	evt = {0};

	evt.type = type;
	evt.side = side;
	s3_send_event(&evt);
}

LOCAL void handle_requests(void)
{
	UINT	ptn = 0;

	/* TMO_POL: never wait here.  TWF_BITCLR clears only our bits. */
	if (tk_wai_flg(flg_s3_req, S3_FLG_LINE_REQ, TWF_ORW | TWF_BITCLR,
		       &ptn, TMO_POL) != E_OK) {
		return;
	}

	if ((ptn & S3_REQ_CALIBRATE) != 0) {
		line_cal_reset(&cal_work);
		calibrating  = TRUE;
		cal_start_ms = now_ms();
		tracker.state = LINE_STATE_CALIBRATE;
	}
	if ((ptn & S3_REQ_RESET_LINE) != 0) {
		line_tracker_reset(&tracker);	/* -> SEARCH, last side 0 */
		calibrating = FALSE;
	}
}

/* Step 1 of calibration: collect min/max until the time is up */
LOCAL void calibrate_step(const UH raw[S3_IR_COUNT])
{
	line_cal_update(&cal_work, raw);

	if ((now_ms() - cal_start_ms) < S3_CAL_DURATION_MS) {
		return;
	}

	calibrating = FALSE;
	if (line_cal_finish(&cal_work, (UB)DRV_IR_ANALOG_MASK,
			    S3_CAL_MIN_SPAN)) {
		cal = cal_work;
		send_simple_event(S3_EVT_CAL_DONE, 0);
	} else {
		send_simple_event(S3_EVT_CAL_FAILED, 0);	/* keep old cal */
	}
	line_tracker_reset(&tracker);
}

/* Barcode sensor -> runs.  A run is sent when the level changes. */
LOCAL void barcode_runs_step(const uint16_t norm[S3_IR_COUNT], UW t_us)
{
	UB		level;
	S3_BC_RUN	run;

	level = (norm[S3_BC_SENSOR] >= params.black_threshold) ? 1 : 0;
	if (level == bc_level) {
		return;
	}

	run.width_us = t_us - bc_run_start_us;	/* wraps correctly */
	run.black    = bc_level;
	(void)tk_snd_mbf(mbf_s3_bc_runs, &run, sizeof(run), TMO_POL);

	bc_level        = level;
	bc_run_start_us = t_us;
}

LOCAL void publish_status(const UH raw[S3_IR_COUNT])
{
	INT	i;

	tk_loc_mtx(mtx_s3_status, TMO_FEVR);
	s3_status.seq++;
	s3_status.time_ms    = now_ms();
	s3_status.line_state = tracker.state;
	s3_status.position   = tracker.position;
	s3_status.error      = tracker.error;
	s3_status.steering   = tracker.steering;
	s3_status.last_side  = tracker.last_side;
	s3_status.black_mask = tracker.black_mask;
	s3_status.calibrated = cal.b_valid ? TRUE : FALSE;
	for (i = 0; i < S3_IR_COUNT; i++) {
		s3_status.raw[i] = raw[i];
	}
	tk_unl_mtx(mtx_s3_status);
}

EXPORT void line_task(INT stacd, void *exinf)
{
	UH		raw[S3_IR_COUNT] = {0};
	uint16_t	norm[S3_IR_COUNT];
	line_evt_t	evt;
	UW		t_us;
	ER		er;

	(void)stacd; (void)exinf;

	line_params_default(&params);
	line_cal_reset(&cal);
	line_tracker_reset(&tracker);

	er = drv_ir_init();
	CHECK(er, "drv_ir_init");

	bc_level        = 0;
	bc_run_start_us = drv_ir_time_us();

	while (1) {
		tk_slp_tsk(TMO_FEVR);		/* woken by s3_line_cyclic */
		s3_line_wakeups++;
		if (tk_can_wup(TSK_SELF) > 0) {
			overruns++;		/* a period was missed */
		}

		handle_requests();

		t_us = drv_ir_time_us();
		if (drv_ir_read(raw) < E_OK) {
			read_errors++;		/* TODO: report S3 fault */
			continue;
		}

		if (calibrating) {
			calibrate_step(raw);
			publish_status(raw);
			continue;
		}

		line_normalise(&cal, (UB)DRV_IR_ANALOG_MASK,
			       S3_ADC_BLACK_IS_HIGH != 0,
			       (UB)S3_IR_BLACK_LEVEL, raw, norm);

		/* A barcode bar can look like a junction: no junctions
		   while barcode_task is reading one */
		evt = line_tracker_step(&tracker, &params, norm,
					s3_bc_scanning ? true : false);

		switch (evt) {
		case LINE_EVT_LOST:
			send_simple_event(S3_EVT_LINE_LOST, tracker.last_side);
			break;
		case LINE_EVT_FOUND:
			send_simple_event(S3_EVT_LINE_FOUND, tracker.last_side);
			break;
		case LINE_EVT_JUNCTION:
			send_simple_event(S3_EVT_JUNCTION, tracker.last_side);
			break;
		default:
			break;
		}

		s3_black_mask_live = tracker.black_mask;
		barcode_runs_step(norm, t_us);
		publish_status(raw);
	}
}
