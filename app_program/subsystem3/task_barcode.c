/*
 *	task_barcode.c
 *	Subsystem 3 -- the Barcode Detection task (Design Review section 3).
 *
 *	line_task turns the barcode sensor into runs ("black for 41 ms",
 *	"white for 18 ms", ...) and sends them through mbf_s3_bc_runs.
 *	This task collects the runs of one barcode and decodes them.
 *
 *	BarcodeState (robot_types.h), Design Review 6.2:
 *
 *	  IDLE   --bar after a long white gap--> DETECT (send BARCODE_START)
 *	  DETECT --> READ    collect bar/space widths
 *	  READ   --white gap > quiet zone, or no change for that long--> DECODE
 *	  READ   --black run too long (a crossing line, not a bar)--> IDLE
 *	  DECODE --ok--> COMPLETE (send BARCODE_DECODED + nav) --> IDLE
 *	  DECODE --error--> RETRY (send BARCODE_FAILED + retries) --> IDLE
 *
 *	Widths are in microseconds, so the robot should hold a steady speed
 *	over the barcode (the controller slows down on BARCODE_START).
 *	Each Code 39 character is classified on its own, which tolerates a
 *	slow speed change.
 *
 *	APIs shown: tk_rcv_mbf with a timeout, tk_wai_flg (TMO_POL)
 */

#include "subsystem3.h"
#include "code39.h"

#define IDLE_POLL_MS	100	/* how often requests are checked when idle */

LOCAL uint32_t	widths[S3_BC_MAX_ELEMENTS];
LOCAL UINT	n_widths;
LOCAL UW	widest_us;
LOCAL UW	retries;
LOCAL BOOL	enabled = TRUE;
LOCAL BarcodeState	state = BARCODE_STATE_IDLE;

LOCAL void set_state(BarcodeState next)
{
	state = next;
	tk_loc_mtx(mtx_s3_status, TMO_FEVR);
	s3_status.bc_state   = next;
	s3_status.bc_enabled = enabled;
	tk_unl_mtx(mtx_s3_status);
}

LOCAL void go_idle(void)
{
	n_widths       = 0;
	widest_us      = 0;
	s3_bc_scanning = FALSE;
	set_state(BARCODE_STATE_IDLE);
}

LOCAL void handle_requests(void)
{
	UINT	ptn = 0;

	if (tk_wai_flg(flg_s3_req, S3_FLG_BC_REQ, TWF_ORW | TWF_BITCLR,
		       &ptn, TMO_POL) != E_OK) {
		return;
	}
	if ((ptn & S3_REQ_BC_DISABLE) != 0) {
		enabled = FALSE;
		go_idle();
	}
	if ((ptn & S3_REQ_BC_ENABLE) != 0) {
		enabled = TRUE;
		set_state(state);
	}
	if ((ptn & S3_REQ_BC_REARM) != 0) {
		retries = 0;
		go_idle();
	}
}

/* Quiet zone: S3_BC_QUIET_FACTOR x widest element, at least the minimum */
LOCAL UW quiet_us(void)
{
	UW	q = widest_us * S3_BC_QUIET_FACTOR;

	return (q < (UW)S3_BC_QUIET_MIN_MS * 1000) ?
		(UW)S3_BC_QUIET_MIN_MS * 1000 : q;
}

LOCAL TMO wait_timeout(void)
{
	if (state != BARCODE_STATE_READ) {
		return IDLE_POLL_MS;
	}
	/* Last element stored was a bar -> the sensor is on white now:
	   end of barcode after a quiet zone.  Otherwise it is on black:
	   give up after the longest allowed bar. */
	if ((n_widths % 2) == 1) {
		return (TMO)(quiet_us() / 1000) + 1;
	}
	return S3_BC_ELEMENT_MAX_MS;
}

LOCAL void decode(void)
{
	S3_EVENT	evt = {0};
	code39_params_t	prm;
	code39_result_t	res;
	bool		reversed = false;

	set_state(BARCODE_STATE_DECODE);
	prm.min_ratio_x10 = S3_BC_MIN_RATIO_X10;

	if (n_widths < S3_BC_MIN_ELEMENTS) {
		res = CODE39_ERR_LENGTH;
	} else {
		res = code39_decode(widths, n_widths, &prm,
				    (char *)evt.text, sizeof(evt.text),
				    &reversed);
	}

	tk_loc_mtx(mtx_s3_status, TMO_FEVR);
	if (res == CODE39_OK) {
		s3_status.bc_ok++;
	} else {
		s3_status.bc_fail++;
	}
	tk_unl_mtx(mtx_s3_status);

	if (res == CODE39_OK) {
		set_state(BARCODE_STATE_COMPLETE);
		retries  = 0;
		evt.type = S3_EVT_BARCODE_DECODED;
		evt.nav  = (UB)s3_text_to_nav(evt.text);
		evt.arg  = reversed ? 1 : 0;	/* 1 = read backwards */
	} else {
		set_state(BARCODE_STATE_RETRY);
		retries++;
		evt.type = S3_EVT_BARCODE_FAILED;
		evt.code = (UB)res;
		evt.arg  = retries;	/* controller gives up after
					   S3_BC_MAX_RETRIES */
	}
	s3_send_event(&evt);
	go_idle();
}

LOCAL void on_run(const S3_BC_RUN *p_run)
{
	S3_EVENT	evt = {0};

	switch (state) {
	case BARCODE_STATE_IDLE:
		/* A white run ended, so a bar has just begun.  It starts a
		   barcode only after a long white gap (the quiet zone). */
		if (!p_run->black &&
		    p_run->width_us >= (UW)S3_BC_QUIET_MIN_MS * 1000) {
			set_state(BARCODE_STATE_DETECT);
			s3_bc_scanning = TRUE;
			n_widths  = 0;
			widest_us = 0;
			evt.type  = S3_EVT_BARCODE_START;
			s3_send_event(&evt);
			set_state(BARCODE_STATE_READ);
		}
		break;

	case BARCODE_STATE_READ:
		if (p_run->black && p_run->width_us >
		    (UW)S3_BC_ELEMENT_MAX_MS * 1000) {
			go_idle();		/* not a barcode bar */
			break;
		}
		if (!p_run->black && n_widths > 0 &&
		    p_run->width_us >= quiet_us()) {
			decode();		/* trailing quiet zone */
			break;
		}
		widths[n_widths++] = p_run->width_us;
		if (p_run->width_us > widest_us) {
			widest_us = p_run->width_us;
		}
		if (n_widths >= S3_BC_MAX_ELEMENTS) {
			decode();
		}
		break;

	default:
		go_idle();
		break;
	}
}

EXPORT void barcode_task(INT stacd, void *exinf)
{
	S3_BC_RUN	run;
	INT		sz;

	(void)stacd; (void)exinf;

	go_idle();

	while (1) {
		handle_requests();

		sz = tk_rcv_mbf(mbf_s3_bc_runs, &run, wait_timeout());

		if (sz == E_TMOUT) {
			if (state == BARCODE_STATE_READ) {
				if ((n_widths % 2) == 1) {
					decode();	/* quiet zone reached */
				} else {
					go_idle();	/* black too long */
				}
			}
			continue;
		}
		if (sz != (INT)sizeof(S3_BC_RUN)) {
			CHECK(sz, "barcode tk_rcv_mbf");
			continue;
		}
		if (!enabled) {
			continue;		/* drain and ignore */
		}
		on_run(&run);
	}
}
