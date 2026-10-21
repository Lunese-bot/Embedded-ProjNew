/*
 *	subsystem3.h
 *	Subsystem 3 -- IR Line Following and Barcode Navigation.
 *	Owner: Buddy 3 (Student ID 2500193, Design Review).
 *
 *	Public interface of subsystem 3.  Other subsystems include it as
 *	"../subsystem3/subsystem3.h".  Everything here is proposed as
 *	interfaces IF-3x in TEAM.md section 5 (see TEAM_S3_ROWS.md).
 *
 *	What subsystem 3 gives the rest of the robot
 *	--------------------------------------------
 *	  IF-31  line status snapshot  s3_get_line_status()   any task, any time
 *	         (position, tracking error, PD steering request, line state)
 *	  IF-32  event stream          mbf_s3_events          Vehicle Controller
 *	         (line lost/found, junction, barcode start/decoded/failed,
 *	          calibration result)  one S3_EVENT per message
 *	  IF-33  requests              s3_request()           Vehicle Controller
 *	         (calibrate, reset tracker, barcode on/off, re-arm barcode)
 *	  IF-S5d s5_port_line_detected()  (s3_ports.c)   Subsystem 5, after a
 *	         bypass: TRUE when any IR sensor sees the line
 *
 *	Subsystem 3 never drives the motors.  It only REQUESTS steering and
 *	navigation; the Vehicle Controller decides and tells Motion Control
 *	(Design Review 8.3: the controller is the sole movement authority).
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "../robot_types.h"
#include "s3_config.h"

/* ---------------------------------------------------------------------
 *  Tasks (priority band 16-17, TEAM.md section 4)
 * --------------------------------------------------------------------- */
#define S3_PRI_LINE		16	/* line_task:    10 ms, owns IR_L/C/R (+ADC) */
#define S3_PRI_BARCODE		17	/* barcode_task: event driven, decodes      */

/* ---------------------------------------------------------------------
 *  IF-31: line status.  Copy it with s3_get_line_status(); never read
 *  s3_status directly (it is updated every 10 ms under a mutex).
 * --------------------------------------------------------------------- */
typedef struct {
	UW		seq;		/* incremented every line sample	*/
	UW		time_ms;	/* tk_get_tim() of the sample		*/
	LineState	line_state;	/* robot_types.h			*/
	H		position;	/* -1000 left .. +1000 right		*/
	H		error;		/* position - S3_LINE_SETPOINT		*/
	H		steering;	/* PD request -1000..+1000, + = right	*/
	B		last_side;	/* -1 left, +1 right, 0 unknown		*/
	UB		black_mask;	/* S3_MASK_L/C/R on black		*/
	UH		raw[S3_IR_COUNT];	/* raw readings (0/1 or ADC)	*/
	BOOL		calibrated;	/* ADC calibration valid		*/
	BarcodeState	bc_state;	/* robot_types.h			*/
	BOOL		bc_enabled;
	UW		bc_ok;		/* barcodes decoded			*/
	UW		bc_fail;	/* barcode reads that failed		*/
	UW		evt_dropped;	/* events lost: mbf_s3_events was full	*/
} S3_LINE_STATUS;

IMPORT ER s3_get_line_status(S3_LINE_STATUS *p_out);

/* ---------------------------------------------------------------------
 *  IF-32: events, received by the Vehicle Controller:
 *	S3_EVENT evt;
 *	INT sz = tk_rcv_mbf(mbf_s3_events, &evt, TMO_POL);  (or a timeout)
 *	if (sz == sizeof(S3_EVENT)) { switch (evt.type) ... }
 *  Subsystem 3 sends with TMO_POL: if nobody reads, events are dropped
 *  (counted in evt_dropped), the line task never blocks.
 * --------------------------------------------------------------------- */
typedef enum {
	S3_EVT_NONE = 0,
	S3_EVT_LINE_LOST,	/* FOLLOW -> LOST; side = last seen side	*/
	S3_EVT_LINE_FOUND,	/* LOST/SEARCH -> FOLLOW			*/
	S3_EVT_JUNCTION,	/* all junction sensors on black		*/
	S3_EVT_BARCODE_START,	/* first bar seen: slow down (DECODE_BARCODE)	*/
	S3_EVT_BARCODE_DECODED,	/* nav + text valid				*/
	S3_EVT_BARCODE_FAILED,	/* code = code39_result_t, retries in arg	*/
	S3_EVT_CAL_DONE,	/* calibration accepted				*/
	S3_EVT_CAL_FAILED	/* calibration span too small; old cal kept	*/
} S3_EVT_TYPE;

typedef struct {
	UB		type;		/* S3_EVT_TYPE				*/
	UB		nav;		/* NavigationCommand (DECODED only)	*/
	B		side;		/* last_side for LINE_LOST		*/
	UB		code;		/* failure reason (FAILED only)		*/
	UW		arg;		/* retries so far (FAILED) / unused	*/
	UW		time_ms;	/* when it happened			*/
	UB		text[S3_BC_TEXT_MAX];	/* decoded text, '\0' ended	*/
} S3_EVENT;

#define S3_EVT_QUEUE_LEN	8	/* events buffered in mbf_s3_events */

IMPORT ID	mbf_s3_events;

/* ---------------------------------------------------------------------
 *  IF-33: requests to subsystem 3 (any task; returns at once)
 * --------------------------------------------------------------------- */
#define S3_REQ_CALIBRATE	0x0001	/* sample min/max for S3_CAL_DURATION_MS */
#define S3_REQ_RESET_LINE	0x0002	/* tracker -> SEARCH, forget last side	*/
#define S3_REQ_BC_ENABLE	0x0004	/* start reading barcodes		*/
#define S3_REQ_BC_DISABLE	0x0008	/* ignore barcodes (e.g. while avoiding)*/
#define S3_REQ_BC_REARM		0x0010	/* clear retries, wait for next barcode	*/

IMPORT ER s3_request(UINT req);

/* ---------------------------------------------------------------------
 *  Private to subsystem 3 (shared between its own .c files)
 * --------------------------------------------------------------------- */
/* One finished run of the barcode sensor, line_task -> barcode_task */
typedef struct {
	UW	width_us;	/* how long the level lasted		*/
	UB	black;		/* 1 = bar (black), 0 = space (white)	*/
} S3_BC_RUN;

#define S3_FLG_LINE_REQ		(S3_REQ_CALIBRATE | S3_REQ_RESET_LINE)
#define S3_FLG_BC_REQ		(S3_REQ_BC_ENABLE | S3_REQ_BC_DISABLE | \
				 S3_REQ_BC_REARM)

IMPORT ID		mtx_s3_status;
IMPORT ID		mbf_s3_bc_runs;
IMPORT ID		flg_s3_req;
IMPORT ID		tid_s3_line;
IMPORT ID		tid_s3_barcode;
IMPORT ID		cyc_s3_line;
IMPORT S3_LINE_STATUS	s3_status;
IMPORT volatile UB	s3_black_mask_live;	/* line_task -> s3_ports.c,
					   readable without the mutex */
IMPORT volatile BOOL	s3_bc_scanning;	/* barcode_task -> line_task:
					   inhibit junction detection */

IMPORT void	s3_line_cyclic(void *exinf);
IMPORT void	line_task(INT stacd, void *exinf);
IMPORT void	barcode_task(INT stacd, void *exinf);
IMPORT void	s3_send_event(S3_EVENT *p_evt);
IMPORT NavigationCommand s3_text_to_nav(const UB *p_text);

#endif /* SUBSYSTEM3_H */
