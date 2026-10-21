/*
 *	s5_local.h
 *	Subsystem 5 -- definitions shared ONLY by the files in this folder.
 *	Other subsystems must not include this; they use subsystem5.h.
 */

#ifndef S5_LOCAL_H
#define S5_LOCAL_H

#include "subsystem5.h"
#include "obstacle_logic.h"

/* Bench mode: 1 = test S5 alone, with no Vehicle Controller or motors.
   The scan task starts the avoidance itself and every motion request
   is printed and treated as done after S5_BENCH_MOVE_MS.  Set to 0
   before integrating with the Vehicle Controller. */
#ifndef S5_BENCH_MODE
#define S5_BENCH_MODE		0
#endif
#define S5_BENCH_MOVE_MS	500

/* Monitor straight ahead from power-up (else wait for S5_CMD_ENABLE) */
#define S5_ENABLE_AT_START	1

/* Timing (NFR2) */
#define S5_PERIOD_MS		20	/* monitoring period              */
#define S5_SCAN_TMO_MS		4000	/* whole coarse+fine scan         */
#define S5_TURN_TMO_MS		3000	/* a TURN must finish within      */
#define S5_MOVE_SLACK_MS	2000	/* added to distance/speed        */
#define S5_ECHO_FAULT_COUNT	25	/* silent pings in a row -> fault
					   (only counted while scanning,
					   open space also gives no echo) */

/* flg_s5 bits (private event flag between the two tasks) */
#define S5_FLG_ENABLED		0x0001	/* VC: monitoring allowed         */
#define S5_FLG_SCAN_REQ		0x0002	/* avoid -> scan: profile please  */
#define S5_FLG_PROFILE		0x0004	/* scan -> avoid: profile done    */
#define S5_FLG_RESUME		0x0008	/* avoid -> scan: back to monitor */

#define S5_EVT_BUF_MSGS		8
#define S5_CMD_BUF_MSGS		8

IMPORT ID	flg_s5;
IMPORT ID	mtx_s5;
IMPORT ID	tid_s5_scan;
IMPORT ID	tid_s5_avoid;

/* Status helpers (subsystem5.c), all lock mtx_s5 */
IMPORT void	s5_set_state(S5_STATE st);
IMPORT void	s5_set_fault(S5_FAULT_CODE f);
IMPORT void	s5_note_ping(UH mm);
IMPORT void	s5_count_obstacle(void);
IMPORT void	s5_count_avoid_ok(void);
IMPORT void	s5_store_profile(const S5_PROFILE *p);
IMPORT void	s5_load_profile(S5_PROFILE *p);

/* Send an event to the Vehicle Controller without blocking */
IMPORT ER	s5_send_evt(S5_EVT *e);
IMPORT UW	s5_evt_dropped;		/* events lost: buffer was full */

IMPORT const char *s5_state_name(S5_STATE st);

/* Tasks */
IMPORT void	s5_scan_task(INT stacd, void *exinf);
IMPORT void	s5_avoid_task(INT stacd, void *exinf);

#endif /* S5_LOCAL_H */
