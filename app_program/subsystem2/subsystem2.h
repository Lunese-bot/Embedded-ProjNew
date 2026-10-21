/*
 *	subsystem2.h
 *	Subsystem 2 -- Motion Control and Wheel Encoders (Buddy 2).
 *	Student ID: 2503000.
 *
 *	Public interface of subsystem 2.  Other subsystems include it as
 *	"../subsystem2/subsystem2.h".
 *
 *	What this subsystem does (Design Review FR3, NFR2, NFR8):
 *	  - owns BOTH motors (PWM) and BOTH wheel encoders: no other task
 *	    touches GP8-GP11 or the encoder pins (TEAM.md section 3)
 *	  - runs a 10 ms control loop: per-wheel PID speed control,
 *	    straight-line correction, distance and encoder-based turns
 *	  - publishes speed, distance and heading for the Vehicle
 *	    Controller and telemetry (Buddy 1)
 *	  - has a latched Safe Stop that only an explicit reset clears
 *
 *	Who may command it: ONLY the Vehicle Controller (Design Review 8.3,
 *	"sole authority for movement commands").  Everyone else may READ
 *	the status.  The s2_cmd_xxx() helpers below run in the CALLER's
 *	task; they only queue a message, they never touch hardware.
 *
 *	Interfaces (add these rows to TEAM.md section 5):
 *	  IF-M1  VC -> S2   message buffer mbf_motion_cmd   MOTION_CMD
 *	  IF-M2  S2 -> any  shared data + mutex mtx_motion  MOTION_STATUS,
 *	                    read only through s2_get_status()
 *	  IF-M3  S2 -> VC   event flag flg_motion           MFLG_xxx bits
 *	  IF-S4-3 S2 -> S4  s4_set_speed_mm_s(), called every 10 ms with
 *	                    the measured vehicle speed (owned by S4)
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

/* ---------------------------------------------------------------------
 *  Task, priority and timing (TEAM.md section 4: band 14-15)
 * --------------------------------------------------------------------- */
#define S2_PRI_MOTION		14	/* motion control task */
#define S2_PRI_DEMO		15	/* optional bench demo task (S2_DEMO) */
#define S2_PERIOD_MS		10	/* control loop period (NFR2: 10 ms) */

/* ---------------------------------------------------------------------
 *  IF-M1: motion commands, Vehicle Controller -> subsystem 2.
 *  Sent by COPY through a message buffer, so the sender needs no memory
 *  pool and may reuse its variable at once.  The control loop reads
 *  every queued command each period; the newest one wins, except that
 *  MOTION_SAFE_STOP latches until MOTION_RESET.
 * --------------------------------------------------------------------- */
typedef enum {
	MOTION_STOP = 0,	/* ramp to zero speed, then hold (brake)    */
	MOTION_DRIVE,		/* run at speed_mm_s with steer_mm_s until
				   the next command (line following)        */
	MOTION_DRIVE_DIST,	/* go straight distance_mm, then stop and
				   set MFLG_CMD_DONE                         */
	MOTION_TURN,		/* pivot on the spot by angle_deg (+ = left,
				   - = right), then stop, MFLG_CMD_DONE      */
	MOTION_SAFE_STOP,	/* NFR8: motors off at once, latched        */
	MOTION_RESET		/* leave SAFE_STOP / FAULT, back to IDLE    */
} MOTION_CMD_TYPE;

typedef struct {
	UW	seq;		/* sender's sequence number, echoed in status */
	INT	type;		/* MOTION_CMD_TYPE */
	INT	speed_mm_s;	/* signed forward speed, |v| <= S2_MAX_SPEED  */
	INT	steer_mm_s;	/* DRIVE only: + = turn left (right wheel
				   faster).  left = v - steer, right = v + steer */
	INT	distance_mm;	/* DRIVE_DIST: > 0 forward, < 0 backward      */
	INT	angle_deg;	/* TURN: + = counter-clockwise (left), - = right.
				   Same convention as S5_MOTION_REQ.          */
} MOTION_CMD;

#define S2_MAX_SPEED_MM_S	400	/* commands are clamped to this */
#define S2_CMD_QUEUE_LEN	8	/* messages the buffer can hold */

IMPORT ID	mbf_motion_cmd;		/* IF-M1, owned by S2 */

/* Helpers: fill a MOTION_CMD and send it without waiting (TMO_POL).
   Return E_OK, or E_TMOUT if the queue is full.  Call from TASKS only. */
IMPORT ER	s2_cmd_stop(void);
IMPORT ER	s2_cmd_drive(INT speed_mm_s, INT steer_mm_s);
IMPORT ER	s2_cmd_drive_dist(INT speed_mm_s, INT distance_mm);
IMPORT ER	s2_cmd_turn(INT speed_mm_s, INT angle_deg);
IMPORT ER	s2_cmd_reset(void);
IMPORT ER	s2_send_cmd(MOTION_CMD *cmd);	/* sets cmd->seq */

/* Emergency stop: may be called from any task.  Sets MFLG_ESTOP_REQ,
   which the control loop checks before anything else, AND queues a
   MOTION_SAFE_STOP.  The motors stop within one period (10 ms). */
IMPORT ER	s2_emergency_stop(void);

/* ---------------------------------------------------------------------
 *  IF-M2: motion status, subsystem 2 -> anyone (VC, telemetry, IMU...)
 * --------------------------------------------------------------------- */
typedef enum {
	MSTATE_IDLE = 0,	/* stopped, waiting for a command */
	MSTATE_DRIVING,
	MSTATE_DRIVING_DIST,
	MSTATE_TURNING,
	MSTATE_STOPPING,	/* ramping down after MOTION_STOP */
	MSTATE_SAFE_STOP,	/* latched, motors off */
	MSTATE_FAULT		/* latched, motors off, see fault */
} MOTION_STATE;

typedef enum {
	MFAULT_NONE = 0,
	MFAULT_STALL_LEFT,	/* drive applied but no encoder ticks */
	MFAULT_STALL_RIGHT,
	MFAULT_DRIVER		/* motor/encoder driver init failed */
} MOTION_FAULT;

typedef struct {
	INT	state;		/* MOTION_STATE */
	INT	fault;		/* MOTION_FAULT, valid in MSTATE_FAULT */
	UW	cmd_seq;	/* seq of the command being executed */
	INT	target_l_mm_s;	/* wheel speed set-points */
	INT	target_r_mm_s;
	INT	speed_l_mm_s;	/* measured wheel speeds (100 ms window) */
	INT	speed_r_mm_s;
	INT	speed_mm_s;	/* vehicle speed = (l + r) / 2 */
	W	odo_l_mm;	/* wheel odometry since boot, signed */
	W	odo_r_mm;
	W	distance_mm;	/* vehicle distance since boot, signed */
	INT	heading_ddeg;	/* encoder heading, 0.1 deg, + = left (CCW) */
	INT	duty_l;		/* motor output, -1000..+1000 permille */
	INT	duty_r;
	UW	ticks_l;	/* raw encoder edge counts since boot */
	UW	ticks_r;
	UW	loop_count;	/* control periods run */
	UW	overruns;	/* periods missed (loop took > 10 ms) */
	UW	time_ms;	/* tk_get_tim() when this snapshot was made */
} MOTION_STATUS;

IMPORT ID	mtx_motion;		/* IF-M2, owned by S2 */

/* Copy the latest status under the mutex.  E_OK, or the mutex error. */
IMPORT ER	s2_get_status(MOTION_STATUS *out);

/* ---------------------------------------------------------------------
 *  IF-M3: motion events, subsystem 2 -> Vehicle Controller.
 *  Wait with tk_wai_flg(flg_motion, bits, TWF_ORW | TWF_BITCLR, ...),
 *  so each event is consumed by the task that handles it.
 * --------------------------------------------------------------------- */
#define MFLG_CMD_DONE		(1U << 0)	/* DRIVE_DIST / TURN finished */
#define MFLG_STOPPED		(1U << 1)	/* MOTION_STOP reached 0 speed */
#define MFLG_SAFE_STOPPED	(1U << 2)	/* entered MSTATE_SAFE_STOP */
#define MFLG_FAULT		(1U << 3)	/* entered MSTATE_FAULT */
#define MFLG_CMD_TIMEOUT	(1U << 4)	/* DRIVE had no fresh command */
#define MFLG_ESTOP_REQ		(1U << 8)	/* internal: s2_emergency_stop() */

IMPORT ID	flg_motion;		/* IF-M3, owned by S2 */

/* ---------------------------------------------------------------------
 *  Example interface kept for the build (see subsystem2.c).
 *  Subsystems 3 and 4 still contain the course's example code, which
 *  uses these.  Set S2_KEEP_EXAMPLE_IF to 0 once they replace it.
 * --------------------------------------------------------------------- */
#ifndef S2_KEEP_EXAMPLE_IF
#define S2_KEEP_EXAMPLE_IF	0
#endif

#if S2_KEEP_EXAMPLE_IF
typedef struct {
	T_MSG	hdr;
	UW	seq;
	UW	time_ms;
	INT	value;
	INT	prc;
} SAMPLE_MSG;
#define S2_POOL_COUNT	4
IMPORT ID	sem_slots;
IMPORT ID	mpf_samples;
#endif

/* ---------------------------------------------------------------------
 *  Private to subsystem 2
 * --------------------------------------------------------------------- */
IMPORT ID	tid_motion;
IMPORT ID	cyc_motion;
IMPORT void	motion_task(INT stacd, void *exinf);
IMPORT void	motion_cyclic_handler(void *exinf);

#endif /* SUBSYSTEM2_H */
