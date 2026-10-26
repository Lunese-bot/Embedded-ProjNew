/*
 *      subsystem2.h
 *      Subsystem 2 -- Motion Control and Wheel Encoders (Buddy 2).
 *      Student ID: 2503000.
 *
 *      Public interface of subsystem 2.  Other subsystems include it as
 *      "../subsystem2/subsystem2.h".
 *
 *      What this subsystem does (Design Review FR3, NFR2, NFR8):
 *        - owns BOTH motors (PWM) and BOTH wheel encoders: no other task
 *          touches GP8-GP11 or the encoder pins (TEAM.md section 3)
 *        - runs a 10 ms control loop: per-wheel PID speed control,
 *          straight-line correction, distance and encoder-based turns
 *        - publishes speed, distance and heading for the Vehicle
 *          Controller and telemetry (Buddy 1)
 *        - has a latched Safe Stop that only an explicit reset clears
 *
 *      Who may command it: ONLY the Vehicle Controller (Design Review 8.3,
 *      "sole authority for movement commands").  Everyone else may READ
 *      the status.  The s2_cmd_xxx() helpers below run in the CALLER's
 *      task; they only queue a message, they never touch hardware.
 *
 *      Interfaces (add these rows to TEAM.md section 5):
 *        IF-M1  VC -> S2   message buffer g_mbf_motion_cmd   motion_cmd_t
 *        IF-M2  S2 -> any  shared data + mutex g_mtx_motion  motion_status_t,
 *                          read only through s2_get_status()
 *        IF-M3  S2 -> VC   event flag g_flg_motion           MFLG_xxx bits
 *        IF-S4-3 S2 -> S4  s4_set_speed_mm_s(), called every 10 ms with
 *                          the measured vehicle speed (owned by S4)
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

/* ---------------------------------------------------------------------
 *  Task, priority and timing (TEAM.md section 4: band 14-15)
 * --------------------------------------------------------------------- */
#define S2_PRI_MOTION 14 /* motion control task */
#define S2_PRI_DEMO   15 /* optional bench demo task (S2_DEMO) */
#define S2_PERIOD_MS  10 /* control loop period (NFR2: 10 ms) */

/* ---------------------------------------------------------------------
 *  IF-M1: motion commands, Vehicle Controller -> subsystem 2.
 *  Sent by COPY through a message buffer, so the sender needs no memory
 *  pool and may reuse its variable at once.  The control loop reads
 *  every queued command each period; the newest one wins, except that
 *  MOTION_SAFE_STOP latches until MOTION_RESET.
 * --------------------------------------------------------------------- */
typedef enum
{
    MOTION_STOP = 0,   /* ramp to zero speed, then hold (brake)    */
    MOTION_DRIVE,      /* run at speed_mm_s with steer_mm_s until
                          the next command (line following)        */
    MOTION_DRIVE_DIST, /* go straight distance_mm, then stop and
                          set MFLG_CMD_DONE                         */
    MOTION_TURN,       /* pivot on the spot by angle_deg (+ = left,
                          - = right), then stop, MFLG_CMD_DONE      */
    MOTION_SAFE_STOP,  /* NFR8: motors off at once, latched        */
    MOTION_RESET       /* leave SAFE_STOP / FAULT, back to IDLE    */
} motion_cmd_type_t;

typedef struct
{
    uint32_t seq;         /* sender's sequence number, echoed in status */
    int32_t  type;        /* motion_cmd_type_t */
    int32_t  speed_mm_s;  /* signed forward speed, |v| <= S2_MAX_SPEED  */
    int32_t  steer_mm_s;  /* DRIVE only: + = turn left (right wheel
                             faster).  left = v - steer, right = v + steer */
    int32_t  distance_mm; /* DRIVE_DIST: > 0 forward, < 0 backward      */
    int32_t  angle_deg;   /* TURN: + = counter-clockwise (left), - = right.
                             Same convention as s5_motion_req_t.          */
} motion_cmd_t;

#define S2_MAX_SPEED_MM_S 400 /* commands are clamped to this */
#define S2_CMD_QUEUE_LEN  8   /* messages the buffer can hold */

extern ID g_mbf_motion_cmd; /* IF-M1, owned by S2 */

/* Helpers: fill a motion_cmd_t and send it without waiting (TMO_POL).
   Return E_OK, or E_TMOUT if the queue is full.  Call from TASKS only. */
extern ER s2_cmd_stop(void);
extern ER s2_cmd_drive(int32_t speed_mm_s, int32_t steer_mm_s);
extern ER s2_cmd_drive_dist(int32_t speed_mm_s, int32_t distance_mm);
extern ER s2_cmd_turn(int32_t speed_mm_s, int32_t angle_deg);
extern ER s2_cmd_reset(void);
extern ER s2_send_cmd(motion_cmd_t * p_cmd); /* sets cmd->seq */

/* Emergency stop: may be called from any task.  Sets MFLG_ESTOP_REQ,
   which the control loop checks before anything else, AND queues a
   MOTION_SAFE_STOP.  The motors stop within one period (10 ms). */
extern ER s2_emergency_stop(void);

/* ---------------------------------------------------------------------
 *  IF-M2: motion status, subsystem 2 -> anyone (VC, telemetry, IMU...)
 * --------------------------------------------------------------------- */
typedef enum
{
    MSTATE_IDLE = 0, /* stopped, waiting for a command */
    MSTATE_DRIVING,
    MSTATE_DRIVING_DIST,
    MSTATE_TURNING,
    MSTATE_STOPPING,  /* ramping down after MOTION_STOP */
    MSTATE_SAFE_STOP, /* latched, motors off */
    MSTATE_FAULT      /* latched, motors off, see fault */
} motion_state_t;

typedef enum
{
    MFAULT_NONE = 0,
    MFAULT_STALL_LEFT, /* drive applied but no encoder ticks */
    MFAULT_STALL_RIGHT,
    MFAULT_DRIVER /* motor/encoder driver init failed */
} motion_fault_t;

typedef struct
{
    int32_t  state;         /* motion_state_t */
    int32_t  fault;         /* motion_fault_t, valid in MSTATE_FAULT */
    uint32_t cmd_seq;       /* seq of the command being executed */
    int32_t  target_l_mm_s; /* wheel speed set-points */
    int32_t  target_r_mm_s;
    int32_t  speed_l_mm_s; /* measured wheel speeds (100 ms window) */
    int32_t  speed_r_mm_s;
    int32_t  speed_mm_s; /* vehicle speed = (l + r) / 2 */
    int32_t  odo_l_mm;   /* wheel odometry since boot, signed */
    int32_t  odo_r_mm;
    int32_t  distance_mm;  /* vehicle distance since boot, signed */
    int32_t  heading_ddeg; /* encoder heading, 0.1 deg, + = left (CCW) */
    int32_t  duty_l;       /* motor output, -1000..+1000 permille */
    int32_t  duty_r;
    uint32_t ticks_l; /* raw encoder edge counts since boot */
    uint32_t ticks_r;
    uint32_t loop_count; /* control periods run */
    uint32_t overruns;   /* periods missed (loop took > 10 ms) */
    uint32_t time_ms;    /* tk_get_tim() when this snapshot was made */
} motion_status_t;

extern ID g_mtx_motion; /* IF-M2, owned by S2 */

/* Copy the latest status under the mutex.  E_OK, or the mutex error. */
extern ER s2_get_status(motion_status_t * p_out);

/* ---------------------------------------------------------------------
 *  IF-M3: motion events, subsystem 2 -> Vehicle Controller.
 *  Wait with tk_wai_flg(g_flg_motion, bits, TWF_ORW | TWF_BITCLR, ...),
 *  so each event is consumed by the task that handles it.
 * --------------------------------------------------------------------- */
#define MFLG_CMD_DONE     (1U << 0) /* DRIVE_DIST / TURN finished */
#define MFLG_STOPPED      (1U << 1) /* MOTION_STOP reached 0 speed */
#define MFLG_SAFE_STOPPED (1U << 2) /* entered MSTATE_SAFE_STOP */
#define MFLG_FAULT        (1U << 3) /* entered MSTATE_FAULT */
#define MFLG_CMD_TIMEOUT  (1U << 4) /* DRIVE had no fresh command */
#define MFLG_ESTOP_REQ    (1U << 8) /* internal: s2_emergency_stop() */

extern ID g_flg_motion; /* IF-M3, owned by S2 */

/* ---------------------------------------------------------------------
 *  Example interface kept for the build (see subsystem2.c).
 *  Subsystems 3 and 4 still contain the course's example code, which
 *  uses these.  Set S2_KEEP_EXAMPLE_IF to 0 once they replace it.
 * --------------------------------------------------------------------- */
#ifndef S2_KEEP_EXAMPLE_IF
#define S2_KEEP_EXAMPLE_IF 0
#endif

#if S2_KEEP_EXAMPLE_IF
typedef struct
{
    T_MSG    hdr;
    uint32_t seq;
    uint32_t time_ms;
    int32_t  value;
    int32_t  prc;
} sample_msg_t;
#define S2_POOL_COUNT 4
extern ID g_sem_slots;
extern ID g_mpf_samples;
#endif

/* ---------------------------------------------------------------------
 *  Private to subsystem 2
 * --------------------------------------------------------------------- */
extern ID   g_tid_motion;
extern ID   g_cyc_motion;
extern void motion_task(int32_t stacd, void * p_exinf);
extern void motion_cyclic_isr(void * p_exinf);

#endif /* SUBSYSTEM2_H */

/*** end of file ***/
