/*
 *      s5_local.h
 *      Subsystem 5 -- definitions shared ONLY by the files in this folder.
 *      Other subsystems must not include this; they use subsystem5.h.
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
#define S5_BENCH_MODE 1
#endif
#define S5_BENCH_MOVE_MS 500

/* Monitor straight ahead from power-up (else wait for S5_CMD_ENABLE) */
#define S5_ENABLE_AT_START 1

/* Timing (NFR2) */
#define S5_PERIOD_MS        20   /* monitoring period              */

/* Servo motion limits.  A hobby servo told to jump 80 deg swings at full
   speed (~600 deg/s) and can throw the sensor off its mount.  The scan
   task ramps instead: S5_SERVO_STEP_DEG every S5_SERVO_STEP_MS, about
   5 / 25 ms = 200 deg/s, then waits S5_SERVO_SETTLE_MS before measuring.
   Lower the speed (bigger STEP_MS) if the mount still shakes. */
#define S5_SERVO_STEP_DEG   5    /* degrees per small move         */
#define S5_SERVO_STEP_MS    25   /* pause between small moves      */
#define S5_SERVO_SETTLE_MS  60   /* after a long move: settle, then ping */
#define S5_SERVO_SETTLE_SHORT_MS 30 /* after one small step (fine scan)  */
#define S5_SCAN_TMO_MS      8000 /* whole coarse+fine scan.  Worst case
                                    with gentle servo ramps ~5.2 s (wide
                                    obstacle, 25 fine angles); was 4000,
                                    which a wide scan could overrun */
#define S5_TURN_TMO_MS      3000 /* a TURN must finish within      */
#define S5_MOVE_SLACK_MS    2000 /* added to distance/speed        */
/* Silent pings in a row -> fault (only counted while scanning, open
   space also gives no echo) */
#define S5_ECHO_FAULT_COUNT 25

/* g_flg_s5 bits (private event flag between the two tasks) */
#define S5_FLG_ENABLED  0x0001 /* VC: monitoring allowed         */
#define S5_FLG_SCAN_REQ 0x0002 /* avoid -> scan: profile please  */
#define S5_FLG_PROFILE  0x0004 /* scan -> avoid: profile done    */
#define S5_FLG_RESUME   0x0008 /* avoid -> scan: back to monitor */

#define S5_EVT_BUF_MSGS 8
#define S5_CMD_BUF_MSGS 8
#define S5_MBF_MSG_OVERHEAD 4 /* kernel header bytes per message */

extern ID g_flg_s5;
extern ID g_mtx_s5;
extern ID g_tid_s5_scan;
extern ID g_tid_s5_avoid;

/* Status helpers (subsystem5.c), all lock g_mtx_s5 */
extern void s5_set_state(s5_state_t state);
extern void s5_set_fault(s5_fault_code_t fault);
extern void s5_note_ping(uint16_t dist_mm);
extern void s5_count_obstacle(void);
extern void s5_count_avoid_ok(void);
extern void s5_store_profile(const s5_profile_t * p_profile);
extern void s5_load_profile(s5_profile_t * p_profile);

/* Send an event to the Vehicle Controller without blocking */
extern ER       s5_send_evt(s5_evt_t * p_evt);
extern uint32_t g_s5_evt_dropped; /* events lost: buffer was full */

/* Safety-critical send (Tribunal 1, Version C: "latched stop path").
   Sends p_evt to mbf like s5_send_evt().  If the buffer is full the
   event is lost, so the Vehicle Controller would never stop the car:
   S5 then calls the emergency stop itself (s2_emergency_stop(), latched
   until MOTION_RESET) so the robot cannot drive into the obstacle.
   This is the ONLY place S5 touches motion, and it can only STOP.
   Returns the send result (E_OK, or the error after the stop). */
extern ER s5_send_critical_to(ID mbf, s5_evt_t * p_evt);
#define s5_send_critical(p_evt) s5_send_critical_to(g_mbf_s5_evt, (p_evt))

/* The stop function used by s5_send_critical_to().  Defaults to
   s2_emergency_stop; the on-target test swaps in a counter so it can
   check the path without latching the real motors. */
typedef ER (*s5_stop_fn_t)(void);
extern s5_stop_fn_t g_s5_stop_fn;
extern uint32_t     g_s5_estops; /* emergency stops S5 has requested */

extern const char * s5_state_name(s5_state_t state);

/* Tasks */
extern void s5_scan_task(int32_t stacd, void * p_exinf);
extern void s5_avoid_task(int32_t stacd, void * p_exinf);

#endif /* S5_LOCAL_H */

/*** end of file ***/
