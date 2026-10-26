/*
 *      s2_config.h
 *      Subsystem 2 -- calibration and tuning values (NFR9: configurable).
 *
 *      Private to subsystem 2.  Every value marked MEASURE must be measured
 *      on the real robot and the result recorded in testing/subsystem2/
 *      (PID tuning report, motion accuracy evaluation).
 */

#ifndef S2_CONFIG_H
#define S2_CONFIG_H

/* ---------------------------------------------------------------------
 *  Pins (Design Review section 2; claim them in TEAM.md section 2)
 *
 *  Robo Pico motor driver: M1A/M1B = GP8/GP9, M2A/M2B = GP10/GP11.
 *  GP8/GP9 are PWM slice 4 (A/B), GP10/GP11 are slice 5 (A/B).
 * --------------------------------------------------------------------- */
#define S2_PIN_ML_A 8  /* left motor  M1A */
#define S2_PIN_ML_B 9  /* left motor  M1B */
#define S2_PIN_MR_A 10 /* right motor M2A */
#define S2_PIN_MR_B 11 /* right motor M2B */

/* Encoders on the Grove ports (Design Review: left = Grove 3 GP4/GP5,
   right = Grove 2 GP2/GP3).  The encoder SIGNAL must be on the ODD pin
   of each pair: drv_encoder.c counts edges in hardware with the PWM
   slice's B input, which only exists on odd GPIOs.
     GP5 = PWM slice 2 B,  GP3 = PWM slice 1 B. */
#define S2_PIN_ENC_L 5
#define S2_PIN_ENC_R 3

/* Set to 1 if a wheel turns backwards for a forward command */
#define S2_MOTOR_L_INVERT 1 /* bench 2026-10-15: both wheels ran backwards */
#define S2_MOTOR_R_INVERT 1

/* ---------------------------------------------------------------------
 *  Geometry                                                    MEASURE
 * --------------------------------------------------------------------- */
#define S2_TICKS_PER_REV 20     /* encoder edges per wheel turn */
#define S2_WHEEL_DIAM_MM 65.0f  /* wheel diameter */
#define S2_TRACK_MM      130.0f /* distance between wheel centres */
#define S2_PI            3.14159265f
#define S2_MM_PER_TICK   ((S2_PI * S2_WHEEL_DIAM_MM) / S2_TICKS_PER_REV)

/* ---------------------------------------------------------------------
 *  Motor PWM: 125 MHz / (TOP + 1) = 20 kHz, above hearing range
 * --------------------------------------------------------------------- */
#define S2_PWM_TOP  6249
#define S2_DUTY_MAX 1000 /* duty is in permille, +-1000 */

/* Bytes of kernel header per message inside a message buffer */
#define S2_MBF_MSG_OVERHEAD 8

/* ---------------------------------------------------------------------
 *  Speed control (per wheel PID + feed-forward)               MEASURE
 *  duty = KFF * target + KP * e + KI * sum(e * dt) + KD * de/dt
 *  Start with KI = KD = 0, raise KFF until the wheel nearly reaches the
 *  target on its own, then add KP, then KI to remove the last error.
 * --------------------------------------------------------------------- */
#define S2_PID_KFF       2.0f /* permille per mm/s */
#define S2_PID_KP        1.5f
#define S2_PID_KI        4.0f /* per second */
#define S2_PID_KD        0.0f
#define S2_PID_I_LIMIT   400.0f /* anti-windup clamp, permille */
/* MEASURE: least duty that turns a wheel; below it output is 0 */
#define S2_DUTY_MIN_MOVE 250

/* Speed = distance over the last S2_SPEED_WINDOW periods.
   With 20 ticks per turn one tick is ~10 mm, so a single 10 ms period
   sees 0 or 1 tick: useless as a speed on its own.  A 100 ms window
   still only resolves ~100 mm/s per tick, so the feed-forward term
   (S2_PID_KFF) does most of the work and PID trims the average.
   Better resolution needs more ticks per turn, or timing the gap
   between encoder edges (see SUBSYSTEM2_NOTES.md).           MEASURE */
#define S2_SPEED_WINDOW 10
#define S2_ACCEL_MM_S2  800 /* set-point ramp, mm/s per second */

/* Straight-line correction: steer = K * (left_mm - right_mm) since the
   DRIVE_DIST command started, in mm/s per mm of difference */
#define S2_STRAIGHT_K 3.0f

/* ---------------------------------------------------------------------
 *  Turns, distance and safety
 * --------------------------------------------------------------------- */
#define S2_TURN_SPEED_MM_S 150   /* default pivot wheel speed */
#define S2_TURN_SLOW_MM    15.0f /* slow down this close to target */
#define S2_STOP_SPEED_MM_S 10    /* below this counts as stopped */
#define S2_MIN_SPEED_MM_S  60    /* MEASURE: slowest speed that still moves */
#define S2_DIST_SLOW_MM    50.0f /* DRIVE_DIST: slow down this close */
#define S2_MS_PER_S        1000

/* DRIVE must be refreshed by the Vehicle Controller at least this
   often, or the robot stops (NFR8 fail-safe).  0 disables the check. */
#define S2_CMD_TIMEOUT_MS 500

/* Stall: |duty| above this for S2_STALL_MS with no ticks -> FAULT */
#define S2_STALL_DUTY 600
#define S2_STALL_MS   500

/* Bench demo (no Vehicle Controller needed): 1 = start a task that
   drives forward 300 mm, turns 90 deg right, and repeats.  Lift the
   robot off the table for the first try. */
#ifndef S2_DEMO
#define S2_DEMO 0
#endif

/* Print one status line every N periods (0 = never) */
#define S2_PRINT_EVERY 50 /* 50 x 10 ms = 0.5 s */

#endif /* S2_CONFIG_H */

/*** end of file ***/
