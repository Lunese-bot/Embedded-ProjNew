/*
 *	drv_servo.h
 *	Subsystem 5 -- hobby servo (SG90 style) that points the ultrasonic.
 *
 *	Call ONLY from the scan task, which owns PWM slice 6 (GP12/GP13).
 *	Angle convention matches S5_MOTION_REQ: 0 = straight ahead,
 *	positive = LEFT, range -SERVO_MAX_DEG..+SERVO_MAX_DEG.
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include "subsystem5.h"

#define SERVO_MAX_DEG		80	/* mechanical limit we allow       */

/* Calibration (NF9): pulse width at straight ahead and per degree.
   Typical SG90: 1500 us centre, ~11 us/deg.  Measure yours and record
   it in testing/subsystem5/TEST_LOG.md.  SERVO_DIR = -1 if mounted so
   that a larger pulse turns it RIGHT. */
#define SERVO_CENTRE_US		1500
#define SERVO_US_PER_DEG	11
#define SERVO_DIR		1

/* Set up PWM: 1 MHz counter, 20 ms period (50 Hz), servo at centre. */
IMPORT ER	servo_init(void);

/* Point the sensor at deg (clamped).  Returns at once; the caller waits
   for the servo to arrive (servo_settle_ms). */
IMPORT void	servo_set_deg(INT deg);

/* Last commanded angle */
IMPORT INT	servo_get_deg(void);

/* Rough time to travel from the current angle to deg, ms (SG90: about
   2 ms per degree under load, plus a 20 ms floor). */
IMPORT INT	servo_settle_ms(INT from_deg, INT to_deg);

/* Stop driving the servo (no pulses: it goes limp, saves power). */
IMPORT void	servo_off(void);

#endif /* DRV_SERVO_H */
