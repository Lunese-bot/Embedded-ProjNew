/*
 *	drv_motor.h
 *	Subsystem 2 -- Robo Pico dual DC motor driver (2 x H-bridge).
 *
 *	Called ONLY from motion_task (the owning task, TEAM.md section 3).
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "../app.h"

typedef enum {
	MOTOR_LEFT = 0,
	MOTOR_RIGHT = 1,
	MOTOR_COUNT
} MOTOR_ID;

/* Set up both motors' PWM pins (20 kHz) and leave the motors coasting. */
IMPORT ER	drv_motor_init(void);

/* duty: -S2_DUTY_MAX .. +S2_DUTY_MAX permille.  + = forward.
   Values outside the range are clamped.  0 = coast. */
IMPORT void	drv_motor_set(MOTOR_ID m, INT duty);

/* Both inputs high: active brake (wheel resists turning). */
IMPORT void	drv_motor_brake(MOTOR_ID m);

/* Both motors coast at once; used by Safe Stop. */
IMPORT void	drv_motor_all_off(void);

#endif /* DRV_MOTOR_H */
