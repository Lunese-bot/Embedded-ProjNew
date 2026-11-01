/*
 *      drv_motor.h
 *      Subsystem 2 -- Robo Pico dual DC motor driver (2 x H-bridge).
 *
 *      Called ONLY from motion_task (the owning task, TEAM.md section 3).
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "../app.h"

typedef enum
{
    MOTOR_LEFT  = 0,
    MOTOR_RIGHT = 1,
    MOTOR_COUNT
} motor_id_t;

/* Set up both motors' PWM pins (20 kHz) and leave the motors coasting. */
extern ER drv_motor_init(void);

/* duty: -S2_DUTY_MAX .. +S2_DUTY_MAX permille.  + = forward.
   Values outside the range are clamped.  0 = coast. */
extern void drv_motor_set(motor_id_t motor, int32_t duty);

/* Both inputs high: active brake (wheel resists turning). */
extern void drv_motor_brake(motor_id_t motor);

/* Both motors coast at once; used by Safe Stop. */
extern void drv_motor_all_off(void);

#endif /* DRV_MOTOR_H */

/*** end of file ***/
