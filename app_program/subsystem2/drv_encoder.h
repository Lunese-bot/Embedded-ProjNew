/*
 *	drv_encoder.h
 *	Subsystem 2 -- wheel encoders, counted in hardware.
 *
 *	Called ONLY from motion_task (the owning task, TEAM.md section 3).
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include "../app.h"
#include "drv_motor.h"		/* MOTOR_ID, MOTOR_LEFT / MOTOR_RIGHT */

/* Set both encoder pins up as hardware edge counters.
   E_PAR if a pin is even (see s2_config.h). */
IMPORT ER	drv_encoder_init(void);

/* Edges counted since the previous call for this wheel (0..65535).
   The counter is 16-bit and free running; calling at least every
   ~1 s (we call every 10 ms) means it can never wrap twice unseen. */
IMPORT UW	drv_encoder_delta(MOTOR_ID m);

#endif /* DRV_ENCODER_H */
