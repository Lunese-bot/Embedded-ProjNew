/*
 *	drv_ir.h
 *	Subsystem 3 -- driver for the three IR line sensors.
 *
 *	Call ONLY from line_task, the owning task of GP06, GP26, GP27 and
 *	(when S3_IR_USE_ADC = 1) the ADC (TEAM.md section 3).
 */

#ifndef DRV_IR_H
#define DRV_IR_H

#include "../app.h"
#include "s3_config.h"

/* Sensors read through the ADC (S3_MASK_x bits), 0 in digital mode */
#if S3_IR_USE_ADC
#define DRV_IR_ANALOG_MASK	(S3_MASK_C | S3_MASK_R)
#else
#define DRV_IR_ANALOG_MASK	0u
#endif

IMPORT ER	drv_ir_init(void);
/* raw[i]: digital 0/1, or ADC 0..4095 for sensors in DRV_IR_ANALOG_MASK */
IMPORT ER	drv_ir_read(UH raw[S3_IR_COUNT]);
/* Free-running 1 MHz RP2040 timer, low 32 bits (wraps every ~71 min) */
IMPORT UW	drv_ir_time_us(void);

#endif /* DRV_IR_H */
