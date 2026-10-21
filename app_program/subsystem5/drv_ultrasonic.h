/*
 *	drv_ultrasonic.h
 *	Subsystem 5 -- HC-SR04 style ultrasonic range finder (TRIG/ECHO).
 *
 *	Call ONLY from the scan task, which owns S5_PIN_TRIG/S5_PIN_ECHO.
 *
 *	Wiring: the HC-SR04 runs at 5 V and its ECHO output is 5 V.  The
 *	RP2040 is NOT 5 V tolerant: put a divider (e.g. 1k + 2k) between
 *	ECHO and GP16.  TRIG can be driven directly from 3.3 V.
 */

#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include "subsystem5.h"

#define US_RANGE_MAX_MM		2000	/* readings beyond are "no echo"   */
#define US_NO_ECHO		0	/* returned distance when silent   */

/* Configure TRIG as output (low) and ECHO as input. */
IMPORT ER	us_init(void);

/* One measurement.  Returns distance in mm, or US_NO_ECHO when nothing
   comes back within the range.  Busy-waits for at most ~12 ms. */
IMPORT UH	us_read_mm(void);

/* Median of n (1..5) readings, us_gap_ms apart; ignores missing ones.
   Returns US_NO_ECHO when every reading was missing. */
IMPORT UH	us_read_median_mm(INT n, INT gap_ms);

/* Free-running 1 MHz hardware timer (RP2040 TIMER), microseconds. */
IMPORT UW	us_now_us(void);

#endif /* DRV_ULTRASONIC_H */
