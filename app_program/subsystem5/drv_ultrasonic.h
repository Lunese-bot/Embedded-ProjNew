/*
 *      drv_ultrasonic.h
 *      Subsystem 5 -- HC-SR04 style ultrasonic range finder (TRIG/ECHO).
 *
 *      Call ONLY from the scan task, which owns S5_PIN_TRIG/S5_PIN_ECHO.
 *
 *      Wiring: the HC-SR04 runs at 5 V and its ECHO output is 5 V.  The
 *      RP2040 is NOT 5 V tolerant: put a divider (e.g. 1k + 2k) between
 *      ECHO and GP16.  TRIG can be driven directly from 3.3 V.
 */

#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include "subsystem5.h"

#define US_RANGE_MAX_MM 2000 /* readings beyond are "no echo"   */
#define US_NO_ECHO      0    /* returned distance when silent   */
/* 1 = pause task switching on this core while an echo is timed (accurate,
   but a trial of it with the integrated build stopped the pings).  0 = off. */
#ifndef S5_PING_NO_PREEMPT
#define S5_PING_NO_PREEMPT 0
#endif
#define US_MIN_MM       30   /* closer than this is not a real echo (the
                                sensor minimum is about 20 mm)          */

/* Raw numbers of the last ping (diagnostics): ECHO level before the trigger,
   microseconds from the trigger to the ECHO rising edge, ECHO high time. */
extern void us_last_raw(uint32_t * p_idle, uint32_t * p_rise_us,
                        uint32_t * p_width_us);

/* Pings thrown away (and measured again) because another task or an interrupt
   ran in the middle of the echo timing.  Diagnostics. */
extern uint32_t us_disturbed_pings(void);

/* ECHO pulses shorter than 150 us that were skipped as not being an echo
   (trigger leaking onto ECHO, receiver hearing the burst).  Diagnostics. */
extern uint32_t us_glitch_pulses(void);

/* TRUE when the sensor answered the last ping (an echo, or ECHO high
   until the end of the range).  FALSE = no answer at all or ECHO stuck
   high: sensor unplugged, unpowered or a broken wire. */
extern bool us_last_answered(void);

/* Configure TRIG as output (low) and ECHO as input. */
extern ER us_init(void);

/* One measurement over the full range (2000 mm).  Returns distance in mm,
   or US_NO_ECHO when nothing comes back.  Busy-waits for at most ~12 ms
   with dispatching disabled on this core. */
extern uint16_t us_read_mm(void);

/* Same, but gives up beyond max_mm (shorter busy-wait: ~6 ms for
   1000 mm).  Used for monitoring while the robot drives. */
extern uint16_t us_read_mm_max(uint16_t max_mm);

/* Median of n (1..5) readings, us_gap_ms apart; ignores missing ones.
   Returns US_NO_ECHO when every reading was missing. */
extern uint16_t us_read_median_mm(int32_t samples, int32_t gap_ms);

/* Same, but each reading gives up beyond max_mm (shorter busy-wait, so
   fewer pings are disturbed by other tasks).  Used by the scan. */
extern uint16_t us_read_median_mm_max(int32_t samples, int32_t gap_ms,
                                      uint16_t max_mm);

/* Free-running 1 MHz hardware timer (RP2040 TIMER), microseconds. */
extern uint32_t us_now_us(void);

#endif /* DRV_ULTRASONIC_H */

/*** end of file ***/
