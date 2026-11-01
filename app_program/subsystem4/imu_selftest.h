/*
 *      imu_selftest.h
 *      Subsystem 4 -- algorithm tests with SYNTHETIC IMU data (pure C).
 *
 *      Run on the Pico by subsystem4_test.c (make TEST=1) and on a PC by
 *      host_test/ (make -C host_test).  No sensor needed: they prove the
 *      maths and the state machine before you trust them on the robot.
 */

#ifndef IMU_SELFTEST_H
#define IMU_SELFTEST_H

#include <stdint.h>

/* atan2 / sqrt helpers within tolerance.  Returns 1 = pass. */
int32_t imu_selftest_math(void);

/* Drive over a trapezoid hump of height_mm (15 deg ramps) at
   150 mm/s.  Writes the measured peak (um) and the number of
   completed humps.  Returns 1 if exactly one hump was reported and the
   peak is within 20 % of height_mm. */
int32_t imu_selftest_hump(int32_t height_mm, int32_t * p_peak_um,
                          int32_t * p_humps);

/* Flat cruising with one 2.5 g spike.  Returns 1 if exactly one impact
   is reported and no hump. */
int32_t imu_selftest_impact(void);

/* Heading turning at +45 deg/s.  Returns 1 if motion = TURNING_LEFT. */
int32_t imu_selftest_turn(void);

/* Robot shaking during calibration.  Returns 1 if calibration is
   rejected (IMU_EV_CAL_FAILED) and the robot stays CALIBRATING. */
int32_t imu_selftest_cal_reject(void);

#endif /* IMU_SELFTEST_H */

/*** end of file ***/
