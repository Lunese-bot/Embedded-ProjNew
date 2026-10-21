/*
 *	imu_math.h
 *	Subsystem 4 -- integer maths helpers (no libm, no floating point).
 *
 *	The build links no maths library and the RP2040 has no FPU, so all
 *	angles are integers in CENTI-DEGREES (1/100 degree): 9000 = 90.00 deg.
 *
 *	Pure C: no kernel calls, so it also compiles on a PC for host tests.
 */

#ifndef IMU_MATH_H
#define IMU_MATH_H

#include <stdint.h>

#define IMU_CDEG_90	9000
#define IMU_CDEG_180	18000
#define IMU_CDEG_360	36000

/* Integer square root, floor(sqrt(v)). */
uint32_t imu_isqrt32(uint32_t v);

/* atan2(y, x) in centi-degrees, range -18000..18000.
   Max error about 0.3 deg, good enough for tilt and heading. */
int32_t  imu_atan2_cdeg(int32_t y, int32_t x);

/* Wrap an angle difference into -18000..17999. */
int32_t  imu_wrap_cdeg(int32_t a);

/* Magnitude of a 3-vector of milli-g values (each |v| < 32768). */
uint32_t imu_norm3(int32_t x, int32_t y, int32_t z);

#endif /* IMU_MATH_H */
