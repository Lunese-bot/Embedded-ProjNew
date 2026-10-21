/*
 *	imu_math.c
 *	Subsystem 4 -- integer maths helpers.  See imu_math.h.
 */

#include "imu_math.h"

uint32_t imu_isqrt32(uint32_t v)
{
	uint32_t res = 0;
	uint32_t bit = 1UL << 30;

	while (bit > v) {
		bit >>= 2;
	}
	while (bit != 0) {
		if (v >= res + bit) {
			v  -= res + bit;
			res = (res >> 1) + bit;
		} else {
			res >>= 1;
		}
		bit >>= 2;
	}
	return res;
}

/*
 * atan(z) for |z| <= 1, z in Q10 (1024 = 1.0), result in centi-degrees.
 * Rational approximation atan(z) ~= z / (1 + 0.28086 z^2) radians.
 */
static int32_t atan_q10_cdeg(int32_t z_q10)
{
	int32_t z2    = (z_q10 * z_q10) >> 10;		/* 0..1024 */
	int32_t denom = 1024 + ((z2 * 288) >> 10);	/* 0.28086 * 1024 */

	/* 5730 = 180/pi * 100 */
	return (z_q10 * 5730) / denom;
}

int32_t imu_atan2_cdeg(int32_t y, int32_t x)
{
	int32_t ax;
	int32_t ay;
	int32_t a;

	/* Scale down large inputs so y * 1024 cannot overflow */
	while ((y > 0x1FFFFF) || (y < -0x1FFFFF) ||
	       (x > 0x1FFFFF) || (x < -0x1FFFFF)) {
		x /= 2;
		y /= 2;
	}

	ax = (x < 0) ? -x : x;
	ay = (y < 0) ? -y : y;

	if ((ax == 0) && (ay == 0)) {
		return 0;
	}

	if (ax >= ay) {
		a = atan_q10_cdeg((y * 1024) / ax);	/* -45..45 deg */
		if (x < 0) {
			a = (y >= 0) ? (IMU_CDEG_180 - a) : (-IMU_CDEG_180 - a);
		}
	} else {
		a = atan_q10_cdeg((x * 1024) / ay);
		a = (y > 0) ? (IMU_CDEG_90 - a) : (-IMU_CDEG_90 - a);
	}
	return a;
}

int32_t imu_wrap_cdeg(int32_t a)
{
	while (a >= IMU_CDEG_180) {
		a -= IMU_CDEG_360;
	}
	while (a < -IMU_CDEG_180) {
		a += IMU_CDEG_360;
	}
	return a;
}

uint32_t imu_norm3(int32_t x, int32_t y, int32_t z)
{
	/* 3 * 32767^2 < 2^32 */
	return imu_isqrt32((uint32_t)(x * x) + (uint32_t)(y * y) +
			   (uint32_t)(z * z));
}
