/*
 *      imu_math.c
 *      Subsystem 4 -- integer maths helpers.  See imu_math.h.
 */

#include "imu_math.h"

#define Q10_ONE         1024     /* 1.0 in Q10 */
#define ATAN_K_Q10      288      /* 0.28086 * 1024 */
#define CDEG_PER_RAD    5730     /* 180/pi * 100 */
#define ATAN2_INPUT_MAX 0x1FFFFF /* so y * 1024 cannot overflow */

uint32_t imu_isqrt32(uint32_t value)
{
    uint32_t res = 0;
    uint32_t bit = 1UL << 30;

    while (bit > value)
    {
        bit >>= 2;
    }
    while (0U != bit)
    {
        if (value >= (res + bit))
        {
            value -= res + bit;
            res = (res >> 1) + bit;
        }
        else
        {
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
    /* Both products are non-negative, so dividing by Q10_ONE gives the
       same result as the original >> 10 without shifting a signed
       value (BARR-C 5.3). */
    int32_t z_sq  = (z_q10 * z_q10) / Q10_ONE; /* 0..1024 */
    int32_t denom = Q10_ONE + ((z_sq * ATAN_K_Q10) / Q10_ONE);

    return (z_q10 * CDEG_PER_RAD) / denom;
}

int32_t imu_atan2_cdeg(int32_t y_val, int32_t x_val)
{
    int32_t abs_x;
    int32_t abs_y;
    int32_t angle;

    /* Scale down large inputs so y * 1024 cannot overflow */
    while ((y_val > ATAN2_INPUT_MAX) || (y_val < -ATAN2_INPUT_MAX) ||
           (x_val > ATAN2_INPUT_MAX) || (x_val < -ATAN2_INPUT_MAX))
    {
        x_val /= 2;
        y_val /= 2;
    }

    abs_x = (x_val < 0) ? -x_val : x_val;
    abs_y = (y_val < 0) ? -y_val : y_val;

    if ((0 == abs_x) && (0 == abs_y))
    {
        angle = 0;
    }
    else if (abs_x >= abs_y)
    {
        angle = atan_q10_cdeg((y_val * Q10_ONE) / abs_x); /* -45..45 deg */
        if (x_val < 0)
        {
            angle =
                (y_val >= 0) ? (IMU_CDEG_180 - angle) : (-IMU_CDEG_180 - angle);
        }
    }
    else
    {
        angle = atan_q10_cdeg((x_val * Q10_ONE) / abs_y);
        angle = (y_val > 0) ? (IMU_CDEG_90 - angle) : (-IMU_CDEG_90 - angle);
    }
    return angle;
}

int32_t imu_wrap_cdeg(int32_t angle)
{
    while (angle >= IMU_CDEG_180)
    {
        angle -= IMU_CDEG_360;
    }
    while (angle < -IMU_CDEG_180)
    {
        angle += IMU_CDEG_360;
    }
    return angle;
}

uint32_t imu_norm3(int32_t x_val, int32_t y_val, int32_t z_val)
{
    /* 3 * 32767^2 < 2^32.  Cast: each square is non-negative. */
    return imu_isqrt32((uint32_t)(x_val * x_val) + (uint32_t)(y_val * y_val) +
                       (uint32_t)(z_val * z_val));
}

/*** end of file ***/
