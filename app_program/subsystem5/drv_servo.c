/*
 *      drv_servo.c
 *      Subsystem 5 -- servo driver on the RP2040 PWM block.
 *
 *      The BSP (libbsp_pwm.c) has no clock-divider call, so the divider is
 *      written straight to the slice's DIV register: 125 MHz / 125 = 1 MHz,
 *      so one counter tick = 1 us and the compare value is the pulse width
 *      in microseconds.  wrap = 19999 -> 20 ms period.
 *
 *      GP12 is slice 6 channel A.  GP13 (channel B) shares the slice, so
 *      if anybody else wants GP13 as PWM it must also run at 50 Hz.
 */

#include "drv_servo.h"

#define SERVO_SLICE                                                    \
    (((uint32_t)S5_PIN_SERVO >> 1U) & 0x07U)           /* 2 pins/slice \
                                                        */
#define SERVO_DIV_INT      (CLK_PERI_FREQ / 1000000UL) /* 125 */
#define SERVO_WRAP         19999                       /* 20 ms at 1 MHz */
#define SERVO_MIN_US       500
#define SERVO_MAX_US       2500
#define SERVO_SLICE_STRIDE 0x14U /* bytes between PWM slice registers */
#define SERVO_DIV_INT_MASK 0xFFU /* DIV integer part, bits 11:4 */
#define SERVO_DIV_INT_LSB  4U
#define SERVO_SETTLE_MS    20 /* settle time for any move ...      */
#define SERVO_MS_PER_DEG   2  /* ... plus this per degree of travel */

static int32_t g_servo_deg_now;

static uint32_t deg_to_us(int32_t deg)
{
    int32_t pulse_us = SERVO_CENTRE_US + SERVO_DIR * deg * SERVO_US_PER_DEG;

    if (pulse_us < SERVO_MIN_US)
    {
        pulse_us = SERVO_MIN_US;
    }
    if (pulse_us > SERVO_MAX_US)
    {
        pulse_us = SERVO_MAX_US;
    }
    return (uint32_t)pulse_us;
}

ER servo_init(void)
{
    ER ercd;

    ercd = pwm_set_pin(S5_PIN_SERVO);
    if (ercd < E_OK)
    {
        return ercd;
    }
    /* DIV register: integer part in bits 11:4, fraction in bits 3:0 */
    out_w(PWM_BASE + PWM_CHx_DIV + (SERVO_SLICE * SERVO_SLICE_STRIDE),
          (SERVO_DIV_INT & SERVO_DIV_INT_MASK) << SERVO_DIV_INT_LSB);
    pwm_set_wrap(S5_PIN_SERVO, SERVO_WRAP);
    g_servo_deg_now = 0;
    pwm_set_cc(S5_PIN_SERVO, deg_to_us(0));
    return pwm_set_enabled(S5_PIN_SERVO, true);
}

void servo_set_deg(int32_t deg)
{
    if (deg > SERVO_MAX_DEG)
    {
        deg = SERVO_MAX_DEG;
    }
    if (deg < -SERVO_MAX_DEG)
    {
        deg = -SERVO_MAX_DEG;
    }
    g_servo_deg_now = deg;
    pwm_set_cc(S5_PIN_SERVO, deg_to_us(deg));
}

int32_t servo_get_deg(void)
{
    return g_servo_deg_now;
}

int32_t servo_settle_ms(int32_t from_deg, int32_t to_deg)
{
    int32_t diff = to_deg - from_deg;

    if (diff < 0)
    {
        diff = -diff;
    }
    return SERVO_SETTLE_MS + (diff * SERVO_MS_PER_DEG);
}

void servo_off(void)
{
    pwm_set_cc(S5_PIN_SERVO, 0); /* 0 % duty: no pulses */
}

/*** end of file ***/
