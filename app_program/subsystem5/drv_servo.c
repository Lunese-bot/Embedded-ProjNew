/*
 *	drv_servo.c
 *	Subsystem 5 -- servo driver on the RP2040 PWM block.
 *
 *	The BSP (libbsp_pwm.c) has no clock-divider call, so the divider is
 *	written straight to the slice's DIV register: 125 MHz / 125 = 1 MHz,
 *	so one counter tick = 1 us and the compare value is the pulse width
 *	in microseconds.  wrap = 19999 -> 20 ms period.
 *
 *	GP12 is slice 6 channel A.  GP13 (channel B) shares the slice, so
 *	if anybody else wants GP13 as PWM it must also run at 50 Hz.
 */

#include "drv_servo.h"

#define SERVO_SLICE		((S5_PIN_SERVO >> 1) & 0x07)
#define SERVO_DIV_INT		(CLK_PERI_FREQ / 1000000UL)	/* 125 */
#define SERVO_WRAP		19999		/* 20 ms at 1 MHz */
#define SERVO_MIN_US		500
#define SERVO_MAX_US		2500

LOCAL INT	servo_deg_now;

LOCAL UW deg_to_us(INT deg)
{
	INT us = SERVO_CENTRE_US + SERVO_DIR * deg * SERVO_US_PER_DEG;

	if (us < SERVO_MIN_US) {
		us = SERVO_MIN_US;
	}
	if (us > SERVO_MAX_US) {
		us = SERVO_MAX_US;
	}
	return (UW)us;
}

EXPORT ER servo_init(void)
{
	ER er;

	er = pwm_set_pin(S5_PIN_SERVO);
	if (er < E_OK) {
		return er;
	}
	/* DIV register: integer part in bits 11:4, fraction in bits 3:0 */
	out_w(PWM_BASE + PWM_CHx_DIV + (SERVO_SLICE * 0x14),
	      (SERVO_DIV_INT & 0xFF) << 4);
	pwm_set_wrap(S5_PIN_SERVO, SERVO_WRAP);
	servo_deg_now = 0;
	pwm_set_cc(S5_PIN_SERVO, deg_to_us(0));
	return pwm_set_enabled(S5_PIN_SERVO, TRUE);
}

EXPORT void servo_set_deg(INT deg)
{
	if (deg > SERVO_MAX_DEG) {
		deg = SERVO_MAX_DEG;
	}
	if (deg < -SERVO_MAX_DEG) {
		deg = -SERVO_MAX_DEG;
	}
	servo_deg_now = deg;
	pwm_set_cc(S5_PIN_SERVO, deg_to_us(deg));
}

EXPORT INT servo_get_deg(void)
{
	return servo_deg_now;
}

EXPORT INT servo_settle_ms(INT from_deg, INT to_deg)
{
	INT diff = to_deg - from_deg;

	if (diff < 0) {
		diff = -diff;
	}
	return 20 + diff * 2;
}

EXPORT void servo_off(void)
{
	pwm_set_cc(S5_PIN_SERVO, 0);	/* 0 % duty: no pulses */
}
