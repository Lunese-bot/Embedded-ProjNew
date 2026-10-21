/*
 *	drv_motor.c
 *	Subsystem 2 -- Robo Pico dual DC motor driver.
 *
 *	Each motor has two inputs, A and B (M1A/M1B, M2A/M2B):
 *	    A = PWM, B = 0   forward, speed set by the duty
 *	    A = 0,   B = PWM reverse
 *	    A = 0,   B = 0   coast
 *	    A = 1,   B = 1   brake
 *	Both inputs of one motor sit on the same PWM slice (channels A and
 *	B), so one slice per motor is configured.
 */

#include "drv_motor.h"
#include "s2_config.h"

typedef struct {
	UINT	pin_a;
	UINT	pin_b;
	BOOL	invert;
} MOTOR_PINS;

LOCAL const MOTOR_PINS motor_pins[MOTOR_COUNT] = {
	{ S2_PIN_ML_A, S2_PIN_ML_B, S2_MOTOR_L_INVERT },
	{ S2_PIN_MR_A, S2_PIN_MR_B, S2_MOTOR_R_INVERT },
};

/* Duty in permille -> compare value for TOP = S2_PWM_TOP */
LOCAL UW duty_to_cc(INT duty)
{
	return (UW)(((W)duty * (S2_PWM_TOP + 1)) / S2_DUTY_MAX);
}

EXPORT ER drv_motor_init(void)
{
	INT	m;
	ER	er = E_OK;

	for (m = 0; m < MOTOR_COUNT; m++) {
		const MOTOR_PINS *p = &motor_pins[m];

		/* Slice clock divider stays at its reset value 1.0, so the
		   counter runs at 125 MHz and wraps at S2_PWM_TOP. */
		er |= pwm_set_cc(p->pin_a, 0);
		er |= pwm_set_cc(p->pin_b, 0);
		er |= pwm_set_wrap(p->pin_a, S2_PWM_TOP);
		er |= pwm_set_pin(p->pin_a);
		er |= pwm_set_pin(p->pin_b);
		er |= pwm_set_enabled(p->pin_a, TRUE);
	}
	return (er != E_OK) ? E_IO : E_OK;
}

EXPORT void drv_motor_set(MOTOR_ID m, INT duty)
{
	const MOTOR_PINS *p;

	if ((UINT)m >= (UINT)MOTOR_COUNT) {
		return;
	}
	p = &motor_pins[m];

	if (duty > S2_DUTY_MAX)  duty = S2_DUTY_MAX;
	if (duty < -S2_DUTY_MAX) duty = -S2_DUTY_MAX;
	if (p->invert)           duty = -duty;

	if (duty >= 0) {
		pwm_set_cc(p->pin_b, 0);
		pwm_set_cc(p->pin_a, duty_to_cc(duty));
	} else {
		pwm_set_cc(p->pin_a, 0);
		pwm_set_cc(p->pin_b, duty_to_cc(-duty));
	}
}

EXPORT void drv_motor_brake(MOTOR_ID m)
{
	if ((UINT)m >= (UINT)MOTOR_COUNT) {
		return;
	}
	/* cc > TOP keeps the output high for the whole period */
	pwm_set_cc(motor_pins[m].pin_a, S2_PWM_TOP + 1);
	pwm_set_cc(motor_pins[m].pin_b, S2_PWM_TOP + 1);
}

EXPORT void drv_motor_all_off(void)
{
	drv_motor_set(MOTOR_LEFT, 0);
	drv_motor_set(MOTOR_RIGHT, 0);
}
