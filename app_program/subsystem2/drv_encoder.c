/*
 *	drv_encoder.c
 *	Subsystem 2 -- wheel encoders, counted by RP2040 PWM slices.
 *
 *	Each RP2040 PWM slice can be switched from "make a waveform" to
 *	"count rising edges on my B pin" (datasheet 4.5.2.6, DIVMODE = 2).
 *	The counter then counts encoder pulses by itself: no interrupt, no
 *	missed pulse while a task is busy, and nothing runs on either core.
 *	The control loop just reads how far the counter moved.
 *
 *	The encoders are single channel (one signal per wheel), so they
 *	give distance but not direction.  motion_task takes the direction
 *	from the sign of the duty it is applying to that wheel.
 *
 *	Uses PWM slices 1 and 2 (GP3, GP5); the motors use slices 4 and 5.
 */

#include "drv_encoder.h"
#include "s2_config.h"

#define ENC_SLICE(pin)		(((pin) >> 1) & 0x07)
#define ENC_CSR(sl)		(PWM_BASE + PWM_CHx_CSR + ((sl) * 0x14))
#define ENC_DIV(sl)		(PWM_BASE + PWM_CHx_DIV + ((sl) * 0x14))
#define ENC_CTR(sl)		(PWM_BASE + PWM_CHx_CTR + ((sl) * 0x14))
#define ENC_TOP(sl)		(PWM_BASE + PWM_CHx_TOP + ((sl) * 0x14))

#define CSR_DIVMODE_RISE	(2U << 4)	/* count rising edges of B */
#define DIV_ONE			(1U << 4)	/* integer 1, fraction 0 */

LOCAL const UINT enc_pin[MOTOR_COUNT] = { S2_PIN_ENC_L, S2_PIN_ENC_R };
LOCAL UH enc_last[MOTOR_COUNT];

EXPORT ER drv_encoder_init(void)
{
	INT	m;
	UINT	pin, sl;

	for (m = 0; m < MOTOR_COUNT; m++) {
		pin = enc_pin[m];
		if ((pin & 1U) == 0 || pin >= GPIO_NUM) {
			return E_PAR;	/* only odd pins are a B input */
		}
		sl = ENC_SLICE(pin);

		out_w(ENC_CSR(sl), 0);			/* stop the slice */
		out_w(ENC_DIV(sl), DIV_ONE);		/* 1 count per edge */
		out_w(ENC_TOP(sl), 0xFFFF);		/* full 16-bit range */
		out_w(ENC_CTR(sl), 0);

		pwm_set_pin(pin);			/* pin -> PWM function,
							   input enabled */
		set_w(GPIO(pin), GPIO_PUE);		/* pull-up for open-
							   collector sensors */
		set_w(GPIO(pin), GPIO_SHEMITT);		/* Schmitt trigger */

		out_w(ENC_CSR(sl), CSR_DIVMODE_RISE | PWM_CH_CSR_EN);
		enc_last[m] = 0;
	}
	return E_OK;
}

EXPORT UW drv_encoder_delta(MOTOR_ID m)
{
	UH	now, delta;

	if ((UINT)m >= (UINT)MOTOR_COUNT) {
		return 0;
	}
	now = (UH)(in_w(ENC_CTR(ENC_SLICE(enc_pin[m]))) & 0xFFFF);
	delta = (UH)(now - enc_last[m]);	/* 16-bit maths handles wrap */
	enc_last[m] = now;
	return (UW)delta;
}
