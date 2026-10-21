/*
 *	drv_ultrasonic.c
 *	Subsystem 5 -- HC-SR04 style ultrasonic driver.
 *
 *	Method: 10 us pulse on TRIG, then time the ECHO high pulse with the
 *	RP2040's 1 MHz TIMER (TIMERAWL, read without latching).
 *	distance_mm = echo_us * 343 m/s / 2 = echo_us * 343 / 2000.
 *
 *	TODO (Buddy 5): this busy-waits for the echo (up to ~12 ms).  It runs
 *	at a low priority so higher tasks still pre-empt it, but a GPIO edge
 *	interrupt (tk_def_int on IO_IRQ_BANK0) would free the CPU.  Measure
 *	the execution time (NFR2) before deciding.
 */

#include "drv_ultrasonic.h"

#define TRIG_PULSE_US		10
#define ECHO_START_TMO_US	3000	/* sensor raises ECHO within ~0.5 ms */
#define ECHO_MAX_US		((US_RANGE_MAX_MM * 2000UL) / 343UL + 200UL)

EXPORT UW us_now_us(void)
{
	return in_w(TIMER_TIMERAWL);
}

LOCAL void busy_wait_us(UW us)
{
	UW t0 = us_now_us();

	while ((UW)(us_now_us() - t0) < us) {
		;
	}
}

EXPORT ER us_init(void)
{
	ER er;

	er = gpio_set_pin(S5_PIN_TRIG, GPIO_MODE_OUT);
	if (er < E_OK) {
		return er;
	}
	gpio_set_val(S5_PIN_TRIG, 0);
	return gpio_set_pin(S5_PIN_ECHO, GPIO_MODE_IN);
}

EXPORT UH us_read_mm(void)
{
	UW t0;
	UW rise;
	UW width;

	/* A previous echo still high means the sensor is stuck or wired
	   wrongly; report "no echo" rather than wait. */
	if (gpio_get_val(S5_PIN_ECHO) != 0) {
		return US_NO_ECHO;
	}

	gpio_set_val(S5_PIN_TRIG, 1);
	busy_wait_us(TRIG_PULSE_US);
	gpio_set_val(S5_PIN_TRIG, 0);

	/* Wait for the rising edge */
	t0 = us_now_us();
	while (gpio_get_val(S5_PIN_ECHO) == 0) {
		if ((UW)(us_now_us() - t0) > ECHO_START_TMO_US) {
			return US_NO_ECHO;
		}
	}
	rise = us_now_us();

	/* Time the high pulse */
	while (gpio_get_val(S5_PIN_ECHO) != 0) {
		if ((UW)(us_now_us() - rise) > ECHO_MAX_US) {
			return US_NO_ECHO;	/* nothing in range */
		}
	}
	width = us_now_us() - rise;

	width = (width * 343UL) / 2000UL;	/* -> mm */
	if (width == 0 || width > US_RANGE_MAX_MM) {
		return US_NO_ECHO;
	}
	return (UH)width;
}

EXPORT UH us_read_median_mm(INT n, INT gap_ms)
{
	UH	v[5];
	INT	got = 0;
	INT	i;
	INT	j;
	UH	d;
	UH	t;

	if (n < 1) {
		n = 1;
	}
	if (n > 5) {
		n = 5;
	}
	for (i = 0; i < n; i++) {
		d = us_read_mm();
		if (d != US_NO_ECHO) {
			v[got++] = d;
		}
		if (i + 1 < n && gap_ms > 0) {
			tk_dly_tsk(gap_ms);
		}
	}
	if (got == 0) {
		return US_NO_ECHO;
	}
	/* insertion sort, at most 5 items */
	for (i = 1; i < got; i++) {
		t = v[i];
		for (j = i - 1; j >= 0 && v[j] > t; j--) {
			v[j + 1] = v[j];
		}
		v[j + 1] = t;
	}
	return v[got / 2];
}
