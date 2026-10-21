/*
 *	drv_ir.c
 *	Subsystem 3 -- driver for the three IR line sensors (see drv_ir.h).
 *
 *	APIs shown: gpio_set_pin / gpio_get_val, tk_opn_dev / tk_srea_dev
 *	(the kernel's "adca" driver, channel 0 = GP26, 1 = GP27)
 */

#include "drv_ir.h"
#include <tk/syslib.h>		/* in_w() */

/* RP2040 TIMER: raw low word of the 1 MHz counter (read only, no side
   effects, so it is safe from any task on either core) */
#define RP2040_TIMERAWL		(0x40054000UL + 0x28UL)

#if S3_IR_USE_ADC
#define ADC_CH_IR_C		0	/* GP26 */
#define ADC_CH_IR_R		1	/* GP27 */
LOCAL ID	adc_dd = E_NOEXS;
#endif

EXPORT ER drv_ir_init(void)
{
	ER	er;

	er = gpio_set_pin(S3_PIN_IR_L, GPIO_MODE_IN);
	if (er < E_OK) {
		return er;
	}

#if S3_IR_USE_ADC
	/* GP26/GP27 are made analogue by the ADC driver itself
	   (DEVCONF_ADC_PIN_INIT_0/1, see s3_config.h) */
	adc_dd = tk_opn_dev((UB*)"adca", TD_READ);
	if (adc_dd < E_OK) {
		return adc_dd;
	}
#else
	er = gpio_set_pin(S3_PIN_IR_C, GPIO_MODE_IN);
	if (er < E_OK) {
		return er;
	}
	er = gpio_set_pin(S3_PIN_IR_R, GPIO_MODE_IN);
	if (er < E_OK) {
		return er;
	}
#endif
	return E_OK;
}

#if S3_IR_USE_ADC
LOCAL ER read_adc(INT ch, UH *p_val)
{
	UW	val = 0;
	SZ	asz;
	ER	er;

	/* start = channel, size = 1 sample (the driver allows only 1) */
	er = tk_srea_dev(adc_dd, ch, &val, 1, &asz);
	*p_val = (UH)(val & 0x0FFF);
	return er;
}
#endif

EXPORT ER drv_ir_read(UH raw[S3_IR_COUNT])
{
	raw[S3_IR_L] = (UH)(gpio_get_val(S3_PIN_IR_L) ? 1 : 0);

#if S3_IR_USE_ADC
	{
		ER er = read_adc(ADC_CH_IR_C, &raw[S3_IR_C]);
		if (er < E_OK) {
			return er;
		}
		return read_adc(ADC_CH_IR_R, &raw[S3_IR_R]);
	}
#else
	raw[S3_IR_C] = (UH)(gpio_get_val(S3_PIN_IR_C) ? 1 : 0);
	raw[S3_IR_R] = (UH)(gpio_get_val(S3_PIN_IR_R) ? 1 : 0);
	return E_OK;
#endif
}

EXPORT UW drv_ir_time_us(void)
{
	return in_w(RP2040_TIMERAWL);
}
