/*
 *      drv_ir.c
 *      Subsystem 3 -- driver for the three IR line sensors (see drv_ir.h).
 *
 *      APIs shown: gpio_set_pin / gpio_get_val, tk_opn_dev / tk_srea_dev
 *      (the kernel's "adca" driver, channel 0 = GP26, 1 = GP27)
 */

#include "drv_ir.h"
#include <tk/syslib.h> /* in_w() */

/* RP2040 TIMER: raw low word of the 1 MHz counter (read only, no side
   effects, so it is safe from any task on either core) */
#define RP2040_TIMERAWL (0x40054000UL + 0x28UL)

#if S3_IR_USE_ADC
#define ADC_CH_IR_C 0       /* GP26 */
#define ADC_CH_IR_R 1       /* GP27 */
#define ADC_MASK    0x0FFFU /* 12-bit conversion result */
static ID g_adc_dd = E_NOEXS;
#endif

ER drv_ir_init(void)
{
    ER ercd;

    ercd = gpio_set_pin(S3_PIN_IR_L, GPIO_MODE_IN);
    if (ercd < E_OK)
    {
        return ercd;
    }

#if S3_IR_USE_ADC
    /* GP26/GP27 are made analogue by the ADC driver itself
       (DEVCONF_ADC_PIN_INIT_0/1, see s3_config.h) */
    /* Cast: the kernel API takes device names as uint8_t strings. */
    g_adc_dd = tk_opn_dev((uint8_t *)"adca", TD_READ);
    if (g_adc_dd < E_OK)
    {
        return g_adc_dd;
    }
#else
    ercd = gpio_set_pin(S3_PIN_IR_C, GPIO_MODE_IN);
    if (ercd < E_OK)
    {
        return ercd;
    }
    ercd = gpio_set_pin(S3_PIN_IR_R, GPIO_MODE_IN);
    if (ercd < E_OK)
    {
        return ercd;
    }
#endif
    return E_OK;
}

#if S3_IR_USE_ADC
static ER read_adc(int32_t ch, uint16_t * p_val)
{
    uint32_t val = 0;
    SZ       asz;
    ER       ercd;

    /* start = channel, size = 1 sample (the driver allows only 1) */
    ercd   = tk_srea_dev(g_adc_dd, ch, &val, 1, &asz);
    *p_val = (uint16_t)(val & ADC_MASK); /* Cast: masked to 12 bits */
    return ercd;
}
#endif

ER drv_ir_read(uint16_t p_raw[S3_IR_COUNT])
{
    /* Digital sensors read as 0 or 1 in the 16-bit sample. */
    p_raw[S3_IR_L] = (0U != gpio_get_val(S3_PIN_IR_L)) ? 1U : 0U;

#if S3_IR_USE_ADC
    {
        ER ercd = read_adc(ADC_CH_IR_C, &p_raw[S3_IR_C]);

        if (ercd < E_OK)
        {
            return ercd;
        }
        return read_adc(ADC_CH_IR_R, &p_raw[S3_IR_R]);
    }
#else
    p_raw[S3_IR_C] = (0U != gpio_get_val(S3_PIN_IR_C)) ? 1U : 0U;
    p_raw[S3_IR_R] = (0U != gpio_get_val(S3_PIN_IR_R)) ? 1U : 0U;
    return E_OK;
#endif
}

uint32_t drv_ir_time_us(void)
{
    return in_w(RP2040_TIMERAWL);
}

/*** end of file ***/
