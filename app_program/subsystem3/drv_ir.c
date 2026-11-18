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

/* RP2040 WATCHDOG.TICK: bit 9 = enable, bits 8..0 = clk_ref cycles per
   tick.  12 cycles of the 12 MHz crystal = 1 us. */
#define RP2040_WDG_TICK       (0x40058000UL + 0x2CUL)
#define RP2040_WDG_TICK_12MHZ (0x200UL | 12UL)

#define DRV_IR_USE_ADC ((0 != S3_IR_USE_ADC) || (0 != S3_IR_L_ADC))

#if DRV_IR_USE_ADC
#define ADC_CH_IR_C 0       /* GP26 */
#define ADC_CH_IR_R 1       /* GP27 */
#define ADC_CH_IR_L 2       /* GP28 */
#define ADC_MASK    0x0FFFU /* 12-bit conversion result */
static ID g_adc_dd = E_NOEXS;
#endif

/* Which sensors are analogue (see s3_config.h):
     C: S3_IR_USE_ADC 1          R: S3_IR_USE_ADC 1 or 2
     L: S3_IR_L_ADC 1 (pin GP28) */
#define C_IS_ADC (1 == S3_IR_USE_ADC)
#define R_IS_ADC (0 != S3_IR_USE_ADC)
#define L_IS_ADC (0 != S3_IR_L_ADC)

ER drv_ir_init(void)
{
    ER ercd = E_OK;

    /* The 1 MHz TIMER is clocked by the watchdog tick: clk_ref (12 MHz
       crystal) divided by WATCHDOG_TICK.CYCLES.  Nothing in this firmware
       set it, so it kept whatever the boot ROM left: 12 after a UF2 flash
       (timer correct), but 0 after a plain power-up, which divides by 512
       and makes every "microsecond" 42.7 us long.  Seen 2026-11-11: bar
       times in the log were 42.6 x too short after a power-up.  That also
       stretched S3_BC_START_WHITE_MS (150 ms -> 6.4 s) and every other
       timing read from TIMERAWL (ultrasonic echo, WiFi driver delays). */
    if (RP2040_WDG_TICK_12MHZ != (in_w(RP2040_WDG_TICK) & 0x3FFUL))
    {
        out_w(RP2040_WDG_TICK, RP2040_WDG_TICK_12MHZ);
    }

    /* Analogue pins are set up by the ADC driver itself
       (DEVCONF_ADC_PIN_INIT_0/1/2, see s3_config.h); only digital
       sensors are made GPIO inputs here. */
#if !L_IS_ADC
    ercd = gpio_set_pin(S3_PIN_IR_L, GPIO_MODE_IN);
    if (ercd < E_OK)
    {
        return ercd;
    }
#endif
#if !C_IS_ADC
    ercd = gpio_set_pin(S3_PIN_IR_C, GPIO_MODE_IN);
    if (ercd < E_OK)
    {
        return ercd;
    }
#endif
#if !R_IS_ADC
    ercd = gpio_set_pin(S3_PIN_IR_R, GPIO_MODE_IN);
    if (ercd < E_OK)
    {
        return ercd;
    }
#endif
#if DRV_IR_USE_ADC
    /* Cast: the kernel API takes device names as uint8_t strings. */
    g_adc_dd = tk_opn_dev((uint8_t *)"adca", TD_READ);
    if (g_adc_dd < E_OK)
    {
        return g_adc_dd;
    }
#endif
    return ercd;
}

#if DRV_IR_USE_ADC
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
    ER ercd = E_OK;

    /* Digital sensors read as 0 or 1, analogue ones as 0..4095. */
#if L_IS_ADC
    {
        /* S3_L_OVERSAMPLE reads averaged (s3_config.h) */
        uint32_t sum = 0U;
        uint32_t k;
        uint16_t one = 0U;

        for (k = 0U; k < (uint32_t)S3_L_OVERSAMPLE; k++)
        {
            ercd = read_adc(ADC_CH_IR_L, &one);
            if (ercd < E_OK)
            {
                return ercd;
            }
            sum += one;
        }
        /* Cast: an average of 12-bit values fits 16 bits */
        p_raw[S3_IR_L] = (uint16_t)(sum / (uint32_t)S3_L_OVERSAMPLE);
    }
#else
    p_raw[S3_IR_L] = (0U != gpio_get_val(S3_PIN_IR_L)) ? 1U : 0U;
#endif
#if C_IS_ADC
    ercd = read_adc(ADC_CH_IR_C, &p_raw[S3_IR_C]);
    if (ercd < E_OK)
    {
        return ercd;
    }
#else
    p_raw[S3_IR_C] = (0U != gpio_get_val(S3_PIN_IR_C)) ? 1U : 0U;
#endif
#if R_IS_ADC
    ercd = read_adc(ADC_CH_IR_R, &p_raw[S3_IR_R]);
#else
    p_raw[S3_IR_R] = (0U != gpio_get_val(S3_PIN_IR_R)) ? 1U : 0U;
#endif
    return ercd;
}

uint32_t drv_ir_time_us(void)
{
    return in_w(RP2040_TIMERAWL);
}

/*** end of file ***/
