/*
 *      drv_ir.c
 *      Subsystem 3 -- driver for the three IR line sensors (see drv_ir.h).
 *
 *      APIs shown: gpio_set_pin / gpio_get_val, tk_opn_dev / tk_srea_dev
 *      (the kernel's "adca" driver, channel 0 = GP26, 1 = GP27)
 */

#include "drv_ir.h"
#include "subsystem3.h" /* g_s3_lf_* counters */
#include "line_algo.h"  /* l_fast_t (S3_L_FAST) */
#include <tk/syslib.h>  /* in_w() */

/* RP2040 TIMER: raw low word of the 1 MHz counter (read only, no side
   effects, so it is safe from any task on either core) */
#define RP2040_TIMERAWL (0x40054000UL + 0x28UL)

/* RP2040 WATCHDOG.TICK: bit 9 = enable, bits 8..0 = clk_ref cycles per
   tick.  12 cycles of the 12 MHz crystal = 1 us. */
#define RP2040_WDG_TICK       (0x40058000UL + 0x2CUL)
#define RP2040_WDG_TICK_12MHZ (0x200UL | 12UL)

#define DRV_IR_USE_ADC ((0 != S3_IR_USE_ADC) || (0 != S3_IR_L_ADC))

/* Which sensors are analogue (see s3_config.h):
     C: S3_IR_USE_ADC 1          R: S3_IR_USE_ADC 1 or 2
     L: S3_IR_L_ADC 1 (pin GP28) */
#define C_IS_ADC (1 == S3_IR_USE_ADC)
#define R_IS_ADC (0 != S3_IR_USE_ADC)
#define L_IS_ADC (0 != S3_IR_L_ADC)

#if S3_L_FAST && !L_IS_ADC
#error "S3_L_FAST needs an analogue L (S3_IR_L_ADC 1)"
#endif
/* S3_L_FAST: the 2 ms interrupt owns the ADC; the "adca" device is not
   opened and no task starts a conversion. */
#define DRV_IR_DEV_ADC (DRV_IR_USE_ADC && !S3_L_FAST)

#if DRV_IR_USE_ADC
#define ADC_CH_IR_C 0       /* GP26 */
#define ADC_CH_IR_R 1       /* GP27 */
#define ADC_CH_IR_L 2       /* GP28 */
#define ADC_MASK    0x0FFFU /* 12-bit conversion result */
#endif
#if DRV_IR_DEV_ADC
static ID g_adc_dd = E_NOEXS;
#endif

#if S3_L_FAST
/* RP2040 registers (datasheet 4.6 TIMER, 4.9 ADC) */
#define LF_TIMER_BASE  0x40054000UL
#define LF_TIMER_ALARM (LF_TIMER_BASE + 0x10UL + (4UL * S3_L_FAST_ALARM))
#define LF_TIMER_INTR  (LF_TIMER_BASE + 0x34UL) /* write 1 to clear */
#define LF_TIMER_INTE  (LF_TIMER_BASE + 0x38UL)
#define LF_ALARM_BIT   (1UL << S3_L_FAST_ALARM)
#define LF_INTNO       ((UINT)S3_L_FAST_ALARM) /* TIMER_IRQ_n = IRQ n */
#define LF_ADC_BASE    0x4004C000UL
#define LF_ADC_CS      (LF_ADC_BASE + 0x00UL)
#define LF_ADC_RESULT  (LF_ADC_BASE + 0x04UL)
#define LF_ADC_INTE    (LF_ADC_BASE + 0x18UL)
#define LF_CS_EN       (1UL << 0)
#define LF_CS_START    (1UL << 2)
#define LF_CS_READY    (1UL << 8)
#define LF_CS_ERR      (1UL << 9)
#define LF_CS_AINSEL   12
/* one conversion is 96 ADC clocks = 2 us at 48 MHz; give up far later */
#define LF_READY_SPIN  2000u

static l_fast_t    g_lf;
static l_fast_rd_t g_lf_rd;
static uint32_t    g_lf_next; /* TIMERAWL of the next alarm */

/* bench counters (dashboard GET /b: lf...) */
volatile uint32_t g_s3_lf_isr;     /* 2 ms samples taken          */
volatile uint32_t g_s3_lf_late;    /* alarm re-armed after a miss */
volatile uint32_t g_s3_lf_max_us;  /* longest interrupt, us       */
volatile uint32_t g_s3_lf_adc_err; /* conversion timeout or error */
volatile uint32_t g_s3_lf_stall;   /* 10 ms reads with no sample  */
volatile uint32_t g_s3_lf_used;    /* samples in the last record  */

static uint16_t lf_convert(uint32_t ch)
{
    uint32_t k;
    uint32_t cs;

    out_w(LF_ADC_CS, (ch << LF_CS_AINSEL) | LF_CS_EN);
    out_w(LF_ADC_CS, (ch << LF_CS_AINSEL) | LF_CS_EN | LF_CS_START);
    for (k = 0u; k < LF_READY_SPIN; k++)
    {
        cs = in_w(LF_ADC_CS);
        if ((0u == (cs & LF_CS_START)) && (0u != (cs & LF_CS_READY)))
        {
            if (0u != (cs & LF_CS_ERR))
            {
                g_s3_lf_adc_err++;
            }
            /* Cast: masked to 12 bits */
            return (uint16_t)(in_w(LF_ADC_RESULT) & ADC_MASK);
        }
    }
    g_s3_lf_adc_err++;
    return 0u;
}

/* TIMER alarm interrupt, every S3_L_FAST_PERIOD_US.  No kernel calls. */
static void lf_isr(UINT intno)
{
    uint32_t t0  = in_w(RP2040_TIMERAWL);
    uint32_t sum = 0u;
    uint32_t k;
    uint16_t c = 0u;
    uint16_t r = 0u;
    uint32_t dt;

    out_w(LF_TIMER_INTR, LF_ALARM_BIT); /* clear the alarm */
    g_lf_next += S3_L_FAST_PERIOD_US;
    /* Cast: wrapped difference taken as signed */
    if ((int32_t)(g_lf_next - t0) < 20)
    {
        g_lf_next = t0 + S3_L_FAST_PERIOD_US; /* fell behind: start again */
        g_s3_lf_late++;
    }
    out_w(LF_TIMER_ALARM, g_lf_next); /* writing it arms the alarm */

    for (k = 0u; k < S3_L_FAST_CONV; k++)
    {
        sum += lf_convert(ADC_CH_IR_L);
    }
#if C_IS_ADC
    c = lf_convert(ADC_CH_IR_C);
#endif
#if R_IS_ADC
    r = lf_convert(ADC_CH_IR_R);
#endif
    /* Cast: a mean of 12-bit values fits 16 bits */
    l_fast_put(&g_lf,
               (uint16_t)((sum + (S3_L_FAST_CONV / 2u)) / S3_L_FAST_CONV), c,
               r);
    g_s3_lf_isr++;
    dt = in_w(RP2040_TIMERAWL) - t0;
    if (dt > g_s3_lf_max_us)
    {
        g_s3_lf_max_us = dt;
    }
    ClearInt(intno);
}

/* Called by line_task (core 1, TP_PRC2): the NVIC is per core, so the
   interrupt runs on core 1 next to its only reader. */
static ER lf_start(void)
{
    T_DINT dint;
    ER     ercd;

    l_fast_reset(&g_lf, &g_lf_rd);
    out_w(LF_ADC_INTE, 0u);     /* no ADC interrupt: we poll READY */
    set_w(LF_ADC_CS, LF_CS_EN); /* already on after dev_adc_llinit */
    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)lf_isr;
    ercd        = tk_def_int(LF_INTNO, &dint);
    if (ercd < E_OK)
    {
        return ercd;
    }
    set_w(LF_TIMER_INTE, LF_ALARM_BIT);
    ClearInt(LF_INTNO);
    EnableInt(LF_INTNO, S3_L_FAST_INTPRI);
    g_lf_next = in_w(RP2040_TIMERAWL) + S3_L_FAST_PERIOD_US;
    out_w(LF_TIMER_ALARM, g_lf_next);
    return E_OK;
}
#endif

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
#if DRV_IR_DEV_ADC
    /* Cast: the kernel API takes device names as uint8_t strings. */
    g_adc_dd = tk_opn_dev((uint8_t *)"adca", TD_READ);
    if (g_adc_dd < E_OK)
    {
        return g_adc_dd;
    }
#endif
#if S3_L_FAST
    ercd = lf_start();
#endif
    return ercd;
}

#if DRV_IR_DEV_ADC
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
#if S3_L_FAST
    {
        uint32_t sum  = 0u;
        uint32_t n    = 0u;
        uint32_t used = 0u;
        uint16_t c    = 0u;
        uint16_t r    = 0u;

        if (!l_fast_get(&g_lf, &sum, &n, &c, &r))
        {
            return E_BUSY; /* counted by line_task as a read error */
        }
        p_raw[S3_IR_L] = l_fast_record(&g_lf_rd, sum, n, &used);
        g_s3_lf_used   = used;
        if (0u == used)
        {
            g_s3_lf_stall++; /* no 2 ms sample since the last read */
        }
#if C_IS_ADC
        p_raw[S3_IR_C] = c;
#else
        p_raw[S3_IR_C] = (0U != gpio_get_val(S3_PIN_IR_C)) ? 1U : 0U;
#endif
#if R_IS_ADC
        p_raw[S3_IR_R] = r;
#else
        p_raw[S3_IR_R] = (0U != gpio_get_val(S3_PIN_IR_R)) ? 1U : 0U;
#endif
        return ercd;
    }
#else /* !S3_L_FAST: the old path */
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
#endif /* S3_L_FAST */
}

uint32_t drv_ir_time_us(void)
{
    return in_w(RP2040_TIMERAWL);
}

/*** end of file ***/
