/*
 *      drv_ultrasonic.c
 *      Subsystem 5 -- HC-SR04 style ultrasonic driver.
 *
 *      Method: 10 us pulse on TRIG, then time the ECHO high pulse with the
 *      RP2040's 1 MHz TIMER (TIMERAWL, read without latching).
 *      distance_mm = echo_us * 343 m/s / 2 = echo_us * 343 / 2000.
 *
 *      The echo is busy-waited, bounded by max_mm: about 6 ms for the
 *      1000 mm monitoring range, about 12 ms for the full 2000 mm range.
 *      With S5_PING_NO_PREEMPT 1 task switching is paused meanwhile
 *      (tk_dis_dsp); it is off by default.
 *      Possible later change: a GPIO edge interrupt (IO_IRQ_BANK0) would
 *      free the CPU completely.
 */

#include "drv_ultrasonic.h"

#define TRIG_PULSE_US     10
#define ECHO_START_TMO_US 3000   /* sensor raises ECHO within ~0.5 ms */
#define SOUND_MM_PER_MS   343UL  /* speed of sound, 343 m/s           */
#define US_PER_MS_2WAY    2000UL /* 1000 us/ms x 2 (there and back)   */
#define ECHO_MARGIN_US    200UL
/* A tight polling loop reads the timer every ~1 us.  A gap this long means
   something else (Wi-Fi task, interrupt) ran in the middle of the ping, so
   the measurement is not trustworthy. */
#define DISTURB_GAP_US    150UL
#define PING_RETRIES      2     /* extra tries after a disturbed ping   */
#define RETRY_GAP_MS      25    /* let late echoes die away first       */
#define ECHO_MAX_US(max_mm) \
    ((((uint32_t)(max_mm) * US_PER_MS_2WAY) / SOUND_MM_PER_MS) + ECHO_MARGIN_US)

/* Raw numbers of the last ping, for the S5_DIAG log (us_last_raw). */
static uint32_t g_raw_idle;     /* ECHO level before the trigger (0 = low) */
static uint32_t g_raw_rise_us;  /* trigger end -> ECHO rising, 0 = none    */
static uint32_t g_raw_width_us; /* ECHO high time, 0 = none                */

void us_last_raw(uint32_t * p_idle, uint32_t * p_rise_us, uint32_t * p_width_us)
{
    *p_idle     = g_raw_idle;
    *p_rise_us  = g_raw_rise_us;
    *p_width_us = g_raw_width_us;
}

static uint32_t g_gap_max_us;    /* longest gap between timer reads, last ping */
static uint32_t g_disturbed_pings; /* pings thrown away because of a gap       */

uint32_t us_disturbed_pings(void)
{
    return g_disturbed_pings;
}

uint32_t us_now_us(void)
{
    return in_w(TIMER_TIMERAWL);
}

static void busy_wait_us(uint32_t delay_us)
{
    uint32_t start_us = us_now_us();

    while ((uint32_t)(us_now_us() - start_us) < delay_us)
    {
        ;
    }
}

ER us_init(void)
{
    ER ercd;

    ercd = gpio_set_pin(S5_PIN_TRIG, GPIO_MODE_OUT);
    if (ercd < E_OK)
    {
        return ercd;
    }
    gpio_set_val(S5_PIN_TRIG, 0);
    return gpio_set_pin(S5_PIN_ECHO, GPIO_MODE_IN);
}

/* Time one echo.  Called with dispatching disabled: one exit path only
   after the trigger, so tk_ena_dsp is never skipped.  Returns the echo
   width in us, or 0 for no echo. */
static uint32_t time_echo_us(uint32_t max_us)
{
    uint32_t start_us;
    uint32_t rise  = 0U;
    uint32_t width = 0U;
    bool     b_ok  = true;

    uint32_t prev_us;
    uint32_t now_us;

    g_gap_max_us = 0U;
    gpio_set_val(S5_PIN_TRIG, 1);
    busy_wait_us(TRIG_PULSE_US);
    gpio_set_val(S5_PIN_TRIG, 0);

    /* Wait for the rising edge */
    start_us = us_now_us();
    prev_us  = start_us;
    while (b_ok && (0U == gpio_get_val(S5_PIN_ECHO)))
    {
        now_us = us_now_us();
        if ((uint32_t)(now_us - prev_us) > g_gap_max_us)
        {
            g_gap_max_us = (uint32_t)(now_us - prev_us);
        }
        prev_us = now_us;
        if ((uint32_t)(now_us - start_us) > ECHO_START_TMO_US)
        {
            b_ok = false;
        }
    }

    /* Time the high pulse */
    if (b_ok)
    {
        rise           = us_now_us();
        g_raw_rise_us  = (uint32_t)(rise - start_us);
        prev_us        = rise;
        while (b_ok && (0U != gpio_get_val(S5_PIN_ECHO)))
        {
            now_us = us_now_us();
            if ((uint32_t)(now_us - prev_us) > g_gap_max_us)
            {
                g_gap_max_us = (uint32_t)(now_us - prev_us);
            }
            prev_us = now_us;
            if ((uint32_t)(now_us - rise) > max_us)
            {
                b_ok = false; /* nothing in range */
            }
        }
    }
    if (b_ok)
    {
        width = us_now_us() - rise;
    }
    g_raw_width_us = width;
    return width;
}

uint16_t us_read_mm_max(uint16_t max_mm)
{
    uint32_t width;
    uint32_t retries = 0U;
    bool     b_dsp_off;

    if ((0U == max_mm) || (max_mm > US_RANGE_MAX_MM))
    {
        max_mm = US_RANGE_MAX_MM;
    }

    /* A previous echo still high means the sensor is stuck or wired
       wrongly; report "no echo" rather than wait. */
    g_raw_idle     = (uint32_t)gpio_get_val(S5_PIN_ECHO);
    g_raw_rise_us  = 0U;
    g_raw_width_us = 0U;
    if (0U != g_raw_idle)
    {
        return US_NO_ECHO;
    }

    /* No task switch on this core while the pulse is timed */
#if S5_PING_NO_PREEMPT
    b_dsp_off = (E_OK == tk_dis_dsp());
#else
    b_dsp_off = false;
#endif
    width     = time_echo_us(ECHO_MAX_US(max_mm));
    if (b_dsp_off)
    {
        (void)tk_ena_dsp();
    }

    /* Something ran in the middle of the ping (gap in the timer reads):
       the width cannot be trusted, and a missed echo here is not "nothing
       in front".  Count it and measure again after the late echoes died. */
    while ((g_gap_max_us > DISTURB_GAP_US) && (retries < PING_RETRIES))
    {
        g_disturbed_pings++;
        retries++;
        tk_dly_tsk(RETRY_GAP_MS);
        if (0U != gpio_get_val(S5_PIN_ECHO))
        {
            return US_NO_ECHO;
        }
        g_raw_rise_us  = 0U;
        g_raw_width_us = 0U;
        width          = time_echo_us(ECHO_MAX_US(max_mm));
    }
    if (g_gap_max_us > DISTURB_GAP_US)
    {
        g_disturbed_pings++;
        return US_NO_ECHO; /* still disturbed: report nothing, not a guess */
    }

    width = (width * SOUND_MM_PER_MS) / US_PER_MS_2WAY; /* -> mm */
    if ((width < US_MIN_MM) || (width > max_mm))
    {
        return US_NO_ECHO;
    }
    return (uint16_t)width;
}

uint16_t us_read_mm(void)
{
    return us_read_mm_max(US_RANGE_MAX_MM);
}

uint16_t us_read_median_mm(int32_t samples, int32_t gap_ms)
{
    uint16_t vals[5];
    int32_t  got = 0;
    int32_t  idx;
    int32_t  jdx;
    uint16_t reading;
    uint16_t tmp;

    if (samples < 1)
    {
        samples = 1;
    }
    if (samples > 5)
    {
        samples = 5;
    }
    for (idx = 0; idx < samples; idx++)
    {
        reading = us_read_mm();
        if (reading != US_NO_ECHO)
        {
            vals[got++] = reading;
        }
        if (((idx + 1) < samples) && (gap_ms > 0))
        {
            tk_dly_tsk(gap_ms);
        }
    }
    if (0 == got)
    {
        return US_NO_ECHO;
    }
    /* insertion sort, at most 5 items */
    for (idx = 1; idx < got; idx++)
    {
        tmp = vals[idx];
        for (jdx = idx - 1; (jdx >= 0) && (vals[jdx] > tmp); jdx--)
        {
            vals[jdx + 1] = vals[jdx];
        }
        vals[jdx + 1] = tmp;
    }
    return vals[got / 2];
}

/*** end of file ***/
