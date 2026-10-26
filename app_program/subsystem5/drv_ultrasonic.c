/*
 *      drv_ultrasonic.c
 *      Subsystem 5 -- HC-SR04 style ultrasonic driver.
 *
 *      Method: 10 us pulse on TRIG, then time the ECHO high pulse with the
 *      RP2040's 1 MHz TIMER (TIMERAWL, read without latching).
 *      distance_mm = echo_us * 343 m/s / 2 = echo_us * 343 / 2000.
 *
 *      TODO (Buddy 5): this busy-waits for the echo (up to ~12 ms).  It runs
 *      at a low priority so higher tasks still pre-empt it, but a GPIO edge
 *      interrupt (tk_def_int on IO_IRQ_BANK0) would free the CPU.  Measure
 *      the execution time (NFR2) before deciding.
 */

#include "drv_ultrasonic.h"

#define TRIG_PULSE_US     10
#define ECHO_START_TMO_US 3000   /* sensor raises ECHO within ~0.5 ms */
#define SOUND_MM_PER_MS   343UL  /* speed of sound, 343 m/s           */
#define US_PER_MS_2WAY    2000UL /* 1000 us/ms x 2 (there and back)   */
#define ECHO_MARGIN_US    200UL
#define ECHO_MAX_US \
    (((US_RANGE_MAX_MM * US_PER_MS_2WAY) / SOUND_MM_PER_MS) + ECHO_MARGIN_US)

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

uint16_t us_read_mm(void)
{
    uint32_t start_us;
    uint32_t rise;
    uint32_t width;

    /* A previous echo still high means the sensor is stuck or wired
       wrongly; report "no echo" rather than wait. */
    if (0U != gpio_get_val(S5_PIN_ECHO))
    {
        return US_NO_ECHO;
    }

    gpio_set_val(S5_PIN_TRIG, 1);
    busy_wait_us(TRIG_PULSE_US);
    gpio_set_val(S5_PIN_TRIG, 0);

    /* Wait for the rising edge */
    start_us = us_now_us();
    while (0U == gpio_get_val(S5_PIN_ECHO))
    {
        if ((uint32_t)(us_now_us() - start_us) > ECHO_START_TMO_US)
        {
            return US_NO_ECHO;
        }
    }
    rise = us_now_us();

    /* Time the high pulse */
    while (0U != gpio_get_val(S5_PIN_ECHO))
    {
        if ((uint32_t)(us_now_us() - rise) > ECHO_MAX_US)
        {
            return US_NO_ECHO; /* nothing in range */
        }
    }
    width = us_now_us() - rise;

    width = (width * SOUND_MM_PER_MS) / US_PER_MS_2WAY; /* -> mm */
    if ((0 == width) || (width > US_RANGE_MAX_MM))
    {
        return US_NO_ECHO;
    }
    return (uint16_t)width;
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
