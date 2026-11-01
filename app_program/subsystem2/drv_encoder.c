/*
 *      drv_encoder.c
 *      Subsystem 2 -- wheel encoders, counted by RP2040 PWM slices.
 *
 *      Each RP2040 PWM slice can be switched from "make a waveform" to
 *      "count rising edges on my B pin" (datasheet 4.5.2.6, DIVMODE = 2).
 *      The counter then counts encoder pulses by itself: no interrupt, no
 *      missed pulse while a task is busy, and nothing runs on either core.
 *      The control loop just reads how far the counter moved.
 *
 *      The encoders are single channel (one signal per wheel), so they
 *      give distance but not direction.  motion_task takes the direction
 *      from the sign of the duty it is applying to that wheel.
 *
 *      Uses PWM slices 1 and 2 (GP3, GP5); the motors use slices 4 and 5.
 */

#include "drv_encoder.h"
#include "s2_config.h"

#define ENC_SLICE_MASK   0x07U     /* 8 slices; GPn is on slice (n / 2) % 8 */
#define ENC_SLICE_STRIDE 0x14U     /* bytes between two slices' registers */
#define ENC_COUNTER_MAX  0xFFFFU   /* 16-bit counter */
#define CSR_DIVMODE_RISE (2U << 4) /* count rising edges of B */
#define DIV_ONE          (1U << 4) /* integer 1, fraction 0 */

/* PWM slice that owns a GPIO pin */
static uint32_t enc_slice(uint32_t pin)
{
    return ((pin >> 1) & ENC_SLICE_MASK);
}

/* Address of one register (offset PWM_CHx_...) of a PWM slice */
static uint32_t enc_reg(uint32_t slice, uint32_t offset)
{
    return (PWM_BASE + offset + (slice * ENC_SLICE_STRIDE));
}

static const uint32_t g_enc_pin[MOTOR_COUNT] = { S2_PIN_ENC_L, S2_PIN_ENC_R };
static uint16_t       g_enc_last[MOTOR_COUNT];

ER drv_encoder_init(void)
{
    int32_t  motor;
    uint32_t pin;
    uint32_t slice;

    for (motor = 0; motor < MOTOR_COUNT; motor++)
    {
        pin = g_enc_pin[motor];
        if ((0 == (pin & 1U)) || (pin >= GPIO_NUM))
        {
            return E_PAR; /* only odd pins are a B input */
        }
        slice = enc_slice(pin);

        out_w(enc_reg(slice, PWM_CHx_CSR), 0);       /* stop the slice */
        out_w(enc_reg(slice, PWM_CHx_DIV), DIV_ONE); /* 1 count per edge */
        out_w(enc_reg(slice, PWM_CHx_TOP), ENC_COUNTER_MAX); /* 16 bits */
        out_w(enc_reg(slice, PWM_CHx_CTR), 0);

        pwm_set_pin(pin);               /* pin -> PWM function,
                                           input enabled */
        set_w(GPIO(pin), GPIO_PUE);     /* pull-up for open-
                                           collector sensors */
        set_w(GPIO(pin), GPIO_SHEMITT); /* Schmitt trigger */

        out_w(enc_reg(slice, PWM_CHx_CSR), CSR_DIVMODE_RISE | PWM_CH_CSR_EN);
        g_enc_last[motor] = 0;
    }
    return E_OK;
}

uint32_t drv_encoder_delta(motor_id_t motor)
{
    uint16_t now;
    uint16_t delta;

    /* Cast to unsigned so a negative (invalid) id also fails the test */
    if ((uint32_t)motor >= (uint32_t)MOTOR_COUNT)
    {
        return 0;
    }
    /* The counter is 16 bits: masked, the value always fits uint16_t.
       The 16-bit subtraction handles the counter wrapping. */
    now   = (uint16_t)(in_w(enc_reg(enc_slice(g_enc_pin[motor]), PWM_CHx_CTR)) &
                     ENC_COUNTER_MAX);
    delta = (uint16_t)(now - g_enc_last[motor]);
    g_enc_last[motor] = now;
    return (uint32_t)delta;
}

/*** end of file ***/
