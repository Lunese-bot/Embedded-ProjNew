/*
 *      drv_motor.c
 *      Subsystem 2 -- Robo Pico dual DC motor driver.
 *
 *      Each motor has two inputs, A and B (M1A/M1B, M2A/M2B):
 *          A = PWM, B = 0   forward, speed set by the duty
 *          A = 0,   B = PWM reverse
 *          A = 0,   B = 0   coast
 *          A = 1,   B = 1   brake
 *      Both inputs of one motor sit on the same PWM slice (channels A and
 *      B), so one slice per motor is configured.
 */

#include "drv_motor.h"
#include "s2_config.h"

typedef struct
{
    uint32_t pin_a;
    uint32_t pin_b;
    bool     invert;
} motor_pins_t;

static const motor_pins_t g_motor_pins[MOTOR_COUNT] = {
    { S2_PIN_ML_A, S2_PIN_ML_B, S2_MOTOR_L_INVERT },
    { S2_PIN_MR_A, S2_PIN_MR_B, S2_MOTOR_R_INVERT },
};

/* Duty in permille (0..S2_DUTY_MAX) -> compare value for TOP = S2_PWM_TOP.
   Callers pass duty >= 0, so the cast to unsigned keeps the value. */
static uint32_t duty_to_cc(int32_t duty)
{
    return (uint32_t)((duty * (S2_PWM_TOP + 1)) / S2_DUTY_MAX);
}

/* Remember that a PWM set-up call failed */
static void note_error(ER ercd, bool * p_b_failed)
{
    if (E_OK != ercd)
    {
        *p_b_failed = true;
    }
}

ER drv_motor_init(void)
{
    int32_t motor;
    bool    b_failed = false;

    for (motor = 0; motor < MOTOR_COUNT; motor++)
    {
        const motor_pins_t * p_pins = &g_motor_pins[motor];

        /* Slice clock divider stays at its reset value 1.0, so the
           counter runs at 125 MHz and wraps at S2_PWM_TOP. */
        note_error(pwm_set_cc(p_pins->pin_a, 0), &b_failed);
        note_error(pwm_set_cc(p_pins->pin_b, 0), &b_failed);
        note_error(pwm_set_wrap(p_pins->pin_a, S2_PWM_TOP), &b_failed);
        note_error(pwm_set_pin(p_pins->pin_a), &b_failed);
        note_error(pwm_set_pin(p_pins->pin_b), &b_failed);
        note_error(pwm_set_enabled(p_pins->pin_a, true), &b_failed);
    }
    return b_failed ? E_IO : E_OK;
}

void drv_motor_set(motor_id_t motor, int32_t duty)
{
    const motor_pins_t * p_pins;

    /* Cast to unsigned so a negative (invalid) id also fails the test */
    if ((uint32_t)motor >= (uint32_t)MOTOR_COUNT)
    {
        return;
    }
    p_pins = &g_motor_pins[motor];

    if (duty > S2_DUTY_MAX)
    {
        duty = S2_DUTY_MAX;
    }
    if (duty < -S2_DUTY_MAX)
    {
        duty = -S2_DUTY_MAX;
    }
    if (p_pins->invert)
    {
        duty = -duty;
    }

    if (duty >= 0)
    {
        pwm_set_cc(p_pins->pin_b, 0);
        pwm_set_cc(p_pins->pin_a, duty_to_cc(duty));
    }
    else
    {
        pwm_set_cc(p_pins->pin_a, 0);
        pwm_set_cc(p_pins->pin_b, duty_to_cc(-duty));
    }
}

void drv_motor_brake(motor_id_t motor)
{
    if ((uint32_t)motor >= (uint32_t)MOTOR_COUNT)
    {
        return;
    }
    /* cc > TOP keeps the output high for the whole period */
    pwm_set_cc(g_motor_pins[motor].pin_a, S2_PWM_TOP + 1);
    pwm_set_cc(g_motor_pins[motor].pin_b, S2_PWM_TOP + 1);
}

void drv_motor_all_off(void)
{
    drv_motor_set(MOTOR_LEFT, 0);
    drv_motor_set(MOTOR_RIGHT, 0);
}

/*** end of file ***/
