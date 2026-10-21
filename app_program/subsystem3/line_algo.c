/*
 *	line_algo.c
 *	Subsystem 3 -- line-position estimation, steering and line state
 *	(see line_algo.h).
 *
 *	Line state machine (Design Review 6.2, LineState in robot_types.h):
 *
 *	  INIT --reset--> SEARCH --line seen FOUND_SAMPLES--> FOLLOW
 *	  FOLLOW --no line LOST_SAMPLES--> LOST --line seen--> FOLLOW
 *	  FOLLOW --all JUNCTION_MASK black JUNCTION_SAMPLES--> JUNCTION
 *	  JUNCTION --not all black--> FOLLOW
 *	  (CALIBRATE is set by task_line.c while a calibration runs)
 */

#include "line_algo.h"

#define ADC_FULL_SCALE  4095u

static int16_t
clamp16 (int32_t value, int16_t limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return (int16_t)(-limit);
    }
    return (int16_t)value;
}

void
line_params_default (line_params_t * p_params)
{
    p_params->line_mask        = (uint8_t)S3_LINE_MASK;
    p_params->junction_mask    = (uint8_t)S3_JUNCTION_MASK;
    p_params->weight[S3_IR_L]  = S3_WEIGHT_L;
    p_params->weight[S3_IR_C]  = S3_WEIGHT_C;
    p_params->weight[S3_IR_R]  = S3_WEIGHT_R;
    p_params->setpoint         = S3_LINE_SETPOINT;
    p_params->black_threshold  = S3_BLACK_THRESHOLD;
    p_params->lost_samples     = S3_LOST_SAMPLES;
    p_params->found_samples    = S3_FOUND_SAMPLES;
    p_params->junction_samples = S3_JUNCTION_SAMPLES;
    p_params->kp_num           = S3_KP_NUM;
    p_params->kd_num           = S3_KD_NUM;
    p_params->gain_den         = S3_GAIN_DEN;
}

void
line_cal_reset (line_cal_t * p_cal)
{
    uint8_t idx;

    for (idx = 0u; idx < S3_IR_COUNT; idx++)
    {
        p_cal->min[idx] = UINT16_MAX;
        p_cal->max[idx] = 0u;
    }
    p_cal->b_valid = false;
}

void
line_cal_update (line_cal_t * p_cal, uint16_t const p_raw[S3_IR_COUNT])
{
    uint8_t idx;

    for (idx = 0u; idx < S3_IR_COUNT; idx++)
    {
        if (p_raw[idx] < p_cal->min[idx])
        {
            p_cal->min[idx] = p_raw[idx];
        }
        if (p_raw[idx] > p_cal->max[idx])
        {
            p_cal->max[idx] = p_raw[idx];
        }
    }
}

bool
line_cal_finish (line_cal_t * p_cal, uint8_t used_mask, uint16_t min_span)
{
    uint8_t idx;
    bool    b_ok = true;

    for (idx = 0u; idx < S3_IR_COUNT; idx++)
    {
        if (0u != (used_mask & (1u << idx)))
        {
            if ((p_cal->max[idx] < p_cal->min[idx]) ||
                ((uint16_t)(p_cal->max[idx] - p_cal->min[idx]) < min_span))
            {
                b_ok = false;
            }
        }
    }

    p_cal->b_valid = b_ok;
    return b_ok;
}

void
line_normalise (line_cal_t const * p_cal, uint8_t analog_mask,
                bool b_black_is_high, uint8_t digital_black_level,
                uint16_t const p_raw[S3_IR_COUNT],
                uint16_t p_norm[S3_IR_COUNT])
{
    uint8_t  idx;
    uint32_t lo;
    uint32_t hi;
    uint32_t value;
    uint32_t norm;

    for (idx = 0u; idx < S3_IR_COUNT; idx++)
    {
        if (0u == (analog_mask & (1u << idx)))
        {
            /* Digital sensor */
            p_norm[idx] = (p_raw[idx] == digital_black_level)
                          ? (uint16_t)LINE_NORM_MAX : 0u;
            continue;
        }

        if ((NULL != p_cal) && p_cal->b_valid)
        {
            lo = p_cal->min[idx];
            hi = p_cal->max[idx];
        }
        else
        {
            lo = 0u;
            hi = ADC_FULL_SCALE;
        }

        value = p_raw[idx];
        if (value < lo)
        {
            value = lo;
        }
        if (value > hi)
        {
            value = hi;
        }

        norm = (hi > lo) ? (((value - lo) * LINE_NORM_MAX) / (hi - lo)) : 0u;
        if (!b_black_is_high)
        {
            norm = LINE_NORM_MAX - norm;
        }
        p_norm[idx] = (uint16_t)norm;
    }
}

void
line_tracker_reset (line_tracker_t * p_trk)
{
    p_trk->state          = LINE_STATE_SEARCH;
    p_trk->position       = 0;
    p_trk->error          = 0;
    p_trk->steering       = 0;
    p_trk->last_side      = 0;
    p_trk->black_mask     = 0u;
    p_trk->lost_count     = 0u;
    p_trk->found_count    = 0u;
    p_trk->junction_count = 0u;
    p_trk->prev_error     = 0;
}

line_evt_t
line_tracker_step (line_tracker_t * p_trk, line_params_t const * p_params,
                   uint16_t const p_norm[S3_IR_COUNT],
                   bool b_junction_inhibit)
{
    int32_t    sum_w = 0;
    int32_t    sum_n = 0;
    int32_t    steer;
    uint8_t    idx;
    uint8_t    mask = 0u;
    bool       b_line_seen;
    bool       b_junction;
    line_evt_t evt = LINE_EVT_NONE;

    /* 1. Which sensors see black, and the weighted line position */
    for (idx = 0u; idx < S3_IR_COUNT; idx++)
    {
        if (p_norm[idx] >= p_params->black_threshold)
        {
            mask |= (uint8_t)(1u << idx);
        }
        if (0u != (p_params->line_mask & (1u << idx)))
        {
            sum_w += (int32_t)p_params->weight[idx] * (int32_t)p_norm[idx];
            sum_n += (int32_t)p_norm[idx];
        }
    }
    p_trk->black_mask = mask;

    b_line_seen = (0u != (mask & p_params->line_mask));
    b_junction  = ((mask & p_params->junction_mask) ==
                   p_params->junction_mask) && !b_junction_inhibit;

    if (b_line_seen && (sum_n > 0))
    {
        p_trk->position = clamp16(sum_w / sum_n, LINE_POS_MAX);
        if (p_trk->position < p_params->setpoint)
        {
            p_trk->last_side = -1;
        }
        else if (p_trk->position > p_params->setpoint)
        {
            p_trk->last_side = 1;
        }
        else
        {
            /* centred: keep the last side */
        }
    }
    else
    {
        /* No line: assume it is past the side it was last seen on */
        p_trk->position = (int16_t)(p_trk->last_side * LINE_POS_MAX);
    }

    /* 2. PD steering on the error */
    p_trk->error = clamp16((int32_t)p_trk->position - p_params->setpoint,
                           2 * LINE_POS_MAX);
    steer = ((int32_t)p_params->kp_num * p_trk->error) +
            ((int32_t)p_params->kd_num * (p_trk->error - p_trk->prev_error));
    if (p_params->gain_den != 0)
    {
        steer /= p_params->gain_den;
    }
    p_trk->steering   = clamp16(steer, LINE_STEER_MAX);
    p_trk->prev_error = p_trk->error;

    /* 3. Debounce counters */
    p_trk->lost_count     = b_line_seen ? 0u : (uint16_t)(p_trk->lost_count + 1u);
    p_trk->found_count    = b_line_seen ? (uint16_t)(p_trk->found_count + 1u) : 0u;
    p_trk->junction_count = b_junction ? (uint16_t)(p_trk->junction_count + 1u) : 0u;

    /* 4. State machine */
    switch (p_trk->state)
    {
        case LINE_STATE_INIT:
        case LINE_STATE_CALIBRATE:
            /* left by task_line.c calling line_tracker_reset() */
            break;

        case LINE_STATE_SEARCH:
        case LINE_STATE_LOST:
            if (p_trk->found_count >= p_params->found_samples)
            {
                p_trk->state = LINE_STATE_FOLLOW;
                evt = LINE_EVT_FOUND;
            }
            break;

        case LINE_STATE_FOLLOW:
            if (p_trk->junction_count >= p_params->junction_samples)
            {
                p_trk->state = LINE_STATE_JUNCTION;
                evt = LINE_EVT_JUNCTION;
            }
            else if (p_trk->lost_count >= p_params->lost_samples)
            {
                p_trk->state = LINE_STATE_LOST;
                evt = LINE_EVT_LOST;
            }
            else
            {
                /* keep following */
            }
            break;

        case LINE_STATE_JUNCTION:
            if (!b_junction)
            {
                p_trk->state = LINE_STATE_FOLLOW;
            }
            break;

        default:
            p_trk->state = LINE_STATE_SEARCH;
            break;
    }

    return evt;
}
