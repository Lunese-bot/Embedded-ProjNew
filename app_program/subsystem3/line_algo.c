/*
 *      line_algo.c
 *      Subsystem 3 -- line-position estimation, steering and line state
 *      (see line_algo.h).
 *
 *      Line state machine (Design Review 6.2, line_state_t in robot_types.h):
 *
 *        INIT --reset--> SEARCH --line seen FOUND_SAMPLES--> FOLLOW
 *        FOLLOW --no line LOST_SAMPLES--> LOST --line seen--> FOLLOW
 *        FOLLOW --all JUNCTION_MASK black JUNCTION_SAMPLES--> JUNCTION
 *        JUNCTION --not all black--> FOLLOW
 *        (CALIBRATE is set by task_line.c while a calibration runs)
 */

#include "line_algo.h"

#define ADC_FULL_SCALE 4095u

static int16_t clamp16(int32_t value, int16_t limit)
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

void line_params_default(line_params_t * p_params)
{
    p_params->line_mask        = (uint8_t)S3_LINE_MASK;
    p_params->junction_mask    = (uint8_t)S3_JUNCTION_MASK;
    p_params->weight[S3_IR_L]  = S3_WEIGHT_L;
#if (S3_LINE_MASK) == (S3_MASK_L | S3_MASK_C | S3_MASK_R)
    p_params->weight[S3_IR_C]  = S3_WEIGHT_C;
#else
    p_params->weight[S3_IR_C]  = S3_WEIGHT_C_CR;
#endif
    p_params->weight[S3_IR_R]  = S3_WEIGHT_R;
    p_params->setpoint         = S3_LINE_SETPOINT;
    p_params->black_threshold  = S3_BLACK_THRESHOLD;
    p_params->lost_samples     = S3_LOST_SAMPLES;
    p_params->found_samples    = S3_FOUND_SAMPLES;
    p_params->junction_samples = S3_JUNCTION_SAMPLES;
    p_params->kp_num           = S3_KP_NUM;
    p_params->kd_num           = S3_KD_NUM;
    p_params->gain_den         = S3_GAIN_DEN;
    p_params->junction_threshold = S3_JUNCTION_THRESHOLD;
    p_params->straddle         = (0 != S3_STRADDLE);
}

void line_cal_reset(line_cal_t * p_cal)
{
    uint8_t idx;

    for (idx = 0u; idx < S3_IR_COUNT; idx++)
    {
        p_cal->min[idx] = UINT16_MAX;
        p_cal->max[idx] = 0u;
    }
    p_cal->b_valid = false;
}

void line_cal_update(line_cal_t * p_cal, uint16_t const p_raw[S3_IR_COUNT])
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

bool line_cal_finish(line_cal_t * p_cal, uint8_t used_mask, uint16_t min_span)
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

void line_normalise(line_cal_t const * p_cal, uint8_t analog_mask,
                    bool b_black_is_high, uint8_t digital_black_level,
                    uint16_t const p_raw[S3_IR_COUNT],
                    uint16_t       p_norm[S3_IR_COUNT])
{
    uint8_t  idx;
    uint32_t low;
    uint32_t high;
    uint32_t value;
    uint32_t norm;

    for (idx = 0u; idx < S3_IR_COUNT; idx++)
    {
        if (0u == (analog_mask & (1u << idx)))
        {
            /* Digital sensor */
            p_norm[idx] = (p_raw[idx] == digital_black_level)
                              ? (uint16_t)LINE_NORM_MAX
                              : 0u;
        }
        else
        {
            bool b_hi = b_black_is_high;

            if ((NULL != p_cal) && (p_cal->b_valid))
            {
                low  = p_cal->min[idx];
                high = p_cal->max[idx];
            }
            else
            {
                /* not calibrated: use the sensor's measured white / black
                   readings from s3_config.h (either order) */
                uint32_t w = (S3_IR_R == idx) ? (uint32_t)S3_R_ADC_WHITE
                           : ((S3_IR_C == idx) ? (uint32_t)S3_C_ADC_WHITE
                                               : (uint32_t)S3_L_ADC_WHITE);
                uint32_t k = (S3_IR_R == idx) ? (uint32_t)S3_R_ADC_BLACK
                           : ((S3_IR_C == idx) ? (uint32_t)S3_C_ADC_BLACK
                                               : (uint32_t)S3_L_ADC_BLACK);
                low  = (w < k) ? w : k;
                high = (w < k) ? k : w;
                b_hi = (k >= w);
            }

            value = p_raw[idx];
            if (value < low)
            {
                value = low;
            }
            if (value > high)
            {
                value = high;
            }

            norm = (high > low)
                       ? (((value - low) * LINE_NORM_MAX) / (high - low))
                       : 0u;
            if (!b_hi)
            {
                norm = LINE_NORM_MAX - norm;
            }
            p_norm[idx] = (uint16_t)norm;
        }
    }
}

void line_tracker_reset(line_tracker_t * p_trk)
{
    p_trk->state          = LINE_STATE_SEARCH;
    p_trk->position       = 0;
    p_trk->error          = 0;
    p_trk->steering       = 0;
    p_trk->last_side      = 0;
    p_trk->black_mask     = 0u;
    p_trk->l_raw          = 0u;
    p_trk->lost_count     = 0u;
    p_trk->found_count    = 0u;
    p_trk->junction_count = 0u;
    p_trk->prev_error     = 0;
    p_trk->corr_side      = 0;
    p_trk->corr_count     = 0u;
    p_trk->corr_mag       = 0;
    p_trk->hold_count     = 0u;
    p_trk->b_hold         = false;
    p_trk->b_reentry      = false;
    p_trk->mem_side       = 0;
}

/* Step 4 of line_tracker_step(): the line state machine */
static line_evt_t tracker_update_state(line_tracker_t *      p_trk,
                                       line_params_t const * p_params,
                                       bool                  b_junction)
{
    line_evt_t evt = LINE_EVT_NONE;

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
                evt          = LINE_EVT_FOUND;
            }
            break;

        case LINE_STATE_FOLLOW:
            if (p_trk->junction_count >= p_params->junction_samples)
            {
                p_trk->state = LINE_STATE_JUNCTION;
                evt          = LINE_EVT_JUNCTION;
            }
            else if ((p_trk->lost_count >= p_params->lost_samples) &&
                     ((!p_params->straddle) || (0 != S3_STRADDLE_LOST)))
            {
                p_trk->state = LINE_STATE_LOST;
                evt          = LINE_EVT_LOST;
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

line_evt_t line_tracker_step(line_tracker_t *      p_trk,
                             line_params_t const * p_params,
                             uint16_t const        p_norm[S3_IR_COUNT],
                             bool                  b_junction_inhibit)
{
    int32_t sum_w = 0;
    int32_t sum_n = 0;
    int32_t steer;
    uint8_t idx;
    uint8_t mask = 0u;
    uint8_t jmask = 0u; /* sensors above the (stricter) junction threshold */
    bool    b_line_seen;
    bool    b_found;
    bool    b_junction;

    /* 1. Which sensors see black, and the weighted line position */
    for (idx = 0u; idx < S3_IR_COUNT; idx++)
    {
        if (p_norm[idx] >= p_params->black_threshold)
        {
            mask |= (uint8_t)(1u << idx);
        }
        if (p_norm[idx] >= p_params->junction_threshold)
        {
            jmask |= (uint8_t)(1u << idx);
        }
        if (0u != (p_params->line_mask & (1u << idx)))
        {
            sum_w += (int32_t)p_params->weight[idx] * (int32_t)p_norm[idx];
            sum_n += (int32_t)p_norm[idx];
        }
    }
    p_trk->black_mask = mask;

    b_line_seen = (0u != (mask & p_params->line_mask));
    b_junction =
        (0u != p_params->junction_mask) && /* 0 = junction detection off */
        ((jmask & p_params->junction_mask) == p_params->junction_mask) &&
        (!b_junction_inhibit);

    if (p_params->straddle)
    {
        /* Line between C and R: position = R - C (weights -1000 / +1000).
           Both white = 0 (straight), C black = left, R black = right. */
        p_trk->position = clamp16(sum_w / (int32_t)LINE_NORM_MAX, LINE_POS_MAX);
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
    else if (b_line_seen && (sum_n > 0))
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

    /* 2. Steering */
    p_trk->error = clamp16((int32_t)p_trk->position - p_params->setpoint,
                           2 * LINE_POS_MAX);
    if (p_params->straddle)
    {
        /* Straddle: correct ONLY while a sensor sees black, and stop the
           moment both are white again, so the robot never keeps turning
           past the line (no over-correction).
             C black only -> steer left, R black only -> steer right,
             both white   -> 0 (straight),
             both black   -> keep the current correction (junction off).
           Strength = a gentle start (kp_num * S3_STRADDLE_BASE) that grows
           by kd_num per 10 ms sample while that sensor stays black, so a
           short touch gives a small nudge and a sharp curve a strong turn. */
        bool    b_c  = (0u != (mask & S3_MASK_C));
        bool    b_r  = (0u != (mask & S3_MASK_R));
        int8_t  side = (b_c && !b_r) ? -1 : ((b_r && !b_c) ? 1 : 0);
        int32_t mag;
        /* L black while a left turn is going on (or remembered): the line
           is further left than C */
        bool    b_far = (0 != S3_STRADDLE_L_ASSIST) &&
                        (0u != (mask & S3_MASK_L)) && (!b_r) &&
                        ((p_trk->corr_side < 0) ||
                         ((p_trk->mem_side < 0) && (p_trk->corr_count > 0u)));

        /* Barcode bar across the line: L, C and R all black at once (L
           judged on its RAW reading, since its cut-off is set low for
           barcodes and white paper can cross it), or C and R black while
           a barcode is being read.  Drive straight and
           forget the turn, so a thick bar does not build up a big
           correction (or start the curve memory) and yank the robot. */
        bool    b_bar = (0 != S3_STRADDLE_BAR_STRAIGHT) && b_c && b_r &&
                        ((p_trk->l_raw >= (uint16_t)S3_BAR_L_RAW) ||
                         b_junction_inhibit);

        if (b_c && b_r && !b_bar)
        {
            side = p_trk->corr_side; /* both black: hold */
        }
        if (b_bar)
        {
            p_trk->b_hold     = false;
            p_trk->b_reentry  = false;
            p_trk->hold_count = 0u;
            p_trk->corr_count = 0u;
            p_trk->corr_mag   = 0;
            p_trk->mem_side   = 0;
            side              = 0;
            steer             = 0;
        }
        else if ((0 == side) && b_far)
        {
            /* only L black: the line is outside on the left.  Turn fully
               and keep turning (curve memory) until it is back under C. */
            p_trk->b_hold     = true;
            p_trk->hold_count = 0u;
            p_trk->b_reentry  = false;
            p_trk->corr_mag   = (int16_t)LINE_STEER_MAX;
            p_trk->mem_side   = -1;
            side              = -1;
            steer             = -(int32_t)LINE_STEER_MAX;
        }
        else if ((0 == side) && p_trk->b_hold)
        {
            /* Curve memory: the line slipped past the sensor to the
               outside of a tight curve, so both read white although the
               robot is NOT centred.  Keep turning the same way until the
               line comes back under a sensor, or for at most
               S3_STRADDLE_HOLD_MAX samples. */
            p_trk->hold_count++;
            if (p_trk->hold_count > S3_STRADDLE_HOLD_MAX)
            {
                /* gave up: drive straight, no new hold until a sensor
                   sees black again */
                p_trk->b_hold     = false;
                p_trk->corr_count = 0u;
                steer             = 0;
                side              = 0;
            }
            else
            {
                steer = (int32_t)p_trk->corr_side * p_trk->corr_mag;
                side  = p_trk->corr_side; /* still correcting this side */
            }
        }
        else if (0 == side)
        {
            /* A HARD correction that ends on white means the line went
               outside (a sharp curve): start the curve memory.  A short
               touch, or the line coming back in after a hold, means the
               robot is centred again: go straight at once. */
            if ((0 != p_trk->corr_side) && (!p_trk->b_reentry) &&
                (S3_STRADDLE_HOLD_MAX > 0) &&
                (p_trk->corr_mag >= S3_STRADDLE_HOLD_MAG))
            {
                p_trk->b_hold     = true;
                p_trk->hold_count = 0u;
                steer             = (int32_t)p_trk->corr_side * p_trk->corr_mag;
                side              = p_trk->corr_side;
            }
            else
            {
                /* straight now, but remember the turn for a short while
                   (S3_STRADDLE_DECAY per sample, 0 = forget at once) */
                p_trk->b_reentry = false;
                if ((0 == S3_STRADDLE_DECAY) ||
                    (p_trk->corr_count <= (uint16_t)S3_STRADDLE_DECAY))
                {
                    p_trk->corr_count = 0u;
                }
                else
                {
                    p_trk->corr_count =
                        (uint16_t)(p_trk->corr_count - S3_STRADDLE_DECAY);
                }
                steer = 0;
            }
        }
        else
        {
            if (p_trk->b_hold)
            {
                /* the line is back under a sensor: the hold is over.  Same
                   side = it re-enters from outside, so the next white means
                   centred (no new hold). */
                p_trk->b_hold    = false;
                p_trk->b_reentry = (side == p_trk->corr_side);
            }
            if ((side != p_trk->mem_side) ||
                ((0u == p_trk->corr_count) && (side != p_trk->corr_side)))
            {
                p_trk->corr_count = 0u; /* other side: start gently */
                p_trk->b_reentry  = false;
            }
            else if (p_trk->corr_count < 1000u)
            {
                p_trk->corr_count++; /* same side: carry on growing */
            }
            p_trk->mem_side = side;
            mag = ((int32_t)p_params->kp_num * S3_STRADDLE_BASE) +
                  ((int32_t)p_params->kd_num * (int32_t)p_trk->corr_count);
            if (b_far && (side < 0))
            {
                mag = LINE_STEER_MAX; /* L and C black: line far left */
            }
            mag   = (mag > LINE_STEER_MAX) ? LINE_STEER_MAX : ((mag < 0) ? 0 : mag);
            steer = (int32_t)side * mag;
            p_trk->corr_mag  = (int16_t)mag;
            p_trk->last_side = side;
        }
        p_trk->corr_side = side;
    }
    else
    {
        /* PD steering on the error */
        steer = ((int32_t)p_params->kp_num * p_trk->error) +
                ((int32_t)p_params->kd_num * (p_trk->error - p_trk->prev_error));
        if (p_params->gain_den != 0)
        {
            steer /= p_params->gain_den;
        }
    }
    p_trk->steering   = clamp16(steer, LINE_STEER_MAX);
    p_trk->prev_error = p_trk->error;

    /* 3. Debounce counters */
    p_trk->lost_count  = b_line_seen ? 0u : (uint16_t)(p_trk->lost_count + 1u);
    /* Straddle: both white is a valid "on the line" reading while searching,
       but to leave LOST a sensor must see black again. */
    b_found = p_params->straddle
                  ? (b_line_seen || (LINE_STATE_SEARCH == p_trk->state))
                  : b_line_seen;
    p_trk->found_count = b_found ? (uint16_t)(p_trk->found_count + 1u) : 0u;
    p_trk->junction_count =
        b_junction ? (uint16_t)(p_trk->junction_count + 1u) : 0u;

    /* 4. State machine */
    return tracker_update_state(p_trk, p_params, b_junction);
}

/* ---- barcode edge finder (see line_algo.h) ---- */
void bc_edge_reset_pos(bc_edge_t * p_e, int32_t raw, uint32_t t_us,
                       uint32_t pos)
{
    p_e->level    = 0U;
    p_e->ext      = raw;
    p_e->prev     = raw;
    p_e->b_prev   = 0U;
    p_e->mid_t_us = t_us;
    p_e->mid_pos  = pos;
    p_e->bar_t_us  = t_us;
    p_e->bar_pos   = pos;
    p_e->last_raw  = raw;
    p_e->last_t_us = t_us;
    p_e->last_pos  = pos;
    p_e->hold      = 0U; /* the caller sets it again if it wants a hold */
    p_e->hold_cnt  = 0U;
}

void bc_edge_reset(bc_edge_t * p_e, int32_t raw, uint32_t t_us)
{
    bc_edge_reset_pos(p_e, raw, t_us, 0U);
}

uint8_t bc_edge_step(bc_edge_t * p_e, int32_t raw, uint32_t t_us,
                     int32_t swing, int32_t floor, uint32_t * p_edge_us)
{
    uint32_t edge_pos = 0U;

    return bc_edge_step_pos(p_e, raw, t_us, 0U, swing, floor, p_edge_us,
                            &edge_pos);
}

uint8_t bc_edge_step_pos(bc_edge_t * p_e, int32_t raw, uint32_t t_us,
                         uint32_t pos, int32_t swing, int32_t floor,
                         uint32_t * p_edge_us, uint32_t * p_edge_pos)
{
    uint8_t next = p_e->level;
    int32_t mid;

    /* halfway between this run's extreme and the previous one; it only
       moves towards the current sample's side as ext grows, so "last time
       on this side of halfway" stays correct */
    if (0U != p_e->level)
    {
        if (raw > p_e->ext) { p_e->ext = raw; }
        mid = (p_e->ext + p_e->prev) / 2;
        if (raw >= mid) { p_e->mid_t_us = t_us; p_e->mid_pos = pos; }
        if ((raw >= p_e->ext) && (p_e->last_raw < mid))
        {
            /* still rising: the sample before this one was the last one
               below halfway, so the bar starts there */
            p_e->bar_t_us = p_e->last_t_us;
            p_e->bar_pos  = p_e->last_pos;
        }
        {
            /* the fall that ends a bar grows with the bar's height, so a
               wobble on top of a dark bar does not end it */
            int32_t need = ((p_e->ext - p_e->prev) * S3_BC_EDGE_FALL_PCT) / 100;
            if (need < swing) { need = swing; }
            if ((raw < (p_e->ext - need)) || (raw < floor)) { next = 0U; }
        }
    }
    else
    {
        if (raw < p_e->ext) { p_e->ext = raw; }
        mid = (0U != p_e->b_prev) ? ((p_e->ext + p_e->prev) / 2)
                                  : (p_e->ext + (swing / 2));
        if (raw <= mid) { p_e->mid_t_us = t_us; p_e->mid_pos = pos; }
        if ((raw > (p_e->ext + swing)) && (raw > floor)) { next = 1U; }
    }

    if (next == p_e->level)
    {
        p_e->hold_cnt = 0U;
    }
    else if (p_e->hold > 1U)
    {
        /* weak print: wait for `hold` samples in a row before believing it */
        p_e->hold_cnt++;
        if (p_e->hold_cnt < p_e->hold)
        {
            next = p_e->level;
        }
    }
    else
    {
        /* change at once */
    }

    if (next == p_e->level)
    {
        p_e->last_raw  = raw;
        p_e->last_t_us = t_us;
        p_e->last_pos  = pos;
        return 0U;
    }
    *p_edge_us    = p_e->mid_t_us;
    *p_edge_pos   = p_e->mid_pos;
    if (0U != next)
    {
        /* a bar begins: it starts at the edge just found, or at the
           sample before this one if that was still below halfway to this
           sample; refined above as the bar's real peak becomes known */
        p_e->bar_t_us = p_e->mid_t_us;
        p_e->bar_pos  = p_e->mid_pos;
        if (p_e->last_raw < ((raw + p_e->ext) / 2))
        {
            p_e->bar_t_us = p_e->last_t_us;
            p_e->bar_pos  = p_e->last_pos;
        }
    }
    p_e->hold_cnt = 0U;
    p_e->level    = next;
    p_e->prev     = p_e->ext;
    p_e->b_prev   = 1U;
    p_e->ext      = raw;
    p_e->mid_t_us = t_us;
    p_e->mid_pos  = pos;
    p_e->last_raw  = raw;
    p_e->last_t_us = t_us;
    p_e->last_pos  = pos;
    return 1U;
}

/*** end of file ***/
