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
    p_trk->rejoin_clear_left = 0u;
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

    /* C and R can both see black where the bypass rejoins the line. Keep
       tracking the line, but wait for a clean stretch before a junction
       can be reported. A brief gap between two strips does not rearm it. */
    if (0u != p_trk->rejoin_clear_left)
    {
        const uint8_t rejoin_mask = S3_MASK_C | S3_MASK_R;

        b_junction_inhibit = true;
        if ((mask & rejoin_mask) == rejoin_mask)
        {
            p_trk->rejoin_clear_left = S3_REJOIN_CLEAR_SAMPLES;
        }
        else
        {
            p_trk->rejoin_clear_left--;
        }
    }

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
           a barcode is being read or the car is rejoining after a bypass.
           Drive straight and forget the turn so a patch of black cannot
           build up a strong steering correction. */
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
    p_e->w_valid   = 0U; /* adaptive path: learn white again */
    p_e->w_relearn = 0U;
    p_e->w_n       = 0U;
    p_e->w_min     = raw;
    p_e->w_max     = raw;
    p_e->w_sum     = 0;
    p_e->w_x16     = 0;
    p_e->w_sd_x16  = 0;
    p_e->w_high    = 0U;
    p_e->w_dsum    = 0;
    p_e->h_prev    = raw;
    p_e->h_prev_white = 0U;
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

/* The edge finder itself.  A bar starts above `enter` and is forced to
   end below `leave` (bc_edge_step_pos passes its one floor for both). */
static uint8_t bc_edge_core(bc_edge_t * p_e, int32_t raw, uint32_t t_us,
                            uint32_t pos, int32_t swing, int32_t enter,
                            int32_t leave, uint32_t * p_edge_us,
                            uint32_t * p_edge_pos)
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
            if ((raw < (p_e->ext - need)) || (raw < leave)) { next = 0U; }
        }
    }
    else
    {
        if (raw < p_e->ext) { p_e->ext = raw; }
        mid = (0U != p_e->b_prev) ? ((p_e->ext + p_e->prev) / 2)
                                  : (p_e->ext + (swing / 2));
        if (raw <= mid) { p_e->mid_t_us = t_us; p_e->mid_pos = pos; }
        if ((raw > (p_e->ext + swing)) && (raw > enter)) { next = 1U; }
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

uint8_t bc_edge_step_pos(bc_edge_t * p_e, int32_t raw, uint32_t t_us,
                         uint32_t pos, int32_t swing, int32_t floor,
                         uint32_t * p_edge_us, uint32_t * p_edge_pos)
{
    return bc_edge_core(p_e, raw, t_us, pos, swing, floor, floor, p_edge_us,
                        p_edge_pos);
}

/* ---- adaptive edge finder: thresholds from the learned white ---- */
void bc_edge_relearn(bc_edge_t * p_e)
{
    p_e->w_relearn = 1U;
}

int32_t bc_edge_margin(bc_edge_t const * p_e)
{
    if (0U == p_e->w_valid)
    {
        return 0;
    }
    {
        int32_t sd = (p_e->w_sd_x16 < (S3_BC_NOISE_SD_MIN * 16))
                         ? (S3_BC_NOISE_SD_MIN * 16)
                         : p_e->w_sd_x16;
        return ((S3_BC_NOISE_K * sd) / 16) + S3_BC_NOISE_ADD;
    }
}

/* start (or restart) the learning window at this sample */
static void bc_white_window(bc_edge_t * p_e, int32_t raw)
{
    p_e->w_n   = 1U;
    p_e->w_min = raw;
    p_e->w_max = raw;
    p_e->w_sum = raw;
    p_e->w_dsum = 0;
}

uint8_t bc_edge_step_adapt(bc_edge_t * p_e, int32_t raw, uint32_t t_us,
                           uint32_t pos, bool b_freeze, uint32_t * p_edge_us,
                           uint32_t * p_edge_pos)
{
    uint8_t changed;
    int32_t m;
    int32_t white;

    /* Cast: wrapping difference of the microsecond clock */
    if ((uint32_t)(t_us - p_e->last_t_us) > S3_BC_WHITE_STALE_US)
    {
        p_e->w_relearn = 1U; /* samples were missed: white may be stale */
    }
    if (0U != p_e->w_relearn)
    {
        uint8_t was = p_e->level;

        p_e->w_relearn = 0U;
        p_e->w_valid   = 0U;
        p_e->w_high    = 0U;
        p_e->hold_cnt  = 0U;
        bc_white_window(p_e, raw);
        if (0U != was)
        {
            /* end the open bar where L was last on its dark side */
            *p_edge_us  = p_e->mid_t_us;
            *p_edge_pos = p_e->mid_pos;
        }
        p_e->level     = 0U;
        p_e->b_prev    = 0U;
        p_e->ext       = raw;
        p_e->prev      = raw;
        p_e->mid_t_us  = t_us;
        p_e->mid_pos   = pos;
        p_e->last_raw  = raw;
        p_e->last_t_us = t_us;
        p_e->last_pos  = pos;
        return was;
    }

    if (0U == p_e->w_valid)
    {
        /* learning: a steady stretch of S3_BC_WHITE_INIT_N samples */
        if (0U == p_e->w_n)
        {
            bc_white_window(p_e, raw);
        }
        else
        {
            int32_t lo = (raw < p_e->w_min) ? raw : p_e->w_min;
            int32_t hi = (raw > p_e->w_max) ? raw : p_e->w_max;
            if ((hi - lo) > S3_BC_WHITE_INIT_SPREAD)
            {
                bc_white_window(p_e, raw); /* not steady: start again */
            }
            else
            {
                p_e->w_min = lo;
                p_e->w_max = hi;
                p_e->w_sum += raw;
                p_e->w_dsum += (raw > p_e->last_raw) ? (raw - p_e->last_raw)
                                                     : (p_e->last_raw - raw);
                p_e->w_n++;
            }
        }
        if (p_e->w_n >= S3_BC_WHITE_INIT_N)
        {
            /* Casts: w_n is at most S3_BC_WHITE_INIT_N */
            p_e->w_x16    = (p_e->w_sum * 16) / (int32_t)p_e->w_n;
            /* noise from the steps between samples, not from the spread,
               so a level that drifts or a window over faint ink cannot
               pass as noise: sd = 0.886 x mean |step| for a normal noise
               (casts: w_n - 1 >= 1 here) */
            p_e->w_sd_x16 = (p_e->w_dsum * 16 * 886) /
                            (1000 * ((int32_t)p_e->w_n - 1));
            p_e->w_valid  = 1U;
            p_e->w_high   = 0U;
            p_e->ext      = raw;
            p_e->b_prev   = 0U;
        }
        p_e->mid_t_us  = t_us;
        p_e->mid_pos   = pos;
        p_e->last_raw  = raw;
        p_e->last_t_us = t_us;
        p_e->last_pos  = pos;
        return 0U;
    }

    m       = bc_edge_margin(p_e);
    white   = p_e->w_x16 / 16;
    changed = bc_edge_core(p_e, raw, t_us, pos, m, white + m, white + (m / 2),
                           p_edge_us, p_edge_pos);

    if ((0U == p_e->level) && !b_freeze)
    {
        int32_t step = raw - p_e->h_prev;       /* from the sample before */
        int32_t d16  = (raw * 16) - p_e->w_x16; /* x 16 */
        int32_t ad16 = (d16 < 0) ? -d16 : d16;
        if (ad16 <= (m * 16))
        {
            /* white travel: follow the level slowly */
            p_e->w_x16 += d16 / (1 << S3_BC_WHITE_EMA_SHIFT);
            if ((d16 < 0) && (0U != p_e->h_prev_white))
            {
                /* and the noise, from the step between two white samples
                   below white only: ink only reads higher, so the sides
                   of faint marks cannot grow it, and an error in the
                   level (learned on ink) is not taken for noise.
                   sd of a normal noise = 0.886 x mean |step|. */
                int32_t as = (step < 0) ? -step : step;
                p_e->w_sd_x16 += (((as * 16 * 886) / 1000) - p_e->w_sd_x16) /
                                 (1 << S3_BC_WHITE_EMA_SHIFT);
            }
        }
        else if (d16 < 0)
        {
            /* far below the learned white (it was learned on ink, or
               the paper reads lower now): follow down fast */
            p_e->w_x16 += d16 / 4;
        }
        else
        {
            /* above white but not a bar: leave the estimate alone */
        }
    }
    if ((0U != p_e->level) && !b_freeze)
    {
        if (p_e->w_high < 0xFFFFU)
        {
            p_e->w_high++;
        }
        if (p_e->w_high >= S3_BC_WHITE_RELEARN_SAMPLES)
        {
            p_e->w_relearn = 1U; /* black far longer than any bar */
        }
    }
    else
    {
        p_e->w_high = 0U;
    }
    p_e->h_prev       = raw;
    p_e->h_prev_white = (0U == p_e->level) ? 1U : 0U;
    return changed;
}

/* ------------------------------------------------------------------ */
/*  Barcode lock (see line_algo.h)                                     */
/* ------------------------------------------------------------------ */

void bc_lock_reset(bc_lock_t * p_l)
{
    p_l->b_lock      = false;
    p_l->b_confirmed = false;
    p_l->b_was_scan  = false;
    p_l->b_after     = false;
    p_l->start_pos   = 0U;
    p_l->read_pos    = 0U;
    p_l->end_pos     = 0U;
}

bool bc_lock_step(bc_lock_t * p_l, bool b_scan, bool b_bars, uint32_t pos,
                  uint32_t first_max, uint32_t after, uint32_t max_counts)
{
    if (b_scan && !p_l->b_was_scan)
    {
        /* a read has just started (first bar) */
        if (!p_l->b_lock)
        {
            p_l->b_lock      = true;
            p_l->b_confirmed = false;
            p_l->start_pos   = pos;
        }
        p_l->read_pos = pos;
        p_l->b_after  = false;
    }
    p_l->b_was_scan = b_scan;

    if (p_l->b_lock)
    {
        bool b_free = false;

        if (b_scan && b_bars)
        {
            p_l->b_confirmed = true;
        }
        if (b_scan)
        {
            /* Casts: wrapping differences of up-counters */
            if (!b_bars && !p_l->b_confirmed &&
                ((uint32_t)(pos - p_l->read_pos) >= first_max))
            {
                b_free = true; /* one long black: not a barcode */
            }
        }
        else
        {
            if (!p_l->b_after)
            {
                p_l->b_after = true;
                p_l->end_pos = pos;
            }
            if (!p_l->b_confirmed ||
                ((uint32_t)(pos - p_l->end_pos) >= after))
            {
                b_free = true;
            }
        }
        if ((uint32_t)(pos - p_l->start_pos) >= max_counts)
        {
            b_free = true;
        }
        if (b_free)
        {
            /* relocks only when the NEXT read starts */
            p_l->b_lock      = false;
            p_l->b_confirmed = false;
            p_l->b_after     = false;
        }
    }
    return p_l->b_lock;
}

void line_tracker_go_straight(line_tracker_t * p_trk)
{
    p_trk->steering   = 0;
    p_trk->b_hold     = false;
    p_trk->b_reentry  = false;
    p_trk->hold_count = 0u;
    p_trk->corr_count = 0u;
    p_trk->corr_mag   = 0;
    p_trk->mem_side   = 0;
}

void bc_lock_steer_reset(bc_lock_steer_t * p_s)
{
    p_s->side     = 0;
    p_s->side_pos = 0U;
}

int16_t bc_lock_steer(bc_lock_steer_t * p_s, uint8_t black_mask, uint32_t pos,
                      int16_t steering, uint32_t side_counts,
                      int16_t max_steer)
{
    bool   b_c  = (0u != (black_mask & S3_MASK_C));
    bool   b_r  = (0u != (black_mask & S3_MASK_R));
    int8_t side = (b_c && !b_r) ? -1 : ((b_r && !b_c) ? 1 : 0);

    if (side != p_s->side)
    {
        p_s->side     = side;
        p_s->side_pos = pos;
    }
    /* Cast: wrapping difference of an up-counter */
    if ((0 == side) || ((uint32_t)(pos - p_s->side_pos) < side_counts))
    {
        return 0;
    }
    if (steering > max_steer)
    {
        return max_steer;
    }
    if (steering < -max_steer)
    {
        return (int16_t)(-max_steer);
    }
    return steering;
}

void bc_lock_ctl_reset(bc_lock_ctl_t * p_c)
{
    bc_lock_reset(&p_c->lock);
    bc_lock_steer_reset(&p_c->steer);
}

bool bc_lock_apply(bc_lock_ctl_t * p_c, bc_lock_cfg_t const * p_cfg,
                   line_tracker_t * p_trk, bool b_scan, bool b_bars,
                   uint32_t pos)
{
    bool b_lock = bc_lock_step(&p_c->lock, b_scan, b_bars, pos,
                               p_cfg->first_max, p_cfg->after,
                               p_cfg->max_counts);

    if ((!b_lock) || (!p_c->lock.b_confirmed) || p_trk->b_hold)
    {
        /* free, unconfirmed (speed cap only), or a curve is being held:
           leave the tracker's steering and its curve memory alone.  The
           side timer restarts so a held curve is not counted as drift. */
        bc_lock_steer_reset(&p_c->steer);
    }
    else
    {
        int16_t st = bc_lock_steer(&p_c->steer, p_trk->black_mask, pos,
                                   p_trk->steering, p_cfg->side_counts,
                                   p_cfg->max_steer);
        if (0 == st)
        {
            /* not holding a curve (checked above), so this only drops a
               turn built up on a bar edge */
            line_tracker_go_straight(p_trk);
        }
        p_trk->steering = st;
    }
    return b_lock;
}

/* ---- 500 Hz L front end (S3_L_FAST) -------------------------------- */

/* Full barrier: orders the counter and the data across the two cores
   (GCC emits DMB on the Cortex-M0+). */
#define L_FAST_BARRIER() __sync_synchronize()
#define L_FAST_TRIES     8u

void l_fast_reset(l_fast_t * p_f, l_fast_rd_t * p_rd)
{
    p_f->seq  = 0u;
    p_f->sum  = 0u;
    p_f->n    = 0u;
    p_f->c    = 0u;
    p_f->r    = 0u;
    p_rd->sum  = 0u;
    p_rd->n    = 0u;
    p_rd->last = 0u;
}

void l_fast_put(l_fast_t * p_f, uint16_t l, uint16_t c, uint16_t r)
{
    p_f->seq = p_f->seq + 1u; /* odd: update in progress */
    L_FAST_BARRIER();
    p_f->sum = p_f->sum + (uint32_t)l;
    p_f->n   = p_f->n + 1u;
    p_f->c   = c;
    p_f->r   = r;
    L_FAST_BARRIER();
    p_f->seq = p_f->seq + 1u; /* even: done */
}

bool l_fast_get(const l_fast_t * p_f, uint32_t * p_sum, uint32_t * p_n,
                uint16_t * p_c, uint16_t * p_r)
{
    uint32_t k;

    for (k = 0u; k < L_FAST_TRIES; k++)
    {
        uint32_t s1 = p_f->seq;
        L_FAST_BARRIER();
        *p_sum = p_f->sum;
        *p_n   = p_f->n;
        *p_c   = p_f->c;
        *p_r   = p_f->r;
        L_FAST_BARRIER();
        if ((0u == (s1 & 1u)) && (s1 == p_f->seq))
        {
            return true;
        }
    }
    return false;
}

uint16_t l_fast_record(l_fast_rd_t * p_rd, uint32_t sum, uint32_t n,
                       uint32_t * p_used)
{
    uint32_t dn = n - p_rd->n;     /* wraps correctly */
    uint32_t ds = sum - p_rd->sum; /* wraps correctly */

    *p_used = dn;
    if (0u != dn)
    {
        /* Cast: a mean of 12-bit samples fits 16 bits */
        p_rd->last = (uint16_t)((ds + (dn / 2u)) / dn);
    }
    p_rd->sum = sum;
    p_rd->n   = n;
    return p_rd->last;
}

/*** end of file ***/
