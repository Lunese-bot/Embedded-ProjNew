/*
 *      obstacle_logic.c
 *      Subsystem 5 -- obstacle profiling and bypass planning (no kernel).
 *
 *      Geometry, seen from above, sensor at the origin, x = forward:
 *      a sample (deg, mm) is a point mm * (cos deg, sin deg).  Its lateral
 *      offset is mm * sin(deg): positive = left of the car's centre line.
 *
 *      Bypass on side s (+1 left, -1 right), from the stopped car:
 *          1. turn s*35 and drive a short diagonal away from the line
 *          2. turn to s*45 and complete the sideways offset L
 *          3. turn -s*45 and drive parallel to pass the obstacle
 *          4. turn -s*40 and search for the line
 *          5. return to the original heading and check the line
 * L = lateral offset of the obstacle edge + half car width + margin.
 */

#include "obstacle_logic.h"

/* sin(0, 5, 10 ... 90 deg) * 1000 */
static const uint16_t g_sin_tab[19] = { 0,   87,  174, 259, 342, 423, 500,
                                        574, 643, 707, 766, 819, 866, 906,
                                        940, 966, 985, 996, 1000 };

int32_t s5_sin1000(int32_t deg)
{
    int32_t sign = 1;
    int32_t idx;
    int32_t result;

    if (deg < 0)
    {
        sign = -1;
        deg  = -deg;
    }
    if (deg > 90)
    {
        deg = 90;
    }
    idx    = deg / 5;
    result = deg % 5;
    if (idx >= 18)
    {
        return sign * 1000;
    }
    return sign * (g_sin_tab[idx] +
                   (((g_sin_tab[idx + 1] - g_sin_tab[idx]) * result) / 5));
}

static bool is_hit(const s5_sample_t * p_sample)
{
    return ((p_sample->mm != 0) && (p_sample->mm <= S5_HIT_RANGE_MM)) ? true
                                                                      : false;
}

int32_t s5_merge_samples(const s5_sample_t * p_a_val, int32_t na_val,
                         const s5_sample_t * p_b_val, int32_t nb_val,
                         s5_sample_t * p_output, int32_t max)
{
    int32_t idx2   = 0;
    int32_t jdx    = 0;
    int32_t count2 = 0;
    while ((count2 < max) && ((idx2 < na_val) || (jdx < nb_val)))
    {
        if ((jdx >= nb_val) ||
            ((idx2 < na_val) && (p_a_val[idx2].deg < p_b_val[jdx].deg)))
        {
            p_output[count2++] = p_a_val[idx2++];
        }
        else
        {
            if ((idx2 < na_val) && (p_a_val[idx2].deg == p_b_val[jdx].deg))
            {
                idx2++;
            }
            p_output[count2++] = p_b_val[jdx++];
        }
    }
    return count2;
}

static int32_t abs_i(int32_t value)
{
    return (value < 0) ? -value : value;
}

bool s5_build_profile(const s5_sample_t * p_samples, int32_t count,
                      s5_profile_t * p_profile)
{
    int32_t  idx;
    int32_t  near = -1;
    int32_t  low;
    int32_t  high;
    int32_t  clr;
    uint16_t dmin;

    *p_profile = (s5_profile_t){ 0 };
    if ((NULL == p_samples) || (count <= 0))
    {
        return false;
    }

    /* 1. nearest hit; ties go to the one closest to straight ahead */
    for (idx = 0; idx < count; idx++)
    {
        if ((is_hit(&p_samples[idx])) &&
            ((near < 0) || (p_samples[idx].mm < p_samples[near].mm) ||
             ((p_samples[idx].mm == p_samples[near].mm) &&
              (abs_i(p_samples[idx].deg) < abs_i(p_samples[near].deg)))))
        {
            near = idx;
        }
    }
    if (near < 0)
    {
        return false;
    }

    /* 2. grow the cluster while neighbours are hits at a similar range */
    low = near;
    while ((low > 0) && (is_hit(&p_samples[low - 1])) &&
           (abs_i((int32_t)p_samples[low - 1].mm -
                  (int32_t)p_samples[low].mm) <= S5_CLUSTER_GAP_MM))
    {
        low--;
    }
    high = near;
    while ((high < (count - 1)) && (is_hit(&p_samples[high + 1])) &&
           (abs_i((int32_t)p_samples[high + 1].mm -
                  (int32_t)p_samples[high].mm) <= S5_CLUSTER_GAP_MM))
    {
        high++;
    }

    dmin                      = p_samples[near].mm;
    p_profile->distance_mm    = dmin;
    p_profile->right_edge_deg = p_samples[low].deg;
    p_profile->left_edge_deg  = p_samples[high].deg;
    p_profile->centre_deg =
        (int8_t)(((int32_t)p_samples[low].deg + (int32_t)p_samples[high].deg) /
                 2);
    p_profile->right_mm = (int16_t)(((int32_t)p_samples[low].mm *
                                     s5_sin1000(p_samples[low].deg)) /
                                    1000);
    p_profile->left_mm  = (int16_t)(((int32_t)p_samples[high].mm *
                                    s5_sin1000(p_samples[high].deg)) /
                                   1000);
    p_profile->width_mm = (uint16_t)abs_i((int32_t)p_profile->left_mm -
                                          (int32_t)p_profile->right_mm);

    /* 3. clearance: the shortest range seen beyond each edge.
          No echo counts as the sensor's full range.  No sample at all
          beyond an edge means "unknown", treated as 0 (not safe). */
    clr = -1;
    for (idx = high + 1; idx < count; idx++)
    {
        int32_t dist = (0 == p_samples[idx].mm) ? 2000 : p_samples[idx].mm;
        if ((clr < 0) || (dist < clr))
        {
            clr = dist;
        }
    }
    p_profile->clear_left_mm = (uint16_t)((clr < 0) ? 0 : clr);

    clr = -1;
    for (idx = 0; idx < low; idx++)
    {
        int32_t dist = (0 == p_samples[idx].mm) ? 2000 : p_samples[idx].mm;
        if ((clr < 0) || (dist < clr))
        {
            clr = dist;
        }
    }
    p_profile->clear_right_mm = (uint16_t)((clr < 0) ? 0 : clr);

    p_profile->valid = 1;
    (void)s5_choose_side(p_profile);
    return true;
}

int32_t s5_lateral_needed_mm(const s5_profile_t * p_profile, s5_side_t side)
{
    int32_t edge;

    if (S5_SIDE_LEFT == side)
    {
        edge = p_profile->left_mm; /* + = left of centre */
    }
    else
    {
        edge = -(int32_t)p_profile->right_mm; /* mirror for the right */
    }
    if (edge < 0)
    {
        edge = 0; /* obstacle entirely on the other side */
    }
    return edge + S5_ROBOT_HALF_WIDTH_MM + S5_SIDE_MARGIN_MM;
}

s5_side_t s5_choose_side(s5_profile_t * p_profile)
{
    bool b_l_ok = (p_profile->clear_left_mm >= S5_MIN_CLEAR_MM) ? true : false;
    bool b_r_ok = (p_profile->clear_right_mm >= S5_MIN_CLEAR_MM) ? true : false;
    s5_side_t side;

    /* A clearly off-centre obstacle rules out starting towards it. For
       a centred obstacle, use the available clearance and detour length. */
    if (p_profile->centre_deg <= -S5_SIDE_BIAS_DEG)
    {
        b_r_ok = false;
    }
    else if (p_profile->centre_deg >= S5_SIDE_BIAS_DEG)
    {
        b_l_ok = false;
    }

    if (b_l_ok && b_r_ok)
    {
        /* both safe: the shorter detour, then the larger clearance */
        int32_t lat_left  = s5_lateral_needed_mm(p_profile, S5_SIDE_LEFT);
        int32_t lat_right = s5_lateral_needed_mm(p_profile, S5_SIDE_RIGHT);
        if (lat_left != lat_right)
        {
            side = (lat_left < lat_right) ? S5_SIDE_LEFT : S5_SIDE_RIGHT;
        }
        else
        {
            side = (p_profile->clear_left_mm > p_profile->clear_right_mm)
                       ? S5_SIDE_LEFT
                       : S5_SIDE_RIGHT;
        }
    }
    else if (b_l_ok)
    {
        side = S5_SIDE_LEFT;
    }
    else if (b_r_ok)
    {
        side = S5_SIDE_RIGHT;
    }
    else
    {
        side = S5_SIDE_NONE;
    }
    p_profile->bypass_side = (uint8_t)side;
    return side;
}

static int32_t add_step(s5_step_t * p_plan, int32_t count, int32_t max,
                        uint8_t kind, int32_t deg, int32_t dist_mm,
                        int32_t speed, uint8_t search)
{
    if (count >= max)
    {
        return count;
    }
    p_plan[count]               = (s5_step_t){ 0 };
    p_plan[count].m.kind        = kind;
    p_plan[count].m.angle_deg   = (int8_t)deg;
    p_plan[count].m.distance_mm = (uint16_t)((dist_mm < 0) ? 0 : dist_mm);
    p_plan[count].m.speed_mm_s  = (uint16_t)speed;
    p_plan[count].search_line   = search;
    return count + 1;
}

/* Round up to reach the full lateral offset at the chosen heading. */
static int32_t diagonal_distance_mm(int32_t lateral_mm, int32_t angle_deg)
{
    int32_t sine = s5_sin1000(angle_deg);

    return (lateral_mm * 1000 + sine - 1) / sine;
}

#if S5_SIDE_PASS || S5_LENGTH_OBSERVE
void s5_geom_from_config(s5_geom_t * p_geom)
{
    p_geom->servo_ahead_mm  = S5_SERVO_AHEAD_MM;
    p_geom->arm_mm          = S5_SERVO_ARM_MM;
    p_geom->rear_mm         = S5_REAR_OVERHANG_MM;
    p_geom->front_mm        = S5_FRONT_OVERHANG_MM;
    p_geom->half_width_mm   = S5_ROBOT_HALF_WIDTH_MM;
    p_geom->margin_mm       = S5_PIVOT_MARGIN_MM;
    p_geom->return_turn_deg = S5_RETURN_TURN_DEG;
}
#endif

int32_t s5_side_window_mm(int32_t look_deg)
{
    int32_t gap_mm = S5_ROBOT_HALF_WIDTH_MM + S5_SIDE_MARGIN_MM;
    int32_t sine   = s5_sin1000(look_deg);
    int32_t window = (gap_mm * S5_SIDE_WINDOW_PCT * 10) / sine;

    return (window > 2000) ? 2000 : window; /* sensor range limit */
}

int32_t s5_pivot_min_axle_mm(const s5_geom_t * p_geom, int32_t gap_mm)
{
    /* Obstacle side is +u, forward is +x, axle at the origin.  The pivot
       turns the heading towards +u, so the obstacle-side corners swing:
         x' = x cos(phi) - u sin(phi),  u' = x sin(phi) + u cos(phi).
       A corner whose u' reaches the face line (gap_mm) must already be
       beyond the rear corner: axle - rear >= -x'. */
    const int32_t corner_x[2] = { p_geom->front_mm, -p_geom->rear_mm };
    int32_t       need        = 0;
    int32_t       phi;
    int32_t       idx;

    for (phi = 1; phi <= p_geom->return_turn_deg; phi++)
    {
        int32_t sn_val = s5_sin1000(phi);
        int32_t cs_val = s5_sin1000(90 - phi);

        for (idx = 0; idx < 2; idx++)
        {
            int32_t x_val  = corner_x[idx];
            int32_t u_val  = p_geom->half_width_mm;
            int32_t xr_val = (x_val * cs_val - u_val * sn_val) / 1000;
            int32_t ur_val = (x_val * sn_val + u_val * cs_val) / 1000;

            if ((ur_val >= gap_mm) && (-xr_val > need))
            {
                need = -xr_val;
            }
        }
    }
    return need;
}

int32_t s5_sensor_along_mm(const s5_geom_t * p_geom, int32_t look_deg)
{
    return p_geom->servo_ahead_mm +
           (p_geom->arm_mm * s5_sin1000(90 - look_deg)) / 1000;
}

int32_t s5_sensor_lateral_mm(const s5_geom_t * p_geom, int32_t look_deg)
{
    return (p_geom->arm_mm * s5_sin1000(look_deg)) / 1000;
}

static int32_t pass_clear_mm(const s5_geom_t * p_geom, int32_t gap_mm)
{
    int32_t body  = p_geom->rear_mm + p_geom->margin_mm;
    int32_t pivot = s5_pivot_min_axle_mm(p_geom, gap_mm) + p_geom->margin_mm;

    return (body > pivot) ? body : pivot;
}

int32_t s5_pass_tail_max_mm(const s5_geom_t * p_geom, int32_t look_deg)
{
    int32_t reach =
        (s5_side_window_mm(look_deg) * s5_sin1000(90 - look_deg)) / 1000;

    return s5_sensor_along_mm(p_geom, look_deg) + reach +
           pass_clear_mm(p_geom, p_geom->half_width_mm + S5_SIDE_GUARD_MM);
}

void s5_pass_init(s5_pass_t * p_pass, const s5_geom_t * p_geom,
                  int32_t look_deg, int32_t fallback_mm, int32_t cap_mm)
{
    *p_pass          = (s5_pass_t){ 0 };
    p_pass->geom     = *p_geom;
    p_pass->look_deg = (look_deg < 1) ? 1 : ((look_deg > 90) ? 90 : look_deg);
    p_pass->fallback_mm = fallback_mm;
    p_pass->cap_mm      = (cap_mm < fallback_mm) ? fallback_mm : cap_mm;
    p_pass->min_gap_mm  = 0x7FFF;
}

void s5_pass_init_observe(s5_pass_t * p_pass, const s5_geom_t * p_geom,
                          int32_t look_deg, int32_t leg_mm)
{
    s5_pass_init(p_pass, p_geom, look_deg, leg_mm, leg_mm);
    p_pass->observe      = 1U;
    p_pass->cap_mm       = 0x3FFFFFFF; /* never: the motion ends the leg */
    p_pass->seen_from_mm = -1;
}

s5_or_length_t s5_pass_length(const s5_pass_t * p_pass, bool b_leg_over,
                              bool b_any_reading, int32_t * p_len_mm,
                              s5_or_reason_t * p_reason)
{
    s5_or_length_t status = S5_OR_L_UNAVAILABLE;

    *p_len_mm = 0;
    *p_reason = S5_OR_R_NONE;
    switch ((s5_pass_phase_t)p_pass->phase)
    {
        case S5_PASS_PASSED:
        case S5_PASS_CLEARED:
            status =
                (0U != p_pass->partial) ? S5_OR_L_PARTIAL : S5_OR_L_MEASURED;
            *p_len_mm = p_pass->depth_mm;
            break;
        case S5_PASS_SENSOR_FAULT:
            *p_reason = S5_OR_R_SENSOR_FAULT;
            break;
        case S5_PASS_ALONGSIDE:
        case S5_PASS_CAP:
        case S5_PASS_TOO_CLOSE:
            if (b_leg_over ||
                (S5_PASS_ALONGSIDE != (s5_pass_phase_t)p_pass->phase))
            {
                *p_reason = S5_OR_R_REAR_NOT_SEEN;
            }
            else
            {
                status = S5_OR_L_MEASURING;
            }
            break;
        case S5_PASS_SEEKING:
        case S5_PASS_FALLBACK:
        default:
            if (!b_leg_over &&
                (S5_PASS_SEEKING == (s5_pass_phase_t)p_pass->phase))
            {
                status = S5_OR_L_MEASURING;
            }
            else
            {
                *p_reason =
                    b_any_reading ? S5_OR_R_NO_SIDE_ECHO : S5_OR_R_LOOK_FAILED;
            }
            break;
    }
    return status;
}

static int32_t pass_gap_mm(const s5_pass_t * p_pass, int32_t range_mm)
{
    /* The nearest return in the cone comes from the ray closest to the
       face normal, so range x sin(angle + half beam) is the gap (inferred:
       beam half-width from the HC-SR04 data sheet; bench step b checks it). */
    int32_t deg = p_pass->look_deg + S5_SIDE_BEAM_HALF_DEG;

    return s5_sensor_lateral_mm(&p_pass->geom, p_pass->look_deg) +
           (range_mm * s5_sin1000((deg > 90) ? 90 : deg)) / 1000;
}

static void pass_note_echo(s5_pass_t * p_pass, int32_t travel_mm,
                           int32_t range_mm)
{
    int32_t gap_mm = pass_gap_mm(p_pass, range_mm);

    p_pass->misses        = 0;
    p_pass->last_mm       = travel_mm;
    p_pass->last_range_mm = range_mm;
    if (gap_mm < p_pass->min_gap_mm)
    {
        p_pass->min_gap_mm = gap_mm;
    }
    if ((0U == p_pass->observe) &&
        (gap_mm < (p_pass->geom.half_width_mm + S5_SIDE_GUARD_MM)))
    {
        p_pass->phase = (uint8_t)S5_PASS_TOO_CLOSE;
    }
}

static void pass_rear_found(s5_pass_t * p_pass, int32_t travel_mm)
{
    int32_t cosine = s5_sin1000(90 - p_pass->look_deg);
    int32_t front  = p_pass->first_mm +
                    s5_sensor_along_mm(&p_pass->geom, p_pass->look_deg) +
                    (p_pass->first_range_mm * cosine) / 1000;

    /* Beam reach: at a shallow look angle the echo ends before the sensor
       is level with the rear corner, by range x cos(angle).  last_mm is
       the travel at the last CONFIRMED echo, so the debounce delay after
       it only makes the decision later, never the corner earlier. */
    p_pass->rear_mm = p_pass->last_mm +
                      s5_sensor_along_mm(&p_pass->geom, p_pass->look_deg) +
                      (p_pass->last_range_mm * cosine) / 1000;
    p_pass->depth_mm = p_pass->rear_mm - front;
    if (p_pass->depth_mm < 0)
    {
        p_pass->depth_mm = 0;
    }
    if (0U != p_pass->observe)
    {
        p_pass->partial =
            (p_pass->first_mm <= (p_pass->seen_from_mm + S5_SIDE_PARTIAL_MM))
                ? 1U
                : 0U;
    }
    else
    {
        p_pass->partial = (p_pass->first_mm <= S5_SIDE_PARTIAL_MM) ? 1U : 0U;
    }
    p_pass->decided_mm = travel_mm;
    p_pass->target_mm =
        p_pass->rear_mm + pass_clear_mm(&p_pass->geom, p_pass->min_gap_mm);
    if (p_pass->target_mm < travel_mm)
    {
        p_pass->target_mm = travel_mm;
    }
    p_pass->phase = (uint8_t)S5_PASS_PASSED;
}

s5_pass_phase_t s5_pass_update(s5_pass_t * p_pass, int32_t travel_mm,
                               const s5_side_reading_t * p_reading)
{
    bool b_fresh = (NULL != p_reading) && (p_reading->seq > p_pass->last_seq);
    bool b_echo  = false;
    bool b_fault = false;
    int32_t range_mm = 0;

    if (b_fresh)
    {
        if ((0U != p_pass->observe) && (p_pass->seen_from_mm < 0))
        {
            p_pass->seen_from_mm = travel_mm;
        }
        p_pass->last_seq = p_reading->seq;
        b_fault          = (S5_SIDE_ST_FAULT == p_reading->status);
        range_mm         = p_reading->mm;
        b_echo = (S5_SIDE_ST_ECHO == p_reading->status) && (range_mm > 0) &&
                 (range_mm <= s5_side_window_mm(p_pass->look_deg));
        p_pass->faults = b_fault ? (p_pass->faults + 1) : 0;
        if (p_pass->faults >= S5_SIDE_FAULT_COUNT)
        {
            p_pass->phase = (uint8_t)S5_PASS_SENSOR_FAULT;
        }
    }

    switch ((s5_pass_phase_t)p_pass->phase)
    {
        case S5_PASS_SEEKING:
            if (b_fresh && !b_fault)
            {
                if (b_echo)
                {
                    if (0 == p_pass->hits)
                    {
                        p_pass->run_start_mm = travel_mm;
                        p_pass->run_range_mm = range_mm;
                    }
                    p_pass->hits++;
                }
                else
                {
                    p_pass->hits = 0;
                }
                if (p_pass->hits >= S5_SIDE_FOUND_COUNT)
                {
                    p_pass->phase          = (uint8_t)S5_PASS_ALONGSIDE;
                    p_pass->first_mm       = p_pass->run_start_mm;
                    p_pass->first_range_mm = p_pass->run_range_mm;
                    pass_note_echo(p_pass, travel_mm, range_mm);
                }
            }
            if ((S5_PASS_SEEKING == (s5_pass_phase_t)p_pass->phase) &&
                (travel_mm >= p_pass->fallback_mm))
            {
                p_pass->phase = (uint8_t)S5_PASS_FALLBACK;
            }
            break;
        case S5_PASS_ALONGSIDE:
            if (b_fresh && !b_fault)
            {
                if (b_echo)
                {
                    pass_note_echo(p_pass, travel_mm, range_mm);
                }
                else
                {
                    p_pass->misses++;
                }
                if ((S5_PASS_ALONGSIDE == (s5_pass_phase_t)p_pass->phase) &&
                    (p_pass->misses >= S5_SIDE_LOST_COUNT) &&
                    ((travel_mm - p_pass->last_mm) >= S5_SIDE_LOST_MM))
                {
                    pass_rear_found(p_pass, travel_mm);
                }
            }
            if ((S5_PASS_ALONGSIDE == (s5_pass_phase_t)p_pass->phase) &&
                (travel_mm >= p_pass->cap_mm))
            {
                p_pass->phase = (uint8_t)S5_PASS_CAP;
            }
            break;
        case S5_PASS_PASSED:
            if (travel_mm >= p_pass->target_mm)
            {
                p_pass->phase = (uint8_t)S5_PASS_CLEARED;
            }
            break;
        default: /* final phases do not change */
            break;
    }
    return (s5_pass_phase_t)p_pass->phase;
}

int32_t s5_plan_bypass(const s5_profile_t * p_profile, s5_step_t * p_plan,
                       int32_t max)
{
    int32_t count = 0;
    int32_t sign;
    int32_t lateral_mm;
    int32_t first_lateral_mm;
    int32_t clearance_mm;
    int32_t outbound_forward_mm;
    int32_t pass_mm;
    int32_t search_mm;

    count = add_step(p_plan, count, max, S5_MOVE_STOP, 0, 0, 0, 0);

    if ((S5_SIDE_NONE == p_profile->bypass_side) || (0U == p_profile->valid))
    {
        /* nowhere to go: back off and let the controller decide */
        count = add_step(p_plan, count, max, S5_MOVE_REVERSE, 0, S5_REVERSE_MM,
                         S5_SEARCH_SPEED_MM_S, 0);
        return count;
    }

    sign = (S5_SIDE_LEFT == p_profile->bypass_side) ? 1 : -1;
    lateral_mm =
        s5_lateral_needed_mm(p_profile, (s5_side_t)p_profile->bypass_side);
    first_lateral_mm =
        (S5_BYPASS_FIRST_MM * s5_sin1000(S5_BYPASS_TURN_DEG)) / 1000;
    /* Finish the offset on a steeper heading to limit forward travel. */
    clearance_mm = diagonal_distance_mm(lateral_mm - first_lateral_mm,
                                        S5_BYPASS_CLEARANCE_DEG);
    outbound_forward_mm =
        ((S5_BYPASS_FIRST_MM * s5_sin1000(90 - S5_BYPASS_TURN_DEG)) +
         (clearance_mm * s5_sin1000(90 - S5_BYPASS_CLEARANCE_DEG))) /
        1000;
    pass_mm = (int32_t)p_profile->distance_mm + S5_OBSTACLE_DEPTH_MM +
              S5_ROBOT_LENGTH_MM - outbound_forward_mm;
    if (pass_mm < 50)
    {
        pass_mm = 50;
    }
    search_mm = diagonal_distance_mm(lateral_mm + S5_SEARCH_MARGIN_MM,
                                     S5_RETURN_TURN_DEG);

    count = add_step(p_plan, count, max, S5_MOVE_TURN,
                     sign * S5_BYPASS_TURN_DEG, 0, 0, 0);
    count = add_step(p_plan, count, max, S5_MOVE_FORWARD, 0, S5_BYPASS_FIRST_MM,
                     S5_BYPASS_SPEED_MM_S, 0);
    count = add_step(p_plan, count, max, S5_MOVE_TURN,
                     sign * (S5_BYPASS_CLEARANCE_DEG - S5_BYPASS_TURN_DEG), 0,
                     0, 0);
    count = add_step(p_plan, count, max, S5_MOVE_FORWARD, 0, clearance_mm,
                     S5_BYPASS_SPEED_MM_S, 0);
    count = add_step(p_plan, count, max, S5_MOVE_TURN,
                     -sign * S5_BYPASS_CLEARANCE_DEG, 0, 0, 0);
    count = add_step(p_plan, count, max, S5_MOVE_FORWARD, 0, pass_mm,
                     S5_BYPASS_SPEED_MM_S, 0);
#if S5_SIDE_PASS
    if (count > 0)
    {
        s5_step_t * p_pass = &p_plan[count - 1];
        int32_t cap_mm = (pass_mm > S5_PASS_CAP_MM) ? pass_mm : S5_PASS_CAP_MM;
        s5_geom_t geom;
        int32_t   request;

        s5_geom_from_config(&geom);
        request = cap_mm + s5_pass_tail_max_mm(&geom, S5_SIDE_LOOK_DEG);

        /* The obstacle is on the side opposite the bypass side. */
        p_pass->side_track = 1U;
        p_pass->look_deg   = (int8_t)(-sign * S5_SIDE_LOOK_DEG);
        p_pass->cap_mm     = (uint16_t)cap_mm;
        p_pass->request_mm = (uint16_t)((request > 65535) ? 65535 : request);
    }
#elif S5_LENGTH_OBSERVE
    if (count > 0)
    {
        /* report only: same motion, distance and speed as before */
        p_plan[count - 1].side_track = 2U;
        p_plan[count - 1].look_deg   = (int8_t)(-sign * S5_SIDE_LOOK_DEG);
    }
#endif
    count = add_step(p_plan, count, max, S5_MOVE_TURN,
                     -sign * S5_RETURN_TURN_DEG, 0, 0, 0);
    count = add_step(p_plan, count, max, S5_MOVE_FORWARD, 0, search_mm,
                     S5_SEARCH_SPEED_MM_S, 1);
    return count;
}

int32_t s5_plan_straight_recovery(int32_t heading_deg, s5_step_t * p_plan,
                                  int32_t max)
{
    int32_t count = 0;

    if (0 != heading_deg)
    {
        count =
            add_step(p_plan, count, max, S5_MOVE_TURN, -heading_deg, 0, 0, 0);
    }
    return add_step(p_plan, count, max, S5_MOVE_FORWARD, 0,
                    S5_RECOVERY_STRAIGHT_MM, S5_SEARCH_SPEED_MM_S, 1);
}

int32_t s5_coarse_angles(int8_t * p_deg, int32_t max)
{
    static const int8_t coarse[] = { -60, -30, 0, 30, 60 };
    int32_t             idx;
    int32_t             count = (int32_t)(sizeof(coarse) / sizeof(coarse[0]));

    for (idx = 0; (idx < count) && (idx < max); idx++)
    {
        p_deg[idx] = coarse[idx];
    }
    return idx;
}

int32_t s5_fine_angles(const s5_profile_t * p_profile, int8_t * p_deg,
                       int32_t max)
{
    /* 5-degree steps from 20 deg beyond the right edge to 20 deg
       beyond the left edge, clamped to the servo range */
    int32_t angle = (int32_t)p_profile->right_edge_deg - 20;
    int32_t high  = (int32_t)p_profile->left_edge_deg + 20;
    int32_t count = 0;

    if (angle < -S5_SCAN_MAX_DEG)
    {
        angle = -S5_SCAN_MAX_DEG;
    }
    if (high > S5_SCAN_MAX_DEG)
    {
        high = S5_SCAN_MAX_DEG;
    }
    for (; (angle <= high) && (count < max); angle += 5)
    {
        p_deg[count++] = (int8_t)angle;
    }
    return count;
}

/*** end of file ***/
