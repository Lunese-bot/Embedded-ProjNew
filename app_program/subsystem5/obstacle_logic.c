/*
 *      obstacle_logic.c
 *      Subsystem 5 -- obstacle profiling and bypass planning (no kernel).
 *
 *      Geometry, seen from above, sensor at the origin, x = forward:
 *      a sample (deg, mm) is a point mm * (cos deg, sin deg).  Its lateral
 *      offset is mm * sin(deg): positive = left of the car's centre line.
 *
 *      Bypass on side s (+1 left, -1 right), from the stopped car:
 *          1. turn  s*45            -> head diagonally away from the line
 *          2. drive L*sqrt2          -> L sideways, L forwards
 *          3. turn -s*45            -> parallel to the line, beside obstacle
 *          4. drive past the obstacle
 *          5. turn -s*45            -> head back towards the line
 *          6. drive (search) until an IR sensor sees the line
 *          7. turn  s*45            -> original heading; line follower takes
 * over L = lateral offset of the obstacle edge + half car width + margin.
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

int32_t s5_plan_bypass(const s5_profile_t * p_profile, s5_step_t * p_plan,
                       int32_t max)
{
    int32_t count = 0;
    int32_t sign;
    int32_t lat;
    int32_t leg1;
    int32_t leg2;

    count = add_step(p_plan, count, max, S5_MOVE_STOP, 0, 0, 0, 0);

    if ((S5_SIDE_NONE == p_profile->bypass_side) || (0U == p_profile->valid))
    {
        /* nowhere to go: back off and let the controller decide */
        count = add_step(p_plan, count, max, S5_MOVE_REVERSE, 0, S5_REVERSE_MM,
                         S5_SEARCH_SPEED_MM_S, 0);
        return count;
    }

    sign = (S5_SIDE_LEFT == p_profile->bypass_side) ? 1 : -1;
    lat  = s5_lateral_needed_mm(p_profile, (s5_side_t)p_profile->bypass_side);
    leg1 = (lat * 1000) / s5_sin1000(S5_BYPASS_TURN_DEG);
    /* forwards already covered by leg 1 = lat (45 deg) */
    leg2 = (int32_t)p_profile->distance_mm + S5_OBSTACLE_DEPTH_MM +
           S5_ROBOT_LENGTH_MM - lat;
    if (leg2 < 50)
    {
        leg2 = 50;
    }

    count = add_step(p_plan, count, max, S5_MOVE_TURN,
                     sign * S5_BYPASS_TURN_DEG, 0, 0, 0);
    count = add_step(p_plan, count, max, S5_MOVE_FORWARD, 0, leg1,
                     S5_BYPASS_SPEED_MM_S, 0);
    count = add_step(p_plan, count, max, S5_MOVE_TURN,
                     -sign * S5_BYPASS_TURN_DEG, 0, 0, 0);
    count = add_step(p_plan, count, max, S5_MOVE_FORWARD, 0, leg2,
                     S5_BYPASS_SPEED_MM_S, 0);
    count = add_step(p_plan, count, max, S5_MOVE_TURN,
                     -sign * S5_BYPASS_TURN_DEG, 0, 0, 0);
    count = add_step(p_plan, count, max, S5_MOVE_FORWARD, 0,
                     leg1 + S5_SEARCH_EXTRA_MM, S5_SEARCH_SPEED_MM_S, 1);
    return count;
}

int32_t s5_plan_recovery(s5_side_t side, int32_t attempt, s5_step_t * p_plan,
                         int32_t max)
{
    int32_t count = 0;
    int32_t sign  = (S5_SIDE_LEFT == side) ? 1 : -1;

    /* turn a little more towards the line each time, then search */
    count = add_step(p_plan, count, max, S5_MOVE_TURN,
                     -sign * S5_RECOVERY_TURN_DEG, 0, 0, 0);
    count = add_step(p_plan, count, max, S5_MOVE_FORWARD, 0,
                     S5_RECOVERY_BASE_MM + attempt * S5_RECOVERY_STEP_MM,
                     S5_SEARCH_SPEED_MM_S, 1);
    return count;
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
