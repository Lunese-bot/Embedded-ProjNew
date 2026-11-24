/* Field-by-field equality of S5 scan and plan structs for the host
   tests (memcmp would also compare padding bytes, which C leaves
   undefined).  Include after obstacle_logic.c. */
#ifndef S5_SAME_H
#define S5_SAME_H

static bool same_samples(const s5_sample_t * a, const s5_sample_t * b, int32_t n)
{
    int32_t i;
    for (i = 0; i < n; i++)
    {
        if ((a[i].deg != b[i].deg) || (a[i].mm != b[i].mm)) { return false; }
    }
    return true;
}

static bool same_profile(const s5_profile_t * a, const s5_profile_t * b)
{
    return (a->distance_mm == b->distance_mm) && (a->left_edge_deg == b->left_edge_deg) &&
           (a->right_edge_deg == b->right_edge_deg) && (a->centre_deg == b->centre_deg) &&
           (a->left_mm == b->left_mm) && (a->right_mm == b->right_mm) &&
           (a->width_mm == b->width_mm) && (a->clear_left_mm == b->clear_left_mm) &&
           (a->clear_right_mm == b->clear_right_mm) && (a->bypass_side == b->bypass_side) &&
           (a->valid == b->valid);
}

static bool same_steps(const s5_step_t * a, const s5_step_t * b, int32_t n)
{
    int32_t i;
    for (i = 0; i < n; i++)
    {
        if ((a[i].m.kind != b[i].m.kind) || (a[i].m.angle_deg != b[i].m.angle_deg) ||
            (a[i].m.speed_mm_s != b[i].m.speed_mm_s) || (a[i].m.distance_mm != b[i].m.distance_mm) ||
            (a[i].m.seq != b[i].m.seq) || (a[i].search_line != b[i].search_line) ||
            (a[i].side_track != b[i].side_track) || (a[i].look_deg != b[i].look_deg) ||
            (a[i].cap_mm != b[i].cap_mm) || (a[i].request_mm != b[i].request_mm))
        {
            return false;
        }
    }
    return true;
}

#endif
