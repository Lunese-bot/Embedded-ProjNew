/*
 *      s5_scan_diag.c
 *      Subsystem 5 -- scan evidence (see s5_scan_diag.h).  No kernel.
 */

#include "s5_scan_diag.h"

uint16_t s5_median_nonzero(const uint16_t * p_raw, int32_t count)
{
    uint16_t vals[5];
    int32_t  got = 0;
    int32_t  idx;
    int32_t  jdx;
    uint16_t tmp;

    if (NULL == p_raw)
    {
        return 0U;
    }
    if (count > 5)
    {
        count = 5;
    }
    for (idx = 0; idx < count; idx++)
    {
        if (0U != p_raw[idx])
        {
            vals[got++] = p_raw[idx];
        }
    }
    if (0 == got)
    {
        return 0U;
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

void s5_trips_to_samples(const s5_trip_t * p_trips, int32_t count,
                         s5_sample_t * p_out)
{
    int32_t idx;

    for (idx = 0; idx < count; idx++)
    {
        p_out[idx].deg = p_trips[idx].deg;
        p_out[idx].mm  = p_trips[idx].used;
    }
}

int32_t s5_merge_trips(const s5_trip_t * p_a_val, int32_t na_val,
                       const s5_trip_t * p_b_val, int32_t nb_val,
                       s5_trip_t * p_output, int32_t max)
{
    int32_t idx2  = 0;
    int32_t jdx2  = 0;
    int32_t count = 0;

    while ((count < max) && ((idx2 < na_val) || (jdx2 < nb_val)))
    {
        if ((jdx2 >= nb_val) ||
            ((idx2 < na_val) && (p_a_val[idx2].deg < p_b_val[jdx2].deg)))
        {
            p_output[count++] = p_a_val[idx2++];
        }
        else
        {
            if ((idx2 < na_val) && (p_a_val[idx2].deg == p_b_val[jdx2].deg))
            {
                idx2++;
            }
            p_output[count++] = p_b_val[jdx2++];
        }
    }
    return count;
}

/* Decimal digits of v (v >= 0) into p, returns how many. */
static int32_t put_uint(char * p_val, int32_t value)
{
    char    digits[8];
    int32_t nd_val = 0;
    int32_t kdx    = 0;

    do
    {
        digits[nd_val++] = (char)('0' + (value % 10));
        value /= 10;
    } while ((value > 0) && (nd_val < 8));
    while (nd_val > 0)
    {
        p_val[kdx++] = digits[--nd_val];
    }
    return kdx;
}

int32_t s5_format_trips(char * p_buf, int32_t size, const s5_trip_t * p_trips,
                        int32_t first, int32_t count)
{
    int32_t len = 0;
    int32_t idx;
    int32_t r_val;

    if ((NULL == p_buf) || (size <= 0))
    {
        return 0;
    }
    p_buf[0] = '\0';
    for (idx = first; (idx < (first + count)) && (NULL != p_trips); idx++)
    {
        char    item[40];
        int32_t kdx   = 0;
        int32_t value = p_trips[idx].deg;

        item[kdx++] = ' ';
        item[kdx++] = (S5_SRC_FINE == p_trips[idx].src) ? 'F' : 'C';
        if (value < 0)
        {
            item[kdx++] = '-';
            value       = -value;
        }
        kdx += put_uint(&item[kdx], value);
        item[kdx++] = ':';
        for (r_val = 0; r_val < S5_TRIP_READS; r_val++)
        {
            if (r_val > 0)
            {
                item[kdx++] = '/';
            }
            kdx += put_uint(&item[kdx], p_trips[idx].raw[r_val]);
        }
        item[kdx++] = '=';
        kdx += put_uint(&item[kdx], p_trips[idx].used);
        if ((len + kdx) >= size)
        {
            break; /* whole items only */
        }
        for (r_val = 0; r_val < kdx; r_val++)
        {
            p_buf[len++] = item[r_val];
        }
        p_buf[len] = '\0';
    }
    return len;
}

static int32_t iabs(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* Pings of trip t that are plausible returns from a cluster reached at
   range ref_mm. */
static int32_t support_of(const s5_trip_t * p_t, int32_t ref_mm)
{
    int32_t r_val;
    int32_t got = 0;

    for (r_val = 0; r_val < S5_TRIP_READS; r_val++)
    {
        int32_t ping = (int32_t)p_t->raw[r_val];

        if ((ping > 0) && (ping <= S5_HIT_RANGE_MM) &&
            (iabs(ping - ref_mm) <= S5_CLUSTER_GAP_MM))
        {
            got++;
        }
    }
    return got;
}

/* Index of the sample at angle deg, -1 if none. */
static int32_t index_of(const s5_trip_t * p_trips, int32_t count, int32_t deg)
{
    int32_t idx;

    for (idx = 0; idx < count; idx++)
    {
        if ((int32_t)p_trips[idx].deg == deg)
        {
            return idx;
        }
    }
    return -1;
}

/* Range that joined sample idx to the cluster grown from near. */
static int32_t ref_range(const s5_trip_t * p_trips, int32_t idx, int32_t near)
{
    if (idx < near)
    {
        return (int32_t)p_trips[idx + 1].used;
    }
    if (idx > near)
    {
        return (int32_t)p_trips[idx - 1].used;
    }
    return (int32_t)p_trips[idx].used;
}

static int16_t offset_mm(const s5_trip_t * p_t)
{
    return (int16_t)(((int32_t)p_t->used * s5_sin1000(p_t->deg)) / 1000);
}

/* s5_width_estimate(), second part: the outermost supported sample on each
   side of [low, high], and the width between them.  Sets p_est->status. */
static s5_we_status_t supported_width(const s5_trip_t * p_trips, int32_t low,
                                      int32_t high, int32_t near,
                                      s5_width_est_t * p_est)
{
    int32_t rs_val;
    int32_t ls_val;

    /* outermost supported sample on each side, walking inward */
    for (rs_val = low; rs_val <= high; rs_val++)
    {
        if (support_of(&p_trips[rs_val], ref_range(p_trips, rs_val, near)) >=
            S5_WE_MIN_SUPPORT)
        {
            break;
        }
    }
    for (ls_val = high; ls_val >= low; ls_val--)
    {
        if (support_of(&p_trips[ls_val], ref_range(p_trips, ls_val, near)) >=
            S5_WE_MIN_SUPPORT)
        {
            break;
        }
    }
    if ((rs_val > high) || (ls_val < low))
    {
        p_est->status = (uint8_t)S5_WE_NO_SUPPORT;
        return S5_WE_NO_SUPPORT;
    }
    p_est->sup_r_deg   = p_trips[rs_val].deg;
    p_est->sup_l_deg   = p_trips[ls_val].deg;
    p_est->sup_r_range = p_trips[rs_val].used;
    p_est->sup_l_range = p_trips[ls_val].used;
    p_est->sup_r_support =
        (uint8_t)support_of(&p_trips[rs_val], ref_range(p_trips, rs_val, near));
    p_est->sup_l_support =
        (uint8_t)support_of(&p_trips[ls_val], ref_range(p_trips, ls_val, near));
    if (rs_val >= ls_val)
    {
        p_est->status = (uint8_t)S5_WE_COLLAPSED;
        return S5_WE_COLLAPSED;
    }
    {
        int16_t r_mm = offset_mm(&p_trips[rs_val]);
        int16_t l_mm = offset_mm(&p_trips[ls_val]);

        if (l_mm <= r_mm)
        {
            /* reversed endpoints: no width is claimed (never abs()) */
            p_est->status = (uint8_t)S5_WE_COLLAPSED;
            return S5_WE_COLLAPSED;
        }
        p_est->sup_right_mm = r_mm;
        p_est->sup_left_mm  = l_mm;
        p_est->sup_width_mm = (uint16_t)(l_mm - r_mm);
    }
    p_est->status = (uint8_t)S5_WE_OK;
    return S5_WE_OK;
}

s5_we_status_t s5_width_estimate(const s5_trip_t * p_trips, int32_t count,
                                 s5_width_est_t * p_est)
{
    s5_sample_t  smp[64];
    s5_profile_t prof;
    int32_t      low;
    int32_t      high;
    int32_t      near = -1;
    int32_t      idx;

    *p_est        = (s5_width_est_t){ 0 };
    p_est->status = (uint8_t)S5_WE_NO_PROFILE;
    if ((NULL == p_trips) || (count <= 0) || (count > 64))
    {
        return S5_WE_NO_PROFILE;
    }
    s5_trips_to_samples(p_trips, count, smp);
    if (!s5_build_profile(smp, count, &prof))
    {
        return S5_WE_NO_PROFILE;
    }
    low  = index_of(p_trips, count, prof.right_edge_deg);
    high = index_of(p_trips, count, prof.left_edge_deg);
    /* the nearest sample, as s5_build_profile picked it */
    for (idx = low; idx <= high; idx++)
    {
        if ((p_trips[idx].used == prof.distance_mm) &&
            ((near < 0) || (iabs(p_trips[idx].deg) < iabs(p_trips[near].deg))))
        {
            near = idx;
        }
    }
    p_est->raw_r_deg    = prof.right_edge_deg;
    p_est->raw_l_deg    = prof.left_edge_deg;
    p_est->raw_r_range  = p_trips[low].used;
    p_est->raw_l_range  = p_trips[high].used;
    p_est->raw_right_mm = prof.right_mm;
    p_est->raw_left_mm  = prof.left_mm;
    p_est->raw_width_mm = prof.width_mm;
    p_est->raw_r_support =
        (uint8_t)support_of(&p_trips[low], ref_range(p_trips, low, near));
    p_est->raw_l_support =
        (uint8_t)support_of(&p_trips[high], ref_range(p_trips, high, near));

    if ((0 == low) || ((count - 1) == high))
    {
        p_est->status = (uint8_t)S5_WE_CLIPPED;
        return S5_WE_CLIPPED;
    }

    return supported_width(p_trips, low, high, near, p_est);
}

const char * s5_we_name(uint8_t status)
{
    static const char * const names[] = { "ok", "no_profile", "clipped",
                                          "no_support", "collapsed" };

    return (status < (uint8_t)(sizeof(names) / sizeof(names[0])))
               ? names[status]
               : "?";
}

/*** end of file ***/
