/*
 *      s5_scan_diag.c
 *      Subsystem 5 -- scan evidence (see s5_scan_diag.h).  No kernel.
 */

#include "s5_scan_diag.h"

uint16_t s5_median_nonzero(const uint16_t * p_raw, int32_t n)
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
    if (n > 5)
    {
        n = 5;
    }
    for (idx = 0; idx < n; idx++)
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

void s5_trips_to_samples(const s5_trip_t * p_trips, int32_t n,
                         s5_sample_t * p_out)
{
    int32_t idx;

    for (idx = 0; idx < n; idx++)
    {
        p_out[idx].deg = p_trips[idx].deg;
        p_out[idx].mm  = p_trips[idx].used;
    }
}

int32_t s5_merge_trips(const s5_trip_t * a, int32_t na, const s5_trip_t * b,
                       int32_t nb, s5_trip_t * out, int32_t max)
{
    int32_t i = 0;
    int32_t j = 0;
    int32_t n = 0;

    while ((n < max) && ((i < na) || (j < nb)))
    {
        if ((j >= nb) || ((i < na) && (a[i].deg < b[j].deg)))
        {
            out[n++] = a[i++];
        }
        else
        {
            if ((i < na) && (a[i].deg == b[j].deg))
            {
                i++;
            }
            out[n++] = b[j++];
        }
    }
    return n;
}

/* Decimal digits of v (v >= 0) into p, returns how many. */
static int32_t put_uint(char * p, int32_t v)
{
    char    digits[8];
    int32_t nd = 0;
    int32_t k  = 0;

    do
    {
        digits[nd++] = (char)('0' + (v % 10));
        v /= 10;
    } while ((v > 0) && (nd < 8));
    while (nd > 0)
    {
        p[k++] = digits[--nd];
    }
    return k;
}

int32_t s5_format_trips(char * p_buf, int32_t size, const s5_trip_t * p_trips,
                        int32_t first, int32_t n)
{
    int32_t len = 0;
    int32_t idx;
    int32_t r;

    if ((NULL == p_buf) || (size <= 0))
    {
        return 0;
    }
    p_buf[0] = '\0';
    for (idx = first; (idx < (first + n)) && (NULL != p_trips); idx++)
    {
        char    item[40];
        int32_t k = 0;
        int32_t v = p_trips[idx].deg;

        item[k++] = ' ';
        item[k++] = (S5_SRC_FINE == p_trips[idx].src) ? 'F' : 'C';
        if (v < 0)
        {
            item[k++] = '-';
            v         = -v;
        }
        k += put_uint(&item[k], v);
        item[k++] = ':';
        for (r = 0; r < S5_TRIP_READS; r++)
        {
            if (r > 0)
            {
                item[k++] = '/';
            }
            k += put_uint(&item[k], p_trips[idx].raw[r]);
        }
        item[k++] = '=';
        k += put_uint(&item[k], p_trips[idx].used);
        if ((len + k) >= size)
        {
            break; /* whole items only */
        }
        for (r = 0; r < k; r++)
        {
            p_buf[len++] = item[r];
        }
        p_buf[len] = '\0';
    }
    return len;
}

static int32_t iabs(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* Pings of trip t that are plausible returns from a cluster reached at
   range ref_mm. */
static int32_t support_of(const s5_trip_t * p_t, int32_t ref_mm)
{
    int32_t r;
    int32_t got = 0;

    for (r = 0; r < S5_TRIP_READS; r++)
    {
        int32_t ping = (int32_t)p_t->raw[r];

        if ((ping > 0) && (ping <= S5_HIT_RANGE_MM) &&
            (iabs(ping - ref_mm) <= S5_CLUSTER_GAP_MM))
        {
            got++;
        }
    }
    return got;
}

/* Index of the sample at angle deg, -1 if none. */
static int32_t index_of(const s5_trip_t * p_trips, int32_t n, int32_t deg)
{
    int32_t idx;

    for (idx = 0; idx < n; idx++)
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

s5_we_status_t s5_width_estimate(const s5_trip_t * p_trips, int32_t n,
                                 s5_width_est_t * p_est)
{
    s5_sample_t  smp[64];
    s5_profile_t prof;
    int32_t      low;
    int32_t      high;
    int32_t      near = -1;
    int32_t      rs;
    int32_t      ls;
    int32_t      idx;

    *p_est        = (s5_width_est_t){ 0 };
    p_est->status = (uint8_t)S5_WE_NO_PROFILE;
    if ((NULL == p_trips) || (n <= 0) || (n > 64))
    {
        return S5_WE_NO_PROFILE;
    }
    s5_trips_to_samples(p_trips, n, smp);
    if (!s5_build_profile(smp, n, &prof))
    {
        return S5_WE_NO_PROFILE;
    }
    low  = index_of(p_trips, n, prof.right_edge_deg);
    high = index_of(p_trips, n, prof.left_edge_deg);
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

    if ((0 == low) || ((n - 1) == high))
    {
        p_est->status = (uint8_t)S5_WE_CLIPPED;
        return S5_WE_CLIPPED;
    }

    /* outermost supported sample on each side, walking inward */
    for (rs = low; rs <= high; rs++)
    {
        if (support_of(&p_trips[rs], ref_range(p_trips, rs, near)) >=
            S5_WE_MIN_SUPPORT)
        {
            break;
        }
    }
    for (ls = high; ls >= low; ls--)
    {
        if (support_of(&p_trips[ls], ref_range(p_trips, ls, near)) >=
            S5_WE_MIN_SUPPORT)
        {
            break;
        }
    }
    if ((rs > high) || (ls < low))
    {
        p_est->status = (uint8_t)S5_WE_NO_SUPPORT;
        return S5_WE_NO_SUPPORT;
    }
    p_est->sup_r_deg   = p_trips[rs].deg;
    p_est->sup_l_deg   = p_trips[ls].deg;
    p_est->sup_r_range = p_trips[rs].used;
    p_est->sup_l_range = p_trips[ls].used;
    p_est->sup_r_support =
        (uint8_t)support_of(&p_trips[rs], ref_range(p_trips, rs, near));
    p_est->sup_l_support =
        (uint8_t)support_of(&p_trips[ls], ref_range(p_trips, ls, near));
    if (rs >= ls)
    {
        p_est->status = (uint8_t)S5_WE_COLLAPSED;
        return S5_WE_COLLAPSED;
    }
    {
        int16_t r_mm = offset_mm(&p_trips[rs]);
        int16_t l_mm = offset_mm(&p_trips[ls]);

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

const char * s5_we_name(uint8_t status)
{
    static const char * const names[] = { "ok", "no_profile", "clipped",
                                          "no_support", "collapsed" };

    return (status < (uint8_t)(sizeof(names) / sizeof(names[0])))
               ? names[status]
               : "?";
}

/*** end of file ***/
