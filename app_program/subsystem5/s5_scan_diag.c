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
            v = -v;
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

/*** end of file ***/
