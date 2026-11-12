/*
 *      code39_geom.c
 *      Subsystem 3 -- fallback one-letter Code 39 reader on the raw L
 *      trace (wide-bar layout fit + blurred-bar model).  See code39_geom.h.
 *
 *      Emulated Cortex-M0+ (qemu, 2026-11-12): about 24 million
 *      instructions for a full 7 s trace, roughly 0.2-0.3 s on the Pico.
 *
 *      Units: positions are kept in 1/16 of a trace unit (wheel count, or
 *      sample when distance is not used) as int32; the fit uses 64-bit
 *      integers, the blurred model a lookup table, so the Cortex-M0+
 *      (no FPU) only does a few hundred float operations per read.
 */

#include "code39_geom.h"
#include <limits.h>
#include <stddef.h>

#define Q            16    /* positions in 1/16 trace unit           */
#define N_LETTERS    26u
#define N_RATIOS     3u
#define N_WIDE       6u    /* wide bars in "*X*" (2 per symbol)       */
#define N_CAND       10u   /* widest humps tried as wide bars         */
#define MAX_SEG      25u   /* humps stored (more than 24 = refused)   */
#define MAX_HUMPS    24u
#define N_ELEMS      29u   /* "*X*": 3 x 9 + 2 gaps                   */
#define N_BARS       15u
#define TOP_MODEL    12u   /* best fits checked with the model        */
#define N_RIVAL      3u    /* next-best fits, checked only as rivals  */
#define C39G_PRUNE_X10 10  /* fits worse than 1 module are not refined */
#define MIN_MODEL_N  20u   /* samples needed under the model          */
#define NARROW_Q8    256   /* one module in 1/256 module              */
#define PHI_HALF_Q8  768   /* table covers +-3 modules                */
#define PHI_ONE      4096  /* table value for "all black"             */
#define HUGE_MSE     1.0e18

/* Code 39 bar/space patterns, 1 = wide; index 26 = '*' */
static const char * const k_pat[N_LETTERS + 1u] = {
    "100001001", "001001001", "101001000", "000011001", "100011000",
    "001011000", "000001101", "100001100", "001001100", "000011100",
    "100000011", "001000011", "101000010", "000010011", "100010010",
    "001010010", "000000111", "100000110", "001000110", "000010110",
    "110000001", "011000001", "111000000", "010010001", "110010000",
    "011010000", "010010100"
};

/* wide:narrow ratios tried, in quarter modules (2.5, 2.75, 3.0) */
static const int32_t k_ratio_q4[N_RATIOS] = { 10, 11, 12 };

/* Normal CDF of a bar edge blurred by sigma = 0.7 module, x 4096, for
   offsets -3 .. +3 modules in steps of 1/64 module (generated with erf) */
static const uint16_t k_phi[385] = {
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    1,    1,    1,    1,    1,    1,    1,    1,
       1,    1,    1,    1,    1,    2,    2,    2,    2,    2,    2,    2,
       3,    3,    3,    3,    4,    4,    4,    5,    5,    5,    6,    6,
       7,    7,    8,    8,    9,    9,   10,   11,   12,   12,   13,   14,
      15,   16,   17,   18,   20,   21,   22,   24,   25,   27,   29,   31,
      33,   35,   37,   39,   42,   44,   47,   49,   52,   56,   59,   62,
      66,   70,   73,   78,   82,   86,   91,   96,  101,  107,  112,  118,
     125,  131,  138,  145,  152,  159,  167,  175,  184,  193,  202,  211,
     221,  231,  242,  253,  264,  276,  288,  301,  314,  327,  341,  355,
     370,  385,  400,  416,  433,  450,  467,  485,  503,  522,  541,  561,
     582,  602,  624,  645,  668,  690,  714,  737,  762,  786,  812,  837,
     864,  890,  917,  945,  973, 1001, 1030, 1060, 1089, 1120, 1150, 1181,
    1213, 1245, 1277, 1309, 1342, 1375, 1409, 1442, 1477, 1511, 1546, 1580,
    1615, 1651, 1686, 1722, 1758, 1794, 1830, 1866, 1902, 1939, 1975, 2012,
    2048, 2084, 2121, 2157, 2194, 2230, 2266, 2302, 2338, 2374, 2410, 2445,
    2481, 2516, 2550, 2585, 2619, 2654, 2687, 2721, 2754, 2787, 2819, 2851,
    2883, 2915, 2946, 2976, 3007, 3036, 3066, 3095, 3123, 3151, 3179, 3206,
    3232, 3259, 3284, 3310, 3334, 3359, 3382, 3406, 3428, 3451, 3472, 3494,
    3514, 3535, 3555, 3574, 3593, 3611, 3629, 3646, 3663, 3680, 3696, 3711,
    3726, 3741, 3755, 3769, 3782, 3795, 3808, 3820, 3832, 3843, 3854, 3865,
    3875, 3885, 3894, 3903, 3912, 3921, 3929, 3937, 3944, 3951, 3958, 3965,
    3971, 3978, 3984, 3989, 3995, 4000, 4005, 4010, 4014, 4018, 4023, 4026,
    4030, 4034, 4037, 4040, 4044, 4047, 4049, 4052, 4054, 4057, 4059, 4061,
    4063, 4065, 4067, 4069, 4071, 4072, 4074, 4075, 4076, 4078, 4079, 4080,
    4081, 4082, 4083, 4084, 4084, 4085, 4086, 4087, 4087, 4088, 4088, 4089,
    4089, 4090, 4090, 4091, 4091, 4091, 4092, 4092, 4092, 4093, 4093, 4093,
    4093, 4094, 4094, 4094, 4094, 4094, 4094, 4094, 4095, 4095, 4095, 4095,
    4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4096, 4096, 4096,
    4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096,
    4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096,
    4096,
};

typedef struct
{
    float   s, e, c, w; /* start, end, centre, width (trace units)   */
    int32_t peak;       /* highest point above the floor               */
} hump_t;

typedef struct
{
    int32_t s[N_BARS]; /* bar starts, 1/256 module from the first edge */
    int32_t e[N_BARS];
    int32_t wide_x8[N_WIDE]; /* wide-bar centres, 1/8 module         */
    int32_t total_q8;        /* whole code width                       */
} layout_t;

typedef struct
{
    float    err;    /* worst centre error, modules                    */
    float    a;      /* trace position of the code's first edge        */
    float    b;      /* trace units per module                         */
    uint16_t order;  /* first found (keeps ties in a fixed order)      */
    uint8_t  ratio;  /* index into k_ratio_q4                          */
    uint8_t  b_ok;
} fit_t;

typedef struct
{
    double  mse;
    double  r2;
    float   b;
    uint8_t key;     /* letter * 2 + reversed                      */
    uint8_t b_rival; /* fit worse than C39G_ERR_MAX_X100: may only
                        block another letter, never be the answer   */
} score_t;

static int32_t g_pos[C39G_MAX_SAMPLES]; /* 1/16 trace unit */
static int32_t g_sig[C39G_MAX_SAMPLES]; /* L above the white floor */
static hump_t  g_hump[MAX_SEG];
static fit_t   g_fit[N_LETTERS * 2u];

static void make_layout(uint32_t letter, int32_t r4, bool b_rev,
                        layout_t * p_lay)
{
    int32_t  w[N_ELEMS];
    bool     b_bar[N_ELEMS];
    bool     b_wide[N_ELEMS];
    uint32_t n = 0u;
    uint32_t k;
    uint32_t i;
    int32_t  x  = 0;
    uint32_t nb = 0u;
    uint32_t nw = 0u;

    for (k = 0u; k < 3u; k++)
    {
        const char * p = k_pat[(1u == k) ? letter : N_LETTERS];
        for (i = 0u; i < 9u; i++)
        {
            b_wide[n] = ('1' == p[i]);
            /* Cast: r4 * 64 is at most 768 */
            w[n]      = b_wide[n] ? (r4 * 64) : NARROW_Q8;
            b_bar[n]  = (0u == (i % 2u));
            n++;
        }
        if (k < 2u)
        {
            w[n]      = NARROW_Q8; /* gap between symbols */
            b_bar[n]  = false;
            b_wide[n] = false;
            n++;
        }
    }
    for (k = 0u; k < n; k++)
    {
        uint32_t j = b_rev ? (n - 1u - k) : k;
        if (b_bar[j])
        {
            p_lay->s[nb] = x;
            p_lay->e[nb] = x + w[j];
            nb++;
            if (b_wide[j])
            {
                p_lay->wide_x8[nw] = (x + (w[j] / 2)) / 32;
                nw++;
            }
        }
        x += w[j];
    }
    p_lay->total_q8 = x;
}

/* Where the trace crosses the threshold between samples i and j */
static float cross(uint32_t i, uint32_t j, int32_t thr2)
{
    /* Casts: positions and ADC values are far inside float's exact range */
    float yi = (float)((2 * g_sig[i]) - thr2);
    float yj = (float)((2 * g_sig[j]) - thr2);
    float pi = (float)g_pos[i] / (float)Q;
    float pj = (float)g_pos[j] / (float)Q;

    if (yi == yj)
    {
        return pi;
    }
    return pi + (((pj - pi) * (-yi)) / (yj - yi));
}

/* Humps where 2 * sig > thr2.  Stores up to max_out of them (when p_out
   is not NULL), keeps the 6 highest peaks in top6[] (descending), and
   returns how many there are. */
static uint32_t find_humps(uint32_t n, int32_t thr2, hump_t * p_out,
                           uint32_t max_out, int32_t top6[N_WIDE])
{
    uint32_t count = 0u;
    uint32_t i     = 0u;
    uint32_t k;

    for (k = 0u; k < N_WIDE; k++)
    {
        top6[k] = -1;
    }
    while (i < n)
    {
        if ((2 * g_sig[i]) > thr2)
        {
            uint32_t j    = i;
            int32_t  peak = g_sig[i];
            while (((j + 1u) < n) && ((2 * g_sig[j + 1u]) > thr2))
            {
                j++;
                if (g_sig[j] > peak)
                {
                    peak = g_sig[j];
                }
            }
            if ((NULL != p_out) && (count < max_out))
            {
                hump_t * p_h = &p_out[count];
                p_h->s = (i > 0u) ? cross(i - 1u, i, thr2)
                                  : ((float)g_pos[i] / (float)Q);
                p_h->e = ((j + 1u) < n) ? cross(j, j + 1u, thr2)
                                        : ((float)g_pos[j] / (float)Q);
                p_h->c    = 0.5f * (p_h->s + p_h->e);
                p_h->w    = p_h->e - p_h->s;
                p_h->peak = peak;
            }
            /* insert into the 6 highest */
            for (k = 0u; k < N_WIDE; k++)
            {
                if (peak > top6[k])
                {
                    uint32_t m;
                    for (m = N_WIDE - 1u; m > k; m--)
                    {
                        top6[m] = top6[m - 1u];
                    }
                    top6[k] = peak;
                    break;
                }
            }
            count++;
            i = j + 1u;
        }
        else
        {
            i++;
        }
    }
    return count;
}

/* Blurred value (0..PHI_ONE) at d (1/256 module) past an edge */
static int32_t phi(int32_t d)
{
    int32_t t = d + PHI_HALF_Q8;
    int32_t idx;
    int32_t lo;

    if (t <= 0)
    {
        return 0;
    }
    if (t >= (2 * PHI_HALF_Q8))
    {
        return PHI_ONE;
    }
    idx = t >> 2;
    lo  = (int32_t)k_phi[idx];
    return lo + ((((int32_t)k_phi[idx + 1] - lo) * (t & 3)) >> 2);
}

/* Mean squared error of the best gain + offset fit of the blurred
   layout (first edge at a, b trace units per module) to the trace. */
static double model_mse(const layout_t * p_lay, uint32_t n, float a, float b,
                        double * p_r2)
{
    float    lo_f = (a - (3.0f * b)) * (float)Q;
    float    hi_f = (a + (((float)p_lay->total_q8 / (float)NARROW_Q8) +
                          3.0f) * b) * (float)Q;
    /* Casts: rounding the window and scale to integers */
    int32_t  lo   = (int32_t)lo_f + ((lo_f > (float)(int32_t)lo_f) ? 1 : 0);
    int32_t  hi   = (int32_t)hi_f - ((hi_f < (float)(int32_t)hi_f) ? 1 : 0);
    int32_t  a16  = (int32_t)((a * (float)Q) + ((a >= 0.0f) ? 0.5f : -0.5f));
    /* x (1/256 module) = (p - a16) * kq / 65536; inside the window
       |x| < 50 modules, so the product stays below 2^31 */
    int32_t  kq   = (int32_t)((16.0f * 65536.0f / b) + 0.5f);
    int32_t  last_x = INT32_MIN;
    uint32_t kb   = 0u; /* first bar that can still touch x */
    int64_t  sm   = 0;
    int64_t  smm  = 0;
    int64_t  sy   = 0;
    int64_t  smy  = 0;
    int64_t  syy  = 0;
    uint32_t cnt  = 0u;
    uint32_t i;
    double   dn;
    double   cmm;
    double   cmy;
    double   cyy;

    for (i = 0u; i < n; i++)
    {
        int32_t p = g_pos[i];
        if ((p >= lo) && (p <= hi))
        {
            int32_t  x = ((p - a16) * kq) / 65536;
            int32_t  m = 0;
            int32_t  y = g_sig[i];
            uint32_t k;
            if (x < last_x)
            {
                kb = 0u; /* wheel went back: start the bar search again */
            }
            last_x = x;
            while ((kb < N_BARS) && (x >= (p_lay->e[kb] + PHI_HALF_Q8)))
            {
                kb++; /* bar fully behind x: adds 1 - 1 = 0 from now on */
            }
            for (k = kb; (k < N_BARS) && (x > (p_lay->s[k] - PHI_HALF_Q8));
                 k++)
            {
                m += phi(x - p_lay->s[k]) - phi(x - p_lay->e[k]);
            }
            /* m and y are at most 4096: products fit 32 bits */
            sm += m;
            smm += m * m;
            sy += y;
            smy += m * y;
            syy += y * y;
            cnt++;
        }
    }
    *p_r2 = 0.0;
    if (cnt < MIN_MODEL_N)
    {
        return HUGE_MSE;
    }
    /* Casts: sums to double for the 2-parameter least squares */
    dn  = (double)cnt;
    cmm = (double)smm - (((double)sm * (double)sm) / dn);
    cmy = (double)smy - (((double)sm * (double)sy) / dn);
    cyy = (double)syy - (((double)sy * (double)sy) / dn);
    if ((cmm <= 0.0) || (cmy <= 0.0)) /* flat model, or bars read white */
    {
        return HUGE_MSE;
    }
    if (cyy > 0.0)
    {
        *p_r2 = (cmy * cmy) / (cmm * cyy); /* share of the trace explained */
    }
    return (cyy - ((cmy * cmy) / cmm)) / dn;
}

/* next 6-of-n combination in lexicographic order; false after the last */
static bool next_combo(uint32_t idx[N_WIDE], uint32_t n)
{
    int32_t k = (int32_t)N_WIDE - 1;

    /* Cast: k is 0..5 */
    while ((k >= 0) &&
           (idx[k] == (n - N_WIDE + (uint32_t)k)))
    {
        k--;
    }
    if (k < 0)
    {
        return false;
    }
    idx[k]++;
    for (k = k + 1; k < (int32_t)N_WIDE; k++)
    {
        idx[k] = idx[k - 1] + 1u;
    }
    return true;
}

static void fill_floor(const uint16_t * p_l, uint32_t n, int32_t fr16)
{
    uint32_t i;
    uint32_t j0 = 0u;
    uint32_t j1 = 0u;

    for (i = 0u; i < n; i++)
    {
        int32_t  fl;
        uint32_t k;
        while (((j0 + 1u) < n) && (g_pos[j0] < (g_pos[i] - fr16)))
        {
            j0++;
        }
        while (((j1 + 1u) < n) && (g_pos[j1 + 1u] <= (g_pos[i] + fr16)))
        {
            j1++;
        }
        fl = (int32_t)p_l[i];
        for (k = j0; k <= j1; k++)
        {
            if ((int32_t)p_l[k] < fl)
            {
                fl = (int32_t)p_l[k];
            }
        }
        g_sig[i] = (int32_t)p_l[i] - fl;
    }
}

c39g_result_t c39g_decode(const uint16_t * p_l, const uint8_t * p_fwd,
                          const int8_t * p_yaw, uint32_t n, bool b_use_dist,
                          void (*p_yield)(void), c39g_info_t * p_info)
{
    int32_t  top6[N_WIDE];
    int32_t  thr2;
    uint32_t n_h;
    uint32_t n_c;
    uint32_t cand[N_CAND];
    int32_t  cq[N_CAND]; /* candidate centres, 1/16 unit */
    int32_t  cw[N_CAND]; /* candidate widths,  1/16 unit */
    uint32_t i;
    uint32_t k;
    uint32_t ri;
    uint32_t key;
    uint16_t order = 0u;
    uint8_t  good[N_LETTERS * 2u];
    uint32_t n_good = 0u;
    score_t  sc[TOP_MODEL + N_RIVAL];
    uint32_t n_sc;
    uint32_t n_riv = 0u;
    int32_t  base;
    double   margin;
    bool     b_dist = b_use_dist && (NULL != p_fwd) && (NULL != p_yaw);

    if ((NULL == p_l) || (NULL == p_info) || (n < MIN_MODEL_N) ||
        (n > C39G_MAX_SAMPLES))
    {
        return C39G_ARG;
    }
    p_info->letter      = 0;
    p_info->best        = 0;
    p_info->runner      = '-';
    p_info->b_reversed  = false;
    p_info->n_humps     = 0u;
    p_info->err_x100    = 0u;
    p_info->margin_x100 = 0u;
    p_info->module_x10  = 0u;
    p_info->ratio_x100  = 0u;
    p_info->fit_x100    = 0u;

    /* 1. position and the signal above a rolling white floor */
    {
        int32_t pos = 0;
        for (i = 0u; i < n; i++)
        {
            if (b_dist)
            {
                /* fwd - yaw = (L + R) - (R - L) = 2 x left wheel */
                pos += ((int32_t)p_fwd[i] - (int32_t)p_yaw[i]) * Q;
            }
            else
            {
                /* Cast: i < C39G_MAX_SAMPLES */
                pos = (int32_t)i * Q;
            }
            g_pos[i] = pos;
        }
    }
    fill_floor(p_l, n, (b_dist ? C39G_FLOOR_COUNTS : C39G_FLOOR_SAMPLES) * Q);

    /* 2. humps: threshold at half the 6th highest */
    if (find_humps(n, 2 * C39G_MIN_HUMP, NULL, 0u, top6) < N_WIDE)
    {
        /* blurred: thin gaps stay above C39G_MIN_HUMP and the bars run
           together.  Try again at half the darkest hump. */
        if ((top6[0] <= (2 * C39G_MIN_HUMP)) ||
            (find_humps(n, top6[0], NULL, 0u, top6) < N_WIDE))
        {
            return C39G_FEW;
        }
    }
    thr2 = (top6[N_WIDE - 1u] > (2 * C39G_MIN_HUMP)) ? top6[N_WIDE - 1u]
                                                      : (2 * C39G_MIN_HUMP);
    n_h = find_humps(n, thr2, g_hump, MAX_SEG, top6);
    /* Cast: n_h is small here or refused below */
    p_info->n_humps = (uint8_t)((n_h > 255u) ? 255u : n_h);
    if (n_h < N_WIDE)
    {
        return C39G_FEW2;
    }
    if (n_h > MAX_HUMPS)
    {
        return C39G_MANY;
    }
    if (NULL != p_yield)
    {
        p_yield();
    }

    /* 3. the N_CAND widest humps (first one wins a tie), in trace order */
    n_c = (n_h < N_CAND) ? n_h : N_CAND;
    {
        bool b_used[MAX_SEG] = { false };
        for (k = 0u; k < n_c; k++)
        {
            uint32_t best = MAX_SEG;
            for (i = 0u; i < n_h; i++)
            {
                if ((!b_used[i]) &&
                    ((MAX_SEG == best) || (g_hump[i].w > g_hump[best].w)))
                {
                    best = i;
                }
            }
            b_used[best] = true;
        }
        k = 0u;
        for (i = 0u; i < n_h; i++) /* humps are already in trace order */
        {
            if (b_used[i])
            {
                cand[k] = i;
                k++;
            }
        }
    }
    for (k = 0u; k < n_c; k++)
    {
        /* Casts: trace positions to 1/16 unit, well inside int32 */
        float c = g_hump[cand[k]].c * (float)Q;
        float w = g_hump[cand[k]].w * (float)Q;
        cq[k] = (int32_t)(c + ((c >= 0.0f) ? 0.5f : -0.5f));
        cw[k] = (int32_t)(w + 0.5f);
    }
    /* from the first candidate on: keeps the fit sums small */
    base = cq[0];
    for (k = 0u; k < n_c; k++)
    {
        cq[k] -= base;
    }

    /* 4. lay the 6 wide bars of every letter on 6 of the candidates */
    for (key = 0u; key < (N_LETTERS * 2u); key++)
    {
        g_fit[key].b_ok = 0u;
    }
    for (ri = 0u; ri < N_RATIOS; ri++)
    {
        int32_t r4 = k_ratio_q4[ri];
        for (key = 0u; key < (N_LETTERS * 2u); key++)
        {
            layout_t lay;
            int64_t  sx  = 0;
            int64_t  sxx = 0;
            uint32_t idx[N_WIDE];
            fit_t *  p_f = &g_fit[key];

            make_layout(key / 2u, r4, (1u == (key % 2u)), &lay);
            for (k = 0u; k < N_WIDE; k++)
            {
                sx += lay.wide_x8[k];
                sxx += (int64_t)lay.wide_x8[k] * lay.wide_x8[k];
                idx[k] = k;
            }
            sxx = ((int64_t)N_WIDE * sxx) - (sx * sx);
            do
            {
                int64_t sp  = 0;
                int64_t sxp = 0;
                int64_t sw  = 0;
                int64_t nb;
                int64_t c0;
                int64_t worst = 0;
                float   err;
                /* quick look first: a fit within C39G_PRUNE_X10 / 10
                   module needs every centre within twice that of the
                   line through the first and last one */
                {
                    int32_t dq  = cq[idx[N_WIDE - 1u]] - cq[idx[0]];
                    int32_t dx  = lay.wide_x8[N_WIDE - 1u] - lay.wide_x8[0];
                    int32_t lim = (2 * 8 * C39G_PRUNE_X10 * dq) / 10;
                    bool    b_far = (dq <= 0);
                    for (k = 1u; (k < (N_WIDE - 1u)) && !b_far; k++)
                    {
                        int32_t dev = ((cq[idx[k]] - cq[idx[0]]) * dx) -
                                      (dq * (lay.wide_x8[k] - lay.wide_x8[0]));
                        b_far = (dev > lim) || (dev < -lim);
                    }
                    if (b_far)
                    {
                        continue;
                    }
                }
                for (k = 0u; k < N_WIDE; k++)
                {
                    sp += cq[idx[k]];
                    sxp += (int64_t)lay.wide_x8[k] * cq[idx[k]];
                    sw += cw[idx[k]];
                }
                /* slope in (1/16 unit) per (1/8 module) is nb / sxx */
                nb = ((int64_t)N_WIDE * sxp) - (sx * sp);
                if (nb <= 0)
                {
                    continue; /* bars in the wrong order */
                }
                /* residual x 6 sxx = 6 sxx q - sxx sp + nb sx - 6 nb x */
                c0 = (nb * sx) - (sxx * sp);
                for (k = 0u; k < N_WIDE; k++)
                {
                    int64_t r = ((int64_t)N_WIDE * sxx * cq[idx[k]]) + c0 -
                                ((int64_t)N_WIDE * nb * lay.wide_x8[k]);
                    r = (r < 0) ? -r : r;
                    if (r > worst)
                    {
                        worst = r;
                    }
                }
                /* Cast: error in modules = worst / (48 nb) */
                err = (float)worst / (48.0f * (float)nb);
                if ((0u == p_f->b_ok) || (err < p_f->err))
                {
                    /* hump width 0.4 .. 1.8 of a wide bar:
                       wid = sw sxx / (48 nb), wide bar = r4 / 4 modules */
                    int64_t lhs = 5 * sw * sxx;
                    if ((lhs >= (24 * r4 * nb)) && (lhs <= (108 * r4 * nb)))
                    {
                        /* Casts: final scale to trace units */
                        double bq = (double)nb / (double)sxx;
                        double aq = ((double)sp - (bq * (double)sx)) /
                                    (double)N_WIDE;
                        if (0u == p_f->b_ok)
                        {
                            p_f->order = order;
                            order++;
                        }
                        p_f->b_ok  = 1u;
                        p_f->err   = err;
                        p_f->a     = (float)((aq + (double)base) / (double)Q);
                        p_f->b     = (float)(bq * 8.0 / (double)Q);
                        p_f->ratio = (uint8_t)ri;
                    }
                }
            } while (next_combo(idx, n_c));
            if ((NULL != p_yield) && (1u == (key % 2u)))
            {
                p_yield(); /* once per letter */
            }
        }
    }

    /* 5. letters that fit within C39G_ERR_MAX_X100, best first */
    for (key = 0u; key < (N_LETTERS * 2u); key++)
    {
        if (0u != g_fit[key].b_ok)
        {
            n_good++;
        }
    }
    if (0u == n_good)
    {
        return C39G_NOFIT;
    }
    n_good = 0u;
    for (key = 0u; key < (N_LETTERS * 2u); key++)
    {
        if ((0u != g_fit[key].b_ok) &&
            (g_fit[key].err < ((float)C39G_ERR_MAX_X100 / 100.0f)))
        {
            /* insertion sort by (err, order) */
            uint32_t m = n_good;
            while ((m > 0u) &&
                   ((g_fit[good[m - 1u]].err > g_fit[key].err) ||
                    ((g_fit[good[m - 1u]].err == g_fit[key].err) &&
                     (g_fit[good[m - 1u]].order > g_fit[key].order))))
            {
                good[m] = good[m - 1u];
                m--;
            }
            /* Cast: key < 52 */
            good[m] = (uint8_t)key;
            n_good++;
        }
    }
    if (0u == n_good)
    {
        float best = 1.0e9f;
        for (key = 0u; key < (N_LETTERS * 2u); key++)
        {
            if ((0u != g_fit[key].b_ok) && (g_fit[key].err < best))
            {
                best = g_fit[key].err;
            }
        }
        /* Cast: shown on the dashboard only */
        p_info->err_x100 = (uint16_t)((best > 600.0f) ? 60000.0f
                                                      : (best * 100.0f));
        return C39G_ERR;
    }

    /* 5b. rivals: the letters that fit best of the rest.  Without them
       a lone letter under the limit would be taken with nothing to beat;
       a test with one thin gap printed black read Z as Q that way. */
    n_sc = (n_good < TOP_MODEL) ? n_good : TOP_MODEL;
    for (k = 0u; k < N_RIVAL; k++)
    {
        uint32_t pick = N_LETTERS * 2u;
        for (key = 0u; key < (N_LETTERS * 2u); key++)
        {
            bool b_taken = (0u == g_fit[key].b_ok) ||
                           (g_fit[key].err < ((float)C39G_ERR_MAX_X100 / 100.0f));
            for (i = 0u; (i < n_riv) && !b_taken; i++)
            {
                b_taken = (good[n_sc + i] == key);
            }
            if ((!b_taken) && (((N_LETTERS * 2u) == pick) ||
                               (g_fit[key].err < g_fit[pick].err)))
            {
                pick = key;
            }
        }
        if ((N_LETTERS * 2u) == pick)
        {
            break;
        }
        /* Cast: pick < 52; good[] has room, rivals are not good */
        good[n_sc + n_riv] = (uint8_t)pick;
        n_riv++;
    }

    /* 6. blurred-bar model of the whole code for the best fits */
    for (k = 0u; k < (n_sc + n_riv); k++)
    {
        static const float k_db[3] = { 0.985f, 1.0f, 1.015f };
        const fit_t *      p_f     = &g_fit[good[k]];
        layout_t           lay;
        double             best    = 0.0;
        double             best_r2 = 0.0;
        float              best_b  = p_f->b;
        uint32_t           di;
        uint32_t           ai;

        make_layout(good[k] / 2u, k_ratio_q4[p_f->ratio],
                    (1u == (good[k] % 2u)), &lay);
        for (di = 0u; di < 3u; di++)
        {
            for (ai = 0u; ai < 9u; ai++)
            {
                /* Cast: shift -1 .. +1 module in quarter steps */
                float  da  = ((float)ai - 4.0f) / 4.0f;
                float  bb  = p_f->b * k_db[di];
                double r2  = 0.0;
                double mse = model_mse(&lay, n, p_f->a + (da * p_f->b), bb,
                                       &r2);
                if (((0u == di) && (0u == ai)) || (mse < best))
                {
                    best    = mse;
                    best_r2 = r2;
                    best_b = bb;
                }
            }
        }
        /* insertion sort by (mse, letter, direction) */
        {
            uint32_t m = k;
            while ((m > 0u) &&
                   ((sc[m - 1u].mse > best) ||
                    ((sc[m - 1u].mse == best) && (sc[m - 1u].key > good[k]))))
            {
                sc[m] = sc[m - 1u];
                m--;
            }
            sc[m].mse = best;
            sc[m].r2  = best_r2;
            sc[m].b   = best_b;
            sc[m].key     = good[k];
            sc[m].b_rival = (k >= n_sc) ? 1u : 0u;
        }
        if (NULL != p_yield)
        {
            p_yield();
        }
    }

    /* 7. the best letter must clearly beat the best other letter */
    n_sc += n_riv;
    margin = 9.9;
    for (k = 1u; k < n_sc; k++)
    {
        if ((sc[k].key / 2u) != (sc[0].key / 2u))
        {
            /* Cast: letters are 'A' + 0..25 */
            p_info->runner = (char)('A' + (int32_t)(sc[k].key / 2u));
            if (sc[0].mse > 0.0)
            {
                margin = (sc[k].mse / sc[0].mse) - 1.0;
            }
            break;
        }
    }
    {
        const fit_t * p_f = &g_fit[sc[0].key];
        /* Casts: rounded values for the dashboard; letters 'A' + 0..25 */
        p_info->best        = (char)('A' + (int32_t)(sc[0].key / 2u));
        p_info->b_reversed  = (1u == (sc[0].key % 2u));
        p_info->err_x100    = (uint16_t)((p_f->err * 100.0f) + 0.5f);
        p_info->margin_x100 = (uint16_t)((margin > 600.0) ? 60000.0
                                                          : ((margin * 100.0) + 0.5));
        p_info->module_x10  = (uint16_t)((sc[0].b * 10.0f) + 0.5f);
        p_info->ratio_x100  = (uint16_t)(k_ratio_q4[p_f->ratio] * 25);
        p_info->fit_x100    = (uint8_t)((sc[0].r2 * 100.0) + 0.5);
    }
    if ((0u != sc[0].b_rival) ||
        (margin < ((double)C39G_MARGIN_X100 / 100.0)))
    {
        return C39G_TIE; /* a rival fits the trace better, or about as well */
    }
    if (sc[0].r2 < ((double)C39G_MIN_FIT_X100 / 100.0))
    {
        return C39G_WEAK; /* the bars do not look like this letter */
    }
    p_info->letter = p_info->best;
    return C39G_OK;
}

/*** end of file ***/
