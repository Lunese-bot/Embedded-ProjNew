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

#define Q              16 /* positions in 1/16 trace unit           */
#define N_LETTERS      26u
#define N_RATIOS       3u
#define N_WIDE         6u  /* wide bars in "*X*" (2 per symbol)       */
#define N_CAND         10u /* widest humps tried as wide bars         */
#define MAX_SEG        25u /* humps stored (more than 24 = refused)   */
#define MAX_HUMPS      24u
#define N_ELEMS        29u /* "*X*": 3 x 9 + 2 gaps                   */
#define N_BARS         15u
#define TOP_MODEL      12u  /* best fits checked with the model        */
#define N_RIVAL        3u   /* next-best fits, checked only as rivals  */
#define C39G_PRUNE_X10 10   /* fits worse than 1 module are not refined */
#define MIN_MODEL_N    20u  /* samples needed under the model          */
#define NARROW_Q8      256  /* one module in 1/256 module              */
#define PHI_HALF_Q8    768  /* table covers +-3 modules                */
#define PHI_ONE        4096 /* table value for "all black"             */
#define HUGE_MSE       1.0e18

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
    7,    7,    8,    8,    9,    9,    10,   11,   12,   12,   13,   14,
    15,   16,   17,   18,   20,   21,   22,   24,   25,   27,   29,   31,
    33,   35,   37,   39,   42,   44,   47,   49,   52,   56,   59,   62,
    66,   70,   73,   78,   82,   86,   91,   96,   101,  107,  112,  118,
    125,  131,  138,  145,  152,  159,  167,  175,  184,  193,  202,  211,
    221,  231,  242,  253,  264,  276,  288,  301,  314,  327,  341,  355,
    370,  385,  400,  416,  433,  450,  467,  485,  503,  522,  541,  561,
    582,  602,  624,  645,  668,  690,  714,  737,  762,  786,  812,  837,
    864,  890,  917,  945,  973,  1001, 1030, 1060, 1089, 1120, 1150, 1181,
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
    /* start, end, centre, width (trace units)   */
    float32_t s;
    float32_t e;
    float32_t c;
    float32_t w;
    int32_t   peak; /* highest point above the floor               */
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
    float32_t err;   /* worst centre error, modules                    */
    float32_t a;     /* trace position of the code's first edge        */
    float32_t b;     /* trace units per module                         */
    uint16_t  order; /* first found (keeps ties in a fixed order)      */
    uint8_t   ratio; /* index into k_ratio_q4                          */
    uint8_t   b_ok;
} fit_t;

typedef struct
{
    float64_t mse;
    float64_t r2;
    float32_t b;
    uint8_t   key;     /* letter * 2 + reversed                      */
    uint8_t   b_rival; /* fit worse than C39G_ERR_MAX_X100: may only
                          block another letter, never be the answer   */
} score_t;

static int32_t g_pos[C39G_MAX_SAMPLES]; /* 1/16 trace unit */
static int32_t g_sig[C39G_MAX_SAMPLES]; /* L above the white floor */
static hump_t  g_hump[MAX_SEG];
static fit_t   g_fit[N_LETTERS * 2u];

static void make_layout(uint32_t letter, int32_t r4_val, bool b_rev,
                        layout_t * p_lay)
{
    int32_t  width[N_ELEMS];
    bool     b_bar[N_ELEMS];
    bool     b_wide[N_ELEMS];
    uint32_t count2 = 0u;
    uint32_t kdx;
    uint32_t idx2;
    int32_t  x_val  = 0;
    uint32_t nb_val = 0u;
    uint32_t nw_val = 0u;

    for (kdx = 0u; kdx < 3u; kdx++)
    {
        const char * p_val = k_pat[(1u == kdx) ? letter : N_LETTERS];
        for (idx2 = 0u; idx2 < 9u; idx2++)
        {
            b_wide[count2] = ('1' == p_val[idx2]);
            /* Cast: r4 * 64 is at most 768 */
            width[count2]  = b_wide[count2] ? (r4_val * 64) : NARROW_Q8;
            b_bar[count2]  = (0u == (idx2 % 2u));
            count2++;
        }
        if (kdx < 2u)
        {
            width[count2]  = NARROW_Q8; /* gap between symbols */
            b_bar[count2]  = false;
            b_wide[count2] = false;
            count2++;
        }
    }
    for (kdx = 0u; kdx < count2; kdx++)
    {
        uint32_t jdx = b_rev ? (count2 - 1u - kdx) : kdx;
        if (b_bar[jdx])
        {
            p_lay->s[nb_val] = x_val;
            p_lay->e[nb_val] = x_val + width[jdx];
            nb_val++;
            if (b_wide[jdx])
            {
                p_lay->wide_x8[nw_val] = (x_val + (width[jdx] / 2)) / 32;
                nw_val++;
            }
        }
        x_val += width[jdx];
    }
    p_lay->total_q8 = x_val;
}

/* Where the trace crosses the threshold between samples i and j */
static float32_t cross(uint32_t idx2, uint32_t jdx, int32_t thr2)
{
    /* Casts: positions and ADC values are far inside float32_t's exact range */
    float32_t yi_val = (float32_t)((2 * g_sig[idx2]) - thr2);
    float32_t yj_val = (float32_t)((2 * g_sig[jdx]) - thr2);
    float32_t pi_val = (float32_t)g_pos[idx2] / (float32_t)Q;
    float32_t pj_val = (float32_t)g_pos[jdx] / (float32_t)Q;

    if (yi_val == yj_val)
    {
        return pi_val;
    }
    return pi_val + (((pj_val - pi_val) * (-yi_val)) / (yj_val - yi_val));
}

/* Humps where 2 * sig > thr2.  Stores up to max_out of them (when p_out
   is not NULL), keeps the 6 highest peaks in top6[] (descending), and
   returns how many there are. */
static uint32_t find_humps(uint32_t count2, int32_t thr2, hump_t * p_out,
                           uint32_t max_out, int32_t p_top6[N_WIDE])
{
    uint32_t count = 0u;
    uint32_t idx2  = 0u;
    uint32_t kdx;

    for (kdx = 0u; kdx < N_WIDE; kdx++)
    {
        p_top6[kdx] = -1;
    }
    while (idx2 < count2)
    {
        if ((2 * g_sig[idx2]) > thr2)
        {
            uint32_t jdx  = idx2;
            int32_t  peak = g_sig[idx2];
            while (((jdx + 1u) < count2) && ((2 * g_sig[jdx + 1u]) > thr2))
            {
                jdx++;
                if (g_sig[jdx] > peak)
                {
                    peak = g_sig[jdx];
                }
            }
            if ((NULL != p_out) && (count < max_out))
            {
                hump_t * p_h = &p_out[count];

                p_h->s = (idx2 > 0u) ? cross(idx2 - 1u, idx2, thr2)
                                     : ((float32_t)g_pos[idx2] / (float32_t)Q);

                p_h->e    = ((jdx + 1u) < count2)
                                ? cross(jdx, jdx + 1u, thr2)
                                : ((float32_t)g_pos[jdx] / (float32_t)Q);
                p_h->c    = 0.5f * (p_h->s + p_h->e);
                p_h->w    = p_h->e - p_h->s;
                p_h->peak = peak;
            }
            /* insert into the 6 highest */
            for (kdx = 0u; kdx < N_WIDE; kdx++)
            {
                if (peak > p_top6[kdx])
                {
                    uint32_t m_val;
                    for (m_val = N_WIDE - 1u; m_val > kdx; m_val--)
                    {
                        p_top6[m_val] = p_top6[m_val - 1u];
                    }
                    p_top6[kdx] = peak;
                    break;
                }
            }
            count++;
            idx2 = jdx + 1u;
        }
        else
        {
            idx2++;
        }
    }
    return count;
}

/* Blurred value (0..PHI_ONE) at d (1/256 module) past an edge */
static int32_t phi(int32_t d_val)
{
    int32_t t_val = d_val + PHI_HALF_Q8;
    int32_t idx;
    int32_t low;

    if (t_val <= 0)
    {
        return 0;
    }
    if (t_val >= (2 * PHI_HALF_Q8))
    {
        return PHI_ONE;
    }
    idx = t_val >> 2;
    low = (int32_t)k_phi[idx];
    return low + ((((int32_t)k_phi[idx + 1] - low) * (t_val & 3)) >> 2);
}

/* Mean squared error of the best gain + offset fit of the blurred
   layout (first edge at a, b trace units per module) to the trace. */
static float64_t model_mse(const layout_t * p_lay, uint32_t count2,
                           float32_t a_val, float32_t b_val, float64_t * p_r2)
{
    float32_t lo_f = (a_val - (3.0f * b_val)) * (float32_t)Q;
    float32_t hi_f =
        (a_val +
         (((float32_t)p_lay->total_q8 / (float32_t)NARROW_Q8) + 3.0f) * b_val) *
        (float32_t)Q;
    /* Casts: rounding the window and scale to integers */
    int32_t low  = (int32_t)lo_f + ((lo_f > (float32_t)(int32_t)lo_f) ? 1 : 0);
    int32_t high = (int32_t)hi_f - ((hi_f < (float32_t)(int32_t)hi_f) ? 1 : 0);
    int32_t a16 =
        (int32_t)((a_val * (float32_t)Q) + ((a_val >= 0.0f) ? 0.5f : -0.5f));
    /* x (1/256 module) = (p - a16) * kq / 65536; inside the window
       |x| < 50 modules, so the product stays below 2^31 */
    int32_t   kq_val = (int32_t)((16.0f * 65536.0f / b_val) + 0.5f);
    int32_t   last_x = INT32_MIN;
    uint32_t  kb_val = 0u; /* first bar that can still touch x */
    int64_t   sm_val = 0;
    int64_t   smm    = 0;
    int64_t   sy_val = 0;
    int64_t   smy    = 0;
    int64_t   syy    = 0;
    uint32_t  cnt    = 0u;
    uint32_t  idx2;
    float64_t dn_val;
    float64_t cmm;
    float64_t cmy;
    float64_t cyy;

    for (idx2 = 0u; idx2 < count2; idx2++)
    {
        int32_t p_val = g_pos[idx2];
        if ((p_val >= low) && (p_val <= high))
        {
            int32_t  x_val = ((p_val - a16) * kq_val) / 65536;
            int32_t  m_val = 0;
            int32_t  y_val = g_sig[idx2];
            uint32_t kdx;
            if (x_val < last_x)
            {
                kb_val = 0u; /* wheel went back: start the bar search again */
            }
            last_x = x_val;
            while ((kb_val < N_BARS) &&
                   (x_val >= (p_lay->e[kb_val] + PHI_HALF_Q8)))
            {
                kb_val++; /* bar fully behind x: adds 1 - 1 = 0 from now on */
            }
            for (kdx = kb_val;
                 (kdx < N_BARS) && (x_val > (p_lay->s[kdx] - PHI_HALF_Q8));
                 kdx++)
            {
                m_val +=
                    phi(x_val - p_lay->s[kdx]) - phi(x_val - p_lay->e[kdx]);
            }
            /* m and y are at most 4096: products fit 32 bits */
            sm_val += m_val;
            smm += m_val * m_val;
            sy_val += y_val;
            smy += m_val * y_val;
            syy += y_val * y_val;
            cnt++;
        }
    }
    *p_r2 = 0.0;
    if (cnt < MIN_MODEL_N)
    {
        return HUGE_MSE;
    }
    /* Casts: sums to float64_t for the 2-parameter least squares */
    dn_val = (float64_t)cnt;
    cmm = (float64_t)smm - (((float64_t)sm_val * (float64_t)sm_val) / dn_val);
    cmy = (float64_t)smy - (((float64_t)sm_val * (float64_t)sy_val) / dn_val);
    cyy = (float64_t)syy - (((float64_t)sy_val * (float64_t)sy_val) / dn_val);
    if ((cmm <= 0.0) || (cmy <= 0.0)) /* flat model, or bars read white */
    {
        return HUGE_MSE;
    }
    if (cyy > 0.0)
    {
        *p_r2 = (cmy * cmy) / (cmm * cyy); /* share of the trace explained */
    }
    return (cyy - ((cmy * cmy) / cmm)) / dn_val;
}

/* next 6-of-n combination in lexicographic order; false after the last */
static bool next_combo(uint32_t p_idx[N_WIDE], uint32_t count2)
{
    int32_t kdx = (int32_t)N_WIDE - 1;

    /* Cast: k is 0..5 */
    while ((kdx >= 0) && (p_idx[kdx] == (count2 - N_WIDE + (uint32_t)kdx)))
    {
        kdx--;
    }
    if (kdx < 0)
    {
        return false;
    }
    p_idx[kdx]++;
    for (kdx = kdx + 1; kdx < (int32_t)N_WIDE; kdx++)
    {
        p_idx[kdx] = p_idx[kdx - 1] + 1u;
    }
    return true;
}

static void fill_floor(const uint16_t * p_l, uint32_t count2, int32_t fr16)
{
    uint32_t idx2;
    uint32_t j0_val = 0u;
    uint32_t j1_val = 0u;

    for (idx2 = 0u; idx2 < count2; idx2++)
    {
        int32_t  fl_val;
        uint32_t kdx;
        while (((j0_val + 1u) < count2) &&
               (g_pos[j0_val] < (g_pos[idx2] - fr16)))
        {
            j0_val++;
        }
        while (((j1_val + 1u) < count2) &&
               (g_pos[j1_val + 1u] <= (g_pos[idx2] + fr16)))
        {
            j1_val++;
        }
        fl_val = (int32_t)p_l[idx2];
        for (kdx = j0_val; kdx <= j1_val; kdx++)
        {
            if ((int32_t)p_l[kdx] < fl_val)
            {
                fl_val = (int32_t)p_l[kdx];
            }
        }
        g_sig[idx2] = (int32_t)p_l[idx2] - fl_val;
    }
}

/* Steps 2-7 on g_sig / g_pos.  hi_permille 0: humps at half the 6th
   highest; otherwise at hi_permille / 1000 of the darkest point.
   min_hump: how far (raw) a hump must rise above the floor at all. */
static c39g_result_t decode_humps(uint32_t count2, int32_t hi_permille,
                                  int32_t       min_hump, void (*p_yield)(void),
                                  c39g_info_t * p_info)
{
    int32_t   top6[N_WIDE];
    int32_t   thr2;
    uint32_t  n_h;
    uint32_t  n_c;
    uint32_t  cand[N_CAND];
    int32_t   cq_val[N_CAND]; /* candidate centres, 1/16 unit */
    int32_t   cw_val[N_CAND]; /* candidate widths,  1/16 unit */
    uint32_t  idx2;
    uint32_t  kdx;
    uint32_t  ri_val;
    uint32_t  key;
    uint16_t  order = 0u;
    uint8_t   good[N_LETTERS * 2u];
    uint32_t  n_good = 0u;
    score_t   sc_val[TOP_MODEL + N_RIVAL];
    uint32_t  n_sc;
    uint32_t  n_riv = 0u;
    int32_t   base;
    float64_t margin;

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

    /* 2. humps: threshold at half the 6th highest */
    if (find_humps(count2, 2 * min_hump, NULL, 0u, top6) < N_WIDE)
    {
        /* blurred: thin gaps stay above min_hump and the bars run
           together.  Try again at half the darkest hump. */
        if ((top6[0] <= (2 * min_hump)) ||
            (find_humps(count2, top6[0], NULL, 0u, top6) < N_WIDE))
        {
            return C39G_FEW;
        }
    }
    thr2 = (top6[N_WIDE - 1u] > (2 * min_hump)) ? top6[N_WIDE - 1u]
                                                : (2 * min_hump);
    if (hi_permille > 0)
    {
        /* Strong blur: thin gaps never come back near white, so the bars
           of a group run together at half height.  Only the wide bars
           reach the top (the sensor saturates on them). */
        int32_t top = 0;
        for (idx2 = 0u; idx2 < count2; idx2++)
        {
            if (g_sig[idx2] > top)
            {
                top = g_sig[idx2];
            }
        }
        thr2 = (2 * top * hi_permille) / 1000;
        if (thr2 < (2 * min_hump))
        {
            return C39G_FEW2;
        }
    }
    n_h             = find_humps(count2, thr2, g_hump, MAX_SEG, top6);
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
        for (kdx = 0u; kdx < n_c; kdx++)
        {
            uint32_t best = MAX_SEG;
            for (idx2 = 0u; idx2 < n_h; idx2++)
            {
                if ((!b_used[idx2]) &&
                    ((MAX_SEG == best) || (g_hump[idx2].w > g_hump[best].w)))
                {
                    best = idx2;
                }
            }
            b_used[best] = true;
        }
        kdx = 0u;
        for (idx2 = 0u; idx2 < n_h; idx2++) /* humps are already in trace order
                                             */
        {
            if (b_used[idx2])
            {
                cand[kdx] = idx2;
                kdx++;
            }
        }
    }
    for (kdx = 0u; kdx < n_c; kdx++)
    {
        /* Casts: trace positions to 1/16 unit, well inside int32 */
        float32_t chr   = g_hump[cand[kdx]].c * (float32_t)Q;
        float32_t width = g_hump[cand[kdx]].w * (float32_t)Q;
        cq_val[kdx]     = (int32_t)(chr + ((chr >= 0.0f) ? 0.5f : -0.5f));
        cw_val[kdx]     = (int32_t)(width + 0.5f);
    }
    /* from the first candidate on: keeps the fit sums small */
    base = cq_val[0];
    for (kdx = 0u; kdx < n_c; kdx++)
    {
        cq_val[kdx] -= base;
    }

    /* 4. lay the 6 wide bars of every letter on 6 of the candidates */
    for (key = 0u; key < (N_LETTERS * 2u); key++)
    {
        g_fit[key].b_ok = 0u;
    }
    for (ri_val = 0u; ri_val < N_RATIOS; ri_val++)
    {
        int32_t r4_val = k_ratio_q4[ri_val];
        for (key = 0u; key < (N_LETTERS * 2u); key++)
        {
            layout_t lay;
            int64_t  sx_val = 0;
            int64_t  sxx    = 0;
            uint32_t idx[N_WIDE];
            fit_t *  p_f = &g_fit[key];

            make_layout(key / 2u, r4_val, (1u == (key % 2u)), &lay);
            for (kdx = 0u; kdx < N_WIDE; kdx++)
            {
                sx_val += lay.wide_x8[kdx];
                sxx += (int64_t)lay.wide_x8[kdx] * lay.wide_x8[kdx];
                idx[kdx] = kdx;
            }
            sxx = ((int64_t)N_WIDE * sxx) - (sx_val * sx_val);
            do
            {
                int64_t   sp_val = 0;
                int64_t   sxp    = 0;
                int64_t   sw_val = 0;
                int64_t   nb_val;
                int64_t   c0_val;
                int64_t   worst = 0;
                float32_t err;
                /* quick look first: a fit within C39G_PRUNE_X10 / 10
                   module needs every centre within twice that of the
                   line through the first and last one */
                {
                    int32_t dq_val = cq_val[idx[N_WIDE - 1u]] - cq_val[idx[0]];
                    int32_t dx_val = lay.wide_x8[N_WIDE - 1u] - lay.wide_x8[0];
                    int32_t lim    = (2 * 8 * C39G_PRUNE_X10 * dq_val) / 10;
                    bool    b_far  = (dq_val <= 0);
                    for (kdx = 1u; (kdx < (N_WIDE - 1u)) && !b_far; kdx++)
                    {
                        int32_t dev =
                            ((cq_val[idx[kdx]] - cq_val[idx[0]]) * dx_val) -
                            (dq_val * (lay.wide_x8[kdx] - lay.wide_x8[0]));
                        b_far = (dev > lim) || (dev < -lim);
                    }
                    if (b_far)
                    {
                        continue;
                    }
                }
                for (kdx = 0u; kdx < N_WIDE; kdx++)
                {
                    sp_val += cq_val[idx[kdx]];
                    sxp += (int64_t)lay.wide_x8[kdx] * cq_val[idx[kdx]];
                    sw_val += cw_val[idx[kdx]];
                }
                /* slope in (1/16 unit) per (1/8 module) is nb / sxx */
                nb_val = ((int64_t)N_WIDE * sxp) - (sx_val * sp_val);
                if (nb_val <= 0)
                {
                    continue; /* bars in the wrong order */
                }
                /* residual x 6 sxx = 6 sxx q - sxx sp + nb sx - 6 nb x */
                c0_val = (nb_val * sx_val) - (sxx * sp_val);
                for (kdx = 0u; kdx < N_WIDE; kdx++)
                {
                    int64_t r_val =
                        ((int64_t)N_WIDE * sxx * cq_val[idx[kdx]]) + c0_val -
                        ((int64_t)N_WIDE * nb_val * lay.wide_x8[kdx]);
                    r_val = (r_val < 0) ? -r_val : r_val;
                    if (r_val > worst)
                    {
                        worst = r_val;
                    }
                }
                /* Cast: error in modules = worst / (48 nb) */
                err = (float32_t)worst / (48.0f * (float32_t)nb_val);
                if ((0u == p_f->b_ok) || (err < p_f->err))
                {
                    /* hump width 0.4 .. 1.8 of a wide bar:
                       wid = sw sxx / (48 nb), wide bar = r4 / 4 modules */
                    int64_t lhs = 5 * sw_val * sxx;
                    if ((lhs >= (24 * r4_val * nb_val)) &&
                        (lhs <= (108 * r4_val * nb_val)))
                    {
                        /* Casts: final scale to trace units */
                        float64_t bq_val = (float64_t)nb_val / (float64_t)sxx;
                        float64_t aq_val =
                            ((float64_t)sp_val - (bq_val * (float64_t)sx_val)) /
                            (float64_t)N_WIDE;
                        if (0u == p_f->b_ok)
                        {
                            p_f->order = order;
                            order++;
                        }
                        p_f->b_ok  = 1u;
                        p_f->err   = err;
                        p_f->a     = (float32_t)((aq_val + (float64_t)base) /
                                             (float64_t)Q);
                        p_f->b     = (float32_t)(bq_val * 8.0 / (float64_t)Q);
                        p_f->ratio = (uint8_t)ri_val;
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
            (g_fit[key].err < ((float32_t)C39G_ERR_MAX_X100 / 100.0f)))
        {
            /* insertion sort by (err, order) */
            uint32_t m_val = n_good;
            while ((m_val > 0u) &&
                   ((g_fit[good[m_val - 1u]].err > g_fit[key].err) ||
                    ((g_fit[good[m_val - 1u]].err == g_fit[key].err) &&
                     (g_fit[good[m_val - 1u]].order > g_fit[key].order))))
            {
                good[m_val] = good[m_val - 1u];
                m_val--;
            }
            /* Cast: key < 52 */
            good[m_val] = (uint8_t)key;
            n_good++;
        }
    }
    if (0u == n_good)
    {
        float32_t best = 1.0e9f;
        for (key = 0u; key < (N_LETTERS * 2u); key++)
        {
            if ((0u != g_fit[key].b_ok) && (g_fit[key].err < best))
            {
                best = g_fit[key].err;
            }
        }
        /* Cast: shown on the dashboard only */
        p_info->err_x100 =
            (uint16_t)((best > 600.0f) ? 60000.0f : (best * 100.0f));
        return C39G_ERR;
    }

    /* 5b. rivals: the letters that fit best of the rest.  Without them
       a lone letter under the limit would be taken with nothing to beat;
       a test with one thin gap printed black read Z as Q that way. */
    n_sc = (n_good < TOP_MODEL) ? n_good : TOP_MODEL;
    for (kdx = 0u; kdx < N_RIVAL; kdx++)
    {
        uint32_t pick = N_LETTERS * 2u;
        for (key = 0u; key < (N_LETTERS * 2u); key++)
        {
            bool b_taken =
                (0u == g_fit[key].b_ok) ||
                (g_fit[key].err < ((float32_t)C39G_ERR_MAX_X100 / 100.0f));
            for (idx2 = 0u; (idx2 < n_riv) && !b_taken; idx2++)
            {
                b_taken = (good[n_sc + idx2] == key);
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
    for (kdx = 0u; kdx < (n_sc + n_riv); kdx++)
    {
        static const float32_t k_db[3] = { 0.985f, 1.0f, 1.015f };
        const fit_t *          p_f     = &g_fit[good[kdx]];
        layout_t               lay;
        float64_t              best    = 0.0;
        float64_t              best_r2 = 0.0;
        float32_t              best_b  = p_f->b;
        uint32_t               di_val;
        uint32_t               ai_val;

        make_layout(good[kdx] / 2u, k_ratio_q4[p_f->ratio],
                    (1u == (good[kdx] % 2u)), &lay);
        for (di_val = 0u; di_val < 3u; di_val++)
        {
            for (ai_val = 0u; ai_val < 9u; ai_val++)
            {
                /* Cast: shift -1 .. +1 module in quarter steps */
                float32_t da_val = ((float32_t)ai_val - 4.0f) / 4.0f;
                float32_t bb_val = p_f->b * k_db[di_val];
                float64_t r2_val = 0.0;
                float64_t mse    = model_mse(
                    &lay, count2, p_f->a + (da_val * p_f->b), bb_val, &r2_val);
                if (((0u == di_val) && (0u == ai_val)) || (mse < best))
                {
                    best    = mse;
                    best_r2 = r2_val;
                    best_b  = bb_val;
                }
            }
        }
        /* insertion sort by (mse, letter, direction) */
        {
            uint32_t m_val = kdx;
            while ((m_val > 0u) && ((sc_val[m_val - 1u].mse > best) ||
                                    ((sc_val[m_val - 1u].mse == best) &&
                                     (sc_val[m_val - 1u].key > good[kdx]))))
            {
                sc_val[m_val] = sc_val[m_val - 1u];
                m_val--;
            }
            sc_val[m_val].mse     = best;
            sc_val[m_val].r2      = best_r2;
            sc_val[m_val].b       = best_b;
            sc_val[m_val].key     = good[kdx];
            sc_val[m_val].b_rival = (kdx >= n_sc) ? 1u : 0u;
        }
        if (NULL != p_yield)
        {
            p_yield();
        }
    }

    /* 7. the best letter must clearly beat the best other letter */
    n_sc += n_riv;
    margin = 9.9;
    for (kdx = 1u; kdx < n_sc; kdx++)
    {
        if ((sc_val[kdx].key / 2u) != (sc_val[0].key / 2u))
        {
            /* Cast: letters are 'A' + 0..25 */
            p_info->runner = (char)('A' + (int32_t)(sc_val[kdx].key / 2u));
            if (sc_val[0].mse > 0.0)
            {
                margin = (sc_val[kdx].mse / sc_val[0].mse) - 1.0;
            }
            break;
        }
    }
    {
        const fit_t * p_f  = &g_fit[sc_val[0].key];
        /* Casts: rounded values for the dashboard; letters 'A' + 0..25 */
        p_info->best       = (char)('A' + (int32_t)(sc_val[0].key / 2u));
        p_info->b_reversed = (1u == (sc_val[0].key % 2u));
        p_info->err_x100   = (uint16_t)((p_f->err * 100.0f) + 0.5f);
        p_info->margin_x100 =
            (uint16_t)((margin > 600.0) ? 60000.0 : ((margin * 100.0) + 0.5));
        p_info->module_x10 = (uint16_t)((sc_val[0].b * 10.0f) + 0.5f);
        p_info->ratio_x100 = (uint16_t)(k_ratio_q4[p_f->ratio] * 25);
        p_info->fit_x100   = (uint8_t)((sc_val[0].r2 * 100.0) + 0.5);
    }
    if ((0u != sc_val[0].b_rival) ||
        (margin < ((float64_t)C39G_MARGIN_X100 / 100.0)))
    {
        return C39G_TIE; /* a rival fits the trace better, or about as well */
    }
    if (sc_val[0].r2 < ((float64_t)C39G_MIN_FIT_X100 / 100.0))
    {
        return C39G_WEAK; /* the bars do not look like this letter */
    }
    p_info->letter = p_info->best;
    return C39G_OK;
}

int32_t c39g_min_hump(int32_t noise_margin)
{
    return (noise_margin > 0) ? (C39G_HUMP_NOISE_K * noise_margin)
                              : C39G_MIN_HUMP;
}

c39g_result_t c39g_decode(const uint16_t * p_l, const uint8_t * p_fwd,
                          const int8_t * p_yaw, uint32_t count2,
                          bool          b_use_dist, void (*p_yield)(void),
                          c39g_info_t * p_info)
{
    return c39g_decode_hump(p_l, p_fwd, p_yaw, count2, b_use_dist,
                            C39G_MIN_HUMP, p_yield, p_info);
}

c39g_result_t c39g_decode_hump(const uint16_t * p_l, const uint8_t * p_fwd,
                               const int8_t * p_yaw, uint32_t count2,
                               bool b_use_dist, int32_t             min_hump,
                               void (*p_yield)(void), c39g_info_t * p_info)
{
    uint32_t      idx2;
    c39g_result_t res;
    bool          b_dist = b_use_dist && (NULL != p_fwd) && (NULL != p_yaw);

    if (NULL == p_info)
    {
        return C39G_ARG;
    }
    p_info->try_n      = 0u;
    p_info->first_res  = (uint8_t)C39G_ARG;
    p_info->first_best = 0;
    if ((NULL == p_l) || (count2 < MIN_MODEL_N) ||
        (count2 > C39G_MAX_SAMPLES) || (min_hump < 1))
    {
        return C39G_ARG;
    }

    /* 1. position and the signal above a rolling white floor */
    {
        int32_t pos = 0;
        for (idx2 = 0u; idx2 < count2; idx2++)
        {
            if (b_dist)
            {
                /* fwd - yaw = (L + R) - (R - L) = 2 x left wheel */
                pos += ((int32_t)p_fwd[idx2] - (int32_t)p_yaw[idx2]) * Q;
            }
            else
            {
                /* Cast: i < C39G_MAX_SAMPLES */
                pos = (int32_t)idx2 * Q;
            }
            g_pos[idx2] = pos;
        }
    }
    fill_floor(p_l, count2,
               (b_dist ? C39G_FLOOR_COUNTS : C39G_FLOOR_SAMPLES) * Q);

    /* 2-7 at half height, then (if humps were found but no letter read)
       once more near the top.  The first try's details are kept unless
       the second one reads a letter. */
    res                = decode_humps(count2, 0, min_hump, p_yield, p_info);
    /* Casts: c39g_result_t values fit 8 bits */
    p_info->try_n      = 1u;
    p_info->first_res  = (uint8_t)res;
    p_info->first_best = p_info->best;
#if (C39G_HI_PERMILLE > 0)
    if ((C39G_OK != res) && (C39G_FEW != res))
    {
        c39g_info_t   high;
        c39g_result_t res_hi =
            decode_humps(count2, C39G_HI_PERMILLE, min_hump, p_yield, &high);
        /* stricter: near the top the humps say less about the thin
           bars, so the model must explain more of the trace */
        if ((C39G_OK == res_hi) && (high.fit_x100 >= C39G_HI_MIN_FIT_X100))
        {
            /* keep what the first try said (a TIE overridden here is
               counted on the dashboard, review R1) */
            high.try_n      = 2u;
            high.first_res  = p_info->first_res;
            high.first_best = p_info->first_best;
            *p_info         = high;
            res             = res_hi;
        }
    }
#endif
    return res;
}

/*** end of file ***/
