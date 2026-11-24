/*
 * bc_replay.c -- replay saved dashboard L traces (10 ms records) through
 * the REAL subsystem 3 barcode code on the PC:
 *   line_algo.c  bc_edge_step_pos()           (edge finder)
 *   code39.c     code39_decode_ex/_partial()  (element decoders)
 *   code39_geom.c c39g_decode()               (shape read)
 * The glue copies task_line.c barcode_runs_step() and task_barcode.c
 * on_run() / timeouts / try_decode() / decode() / shape_read().  If those
 * change, change this file the same way.
 *
 * The edge finder is the one task_line.c uses: bc_edge_step_adapt() with
 * S3_BC_EDGE_ADAPT 1 (b_freeze = a read is open), else bc_edge_step_pos()
 * with S3_BC_EDGE_SWING / S3_BC_EDGE_FLOOR.  Build with
 * -DS3_BC_EDGE_ADAPT=0 to replay the old path.
 *
 * Trace file: one 10 ms record per line, "L_raw fwd yaw" where
 * fwd = left + right wheel counts and yaw = right - left in that record
 * (the dashboard CSV columns).  Position = 2 x left wheel = sum(fwd - yaw),
 * the same as s3_bc_pos() with S3_BC_DIST_WHEEL 1.
 *
 * usage: bc_replay [-v] [-prime] [-norearm] trace.txt [trace.txt ...]
 * After every failed read the white level is relearned, as the robot's
 * retry rearm does; -norearm turns that off.
 * Sample numbers count from the first record of the file; with -prime they
 * include the PRIME_N records put in front (see prime()).
 * One line per read:
 *   <file> start=<sample> end=<sample> why=<..> elems=<n> path=<..> out=<..>
 * and one summary line per file.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "s3_config.h"
#include "line_algo.h"
#include "code39.h"
#include "code39_geom.h"

#define NMAX 4000
#define T_US 10000u /* one record */
/* = S3_LTRACE_N (subsystem3.h needs the kernel headers) */
#define LTRACE_N 1400u

static uint16_t g_l[NMAX];
static uint8_t  g_f[NMAX];
static int8_t   g_y[NMAX];
static int      g_n;
static int      g_verbose;
static int      g_prime; /* -prime: see prime() */
static int      g_primed_n; /* records added in front */
static int32_t  g_margin;   /* bc_edge_margin() now (adaptive path) */
static int      g_norearm;  /* -norearm: no white relearn after a failed read */
static int      g_rearm;    /* a failed read asked for a rearm */
static const char * g_name;

typedef struct
{
    uint8_t  black;
    uint32_t width_us, width_pos, lead_us, lead_pos;
} run_t;

/* barcode_task state */
static uint32_t W[S3_BC_MAX_ELEMENTS], WP[S3_BC_MAX_ELEMENTS];
static uint32_t nw, widest, total_pos, last_run_pos, last_run_t, white_ms;
static int      st_read; /* 0 idle, 1 read */
static int      read_start;
static int      n_starts, n_letters, n_fail;
static char     letters[64];

static uint32_t quiet_us(void)
{
    uint32_t q = widest * S3_BC_QUIET_FACTOR;
    uint32_t m = (uint32_t)S3_BC_QUIET_MIN_MS * 1000u;
    return (q < m) ? m : q;
}
static int dist_live(void) { return total_pos >= 10u; }
static void go_idle(void)
{
    nw = 0; widest = 0; total_pos = 0; st_read = 0;
}

/* task_barcode.c try_decode(); *p_path: what gave the result */
static code39_result_t try_decode(char * text, bool * rev, const char ** p_path)
{
    code39_options_t o;
    code39_result_t  res = CODE39_ERR_LENGTH, rt;
    int              unit = 0;

    o.min_ratio_x10 = S3_BC_MIN_RATIO_X10;
    o.letters_only  = (S3_BC_LETTERS_ONLY != 0);
    o.max_chars     = (uint8_t)S3_BC_MAX_CHARS;
    *p_path = "none";
    if (nw < S3_BC_MIN_ELEMENTS) { return CODE39_ERR_LENGTH; }
    if (total_pos >= S3_BC_DIST_MIN)
    {
        unit = 1;
        res  = code39_decode_ex(WP, nw, &o, text, S3_BC_TEXT_MAX, rev);
        if (CODE39_OK == res) { *p_path = "full/dist"; return res; }
    }
    rt = code39_decode_ex(W, nw, &o, text, S3_BC_TEXT_MAX, rev);
    if ((CODE39_OK == rt) || (0 == unit)) { res = rt; }
    if (CODE39_OK == res) { *p_path = "full/time"; return res; }
#if S3_BC_PARTIAL
    if ((total_pos >= S3_BC_DIST_MIN) &&
        (CODE39_OK == code39_decode_partial(WP, nw, &o, text, S3_BC_TEXT_MAX, rev)))
    { *p_path = "partial/dist"; return CODE39_OK; }
    if (CODE39_OK == code39_decode_partial(W, nw, &o, text, S3_BC_TEXT_MAX, rev))
    { *p_path = "partial/time"; return CODE39_OK; }
    text[0] = 0;
#endif
    return res;
}

static const char * c39_err(code39_result_t r)
{
    static const char * const s[] = { "ok", "param", "length", "ratio",
                                      "pattern", "frame", "charset" };
    return ((unsigned)r < 7u) ? s[r] : "?";
}
static const char * geom_err(c39g_result_t r)
{
    static const char * const s[] = { "ok", "few-humps", "few-humps2", "many",
                                      "nofit", "err", "tie", "weak", "arg" };
    return ((unsigned)r < 9u) ? s[r] : "?";
}

static void decode(int idx, const char * why)
{
    char        text[S3_BC_TEXT_MAX + 2] = { 0 };
    bool        rev  = false;
    const char * path = "none";
    char        out[96];
    code39_result_t res = try_decode(text, &rev, &path);

    snprintf(out, sizeof out, "reject:%s", c39_err(res));
#if S3_BC_GEOM
    if (CODE39_OK != res)
    {
        if (nw >= S3_BC_GEOM_MIN_ELEMENTS)
        {
            /* task_barcode.c shape_read(): frozen ring = last S3_LTRACE_N
               records up to now, from S3_BC_GEOM_LEAD before the start */
            int cnt  = idx + 1;
            int lo   = (cnt > (int)LTRACE_N) ? (cnt - (int)LTRACE_N) : 0;
            int back = (idx - read_start) + (int)S3_BC_GEOM_LEAD;
            int off  = (back < (cnt - lo)) ? (cnt - back) : lo;
            int32_t dist = 0;
            int k;
            c39g_info_t   info;
            c39g_result_t r;
            for (k = off; k < cnt; k++) { dist += (int32_t)g_f[k] - (int32_t)g_y[k]; }
            memset(&info, 0, sizeof info);
            r = c39g_decode_hump(&g_l[off], &g_f[off], &g_y[off], (uint32_t)(cnt - off),
                                 dist >= (int32_t)S3_BC_DIST_MIN,
                                 c39g_min_hump(g_margin), NULL, &info);
            if (C39G_OK == r)
            {
                text[0] = info.letter; text[1] = 0; rev = info.b_reversed;
                res = CODE39_OK; path = "shape";
            }
            else
            {
                snprintf(out, sizeof out, "reject:%s+shape:%s(best %c)",
                         c39_err(res), geom_err(r), info.best ? info.best : '-');
            }
        }
        else
        {
            snprintf(out, sizeof out, "reject:%s+shape:not-tried(<%u)",
                     c39_err(res), (unsigned)S3_BC_GEOM_MIN_ELEMENTS);
        }
    }
#endif
    if (CODE39_OK == res)
    {
        snprintf(out, sizeof out, "LETTER:%s%s", text, rev ? "(rev)" : "");
        n_letters++;
        if (strlen(letters) + strlen(text) + 2 < sizeof letters)
        {
            strcat(letters, text);
        }
    }
    else
    {
        n_fail++;
        g_rearm = 1;
    }
    printf("%s start=%d end=%d why=%s elems=%u path=%s out=%s\n", g_name,
           read_start, idx, why, nw, path, out);
    if (g_verbose)
    {
        uint32_t k;
        printf("   counts:");
        for (k = 0; k < nw; k++) { printf(" %u", WP[k]); }
        printf("\n   ms:");
        for (k = 0; k < nw; k++) { printf(" %u", W[k] / 1000u); }
        printf("\n");
    }
    go_idle();
}

/* task_barcode.c on_run() */
static void on_run(const run_t * r, int idx)
{
    if (0 == st_read)
    {
        if (!r->black && (r->width_us >= (uint32_t)S3_BC_START_WHITE_MS * 1000u))
        {
            st_read = 1; read_start = idx; nw = 0; widest = 0; total_pos = 0;
            n_starts++;
        }
        return;
    }
    if (r->black && (r->width_us > (uint32_t)S3_BC_ELEMENT_MAX_MS * 1000u))
    {
        printf("%s start=%d end=%d why=black-too-long elems=%u path=none out=abort\n",
               g_name, read_start, idx, nw);
        n_fail++;
        g_rearm = 1;
        go_idle();
        return;
    }
    if (!r->black && (nw > 0u) &&
        (dist_live() ? (r->width_pos >= S3_BC_QUIET_COUNTS) : (r->width_us >= quiet_us())))
    {
        decode(idx, "quiet-run");
        return;
    }
    if (nw < S3_BC_MAX_ELEMENTS)
    {
        if (r->black && (nw > 0u))
        {
            W[nw - 1u] += r->lead_us; WP[nw - 1u] += r->lead_pos;
            total_pos += r->lead_pos;
        }
        W[nw] = r->width_us; WP[nw] = r->width_pos; total_pos += r->width_pos;
        nw++;
        if (r->width_us > widest) { widest = r->width_us; }
        if (nw >= S3_BC_MAX_ELEMENTS) { decode(idx, "max-elements"); }
#if S3_BC_EARLY_END
        else if ((nw >= S3_BC_MIN_ELEMENTS) && (9u == (nw % 10u)))
        {
            char t[S3_BC_TEXT_MAX + 2]; bool rv; const char * p;
            if (CODE39_OK == try_decode(t, &rv, &p)) { decode(idx, "early-end"); }
        }
#endif
    }
}

static void replay(void)
{
    bc_edge_t e;
    uint32_t  pos = 0u, rs_us = 0u, rs_pos = 0u;
    uint8_t   level = 0u;
    int       i;

    n_starts = n_letters = n_fail = 0; letters[0] = 0; g_rearm = 0;
    go_idle();
    bc_edge_reset_pos(&e, (int32_t)g_l[0], 0u, 0u);
    for (i = 0; i < g_n; i++)
    {
        uint32_t t = (uint32_t)i * T_US;
        uint32_t eu = t, ep;
        pos += (uint32_t)((int32_t)g_f[i] - (int32_t)g_y[i]);
        ep = pos;
        /* task_line.c barcode_runs_step() */
#if S3_BC_EDGE_ADAPT
        /* vc_sm.c barcode_retry() asks for BARCODE_READ again after a failed
           read; task_barcode.c then bumps g_s3_bc_relearn_seq and
           task_line.c relearns white on its next step (inferred timing:
           the next record). */
        if ((0 != g_rearm) && (0 == g_norearm)) { bc_edge_relearn(&e); }
        g_rearm = 0;
        e.hold = (uint8_t)S3_BC_ADAPT_HOLD;
        uint8_t b_edge = bc_edge_step_adapt(&e, (int32_t)g_l[i], t, pos, (1 == st_read),
                                            &eu, &ep);
        g_margin = bc_edge_margin(&e);
        if (0U != b_edge)
#else
        e.hold = (S3_BC_EDGE_SWING <= S3_BC_WEAK_SWING_MAX) ? (uint8_t)S3_BC_WEAK_HOLD : 0U;
        if (0U != bc_edge_step_pos(&e, (int32_t)g_l[i], t, pos, S3_BC_EDGE_SWING,
                                   S3_BC_EDGE_FLOOR, &eu, &ep))
#endif
        {
            run_t r;
            r.lead_us = 0u; r.lead_pos = 0u;
            if (0U != level)
            {
                r.lead_us  = e.bar_t_us - rs_us;
                r.lead_pos = e.bar_pos - rs_pos;
                rs_us = e.bar_t_us; rs_pos = e.bar_pos;
            }
            r.width_us = eu - rs_us; r.width_pos = ep - rs_pos; r.black = level;
            level = e.level; rs_us = eu; rs_pos = ep;
            last_run_pos = pos; last_run_t = t; white_ms = 0u;
            if (g_verbose > 1)
            {
                printf("   run %s %u ms %u counts @%d\n", r.black ? "BAR" : "spc",
                       r.width_us / 1000u, r.width_pos, i);
            }
            if (g_verbose > 1) { printf("   white=%d m=%d\n", e.w_x16 / 16, bc_edge_margin(&e)); }
            on_run(&r, i);
        }
        else if (1 == st_read)
        {
            /* barcode_task timeouts (polled every 50 ms by distance) */
            if ((1u == (nw % 2u)) && dist_live())
            {
                if (0 == (i % 5))
                {
                    white_ms += 50u;
                    if ((pos - last_run_pos) >= S3_BC_QUIET_COUNTS) { decode(i, "quiet-dist"); }
                    else if (white_ms >= (uint32_t)S3_BC_QUIET_STOP_MS) { decode(i, "stopped"); }
                }
            }
            else if (1u == (nw % 2u))
            {
                if ((t - last_run_t) >= quiet_us()) { decode(i, "quiet-time"); }
            }
            else if ((0U != level) && ((t - last_run_t) >= (uint32_t)S3_BC_ELEMENT_MAX_MS * 1000u))
            {
                printf("%s start=%d end=%d why=black-too-long elems=%u path=none out=abort\n",
                       g_name, read_start, i, nw);
                n_fail++;
                g_rearm = 1;
                go_idle();
                last_run_t = t;
            }
        }
    }
    if (1 == st_read)
    {
        /* The dashboard froze the trace when the robot decoded, so the
           read the robot made ends at the last record: decode it there. */
        decode(g_n - 1, "trace-end");
    }
    printf("%s SUMMARY starts=%d letters=%d [%s] rejected=%d\n", g_name,
           n_starts, n_letters, letters, n_fail);
}

/* -prime: the robot learns white before a barcode, but many saved traces
   start only a few records before (or inside) one.  Put in front a copy of
   this pass's own steadiest low 40-record stretch, so the learner has the
   approach it would have had.  Diagnostic only: the copy is inferred. */
#define PRIME_N 40
static void prime(void)
{
    int best = -1, best_spread = 1 << 30, i, k;
    int sorted_q;
    {
        /* 25th percentile of the pass: the stretch must lie on white */
        static uint16_t tmp[NMAX];
        memcpy(tmp, g_l, sizeof(uint16_t) * (size_t)g_n);
        for (i = 1; i < g_n; i++)
        {
            uint16_t v = tmp[i];
            for (k = i - 1; (k >= 0) && (tmp[k] > v); k--) { tmp[k + 1] = tmp[k]; }
            tmp[k + 1] = v;
        }
        sorted_q = tmp[g_n / 4];
    }
    for (i = 0; (i + PRIME_N) <= g_n; i++)
    {
        int lo = 1 << 30, hi = 0;
        for (k = i; k < (i + PRIME_N); k++)
        {
            if (g_l[k] < lo) { lo = g_l[k]; }
            if (g_l[k] > hi) { hi = g_l[k]; }
        }
        if ((lo <= sorted_q) && ((hi - lo) < best_spread)) { best_spread = hi - lo; best = i; }
    }
    g_primed_n = 0;
    if ((best < 0) || ((g_n + PRIME_N) > NMAX)) { return; }
    memmove(&g_l[PRIME_N], g_l, sizeof(uint16_t) * (size_t)g_n);
    memmove(&g_f[PRIME_N], g_f, (size_t)g_n);
    memmove(&g_y[PRIME_N], g_y, (size_t)g_n);
    best += PRIME_N;
    for (k = 0; k < PRIME_N; k++)
    {
        g_l[k] = g_l[best + k]; g_f[k] = g_f[best + k]; g_y[k] = g_y[best + k];
    }
    g_n += PRIME_N;
    g_primed_n = PRIME_N;
}

int main(int argc, char ** argv)
{
    int a;
    for (a = 1; a < argc; a++)
    {
        FILE * f;
        int x, y, z;
        if (0 == strcmp(argv[a], "-v")) { g_verbose++; continue; }
        if (0 == strcmp(argv[a], "-prime")) { g_prime = 1; continue; }
        if (0 == strcmp(argv[a], "-norearm")) { g_norearm = 1; continue; }
        f = fopen(argv[a], "r");
        if (NULL == f) { perror(argv[a]); return 1; }
        g_n = 0;
        while ((g_n < NMAX) && (3 == fscanf(f, "%d %d %d", &x, &y, &z)))
        {
            g_l[g_n] = (uint16_t)x; g_f[g_n] = (uint8_t)y; g_y[g_n] = (int8_t)z; g_n++;
        }
        fclose(f);
        g_name = strrchr(argv[a], '/') ? strrchr(argv[a], '/') + 1 : argv[a];
        g_primed_n = 0;
        if (0 != g_prime) { prime(); }
        replay();
    }
    return 0;
}
