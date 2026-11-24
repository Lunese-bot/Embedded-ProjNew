/* s5_replay: read the robot's USB log (the "s5d:" records of
   task_obstacle_scan.c) and run every captured scan through the
   production code again, raw edges and supported edges side by side.

     make s5_replay
     ./s5_replay --width 150 --offset 0 < centred_150mm.log

   --width  real face width (mm), --offset  real box centre, + = left of
   the robot's centre line (mm).  Without them only the numbers are
   printed.  One log file per box position.  Edge errors are signed
   "outward positive": + = the edge is placed further out than the real
   corner (over-read), - = inside it (under-read).
   check=ok means the replayed raw profile equals the robot's own "raw"
   line (same code, same pings); MISMATCH means the log and this build
   disagree. */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define APP_H
typedef int ID;
typedef int ER;

#include "../../../app_program/subsystem5/obstacle_logic.c"
#include "../../../app_program/subsystem5/s5_scan_diag.c"

typedef struct
{
    long     id;
    int      dur_ms, fb, n_expect;
    int      n;
    s5_trip_t t[64];
    bool     have_raw;
    int      raw_w, raw_r, raw_l;
} scan_t;

typedef struct { int n, unresolved; double sum, sumL, sumR; int lo, hi, worst_under; int edges; } stats_t;

static bool g_truth = false;
static double g_w = 0, g_off = 0;
static stats_t g_raw = { 0, 0, 0, 0, 0, 100000, -100000, 100000, 0 };
static stats_t g_sup = { 0, 0, 0, 0, 0, 100000, -100000, 100000, 0 };
static int g_scans = 0, g_maxdur = 0, g_mismatch = 0;

static void add(stats_t * s, int w, int rmm, int lmm)
{
    int bias = w - (int)g_w;
    s->n++;
    s->sum += bias;
    if (bias < s->lo) { s->lo = bias; }
    if (bias > s->hi) { s->hi = bias; }
    s->sumL += lmm - (g_off + g_w / 2.0);
    s->sumR += (g_off - g_w / 2.0) - rmm;
    {
        int el = (int)(lmm - (g_off + g_w / 2.0)), er = (int)((g_off - g_w / 2.0) - rmm);
        int m = (el < er) ? el : er;
        if (m < s->worst_under) { s->worst_under = m; }
    }
}

static void finish(scan_t * sc)
{
    s5_width_est_t e;
    const char * check = "-";

    if (sc->n <= 0) { return; }
    (void)s5_width_estimate(sc->t, sc->n, &e);
    if (sc->have_raw)
    {
        bool same = (e.raw_width_mm == sc->raw_w) && (e.raw_r_deg == sc->raw_r) && (e.raw_l_deg == sc->raw_l);
        check = same ? "ok" : "MISMATCH";
        g_mismatch += same ? 0 : 1;
    }
    if ((sc->n_expect > 0) && (sc->n != sc->n_expect)) { check = "INCOMPLETE"; g_mismatch++; }
    g_scans++;
    if (sc->dur_ms > g_maxdur) { g_maxdur = sc->dur_ms; }
    printf("#%-3ld dur=%4dms fb=%d | raw R%3d:%4d L%3d:%4d w=%4d pings R%d/3 L%d/3 | sup %-10s R%3d L%3d w=%4d | check=%s",
           sc->id, sc->dur_ms, sc->fb, e.raw_r_deg, e.raw_right_mm, e.raw_l_deg, e.raw_left_mm, e.raw_width_mm,
           e.raw_r_support, e.raw_l_support, s5_we_name(e.status), e.sup_right_mm, e.sup_left_mm, e.sup_width_mm, check);
    if (g_truth && (S5_WE_NO_PROFILE != e.status))
    {
        printf(" | err raw R%+4d L%+4d w%+4d", (int)((g_off - g_w / 2.0) - e.raw_right_mm),
               (int)(e.raw_left_mm - (g_off + g_w / 2.0)), e.raw_width_mm - (int)g_w);
        add(&g_raw, e.raw_width_mm, e.raw_right_mm, e.raw_left_mm);
        if (S5_WE_OK == e.status)
        {
            printf(" sup R%+4d L%+4d w%+4d", (int)((g_off - g_w / 2.0) - e.sup_right_mm),
                   (int)(e.sup_left_mm - (g_off + g_w / 2.0)), e.sup_width_mm - (int)g_w);
            add(&g_sup, e.sup_width_mm, e.sup_right_mm, e.sup_left_mm);
        }
        else
        {
            g_sup.unresolved++;
        }
    }
    printf("\n");
    sc->n = 0;
}

static void summary(const char * tag, const stats_t * s)
{
    if (0 == s->n) { printf("%s: no resolved scans (unresolved %d)\n", tag, s->unresolved); return; }
    printf("%s: n=%d width bias mean %+.1f mm, spread %d mm (%+d..%+d), edge error mean R %+.1f L %+.1f, worst under-read %d mm, unresolved %d\n",
           tag, s->n, s->sum / s->n, s->hi - s->lo, s->lo, s->hi, s->sumR / s->n, s->sumL / s->n,
           (s->worst_under < 0) ? s->worst_under : 0, s->unresolved);
}

/* " C-60:0/0/0=0 F-45:0/212/0=212" */
static void parse_trips(const char * p, scan_t * sc)
{
    while (*p)
    {
        char src;
        int deg, a, b, c, u, used;
        while (*p == ' ') { p++; }
        if (!*p || *p == '\n' || *p == '\r') { break; }
        if (6 != sscanf(p, "%c%d:%d/%d/%d=%d%n", &src, &deg, &a, &b, &c, &u, &used)) { break; }
        if (sc->n < 64)
        {
            s5_trip_t * t = &sc->t[sc->n++];
            t->deg = (int8_t)deg; t->src = (uint8_t)(('F' == src) ? S5_SRC_FINE : S5_SRC_COARSE);
            t->raw[0] = (uint16_t)a; t->raw[1] = (uint16_t)b; t->raw[2] = (uint16_t)c; t->used = (uint16_t)u;
        }
        p += used;
    }
}

int main(int argc, char ** argv)
{
    char line[1024];
    scan_t sc;
    int i;

    memset(&sc, 0, sizeof sc);
    sc.id = -1;
    for (i = 1; i + 1 < argc; i += 2)
    {
        if (0 == strcmp(argv[i], "--width")) { g_w = atof(argv[i + 1]); g_truth = true; }
        else if (0 == strcmp(argv[i], "--offset")) { g_off = atof(argv[i + 1]); }
        else { fprintf(stderr, "usage: s5_replay [--width mm] [--offset mm] < log\n"); return 2; }
    }
    while (fgets(line, sizeof line, stdin))
    {
        const char * p = strstr(line, "s5d: #");
        long id;
        int k = 0;
        if (NULL == p) { continue; }
        p += 6;
        if (1 != sscanf(p, "%ld%n", &id, &k)) { continue; }
        p += k;
        if (0 == strncmp(p, " scan ", 6))
        {
            finish(&sc);
            memset(&sc, 0, sizeof sc);
            sc.id = id;
            (void)sscanf(p, " scan dur=%dms fb=%d coarse=%*d fine=%*d n=%d", &sc.dur_ms, &sc.fb, &sc.n_expect);
        }
        else if ((id == sc.id) && (0 == strncmp(p, " p ", 3)))
        {
            const char * c = strchr(p, ':');
            if (c) { parse_trips(c + 1, &sc); }
        }
        else if ((id == sc.id) && (0 == strncmp(p, " raw d=", 7)))
        {
            int d;
            if (4 == sscanf(p, " raw d=%d R%d:%*d L%d:%*d rmm=%*d lmm=%*d w=%d", &d, &sc.raw_r, &sc.raw_l, &sc.raw_w))
            {
                sc.have_raw = true;
            }
        }
    }
    finish(&sc);
    printf("scans %d, longest scan %d ms (limit %d ms), log/replay mismatches %d\n", g_scans, g_maxdur, 8000 /* S5_SCAN_TMO_MS */, g_mismatch);
    if (g_truth) { summary("raw", &g_raw); summary("sup", &g_sup); }
    return g_mismatch ? 1 : 0;
}
