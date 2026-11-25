/* S5 supported-edge width estimate (s5_scan_diag.c s5_width_estimate).
     S5-W01  support rule: [0,190,0] is a raw hit but not a supported
             edge; [188,0,192] supports a cluster near 190; one box echo
             plus one distant background echo is one supporting ping;
             three misses stay a miss.
     S5-W02  hand-built edges: fine trip replacing a coarse one brings
             only its own pings; coarse-only outer hit and scan-limit
             hits are CLIPPED; one supported sample is COLLAPSED; none is
             NO_SUPPORT; reversed endpoints are COLLAPSED with width 0
             (never abs()).
     S5-W03  model fixtures (s5_scan_model.h): box 100/150/250 mm,
             centred and 40 mm left/right, stop 120..225 mm, with and
             without a background echo; mirrored scans give mirrored
             estimates; a resolved supported edge is never outside the
             raw edge.
     S5-W04  for every fixture the existing decision is untouched: the
             profile, chosen side, clearances, lateral offset and every
             planned motion step are identical whether or not the
             estimate ran, and the trips are not modified.
   The model table printed with "table" is information, not a check: the
   estimator is not tuned to it. */
#include <assert.h>
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
#include "s5_same.h"
#include "s5_scan_model.h"

static int checks = 0;
static int fixtures = 0;
#define CHECK(c) do { checks++; if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); exit(1); } } while (0)

static s5_trip_t T(int deg, s5_src_t src, uint16_t a, uint16_t b, uint16_t c)
{
    s5_trip_t t;
    t.deg = (int8_t)deg; t.src = (uint8_t)src;
    t.raw[0] = a; t.raw[1] = b; t.raw[2] = c;
    t.used = s5_median_nonzero(t.raw, 3);
    return t;
}
#define C_(d, a, b, c) T(d, S5_SRC_COARSE, a, b, c)
#define F_(d, a, b, c) T(d, S5_SRC_FINE, a, b, c)

/* S5-W04 plus the raw half of the estimate. */
static s5_we_status_t estimate_checked(const s5_trip_t * t, int32_t n, s5_width_est_t * e)
{
    s5_trip_t copy[64];
    s5_sample_t s[64];
    s5_profile_t before, after;
    s5_step_t pb[S5_PLAN_MAX], pa[S5_PLAN_MAX];
    int32_t nb = 0, na = 0, lb = 0, la = 0, i;
    bool okb, oka;
    s5_we_status_t st;

    fixtures++;
    memcpy(copy, t, (size_t)n * sizeof(s5_trip_t));
    s5_trips_to_samples(t, n, s);
    okb = s5_build_profile(s, n, &before);
    if (okb && (S5_SIDE_NONE != (s5_side_t)before.bypass_side))
    {
        lb = s5_lateral_needed_mm(&before, (s5_side_t)before.bypass_side);
        memset(pb, 0, sizeof pb);
        nb = s5_plan_bypass(&before, pb, S5_PLAN_MAX);
    }

    st = s5_width_estimate(t, n, e);
    CHECK((uint8_t)st == e->status);

    for (i = 0; i < n; i++)
    {
        CHECK((copy[i].deg == t[i].deg) && (copy[i].src == t[i].src) && (copy[i].used == t[i].used) &&
              (copy[i].raw[0] == t[i].raw[0]) && (copy[i].raw[1] == t[i].raw[1]) && (copy[i].raw[2] == t[i].raw[2]));
    }
    s5_trips_to_samples(t, n, s);
    oka = s5_build_profile(s, n, &after);
    CHECK(oka == okb);
    CHECK(same_profile(&after, &before));
    if (oka && (S5_SIDE_NONE != (s5_side_t)after.bypass_side))
    {
        la = s5_lateral_needed_mm(&after, (s5_side_t)after.bypass_side);
        memset(pa, 0, sizeof pa);
        na = s5_plan_bypass(&after, pa, S5_PLAN_MAX);
        CHECK(la == lb);
        CHECK(na == nb);
        CHECK(same_steps(pa, pb, na));
    }
    if (!okb)
    {
        CHECK(S5_WE_NO_PROFILE == st);
        return st;
    }
    /* raw numbers are the profile's own */
    CHECK(e->raw_r_deg == before.right_edge_deg);
    CHECK(e->raw_l_deg == before.left_edge_deg);
    CHECK(e->raw_right_mm == before.right_mm);
    CHECK(e->raw_left_mm == before.left_mm);
    CHECK(e->raw_width_mm == before.width_mm);
    if (S5_WE_OK == st)
    {
        /* supported edges lie on or inside the raw edges, and left of right */
        CHECK(e->sup_r_deg >= e->raw_r_deg);
        CHECK(e->sup_l_deg <= e->raw_l_deg);
        CHECK(e->sup_r_deg < e->sup_l_deg);
        CHECK(e->sup_left_mm > e->sup_right_mm);
        CHECK(e->sup_width_mm == (uint16_t)(e->sup_left_mm - e->sup_right_mm));
        CHECK((e->sup_r_support >= S5_WE_MIN_SUPPORT) && (e->sup_l_support >= S5_WE_MIN_SUPPORT));
    }
    else
    {
        CHECK((0 == e->sup_width_mm) && (0 == e->sup_left_mm) && (0 == e->sup_right_mm));
    }
    return st;
}

static void check_mirror(const s5_trip_t * t, int32_t n, const s5_width_est_t * e)
{
    s5_trip_t m[64];
    s5_width_est_t me;
    mirror_trips(t, n, m);
    (void)estimate_checked(m, n, &me);
    CHECK(me.status == e->status);
    CHECK(me.raw_width_mm == e->raw_width_mm);
    CHECK(me.raw_r_deg == -e->raw_l_deg);
    if (S5_WE_OK == e->status)
    {
        CHECK(me.sup_width_mm == e->sup_width_mm);
        CHECK(me.sup_r_deg == -e->sup_l_deg);
        CHECK(me.sup_l_deg == -e->sup_r_deg);
    }
}

/* "log": model scans printed in the robot's "s5d:" format
   (task_obstacle_scan.c print_diag) for the s5_replay round trip. */
static int print_log(void)
{
    static const double offs[] = { -40, 0, 40 };
    char line[256];
    int i, id = 0;
    for (i = 0; i < 3; i++)
    {
        s5_model_t m = { 180, offs[i], 150, 15.0, 30.0, { 0, 0, 0 } };
        s5_trip_t t[64];
        s5_sample_t s[64];
        s5_profile_t p;
        int32_t n = model_scan(&m, t, 64), first;
        id++;
        s5_trips_to_samples(t, n, s);
        if (!s5_build_profile(s, n, &p)) { return 1; }
        printf("12:00:0%d.123 -> s5d: #%d scan dur=%dms fb=0 coarse=5 fine=%d n=%d\n", i, id, 2400 + i, n - 2, n);
        for (first = 0; first < n; first += 8)
        {
            (void)s5_format_trips(line, (int32_t)sizeof line, t, first, ((n - first) > 8) ? 8 : (n - first));
            printf("s5d: #%d p %d/%d:%s\n", id, (first / 8) + 1, (n + 7) / 8, line);
        }
        printf("s5d: #%d raw d=%d R%d:%d L%d:%d rmm=%d lmm=%d w=%d side=%d lat=%d\n", id, p.distance_mm,
               p.right_edge_deg, 0, p.left_edge_deg, 0, p.right_mm, p.left_mm, p.width_mm, p.bypass_side,
               s5_lateral_needed_mm(&p, (s5_side_t)p.bypass_side));
    }
    return 0;
}

int main(int argc, char ** argv)
{
    bool b_table = (argc > 1) && (0 == strcmp(argv[1], "table"));
    if ((argc > 1) && (0 == strcmp(argv[1], "log"))) { return print_log(); }
    s5_width_est_t e;

    /* ---- S5-W01 support rule ---- */
    {
        s5_trip_t a = F_(-25, 0, 190, 0), b = F_(-25, 188, 0, 192), c = F_(-25, 190, 470, 0),
                  d = F_(-25, 0, 0, 0), g = F_(-25, 30, 195, 0);
        CHECK(190 == a.used);                 /* raw hit ... */
        CHECK(1 == support_of(&a, 190));      /* ... but one ping only */
        CHECK(2 == support_of(&b, 190));
        CHECK(1 == support_of(&c, 190));      /* background 470 is a hit, not this cluster */
        CHECK(0 == support_of(&d, 190));
        CHECK(0 == d.used);                   /* three misses stay a miss */
        CHECK(1 == support_of(&g, 190));      /* 30 mm ghost is not this cluster */
    }
    /* [0,190,0] at the edge: raw edge, not the supported one */
    {
        s5_trip_t t[] = { C_(-60, 0, 0, 0), F_(-30, 0, 0, 0), F_(-25, 0, 196, 0), F_(-20, 192, 194, 191),
                          F_(-15, 190, 191, 189), F_(-10, 188, 189, 187), F_(-5, 187, 188, 186), F_(0, 186, 188, 185),
                          F_(5, 187, 188, 186), F_(10, 188, 0, 190), F_(15, 0, 192, 0), F_(20, 0, 0, 0), C_(60, 0, 0, 0) };
        int32_t n = (int32_t)(sizeof t / sizeof t[0]);
        CHECK(S5_WE_OK == estimate_checked(t, n, &e));
        CHECK((-25 == e.raw_r_deg) && (15 == e.raw_l_deg));
        CHECK((1 == e.raw_r_support) && (1 == e.raw_l_support));
        CHECK((-20 == e.sup_r_deg) && (10 == e.sup_l_deg));   /* [188,0,190] supports */
        CHECK(e.sup_width_mm < e.raw_width_mm);
        check_mirror(t, n, &e);
    }

    /* ---- S5-W02 hand-built edge cases ---- */
    /* the fine trip at 30 replaced a coarse 3-echo trip: only the fine
       pings count, so +30 is not supported even though coarse saw it */
    {
        s5_trip_t c[] = { C_(-60, 0, 0, 0), C_(-30, 238, 240, 237), C_(0, 200, 201, 199), C_(30, 236, 238, 235), C_(60, 0, 0, 0) };
        s5_trip_t f[] = { F_(-45, 0, 0, 0), F_(-40, 0, 0, 0), F_(-35, 0, 0, 0), F_(-30, 239, 241, 238), F_(-25, 224, 226, 223),
                          F_(0, 200, 202, 199), F_(25, 224, 225, 223), F_(30, 0, 236, 0), F_(35, 0, 0, 0), F_(40, 0, 0, 0), F_(45, 0, 0, 0) };
        s5_trip_t m[40];
        int32_t n = s5_merge_trips(c, 5, f, 11, m, 40), i, at30 = -1;
        for (i = 0; i < n; i++) { if (30 == m[i].deg) { at30 = i; } }
        CHECK((at30 >= 0) && (S5_SRC_FINE == m[at30].src) && (0 == m[at30].raw[0]));
        CHECK(S5_WE_OK == estimate_checked(m, n, &e));
        CHECK((30 == e.raw_l_deg) && (1 == e.raw_l_support));
        CHECK(25 == e.sup_l_deg);
        CHECK(-30 == e.sup_r_deg);
    }
    /* coarse-only outer hit (fine stops at -45, coarse -60 still hits) */
    {
        s5_trip_t t[] = { C_(-60, 260, 262, 259), F_(-45, 240, 241, 239), F_(-40, 230, 231, 229), F_(-35, 0, 0, 0), C_(0, 0, 0, 0), C_(30, 0, 0, 0), C_(60, 0, 0, 0) };
        CHECK(S5_WE_CLIPPED == estimate_checked(t, 7, &e));
        CHECK(-60 == e.raw_r_deg);
        check_mirror(t, 7, &e);
    }
    /* face wider than the scan: both limits hit */
    {
        s5_trip_t t[] = { C_(-60, 300, 301, 299), F_(-45, 260, 261, 259), C_(-30, 230, 231, 229), C_(0, 200, 201, 199), C_(30, 230, 231, 229), F_(45, 260, 261, 259), C_(60, 300, 302, 299) };
        CHECK(S5_WE_CLIPPED == estimate_checked(t, 7, &e));
    }
    /* too narrow: only one supported sample */
    {
        s5_trip_t t[] = { C_(-60, 0, 0, 0), F_(-10, 0, 0, 0), F_(-5, 0, 205, 0), F_(0, 200, 201, 199), F_(5, 0, 204, 0), F_(10, 0, 0, 0), C_(60, 0, 0, 0) };
        CHECK(S5_WE_COLLAPSED == estimate_checked(t, 7, &e));
        CHECK((0 == e.sup_r_deg) && (0 == e.sup_l_deg));
        CHECK(e.raw_width_mm > 0);
        check_mirror(t, 7, &e);
    }
    /* no supported sample at all */
    {
        s5_trip_t t[] = { C_(-60, 0, 0, 0), F_(-5, 0, 205, 0), F_(0, 0, 200, 0), F_(5, 0, 204, 0), C_(60, 0, 0, 0) };
        CHECK(S5_WE_NO_SUPPORT == estimate_checked(t, 5, &e));
    }
    /* reversed endpoints: right edge offset ends up left of the left one */
    {
        s5_trip_t t[] = { C_(-60, 0, 0, 0), F_(-15, 0, 0, 0), F_(-10, 100, 101, 99), F_(-5, 240, 241, 239), F_(0, 0, 0, 0), C_(60, 0, 0, 0) };
        CHECK(S5_WE_COLLAPSED == estimate_checked(t, 6, &e));
        CHECK(0 == e.sup_width_mm);
    }
    /* nothing there */
    {
        s5_trip_t t[] = { C_(-60, 0, 0, 0), C_(-30, 0, 0, 0), C_(0, 0, 0, 0), C_(30, 0, 0, 0), C_(60, 0, 0, 0) };
        CHECK(S5_WE_NO_PROFILE == estimate_checked(t, 5, &e));
        CHECK(S5_WE_NO_PROFILE == s5_width_estimate(NULL, 5, &e));
    }
    CHECK(0 == strcmp("ok", s5_we_name(S5_WE_OK)));
    CHECK(0 == strcmp("collapsed", s5_we_name(S5_WE_COLLAPSED)));
    CHECK(0 == strcmp("?", s5_we_name(200)));

    /* ---- S5-W03 / W04 model fixtures ---- */
    {
        static const double widths[] = { 100, 150, 250 };
        static const double offs[] = { -40, 0, 40 };
        static const double dists[] = { 120, 150, 180, 200, 225 };
        static const uint16_t bgs[2][3] = { { 0, 0, 0 }, { 0, 455, 0 } };
        int wi, oi, di, bi, resolved = 0, total = 0;

        if (b_table) { printf("box  off  stop bg | raw w  sup w  status\n"); }
        for (wi = 0; wi < 3; wi++)
            for (oi = 0; oi < 3; oi++)
                for (di = 0; di < 5; di++)
                    for (bi = 0; bi < 2; bi++)
                    {
                        s5_model_t m = { dists[di], offs[oi], widths[wi], 15.0, 30.0, { bgs[bi][0], bgs[bi][1], bgs[bi][2] } };
                        s5_trip_t t[64];
                        int32_t n = model_scan(&m, t, 64);
                        s5_we_status_t st = estimate_checked(t, n, &e);
                        total++;
                        resolved += (S5_WE_OK == st) ? 1 : 0;
                        check_mirror(t, n, &e);
                        if (b_table)
                        {
                            printf("%4.0f %+4.0f %5.0f %2d | %5d  %5d  %s\n", widths[wi], offs[oi], dists[di], bi,
                                   e.raw_width_mm, e.sup_width_mm, s5_we_name(e.status));
                        }
                    }
        CHECK(total == 90);
        CHECK(resolved > 0);
    }

    printf("[TEST] S5-W01..W04 PASS  supported-edge estimate beside an unchanged bypass  (%d fixtures, %d checks)\n",
           fixtures, checks);
    return 0;
}
