/* S5 scan evidence (s5_scan_diag.c): the "s5d:" record must not change
   what the robot does.
     S5-D01  s5_median_nonzero = the old us_read_median_mm rule (copied
             below from drv_ultrasonic.c before this change) for every
             combination of 1..5 readings from a value set with misses,
             ties and far echoes.
     S5-D02  [0,190,0] -> 190 (one echo is still a hit), [188,0,192] -> 192,
             [0,0,0] -> 0.
     S5-D03  merged trips give exactly the merged samples (fine replaces
             coarse at the same angle and brings its own pings).
     S5-D04  profile, side, clearance, lateral offset and bypass plan built
             from the trips equal those built from the plain samples.
     S5-D05  exact record text; whole items only; never past the buffer. */
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

static int checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); exit(1); } } while (0)

/* drv_ultrasonic.c us_read_median_mm before this change, minus the
   hardware reads: the readings come from p_in instead. */
static uint16_t legacy_median(const uint16_t * p_in, int32_t samples)
{
    uint16_t vals[5];
    int32_t  got = 0;
    int32_t  idx;
    int32_t  jdx;
    uint16_t reading;
    uint16_t tmp;

    if (samples < 1) { samples = 1; }
    if (samples > 5) { samples = 5; }
    for (idx = 0; idx < samples; idx++)
    {
        reading = p_in[idx];
        if (reading != 0) { vals[got++] = reading; }
    }
    if (0 == got) { return 0; }
    for (idx = 1; idx < got; idx++)
    {
        tmp = vals[idx];
        for (jdx = idx - 1; (jdx >= 0) && (vals[jdx] > tmp); jdx--) { vals[jdx + 1] = vals[jdx]; }
        vals[jdx + 1] = tmp;
    }
    return vals[got / 2];
}

static const uint16_t VALS[] = { 0, 1, 150, 188, 190, 192, 450, 2000 };
#define NV ((int32_t)(sizeof VALS / sizeof VALS[0]))

static void all_medians(uint16_t * buf, int32_t pos, int32_t n)
{
    int32_t v;
    if (pos == n)
    {
        CHECK(s5_median_nonzero(buf, n) == legacy_median(buf, n));
        return;
    }
    for (v = 0; v < NV; v++) { buf[pos] = VALS[v]; all_medians(buf, pos + 1, n); }
}

static s5_trip_t trip(int deg, s5_src_t src, uint16_t a, uint16_t b, uint16_t c)
{
    s5_trip_t t;
    t.deg = (int8_t)deg; t.src = (uint8_t)src;
    t.raw[0] = a; t.raw[1] = b; t.raw[2] = c;
    t.used = s5_median_nonzero(t.raw, 3);
    return t;
}

static void same_decision(const s5_sample_t * s, int32_t n, const s5_trip_t * t, int32_t nt)
{
    s5_sample_t ts[64];
    s5_profile_t a, b;
    s5_step_t pa[S5_PLAN_MAX], pb[S5_PLAN_MAX];
    int32_t na, nb;
    bool ba, bb;

    CHECK(nt == n);
    s5_trips_to_samples(t, nt, ts);
    CHECK(same_samples(ts, s, n));
    ba = s5_build_profile(s, n, &a);
    bb = s5_build_profile(ts, nt, &b);
    CHECK(ba == bb);
    CHECK(same_profile(&a, &b));
    if (ba && (S5_SIDE_NONE != (s5_side_t)a.bypass_side))
    {
        CHECK(s5_lateral_needed_mm(&a, (s5_side_t)a.bypass_side) ==
              s5_lateral_needed_mm(&b, (s5_side_t)b.bypass_side));
        memset(pa, 0, sizeof pa); memset(pb, 0, sizeof pb);
        na = s5_plan_bypass(&a, pa, S5_PLAN_MAX);
        nb = s5_plan_bypass(&b, pb, S5_PLAN_MAX);
        CHECK(na == nb);
        CHECK(same_steps(pa, pb, na));
    }
}

int main(void)
{
    uint16_t buf[5];
    int32_t n;

    /* S5-D01 */
    for (n = 1; n <= 5; n++) { all_medians(buf, 0, n); }

    /* S5-D02 */
    { uint16_t r[3] = { 0, 190, 0 };   CHECK(190 == s5_median_nonzero(r, 3)); }
    { uint16_t r[3] = { 188, 0, 192 }; CHECK(192 == s5_median_nonzero(r, 3)); }
    { uint16_t r[3] = { 0, 0, 0 };     CHECK(0 == s5_median_nonzero(r, 3)); }
    { uint16_t r[3] = { 190, 450, 0 }; CHECK(450 == s5_median_nonzero(r, 3)); }
    CHECK(0 == s5_median_nonzero(NULL, 3));

    /* S5-D03 / D04: coarse + fine with duplicate angles, both orders of
       magnitude of the fine list, and coarse alone (fallback) */
    {
        s5_trip_t c[5], f[32], m[40];
        s5_sample_t cs[5], fs[32], ms[40];
        int32_t nc = 0, nf = 0, nm, nms, i;

        c[nc++] = trip(-60, S5_SRC_COARSE, 0, 0, 0);
        c[nc++] = trip(-30, S5_SRC_COARSE, 0, 240, 0);
        c[nc++] = trip(0, S5_SRC_COARSE, 200, 201, 199);
        c[nc++] = trip(30, S5_SRC_COARSE, 236, 0, 238);
        c[nc++] = trip(60, S5_SRC_COARSE, 0, 0, 0);
        for (i = -45; i <= 45; i += 5)
        {
            uint16_t r = (uint16_t)((abs(i) <= 30) ? (200 + (abs(i) * abs(i)) / 25) : 0);
            f[nf++] = (abs(i) == 30) ? trip(i, S5_SRC_FINE, 0, r, 0) : trip(i, S5_SRC_FINE, r, r, (uint16_t)(r ? r + 2 : 0));
        }
        s5_trips_to_samples(c, nc, cs);
        s5_trips_to_samples(f, nf, fs);
        nm  = s5_merge_trips(c, nc, f, nf, m, 40);
        nms = s5_merge_samples(cs, nc, fs, nf, ms, 40);
        CHECK(nm == nms);
        CHECK(nm == nf + 2); /* -60 and +60 kept, -30 0 30 replaced */
        for (i = 0; i < nm; i++)
        {
            if ((m[i].deg == -30) || (m[i].deg == 0) || (m[i].deg == 30)) { CHECK(S5_SRC_FINE == m[i].src); }
            if (m[i].deg == 30) { CHECK((0 == m[i].raw[0]) && (0 == m[i].raw[2])); }
        }
        same_decision(ms, nms, m, nm);
        same_decision(cs, nc, c, nc);
        /* capacity limit: same truncation as the sample merge */
        nm  = s5_merge_trips(c, nc, f, nf, m, 7);
        nms = s5_merge_samples(cs, nc, fs, nf, ms, 7);
        CHECK(nm == nms);
        same_decision(ms, nms, m, nm);
    }

    /* S5-D05 */
    {
        char out[128];
        s5_trip_t t[3];
        t[0] = trip(-60, S5_SRC_COARSE, 0, 0, 0);
        t[1] = trip(-5, S5_SRC_FINE, 188, 0, 192);
        t[2] = trip(45, S5_SRC_FINE, 2000, 0, 0);
        n = s5_format_trips(out, (int32_t)sizeof out, t, 0, 3);
        CHECK(0 == strcmp(out, " C-60:0/0/0=0 F-5:188/0/192=192 F45:2000/0/0=2000"));
        CHECK(n == (int32_t)strlen(out));
        memset(out, 'x', sizeof out);
        n = s5_format_trips(out, 32, t, 0, 3); /* " C-60:0/0/0=0" 13 + 18 = 31 */
        CHECK(0 == strcmp(out, " C-60:0/0/0=0 F-5:188/0/192=192"));
        CHECK((31 == n) && ('x' == out[32]));
        n = s5_format_trips(out, 31, t, 0, 3);
        CHECK((0 == strcmp(out, " C-60:0/0/0=0")) && (13 == n));
        n = s5_format_trips(out, 1, t, 0, 3);
        CHECK((0 == n) && ('\0' == out[0]));
        CHECK(0 == s5_format_trips(out, 0, t, 0, 3));
        n = s5_format_trips(out, (int32_t)sizeof out, NULL, 0, 3);
        CHECK((0 == n) && ('\0' == out[0]));
    }

    printf("[TEST] S5-D01..D05 PASS  scan evidence leaves the median, profile and plan unchanged  (%d checks)\n", checks);
    return 0;
}
