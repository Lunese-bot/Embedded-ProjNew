/*
 *	test_s3_host.c
 *	PC unit tests for subsystem 3's pure algorithms (code39.c,
 *	line_algo.c).  No robot needed:
 *
 *	    cd testing/subsystem3/host && make
 *
 *	Output uses the same "[TEST] ID PASS/FAIL" lines as the on-target
 *	tests, so results go into testing/subsystem3/TEST_LOG.md the same way.
 */

#include <stdio.h>
#include <string.h>
#include "code39.h"
#include "line_algo.h"

static int g_pass;
static int g_fail;

#define CHECK(id, cond, desc)                                           \
    do {                                                                \
        if (cond) { g_pass++; printf("[TEST] %s PASS  %s\n", id, desc); } \
        else      { g_fail++; printf("[TEST] %s FAIL  %s   (%s:%d)\n",  \
                                     id, desc, __FILE__, __LINE__); }   \
    } while (0)

static code39_params_t const g_prm = { S3_BC_MIN_RATIO_X10 };

static void reverse(uint32_t * p, uint32_t n)
{
    uint32_t i;
    for (i = 0u; i < n / 2u; i++)
    {
        uint32_t t = p[i]; p[i] = p[n - 1u - i]; p[n - 1u - i] = t;
    }
}

static void test_code39(void)
{
    static char const all[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";
    uint32_t w[200];
    char     text[16];
    char     one[2] = { 0, 0 };
    bool     rev;
    uint32_t n;
    uint32_t i;
    int      ok = 1;

    /* H-C01: every character round-trips, both directions */
    for (i = 0u; i < (uint32_t)strlen(all); i++)
    {
        one[0] = all[i];
        n = code39_encode(one, 10u, 25u, w, 200u);
        if ((29u != n) ||
            (CODE39_OK != code39_decode(w, n, &g_prm, text, sizeof text, &rev)) ||
            (text[0] != all[i]) || rev)
        {
            ok = 0; printf("  fwd fail '%c'\n", all[i]);
        }
        reverse(w, n);
        if ((CODE39_OK != code39_decode(w, n, &g_prm, text, sizeof text, &rev)) ||
            (text[0] != all[i]) || !rev)
        {
            ok = 0; printf("  rev fail '%c'\n", all[i]);
        }
    }
    CHECK("H-C01", ok, "all 43 characters decode forwards and backwards");

    /* H-C02: multi-character text, 2:1 ratio */
    n = code39_encode("LR2", 100u, 200u, w, 200u);
    CHECK("H-C02", (49u == n) &&
          (CODE39_OK == code39_decode(w, n, &g_prm, text, sizeof text, &rev)) &&
          (0 == strcmp(text, "LR2")), "\"*LR2*\" at 2:1 ratio");

    /* H-C03: timing jitter +-20% on every element still decodes */
    n = code39_encode("U", 1000u, 2500u, w, 200u);
    for (i = 0u; i < n; i++)
    {
        w[i] = (i % 3u == 0u) ? (w[i] * 12u) / 10u
             : (i % 3u == 1u) ? (w[i] * 8u) / 10u : w[i];
    }
    CHECK("H-C03", (CODE39_OK == code39_decode(w, n, &g_prm, text, sizeof text, &rev)) &&
          (text[0] == 'U'), "+-20% jitter tolerated at 2.5:1");

    /* H-C04: speed ramp (robot slowing 2x across the barcode) */
    n = code39_encode("S", 1000u, 2500u, w, 200u);
    for (i = 0u; i < n; i++)
    {
        w[i] = (w[i] * (100u + (i * 100u) / n)) / 100u;
    }
    CHECK("H-C04", (CODE39_OK == code39_decode(w, n, &g_prm, text, sizeof text, &rev)) &&
          (text[0] == 'S'), "2x speed change across barcode tolerated");

    /* H-C05: wrong length rejected */
    CHECK("H-C05", CODE39_ERR_LENGTH ==
          code39_decode(w, n - 1u, &g_prm, text, sizeof text, &rev),
          "element count not 10n-1 rejected");

    /* H-C06: all elements equal -> ratio error */
    for (i = 0u; i < 29u; i++) { w[i] = 100u; }
    CHECK("H-C06", CODE39_ERR_RATIO ==
          code39_decode(w, 29u, &g_prm, text, sizeof text, &rev),
          "no wide/narrow contrast rejected");

    /* H-C07: missing start/stop -> frame error */
    n = code39_encode("AB", 10u, 25u, w, 200u);
    CHECK("H-C07", CODE39_OK !=
          code39_decode(&w[10], 29u, &g_prm, text, sizeof text, &rev),
          "barcode without '*' framing rejected");

    /* H-C08: text buffer too small */
    n = code39_encode("ABCD", 10u, 25u, w, 200u);
    CHECK("H-C08", CODE39_ERR_PARAM ==
          code39_decode(w, n, &g_prm, text, 3u, &rev),
          "output overflow reported, not written past");
}

static void test_line(void)
{
    line_params_t  p;
    line_tracker_t t;
    line_cal_t     cal;
    uint16_t       raw[S3_IR_COUNT];
    uint16_t       norm[S3_IR_COUNT];
    uint16_t       white[S3_IR_COUNT] = { 0u, 0u, 0u };
    uint16_t       centre[S3_IR_COUNT] = { 0u, 1000u, 0u };
    uint16_t       left[S3_IR_COUNT] = { 1000u, 0u, 0u };
    uint16_t       all[S3_IR_COUNT] = { 1000u, 1000u, 1000u };
    line_evt_t     e;
    int            i;
    int            lost_at = -1;

    line_params_default(&p);
    p.line_mask = S3_MASK_L | S3_MASK_C | S3_MASK_R;

    /* H-L01: SEARCH -> FOLLOW after FOUND_SAMPLES */
    line_tracker_reset(&t);
    e = line_tracker_step(&t, &p, centre, false);
    e = line_tracker_step(&t, &p, centre, false);
    CHECK("H-L01", (LINE_EVT_FOUND == e) && (LINE_STATE_FOLLOW == t.state) &&
          (0 == t.position) && (0 == t.steering), "centred line -> FOLLOW, no steering");

    /* H-L02: line on the left -> negative position and steering */
    (void)line_tracker_step(&t, &p, left, false);
    CHECK("H-L02", (-1000 == t.position) && (t.steering < 0) && (-1 == t.last_side),
          "line left -> position -1000, steer left");

    /* H-L03: LOST after LOST_SAMPLES of white, remembers side */
    for (i = 0; i < 20; i++)
    {
        if (LINE_EVT_LOST == line_tracker_step(&t, &p, white, false)) { lost_at = i + 1; break; }
    }
    CHECK("H-L03", (S3_LOST_SAMPLES == lost_at) && (LINE_STATE_LOST == t.state) &&
          (-1 == t.last_side) && (-1000 == t.position),
          "lost after 5 white samples, last side kept");

    /* H-L04: junction after JUNCTION_SAMPLES, inhibited while scanning */
    line_tracker_reset(&t);
    (void)line_tracker_step(&t, &p, centre, false);
    (void)line_tracker_step(&t, &p, centre, false);
    for (i = 0; i < 10; i++) { e = line_tracker_step(&t, &p, all, true); }
    CHECK("H-L04a", LINE_STATE_FOLLOW == t.state, "no junction while barcode scanning");
    e = LINE_EVT_NONE;
    for (i = 0; (i < S3_JUNCTION_SAMPLES) && (LINE_EVT_JUNCTION != e); i++)
    {
        e = line_tracker_step(&t, &p, all, false);
    }
    CHECK("H-L04b", (LINE_EVT_JUNCTION == e) && (LINE_STATE_JUNCTION == t.state),
          "junction after 3 all-black samples");

    /* H-L05: calibration + normalisation (analogue C, R) */
    line_cal_reset(&cal);
    raw[0] = 0u; raw[1] = 300u;  raw[2] = 250u;  line_cal_update(&cal, raw);
    raw[0] = 1u; raw[1] = 3300u; raw[2] = 3250u; line_cal_update(&cal, raw);
    CHECK("H-L05a", line_cal_finish(&cal, S3_MASK_C | S3_MASK_R, S3_CAL_MIN_SPAN),
          "calibration with 3000-count span accepted");
    raw[0] = 1u; raw[1] = 1800u; raw[2] = 250u;
    line_normalise(&cal, S3_MASK_C | S3_MASK_R, true, 1u, raw, norm);
    CHECK("H-L05b", (1000u == norm[0]) && (500u == norm[1]) && (0u == norm[2]),
          "digital L=black, C mid-scale=500, R floor=0");

    line_cal_reset(&cal);
    raw[1] = 1000u; raw[2] = 1000u; line_cal_update(&cal, raw);
    raw[1] = 1100u; raw[2] = 1100u; line_cal_update(&cal, raw);
    CHECK("H-L05c", !line_cal_finish(&cal, S3_MASK_C | S3_MASK_R, S3_CAL_MIN_SPAN),
          "calibration with 100-count span rejected");
}

int main(void)
{
    test_code39();
    test_line();
    printf("[TEST] subsystem3-host SUMMARY %d passed, %d failed\n", g_pass, g_fail);
    return (0 == g_fail) ? 0 : 1;
}
