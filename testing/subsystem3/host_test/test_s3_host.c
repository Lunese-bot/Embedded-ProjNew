/*
 *	test_s3_host.c
 *	PC unit tests for subsystem 3's pure algorithms (code39.c,
 *	line_algo.c).  No robot needed:
 *
 *	    cd testing/subsystem3/host_test && make
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
    p.weight[S3_IR_C] = S3_WEIGHT_C; /* classic weighted-average mode */
    p.straddle        = false;
    p.lost_samples    = 5u;

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
    CHECK("H-L03", (5 == lost_at) && (LINE_STATE_LOST == t.state) &&
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

    /* H-L06..09: straddle layout, line BETWEEN C and R */
    {
        uint16_t c_only[S3_IR_COUNT] = { 0u, 1000u, 0u };
        uint16_t r_only[S3_IR_COUNT] = { 0u, 0u, 1000u };
        uint16_t both[S3_IR_COUNT]   = { 0u, 1000u, 1000u };

        line_params_default(&p);
        CHECK("H-L06", p.straddle && (-1000 == p.weight[S3_IR_C]) &&
              (1000 == p.weight[S3_IR_R]), "straddle is the default, C/R weights symmetric");
        line_tracker_reset(&t);
        (void)line_tracker_step(&t, &p, white, false);
        e = line_tracker_step(&t, &p, white, false);
        CHECK("H-L07", (LINE_STATE_FOLLOW == t.state) && (0 == t.position) &&
              (0 == t.steering), "both white -> FOLLOW and straight");
        (void)line_tracker_step(&t, &p, c_only, false);
        CHECK("H-L08a", (t.position < 0) && (t.steering < 0), "C black -> steer left");
        (void)line_tracker_step(&t, &p, r_only, false);
        CHECK("H-L08b", (t.position > 0) && (t.steering > 0), "R black -> steer right");
        line_tracker_reset(&t);
        (void)line_tracker_step(&t, &p, white, false);
        (void)line_tracker_step(&t, &p, white, false);
        for (i = 0; i < 100; i++) { e = line_tracker_step(&t, &p, white, false); }
        CHECK("H-L09a", (LINE_STATE_FOLLOW == t.state), "1 s of white is not lost");
        lost_at = -1;
        for (i = 0; i < 100; i++)
        {
            if (LINE_EVT_LOST == line_tracker_step(&t, &p, white, false)) { lost_at = i; break; }
        }
        for (i = 0; i < 1000; i++) { (void)line_tracker_step(&t, &p, white, false); }
        CHECK("H-L09b", (LINE_STATE_FOLLOW == t.state) && (0 == t.steering),
              "straddle: 10 s of white stays FOLLOW and straight (no LOST)");
        for (i = 0; i < 3; i++) { e = line_tracker_step(&t, &p, c_only, false); }
        CHECK("H-L09d", (LINE_STATE_FOLLOW == t.state) && (t.steering < 0),
              "C black still steers left");
        for (i = 0; i < 3; i++) { e = line_tracker_step(&t, &p, both, false); }
        CHECK("H-L09e", (LINE_EVT_JUNCTION == e) && (LINE_STATE_JUNCTION == t.state),
              "C and R both black -> JUNCTION");
        (void)e;

        /* H-L10..12: no over-correction, gentle start that grows */
        {
            int16_t first;
            int16_t later;

            line_params_default(&p);
            p.kp_num = 4;   /* first nudge 100 */
            p.kd_num = 10;  /* +10 per sample */
            line_tracker_reset(&t);
            (void)line_tracker_step(&t, &p, white, false);
            (void)line_tracker_step(&t, &p, white, false);
            (void)line_tracker_step(&t, &p, r_only, false);
            first = t.steering;
            for (i = 0; i < 20; i++) { (void)line_tracker_step(&t, &p, r_only, false); }
            later = t.steering;
            CHECK("H-L10", (100 == first) && (300 == later),
                  "R black: starts at 100, grows to 300 after 20 more samples");
            (void)line_tracker_step(&t, &p, both, false);
            CHECK("H-L11a", t.steering >= later, "both black holds the correction");
            (void)line_tracker_step(&t, &p, white, false);
            CHECK("H-L11b", 0 == t.steering, "medium correction (310) then white: straight at once");
            (void)line_tracker_step(&t, &p, c_only, false);
            CHECK("H-L12", -100 == t.steering, "other side starts gently again");
            /* H-L13..16: curve memory for tight curves / U-turns */
            p.kp_num = 4; p.kd_num = 10;
            line_tracker_reset(&t);
            (void)line_tracker_step(&t, &p, white, false);
            (void)line_tracker_step(&t, &p, white, false);
            for (i = 0; i < 5; i++) { (void)line_tracker_step(&t, &p, c_only, false); }
            (void)line_tracker_step(&t, &p, white, false);
            CHECK("H-L13", 0 == t.steering, "short touch then white: straight at once");
            for (i = 0; i < 70; i++) { (void)line_tracker_step(&t, &p, c_only, false); }
            later = t.steering;
            (void)line_tracker_step(&t, &p, white, false);
            CHECK("H-L14a", (later < 0) && (t.steering == later),
                  "long left correction then white: keeps turning left (line outside)");
            for (i = 0; i < 50; i++) { (void)line_tracker_step(&t, &p, white, false); }
            CHECK("H-L14b", t.steering == later, "still turning after 0.5 s of white");
            (void)line_tracker_step(&t, &p, c_only, false);
            (void)line_tracker_step(&t, &p, white, false);
            CHECK("H-L15", 0 == t.steering, "line came back in, then white: straight (no new hold)");
            for (i = 0; i < 70; i++) { (void)line_tracker_step(&t, &p, c_only, false); }
            for (i = 0; i < S3_STRADDLE_HOLD_MAX + 2; i++) { (void)line_tracker_step(&t, &p, white, false); }
            CHECK("H-L16a", 0 == t.steering, "curve memory gives up after HOLD_MAX");
            for (i = 0; i < 5; i++) { (void)line_tracker_step(&t, &p, white, false); }
            CHECK("H-L16b", 0 == t.steering, "and does not start again on white");
            for (i = 0; i < 70; i++) { (void)line_tracker_step(&t, &p, c_only, false); }
            (void)line_tracker_step(&t, &p, white, false);
            (void)line_tracker_step(&t, &p, r_only, false);
            CHECK("H-L16c", 100 == t.steering, "other sensor during hold: gentle right (100)");
            p.kp_num = 0; p.kd_num = 0;
            (void)line_tracker_step(&t, &p, c_only, false);
            CHECK("H-L12b", 0 == t.steering, "Kp 0 and Kd 0: no steering");
#if (S3_STRADDLE_DECAY > 0)
            /* H-L17: turn memory over short white gaps (S3_STRADDLE_DECAY) */
            p.kp_num = 4; p.kd_num = 10;
            line_tracker_reset(&t);
            (void)line_tracker_step(&t, &p, white, false);
            (void)line_tracker_step(&t, &p, white, false);
            for (i = 0; i < 10; i++) { (void)line_tracker_step(&t, &p, r_only, false); }
            CHECK("H-L17a", 190 == t.steering, "R black 10 samples: 190");
            for (i = 0; i < 3; i++) { (void)line_tracker_step(&t, &p, white, false); }
            CHECK("H-L17b", 0 == t.steering, "short white gap: straight (no over-correction)");
            (void)line_tracker_step(&t, &p, r_only, false);
            CHECK("H-L17c", (100 + 10 * (9 - 3 * S3_STRADDLE_DECAY + 1)) == t.steering,
                  "R black again soon: carries on, not back to the first nudge");
            for (i = 0; i < 200; i++) { (void)line_tracker_step(&t, &p, white, false); }
            (void)line_tracker_step(&t, &p, r_only, false);
            CHECK("H-L17d", 100 == t.steering, "after a long white gap: first nudge again");
            for (i = 0; i < 10; i++) { (void)line_tracker_step(&t, &p, r_only, false); }
            (void)line_tracker_step(&t, &p, white, false);
            (void)line_tracker_step(&t, &p, c_only, false);
            CHECK("H-L17e", -100 == t.steering, "other side always starts gently");
            /* flicker on a tight curve: 5 black / 2 white repeated builds up to a hold */
            line_tracker_reset(&t);
            (void)line_tracker_step(&t, &p, white, false);
            (void)line_tracker_step(&t, &p, white, false);
            for (int j = 0; j < 30; j++)
            {
                for (i = 0; i < 5; i++) { (void)line_tracker_step(&t, &p, r_only, false); }
                for (i = 0; i < 2; i++) { (void)line_tracker_step(&t, &p, white, false); }
            }
            CHECK("H-L17f", t.steering >= S3_STRADDLE_HOLD_MAG,
                  "R flickering on a curve: grows and the curve memory takes over");
#endif
        }
    }

    /* H-L19: R-only analogue, not calibrated: scaled between the measured
       S3_R_ADC_WHITE and S3_R_ADC_BLACK, C and L stay digital */
    {
        uint16_t rw[S3_IR_COUNT];
        uint16_t nm[S3_IR_COUNT];
        line_cal_t nocal;

        line_cal_reset(&nocal); /* not valid */
        rw[S3_IR_L] = 0u; rw[S3_IR_C] = (uint16_t)S3_IR_BLACK_LEVEL; rw[S3_IR_R] = (uint16_t)S3_R_ADC_WHITE;
        line_normalise(&nocal, (uint8_t)S3_MASK_R, true, (uint8_t)S3_IR_BLACK_LEVEL, rw, nm);
        CHECK("H-L19a", (0u == nm[S3_IR_R]) && (1000u == nm[S3_IR_C]) && (0u == nm[S3_IR_L]),
              "R over white reads 0, C digital black reads 1000");
        rw[S3_IR_R] = (uint16_t)S3_R_ADC_BLACK;
        line_normalise(&nocal, (uint8_t)S3_MASK_R, false, (uint8_t)S3_IR_BLACK_LEVEL, rw, nm);
        CHECK("H-L19b", 1000u == nm[S3_IR_R], "R over black reads 1000 (direction from the config values)");
        rw[S3_IR_R] = (uint16_t)((S3_R_ADC_WHITE + S3_R_ADC_BLACK) / 2);
        line_normalise(&nocal, (uint8_t)S3_MASK_R, true, (uint8_t)S3_IR_BLACK_LEVEL, rw, nm);
        CHECK("H-L19c", (nm[S3_IR_R] >= 495u) && (nm[S3_IR_R] <= 505u), "halfway reads about 500 = the black threshold");
        rw[S3_IR_C] = (uint16_t)S3_C_ADC_WHITE;
        line_normalise(&nocal, (uint8_t)(S3_MASK_C | S3_MASK_R), true, (uint8_t)S3_IR_BLACK_LEVEL, rw, nm);
        CHECK("H-L19d", 0u == nm[S3_IR_C], "C analogue over white reads 0");
        rw[S3_IR_C] = (uint16_t)S3_C_ADC_BLACK;
        line_normalise(&nocal, (uint8_t)(S3_MASK_C | S3_MASK_R), true, (uint8_t)S3_IR_BLACK_LEVEL, rw, nm);
        CHECK("H-L19e", 1000u == nm[S3_IR_C], "C analogue over black reads 1000");
        rw[S3_IR_L] = (uint16_t)S3_L_ADC_WHITE;
        line_normalise(&nocal, (uint8_t)(S3_MASK_L | S3_MASK_C | S3_MASK_R), true, (uint8_t)S3_IR_BLACK_LEVEL, rw, nm);
        CHECK("H-L19f", 0u == nm[S3_IR_L], "L analogue over white reads 0");
        rw[S3_IR_L] = (uint16_t)S3_L_ADC_BLACK;
        line_normalise(&nocal, (uint8_t)(S3_MASK_L | S3_MASK_C | S3_MASK_R), true, (uint8_t)S3_IR_BLACK_LEVEL, rw, nm);
        CHECK("H-L19g", 1000u == nm[S3_IR_L], "L analogue over black reads 1000");
    }

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

/* ---- barcode edge finder (bc_edge_step) ---------------------------- */
/* MY's real L trace over 'A', 2026-11-05 05:59, raw every 10 ms */
static uint16_t const g_trace_a[] = {
238, 234, 242, 248, 314, 876, 1404, 1753, 2042, 2230, 2421, 2459, 2522,
2569, 2606, 2630, 2587, 2569, 2596, 2601, 2590, 2514, 2379, 2049, 1512,
1177, 1325, 1578, 1782, 1856, 1937, 2051, 2212, 2272, 2286, 2281, 2234,
2209, 2246, 2240, 2195, 2239, 2277, 2251, 2274, 2244, 2162, 2037, 1892,
1812, 1678, 1621, 1504, 1376, 1185, 1105, 1096, 1153, 1262, 1238, 1241,
1294, 1381, 1416, 1535, 1707, 1738, 1583, 1348, 682, 241, 223, 227, 296,
702, 1642, 2450, 2865, 3123, 3288, 3291, 3305, 3347, 3313, 3316, 3315, 3378,
3362, 3348, 3403, 3409, 3366, 3371, 3357, 3412, 3418, 3372, 3376, 3413,
3404, 3350, 3364, 3319, 3317, 3283, 3277, 3286, 3291, 3245, 3250, 3230,
3189, 3197, 3165, 3155, 3186, 3154, 3163, 3164, 3157, 3121, 3108, 3072,
3027, 3012, 2873, 2786, 2675, 2606, 2444, 2292, 2052, 1845, 1739, 1632,
1656, 1757, 1904, 2144, 2398, 2594, 2712, 2806, 2874, 2932, 3003, 3014,
3026, 2976, 2955, 2926, 2896, 2905, 2882, 2864, 2880, 2923, 2926, 2947,
2969, 2936, 2981, 2985, 2972, 2962, 2945, 2971, 2933, 2927, 2973, 2947,
2988, 2944, 2978, 2921, 2946, 2909, 2889, 2873, 2886, 2828, 2848, 2772,
2668, 2557, 2438, 2250, 2145, 1968, 1715, 1535, 1324, 1224, 1119, 1125,
1183, 1183, 1236, 1301, 1396, 1427, 1526, 1566, 1648, 1681, 1753, 1760,
1811, 1836, 1836, 1784, 1754, 1692, 1667, 1685, 1711, 1752, 1791, 1832,
1876, 1821, 1826, 1745, 1686, 1535, 1342, 1097, 736, 364, 322, 278, 275,
270, 264, 259, 243, 235, 234, 184, 195, 244, 197, 196, 201, 179, 217, 179,
169, 181, 205, 169, 152, 187, 207, 196, 200, 172, 157, 152, 190, 187, 149,
201, 181, 146, 144, 147, 154, 193, 181, 148, 170, 189, 179, 182, 179, 147,
164, 148, 188, 171, 153, 153, 203, 153, 160, 194, 152, 172, 177, 153, 145,
178, 197, 188, 155, 165, 202, 201, 198, 232, 153, 155, 156, 149, 184, 188,
145, 201, 159, 157, 184, 149, 171, 152, 153, 148, 166, 161, 144, 191, 186,
184, 147, 198, 147, 192, 200, 174, 203, 185, 166, 168, 146, 156, 187, 190,
196, 149, 146, 178, 154, 153, 198, 170, 185, 155, 146, 192, 148, 185, 198,
149, 154, 194, 155, 173, 173, 198, 163, 189, 183, 149, 157, 141, 187, 170,
152, 153, 191, 143, 158, 172, 176, 208, 261, 1904, 2917, 2954, 2998, 2980,
2978, 2968, 2948, 2965, 2956, 3006, 2984, 3020, 3100, 3112, 3185, 3218,
3280, 3261, 3290, 3318, 3342, 3353, 3299
};

/* run the edge finder over a sampled signal, collect run widths (us) from
   the first black run to the last black run */
static uint32_t edge_runs(int32_t const * p_v, uint32_t n, int32_t swing,
                          int32_t floor, uint32_t * p_w, uint32_t w_max)
{
    bc_edge_t e;
    uint32_t  start = 0u, edge = 0u, k = 0u, i;
    bc_edge_reset(&e, p_v[0], 0u);
    for (i = 0u; i < n; i++)
    {
        uint32_t t = i * 10000u;
        uint8_t  was = e.level;
        if (0u != bc_edge_step(&e, p_v[i], t, swing, floor, &edge))
        {
            if ((0u != was) || (k > 0u)) /* skip the leading white */
            {
                if (k < w_max) { p_w[k++] = edge - start; }
            }
            start = edge;
        }
    }
    if ((k > 0u) && (0u == (k % 2u))) { k--; } /* drop trailing white */
    return k;
}

static void test_bc_edge(void)
{
    static int32_t v[1200];
    uint32_t w[200], runs[100], ideal[100];
    char     text[16];
    bool     rev;
    uint32_t n, i, k, j, t, nr;
    int32_t  ok;

    /* H-E01..03: ideal *A* (narrow 47 ms, wide 141 ms) sampled every 10 ms,
       paper white 200 / black 3000, then blurred (average of
       7 samples, a spot ~8 mm wide at 120 mm/s) so thin gaps only drop to
       about 1000-1300, like MY's trace.  Plus a slow drift of +600. */
    nr = code39_encode("A", 47u, 141u, ideal, 100u);
    n = 0u;
    for (i = 0u; i < 30u; i++) { v[n++] = 200; }        /* quiet zone */
    t = 0u;
    j = 0u; /* exact edge time (us); t = sample time */
    for (k = 0u; k < nr; k++)
    {
        uint32_t end = (j += ideal[k] * 1000u);
        int32_t  lev = (0u == (k % 2u)) ? 3000 : 200;   /* bar first */
        for (; t < end; t += 10000u) { v[n++] = lev; }  /* t in us */
    }
    for (i = 0u; i < 60u; i++) { v[n++] = 200; }
    {
        static int32_t b[1200];
        for (i = 0u; i < n; i++)
        {
            int32_t s = 0;
            for (j = 0u; j < 7u; j++) { s += v[(i >= j) ? (i - j) : 0u]; }
            b[i] = s / 7;
        }
        for (i = 0u; i < n; i++) { v[i] = b[i]; }
    }
    /* how white do the thin gaps get?  for each space, its lowest point
       (the blur shifts it up to 7 samples late); keep the worst one */
    ok = 0;
    t = 0u;
    for (k = 0u; k < nr; k++)
    {
        uint32_t a = 30u + t / 10000u, z;
        int32_t  lo = 5000;
        t += ideal[k] * 1000u;
        z = 30u + t / 10000u + 7u;
        if (1u == (k % 2u))
        {
            for (i = a; i < z; i++) { if (v[i] < lo) { lo = v[i]; } }
            if (lo > ok) { ok = lo; }
        }
    }
    printf("        blurred *A*: the thinnest gaps only get down to %d (white = 200)\n", (int)ok);
    CHECK("H-E00", ok > 900, "test signal is as blurry as MY's (gaps stay above 900)");
    k = edge_runs(v, n, S3_BC_EDGE_SWING, S3_BC_EDGE_FLOOR, runs, 100u);
    CHECK("H-E01", k == nr, "edge finder: blurred *A* gives 29 bars+spaces");
    CHECK("H-E02", (CODE39_OK == code39_decode(runs, k, &g_prm, text, sizeof text, &rev))
                   && (0 == strcmp(text, "A")),
          "edge finder: blurred *A* decodes as A");
    for (i = 0u; i < n; i++) { v[i] += (int32_t)((i * 600u) / n); }
    k = edge_runs(v, n, S3_BC_EDGE_SWING, S3_BC_EDGE_FLOOR, runs, 100u);
    printf("        drift: %u runs:", (unsigned)k);
    for (i = 0u; i < k; i++) { printf(" %u", (unsigned)(runs[i] / 1000u)); }
    printf(" ms\n");
    CHECK("H-E03", (k == nr) &&
                   (CODE39_OK == code39_decode(runs, k, &g_prm, text, sizeof text, &rev))
                   && (0 == strcmp(text, "A")),
          "edge finder: still decodes A with the baseline drifting +600");
    (void)ok; (void)w;

    /* H-E06/07: like MY's 06:57 pass: weak bumps (white 200, bars only
       reach ~950) on a slow pass (narrow 120 ms, wide 360 ms), blurred
       over 9 samples.  Must give 29 runs, decode A, and no white gap
       inside the barcode may be long enough to end the read. */
    {
        static int32_t b[1200];
        uint32_t te = 0u, widest = 0u;
        nr = code39_encode("A", 120u, 360u, ideal, 100u);
        n = 0u; t = 0u;
        for (i = 0u; i < 40u; i++) { v[n++] = 200; }
        for (k = 0u; k < nr; k++)
        {
            uint32_t end = (te += ideal[k] * 1000u);
            int32_t  lev = (0u == (k % 2u)) ? 950 : 200;
            for (; t < end; t += 10000u) { v[n++] = lev; }
        }
        for (i = 0u; i < 120u; i++) { v[n++] = 200; }
        for (i = 0u; i < n; i++)
        {
            int32_t s9 = 0;
            for (j = 0u; j < 9u; j++) { s9 += v[(i >= j) ? (i - j) : 0u]; }
            b[i] = s9 / 9;
        }
        for (i = 0u; i < n; i++) { v[i] = b[i]; }
        k = edge_runs(v, n, S3_BC_EDGE_SWING, S3_BC_EDGE_FLOOR, runs, 100u);
        for (i = 1u; i < k; i += 2u) { if (runs[i] > widest) { widest = runs[i]; } }
        printf("        weak slow *A*: %u runs, longest white inside %u ms:", (unsigned)k, (unsigned)(widest / 1000u));
        for (i = 0u; i < k; i++) { printf(" %u", (unsigned)(runs[i] / 1000u)); }
        printf(" ms\n");
        CHECK("H-E06", (k == nr) &&
                       (CODE39_OK == code39_decode(runs, k, &g_prm, text, sizeof text, &rev))
                       && (0 == strcmp(text, "A")),
              "weak, slow, blurred *A* (bars only reach 950) decodes as A");
        CHECK("H-E07", widest < ((uint32_t)S3_BC_QUIET_MIN_MS * 1000u),
              "no white gap inside it is long enough to end the read early");
    }

    /* H-E04/05: MY's real trace */
    n = (uint32_t)(sizeof g_trace_a / sizeof g_trace_a[0]);
    for (i = 0u; i < n; i++) { v[i] = (int32_t)g_trace_a[i]; }
    k = edge_runs(v, n, S3_BC_EDGE_SWING, S3_BC_EDGE_FLOOR, runs, 100u);
    printf("        real trace: %u bars+spaces found (fixed cut-off found 1):", (unsigned)k);
    for (i = 0u; i < k; i++) { printf(" %u", (unsigned)(runs[i] / 1000u)); }
    printf(" ms\n");
    CHECK("H-E04", k >= 11u, "real 'A' trace: edge finder sees at least 6 bars (cut-off saw 1)");
    {
        /* white paper part only (samples 240..380): no bar from the noise */
        bc_edge_t e; uint32_t edge = 0u; int32_t flips = 0;
        bc_edge_reset(&e, (int32_t)g_trace_a[240], 0u);
        for (i = 240u; i < 370u; i++)
        {
            flips += bc_edge_step(&e, (int32_t)g_trace_a[i], i * 10000u,
                                  S3_BC_EDGE_SWING, S3_BC_EDGE_FLOOR, &edge);
        }
        CHECK("H-E05", 0 == flips, "white paper noise (140-260) never counts as a bar");
    }
}

int main(void)
{
    test_code39();
    test_line();
    test_bc_edge();
    printf("[TEST] subsystem3-host SUMMARY %d passed, %d failed\n", g_pass, g_fail);
    return (0 == g_fail) ? 0 : 1;
}
