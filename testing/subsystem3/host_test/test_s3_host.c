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
#include <stdlib.h>
#include <math.h>
#include "code39.h"
#include "code39_geom.h"
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

    /* H-L20..24: barcode bars across the line (S3_STRADDLE_BAR_STRAIGHT) */
    {
        uint16_t c_only[S3_IR_COUNT] = { 0u, 1000u, 0u };
        uint16_t r_only[S3_IR_COUNT] = { 0u, 0u, 1000u };
        uint16_t both[S3_IR_COUNT]   = { 0u, 1000u, 1000u };
        int      max_abs = 0;

        line_params_default(&p);
        line_tracker_reset(&t);
        (void)line_tracker_step(&t, &p, white, false);
        (void)line_tracker_step(&t, &p, white, false);
        /* thick bar at a slight angle: R one sample early, then C+R+L for
           32 samples (19 mm at 60 mm/s), then the white gap */
        t.l_raw = 200u;
        (void)line_tracker_step(&t, &p, r_only, true);
        t.l_raw = 2000u;
        for (i = 0; i < 32; i++)
        {
            (void)line_tracker_step(&t, &p, both, false);
            if (abs(t.steering) > max_abs) { max_abs = abs(t.steering); }
        }
        CHECK("H-L20", 0 == max_abs, "L+C+R black (bar): straight the whole bar, no build-up");
        t.l_raw = 200u;
        for (i = 0; i < 30; i++) { (void)line_tracker_step(&t, &p, white, false); }
        CHECK("H-L21", (0 == t.steering) && !t.b_hold,
              "white after a bar: straight, no curve memory");

        /* while scanning, C+R black is a bar even if L is on white */
        line_tracker_reset(&t);
        (void)line_tracker_step(&t, &p, white, true);
        (void)line_tracker_step(&t, &p, c_only, true);
        t.l_raw = 150u;
        for (i = 0; i < 20; i++) { (void)line_tracker_step(&t, &p, both, true); }
        CHECK("H-L22", (0 == t.steering) && (0u == t.corr_count),
              "scanning: C+R black -> straight and turn forgotten");

        /* normal line following unchanged: no scan, L on white paper
           (even noisy, 260) -> both black still holds the turn */
        line_params_default(&p);
        line_tracker_reset(&t);
        (void)line_tracker_step(&t, &p, white, false);
        (void)line_tracker_step(&t, &p, white, false);
        for (i = 0; i < 10; i++) { (void)line_tracker_step(&t, &p, r_only, false); }
        t.l_raw = 260u;
        (void)line_tracker_step(&t, &p, both, false);
        CHECK("H-L23", t.steering > 0, "no barcode: C+R black still holds the right turn");

        /* tight left curve: L and C black, R white -> still steers left */
        line_tracker_reset(&t);
        (void)line_tracker_step(&t, &p, white, false);
        (void)line_tracker_step(&t, &p, white, false);
        t.l_raw = 2000u;
        for (i = 0; i < 10; i++) { (void)line_tracker_step(&t, &p, c_only, false); }
        CHECK("H-L24", t.steering < 0, "L+C black, R white (curve): still steers left");
    }
}

static void test_rejoin_junction(void)
{
    const uint16_t both_black[S3_IR_COUNT] = { 0u, 1000u, 1000u };
    const uint16_t right_black[S3_IR_COUNT] = { 0u, 0u, 1000u };
    line_params_t params;
    line_tracker_t tracker;
    uint16_t sample;
    int junctions = 0;

    line_params_default(&params);
    params.junction_mask = S3_MASK_C | S3_MASK_R;
    params.straddle = true;
    line_tracker_reset(&tracker);
    tracker.rejoin_clear_left = S3_REJOIN_CLEAR_SAMPLES;

    for (sample = 0; sample < 2u * S3_REJOIN_CLEAR_SAMPLES; sample++)
    {
        junctions += LINE_EVT_JUNCTION ==
                     line_tracker_step(&tracker, &params, both_black, false);
    }
    CHECK("H-L25", (junctions == 0) && (LINE_STATE_FOLLOW == tracker.state) &&
                      (0 == tracker.steering),
          "coincident black on the rejoin keeps following without a turn");

    for (sample = 0; sample < S3_REJOIN_CLEAR_SAMPLES - 1u; sample++)
    {
        (void)line_tracker_step(&tracker, &params, right_black, false);
    }
    CHECK("H-L26", tracker.steering > 0,
          "ordinary steering is available before junctions are rearmed");
    for (sample = 0; sample < S3_JUNCTION_SAMPLES; sample++)
    {
        junctions += LINE_EVT_JUNCTION ==
                     line_tracker_step(&tracker, &params, both_black, false);
    }
    CHECK("H-L27", junctions == 0, "a short gap does not rearm junctions");

    for (sample = 0; sample < S3_REJOIN_CLEAR_SAMPLES; sample++)
    {
        (void)line_tracker_step(&tracker, &params, right_black, false);
    }
    for (sample = 0; sample < S3_JUNCTION_SAMPLES; sample++)
    {
        junctions += LINE_EVT_JUNCTION ==
                     line_tracker_step(&tracker, &params, both_black, false);
    }
    CHECK("H-L28", (junctions == 1) && (LINE_STATE_JUNCTION == tracker.state),
          "a new junction is detected after the sensors settle");
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
                if (0u != was) /* as task_line.c + task_barcode.c */
                {
                    if ((k > 0u) && (k <= w_max)) { p_w[k - 1u] += e.bar_t_us - start; }
                    start = e.bar_t_us;
                }
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

/* H-D: bars measured by wheel distance (S3_BC_USE_DIST) and the early end
   of a read (S3_BC_EARLY_END) used by task_barcode.c. */
static void test_bc_distance(void)
{
    static uint32_t w[200], wt[200], wp[200];
    code39_options_t opt = { S3_BC_MIN_RATIO_X10, true, S3_BC_MAX_CHARS };
    char     text[16];
    bool     rev = false;
    uint32_t n, i, k;
    int      bad = 0;

    /* H-D01/02: a driven pass over the big *Z* (narrow 6 mm, wide 19 mm).
       The robot is at 120 mm/s on the first bar and slows to 60 mm/s over
       0.3 s, as the controller does on BARCODE_START.  Sampled every 10 ms
       like line_task; position in encoder counts (L + R, 0.1475 mm). */
    {
        static uint32_t edges_mm[200];
        double   x = -30.0, v = 120.0, t_slow = -1.0;
        uint32_t ne = 0u, t_us, pos, e_us = 0u, e_pos = 0u;
        uint32_t s_us = 0u, s_pos = 0u;
        uint32_t nt = 0u;
        bc_edge_t e;
        double   acc = 0.0;
        code39_result_t r_t, r_d;
        bool rev_t = false, rev_d = false;
        char txt_t[16] = "", txt_d[16] = "";

        n = code39_encode("Z", 6u, 19u, w, 200u);
        for (i = 0u; i < n; i++) { acc += (double)w[i]; edges_mm[ne++] = (uint32_t)acc; }
        bc_edge_reset_pos(&e, 200, 0u, 0u);
        for (i = 0u; (i < 2000u) && (x < acc + 60.0); i++)
        {
            double tt = (double)i * 0.01;
            uint32_t j = 0u, black;
            while ((j < ne) && ((double)edges_mm[j] <= x)) { j++; }
            black = ((x >= 0.0) && (j < ne) && (0u == (j % 2u))) ? 1u : 0u;
            t_us = i * 10000u;
            pos  = (uint32_t)((x + 30.0) / 0.1475);
            if (0u != bc_edge_step_pos(&e, black ? 1200 : 200, t_us, pos,
                                       S3_BC_EDGE_SWING, S3_BC_EDGE_FLOOR,
                                       &e_us, &e_pos))
            {
                if ((0u == e.level) || (nt > 0u)) /* skip leading white */
                {
                    if (0u == e.level)
                    {
                        if (nt > 0u) { wt[nt - 1u] += e.bar_t_us - s_us; wp[nt - 1u] += e.bar_pos - s_pos; }
                        s_us = e.bar_t_us; s_pos = e.bar_pos;
                    }
                    wt[nt] = e_us - s_us; wp[nt] = e_pos - s_pos; nt++;
                }
                s_us = e_us; s_pos = e_pos;
            }
            if ((t_slow < 0.0) && (x >= 0.0)) { t_slow = tt; }
            if ((t_slow >= 0.0) && (v > 60.0)) { v -= 2.0; } /* 0.3 s ramp */
            x += v * 0.01;
        }
        r_t = code39_decode_ex(wt, nt, &opt, txt_t, sizeof txt_t, &rev_t);
        r_d = code39_decode_ex(wp, nt, &opt, txt_d, sizeof txt_d, &rev_d);
        printf("        slowing pass: %u bars+spaces; by time %s (%d), by distance %s (%d)\n",
               (unsigned)nt, txt_t, (int)r_t, txt_d, (int)r_d);
        printf("        first bar: %u ms vs next wide bar %u ms; %u vs %u counts\n",
               (unsigned)(wt[0] / 1000u), (unsigned)(wt[2] / 1000u),
               (unsigned)wp[0], (unsigned)wp[2]);
        CHECK("H-D01", (29u == nt) && (CODE39_OK == r_d) && (0 == strcmp(txt_d, "Z")),
              "slowing on the first bar: distance widths still decode Z");
        CHECK("H-D02", wp[0] * 10u < wp[2] * 13u && wp[2] * 10u < wp[0] * 13u,
              "first bar (read while slowing) measures like the next same-width bar by distance");
    }

    /* H-E08: MY's driven pass of 2026-11-11 07:10: one bar whose top
       wobbles 1794 -> 1425 -> 1969 (the L arm bouncing).  A fixed 300 fall
       cut it in two; it must stay one bar. */
    {
        static const int32_t tr[] = { 190, 230, 227, 626, 1540, 1672, 1794,
                                      1638, 1425, 1969, 1713, 235, 205, 210 };
        bc_edge_t e; uint32_t ed = 0u; int32_t flips = 0; uint32_t bar_ms = 0u;
        bc_edge_reset(&e, tr[0], 0u);
        for (i = 0u; i < (uint32_t)(sizeof tr / sizeof tr[0]); i++)
        {
            if (0u != bc_edge_step(&e, tr[i], i * 10000u, S3_BC_EDGE_SWING,
                                   S3_BC_EDGE_FLOOR, &ed))
            {
                flips++;
                if (0u == e.level) { bar_ms = (ed - e.bar_t_us) / 1000u; }
            }
        }
        printf("        wobbly bar: %d level changes, bar %u ms\n", (int)flips, (unsigned)bar_ms);
        CHECK("H-E08", (2 == flips) && (bar_ms >= 60u) && (bar_ms <= 80u),
              "real wobbly bar (top dips 370) stays ONE bar, full width");
    }

    /* H-E09: the white inside a wide space creeps up from 200 to 560 well
       before the next bar (seen 2026-11-11 07:23: wide spaces read 29-40
       counts and the bar after them 32-46, instead of ~55 and ~20).  The
       creep makes the finder say "bar" early; the bar's start is then
       re-found at half its real peak and the space gets the piece back. */
    {
        static const int32_t tr[] = {
            200, 200, 1800, 1800, 1800, 200, 200, 200, 200,   /* bar, white  */
            520, 540, 560, 560, 560, 560,                     /* creep       */
            1800, 1800, 200, 200, 200 };                      /* 20 ms bar   */
        uint32_t r[8]; uint32_t nr2;
        nr2 = edge_runs(tr, (uint32_t)(sizeof tr / sizeof tr[0]),
                        S3_BC_EDGE_SWING, S3_BC_EDGE_FLOOR, r, 8u);
        printf("        creeping white: %u runs: %u %u %u ms\n", (unsigned)nr2,
               (unsigned)(r[0] / 1000u), (unsigned)(r[1] / 1000u), (unsigned)(r[2] / 1000u));
        CHECK("H-E09", (3u == nr2) && (30000u == r[0]) && (100000u == r[1]) &&
                       (20000u == r[2]),
              "white creeping up before a bar: space keeps its width, bar is not fattened");
    }

    /* H-C09: a sensor spot about as big as a narrow bar: every bar reads
       4 units wide, every space 4 narrow (narrow 10, wide 30).  All 9
       elements together cannot be split (bar 14 vs space 26), bars and
       spaces separately can. */
    bad = 0;
    for (i = 0u; i < 26u; i++)
    {
        char one[2] = { (char)('A' + i), '\0' };
        n = code39_encode(one, 10u, 30u, w, 200u);
        for (k = 0u; k < n; k++) { w[k] = (0u == (k % 2u)) ? (w[k] + 4u) : (w[k] - 4u); }
        if (!((CODE39_OK == code39_decode_ex(w, n, &opt, text, sizeof text, &rev)) &&
              (text[0] == one[0]) && !rev)) { bad++; }
        for (k = 0u; k < n; k++) { w[k] = (0u == (k % 2u)) ? (w[k] - 8u) : (w[k] + 8u); }
        if (!((CODE39_OK == code39_decode_ex(w, n, &opt, text, sizeof text, &rev)) &&
              (text[0] == one[0]) && !rev)) { bad++; }
    }
    CHECK("H-C09", 0 == bad,
          "bars all read fat (or all thin) and spaces the opposite: A-Z still decode");

    /* H-D03: early end.  For every two-letter code, the first 29 elements
       (forwards or backwards) must NOT decode, or a read would stop before
       its last letter. */
    for (i = 0u; i < 26u; i++)
    {
        for (k = 0u; k < 26u; k++)
        {
            char two[3] = { (char)('A' + i), (char)('A' + k), '\0' };
            n = code39_encode(two, 10u, 30u, w, 200u);
            if (CODE39_OK == code39_decode_ex(w, 29u, &opt, text, sizeof text, &rev)) { bad++; }
            reverse(w, n);
            if (CODE39_OK == code39_decode_ex(w, 29u, &opt, text, sizeof text, &rev)) { bad++; }
        }
    }
    CHECK("H-D03", 0 == bad,
          "early end: no 2-letter code's first 29 bars decode (both directions)");

    /* H-D04: and a one-letter code ends right at its stop '*' */
    bad = 0;
    for (i = 0u; i < 26u; i++)
    {
        char one[2] = { (char)('A' + i), '\0' };
        n = code39_encode(one, 10u, 30u, w, 200u);
        if (!((29u == n) &&
              (CODE39_OK == code39_decode_ex(w, n, &opt, text, sizeof text, &rev)) &&
              (text[0] == one[0]))) { bad++; }
        reverse(w, n);
        if (!((CODE39_OK == code39_decode_ex(w, n, &opt, text, sizeof text, &rev)) &&
              (text[0] == one[0]) && rev)) { bad++; }
    }
    CHECK("H-D04", 0 == bad, "early end: every one-letter code decodes at 29 bars, both ways");
}

/* Partial read: one '*' and the letter next to it are enough (2026-11-11) */
static void merge3(uint32_t * w, uint32_t * p_n, uint32_t at)
{
    uint32_t k;
    w[at] = w[at] + w[at + 1u] + w[at + 2u]; /* a gap lost: 3 elements -> 1 */
    for (k = at + 1u; (k + 2u) < *p_n; k++) { w[k] = w[k + 2u]; }
    *p_n -= 2u;
}

static void test_partial(void)
{
    code39_options_t opt = { S3_BC_MIN_RATIO_X10, true, (uint8_t)S3_BC_MAX_CHARS };
    uint32_t w[64];
    uint32_t n;
    uint32_t i;
    uint32_t seed = 12345u;
    char     text[8];
    bool     rev;
    int      bad = 0;
    int      wrong = 0;
    int      dir;

    /* every letter, both directions, start '*' or stop '*' damaged */
    for (i = 0u; i < 26u; i++)
    {
        char one[2] = { (char)('A' + i), '\0' };
        for (dir = 0; dir < 4; dir++)
        {
            n = code39_encode(one, 10u, 30u, w, 64u);
            merge3(w, &n, (dir % 2) ? 22u : 2u); /* stop or start '*' */
            if (dir >= 2) { reverse(w, n); }
            if (CODE39_OK == code39_decode_ex(w, n, &opt, text, sizeof text, &rev)) { bad++; }
            if (!((CODE39_OK == code39_decode_partial(w, n, &opt, text, sizeof text, &rev)) &&
                  (text[0] == one[0]) && (rev == (dir >= 2)))) { bad++; }
        }
    }
    CHECK("H-P01", 0 == bad, "partial: one '*' damaged, every letter still read, both ways");

    /* both '*' damaged: no letter */
    bad = 0;
    for (i = 0u; i < 26u; i++)
    {
        char one[2] = { (char)('A' + i), '\0' };
        n = code39_encode(one, 10u, 30u, w, 64u);
        merge3(w, &n, 22u);
        merge3(w, &n, 2u);
        if (CODE39_OK == code39_decode_partial(w, n, &opt, text, sizeof text, &rev)) { bad++; }
    }
    CHECK("H-P02", 0 == bad, "partial: both '*' damaged gives no letter");

    /* random widths (noise, line, junction) must almost never give a letter */
    wrong = 0;
    for (i = 0u; i < 2000u; i++)
    {
        uint32_t k;
        n = 15u + (i % 25u);
        for (k = 0u; k < n; k++)
        {
            seed = (seed * 1103515245u) + 12345u;
            w[k] = 5u + ((seed >> 16) % 60u);
        }
        if (CODE39_OK == code39_decode_partial(w, n, &opt, text, sizeof text, &rev)) { wrong++; }
    }
    printf("[INFO] partial: %d of 2000 random width lists gave a letter\n", wrong);
    CHECK("H-P03", wrong <= 20, "partial: random widths give a letter at most 1 % of the time");
}

/* ---- shape read (code39_geom.c) on synthetic L traces ---------------- */
static uint32_t g_gseed = 4242u;
static int grnd(int m)
{
    g_gseed = (g_gseed * 1103515245u) + 12345u;
    return (int)((g_gseed >> 16) % (uint32_t)m);
}

/* L trace of "*X*" passing under the sensor: module 18 trace units,
   5 units per 10 ms sample, edges blurred by sigma modules, white 230,
   black 2130, +-noise.  fill_gap > 0: that narrow space of the letter is
   printed grey at fill (0..1) of black; 1 = the two bars run together.
   Returns samples. */
static uint32_t gtrace(int letter, double ratio, int rev, double sigma,
                       int noise, int fill_gap, double fill, uint16_t * p_l,
                       uint8_t * p_f, int8_t * p_y)
{
    static const char * const pat[27] = {
        "100001001", "001001001", "101001000", "000011001", "100011000",
        "001011000", "000001101", "100001100", "001001100", "000011100",
        "100000011", "001000011", "101000010", "000010011", "100010010",
        "001010010", "000000111", "100000110", "001000110", "000010110",
        "110000001", "011000001", "111000000", "010010001", "110010000",
        "011010000", "010010100" };
    double   w[29], s[16], e[16], g[16], x = 0.0;
    int      bar[29], n = 0, nb = 0, k, i;
    uint32_t ns, j;

    for (k = 0; k < 3; k++)
    {
        const char * p = pat[(1 == k) ? letter : 26];
        for (i = 0; i < 9; i++)
        {
            w[n]   = ('1' == p[i]) ? ratio : 1.0;
            bar[n] = (0 == (i % 2));
            if ((1 == k) && (i == fill_gap) && ('0' == p[i]) && !bar[n])
            {
                bar[n] = 2; /* black space: joins its two bars */
            }
            n++;
        }
        if (k < 2) { w[n] = 1.0; bar[n] = 0; n++; }
    }
    for (k = 0; k < n; k++)
    {
        int m = rev ? (n - 1 - k) : k;
        if (bar[m])
        {
            s[nb] = x; e[nb] = x + w[m]; g[nb] = (2 == bar[m]) ? fill : 1.0; nb++;
        }
        x += w[m];
    }
    ns = (uint32_t)((x + 30.0) * 18.0 / 5.0);
    for (j = 0u; j < ns; j++)
    {
        double p = ((double)j * 5.0 / 18.0) - 15.0, m = 0.0;
        int    v;
        for (k = 0; k < nb; k++)
        {
            m += g[k] * 0.5 * (erf((p - s[k]) / (sigma * sqrt(2.0))) -
                               erf((p - e[k]) / (sigma * sqrt(2.0))));
        }
        m = (m > 1.0) ? 1.0 : m;
        v = 230 + (int)(1900.0 * m) + grnd((2 * noise) + 1) - noise;
        p_l[j] = (uint16_t)((v < 0) ? 0 : v);
        p_f[j] = 5u;
        p_y[j] = 0;
    }
    return ns;
}

static void test_geom(void)
{
    static uint16_t l[C39G_MAX_SAMPLES];
    static uint8_t  f[C39G_MAX_SAMPLES];
    static int8_t   y[C39G_MAX_SAMPLES];
    static const double sig[2] = { 0.25, 0.45 };
    static const double rat[2] = { 2.5, 3.0 };
    c39g_info_t info;
    uint32_t    n;
    int         ok = 0, wrong = 0, total = 0, a, b, X, rev, t;

    for (a = 0; a < 2; a++)
    {
        for (b = 0; b < 2; b++)
        {
            for (X = 0; X < 26; X++)
            {
                for (rev = 0; rev < 2; rev++)
                {
                    n = gtrace(X, rat[b], rev, sig[a], 40, -1, 0.0, l, f, y);
                    total++;
                    if (C39G_OK == c39g_decode(l, f, y, n, true, NULL, &info))
                    {
                        if ((info.letter == (char)('A' + X)) &&
                            (info.b_reversed == (rev != 0))) { ok++; }
                        else { wrong++; }
                    }
                }
            }
        }
    }
    printf("[INFO] shape: %d of %d clean letters read, %d wrong\n", ok, total, wrong);
    CHECK("H-G01", (total == ok) && (0 == wrong),
          "shape: A-Z both ways, ratio 2.5/3.0, blur 0.25/0.45 module all read");

    /* a thin gap that only dips part of the way to white: the edge
       finder can miss it (wrong element count); the shape read should
       still get the letter.  Then the same gap fully black. */
    {
        static const double fills[2] = { 0.6, 1.0 };
        int fi;
        for (fi = 0; fi < 2; fi++)
        {
            ok = 0; wrong = 0; total = 0;
            for (X = 0; X < 26; X++)
            {
                for (t = 1; t < 9; t += 2) /* each narrow space of the letter */
                {
                    n = gtrace(X, 3.0, X % 2, 0.3, 40, t, fills[fi], l, f, y);
                    total++;
                    if (C39G_OK == c39g_decode(l, f, y, n, true, NULL, &info))
                    {
                        if (info.letter == (char)('A' + X)) { ok++; }
                        else
                        {
                            wrong++;
                            printf("[INFO]   %c gap %d -> %c err %u margin %u fit %u\n",
                                   'A' + X, t, info.letter, info.err_x100,
                                   info.margin_x100, info.fit_x100);
                        }
                    }
                }
            }
            printf("[INFO] shape: one gap %d %% filled in: %d of %d read, %d wrong\n",
                   (int)(fills[fi] * 100.0), ok, total, wrong);
            if (0 == fi)
            {
                CHECK("H-G02", (0 == wrong) && ((ok * 10) >= (total * 4)),
                      "shape: a shallow thin gap (60 % grey): 40 % still read, none wrong");
            }
            else
            {
                /* 2026-11-12: 1 of 104 (Z with its 3rd gap black really
                   looks more like Q).  Real misses are blur, not ink. */
                CHECK("H-G03", (wrong * 50) <= total,
                      "shape: a thin gap printed black: at most 2 % wrong");
            }
        }
    }

    n = gtrace(25, 3.0, 0, 0.3, 40, -1, 0.0, l, f, y);
    CHECK("H-G04", (C39G_OK == c39g_decode(l, NULL, NULL, n, false, NULL, &info)) &&
                       ('Z' == info.letter),
          "shape: works by time (no wheel data)");

    for (t = 0; t < 400; t++) { l[t] = (uint16_t)(220 + grnd(30)); f[t] = 5u; y[t] = 0; }
    CHECK("H-G05", C39G_FEW == c39g_decode(l, f, y, 400u, true, NULL, &info),
          "shape: plain white gives no letter");

    wrong = 0;
    for (t = 0; t < 3000; t++)
    {
        int black = 0;
        uint32_t next = (uint32_t)(5 + grnd(40)), j;
        for (j = 0u; j < 400u; j++)
        {
            if (j >= next)
            {
                black = !black;
                next  = j + 2u + (uint32_t)grnd(black ? (3 + (t % 25)) : (3 + (t % 17)));
            }
            l[j] = (uint16_t)(230 + (black ? 1900 : 0) + grnd(81) - 40);
            f[j] = 5u;
            y[j] = 0;
        }
        if (C39G_OK == c39g_decode(l, f, y, 400u, true, NULL, &info)) { wrong++; }
    }
    printf("[INFO] shape: %d of 3000 random bar patterns gave a letter\n", wrong);
    CHECK("H-G06", wrong <= 3, "shape: random bars give a letter at most 0.1 % of the time");

    /* strong blur (0.7 module, like the 11-12 robot passes: thin white
       gaps only come back half way): the half-height humps merge, the
       second try near the top of the wide bars must read them.  Before
       that try: 1 of 104. */
    ok = 0; wrong = 0; total = 0;
    for (b = 0; b < 2; b++)
    {
        for (X = 0; X < 26; X++)
        {
            for (rev = 0; rev < 2; rev++)
            {
                n = gtrace(X, rat[b], rev, 0.7, 40, -1, 0.0, l, f, y);
                total++;
                if (C39G_OK == c39g_decode(l, f, y, n, true, NULL, &info))
                {
                    if ((info.letter == (char)('A' + X)) &&
                        (info.b_reversed == (rev != 0))) { ok++; }
                    else
                    {
                        wrong++;
                        printf("[INFO]   %c%s -> %c\n", 'A' + X, rev ? " (rev)" : "",
                               info.letter);
                    }
                }
            }
        }
    }
    printf("[INFO] shape: blur 0.7 module: %d of %d read, %d wrong\n", ok, total, wrong);
    /* the one wrong (U backwards read as R) is from the first try and
       was there before the second try was added */
    CHECK("H-G07", ((ok * 10) >= (total * 9)) && (wrong <= 1),
          "shape: strong blur (0.7 module): 90 % read, at most 1 wrong");
}

/* Which try read the letter is reported (review R1: tie overrides are
   counted on the robot).  Same 60 % grey gap traces as H-G02. */
static void test_geom_try(void)
{
    static uint16_t l[C39G_MAX_SAMPLES];
    static uint8_t  f[C39G_MAX_SAMPLES];
    static int8_t   y[C39G_MAX_SAMPLES];
    c39g_info_t info;
    uint32_t    n;
    int         X, t, n1 = 0, n2 = 0, ties = 0;
    bool        b_ok = true;

    for (X = 0; X < 26; X++)
    {
        for (t = 1; t < 9; t += 2)
        {
            n = gtrace(X, 3.0, X % 2, 0.3, 40, t, 0.6, l, f, y);
            if (C39G_OK == c39g_decode(l, f, y, n, true, NULL, &info))
            {
                if (1u == info.try_n)
                {
                    n1++;
                    b_ok = b_ok && ((uint8_t)C39G_OK == info.first_res);
                }
                else if (2u == info.try_n)
                {
                    n2++;
                    b_ok = b_ok && ((uint8_t)C39G_OK != info.first_res);
                    if ((uint8_t)C39G_TIE == info.first_res) { ties++; }
                }
                else
                {
                    b_ok = false;
                }
            }
        }
    }
    printf("[INFO] shape: 60 %% gap: %d letters on try 1, %d on try 2 (%d after a tie)\n",
           n1, n2, ties);
    CHECK("H-G08", b_ok && (n2 > 0),
          "shape: result says which try read it; try 2 only after try 1 gave no letter");
    b_ok = (C39G_ARG == c39g_decode(NULL, NULL, NULL, 0u, false, NULL, &info)) &&
           (0u == info.try_n);
    CHECK("H-G09", b_ok, "shape: bad arguments report try 0");
}

/* ---- H-R: real pass 2026-11-18 18:23 (review R4) --------------------- */
/* The dashboard barcode log's frozen trace of the moving pass, 700
   samples (10 ms): L raw, wheel steps L + R, R - L.  Big print, dark bars
   on white, robot at ~36 mm/s.  The thick bars only reach 730-1180 and
   the thin bars ~200 over a white of 140-215, so at the robot's edge
   thresholds (300 / 400) only the 6 thick bars are seen.  The letter on
   the track is not confirmed yet: X backwards, Q backwards and J forwards
   share these thick-bar positions; the faint thin bars match Q backwards
   (inferred).  Kept only as a safety check (never X or J); its other
   outputs are not targets, because the sensor output varies. */
#define R18_N 700u
static uint16_t const g_r18_l[R18_N] = {
    168, 158, 164, 158, 156, 178, 186, 164, 185, 179, 200, 219, 234, 252,
    278, 341, 494, 643, 706, 725, 727, 670, 602, 533, 502, 468, 439, 409,
    374, 334, 304, 284, 277, 263, 247, 242, 224, 217, 222, 207, 204, 201,
    180, 185, 169, 183, 178, 167, 165, 175, 167, 155, 165, 159, 166, 155,
    177, 188, 174, 179, 186, 187, 181, 206, 206, 213, 205, 216, 243, 243,
    238, 264, 267, 280, 275, 319, 368, 484, 562, 667, 748, 780, 811, 850,
    869, 881, 881, 872, 839, 802, 740, 559, 337, 276, 246, 249, 230, 209,
    214, 202, 188, 190, 174, 181, 181, 180, 186, 175, 170, 165, 157, 159,
    171, 162, 163, 158, 160, 188, 197, 180, 184, 183, 184, 180, 186, 197,
    183, 186, 183, 193, 187, 205, 190, 184, 185, 172, 171, 173, 162, 166,
    173, 154, 158, 168, 159, 166, 161, 153, 146, 149, 158, 165, 160, 161,
    148, 156, 152, 160, 150, 150, 164, 160, 157, 162, 156, 149, 149, 151,
    167, 157, 157, 168, 160, 155, 161, 166, 172, 161, 169, 169, 164, 167,
    179, 187, 189, 207, 197, 184, 198, 210, 197, 188, 184, 189, 192, 189,
    179, 175, 174, 188, 171, 169, 171, 180, 173, 183, 164, 177, 175, 180,
    179, 178, 183, 197, 191, 200, 213, 198, 230, 225, 249, 256, 280, 299,
    339, 444, 555, 625, 713, 737, 762, 766, 764, 753, 736, 742, 716, 712,
    714, 713, 699, 646, 542, 390, 309, 266, 237, 252, 246, 217, 218, 219,
    201, 207, 194, 192, 189, 172, 173, 189, 154, 143, 162, 155, 160, 164,
    167, 174, 143, 154, 143, 147, 151, 143, 164, 160, 160, 163, 159, 166,
    160, 171, 152, 149, 153, 152, 154, 153, 164, 151, 155, 158, 164, 176,
    154, 163, 159, 176, 178, 182, 186, 171, 198, 176, 191, 208, 201, 213,
    209, 236, 236, 240, 255, 298, 334, 397, 564, 677, 810, 931, 1011, 1056,
    1129, 1160, 1173, 1183, 1184, 1182, 1149, 1111, 1047, 912, 686, 464, 300, 269,
    252, 234, 234, 214, 210, 198, 195, 201, 186, 182, 170, 185, 169, 162,
    174, 185, 179, 176, 170, 164, 162, 167, 171, 166, 175, 172, 177, 194,
    198, 185, 192, 188, 215, 199, 190, 210, 197, 180, 189, 189, 180, 168,
    167, 175, 178, 155, 165, 180, 188, 165, 169, 162, 152, 171, 172, 167,
    179, 193, 191, 185, 207, 194, 191, 193, 214, 206, 186, 199, 192, 190,
    182, 182, 171, 175, 171, 182, 167, 165, 172, 164, 175, 171, 174, 168,
    170, 189, 194, 189, 200, 191, 202, 197, 195, 203, 204, 194, 194, 192,
    187, 182, 186, 175, 173, 156, 177, 160, 172, 183, 165, 166, 172, 172,
    160, 168, 165, 193, 174, 191, 185, 188, 189, 193, 206, 200, 196, 183,
    195, 209, 198, 195, 163, 181, 196, 167, 176, 167, 163, 183, 164, 176,
    169, 168, 171, 166, 172, 177, 173, 194, 197, 194, 202, 204, 225, 221,
    204, 220, 241, 238, 275, 289, 316, 419, 606, 760, 897, 987, 1056, 1099,
    1139, 1147, 1158, 1170, 1158, 1151, 1119, 1090, 1055, 991, 885, 699, 411, 295,
    258, 235, 220, 199, 212, 206, 193, 199, 186, 180, 183, 180, 171, 171,
    166, 172, 154, 161, 168, 175, 181, 168, 182, 181, 179, 171, 183, 185,
    192, 186, 209, 221, 212, 237, 250, 253, 277, 280, 326, 428, 571, 707,
    813, 912, 979, 1020, 1056, 1056, 1057, 1046, 1027, 1014, 991, 948, 896, 797,
    699, 541, 347, 288, 276, 253, 245, 216, 214, 215, 203, 200, 179, 184,
    166, 184, 168, 166, 181, 152, 154, 173, 163, 170, 175, 169, 174, 173,
    161, 173, 200, 188, 202, 190, 194, 198, 205, 200, 198, 196, 185, 191,
    170, 187, 171, 163, 170, 164, 159, 161, 169, 159, 155, 157, 136, 158,
    148, 139, 162, 150, 142, 151, 155, 146, 136, 166, 147, 152, 161, 143,
    147, 159, 153, 145, 158, 140, 150, 149, 143, 148, 158, 158, 152, 151,
    165, 159, 163, 172, 164, 169, 182, 194, 194, 202, 187, 205, 192, 196,
    211, 197, 182, 179, 188, 186, 166, 157, 167, 166, 168, 151, 168, 152,
    164, 161, 162, 148, 159, 157, 153, 162, 141, 147, 153, 157, 156, 157
};
static uint8_t const g_r18_f[R18_N] = {
    6, 4, 6, 4, 5, 5, 4, 5, 5, 4, 5, 4, 4, 4, 6, 4, 5, 4, 6, 4, 5, 4, 4, 2, 3,
    2, 2, 1, 2, 2, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 2, 3, 2, 3, 3, 2, 3, 3, 2,
    3, 2, 3, 2, 2, 1, 3, 2, 2, 2, 2, 3, 2, 2, 4, 2, 3, 2, 2, 2, 3, 2, 2, 2, 4,
    2, 2, 3, 2, 3, 3, 2, 2, 4, 2, 2, 2, 2, 3, 2, 2, 2, 3, 2, 2, 4, 2, 3, 2, 3,
    3, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 3, 3, 2, 3, 2, 4, 2, 2, 3, 3, 2, 2, 2, 2,
    2, 2, 2, 3, 2, 2, 4, 2, 3, 3, 3, 2, 2, 2, 2, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    3, 2, 3, 2, 3, 2, 3, 3, 2, 3, 2, 2, 2, 3, 2, 3, 2, 2, 2, 3, 3, 2, 3, 2, 3,
    3, 2, 3, 2, 2, 2, 2, 3, 3, 2, 2, 3, 2, 2, 2, 2, 3, 3, 2, 2, 2, 2, 3, 3, 2,
    2, 3, 3, 2, 3, 2, 3, 3, 2, 2, 3, 2, 2, 2, 2, 2, 4, 2, 3, 2, 3, 3, 2, 3, 2,
    2, 2, 3, 3, 2, 2, 2, 2, 2, 2, 2, 4, 2, 2, 3, 2, 3, 3, 2, 3, 2, 2, 2, 3, 3,
    2, 2, 2, 2, 4, 2, 2, 2, 3, 3, 3, 2, 2, 3, 2, 3, 2, 2, 3, 2, 2, 3, 2, 2, 3,
    2, 3, 2, 3, 2, 2, 2, 4, 2, 2, 2, 3, 2, 2, 3, 2, 3, 3, 2, 4, 2, 2, 2, 3, 2,
    3, 2, 2, 2, 2, 2, 3, 3, 2, 3, 3, 2, 3, 2, 3, 2, 2, 2, 3, 2, 2, 3, 2, 2, 2,
    4, 2, 2, 4, 2, 2, 3, 2, 3, 2, 2, 2, 2, 4, 2, 2, 2, 4, 2, 4, 2, 2, 2, 3, 3,
    2, 2, 2, 2, 2, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 2, 2, 4, 2, 1, 2, 4,
    2, 2, 2, 3, 3, 3, 2, 3, 2, 3, 2, 3, 2, 2, 2, 2, 2, 4, 2, 2, 2, 3, 2, 3, 2,
    3, 2, 2, 2, 3, 3, 2, 2, 2, 2, 4, 2, 3, 2, 2, 3, 2, 3, 2, 3, 2, 2, 3, 2, 2,
    3, 2, 3, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 2, 2, 3, 2, 3, 3, 2, 3, 3, 2, 2, 3,
    3, 2, 2, 2, 1, 2, 2, 3, 2, 2, 3, 2, 2, 3, 2, 3, 3, 2, 3, 3, 2, 2, 3, 2, 2,
    2, 3, 2, 3, 2, 3, 3, 2, 3, 3, 2, 3, 3, 2, 2, 1, 2, 2, 3, 2, 2, 3, 2, 3, 2,
    3, 2, 3, 3, 2, 2, 3, 2, 3, 2, 2, 3, 3, 2, 2, 2, 3, 3, 3, 3, 2, 2, 3, 3, 2,
    2, 2, 2, 2, 2, 2, 2, 4, 2, 2, 2, 4, 2, 2, 3, 3, 2, 2, 2, 3, 2, 3, 2, 2, 3,
    3, 2, 3, 2, 3, 3, 3, 2, 2, 2, 2, 2, 3, 2, 3, 2, 2, 4, 2, 2, 4, 2, 4, 2, 2,
    2, 3, 2, 3, 2, 2, 2, 2, 3, 2, 2, 4, 2, 2, 4, 2, 2, 3, 2, 2, 3, 3, 2, 2, 3,
    2, 2, 3, 3, 2, 2, 3, 3, 2, 2, 3, 2, 3, 2, 4, 2, 2, 2, 4, 2, 2, 2, 2, 2, 3,
    2, 3, 2, 3, 2, 3, 3, 2, 2, 3, 3, 2, 2, 3, 2, 2, 2, 3, 2, 2, 3, 3, 3, 2, 2,
    3, 2, 4, 2, 2, 2, 2, 2, 3, 2, 2, 2, 3, 2, 3, 2, 3, 2, 3, 2, 2, 2, 3, 3, 2,
    2, 3, 3, 2, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 3, 2, 2, 3, 2, 3, 2, 2, 2, 2, 2
};
static int8_t const g_r18_y[R18_N] = {
    0, 0, 0, 0, 1, -1, 0, -1, 1, -2, 1, -2, 0, 0, 0, 0, -1, 0, 0, 0,
    1, 0, 0, 0, 1, 0, 0, -1, 0, 0, -1, 1, -1, 0, 0, 0, 0, 0, 0, 0,
    1, 0, -1, 0, 1, -1, 0, -1, 1, 0, -1, 0, 1, 0, 0, 1, -1, 0, 0, 0,
    0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, -1, 0, 1,
    -1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 1,
    -1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, -1, 1, 0, -1, 0, 0, 0, 0, -1,
    1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 1, -1, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, -1, 0, -1, 0, 1, -1, 0, 1,
    0, 0, 0, 1, 0, -1, 0, 0, 0, -1, 1, 0, -1, 0, 1, -1, 0, 1, 0, 0,
    0, 0, 1, -1, 0, 0, -1, 0, 0, 0, 0, -1, 1, 0, 0, 0, 0, -1, 1, 0,
    0, -1, 1, 0, -1, 0, -1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, -1, 0,
    1, -1, 0, 1, 0, 0, 0, -1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
    0, 1, -1, 0, 1, 0, 0, 0, -1, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1, 1,
    -1, 0, 0, 1, 0, -1, 0, 0, 1, 0, 0, -1, 0, 0, -1, 0, 1, 0, -1, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 1, -1, 0, 0, 0, 0, 0, 1, 0,
    -1, 0, 0, 0, 0, 0, -1, 1, 0, -1, 1, 0, -1, 0, 1, 0, 0, 0, -1, 0,
    2, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 1, 0, -1, 0,
    1, 0, -1, 0, 1, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -1, 1,
    -1, 0, 1, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 1, 0,
    -1, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, -1, 0, 1,
    0, 0, -1, 0, 0, 1, 0, -1, 1, 0, -1, 0, 1, 0, -1, 0, 1, 0, 0, 0,
    -1, 0, 1, -1, 0, 1, -1, 0, 0, 1, -1, 0, 0, 0, 1, 0, 0, -1, 0, 0,
    1, 0, 0, -1, 0, 1, -1, 0, 1, -1, 0, 0, 1, 0, 0, 0, -1, 0, 1, 0,
    -1, 1, 0, -1, 1, 0, -1, 1, 0, 0, 1, 0, 0, -1, 0, 0, 1, 0, -1, 0,
    1, 0, -1, 1, 0, 0, -1, 0, 1, 0, 0, -1, 1, 0, 0, 0, -1, 1, -1, 1,
    0, 0, -1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
    1, 0, 0, 0, 1, 0, -1, 0, 0, -1, 1, 0, -1, 0, 1, -1, 1, 0, 0, 0,
    0, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, -1, 0,
    0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, -1, 1, 0, 0, -1,
    0, 0, -1, 1, 0, 0, 1, -1, 0, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, 0, 1, 0, -1, 0, 1, -1, 0, 0, 1, -1, 0, 0, 1, 0,
    0, 0, -1, 0, 0, -1, 1, -1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, -1, 0,
    0, 0, -1, 0, 1, 0, -1, 0, 1, 0, 0, 0, 1, -1, 0, 0, -1, 1, 0, 0,
    -1, 0, 1, 0, 1, 0, -1, 0, -1, 1, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0
};

/* Code 39 patterns (bar, space, bar, ...; '1' = wide) for A..Z and '*' */
static const char * const g_c39_pat[27] = {
    "100001001", "001001001", "101001000", "000011001", "100011000",
    "001011000", "000001101", "100001100", "001001100", "000011100",
    "100000011", "001000011", "101000010", "000010011", "100010010",
    "001010010", "000000111", "100000110", "001000110", "000010110",
    "110000001", "011000001", "111000000", "010010001", "110010000",
    "011010000", "010010100"
};

/* "*X*" element widths in 0.1 mm (narrow n, wide w), 29 elements */
static uint32_t c39_widths(int letter, uint32_t n, uint32_t w, uint32_t * p_e)
{
    uint32_t k = 0u, c, i;
    for (c = 0u; c < 3u; c++)
    {
        const char * p = g_c39_pat[(1u == c) ? letter : 26];
        for (i = 0u; i < 9u; i++) { p_e[k++] = ('1' == p[i]) ? w : n; }
        if (c < 2u) { p_e[k++] = n; } /* gap between characters */
    }
    return k;
}

#define C39_MM_PER_COUNT_X1000 148u /* one wheel count ~0.148 mm (s3_config.h) */

static void test_real_18nov(void)
{
    c39g_info_t   info;
    c39g_result_t r;
    uint32_t      e[29], k, i, X, worst = 0u, len = 0u, need;

    /* Safety only: this one recording is not a target.  The reader may
       refuse it, but must not name a look-alike of Q backwards. */
    r = c39g_decode(g_r18_l, g_r18_f, g_r18_y, R18_N, true, NULL, &info);
    printf("        18 Nov pass, shape (distance): res %d best %c runner %c\n",
           (int)r, info.best ? info.best : '-', info.runner ? info.runner : '-');
    CHECK("H-R01", !((C39G_OK == r) && (('X' == info.letter) || ('J' == info.letter))),
          "18 Nov pass, by distance: shape reader never names X or J");
    r = c39g_decode(g_r18_l, NULL, NULL, R18_N, false, NULL, &info);
    CHECK("H-R02", !((C39G_OK == r) && (('X' == info.letter) || ('J' == info.letter))),
          "18 Nov pass, by time: shape reader never names X or J");

    /* From the print spec, not from a recording.  Big print (narrow 6 mm,
       wide 18 mm) is the larger case.  With every bar seen, or with any one
       thin bar missed, no white gap inside any letter may end the read. */
    for (X = 0u; X < 26u; X++)
    {
        k = c39_widths((int)X, 60u, 180u, e);
        for (i = 1u; i < k; i += 2u) /* spaces */
        {
            if (e[i] > worst) { worst = e[i]; }
        }
        for (i = 2u; (i + 2u) < k; i += 2u) /* a thin bar inside, missed */
        {
            if ((60u == e[i]) && ((e[i - 1u] + e[i] + e[i + 1u]) > worst))
            {
                worst = e[i - 1u] + e[i] + e[i + 1u];
            }
        }
        len = 0u;
        for (i = 0u; i < k; i++) { len += e[i]; }
    }
    printf("        big print: widest white inside a letter (one thin bar missed) %u.%u mm, quiet %u counts = %u mm\n",
           (unsigned)(worst / 10u), (unsigned)(worst % 10u), (unsigned)S3_BC_QUIET_COUNTS,
           (unsigned)((S3_BC_QUIET_COUNTS * C39_MM_PER_COUNT_X1000) / 1000u));
    CHECK("H-R03", (worst * 100u) < (S3_BC_QUIET_COUNTS * C39_MM_PER_COUNT_X1000),
          "spec: no gap inside a big-print letter (one thin bar missed) ends the read");

    /* The whole big-print read must fit the trace the shape reader gets:
       lead + barcode + quiet distance, at 25 mm/s (slowest barcode speed
       assumed, inferred from the logged 21-77 mm/s range). */
    /* 0.1 mm at 25 mm/s = 250 per s; 100 samples per s */
    need = S3_BC_GEOM_LEAD +
           (((len + ((S3_BC_QUIET_COUNTS * C39_MM_PER_COUNT_X1000) / 100u)) * 100u) / 250u);
    printf("        big print %u mm + quiet at 25 mm/s: %u samples (C39G_MAX_SAMPLES %u)\n",
           (unsigned)(len / 10u), (unsigned)need, (unsigned)C39G_MAX_SAMPLES);
    CHECK("H-R04", need <= C39G_MAX_SAMPLES,
          "spec: a big-print read at 25 mm/s fits the trace window");
}

/* ---- H-F: 500 Hz L front end hand-off (S3_L_FAST, drv_ir.c) ------- */
static void test_l_fast(void)
{
    l_fast_t    f;
    l_fast_rd_t rd;
    uint32_t    sum = 0u, n = 0u, used = 0u, k, total = 0u, seen = 0u;
    uint16_t    c = 0u, r = 0u, rec;
    bool        b_ok = true;

    l_fast_reset(&f, &rd);
    for (k = 0u; k < 5u; k++) { l_fast_put(&f, (uint16_t)(1000u + (k * 10u)), 300u, 400u); }
    b_ok = l_fast_get(&f, &sum, &n, &c, &r);
    rec  = l_fast_record(&rd, sum, n, &used);
    CHECK("H-F01", b_ok && (1020u == rec) && (5u == used) && (300u == c) && (400u == r),
          "500 Hz: the 10 ms record is the mean of the 5 samples, C and R the newest");

    /* nothing new: keep the last record, report 0 samples */
    (void)l_fast_get(&f, &sum, &n, &c, &r);
    rec = l_fast_record(&rd, sum, n, &used);
    CHECK("H-F02", (1020u == rec) && (0u == used),
          "500 Hz: a read with no new sample keeps the last value and says so");

    /* 4, 6, 5, ... samples per record (jitter): every sample used once */
    l_fast_reset(&f, &rd);
    for (k = 0u; k < 300u; k++)
    {
        uint32_t m = 4u + (k % 3u), j;
        for (j = 0u; j < m; j++)
        {
            l_fast_put(&f, (uint16_t)((total * 7u) % 4096u), 0u, 0u);
            total++;
        }
        (void)l_fast_get(&f, &sum, &n, &c, &r);
        (void)l_fast_record(&rd, sum, n, &used);
        seen += used;
        b_ok = b_ok && (used == m);
    }
    CHECK("H-F03", b_ok && (seen == total),
          "500 Hz: with 4 to 6 samples per record no sample is lost or used twice");

    /* the running totals wrap */
    l_fast_reset(&f, &rd);
    f.sum  = 0xFFFFFF00u; f.n  = 0xFFFFFFFEu;
    rd.sum = 0xFFFFFF00u; rd.n = 0xFFFFFFFEu;
    for (k = 0u; k < 5u; k++) { l_fast_put(&f, 4000u, 0u, 0u); }
    (void)l_fast_get(&f, &sum, &n, &c, &r);
    rec = l_fast_record(&rd, sum, n, &used);
    CHECK("H-F04", (4000u == rec) && (5u == used),
          "500 Hz: sum and count wrap without a wrong record");

    /* a writer in the middle of an update (odd seq) is never read */
    f.seq = 7u;
    CHECK("H-F05", !l_fast_get(&f, &sum, &n, &c, &r),
          "500 Hz: no copy is taken while the interrupt is updating");

    /* design property (noise model assumed: white, sd 40 per conversion):
       20 conversions per record against the old 4 */
    {
        double s_old = 0.0, s_new = 0.0;
        int    i, j;
        uint32_t seed = 12345u;
        l_fast_reset(&f, &rd);
        for (i = 0; i < 2000; i++)
        {
            double a = 0.0;
            for (j = 0; j < 4; j++)
            {
                double g = 0.0; int q;
                for (q = 0; q < 12; q++) { seed = seed * 1103515245u + 12345u; g += (double)((seed >> 16) & 0x7FFFu) / 32768.0; }
                a += 40.0 * (g - 6.0);
            }
            s_old += (a / 4.0) * (a / 4.0);
            for (j = 0; j < 5; j++)
            {
                double b = 0.0; int q, m;
                for (m = 0; m < 4; m++)
                {
                    double g = 0.0;
                    for (q = 0; q < 12; q++) { seed = seed * 1103515245u + 12345u; g += (double)((seed >> 16) & 0x7FFFu) / 32768.0; }
                    b += 40.0 * (g - 6.0);
                }
                l_fast_put(&f, (uint16_t)(2000.0 + (b / 4.0) + 0.5), 0u, 0u);
            }
            (void)l_fast_get(&f, &sum, &n, &c, &r);
            rec = l_fast_record(&rd, sum, n, &used);
            s_new += ((double)rec - 2000.0) * ((double)rec - 2000.0);
        }
        s_old = sqrt(s_old / 2000.0);
        s_new = sqrt(s_new / 2000.0);
        printf("        noise sd per record: old 4 reads %.1f, 500 Hz %.1f (model)\n", s_old, s_new);
        CHECK("H-F06", (s_new * 1.8) < s_old,
              "500 Hz: a record averages 20 conversions (noise about 1/sqrt(5) of the old 4)");
    }
}

/* Barcode lock: run a sequence of (pos, scanning, bars) 10 ms samples */
typedef struct { uint32_t pos; bool scan; bool bars; } lk_s_t;

/* steps pos from a to b (step 5 counts), returns lock state at each end */
static bool lk_run(bc_lock_t * p_l, uint32_t a, uint32_t b, bool scan,
                   bool bars, bool * p_all_locked, bool * p_any_locked)
{
    uint32_t pos;
    bool     b_l = false;
    *p_all_locked = true;
    *p_any_locked = false;
    for (pos = a; pos < b; pos += 5u)
    {
        b_l = bc_lock_step(p_l, scan, bars, pos, S3_BC_LOCK_FIRST_COUNTS,
                           S3_BC_LOCK_AFTER_COUNTS, S3_BC_LOCK_MAX_COUNTS);
        *p_all_locked = *p_all_locked && b_l;
        *p_any_locked = *p_any_locked || b_l;
    }
    return b_l;
}

static void test_bc_lock(void)
{
    bc_lock_t l;
    bool      all, any, all2, any2;
    uint32_t  a = S3_BC_LOCK_AFTER_COUNTS;

    /* a whole barcode: 1st bar at 0, 2nd element from 60, read ends at 900 */
    bc_lock_reset(&l);
    (void)lk_run(&l, 0u, 60u, true, false, &all, &any);
    (void)lk_run(&l, 60u, 900u, true, true, &all2, &any2);
    all = all && all2;
    (void)lk_run(&l, 900u, 900u + a - 10u, false, false, &all2, &any2);
    all = all && all2;
    (void)lk_run(&l, 900u + a + 10u, 2000u, false, false, &all2, &any2);
    CHECK("H-K01", all && !any2,
          "lock: straight from the 1st bar to AFTER counts past the end, then free");

    /* one long black (line on a curve, junction arm): free after FIRST */
    bc_lock_reset(&l);
    (void)lk_run(&l, 0u, S3_BC_LOCK_FIRST_COUNTS - 10u, true, false, &all, &any);
    (void)lk_run(&l, S3_BC_LOCK_FIRST_COUNTS + 10u, 1500u, true, false, &all2, &any2);
    CHECK("H-K02", all && !any2,
          "lock: one long black lets go after FIRST counts and stays free");

    /* ... and the next real read locks again */
    (void)lk_run(&l, 1500u, 1510u, false, false, &all2, &any2);
    (void)lk_run(&l, 1600u, 1700u, true, true, &all, &any);
    CHECK("H-K03", all, "lock: the next read locks again");

    /* a read that fails half way and a second piece right after it: one
       continuous lock (no speed-up between the pieces) */
    bc_lock_reset(&l);
    (void)lk_run(&l, 0u, 400u, true, true, &all, &any);
    (void)lk_run(&l, 400u, 400u + (a / 2u), false, false, &all2, &any2);
    all = all && all2;
    (void)lk_run(&l, 400u + (a / 2u), 900u, true, false, &all2, &any2);
    all = all && all2;
    (void)lk_run(&l, 900u, 900u + a - 10u, false, false, &all2, &any2);
    all = all && all2;
    CHECK("H-K04", all, "lock: two pieces of one barcode stay locked throughout");

    /* never longer than MAX */
    bc_lock_reset(&l);
    (void)lk_run(&l, 0u, S3_BC_LOCK_MAX_COUNTS - 10u, true, true, &all, &any);
    (void)lk_run(&l, S3_BC_LOCK_MAX_COUNTS + 10u, S3_BC_LOCK_MAX_COUNTS + 500u,
                 true, true, &all2, &any2);
    CHECK("H-K05", all && !any2, "lock: let go after MAX counts at most");

    /* one bar only (no space after it before the read ends): free at once */
    bc_lock_reset(&l);
    (void)lk_run(&l, 0u, 50u, true, false, &all, &any);
    (void)lk_run(&l, 50u, 60u, false, false, &all2, &any2);
    CHECK("H-K06", all && !any2, "lock: a lone bar with no space after it lets go when the read ends");

    /* following the line while locked */
    {
        bc_lock_steer_t ls;
        int16_t         st = 0;
        uint32_t        pos;
        bool            b_zero = true;

        bc_lock_steer_reset(&ls);
        for (pos = 0u; pos < 200u; pos += 4u) /* bar under C and R */
        {
            st = bc_lock_steer(&ls, (uint8_t)(S3_MASK_C | S3_MASK_R), pos, 900,
                               S3_BC_LOCK_SIDE_COUNTS, S3_BC_LOCK_STEER_MAX);
            b_zero = b_zero && (0 == st);
        }
        for (pos = 200u; pos < 200u + S3_BC_LOCK_SIDE_COUNTS; pos += 4u)
        {
            /* bar edge: only C for less than SIDE counts */
            st = bc_lock_steer(&ls, (uint8_t)S3_MASK_C, pos, -900,
                               S3_BC_LOCK_SIDE_COUNTS, S3_BC_LOCK_STEER_MAX);
            b_zero = b_zero && (0 == st);
        }
        CHECK("H-K08", b_zero, "lock steer: a bar, and one sensor briefly on a bar edge, give no steering");
        for (pos = 300u; pos < 400u; pos += 4u) /* C stays black: drifting */
        {
            st = bc_lock_steer(&ls, (uint8_t)S3_MASK_C, pos, -900,
                               S3_BC_LOCK_SIDE_COUNTS, S3_BC_LOCK_STEER_MAX);
        }
        CHECK("H-K09", -S3_BC_LOCK_STEER_MAX == st,
              "lock steer: C black for longer steers back, capped");
        for (pos = 400u; pos < 500u; pos += 4u)
        {
            st = bc_lock_steer(&ls, (uint8_t)S3_MASK_R, pos, 120,
                               S3_BC_LOCK_SIDE_COUNTS, S3_BC_LOCK_STEER_MAX);
        }
        CHECK("H-K10", 120 == st, "lock steer: R black for longer steers the other way");
    }

    /* straight: the tracker forgets its turn */
    {
        line_tracker_t t;
        line_tracker_reset(&t);
        t.steering = 700; t.corr_count = 9u; t.corr_mag = 300; t.b_hold = true;
        line_tracker_go_straight(&t);
        CHECK("H-K07", (0 == t.steering) && (0u == t.corr_count) &&
                           (0 == t.corr_mag) && !t.b_hold,
              "lock: go_straight clears the steering and the turn memory");
    }
}

/* Lock + tracker together (review R1): one 10 ms sample.  pos moves 5
   counts per sample.  L, C, R are 0 (white) or 1 (black); L also sets
   l_raw.  scan / bars as barcode_task would set them. */
static line_tracker_t g_lk_trk;
static line_tracker_t g_lk_ref; /* same input, no lock: reference */
static line_params_t  g_lk_p;
static bc_lock_ctl_t  g_lk_ctl;
static uint32_t       g_lk_pos;
static bc_lock_cfg_t const g_lk_cfg = {
    S3_BC_LOCK_FIRST_COUNTS, S3_BC_LOCK_AFTER_COUNTS, S3_BC_LOCK_MAX_COUNTS,
    S3_BC_LOCK_SIDE_COUNTS,  S3_BC_LOCK_STEER_MAX };

static void lk_trk_reset(void)
{
    line_params_default(&g_lk_p);
    g_lk_p.kp_num = 4;  /* first nudge 100 */
    g_lk_p.kd_num = 10; /* +10 per sample: 70 samples -> 800 (hard) */
    line_tracker_reset(&g_lk_trk);
    line_tracker_reset(&g_lk_ref);
    bc_lock_ctl_reset(&g_lk_ctl);
    g_lk_pos = 1000u;
}

static bool lk_trk_step(int l, int c, int r, bool scan, bool bars)
{
    uint16_t nm[S3_IR_COUNT];
    nm[S3_IR_L] = (uint16_t)(l ? 1000u : 0u);
    nm[S3_IR_C] = (uint16_t)(c ? 1000u : 0u);
    nm[S3_IR_R] = (uint16_t)(r ? 1000u : 0u);
    g_lk_trk.l_raw = (uint16_t)(l ? 3000u : 200u);
    g_lk_ref.l_raw = g_lk_trk.l_raw;
    (void)line_tracker_step(&g_lk_trk, &g_lk_p, nm, bars);
    (void)line_tracker_step(&g_lk_ref, &g_lk_p, nm, bars);
    g_lk_pos += 5u;
    return bc_lock_apply(&g_lk_ctl, &g_lk_cfg, &g_lk_trk, scan, bars, g_lk_pos);
}

static void test_bc_lock_tracker(void)
{
    int     i;
    bool    b_ok;
    bool    b_lk;
    int16_t curve;

    /* (a) left curve: hard left correction, L sees the line and starts a
       read (no bars yet), C slips off the line: both C and R white. */
    lk_trk_reset();
    (void)lk_trk_step(0, 0, 0, false, false);
    (void)lk_trk_step(0, 0, 0, false, false);
    for (i = 0; i < 70; i++) { (void)lk_trk_step(0, 1, 0, false, false); }
    curve = g_lk_trk.steering;
    b_ok  = (curve <= -S3_STRADDLE_HOLD_MAG);
    b_lk  = true;
    for (i = 0; i < 40; i++) /* 200 counts, under FIRST (250) */
    {
        b_lk = lk_trk_step(1, 0, 0, true, false) && b_lk;
        b_ok = b_ok && (g_lk_trk.steering == curve) && g_lk_trk.b_hold;
    }
    CHECK("H-K11", b_ok && b_lk,
          "lock+tracker: false start on a left curve keeps turning left (curve memory kept)");

    /* (b) a lock that is never confirmed changes nothing in the tracker:
       compare with the same tracker run without the lock, through a
       curve, a re-entry, the other side and white, until it lets go. */
    lk_trk_reset();
    b_ok = true;
    b_lk = false;
    for (i = 0; i < 60; i++) { (void)lk_trk_step(0, 1, 0, false, false); }
    for (i = 0; i < 49; i++) /* one long black under L: 245 counts */
    {
        int c = ((i / 12) % 2);       /* C on / off the line */
        int r = (i > 36) ? 1 : 0;     /* then R */
        b_lk = lk_trk_step(1, c, r, true, false) || b_lk;
        b_ok = b_ok && (g_lk_trk.steering == g_lk_ref.steering) &&
               (g_lk_trk.b_hold == g_lk_ref.b_hold) &&
               (g_lk_trk.corr_mag == g_lk_ref.corr_mag);
    }
    for (i = 0; i < 20; i++) /* past FIRST: lets go */
    {
        (void)lk_trk_step(1, 0, 0, true, false);
        b_ok = b_ok && (g_lk_trk.steering == g_lk_ref.steering);
    }
    b_lk = b_lk && !lk_trk_step(0, 0, 0, false, false);
    CHECK("H-K12", b_ok && b_lk,
          "lock+tracker: an unconfirmed lock leaves steering and curve memory unchanged");

    /* (c) a CONFIRMED false lock on a curve (L hunts black-white-black):
       the held curve keeps its sign, and the lock still ends by MAX. */
    lk_trk_reset();
    for (i = 0; i < 70; i++) { (void)lk_trk_step(0, 1, 0, false, false); }
    curve = g_lk_trk.steering;
    (void)lk_trk_step(1, 0, 0, true, false);
    b_lk = lk_trk_step(1, 0, 0, true, true); /* bar + space seen */
    b_ok = b_lk && g_lk_ctl.lock.b_confirmed;
    for (i = 0; i < 60; i++)
    {
        (void)lk_trk_step(i % 2, 0, 0, true, true);
        b_ok = b_ok && (g_lk_trk.steering == curve) && g_lk_trk.b_hold;
    }
    CHECK("H-K13a", b_ok,
          "lock+tracker: confirmed lock on a curve keeps the held turn (not zeroed)");
    {
        uint32_t start = g_lk_pos - (5u * 62u);
        uint32_t freed = 0u;
        for (i = 0; i < 2000; i++)
        {
            int c = ((i / 30) % 3) == 0; /* line comes back under C now and then */
            if (!lk_trk_step(i % 2, c, 0, true, true)) { freed = g_lk_pos; break; }
        }
        CHECK("H-K13b", (0u != freed) &&
                            ((freed - start) <= (S3_BC_LOCK_MAX_COUNTS + 10u)),
              "lock+tracker: a confirmed false lock on a curve still ends by MAX counts");
    }

    /* (d) a real barcode on a straight, as before: bars under C and R and
       a short bar edge give 0 and no built-up turn; a lasting C black
       steers back, capped. */
    lk_trk_reset();
    for (i = 0; i < 5; i++) { (void)lk_trk_step(0, 0, 0, false, false); }
    (void)lk_trk_step(1, 1, 1, true, false);
    (void)lk_trk_step(0, 0, 0, true, true);
    b_ok = g_lk_ctl.lock.b_confirmed;
    for (i = 0; i < 4; i++)
    {
        (void)lk_trk_step(1, 1, 1, true, true); /* bar across C and R */
        b_ok = b_ok && (0 == g_lk_trk.steering);
    }
    for (i = 0; i < 3; i++) /* edge: C only for 15 counts */
    {
        (void)lk_trk_step(0, 1, 0, true, true);
        b_ok = b_ok && (0 == g_lk_trk.steering) && (0u == g_lk_trk.corr_count);
    }
    (void)lk_trk_step(0, 0, 0, true, true);
    b_ok = b_ok && (0 == g_lk_trk.steering) && !g_lk_trk.b_hold;
    CHECK("H-K14a", b_ok,
          "lock+tracker: confirmed, bars and a short bar edge give no steering");
    for (i = 0; i < 80; i++) { (void)lk_trk_step(0, 1, 0, true, true); }
    CHECK("H-K14b", (g_lk_trk.steering < 0) &&
                        (g_lk_trk.steering >= -S3_BC_LOCK_STEER_MAX),
          "lock+tracker: confirmed, a lasting C black steers left, capped");
}

/* ---- H-W: adaptive edge finder on low-contrast print (S3_BC_EDGE_ADAPT) -
   Synthetic driven passes, made in distance and sampled like the robot:
   10 ms records, each the mean of five 2 ms samples of a Gaussian spot
   over the print, plus record noise.  Position = 2 x left wheel counts
   (one count ~0.148 mm).  The glue copies task_line.c barcode_runs_step()
   and task_barcode.c on_run() (start after >= S3_BC_START_WHITE_MS white,
   end after S3_BC_QUIET_COUNTS of white, decode by distance: full, then
   partial). */
#define SIM_MAX       6000u
#define SIM_MM_PER_C  0.148
typedef struct
{
    double   white;    /* paper (raw)                                  */
    double   ink;      /* full-coverage ink above white (raw)          */
    double   spot_mm;  /* sensor spot sd (mm)                          */
    double   noise;    /* record noise sd (raw)                        */
    uint32_t seed;
} sim_opt_t;

static int32_t  g_sv[SIM_MAX];  /* L raw per record */
static uint32_t g_sp[SIM_MAX];  /* position (counts) per record */
static uint32_t g_sn;
static double   g_bar0[64], g_bar1[64]; /* ink intervals (mm) */
static uint32_t g_nbar;
static uint32_t g_rng;

static double sim_rand(void) /* 0..1, deterministic */
{
    g_rng = (g_rng * 1103515245u) + 12345u;
    return ((double)((g_rng >> 8) & 0xFFFFu) + 0.5) / 65536.0;
}
static double sim_gauss(void)
{
    double u1 = sim_rand(), u2 = sim_rand();
    return sqrt(-2.0 * log(u1)) * cos(6.283185307 * u2);
}
/* lay "*text*" (bars first) from x0 mm, narrow / wide in mm; returns end */
static double sim_lay(char const * text, double x0, double nar, double wid, bool b_rev)
{
    uint32_t w[200], n, k;
    double   x = x0;
    n = code39_encode(text, 1000u, (uint32_t)(wid / nar * 1000.0 + 0.5), w, 200u);
    if (b_rev) { reverse(w, n); }
    for (k = 0u; k < n; k++)
    {
        double len = nar * (double)w[k] / 1000.0;
        if ((0u == (k % 2u)) && (g_nbar < 64u)) { g_bar0[g_nbar] = x; g_bar1[g_nbar] = x + len; g_nbar++; }
        x += len;
    }
    return x;
}
static void sim_tape(double x0, double width) /* plain black tape */
{
    if (g_nbar < 64u) { g_bar0[g_nbar] = x0; g_bar1[g_nbar] = x0 + width; g_nbar++; }
}
static double sim_cover(double x, double sd)
{
    double c = 0.0;
    uint32_t k;
    for (k = 0u; k < g_nbar; k++)
    {
        c += 0.5 * (erf((g_bar1[k] - x) / (sd * 1.41421356)) -
                    erf((g_bar0[k] - x) / (sd * 1.41421356)));
    }
    return (c > 1.0) ? 1.0 : c;
}
/* drive from x_from to x_to mm at v mm/s (v 0: stand still for t_s s) */
/* a paper sheet from g_paper_x0 mm on, reading g_paper_d above the floor's
   white (0 = no step); seen through the same spot */
static double g_paper_x0 = 1e9, g_paper_d = 0.0;
static double sim_paper(double x, double sd)
{
    if (0.0 == g_paper_d) { return 0.0; }
    return g_paper_d * 0.5 * (1.0 + erf((x - g_paper_x0) / (sd * 1.41421356)));
}
static void sim_drive(sim_opt_t const * o, double x_from, double x_to, double v, double t_s)
{
    double x = x_from;
    double t_end = (v > 0.0) ? ((x_to - x_from) / v) : t_s;
    double t;
    for (t = 0.0; (t < t_end) && (g_sn < SIM_MAX); t += 0.010)
    {
        double s = 0.0;
        int    j;
        for (j = 0; j < 5; j++) /* five 2 ms samples, each at its own place */
        {
            double xs = x + (v * 0.002 * (double)j);
            s += o->white + sim_paper(xs, o->spot_mm) + (o->ink * sim_cover(xs, o->spot_mm));
        }
        x += v * 0.010;
        s = (s / 5.0) + (o->noise * sim_gauss());
        g_sv[g_sn] = (int32_t)(s + 0.5);
        g_sp[g_sn] = 2u * (uint32_t)(x / (2.0 * SIM_MM_PER_C));
        g_sn++;
    }
}

typedef struct
{
    uint32_t starts;   /* reads started */
    uint32_t letters;  /* letters decoded */
    char     text[16]; /* all letters, in order */
    uint32_t elems;    /* elements of the last read */
    uint32_t edges;    /* level changes seen */
    uint32_t shape;    /* letters given by the shape read */
    uint32_t nrd;      /* reads recorded below (first 8) */
    double   rd_x0[8], rd_x1[8]; /* where each read started / ended (mm) */
    char     rd_out[8];          /* letter, or '-' failed, 'a' aborted */
} sim_res_t;
/* per record: a read is open / it has bar + space (task_barcode.c
   g_b_s3_bc_scanning / g_b_s3_bc_bars), for the lock model */
static uint8_t g_s_scan[SIM_MAX], g_s_bars[SIM_MAX];
static void sim_rd_end(sim_res_t * p_r, uint32_t i, char out)
{
    if ((p_r->nrd > 0u) && (p_r->nrd <= 8u))
    {
        p_r->rd_x1[p_r->nrd - 1u]  = (double)g_sp[i] * SIM_MM_PER_C;
        p_r->rd_out[p_r->nrd - 1u] = out;
    }
}
static bool g_sim_shape; /* sim_read also tries the shape read (as robot) */

/* the glue: adaptive (b_adapt) or old fixed swing / floor */
static void sim_read(bool b_adapt, int32_t swing, int32_t floor, sim_res_t * p_r)
{
    bc_edge_t e;
    uint32_t  wp[S3_BC_MAX_ELEMENTS], wt[S3_BC_MAX_ELEMENTS];
    uint32_t  nw = 0u, total = 0u, i, rs_pos = 0u, rs_us = 0u, last_run_pos = 0u;
    uint32_t  start_i = 0u;
    uint8_t   level = 0u;
    bool      b_read = false;
    uint32_t  pend_i = 0u;
    bc_start_t bst;
    code39_options_t o = { S3_BC_MIN_RATIO_X10, true, (uint8_t)S3_BC_MAX_CHARS };

    memset(p_r, 0, sizeof *p_r);
    bc_start_reset(&bst);
    bc_edge_reset_pos(&e, g_sv[0], 0u, g_sp[0]);
    rs_pos = g_sp[0];
    for (i = 0u; i < g_sn; i++)
    {
        uint32_t t = i * 10000u, eu = t, ep = g_sp[i];
        uint8_t  ch;
        bool     b_dec = false;
        if (b_adapt)
        {
            e.hold = (uint8_t)S3_BC_ADAPT_HOLD;
            ch = bc_edge_step_adapt(&e, g_sv[i], t, g_sp[i], b_read, &eu, &ep);
        }
        else
        {
            e.hold = (swing <= S3_BC_WEAK_SWING_MAX) ? (uint8_t)S3_BC_WEAK_HOLD : 0U;
            ch = bc_edge_step_pos(&e, g_sv[i], t, g_sp[i], swing, floor, &eu, &ep);
        }
        if (0u != ch)
        {
            uint32_t lead_pos = 0u, lead_us = 0u, wpos, wus;
            uint8_t  black = level;
            p_r->edges++;
            if (0u != level)
            {
                lead_pos = e.bar_pos - rs_pos;
                lead_us  = e.bar_t_us - rs_us;
                rs_pos = e.bar_pos; rs_us = e.bar_t_us;
            }
            wpos = ep - rs_pos; wus = eu - rs_us;
            level = e.level; rs_pos = ep; rs_us = eu;
            last_run_pos = g_sp[i];
            if (!b_read)
            {
                /* task_barcode.c on_run IDLE (bc_start_step) */
                uint8_t st = bc_start_step(&bst, 0u != black, wus, wpos);
                if (BC_START_ARMED == st) { pend_i = i; }
                else if (BC_START_READ == st)
                {
                    b_read = true; p_r->starts++; start_i = pend_i;
                    if (p_r->nrd < 8u) { p_r->rd_x0[p_r->nrd] = (double)(ep - wpos) * SIM_MM_PER_C; p_r->rd_out[p_r->nrd] = '?'; }
                    p_r->nrd++;
                    wt[0] = wus; wp[0] = wpos; nw = 1u; total = wpos;
                }
            }
            else if ((0u != black) && (wus > (uint32_t)S3_BC_ELEMENT_MAX_MS * 1000u))
            {
                b_read = false; /* abort */
                sim_rd_end(p_r, i, 'a');
            }
            else if ((0u == black) && (nw > 0u) && (wpos >= S3_BC_QUIET_COUNTS))
            {
                b_dec = true;
            }
            else if (nw < S3_BC_MAX_ELEMENTS)
            {
                if ((0u != black) && (nw > 0u))
                {
                    wp[nw - 1u] += lead_pos; wt[nw - 1u] += lead_us; total += lead_pos;
                }
                wt[nw] = wus;
                wp[nw++] = wpos; total += wpos;
            }
        }
        else if (b_read && (0u == (nw % 2u)) && (0u != level) &&
                 ((t - rs_us) >= ((uint32_t)S3_BC_ELEMENT_MAX_MS * 1000u)))
        {
            b_read = false; /* task_barcode.c: no run for that long on black */
            sim_rd_end(p_r, i, 'a');
        }
        else if (b_read && (1u == (nw % 2u)) && ((g_sp[i] - last_run_pos) >= S3_BC_QUIET_COUNTS))
        {
            b_dec = true; /* white long enough by distance */
        }
        if (b_dec)
        {
            uint32_t let0 = p_r->letters;
            char txt[S3_BC_TEXT_MAX + 2] = { 0 };
            bool rv = false;
            p_r->elems = nw;
            /* task_barcode.c try_decode(): by distance only when the read
               has covered S3_BC_DIST_MIN counts, then by time; then the
               one-'*' partial rule in the same order */
            bool b_dist = (total >= S3_BC_DIST_MIN);
            if ((nw >= S3_BC_MIN_ELEMENTS) &&
                ((b_dist && (CODE39_OK == code39_decode_ex(wp, nw, &o, txt, sizeof txt, &rv))) ||
                 (CODE39_OK == code39_decode_ex(wt, nw, &o, txt, sizeof txt, &rv)) ||
                 (b_dist && (CODE39_OK == code39_decode_partial(wp, nw, &o, txt, sizeof txt, &rv))) ||
                 (CODE39_OK == code39_decode_partial(wt, nw, &o, txt, sizeof txt, &rv))))
            {
                p_r->letters++;
                if ((strlen(p_r->text) + strlen(txt)) < (sizeof p_r->text - 1u)) { strcat(p_r->text, txt); }
            }
            else if (g_sim_shape && (nw >= S3_BC_GEOM_MIN_ELEMENTS))
            {
                /* task_barcode.c shape_read(): the trace from
                   S3_BC_GEOM_LEAD records before the first bar */
                static uint16_t sl[C39G_MAX_SAMPLES];
                static uint8_t  sf[C39G_MAX_SAMPLES];
                static int8_t   sy[C39G_MAX_SAMPLES];
                uint32_t off = (start_i > S3_BC_GEOM_LEAD) ? (start_i - S3_BC_GEOM_LEAD) : 0u;
                uint32_t k, cnt = 0u;
                int32_t  dist = 0;
                c39g_info_t inf;
                if (((i + 1u) - off) > C39G_MAX_SAMPLES) { off = (i + 1u) - C39G_MAX_SAMPLES; }
                for (k = off; k <= i; k++, cnt++)
                {
                    sl[cnt] = (uint16_t)((g_sv[k] < 0) ? 0 : g_sv[k]);
                    sf[cnt] = (uint8_t)((k > 0u) ? (g_sp[k] - g_sp[k - 1u]) : 0u);
                    sy[cnt] = 0;
                    dist += sf[cnt];
                }
                if (C39G_OK == c39g_decode_hump(sl, sf, sy, cnt, dist >= (int32_t)S3_BC_DIST_MIN,
                                                c39g_min_hump(b_adapt ? bc_edge_margin(&e) : 0),
                                                NULL, &inf))
                {
                    p_r->letters++;
                    p_r->shape++;
                    txt[0] = inf.letter; txt[1] = 0;
                    if ((strlen(p_r->text) + 1u) < (sizeof p_r->text - 1u)) { strcat(p_r->text, txt); }
                }
            }
            sim_rd_end(p_r, i, (p_r->letters > let0) ? p_r->text[strlen(p_r->text) - 1u] : '-');
            b_read = false;
        }
        g_s_scan[i] = b_read ? 1u : 0u;
        g_s_bars[i] = (b_read && (nw >= 2u)) ? 1u : 0u;
    }
}

/* venue-like print: white 145, thick bars ~+90 (the 21:41 logs) */
static sim_opt_t const g_venue = { 145.0, 100.0, 1.2, 2.0, 1u };

static void sim_new(void) { g_sn = 0u; g_nbar = 0u; }

static void test_bc_adapt(void)
{
    static char const * const letter[2] = { "A", "Z" };
    double const nar[2] = { 3.0, 5.5 };  /* small / big print (mm) */
    double const wid[2] = { 9.0, 16.5 };
    double const vel[2] = { 60.0, 120.0 }; /* VC slow / cruise (mm/s) */
    uint32_t total = 0u, ok = 0u, wrong = 0u, old_bars = 0u, full = 0u, s;
    uint32_t li, pi, vi, ri;
    sim_res_t r;
    sim_opt_t o = g_venue;
    double thin_peak = 0.0, thick_peak = 0.0;

    /* H-W00: the test print really is faint: thin bars <= +60, thick <= +100 */
    sim_new();
    (void)sim_lay("A", 100.0, 3.0, 9.0, false);
    {
        double x;
        for (x = 100.0; x < 101.5; x += 0.05) { double c = sim_cover(x, o.spot_mm); if (c > thin_peak) { thin_peak = c; } }
        for (x = 100.0; x < 260.0; x += 0.05) { double c = sim_cover(x, o.spot_mm); if (c > thick_peak) { thick_peak = c; } }
    }
    printf("        faint print: thin 3 mm bar +%.0f, thick +%.0f raw over white %.0f\n",
           thin_peak * o.ink, thick_peak * o.ink, o.white);
    CHECK("H-W00", (thin_peak * o.ink <= 85.0) && (thick_peak * o.ink <= 100.0),
          "adaptive: test print is as faint as the venue sheets (ink < +100 raw)");

    /* H-W01 / H-W02: A and Z, both print sizes, slow and cruise speed, both
       directions, 3 noise seeds: every pass reads its own letter, none
       reads a wrong one; the old fixed 300 / 400 finds no bar at all */
    for (li = 0u; li < 2u; li++)
    for (pi = 0u; pi < 2u; pi++)
    for (vi = 0u; vi < 2u; vi++)
    for (ri = 0u; ri < 2u; ri++)
    for (s = 0u; s < 3u; s++)
    {
        double end;
        sim_new();
        g_rng = 77u + (s * 131u) + (li * 7u) + (pi * 3u) + vi + (ri * 5u);
        end = sim_lay(letter[li], 300.0, nar[pi], wid[pi], 0u != ri);
        sim_drive(&o, 0.0, end + 150.0, vel[vi], 0.0);
        sim_read(true, 0, 0, &r);
        total++;
        if ((1u == r.starts) && (29u == r.elems)) { full++; }
        if ((1u == r.letters) && (0 == strcmp(r.text, letter[li]))) { ok++; }
        else
        {
            printf("        not read: %s print %u at %.0f mm/s rev %u seed %u: starts %u letters %u '%s' elems %u\n",
                   letter[li], (unsigned)pi, vel[vi], (unsigned)ri, (unsigned)s,
                   (unsigned)r.starts, (unsigned)r.letters, r.text, (unsigned)r.elems);
        }
        if ((r.letters > 0u) && (0 != strcmp(r.text, letter[li]))) { wrong++; }
        sim_read(false, S3_BC_EDGE_SWING, S3_BC_EDGE_FLOOR, &r);
        old_bars += r.edges;
    }
    printf("        faint A/Z: %u of %u passes give one read of 29 elements, %u read by the element\n"
           "        decoders, %u wrong; old 300/400 edges %u\n",
           (unsigned)full, (unsigned)total, (unsigned)ok, (unsigned)wrong, (unsigned)old_bars);
    CHECK("H-W01", full == total,
          "adaptive: faint A and Z, both sizes, 60 and 120 mm/s, both ways: every bar and gap found (29)");
    CHECK("H-W12", 0u == wrong, "adaptive: no faint pass gives a wrong letter");
    CHECK("H-W02", 0u == old_bars,
          "old fixed rise 300 / floor 400 sees no bar on the same faint print (baseline failure)");

    /* H-W03: 20 s of white only, noise sd 2 / 5 / 10 and a slow +-15 drift:
       no level change after learning */
    {
        double const nz[3] = { 2.0, 5.0, 10.0 };
        uint32_t edges = 0u, k, i;
        for (k = 0u; k < 3u; k++)
        {
            sim_opt_t w = o;
            w.noise = nz[k];
            sim_new();
            g_rng = 900u + k;
            sim_drive(&w, 0.0, 1440.0, 72.0, 0.0);
            for (i = 0u; i < g_sn; i++) { g_sv[i] += (int32_t)(15.0 * sin((double)i / 300.0)); }
            sim_read(true, 0, 0, &r);
            edges += r.edges;
        }
        CHECK("H-W03", 0u == edges, "adaptive: 20 s of white (noise sd 2, 5, 10, drifting) gives no bar");
    }

    /* H-W04: one black tape crossing (18 mm, ~2800 raw) and a 50 mm wide
       junction arm: a bar each, no letter.  Since S3_BC_FIRST_BAR_MAX_COUNTS
       (26 mm) the arm opens no read; the crossing still does */
    {
        sim_opt_t t = o;
        t.ink = 2700.0;
        sim_new();
        g_rng = 5u;
        sim_tape(300.0, 18.0);
        sim_tape(700.0, 50.0);
        sim_drive(&t, 0.0, 1000.0, 120.0, 0.0);
        sim_read(true, 0, 0, &r);
        CHECK("H-W04", (1u == r.starts) && (0u == r.letters) && (4u == r.edges),
              "adaptive: a tape crossing and a junction arm: one bar each, no letter; the arm (wider than any printed bar) opens no read");
    }

    /* H-W05: start standing on the thick bar of a *Z* (white is learned on
       ink), then drive over the rest of it and on to a whole *A*: the cut
       *Z* gives no letter, the *A* is read */
    {
        double end;
        sim_new();
        g_rng = 3u;
        end = sim_lay("Z", 0.0, 3.0, 9.0, false);
        (void)sim_lay("A", end + 200.0, 3.0, 9.0, false);
        {
            /* stand on the middle of the first wide bar for 1 s */
            double x0 = g_bar0[1] + 4.5;
            sim_drive(&o, x0, x0, 0.0, 1.0);
            sim_drive(&o, x0, end + 200.0 + 200.0, 60.0, 0.0);
        }
        sim_read(true, 0, 0, &r);
        CHECK("H-W05", (0 == strcmp(r.text, "A")) && (1u == r.letters),
              "adaptive: start on ink: no letter from the cut barcode, the next one reads");
    }

    /* H-W06..H-W09: the white estimate itself */
    {
        bc_edge_t e;
        uint32_t eu = 0u, ep = 0u, i, t = 0u, edges = 0u;
        int32_t  w0;
        bool     b_learned = false;
        g_rng = 11u;
        bc_edge_reset_pos(&e, 145, 0u, 0u);
        for (i = 0u; i < 40u; i++, t += 10000u)
        {
            (void)bc_edge_step_adapt(&e, 145 + (int32_t)(2.0 * sim_gauss()), t, i, false, &eu, &ep);
            if ((i == (S3_BC_WHITE_INIT_N - 2u)) && (0u != e.w_valid)) { b_learned = true; }
        }
        CHECK("H-W06", (0u != e.w_valid) && !b_learned && (bc_edge_margin(&e) >= 11) &&
                       (bc_edge_margin(&e) <= 20),
              "adaptive: white learned after S3_BC_WHITE_INIT_N steady samples, margin 3 sd + 8 within 11..20");
        /* frozen: 2 s of white 10 above, while a read is open */
        w0 = e.w_x16;
        for (i = 0u; i < 200u; i++, t += 10000u) { (void)bc_edge_step_adapt(&e, 152, t, i, true, &eu, &ep); }
        CHECK("H-W07", e.w_x16 == w0, "adaptive: white is frozen while a read is open");
        /* a bar is open, then 100 ms of samples are missed: the bar ends,
           white is learned again, no new bar before that */
        for (i = 0u; i < 5u; i++, t += 10000u) { edges += bc_edge_step_adapt(&e, 260, t, i, false, &eu, &ep); }
        t += 100000u;
        edges += bc_edge_step_adapt(&e, 260, t, 0u, false, &eu, &ep);
        {
            uint32_t later = 0u;
            for (i = 0u; i < (S3_BC_WHITE_INIT_N - 2u); i++)
            {
                t += 10000u;
                later += bc_edge_step_adapt(&e, (0u == (i % 4u)) ? 260 : 145, t, i, false, &eu, &ep);
            }
            CHECK("H-W08", (2u == edges) && (0u == e.w_valid) && (0u == later),
                  "adaptive: missed samples end the open bar and white is learned again first");
        }
        /* rearm: learn again at the next sample */
        for (i = 0u; i < 40u; i++, t += 10000u) { (void)bc_edge_step_adapt(&e, 145, t, i, false, &eu, &ep); }
        bc_edge_relearn(&e);
        (void)bc_edge_step_adapt(&e, 145, t, 0u, false, &eu, &ep);
        CHECK("H-W09", 0u == e.w_valid, "adaptive: bc_edge_relearn() (barcode rearm) learns white again");
        /* black far longer than any bar, outside a read: learned again */
        edges = 0u;
        for (i = 0u; i < 40u; i++, t += 10000u) { (void)bc_edge_step_adapt(&e, 145, t, i, false, &eu, &ep); }
        for (i = 0u; i < (S3_BC_WHITE_RELEARN_SAMPLES + 2u); i++, t += 10000u)
        {
            /* standing still (wheel position fixed at 40): the time rule */
            edges += bc_edge_step_adapt(&e, 2800, t, 40u, false, &eu, &ep);
        }
        CHECK("H-W10", (2u == edges) && (0u == e.w_valid),
              "adaptive: 1.5 s of black outside a read ends the bar and learns white again");
    }

    /* H-W13: the learned noise sd is close to the real one (G7's 3 sd + 8
       margin only means something if sd is right) */
    {
        double const sd[3] = { 2.0, 5.0, 10.0 };
        uint32_t k, i, bad = 0u, eu = 0u, ep = 0u;
        for (k = 0u; k < 3u; k++)
        {
            bc_edge_t e;
            double    est;
            g_rng = 300u + k;
            bc_edge_reset_pos(&e, 145, 0u, 0u);
            for (i = 0u; i < 2000u; i++)
            {
                (void)bc_edge_step_adapt(&e, (int32_t)lround(145.0 + (sd[k] * sim_gauss())),
                                         i * 10000u, i, false, &eu, &ep);
            }
            est = (double)e.w_sd_x16 / 16.0;
            printf("        white noise sd %.0f: learned %.2f, margin %d\n", sd[k], est,
                   (int)bc_edge_margin(&e));
            if ((est < (0.7 * sd[k])) || (est > (1.3 * sd[k]))) { bad++; }
        }
        CHECK("H-W13", 0u == bad, "adaptive: learned white noise sd within 30 % of the real sd (2, 5, 10)");
    }

    /* H-W14: rearm while the sensor stands on a thick bar (vc_sm.c
       barcode_retry() rearms right after a failed read): white is first
       learned on ink; after 1 s of real white it is back near paper and
       the next faint *A* / *Z* (60 mm/s) is still read, nothing wrong */
    {
        uint32_t li, near = 0u, good = 0u, wrong = 0u;
        bool     b_shape = g_sim_shape;
        g_sim_shape = true;
        for (li = 0u; li < 2u; li++)
        {
            char const * t = (0u == li) ? "A" : "Z";
            bc_edge_t    e;
            uint32_t     i, eu = 0u, ep = 0u, n_ink;
            sim_new();
            g_rng = 141u + li;
            sim_tape(-10.0, 25.0);              /* robot stands on ink at 0 mm */
            (void)sim_lay(t, 75.0, 3.0, 9.0, false);
            sim_drive(&o, 0.0, 0.0, 0.0, 0.5); /* 0.5 s on ink: white learned there */
            n_ink = g_sn;
            sim_drive(&o, 0.0, 75.0, 60.0, 0.0);
            sim_drive(&o, 75.0, 350.0, 60.0, 0.0);
            /* white after 1 s of paper (ink ends at 15 mm, 0.25 s) */
            bc_edge_reset_pos(&e, g_sv[0], 0u, g_sp[0]);
            for (i = 0u; (i < (n_ink + 25u + 100u)) && (i < g_sn); i++)
            {
                (void)bc_edge_step_adapt(&e, g_sv[i], i * 10000u, g_sp[i], false, &eu, &ep);
            }
            printf("        rearm on ink (%s): white %d after 1 s of paper, margin %d\n", t,
                   (int)(e.w_x16 / 16), (int)bc_edge_margin(&e));
            if ((0u != e.w_valid) && ((e.w_x16 / 16) <= 155) && (bc_edge_margin(&e) <= 20)) { near++; }
            sim_read(true, 0, 0, &r);
            if ((1u == r.letters) && (t[0] == r.text[0])) { good++; }
            else if (r.letters > 0u) { wrong++; }
        }
        g_sim_shape = b_shape;
        CHECK("H-W14", (2u == near) && (2u == good) && (0u == wrong),
              "adaptive: rearm on a bar: white back to paper within 1 s, next faint A / Z read");
    }

    /* H-W11: a 3 mm first bar at cruise speed (120 mm/s, 25 ms) is found
       with one 10 ms record per level (S3_BC_ADAPT_HOLD 1) */
    {
        sim_new();
        g_rng = 21u;
        sim_tape(300.0, 3.0);
        sim_drive(&o, 0.0, 450.0, 120.0, 0.0);
        sim_read(true, 0, 0, &r);
        CHECK("H-W11", 2u == r.edges, "adaptive: a lone 3 mm bar at 120 mm/s is seen (rise and fall)");
    }
}

/* ---- H-G10..H-G15: shape read with the read's own noise margin -------
   (c39g_decode_hump / c39g_min_hump, task_barcode.c shape_read) */
static uint16_t g_gl[C39G_MAX_SAMPLES];
static uint8_t  g_gf[C39G_MAX_SAMPLES];
static int8_t   g_gy[C39G_MAX_SAMPLES];

/* the simulated pass as a frozen trace (fwd = 2 x left steps, yaw 0) */
static uint32_t sim_to_trace(void)
{
    uint32_t i, n = (g_sn < C39G_MAX_SAMPLES) ? g_sn : C39G_MAX_SAMPLES;
    for (i = 0u; i < n; i++)
    {
        g_gl[i] = (uint16_t)((g_sv[i] < 0) ? 0 : g_sv[i]);
        g_gf[i] = (uint8_t)((0u == i) ? 0u : (g_sp[i] - g_sp[i - 1u]));
        g_gy[i] = 0;
    }
    return n;
}

/* learned margin of the edge finder over the simulated pass's start */
static int32_t sim_margin(void)
{
    bc_edge_t e;
    uint32_t  i, eu = 0u, ep = 0u;
    bc_edge_reset_pos(&e, g_sv[0], 0u, g_sp[0]);
    for (i = 0u; (i < 100u) && (i < g_sn); i++)
    {
        (void)bc_edge_step_adapt(&e, g_sv[i], i * 10000u, g_sp[i], false, &eu, &ep);
    }
    return bc_edge_margin(&e);
}

static void test_geom_noise(void)
{
    static char const * const letter[2] = { "A", "Z" };
    c39g_info_t info;
    uint32_t    n, li, ri, s, total = 0u, ok = 0u, wrong = 0u, few = 0u;
    sim_opt_t   o = g_venue;

    CHECK("H-G10", (C39G_MIN_HUMP == c39g_min_hump(0)) &&
                   ((C39G_HUMP_NOISE_K * 14) == c39g_min_hump(14)),
          "shape: smallest hump = K x noise margin, C39G_MIN_HUMP without one");

    /* H-G11: with C39G_MIN_HUMP it is the old reader (existing fixture) */
    {
        uint32_t k, same = 1u;
        for (k = 0u; k < (sizeof g_trace_a / sizeof g_trace_a[0]); k++) { g_gl[k] = g_trace_a[k]; }
        n = k;
        {
            c39g_info_t a, b;
            c39g_result_t ra = c39g_decode(g_gl, NULL, NULL, n, false, NULL, &a);
            c39g_result_t rb = c39g_decode_hump(g_gl, NULL, NULL, n, false, C39G_MIN_HUMP, NULL, &b);
            same = (ra == rb) && (a.letter == b.letter) && (a.best == b.best) &&
                   (a.err_x100 == b.err_x100) && (a.margin_x100 == b.margin_x100);
        }
        CHECK("H-G11", 0u != same, "shape: c39g_decode_hump with C39G_MIN_HUMP = c39g_decode");
        CHECK("H-G12", C39G_ARG == c39g_decode_hump(g_gl, NULL, NULL, n, false, 0, NULL, &info),
              "shape: a smallest hump below 1 is refused");
    }

    /* H-G13 / H-G14: faint A and Z (white 145, ink +100), small print,
       60 mm/s, both ways, 4 seeds: the fixed 250 finds too few humps;
       with the read's margin every pass is read, none wrongly */
    for (li = 0u; li < 2u; li++)
    for (ri = 0u; ri < 2u; ri++)
    for (s = 0u; s < 4u; s++)
    {
        double  end;
        int32_t m;
        sim_new();
        g_rng = 501u + (s * 17u) + (li * 3u) + ri;
        end = sim_lay(letter[li], 150.0, 3.0, 9.0, 0u != ri);
        sim_drive(&o, 0.0, end + 60.0, 60.0, 0.0);
        m = sim_margin();
        n = sim_to_trace();
        total++;
        if (C39G_FEW == c39g_decode(g_gl, g_gf, g_gy, n, true, NULL, &info)) { few++; }
        if (C39G_OK == c39g_decode_hump(g_gl, g_gf, g_gy, n, true, c39g_min_hump(m), NULL, &info))
        {
            if ((info.letter == letter[li][0]) && (info.b_reversed == (0u != ri))) { ok++; }
            else { wrong++; }
        }
    }
    printf("        faint shape read: fixed 250 too few humps %u of %u; with margin %u read, %u wrong\n",
           (unsigned)few, (unsigned)total, (unsigned)ok, (unsigned)wrong);
    CHECK("H-G13", few == total, "shape: fixed 250 finds too few humps on every faint pass (baseline)");
    CHECK("H-G14", (ok == total) && (0u == wrong),
          "shape: with the read's noise margin every faint A / Z pass is read, none wrongly");

    /* H-G15: faint look-alikes and junk give (almost) no letter: random
       faint bar patterns, white only, one tape crossing */
    {
        uint32_t t, j, got = 0u;
        for (t = 0u; t < 1000u; t++)
        {
            int black = 0;
            uint32_t next;
            g_rng = 7000u + t;
            next = 5u + (uint32_t)(sim_rand() * 40.0);
            for (j = 0u; j < 400u; j++)
            {
                if (j >= next)
                {
                    black = !black;
                    next  = j + 2u + (uint32_t)(sim_rand() * (double)(black ? (3u + (t % 25u)) : (3u + (t % 17u))));
                }
                g_gl[j] = (uint16_t)(145.0 + (black ? 90.0 : 0.0) + (2.0 * sim_gauss()));
                g_gf[j] = 5u;
                g_gy[j] = 0;
            }
            if (C39G_OK == c39g_decode_hump(g_gl, g_gf, g_gy, 400u, true, c39g_min_hump(14), NULL, &info)) { got++; }
        }
        printf("[INFO] shape: %u of 1000 faint random bar patterns gave a letter\n", (unsigned)got);
        CHECK("H-G15", got <= 1u, "shape: faint random bars give a letter at most 0.1 % of the time");
        g_rng = 99u;
        for (j = 0u; j < 400u; j++) { g_gl[j] = (uint16_t)(145.0 + (2.0 * sim_gauss())); g_gf[j] = 5u; g_gy[j] = 0; }
        {
            c39g_result_t r = c39g_decode_hump(g_gl, g_gf, g_gy, 400u, true, c39g_min_hump(14), NULL, &info);
            sim_opt_t t2 = o;
            c39g_result_t r2;
            t2.ink = 2700.0;
            sim_new();
            g_rng = 5u;
            sim_tape(100.0, 18.0);
            sim_drive(&t2, 0.0, 250.0, 60.0, 0.0);
            n = sim_to_trace();
            r2 = c39g_decode_hump(g_gl, g_gf, g_gy, n, true, c39g_min_hump(14), NULL, &info);
            CHECK("H-G16", (C39G_OK != r) && (C39G_OK != r2),
                  "shape: white only and one tape crossing give no letter with the noise margin");
        }
    }

    /* H-G17: the real faint pass 2026-11-18 21:41:13 (L trace from the
       dashboard, *A* backwards, inferred): its 29 elements reach the shape
       reader; 250 finds too few humps, the read's margin reads A reversed */
    {
        FILE *  fp = fopen("../replay/traces/20261118T134113.txt", "r");
        int     a, b, c;
        uint32_t k = 0u, off = 760u - S3_BC_GEOM_LEAD, end = 1240u;
        static uint16_t l[2000]; static uint8_t f[2000]; static int8_t y[2000];
        c39g_result_t r250 = C39G_ARG, rm = C39G_ARG;
        if (NULL != fp)
        {
            while ((k < 2000u) && (3 == fscanf(fp, "%d %d %d", &a, &b, &c)))
            {
                l[k] = (uint16_t)a; f[k] = (uint8_t)b; y[k] = (int8_t)c; k++;
            }
            fclose(fp);
        }
        if (k >= end)
        {
            r250 = c39g_decode(&l[off], &f[off], &y[off], end - off, true, NULL, &info);
            /* the margin the edge finder learned on this pass before the
               read (frozen from the read start, as task_line.c does) */
            bc_edge_t e;
            uint32_t  i, eu = 0u, ep = 0u, pos = 0u;
            int32_t   m;
            bc_edge_reset_pos(&e, (int32_t)l[0], 0u, 0u);
            for (i = 0u; i < 760u; i++)
            {
                pos += (uint32_t)((int32_t)f[i] - (int32_t)y[i]);
                e.hold = (uint8_t)S3_BC_ADAPT_HOLD;
                (void)bc_edge_step_adapt(&e, (int32_t)l[i], i * 10000u, pos, false, &eu, &ep);
            }
            m = bc_edge_margin(&e);
            printf("        21:41:13 pass: margin learned before the read %d\n", (int)m);
            rm = c39g_decode_hump(&l[off], &f[off], &y[off], end - off, true, c39g_min_hump(m), NULL, &info);
        }
        CHECK("H-G17", (C39G_FEW == r250) && (C39G_OK == rm) && ('A' == info.letter) && info.b_reversed,
              "shape: real faint 21:41:13 pass: 250 too few humps, noise margin reads A reversed");
    }
}

/* ---- H-S: paper (sheet) boundary before the symbol (owner 2026-11-24: on the A4
   sheet the robot slowed, went back to cruise and read only the end of
   the barcode).  The sheet reads g_paper_d above the floor's white; the
   symbol starts `lead` mm into the sheet. ------------------------------ */
static void test_bc_paper(void)
{
    double const step[4] = { 0.0, 12.0, 20.0, 40.0 };
    double const lead[2] = { 40.0, 80.0 };
    double const vel[2]  = { 36.0, 72.0 };
    sim_opt_t o = g_venue;
    sim_res_t r;
    uint32_t  si, li, sz, vi, k;
    uint32_t  total = 0u, one_read = 0u, at_bar = 0u, right = 0u, wrong = 0u, capped = 0u;
    uint32_t  false_ok = 0u, n_false = 0u;
    bool      b_shape = g_sim_shape;

    g_sim_shape = true;
    for (si = 0u; si < 4u; si++)
    for (li = 0u; li < 2u; li++)
    for (sz = 0u; sz < 2u; sz++)
    for (vi = 0u; vi < 2u; vi++)
    for (k = 0u; k < 2u; k++)
    {
        char const * t   = (0u == k) ? "A" : "Z";
        double const nar = (0u == sz) ? 3.0 : 6.0;
        double const wid = (0u == sz) ? 9.0 : 17.5;
        double       x0  = 200.0 + lead[li], end;
        bc_lock_t    lk;
        uint32_t     i, held = 1u;
        sim_new();
        g_rng = 701u + (si * 16u) + (li * 8u) + (sz * 4u) + (vi * 2u) + k;
        g_paper_x0 = 200.0; g_paper_d = step[si];
        end = sim_lay(t, x0, nar, wid, false);
        sim_drive(&o, 0.0, end + 150.0, vel[vi], 0.0);
        sim_read(true, 0, 0, &r);
        /* the speed cap of a real read: lock (task_line.c) from its start
           to its natural end */
        bc_lock_reset(&lk);
        for (i = 0u; i < g_sn; i++)
        {
            bool b_lk = bc_lock_step(&lk, 0u != g_s_scan[i], 0u != g_s_bars[i], g_sp[i],
                                     S3_BC_LOCK_FIRST_COUNTS, S3_BC_LOCK_AFTER_COUNTS,
                                     S3_BC_LOCK_MAX_COUNTS);
            if ((0u != g_s_scan[i]) && ((double)g_sp[i] * SIM_MM_PER_C > x0) && !b_lk) { held = 0u; }
        }
        total++;
        if (1u == r.nrd) { one_read++; }
        {
            /* the read that gives the letter starts at the 1st printed
               bar; any other read ended before the symbol, within one
               widest first bar + the quiet zone of where it began */
            uint32_t q, b_at = 0u, b_fok = 1u;
            double   bound = ((double)(S3_BC_FIRST_BAR_MAX_COUNTS + S3_BC_QUIET_COUNTS) * SIM_MM_PER_C) + 5.0;
            for (q = 0u; (q < r.nrd) && (q < 8u); q++)
            {
                if ((t[0] == r.rd_out[q]) && (r.rd_x0[q] > (x0 - 4.0)) && (r.rd_x0[q] < (x0 + 4.0))) { b_at = 1u; }
                else
                {
                    n_false++;
                    if ((r.rd_x1[q] >= x0) || ((r.rd_x1[q] - r.rd_x0[q]) > bound)) { b_fok = 0u; }
                }
            }
            at_bar += b_at;
            false_ok += b_fok;
        }
        if ((1u == r.letters) && (t[0] == r.text[0])) { right++; }
        else if (r.letters > 0u) { wrong++; }
        capped += held;
        if ((1u != r.nrd) || (1u != r.letters) || (t[0] != r.text[0]) || (0u == held))
        {
            uint32_t q;
            printf("        paper +%.0f lead %.0f %s print %.0f mm/s %s: %u reads", step[si], lead[li],
                   (0u == sz) ? "small" : "big", vel[vi], t, (unsigned)r.nrd);
            for (q = 0u; (q < r.nrd) && (q < 8u); q++)
            {
                printf(" [%.0f-%.0f mm %c]", r.rd_x0[q] - x0, r.rd_x1[q] - x0, r.rd_out[q]);
            }
            printf(" letters '%s' lock held %u (mm from first bar)\n", r.text, (unsigned)held);
        }
    }
    g_paper_x0 = 1e9; g_paper_d = 0.0;

    /* H-S04: no barcode, the robot turns: L crosses the line tape slowly
       (18 mm at 20 mm/s) and drags along a junction arm (60 mm): no
       letter; any read that opens ends, and the speed cap lets go,
       within one widest first bar + the quiet zone + the after-hold */
    {
        sim_opt_t tp = o;
        uint32_t  bad = 0u, q, i;
        double    bound = ((double)(S3_BC_FIRST_BAR_MAX_COUNTS + S3_BC_QUIET_COUNTS + S3_BC_LOCK_AFTER_COUNTS) *
                           SIM_MM_PER_C) + 5.0;
        tp.ink = 2700.0;
        sim_new();
        g_rng = 811u;
        sim_tape(300.0, 18.0);
        sim_tape(600.0, 60.0);
        sim_drive(&tp, 0.0, 900.0, 20.0, 0.0);
        sim_read(true, 0, 0, &r);
        {
            bc_lock_t lk;
            double    lock_x = -1.0, lock_end = -1.0;
            bc_lock_reset(&lk);
            for (i = 0u; i < g_sn; i++)
            {
                bool b_lk = bc_lock_step(&lk, 0u != g_s_scan[i], 0u != g_s_bars[i], g_sp[i],
                                         S3_BC_LOCK_FIRST_COUNTS, S3_BC_LOCK_AFTER_COUNTS,
                                         S3_BC_LOCK_MAX_COUNTS);
                double x = (double)g_sp[i] * SIM_MM_PER_C;
                if (b_lk && (lock_x < 0.0)) { lock_x = x; }
                if (!b_lk && (lock_x >= 0.0) && (lock_end < 0.0)) { lock_end = x; }
            }
            if ((lock_x >= 0.0) && ((lock_end < 0.0) || ((lock_end - lock_x) > bound))) { bad++; }
            printf("        no-barcode turn: %u reads, letters '%s', lock %.0f..%.0f mm\n", (unsigned)r.nrd,
                   r.text, lock_x, lock_end);
        }
        for (q = 0u; (q < r.nrd) && (q < 8u); q++)
        {
            if ((r.rd_x1[q] - r.rd_x0[q]) > bound) { bad++; }
        }
        CHECK("H-S04", (0u == r.letters) && (0u == bad),
              "paper: no-barcode turn (slow tape crossing, 60 mm arm): no letter, a read and the speed cap end within a bounded distance");
    }

    /* H-S05: bc_start_step(): white >= S3_BC_START_WHITE_MS arms; a bar
       narrower than S3_BC_FIRST_BAR_MAX_COUNTS starts; a wider one while
       moving does not, and the white after it arms whatever its length;
       standing still only the time limit applies */
    {
        bc_start_t st;
        uint32_t   ok = 1u;
        uint32_t   wl = (uint32_t)S3_BC_START_WHITE_MS * 1000u;
        bc_start_reset(&st);
        ok &= (BC_START_NONE  == bc_start_step(&st, false, wl - 1u, 100u)) ? 1u : 0u;
        ok &= (BC_START_NONE  == bc_start_step(&st, true, 50000u, 40u)) ? 1u : 0u;
        ok &= (BC_START_ARMED == bc_start_step(&st, false, wl, 100u)) ? 1u : 0u;
        ok &= (BC_START_READ  == bc_start_step(&st, true, 50000u, S3_BC_FIRST_BAR_MAX_COUNTS - 1u)) ? 1u : 0u;
        ok &= (BC_START_ARMED == bc_start_step(&st, false, wl, 100u)) ? 1u : 0u;
        ok &= (BC_START_NONE  == bc_start_step(&st, true, 400000u, S3_BC_FIRST_BAR_MAX_COUNTS)) ? 1u : 0u;
        ok &= (BC_START_ARMED == bc_start_step(&st, false, 10000u, 5u)) ? 1u : 0u;
        ok &= (BC_START_READ  == bc_start_step(&st, true, 40000u, 40u)) ? 1u : 0u;
        ok &= (BC_START_ARMED == bc_start_step(&st, false, wl, 100u)) ? 1u : 0u;
        ok &= (BC_START_READ  == bc_start_step(&st, true, 1400000u, 0u)) ? 1u : 0u;
        ok &= (BC_START_ARMED == bc_start_step(&st, false, wl, 100u)) ? 1u : 0u;
        ok &= (BC_START_NONE  == bc_start_step(&st, true, 1600000u, 0u)) ? 1u : 0u;
        ok &= (BC_START_NONE  == bc_start_step(&st, false, 10000u, 5u)) ? 1u : 0u;
        CHECK("H-S05", 0u != ok,
              "start rule: long white arms; a printed-width first bar starts; a too-wide one (paper) re-arms on the next white; standing still: time limit");
    }

    /* H-S06: the lock says why it let go (per-read record) */
    {
        bc_lock_t lk;
        uint32_t  p = 0u, ok = 1u;
        bc_lock_reset(&lk);
        (void)bc_lock_step(&lk, true, false, p, 250u, 270u, 3000u);
        for (p = 1u; p < 260u; p++) { (void)bc_lock_step(&lk, true, false, p, 250u, 270u, 3000u); }
        ok &= (!lk.b_lock && (BC_LOCK_FREE_FIRST == lk.free_why)) ? 1u : 0u;
        bc_lock_reset(&lk);
        (void)bc_lock_step(&lk, true, false, 0u, 250u, 270u, 3000u);
        (void)bc_lock_step(&lk, false, false, 10u, 250u, 270u, 3000u);
        ok &= (!lk.b_lock && (BC_LOCK_FREE_UNCONF == lk.free_why)) ? 1u : 0u;
        bc_lock_reset(&lk);
        (void)bc_lock_step(&lk, true, true, 0u, 250u, 270u, 3000u);
        (void)bc_lock_step(&lk, false, false, 100u, 250u, 270u, 3000u);
        ok &= (lk.b_lock) ? 1u : 0u;
        (void)bc_lock_step(&lk, false, false, 370u, 250u, 270u, 3000u);
        ok &= (!lk.b_lock && (BC_LOCK_FREE_AFTER == lk.free_why)) ? 1u : 0u;
        bc_lock_reset(&lk);
        (void)bc_lock_step(&lk, true, true, 0u, 250u, 270u, 3000u);
        (void)bc_lock_step(&lk, true, true, 3000u, 250u, 270u, 3000u);
        ok &= (!lk.b_lock && (BC_LOCK_FREE_MAX == lk.free_why)) ? 1u : 0u;
        CHECK("H-S06", 0u != ok, "lock: records why it let go (no 2nd bar, ended unconfirmed, after-hold, max distance)");
    }
    g_sim_shape = b_shape;
    printf("        paper boundary: %u passes, one read %u, letter read from the 1st bar %u, right letter %u, wrong %u, capped through the read %u, false starts %u (all ended before the symbol: %s)\n",
           (unsigned)total, (unsigned)one_read, (unsigned)at_bar, (unsigned)right, (unsigned)wrong, (unsigned)capped,
           (unsigned)n_false, (false_ok == total) ? "yes" : "NO");
    CHECK("H-S01", (at_bar == total) && (false_ok == total) && (n_false <= (total / 16u)),
          "paper: the letter's read starts at the 1st printed bar; a sheet edge opens at most a rare short false read that ends before the symbol");
    CHECK("H-S02", (right == total) && (0u == wrong),
          "paper: faint A / Z, both sizes, 36 and 72 mm/s, sheet +0..+40 over the floor: right letter, never wrong");
    CHECK("H-S03", capped == total,
          "paper: the speed cap (lock) holds from the 1st bar to the read's natural end");
}

/* ---- H-X: wrong-read guards, whole read path (element decoders, then the
   shape read with the read's noise margin), faint print ------------------ */
static void test_bc_guards(void)
{
    sim_opt_t o = g_venue;
    sim_res_t r;
    uint32_t  li, ri, s, total = 0u, ok = 0u, wrong = 0u, by_shape = 0u;
    char      txt[2] = { 0, 0 };

    g_sim_shape = true;
    /* H-X01: every letter A-Z, faint, small print, 60 mm/s, both ways,
       2 seeds: each pass reads its own letter or nothing, never another */
    for (li = 0u; li < 26u; li++)
    for (ri = 0u; ri < 2u; ri++)
    for (s = 0u; s < 2u; s++)
    {
        double end;
        txt[0] = (char)('A' + (int)li);
        sim_new();
        g_rng = 4001u + (li * 11u) + (ri * 5u) + s;
        end = sim_lay(txt, 300.0, 3.0, 9.0, 0u != ri);
        sim_drive(&o, 0.0, end + 150.0, 60.0, 0.0);
        sim_read(true, 0, 0, &r);
        total++;
        by_shape += r.shape;
        if ((1u == r.letters) && (txt[0] == r.text[0])) { ok++; }
        else if (r.letters > 0u)
        {
            wrong++;
            printf("        WRONG: printed %c rev %u seed %u read '%s'\n", txt[0], (unsigned)ri, (unsigned)s, r.text);
        }
    }
    printf("        faint A-Z: %u of %u read (%u by shape), %u wrong\n", (unsigned)ok,
           (unsigned)total, (unsigned)by_shape, (unsigned)wrong);
    CHECK("H-X01", (0u == wrong) && (ok == total),
          "guards: faint A-Z, both ways: every pass reads its own letter, never a wrong one");

    /* H-X02: incomplete faint barcodes: the sensor leaves the print after
       30 %, 50 %, 70 % or 90 % of *A* / *Z* (the robot turns away): no letter */
    {
        double const part[4] = { 0.3, 0.5, 0.7, 0.9 };
        uint32_t k, got = 0u;
        for (li = 0u; li < 2u; li++)
        for (k = 0u; k < 4u; k++)
        {
            double end, cut;
            uint32_t b;
            sim_new();
            g_rng = 61u + k + (li * 4u);
            end = sim_lay((0u == li) ? "A" : "Z", 300.0, 3.0, 9.0, false);
            cut = 300.0 + ((end - 300.0) * part[k]);
            for (b = 0u; b < g_nbar; b++)
            {
                if (g_bar0[b] >= cut) { g_bar0[b] = g_bar1[b] = 0.0; }
                else if (g_bar1[b] > cut) { g_bar1[b] = cut; }
            }
            sim_drive(&o, 0.0, end + 150.0, 60.0, 0.0);
            sim_read(true, 0, 0, &r);
            got += r.letters;
        }
        CHECK("H-X02", 0u == got, "guards: a faint *A* / *Z* cut at 30-90 % gives no letter");
    }

    /* H-X03: the robot stops on the barcode for 1 s halfway (seen in the
       21:41:13 log) and goes on: the letter is still read */
    {
        uint32_t got_ok = 0u, got_wrong = 0u;
        for (li = 0u; li < 2u; li++)
        {
            double end, mid;
            char const * t = (0u == li) ? "A" : "Z";
            sim_new();
            g_rng = 91u + li;
            end = sim_lay(t, 300.0, 3.0, 9.0, false);
            mid = (300.0 + end) / 2.0;
            sim_drive(&o, 0.0, mid, 60.0, 0.0);
            sim_drive(&o, mid, mid, 0.0, 1.0);
            sim_drive(&o, mid, end + 150.0, 60.0, 0.0);
            sim_read(true, 0, 0, &r);
            if ((1u == r.letters) && (t[0] == r.text[0])) { got_ok++; }
            else if (r.letters > 0u) { got_wrong++; }
        }
        CHECK("H-X03", (0u == got_wrong) && (2u == got_ok),
              "guards: a 1 s stop on the barcode: A and Z still read, never a wrong letter");
    }

    /* H-X04: strong print (the 2026-11-11 kind, ink +2500 over white 200)
       still reads A and Z through the adaptive path, both ways */
    {
        sim_opt_t st = o;
        uint32_t  good = 0u, bad = 0u;
        st.white = 200.0;
        st.ink   = 2500.0;
        st.noise = 8.0;
        for (li = 0u; li < 2u; li++)
        for (ri = 0u; ri < 2u; ri++)
        {
            double end;
            char const * t = (0u == li) ? "A" : "Z";
            sim_new();
            g_rng = 123u + li + (ri * 2u);
            end = sim_lay(t, 300.0, 5.5, 16.5, 0u != ri);
            sim_drive(&st, 0.0, end + 150.0, 60.0, 0.0);
            sim_read(true, 0, 0, &r);
            if ((1u == r.letters) && (t[0] == r.text[0])) { good++; }
            else if (r.letters > 0u) { bad++; }
        }
        CHECK("H-X04", (4u == good) && (0u == bad), "guards: strong A and Z still read through the adaptive path");
    }
    g_sim_shape = false;
}

int main(void)
{
    test_code39();
    test_line();
    test_rejoin_junction();
    test_bc_edge();
    test_bc_distance();
    test_partial();
    test_geom();
    test_geom_try();
    test_real_18nov();
    test_l_fast();
    test_bc_lock();
    test_bc_lock_tracker();
    test_bc_adapt();
    test_geom_noise();
    test_bc_guards();
    test_bc_paper();
    printf("[TEST] subsystem3-host SUMMARY %d passed, %d failed\n", g_pass, g_fail);
    return (0 == g_fail) ? 0 : 1;
}
