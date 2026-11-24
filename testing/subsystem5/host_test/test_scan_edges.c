/* S5 edge estimate and the USB sample dump (obstacle_logic.c).
   Built three times (Makefile):
     test_scan_edges        the default S5_EDGE_TRIM_DEG (2): inward only.
     test_scan_edges_off    S5_EDGE_TRIM_DEG = 0: edges are the legacy
                            r * sin(edge angle), bit for bit.
     test_scan_edges_trim   S5_EDGE_TRIM_DEG = 10: each edge moves toward
                            the other, never outward; crossing edges meet
                            in the middle; reported angles stay raw.
   Both builds check s5_format_samples output exactly. */
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

static int checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); exit(1); } } while (0)

static int32_t legacy_edge(const s5_sample_t * s)
{
    return ((int32_t)s->mm * s5_sin1000(s->deg)) / 1000;
}

/* Find the sample that the profile used as an edge (by angle). */
static const s5_sample_t * at_deg(const s5_sample_t * s, int32_t n, int32_t deg)
{
    int32_t i;
    for (i = 0; i < n; i++) { if (s[i].deg == deg) { return &s[i]; } }
    return NULL;
}

static void check_set(const s5_sample_t * s, int32_t n)
{
    s5_profile_t p;
    const s5_sample_t * r;
    const s5_sample_t * l;
    int32_t legacy_w;

    CHECK(s5_build_profile(s, n, &p));
    r = at_deg(s, n, p.right_edge_deg);
    l = at_deg(s, n, p.left_edge_deg);
    CHECK((NULL != r) && (NULL != l));
    legacy_w = abs_i(legacy_edge(l) - legacy_edge(r));
#if S5_EDGE_TRIM_DEG == 0
    CHECK(p.right_mm == legacy_edge(r));                    /* S5-E01 */
    CHECK(p.left_mm == legacy_edge(l));
    CHECK(p.width_mm == legacy_w);
#else
    /* S5-E02: edges move inward, never outward */
    CHECK(p.right_mm >= legacy_edge(r));
    CHECK(p.left_mm <= legacy_edge(l));
    CHECK(p.left_mm >= p.right_mm);
    CHECK((int32_t)p.width_mm <= legacy_w);
    /* the bypass turn-out can only shrink, by the same edge */
    {
        s5_profile_t q = p;
        q.right_mm = (int16_t)legacy_edge(r);
        q.left_mm  = (int16_t)legacy_edge(l);
        CHECK(s5_lateral_needed_mm(&p, S5_SIDE_LEFT) <= s5_lateral_needed_mm(&q, S5_SIDE_LEFT));
        CHECK(s5_lateral_needed_mm(&p, S5_SIDE_RIGHT) <= s5_lateral_needed_mm(&q, S5_SIDE_RIGHT));
    }
#endif
}

int main(void)
{
    /* a: centred 150 mm face at 200 mm, 5 deg fine steps */
    static const s5_sample_t a[] = {
        {-60, 0}, {-45, 0}, {-40, 0}, {-35, 0}, {-30, 238}, {-25, 224},
        {-20, 214}, {-15, 207}, {-10, 203}, {-5, 201}, {0, 200}, {5, 201},
        {10, 203}, {15, 207}, {20, 214}, {25, 224}, {30, 236}, {35, 0},
        {40, 0}, {45, 0}, {60, 0}};
    /* b: offset to the left, far wall on the right outside the gap */
    static const s5_sample_t b[] = {
        {-60, 0}, {-30, 0}, {-15, 0}, {-10, 430}, {-5, 230}, {0, 225},
        {5, 226}, {10, 228}, {15, 233}, {20, 240}, {25, 250}, {30, 262},
        {35, 275}, {40, 0}, {45, 0}, {60, 0}};
    /* c: narrow (two samples): a 10 deg trim crosses the edges */
    static const s5_sample_t c[] = {
        {-30, 0}, {-5, 0}, {0, 180}, {5, 182}, {10, 0}, {30, 0}};
    /* d: single hit */
    static const s5_sample_t d[] = {{-30, 0}, {0, 190}, {30, 0}};
    /* e: all to the right of centre (both edges negative) */
    static const s5_sample_t e[] = {
        {-60, 0}, {-45, 260}, {-40, 245}, {-35, 236}, {-30, 232},
        {-25, 233}, {-20, 238}, {-15, 0}, {0, 0}, {30, 0}, {60, 0}};

    check_set(a, (int32_t)(sizeof a / sizeof a[0]));
    check_set(b, (int32_t)(sizeof b / sizeof b[0]));
    check_set(c, (int32_t)(sizeof c / sizeof c[0]));
    check_set(d, (int32_t)(sizeof d / sizeof d[0]));
    check_set(e, (int32_t)(sizeof e / sizeof e[0]));

    {
        s5_profile_t p;
        CHECK(s5_build_profile(a, (int32_t)(sizeof a / sizeof a[0]), &p));
        /* S5-E03: reported angles are raw in both builds */
        CHECK(p.right_edge_deg == -30);
        CHECK(p.left_edge_deg == 30);
        CHECK(p.centre_deg == 0);
#if S5_EDGE_TRIM_DEG == 2
        /* S5-E06: the shipped trim takes 10..20 mm off a 150 mm face
           scanned at 200 mm (legacy 237 mm here) */
        CHECK(p.width_mm <= 237 - 10);
        CHECK(p.width_mm >= 237 - 20);
#endif
#if S5_EDGE_TRIM_DEG == 10
        /* 238 * sin(-20) = -81, 236 * sin(20) = 80 */
        CHECK(p.right_mm == ((238 * s5_sin1000(-20)) / 1000));
        CHECK(p.left_mm == ((236 * s5_sin1000(20)) / 1000));
#endif
        CHECK(s5_build_profile(c, (int32_t)(sizeof c / sizeof c[0]), &p));
#if S5_EDGE_TRIM_DEG == 10
        /* S5-E04: crossing edges meet at the midpoint angle (2 deg) */
        CHECK(p.right_mm == ((180 * s5_sin1000(2)) / 1000));
        CHECK(p.left_mm == ((182 * s5_sin1000(2)) / 1000));
#endif
    }

    /* S5-E05: USB sample dump, exact text */
    {
        char buf[128];
        int32_t n;
        static const s5_sample_t f[] = {{-60, 0}, {-5, 1234}, {0, 7}, {45, 65535}};

        n = s5_format_samples(buf, (int32_t)sizeof buf, f, 0, 4);
        CHECK(0 == strcmp(buf, " -60:0 -5:1234 0:7 45:65535"));
        CHECK(n == (int32_t)strlen(buf));
        n = s5_format_samples(buf, (int32_t)sizeof buf, f, 1, 2);
        CHECK(0 == strcmp(buf, " -5:1234 0:7"));
        CHECK(n == 12);
        /* only whole pairs fit; always terminated, never past size */
        memset(buf, 'x', sizeof buf);
        n = s5_format_samples(buf, 15, f, 0, 4);
        CHECK(0 == strcmp(buf, " -60:0 -5:1234"));
        CHECK(n == 14);
        CHECK(buf[15] == 'x');
        memset(buf, 'x', sizeof buf);
        n = s5_format_samples(buf, 14, f, 0, 4);
        CHECK(0 == strcmp(buf, " -60:0"));
        CHECK(n == 6);
        n = s5_format_samples(buf, 1, f, 0, 4);
        CHECK((0 == n) && ('\0' == buf[0]));
        CHECK(0 == s5_format_samples(buf, 0, f, 0, 4));
        n = s5_format_samples(buf, (int32_t)sizeof buf, f, 0, 0);
        CHECK((0 == n) && ('\0' == buf[0]));
        n = s5_format_samples(buf, (int32_t)sizeof buf, NULL, 0, 4);
        CHECK((0 == n) && ('\0' == buf[0]));
    }

    printf("[TEST] S5-E01..E06 PASS  edge estimate (trim %d deg), sample dump  (%d checks)\n",
           S5_EDGE_TRIM_DEG, checks);
    return 0;
}
