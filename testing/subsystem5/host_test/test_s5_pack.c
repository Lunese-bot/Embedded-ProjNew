/*
 *  test_s5_pack.c -- PC test for the 16-bit telemetry word (s5_pack.c).
 *  Run: make
 */
#include <stdio.h>
#include "s5_pack.h"

static int g_pass = 0;
static int g_fail = 0;

static void check(const char * id, int ok, const char * what)
{
    printf("[TEST] %s %s  %s\n", id, ok ? "PASS" : "FAIL", what);
    if (ok) { g_pass++; } else { g_fail++; }
}

int main(void)
{
    uint16_t w, mm, all_ok = 1;
    uint8_t  st;
    bool     det;

    w = s5_pack_status(230U, 2U, true);
    s5_unpack_status(w, &mm, &st, &det);
    check("S5-H01", mm == 230U && st == 2U && det, "round trip 230 mm, state 2, det");
    check("S5-H02", w == (uint16_t)(0x8000U | (2U << 11) | 230U), "bit layout det|state|mm");

    w = s5_pack_status(0U, 0U, false);
    check("S5-H03", w == 0U, "no echo, IDLE, no obstacle = 0x0000");

    w = s5_pack_status(2000U, 9U, false);
    s5_unpack_status(w, &mm, &st, &det);
    check("S5-H04", mm == 2000U && st == 9U && !det, "max range 2000 mm, ERROR state");

    w = s5_pack_status(5000U, 1U, false);
    s5_unpack_status(w, &mm, NULL, NULL);
    check("S5-H05", mm == 2047U, "distance above 2047 saturates");

    w = s5_pack_status(100U, 0xFFU, false);
    s5_unpack_status(w, &mm, &st, &det);
    check("S5-H06", mm == 100U && st == 15U && !det, "bad state cannot spill into other fields");

    for (unsigned d = 0; d <= 2047U; d++)
    {
        for (unsigned s = 0; s < 16U; s++)
        {
            for (int b = 0; b < 2; b++)
            {
                s5_unpack_status(s5_pack_status((uint16_t)d, (uint8_t)s, b), &mm, &st, &det);
                if (mm != d || st != s || det != (bool)b) { all_ok = 0; }
            }
        }
    }
    check("S5-H07", all_ok, "all 65536 combinations round-trip exactly");
    check("S5-H08", sizeof(w) == 2U, "word is 2 bytes");

    printf("[TEST] subsystem5 host SUMMARY %d passed, %d failed\n", g_pass, g_fail);
    return g_fail != 0;
}
