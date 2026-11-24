/*
 *      test_obst_pub.c
 *      Host test of the obstacle profile publish decision (s1_obst_pub.c)
 *      with the real S5 record code (s5_obst_report.c).
 */

#include <stdio.h>
#include <string.h>

#include "../../../app_program/subsystem1/s1_obst_pub.h"

static int g_passed;
static int g_failed;

#define CHECK(id, condition, description) \
    do \
    { \
        if (condition) { g_passed++; printf("[TEST] %s PASS  %s\n", id, description); } \
        else { g_failed++; printf("[TEST] %s FAIL  %s (line %d)\n", id, description, __LINE__); } \
    } while (0)

/* One telemetry period: due? then "post" it (the queue accepts unless
   b_refuse).  Returns true when a frame went out. */
static bool poll(S1_OBST_PUB * p, const s5_obst_report_t * r, bool up, bool b_refuse)
{
    if (s1_obst_pub_due(p, r, up))
    {
        if (!b_refuse)
        {
            s1_obst_pub_sent(p, r);
            return true;
        }
    }
    return false;
}

int main(void)
{
    S1_OBST_PUB      pub;
    s5_obst_report_t rec;
    int              sent = 0;
    int              idx;

    s1_obst_pub_reset(&pub);
    rec = (s5_obst_report_t){ 0 };
    CHECK("S1-P01", !poll(&pub, &rec, true, false), "no encounter since boot (id 0): nothing published");

    s5_or_begin(&rec, 1, 5000U);
    CHECK("S1-P02", !poll(&pub, &rec, false, false), "MQTT down: nothing published");
    CHECK("S1-P02", poll(&pub, &rec, true, false), "MQTT up: the record goes out");
    for (idx = 0; idx < 10; idx++) { sent += poll(&pub, &rec, true, false) ? 1 : 0; }
    CHECK("S1-P03", 0 == sent, "unchanged record: published once, not every 200 ms");

    s5_or_front(&rec, 1, 200, -20, 20, 180, 1, -60, 60, true, S5_OR_R_NONE);
    CHECK("S1-P04", poll(&pub, &rec, true, true) == false, "queue short of room: refused");
    CHECK("S1-P04", poll(&pub, &rec, true, false), "refused record is due again next period");
    CHECK("S1-P04", !poll(&pub, &rec, true, false), "and then sent only once");

    CHECK("S1-P05", !poll(&pub, &rec, false, false), "link lost");
    CHECK("S1-P05", poll(&pub, &rec, true, false), "reconnect: the latest record is sent again");

    s5_or_length(&rec, S5_OR_L_MEASURED, 240, S5_OR_R_NONE);
    s5_or_final(&rec, S5_OR_R_NONE);
    CHECK("S1-P06", poll(&pub, &rec, true, false) && !poll(&pub, &rec, true, false),
          "two changes inside one period: only the newest goes out, once");

    s5_or_begin(&rec, 1, 900U); /* robot restart: id 1 again, new t0 */
    rec.rev = pub.rev;          /* even with the same rev */
    CHECK("S1-P07", poll(&pub, &rec, true, false), "restart (same id and rev, new t0): published as new");

    printf("[TEST] subsystem1 obstacle profile SUMMARY %d passed, %d failed\n", g_passed, g_failed);
    return (0 == g_failed) ? 0 : 1;
}

/*** end of file ***/
