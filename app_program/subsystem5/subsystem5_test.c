/*
 *      subsystem5_test.c
 *      On-target tests for subsystem 5.  Built only with: make TEST=1
 *      Record results in testing/subsystem5/TEST_LOG.md.  IDs: S5-U01, ...
 *
 *      U01-U09 test the pure algorithms with made-up scans, so they pass
 *      without any hardware connected.  U10-U11 need the sensor wired.
 */

#include "s5_local.h"
#include "drv_ultrasonic.h"
#include "../test_report.h"

#if APP_TEST

/* A box straight ahead at 200 mm, about 120 mm wide, open both sides */
static const s5_sample_t g_scan_centre[] = {
    { -60, 0 },  { -40, 0 },  { -20, 210 }, { -10, 200 }, { 0, 200 },
    { 10, 200 }, { 20, 215 }, { 40, 0 },    { 60, 0 }
};

/* Same box, but a wall close on the left */
static const s5_sample_t g_scan_wall_left[] = {
    { -60, 0 },  { -40, 0 },  { -20, 210 }, { -10, 200 }, { 0, 200 },
    { 10, 200 }, { 20, 215 }, { 40, 650 },  { 60, 220 }
};

/* Blocked everywhere */
static const s5_sample_t g_scan_blocked[] = {
    { -60, 230 }, { -30, 210 }, { 0, 200 }, { 30, 210 }, { 60, 230 }
};

/* Number of elements of an array.  A macro (BARR-C 6.3 deviation):
   sizeof must see the array itself, a function would see a pointer. */
#define S5_ARRAY_LEN(a) ((int32_t)(sizeof(a) / sizeof((a)[0])))

void subsystem5_test(void)
{
    s5_profile_t profile;
    s5_step_t    plan[S5_PLAN_MAX];
    s5_status_t  status;
    s5_sample_t  empty[3] = { { -30, 0 }, { 0, 0 }, { 30, 0 } };
    int32_t      count;
    uint16_t     dist_mm;
    bool         b_ok;

    TEST_INIT();

    TEST_CHECK("S5-U01",
               (500 == s5_sin1000(30)) && (-1000 == s5_sin1000(-90)) &&
                   (0 == s5_sin1000(0)),
               "sine table");

    b_ok =
        s5_build_profile(g_scan_centre, S5_ARRAY_LEN(g_scan_centre), &profile);
    TEST_CHECK("S5-U02",
               b_ok && (200 == profile.distance_mm) &&
                   (-20 == profile.right_edge_deg) &&
                   (20 == profile.left_edge_deg),
               "profile: distance and edges of a centred box");
    TEST_CHECK("S5-U03", (profile.width_mm > 100) && (profile.width_mm < 200),
               "profile: width of a centred box ~145 mm");

    TEST_CHECK("S5-U04",
               (!s5_build_profile(empty, 3, &profile)) && (0U == profile.valid),
               "profile: no echo anywhere = no obstacle");

    (void)s5_build_profile(g_scan_wall_left, S5_ARRAY_LEN(g_scan_wall_left),
                           &profile);
    TEST_CHECK("S5-U05", S5_SIDE_RIGHT == profile.bypass_side,
               "side: wall on the left -> bypass right");

    (void)s5_build_profile(g_scan_blocked, S5_ARRAY_LEN(g_scan_blocked),
                           &profile);
    TEST_CHECK("S5-U06", S5_SIDE_NONE == profile.bypass_side,
               "side: blocked both sides -> none");
    count = s5_plan_bypass(&profile, plan, S5_PLAN_MAX);
    TEST_CHECK("S5-U07", (2 == count) && (S5_MOVE_REVERSE == plan[1].m.kind),
               "plan: blocked -> stop and reverse only");

    (void)s5_build_profile(g_scan_wall_left, S5_ARRAY_LEN(g_scan_wall_left),
                           &profile);
    count = s5_plan_bypass(&profile, plan, S5_PLAN_MAX);
    TEST_CHECK("S5-U08",
               (7 == count) && (S5_MOVE_STOP == plan[0].m.kind) &&
                   (-S5_BYPASS_TURN_DEG == plan[1].m.angle_deg) &&
                   (S5_BYPASS_TURN_DEG == plan[5].m.angle_deg) &&
                   (1U == plan[6].search_line),
               "plan: right bypass = stop, R45, fwd, L45, fwd, L45, search");

    count = s5_plan_recovery(S5_SIDE_RIGHT, 2, plan, S5_PLAN_MAX);
    TEST_CHECK("S5-U09",
               (2 == count) && (0U != plan[1].search_line) &&
                   ((S5_RECOVERY_BASE_MM + (2 * S5_RECOVERY_STEP_MM)) ==
                    plan[1].m.distance_mm),
               "recovery: attempt 3 searches further");

    tk_dly_tsk(300); /* let the scan task run a few periods */
    TEST_CHECK("S5-U10",
               (E_OK == s5_get_status(&status)) && (status.pings > 0U),
               "scan task is pinging (status counts pings)");

    /* Needs the sensor: hold a flat object 20-30 cm in front */
    dist_mm = status.last_distance_mm;
    tm_printf((uint8_t *)"[TEST] S5-U11 info: last distance %d mm, %d pings\n",
              (int32_t)dist_mm, (int32_t)status.pings);
    TEST_CHECK("S5-U11",
               (US_NO_ECHO == dist_mm) ||
                   ((dist_mm > 20U) && (dist_mm < 2000U)),
               "last distance is 'no echo' or in sensor range");

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */

/*** end of file ***/
