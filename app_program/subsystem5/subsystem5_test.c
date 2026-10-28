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

/* Stand-in for s2_emergency_stop() during S5-U21 */
static int32_t g_test_stops;

static ER test_fake_stop(void)
{
    g_test_stops++;
    return E_OK;
}

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

    /* ---- 16-bit telemetry word (min-bit, s5_pack.h) ---- */
    {
        uint16_t word;
        uint16_t mm;
        uint8_t  st;
        bool     det;

        word = s5_pack_status(230U, (uint8_t)S5_ST_DETECTED, true);
        s5_unpack_status(word, &mm, &st, &det);
        TEST_CHECK("S5-U17",
                   (230U == mm) && ((uint8_t)S5_ST_DETECTED == st) && det,
                   "pack/unpack round trip: 230 mm, DETECTED, det=1");

        word = s5_pack_status(3000U, (uint8_t)S5_ST_ERROR, false);
        s5_unpack_status(word, &mm, &st, NULL);
        TEST_CHECK("S5-U18",
                   (S5_PACK_DIST_MAX_MM == mm) && ((uint8_t)S5_ST_ERROR == st),
                   "distance above 2047 mm saturates, state kept");

        TEST_CHECK("S5-U19",
                   ((uint8_t)S5_ST_ERROR < (1U << S5_PACK_STATE_BITS)) &&
                       (2000U <= S5_PACK_DIST_MAX_MM) &&
                       (sizeof(word) == 2U),
                   "every state fits 4 bits, 2000 mm fits 11 bits, 2 bytes");

        TEST_CHECK("S5-U20",
                   (E_OK == s5_get_status_packed(&word)) &&
                       (((word >> S5_PACK_STATE_SHIFT) & S5_PACK_STATE_MASK)
                        <= (uint16_t)S5_ST_ERROR),
                   "s5_get_status_packed() returns a valid live state");
        tm_printf((uint8_t *)"[TEST] S5-U20 info: packed word 0x%04x\n",
                  (uint32_t)word);
    }

    /* ---- latched stop path (Version C): full queue -> stop ---- */
    {
        T_CMBF       cm       = { 0 };
        s5_evt_t     evt      = { 0 };
        s5_stop_fn_t real_fn  = g_s5_stop_fn;
        uint32_t     before   = g_s5_estops;
        ID           mbf_test;
        ER           first;
        ER           second;

        /* private buffer that holds exactly one event, so the second
           send finds it full, like a VC queue that is not being read */
        cm.mbfatr  = TA_TFIFO;
        cm.maxmsz  = sizeof(s5_evt_t);
        cm.bufsz   = sizeof(s5_evt_t) + S5_MBF_MSG_OVERHEAD;
        mbf_test   = tk_cre_mbf(&cm);
        g_s5_stop_fn = test_fake_stop; /* do not latch the real motors */
        g_test_stops = 0;

        evt.type        = S5_EVT_OBSTACLE_DETECTED;
        evt.distance_mm = 200U;
        first  = s5_send_critical_to(mbf_test, &evt);
        second = s5_send_critical_to(mbf_test, &evt);

        g_s5_stop_fn = real_fn;
        TEST_CHECK("S5-U21",
                   (mbf_test > 0) && (E_OK == first) && (E_OK != second) &&
                       (1 == g_test_stops) && ((before + 1U) == g_s5_estops),
                   "queue full: obstacle event lost -> emergency stop called once");
        if (mbf_test > 0)
        {
            tk_del_mbf(mbf_test);
        }
    }

    TEST_CHECK("S5-U23", S5_DETECT_MM >= S5_STOP_NEEDED_MM,
               "detect distance covers reaction + braking + margin");
    tm_printf((uint8_t *)"[TEST] S5-U23 info: latency %d ms, reaction %d mm, "
              "needed %d mm, detect %d mm\n",
              (int32_t)S5_LATENCY_MS, (int32_t)S5_REACTION_MM,
              (int32_t)S5_STOP_NEEDED_MM, (int32_t)S5_DETECT_MM);

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */

/*** end of file ***/
