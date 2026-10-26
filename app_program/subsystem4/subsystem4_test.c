/*
 *      subsystem4_test.c
 *      On-target tests for subsystem 4.  Built only with: make TEST=1
 *      Record results in testing/subsystem4/TEST_LOG.md (IDs match).
 *
 *      Needs the LSM303DLHC wired and the robot STILL and LEVEL at
 *      power-up (S4-U03..U05).  S4-U06..U10 use synthetic data only.
 */

#include "subsystem4.h"
#include "drv_lsm303.h"
#include "imu_selftest.h"
#include "../test_report.h"

#if APP_TEST

void subsystem4_test(void)
{
    T_RMBF       rmbf;
    imu_status_t status;
    uint32_t     before;
    int32_t      peak_um;
    int32_t      humps;

    TEST_INIT();

    /* S4-U01: event queue exists with the documented message size */
    TEST_CHECK("S4-U01",
               (E_OK == tk_ref_mbf(g_mbf_imu_evt, &rmbf)) &&
                   ((int32_t)sizeof(imu_event_msg_t) == rmbf.maxmsz),
               "mbf_imu_evt max message = sizeof(IMU_EVENT_MSG)");

    /* S4-U02: status can be read through the public API */
    TEST_CHECK("S4-U02", E_OK == s4_get_status(&status),
               "s4_get_status() returns E_OK");

    /* Bring-up aid: list what answers on I2C0 */
    (void)lsm303_scan();

    /* S4-U03: sensor present and calibrated (cal takes 1 s) */
    tk_dly_tsk(1500);
    s4_get_status(&status);
    tm_printf(
        (uint8_t *)"[TEST] S4-U03 info: state %d calibrated %d i2c errors %d"
                   " az %d mg\n",
        (int32_t)status.state, (int32_t)status.calibrated,
        (int32_t)g_s4_i2c_errors, (int32_t)status.az_mg);
    TEST_CHECK("S4-U03",
               (0U != status.calibrated) && (IMU_ST_ERROR != status.state),
               "LSM303DLHC answers and calibration completes");

    /* S4-U04: robot level after calibration */
    TEST_CHECK("S4-U04",
               (status.pitch_cdeg > -300) && (status.pitch_cdeg < 300) &&
                   (status.roll_cdeg > -300) && (status.roll_cdeg < 300) &&
                   (status.az_mg > 900) && (status.az_mg < 1100),
               "level and still: |pitch|,|roll| < 3 deg, az ~ 1 g");

    /* S4-U05: 20 ms period holds (NFR2): ~50 samples per second */
    before = status.sample_count;
    tk_dly_tsk(1000);
    s4_get_status(&status);
    tm_printf((uint8_t *)"[TEST] S4-U05 info: %d samples in 1 s, overruns %d\n",
              (int32_t)(status.sample_count - before), (int32_t)g_s4_overruns);
    TEST_CHECK("S4-U05",
               ((status.sample_count - before) >= 48U) &&
                   ((status.sample_count - before) <= 52U) &&
                   (0U == g_s4_overruns),
               "imu_task runs every 20 ms with no overruns");

    /* S4-U06..U10: algorithm on synthetic data */
    TEST_CHECK("S4-U06", 0 != imu_selftest_math(),
               "integer atan2 / sqrt within 0.5 deg");
    TEST_CHECK("S4-U07", 0 != imu_selftest_hump(30, &peak_um, &humps),
               "30 mm synthetic hump measured within 20 %");
    tm_printf((uint8_t *)"[TEST] S4-U07 info: peak %d um, humps %d\n",
              (int32_t)peak_um, (int32_t)humps);
    TEST_CHECK("S4-U08", 0 != imu_selftest_impact(),
               "one 2.5 g spike = exactly one impact event");
    TEST_CHECK("S4-U09", 0 != imu_selftest_turn(),
               "60 deg/s heading change = TURNING_LEFT");
    TEST_CHECK("S4-U10", 0 != imu_selftest_cal_reject(),
               "calibration rejected while the robot shakes");

    /* S4-U11: peak reset request is served by imu_task */
    s4_request_peak_reset();
    tk_dly_tsk(3 * S4_PERIOD_MS);
    s4_get_status(&status);
    TEST_CHECK("S4-U11", (0 == status.max_peak_um) && (0U == status.hump_count),
               "s4_request_peak_reset() clears hump statistics");

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */

/*** end of file ***/
