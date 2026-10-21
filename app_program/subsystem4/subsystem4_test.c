/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *	Record results in testing/subsystem4/TEST_LOG.md (IDs match).
 *
 *	Needs the LSM303DLHC wired and the robot STILL and LEVEL at
 *	power-up (S4-U03..U05).  S4-U06..U10 use synthetic data only.
 */

#include "subsystem4.h"
#include "drv_lsm303.h"
#include "imu_selftest.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem4_test(void)
{
	T_RMBF		rmbf;
	IMU_STATUS	st;
	UW		before;
	int32_t		peak_um;
	int32_t		humps;

	TEST_INIT();

	/* S4-U01: event queue exists with the documented message size */
	TEST_CHECK("S4-U01",
		   tk_ref_mbf(mbf_imu_evt, &rmbf) == E_OK &&
		   rmbf.maxmsz == (INT)sizeof(IMU_EVENT_MSG),
		   "mbf_imu_evt max message = sizeof(IMU_EVENT_MSG)");

	/* S4-U02: status can be read through the public API */
	TEST_CHECK("S4-U02", s4_get_status(&st) == E_OK,
		   "s4_get_status() returns E_OK");

	/* Bring-up aid: list what answers on I2C0 */
	(void)lsm303_scan();

	/* S4-U03: sensor present and calibrated (cal takes 1 s) */
	tk_dly_tsk(1500);
	s4_get_status(&st);
	tm_printf((UB*)"[TEST] S4-U03 info: state %d calibrated %d i2c errors %d"
		  " az %d mg\n", (INT)st.state, (INT)st.calibrated,
		  (INT)s4_i2c_errors, (INT)st.az_mg);
	TEST_CHECK("S4-U03", st.calibrated && st.state != IMU_ST_ERROR,
		   "LSM303DLHC answers and calibration completes");

	/* S4-U04: robot level after calibration */
	TEST_CHECK("S4-U04",
		   st.pitch_cdeg > -300 && st.pitch_cdeg < 300 &&
		   st.roll_cdeg  > -300 && st.roll_cdeg  < 300 &&
		   st.az_mg > 900 && st.az_mg < 1100,
		   "level and still: |pitch|,|roll| < 3 deg, az ~ 1 g");

	/* S4-U05: 20 ms period holds (NFR2): ~50 samples per second */
	before = st.sample_count;
	tk_dly_tsk(1000);
	s4_get_status(&st);
	tm_printf((UB*)"[TEST] S4-U05 info: %d samples in 1 s, overruns %d\n",
		  (INT)(st.sample_count - before), (INT)s4_overruns);
	TEST_CHECK("S4-U05",
		   st.sample_count - before >= 48 &&
		   st.sample_count - before <= 52 && s4_overruns == 0,
		   "imu_task runs every 20 ms with no overruns");

	/* S4-U06..U10: algorithm on synthetic data */
	TEST_CHECK("S4-U06", imu_selftest_math(),
		   "integer atan2 / sqrt within 0.5 deg");
	TEST_CHECK("S4-U07", imu_selftest_hump(30, &peak_um, &humps),
		   "30 mm synthetic hump measured within 20 %");
	tm_printf((UB*)"[TEST] S4-U07 info: peak %d um, humps %d\n",
		  (INT)peak_um, (INT)humps);
	TEST_CHECK("S4-U08", imu_selftest_impact(),
		   "one 2.5 g spike = exactly one impact event");
	TEST_CHECK("S4-U09", imu_selftest_turn(),
		   "60 deg/s heading change = TURNING_LEFT");
	TEST_CHECK("S4-U10", imu_selftest_cal_reject(),
		   "calibration rejected while the robot shakes");

	/* S4-U11: peak reset request is served by imu_task */
	s4_request_peak_reset();
	tk_dly_tsk(3 * S4_PERIOD_MS);
	s4_get_status(&st);
	TEST_CHECK("S4-U11", st.max_peak_um == 0 && st.hump_count == 0,
		   "s4_request_peak_reset() clears hump statistics");

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
