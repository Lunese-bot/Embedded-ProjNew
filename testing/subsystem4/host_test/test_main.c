/*
 *	test_main.c
 *	Host (PC) tests for subsystem 4's pure-C algorithm.
 *
 *	    make -C host_test        (needs gcc; no Pico, no SDK)
 *
 *	Uses the SAME imu_selftest.c as the on-target test build, plus a
 *	height sweep that prints measured vs. true hump height.
 */

#include <stdio.h>
#include "imu_selftest.h"

static int fails;

static void check(const char *id, int ok, const char *what)
{
	printf("[HOST] %s %s  %s\n", id, ok ? "PASS" : "FAIL", what);
	if (!ok) {
		fails++;
	}
}

int main(void)
{
	int32_t	peak;
	int32_t	humps;
	int32_t	h;

	check("S4-U06", imu_selftest_math(), "integer atan2 / sqrt");
	check("S4-U07", imu_selftest_hump(30, &peak, &humps), "30 mm hump");
	check("S4-U08", imu_selftest_impact(), "impact detection");
	check("S4-U09", imu_selftest_turn(), "turn detection");
	check("S4-U10", imu_selftest_cal_reject(), "calibration rejects motion");

	printf("\nheight sweep (15 deg ramps, 150 mm/s):\n");
	for (h = 10; h <= 80; h += 10) {
		int ok = imu_selftest_hump(h, &peak, &humps);
		printf("  true %3d mm  measured %6.1f mm  humps %d  %s\n",
		       (int)h, peak / 1000.0, (int)humps, ok ? "ok" : "OUT OF 20%");
	}

	printf("\n%s: %d failed\n", fails ? "FAIL" : "PASS", fails);
	return fails ? 1 : 0;
}
