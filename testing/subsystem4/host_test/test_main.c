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
#include "imu_terrain.h"

static int fails;

static void check(const char *id, int ok, const char *what)
{
	printf("[HOST] %s %s  %s\n", id, ok ? "PASS" : "FAIL", what);
	if (!ok) {
		fails++;
	}
}


static int test_one_wheel_hump(void)
{
    const int16_t ax[] = { 87, 0, -87, 0 };
    const int16_t ay[] = { 173, 173, 173, 0 };
    const int16_t az[] = { 981, 985, 981, 1000 };
    const int durations[] = { 40, 20, 40, 50 };
    imu_cfg_t cfg;
    imu_terrain_t terrain;
    imu_sample_t sample = {0};
    uint32_t events = 0U;
    int stage, i;

    imu_terrain_default_cfg(&cfg);
    imu_terrain_init(&terrain, &cfg);

    sample.az_mg = 1000;
    sample.mx = 300;

    for (i = 0; i < cfg.cal_samples; i++) {
        events |= imu_terrain_update(&terrain, &sample, 0);
    }

    if ((events & IMU_EV_CAL_DONE) == 0U) {
        return 0;
    }

    events = 0U;

    for (stage = 0; stage < 4; stage++) {
        sample.ax_mg = ax[stage];
        sample.ay_mg = ay[stage];
        sample.az_mg = az[stage];

        for (i = 0; i < durations[stage]; i++) {
            events |= imu_terrain_update(&terrain, &sample, 150);
        }

		printf("[DEBUG] stage=%d pitch=%d roll=%d state=%s\n",
           stage,
           terrain.st.pitch_cdeg,
           terrain.st.roll_cdeg,
           imu_state_name(terrain.st.state));
    }

	printf("[DEBUG] S4-U11 state=%s start=%d end=%d humps=%u\n",
       imu_state_name(terrain.st.state),
       (events & IMU_EV_HUMP_START) != 0U,
       (events & IMU_EV_HUMP_END) != 0U,
       (unsigned int)terrain.st.hump_count);

	printf("[DEBUG] S4-U11 estimated height = %.2f mm\n",
       (double)terrain.st.max_peak_um / 1000.0);
    return ((events & IMU_EV_HUMP_END) != 0U) &&
           (terrain.st.hump_count == 1U) &&
           (terrain.st.max_peak_um > 0);
}


static int test_flat_ground_no_hump(void)
{
    imu_cfg_t cfg;
    imu_terrain_t terrain;
    imu_sample_t sample = {0};
    uint32_t events = 0U;
    int i;

    imu_terrain_default_cfg(&cfg);
    imu_terrain_init(&terrain, &cfg);

    sample.az_mg = 1000;
    sample.mx = 300;

    /* Complete calibration on level ground */
    for (i = 0; i < cfg.cal_samples; i++) {
        events |= imu_terrain_update(&terrain, &sample, 0);
    }

    if ((events & IMU_EV_CAL_DONE) == 0U) {
        return 0;
    }

    events = 0U;

    /* Simulate movement with sideways roll and minor pitch noise */
    for (i = 0; i < 150; i++) {
        sample.ax_mg = (i == 75) ? 87 : 35;
        sample.ay_mg = 156;
        sample.az_mg = 987;

        events |= imu_terrain_update(&terrain, &sample, 150);
    }

    return ((events & (IMU_EV_HUMP_START | IMU_EV_HUMP_END)) == 0U) &&
           (terrain.st.hump_count == 0U) &&
           (terrain.st.state == IMU_ST_MONITORING);
}


static int test_stationary_tilt_not_braking(void)
{
    imu_cfg_t cfg;
    imu_terrain_t terrain;
    imu_sample_t sample = {0};
    uint32_t events = 0U;
    int i;

    imu_terrain_default_cfg(&cfg);
    imu_terrain_init(&terrain, &cfg);

    /* Calibrate on level ground */
    sample.az_mg = 1000;
    sample.mx = 300;

    for (i = 0; i < cfg.cal_samples; i++) {
        events |= imu_terrain_update(&terrain, &sample, 0);
    }

    if ((events & IMU_EV_CAL_DONE) == 0U) {
        return 0;
    }

    /* Tilt downhill while wheel speed remains zero */
    sample.ax_mg = -208;
    sample.az_mg = 978;

    for (i = 0; i < 50; i++) {
        (void)imu_terrain_update(&terrain, &sample, 0);
    }

    printf("[DEBUG] S4-U13 pitch=%d motion=%s\n",
           terrain.st.pitch_cdeg,
           imu_motion_name(terrain.st.motion));

    return terrain.st.motion == IMU_MOT_STATIONARY;
}


static int32_t simulate_hump_speed_spike(int spike)
{
    const int16_t ax[] = {87, 0, -87, 0};
    const int16_t ay[] = {173, 173, 173, 0};
    const int16_t az[] = {981, 985, 981, 1000};
    const int durations[] = {40, 20, 40, 50};

    imu_cfg_t cfg;
    imu_terrain_t terrain;
    imu_sample_t sample = {0};
    uint32_t events = 0U;
    int stage;
    int i;

    imu_terrain_default_cfg(&cfg);
    imu_terrain_init(&terrain, &cfg);

    sample.az_mg = 1000;
    sample.mx = 300;

    for (i = 0; i < cfg.cal_samples; i++) {
        events |= imu_terrain_update(&terrain, &sample, 0);
    }

    if ((events & IMU_EV_CAL_DONE) == 0U) {
        return -1;
    }

    for (stage = 0; stage < 4; stage++) {
        sample.ax_mg = ax[stage];
        sample.ay_mg = ay[stage];
        sample.az_mg = az[stage];

        for (i = 0; i < durations[stage]; i++) {
            int32_t speed = 150;

            /* Inject one incorrect encoder reading */
            if (spike && stage == 0 && i == 18) {
                speed = 3000;
            }

            events |= imu_terrain_update(
                &terrain, &sample, speed);
        }
    }

    if (terrain.st.hump_count != 1U ||
        (events & IMU_EV_HUMP_END) == 0U) {
        return -1;
    }

    return terrain.st.max_peak_um;
}

static int test_encoder_speed_spike(void)
{
    int32_t normal = simulate_hump_speed_spike(0);
    int32_t spike = simulate_hump_speed_spike(1);

    printf("[DEBUG] S4-U14 normal=%ldum spike=%ldum\n",
           (long)normal, (long)spike);

    /* Provisional tolerance: at most 20% extra height */
    return normal > 0 &&
           spike > 0 &&
           (int64_t)spike * 100 <= (int64_t)normal * 120;
}


int main(void)
{

	printf("[SIZE] imu_status_t = %zu bytes\n",
       sizeof(imu_status_t));

	printf("[SIZE] imu_terrain_t = %zu bytes\n",
       sizeof(imu_terrain_t));

	int32_t	peak;
	int32_t	humps;
	int32_t	h;

	check("S4-U06", imu_selftest_math(), "integer atan2 / sqrt");
	check("S4-U07", imu_selftest_hump(30, &peak, &humps), "30 mm hump");
	check("S4-U08", imu_selftest_impact(), "impact detection");
	check("S4-U09", imu_selftest_turn(), "turn detection");
	check("S4-U10", imu_selftest_cal_reject(), "calibration rejects motion");
	check("S4-U11", test_one_wheel_hump(),
      "one-wheel hump with pitch and roll");
	check("S4-U12", test_flat_ground_no_hump(),
      "no false hump during normal movement");
	check("S4-U13", test_stationary_tilt_not_braking(),
      "stationary downhill tilt is not braking");
	check("S4-U14", test_encoder_speed_spike(),
      "hump height rejects abnormal encoder speed");

	printf("\nheight sweep (15 deg ramps, 150 mm/s):\n");
	for (h = 10; h <= 80; h += 10) {
		int ok = imu_selftest_hump(h, &peak, &humps);
		printf("  true %3d mm  measured %6.1f mm  humps %d  %s\n",
		       (int)h, peak / 1000.0, (int)humps, ok ? "ok" : "OUT OF 20%");
	}

	printf("\n%s: %d failed\n", fails ? "FAIL" : "PASS", fails);
	return fails ? 1 : 0;
}
