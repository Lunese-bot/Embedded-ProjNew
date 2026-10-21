/*
 *	imu_selftest.c
 *	Subsystem 4 -- synthetic-data tests of imu_terrain.c.  See header.
 */

#include "imu_selftest.h"
#include "imu_terrain.h"
#include "imu_math.h"

#define SPEED_MM_S	150
#define SIN15_MG	259	/* 1000 * sin(15 deg) */
#define COS15_MG	966	/* 1000 * cos(15 deg) */

static imu_terrain_t	t;		/* test instance, not the task's */
static uint32_t		lcg = 12345U;
static uint32_t		ev_all;

/* Deterministic noise in -8..+8 mg */
static int16_t noise(void)
{
	lcg = (lcg * 1103515245U) + 12345U;
	return (int16_t)((int32_t)((lcg >> 16) % 17U) - 8);
}

static void feed(int16_t ax, int16_t ay, int16_t az,
		 int16_t mx, int16_t my, int32_t n, int32_t speed)
{
	imu_sample_t	s;
	int32_t		i;

	for (i = 0; i < n; i++) {
		s.ax_mg = (int16_t)(ax + noise());
		s.ay_mg = (int16_t)(ay + noise());
		s.az_mg = (int16_t)(az + noise());
		s.mx    = mx;
		s.my    = my;
		s.mz    = 0;
		ev_all |= imu_terrain_update(&t, &s, speed);
	}
}

/* Fresh instance, calibrated level and still */
static int setup(void)
{
	imu_cfg_t cfg;

	imu_terrain_default_cfg(&cfg);
	imu_terrain_init(&t, &cfg);
	ev_all = 0U;
	feed(0, 0, 1000, 300, 0, cfg.cal_samples, 0);
	return (t.st.state == IMU_ST_MONITORING) &&
	       ((ev_all & IMU_EV_CAL_DONE) != 0U);
}

int imu_selftest_math(void)
{
	static const struct { int32_t y, x, want; } cases[] = {
		{    0, 1000,      0 }, { 1000, 1000,   4500 },
		{ 1000,    0,   9000 }, { 1000, -1000, 13500 },
		{    0, -1000, 18000 }, { -1000, -1000, -13500 },
		{ -1000,   0,  -9000 }, { -1000, 1000, -4500 },
		{  259,  966,   1500 }, { 500,  866,    3000 },
		{  866,  500,   6000 }, { -500, -866, -15000 },
	};
	uint32_t	i;
	int32_t		err;

	for (i = 0U; i < (sizeof(cases) / sizeof(cases[0])); i++) {
		err = imu_atan2_cdeg(cases[i].y, cases[i].x) - cases[i].want;
		if ((err > 50) || (err < -50)) {		/* 0.5 deg */
			return 0;
		}
	}
	if ((imu_isqrt32(1000000U) != 1000U) || (imu_isqrt32(99U) != 9U)) {
		return 0;
	}
	if ((imu_wrap_cdeg(35000) != -1000) || (imu_wrap_cdeg(-19000) != 17000)) {
		return 0;
	}
	return 1;
}

int imu_selftest_hump(int32_t height_mm, int32_t *peak_um, int32_t *humps)
{
	const int32_t	mm_per_sample = (SPEED_MM_S * 20) / 1000;	/* 3 */
	int32_t		ramp_mm = (height_mm * 1000) / SIN15_MG;
	int32_t		ramp_n  = ramp_mm / mm_per_sample;

	if (!setup()) {
		return 0;
	}
	feed(0, 0, 1000, 300, 0, 50, SPEED_MM_S);			/* flat */
	feed(SIN15_MG, 0, COS15_MG, 300, 0, ramp_n, SPEED_MM_S);	/* up */
	feed(0, 0, 1000, 300, 0, 20, SPEED_MM_S);			/* top */
	feed(-SIN15_MG, 0, COS15_MG, 300, 0, ramp_n, SPEED_MM_S);	/* down */
	feed(0, 0, 1000, 300, 0, 50, SPEED_MM_S);			/* flat */

	*peak_um = t.st.max_peak_um;
	*humps   = t.st.hump_count;

	return (t.st.hump_count == 1U) &&
	       ((ev_all & IMU_EV_HUMP_START) != 0U) &&
	       ((ev_all & IMU_EV_HUMP_END) != 0U) &&
	       (t.st.state == IMU_ST_MONITORING) &&
	       (t.st.max_peak_um > height_mm * 800) &&		/* -20 % */
	       (t.st.max_peak_um < height_mm * 1200);		/* +20 % */
}

int imu_selftest_impact(void)
{
	if (!setup()) {
		return 0;
	}
	feed(0, 0, 1000, 300, 0, 50, SPEED_MM_S);
	feed(1200, 0, 2200, 300, 0, 1, SPEED_MM_S);	/* ~2.5 g spike */
	feed(0, 0, 1000, 300, 0, 50, SPEED_MM_S);

	return (t.st.impact_count == 1U) &&
	       ((ev_all & IMU_EV_IMPACT) != 0U) &&
	       (t.st.hump_count == 0U);
}

int imu_selftest_turn(void)
{
	int32_t i;
	int32_t hdg_mrad = 0;

	if (!setup()) {
		return 0;
	}
	/* Field vector (1000, 1000 * angle): for small angles atan2 of it
	   is about the angle, so the heading grows ~60 deg/s
	   (21 mrad per 20 ms).  30 samples stay below 40 deg. */
	for (i = 0; i < 30; i++) {
		hdg_mrad += 21;
		feed(0, 0, 1000, 1000, (int16_t)hdg_mrad, 1, SPEED_MM_S);
	}
	return (t.st.motion == IMU_MOT_TURNING_LEFT);
}

int imu_selftest_cal_reject(void)
{
	imu_cfg_t	cfg;
	int32_t		i;

	imu_terrain_default_cfg(&cfg);
	imu_terrain_init(&t, &cfg);
	ev_all = 0U;
	for (i = 0; i < cfg.cal_samples; i++) {
		feed((int16_t)(((i & 1) != 0) ? 200 : -200), 0, 1000,
		     300, 0, 1, 0);
	}
	return ((ev_all & IMU_EV_CAL_FAILED) != 0U) &&
	       (t.st.state == IMU_ST_CALIBRATING) &&
	       (t.st.calibrated == 0U);
}
