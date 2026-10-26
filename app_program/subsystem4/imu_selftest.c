/*
 *      imu_selftest.c
 *      Subsystem 4 -- synthetic-data tests of imu_terrain.c.  See header.
 */

#include "imu_selftest.h"
#include "imu_terrain.h"
#include "imu_math.h"

#define SPEED_MM_S 150
#define SIN15_MG   259 /* 1000 * sin(15 deg) */
#define COS15_MG   966 /* 1000 * cos(15 deg) */

static imu_terrain_t g_terrain; /* test instance, not the task's */
static uint32_t      g_lcg_state = 12345U;
static uint32_t      g_ev_all;

/* Deterministic noise in -8..+8 mg */
static int16_t noise(void)
{
    g_lcg_state = (g_lcg_state * 1103515245U) + 12345U;
    return (int16_t)((int32_t)((g_lcg_state >> 16) % 17U) - 8);
}

static void feed(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t mag_x,
                 int16_t mag_y, int32_t count, int32_t speed)
{
    imu_sample_t sample;
    int32_t      idx;

    for (idx = 0; idx < count; idx++)
    {
        sample.ax_mg = (int16_t)(acc_x + noise());
        sample.ay_mg = (int16_t)(acc_y + noise());
        sample.az_mg = (int16_t)(acc_z + noise());
        sample.mx    = mag_x;
        sample.my    = mag_y;
        sample.mz    = 0;
        g_ev_all |= imu_terrain_update(&g_terrain, &sample, speed);
    }
}

/* Fresh instance, calibrated level and still */
static int32_t setup(void)
{
    imu_cfg_t cfg;

    imu_terrain_default_cfg(&cfg);
    imu_terrain_init(&g_terrain, &cfg);
    g_ev_all = 0U;
    feed(0, 0, 1000, 300, 0, cfg.cal_samples, 0);
    return (IMU_ST_MONITORING == g_terrain.st.state) &&
           ((g_ev_all & IMU_EV_CAL_DONE) != 0U);
}

int32_t imu_selftest_math(void)
{
    static const struct
    {
        int32_t y;
        int32_t x;
        int32_t want;
    } cases[] = {
        { 0, 1000, 0 },      { 1000, 1000, 4500 },
        { 1000, 0, 9000 },   { 1000, -1000, 13500 },
        { 0, -1000, 18000 }, { -1000, -1000, -13500 },
        { -1000, 0, -9000 }, { -1000, 1000, -4500 },
        { 259, 966, 1500 },  { 500, 866, 3000 },
        { 866, 500, 6000 },  { -500, -866, -15000 },
    };
    uint32_t idx;
    int32_t  err;

    for (idx = 0U; idx < (sizeof(cases) / sizeof(cases[0])); idx++)
    {
        err = imu_atan2_cdeg(cases[idx].y, cases[idx].x) - cases[idx].want;
        if ((err > 50) || (err < -50))
        { /* 0.5 deg */
            return 0;
        }
    }
    if ((imu_isqrt32(1000000U) != 1000U) || (imu_isqrt32(99U) != 9U))
    {
        return 0;
    }
    if ((imu_wrap_cdeg(35000) != -1000) || (imu_wrap_cdeg(-19000) != 17000))
    {
        return 0;
    }
    return 1;
}

int32_t imu_selftest_hump(int32_t height_mm, int32_t * p_peak_um,
                          int32_t * p_humps)
{
    const int32_t mm_per_sample = (SPEED_MM_S * 20) / 1000; /* 3 */
    int32_t       ramp_mm       = (height_mm * 1000) / SIN15_MG;
    int32_t       ramp_n        = ramp_mm / mm_per_sample;

    if (!setup())
    {
        return 0;
    }
    feed(0, 0, 1000, 300, 0, 50, SPEED_MM_S);                 /* flat */
    feed(SIN15_MG, 0, COS15_MG, 300, 0, ramp_n, SPEED_MM_S);  /* up */
    feed(0, 0, 1000, 300, 0, 20, SPEED_MM_S);                 /* top */
    feed(-SIN15_MG, 0, COS15_MG, 300, 0, ramp_n, SPEED_MM_S); /* down */
    feed(0, 0, 1000, 300, 0, 50, SPEED_MM_S);                 /* flat */

    *p_peak_um = g_terrain.st.max_peak_um;
    *p_humps   = g_terrain.st.hump_count;

    return (1U == g_terrain.st.hump_count) &&
           ((g_ev_all & IMU_EV_HUMP_START) != 0U) &&
           ((g_ev_all & IMU_EV_HUMP_END) != 0U) &&
           (IMU_ST_MONITORING == g_terrain.st.state) &&
           (g_terrain.st.max_peak_um > (height_mm * 800)) && /* -20 % */
           (g_terrain.st.max_peak_um < (height_mm * 1200));  /* +20 % */
}

int32_t imu_selftest_impact(void)
{
    if (!setup())
    {
        return 0;
    }
    feed(0, 0, 1000, 300, 0, 50, SPEED_MM_S);
    feed(1200, 0, 2200, 300, 0, 1, SPEED_MM_S); /* ~2.5 g spike */
    feed(0, 0, 1000, 300, 0, 50, SPEED_MM_S);

    return (1U == g_terrain.st.impact_count) &&
           ((g_ev_all & IMU_EV_IMPACT) != 0U) &&
           (0U == g_terrain.st.hump_count);
}

int32_t imu_selftest_turn(void)
{
    int32_t idx;
    int32_t hdg_mrad = 0;

    if (!setup())
    {
        return 0;
    }
    /* Field vector (1000, 1000 * angle): for small angles atan2 of it
       is about the angle, so the heading grows ~60 deg/s
       (21 mrad per 20 ms).  30 samples stay below 40 deg. */
    for (idx = 0; idx < 30; idx++)
    {
        hdg_mrad += 21;
        feed(0, 0, 1000, 1000, (int16_t)hdg_mrad, 1, SPEED_MM_S);
    }
    return (IMU_MOT_TURNING_LEFT == g_terrain.st.motion);
}

int32_t imu_selftest_cal_reject(void)
{
    imu_cfg_t cfg;
    int32_t   idx;

    imu_terrain_default_cfg(&cfg);
    imu_terrain_init(&g_terrain, &cfg);
    g_ev_all = 0U;
    for (idx = 0; idx < cfg.cal_samples; idx++)
    {
        feed((0 != (idx % 2)) ? 200 : -200, 0, 1000, 300, 0, 1, 0);
    }
    return ((g_ev_all & IMU_EV_CAL_FAILED) != 0U) &&
           (IMU_ST_CALIBRATING == g_terrain.st.state) &&
           (0U == g_terrain.st.calibrated);
}

/*** end of file ***/
