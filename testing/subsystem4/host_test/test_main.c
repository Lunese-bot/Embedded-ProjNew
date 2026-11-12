/*
 *  test_main.c
 *  Host (PC) tests for subsystem 4's pure-C algorithm.
 *
 *      make -C host_test        (needs gcc; no Pico, no SDK)
 *
 *  Uses the SAME imu_selftest.c as the on-target test build, plus a
 *  height sweep that prints measured vs. true hump height.
 */

#include <stdio.h>
#include "imu_selftest.h"
#include "imu_terrain.h"
#include "s4_config.h"

static int fails;

/* Gravity X in mg sets the shallow ramp angle (61 = 3.5, 49 = 2.8 deg). */
static int test_shallow_hump(int height_mm, int plateau, int ramp_mg)
{
    imu_cfg_t cfg;
    imu_terrain_t terrain;
    imu_sample_t sample = {0};
    const int ax[] = {ramp_mg, 0, -ramp_mg, 0};
    const int ramp_z = ramp_mg == 49 ? 999 : 998;
    const int az[] = {ramp_z, 1000, ramp_z, 1000};
    int counts[4];
    uint32_t events = 0U;
    int stage, i;

    imu_terrain_default_cfg(&cfg);
    cfg.hump_lever_mm = 0; /* whole robot on a ramp: summed height */
    imu_terrain_init(&terrain, &cfg);
    sample.az_mg = 1000;
    sample.mx = 300;
    for (i = 0; i < cfg.cal_samples; i++) {
        (void)imu_terrain_update(&terrain, &sample, 0);
    }
    counts[0] = counts[2] =
        (height_mm * 1000 + 3 * ramp_mg / 2) / (3 * ramp_mg);
    counts[1] = plateau;
    counts[3] = 50;
    for (stage = 0; stage < 4; stage++) {
        for (i = 0; i < counts[stage]; i++) {
            /* Repeatable vibration of +/-4 mg around the ramp angle. */
            sample.ax_mg = (int16_t)(ax[stage] + (i % 3 - 1) * 4);
            sample.az_mg = (int16_t)az[stage];
            events |= imu_terrain_update(&terrain, &sample, 150);
        }
    }
    printf("[HEIGHT] shallow %d mm ax=%dmg plateau=%d measured=%.2f mm\n",
           height_mm, ramp_mg, plateau, terrain.st.last_peak_um / 1000.0);
    return (events & IMU_EV_HUMP_START) && (events & IMU_EV_HUMP_END) &&
           !(events & IMU_EV_HUMP_ABORT) && terrain.st.hump_count == 1 &&
           terrain.st.state == IMU_ST_MONITORING &&
           terrain.st.last_peak_um > height_mm * 800 &&
           terrain.st.last_peak_um < height_mm * 1200 &&
           terrain.st.safety_stop == IMU_SAFETY_NONE;
}

static int test_consecutive_transitions(void)
{
    const imu_state_t from[] = {IMU_ST_CLIMBING, IMU_ST_AT_PEAK,
                               IMU_ST_DESCENDING};
    const imu_state_t to[] = {IMU_ST_AT_PEAK, IMU_ST_DESCENDING,
                             IMU_ST_MONITORING};
    imu_cfg_t cfg;
    imu_terrain_t terrain;
    imu_sample_t sample = {0};
    int stage, i;

    imu_terrain_default_cfg(&cfg);
    cfg.hump_lever_mm = 0; /* whole robot on a ramp: summed height */
    cfg.lpf_shift = 0; /* Isolate debounce from filter settling. */
    cfg.hump_lpf_shift = 0;
    for (stage = 0; stage < 3; stage++) {
        imu_terrain_init(&terrain, &cfg);
        sample.ax_mg = 0;
        sample.az_mg = 1000;
        for (i = 0; i < cfg.cal_samples; i++) {
            (void)imu_terrain_update(&terrain, &sample, 0);
        }
        terrain.st.state = from[stage];
        terrain.peak_um = cfg.hump_min_peak_um; /* a real hump so far */
        for (i = 0; i < cfg.hump_confirm * 2; i++) {
            sample.ax_mg = (i % 2) ? 35 : (stage == 1 ? -70 : 0);
            (void)imu_terrain_update(&terrain, &sample, 150);
            if (terrain.st.state != from[stage]) {
                return 0;
            }
        }
        sample.ax_mg = stage == 1 ? -70 : 0;
        for (i = 0; i < cfg.hump_confirm; i++) {
            (void)imu_terrain_update(&terrain, &sample, 150);
        }
        if (terrain.st.state != to[stage]) {
            return 0;
        }
    }
    return 1;
}

static int test_live_maximum(void)
{
    imu_cfg_t cfg;
    imu_terrain_t terrain;
    imu_sample_t sample = {0};
    int i;
    int32_t peak;

    imu_terrain_default_cfg(&cfg);
    cfg.hump_lever_mm = 0; /* whole robot on a ramp: summed height */
    imu_terrain_init(&terrain, &cfg);
    sample.az_mg = 1000;
    sample.mx = 300;
    for (i = 0; i < cfg.cal_samples; i++) {
        (void)imu_terrain_update(&terrain, &sample, 0);
    }
    sample.ax_mg = 87;
    sample.az_mg = 996;
    for (i = 0; i < 40; i++) {
        (void)imu_terrain_update(&terrain, &sample, 150);
        if (terrain.peak_um < cfg.hump_min_peak_um &&
            terrain.st.max_peak_um != 0) {
            return 0;
        }
    }
    peak = terrain.st.max_peak_um;
    if (terrain.st.state != IMU_ST_CLIMBING || peak < 8000 ||
        peak != terrain.peak_um || terrain.st.hump_count != 0 ||
        terrain.st.last_peak_um != 0) {
        return 0;
    }
    /* A stopped car on the ramp retains its measured maximum. */
    for (i = 0; i < 20; i++) {
        (void)imu_terrain_update(&terrain, &sample, 0);
    }
    return terrain.st.max_peak_um == peak;
}

static void feed_level(imu_terrain_t *terrain, int count, int speed)
{
    imu_sample_t sample = {0};
    int i;

    sample.az_mg = 1000;
    for (i = 0; i < count; i++) {
        (void)imu_terrain_update(terrain, &sample, speed);
    }
}

/* 2026-11-11: course hump under the RIGHT REAR wheel only (10 mm high,
   150 mm up, 5 mm top, 150 mm down), left wheel and front support on the
   floor.  The robot tilts by asin(h / lever), mostly sideways (direction
   55 deg from forward, as in the robot log).  Height must read 10 mm +-20 %. */
#include <math.h>
static int test_one_wheel_lever(void)
{
    imu_cfg_t cfg;
    imu_terrain_t terrain;
    imu_sample_t sample = {0};
    double lever; const double dir = 55.0 * 3.14159265 / 180.0;
    const int speed = 40; /* mm/s */
    double s_mm = 0.0;
    int i;

    imu_terrain_default_cfg(&cfg);
    if (cfg.hump_lever_mm <= 0) {
        return 0;
    }
    lever = (double)cfg.hump_lever_mm;
    imu_terrain_init(&terrain, &cfg);
    feed_level(&terrain, cfg.cal_samples, 0);
    feed_level(&terrain, 50, speed);
    for (i = 0; i < 700; i++) {
        double h = 0.0, t;
        s_mm += speed * 0.020;
        if (s_mm < 150.0)      h = 10.0 * s_mm / 150.0;
        else if (s_mm < 155.0) h = 10.0;
        else if (s_mm < 305.0) h = 10.0 * (305.0 - s_mm) / 150.0;
        t = asin(h / lever);
        sample.ax_mg = (int16_t)(1000.0 * sin(t) * cos(dir));
        sample.ay_mg = (int16_t)(-1000.0 * sin(t) * sin(dir));
        sample.az_mg = (int16_t)(1000.0 * cos(t));
        (void)imu_terrain_update(&terrain, &sample, speed);
    }
    printf("[HEIGHT] one-wheel 10 mm hump: max %.2f mm, last %.2f mm, humps %d\n",
           terrain.st.max_peak_um / 1000.0, terrain.st.last_peak_um / 1000.0,
           (int)terrain.st.hump_count);
    return (terrain.st.max_peak_um >= 8000) && (terrain.st.max_peak_um <= 12000);
}

static int test_pickup_safety(void)
{
    /* Pickup, unloading, roll both ways, pitch both ways, inverted.
       Overturn and free fall must also work before calibration completes;
       an upward lift needs the first second after power-on to learn 1 g. */
    const int16_t vectors[][3] = {
        {0, 0, 1250}, {0, 0, 400}, {0, 800, 600},
        {0, -800, 600}, {800, 0, 600}, {-800, 0, 600}, {0, 0, -1000}
    };
    imu_cfg_t cfg;
    imu_terrain_t terrain;
    imu_sample_t sample = {0};
    int v, i, vector, min_n, max_n;
    uint32_t events;

    imu_terrain_default_cfg(&cfg);
    cfg.hump_lever_mm = 0; /* whole robot on a ramp: summed height */
    for (v = 0; v < 14; v++) {
        vector = v % 7;
        imu_terrain_init(&terrain, &cfg);
        if (v >= 7 || vector == 0) {
            feed_level(&terrain, cfg.cal_samples, 0);
        }
        sample.ax_mg = vectors[vector][0];
        sample.ay_mg = vectors[vector][1];
        sample.az_mg = vectors[vector][2];
        events = 0U;
        /* A 0.25 g push up is a lift within 60-200 ms; the others need
           exactly their confirm count. */
        min_n = vector == 0 ? 3 : (vector == 1 ? S4_LOW_G_CONFIRM
                                               : S4_OVERTURN_CONFIRM);
        max_n = vector == 0 ? 10 : min_n;
        for (i = 0; i < max_n && !terrain.st.safety_stop; i++) {
            events |= imu_terrain_update(&terrain, &sample, 0);
            if (i < min_n - 1 && terrain.st.safety_stop) {
                return 0;
            }
        }
        if (!(events & IMU_EV_SAFETY_STOP) ||
            terrain.st.safety_stop !=
                (vector < 2 ? IMU_SAFETY_LIFT : IMU_SAFETY_OVERTURN)) {
            return 0;
        }
        /* Set down level and still: stays stopped just short of the
           settle time, then clears by itself (no reboot, 2026-11-11). */
        feed_level(&terrain, S4_SETTLE_SAMPLES - 1, 0);
        if (!terrain.st.safety_stop) {
            return 0;
        }
        feed_level(&terrain, 1, 0);
        if (terrain.st.safety_stop) {
            return 0;
        }
    }
    return 1;
}

/* A hump's jolts must not stop the robot, whatever the hump state says,
   and a lift on the hump must still stop it (2026-11-11). */
static int test_safety_hump(void)
{
    imu_cfg_t cfg;
    imu_terrain_t terrain;
    imu_sample_t sample = {0};
    int i, rep;

    imu_terrain_default_cfg(&cfg);
    cfg.hump_lever_mm = 0; /* whole robot on a ramp: summed height */
    imu_terrain_init(&terrain, &cfg);
    feed_level(&terrain, cfg.cal_samples, 0);
    /* 1. hitting the foot of a hump: 0.6 g up for 40 ms, then the same
       back down (the robot does not keep rising), five times */
    for (rep = 0; rep < 5; rep++) {
        for (i = 0; i < 4; i++) {
            sample.az_mg = (int16_t)(i < 2 ? 1600 : 400);
            (void)imu_terrain_update(&terrain, &sample, 150);
        }
        feed_level(&terrain, 10, 150);
        if (terrain.st.safety_stop) {
            return 0;
        }
    }
    /* 2. driving up a real 10 degree ramp at 150 mm/s: the robot gains
       26 mm/s of upward speed in 100 ms (27 mg), then holds it */
    sample.ax_mg = 174;
    for (i = 0; i < 100; i++) {
        sample.az_mg = (int16_t)(i < 5 ? 1012 : 985);
        (void)imu_terrain_update(&terrain, &sample, 150);
    }
    if (terrain.st.safety_stop || terrain.st.state != IMU_ST_CLIMBING) {
        return 0;
    }
    /* 3. picked up while on the ramp (hump state CLIMBING): 0.15 g up */
    sample.az_mg = 1135;
    for (i = 0; i < 15 && !terrain.st.safety_stop; i++) {
        (void)imu_terrain_update(&terrain, &sample, 150);
    }
    return IMU_SAFETY_LIFT == terrain.st.safety_stop;
}

/* Speeding up and braking tilt the accelerometer's "pitch" but are not a
   hump: no CLIMBING, and never left sitting in AT_PEAK (2026-11-11). */
static int test_accel_not_hump(void)
{
    imu_cfg_t cfg;
    imu_terrain_t terrain;
    imu_sample_t sample = {0};
    uint32_t events = 0U;
    int i, rep;

    imu_terrain_default_cfg(&cfg);
    cfg.hump_lever_mm = 0; /* whole robot on a ramp: summed height */
    imu_terrain_init(&terrain, &cfg);
    feed_level(&terrain, cfg.cal_samples, 0);
    sample.az_mg = 1000;
    for (rep = 0; rep < 4; rep++) {
        /* 0 -> 120 mm/s in 100 ms = 122 mg (7 deg), cruise, brake */
        for (i = 0; i < 100; i++) {
            sample.ax_mg = (int16_t)(i < 5 ? 122 : (i >= 60 && i < 65 ? -122 : 0));
            events |= imu_terrain_update(&terrain, &sample,
                                         (i < 60) ? 120 : 0);
            if (terrain.st.state == IMU_ST_AT_PEAK) {
                return 0;
            }
        }
    }
    if ((events & IMU_EV_HUMP_START) || terrain.st.hump_count != 0) {
        return 0;
    }
    /* A longer push (0.4 s at 3.4 deg) does read as a climb, but with no
       height gained it must drop back to MONITORING, not wait in AT_PEAK. */
    sample.ax_mg = 60;
    for (i = 0; i < 40; i++) {
        (void)imu_terrain_update(&terrain, &sample, 20);
    }
    sample.ax_mg = 0;
    for (i = 0; i < 60; i++) {
        (void)imu_terrain_update(&terrain, &sample, 20);
        if (terrain.st.state == IMU_ST_AT_PEAK) {
            return 0;
        }
    }
    return terrain.st.state == IMU_ST_MONITORING &&
           terrain.st.hump_count == 0;
}

static int test_safety_noise(void)
{
    imu_cfg_t cfg;
    imu_terrain_t terrain;
    imu_sample_t sample = {0};
    int i;
    imu_terrain_default_cfg(&cfg);
    cfg.hump_lever_mm = 0; /* whole robot on a ramp: summed height */
    imu_terrain_init(&terrain, &cfg);
    sample.az_mg = 1000;
    for (i = 0; i < cfg.cal_samples; i++) {
        (void)imu_terrain_update(&terrain, &sample, 0);
    }
    /* Isolated jolts cannot accumulate; ordinary ramp/turn acceleration
       must not trigger the additional stop.  (The jolts go up and down:
       a robot that stays on the floor averages 1 g.) */
    for (i = 0; i < 200; i++) {
        sample.ax_mg = (i % 2) ? 259 : -259;
        sample.ay_mg = (i % 2) ? 100 : -100;
        sample.az_mg = i % 3 == 0 ? 1300 : 810;
        sample.my = (int16_t)(i * 5);
        (void)imu_terrain_update(&terrain, &sample, 150);
        if (terrain.st.safety_stop) {
            return 0;
        }
    }
    return 1;
}

/* A sensor that reads 1.12 g at rest (the GY-511 does): after a lift the
   stop must still clear once it is set down (2026-11-11). */
static int test_settle_offset_sensor(void)
{
    imu_cfg_t cfg;
    imu_terrain_t terrain;
    imu_sample_t sample = {0};
    int i;

    imu_terrain_default_cfg(&cfg);
    imu_terrain_init(&terrain, &cfg);
    sample.az_mg = 1120;
    for (i = 0; i < 100; i++) {
        (void)imu_terrain_update(&terrain, &sample, 0);
    }
    if (terrain.st.safety_stop) {
        return 0;
    }
    sample.az_mg = 1400; /* lifted */
    for (i = 0; i < 15 && !terrain.st.safety_stop; i++) {
        (void)imu_terrain_update(&terrain, &sample, 0);
    }
    if (IMU_SAFETY_LIFT != terrain.st.safety_stop) {
        return 0;
    }
    sample.az_mg = 1120; /* set down */
    for (i = 0; i < 200 && terrain.st.safety_stop; i++) {
        (void)imu_terrain_update(&terrain, &sample, 0);
    }
    return IMU_SAFETY_NONE == terrain.st.safety_stop;
}

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
    cfg.hump_lever_mm = 0; /* whole robot on a ramp: summed height */
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
    cfg.hump_lever_mm = 0; /* whole robot on a ramp: summed height */
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
    cfg.hump_lever_mm = 0; /* whole robot on a ramp: summed height */
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
    cfg.hump_lever_mm = 0; /* whole robot on a ramp: summed height */
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

    int32_t peak;
    int32_t humps;
    int32_t h;

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
    check("S4-U15", test_shallow_hump(10, 20, 61), "shallow 10 mm hump");
    check("S4-U16", test_shallow_hump(20, 20, 61), "shallow 20 mm hump");
    check("S4-U17", test_shallow_hump(10, 0, 61), "shallow sharp crest");
    check("S4-U18", test_consecutive_transitions(),
          "interrupted samples cannot confirm a transition");
    check("S4-U19", test_shallow_hump(10, 20, 49), "2.8 degree 10 mm hump");
    check("S4-U20", test_shallow_hump(20, 20, 49), "2.8 degree 20 mm hump");

    check("S4-U21", test_live_maximum(),
          "maximum updates during climb before hump completion");
    check("S4-U22", test_pickup_safety(),
          "pickup/overturn stop holds, then clears 1 s after setting down");
    check("S4-U26", test_one_wheel_lever(),
          "one-wheel 10 mm hump reads 10 mm +-20 % (lever height)");
    check("S4-U25", test_accel_not_hump(),
          "speeding up / braking is not a hump");
    check("S4-U27", test_settle_offset_sensor(),
          "stop clears on a sensor that reads 1.12 g at rest");
    check("S4-U24", test_safety_hump(),
          "hump jolts and short bumps do not stop; a real lift still does");
    check("S4-U23", test_safety_noise(),
          "normal ramps, turns and isolated jolts do not latch safety stop");

    printf("\nheight sweep (15 deg ramps, 150 mm/s):\n");
    for (h = 10; h <= 80; h += 10) {
        int ok = imu_selftest_hump(h, &peak, &humps);
        if (!ok) {
            fails++;
        }
        printf("  true %3d mm  measured %6.1f mm  humps %d  %s\n",
               (int)h, peak / 1000.0, (int)humps, ok ? "ok" : "OUT OF 20%");
    }

    printf("\n%s: %d failed\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
