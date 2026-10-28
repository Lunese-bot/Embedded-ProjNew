/*
 *      imu_terrain.c
 *      Subsystem 4 -- IMU processing and terrain state machine.
 *      See imu_terrain.h for the states, axes and units.
 *
 *      Hump height method
 *      ------------------
 *      On a slope the accelerometer's forward axis sees gravity * sin(pitch),
 *      so sin(pitch) ~= ax / |a|.  Each sample the robot moves
 *      ds = speed * period, and climbs dh = ds * sin(pitch).  Summing dh
 *      from the foot of the hump gives its height; the largest value is the
 *      peak.  This avoids double-integrating vertical acceleration, which
 *      drifts badly (the "weak AI idea" rejected in design review 8.1).
 *      Accuracy depends on the speed from motion control (subsystem 2) and
 *      on the robot not accelerating hard on the slope; validate per A5.
 */

#include "imu_terrain.h"
#include "imu_math.h"
#include "subsystem4_cfg.h"

#define ONE_G_MG          1000
#define S4_MS_PER_S       1000
#define S4_TILT_HYST_CDEG 500 /* 5 deg hysteresis on the tilt latch */

/* Exponential moving average kept at 2^shift scale */
static int32_t ema_step(int32_t * p_acc, int32_t input, int32_t shift)
{
    /* Cast: shift an unsigned 1; the shift count is 0..15. */
    int32_t scale = (int32_t)(1U << (uint32_t)shift);

    *p_acc += input - (*p_acc / scale);
    return *p_acc / scale;
}

static int32_t iabs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

void imu_terrain_default_cfg(imu_cfg_t * p_cfg)
{
    p_cfg->period_ms          = S4_PERIOD_MS;
    p_cfg->lpf_shift          = S4_LPF_SHIFT;
    p_cfg->cal_samples        = S4_CAL_SAMPLES;
    p_cfg->cal_max_spread_mg  = S4_CAL_MAX_SPREAD_MG;
    p_cfg->hump_enter_cdeg    = S4_HUMP_ENTER_CDEG;
    p_cfg->hump_level_cdeg    = S4_HUMP_LEVEL_CDEG;
    p_cfg->hump_confirm       = S4_HUMP_CONFIRM;
    p_cfg->hump_timeout       = S4_HUMP_TIMEOUT;
    p_cfg->hump_min_peak_um   = S4_HUMP_MIN_PEAK_UM;
    p_cfg->nominal_speed_mm_s = S4_NOMINAL_SPEED_MM_S;
    p_cfg->impact_mg          = S4_IMPACT_MG;
    p_cfg->impact_holdoff     = S4_IMPACT_HOLDOFF;
    p_cfg->tilt_limit_cdeg    = S4_TILT_LIMIT_CDEG;
    p_cfg->accel_mg           = S4_ACCEL_MG;
    p_cfg->still_mg           = S4_STILL_MG;
    p_cfg->still_samples      = S4_STILL_SAMPLES;
    p_cfg->turn_rate_cdeg_s   = S4_TURN_RATE_CDEG_S;
}

static void start_calibration(imu_terrain_t * p_terrain)
{
    int32_t idx;

    p_terrain->cal_n = 0;
    for (idx = 0; idx < 3; idx++)
    {
        p_terrain->cal_sum[idx] = 0;
        p_terrain->cal_min[idx] = INT16_MAX;
        p_terrain->cal_max[idx] = INT16_MIN;
    }
    p_terrain->st.state      = IMU_ST_CALIBRATING;
    p_terrain->st.calibrated = 0U;
}

void imu_terrain_init(imu_terrain_t * p_terrain, const imu_cfg_t * p_cfg)
{
    uint8_t * p_bytes = (uint8_t *)p_terrain;
    uint32_t  idx;

    for (idx = 0U; idx < sizeof(*p_terrain); idx++)
    { /* no memset in freestanding */
        p_bytes[idx] = 0U;
    }
    p_terrain->cfg       = *p_cfg;
    p_terrain->st.state  = IMU_ST_INIT;
    p_terrain->st.motion = IMU_MOT_UNKNOWN;
    p_terrain->pending   = -1;
    start_calibration(p_terrain);
}

void imu_terrain_recalibrate(imu_terrain_t * p_terrain)
{
    start_calibration(p_terrain);
}

void imu_terrain_reset_peaks(imu_terrain_t * p_terrain)
{
    p_terrain->st.last_peak_um = 0;
    p_terrain->st.max_peak_um  = 0;
    p_terrain->st.hump_count   = 0U;
}

void imu_terrain_set_fault(imu_terrain_t * p_terrain, int32_t fault)
{
    if (fault != 0)
    {
        p_terrain->st.state = IMU_ST_ERROR;
        p_terrain->st.fault = 1U;
    }
    else if (p_terrain->st.fault != 0U)
    {
        /* Recovered: the filters are stale, so calibrate again */
        p_terrain->st.fault = 0U;
        start_calibration(p_terrain);
    }
    else
    {
        /* not in fault: nothing to do */
    }
}

/* ---------------------------------------------------------------------
 * Calibration: average still samples to find the accelerometer bias.
 * Assumes the robot is LEVEL and STILL (on the start line).
 * ------------------------------------------------------------------- */
static uint32_t calibrate_step(imu_terrain_t *      p_terrain,
                               const imu_sample_t * p_sample)
{
    int16_t vec[3];
    int32_t idx;

    vec[0] = p_sample->ax_mg;
    vec[1] = p_sample->ay_mg;
    vec[2] = p_sample->az_mg;

    for (idx = 0; idx < 3; idx++)
    {
        p_terrain->cal_sum[idx] += vec[idx];
        if (vec[idx] < p_terrain->cal_min[idx])
        {
            p_terrain->cal_min[idx] = vec[idx];
        }
        if (vec[idx] > p_terrain->cal_max[idx])
        {
            p_terrain->cal_max[idx] = vec[idx];
        }
    }
    p_terrain->cal_n++;

    if (p_terrain->cal_n < p_terrain->cfg.cal_samples)
    {
        return 0U;
    }

    for (idx = 0; idx < 3; idx++)
    {
        if ((p_terrain->cal_max[idx] - p_terrain->cal_min[idx]) >
            p_terrain->cfg.cal_max_spread_mg)
        {
            start_calibration(p_terrain); /* moved: try again */
            return IMU_EV_CAL_FAILED;
        }
    }

    p_terrain->bias[0] = (int16_t)(p_terrain->cal_sum[0] / p_terrain->cal_n);
    p_terrain->bias[1] = (int16_t)(p_terrain->cal_sum[1] / p_terrain->cal_n);
    p_terrain->bias[2] =
        (int16_t)((p_terrain->cal_sum[2] / p_terrain->cal_n) - ONE_G_MG);

    /* Seed the filters with a level, still robot */
    p_terrain->f_acc[0] = 0;
    p_terrain->f_acc[1] = 0;
    p_terrain->f_acc[2] =
        ONE_G_MG * (int32_t)(1U << (uint32_t)p_terrain->cfg.lpf_shift);
    p_terrain->f_rate       = 0;
    p_terrain->have_heading = 0U;

    p_terrain->st.calibrated     = 1U;
    p_terrain->st.state          = IMU_ST_MONITORING;
    p_terrain->st.hump_height_um = 0;
    p_terrain->pending           = -1;
    p_terrain->confirm           = 0;
    return IMU_EV_CAL_DONE;
}

/* A transition happens only after `hump_confirm` samples in a row
   asked for the same next state (debounce). */
static bool confirmed(imu_terrain_t * p_terrain, imu_state_t next)
{
    if (p_terrain->pending == (int32_t)next)
    {
        p_terrain->confirm++;
    }
    else
    {
        p_terrain->pending = (int32_t)next;
        p_terrain->confirm = 1;
    }
    return (p_terrain->confirm >= p_terrain->cfg.hump_confirm);
}

static void enter(imu_terrain_t * p_terrain, imu_state_t next)
{
    p_terrain->st.state = next;
    p_terrain->in_state = 0;
    p_terrain->pending  = -1;
    p_terrain->confirm  = 0;
}

/* ---------------------------------------------------------------------
 * Hump state machine (Statechart 4)
 * ------------------------------------------------------------------- */
static void hump_reset_height(imu_terrain_t * p_terrain)
{
    p_terrain->st.hump_height_um = 0;
    p_terrain->peak_um           = 0;
}

static uint32_t hump_monitoring(imu_terrain_t * p_terrain, bool b_rising,
                                bool b_level)
{
    uint32_t events = 0U;

    if (b_level)
    {
        hump_reset_height(p_terrain); /* re-zero on flat ground */
    }
    if (b_rising)
    {
        if (confirmed(p_terrain, IMU_ST_CLIMBING))
        {
            enter(p_terrain, IMU_ST_CLIMBING);
            events |= IMU_EV_HUMP_START;
        }
    }
    else
    {
        p_terrain->pending = -1;
        p_terrain->confirm = 0;
    }
    return events;
}

static uint32_t hump_climbing(imu_terrain_t * p_terrain, bool b_rising,
                              bool b_down, bool b_level)
{
    uint32_t events = 0U;

    if (b_level && (confirmed(p_terrain, IMU_ST_AT_PEAK)))
    {
        enter(p_terrain, IMU_ST_AT_PEAK);
        events |= IMU_EV_HUMP_PEAK;
    }
    else if (b_down && (confirmed(p_terrain, IMU_ST_DESCENDING)))
    {
        enter(p_terrain, IMU_ST_DESCENDING); /* sharp crest */
        events |= IMU_EV_HUMP_PEAK;
    }
    else if (b_rising)
    {
        p_terrain->pending = -1;
        p_terrain->confirm = 0;
    }
    else
    {
        /* between thresholds: keep waiting */
    }
    return events;
}

static void hump_at_peak(imu_terrain_t * p_terrain, bool b_rising, bool b_down)
{
    if (b_down && (confirmed(p_terrain, IMU_ST_DESCENDING)))
    {
        enter(p_terrain, IMU_ST_DESCENDING);
    }
    else if (b_rising && (confirmed(p_terrain, IMU_ST_CLIMBING)))
    {
        /* plateau, then more climb */
        enter(p_terrain, IMU_ST_CLIMBING);
    }
    else
    {
        /* still on top */
    }
}

static uint32_t hump_descending(imu_terrain_t * p_terrain, bool b_rising,
                                bool b_level)
{
    uint32_t events = 0U;

    if (b_level && (confirmed(p_terrain, IMU_ST_MONITORING)))
    {
        p_terrain->st.last_peak_um = p_terrain->peak_um;
        if (p_terrain->peak_um >= p_terrain->cfg.hump_min_peak_um)
        {
            p_terrain->st.hump_count++;
            if (p_terrain->peak_um > p_terrain->st.max_peak_um)
            {
                p_terrain->st.max_peak_um = p_terrain->peak_um;
            }
            events |= IMU_EV_HUMP_END;
        }
        else
        {
            events |= IMU_EV_HUMP_ABORT; /* too small */
        }
        enter(p_terrain, IMU_ST_MONITORING);
        hump_reset_height(p_terrain);
    }
    else if (b_rising && (confirmed(p_terrain, IMU_ST_CLIMBING)))
    {
        enter(p_terrain, IMU_ST_CLIMBING); /* double hump */
    }
    else
    {
        /* still descending */
    }
    return events;
}


static uint32_t hump_step(imu_terrain_t * p_terrain, int32_t pitch,
                          int32_t speed_mm_s)
{
    const imu_cfg_t * p_cfg = &p_terrain->cfg;
    uint32_t          events = 0U;

    /* One-wheel hump: smaller pitch accompanied by sideways roll */
    bool b_one_wheel =
        (speed_mm_s > 0) &&
        (iabs32(p_terrain->st.roll_cdeg) > S4_ONE_WHEEL_ROLL_CDEG);

    bool b_rising =
        (pitch > p_cfg->hump_enter_cdeg) ||
        (b_one_wheel && (pitch > S4_ONE_WHEEL_PITCH_CDEG));

    bool b_down =
        (pitch < -p_cfg->hump_enter_cdeg) ||
        (b_one_wheel && (pitch < -S4_ONE_WHEEL_PITCH_CDEG));

    bool b_level =
        (iabs32(pitch) < p_cfg->hump_level_cdeg);


    if (p_terrain->st.hump_height_um > p_terrain->peak_um)
    {
        p_terrain->peak_um = p_terrain->st.hump_height_um;
    }

    switch (p_terrain->st.state)
    {
        case IMU_ST_MONITORING:
            events |= hump_monitoring(p_terrain, b_rising, b_level);
            break;

        case IMU_ST_CLIMBING:
            events |= hump_climbing(p_terrain, b_rising, b_down, b_level);
            break;

        case IMU_ST_AT_PEAK:
            hump_at_peak(p_terrain, b_rising, b_down);
            break;

        case IMU_ST_DESCENDING:
            events |= hump_descending(p_terrain, b_rising, b_level);
            break;

        default:
            break;
    }

    /* Bounded time in any hump state (NFR5) */
    if ((IMU_ST_CLIMBING == p_terrain->st.state) ||
        (IMU_ST_AT_PEAK == p_terrain->st.state) ||
        (IMU_ST_DESCENDING == p_terrain->st.state))
    {
        p_terrain->in_state++;
        if (p_terrain->in_state > p_cfg->hump_timeout)
        {
            enter(p_terrain, IMU_ST_MONITORING);
            hump_reset_height(p_terrain);
            events |= IMU_EV_HUMP_ABORT;
        }
    }
    return events;
}

/* ---------------------------------------------------------------------
 * Motion classification (only meaningful on level ground)
 * ------------------------------------------------------------------- */
static imu_motion_t classify(imu_terrain_t * p_terrain, int32_t dyn_mg,
                             int32_t speed_mm_s)
{
    const imu_cfg_t * p_cfg = &p_terrain->cfg;
    imu_motion_t      motion;

    if (iabs32(dyn_mg) < p_cfg->still_mg)
    {
        p_terrain->still_cnt++;
    }
    else
    {
        p_terrain->still_cnt = 0;
    }


    if (p_terrain->st.turn_rate_cdeg_s > p_cfg->turn_rate_cdeg_s)
    {
        motion = IMU_MOT_TURNING_LEFT;
    }
    else if (p_terrain->st.turn_rate_cdeg_s < -p_cfg->turn_rate_cdeg_s)
    {
        motion = IMU_MOT_TURNING_RIGHT;
    }
    else if ((0 == speed_mm_s) ||
             ((speed_mm_s < 0) &&
              (p_terrain->still_cnt >= p_cfg->still_samples)))
    {
        motion = IMU_MOT_STATIONARY;
    }
    else if (p_terrain->st.ax_mg > p_cfg->accel_mg)
    {
        motion = IMU_MOT_ACCELERATING;
    }
    else if (p_terrain->st.ax_mg < -p_cfg->accel_mg)
    {
        motion = IMU_MOT_BRAKING;
    }
    else
    {
        motion = IMU_MOT_CRUISING;
    }

    return motion;
}

/* Step 2: impact check on the RAW magnitude (filtering would hide it) */
static uint32_t check_impact(imu_terrain_t * p_terrain, int32_t raw_norm)
{
    uint32_t events = 0U;

    if (p_terrain->holdoff > 0)
    {
        p_terrain->holdoff--;
    }
    else if (iabs32(raw_norm - ONE_G_MG) > p_terrain->cfg.impact_mg)
    {
        p_terrain->st.impact_count++;
        p_terrain->st.last_impact_mg = raw_norm - ONE_G_MG;
        p_terrain->holdoff           = p_terrain->cfg.impact_holdoff;
        events |= IMU_EV_IMPACT;
    }
    else
    {
        /* no impact */
    }
    return events;
}

/* Step 3: low-pass filter, then tilt and the tilt-limit latch */
static uint32_t update_tilt(imu_terrain_t * p_terrain, int32_t acc_x,
                            int32_t acc_y, int32_t acc_z)
{
    const imu_cfg_t * p_cfg        = &p_terrain->cfg;
    uint32_t          events       = 0U;
    int32_t           release_cdeg = p_cfg->tilt_limit_cdeg - S4_TILT_HYST_CDEG;

    /* Cast: filtered values stay within the +-2 g sensor range. */
    p_terrain->st.ax_mg =
        (int16_t)ema_step(&p_terrain->f_acc[0], acc_x, p_cfg->lpf_shift);
    p_terrain->st.ay_mg =
        (int16_t)ema_step(&p_terrain->f_acc[1], acc_y, p_cfg->lpf_shift);
    p_terrain->st.az_mg =
        (int16_t)ema_step(&p_terrain->f_acc[2], acc_z, p_cfg->lpf_shift);

    /* Cast: angles are within +-18000 cdeg; squares are non-negative. */
    p_terrain->st.pitch_cdeg = (int16_t)imu_atan2_cdeg(
        p_terrain->st.ax_mg,
        (int32_t)imu_isqrt32(
            (uint32_t)(p_terrain->st.ay_mg * p_terrain->st.ay_mg) +
            (uint32_t)(p_terrain->st.az_mg * p_terrain->st.az_mg)));
    p_terrain->st.roll_cdeg =
        (int16_t)imu_atan2_cdeg(p_terrain->st.ay_mg, p_terrain->st.az_mg);

    if ((iabs32(p_terrain->st.pitch_cdeg) > p_cfg->tilt_limit_cdeg) ||
        (iabs32(p_terrain->st.roll_cdeg) > p_cfg->tilt_limit_cdeg))
    {
        if (0U == p_terrain->tilt_latched)
        {
            p_terrain->tilt_latched = 1U;
            events |= IMU_EV_TILT_LIMIT;
        }
    }
    else if ((iabs32(p_terrain->st.pitch_cdeg) < release_cdeg) &&
             (iabs32(p_terrain->st.roll_cdeg) < release_cdeg))
    {
        p_terrain->tilt_latched = 0U;
    }
    else
    {
        /* in the hysteresis band */
    }
    return events;
}

/* Step 4: heading and turn rate from the magnetometer.
   Not tilt compensated: on a hump the heading wobbles, which is
   why turning is only classified on level ground. */
static void update_heading(imu_terrain_t *      p_terrain,
                           const imu_sample_t * p_sample)
{
    int32_t heading;
    int32_t rate;

    heading = imu_atan2_cdeg(p_sample->my, p_sample->mx);
    if (p_terrain->have_heading != 0U)
    {
        rate = imu_wrap_cdeg(heading - p_terrain->prev_heading) *
               (S4_MS_PER_S / p_terrain->cfg.period_ms);
        p_terrain->st.turn_rate_cdeg_s =
            ema_step(&p_terrain->f_rate, rate, p_terrain->cfg.lpf_shift);
    }
    /* Cast: the heading is within +-18000 cdeg. */
    p_terrain->prev_heading    = (int16_t)heading;
    p_terrain->have_heading    = 1U;
    p_terrain->st.heading_cdeg = (int16_t)heading;
}

/* Step 5: height, dh = ds * sin(pitch) = ds * ax / |a| */

static void update_height(imu_terrain_t * p_terrain, int32_t speed_mm_s)
{
    int32_t ds_um;
    int32_t norm;

    /* Ignore implausibly high encoder-speed readings. */
    if (speed_mm_s > S4_HUMP_MAX_SPEED_MM_S)
    {
        return;
    }

    ds_um = speed_mm_s * p_terrain->cfg.period_ms;

    norm  = (int32_t)imu_norm3(p_terrain->st.ax_mg, p_terrain->st.ay_mg,
                               p_terrain->st.az_mg);
    if (norm > 0)
    {
        p_terrain->st.hump_height_um += (ds_um * p_terrain->st.ax_mg) / norm;
    }
}

uint32_t imu_terrain_update(imu_terrain_t *      p_terrain,
                            const imu_sample_t * p_sample, int32_t speed_mm_s)
{
    uint32_t     events = 0U;
    int32_t      acc_x;
    int32_t      acc_y;
    int32_t      acc_z;
    int32_t      raw_norm;
    int32_t      speed;
    imu_motion_t motion;

    p_terrain->st.sample_count++;

    if (IMU_ST_ERROR == p_terrain->st.state)
    {
        events = 0U;
    }
    else if (IMU_ST_CALIBRATING == p_terrain->st.state)
    {
        events = calibrate_step(p_terrain, p_sample);
    }
    else
    {
        /* 1. Remove bias */
        acc_x = (int32_t)p_sample->ax_mg - p_terrain->bias[0];
        acc_y = (int32_t)p_sample->ay_mg - p_terrain->bias[1];
        acc_z = (int32_t)p_sample->az_mg - p_terrain->bias[2];

        /* 2. Impact, 3. tilt, 4. heading */
        /* Cast: the norm of a +-2 g vector fits int32_t. */
        raw_norm = (int32_t)imu_norm3(acc_x, acc_y, acc_z);
        events |= check_impact(p_terrain, raw_norm);
        events |= update_tilt(p_terrain, acc_x, acc_y, acc_z);
        update_heading(p_terrain, p_sample);

        /* 5. Height (no wheel speed: assume the nominal speed) */
        speed =
            (speed_mm_s < 0) ? p_terrain->cfg.nominal_speed_mm_s : speed_mm_s;
        update_height(p_terrain, speed);

        /* 6. Hump state machine */
        events |= hump_step(p_terrain, p_terrain->st.pitch_cdeg, speed_mm_s);

        /* 7. Motion class */
        if (IMU_ST_MONITORING == p_terrain->st.state)
        {
            motion = classify(p_terrain, raw_norm - ONE_G_MG, speed);
        }
        else
        {
            motion = IMU_MOT_CRUISING; /* on a hump */
        }
        if (motion != p_terrain->st.motion)
        {
            p_terrain->st.motion = motion;
            events |= IMU_EV_MOTION_CHANGE;
        }
    }
    return events;
}

const char * imu_state_name(imu_state_t state)
{
    const char * p_name;

    switch (state)
    {
        case IMU_ST_INIT:
            p_name = "INIT";
            break;
        case IMU_ST_CALIBRATING:
            p_name = "CALIBRATING";
            break;
        case IMU_ST_MONITORING:
            p_name = "MONITORING";
            break;
        case IMU_ST_CLIMBING:
            p_name = "CLIMBING";
            break;
        case IMU_ST_AT_PEAK:
            p_name = "AT_PEAK";
            break;
        case IMU_ST_DESCENDING:
            p_name = "DESCENDING";
            break;
        case IMU_ST_ERROR:
            p_name = "ERROR";
            break;
        default:
            p_name = "?";
            break;
    }
    return p_name;
}

const char * imu_motion_name(imu_motion_t motion)
{
    const char * p_name;

    switch (motion)
    {
        case IMU_MOT_STATIONARY:
            p_name = "STATIONARY";
            break;
        case IMU_MOT_CRUISING:
            p_name = "CRUISING";
            break;
        case IMU_MOT_ACCELERATING:
            p_name = "ACCELERATING";
            break;
        case IMU_MOT_BRAKING:
            p_name = "BRAKING";
            break;
        case IMU_MOT_TURNING_LEFT:
            p_name = "TURN_LEFT";
            break;
        case IMU_MOT_TURNING_RIGHT:
            p_name = "TURN_RIGHT";
            break;
        default:
            p_name = "UNKNOWN";
            break;
    }
    return p_name;
}

/*** end of file ***/
