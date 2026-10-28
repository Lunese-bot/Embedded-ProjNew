/*
 *      imu_terrain.h
 *      Subsystem 4 -- IMU processing and terrain state machine (pure C).
 *
 *      Implements Design Review Statechart 4 (IMU-Based Terrain Monitoring):
 *
 *         INIT -> CALIBRATING -> MONITORING <-> CLIMBING -> AT_PEAK
 *                                    ^                         |
 *                                    +------ DESCENDING <------+
 *         any state -> ERROR (set by the task on I2C/IMU failure)
 *
 *      Motion classes while monitoring: STATIONARY, CRUISING, ACCELERATING,
 *      BRAKING, TURNING_LEFT, TURNING_RIGHT.  Impacts and tilt-limit are
 *      EVENTS: they are reported without breaking hump tracking.
 *
 *      No kernel calls here: the task (task_imu.c) feeds one sample every
 *      period and publishes the result.  This keeps the algorithm testable
 *      on a PC (see host_test/).
 *
 *      Axes after mounting remap (subsystem4_cfg.h):
 *        X = forward, Y = left, Z = up.  Accelerometer reads +1000 mg on Z
 *        when level and still.  Pitch > 0 = nose up.  Roll > 0 = left side up.
 */

#ifndef IMU_TERRAIN_H
#define IMU_TERRAIN_H

#include <stdint.h>
#include <stdbool.h>

/* ---- states and classes -------------------------------------------- */
typedef enum
{
    IMU_ST_INIT = 0,
    IMU_ST_CALIBRATING,
    IMU_ST_MONITORING,
    IMU_ST_CLIMBING,
    IMU_ST_AT_PEAK,
    IMU_ST_DESCENDING,
    IMU_ST_ERROR
} imu_state_t;

typedef enum
{
    IMU_MOT_UNKNOWN = 0,
    IMU_MOT_STATIONARY,
    IMU_MOT_CRUISING,
    IMU_MOT_ACCELERATING,
    IMU_MOT_BRAKING,
    IMU_MOT_TURNING_LEFT,
    IMU_MOT_TURNING_RIGHT
} imu_motion_t;

/* Events returned by imu_terrain_update() as a bit mask */
#define IMU_EV_CAL_DONE      0x0001u /* calibration finished */
#define IMU_EV_CAL_FAILED    0x0002u /* robot moved during calibration */
#define IMU_EV_HUMP_START    0x0004u /* climb confirmed */
#define IMU_EV_HUMP_PEAK     0x0008u /* top of hump reached */
#define IMU_EV_HUMP_END      0x0010u /* back on level ground; peak valid */
#define IMU_EV_HUMP_ABORT    0x0020u /* hump timed out, measurement dropped */
#define IMU_EV_IMPACT        0x0040u /* sudden shock / collision */
#define IMU_EV_TILT_LIMIT    0x0080u /* pitch or roll beyond safe limit */
#define IMU_EV_MOTION_CHANGE 0x0100u /* motion class changed */

/* ---- inputs --------------------------------------------------------- */

/* One sample, already remapped to robot axes */
typedef struct
{
    int16_t ax_mg; /* acceleration, milli-g */
    int16_t ay_mg;
    int16_t az_mg;
    int16_t mx; /* magnetometer, raw counts (any scale) */
    int16_t my;
    int16_t mz;
} imu_sample_t;

/* Tunable parameters (NFR9: calibration must be configurable) */
typedef struct
{
    int32_t period_ms;         /* sample period */
    int32_t lpf_shift;         /* EMA filter: alpha = 1 / 2^shift */
    int32_t cal_samples;       /* samples averaged for bias */
    int32_t cal_max_spread_mg; /* max max-min during calibration */

    int32_t hump_enter_cdeg;    /* pitch to start climb / descent */
    int32_t hump_level_cdeg;    /* |pitch| below this = level */
    int32_t hump_confirm;       /* samples to confirm a transition */
    int32_t hump_timeout;       /* samples before a hump is aborted */
    int32_t hump_min_peak_um;   /* ignore "humps" lower than this */
    int32_t nominal_speed_mm_s; /* used when speed_mm_s < 0 */

    int32_t impact_mg;       /* |a| - 1 g, raw (unfiltered) */
    int32_t impact_holdoff;  /* samples between impact events */
    int32_t tilt_limit_cdeg; /* pitch/roll safety limit */

    int32_t accel_mg;         /* forward accel for ACCEL/BRAKING */
    int32_t still_mg;         /* |a_dyn| below this may be still */
    int32_t still_samples;    /* ... for this many samples */
    int32_t turn_rate_cdeg_s; /* heading rate for TURNING */
} imu_cfg_t;

/* ---- outputs -------------------------------------------------------- */
typedef struct
{
    // imu_state_t  state;
    // imu_motion_t motion;
    // uint8_t      calibrated; /* 1 after IMU_EV_CAL_DONE */
    // uint8_t      fault;      /* 1 while in IMU_ST_ERROR */

    uint8_t state      : 3;
    uint8_t motion     : 3;
    uint8_t calibrated : 1;
    uint8_t fault      : 1;

    int16_t ax_mg; /* filtered, bias removed */
    int16_t ay_mg;
    int16_t az_mg;
    int16_t pitch_cdeg;       /* + = nose up */
    int16_t roll_cdeg;        /* + = left side up */
    int16_t heading_cdeg;     /* magnetic, NOT tilt compensated */
    int32_t turn_rate_cdeg_s; /* + = turning left (CCW) */

    int32_t  hump_height_um; /* height above hump start, now */
    int32_t  last_peak_um;   /* peak of the last completed hump */
    int32_t  max_peak_um;    /* highest hump this run */
    uint16_t hump_count;
    uint16_t impact_count;
    int32_t  last_impact_mg; /* size of the last impact */
    uint32_t sample_count;
} imu_status_t;

/* Complete algorithm state (one instance, owned by the IMU task) */
typedef struct
{
    imu_cfg_t    cfg;
    imu_status_t st;

    /* calibration */
    int32_t cal_n;
    int32_t cal_sum[3];
    int16_t cal_min[3];
    int16_t cal_max[3];
    int16_t bias[3]; /* subtracted from raw accel */

    /* filters, scaled by 2^lpf_shift for precision */
    int32_t f_acc[3];
    int32_t f_rate;
    int16_t prev_heading;
    uint8_t have_heading;

    /* state machine helpers */
    int32_t pending;  /* candidate next state (imu_state_t) */
    int32_t confirm;  /* consecutive samples it was seen */
    int32_t in_state; /* samples spent in current hump state */
    int32_t peak_um;  /* running max during this hump */
    int32_t still_cnt;
    int32_t holdoff;
    uint8_t tilt_latched;
    int32_t level_ax_mg; /* forward accel on level ground */
} imu_terrain_t;

/* Fill cfg with the defaults from subsystem4_cfg.h values */
void imu_terrain_default_cfg(imu_cfg_t * p_cfg);

/* Reset everything and start calibrating (robot must be still) */
void imu_terrain_init(imu_terrain_t * p_terrain, const imu_cfg_t * p_cfg);

/* Re-run calibration without losing hump statistics */
void imu_terrain_recalibrate(imu_terrain_t * p_terrain);

/* Forget hump statistics (new run) */
void imu_terrain_reset_peaks(imu_terrain_t * p_terrain);

/* Feed one sample.  speed_mm_s = forward wheel speed from motion
   control (used to turn pitch into height), or -1 if unknown (then
   cfg.nominal_speed_mm_s is used).  Returns IMU_EV_* bits. */
uint32_t imu_terrain_update(imu_terrain_t *      p_terrain,
                            const imu_sample_t * p_sample, int32_t speed_mm_s);

/* Enter / leave ERROR (the task decides, from I2C results) */
void imu_terrain_set_fault(imu_terrain_t * p_terrain, int32_t fault);

const char * imu_state_name(imu_state_t state);
const char * imu_motion_name(imu_motion_t motion);

#endif /* IMU_TERRAIN_H */

/*** end of file ***/
