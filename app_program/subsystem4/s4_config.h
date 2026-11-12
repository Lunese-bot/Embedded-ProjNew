/*
 *      s4_config.h
 *      Subsystem 4 -- every tunable number in one place.
 *
 *      All thresholds are FIRST GUESSES.  Assumption A5 in the design
 *      review says they must be tuned from logged data over flat ground
 *      and the real humps; record the final values in
 *      testing/subsystem4/TEST_LOG.md together with the evidence.
 *
 *      Pure macros (no kernel types): also used by the host tests.
 */

#ifndef S4_CONFIG_H
#define S4_CONFIG_H

/* ---- timing (NFR2: IMU every 20 ms) -------------------------------- */
#define S4_PERIOD_MS 20

/* ---- mounting: robot axis = sign * sensor axis ---------------------
 * Robot axes: X forward, Y left, Z up.  Set these after looking at
 * which way the LSM303DLHC board is mounted on the chassis.  Check with
 * test S4-M02 (tilt the robot nose-up: pitch must go positive). */
#define S4_ACC_X_SRC  0 /* 0 = sensor X, 1 = Y, 2 = Z */
#define S4_ACC_Y_SRC  1
#define S4_ACC_Z_SRC  2
#define S4_ACC_X_SIGN (-1) /* 2026-11-11: hump log showed climbing as negative pitch; was (+1) */
#define S4_ACC_Y_SIGN (+1)
#define S4_ACC_Z_SIGN (+1)
#define S4_MAG_X_SIGN (+1) /* magnetometer uses the same */
#define S4_MAG_Y_SIGN (+1) /* source axes as the accel     */
#define S4_MAG_Z_SIGN (+1)

/* ---- filter and calibration --------------------------------------- */
#define S4_LPF_SHIFT         2  /* EMA alpha = 1/4  (~70 ms at 50 Hz) */
#define S4_CAL_SAMPLES       50 /* 1 s of still samples */
#define S4_CAL_MAX_SPREAD_MG 60 /* moved during cal if spread > this */

/* ---- hump detection ------------------------------------------------ */
#define S4_HUMP_ENTER_CDEG    250  /* 2.5 deg; shallow 1-2 cm humps */
/* 2026-11-11: the way down is taken at ~70 mm/s (twice the climb speed),
   so it is short and the slow pitch only reached -2.3 deg: AT_PEAK never
   became DESCENDING.  Only used after a climb (CLIMBING / AT_PEAK). */
#define S4_HUMP_DOWN_CDEG     150  /* 1.5 deg */
#define S4_ONE_WHEEL_PITCH_CDEG 400
#define S4_ONE_WHEEL_ROLL_CDEG  700
#define S4_HUMP_LEVEL_CDEG    100  /* |pitch| < 1.0 deg = level (hysteresis) */
#define S4_HUMP_CONFIRM       3    /* 60 ms to confirm a transition */
/* 2026-11-11: the hump states use a slower copy of the pitch (EMA 1/16,
   ~0.3 s).  Pitch comes from the accelerometer alone, so speeding up or
   braking looks like a tilt; a speed change of up to ~130 mm/s now stays
   under HUMP_ENTER, while a real ramp (seconds long) passes. */
#define S4_HUMP_LPF_SHIFT     4
/* Hump height, 2026-11-11 (MY): only the RIGHT REAR wheel rides over the
   hump; the left rear wheel and the front support stay on the floor.  The
   robot then rests on three points and tilts (forward + sideways) by
   asin(h / lever), lever = distance from the lifted wheel to the line
   through the other two contacts = B*A / sqrt(A^2/4 + B^2).
   1 = height from that lever (one-wheel hump, matches the course),
   0 = old method: sum of distance x sin(pitch) (whole robot on a ramp;
       came out 5.9 mm on the 10 mm hump). */
#define S4_HUMP_HEIGHT_TILT   1
/* Lever mode: a climb must also cover this much wheel travel, or a push
   from the motors (forward acceleration looks like tilt) would count. */
#define S4_HUMP_MIN_TRAVEL_MM 30
/* Lever mode: a tilt worth more than this is not the course hump (10 mm)
   but the robot being picked up or carried: drop the measurement and wait
   for level ground.  2026-11-11 log: handling read 20 and 47 mm. */
#define S4_HUMP_MAX_PEAK_MM   18
#define S4_TRACK_MM           117 /* A: rear wheel centre to centre (MY, 117 mm) */
#define S4_AXLE_TO_FRONT_MM   82  /* B: rear axle to front support centre (MY, 82 mm) */
#define S4_HUMP_TIMEOUT       500  /* 10 s in one hump state = abort */
#define S4_HUMP_MIN_PEAK_UM   3000 /* < 3 mm is noise, not a hump */
#define S4_NOMINAL_SPEED_MM_S 150  /* used when motion speed unknown */
#define S4_HUMP_MAX_SPEED_MM_S 500

/* ---- impact / safety ----------------------------------------------- */
#define S4_IMPACT_MG       900  /* | |a| - 1 g | above this = impact */
#define S4_IMPACT_HOLDOFF  25   /* 500 ms between impact events */
#define S4_TILT_LIMIT_CDEG 3000 /* 30 deg pitch or roll = unsafe */

/* Additional pickup/overturn stop, always enabled.  Raw samples avoid EMA
   delay.  Thresholds require validation on the car (IMU recorder).
   2026-11-11 (MY): a hump must NOT stop the robot, and no reboot needed.
   - LIFT = the robot gained upward speed.  |a| - 1 g is summed with a leak
     (time constant 2^LIFT_LEAK_SHIFT samples), which is the vertical speed
     in mg x samples (1000 ~ 0.2 m/s).  A hand lift reaches 0.3-1 m/s; a
     hump at driving speed gives < 0.05 m/s and its jolts go up AND down,
     so they cancel.  It does not depend on the hump state at all.
     "1 g" is a slow average of |a| (2^LIFT_BASE_SHIFT samples), learned
     during the first LIFT_WARMUP samples after power-on.
   - Near free fall (|a| < LOW_G) for LOW_G_CONFIRM samples is also a lift.
   - Overturn (> 30 deg) must last OVERTURN_CONFIRM: a real overturn stays.
   - The stop clears itself once the robot is set down level and still for
     SETTLE_SAMPLES; the motors then stay off until Start / Reset. */
#define S4_LIFT_VEL_MGS     1000 /* upward speed, mg x samples (~0.2 m/s) */
#define S4_LIFT_LEAK_SHIFT     5 /* speed estimate forgets in ~0.6 s */
#define S4_LIFT_CLAMP_MG     250 /* one sample adds at most this: a lift
                                    needs >= 5 samples (100 ms) of push */
#define S4_LIFT_BASE_SHIFT     6 /* "1 g" average over ~1.3 s */
#define S4_LIFT_WARMUP        50 /* 1 s after power-on: learn 1 g only */
#define S4_LIFT_LOW_G_MG     650 /* unloading / near free fall */
#define S4_LOW_G_CONFIRM       3 /* 60 ms in a row */
#define S4_SAFETY_UP_COS    866 /* cos(30 deg), scaled by 1000 */
#define S4_OVERTURN_CONFIRM  15  /* 300 ms tilted past 30 deg */
#define S4_SETTLE_MG          80 /* | |a| - 1 g | below this = set down */
#define S4_SETTLE_UP_COS     950 /* and within ~18 deg of level */
#define S4_SETTLE_SAMPLES     50 /* for 1 s: safety stop clears */

/* ---- motion classes ------------------------------------------------ */
#define S4_ACCEL_MG         120  /* forward accel for ACCEL/BRAKING */
#define S4_STILL_MG         25   /* | |a| - 1 g | below this = still */
#define S4_STILL_SAMPLES    25   /* 500 ms still = STATIONARY */
#define S4_TURN_RATE_CDEG_S 2500 /* 25 deg/s heading change = turning */

/* ---- I2C fault handling (FR8, NFR5: bounded recovery) ------------- */
#define S4_I2C_MAX_CONSEC 5  /* consecutive failures -> ERROR */
#define S4_REINIT_EVERY   50 /* in ERROR, re-init every 1 s */
#define S4_REINIT_MAX     5  /* then report a fatal IMU fault */

#endif /* S4_CONFIG_H */

/*** end of file ***/
