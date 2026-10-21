/*
 *	subsystem4_cfg.h
 *	Subsystem 4 -- every tunable number in one place.
 *
 *	All thresholds are FIRST GUESSES.  Assumption A5 in the design
 *	review says they must be tuned from logged data over flat ground
 *	and the real humps; record the final values in
 *	testing/subsystem4/TEST_LOG.md together with the evidence.
 *
 *	Pure macros (no kernel types): also used by the host tests.
 */

#ifndef SUBSYSTEM4_CFG_H
#define SUBSYSTEM4_CFG_H

/* ---- timing (NFR2: IMU every 20 ms) -------------------------------- */
#define S4_PERIOD_MS		20

/* ---- mounting: robot axis = sign * sensor axis ---------------------
 * Robot axes: X forward, Y left, Z up.  Set these after looking at
 * which way the LSM303DLHC board is mounted on the chassis.  Check with
 * test S4-M02 (tilt the robot nose-up: pitch must go positive). */
#define S4_ACC_X_SRC		0	/* 0 = sensor X, 1 = Y, 2 = Z */
#define S4_ACC_Y_SRC		1
#define S4_ACC_Z_SRC		2
#define S4_ACC_X_SIGN		(+1)
#define S4_ACC_Y_SIGN		(+1)
#define S4_ACC_Z_SIGN		(+1)
#define S4_MAG_X_SIGN		(+1)	/* magnetometer uses the same */
#define S4_MAG_Y_SIGN		(+1)	/* source axes as the accel     */
#define S4_MAG_Z_SIGN		(+1)

/* ---- filter and calibration --------------------------------------- */
#define S4_LPF_SHIFT		2	/* EMA alpha = 1/4  (~70 ms at 50 Hz) */
#define S4_CAL_SAMPLES		50	/* 1 s of still samples */
#define S4_CAL_MAX_SPREAD_MG	60	/* moved during cal if spread > this */

/* ---- hump detection ------------------------------------------------ */
#define S4_HUMP_ENTER_CDEG	600	/* 6.0 deg nose up/down = slope */
#define S4_HUMP_LEVEL_CDEG	250	/* |pitch| < 2.5 deg = level */
#define S4_HUMP_CONFIRM		3	/* 60 ms to confirm a transition */
#define S4_HUMP_TIMEOUT		500	/* 10 s in one hump state = abort */
#define S4_HUMP_MIN_PEAK_UM	3000	/* < 3 mm is noise, not a hump */
#define S4_NOMINAL_SPEED_MM_S	150	/* used when motion speed unknown */

/* ---- impact / safety ----------------------------------------------- */
#define S4_IMPACT_MG		900	/* | |a| - 1 g | above this = impact */
#define S4_IMPACT_HOLDOFF	25	/* 500 ms between impact events */
#define S4_TILT_LIMIT_CDEG	3000	/* 30 deg pitch or roll = unsafe */

/* ---- motion classes ------------------------------------------------ */
#define S4_ACCEL_MG		120	/* forward accel for ACCEL/BRAKING */
#define S4_STILL_MG		25	/* | |a| - 1 g | below this = still */
#define S4_STILL_SAMPLES	25	/* 500 ms still = STATIONARY */
#define S4_TURN_RATE_CDEG_S	2500	/* 25 deg/s heading change = turning */

/* ---- I2C fault handling (FR8, NFR5: bounded recovery) ------------- */
#define S4_I2C_MAX_CONSEC	5	/* consecutive failures -> ERROR */
#define S4_REINIT_EVERY		50	/* in ERROR, re-init every 1 s */
#define S4_REINIT_MAX		5	/* then report a fatal IMU fault */

#endif /* SUBSYSTEM4_CFG_H */
