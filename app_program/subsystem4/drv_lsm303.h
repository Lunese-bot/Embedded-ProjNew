/*
 *      drv_lsm303.h
 *      Subsystem 4 -- LSM303DLHC (or LSM303D, auto-detected) accel + mag
 * driver.
 *
 *      Uses the kernel's own I2C driver ("iica" = I2C0) through the
 *      micro T-Kernel device API (tk_opn_dev / tk_swri_dev).
 *      Call ONLY from imu_task, the owning task of I2C0 (TEAM.md section 3).
 *
 *      The LSM303DLHC has NO gyroscope (design review 8.3): turn rate comes
 *      from the change in magnetometer heading.
 */

#ifndef DRV_LSM303_H
#define DRV_LSM303_H

#include "../app.h"

/* 7-bit I2C addresses */
#define LSM303_ACC_ADDR 0x19
#define LSM303_MAG_ADDR 0x1E

/* Raw readings in SENSOR axes */
typedef struct
{
    int16_t ax_mg; /* +-2 g, high-resolution: 1 mg per LSB */
    int16_t ay_mg;
    int16_t az_mg;
    int16_t mx; /* +-1.3 gauss range, raw counts */
    int16_t my;
    int16_t mz;
} lsm303_raw_t;

/* Open "iica" (if not yet open), configure both sensors, check the
   magnetometer ID.  Safe to call again to recover after I2C errors. */
extern ER lsm303_init(void);

/* Read one accelerometer + magnetometer sample */
extern ER lsm303_read(lsm303_raw_t * p_raw);

/* Close the device (used before a full re-init) */
extern void    lsm303_close(void);
extern int32_t lsm303_scan(void); /* bring-up: list I2C0 devices */

#endif /* DRV_LSM303_H */

/*** end of file ***/
