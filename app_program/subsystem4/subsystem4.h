/*
 *      subsystem4.h
 *      Subsystem 4 -- IMU-Based Motion and Terrain Monitoring (Buddy 4).
 *      Student ID: 2500366.
 *
 *      Hardware: LSM303DLHC (accelerometer + magnetometer, no gyro) on I2C0,
 *      SDA = GP0, SCL = GP1 (design review section 2).
 *      Task:     imu_task, periodic 20 ms (NFR2), priority 18 (band 18-19).
 *
 *      Public interface of subsystem 4.  Other subsystems include it as
 *      "../subsystem4/subsystem4.h".  Everything here is safe to call from
 *      any task on either core; none of it may be called from a handler.
 *
 *        IF-S4-1  S4 -> Vehicle Controller   message buffer g_mbf_imu_evt
 *                 imu_event_msg_t per terrain/safety event (owned by S4)
 *        IF-S4-2  S4 -> anyone (VC, telemetry) s4_get_status()
 *                 copy of imu_status_t taken under S4's mutex
 *        IF-S4-3  Motion control -> S4       s4_set_speed_mm_s()
 *                 wheel speed, turns pitch into hump height
 *        IF-S4-4  VC / MQTT -> S4            s4_request_calibration(),
 *                 s4_request_peak_reset()
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "subsystem4_cfg.h"
#include "imu_terrain.h"

#define S4_PRI_IMU 18 /* priority band 18-19 (TEAM.md) */

/* ---- IF-S4-1: events to the Vehicle Controller ---------------------
 * Receive with:
 *      imu_event_msg_t ev;
 *      if (tk_rcv_mbf(g_mbf_imu_evt, &ev, TMO_POL) == sizeof(ev)) { ... }
 * Severity follows FR8; the Vehicle Controller makes the final call. */
typedef enum
{
    IMU_SEV_INFO = 0,    /* hump, motion change: log / telemetry */
    IMU_SEV_NONCRITICAL, /* report, keep navigating */
    IMU_SEV_RECOVERABLE, /* S4 is retrying on its own (bounded) */
    IMU_SEV_CRITICAL     /* suggests Safe Stop (impact, tilt) */
} imu_severity_t;

typedef enum
{
    IMU_EVT_CAL_DONE = 1, /* value = 0 */
    IMU_EVT_CAL_FAILED,   /* value = 0, robot moved: retrying */
    IMU_EVT_HUMP_START,   /* value = 0 */
    IMU_EVT_HUMP_PEAK,    /* value = height so far, um */
    IMU_EVT_HUMP_END,     /* value = peak height of this hump, um */
    IMU_EVT_HUMP_ABORT,   /* value = 0 */
    IMU_EVT_IMPACT,       /* value = |a| - 1 g, mg */
    IMU_EVT_TILT_LIMIT,   /* value = pitch cdeg (roll if larger) */
    IMU_EVT_MOTION,       /* value = imu_motion_t */
    IMU_EVT_FAULT,        /* value = kernel error code; retrying */
    IMU_EVT_FAULT_FATAL,  /* value = kernel error code; gave up */
    IMU_EVT_RECOVERED     /* value = 0, IMU back after a fault */
} imu_event_type_t;

typedef struct
{
    uint8_t  type;     /* imu_event_type_t */
    uint8_t  severity; /* imu_severity_t */
    uint16_t seq;      /* increments per event, spots drops */
    int32_t  value;    /* see imu_event_type_t */
    uint32_t time_ms;  /* tk_get_tim low word */
} imu_event_msg_t;

#define S4_EVT_QUEUE_LEN 8 /* messages the buffer can hold */

extern ID g_mbf_imu_evt;

/* ---- IF-S4-2: status snapshot -------------------------------------- */
typedef imu_status_t imu_status_t; /* fields: see imu_terrain.h */

extern ER s4_get_status(imu_status_t * p_out);

/* ---- IF-S4-3 / IF-S4-4: inputs to subsystem 4 ---------------------- */
extern void s4_set_speed_mm_s(int32_t speed_mm_s); /* -1 = unknown */
extern ER   s4_request_calibration(void); /* robot must be still, level */
extern ER   s4_request_peak_reset(void);  /* new run: clear max hump */

/* ---- private to subsystem 4 (shared between its own .c files) ------ */
#define S4_REQ_CALIBRATE  0x0001
#define S4_REQ_PEAK_RESET 0x0002

extern ID               g_tid_imu;
extern ID               g_cyc_imu;
extern ID               g_mtx_imu;
extern ID               g_flg_imu_req;
extern imu_status_t     g_s4_status; /* guarded by g_mtx_imu */
extern volatile int32_t g_s4_speed_mm_s;
extern uint32_t         g_s4_evt_dropped; /* events lost: buffer full */
extern uint32_t         g_s4_overruns;    /* periods missed by imu_task */
extern uint32_t         g_s4_i2c_errors;

extern void imu_task(int32_t stacd, void * p_exinf);
extern void imu_cyclic_isr(void * p_exinf);

#endif /* SUBSYSTEM4_H */

/*** end of file ***/
