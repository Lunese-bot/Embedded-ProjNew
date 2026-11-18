/*
 *      live_tune.h
 *      Values the dashboard can change while the robot runs (bench only,
 *      S2_WEBDEBUG = 1).  They start at the config-file defaults, are lost
 *      on reboot, and are applied in the owning subsystem's own loop:
 *        g_lt_pid_kp100 / g_lt_pid_ki100   motor speed PID   (task_motion.c)
 *        g_lt_line_kp / g_lt_line_kd       line steering PD  (task_line.c)
 *        g_lt_speed_pct                    mission speed %   (vc_bridge_s2.c)
 *        g_lt_obst_en                      obstacle checking (vc_bridge_s5.c)
 *      With S2_WEBDEBUG = 0 this header is empty and nothing changes.
 */

#ifndef LIVE_TUNE_H
#define LIVE_TUNE_H

#include <stdint.h>
#include "../subsystem2/s2_config.h"

#if S2_WEBDEBUG
extern volatile int32_t g_lt_pid_kp100; /* S2_PID_KP * 100 */
extern volatile int32_t g_lt_pid_ki100; /* S2_PID_KI * 100 */
extern volatile int32_t g_lt_line_kp;   /* S3_KP_NUM */
extern volatile int32_t g_lt_line_kd;   /* S3_KD_NUM */
extern volatile int32_t g_lt_speed_pct; /* 100 = the controller's own speed */
extern volatile int32_t g_lt_line_sp;   /* S3_LINE_SETPOINT, -1000..+1000 */
extern volatile int32_t g_lt_line_all;  /* 1 = L, C and R steer; 0 = S3_LINE_MASK */
extern volatile int32_t g_lt_junc_mode; /* 0 off, 1 C+R, 2 L+C+R, -1 = config default */
extern volatile int32_t g_lt_junc_thr;  /* S3_JUNCTION_THRESHOLD, 0..1000 */
extern volatile int32_t g_lt_lost;      /* samples (10 ms) with no line before LOST */
extern volatile int32_t g_lt_steer_inv; /* 1 = swap left/right line steering */
extern volatile int32_t g_lt_bc_en;     /* 1 = mission reads barcodes */
extern volatile int32_t g_lt_imu_en;    /* 1 = mission uses the IMU events */
extern volatile int32_t g_lt_obst_en;   /* 1 = mission uses the obstacle sensor */
extern volatile int32_t g_lt_lost_en;   /* 1 = mission reacts to LINE_LOST */
extern volatile int32_t g_lt_dmin;     /* least duty while a wheel turns (bench), permille */
extern volatile int32_t g_lt_curve;    /* curve slow-down, permille at full steer */
extern volatile int32_t g_lt_ff_l;      /* left feed-forward, permille of S2_PID_KFF (1000 = config) */
extern volatile int32_t g_lt_ff_r;      /* right feed-forward, permille of S2_PID_KFF */
extern volatile int32_t g_lt_ilim;      /* PID integral limit, permille duty */
extern char             g_lt_bc_text[16]; /* last barcode seen by the mission bridge (dashboard) */
extern char             g_lt_bc_hist[48]; /* the last few results, newest first, space separated */
extern volatile int32_t g_lt_next_nav;  /* every junction: 0 = barcode decides, 1 left, 2 right, 3 straight */
extern volatile int32_t g_lt_bc_edge;   /* barcode: 1 = edge finder, 0 = fixed cut-off */
extern volatile int32_t g_lt_bc_swing;  /* S3_BC_EDGE_SWING (raw) */
extern volatile int32_t g_lt_bc_floor;  /* S3_BC_EDGE_FLOOR (raw) */
extern volatile int32_t g_lt_steer_hold; /* 0 steer normally, 1 straight while reading a barcode, 2 never steer */
extern volatile int32_t g_lt_trim_step; /* trim change per button press, permille */
#endif

#endif /* LIVE_TUNE_H */

/*** end of file ***/
