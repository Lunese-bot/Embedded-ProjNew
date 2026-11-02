/*
 *      live_tune.h
 *      Values the dashboard can change while the robot runs (bench only,
 *      S2_WEBDEBUG = 1).  They start at the config-file defaults, are lost
 *      on reboot, and are applied in the owning subsystem's own loop:
 *        g_lt_pid_kp100 / g_lt_pid_ki100   motor speed PID   (task_motion.c)
 *        g_lt_line_kp / g_lt_line_kd       line steering PD  (task_line.c)
 *        g_lt_speed_pct                    mission speed %   (vc_bridge_s2.c)
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
#endif

#endif /* LIVE_TUNE_H */

/*** end of file ***/
