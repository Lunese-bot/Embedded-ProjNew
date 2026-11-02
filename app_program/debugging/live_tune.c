/*
 *      live_tune.c
 *      Storage for the live-tuning values (see live_tune.h).
 */

#include "live_tune.h"

#if S2_WEBDEBUG

#include "../subsystem3/s3_config.h"

volatile int32_t g_lt_pid_kp100 = (int32_t)(S2_PID_KP * 100.0f);
volatile int32_t g_lt_pid_ki100 = (int32_t)(S2_PID_KI * 100.0f);
volatile int32_t g_lt_line_kp   = S3_KP_NUM;
volatile int32_t g_lt_line_kd   = S3_KD_NUM;
volatile int32_t g_lt_speed_pct = 100;

#endif /* S2_WEBDEBUG */

/*** end of file ***/
