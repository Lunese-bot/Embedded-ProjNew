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
volatile int32_t g_lt_line_sp   = S3_LINE_SETPOINT;
volatile int32_t g_lt_line_all  = 0;
volatile int32_t g_lt_junc_mode = -1;
volatile int32_t g_lt_junc_thr  = S3_JUNCTION_THRESHOLD;
volatile int32_t g_lt_lost      = S3_LOST_SAMPLES;
volatile int32_t g_lt_steer_inv = 0;
volatile int32_t g_lt_bc_en     = 1;
volatile int32_t g_lt_imu_en    = 1;
volatile int32_t g_lt_obst_en   = 1;
volatile int32_t g_lt_ff_l      = 1000;
volatile int32_t g_lt_ff_r      = 1000;
volatile int32_t g_lt_ilim      = (int32_t)S2_PID_I_LIMIT;
volatile int32_t g_lt_trim_step = 2;

#endif /* S2_WEBDEBUG */

/*** end of file ***/
