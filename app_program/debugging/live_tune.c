/*
 *      live_tune.c
 *      Storage for the live-tuning values (see live_tune.h).
 */

#include "live_tune.h"

#if S2_WEBDEBUG

#include "../subsystem3/s3_config.h"
#include "../vehicle_controller/vc_sm.h"

volatile int32_t g_lt_pid_kp100 = (int32_t)(S2_PID_KP * 100.0f);
volatile int32_t g_lt_pid_ki100 = (int32_t)(S2_PID_KI * 100.0f);
volatile int32_t g_lt_line_kp   = S3_KP_NUM;
volatile int32_t g_lt_line_kd   = S3_KD_NUM;
volatile int32_t g_lt_speed_pct = 100; /* 100 % = the tuned VC_SPEED_CRUISE_MMS */
volatile int32_t g_lt_line_sp   = S3_LINE_SETPOINT;
volatile int32_t g_lt_line_all  = 0;
volatile int32_t g_lt_junc_mode = 0;   /* start with junction detection OFF */
volatile int32_t g_lt_junc_thr  = S3_JUNCTION_THRESHOLD;
volatile int32_t g_lt_lost      = S3_LOST_SAMPLES;
volatile int32_t g_lt_steer_inv = 0;
volatile int32_t g_lt_bc_en     = 0;   /* start with barcode reading OFF */
volatile int32_t g_lt_imu_en    = 0;   /* start with IMU checking OFF */
volatile int32_t g_lt_obst_en   = 0;   /* start with obstacle checking OFF */
volatile int32_t g_lt_lost_en    = 0;  /* start with lost-line handling OFF */
volatile int32_t g_lt_dmin      = S2_DUTY_MIN_RUN;
volatile int32_t g_lt_curve     = VC_CURVE_SLOW_PERMILLE;
volatile int32_t g_lt_ff_l      = S2_PID_KFF_BOOST_L;
volatile int32_t g_lt_ff_r      = S2_PID_KFF_BOOST_R;
volatile int32_t g_lt_ilim      = (int32_t)S2_PID_I_LIMIT;
volatile int32_t g_lt_trim_step = 2;

#endif /* S2_WEBDEBUG */

/*** end of file ***/
