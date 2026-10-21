/*
 *	line_algo.h
 *	Subsystem 3 -- line-position estimation, steering and line state.
 *
 *	Pure C, no kernel calls, so it is unit-tested on the PC
 *	(testing/subsystem3/host/) as well as on the robot.  task_line.c
 *	owns the sensors and calls these functions every period.
 */

#ifndef LINE_ALGO_H
#define LINE_ALGO_H

#include <stdint.h>
#include <stdbool.h>

/* No <stddef.h>: the kernel headers define their own size_t */
#ifndef NULL
#define NULL    ((void *)0)
#endif
#include "s3_config.h"
#include "../robot_types.h"

#define LINE_NORM_MAX       1000u   /* normalised reading: 1000 = black */
#define LINE_POS_MAX        1000
#define LINE_STEER_MAX      1000

/* Calibration: min and max raw reading seen for each sensor */
typedef struct
{
    uint16_t min[S3_IR_COUNT];
    uint16_t max[S3_IR_COUNT];
    bool     b_valid;
} line_cal_t;

typedef struct
{
    uint8_t  line_mask;          /* sensors used for the position    */
    uint8_t  junction_mask;      /* sensors all black = junction     */
    int16_t  weight[S3_IR_COUNT];
    int16_t  setpoint;
    uint16_t black_threshold;    /* 0..1000                          */
    uint16_t lost_samples;
    uint16_t found_samples;
    uint16_t junction_samples;
    int16_t  kp_num;
    int16_t  kd_num;
    int16_t  gain_den;
} line_params_t;

typedef enum
{
    LINE_EVT_NONE = 0,
    LINE_EVT_LOST,          /* FOLLOW -> LOST            */
    LINE_EVT_FOUND,         /* LOST/SEARCH -> FOLLOW     */
    LINE_EVT_JUNCTION       /* entered JUNCTION          */
} line_evt_t;

typedef struct
{
    LineState state;         /* robot_types.h                       */
    int16_t   position;      /* -1000 left .. +1000 right           */
    int16_t   error;         /* position - setpoint                 */
    int16_t   steering;      /* -1000 .. +1000, + = steer right     */
    int8_t    last_side;     /* -1 left, +1 right, 0 unknown        */
    uint8_t   black_mask;    /* bit per sensor on black (S3_MASK_x) */
    uint16_t  lost_count;
    uint16_t  found_count;
    uint16_t  junction_count;
    int16_t   prev_error;
} line_tracker_t;

/* Default parameters from s3_config.h */
void line_params_default(line_params_t * p_params);

/* Calibration */
void line_cal_reset(line_cal_t * p_cal);
void line_cal_update(line_cal_t * p_cal, uint16_t const p_raw[S3_IR_COUNT]);
bool line_cal_finish(line_cal_t * p_cal, uint8_t used_mask,
                     uint16_t min_span);

/*
 * Raw readings -> 0 (white) .. 1000 (black).
 * Digital inputs (raw 0/1) use black_level; analogue ones use the
 * calibration (or pass through 0..4095 scaled if not calibrated).
 */
void line_normalise(line_cal_t const * p_cal, uint8_t analog_mask,
                    bool b_black_is_high, uint8_t digital_black_level,
                    uint16_t const p_raw[S3_IR_COUNT],
                    uint16_t p_norm[S3_IR_COUNT]);

/* Tracker */
void       line_tracker_reset(line_tracker_t * p_trk);
line_evt_t line_tracker_step(line_tracker_t * p_trk,
                             line_params_t const * p_params,
                             uint16_t const p_norm[S3_IR_COUNT],
                             bool b_junction_inhibit);

#endif /* LINE_ALGO_H */
