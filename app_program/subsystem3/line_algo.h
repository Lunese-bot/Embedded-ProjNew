/*
 *      line_algo.h
 *      Subsystem 3 -- line-position estimation, steering and line state.
 *
 *      Pure C, no kernel calls, so it is unit-tested on the PC
 *      (testing/subsystem3/host_test/) as well as on the robot.  task_line.c
 *      owns the sensors and calls these functions every period.
 */

#ifndef LINE_ALGO_H
#define LINE_ALGO_H

#include <stdint.h>
#include <stdbool.h>

/* No <stddef.h>: the kernel headers define their own size_t */
#ifndef NULL
#define NULL ((void *)0)
#endif
#include "s3_config.h"
#include "../robot_types.h"

#define LINE_NORM_MAX  1000u /* normalised reading: 1000 = black */
#define LINE_POS_MAX   1000
#define LINE_STEER_MAX 1000

/* Calibration: min and max raw reading seen for each sensor */
typedef struct
{
    uint16_t min[S3_IR_COUNT];
    uint16_t max[S3_IR_COUNT];
    bool     b_valid;
} line_cal_t;

typedef struct
{
    uint8_t  line_mask;     /* sensors used for the position    */
    uint8_t  junction_mask; /* sensors all black = junction     */
    int16_t  weight[S3_IR_COUNT];
    int16_t  setpoint;
    uint16_t black_threshold; /* 0..1000                          */
    uint16_t lost_samples;
    uint16_t found_samples;
    uint16_t junction_samples;
    int16_t  kp_num;
    int16_t  kd_num;
    int16_t  gain_den;
    uint16_t junction_threshold; /* 0..1000, stricter than black_threshold */
    bool     straddle; /* line runs between C and R: both white = straight */
} line_params_t;

typedef enum
{
    LINE_EVT_NONE = 0,
    LINE_EVT_LOST,    /* FOLLOW -> LOST            */
    LINE_EVT_FOUND,   /* LOST/SEARCH -> FOLLOW     */
    LINE_EVT_JUNCTION /* entered JUNCTION          */
} line_evt_t;

typedef struct
{
    line_state_t state;      /* robot_types.h                       */
    int16_t      position;   /* -1000 left .. +1000 right           */
    int16_t      error;      /* position - setpoint                 */
    int16_t      steering;   /* -1000 .. +1000, + = steer right     */
    int8_t       last_side;  /* -1 left, +1 right, 0 unknown        */
    uint8_t      black_mask; /* bit per sensor on black (S3_MASK_x) */
    uint16_t     lost_count;
    uint16_t     found_count;
    uint16_t     junction_count;
    uint16_t     rejoin_clear_left; /* clear samples before rearming junctions */
    int16_t      prev_error;
    int8_t       corr_side;  /* straddle: side being corrected, 0 = none */
    uint16_t     corr_count; /* straddle: samples that side has been black */
    int16_t      corr_mag;   /* straddle: last correction strength        */
    uint16_t     hold_count; /* straddle: samples of curve memory so far   */
    bool         b_hold;     /* straddle: line passed outside, keep turning */
    bool         b_reentry;  /* straddle: line came back in after a hold    */
    int8_t       mem_side;   /* straddle: side corr_count belongs to (kept over short white gaps) */
    uint16_t     l_raw;      /* L raw reading, set by the caller before each step (barcode-bar check) */
} line_tracker_t;

/* Default parameters from s3_config.h */
void line_params_default(line_params_t * p_params);

/* Calibration */
void line_cal_reset(line_cal_t * p_cal);
void line_cal_update(line_cal_t * p_cal, uint16_t const p_raw[S3_IR_COUNT]);
bool line_cal_finish(line_cal_t * p_cal, uint8_t used_mask, uint16_t min_span);

/*
 * Raw readings -> 0 (white) .. 1000 (black).
 * Digital inputs (raw 0/1) use black_level; analogue ones use the
 * calibration (or pass through 0..4095 scaled if not calibrated).
 */
void line_normalise(line_cal_t const * p_cal, uint8_t analog_mask,
                    bool b_black_is_high, uint8_t digital_black_level,
                    uint16_t const p_raw[S3_IR_COUNT],
                    uint16_t       p_norm[S3_IR_COUNT]);

/* Tracker */
void       line_tracker_reset(line_tracker_t * p_trk);
line_evt_t line_tracker_step(line_tracker_t *      p_trk,
                             line_params_t const * p_params,
                             uint16_t const        p_norm[S3_IR_COUNT],
                             bool                  b_junction_inhibit);


/*
 * Barcode edge finder (S3_BC_EDGE = 1).  Works on L's RAW reading and
 * looks for rises and falls instead of a fixed cut-off, so a thin white
 * gap counts even when L never gets back to real white.
 *   black: follow the highest reading; turn white once L drops `swing`
 *          below it (or under `floor`).
 *   white: follow the lowest reading; turn black once L rises `swing`
 *          above it AND is above `floor`.
 * The edge time is when L crossed halfway between this bar's peak and
 * the gap's valley (50 %), so a weak, blurry bump is not timed narrower
 * than its gaps and rising / falling edges are timed the same way.
 */
typedef struct
{
    uint8_t  level;    /* 1 = black */
    int32_t  ext;      /* peak (black) or valley (white) so far */
    int32_t  prev;     /* the previous valley (black) or peak (white) */
    uint8_t  b_prev;   /* 0 = no previous peak yet (first bar): use swing/2 */
    uint32_t mid_t_us; /* last time L was on this side of halfway */
    uint32_t mid_pos;  /* wheel position (encoder counts) at that time */
    /* Start of the current bar, found the same 50 % way as its end: the
       last sample below halfway between the bar's own peak and the white
       before it.  (The edge reported when the bar began had to use the
       PREVIOUS bar's peak, or a guess for the first bar.)  When a bar
       ends, bar_t_us / bar_pos say where it began: use them for its width. */
    uint32_t bar_t_us;
    uint32_t bar_pos;
    int32_t  last_raw; /* the sample before this one */
    uint32_t last_t_us;
    uint32_t last_pos;
    /* Weak print (2026-11-11): a change of level only counts once `hold`
       samples in a row ask for it (0 or 1 = at once).  Set by the caller;
       the edge is still placed at the 50 % point, so holding adds no shift. */
    uint8_t  hold;
    uint8_t  hold_cnt;
    /* Learned white (S3_BC_EDGE_ADAPT, bc_edge_step_adapt): L's level and
       noise on white paper, raw x 16.  Learned from a steady stretch,
       updated on white outside a read, frozen during one. */
    uint8_t  w_valid;   /* 0 = still learning: no bars are found       */
    uint8_t  w_relearn; /* 1 = learn again at the next sample           */
    uint16_t w_n;       /* samples in the learning window               */
    int32_t  w_min;     /* learning window: lowest, highest, sum        */
    int32_t  w_max;
    int32_t  w_sum;
    int32_t  w_x16;     /* white level x 16                              */
    int32_t  w_sd_x16;  /* white noise (standard deviation) x 16         */
    uint16_t w_high;    /* samples black in a row outside a read         */
} bc_edge_t;

void bc_edge_reset(bc_edge_t * p_e, int32_t raw, uint32_t t_us);
void bc_edge_reset_pos(bc_edge_t * p_e, int32_t raw, uint32_t t_us,
                       uint32_t pos);

/* Returns 1 when the level changed; *p_edge_us = when the edge happened. */
uint8_t bc_edge_step(bc_edge_t * p_e, int32_t raw, uint32_t t_us,
                     int32_t swing, int32_t floor, uint32_t * p_edge_us);

/* Same, also tracking the wheel position `pos` (any unit that counts up
   with distance, e.g. encoder counts): *p_edge_pos = where the edge was,
   found the same 50 % way as its time. */
uint8_t bc_edge_step_pos(bc_edge_t * p_e, int32_t raw, uint32_t t_us,
                         uint32_t pos, int32_t swing, int32_t floor,
                         uint32_t * p_edge_us, uint32_t * p_edge_pos);

/*
 * Adaptive edge finder (S3_BC_EDGE_ADAPT): the same rises and falls, but
 * the thresholds come from L's learned white level W and noise sd:
 *   m = S3_BC_NOISE_K x sd + S3_BC_NOISE_ADD  (G7's 3 sd + 8, a candidate)
 *   a bar starts once L is m above the gap's valley AND above W + m;
 *   a bar ends once L falls max(m, S3_BC_EDGE_FALL_PCT of the bar's
 *   height) below its peak, or below W + m / 2.
 * White is learned from S3_BC_WHITE_INIT_N samples in a row that stay
 * within S3_BC_WHITE_INIT_SPREAD; no bar is found before that.  It is
 * then updated on white samples within m of W while b_freeze is false
 * (no read open), follows L down quickly when L sits below W - m outside
 * a read, and is learned again (a bar still open is ended first) after
 * bc_edge_relearn(), bc_edge_reset_pos(), a gap of more than
 * S3_BC_WHITE_STALE_US between samples, or S3_BC_WHITE_RELEARN_SAMPLES
 * of black outside a read.  Returns 1 when the level changed, like
 * bc_edge_step_pos().
 */
uint8_t bc_edge_step_adapt(bc_edge_t * p_e, int32_t raw, uint32_t t_us,
                           uint32_t pos, bool b_freeze, uint32_t * p_edge_us,
                           uint32_t * p_edge_pos);
/* learn white again from the next sample (e.g. after a barcode rearm) */
void bc_edge_relearn(bc_edge_t * p_e);
/* m above (raw); 0 while white is not learned */
int32_t bc_edge_margin(bc_edge_t const * p_e);

/*
 * Barcode lock (S3_BC_LOCK, MY 2026-11-12): from the first bar of a
 * barcode until the robot is past it, drive straight at one slow speed
 * instead of steering (and slowing for "curves") at every bar.
 *   - locks when a read starts (barcode_task's first bar);
 *   - lets go if no second bar comes within first_max counts (one long
 *     black is a junction arm or the line on a curve, not a barcode);
 *   - after a read ends, stays locked for `after` counts, so the next
 *     piece of the same barcode is read at the same speed;
 *   - never longer than max_counts in one go.
 * pos: wheel counts that only go up (s3_bc_pos()).
 */
typedef struct
{
    bool     b_lock;      /* drive straight and slow now               */
    bool     b_confirmed; /* a read in this lock saw bar + space        */
    bool     b_was_scan;
    bool     b_after;     /* read over, counting `after`                */
    uint32_t start_pos;   /* where this lock began                      */
    uint32_t read_pos;    /* where the current read began               */
    uint32_t end_pos;     /* where the last read ended                  */
} bc_lock_t;

void bc_lock_reset(bc_lock_t * p_l);
/* One 10 ms step; returns p_l->b_lock */
bool bc_lock_step(bc_lock_t * p_l, bool b_scan, bool b_bars, uint32_t pos,
                  uint32_t first_max, uint32_t after, uint32_t max_counts);

/* Steer straight this sample and forget any turn being built up */
void line_tracker_go_straight(line_tracker_t * p_trk);

/* While the barcode lock is on, keep following the line, but only when
   ONE of C / R has been black for side_counts of travel (the robot really
   is drifting off the line).  A bar under both, or the short moment only
   one of them is over a bar edge, gives no steering.  Steering is capped
   at +-max_steer.  Returns the steering to use; when it returns 0 the
   caller also calls line_tracker_go_straight() so no turn builds up
   (bc_lock_apply() below does this, and never while a curve is held). */
typedef struct
{
    int8_t   side;     /* -1 C only, +1 R only, 0 none or both */
    uint32_t side_pos; /* where that started                   */
} bc_lock_steer_t;

void    bc_lock_steer_reset(bc_lock_steer_t * p_s);
int16_t bc_lock_steer(bc_lock_steer_t * p_s, uint8_t black_mask, uint32_t pos,
                      int16_t steering, uint32_t side_counts,
                      int16_t max_steer);

/*
 * Lock + tracker glue (review R1, 2026-11-14).  Call once per 10 ms
 * sample AFTER line_tracker_step().  Returns the lock state (the VC caps
 * the speed while it is true) and may change p_trk->steering:
 *   - not locked: tracker untouched;
 *   - locked, not yet confirmed (no bar + space seen): tracker untouched,
 *     speed cap only.  A false start on a curve (L sees the line) then
 *     costs nothing but speed;
 *   - confirmed, tracker holding a curve (b_hold): tracker untouched, so
 *     the curve memory keeps turning and is never cleared;
 *   - confirmed, no curve held: bc_lock_steer() rules; when it gives 0
 *     the tracker is told to go straight (only bar edges are zeroed).
 */
typedef struct
{
    bc_lock_t       lock;
    bc_lock_steer_t steer;
} bc_lock_ctl_t;

typedef struct
{
    uint32_t first_max;   /* S3_BC_LOCK_FIRST_COUNTS */
    uint32_t after;       /* S3_BC_LOCK_AFTER_COUNTS */
    uint32_t max_counts;  /* S3_BC_LOCK_MAX_COUNTS   */
    uint32_t side_counts; /* S3_BC_LOCK_SIDE_COUNTS  */
    int16_t  max_steer;   /* S3_BC_LOCK_STEER_MAX    */
} bc_lock_cfg_t;

/* 500 Hz L front end (S3_L_FAST): hand-off from the 2 ms interrupt
   (one writer) to line_task (one reader) without a lock.  The writer adds
   each 2 ms sample to running totals; the reader takes the totals and
   uses the change since its last read, so no sample is used twice or
   lost.  seq is odd while the writer is in the middle of an update. */
typedef struct
{
    volatile uint32_t seq;
    volatile uint32_t sum; /* sum of all 2 ms L samples (wraps)  */
    volatile uint32_t n;   /* number of 2 ms samples (wraps)      */
    volatile uint16_t c;   /* newest C conversion                 */
    volatile uint16_t r;   /* newest R conversion                 */
} l_fast_t;

typedef struct
{
    uint32_t sum;  /* totals at the last record */
    uint32_t n;
    uint16_t last; /* last L record             */
} l_fast_rd_t;

void l_fast_reset(l_fast_t * p_f, l_fast_rd_t * p_rd);
/* interrupt side: one 2 ms sample */
void l_fast_put(l_fast_t * p_f, uint16_t l, uint16_t c, uint16_t r);
/* task side: a consistent copy; false if no stable copy in a few tries */
bool l_fast_get(const l_fast_t * p_f, uint32_t * p_sum, uint32_t * p_n,
                uint16_t * p_c, uint16_t * p_r);
/* task side: the 10 ms L record = mean of the samples since the last
   record.  *p_used = how many (0: none came, the last record is kept). */
uint16_t l_fast_record(l_fast_rd_t * p_rd, uint32_t sum, uint32_t n,
                       uint32_t * p_used);

void bc_lock_ctl_reset(bc_lock_ctl_t * p_c);
bool bc_lock_apply(bc_lock_ctl_t * p_c, bc_lock_cfg_t const * p_cfg,
                   line_tracker_t * p_trk, bool b_scan, bool b_bars,
                   uint32_t pos);

#endif /* LINE_ALGO_H */

/*** end of file ***/
