/*
 *      subsystem3.h
 *      Subsystem 3 -- IR Line Following and Barcode Navigation.
 *      Owner: Buddy 3 (Student ID 2500193, Design Review).
 *
 *      Public interface of subsystem 3.  Other subsystems include it as
 *      "../subsystem3/subsystem3.h".  Everything here is proposed as
 *      interfaces IF-3x in TEAM.md section 5 (see TEAM_S3_ROWS.md).
 *
 *      What subsystem 3 gives the rest of the robot
 *      --------------------------------------------
 *        IF-31  line status snapshot  s3_get_line_status()   any task, any time
 *               (position, tracking error, PD steering request, line state)
 *        IF-32  event stream          g_mbf_s3_events          Vehicle
 * Controller (line lost/found, junction, barcode start/decoded/failed,
 *                calibration result)  one s3_event_t per message
 *        IF-33  requests              s3_request()           Vehicle Controller
 *               (calibrate, reset tracker, barcode on/off, re-arm barcode)
 *        IF-S5d s5_port_line_detected()  (s3_ports.c)   Subsystem 5, after a
 *               bypass: TRUE when any IR sensor sees the line
 *
 *      Subsystem 3 never drives the motors.  It only REQUESTS steering and
 *      navigation; the Vehicle Controller decides and tells Motion Control
 *      (Design Review 8.3: the controller is the sole movement authority).
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "../robot_types.h"
#include "s3_config.h"

/* ---------------------------------------------------------------------
 *  Tasks (priority band 16-17, TEAM.md section 4)
 * --------------------------------------------------------------------- */
#define S3_PRI_LINE    16 /* line_task:    10 ms, owns IR_L/C/R (+ADC) */
#define S3_PRI_BARCODE 17 /* barcode_task: event driven, decodes      */

/* ---------------------------------------------------------------------
 *  IF-31: line status.  Copy it with s3_get_line_status(); never read
 *  g_s3_status directly (it is updated every 10 ms under a mutex).
 * --------------------------------------------------------------------- */
typedef struct
{
    uint32_t        seq;              /* incremented every line sample        */
    uint32_t        time_ms;          /* tk_get_tim() of the sample           */
    line_state_t    line_state;       /* robot_types.h                        */
    int16_t         position;         /* -1000 left .. +1000 right            */
    int16_t         error;            /* position - S3_LINE_SETPOINT          */
    int16_t         steering;         /* PD request -1000..+1000, + = right   */
    int8_t          last_side;        /* -1 left, +1 right, 0 unknown         */
    uint8_t         black_mask;       /* S3_MASK_L/C/R on black               */
    uint16_t        raw[S3_IR_COUNT]; /* raw readings (0/1 or ADC)    */
    bool            calibrated;       /* ADC calibration valid                */
    barcode_state_t bc_state;         /* robot_types.h                        */
    bool            bc_enabled;
    uint32_t        bc_ok;   /* barcodes decoded                     */
    uint32_t        bc_fail; /* barcode reads that failed            */
    uint16_t        bc_last_elems; /* bars + spaces in the last read (one
                                      letter between '*' markers = 29) */
    uint8_t         bc_last_err;   /* code39_result_t of the last read, 0 = ok */
    uint32_t evt_dropped;    /* events lost: g_mbf_s3_events was full        */
} s3_line_status_t;

extern ER s3_get_line_status(s3_line_status_t * p_out);

/* Bench analysis (dashboard barcode recorder): a copy of the bar and space
   widths of the LAST decode attempt, written by barcode_task.  Element 0 is
   a black bar, then they alternate.  g_s3_bc_dbg_seq goes up by one after
   each copy; a reader copies n widths, then checks seq did not change. */
extern volatile uint32_t g_s3_bc_dbg_seq;
extern volatile uint32_t g_s3_bc_dbg_n;
extern volatile uint32_t g_s3_bc_dbg_us[S3_BC_MAX_ELEMENTS];
/* the same elements measured by wheel distance (encoder counts, L + R),
   and which one the decoder used: 1 = distance, 0 = time, 2 = shape read
   (S3_BC_GEOM, code39_geom.c) */
extern volatile uint32_t g_s3_bc_dbg_pos[S3_BC_MAX_ELEMENTS];
extern volatile uint32_t g_s3_bc_dbg_unit;
/* Shape read of the same attempt (written before g_s3_bc_dbg_seq goes
   up): result c39g_result_t (0 = letter found, 255 = not tried), best
   letter (also when refused), worst wide-bar error and margin over the
   next letter in 1/100, share of the trace explained in %. */
extern volatile uint8_t  g_s3_bc_geom_res;
extern volatile char     g_s3_bc_geom_best;
extern volatile uint16_t g_s3_bc_geom_err_x100;
extern volatile uint16_t g_s3_bc_geom_margin_x100;
extern volatile uint8_t  g_s3_bc_geom_fit;
extern volatile uint32_t g_s3_bc_geom_ok; /* letters found by it */
extern volatile uint16_t g_s3_bc_geom_ms; /* how long it took */
/* Wheel position used for barcode widths (S3_BC_DIST_WHEEL), in counts of
   about 0.15 mm; 0 when S3_BC_USE_DIST is off.  task_line.c */
extern uint32_t s3_bc_pos(void);

/* Bench analysis: the barcode sensor's raw reading for the last
   S3_LTRACE_N line samples (10 ms each), a ring written by line_task.
   g_s3_ltrace_idx counts samples written (next slot = idx % N). */
#define S3_LTRACE_N 700u /* 7 s (was 400: a slow pass did not fit) */
extern volatile uint16_t g_s3_ltrace[S3_LTRACE_N];
extern volatile uint32_t g_s3_ltrace_idx;
/* Same samples: how far the wheels went since the sample before, in
   encoder counts.  _fwd = left + right (forward travel), _yaw = right -
   left (+ = the robot is twisting to the left).  Shows whether the robot
   twists while L crosses a barcode. */
extern volatile uint8_t g_s3_ltrace_fwd[S3_LTRACE_N];
extern volatile int8_t  g_s3_ltrace_yaw[S3_LTRACE_N];
/* Copy of the trace taken by barcode_task the moment a read ends (oldest
   first), so the bars are still in it when the dashboard asks. */
extern volatile uint16_t g_s3_ltrace_frz[S3_LTRACE_N];
extern volatile uint8_t  g_s3_ltrace_frz_fwd[S3_LTRACE_N];
extern volatile int8_t   g_s3_ltrace_frz_yaw[S3_LTRACE_N];
extern volatile uint32_t g_s3_ltrace_frz_n; /* samples in the copy, 0 = none yet */

/* ---------------------------------------------------------------------
 *  IF-32: events, received by the Vehicle Controller:
 *      s3_event_t evt;
 *      INT sz = tk_rcv_mbf(g_mbf_s3_events, &evt, TMO_POL);  (or a timeout)
 *      if (sz == sizeof(s3_event_t)) { switch (evt.type) ... }
 *  Subsystem 3 sends with TMO_POL: if nobody reads, events are dropped
 *  (counted in evt_dropped), the line task never blocks.
 * --------------------------------------------------------------------- */
typedef enum
{
    S3_EVT_NONE = 0,
    S3_EVT_LINE_LOST,       /* FOLLOW -> LOST; side = last seen side        */
    S3_EVT_LINE_FOUND,      /* LOST/SEARCH -> FOLLOW                        */
    S3_EVT_JUNCTION,        /* all junction sensors on black                */
    S3_EVT_BARCODE_START,   /* first bar seen: slow down (DECODE_BARCODE)   */
    S3_EVT_BARCODE_DECODED, /* nav + text valid                             */
    S3_EVT_BARCODE_FAILED,  /* code = code39_result_t, retries in arg       */
    S3_EVT_CAL_DONE,        /* calibration accepted                         */
    S3_EVT_CAL_FAILED       /* calibration span too small; old cal kept     */
} s3_evt_type_t;

typedef struct
{
    uint8_t  type;    /* s3_evt_type_t                                */
    uint8_t  nav;     /* navigation_command_t (DECODED only)  */
    int8_t   side;    /* last_side for LINE_LOST              */
    uint8_t  code;    /* failure reason (FAILED only)         */
    uint32_t arg;     /* retries so far (FAILED) / unused     */
    uint32_t time_ms; /* when it happened                     */
    uint8_t  text[S3_BC_TEXT_MAX]; /* decoded text, '\0' ended     */
} s3_event_t;

#define S3_EVT_QUEUE_LEN 8 /* events buffered in g_mbf_s3_events */

extern ID g_mbf_s3_events;

/* ---------------------------------------------------------------------
 *  IF-33: requests to subsystem 3 (any task; returns at once)
 * --------------------------------------------------------------------- */
#define S3_REQ_CALIBRATE  0x0001 /* sample min/max for S3_CAL_DURATION_MS */
#define S3_REQ_RESET_LINE 0x0002 /* tracker -> SEARCH, forget last side  */
#define S3_REQ_BC_ENABLE  0x0004 /* start reading barcodes               */
#define S3_REQ_BC_DISABLE 0x0008 /* ignore barcodes (e.g. while avoiding)*/
#define S3_REQ_BC_REARM   0x0010 /* clear retries, wait for next barcode */

extern ER s3_request(uint32_t req);

/* ---------------------------------------------------------------------
 *  Private to subsystem 3 (shared between its own .c files)
 * --------------------------------------------------------------------- */
/* One finished run of the barcode sensor, line_task -> barcode_task */
typedef struct
{
    uint32_t width_us;  /* how long the level lasted            */
    uint32_t width_pos; /* how far the wheels went meanwhile, in
                           encoder counts (L + R); 0 = not measured */
    uint8_t  black;     /* 1 = bar (black), 0 = space (white)   */
    /* Bars only: the bar really began this much AFTER the edge that ended
       the space before it (its start is re-found at 50 % of its own peak
       once the peak is known).  barcode_task adds it to that space. */
    uint32_t lead_us;
    uint32_t lead_pos;
} s3_bc_run_t;

#define S3_FLG_LINE_REQ (S3_REQ_CALIBRATE | S3_REQ_RESET_LINE)
#define S3_FLG_BC_REQ   (S3_REQ_BC_ENABLE | S3_REQ_BC_DISABLE | S3_REQ_BC_REARM)

extern ID               g_mtx_s3_status;
extern ID               g_mbf_s3_bc_runs;
extern ID               g_flg_s3_req;
extern ID               g_tid_s3_line;
extern ID               g_tid_s3_barcode;
extern ID               g_cyc_s3_line;
extern s3_line_status_t g_s3_status;
extern volatile uint8_t g_s3_black_mask_live; /* line_task -> s3_ports.c,
                                         readable without the mutex */
extern volatile bool    g_b_s3_bc_scanning;   /* barcode_task -> line_task:
                                         inhibit junction detection */
/* Set once a read has seen a bar AND the space after it, i.e. real
   barcode bars.  Junctions are only ignored from then on: one long black
   under L with nothing after it is a junction's side arm (or the line on
   a curve), and the junction behind it must still be reported. */
extern volatile bool    g_b_s3_bc_bars;
extern uint32_t         g_s3_line_overruns;   /* line_task diagnostics */
extern uint32_t         g_s3_line_read_errors;
extern uint32_t         g_s3_line_wakeups;

extern void                 s3_line_cyclic_isr(void * p_exinf);
extern void                 line_task(int32_t stacd, void * p_exinf);
extern void                 barcode_task(int32_t stacd, void * p_exinf);
extern void                 s3_send_event(s3_event_t * p_evt);
extern navigation_command_t s3_text_to_nav(const uint8_t * p_text);

#endif /* SUBSYSTEM3_H */

/*** end of file ***/
