/*
 *      vc_sm.h
 *      Subsystem 6 -- Vehicle Controller mission state machine.
 *
 *      Pure logic: vc_sm_handle() takes one event and returns a list of
 *      ACTIONS (move, request, report).  It never calls the kernel or a
 *      driver, so the same code runs in the controller task, in the
 *      on-target tests (on a private vc_sm_t, motors untouched) and on a PC.
 *      The task (task_vehicle_ctrl.c) carries the actions out through
 *      vc_ports.c.
 */

#ifndef VC_SM_H
#define VC_SM_H

#include "vc_types.h"
#include "../subsystem2/s2_config.h" /* track and tick size for VC_PIVOT_COUNTS */

/* ---------------------------------------------------------------------
 *  Tuning.  Preliminary values: validate on the track (NFR2, NFR5,
 *  design review 8.3 "recovery limits ... need validation").
 * --------------------------------------------------------------------- */
#define VC_SPEED_CRUISE_MMS 120 /* normal line following (tuned 2026-11-04: was 200 at 60%) */
#define VC_SPEED_SLOW_MMS   60  /* barcode reading, humps and post-bypass follow */
#define VC_REJOIN_STEER_SAMPLES 50 /* 500 ms of fresh steering before cruise */
/* Slow down in curves: forward speed is cut by up to this much (permille)
   at full steering, in proportion to how hard S3 steers.  400 = at a full
   turn the robot drives at 60 % of cruise; a small nudge barely slows it.
   Never below VC_CURVE_MIN_MMS.  TUNE (live on the dashboard). */
#define VC_CURVE_SLOW_PERMILLE 600 /* tuned 2026-11-04: U-turn works at 60 % */
#define VC_CURVE_MIN_MMS       60
#define VC_TURN_DEG         90  /* barcode LEFT / RIGHT */
/* Junction turns end on the line (2026-11-12, idea from Group 7's code).
   The turn ignores the line until it has turned VC_TURN_GATE_DEG (U-turn:
   see the U-turn below), then ends on the first C or R black reading, so a
   wrong S2_TRACK_MM or wheel slip no longer leaves the robot beside the new
   arm.  LEFT / RIGHT ask S2 for VC_TURN_MAX_DEG; if no line is seen by then
   the turn ends there as before.  The angle comes from S2's odometer
   (g_s2_odo_ticks, L + R counts), sent in TICK and LINE_FOUND events (p2).
   0 = old fixed VC_TURN_DEG turn.  TUNE the gates on the track. */
#ifndef VC_TURN_END_ON_LINE
#define VC_TURN_END_ON_LINE 1
#endif
#define VC_TURN_GATE_DEG  60  /* LEFT / RIGHT: line ignored before this */
#define VC_TURN_MAX_DEG   135 /* LEFT / RIGHT: turn ends here if no line */
#if VC_TURN_END_ON_LINE
#define VC_TURN_CMD_DEG VC_TURN_MAX_DEG
#else
#define VC_TURN_CMD_DEG VC_TURN_DEG
#endif
/* L + R odometer counts for a pivot of `deg` degrees: the two wheels roll
   pi x S2_TRACK_MM x deg / 180 mm between them.  90 deg = ~692 counts. */
#define VC_PIVOT_COUNTS(deg) \
    ((uint32_t)((S2_PI * S2_TRACK_MM * (float)(deg)) / (180.0f * S2_MM_PER_TICK)))

/* U-turn (2026-11-25): stop at the junction, then
     1. a fast encoder pivot (S2 TURN) to VC_UTURN_FAST_DEG, short of the
        expected return heading VC_UTURN_CMD_DEG (vc_types.h).  Line
        readings are ignored: the junction and its 90 deg arm cannot end it;
     2. a slow pivot (VC_MOVE_PIVOT at VC_UTURN_SLOW_MMS) the same way,
        watching every fresh C / R sample.  A credible sighting (C or R
        black for VC_UTURN_SIGHT_SAMPLES samples in a row) asks S2 to stop;
     3. once S2 confirms the wheels stopped, fresh samples decide: C or R
        black, or both white just after a sensor let the line go and
        before the other one saw it (tape between them) = aligned -> slow
        line following, then cruise.  The other sensor let it go = turned
        past it -> pivot back slowly toward the sighting.  No sighting by
        VC_UTURN_CMD_DEG + VC_UTURN_SECTOR_DEG -> sweep back to
        VC_UTURN_CMD_DEG - VC_UTURN_SECTOR_DEG;
     4. at most VC_UTURN_MAX_SWEEPS slow sweeps after the first, each bounded
        by the encoder, and VC_UTURN_MS for the whole U-turn; then FAULT
        VC_FC_LINE_RECOVERY with the motors stopped.  The robot never
        drives off on the encoder angle alone.
   Angles here are S2 command degrees (what VC_PIVOT_COUNTS() converts),
   not body degrees: the owner's runs show the body turns more than S2
   counts.  TUNE on the track. */
#define VC_UTURN_APPROACH_DEG 40  /* slow window before the expected heading */
#define VC_UTURN_FAST_DEG     (VC_UTURN_CMD_DEG - VC_UTURN_APPROACH_DEG)
#define VC_UTURN_SECTOR_DEG   15  /* search this far either side of it */
#define VC_UTURN_CORRECT_DEG  10  /* a correction may go this far past the sighting */
#define VC_UTURN_SLOW_MMS     60  /* slow pivot wheel speed (S2_MIN_SPEED_MM_S) */
#define VC_UTURN_SIGHT_SAMPLES  2 /* 10 ms samples: black = sighting, white = left */
#define VC_UTURN_VERIFY_SAMPLES 3 /* fresh samples after the stop, all agreeing */
#define VC_UTURN_MAX_SWEEPS   3   /* slow sweeps after the approach */
#define VC_UTURN_REFRESH_MS   100 /* re-send PIVOT (S2 stops a DRIVE after 500) */
#define VC_UTURN_SETTLE_MS    1000 /* S2 must confirm the stop within this */
#define VC_UTURN_VERIFY_MS    500 /* fresh samples must agree within this */
#define VC_UTURN_MS           12000 /* the whole U-turn */
_Static_assert(VC_UTURN_FAST_DEG >= 90,
               "the fast pivot must pass the 90 deg arm before the line counts");
_Static_assert((VC_UTURN_SECTOR_DEG < VC_UTURN_APPROACH_DEG) &&
                   (VC_UTURN_CORRECT_DEG < VC_UTURN_APPROACH_DEG),
               "U-turn sweeps stay inside the watched window");
_Static_assert((VC_UTURN_REFRESH_MS * 2) < 500,
               "PIVOT must be refreshed well within S2_CMD_TIMEOUT_MS");

/* ---------------------------------------------------------------------
 *  Lift mode (owner, 2026-11-25).  Chosen before a run; the run keeps the
 *  mode it started with (vc_status_t.lift_mode, taken at Start).
 *    VC_LIFT_MODE_END    (0, DEFAULT, the assessed demo): a lift ends the
 *                        run, MISSION_COMPLETE, fault code 0x4005.
 *    VC_LIFT_MODE_PAUSE  (1): a lift while the mission is running pauses
 *                        the SAME mission; it resumes by itself only after
 *                        the checks below.  The lift wins over any
 *                        manoeuvre (owner, 2026-11-25: a lifted robot's S3
 *                        sees black and may report a junction or barcode
 *                        first): from JUNCTION_STOP, EXECUTE_ACTION,
 *                        DECODE_BARCODE, AVOID_OBSTACLE, LOST_LINE or
 *                        TERRAIN_CHECK the controller drops that manoeuvre
 *                        and pauses in FOLLOW_LINE.  A turn already started
 *                        is never resumed (its barcode action is dropped);
 *                        an action not yet started stays pending; S5's
 *                        bypass is aborted.  A lift in MISSION_START is
 *                        still a FAULT.
 *  Selected at build time: make ... LIFT_RESUME=1 (pico_rp2040.mk) sets
 *  -DVC_LIFT_MODE=1 and names the .uf2 *_liftresume.  vc_sm_set_lift_mode()
 *  changes it only while no mission is running.
 *
 *  Pause on lift, step by step (all motion stays with the controller; S4
 *  latches S2's motors off by itself, before any of this runs):
 *    LIFTED     S2's IMU interlock is set (TICK p1 != 0).  Motors off.
 *    SETTLING   S4 cleared the interlock: its own "set down, level and
 *               still for S4_SETTLE_SAMPLES" rule (imu_terrain.c).  The
 *               interlock must stay clear and the wheels must not turn
 *               (odometer within VC_LIFT_ODO_TOL) for VC_LIFT_SETTLE_MS.
 *    WAIT_LINE  only line samples taken after this point count, each one
 *               once (its own time must be later than the last one used).
 *               Line confirmed = the same black pattern (C only, R only,
 *               or C and R both: on the tape, owner 2026-11-25) in
 *               VC_LIFT_LINE_SAMPLES fresh samples in a row spanning at
 *               least VC_LIFT_LINE_MS.  "No floor" is already ruled out by
 *               S4's set-down rule and the settle time.  Both white
 *               (centred on the tape OR plain floor: the straddle layout
 *               cannot tell them apart) and anything changing keep the
 *               robot stopped, for ever.
 *    RESUMING   S2 RESET + readiness check (VC_EV_READY from S2 within
 *               VC_LIFT_READY_MS), then VC_LIFT_VERIFY_SAMPLES fresh samples
 *               taken after READY must still show the same sensor black,
 *               interlock clear, wheels still.  Then slow line following
 *               (VC_REJOIN_STEER_SAMPLES) and cruise.
 *  Any interlock -> LIFTED again (a new lift episode).  Wheels turning ->
 *  SETTLING again.  Line not confirmed during RESUMING -> WAIT_LINE.
 *  Overturn, Stop, a critical fault -> FAULT; Reset -> IDLE.  No timeout
 *  ever starts motion.  Operator Pause is kept: no resume until Resume.
 * --------------------------------------------------------------------- */
#define VC_LIFT_MODE_END   0
#define VC_LIFT_MODE_PAUSE 1
#ifndef VC_LIFT_MODE
#define VC_LIFT_MODE VC_LIFT_MODE_END
#endif
_Static_assert((VC_LIFT_MODE == VC_LIFT_MODE_END) ||
                   (VC_LIFT_MODE == VC_LIFT_MODE_PAUSE),
               "VC_LIFT_MODE must be VC_LIFT_MODE_END or VC_LIFT_MODE_PAUSE");

/* vc_status_t.lift_phase */
#define VC_LP_NONE      0u
#define VC_LP_LIFTED    1u
#define VC_LP_SETTLING  2u
#define VC_LP_WAIT_LINE 3u
#define VC_LP_RESUMING  4u

#define VC_LIFT_SETTLE_MS      500 /* after S4's own 1 s set-down rule */
#define VC_LIFT_ODO_TOL        2   /* L + R counts: more = wheels turned */
#define VC_LIFT_LINE_SAMPLES   30  /* 10 ms S3 samples in a row ... */
#define VC_LIFT_LINE_MS        250 /* ... spanning at least this long */
#define VC_LIFT_READY_MS       1000 /* S2 must report READY after RESET */
#define VC_LIFT_VERIFY_SAMPLES 3   /* fresh samples after READY */
#define VC_LIFT_VERIFY_MS      500 /* ... within this */
_Static_assert(VC_LIFT_LINE_MS <= ((VC_LIFT_LINE_SAMPLES - 1) * 10),
               "the confirmation interval must fit in the sample count");
_Static_assert(VC_LIFT_LINE_SAMPLES >= 2,
               "one sample can never confirm the line");

/* Lift pause working data (FOLLOW_LINE with lp.phase != VC_LP_NONE) */
typedef struct
{
    uint8_t  phase;     /* VC_LP_* (also shown in vc_status_t.lift_phase) */
    uint8_t  step;      /* RESUMING: 0 = waiting for READY, 1 = verifying */
    uint8_t  pattern;   /* VC_LINE_C or VC_LINE_R being confirmed, 0 = none */
    bool     op_hold;   /* operator Pause: no resume until Resume */
    bool     have_sample; /* last_ms holds a used sample's time */
    uint16_t count;     /* fresh samples in a row showing `pattern` */
    uint16_t episode;   /* lifts in this pause (debug) */
    uint32_t phase_ms;  /* phase / step start: older samples are stale */
    uint32_t first_ms;  /* time of the first sample of the run of `pattern` */
    uint32_t last_ms;   /* time of the last sample used */
    uint32_t odo0;      /* odometer when the wheels were last known still */
    int32_t  imu;       /* latest TICK p1: S2 IMU interlock (0 = clear) */
} vc_lift_t;

#define VC_SEARCH_TURN_DEG  30  /* LOST_LINE attempt 1 (attempt 2: x2) */
#define VC_SEARCH_BACK_MM   50  /* LOST_LINE attempt 3: reverse */

#define VC_SELF_CHECK_MS     2000  /* MISSION_START: wait for READY */
#define VC_LINE_ATTEMPT_MS   1500  /* LOST_LINE: one search attempt */
#define VC_LINE_MAX_ATTEMPTS 3     /* then FAULT (bounded recovery) */
/* DECODE_BARCODE: one read.  Was 1500, but a whole read takes far longer:
   big-print *A* is ~291 mm long, so at 60 mm/s it takes ~4.9 s to cross,
   plus the end-of-barcode quiet zone (5 x widest bar = ~1.6 s).  After
   1500 ms the retry re-armed barcode_task, which threw away every bar read
   so far (2026-11-08: cause of "too few elements").  8000 covers big *A*
   at 60 mm/s with margin; S3 still ends a read itself on its quiet zone. */
#define VC_DECODE_MS         12000 /* big print *A* is 291 mm = 8.1 s at 36 mm/s (60 % speed); was 8000 */
/* Was 2.  A retry cannot work while driving forward: after a failed read the
   barcode is already behind the robot, so each "retry" only held the robot
   in DECODE_BARCODE (slow, junctions ignored) for another VC_DECODE_MS.
   0 = one read, then report it and keep following (review 2026-11-10). */
#define VC_BARCODE_MAX_RETRY 0     /* then give up, keep following */
/* EXECUTE_ACTION sub-steps (exec_phase) */
#define VC_EXEC_TURN       0u /* the action's own move (or the only one) */
#define VC_EXEC_ADVANCE    1u /* LEFT / RIGHT: axle onto the crossing first */
#define VC_EXEC_UTURN      2u /* U-turn: fast encoder pivot (S2 TURN) */
#define VC_EXEC_UT_SWEEP   3u /* U-turn: slow pivot, watching C / R */
#define VC_EXEC_UT_BRAKE   4u /* U-turn: stop asked, waiting for S2 */
#define VC_EXEC_UT_VERIFY  5u /* U-turn: standing, fresh samples decide */
#define VC_EXECUTE_MS        6000  /* EXECUTE_ACTION: advance + turn (was 4000) */
#define VC_JUNCTION_STOP_MS  2000  /* wait for S2 to confirm zero speed */
/* Stand still this long at a junction before the turn / drive-on, so the
   stop is a clear, visible pause (2026-11-11).  0 = act as soon as S2
   confirms zero speed. */
#ifndef VC_JUNCTION_HOLD_MS
#define VC_JUNCTION_HOLD_MS  1000
#endif
#define VC_JUNCTION_CLEAR_MM 60    /* straight distance to clear the crossing; TUNE */
/* LEFT / RIGHT at a junction: drive this far first, so the AXLE (not the
   sensors) is over the crossing when the robot pivots.  See
   ROBOT_IR_TO_AXLE_MM in robot_types.h.  0 = turn at once. */
#ifndef VC_JUNCTION_ADVANCE_MM
#define VC_JUNCTION_ADVANCE_MM ROBOT_IR_TO_AXLE_MM
#endif
#define VC_TERRAIN_MS        5000  /* TERRAIN_CHECK: hump must clear */
/* Was 15000, shorter than a normal bypass: scan (up to ~5 s) + 4 turns +
   3 legs (~9 s) + up to 3 line recoveries (~9 s).  The controller faulted
   while S5 was still searching for the line (review 2026-11-10). */
#define VC_AVOID_MS          90000 /* scan, slow bypass and bounded recovery */

/* ---------------------------------------------------------------------
 *  Actions produced by the state machine
 * --------------------------------------------------------------------- */
typedef enum
{
    VC_ACT_MOVE = 1, /* p1 = vc_move_t, p2 = value          -> S2 */
    VC_ACT_REQUEST,  /* target = vc_source_t (0 = all), p1 = vc_request_t, p2 =
                        arg */
    VC_ACT_REPORT,   /* target = source of fault, p1 = severity, p2 = code -> S1
                      */
    VC_ACT_STATE,    /* p1 = new vc_state_t, p2 = old vc_state_t */
    VC_ACT_CONFIG,   /* p1 = 1 apply the pending barcode map write, 0 refuse
                        it (mission active); p2 = request id */
    VC_ACT_JUNCTION, /* p1 = vc_junction_note_t, p2 = (letter << 8) | wire
                        action code (vc_barcode_map.h); letter 'A'..'Z', or
                        0 when there was no single-letter barcode -> S1 */
    VC_ACT_TRACE,    /* U-turn log line: p1 = vc_ut_trace_t, p2 = heading
                        (counts turned, + = left), rsv = detail -> console */
    VC_ACT_LIFT      /* pause-on-lift log line: p1 = VC_LP_* phase now,
                        p2 = detail (vc_lift_note_t) -> console */
} vc_action_kind_t;

/* VC_ACT_LIFT p2: why the phase changed ("vc: lift <phase> <why>") */
typedef enum
{
    VC_LN_LIFT = 1,      /* a lift interrupted the run */
    VC_LN_RELIFT,        /* lifted again: a new lift episode */
    VC_LN_DOWN,          /* S4: set down, level and still */
    VC_LN_WHEELS,        /* the wheels turned (pushed): settle again */
    VC_LN_SETTLED,       /* settle time over: watching the line */
    VC_LN_CONFIRMED,     /* line confirmed: S2 reset + readiness check */
    VC_LN_READY,         /* S2 ready: verifying fresh samples */
    VC_LN_LOST,          /* line not confirmed during the handshake */
    VC_LN_NO_READY,      /* S2 did not report ready in time */
    VC_LN_GO,            /* slow line following started */
    VC_LN_CRUISE,        /* back to cruise: the lift pause is over */
    VC_LN_HOLD           /* operator Pause holds the robot */
} vc_lift_note_t;

/* VC_ACT_TRACE: one console line per U-turn step ("vc: ut ...") */
typedef enum
{
    VC_UT_START = 1, /* rsv = fast pivot angle asked of S2 (deg) */
    VC_UT_EARLY,     /* line seen during the fast pivot, ignored; rsv = sensor */
    VC_UT_FAST_DONE, /* S2 finished the fast pivot */
    VC_UT_SWEEP,     /* slow pivot starts; rsv = vc_ut_sweep_t x dir (+1 / -1),
                        p2 = its bound */
    VC_UT_SIGHT,     /* first credible sighting of the sweep; rsv = sensor */
    VC_UT_BRAKE,     /* stop asked; rsv = 1 sighting, 2 bound reached */
    VC_UT_SETTLED,   /* S2: wheels stopped; rsv = counts after the stop request */
    VC_UT_VERDICT,   /* rsv = vc_ut_verdict_t */
    VC_UT_DONE,      /* aligned: slow line following takes over */
    VC_UT_FAIL       /* rsv = 1 sweeps used up, 2 no stop, 3 verify, 4 time */
} vc_ut_trace_t;

typedef enum
{
    VC_UT_APPROACH = 1, /* first slow sweep, same way as the fast pivot */
    VC_UT_CORRECT  = 2, /* back toward a line the robot turned past */
    VC_UT_SEARCH   = 3  /* no line in the approach: the other way */
} vc_ut_sweep_t;

typedef enum
{
    VC_UT_V_UNSURE  = 0, /* evidence still changing */
    VC_UT_V_ON_LINE = 1, /* C or R black now */
    VC_UT_V_BETWEEN = 2, /* both white, the tape is between C and R */
    VC_UT_V_PASSED  = 3, /* both white, the line went out past the far sensor */
    VC_UT_V_NONE    = 4  /* no line seen in this sweep */
} vc_ut_verdict_t;

/* U-turn working data (EXECUTE_ACTION, VC_EXEC_UTURN .. VC_EXEC_UT_VERIFY) */
typedef struct
{
    int32_t  heading;    /* counts turned since the U-turn began, + = left */
    int32_t  bound;      /* this sweep stops here (counts) */
    int32_t  sight_h;    /* heading of the sweep's first credible sighting */
    int32_t  stop_h;     /* heading when the stop was asked */
    uint32_t t0_ms;      /* U-turn start */
    uint32_t phase_ms;   /* phase start: samples older than this are stale */
    uint32_t cmd_ms;     /* last PIVOT sent */
    int8_t   dir;        /* +1 left, -1 right: the way the wheels turn */
    uint8_t  sweep;      /* vc_ut_sweep_t of the current sweep */
    uint8_t  sweeps;     /* slow sweeps after the approach */
    uint8_t  sight;      /* VC_LINE_C / VC_LINE_R of that sighting, 0 = none */
    uint8_t  early;      /* fast pivot: a line was seen (traced once) */
    uint8_t  run_black[2]; /* [0] = C, [1] = R: black samples in a row */
    uint8_t  run_white[2];
    uint8_t  seen[2];    /* sensor saw the line this sweep (debounced) */
    uint8_t  left[2];    /* ... and then let it go (debounced) */
    uint8_t  verdict;    /* VERIFY: verdict being confirmed */
    uint8_t  verdict_n;  /* fresh samples agreeing on it */
    bool     handover;   /* FOLLOW_LINE entered from a finished U-turn */
} vc_uturn_t;

/* Evidence for the dashboard's "Read B -> Left turn pending -> Left turn
   completed".  ASSIGNED proves only the lookup; COMPLETED is sent when the
   manoeuvre has finished and line following has taken over; DROPPED when a
   pending action was cancelled inside a run (obstacle, manoeuvre timeout,
   end of course, lift).  A run ending by Stop / fault / Reset sends nothing:
   the state change is the evidence. */
typedef enum
{
    VC_JX_ASSIGNED  = 1,
    VC_JX_COMPLETED = 2,
    VC_JX_DROPPED   = 3
} vc_junction_note_t;

typedef struct
{
    uint8_t kind;   /* vc_action_kind_t */
    uint8_t target; /* vc_source_t, 0 = broadcast */
    int16_t rsv;
    int32_t p1;
    int32_t p2;
} vc_action_t;

#define VC_MAX_ACTIONS 12

typedef struct
{
    int32_t     n;
    bool        overflow; /* more than VC_MAX_ACTIONS: a bug */
    vc_action_t a[VC_MAX_ACTIONS];
} vc_actions_t;

/* The whole controller state.  One instance lives in the task. */
typedef struct
{
    vc_status_t          st;
    uint32_t             ready_mask;       /* VC_READY_BIT()s seen */
    uint32_t             attempt_since_ms; /* LOST_LINE / DECODE timer */
    uint8_t              attempts;         /* LOST_LINE search attempts */
    uint8_t              retries;          /* barcode read retries */
    bool                 avoid_wait_stop; /* wheels must stop before scanning */
    uint8_t              rejoin_steer_left; /* slow steering samples left after bypass */
    uint32_t             junction_stop_ms; /* JUNCTION_STOP: when S2 confirmed the stop */
    uint8_t              junction_stopped; /* JUNCTION_STOP: 1 = standing, holding */
    uint8_t              exec_phase;       /* EXECUTE_ACTION: VC_EXEC_* */
    bool                 pause_stopped;
    uint32_t             pause_since_ms;
    uint32_t             odo;              /* last S2 odometer (L + R counts) seen in an event */
    uint32_t             turn_odo0;        /* odometer when the junction turn began */
    uint32_t             turn_gate;        /* counts before the line may end the turn, 0 = not watching */
    int8_t               lost_side;        /* -1 left, +1 right, 0 unknown */
    vc_uturn_t           ut;               /* U-turn steps */
    vc_lift_t            lp;               /* pause on lift */
    uint8_t              lift_mode_cfg;    /* VC_LIFT_MODE_* for the next Start */
    bool                 cmd_ok;           /* last Pause / Resume accepted */
    navigation_command_t pending_nav;      /* decoded action for next junction */
    char                 pending_barcode[16]; /* full decoded barcode text */
} vc_sm_t;

extern void vc_sm_init(vc_sm_t * p_sm, uint32_t now_ms);
extern void vc_sm_handle(vc_sm_t * p_sm, const vc_event_t * p_event,
                         uint32_t now_ms, vc_actions_t * p_out);
/* Lift mode for the next Start.  Refused (false, nothing changes) while a
   mission is running: a run keeps the mode it started with. */
extern bool vc_sm_set_lift_mode(vc_sm_t * p_sm, uint8_t mode);

/* Which subsystem may move the robot in a state (VC_SRC_NONE = only the
   controller's own scripted moves) */
extern vc_source_t vc_sm_motion_owner(vc_state_t state);
extern bool        vc_sm_is_active(vc_state_t state);

extern const char * vc_state_name(vc_state_t state);
extern const char * vc_event_name(vc_event_type_t type);

#endif /* VC_SM_H */

/*** end of file ***/
