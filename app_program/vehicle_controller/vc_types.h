/*
 *      vc_types.h
 *      Subsystem 6 -- Vehicle Controller.  Data formats shared with the
 *      other subsystems: mission states, events and movement commands.
 *
 *      No kernel calls in here, so the state machine (vc_sm.c) can also be
 *      compiled and tested on a PC (see host_test/).
 */

#ifndef VC_TYPES_H
#define VC_TYPES_H

#ifdef VC_HOST_TEST
#include "host_test/host_types.h" /* PC build: plain C typedefs */
#else
#include <tk/tkernel.h> /* target build: kernel types */
#endif

#include <stdint.h>
#include <stdbool.h>

#include "../robot_types.h" /* navigation_command_t (shared) */

/* ---------------------------------------------------------------------
 *  Mission states (design review section 5, "Vehicle controller" table)
 * --------------------------------------------------------------------- */
typedef enum
{
    VC_ST_IDLE = 0,         /* motors off, wait for start */
    VC_ST_MISSION_START,    /* initialise subsystems, self check */
    VC_ST_FOLLOW_LINE,      /* hub state */
    VC_ST_JUNCTION_STOP,    /* stop at junction; wait for S2 confirmation */
    VC_ST_LOST_LINE,        /* stop, then bounded line recovery */
    VC_ST_AVOID_OBSTACLE,   /* S5 profiles and bypasses the obstacle */
    VC_ST_DECODE_BARCODE,   /* slow, let S3 read the barcode */
    VC_ST_EXECUTE_ACTION,   /* run the barcode's manoeuvre */
    VC_ST_TERRAIN_CHECK,    /* slow over a hump, S4 measures it */
    VC_ST_FAULT,            /* safe stop: motors off, report */
    VC_ST_MISSION_COMPLETE, /* stop and report */
    VC_ST_COUNT
} vc_state_t;

/* ---------------------------------------------------------------------
 *  Who sent an event.  Matches the subsystem folder numbers.
 * --------------------------------------------------------------------- */
typedef enum
{
    VC_SRC_NONE   = 0,
    VC_SRC_COMMS  = 1, /* S1 WiFi / MQTT / telemetry */
    VC_SRC_MOTION = 2, /* S2 motors + encoders */
    VC_SRC_LINE   = 3, /* S3 IR line following + barcode */
    VC_SRC_IMU    = 4, /* S4 IMU terrain monitoring */
    VC_SRC_OBST   = 5, /* S5 ultrasonic scan + avoidance */
    VC_SRC_VC     = 6  /* the Vehicle Controller itself (timeouts) */
} vc_source_t;

/* Ready mask bits used during MISSION_START (1 << source) */
#define VC_READY_BIT(src) (1U << (uint32_t)(src))
#define VC_READY_REQUIRED \
    (VC_READY_BIT(VC_SRC_MOTION) | VC_READY_BIT(VC_SRC_LINE))
#define VC_READY_ALL \
    (VC_READY_REQUIRED | VC_READY_BIT(VC_SRC_IMU) | VC_READY_BIT(VC_SRC_OBST))

/* ---------------------------------------------------------------------
 *  Events sent TO the Vehicle Controller (IF-06).  p1/p2 meaning in
 *  the comment; unused parameters are 0.
 * --------------------------------------------------------------------- */
typedef enum
{
    VC_EV_NONE = 0,

    /* any subsystem */
    VC_EV_READY,        /* subsystem finished self check */
    VC_EV_MOVE_REQUEST, /* p1 = vc_move_t, p2 = value (see vc_move_t) */
    VC_EV_FAULT,        /* p1 = vc_fault_severity_t, p2 = fault code */

    /* S1 comms: remote commands never drive the motors directly */
    VC_EV_CMD_START, /* start the mission */
    VC_EV_CMD_STOP,  /* stop: go to FAULT (operator stop) */
    VC_EV_CMD_RESET, /* acknowledge fault, back to IDLE */
    VC_EV_CMD_PAUSE, /* request pause while following the line */
    VC_EV_CMD_RESUME, /* resume after stop confirmation */
    VC_EV_WIFI_DOWN, /* non-critical: keep navigating */
    VC_EV_WIFI_UP,

    /* S2 motion: send after every TURN_*, U_TURN, REVERSE, DRIVE_DIST */
    VC_EV_MANOEUVRE_DONE, /* p1 = 1 completed, 0 failed (navigation_complete);
                             p2 = S2 odometer (L + R counts), 0 = not given */
    VC_EV_MOTION_STOPPED,  /* S2 confirms MOTION_STOP reached zero speed;
                              p2 = S2 odometer then, 0 = not given */

    /* S3 line + barcode */
    VC_EV_LINE_LOST,     /* p1 = last side seen: -1 left, +1 right, 0 unknown */
    VC_EV_LINE_FOUND,    /* line_reacquired */
    VC_EV_JUNCTION,      /* p1 = junction type (S3 defines) */
    VC_EV_BARCODE_START, /* barcode_detected: start of bars seen */
    VC_EV_BARCODE_DECODED, /* p1 = legacy nav, barcode_text = full A-Z code */
    VC_EV_BARCODE_FAILED,  /* barcode_read_failed */
    VC_EV_END_OF_COURSE,

    /* S4 IMU */
    VC_EV_HUMP_START,   /* hump_detected */
    VC_EV_HUMP_CLEARED, /* p1 = peak height (um), 0 = aborted */
    VC_EV_LIFTED,       /* robot lifted (S4). p1 = IMU_SAFETY_LIFT, p2 = fault
                           code to report if no mission is running.  During a
                           mission this ends the run: MISSION_COMPLETE, motors
                           stopped.  (Collision detection was removed.) */

    /* S5 obstacle */
    VC_EV_OBSTACLE,     /* p1 = distance (mm) */
    VC_EV_AVOID_DONE,   /* bypassed and line reacquired */
    VC_EV_AVOID_FAILED, /* bypass/recovery failed: stop */

    /* S6 internal */
    VC_EV_TICK, /* periodic, drives state timeouts */

    /* S1 (added last so no existing event number changes) */
    VC_EV_CMD_CONFIG, /* a barcode map write is pending (vc_barcode_map.h);
                         p1 = request id.  Applied only when no mission is
                         active, decided by the controller itself. */

    /* S3, only while the controller asked for it (VC_REQ_LINE_WATCH) in
       EXECUTE_ACTION: one per NEW line sample, black or white.
       p1 = VC_LINE_C | VC_LINE_R bits on black, p2 = S2 odometer (L + R
       counts) when it was read, time_ms = the sample's own time, so the
       controller can tell a sample taken before the wheels stopped. */
    VC_EV_LINE_SAMPLE,

    VC_EV_COUNT
} vc_event_type_t;

/* VC_EV_LINE_SAMPLE p1: which line sensor reads black.  S3's own mask bits
   stay inside the S3 bridge, so the controller compiles without S3. */
#define VC_LINE_C 1
#define VC_LINE_R 2

/* Fault severity (FR8) */
typedef enum
{
    VC_FAULT_NON_CRITICAL = 0, /* report, keep going */
    VC_FAULT_RECOVERABLE  = 1, /* bounded recovery */
    VC_FAULT_CRITICAL     = 2  /* safe stop */
} vc_fault_severity_t;

/* Fault codes.  0x00xx are raised by the controller itself; every other
   subsystem uses its own range 0xN0xx..0xN0FF, N = its folder number
   (S2 motion uses 0x2000..0x20FF, S4 IMU 0x4000..0x40FF, ...). */
#define VC_FC_NONE          0x0000
#define VC_FC_OPERATOR_STOP 0x0001
#define VC_FC_SELF_CHECK    0x0002 /* required subsystem not ready */
#define VC_FC_LINE_RECOVERY 0x0003 /* line not found within limits */
#define VC_FC_AVOID_FAILED  0x0004
/* 0x0005 was IMPACT: removed (professor's IMU change), do not reuse */
#define VC_FC_STATE_TIMEOUT 0x0006 /* a state took too long */
#define VC_FC_BARCODE       0x0007 /* barcode retries used up (non-critical) */
#define VC_FC_JUNCTION_STOP 0x0008 /* S2 did not confirm stop at junction */
#define VC_FC_PAUSE_INTERRUPTED 0x0009 /* unsafe event while paused */
#define VC_FC_WIFI_DOWN     0x1000 /* reported for S1 (non-critical) */
/* Codes the controller itself tests; the bridges check they match. */
#define VC_FC_S2_SAFE_STOP  0x2011 /* S2_FC_SAFE_STOP, vc_bridge_s2.c */
#define VC_FC_IMU_SAFETY    0x4005 /* S4_FC_SAFETY (lift / overturn), vc_bridge_s4.c */

/* vc_event_t.cause.  Only S2's critical safe-stop fault (0x2011) sets it:
   it says whether S4's lift / overturn interlock raised that stop
   (s2_imu_stop_cause()).  The values equal S2_IMU_CAUSE_* (checked in
   vc_bridge_s2.c).  NONE = no IMU interlock: a genuine critical fault. */
#define VC_CAUSE_NONE         0
#define VC_CAUSE_IMU_LIFT     1
#define VC_CAUSE_IMU_OVERTURN 2
#define VC_CAUSE_IMU_UNKNOWN  3 /* IMU interlock of unknown kind: FAULT */

/* One event, copied into the message buffer.  barcode_text is populated
   only for VC_EV_BARCODE_DECODED; all other producers set it to empty. */
typedef struct
{
    uint8_t  src;  /* vc_source_t */
    uint8_t  type; /* vc_event_type_t */
    int16_t  cause; /* VC_CAUSE_*: only on S2's safe-stop fault, else 0 */
    int32_t  p1;
    int32_t  p2;
    uint32_t time_ms; /* sender's tk_get_tim() low word, 0 if unknown */
    char     barcode_text[16]; /* full decoded text for BARCODE_DECODED */
} vc_event_t;

/* ---------------------------------------------------------------------
 *  Movement commands.  The controller is the ONLY source of these;
 *  other subsystems ask with VC_EV_MOVE_REQUEST and the controller
 *  forwards the request only if that subsystem owns motion in the
 *  current state (S3 in FOLLOW_LINE, S5 in AVOID_OBSTACLE ...).
 * --------------------------------------------------------------------- */
/* Expected return heading of the U-turn, in S2 command degrees (encoder
   angle, VC_PIVOT_COUNTS).  The owner's runs (2026-11-24): a 180 request
   turned the body ~215 deg, 151 ~192, 141 ~185, so 141 x 180 / 185 = 137.
   Since 2026-11-25 the U-turn no longer ends here: the fast pivot stops
   VC_UTURN_APPROACH_DEG short of it, a slow pivot finds the return line,
   and VC_UTURN_SECTOR_DEG either side of it bounds the search (vc_sm.h).
   Only the U-turn uses it: LEFT / RIGHT end on the line and are accurate.
   Build override: -DVC_UTURN_CMD_DEG=<deg>. */
#ifndef VC_UTURN_CMD_DEG
#define VC_UTURN_CMD_DEG 137
#endif
_Static_assert((VC_UTURN_CMD_DEG >= 120) && (VC_UTURN_CMD_DEG <= 240),
               "VC_UTURN_CMD_DEG must stay a U-turn (120..240 deg)");

typedef enum
{
    VC_MOVE_STOP = 0, /* value unused; ordinary stop, no completion event */
    VC_MOVE_STOP_CONFIRM, /* stop and report S2's MFLG_STOPPED to the Vehicle Controller */
    VC_MOVE_FORWARD,  /* value = speed (mm/s) */
    VC_MOVE_STEER,    /* value = steering (-1000 left .. +1000 right) at current
                         speed */
    VC_MOVE_TURN_LEFT,  /* value = angle (deg) */
    VC_MOVE_TURN_RIGHT, /* value = angle (deg) */
    VC_MOVE_U_TURN,     /* value = angle (deg, left), 0 = VC_UTURN_CMD_DEG */
    VC_MOVE_REVERSE,    /* value = distance (mm) */
    VC_MOVE_SET_SPEED,  /* value = speed limit (mm/s) */
    VC_MOVE_DRIVE_DIST, /* value = distance (mm) forward at the set speed */
    VC_MOVE_SAFE_STOP,  /* motors off at once, latched (NFR8) */
    VC_MOVE_RESET,      /* clear the latch: controller only, never requested */
    VC_MOVE_PIVOT,      /* value = wheel speed (mm/s) of a pivot on the spot,
                           + = left (counter-clockwise), - = right.  Keeps
                           turning until the next move: the controller
                           refreshes it (S2 stops a DRIVE after 500 ms) and
                           ends it with STOP_CONFIRM.  Controller only. */
    VC_MOVE_COUNT
} vc_move_t;

/* ---------------------------------------------------------------------
 *  Requests from the controller to a subsystem (outbound, vc_ports.c)
 * --------------------------------------------------------------------- */
typedef enum
{
    VC_REQ_NONE = 0,
    VC_REQ_SELF_CHECK,   /* all: run self check, answer VC_EV_READY */
    VC_REQ_LINE_FOLLOW,  /* S3: follow the line, send steering */
    VC_REQ_LINE_SEARCH,  /* S3: controller is searching (p = attempt no.) */
    VC_REQ_BARCODE_READ, /* S3: read the barcode (p = retry no., 0 = first) */
    VC_REQ_BARCODE_OFF,  /* S3: ignore barcodes (while avoiding) */
    VC_REQ_OBST_AVOID,   /* S5: scan, profile and bypass */
    VC_REQ_OBST_MONITOR, /* S5: back to monitoring */
    VC_REQ_TERRAIN_MEASURE, /* S4: measure the hump */
    VC_REQ_STOP_ALL,        /* all: mission over / safe stop */
    VC_REQ_LINE_WATCH,      /* S3: p = 1 send every new sample as
                               VC_EV_LINE_SAMPLE (U-turn), 0 = stop */
    VC_REQ_COUNT
} vc_request_t;

/* ---------------------------------------------------------------------
 *  Controller status, read by any subsystem with vc_get_status()
 * --------------------------------------------------------------------- */
typedef struct
{
    vc_state_t           state;
    vc_state_t           prev_state;
    uint32_t             state_since_ms;
    uint32_t             mission_start_ms;
    /* last_fault_*: the latest report while no run has ended.  In FAULT and
       MISSION_COMPLETE they hold the cause that ENDED the run and do not
       change until Reset / Start: a lift is IMU / non-critical / 0x4005 in
       MISSION_COMPLETE, an overturn IMU / critical / 0x4005 in FAULT.
       Later reports are still sent (fault topic, console) and counted in
       reports_after_end. */
    uint32_t             last_fault_code;
    vc_fault_severity_t  last_fault_sev;
    uint8_t              last_fault_src;
    uint32_t             reports_after_end; /* reports since the run ended */
    navigation_command_t last_nav;    /* last barcode command */
    char                 last_barcode[16]; /* last decoded text ("A", "Z"); a letter
                                              that goes straight is still recorded */
    int32_t              max_hump_um; /* highest hump this run, micrometres */
    uint32_t             barcodes_read;
    uint32_t             events_handled;
    uint32_t             moves_rejected; /* requests from a non-owner */
    bool                 wifi_up;
    bool                 paused;
} vc_status_t;

#endif /* VC_TYPES_H */

/*** end of file ***/
