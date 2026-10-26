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
    VC_SRC_VC     = 6  /* S6 itself (timeouts) */
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
    VC_EV_WIFI_DOWN, /* non-critical: keep navigating */
    VC_EV_WIFI_UP,

    /* S2 motion: send after every TURN_*, U_TURN, REVERSE, DRIVE_DIST */
    VC_EV_MANOEUVRE_DONE, /* p1 = 1 completed, 0 failed (navigation_complete) */

    /* S3 line + barcode */
    VC_EV_LINE_LOST,     /* p1 = last side seen: -1 left, +1 right, 0 unknown */
    VC_EV_LINE_FOUND,    /* line_reacquired */
    VC_EV_JUNCTION,      /* p1 = junction type (S3 defines) */
    VC_EV_BARCODE_START, /* barcode_detected: start of bars seen */
    VC_EV_BARCODE_DECODED, /* p1 = navigation_command_t, p2 = raw code */
    VC_EV_BARCODE_FAILED,  /* barcode_read_failed */
    VC_EV_END_OF_COURSE,

    /* S4 IMU */
    VC_EV_HUMP_START,   /* hump_detected */
    VC_EV_HUMP_CLEARED, /* p1 = peak height (um), 0 = aborted */
    VC_EV_IMPACT,       /* collision: critical */

    /* S5 obstacle */
    VC_EV_OBSTACLE,     /* p1 = distance (mm) */
    VC_EV_AVOID_DONE,   /* bypassed and line reacquired */
    VC_EV_AVOID_FAILED, /* bypass done but line not found: S6 searches */

    /* S6 internal */
    VC_EV_TICK, /* periodic, drives state timeouts */

    VC_EV_COUNT
} vc_event_type_t;

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
#define VC_FC_IMPACT        0x0005
#define VC_FC_STATE_TIMEOUT 0x0006 /* a state took too long */
#define VC_FC_BARCODE       0x0007 /* barcode retries used up (non-critical) */
#define VC_FC_WIFI_DOWN     0x1000 /* reported for S1 (non-critical) */

/* One event, as copied into the message buffer.  16 bytes. */
typedef struct
{
    uint8_t  src;  /* vc_source_t */
    uint8_t  type; /* vc_event_type_t */
    int16_t  rsv;  /* 0 */
    int32_t  p1;
    int32_t  p2;
    uint32_t time_ms; /* sender's tk_get_tim() low word, 0 if unknown */
} vc_event_t;

/* ---------------------------------------------------------------------
 *  Movement commands.  The controller is the ONLY source of these;
 *  other subsystems ask with VC_EV_MOVE_REQUEST and the controller
 *  forwards the request only if that subsystem owns motion in the
 *  current state (S3 in FOLLOW_LINE, S5 in AVOID_OBSTACLE ...).
 * --------------------------------------------------------------------- */
typedef enum
{
    VC_MOVE_STOP = 0, /* value unused */
    VC_MOVE_FORWARD,  /* value = speed (mm/s) */
    VC_MOVE_STEER,    /* value = steering (-1000 left .. +1000 right) at current
                         speed */
    VC_MOVE_TURN_LEFT,  /* value = angle (deg) */
    VC_MOVE_TURN_RIGHT, /* value = angle (deg) */
    VC_MOVE_U_TURN,     /* value unused */
    VC_MOVE_REVERSE,    /* value = distance (mm) */
    VC_MOVE_SET_SPEED,  /* value = speed limit (mm/s) */
    VC_MOVE_DRIVE_DIST, /* value = distance (mm) forward at the set speed */
    VC_MOVE_SAFE_STOP,  /* motors off at once, latched (NFR8) */
    VC_MOVE_RESET,      /* clear the latch: controller only, never requested */
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
    VC_REQ_STOP_ALL         /* all: mission over / safe stop */
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
    uint32_t             last_fault_code;
    vc_fault_severity_t  last_fault_sev;
    uint8_t              last_fault_src;
    navigation_command_t last_nav;    /* last barcode command */
    int32_t              max_hump_um; /* highest hump this run, micrometres */
    uint32_t             barcodes_read;
    uint32_t             events_handled;
    uint32_t             moves_rejected; /* requests from a non-owner */
    bool                 wifi_up;
} vc_status_t;

#endif /* VC_TYPES_H */

/*** end of file ***/
