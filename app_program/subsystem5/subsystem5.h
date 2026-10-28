/*
 *      subsystem5.h
 *      Subsystem 5 -- Ultrasonic Obstacle Detection and Avoidance (Buddy 5).
 *      Student ID: 2503330.
 *
 *      Design review refs: FR4, NFR2 (20 ms monitoring), Statechart 6.3,
 *      assumption A6.  Hardware: HC-SR04 style ultrasonic sensor on a
 *      hobby servo (SG90 style) that turns it left/right.
 *
 *      Public interface of subsystem 5.  Other subsystems include it as
 *      "../subsystem5/subsystem5.h".  Everything else stays LOCAL.
 *
 *      How subsystem 5 talks to the rest of the robot
 *      -----------------------------------------------
 *        Vehicle Controller -> S5   g_mbf_s5_cmd   s5_cmd_t   (owned by S5)
 *        S5 -> Vehicle Controller   g_mbf_s5_evt   s5_evt_t   (owned by S5 *)
 *        any task (e.g. telemetry)  s5_get_status()        snapshot, mutex
 *        S5 <- Line Following (S3)  s5_port_line_detected()  weak hook
 *
 *      (*) TEAM.md says the receiver owns a channel.  The Vehicle
 *          Controller did not exist when this was written, so S5 creates
 *          its outbound buffer.  Moving it to the controller later only
 *          changes where tk_cre_mbf is called.
 *
 *      S5 never drives the motors.  It asks for motion by sending
 *      S5_EVT_MOTION_REQ events; the Vehicle Controller (sole movement
 *      authority, design review 8.3) forwards them to Motion Control and
 *      answers with S5_CMD_MOTION_DONE carrying the same sequence number.
 */

/*
 *  Native vs packed (Tribunal 1 Version C):
 *    S5 <-> Vehicle Controller  native s5_evt_t / s5_cmd_t structs
 *    S5 -> telemetry / MQTT     packed 16-bit word, s5_get_status_packed()
 *                               (layout in s5_pack.h)    
 *  Safety: OBSTACLE_DETECTED goes through a latched stop path, so a
 *  full VC queue cannot leave the robot driving (s5_local.h).
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "s5_pack.h" /* 16-bit telemetry word */

/* ---------------------------------------------------------------------
 *  Pins (design review section 2; claim them in TEAM.md section 2)
 * --------------------------------------------------------------------- */
#define S5_PIN_SERVO 12 /* PWM slice 6 channel A */
#define S5_PIN_ECHO  16 /* input.  5 V echo: use a divider! */
#define S5_PIN_TRIG  17 /* output */

/* ---------------------------------------------------------------------
 *  Tasks.  TEAM.md gives S5 the band 20-21.  The design review rates
 *  both tasks "High", so the team may move this band up later.
 * --------------------------------------------------------------------- */
#define S5_PRI_AVOID 20 /* obstacle avoidance (event driven) */
#define S5_PRI_SCAN  21 /* detection + scanning (20 ms)      */

/* ---------------------------------------------------------------------
 *  Commands: Vehicle Controller -> S5, sent to g_mbf_s5_cmd
 * --------------------------------------------------------------------- */
typedef enum
{
    S5_CMD_ENABLE = 1,  /* start forward monitoring             */
    S5_CMD_DISABLE,     /* stop monitoring (servo parks centre) */
    S5_CMD_START_AVOID, /* car is stopped: scan, plan, bypass   */
    S5_CMD_MOTION_DONE, /* a motion request has finished        */
    S5_CMD_ABORT        /* stop avoiding now, back to IDLE      */
} s5_cmd_type_t;

typedef struct
{
    uint8_t  type; /* s5_cmd_type_t                                */
    uint8_t  ok;   /* MOTION_DONE: 1 = completed, 0 = failed     */
    uint16_t seq;  /* MOTION_DONE: seq of the request it answers */
} s5_cmd_t;

/* ---------------------------------------------------------------------
 *  Motion requests: S5 -> Vehicle Controller -> Motion Control (S2)
 *  Angles in degrees, positive = LEFT (counter-clockwise from above).
 * --------------------------------------------------------------------- */
typedef enum
{
    S5_MOVE_STOP = 1, /* stop both wheels                       */
    S5_MOVE_FORWARD,  /* drive distance_mm straight             */
    S5_MOVE_REVERSE,  /* drive distance_mm backwards            */
    S5_MOVE_TURN      /* spin in place by angle_deg             */
} s5_move_kind_t;

typedef struct
{
    uint8_t  kind;        /* s5_move_kind_t               */
    int8_t   angle_deg;   /* TURN: -90..+90, + = left   */
    uint16_t speed_mm_s;  /* FORWARD/REVERSE            */
    uint16_t distance_mm; /* FORWARD/REVERSE            */
    uint16_t seq;         /* echo back in MOTION_DONE   */
} s5_motion_req_t;

/* ---------------------------------------------------------------------
 *  Obstacle profile, produced by the coarse + fine scan
 * --------------------------------------------------------------------- */
typedef enum
{
    S5_SIDE_NONE  = 0, /* no safe side: reverse / give up */
    S5_SIDE_LEFT  = 1,
    S5_SIDE_RIGHT = 2
} s5_side_t;

typedef struct
{
    uint16_t distance_mm;    /* nearest point of the obstacle            */
    int8_t   left_edge_deg;  /* leftmost angle that still hits it        */
    int8_t   right_edge_deg; /* rightmost angle that still hits it       */
    int8_t   centre_deg;     /* middle of the two edges                  */
    int16_t  left_mm;        /* lateral offset of left edge  (+ = left)  */
    int16_t  right_mm;       /* lateral offset of right edge (+ = left)  */
    uint16_t width_mm;       /* left_mm - right_mm                       */
    uint16_t clear_left_mm;  /* free range seen beyond the left edge     */
    uint16_t clear_right_mm; /* free range seen beyond the right edge    */
    uint8_t  bypass_side;    /* s5_side_t chosen by the planner            */
    uint8_t  valid;          /* 1 = profile usable                       */
} s5_profile_t;

/* ---------------------------------------------------------------------
 *  Events: S5 -> Vehicle Controller, read from g_mbf_s5_evt
 * --------------------------------------------------------------------- */
typedef enum
{
    S5_EVT_OBSTACLE_DETECTED = 1, /* please stop, then START_AVOID */
    S5_EVT_PROFILE_READY,         /* profile filled in              */
    S5_EVT_MOTION_REQ,            /* please execute motion          */
    S5_EVT_LINE_REACQUIRED,       /* bypass done, back on the line  */
    S5_EVT_AVOID_FAILED,          /* recoverable: line not found    */
    S5_EVT_FAULT                  /* sensor/servo fault, see code   */
} s5_evt_type_t;

typedef enum
{
    S5_FAULT_NONE = 0,
    S5_FAULT_ECHO_TIMEOUT,   /* sensor silent N times in a row */
    S5_FAULT_MOTION_TIMEOUT, /* no MOTION_DONE in time         */
    S5_FAULT_NO_PROFILE,     /* scan did not finish            */
    S5_FAULT_BLOCKED         /* no side has enough clearance   */
} s5_fault_code_t;

typedef struct
{
    uint8_t  type;        /* s5_evt_type_t                         */
    uint8_t  code;        /* FAULT / AVOID_FAILED: s5_fault_code_t */
    uint16_t distance_mm; /* OBSTACLE_DETECTED                   */
    uint32_t time_ms;     /* system time when sent (low 32 bits) */
    union
    {
        s5_motion_req_t motion;  /* MOTION_REQ    */
        s5_profile_t    profile; /* PROFILE_READY */
    } u;
} s5_evt_t;

/* ---------------------------------------------------------------------
 *  Status snapshot for telemetry (Buddy 1) or the controller
 * --------------------------------------------------------------------- */
typedef enum
{
    S5_ST_IDLE = 0,   /* monitoring disabled              */
    S5_ST_MONITORING, /* pinging straight ahead, 20 ms    */
    S5_ST_DETECTED,   /* obstacle seen, waiting for stop  */
    S5_ST_COARSE_SCAN,
    S5_ST_FINE_SCAN,
    S5_ST_PLANNING,
    S5_ST_BYPASSING,
    S5_ST_SEARCH_LINE,   /* driving back towards the line    */
    S5_ST_LINE_RECOVERY, /* wider search, bounded retries    */
    S5_ST_ERROR
} s5_state_t;

typedef struct
{
    uint8_t      state;            /* s5_state_t                    */
    uint8_t      last_fault;       /* s5_fault_code_t               */
    uint16_t     last_distance_mm; /* 0 = no echo         */
    uint32_t     pings;            /* total measurements          */
    uint32_t     echo_timeouts;    /* total missing echoes        */
    uint32_t     obstacles;        /* obstacles detected          */
    uint32_t     avoid_ok;         /* bypasses that found line    */
    s5_profile_t profile;          /* most recent profile         */
} s5_status_t;

/* Public kernel objects (created in subsystem5_init) */
extern ID g_mbf_s5_cmd; /* Vehicle Controller -> S5, s5_cmd_t */
extern ID g_mbf_s5_evt; /* S5 -> Vehicle Controller, s5_evt_t */

/* Copy the current status.  Safe from any task (locks a mutex). */
extern ER s5_get_status(s5_status_t * p_out);

/* Minimum-bit telemetry: distance, state and detected flag packed into
   one 16-bit word (layout in s5_pack.h).  Safe from any task.
   Returns E_OK, or the error from s5_get_status (then *p_word = 0). */
extern ER s5_get_status_packed(uint16_t * p_word);

/* Hooks to other subsystems.  Defined __attribute__((weak)) in
   s5_ports.c; the owning subsystem overrides them with a real one. */
extern bool s5_port_line_detected(void); /* S3: any IR sees line */

#endif /* SUBSYSTEM5_H */

/*** end of file ***/
