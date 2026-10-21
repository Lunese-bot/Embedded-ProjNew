/*
 *	subsystem5.h
 *	Subsystem 5 -- Ultrasonic Obstacle Detection and Avoidance (Buddy 5).
 *	Student ID: 2503330.
 *
 *	Design review refs: FR4, NFR2 (20 ms monitoring), Statechart 6.3,
 *	assumption A6.  Hardware: HC-SR04 style ultrasonic sensor on a
 *	hobby servo (SG90 style) that turns it left/right.
 *
 *	Public interface of subsystem 5.  Other subsystems include it as
 *	"../subsystem5/subsystem5.h".  Everything else stays LOCAL.
 *
 *	How subsystem 5 talks to the rest of the robot
 *	-----------------------------------------------
 *	  Vehicle Controller -> S5   mbf_s5_cmd   S5_CMD   (owned by S5)
 *	  S5 -> Vehicle Controller   mbf_s5_evt   S5_EVT   (owned by S5 *)
 *	  any task (e.g. telemetry)  s5_get_status()        snapshot, mutex
 *	  S5 <- Line Following (S3)  s5_port_line_detected()  weak hook
 *
 *	(*) TEAM.md says the receiver owns a channel.  The Vehicle
 *	    Controller did not exist when this was written, so S5 creates
 *	    its outbound buffer.  Moving it to the controller later only
 *	    changes where tk_cre_mbf is called.
 *
 *	S5 never drives the motors.  It asks for motion by sending
 *	S5_EVT_MOTION_REQ events; the Vehicle Controller (sole movement
 *	authority, design review 8.3) forwards them to Motion Control and
 *	answers with S5_CMD_MOTION_DONE carrying the same sequence number.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "buddy1_mqtt.h"

<<<<<<< HEAD
#define BUDDY1_TELEMETRY_DEPTH	4
#define BUDDY1_COMMAND_DEPTH	4

/* Applications may enqueue telemetry without waiting. If its bounded queue
   is full, the caller receives E_TMOUT and can count/report the dropped
   update. MQTT drains the queue to the newest snapshot every 200 ms. */
IMPORT ER subsystem5_publish_telemetry(const BUDDY1_TELEMETRY *telemetry);

/* The Vehicle Controller consumes validated MQTT commands from this queue. */
IMPORT ER subsystem5_receive_command(BUDDY1_COMMAND *command, TMO timeout);
=======
/* ---------------------------------------------------------------------
 *  Pins (design review section 2; claim them in TEAM.md section 2)
 * --------------------------------------------------------------------- */
#define S5_PIN_SERVO		12	/* PWM slice 6 channel A */
#define S5_PIN_ECHO		16	/* input.  5 V echo: use a divider! */
#define S5_PIN_TRIG		17	/* output */

/* ---------------------------------------------------------------------
 *  Tasks.  TEAM.md gives S5 the band 20-21.  The design review rates
 *  both tasks "High", so the team may move this band up later.
 * --------------------------------------------------------------------- */
#define S5_PRI_AVOID		20	/* obstacle avoidance (event driven) */
#define S5_PRI_SCAN		21	/* detection + scanning (20 ms)      */

/* ---------------------------------------------------------------------
 *  Commands: Vehicle Controller -> S5, sent to mbf_s5_cmd
 * --------------------------------------------------------------------- */
typedef enum {
	S5_CMD_ENABLE = 1,	/* start forward monitoring             */
	S5_CMD_DISABLE,		/* stop monitoring (servo parks centre) */
	S5_CMD_START_AVOID,	/* car is stopped: scan, plan, bypass   */
	S5_CMD_MOTION_DONE,	/* a motion request has finished        */
	S5_CMD_ABORT		/* stop avoiding now, back to IDLE      */
} S5_CMD_TYPE;

typedef struct {
	UB	type;		/* S5_CMD_TYPE                                */
	UB	ok;		/* MOTION_DONE: 1 = completed, 0 = failed     */
	UH	seq;		/* MOTION_DONE: seq of the request it answers */
} S5_CMD;

/* ---------------------------------------------------------------------
 *  Motion requests: S5 -> Vehicle Controller -> Motion Control (S2)
 *  Angles in degrees, positive = LEFT (counter-clockwise from above).
 * --------------------------------------------------------------------- */
typedef enum {
	S5_MOVE_STOP = 1,	/* stop both wheels                       */
	S5_MOVE_FORWARD,	/* drive distance_mm straight             */
	S5_MOVE_REVERSE,	/* drive distance_mm backwards            */
	S5_MOVE_TURN		/* spin in place by angle_deg             */
} S5_MOVE_KIND;

typedef struct {
	UB	kind;		/* S5_MOVE_KIND               */
	B	angle_deg;	/* TURN: -90..+90, + = left   */
	UH	speed_mm_s;	/* FORWARD/REVERSE            */
	UH	distance_mm;	/* FORWARD/REVERSE            */
	UH	seq;		/* echo back in MOTION_DONE   */
} S5_MOTION_REQ;

/* ---------------------------------------------------------------------
 *  Obstacle profile, produced by the coarse + fine scan
 * --------------------------------------------------------------------- */
typedef enum {
	S5_SIDE_NONE  = 0,	/* no safe side: reverse / give up */
	S5_SIDE_LEFT  = 1,
	S5_SIDE_RIGHT = 2
} S5_SIDE;

typedef struct {
	UH	distance_mm;	/* nearest point of the obstacle            */
	B	left_edge_deg;	/* leftmost angle that still hits it        */
	B	right_edge_deg;	/* rightmost angle that still hits it       */
	B	centre_deg;	/* middle of the two edges                  */
	H	left_mm;	/* lateral offset of left edge  (+ = left)  */
	H	right_mm;	/* lateral offset of right edge (+ = left)  */
	UH	width_mm;	/* left_mm - right_mm                       */
	UH	clear_left_mm;	/* free range seen beyond the left edge     */
	UH	clear_right_mm;	/* free range seen beyond the right edge    */
	UB	bypass_side;	/* S5_SIDE chosen by the planner            */
	UB	valid;		/* 1 = profile usable                       */
} S5_PROFILE;

/* ---------------------------------------------------------------------
 *  Events: S5 -> Vehicle Controller, read from mbf_s5_evt
 * --------------------------------------------------------------------- */
typedef enum {
	S5_EVT_OBSTACLE_DETECTED = 1,	/* please stop, then START_AVOID */
	S5_EVT_PROFILE_READY,		/* profile filled in              */
	S5_EVT_MOTION_REQ,		/* please execute motion          */
	S5_EVT_LINE_REACQUIRED,		/* bypass done, back on the line  */
	S5_EVT_AVOID_FAILED,		/* recoverable: line not found    */
	S5_EVT_FAULT			/* sensor/servo fault, see code   */
} S5_EVT_TYPE;

typedef enum {
	S5_FAULT_NONE = 0,
	S5_FAULT_ECHO_TIMEOUT,		/* sensor silent N times in a row */
	S5_FAULT_MOTION_TIMEOUT,	/* no MOTION_DONE in time         */
	S5_FAULT_NO_PROFILE,		/* scan did not finish            */
	S5_FAULT_BLOCKED		/* no side has enough clearance   */
} S5_FAULT_CODE;

typedef struct {
	UB	type;		/* S5_EVT_TYPE                         */
	UB	code;		/* FAULT / AVOID_FAILED: S5_FAULT_CODE */
	UH	distance_mm;	/* OBSTACLE_DETECTED                   */
	UW	time_ms;	/* system time when sent (low 32 bits) */
	union {
		S5_MOTION_REQ	motion;		/* MOTION_REQ    */
		S5_PROFILE	profile;	/* PROFILE_READY */
	} u;
} S5_EVT;

/* ---------------------------------------------------------------------
 *  Status snapshot for telemetry (Buddy 1) or the controller
 * --------------------------------------------------------------------- */
typedef enum {
	S5_ST_IDLE = 0,		/* monitoring disabled              */
	S5_ST_MONITORING,	/* pinging straight ahead, 20 ms    */
	S5_ST_DETECTED,		/* obstacle seen, waiting for stop  */
	S5_ST_COARSE_SCAN,
	S5_ST_FINE_SCAN,
	S5_ST_PLANNING,
	S5_ST_BYPASSING,
	S5_ST_SEARCH_LINE,	/* driving back towards the line    */
	S5_ST_LINE_RECOVERY,	/* wider search, bounded retries    */
	S5_ST_ERROR
} S5_STATE;

typedef struct {
	UB		state;		/* S5_STATE                    */
	UB		last_fault;	/* S5_FAULT_CODE               */
	UH		last_distance_mm;	/* 0 = no echo         */
	UW		pings;		/* total measurements          */
	UW		echo_timeouts;	/* total missing echoes        */
	UW		obstacles;	/* obstacles detected          */
	UW		avoid_ok;	/* bypasses that found line    */
	S5_PROFILE	profile;	/* most recent profile         */
} S5_STATUS;

/* Public kernel objects (created in subsystem5_init) */
IMPORT ID	mbf_s5_cmd;	/* Vehicle Controller -> S5, S5_CMD */
IMPORT ID	mbf_s5_evt;	/* S5 -> Vehicle Controller, S5_EVT */

/* Copy the current status.  Safe from any task (locks a mutex). */
IMPORT ER	s5_get_status(S5_STATUS *out);

/* Hooks to other subsystems.  Defined __attribute__((weak)) in
   s5_ports.c; the owning subsystem overrides them with a real one. */
IMPORT BOOL	s5_port_line_detected(void);	/* S3: any IR sees line */
>>>>>>> test

#endif /* SUBSYSTEM5_H */
