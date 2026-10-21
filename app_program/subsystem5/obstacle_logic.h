/*
 *	obstacle_logic.h
 *	Subsystem 5 -- pure algorithms: obstacle profiling and bypass
 *	planning.  No kernel calls and no hardware, so every function here
 *	can be unit-tested with made-up scan data (subsystem5_test.c).
 *
 *	All tuning values (A6, "needs validation" in design review 8.3) are
 *	collected here.  Change them after measuring, and record why.
 */

#ifndef OBSTACLE_LOGIC_H
#define OBSTACLE_LOGIC_H

#include "subsystem5.h"

/* ---- Detection -------------------------------------------------- */
#define S5_DETECT_MM		250	/* obstacle closer than this ...   */
#define S5_DETECT_COUNT		3	/* ... this many pings in a row     */
#define S5_HIT_RANGE_MM		500	/* during a scan: "part of it"      */
#define S5_CLUSTER_GAP_MM	150	/* jump that separates two objects  */

/* ---- Robot geometry (measure the real car) ---------------------- */
#define S5_ROBOT_HALF_WIDTH_MM	80
#define S5_ROBOT_LENGTH_MM	200
#define S5_SIDE_MARGIN_MM	60	/* extra gap beside the obstacle    */
#define S5_OBSTACLE_DEPTH_MM	150	/* assumed: ultrasonic can't see it */
#define S5_MIN_CLEAR_MM		300	/* free range needed on bypass side */

/* ---- Bypass manoeuvre ------------------------------------------- */
#define S5_BYPASS_TURN_DEG	45
#define S5_BYPASS_SPEED_MM_S	150
#define S5_SEARCH_SPEED_MM_S	100
#define S5_SEARCH_EXTRA_MM	100	/* drive this far past expected line */
#define S5_REVERSE_MM		150	/* when no side is safe              */

/* ---- Line recovery (bounded, NFR5) ------------------------------ */
#define S5_MAX_RECOVERY		3
#define S5_RECOVERY_TURN_DEG	20
#define S5_RECOVERY_BASE_MM	100
#define S5_RECOVERY_STEP_MM	50

/* One ultrasonic sample taken during a scan */
typedef struct {
	B	deg;		/* servo angle, + = left               */
	UH	mm;		/* distance, 0 = no echo (free space)  */
} S5_SAMPLE;

/* One step of a manoeuvre */
typedef struct {
	S5_MOTION_REQ	m;
	UB		search_line;	/* 1 = stop early when line seen */
} S5_STEP;

#define S5_PLAN_MAX		10

/* sin(deg) * 1000, deg in -90..+90 */
IMPORT INT	s5_sin1000(INT deg);

/* Build a profile from samples sorted by angle (most negative first).
   Returns TRUE and fills *p when an obstacle is found. */
IMPORT BOOL	s5_build_profile(const S5_SAMPLE *s, INT n, S5_PROFILE *p);

/* Pick the bypass side for a profile (also stored in p->bypass_side). */
IMPORT S5_SIDE	s5_choose_side(S5_PROFILE *p);

/* Lateral offset (mm) the car must move sideways to clear the obstacle
   on the given side. */
IMPORT INT	s5_lateral_needed_mm(const S5_PROFILE *p, S5_SIDE side);

/* Steps of the bypass for p->bypass_side.  Returns the step count. */
IMPORT INT	s5_plan_bypass(const S5_PROFILE *p, S5_STEP *plan, INT max);

/* Steps of recovery attempt (0-based) after the line was not found. */
IMPORT INT	s5_plan_recovery(S5_SIDE side, INT attempt,
				 S5_STEP *plan, INT max);

/* Angles for the coarse scan, and the fine scan around a profile */
IMPORT INT	s5_coarse_angles(B *deg, INT max);
IMPORT INT	s5_fine_angles(const S5_PROFILE *p, B *deg, INT max);

#endif /* OBSTACLE_LOGIC_H */
