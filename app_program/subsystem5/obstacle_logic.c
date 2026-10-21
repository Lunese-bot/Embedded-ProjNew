/*
 *	obstacle_logic.c
 *	Subsystem 5 -- obstacle profiling and bypass planning (no kernel).
 *
 *	Geometry, seen from above, sensor at the origin, x = forward:
 *	a sample (deg, mm) is a point mm * (cos deg, sin deg).  Its lateral
 *	offset is mm * sin(deg): positive = left of the car's centre line.
 *
 *	Bypass on side s (+1 left, -1 right), from the stopped car:
 *	    1. turn  s*45            -> head diagonally away from the line
 *	    2. drive L*sqrt2          -> L sideways, L forwards
 *	    3. turn -s*45            -> parallel to the line, beside obstacle
 *	    4. drive past the obstacle
 *	    5. turn -s*45            -> head back towards the line
 *	    6. drive (search) until an IR sensor sees the line
 *	    7. turn  s*45            -> original heading; line follower takes over
 *	L = lateral offset of the obstacle edge + half car width + margin.
 */

#include "obstacle_logic.h"

/* sin(0, 5, 10 ... 90 deg) * 1000 */
LOCAL const UH sin_tab[19] = {
	   0,   87,  174,  259,  342,  423,  500,  574,  643,  707,
	 766,  819,  866,  906,  940,  966,  985,  996, 1000
};

EXPORT INT s5_sin1000(INT deg)
{
	INT sign = 1;
	INT i;
	INT r;

	if (deg < 0) {
		sign = -1;
		deg = -deg;
	}
	if (deg > 90) {
		deg = 90;
	}
	i = deg / 5;
	r = deg % 5;
	if (i >= 18) {
		return sign * 1000;
	}
	return sign * (sin_tab[i] + ((sin_tab[i + 1] - sin_tab[i]) * r) / 5);
}

LOCAL BOOL is_hit(const S5_SAMPLE *s)
{
	return (s->mm != 0 && s->mm <= S5_HIT_RANGE_MM) ? TRUE : FALSE;
}

LOCAL INT abs_i(INT v)
{
	return (v < 0) ? -v : v;
}

EXPORT BOOL s5_build_profile(const S5_SAMPLE *s, INT n, S5_PROFILE *p)
{
	INT	i;
	INT	near = -1;
	INT	lo;
	INT	hi;
	INT	clr;
	UH	dmin;

	*p = (S5_PROFILE){0};
	if (s == NULL || n <= 0) {
		return FALSE;
	}

	/* 1. nearest hit; ties go to the one closest to straight ahead */
	for (i = 0; i < n; i++) {
		if (!is_hit(&s[i])) {
			continue;
		}
		if (near < 0 || s[i].mm < s[near].mm ||
		    (s[i].mm == s[near].mm &&
		     abs_i(s[i].deg) < abs_i(s[near].deg))) {
			near = i;
		}
	}
	if (near < 0) {
		return FALSE;
	}

	/* 2. grow the cluster while neighbours are hits at a similar range */
	lo = near;
	while (lo > 0 && is_hit(&s[lo - 1]) &&
	       abs_i((INT)s[lo - 1].mm - (INT)s[lo].mm) <= S5_CLUSTER_GAP_MM) {
		lo--;
	}
	hi = near;
	while (hi < n - 1 && is_hit(&s[hi + 1]) &&
	       abs_i((INT)s[hi + 1].mm - (INT)s[hi].mm) <= S5_CLUSTER_GAP_MM) {
		hi++;
	}

	dmin = s[near].mm;
	p->distance_mm    = dmin;
	p->right_edge_deg = s[lo].deg;
	p->left_edge_deg  = s[hi].deg;
	p->centre_deg     = (B)(((INT)s[lo].deg + (INT)s[hi].deg) / 2);
	p->right_mm = (H)(((INT)s[lo].mm * s5_sin1000(s[lo].deg)) / 1000);
	p->left_mm  = (H)(((INT)s[hi].mm * s5_sin1000(s[hi].deg)) / 1000);
	p->width_mm = (UH)abs_i((INT)p->left_mm - (INT)p->right_mm);

	/* 3. clearance: the shortest range seen beyond each edge.
	      No echo counts as the sensor's full range.  No sample at all
	      beyond an edge means "unknown", treated as 0 (not safe). */
	clr = -1;
	for (i = hi + 1; i < n; i++) {
		INT d = (s[i].mm == 0) ? 2000 : s[i].mm;
		if (clr < 0 || d < clr) {
			clr = d;
		}
	}
	p->clear_left_mm = (UH)((clr < 0) ? 0 : clr);

	clr = -1;
	for (i = 0; i < lo; i++) {
		INT d = (s[i].mm == 0) ? 2000 : s[i].mm;
		if (clr < 0 || d < clr) {
			clr = d;
		}
	}
	p->clear_right_mm = (UH)((clr < 0) ? 0 : clr);

	p->valid = 1;
	(void)s5_choose_side(p);
	return TRUE;
}

EXPORT INT s5_lateral_needed_mm(const S5_PROFILE *p, S5_SIDE side)
{
	INT edge;

	if (side == S5_SIDE_LEFT) {
		edge = p->left_mm;		/* + = left of centre */
	} else {
		edge = -(INT)p->right_mm;	/* mirror for the right */
	}
	if (edge < 0) {
		edge = 0;	/* obstacle entirely on the other side */
	}
	return edge + S5_ROBOT_HALF_WIDTH_MM + S5_SIDE_MARGIN_MM;
}

EXPORT S5_SIDE s5_choose_side(S5_PROFILE *p)
{
	BOOL	l_ok = (p->clear_left_mm  >= S5_MIN_CLEAR_MM) ? TRUE : FALSE;
	BOOL	r_ok = (p->clear_right_mm >= S5_MIN_CLEAR_MM) ? TRUE : FALSE;
	S5_SIDE	side;

	if (l_ok && r_ok) {
		/* both safe: the shorter detour, then the larger clearance */
		INT ll = s5_lateral_needed_mm(p, S5_SIDE_LEFT);
		INT lr = s5_lateral_needed_mm(p, S5_SIDE_RIGHT);
		if (ll != lr) {
			side = (ll < lr) ? S5_SIDE_LEFT : S5_SIDE_RIGHT;
		} else {
			side = (p->clear_left_mm > p->clear_right_mm)
				? S5_SIDE_LEFT : S5_SIDE_RIGHT;
		}
	} else if (l_ok) {
		side = S5_SIDE_LEFT;
	} else if (r_ok) {
		side = S5_SIDE_RIGHT;
	} else {
		side = S5_SIDE_NONE;
	}
	p->bypass_side = (UB)side;
	return side;
}

LOCAL INT add_step(S5_STEP *plan, INT n, INT max, UB kind, INT deg,
		   INT mm, INT speed, UB search)
{
	if (n >= max) {
		return n;
	}
	plan[n] = (S5_STEP){0};
	plan[n].m.kind        = kind;
	plan[n].m.angle_deg   = (B)deg;
	plan[n].m.distance_mm = (UH)((mm < 0) ? 0 : mm);
	plan[n].m.speed_mm_s  = (UH)speed;
	plan[n].search_line   = search;
	return n + 1;
}

EXPORT INT s5_plan_bypass(const S5_PROFILE *p, S5_STEP *plan, INT max)
{
	INT	n = 0;
	INT	s;
	INT	lat;
	INT	leg1;
	INT	leg2;

	n = add_step(plan, n, max, S5_MOVE_STOP, 0, 0, 0, 0);

	if (p->bypass_side == S5_SIDE_NONE || !p->valid) {
		/* nowhere to go: back off and let the controller decide */
		n = add_step(plan, n, max, S5_MOVE_REVERSE, 0, S5_REVERSE_MM,
			     S5_SEARCH_SPEED_MM_S, 0);
		return n;
	}

	s    = (p->bypass_side == S5_SIDE_LEFT) ? 1 : -1;
	lat  = s5_lateral_needed_mm(p, (S5_SIDE)p->bypass_side);
	leg1 = (lat * 1000) / s5_sin1000(S5_BYPASS_TURN_DEG);
	/* forwards already covered by leg 1 = lat (45 deg) */
	leg2 = (INT)p->distance_mm + S5_OBSTACLE_DEPTH_MM
	     + S5_ROBOT_LENGTH_MM - lat;
	if (leg2 < 50) {
		leg2 = 50;
	}

	n = add_step(plan, n, max, S5_MOVE_TURN, s * S5_BYPASS_TURN_DEG,
		     0, 0, 0);
	n = add_step(plan, n, max, S5_MOVE_FORWARD, 0, leg1,
		     S5_BYPASS_SPEED_MM_S, 0);
	n = add_step(plan, n, max, S5_MOVE_TURN, -s * S5_BYPASS_TURN_DEG,
		     0, 0, 0);
	n = add_step(plan, n, max, S5_MOVE_FORWARD, 0, leg2,
		     S5_BYPASS_SPEED_MM_S, 0);
	n = add_step(plan, n, max, S5_MOVE_TURN, -s * S5_BYPASS_TURN_DEG,
		     0, 0, 0);
	n = add_step(plan, n, max, S5_MOVE_FORWARD, 0,
		     leg1 + S5_SEARCH_EXTRA_MM, S5_SEARCH_SPEED_MM_S, 1);
	return n;
}

EXPORT INT s5_plan_recovery(S5_SIDE side, INT attempt, S5_STEP *plan,
			    INT max)
{
	INT n = 0;
	INT s = (side == S5_SIDE_LEFT) ? 1 : -1;

	/* turn a little more towards the line each time, then search */
	n = add_step(plan, n, max, S5_MOVE_TURN, -s * S5_RECOVERY_TURN_DEG,
		     0, 0, 0);
	n = add_step(plan, n, max, S5_MOVE_FORWARD, 0,
		     S5_RECOVERY_BASE_MM + attempt * S5_RECOVERY_STEP_MM,
		     S5_SEARCH_SPEED_MM_S, 1);
	return n;
}

EXPORT INT s5_coarse_angles(B *deg, INT max)
{
	LOCAL const B coarse[] = { -60, -30, 0, 30, 60 };
	INT i;
	INT n = (INT)(sizeof(coarse) / sizeof(coarse[0]));

	for (i = 0; i < n && i < max; i++) {
		deg[i] = coarse[i];
	}
	return i;
}

EXPORT INT s5_fine_angles(const S5_PROFILE *p, B *deg, INT max)
{
	/* 5-degree steps from 20 deg beyond the right edge to 20 deg
	   beyond the left edge, clamped to the servo range */
	INT a  = (INT)p->right_edge_deg - 20;
	INT hi = (INT)p->left_edge_deg + 20;
	INT n  = 0;

	if (a < -80) {
		a = -80;
	}
	if (hi > 80) {
		hi = 80;
	}
	for (; a <= hi && n < max; a += 5) {
		deg[n++] = (B)a;
	}
	return n;
}
