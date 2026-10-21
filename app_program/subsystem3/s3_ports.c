/*
 *	s3_ports.c
 *	Subsystem 3 -- real versions of hooks that other subsystems declare
 *	weak (TEAM.md section 5).  A normal definition here replaces the
 *	weak default at link time; if that subsystem is not in the build,
 *	the function is simply unused.
 */

#include "subsystem3.h"

/*
 * IF-S5d (subsystem 5 obstacle avoidance, s5_ports.c): TRUE when any
 * IR sensor sees the line.  Called every 20 ms from s5_avoid_task while
 * it searches for the line after a bypass, so it must not block: it
 * reads the mask line_task stores every 10 ms, without the mutex.
 */
EXPORT BOOL s5_port_line_detected(void)
{
	return (s3_black_mask_live != 0) ? TRUE : FALSE;
}
