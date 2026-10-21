/*
 *	s5_ports.c
 *	Subsystem 5 -- hooks to data owned by other subsystems.
 *
 *	Each hook is a WEAK default so subsystem 5 builds and runs on its
 *	own.  The owning subsystem replaces it by defining a normal
 *	(non-weak) function with the same name in its own folder; the
 *	linker then uses theirs.  Record each one as an interface in
 *	TEAM.md section 5.
 */

#include "subsystem5.h"

/*
 * Line Following (Buddy 3): TRUE when any IR sensor currently sees the
 * line.  Called every 20 ms while searching for the line after a
 * bypass, from s5_avoid_task, so it must not block.
 *
 * STUB: always FALSE, so on the bench every search runs its full
 * distance and the bounded recovery is exercised.
 */
EXPORT __attribute__((weak)) BOOL s5_port_line_detected(void)
{
	return FALSE;
}
