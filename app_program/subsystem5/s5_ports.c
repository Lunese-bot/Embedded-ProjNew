/*
 *      s5_ports.c
 *      Subsystem 5 -- hooks to data owned by other subsystems.
 *
 *      Each hook is a WEAK default so subsystem 5 builds and runs on its
 *      own.  The owning subsystem replaces it by defining a normal
 *      (non-weak) function with the same name in its own folder; the
 *      linker then uses theirs.  Record each one as an interface in
 *      TEAM.md section 5.
 */

#include "subsystem5.h"
#include "../subsystem3/subsystem3.h"

/* During avoidance S3 updates the live mask, while its normal tracking
   status is paused. A standalone S5 build has no live mask. */
extern volatile uint8_t g_s3_black_mask_live __attribute__((weak));

__attribute__((weak)) void s5_port_set_line_search(bool b_active)
{
    (void)b_active;
}

/*
 * Line Following (Buddy 3): TRUE when C or R currently sees the
 * line.  Called every 20 ms while searching for the line after a
 * bypass, from s5_avoid_task, so it must not block.
 *
 * STUB: always FALSE, so on the bench every search runs its full
 * distance and the bounded recovery is exercised.
 */
__attribute__((weak)) bool s5_port_line_detected(void)
{
    return false;
}

bool s5_port_line_both_black(void)
{
    const uint8_t sensors = S3_MASK_C | S3_MASK_R;

    return (&g_s3_black_mask_live != NULL) &&
           (sensors == (g_s3_black_mask_live & sensors));
}

/*** end of file ***/
