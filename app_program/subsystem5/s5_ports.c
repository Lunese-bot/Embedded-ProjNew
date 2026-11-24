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
#include "../subsystem2/subsystem2.h"
#include "../subsystem2/s2_config.h" /* S2_MM_PER_TICK */
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

/* S2 odometer: left + right encoder counts, one count is S2_MM_PER_TICK / 2
   of vehicle travel (subsystem2.h).  A standalone S5 build has none.  The
   count also rises while turning, so callers take differences only over a
   straight drive. */
extern volatile uint32_t g_s2_odo_ticks __attribute__((weak));

uint32_t s5_port_odo_mm(void)
{
    uint32_t ticks;

    if (&g_s2_odo_ticks == NULL)
    {
        return 0U;
    }
    ticks = g_s2_odo_ticks;
    /* Integer maths on a compile-time constant (0.5 x S2_MM_PER_TICK in
       units of 1/100000 mm), so no float rounding on a growing count. */
    return (uint32_t)(((uint64_t)ticks * (uint32_t)(S2_MM_PER_TICK * 50000.0f)) /
                      100000U);
}

bool s5_port_line_both_black(void)
{
    const uint8_t sensors = S3_MASK_C | S3_MASK_R;

    return (&g_s3_black_mask_live != NULL) &&
           (sensors == (g_s3_black_mask_live & sensors));
}

/*** end of file ***/
