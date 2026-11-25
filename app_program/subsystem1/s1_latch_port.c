/*
 *      s1_latch_port.c
 *      Subsystem 1 -- read the IMU stop interlock for the heartbeat
 *      (s1_stats.h, "IMU stop interlock").  Target only: the host tests
 *      provide their own s1_latch_read().
 *
 *      S4's task sets g_s2_imu_stop (s2_latch_imu_stop) and then publishes
 *      the status that holds the cause; on set-down it clears the interlock
 *      and then publishes NONE (task_imu.c).  Reading the interlock before
 *      and after the status lets s1_latch_resolve() report "unknown" rather
 *      than a cause that belongs to another moment.
 */

#include "subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem4/subsystem4.h"

void s1_latch_read(s1_latch_t * p_out)
{
    imu_status_t status;
    uint32_t     before;
    uint32_t     after;
    int32_t      b_ok;

    before = g_s2_imu_stop;
    b_ok   = (E_OK == s4_get_status(&status)) ? 1 : 0; /* bounded wait */
    after  = g_s2_imu_stop;
    s1_latch_resolve(p_out, before, b_ok,
                     (b_ok != 0) ? (uint32_t)status.safety_stop : 0U, after);
}

/*** end of file ***/
