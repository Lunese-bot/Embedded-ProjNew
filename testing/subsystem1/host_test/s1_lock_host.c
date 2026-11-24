/*
 *      s1_lock_host.c
 *      Host stand-in for app_program/subsystem1/s1_lock.c.
 *
 *      Single-threaded, so the lock only records how deeply it is held:
 *      host tests assert that it is free whenever a publish stub runs
 *      (never held across lwIP).  s1_lock_host_refuse makes the next
 *      s1_lock() calls fail, to exercise the lock-failure paths.
 */

#include "s1_lock.h"

int32_t  s1_lock_host_depth;
int32_t  s1_lock_host_refuse;
uint32_t s1_lock_host_fail;

int32_t s1_lock_init(void)
{
    s1_lock_host_depth = 0;
    return 0;
}

int32_t s1_lock(void)
{
    if (s1_lock_host_refuse > 0)
    {
        s1_lock_host_refuse--;
        return 0;
    }
    s1_lock_host_depth++;
    return 1;
}

void s1_unlock(void)
{
    s1_lock_host_depth--;
}

void s1_lock_failed(void)
{
    s1_lock_host_fail++;
}

uint32_t s1_lock_fail_count(void)
{
    return s1_lock_host_fail;
}

/*** end of file ***/
