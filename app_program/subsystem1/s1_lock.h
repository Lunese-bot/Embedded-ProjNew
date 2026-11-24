/*
 *      s1_lock.h
 *      Subsystem 1 -- one short lock around S1's shared queue state and
 *      counters.  Producers on either core and the WiFi service task take
 *      it only to copy a few bytes; nobody holds it across an lwIP call or
 *      any blocking call.
 *
 *      Plain C types: s1_lock.c implements it with a T-Kernel mutex, and the
 *      host tests link testing/subsystem1/host_test/s1_lock_host.c instead.
 */

#ifndef S1_LOCK_H
#define S1_LOCK_H

#include <stdint.h>

/* Longest wait for the lock.  Holders never block, so this only bounds the
   time a caller can be delayed by a holder that was pre-empted. */
#define S1_LOCK_TMO_MS 5

/* Create the lock; 0 on success, a negative kernel error otherwise. */
int32_t s1_lock_init(void);

/* Take the lock: non-zero on success.  On failure (lock not created yet,
   or S1_LOCK_TMO_MS passed) the caller skips its work and calls
   s1_lock_failed(). */
int32_t s1_lock(void);
void    s1_unlock(void);

/* Best-effort count of failed s1_lock() calls (S1_STATS.lock_fail). */
void     s1_lock_failed(void);
uint32_t s1_lock_fail_count(void);

#endif /* S1_LOCK_H */

/*** end of file ***/
