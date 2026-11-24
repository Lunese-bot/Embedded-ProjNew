/*
 *      s1_lock.c
 *      Subsystem 1 -- the S1 lock on target (see s1_lock.h).
 */

#include "subsystem1.h"
#include "s1_lock.h"

static ID                g_mtx_s1;
static volatile uint32_t g_s1_lock_fail;

int32_t s1_lock_init(void)
{
    T_CMTX cmtx = { 0 };

    cmtx.mtxatr = TA_INHERIT;
    g_mtx_s1    = tk_cre_mtx(&cmtx);
    return (g_mtx_s1 < E_OK) ? (int32_t)g_mtx_s1 : 0;
}

int32_t s1_lock(void)
{
    if (g_mtx_s1 <= 0)
    {
        return 0;
    }
    return (E_OK == tk_loc_mtx(g_mtx_s1, S1_LOCK_TMO_MS));
}

void s1_unlock(void)
{
    (void)tk_unl_mtx(g_mtx_s1);
}

/* Not atomic across cores: two simultaneous failures may count once. */
void s1_lock_failed(void)
{
    g_s1_lock_fail++;
}

uint32_t s1_lock_fail_count(void)
{
    return g_s1_lock_fail;
}

/*** end of file ***/
