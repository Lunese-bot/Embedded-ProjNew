/*
 *      s1_host_kernel.c
 *      Compiles the real subsystem1.c and s1_bridge.c against the host
 *      stand-ins declared in s1_host_kernel.h.
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "s1_host_kernel.h"

#include "../../../app_program/subsystem1/subsystem1.c"
#include "../../../app_program/subsystem1/s1_bridge.c"

#define HOST_MSG_MAX 256 /* messages per buffer */
#define HOST_HEADER  4   /* kernel HEADERSZ */

typedef struct
{
    int32_t capacity;
    int32_t head;
    uint8_t data[HOST_MSG_MAX][S1_LINE_MAX];
    int32_t len[HOST_MSG_MAX];
} host_mbf_t;

static host_mbf_t g_host_mbf[HOST_MBF_MAX];
static int32_t    g_host_mbf_created;

int32_t host_mbf_used[HOST_MBF_MAX];
int32_t host_mbf_count[HOST_MBF_MAX];
int32_t host_vc_calls;
int32_t host_vc_result;
int32_t host_app_check_failures;
char    host_last_printf[512];
uint32_t host_latch_stop_before;
uint32_t host_latch_stop_after;
int32_t  host_latch_status_ok = 1;
uint32_t host_latch_safety;

void s1_latch_read(s1_latch_t * p_out)
{
    s1_latch_resolve(p_out, host_latch_stop_before, host_latch_status_ok,
                     host_latch_safety, host_latch_stop_after);
}

static int32_t msg_cost(int32_t len)
{
    return HOST_HEADER + ((len + 3) & ~3);
}

static host_mbf_t * mbf_of(ID id)
{
    if ((id < 1) || (id > g_host_mbf_created))
    {
        return NULL;
    }
    return &g_host_mbf[id - 1];
}

ID tk_cre_mbf(T_CMBF * p_cmbf)
{
    if (g_host_mbf_created >= HOST_MBF_MAX)
    {
        return -1;
    }
    g_host_mbf[g_host_mbf_created].capacity =
        (int32_t)((p_cmbf->bufsz + 3U) & ~3U);
    g_host_mbf_created++;
    return g_host_mbf_created;
}

ER tk_snd_mbf(ID id, const void * p_data, int len, int timeout)
{
    host_mbf_t * p_mbf = mbf_of(id);
    int32_t      slot;

    (void)timeout;
    if ((p_mbf == NULL) || (len < 1) || (len > S1_LINE_MAX))
    {
        return E_PAR;
    }
    if ((host_mbf_used[id - 1] + msg_cost(len) > p_mbf->capacity) ||
        (host_mbf_count[id - 1] >= HOST_MSG_MAX))
    {
        return E_TMOUT;
    }
    slot = (p_mbf->head + host_mbf_count[id - 1]) % HOST_MSG_MAX;
    memcpy(p_mbf->data[slot], p_data, (size_t)len);
    p_mbf->len[slot] = len;
    host_mbf_count[id - 1]++;
    host_mbf_used[id - 1] += msg_cost(len);
    return E_OK;
}

int32_t tk_rcv_mbf(ID id, void * p_data, int timeout)
{
    host_mbf_t * p_mbf = mbf_of(id);
    int32_t      len;

    (void)timeout;
    if (p_mbf == NULL)
    {
        return -18; /* E_ID */
    }
    if (host_mbf_count[id - 1] == 0)
    {
        return E_TMOUT;
    }
    len = p_mbf->len[p_mbf->head];
    memcpy(p_data, p_mbf->data[p_mbf->head], (size_t)len);
    p_mbf->head = (p_mbf->head + 1) % HOST_MSG_MAX;
    host_mbf_count[id - 1]--;
    host_mbf_used[id - 1] -= msg_cost(len);
    return len;
}

ER tk_wup_tsk(ID id)
{
    (void)id;
    return E_OK;
}

ID tk_cre_cyc(T_CCYC * p_ccyc)
{
    (void)p_ccyc;
    return 1;
}

ID app_start_task(FP entry, int pri, uint32_t prc, const char * p_name)
{
    (void)entry;
    (void)pri;
    (void)prc;
    (void)p_name;
    return 1;
}

void app_check(int32_t ercd, const char * p_what)
{
    (void)p_what;
    if (ercd < E_OK)
    {
        host_app_check_failures++;
    }
}

ER vc_post_event(int src, int type, int p1, int p2, int timeout)
{
    (void)src;
    (void)type;
    (void)p1;
    (void)p2;
    (void)timeout;
    host_vc_calls++;
    return host_vc_result;
}

ER tk_get_tim(SYSTIM * p_time)
{
    p_time->lo = 0;
    return E_OK;
}

ER tk_dly_tsk(int delay)
{
    (void)delay;
    return E_OK;
}

int tm_printf(const UB * p_format, ...)
{
    va_list args;
    int     result;

    va_start(args, p_format);
    result = vsnprintf(host_last_printf, sizeof(host_last_printf),
                       (const char *)p_format, args);
    va_end(args);
    return result;
}

void telemetry_task(int32_t stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;
}

void host_kernel_reset(void)
{
    memset(g_host_mbf, 0, sizeof(g_host_mbf));
    memset(host_mbf_used, 0, sizeof(host_mbf_used));
    memset(host_mbf_count, 0, sizeof(host_mbf_count));
    g_host_mbf_created      = 0;
    host_vc_calls           = 0;
    host_vc_result          = E_OK;
    host_app_check_failures = 0;
    memset(&g_s1_stat, 0, sizeof(g_s1_stat));
    memset(&g_s1_stats, 0, sizeof(g_s1_stats));
    g_mbf_telemetry = 0;
    s1_txq_reset(&g_s1_txq);
    s_b_unfinished  = false;
    host_latch_stop_before = 0U;
    host_latch_stop_after  = 0U;
    host_latch_status_ok   = 1;
    host_latch_safety      = 0U;
}

/*** end of file ***/
