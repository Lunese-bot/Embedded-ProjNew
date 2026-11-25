/*
 *      subsystem1.c
 *      Subsystem 1 -- telemetry: objects, task start, IF-T1.
 */

#include "subsystem1.h"

ID             g_tid_telemetry;
ID             g_mbf_telemetry; /* IF-T1 */
comms_status_t g_s1_stat;
s1_stats_t     g_s1_stats;
s1_txq_t       g_s1_txq;

static ID g_cyc_telemetry;

#if APP_TEST
ID g_mbf_s1_test;

static void capture_test_message(const uint8_t * p_line, int32_t len)
{
    uint8_t old[S1_LINE_MAX];
    ER      ercd = tk_snd_mbf(g_mbf_s1_test, p_line, len, TMO_POL);

    if (E_TMOUT == ercd)
    {
        int32_t received = tk_rcv_mbf(g_mbf_s1_test, old, TMO_POL);
        if ((received < E_OK) && (received != E_TMOUT))
        {
            app_check(received, "s1 test capture receive");
            return;
        }
        ercd = tk_snd_mbf(g_mbf_s1_test, p_line, len, TMO_POL);
    }
    app_check(ercd, "s1 test capture send");
}
#endif

/* Cyclic handler: wake telemetry_task every S1_PERIOD_MS */
static void telemetry_cyclic_isr(void * p_exinf)
{
    (void)p_exinf;
    tk_wup_tsk(g_tid_telemetry);
}

ER subsystem1_init(void)
{
    T_CMBF cmbf = { 0 };
    ER     ercd;

    ercd = s1_lock_init();
    if (ercd < E_OK)
    {
        return ercd;
    }
    cmbf.mbfatr     = TA_TFIFO;
    /* events only, at most S1_EVT_QUEUE_MAX, so size for the longest */
    cmbf.bufsz      = S1_EVT_QUEUE_MAX * (S1_LINE_MAX + 4);
    cmbf.maxmsz     = S1_LINE_MAX;
    g_mbf_telemetry = tk_cre_mbf(&cmbf);
    if (g_mbf_telemetry < E_OK)
    {
        return g_mbf_telemetry;
    }
#if APP_TEST
    cmbf.bufsz    = S1_QUEUE_LINES * (S1_LINE_MAX + sizeof(int32_t));
    g_mbf_s1_test = tk_cre_mbf(&cmbf);
    if (g_mbf_s1_test < E_OK)
    {
        return g_mbf_s1_test;
    }
#endif
    return E_OK;
}

ER subsystem1_start(void)
{
    T_CCYC ccyc = { 0 };

    g_tid_telemetry =
        app_start_task(telemetry_task, S1_PRI_TELEMETRY, 0, "telemetry_task");
    if (g_tid_telemetry < E_OK)
    {
        return g_tid_telemetry;
    }
    ccyc.cycatr     = TA_HLNG | TA_STA;
    ccyc.cychdr     = (FP)telemetry_cyclic_isr;
    ccyc.cyctim     = S1_PERIOD_MS;
    ccyc.cycphs     = S1_PERIOD_MS;
    g_cyc_telemetry = tk_cre_cyc(&ccyc);
    return (g_cyc_telemetry < E_OK) ? g_cyc_telemetry : E_OK;
}

/* Event path of s1_post(), S1 lock held.  At S1_EVT_QUEUE_MAX waiting
   events the oldest waiting one is evicted (the staged one is not in the
   buffer, so it cannot be). */
static ER post_event(const uint8_t * p_line, int32_t len, int32_t kind)
{
    uint8_t old[S1_LINE_MAX];
    int32_t old_len;
    ER      ercd;

    if (g_s1_txq.evt_waiting >= S1_EVT_QUEUE_MAX)
    {
        old_len = tk_rcv_mbf(g_mbf_telemetry, old, TMO_POL);
        if (old_len > 0)
        {
            g_s1_txq.evt_waiting--;
            g_s1_stat.dropped++;
            s1_stats_lost(&g_s1_stats, s1_stats_line_kind(old, old_len));
        }
    }
    /* counted before the send: a host test's buffer may hand the message
       straight to s1_bridge_pop(), which counts it out again */
    g_s1_txq.evt_waiting++;
    ercd = tk_snd_mbf(g_mbf_telemetry, p_line, len, TMO_POL);
    if (ercd < E_OK)
    {
        g_s1_txq.evt_waiting--;
        g_s1_stat.dropped++;
        s1_stats_lost(&g_s1_stats, kind);
    }
    else
    {
        g_s1_stats.evt_posted++;
    }
    return ercd;
}

/* IF-T1 */
ER s1_post(const char * p_topic, const void * p_data, int32_t data_len)
{
    uint8_t         line[S1_LINE_MAX];
    const uint8_t * p_data2   = (const uint8_t *)p_data;
    int32_t         topic_len = 0;
    int32_t         len;
    int32_t         idx;
    int32_t         kind;
    ER              ercd;

    if ((NULL == p_topic) || (data_len < 1) || (NULL == p_data))
    {
        return E_PAR;
    }
    while ((p_topic[topic_len] != '\0') && (topic_len < S1_BRIDGE_TOPIC_MAX))
    {
        topic_len++;
    }
    if ((p_topic[topic_len] != '\0') ||
        (data_len > (S1_LINE_MAX - topic_len - 1)))
    {
        return E_PAR;
    }

    for (idx = 0; idx < topic_len; idx++)
    {
        line[idx] = (uint8_t)p_topic[idx];
    }
    line[topic_len] = '\0';
    for (idx = 0; idx < data_len; idx++)
    {
        line[topic_len + 1 + idx] = p_data2[idx];
    }
    len = topic_len + 1 + data_len;

    kind = s1_stats_kind(p_topic);
    if (!s1_lock())
    {
        s1_lock_failed();
        return E_TMOUT; /* dropped: never wait longer than S1_LOCK_TMO_MS */
    }
    if (S1_KIND_TLM == kind)
    {
        s1_txq_post_tlm(&g_s1_txq, &g_s1_stats, line, len);
        ercd = E_OK;
    }
    else
    {
        ercd = post_event(line, len, kind);
    }
    s1_unlock();
#if APP_TEST
    if (E_OK == ercd)
    {
        capture_test_message(line, len);
    }
#endif
    return ercd;
}

/* IF-T1b */
ER s1_post_spare(const char * p_topic, const void * p_data, int32_t len,
                 int32_t spare)
{
    int32_t waiting;

    if ((spare < 1) || (spare >= S1_EVT_QUEUE_MAX) ||
        (S1_KIND_TLM == s1_stats_kind(p_topic)))
    {
        return E_PAR;
    }
    if (!s1_lock())
    {
        s1_lock_failed();
        return E_TMOUT;
    }
    waiting = g_s1_txq.evt_waiting;
    s1_unlock();
    if ((waiting + 1 + spare) > S1_EVT_QUEUE_MAX)
    {
        return E_LIMIT;
    }
    return s1_post(p_topic, p_data, len);
}

ER s1_get_status(comms_status_t * p_status)
{
    *p_status = g_s1_stat;
    return E_OK;
}

ER s1_get_stats(s1_stats_t * p_stats)
{
    if (!s1_lock())
    {
        s1_lock_failed();
        return E_TMOUT;
    }
    *p_stats = g_s1_stats;
    s1_unlock();
    p_stats->lock_fail = s1_lock_fail_count();
    return E_OK;
}

/*** end of file ***/
