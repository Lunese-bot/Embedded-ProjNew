/*
 *      subsystem1.c
 *      Subsystem 1 -- telemetry: objects, task start, IF-T1.
 */

#include "subsystem1.h"

ID             g_tid_telemetry;
ID             g_mbf_telemetry; /* IF-T1 */
comms_status_t g_s1_stat;

static ID g_cyc_telemetry;

#if APP_TEST
ID g_mbf_s1_test;

static void capture_test_message(const uint8_t * p_line, int32_t len)
{
    uint8_t old[S1_LINE_MAX];
    ER ercd = tk_snd_mbf(g_mbf_s1_test, p_line, len, TMO_POL);

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

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = S1_QUEUE_LINES * ((S1_LINE_MAX / 2) + 4); /* typical line */
    cmbf.maxmsz = S1_LINE_MAX;
    g_mbf_telemetry = tk_cre_mbf(&cmbf);
    if (g_mbf_telemetry < E_OK)
    {
        return g_mbf_telemetry;
    }
#if APP_TEST
    cmbf.bufsz = S1_QUEUE_LINES * (S1_LINE_MAX + sizeof(int32_t));
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

/* IF-T1 */
ER s1_post(const char * p_topic, const void * p_data, int32_t data_len)
{
    uint8_t line[S1_LINE_MAX];
    const uint8_t * data = (const uint8_t *)p_data;
    int32_t topic_len = 0;
    int32_t len;
    int32_t idx;
    ER      ercd;

    if ((p_topic == NULL) || (data_len < 1) || (p_data == NULL))
    {
        return E_PAR;
    }
    while ((p_topic[topic_len] != '\0') &&
           (topic_len < S1_BRIDGE_TOPIC_MAX))
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
        line[topic_len + 1 + idx] = data[idx];
    }
    len = topic_len + 1 + data_len;

    ercd = tk_snd_mbf(g_mbf_telemetry, line, len, TMO_POL);
    if (ercd < E_OK)
    {
        /* full: drop the OLDEST line so telemetry stays fresh */
        uint8_t old[S1_LINE_MAX];
        (void)tk_rcv_mbf(g_mbf_telemetry, old, TMO_POL);
        g_s1_stat.dropped++;
        ercd = tk_snd_mbf(g_mbf_telemetry, line, len, TMO_POL);
    }
#if APP_TEST
    if (E_OK == ercd)
    {
        capture_test_message(line, len);
    }
#endif
    return ercd;
}

ER s1_get_status(comms_status_t * p_status)
{
    *p_status = g_s1_stat;
    return E_OK;
}

/*** end of file ***/
