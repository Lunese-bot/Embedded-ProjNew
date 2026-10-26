/*
 *      subsystem1.c
 *      Subsystem 1 -- telemetry: objects, task start, IF-T1.
 */

#include "subsystem1.h"

ID             g_tid_telemetry;
ID             g_mbf_telemetry; /* IF-T1 */
comms_status_t g_s1_stat;

static ID g_cyc_telemetry;

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
    return (g_mbf_telemetry < E_OK) ? g_mbf_telemetry : E_OK;
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
ER s1_post(const char * p_topic, const char * p_text)
{
    char    line[S1_LINE_MAX];
    int32_t len = 0;
    int32_t idx;
    ER      ercd;

    for (idx = 0; (p_topic[idx] != '\0') && (len < S1_BRIDGE_TOPIC_MAX); idx++)
    {
        line[len++] = p_topic[idx];
    }
    line[len++] = '\0';
    for (idx = 0; (p_text[idx] != '\0') && (len < S1_LINE_MAX - 1); idx++)
    {
        line[len++] = p_text[idx];
    }
    line[len++] = '\0';

    ercd = tk_snd_mbf(g_mbf_telemetry, line, len, TMO_POL);
    if (ercd < E_OK)
    {
        /* full: drop the OLDEST line so telemetry stays fresh */
        char old[S1_LINE_MAX];
        (void)tk_rcv_mbf(g_mbf_telemetry, old, TMO_POL);
        g_s1_stat.dropped++;
        ercd = tk_snd_mbf(g_mbf_telemetry, line, len, TMO_POL);
    }
    return ercd;
}

ER s1_get_status(comms_status_t * p_status)
{
    *p_status = g_s1_stat;
    return E_OK;
}

/*** end of file ***/
