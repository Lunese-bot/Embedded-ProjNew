/*
 *	subsystem1.c
 *	Subsystem 1 -- telemetry: objects, task start, IF-T1.
 */

#include "subsystem1.h"

EXPORT ID		tid_telemetry;
EXPORT ID		mbf_telemetry;	/* IF-T1 */
EXPORT COMMS_STATUS	s1_stat;

LOCAL ID		cyc_telemetry;

/* Cyclic handler: wake telemetry_task every S1_PERIOD_MS */
LOCAL void telemetry_cyclic(void *exinf)
{
	(void)exinf;
	tk_wup_tsk(tid_telemetry);
}

EXPORT ER subsystem1_init(void)
{
	T_CMBF	cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = S1_QUEUE_LINES * (S1_LINE_MAX / 2 + 4);	/* typical line */
	cmbf.maxmsz = S1_LINE_MAX;
	mbf_telemetry = tk_cre_mbf(&cmbf);
	return (mbf_telemetry < E_OK) ? mbf_telemetry : E_OK;
}

EXPORT ER subsystem1_start(void)
{
	T_CCYC	ccyc = {0};

	tid_telemetry = app_start_task(telemetry_task, S1_PRI_TELEMETRY, 0,
				       "telemetry_task");
	if (tid_telemetry < E_OK) {
		return tid_telemetry;
	}
	ccyc.cycatr = TA_HLNG | TA_STA;
	ccyc.cychdr = (FP)telemetry_cyclic;
	ccyc.cyctim = S1_PERIOD_MS;
	ccyc.cycphs = S1_PERIOD_MS;
	cyc_telemetry = tk_cre_cyc(&ccyc);
	return (cyc_telemetry < E_OK) ? cyc_telemetry : E_OK;
}

/* IF-T1: stored as "<topic suffix>\0<payload bytes>" */
EXPORT ER s1_post(const char *topic, const void *data, INT len)
{
	UB	line[S1_LINE_MAX];
	const UB *src = (const UB *)data;
	INT	k = 0, i;
	ER	er;

	if (len < 1) {
		return E_PAR;
	}
	for (i = 0; topic[i] != '\0'; i++) {
		if (k >= S1_BRIDGE_TOPIC_MAX) {
			return E_PAR;
		}
		line[k++] = (UB)topic[i];
	}
	if (k + 1 + len > S1_LINE_MAX) {
		return E_PAR;
	}
	line[k++] = '\0';
	for (i = 0; i < len; i++) {
		line[k++] = src[i];
	}

	er = tk_snd_mbf(mbf_telemetry, line, k, TMO_POL);
	if (er < E_OK) {
		/* full: drop the OLDEST line so telemetry stays fresh */
		UB	old[S1_LINE_MAX];
		(void)tk_rcv_mbf(mbf_telemetry, old, TMO_POL);
		s1_stat.dropped++;
		er = tk_snd_mbf(mbf_telemetry, line, k, TMO_POL);
	}
	return er;
}

EXPORT ER s1_get_status(COMMS_STATUS *st)
{
	*st = s1_stat;
	return E_OK;
}
