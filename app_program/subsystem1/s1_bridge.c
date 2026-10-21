/*
 *	s1_bridge.c
 *	Subsystem 1 -- functions called from the WiFi service task (see
 *	s1_bridge.h).  Everything here is non-blocking (TMO_POL).
 */

#include "subsystem1.h"
#include "s1_codec.h"
#include "../subsystem6/subsystem6.h"

EXPORT int s1_bridge_pop(char *topic, int topic_max, char *payload, int payload_max)
{
	UB	line[S1_LINE_MAX];
	INT	n, i, k, r;

	n = tk_rcv_mbf(mbf_telemetry, line, TMO_POL);
	if (n <= 0) {
		return 0;
	}
	/* stored as "<suffix>\0<payload bytes>" */
	k = 0;
	for (r = 0; S1_TOPIC_ROOT[r] != '\0' && k < topic_max - 1; r++) {
		topic[k++] = S1_TOPIC_ROOT[r];
	}
	for (i = 0; i < n && line[i] != '\0' && k < topic_max - 1; i++) {
		topic[k++] = (char)line[i];
	}
	topic[k] = '\0';
	while (i < n && line[i] != '\0') {
		i++;
	}
	i++;					/* skip the NUL */
	for (k = 0; i < n && k < payload_max; i++) {
		payload[k++] = (char)line[i];
	}
	s1_stat.sent++;
	return k;
}

/* One ack byte on robot/g10/ack: result + the command's sequence.
   SENT means queued, not executed: the controller may still refuse the
   command in its current state; the telemetry state shows the outcome. */
LOCAL void ack(S1_ACK res, UB seq)
{
	UB	b = s1_ack_encode(res, seq);

	(void)s1_post("ack", &b, 1);
}

EXPORT void s1_bridge_on_command(const char *p, int len)
{
	UB		b;
	VC_EVENT_TYPE	ev;

	s1_stat.commands++;
	/* The payload is untrusted: exactly one byte, never echoed.  All four
	   2-bit codes are valid, so a bad length is the only way to be refused. */
	if (len != 1) {
		s1_stat.rejected++;
		ack(S1_ACK_REJECTED, 0);
		return;
	}
	b = (UB)p[0];

	/* Remote commands go to the Vehicle Controller (subsystem 6), never
	   to the motors (design review 8.1, Buddy 1 decision). */
	switch (s1_cmd_code(b)) {
	case S1_CMD_START:	ev = VC_EV_CMD_START;	break;
	case S1_CMD_STOP:	ev = VC_EV_CMD_STOP;	break;
	case S1_CMD_PAUSE:	ev = VC_EV_CMD_PAUSE;	break;
	default:		ev = VC_EV_CMD_RESUME;	break;
	}
	if (vc_post_event(VC_SRC_COMMS, ev, 0, 0, TMO_POL) < E_OK) {
		s1_stat.rejected++;
		ack(S1_ACK_BUSY, s1_cmd_seq(b));
		return;
	}
	ack(S1_ACK_SENT, s1_cmd_seq(b));
}

/* Drop telemetry lines queued while offline (they are obsolete), but keep
   events such as faults.  Runs in the WiFi task, so a static buffer is fine. */
#define S1_KEEP_MAX	4

LOCAL BOOL str_eq(const char *a, const char *b)
{
	while (*a != '\0' && *a == *b) {
		a++;
		b++;
	}
	return *a == *b;
}

LOCAL void flush_stale_telemetry(void)
{
	LOCAL char	kept[S1_KEEP_MAX][S1_LINE_MAX];
	LOCAL INT	kept_len[S1_KEEP_MAX];
	char		line[S1_LINE_MAX];
	INT		n, i, nkept = 0, budget = S1_QUEUE_LINES * 2;

	while (budget-- > 0 && (n = tk_rcv_mbf(mbf_telemetry, line, TMO_POL)) > 0) {
		if (str_eq(line, S1_TOPIC_TLM)) {
			s1_stat.dropped++;
		} else if (nkept < S1_KEEP_MAX) {
			for (i = 0; i < n; i++) {
				kept[nkept][i] = line[i];
			}
			kept_len[nkept++] = n;
		} else {
			s1_stat.dropped++;
		}
	}
	for (i = 0; i < nkept; i++) {
		(void)tk_snd_mbf(mbf_telemetry, kept[i], kept_len[i], TMO_POL);
	}
}

EXPORT void s1_bridge_set_connected(int connected)
{
	BOOL	was = s1_stat.mqtt_up;

	s1_stat.mqtt_up = connected ? TRUE : FALSE;
	if (connected && !was) {
		flush_stale_telemetry();
	}
	if (was != s1_stat.mqtt_up) {
		/* non-critical (FR8): the controller keeps navigating */
		vc_post_event(VC_SRC_COMMS,
			      connected ? VC_EV_WIFI_UP : VC_EV_WIFI_DOWN,
			      0, 0, TMO_POL);
	}
}
