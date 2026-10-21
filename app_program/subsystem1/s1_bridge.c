/*
 *	s1_bridge.c
 *	Subsystem 1 -- functions called from the WiFi service task (see
 *	s1_bridge.h).  Everything here is non-blocking (TMO_POL).
 */

#include "subsystem1.h"
#include "../subsystem6/subsystem6.h"

EXPORT int s1_bridge_pop(char *topic, int topic_max, char *payload, int payload_max)
{
	char	line[S1_LINE_MAX];
	INT	n, i, k, r;

	n = tk_rcv_mbf(mbf_telemetry, line, TMO_POL);
	if (n <= 0) {
		return 0;
	}
	/* stored as "<suffix>\0<payload>" */
	k = 0;
	for (r = 0; S1_TOPIC_ROOT[r] != '\0' && k < topic_max - 1; r++) {
		topic[k++] = S1_TOPIC_ROOT[r];
	}
	for (i = 0; i < n && line[i] != '\0' && k < topic_max - 1; i++) {
		topic[k++] = line[i];
	}
	topic[k] = '\0';
	while (i < n && line[i] != '\0') {
		i++;
	}
	i++;					/* skip the NUL */
	for (k = 0; i < n && line[i] != '\0' && k < payload_max - 1; i++) {
		payload[k++] = line[i];
	}
	payload[k] = '\0';
	s1_stat.sent++;
	return k;
}

/* Exact allowlist match; only trailing CR/LF/space (terminal tools) is ignored */
LOCAL BOOL cmd_is(const char *s, INT len, const char *w)
{
	INT	i;

	while (len > 0 && (s[len - 1] == '\r' || s[len - 1] == '\n' || s[len - 1] == ' ')) {
		len--;
	}
	for (i = 0; w[i] != '\0'; i++) {
		if (i >= len || s[i] != w[i]) {
			return FALSE;
		}
	}
	return i == len;
}

/* Acknowledge on robot/g10/ack so the operator knows what happened:
   {"cmd":"start","res":"sent"}      handed to the Vehicle Controller
   {"cmd":"start","res":"busy"}      controller queue full, send it again
   {"cmd":"?","res":"rejected"}      not in the allowlist (nothing was done)
   "sent" means queued, not executed: the controller may still refuse it
   in its current state; the "st" field in telemetry shows the outcome. */
LOCAL void ack(const char *cmd, const char *res)
{
	char	buf[48];
	INT	k = 0;
	const char *s;

	s = "{\"cmd\":\"";
	while (*s != '\0') { buf[k++] = *s++; }
	s = cmd;
	while (*s != '\0') { buf[k++] = *s++; }
	s = "\",\"res\":\"";
	while (*s != '\0') { buf[k++] = *s++; }
	s = res;
	while (*s != '\0') { buf[k++] = *s++; }
	buf[k++] = '"';
	buf[k++] = '}';
	buf[k] = '\0';
	(void)s1_post("ack", buf);
}

EXPORT void s1_bridge_on_command(const char *p, int len)
{
	const char	*name = NULL;
	VC_EVENT_TYPE	ev = VC_EV_NONE;

	s1_stat.commands++;
	/* Remote commands go to the Vehicle Controller (subsystem 6), never
	   to the motors (design review 8.1, Buddy 1 decision). */
	if (cmd_is(p, len, "start")) {
		name = "start"; ev = VC_EV_CMD_START;
	} else if (cmd_is(p, len, "stop")) {
		name = "stop"; ev = VC_EV_CMD_STOP;
	} else if (cmd_is(p, len, "reset")) {
		name = "reset"; ev = VC_EV_CMD_RESET;
	}

	if (name == NULL) {
		/* the rejected payload is not echoed: it is untrusted text */
		s1_stat.rejected++;
		ack("?", "rejected");
		return;
	}
	if (vc_post_event(VC_SRC_COMMS, ev, 0, 0, TMO_POL) < E_OK) {
		s1_stat.rejected++;
		ack(name, "busy");
		return;
	}
	ack(name, "sent");
	/* TODO(Buddy 1): JSON commands and a speed command (needs a VC event
	   in subsystem 6 first) */
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
		if (str_eq(line, "telemetry")) {
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
