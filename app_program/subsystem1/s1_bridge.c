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

LOCAL BOOL starts_with(const char *s, INT len, const char *w)
{
	INT	i;

	for (i = 0; w[i] != '\0'; i++) {
		if (i >= len || s[i] != w[i]) {
			return FALSE;
		}
	}
	return TRUE;
}

EXPORT void s1_bridge_on_command(const char *p, int len)
{
	s1_stat.commands++;
	/* Remote commands go to the Vehicle Controller (subsystem 6), never
	   to the motors (design review 8.1, Buddy 1 decision). */
	if (starts_with(p, len, "start")) {
		vc_post_event(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0, TMO_POL);
	} else if (starts_with(p, len, "stop")) {
		vc_post_event(VC_SRC_COMMS, VC_EV_CMD_STOP, 0, 0, TMO_POL);
	} else if (starts_with(p, len, "reset")) {
		vc_post_event(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0, TMO_POL);
	}
	/* TODO(Buddy 1): JSON commands, acknowledgements on robot/g10/ack,
	   a speed command (needs a VC event in subsystem 6 first) */
}

EXPORT void s1_bridge_set_connected(int connected)
{
	BOOL	was = s1_stat.mqtt_up;

	s1_stat.mqtt_up = connected ? TRUE : FALSE;
	if (was != s1_stat.mqtt_up) {
		/* non-critical (FR8): the controller keeps navigating */
		vc_post_event(VC_SRC_COMMS,
			      connected ? VC_EV_WIFI_UP : VC_EV_WIFI_DOWN,
			      0, 0, TMO_POL);
	}
}
