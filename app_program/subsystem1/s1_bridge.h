/*
 *	s1_bridge.h
 *	Subsystem 1 -- bridge between application tasks and the MQTT client
 *	that runs inside the WiFi service task.
 *
 *	This header uses plain C types only: mqtt_utk.c may not include
 *	tk/tkernel.h (see the comment at the top of that file).
 */

#ifndef S1_BRIDGE_H
#define S1_BRIDGE_H

#define S1_TOPIC_ROOT		"robot/g10/"
#define S1_TOPIC_CMD		S1_TOPIC_ROOT "cmd"
#define S1_TOPIC_TLM		"t"	/* suffix: telemetry frame */
#define S1_BRIDGE_LINE_MAX	64	/* topic suffix + NUL + packed payload */
#define S1_BRIDGE_TOPIC_MAX	16	/* longest topic suffix */
#define S1_BRIDGE_FULL_TOPIC_MAX 32	/* buffer for root + suffix + NUL */

/* Values of the one-byte robot/g10/status message (retained) */
#define S1_STATUS_ONLINE	1
#define S1_STATUS_OFFLINE	2

/* Take the next queued message, never blocks.  Returns the payload
   length in bytes (> 0) and fills topic (full topic, NUL-terminated) and
   payload (binary, NOT NUL-terminated), or 0 when nothing is queued. */
int	s1_bridge_pop(char *topic, int topic_max, char *payload, int payload_max);

/* A message arrived on S1_TOPIC_CMD.  The payload must be exactly one
   command byte (s1_codec.h: 2-bit code + 6-bit sequence); anything else
   is rejected.  Every command is answered on robot/g10/ack with one
   byte carrying the result and the command's sequence. */
void	s1_bridge_on_command(const char *payload, int len);

/* Broker connection state, from mqtt_utk.c's connection callback */
void	s1_bridge_set_connected(int connected);

#endif /* S1_BRIDGE_H */
