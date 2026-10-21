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
#define S1_BRIDGE_LINE_MAX	192
#define S1_BRIDGE_TOPIC_MAX	48

/* Take the next queued message, never blocks.  Returns the payload
   length (> 0) and fills topic (full topic, NUL-terminated) and payload,
   or 0 when nothing is queued. */
int	s1_bridge_pop(char *topic, int topic_max, char *payload, int payload_max);

/* A message arrived on S1_TOPIC_CMD.  Only the exact payloads "start",
   "stop" and "reset" are accepted (trailing CR/LF/space ignored); anything
   else is ignored. */
void	s1_bridge_on_command(const char *payload, int len);

/* Broker connection state, from mqtt_utk.c's connection callback */
void	s1_bridge_set_connected(int connected);

#endif /* S1_BRIDGE_H */
