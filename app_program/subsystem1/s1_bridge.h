/*
 *      s1_bridge.h
 *      Subsystem 1 -- bridge between application tasks and the MQTT client
 *      that runs inside the WiFi service task.
 *
 *      This header uses plain C types only: mqtt_utk.c may not include
 *      tk/tkernel.h (see the comment at the top of that file).
 */

#ifndef S1_BRIDGE_H
#define S1_BRIDGE_H

#include <stdint.h>

#define S1_TOPIC_ROOT       "robot/g10/"
#define S1_TOPIC_CMD        S1_TOPIC_ROOT "cmd"
#define S1_TOPIC_TLM        "t"
#define S1_TOPIC_ACK        "ack"
#define S1_BRIDGE_LINE_MAX  192
#define S1_BRIDGE_TOPIC_MAX 48
#define S1_BRIDGE_FULL_TOPIC_MAX \
    (sizeof(S1_TOPIC_ROOT) + S1_BRIDGE_TOPIC_MAX)

/* Take the next queued message without blocking. The payload is binary and
   is not NUL-terminated; return its exact byte length, or 0 when empty. */
int32_t s1_bridge_pop(char * p_topic, int32_t topic_max, uint8_t * p_payload,
                      int32_t payload_max);

/* S1_TOPIC_CMD carries one command byte. S1_TOPIC_ACK returns one ack byte. */
void s1_bridge_on_command(const uint8_t * p_payload, int32_t len);

/* Broker connection state, from mqtt_utk.c's connection callback */
void s1_bridge_set_connected(int32_t connected);

#endif /* S1_BRIDGE_H */

/*** end of file ***/
