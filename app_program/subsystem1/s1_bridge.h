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
#define S1_TOPIC_BARCODE    "barcode"
/* barcode letter map read / replace (s1_cfg.h); requests are never retained */
#define S1_TOPIC_CFG        S1_TOPIC_ROOT "cfg"
#define S1_TOPIC_CFG_REPLY  "cfgr"
#define S1_TOPIC_JUNCTION   "jx"
#define S1_BRIDGE_LINE_MAX  192
#define S1_BRIDGE_TOPIC_MAX 48
#define S1_BRIDGE_FULL_TOPIC_MAX \
    (sizeof(S1_TOPIC_ROOT) + S1_BRIDGE_TOPIC_MAX)

/* Bytes one MQTT 3.1.1 QoS 0 PUBLISH takes in lwIP's output ring: fixed
   header byte, remaining-length bytes (1 below 128, 2 below 16384), the
   2-byte topic length, the topic and the payload. */
#define S1_MQTT_RL_BYTES(rem) (((rem) < 128U) ? 1U : (((rem) < 16384U) ? 2U : 3U))
#define S1_MQTT_PUBLISH_BYTES(topic_len, payload_len) \
    (1U + S1_MQTT_RL_BYTES(2U + (topic_len) + (payload_len)) + 2U + \
     (topic_len) + (payload_len))

/* The longest S1 publish: s1_post() keeps topic suffix + NUL + payload
   within S1_BRIDGE_LINE_MAX, and S1 adds the root. */
#define S1_BRIDGE_PUBLISH_MAX \
    S1_MQTT_PUBLISH_BYTES(sizeof(S1_TOPIC_ROOT) - 1U, S1_BRIDGE_LINE_MAX - 1U)

/* Set to 0 to reject remote Pause/Resume at the firmware bridge. */
#ifndef S1_ENABLE_REMOTE_PAUSE
#define S1_ENABLE_REMOTE_PAUSE 1
#endif

/* Take the next queued message without blocking, in publish order, as if
   it had been published. The payload is binary and is not NUL-terminated;
   return its exact byte length, or 0 when empty.  mqtt_utk.c now uses
   s1_bridge_service() instead. */
int32_t s1_bridge_pop(char * p_topic, int32_t topic_max, uint8_t * p_payload,
                      int32_t payload_max);

/* Commands are one byte or a two-byte extension; acknowledgements are one byte. */
void s1_bridge_on_command(const uint8_t * p_payload, int32_t len);
void s1_bridge_command_result(uint8_t seq, int32_t accepted);

/* robot/g10/cfg: read the barcode map (answered at once) or hand a whole
   map write to the Vehicle Controller (answered after it was applied or
   refused).  len -1 = truncated message. */
void s1_bridge_on_config(const uint8_t * p_payload, int32_t len);
/* Vehicle Controller: publish the reply to a config request. */
void s1_bridge_config_reply(uint8_t op, uint16_t request_id, uint8_t result);
/* VC task: junction evidence (vc_junction_note_t kind, letter or 0, wire
   action code) on robot/g10/jx with the active map's generation. */
void s1_bridge_junction(uint8_t kind, uint8_t letter, uint8_t code);

/* Broker connection state, from mqtt_utk.c's connection callback.  Every
   change of state starts a new connection generation: publishes still
   waiting for completion are counted as unconfirmed (lwIP drops its request
   list without callbacks), and later completions carrying an old
   generation are counted as stale and otherwise ignored. */
void s1_bridge_set_connected(int32_t connected);

/* ---- publishing, all called from the WiFi service task ------------- */

/* Most telemetry publishes accepted by lwIP and still waiting for their
   completion callback.  At this cap only the newest unsent frame is kept,
   and the remaining lwIP request slots stay free for events. */
#define S1_TLM_INFLIGHT_MAX 10

/* Publish adapter: the only place that calls mqtt_publish() (mqtt_utk.c).
   Returns lwIP's err_t; p_tag is passed to lwIP as the callback argument
   and comes back through s1_bridge_publish_done(). */
typedef int32_t (*s1_publish_fn)(const char * p_topic,
                                 const uint8_t * p_payload, uint16_t len,
                                 void * p_tag);

/* Offer up to max_attempts queued messages to publish, events first (see
   s1_txq.h).  Stops at the first refusal; the refused message stays in S1
   for the next call.  Never holds the S1 lock while publish runs.
   Returns the number accepted. */
int32_t s1_bridge_service(s1_publish_fn publish, int32_t max_attempts);

/* lwIP's completion callback (mqtt_request_cb_t) for a tagged publish. */
void s1_bridge_publish_done(void * p_tag, int32_t err);

/* mqtt_publish() returned err for the text heartbeat. */
void s1_bridge_heartbeat_result(int32_t err);

/* Coherent snapshot of every counter as one bounded text line (see
   s1_stats_format); returns its length. */
int32_t s1_bridge_format_counters(char * p_buf, int32_t size);

#endif /* S1_BRIDGE_H */

/*** end of file ***/
