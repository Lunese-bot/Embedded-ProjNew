/*
 *      subsystem1.h
 *      Subsystem 1 -- WiFi Communication, Command and Telemetry (Buddy 1).
 *      Student ID: 2500577.
 *
 *      The radio, lwIP and MQTT belong to the kernel's WiFi service task
 *      (TEAM.md section 3): no subsystem task may call cyw43 / lwIP / MQTT.
 *      So subsystem 1 is split in two halves joined by kernel objects:
 *
 *        telemetry_task (this folder, priority 12)
 *          every 200 ms (NFR2) packs a binary MQTT telemetry frame from
 *          subsystem status and queues it in g_mbf_telemetry. It also prints
 *          a JSON diagnostic line to USB while MQTT is down.
 *
 *        s1_bridge.c (this folder, called FROM the WiFi service task)
 *          s1_bridge_pop()  takes queued binary payloads without blocking,
 *          and s1_bridge_on_command() turns a binary MQTT command into a Vehicle
 *          Controller event.
 *          lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c calls them
 *          (build with NET=lwip MQTT=1).
 *
 *      Public interface (TEAM.md section 5):
 *        IF-T1  s1_post(topic, data, len) any task -> S1 (g_mbf_telemetry)
 *        S1 -> VC  vc_post_event(VC_SRC_COMMS, VC_EV_CMD_START / CMD_STOP /
 *                  CMD_RESET / WIFI_UP / WIFI_DOWN)                 (IF-06)
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "s1_bridge.h"
#include "s1_stats.h"
#include "s1_lock.h"
#include "s1_txq.h"

#define S1_PRI_TELEMETRY 12 /* priority band 12-13 (TEAM.md) */
#define S1_PERIOD_MS     200
#define S1_LINE_MAX      S1_BRIDGE_LINE_MAX /* topic + payload */
#define S1_QUEUE_LINES   8
#define S1_EVT_QUEUE_MAX S1_QUEUE_LINES /* waiting events (s1_txq.h) */
#define S1_CONSOLE_LINE_MAX 224
/* Also print telemetry on the console every 5 x 200 ms = 1 s while MQTT
   is not connected (read by tools/dashboard.html). */
#define S1_CONSOLE_EVERY 5
/* Print the delivery counters (s1_stats.h) on the console every
   25 x 200 ms = 5 s, connected or not. */
#define S1_STATS_EVERY 25

/* IF-T1: queue one MQTT message. The payload may contain any byte value.
   topic is the suffix after S1_TOPIC_ROOT (for example "t", "ack", "fault").
   Never blocks for more than S1_LOCK_TMO_MS.  Telemetry ("t") and events
   (any other topic) are kept apart; see s1_txq.h for capacity, order and
   the only cases in which a queued message is discarded. */
extern ER s1_post(const char * p_topic, const void * p_data, int32_t len);

/* IF-T1b: like s1_post() for an event topic, but only while at least
   `spare` event slots stay free after it (0 < spare < S1_EVT_QUEUE_MAX).
   Otherwise returns E_LIMIT and queues nothing, so it never evicts an
   ack or a fault.  The check and the post are two lock sections: a post
   from another task in between can use one of the spare slots, never
   push the queue past full.  Telemetry ("t") is refused with E_PAR. */
extern ER s1_post_spare(const char * p_topic, const void * p_data,
                        int32_t len, int32_t spare);

typedef struct
{
    bool     wifi_up;
    bool     mqtt_up;
    uint32_t sent;
    uint32_t dropped;
    uint32_t commands;
} comms_status_t;

extern ER s1_get_status(comms_status_t * p_status);

/* Coherent copy of the delivery counters (s1_stats.h).  E_OK, or E_TMOUT
   when the S1 lock was busy (*p_stats is then left unchanged). */
extern ER s1_get_stats(S1_STATS * p_stats);

/* ---- private to subsystem 1 ---------------------------------------- */
extern ID             g_tid_telemetry;
extern ID             g_mbf_telemetry;
extern comms_status_t g_s1_stat; /* counters, single-word updates */
extern S1_STATS       g_s1_stats; /* delivery counters, under s1_lock() */
extern void           telemetry_task(int32_t stacd, void * p_exinf);
extern S1_TXQ         g_s1_txq;   /* delivery policy, under s1_lock() */

#if APP_TEST
extern ID g_mbf_s1_test;
extern int32_t s1_test_bridge_pop(char * p_topic, int32_t topic_max,
                                  uint8_t * p_payload, int32_t payload_max);
#endif

#endif /* SUBSYSTEM1_H */

/*** end of file ***/
