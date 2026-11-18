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

#define S1_PRI_TELEMETRY 12 /* priority band 12-13 (TEAM.md) */
#define S1_PERIOD_MS     200
#define S1_LINE_MAX      S1_BRIDGE_LINE_MAX /* topic + payload */
#define S1_QUEUE_LINES   8
#define S1_CONSOLE_LINE_MAX 224
/* Also print telemetry on the console every 5 x 200 ms = 1 s while MQTT
   is not connected (read by tools/dashboard.html). */
#define S1_CONSOLE_EVERY 5

/* IF-T1: queue one MQTT message. The payload may contain any byte value.
   topic is the suffix after S1_TOPIC_ROOT (for example "t", "ack", "fault").
   Never blocks; if the queue is full, the oldest message is dropped. */
extern ER s1_post(const char * p_topic, const void * p_data, int32_t len);

typedef struct
{
    bool     wifi_up;
    bool     mqtt_up;
    uint32_t sent;
    uint32_t dropped;
    uint32_t commands;
} comms_status_t;

extern ER s1_get_status(comms_status_t * p_status);

/* ---- private to subsystem 1 ---------------------------------------- */
extern ID             g_tid_telemetry;
extern ID             g_mbf_telemetry;
extern comms_status_t g_s1_stat; /* counters, single-word updates */
extern void           telemetry_task(int32_t stacd, void * p_exinf);

#endif /* SUBSYSTEM1_H */

/*** end of file ***/
