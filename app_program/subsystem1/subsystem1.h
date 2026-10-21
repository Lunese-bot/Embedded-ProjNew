/*
 *	subsystem1.h
 *	Subsystem 1 -- WiFi Communication, Command and Telemetry (Buddy 1).
 *	Student ID: 2500577.
 *
 *	The radio, lwIP and MQTT belong to the kernel's WiFi service task
 *	(TEAM.md section 3): no subsystem task may call cyw43 / lwIP / MQTT.
 *	So subsystem 1 is split in two halves joined by kernel objects:
 *
 *	  telemetry_task (this folder, priority 12)
 *	    every 200 ms (NFR2) packs a 14-byte binary telemetry frame from
 *	    every subsystem's status snapshot (s1_codec.h) and queues it in
 *	    mbf_telemetry; also
 *	    watches the WiFi link (cyw43_utk_get_status, always safe).
 *
 *	  s1_bridge.c (this folder, called FROM the WiFi service task)
 *	    s1_bridge_pop()  takes queued lines without blocking, and
 *	    s1_bridge_on_command() turns an MQTT command into a Vehicle
 *	    Controller event (subsystem 6).
 *	    lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c calls them
 *	    (build with NET=lwip MQTT=1).
 *
 *	Public interface (TEAM.md section 5):
 *	  IF-T1  s1_post(topic, data, len)  any task -> S1 (mbf_telemetry)
 *	  S1 -> S6  vc_post_event(VC_SRC_COMMS, VC_EV_CMD_START / CMD_STOP /
 *	            CMD_PAUSE / CMD_RESUME / WIFI_UP / WIFI_DOWN)    (IF-06)
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "s1_bridge.h"
#include "s1_codec.h"

#define S1_PRI_TELEMETRY	12	/* priority band 12-13 (TEAM.md) */
#define S1_PERIOD_MS		200
#define S1_LINE_MAX		S1_BRIDGE_LINE_MAX	/* topic + payload */
#define S1_QUEUE_LINES		8
#define S1_CONSOLE_LINE_MAX	224	/* JSON console line */
#define S1_CONSOLE_EVERY	5	/* also print telemetry on the console every
				   5 x 200 ms = 1 s while MQTT is not
				   connected (read by tools/dashboard.html) */

/* IF-T1: queue one binary message for MQTT.  topic is the suffix after
   S1_TOPIC_ROOT ("t", "f", "ack", ...), data/len the packed payload
   (len >= 1, topic + payload must fit S1_LINE_MAX, else E_PAR).  Never
   blocks; if the queue is full the OLDEST line is dropped and counted.
   Formats: s1_codec.h. */
IMPORT ER	s1_post(const char *topic, const void *data, INT len);

typedef struct
{
	BOOL	wifi_up;
	BOOL	mqtt_up;
	UW	sent;
	UW	dropped;
	UW	commands;
	UW	rejected;	/* commands not allowlisted or not accepted by S6 */
} COMMS_STATUS;

IMPORT ER	s1_get_status(COMMS_STATUS *st);

/* ---- private to subsystem 1 ---------------------------------------- */
IMPORT ID		tid_telemetry;
IMPORT ID		mbf_telemetry;
IMPORT COMMS_STATUS	s1_stat;	/* counters, single-word updates */
IMPORT void		telemetry_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM1_H */
