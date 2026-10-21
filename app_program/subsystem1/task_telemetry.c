/*
 *	task_telemetry.c
 *	Subsystem 1 -- telemetry_task, every 200 ms.
 *
 *	Builds one JSON line from every subsystem's status and queues it for
 *	MQTT (topic robot/g10/telemetry).  Without a WiFi build (or before
 *	MQTT connects) the line is also printed once a second, so the same
 *	telemetry can be checked on the console.
 *
 *	TODO(Buddy 1): agree the topic list and JSON fields with the team
 *	and record them in TEAM.md section 5 / the MQTT documentation.
 */

#include "subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"
#include "../subsystem6/subsystem6.h"
#include "../subsystem6/vc_sm.h"
#if TM_WIFI_CYW43
#include "cyw43_utk.h"
#endif

EXPORT void telemetry_task(INT stacd, void *exinf)
{
	VC_STATUS	c;
	MOTION_STATUS	m;
	S3_LINE_STATUS	l;
	IMU_STATUS	i;
	S5_STATUS	o;
	UB		buf[S1_LINE_MAX + 32];	/* tm_sprintf has no bound */
	UW		tick = 0;
	SYSTIM		now;
#if TM_WIFI_CYW43
	T_CYW43_UTK_STATUS ws;
#endif

	(void)stacd; (void)exinf;

	while (1) {
		tk_slp_tsk(TMO_FEVR);			/* every 200 ms */
		tick++;

		vc_get_status(&c);
		s2_get_status(&m);
		s3_get_line_status(&l);
		s4_get_status(&i);
		s5_get_status(&o);
		tk_get_tim(&now);

#if TM_WIFI_CYW43
		cyw43_utk_get_status(&ws);
		s1_stat.wifi_up = (ws.link_status == 3);	/* CYW43_LINK_UP */
#endif
		tm_sprintf(buf, (const UB*)
			"{\"t\":%u,\"st\":\"%s\",\"v\":[%d,%d],\"odo\":%d,"
			"\"line\":%d,\"bc\":%u,\"nav\":%d,\"pitch\":%d,"
			"\"hump\":%d,\"maxhump\":%d,\"us\":%d,\"obs\":%d}",
			(unsigned)now.lo, vc_state_name(c.state),
			(int)m.speed_l_mm_s, (int)m.speed_r_mm_s,
			(int)m.distance_mm, (int)l.position,
			(unsigned)c.barcodes_read, (int)c.last_nav,
			(int)i.pitch_cdeg, (int)(i.hump_height_um / 1000),
			(int)(i.max_peak_um / 1000),
			(int)o.last_distance_mm, (int)o.state);

		s1_post("telemetry", (const char *)buf);

		if (!s1_stat.mqtt_up && (tick % S1_CONSOLE_EVERY) == 0) {
			tm_printf((UB*)"%s\n", buf);
		}
	}
}
