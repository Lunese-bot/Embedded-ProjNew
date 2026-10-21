/*
 *	task_telemetry.c
 *	Subsystem 1 -- telemetry_task, every 200 ms.
 *
 *	Packs one 14-byte binary frame (s1_codec.h) from every subsystem's
 *	status and queues it for MQTT (topic robot/g10/t).  Before MQTT
 *	connects the same data is printed once a second as a JSON line, so
 *	telemetry can still be checked on the console / dashboard.
 *
 *	TODO(Buddy 1): the widths and scales in s1_codec.h are placeholders;
 *	agree them with the team and record them in TEAM.md section 5.
 */

#include "subsystem1.h"
#include "s1_codec.h"
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
	S1_TLM		t;
	UB		frame[S1_TLM_BYTES];
	UB		buf[S1_CONSOLE_LINE_MAX];	/* tm_sprintf has no bound */
	UB		seq = 0;
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
		/* Packed frame for MQTT; the encoder clamps every field.
		   The sequence number lets the receiver spot lost or stale
		   frames. */
		t.version      = S1_PROTO_VERSION;
		t.seq          = seq++;
		t.state        = (UB)c.state;
		t.speed_l_mms  = m.speed_l_mm_s;
		t.speed_r_mms  = m.speed_r_mm_s;
		t.distance_mm  = m.distance_mm;
		t.line_pos     = l.position;
		/* 0 mm = no echo: report "far" (127 cm) */
		t.obs_dist_cm  = (o.last_distance_mm == 0) ? S1_OBS_MAX_CM
				 : (UW)(o.last_distance_mm / 10);
		t.obs_detected = (o.state >= S5_ST_DETECTED && o.state != S5_ST_ERROR);
		t.obs_dir      = 0;	/* placeholder: S5 does not report a side yet */
		t.nav          = (UB)c.last_nav;
		t.barcodes     = (c.barcodes_read > 15) ? 15 : (UB)c.barcodes_read;
		t.pitch_cdeg   = i.pitch_cdeg;
		t.hump_now_um  = i.hump_height_um;
		t.hump_max_um  = c.max_hump_um;
		t.fault_code   = c.last_fault_code;
		t.wifi_up      = s1_stat.wifi_up;
		t.paused       = c.paused;
		s1_tlm_encode(&t, frame);
		(void)s1_post(S1_TOPIC_TLM, frame, S1_TLM_BYTES);

		/* The USB console keeps readable JSON for tools/dashboard.html */
		if (!s1_stat.mqtt_up && (tick % S1_CONSOLE_EVERY) == 0) {
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
			tm_printf((UB*)"%s\n", buf);
		}
	}
}
