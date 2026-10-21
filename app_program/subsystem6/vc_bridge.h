/*
 *	vc_bridge.h
 *	Subsystem 6 -- adapters for subsystems that publish their OWN
 *	event channel to the Vehicle Controller instead of calling
 *	vc_post_event().  Each bridge reads that channel (never blocking)
 *	and turns it into VC events, and turns controller requests into
 *	that subsystem's commands, so the state machine never changes.
 *	All calls come from vehicle_ctrl_task only.
 *
 *	  S2 motion    vc_bridge_s2.c   mbf_motion_cmd (s2_cmd_*), flg_motion
 *	  S3 line      vc_bridge_s3.c   mbf_s3_events, s3_get_line_status, s3_request
 *	  S4 IMU       vc_bridge_s4.c   mbf_imu_evt, s4_request_*()
 *	  S5 obstacle  vc_bridge_s5.c   mbf_s5_evt / mbf_s5_cmd
 *
 *	S1 (WiFi/MQTT) has no bridge yet: see vc_port_report/telemetry.
 *
 *	A bridge compiles to empty functions while that subsystem's folder
 *	still holds the original example/template, so S6 always builds.
 */

#ifndef VC_BRIDGE_H
#define VC_BRIDGE_H

#include "subsystem6.h"

/* ---- S2 motion control -------------------------------------------- */

/* One authorised movement command (called by vc_port_move) */
IMPORT ER vc_bridge_s2_move(VC_MOVE m, W value);

/* Next S2 event (manoeuvre done, fault) as a VC event */
IMPORT BOOL vc_bridge_s2_next(VC_EVENT *ev);

/* Controller request for S2 (called by vc_port_request) */
IMPORT ER vc_bridge_s2_request(VC_REQUEST req, W arg);

/* ---- S3 line following + barcode ---------------------------------- */

/* Next S3 event, or S3's latest steering as a move request while S3
   owns motion in `state` */
IMPORT BOOL vc_bridge_s3_next(VC_EVENT *ev, VC_STATE state);

/* Controller request for S3 (called by vc_port_request) */
IMPORT ER vc_bridge_s3_request(VC_REQUEST req, W arg);

/* ---- S4 IMU terrain monitoring ------------------------------------ */

/* Next S4 event as a VC event; FALSE when there is none */
IMPORT BOOL vc_bridge_s4_next(VC_EVENT *ev);

/* Controller request for S4 (called by vc_port_request) */
IMPORT ER vc_bridge_s4_request(VC_REQUEST req, W arg);

/* ---- S5 ultrasonic obstacle avoidance ----------------------------- */

/* Next S5 event as a VC event; FALSE when there is none */
IMPORT BOOL vc_bridge_s5_next(VC_EVENT *ev);

/* Call after the state machine handled an event from
   vc_bridge_s5_next().  rejected = the move request was refused. */
IMPORT void vc_bridge_s5_after(const VC_EVENT *ev, VC_STATE state_after,
			       BOOL rejected);

/* Motion Control finished (or failed) a manoeuvre while S5 owns motion */
IMPORT void vc_bridge_s5_manoeuvre_done(BOOL ok);

/* Controller request for S5 (called by vc_port_request) */
IMPORT ER vc_bridge_s5_request(VC_REQUEST req, W arg);

#endif /* VC_BRIDGE_H */
