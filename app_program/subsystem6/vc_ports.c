/*
 *	vc_ports.c
 *	Subsystem 6 -- outbound connections (STUBS, see vc_ports.h).
 *
 *	Rule from TEAM.md section 5: the RECEIVER owns the kernel object.
 *	So S2 creates its motion-command buffer, S3/S4/S5 their request
 *	objects and S1 its telemetry buffer; S6 only sends to them, always
 *	with TMO_POL or a short timeout so a slow subsystem can never block
 *	the controller.
 *
 *	Set VC_PORT_VERBOSE to 0 once the real bindings are in.
 */

#include "vc_ports.h"
#include "vc_sm.h"
#include "vc_bridge.h"
#include "../subsystem1/subsystem1.h"

#define VC_PORT_VERBOSE		1

EXPORT VC_PORT_LOG vc_port_log;

LOCAL const char * const move_names[VC_MOVE_COUNT] = {
	"STOP", "FORWARD", "STEER", "TURN_LEFT", "TURN_RIGHT",
	"U_TURN", "REVERSE", "SET_SPEED", "DRIVE_DIST",
	"SAFE_STOP", "RESET"
};

LOCAL const char * const req_names[] = {
	"NONE", "SELF_CHECK", "LINE_FOLLOW", "LINE_SEARCH", "BARCODE_READ",
	"BARCODE_OFF", "OBST_AVOID", "OBST_MONITOR", "TERRAIN_MEASURE",
	"STOP_ALL"
};

EXPORT ER vc_port_move(VC_MOVE m, W value)
{
	vc_port_log.last_move       = m;
	vc_port_log.last_move_value = value;
	vc_port_log.moves++;

#if VC_PORT_VERBOSE
	/* STEER arrives every 10 ms from S3: do not flood the console */
	if (m != VC_MOVE_STEER) {
		tm_printf((UB*)"vc: move %s %d\n",
			  ((UINT)m < VC_MOVE_COUNT) ? move_names[m] : "?", value);
	}
#endif
	/* S2 Motion Control (vc_bridge_s2.c).  Does nothing while the S2
	   folder still holds the course example. */
	return vc_bridge_s2_move(m, value);
}

EXPORT ER vc_port_request(VC_SOURCE target, VC_REQUEST req, W arg)
{
	vc_port_log.last_req_target = target;
	vc_port_log.last_req        = req;
	vc_port_log.requests++;

	/* Each subsystem through its own channels (vc_bridge_*.c);
	   VC_SRC_NONE = everyone */
	if (target == VC_SRC_MOTION || target == VC_SRC_NONE) {
		CHECK(vc_bridge_s2_request(req, arg), "vc->s2 request");
	}
	if (target == VC_SRC_LINE || target == VC_SRC_NONE) {
		CHECK(vc_bridge_s3_request(req, arg), "vc->s3 request");
	}
	if (target == VC_SRC_IMU || target == VC_SRC_NONE) {
		CHECK(vc_bridge_s4_request(req, arg), "vc->s4 request");
	}
	if (target == VC_SRC_OBST || target == VC_SRC_NONE) {
		CHECK(vc_bridge_s5_request(req, arg), "vc->s5 request");
	}

#if VC_PORT_VERBOSE
	if (req != VC_REQ_LINE_FOLLOW || arg != 0) {
		tm_printf((UB*)"vc: request S%d %s (%d)\n", (INT)target,
			  ((UINT)req <= (UINT)VC_REQ_STOP_ALL) ? req_names[req] : "?",
			  arg);
	}
#endif
	return E_OK;
}

EXPORT ER vc_port_report(VC_SOURCE src, VC_FAULT_SEVERITY sev, UW code)
{
	LOCAL const char * const sev_names[] = { "non-critical", "recoverable", "CRITICAL" };

	vc_port_log.reports++;

	/* Fault message to MQTT (robot/g10/fault) through S1's queue;
	 * s1_post() never blocks, so the controller cannot stall (FR8). */
	{
		UB	msg[64];
		tm_sprintf(msg, (const UB*)"{\"src\":%d,\"sev\":%d,\"code\":%u}",
			   (INT)src, (INT)sev, (unsigned)code);
		(void)s1_post("fault", (const char *)msg);
	}
	tm_printf((UB*)"vc: fault from S%d  %s  code 0x%04x\n", (INT)src,
		  ((UINT)sev <= 2U) ? sev_names[sev] : "?", code);
	return E_OK;
}

EXPORT ER vc_port_telemetry(const VC_STATUS *st)
{
	vc_port_log.telemetry++;

	/* S1's telemetry_task reads vc_get_status() itself every 200 ms
	 * and puts the mission state in robot/g10/telemetry, so nothing
	 * needs to be sent from here. */
	(void)st;
	return E_OK;
}
