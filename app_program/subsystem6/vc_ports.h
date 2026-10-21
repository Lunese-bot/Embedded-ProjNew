/*
 *	vc_ports.h
 *	Subsystem 6 -- outbound connections from the Vehicle Controller to
 *	the other subsystems.  Called ONLY from vehicle_ctrl_task.
 *
 *	Each function is the single place where S6 talks to the other
 *	subsystems.  They log what they do, record it in vc_port_log (for
 *	the tests) and pass it to that subsystem's adapter in vc_bridge_*.c.
 *	S1 (fault reports, telemetry) is still a STUB: see the TODOs.
 */

#ifndef VC_PORTS_H
#define VC_PORTS_H

#include "subsystem6.h"

/* S2 Motion Control: one authorised movement command */
IMPORT ER vc_port_move(VC_MOVE m, W value);

/* S3 / S4 / S5 (or all, target = VC_SRC_NONE): change what they do */
IMPORT ER vc_port_request(VC_SOURCE target, VC_REQUEST req, W arg);

/* S1 Comms: a fault report, published over MQTT */
IMPORT ER vc_port_report(VC_SOURCE src, VC_FAULT_SEVERITY sev, UW code);

/* S1 Comms: periodic / on-change status, published over MQTT */
IMPORT ER vc_port_telemetry(const VC_STATUS *st);

/* Last calls, for tests and debugging */
typedef struct {
	VC_MOVE		last_move;
	W		last_move_value;
	UW		moves;
	VC_SOURCE	last_req_target;
	VC_REQUEST	last_req;
	UW		requests;
	UW		reports;
	UW		telemetry;
} VC_PORT_LOG;

IMPORT VC_PORT_LOG vc_port_log;

#endif /* VC_PORTS_H */
