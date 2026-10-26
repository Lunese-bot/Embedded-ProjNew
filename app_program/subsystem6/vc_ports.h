/*
 *      vc_ports.h
 *      Subsystem 6 -- outbound connections from the Vehicle Controller to
 *      the other subsystems.  Called ONLY from vehicle_ctrl_task.
 *
 *      Each function is the single place where S6 talks to the other
 *      subsystems.  They log what they do, record it in g_vc_port_log (for
 *      the tests) and pass it to that subsystem's adapter in vc_bridge_*.c.
 *      S1 (fault reports, telemetry) is still a STUB: see the TODOs.
 */

#ifndef VC_PORTS_H
#define VC_PORTS_H

#include "subsystem6.h"

/* S2 Motion Control: one authorised movement command */
extern ER vc_port_move(vc_move_t move, int32_t value);

/* S3 / S4 / S5 (or all, target = VC_SRC_NONE): change what they do */
extern ER vc_port_request(vc_source_t target, vc_request_t req, int32_t arg);

/* S1 Comms: a fault report, published over MQTT */
extern ER vc_port_report(vc_source_t src, vc_fault_severity_t sev,
                         uint32_t code);

/* S1 Comms: periodic / on-change status, published over MQTT */
extern ER vc_port_telemetry(const vc_status_t * p_status);

/* Last calls, for tests and debugging */
typedef struct
{
    vc_move_t    last_move;
    int32_t      last_move_value;
    uint32_t     moves;
    vc_source_t  last_req_target;
    vc_request_t last_req;
    uint32_t     requests;
    uint32_t     reports;
    uint32_t     telemetry;
} vc_port_log_t;

extern vc_port_log_t g_vc_port_log;

#endif /* VC_PORTS_H */

/*** end of file ***/
