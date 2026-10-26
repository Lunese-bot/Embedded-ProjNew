/*
 *      vc_ports.c
 *      Subsystem 6 -- outbound connections (STUBS, see vc_ports.h).
 *
 *      Rule from TEAM.md section 5: the RECEIVER owns the kernel object.
 *      So S2 creates its motion-command buffer, S3/S4/S5 their request
 *      objects and S1 its telemetry buffer; S6 only sends to them, always
 *      with TMO_POL or a short timeout so a slow subsystem can never block
 *      the controller.
 *
 *      Set VC_PORT_VERBOSE to 0 once the real bindings are in.
 */

#include "vc_ports.h"
#include "vc_sm.h"
#include "vc_bridge.h"
#include "../subsystem1/subsystem1.h"

#define VC_PORT_VERBOSE  1
#define VC_FAULT_MSG_MAX 64 /* bytes for one JSON fault message */

vc_port_log_t g_vc_port_log;

static const char * const g_move_names[VC_MOVE_COUNT] = {
    "STOP",    "FORWARD",   "STEER",      "TURN_LEFT", "TURN_RIGHT", "U_TURN",
    "REVERSE", "SET_SPEED", "DRIVE_DIST", "SAFE_STOP", "RESET"
};

static const char * const g_req_names[] = {
    "NONE",        "SELF_CHECK",   "LINE_FOLLOW",
    "LINE_SEARCH", "BARCODE_READ", "BARCODE_OFF",
    "OBST_AVOID",  "OBST_MONITOR", "TERRAIN_MEASURE",
    "STOP_ALL"
};

ER vc_port_move(vc_move_t move, int32_t value)
{
    g_vc_port_log.last_move       = move;
    g_vc_port_log.last_move_value = value;
    g_vc_port_log.moves++;

#if VC_PORT_VERBOSE
    /* STEER arrives every 10 ms from S3: do not flood the console */
    if (move != VC_MOVE_STEER)
    {
        tm_printf((uint8_t *)"vc: move %s %d\n",
                  ((uint32_t)move < VC_MOVE_COUNT) ? g_move_names[move] : "?",
                  value);
    }
#endif
    /* S2 Motion Control (vc_bridge_s2.c).  Does nothing while the S2
       folder still holds the course example. */
    return vc_bridge_s2_move(move, value);
}

ER vc_port_request(vc_source_t target, vc_request_t req, int32_t arg)
{
    g_vc_port_log.last_req_target = target;
    g_vc_port_log.last_req        = req;
    g_vc_port_log.requests++;

    /* Each subsystem through its own channels (vc_bridge_*.c);
       VC_SRC_NONE = everyone */
    if ((VC_SRC_MOTION == target) || (VC_SRC_NONE == target))
    {
        app_check(vc_bridge_s2_request(req, arg), "vc->s2 request");
    }
    if ((VC_SRC_LINE == target) || (VC_SRC_NONE == target))
    {
        app_check(vc_bridge_s3_request(req, arg), "vc->s3 request");
    }
    if ((VC_SRC_IMU == target) || (VC_SRC_NONE == target))
    {
        app_check(vc_bridge_s4_request(req, arg), "vc->s4 request");
    }
    if ((VC_SRC_OBST == target) || (VC_SRC_NONE == target))
    {
        app_check(vc_bridge_s5_request(req, arg), "vc->s5 request");
    }

#if VC_PORT_VERBOSE
    if ((req != VC_REQ_LINE_FOLLOW) || (arg != 0))
    {
        tm_printf((uint8_t *)"vc: request S%d %s (%d)\n", (int32_t)target,
                  ((uint32_t)req <= (uint32_t)VC_REQ_STOP_ALL)
                      ? g_req_names[req]
                      : "?",
                  arg);
    }
#endif
    return E_OK;
}

ER vc_port_report(vc_source_t src, vc_fault_severity_t sev, uint32_t code)
{
    static const char * const sev_names[] = { "non-critical", "recoverable",
                                              "CRITICAL" };
    uint8_t                   msg[VC_FAULT_MSG_MAX];

    g_vc_port_log.reports++;

    /* Fault message to MQTT (robot/g10/fault) through S1's queue;
     * s1_post() never blocks, so the controller cannot stall (FR8). */
    /* Casts: the console API takes uint8_t strings and int32_t args;
       s1_post takes plain char strings. */
    tm_sprintf(msg, (const uint8_t *)"{\"src\":%d,\"sev\":%d,\"code\":%u}",
               (int32_t)src, (int32_t)sev, code);
    (void)s1_post("fault", (const char *)msg);
    tm_printf((uint8_t *)"vc: fault from S%d  %s  code 0x%04x\n", (int32_t)src,
              ((uint32_t)sev <= (uint32_t)VC_FAULT_CRITICAL) ? sev_names[sev]
                                                             : "?",
              code);
    return E_OK;
}

ER vc_port_telemetry(const vc_status_t * p_status)
{
    g_vc_port_log.telemetry++;

    /* S1's telemetry_task reads vc_get_status() itself every 200 ms
     * and puts the mission state in robot/g10/telemetry, so nothing
     * needs to be sent from here. */
    (void)p_status;
    return E_OK;
}

/*** end of file ***/
