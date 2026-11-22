/*
 *      s1_bridge.c
 *      Subsystem 1 -- functions called from the WiFi service task (see
 *      s1_bridge.h).  Everything here is non-blocking (TMO_POL).
 */

#include "subsystem1.h"
#include "s1_codec.h"
#include "../vehicle_controller/vehicle_controller.h"

int32_t s1_bridge_pop(char * p_topic, int32_t topic_max, uint8_t * p_payload,
                      int32_t payload_max)
{
    uint8_t line[S1_LINE_MAX];
    int32_t msg_len;
    int32_t topic_len = 0;
    int32_t payload_len;
    int32_t idx;
    int32_t dst;

    if ((p_topic == NULL) || (topic_max <= 0) ||
        (p_payload == NULL) || (payload_max < 0))
    {
        return -1;
    }
    msg_len = tk_rcv_mbf(g_mbf_telemetry, (char *)line, TMO_POL);
    if (msg_len <= 0)
    {
        return 0;
    }
    while ((topic_len < msg_len) && (line[topic_len] != '\0'))
    {
        topic_len++;
    }
    if (topic_len >= msg_len)
    {
        return -1;
    }
    if (((int32_t)(sizeof(S1_TOPIC_ROOT) - 1) + topic_len >= topic_max) ||
        (msg_len - topic_len - 1 > payload_max))
    {
        return -1;
    }
    dst = 0;
    for (idx = 0; S1_TOPIC_ROOT[idx] != '\0'; idx++)
    {
        p_topic[dst++] = S1_TOPIC_ROOT[idx];
    }
    for (idx = 0; idx < topic_len; idx++)
    {
        p_topic[dst++] = (char)line[idx];
    }
    p_topic[dst] = '\0';
    payload_len = msg_len - topic_len - 1;
    for (idx = 0; idx < payload_len; idx++)
    {
        p_payload[idx] = line[topic_len + 1 + idx];
    }
    g_s1_stat.sent++;
    return payload_len;
}

static void post_ack(S1_ACK result, uint8_t seq)
{
    uint8_t ack = s1_ack_encode(result, seq);

    app_check(s1_post(S1_TOPIC_ACK, &ack, sizeof(ack)), "s1 command ack");
}

void s1_bridge_command_result(uint8_t seq, int32_t accepted)
{
    post_ack(accepted ? S1_ACK_QUEUED : S1_ACK_REJECTED, seq);
}

void s1_bridge_on_command(const uint8_t * p_payload, int32_t len)
{
    uint8_t command_byte;
    S1_REQUEST command;
    vc_event_type_t event;

    g_s1_stat.commands++;
    if ((p_payload == NULL) || ((len != 1) && (len != 2)))
    {
        post_ack(S1_ACK_REJECTED, 0);
        return;
    }
    command_byte = p_payload[0];
    command = s1_cmd_decode(p_payload, len);
    switch (command)
    {
    case S1_REQ_START:
        event = VC_EV_CMD_START;
        break;
    case S1_REQ_STOP:
        event = VC_EV_CMD_STOP;
        break;
    case S1_REQ_RESET:
        event = VC_EV_CMD_RESET;
        break;
    case S1_REQ_PAUSE:
    case S1_REQ_RESUME:
#if S1_ENABLE_REMOTE_PAUSE
        event = (command == S1_REQ_PAUSE) ? VC_EV_CMD_PAUSE :
                                            VC_EV_CMD_RESUME;
        if (vc_post_event(VC_SRC_COMMS, event, s1_cmd_seq(command_byte), 0,
                          TMO_POL) < E_OK)
        {
            post_ack(S1_ACK_BUSY, s1_cmd_seq(command_byte));
        }
        /* The controller acknowledges after checking its actual state. */
#else
        post_ack(S1_ACK_REJECTED, s1_cmd_seq(command_byte));
#endif
        return;
    default:
        post_ack(S1_ACK_REJECTED, s1_cmd_seq(command_byte));
        return;
    }
    /* Commands enter the Vehicle Controller; S1 never drives the motors. */
    if (vc_post_event(VC_SRC_COMMS, event, 0, 0, TMO_POL) < E_OK)
    {
        post_ack(S1_ACK_BUSY, s1_cmd_seq(command_byte));
    }
    else
    {
        post_ack(S1_ACK_QUEUED, s1_cmd_seq(command_byte));
    }
}

void s1_bridge_set_connected(int32_t connected)
{
    bool b_was = g_s1_stat.mqtt_up;

    g_s1_stat.mqtt_up = (0 != connected);
    if (b_was != g_s1_stat.mqtt_up)
    {
        /* non-critical (FR8): the controller keeps navigating */
        vc_post_event(VC_SRC_COMMS,
                      g_s1_stat.mqtt_up ? VC_EV_WIFI_UP : VC_EV_WIFI_DOWN, 0, 0,
                      TMO_POL);
    }
}

/*** end of file ***/
