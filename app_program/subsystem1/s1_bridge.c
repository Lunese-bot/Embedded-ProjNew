/*
 *      s1_bridge.c
 *      Subsystem 1 -- functions called from the WiFi service task (see
 *      s1_bridge.h).  Everything here is non-blocking (TMO_POL).
 */

#include "subsystem1.h"
#include "../subsystem6/subsystem6.h"

int32_t s1_bridge_pop(char * p_topic, int32_t topic_max, char * p_payload,
                      int32_t payload_max)
{
    char    line[S1_LINE_MAX];
    int32_t msg_len;
    int32_t src;
    int32_t dst;
    int32_t root_idx;

    msg_len = tk_rcv_mbf(g_mbf_telemetry, line, TMO_POL);
    if (msg_len <= 0)
    {
        return 0;
    }
    /* stored as "<suffix>\0<payload>" */
    dst = 0;
    for (root_idx = 0;
         ('\0' != S1_TOPIC_ROOT[root_idx]) && (dst < (topic_max - 1));
         root_idx++)
    {
        p_topic[dst++] = S1_TOPIC_ROOT[root_idx];
    }
    for (src = 0;
         (src < msg_len) && (line[src] != '\0') && (dst < (topic_max - 1));
         src++)
    {
        p_topic[dst++] = line[src];
    }
    p_topic[dst] = '\0';
    while ((src < msg_len) && (line[src] != '\0'))
    {
        src++;
    }
    src++; /* skip the NUL */
    for (dst = 0;
         (src < msg_len) && (line[src] != '\0') && (dst < (payload_max - 1));
         src++)
    {
        p_payload[dst++] = line[src];
    }
    p_payload[dst] = '\0';
    g_s1_stat.sent++;
    return dst;
}

static bool starts_with(const char * p_text, int32_t len, const char * p_word)
{
    int32_t idx;

    for (idx = 0; p_word[idx] != '\0'; idx++)
    {
        if ((idx >= len) || (p_text[idx] != p_word[idx]))
        {
            return false;
        }
    }
    return true;
}

void s1_bridge_on_command(const char * p_payload, int32_t len)
{
    g_s1_stat.commands++;
    /* Remote commands go to the Vehicle Controller (subsystem 6), never
       to the motors (design review 8.1, Buddy 1 decision). */
    if (starts_with(p_payload, len, "start"))
    {
        vc_post_event(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0, TMO_POL);
    }
    else if (starts_with(p_payload, len, "stop"))
    {
        vc_post_event(VC_SRC_COMMS, VC_EV_CMD_STOP, 0, 0, TMO_POL);
    }
    else if (starts_with(p_payload, len, "reset"))
    {
        vc_post_event(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0, TMO_POL);
    }
    else
    {
        /* Unknown command: counted above, otherwise ignored */
    }
    /* TODO(Buddy 1): JSON commands, acknowledgements on robot/g10/ack,
       a speed command (needs a VC event in subsystem 6 first) */
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
