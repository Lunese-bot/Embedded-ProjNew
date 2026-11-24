/*
 *      s1_bridge.c
 *      Subsystem 1 -- functions called from the WiFi service task (see
 *      s1_bridge.h).  Everything here is non-blocking (TMO_POL).
 */

#include "subsystem1.h"
#include "s1_codec.h"
#include "../vehicle_controller/vehicle_controller.h"

/* s1_bridge_publish_tag(): bits 0-2 hold kind - S1_KIND_TLM + 1, that is
   1..5 (never 0, so a tag is never NULL), bits 3.. the connection
   generation. */
#define S1_TAG_KIND_BIAS (1 - S1_KIND_TLM)
#define S1_TAG_KIND_MASK 7U
#define S1_TAG_GEN_SHIFT 3
#define S1_TAG_GEN_MASK  (UINTPTR_MAX >> S1_TAG_GEN_SHIFT)

static uint32_t g_s1_conn_gen; /* WiFi service task only */

static void bridge_count_lost(int32_t kind)
{
    if (s1_lock())
    {
        s1_stats_lost(&g_s1_stats, kind);
        s1_unlock();
    }
    else
    {
        s1_lock_failed();
    }
}

static int32_t bridge_pop_from(ID mbf, bool count_sent, char * p_topic,
                               int32_t topic_max, uint8_t * p_payload,
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
    msg_len = tk_rcv_mbf(mbf, (char *)line, TMO_POL);
    if (msg_len <= 0)
    {
        return 0;
    }
    while ((topic_len < msg_len) && (line[topic_len] != '\0'))
    {
        topic_len++;
    }
    if ((topic_len >= msg_len) ||
        ((int32_t)(sizeof(S1_TOPIC_ROOT) - 1) + topic_len >= topic_max) ||
        (msg_len - topic_len - 1 > payload_max))
    {
        if (count_sent)
        {
            /* taken from the queue but unusable: lost */
            bridge_count_lost(s1_line_kind(line, msg_len));
        }
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
    if (count_sent)
    {
        g_s1_stat.sent++;
    }
    return payload_len;
}

int32_t s1_bridge_pop(char * p_topic, int32_t topic_max, uint8_t * p_payload,
                      int32_t payload_max)
{
    return bridge_pop_from(g_mbf_telemetry, true, p_topic, topic_max,
                           p_payload, payload_max);
}

#if APP_TEST
int32_t s1_test_bridge_pop(char * p_topic, int32_t topic_max,
                           uint8_t * p_payload, int32_t payload_max)
{
    return bridge_pop_from(g_mbf_s1_test, false, p_topic, topic_max,
                           p_payload, payload_max);
}
#endif

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

    /* New generation: nothing accepted earlier can complete any more. */
    g_s1_conn_gen++;
    if (s1_lock())
    {
        g_s1_stats.unconfirmed +=
            g_s1_stats.inflight_tlm + g_s1_stats.inflight_evt;
        g_s1_stats.inflight_tlm = 0;
        g_s1_stats.inflight_evt = 0;
        s1_unlock();
    }
    else
    {
        s1_lock_failed();
    }
    g_s1_stat.mqtt_up = (0 != connected);
    if (b_was != g_s1_stat.mqtt_up)
    {
        /* non-critical (FR8): the controller keeps navigating */
        vc_post_event(VC_SRC_COMMS,
                      g_s1_stat.mqtt_up ? VC_EV_WIFI_UP : VC_EV_WIFI_DOWN, 0, 0,
                      TMO_POL);
    }
}

void * s1_bridge_publish_tag(const char * p_topic)
{
    uintptr_t tag = (uintptr_t)(s1_stats_kind(p_topic) + S1_TAG_KIND_BIAS);

    tag |= ((uintptr_t)g_s1_conn_gen & S1_TAG_GEN_MASK) << S1_TAG_GEN_SHIFT;
    return (void *)tag;
}

/* 0 for a tag that s1_bridge_publish_tag() cannot have made */
static int32_t tag_kind(uintptr_t tag, int32_t * p_kind)
{
    uintptr_t kind1 = tag & S1_TAG_KIND_MASK;

    if ((kind1 == 0U) ||
        (kind1 > (uintptr_t)(S1_EVT_KINDS - 1 + S1_TAG_KIND_BIAS)))
    {
        return 0;
    }
    *p_kind = (int32_t)kind1 - S1_TAG_KIND_BIAS;
    return 1;
}

void s1_bridge_published(void * p_tag, int32_t err)
{
    int32_t kind = S1_EVT_OTHER;

    (void)tag_kind((uintptr_t)p_tag, &kind);
    if (!s1_lock())
    {
        s1_lock_failed();
        return;
    }
    s1_stats_publish(&g_s1_stats, kind, err);
    if (err != S1_ERR_OK)
    {
        /* Popped before the call (unchanged queue rule): now gone. */
        s1_stats_lost(&g_s1_stats, kind);
    }
    else if (kind == S1_KIND_TLM)
    {
        g_s1_stats.inflight_tlm++;
    }
    else
    {
        g_s1_stats.inflight_evt++;
    }
    s1_unlock();
}

void s1_bridge_publish_done(void * p_tag, int32_t err)
{
    uintptr_t  tag  = (uintptr_t)p_tag;
    int32_t    kind = S1_EVT_OTHER;
    uint32_t * p_inflight;

    if (!s1_lock())
    {
        s1_lock_failed();
        return;
    }
    if ((!tag_kind(tag, &kind)) ||
        ((tag >> S1_TAG_GEN_SHIFT) !=
         ((uintptr_t)g_s1_conn_gen & S1_TAG_GEN_MASK)))
    {
        g_s1_stats.cb_stale++;
    }
    else
    {
        if (err == S1_ERR_OK)
        {
            g_s1_stats.cb_ok++;
        }
        else
        {
            g_s1_stats.cb_err++;
        }
        p_inflight = (kind == S1_KIND_TLM) ? &g_s1_stats.inflight_tlm :
                                             &g_s1_stats.inflight_evt;
        if (*p_inflight > 0U)
        {
            (*p_inflight)--;
        }
    }
    s1_unlock();
}

void s1_bridge_heartbeat_result(int32_t err)
{
    if (err == S1_ERR_OK)
    {
        return;
    }
    if (s1_lock())
    {
        g_s1_stats.hb_refused++;
        s1_unlock();
    }
    else
    {
        s1_lock_failed();
    }
}

int32_t s1_bridge_format_counters(char * p_buf, int32_t size)
{
    S1_STATS stats;

    if (E_OK != s1_get_stats(&stats))
    {
        return s1_stats_format(NULL, p_buf, size);
    }
    return s1_stats_format(&stats, p_buf, size);
}

/*** end of file ***/
