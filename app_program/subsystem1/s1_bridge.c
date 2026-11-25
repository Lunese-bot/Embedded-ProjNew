/*
 *      s1_bridge.c
 *      Subsystem 1 -- functions called from the WiFi service task (see
 *      s1_bridge.h).  Everything here is non-blocking (TMO_POL).
 */

#include "subsystem1.h"
#include "s1_codec.h"
#include "s1_txq.h"
#include "s1_cfg.h"
#include "../vehicle_controller/vehicle_controller.h"
#include "../vehicle_controller/vc_barcode_map.h"

/* Callback tags: bits 0-2 hold kind - S1_KIND_TLM + 1, that is 1..5 (never
   0, so a tag is never NULL), bits 3.. the connection generation. */
#define S1_TAG_KIND_BIAS (1 - S1_KIND_TLM)
#define S1_TAG_KIND_MASK 7U
#define S1_TAG_GEN_SHIFT 3
#define S1_TAG_GEN_MASK  (UINTPTR_MAX >> S1_TAG_GEN_SHIFT)

/* "topic\0payload" -> full topic and payload; payload length, or -1 when
   the line cannot be expressed in the caller's buffers. */
static int32_t line_to_message(const uint8_t * p_line, int32_t msg_len,
                               char * p_topic, int32_t topic_max,
                               uint8_t * p_payload, int32_t payload_max)
{
    int32_t topic_len = 0;
    int32_t payload_len;
    int32_t idx;
    int32_t dst;

    while ((topic_len < msg_len) && (p_line[topic_len] != '\0'))
    {
        topic_len++;
    }
    if ((topic_len >= msg_len) ||
        ((int32_t)(sizeof(S1_TOPIC_ROOT) - 1) + topic_len >= topic_max) ||
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
        p_topic[dst++] = (char)p_line[idx];
    }
    p_topic[dst] = '\0';
    payload_len  = msg_len - topic_len - 1;
    for (idx = 0; idx < payload_len; idx++)
    {
        p_payload[idx] = p_line[topic_len + 1 + idx];
    }
    return payload_len;
}

static bool args_ok(const char * p_topic, int32_t topic_max,
                    const uint8_t * p_payload, int32_t payload_max)
{
    return (p_topic != NULL) && (topic_max > 0) && (p_payload != NULL) &&
           (payload_max >= 0);
}

/* s1_txq_pull_fn_t for the event buffer (S1 lock held) */
static int32_t pull_event(uint8_t * p_line)
{
    int32_t len = tk_rcv_mbf(g_mbf_telemetry, (char *)p_line, TMO_POL);

    return (len > 0) ? len : 0;
}

/* Legacy pop: the next message in publish order, removed at once. */
int32_t s1_bridge_pop(char * p_topic, int32_t topic_max, uint8_t * p_payload,
                      int32_t payload_max)
{
    s1_txq_item_t item;
    int32_t       len;

    if (!args_ok(p_topic, topic_max, p_payload, payload_max))
    {
        return -1;
    }
    if (!s1_lock())
    {
        s1_lock_failed();
        return 0;
    }
    len = s1_txq_take(&g_s1_txq, pull_event, &item);
    if (len > 0)
    {
        len = line_to_message(item.line, item.len, p_topic, topic_max,
                              p_payload, payload_max);
        if (len < 0)
        {
            s1_stats_lost(&g_s1_stats, s1_stats_line_kind(item.line, item.len));
        }
        else
        {
            g_s1_stat.sent++;
        }
    }
    s1_unlock();
    return len;
}

#if APP_TEST
int32_t s1_test_bridge_pop(char * p_topic, int32_t topic_max,
                           uint8_t * p_payload, int32_t payload_max)
{
    uint8_t line[S1_LINE_MAX];
    int32_t msg_len;

    if (!args_ok(p_topic, topic_max, p_payload, payload_max))
    {
        return -1;
    }
    msg_len = tk_rcv_mbf(g_mbf_s1_test, (char *)line, TMO_POL);
    if (msg_len <= 0)
    {
        return 0;
    }
    return line_to_message(line, msg_len, p_topic, topic_max, p_payload,
                           payload_max);
}
#endif

static void post_ack(s1_ack_t result, uint8_t seq)
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
    uint8_t         command_byte;
    s1_request_t    command;
    vc_event_type_t event;

    g_s1_stat.commands++;
    if ((p_payload == NULL) || ((len != 1) && (len != 2)))
    {
        post_ack(S1_ACK_REJECTED, 0);
        return;
    }
    command_byte = p_payload[0];
    command      = s1_cmd_decode(p_payload, len);
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
            event =
                (command == S1_REQ_PAUSE) ? VC_EV_CMD_PAUSE : VC_EV_CMD_RESUME;
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

/* ---- barcode map read / replace (s1_cfg.h) -------------------------- */

void s1_bridge_config_reply(uint8_t opcode, uint16_t request_id, uint8_t result)
{
    vc_bcmap_snapshot_t snap;
    uint8_t             reply[S1_CFG_REPLY_LEN];

    if (!vc_bcmap_get(&snap))
    {
        /* the map was being replaced during the copy: say BUSY, no map */
        uint8_t none[S1_CFG_MAP_BYTES] = { 0 };

        (void)s1_cfg_encode_reply(opcode, request_id, (uint8_t)VC_BCMAP_BUSY,
                                  0u, 0u, none, reply);
    }
    else
    {
        if (S1_CFG_OP_FACTORY == opcode)
        {
            /* the firmware's factory map; gen and flags stay the active
               map's */
            vc_bcmap_factory(snap.packed);
        }
        (void)s1_cfg_encode_reply(opcode, request_id, result,
                                  (uint16_t)snap.gen, snap.flags, snap.packed,
                                  reply);
    }
    app_check(s1_post(S1_TOPIC_CFG_REPLY, reply, (int32_t)sizeof(reply)),
              "s1 config reply");
}

void s1_bridge_junction(uint8_t kind, uint8_t letter, uint8_t code)
{
    vc_bcmap_snapshot_t snap;
    uint8_t             note[S1_JX_LEN];
    uint16_t            gen = 0u;

    if (vc_bcmap_get(&snap))
    {
        gen = (uint16_t)snap.gen;
    }
    (void)s1_cfg_encode_junction(kind, letter, code, gen, note);
    app_check(s1_post(S1_TOPIC_JUNCTION, note, (int32_t)sizeof(note)),
              "s1 junction note");
}

void s1_bridge_on_config(const uint8_t * p_payload, int32_t len)
{
    s1_cfg_req_t req;

    g_s1_stat.commands++;
    if (0 != s1_cfg_decode_request(p_payload, (int32_t)len, &req))
    {
        s1_bridge_config_reply(req.op, req.request_id,
                               (uint8_t)VC_BCMAP_REJ_MALFORMED);
        return;
    }
    if ((S1_CFG_OP_READ == req.op) || (S1_CFG_OP_FACTORY == req.op))
    {
        s1_bridge_config_reply(req.op, req.request_id, (uint8_t)VC_BCMAP_OK);
        return;
    }
    /* WRITE: the Vehicle Controller applies it, so the "no mission is
       running" check and the change happen in the same task, in order with
       Start.  It replies through s1_bridge_config_reply(). */
    if (!vc_bcmap_post_write(req.request_id, req.expected_gen, req.map))
    {
        s1_bridge_config_reply(req.op, req.request_id, (uint8_t)VC_BCMAP_BUSY);
        return;
    }
    if (vc_post_event(VC_SRC_COMMS, VC_EV_CMD_CONFIG, (int32_t)req.request_id,
                      0, TMO_POL) < E_OK)
    {
        vc_bcmap_cancel_write();
        s1_bridge_config_reply(req.op, req.request_id, (uint8_t)VC_BCMAP_BUSY);
    }
}

void s1_bridge_set_connected(int32_t connected)
{
    bool b_was = g_s1_stat.mqtt_up;

    if (s1_lock())
    {
        s1_txq_set_connected(&g_s1_txq, &g_s1_stats, connected);
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

/* A publish whose result could not be recorded (S1 lock busy afterwards) */
static s1_txq_claim_t s_unfinished;
static int32_t        s_unfinished_err;
static bool           s_b_unfinished_bad;
static bool           s_b_unfinished;

static void * make_tag(int32_t kind, uint32_t gen)
{
    uintptr_t tag = (uintptr_t)(kind + S1_TAG_KIND_BIAS);

    tag |= ((uintptr_t)gen & S1_TAG_GEN_MASK) << S1_TAG_GEN_SHIFT;
    return (void *)tag;
}

int32_t s1_bridge_service(s1_publish_fn_t publish, int32_t max_attempts)
{
    static char    topic[S1_BRIDGE_FULL_TOPIC_MAX];
    static uint8_t payload[S1_BRIDGE_LINE_MAX];
    s1_txq_claim_t claim;
    int32_t        accepted = 0;
    int32_t        attempt;
    int32_t        len;
    int32_t        err;

    if (s_b_unfinished)
    {
        if (!s1_lock())
        {
            s1_lock_failed();
            return 0;
        }
        if (s_b_unfinished_bad)
        {
            s1_txq_drop_claim(&g_s1_txq, &g_s1_stats, &s_unfinished);
        }
        else
        {
            s1_txq_finish(&g_s1_txq, &g_s1_stats, &s_unfinished,
                          s_unfinished_err);
        }
        s1_unlock();
        s_b_unfinished = false;
        if ((!s_b_unfinished_bad) && (s_unfinished_err == S1_ERR_OK))
        {
            g_s1_stat.sent++;
            accepted++;
        }
    }
    for (attempt = 0; attempt < max_attempts; attempt++)
    {
        if (!s1_lock())
        {
            s1_lock_failed();
            break;
        }
        if (!s1_txq_claim(&g_s1_txq, &g_s1_stats, pull_event, &claim))
        {
            s1_unlock();
            break;
        }
        s1_unlock();

        len = line_to_message(claim.item.line, claim.item.len, topic,
                              (int32_t)sizeof(topic), payload,
                              (int32_t)sizeof(payload));
        /* Lock released: producers may post while this item is claimed. */
        err = S1_ERR_MEM;
        if (len >= 0)
        {
            err = publish(topic, payload, (uint16_t)len,
                          make_tag(claim.kind, claim.gen));
        }
        if (!s1_lock())
        {
            /* Record the result on the next drain, before anything else;
               the item stays claimed until then. */
            s1_lock_failed();
            s_unfinished       = claim;
            s_unfinished_err   = (len < 0) ? S1_ERR_OK : err;
            s_b_unfinished_bad = (len < 0);
            s_b_unfinished     = true;
            break;
        }
        if (len < 0)
        {
            /* cannot happen for lines s1_post() accepted; never retry */
            s1_txq_drop_claim(&g_s1_txq, &g_s1_stats, &claim);
            s1_unlock();
            continue;
        }
        s1_txq_finish(&g_s1_txq, &g_s1_stats, &claim, err);
        s1_unlock();
        if (err != S1_ERR_OK)
        {
            break; /* retried on the next drain, never in a tight loop */
        }
        g_s1_stat.sent++;
        accepted++;
    }
    return accepted;
}

void s1_bridge_publish_done(void * p_tag, int32_t err)
{
    uintptr_t tag   = (uintptr_t)p_tag;
    uintptr_t kind1 = tag & S1_TAG_KIND_MASK;

    if (!s1_lock())
    {
        s1_lock_failed();
        return;
    }
    if ((kind1 == 0U) ||
        (kind1 > (uintptr_t)(S1_EVT_KINDS - 1 + S1_TAG_KIND_BIAS)))
    {
        g_s1_stats.cb_stale++; /* not a tag S1 made */
    }
    else
    {
        s1_txq_done(&g_s1_txq, &g_s1_stats, (int32_t)kind1 - S1_TAG_KIND_BIAS,
                    (uint32_t)((tag >> S1_TAG_GEN_SHIFT) & S1_TAG_GEN_MASK),
                    err);
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
    s1_stats_t stats;
    s1_latch_t latch;
    int32_t    len;

    if (E_OK != s1_get_stats(&stats))
    {
        return s1_stats_format(NULL, p_buf, size);
    }
    len = s1_stats_format(&stats, p_buf, size);
    s1_latch_read(&latch);
    len += s1_latch_format(&latch, &p_buf[len], size - len);
    return len;
}

/*** end of file ***/
