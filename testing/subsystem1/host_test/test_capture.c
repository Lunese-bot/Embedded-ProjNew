/* Exercise the test capture and production bridge with simulated message buffers. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define APP_H
#define VEHICLE_CONTROLLER_H
#define __TM_TMONITOR_H__
#define APP_TEST 1
typedef int ID;
typedef int ER;
typedef int vc_event_type_t;
typedef void (*FP)();
typedef struct { int mbfatr; size_t bufsz, maxmsz; } T_CMBF;
typedef struct { int cycatr; FP cychdr; int cyctim, cycphs; } T_CCYC;
typedef struct { uint32_t lo; } SYSTIM;
#define E_OK 0
#define E_PAR (-17)
#define E_TMOUT (-50)
#define TMO_POL 0
#define TA_TFIFO 0
#define TA_HLNG 0
#define TA_STA 0
enum { VC_SRC_COMMS, VC_EV_CMD_START, VC_EV_CMD_STOP, VC_EV_CMD_RESET,
       VC_EV_CMD_PAUSE, VC_EV_CMD_RESUME, VC_EV_WIFI_UP, VC_EV_WIFI_DOWN };

static ID tk_cre_mbf(T_CMBF *p);
static ER tk_snd_mbf(ID id, const void *data, int len, int timeout);
static int32_t tk_rcv_mbf(ID id, void *data, int timeout);
static ER tk_wup_tsk(ID id);
static ID tk_cre_cyc(T_CCYC *p);
static ID app_start_task(FP entry, int pri, uint32_t prc, const char *name);
static void app_check(int32_t ercd, const char *what);
static ER vc_post_event(int src, int type, int p1, int p2, int timeout);
static ER tk_get_tim(SYSTIM *time);
static ER tk_dly_tsk(int delay);
static int tm_printf(const uint8_t *format, ...);

#include "../../../app_program/subsystem1/subsystem1.c"
#include "../../../app_program/subsystem1/s1_bridge.c"
#include "../../../app_program/subsystem1/subsystem1_test.c"

/* Target reads S2/S4 (s1_latch_port.c); the capture tests need no latch. */
void s1_latch_read(S1_LATCH * p_out)
{
    s1_latch_resolve(p_out, 0U, 1, 0U, 0U);
}

#define FAKE_MAX_MESSAGES (S1_QUEUE_LINES * (S1_LINE_MAX + 4) / 8)

typedef struct
{
    uint8_t messages[FAKE_MAX_MESSAGES][S1_LINE_MAX];
    int32_t lengths[FAKE_MAX_MESSAGES];
    int count;
    size_t used;
} fake_queue_t;
static fake_queue_t queues[2];
static size_t queue_capacity[2];
static int queue_count;
static int drain_immediately, test_failures, test_passes, telemetry_enabled;
static int vc_calls, vc_last_type, vc_last_seq, vc_result;
static uint32_t clock_ms;

static ID tk_cre_mbf(T_CMBF *p)
{
    assert(queue_count < 2);
    queue_capacity[queue_count] = p->bufsz;
    return ++queue_count;
}
static size_t message_size(int len)
{
    return sizeof(int32_t) + (((size_t)len + 3U) & ~(size_t)3U);
}
static ER tk_snd_mbf(ID id, const void *data, int len, int timeout)
{
    fake_queue_t *q;
    (void)timeout;
    assert(id > 0 && id <= 2 && len <= S1_LINE_MAX);
    q = &queues[id - 1];
    if (q->used + message_size(len) > queue_capacity[id - 1]) return E_TMOUT;
    assert(q->count < FAKE_MAX_MESSAGES);
    memcpy(q->messages[q->count], data, (size_t)len);
    q->lengths[q->count++] = len;
    q->used += message_size(len);
    if (drain_immediately && id == g_mbf_telemetry)
    {
        char topic[S1_BRIDGE_FULL_TOPIC_MAX];
        uint8_t payload[S1_BRIDGE_LINE_MAX];
        assert(s1_bridge_pop(topic, sizeof(topic), payload, sizeof(payload)) > 0);
    }
    return E_OK;
}
static int32_t tk_rcv_mbf(ID id, void *data, int timeout)
{
    fake_queue_t *q;
    int32_t len;
    (void)timeout;
    assert(id > 0 && id <= 2);
    q = &queues[id - 1];
    if (!q->count) return E_TMOUT;
    len = q->lengths[0];
    memcpy(data, q->messages[0], (size_t)len);
    q->used -= message_size(len);
    q->count--;
    memmove(q->messages, q->messages + 1, (size_t)q->count * sizeof(q->messages[0]));
    memmove(q->lengths, q->lengths + 1, (size_t)q->count * sizeof(q->lengths[0]));
    return len;
}
static ER tk_wup_tsk(ID id) { (void)id; return E_OK; }
static ID tk_cre_cyc(T_CCYC *p) { (void)p; return 1; }
static ID app_start_task(FP entry, int pri, uint32_t prc, const char *name)
{ (void)entry; (void)pri; (void)prc; (void)name; return 1; }
static void app_check(int32_t ercd, const char *what)
{ (void)what; assert(ercd == E_OK); }
static ER vc_post_event(int src, int type, int p1, int p2, int timeout)
{
    assert(src == VC_SRC_COMMS && p2 == 0 && timeout == TMO_POL);
    vc_calls++;
    vc_last_type = type;
    vc_last_seq = p1;
    return vc_result;
}
static ER tk_get_tim(SYSTIM *time) { time->lo = clock_ms; return E_OK; }
static ER tk_dly_tsk(int delay)
{
    uint32_t previous = clock_ms;
    clock_ms += (uint32_t)delay;
    if (telemetry_enabled &&
        ((previous / S1_PERIOD_MS) != (clock_ms / S1_PERIOD_MS)))
    {
        S1_TLM frame = {0};
        uint8_t encoded[S1_TLM_BYTES];
        frame.version = S1_PROTO_VERSION;
        s1_tlm_encode(&frame, encoded);
        assert(s1_post(S1_TOPIC_TLM, encoded, sizeof(encoded)) == E_OK);
    }
    return E_OK;
}
static int tm_printf(const uint8_t *format, ...)
{
    char line[256];
    va_list args;
    int result;
    va_start(args, format);
    result = vsnprintf(line, sizeof(line), (const char *)format, args);
    va_end(args);
    if (strstr(line, " FAIL ")) test_failures++;
    if (strstr(line, " PASS ")) test_passes++;
    return result;
}
void telemetry_task(int32_t stacd, void *p_exinf)
{ (void)stacd; (void)p_exinf; }

int main(void)
{
    const uint8_t binary[] = { 0x41, 0x00, 0xff };
    char topic[S1_BRIDGE_FULL_TOPIC_MAX];
    uint8_t payload[S1_BRIDGE_LINE_MAX];
    S1_TLM frame = {0}, decoded;
    uint8_t encoded[S1_TLM_BYTES];
    int i;
    int retained;

    assert(subsystem1_init() == E_OK);
    drain_immediately = 1;
    telemetry_enabled = 1;
    for (i = 0; i < 20; i++)
    {
        assert(s1_post("ack", binary, sizeof(binary)) == E_OK);
    }
    subsystem1_test();
    assert(test_failures == 0 && test_passes == 4 && clock_ms >= S1_PERIOD_MS);
    memset(queues, 0, sizeof(queues));
    memset(&g_s1_stat, 0, sizeof(g_s1_stat));
    test_failures = test_passes = 0;
    clock_ms = 0;
    telemetry_enabled = 0;
    frame.version = S1_PROTO_VERSION;
    s1_tlm_encode(&frame, encoded);
    for (i = 0; i < 40; i++)
    {
        assert(s1_post(S1_TOPIC_TLM, encoded, sizeof(encoded)) == E_OK);
    }
    subsystem1_test();
    assert(test_failures == 1 && test_passes == 3 && clock_ms >= 400U);
    drain_immediately = 0;
    memset(queues, 0, sizeof(queues));
    memset(&g_s1_stat, 0, sizeof(g_s1_stat));

    assert(s1_post("selftest", binary, sizeof(binary)) == E_OK);
    assert(s1_bridge_pop(topic, sizeof(topic), payload, sizeof(payload)) == sizeof(binary));
    assert(s1_test_bridge_pop(topic, sizeof(topic), payload, sizeof(payload)) == sizeof(binary));
    assert(strcmp(topic, S1_TOPIC_ROOT "selftest") == 0);
    assert(memcmp(payload, binary, sizeof(binary)) == 0);
    assert(g_s1_stat.sent == 1);

    frame.version = S1_PROTO_VERSION;
    for (i = 0; i < 90; i++)
    {
        frame.seq = (uint8_t)i;
        s1_tlm_encode(&frame, encoded);
        assert(s1_post(S1_TOPIC_TLM, encoded, sizeof(encoded)) == E_OK);
        assert(s1_bridge_pop(topic, sizeof(topic), payload, sizeof(payload)) == S1_TLM_BYTES);
    }
    retained = (int)(queue_capacity[g_mbf_s1_test - 1] /
                     message_size(2 + S1_TLM_BYTES));
    for (i = 90 - retained; i < 90; i++)
    {
        assert(s1_test_bridge_pop(topic, sizeof(topic), payload, sizeof(payload)) == S1_TLM_BYTES);
        assert(strcmp(topic, S1_TOPIC_ROOT S1_TOPIC_TLM) == 0);
        assert(s1_tlm_decode(payload, S1_TLM_BYTES, &decoded) == 0);
        assert(decoded.seq == i);
    }
    assert(s1_test_bridge_pop(topic, sizeof(topic), payload, sizeof(payload)) == 0);
    assert(g_s1_stat.sent == 91);

    memset(queues, 0, sizeof(queues));
    {
        const uint8_t pause[] = { 0xc5, S1_EXT_PAUSE };
        const uint8_t resume[] = { 0xc6, S1_EXT_RESUME };

        vc_calls = 0;
        s1_bridge_on_command(pause, sizeof(pause));
#if S1_ENABLE_REMOTE_PAUSE
        assert(vc_calls == 1 && vc_last_type == VC_EV_CMD_PAUSE && vc_last_seq == 5);
        assert(s1_bridge_pop(topic, sizeof(topic), payload, sizeof(payload)) == 0);
        s1_bridge_command_result(5, 1);
        assert(s1_bridge_pop(topic, sizeof(topic), payload, sizeof(payload)) == 1);
        assert(payload[0] == s1_ack_encode(S1_ACK_QUEUED, 5));

        vc_result = E_TMOUT;
        s1_bridge_on_command(resume, sizeof(resume));
        assert(vc_calls == 2 && vc_last_type == VC_EV_CMD_RESUME && vc_last_seq == 6);
        assert(s1_bridge_pop(topic, sizeof(topic), payload, sizeof(payload)) == 1);
        assert(payload[0] == s1_ack_encode(S1_ACK_BUSY, 6));

        vc_result = E_OK;
        s1_bridge_on_command(resume, sizeof(resume));
        assert(vc_calls == 3 && vc_last_type == VC_EV_CMD_RESUME && vc_last_seq == 6);
        assert(s1_bridge_pop(topic, sizeof(topic), payload, sizeof(payload)) == 0);
        s1_bridge_command_result(6, 0);
        assert(s1_bridge_pop(topic, sizeof(topic), payload, sizeof(payload)) == 1);
        assert(payload[0] == s1_ack_encode(S1_ACK_REJECTED, 6));
#else
        assert(vc_calls == 0);
        assert(s1_bridge_pop(topic, sizeof(topic), payload, sizeof(payload)) == 1);
        assert(payload[0] == s1_ack_encode(S1_ACK_REJECTED, 5));
        s1_bridge_on_command(resume, sizeof(resume));
        assert(vc_calls == 0);
        assert(s1_bridge_pop(topic, sizeof(topic), payload, sizeof(payload)) == 1);
        assert(payload[0] == s1_ack_encode(S1_ACK_REJECTED, 6));
#endif
    }
    puts("S1 Pause/Resume command bridge: PASS");
    puts("S1 independent test capture: PASS");
    return 0;
}
