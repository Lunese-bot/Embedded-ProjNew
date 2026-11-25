/*
 *      test_s1_queue.c
 *      Host test of the real S1 queue (subsystem1.c, s1_bridge.c) and its
 *      delivery counters (s1_stats.c), with the kernel replaced by
 *      s1_host_kernel.c.
 */

#include <stdio.h>
#include <string.h>

#include "s1_host_kernel.h"
#include "../../../app_program/subsystem1/subsystem1.h"
#include "../../../app_program/subsystem1/s1_codec.h"
#include "../../../app_program/subsystem4/imu_terrain.h"
#include "../../../app_program/subsystem1/s1_cfg.h"
#include "../../../app_program/vehicle_controller/vc_barcode_map.h"

extern int32_t  s1_lock_host_refuse;
extern uint32_t s1_lock_host_fail;

static int32_t g_passed;
static int32_t g_failed;

#define CHECK(id, condition, description) \
    do \
    { \
        if (condition) \
        { \
            g_passed++; \
            printf("[TEST] %s PASS  %s\n", id, description); \
        } \
        else \
        { \
            g_failed++; \
            printf("[TEST] %s FAIL  %s (line %d)\n", id, description, \
                   __LINE__); \
        } \
    } while (0)

static void reset_all(void)
{
    host_kernel_reset();
    s1_lock_host_fail   = 0;
    s1_lock_host_refuse = 0;
    (void)subsystem1_init();
}

static s1_stats_t stats(void)
{
    s1_stats_t snapshot;

    (void)s1_get_stats(&snapshot);
    return snapshot;
}

static void tlm_frame(uint8_t seq, uint8_t frame[S1_TLM_BYTES])
{
    s1_tlm_t tlm = { 0 };

    tlm.version = S1_PROTO_VERSION;
    tlm.seq     = seq;
    s1_tlm_encode(&tlm, frame);
}

static void test_kinds(void)
{
    CHECK("S1-Q01", (s1_stats_kind("t") == S1_KIND_TLM) &&
          (s1_stats_kind(S1_TOPIC_ROOT "t") == S1_KIND_TLM) &&
          (s1_stats_kind("ack") == S1_EVT_ACK) &&
          (s1_stats_kind(S1_TOPIC_ROOT "fault") == S1_EVT_FAULT) &&
          (s1_stats_kind("barcode") == S1_EVT_BARCODE) &&
          (s1_stats_kind("selftest") == S1_EVT_OTHER) &&
          (s1_stats_kind("tt") == S1_EVT_OTHER) &&
          (s1_stats_kind(NULL) == S1_EVT_OTHER),
          "topics are classified by their actual name, with or without root");
}

static void test_format(void)
{
    s1_stats_t st;
    char     line[S1_STATS_TEXT_MAX + 8];
    int32_t  len;

    memset(&st, 0xFF, sizeof(st));
    memset(line, 'X', sizeof(line));
    len = s1_stats_format(&st, line, S1_STATS_TEXT_MAX);
    CHECK("S1-Q02", (len == 255) && (line[255] == '\0') &&
          (line[S1_STATS_TEXT_MAX] == 'X') &&
          (strcmp(line + len - 14, " lf=4294967295") == 0),
          "every counter at its maximum fits S1_STATS_TEXT_MAX (255 chars)");
    memset(line, 'X', sizeof(line));
    len = s1_stats_format(&st, line, 9);
    CHECK("S1-Q03", (len == 8) && (line[8] == '\0') && (line[9] == 'X') &&
          (s1_stats_format(&st, line, 0) == 0),
          "a short buffer is cut and NUL-terminated, never overrun");
}

static int32_t latch_text(uint32_t before, int32_t ok, uint32_t safety,
                          uint32_t after, char * p_line, int32_t size)
{
    s1_latch_t latch;

    s1_latch_resolve(&latch, before, ok, safety, after);
    return s1_latch_format(&latch, p_line, size);
}

static void test_latch(void)
{
    char     line[S1_STATS_TEXT_MAX + 8];
    char     big[S1_STATS_TEXT_MAX];
    s1_latch_t latch;
    int32_t  len;

    (void)latch_text(0U, 1, IMU_SAFETY_LIFT, 0U, line, (int32_t)sizeof(line));
    CHECK("S1-Q07", strcmp(line, " il=0") == 0,
          "interlock clear reads il=0 with no cause, even if S4 still shows one");
    (void)latch_text(1U, 1, IMU_SAFETY_LIFT, 1U, line, (int32_t)sizeof(line));
    CHECK("S1-Q08", strcmp(line, " il=1 ic=lift") == 0,
          "interlock set by a lift reads il=1 ic=lift");
    (void)latch_text(1U, 1, IMU_SAFETY_OVERTURN, 1U, line, (int32_t)sizeof(line));
    CHECK("S1-Q09", strcmp(line, " il=1 ic=overturn") == 0,
          "interlock set by an overturn reads il=1 ic=overturn");
    len = 1;
    (void)latch_text(1U, 1, IMU_SAFETY_NONE, 1U, line, (int32_t)sizeof(line));
    len = len && (strcmp(line, " il=1 ic=unknown") == 0);
    (void)latch_text(1U, 0, IMU_SAFETY_LIFT, 1U, line, (int32_t)sizeof(line));
    len = len && (strcmp(line, " il=1 ic=unknown") == 0);
    (void)latch_text(0U, 1, IMU_SAFETY_LIFT, 1U, line, (int32_t)sizeof(line));
    len = len && (strcmp(line, " il=1 ic=unknown") == 0);
    (void)latch_text(1U, 1, 7U, 1U, line, (int32_t)sizeof(line));
    len = len && (strcmp(line, " il=1 ic=unknown") == 0);
    (void)latch_text(1U, 1, IMU_SAFETY_LIFT, 0U, line, (int32_t)sizeof(line));
    len = len && (strcmp(line, " il=0") == 0);
    CHECK("S1-Q10", len,
          "cause unknown when S4 has not published it, its lock failed, the "
          "interlock changed while read, or the code is unrecognised");
    /* g_s2_imu_stop now holds S2_IMU_CAUSE_* (1 lift, 2 overturn, 3 unknown),
       not only 1: any non-zero value is still "set" */
    len = 1;
    (void)latch_text(2U, 1, IMU_SAFETY_OVERTURN, 2U, line, (int32_t)sizeof(line));
    len = len && (strcmp(line, " il=1 ic=overturn") == 0);
    (void)latch_text(3U, 1, IMU_SAFETY_NONE, 3U, line, (int32_t)sizeof(line));
    len = len && (strcmp(line, " il=1 ic=unknown") == 0);
    (void)latch_text(2U, 1, IMU_SAFETY_OVERTURN, 0U, line, (int32_t)sizeof(line));
    len = len && (strcmp(line, " il=0") == 0);
    CHECK("S1-Q13", len,
          "interlock values 2 and 3 (S2's overturn / unknown cause) still read as set");
    memset(line, 'X', sizeof(line));
    s1_latch_resolve(&latch, 1U, 1, IMU_SAFETY_OVERTURN, 1U);
    CHECK("S1-Q11", (s1_latch_format(&latch, line, S1_LATCH_TEXT_MAX) == 0) &&
          (line[0] == '\0') && (line[1] == 'X') &&
          (s1_latch_format(&latch, line, S1_LATCH_TEXT_MAX + 1) ==
           S1_LATCH_TEXT_MAX) && (line[S1_LATCH_TEXT_MAX] == '\0'),
          "a latch field that does not fit is left out whole, never cut");
    host_kernel_reset();
    memset(&g_s1_stats, 0xFF, sizeof(g_s1_stats));
    host_latch_stop_before = 1U;
    host_latch_stop_after  = 1U;
    host_latch_safety      = IMU_SAFETY_LIFT;
    len = s1_bridge_format_counters(big, (int32_t)sizeof(big));
    CHECK("S1-Q12", (len == (int32_t)strlen(big)) && (strncmp(big, "s1 tq=4294967295 ", 17) == 0) &&
          (strstr(big, " hr=4294967295 lf=") != NULL) &&
          (strcmp(big + len - 13, " il=1 ic=lift") == 0),
          "the counters line (MQTT heartbeat and USB) keeps every counter and "
          "ends with the latch field");
    host_kernel_reset();
}

/* ---- controllable publish stub ---------------------------------------- */
#define PUB_MAX 512

typedef struct
{
    char     topic[S1_BRIDGE_FULL_TOPIC_MAX];
    uint8_t  payload[S1_BRIDGE_LINE_MAX];
    uint16_t len;
    void *   tag;
} pub_t;

static pub_t   g_pub[PUB_MAX];
static int32_t g_pub_count;    /* accepted publishes, in order */
static int32_t g_pub_calls;    /* every call, accepted or not */
static int32_t g_refuse;       /* refuse this many calls next ... */
static int32_t g_refuse_err;   /* ... with this error */
static void (*g_during)(void); /* runs inside publish, lock released */

static int32_t stub_publish(const char * p_topic, const uint8_t * p_payload,
                            uint16_t len, void * p_tag)
{
    extern int32_t s1_lock_host_depth;
    pub_t *        p_pub;

    g_pub_calls++;
    if (s1_lock_host_depth != 0)
    {
        g_failed++;
        printf("[TEST] LOCK FAIL  publish ran with the S1 lock held\n");
    }
    if (g_during != NULL)
    {
        g_during();
    }
    if (g_refuse > 0)
    {
        g_refuse--;
        return g_refuse_err;
    }
    if (g_pub_count < PUB_MAX)
    {
        p_pub = &g_pub[g_pub_count];
        snprintf(p_pub->topic, sizeof(p_pub->topic), "%s", p_topic);
        memcpy(p_pub->payload, p_payload, len);
        p_pub->len = len;
        p_pub->tag = p_tag;
    }
    g_pub_count++;
    return S1_ERR_OK;
}

static void reset_policy(int32_t connected)
{
    reset_all();
    memset(g_pub, 0, sizeof(g_pub));
    g_pub_count  = 0;
    g_pub_calls  = 0;
    g_refuse     = 0;
    g_refuse_err = S1_ERR_MEM;
    g_during     = NULL;
    if (connected)
    {
        s1_bridge_set_connected(1);
    }
}

static void post_tlm(uint8_t seq)
{
    uint8_t frame[S1_TLM_BYTES];

    tlm_frame(seq, frame);
    (void)s1_post(S1_TOPIC_TLM, frame, sizeof(frame));
}

static void post_ack(uint8_t seq)
{
    uint8_t ack = s1_ack_encode(S1_ACK_QUEUED, seq);

    (void)s1_post(S1_TOPIC_ACK, &ack, 1);
}

static int32_t pub_seq(int32_t idx)
{
    s1_tlm_t tlm;

    if (s1_tlm_decode(g_pub[idx].payload, g_pub[idx].len, &tlm) != 0)
    {
        return -1;
    }
    return tlm.seq;
}

static bool pub_is(int32_t idx, const char * p_suffix)
{
    char full[S1_BRIDGE_FULL_TOPIC_MAX];

    snprintf(full, sizeof(full), "%s%s", S1_TOPIC_ROOT, p_suffix);
    return (idx < g_pub_count) && (strcmp(g_pub[idx].topic, full) == 0);
}

/* lwIP completes every pending QoS 0 request together on a TCP ACK */
static void complete(int32_t from, int32_t to, int32_t err)
{
    int32_t idx;

    for (idx = from; idx < to; idx++)
    {
        s1_bridge_publish_done(g_pub[idx].tag, err);
    }
}

static void test_refused_event(void)
{
    const uint8_t barcode[] = { 0x10, 0x82, 0x00, 0xFF };
    s1_stats_t      st;
    int32_t       idx;

    reset_policy(1);
    (void)s1_post(S1_TOPIC_BARCODE, barcode, sizeof(barcode));
    g_refuse = 5;
    for (idx = 0; idx < 5; idx++)
    {
        (void)s1_bridge_service(stub_publish, 4);
    }
    st = stats();
    CHECK("S1-P01", (g_pub_calls == 5) && (g_pub_count == 0) &&
          (st.ref_mem == 5) && (st.evt_drop[S1_EVT_BARCODE] == 0) &&
          (st.pub_evt == 0),
          "five refusals of one event: five attempts, no drop, one call each");
    (void)s1_bridge_service(stub_publish, 4);
    (void)s1_bridge_service(stub_publish, 4);
    st = stats();
    CHECK("S1-P02", (g_pub_count == 1) && pub_is(0, S1_TOPIC_BARCODE) &&
          (g_pub[0].len == sizeof(barcode)) &&
          (memcmp(g_pub[0].payload, barcode, sizeof(barcode)) == 0) &&
          (st.pub_evt == 1) && (st.inflight_evt == 1),
          "then accepted: same topic, bytes and length, published once");

    reset_policy(1);
    (void)s1_post("selftest", barcode, 3);
    (void)s1_bridge_service(stub_publish, 4);
    CHECK("S1-P03", pub_is(0, "selftest") && (g_pub[0].len == 3),
          "other topics from existing callers are still published");
}

static void test_event_priority(void)
{
    int32_t  idx;
    bool     b_ok = true;
    s1_stats_t st;

    reset_policy(1);
    for (idx = 0; idx < 20; idx++)
    {
        post_tlm((uint8_t)idx);
    }
    post_ack(30);
    (void)s1_bridge_service(stub_publish, 1);
    CHECK("S1-P04", (g_pub_count == 1) && pub_is(0, S1_TOPIC_ACK) &&
          (g_pub[0].payload[0] == s1_ack_encode(S1_ACK_QUEUED, 30)),
          "after 20 telemetry frames, the ack takes the next publish");
    (void)s1_bridge_service(stub_publish, 4);
    for (idx = 1; idx < g_pub_count; idx++)
    {
        b_ok = b_ok && pub_is(idx, S1_TOPIC_TLM) &&
               (pub_seq(idx) == 15 + idx);
    }
    st = stats();
    CHECK("S1-P05", b_ok && (g_pub_count == 5) && (st.tlm_lost == 16),
          "telemetry under the cap keeps FIFO order (newest 4 of 20)");

    /* full backlog: telemetry at its in-flight cap, frames still arriving */
    reset_policy(1);
    for (idx = 0; idx < 12; idx++)
    {
        post_tlm((uint8_t)idx);
        (void)s1_bridge_service(stub_publish, 4);
    }
    (void)s1_post("fault", "\x06\x02\x00\x02", 4);
    post_ack(36);
    for (idx = 12; idx < 40; idx++)
    {
        post_tlm((uint8_t)idx);
    }
    g_refuse = 3; /* the radio is stalled too */
    for (idx = 0; idx < 4; idx++)
    {
        (void)s1_bridge_service(stub_publish, 4);
    }
    st = stats();
    CHECK("S1-P06", (g_pub_count == 12) && pub_is(10, "fault") &&
          pub_is(11, S1_TOPIC_ACK) &&
          (g_pub[11].payload[0] == s1_ack_encode(S1_ACK_QUEUED, 36)) &&
          (st.evt_drop[S1_EVT_FAULT] == 0) && (st.evt_drop[S1_EVT_ACK] == 0),
          "fault and ack survive telemetry pressure at the cap, in order");
}

static void test_inflight_cap(void)
{
    int32_t  idx;
    bool     b_cap = true;
    s1_stats_t st;

    reset_policy(1);
    for (idx = 0; idx < 50; idx++)
    {
        post_tlm((uint8_t)idx);
        (void)s1_bridge_service(stub_publish, 4);
        b_cap = b_cap && (stats().inflight_tlm <= S1_TLM_INFLIGHT_MAX);
    }
    st = stats();
    CHECK("S1-P07", b_cap && (st.inflight_tlm == 10) && (st.pub_tlm == 10) &&
          (s1_txq_tlm_pending(&g_s1_txq) == 1) && (st.tlm_lost == 39),
          "callbacks withheld: never more than 10 telemetry in flight");

    post_ack(7);
    (void)s1_bridge_service(stub_publish, 4);
    CHECK("S1-P08", (g_pub_count == 11) && pub_is(10, S1_TOPIC_ACK),
          "an event still gets its publish while telemetry is at the cap");

    complete(0, 10, S1_ERR_OK); /* one TCP ACK completes all ten */
    (void)s1_bridge_service(stub_publish, 4);
    st = stats();
    CHECK("S1-P09", (st.cb_ok == 10) && (st.inflight_tlm == 1) &&
          (g_pub_count == 12) && (pub_seq(11) == 49),
          "after a batched completion the next telemetry is the newest");

    complete(11, 12, -3 /* ERR_TIMEOUT */);
    st = stats();
    CHECK("S1-P10", (st.cb_err == 1) && (st.inflight_tlm == 0),
          "a timeout completion releases its slot");

    for (idx = 50; idx < 55; idx++)
    {
        post_tlm((uint8_t)idx);
        (void)s1_bridge_service(stub_publish, 4);
    }
    s1_bridge_set_connected(0); /* lwIP clears requests, no callbacks */
    st = stats();
    CHECK("S1-P11", (st.unconfirmed == 6) && (st.inflight_tlm == 0) &&
          (st.inflight_evt == 0),
          "disconnect without callbacks: in-flight become unconfirmed");

    s1_bridge_set_connected(1);
    post_tlm(60);
    (void)s1_bridge_service(stub_publish, 4);
    complete(12, 17, S1_ERR_OK); /* late completions of the old link */
    st = stats();
    CHECK("S1-P12", (st.cb_stale == 5) && (st.inflight_tlm == 1) &&
          (st.cb_ok == 10),
          "stale completions never touch the new connection's count");
}

static void test_offline(void)
{
    int32_t  idx;
    bool     b_order = true;
    s1_stats_t st;

    reset_policy(0);
    for (idx = 0; idx < 300; idx++) /* 60 s at 200 ms */
    {
        post_tlm((uint8_t)idx);
    }
    st = stats();
    CHECK("S1-P13", (st.tlm_posted == 300) && (st.tlm_lost == 299) &&
          (s1_txq_tlm_pending(&g_s1_txq) == 1),
          "300 posts offline leave one frame and 299 skipped");

    for (idx = 0; idx < 12; idx++)
    {
        post_ack((uint8_t)idx);
    }
    st = stats();
    CHECK("S1-P14", (st.evt_posted == 12) && (st.evt_drop[S1_EVT_ACK] == 4) &&
          (g_s1_txq.evt_waiting == S1_EVT_QUEUE_MAX),
          "12 offline acks: the 4 oldest are evicted and counted");

    (void)s1_bridge_service(stub_publish, 4);
    CHECK("S1-P15", g_pub_calls == 0, "nothing is offered while disconnected");

    s1_bridge_set_connected(1);
    for (idx = 0; idx < 4; idx++)
    {
        (void)s1_bridge_service(stub_publish, 4);
    }
    for (idx = 0; idx < 8; idx++)
    {
        b_order = b_order && pub_is(idx, S1_TOPIC_ACK) &&
                  (g_pub[idx].payload[0] ==
                   s1_ack_encode(S1_ACK_QUEUED, (uint8_t)(idx + 4)));
    }
    CHECK("S1-P16", b_order && (g_pub_count == 9) &&
          (pub_seq(8) == (uint8_t)299),
          "after reconnect: acks 4..11 in order, then the newest frame");
}

/* Producers on the other core post while an item is claimed. */
static int32_t g_during_seq;
static int32_t g_during_calls;
static void post_during(void)
{
    if ((g_during_calls % 3) == 0) /* below the drain rate: no overflow */
    {
        post_ack((uint8_t)(g_during_seq & 0x3F));
        g_during_seq++;
    }
    post_tlm((uint8_t)g_during_calls);
    g_during_calls++;
}

static void test_claimed_interleave(void)
{
    int32_t  idx;
    bool     b_ok = true;
    s1_stats_t st;
    uint8_t  next_ack = 0;

    reset_policy(1);
    g_during       = post_during;
    g_during_seq   = 0;
    g_during_calls = 0;
    post_ack(63);
    for (idx = 0; idx < 40; idx++)
    {
        g_refuse = (idx % 3 == 0) ? 1 : 0;
        (void)s1_bridge_service(stub_publish, 2);
        if ((idx % 4) == 3)
        {
            complete(0, g_pub_count, S1_ERR_OK); /* extra completions are stale */
        }
    }
    g_during = NULL;
    s1_bridge_set_connected(0);
    s1_bridge_set_connected(1);
    for (idx = 0; idx < 200; idx++)
    {
        (void)s1_bridge_service(stub_publish, 4);
    }
    /* every ack in order, once: 63 first, then 0, 1, 2 ... */
    for (idx = 0; idx < g_pub_count; idx++)
    {
        if (pub_is(idx, S1_TOPIC_ACK))
        {
            uint8_t want = (idx == 0) ? 63 : next_ack++;
            b_ok = b_ok && (s1_ack_seq(g_pub[idx].payload[0]) == want);
        }
    }
    st = stats();
    CHECK("S1-P17", b_ok && (st.evt_posted == st.pub_evt) &&
          (st.evt_drop[S1_EVT_ACK] == 0) &&
          (st.evt_posted == (uint32_t)g_during_seq + 1U),
          "posts during a claimed publish: no duplicate, no lost ack");
    CHECK("S1-P18", (st.tlm_posted == st.pub_tlm + st.tlm_lost +
                     (uint32_t)s1_txq_tlm_pending(&g_s1_txq)) &&
          (st.inflight_tlm <= S1_TLM_INFLIGHT_MAX) &&
          (st.ref_mem == 14),
          "telemetry accounting balances; refusals counted per attempt");
}

static void refuse_next_lock(void)
{
    s1_lock_host_refuse = 1; /* the lock after publish returns is busy */
}

static void test_lock_busy(void)
{
    s1_stats_t st;

    reset_policy(1);
    post_ack(9);
    g_during = refuse_next_lock;
    (void)s1_bridge_service(stub_publish, 4);
    g_during = NULL;
    st = stats();
    CHECK("S1-P21", (g_pub_count == 1) && (st.pub_evt == 0) &&
          (g_s1_txq.evt_claimed == 1),
          "lock busy after an accepted publish: the item stays claimed");
    (void)s1_bridge_service(stub_publish, 4);
    st = stats();
    CHECK("S1-P22", (g_pub_count == 1) && (st.pub_evt == 1) &&
          (st.inflight_evt == 1) && (g_s1_txq.evt_claimed == 0) &&
          (g_s1_txq.has_evt == 0),
          "the next drain commits it exactly once, with no second publish");

    reset_policy(1);
    s1_lock_host_refuse = 1;
    CHECK("S1-P19", s1_post(S1_TOPIC_ACK, "\x01", 1) == E_TMOUT,
          "a busy lock makes s1_post return at once with E_TMOUT");
    st = stats();
    CHECK("S1-P20", (st.lock_fail == 1) && (st.evt_posted == 0),
          "and the failure is counted");
}

/* ---- counters at the buffer level ----------------------------------- */
static void test_eviction_counts(void)
{
    int32_t  idx;
    s1_stats_t st;

    reset_policy(0);
    post_ack(1);
    for (idx = 0; idx < 40; idx++)
    {
        post_tlm((uint8_t)idx);
    }
    st = stats();
    CHECK("S1-Q04", (st.tlm_posted == 40) && (st.evt_drop[S1_EVT_ACK] == 0) &&
          (st.evt_posted == 1) && (g_s1_txq.evt_waiting == 1),
          "telemetry no longer evicts an ack (the stage-1 loss path)");

    for (idx = 0; idx < S1_EVT_QUEUE_MAX; idx++)
    {
        (void)s1_post(S1_TOPIC_BARCODE, "\x41\x42", 2);
    }
    st = stats();
    CHECK("S1-Q05", (st.evt_drop[S1_EVT_ACK] == 1) &&
          (st.evt_drop[S1_EVT_BARCODE] == 0) &&
          (host_mbf_used[0] <= S1_EVT_QUEUE_MAX * (S1_LINE_MAX + 4)),
          "the ninth event evicts the oldest, counted under its own topic");

    for (idx = 0; idx < S1_EVT_QUEUE_MAX; idx++)
    {
        uint8_t big[S1_LINE_MAX - 6];

        memset(big, idx, sizeof(big));
        CHECK("S1-Q06", s1_post("fault", big, sizeof(big)) == E_OK,
              "longest events always fit the buffer at the event cap");
    }
}


/* robot/g10/cfg -> robot/g10/cfgr through the real S1 bridge and queue */
static int32_t cfg_reply(uint8_t out[S1_CFG_REPLY_LEN])
{
    int32_t start = g_pub_count;

    (void)s1_bridge_service(stub_publish, 1);
    if ((g_pub_count != start + 1) || !pub_is(start, S1_TOPIC_CFG_REPLY) ||
        (g_pub[start].len != S1_CFG_REPLY_LEN))
    {
        return 0;
    }
    memcpy(out, g_pub[start].payload, S1_CFG_REPLY_LEN);
    return 1;
}

static void test_config(void)
{
    static const uint8_t factory[7] = { 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x20 };
    uint8_t msg[S1_CFG_WRITE_LEN];
    uint8_t rep[S1_CFG_REPLY_LEN];
    int32_t calls;

    reset_policy(1);
    vc_bcmap_init(NULL); /* factory map, RAM only */
    calls = host_vc_calls;
    (void)s1_cfg_encode_read(0x1234u, msg);
    s1_bridge_on_config(msg, S1_CFG_READ_LEN);
    CHECK("S1-G01", cfg_reply(rep) && (rep[0] == 0xC1) && (rep[1] == 0x81) &&
          (rep[2] == 0x12) && (rep[3] == 0x34) && (rep[4] == VC_BCMAP_OK) &&
          (memcmp(&rep[8], factory, 7) == 0) && (host_vc_calls == calls),
          "READ is answered at once on robot/g10/cfgr with the id and the active map");

    reset_policy(1);
    calls = host_vc_calls;
    msg[0] = 0xC2;
    s1_bridge_on_config(msg, S1_CFG_READ_LEN);
    s1_bridge_on_config(msg, -1); /* truncated */
    CHECK("S1-G02", cfg_reply(rep) && (rep[4] == VC_BCMAP_REJ_MALFORMED) &&
          (rep[1] == 0x80) && cfg_reply(rep) && (rep[4] == VC_BCMAP_REJ_MALFORMED) &&
          (host_vc_calls == calls),
          "malformed or truncated request: MALFORMED reply, nothing reaches the VC");

    reset_policy(1);
    (void)s1_cfg_encode_write(0x0202u, 0u, factory, msg);
    calls = host_vc_calls;
    s1_bridge_on_config(msg, S1_CFG_WRITE_LEN);
    (void)s1_bridge_service(stub_publish, 4);
    CHECK("S1-G03", (host_vc_calls == calls + 1) && (g_pub_count == 0),
          "WRITE goes to the Vehicle Controller (one event); no reply until it is applied");

    s1_bridge_on_config(msg, S1_CFG_WRITE_LEN);
    CHECK("S1-G04", cfg_reply(rep) && (rep[4] == VC_BCMAP_BUSY) &&
          (rep[2] == 0x02) && (host_vc_calls == calls + 1),
          "a second WRITE while one is pending: BUSY, not queued");
    vc_bcmap_cancel_write();

    reset_policy(1);
    host_vc_result = E_TMOUT;
    s1_bridge_on_config(msg, S1_CFG_WRITE_LEN);
    host_vc_result = E_OK;
    CHECK("S1-G05", cfg_reply(rep) && (rep[4] == VC_BCMAP_BUSY) &&
          vc_bcmap_post_write(1u, 0u, factory),
          "VC queue full: BUSY reply and the pending slot is released");
    vc_bcmap_cancel_write();

    reset_policy(1);
    s1_bridge_config_reply(S1_CFG_OP_WRITE, 0x0303u, VC_BCMAP_REJ_ACTIVE);
    CHECK("S1-G06", cfg_reply(rep) && (rep[1] == 0x82) && (rep[2] == 0x03) &&
          (rep[3] == 0x03) && (rep[4] == VC_BCMAP_REJ_ACTIVE),
          "the VC's answer is published with the request id and result");

    /* READ_FACTORY: the firmware's factory map, gen/flags of the active map */
    reset_policy(1);
    {
        uint8_t m[7];
        int32_t pub0;

        memcpy(m, factory, 7);
        m[0] = 0x10; /* B = left */
        (void)vc_bcmap_replace(m, 0u, false);
        msg[0] = S1_CFG_MARKER; msg[1] = S1_CFG_OP_FACTORY; msg[2] = 0x00; msg[3] = 0x09;
        calls = host_vc_calls;
        s1_bridge_on_config(msg, S1_CFG_READ_LEN);
        CHECK("S1-G07", cfg_reply(rep) && (rep[1] == 0x83) && (rep[3] == 0x09) &&
              (rep[4] == VC_BCMAP_OK) && (memcmp(&rep[8], factory, 7) == 0) &&
              (((rep[5] << 8) | rep[6]) == 1) && ((rep[7] & VC_BCMAP_FLAG_FACTORY_EQUAL) == 0) &&
              (host_vc_calls == calls),
              "READ_FACTORY answers at once with the firmware's factory map; gen and flags are the active (edited) map's");
        s1_bridge_on_config(msg, S1_CFG_READ_LEN + 1);
        CHECK("S1-G08", cfg_reply(rep) && (rep[4] == VC_BCMAP_REJ_MALFORMED),
              "READ_FACTORY with extra bytes is malformed");

        reset_policy(1);
        pub0 = g_pub_count;
        s1_bridge_junction(2u, (uint8_t)'B', 1u);
        (void)s1_bridge_service(stub_publish, 1);
        CHECK("S1-G09", (g_pub_count == pub0 + 1) && pub_is(pub0, S1_TOPIC_JUNCTION) &&
              (g_pub[pub0].len == S1_JX_LEN) && (g_pub[pub0].payload[0] == 0xD1) &&
              (g_pub[pub0].payload[1] == 2) && (g_pub[pub0].payload[2] == 'B') &&
              (g_pub[pub0].payload[3] == 1) && (g_pub[pub0].payload[5] == 1),
              "junction note goes out on robot/g10/jx: D1 kind letter code gen");
        vc_bcmap_init(NULL);
    }
}

/* S1-Q20: the obstacle profile never evicts an ack or fault */
static void test_post_spare(void)
{
    static const uint8_t frame[20] = { 1, 0, 1 };
    int32_t  idx;
    int32_t  posted = 0;
    s1_stats_t st;

    reset_policy(0);
    for (idx = 0; idx < S1_EVT_QUEUE_MAX + 4; idx++)
    {
        if (E_OK == s1_post_spare("obstacle/profile", frame, sizeof(frame), 3))
        {
            posted++;
        }
    }
    CHECK("S1-Q20", (posted == S1_EVT_QUEUE_MAX - 3) &&
          (g_s1_txq.evt_waiting == S1_EVT_QUEUE_MAX - 3) &&
          (stats().evt_drop[S1_EVT_OTHER] == 0),
          "profile posts stop with 3 event slots still free, nothing evicted");
    post_ack(1);
    post_ack(2);
    (void)s1_post("fault", "\x01\x02", 2);
    st = stats();
    CHECK("S1-Q20", (st.evt_drop[S1_EVT_ACK] == 0) &&
          (st.evt_drop[S1_EVT_FAULT] == 0) && (st.evt_drop[S1_EVT_OTHER] == 0) &&
          (g_s1_txq.evt_waiting == S1_EVT_QUEUE_MAX),
          "two acks and a fault still fit after the profile filled its share");
    CHECK("S1-Q20", (E_PAR == s1_post_spare(S1_TOPIC_TLM, frame, sizeof(frame), 3)) &&
          (E_PAR == s1_post_spare("obstacle/profile", frame, sizeof(frame), 0)) &&
          (E_PAR == s1_post_spare("obstacle/profile", frame, sizeof(frame), S1_EVT_QUEUE_MAX)),
          "telemetry topic and spare outside 1..max-1 are refused");
}

int main(void)
{
    test_post_spare();
    test_kinds();
    test_format();
    test_latch();
    test_eviction_counts();
    test_refused_event();
    test_event_priority();
    test_inflight_cap();
    test_offline();
    test_claimed_interleave();
    test_lock_busy();
    test_config();
    printf("[TEST] subsystem1 queue host SUMMARY %d passed, %d failed\n",
           (int)g_passed, (int)g_failed);
    return (g_failed == 0) ? 0 : 1;
}

/*** end of file ***/
