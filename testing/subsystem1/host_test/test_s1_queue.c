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

static S1_STATS stats(void)
{
    S1_STATS snapshot;

    (void)s1_get_stats(&snapshot);
    return snapshot;
}

static void tlm_frame(uint8_t seq, uint8_t frame[S1_TLM_BYTES])
{
    S1_TLM tlm = { 0 };

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
    S1_STATS st;
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

/* The 800-byte queue holds 66 one-byte acks (12 bytes each). */
static void test_eviction_counts(void)
{
    const uint8_t ack = 0x01;
    uint8_t       frame[S1_TLM_BYTES];
    int32_t       idx;
    S1_STATS      st;

    reset_all();
    for (idx = 0; idx < 70; idx++)
    {
        (void)s1_post(S1_TOPIC_ACK, &ack, 1);
    }
    st = stats();
    CHECK("S1-Q04", (st.evt_posted == 70) && (st.evt_drop[S1_EVT_ACK] == 4) &&
          (g_s1_stat.dropped == 4) && (host_mbf_count[0] == 66),
          "each evicted ack is counted once, as an ack");

    reset_all();
    (void)s1_post(S1_TOPIC_ACK, &ack, 1);
    for (idx = 0; idx < 40; idx++)
    {
        tlm_frame((uint8_t)idx, frame);
        (void)s1_post(S1_TOPIC_TLM, frame, sizeof(frame));
    }
    st = stats();
    CHECK("S1-Q05", (st.tlm_posted == 40) && (st.evt_drop[S1_EVT_ACK] == 1) &&
          (st.tlm_lost == 0),
          "baseline rule: telemetry filling the queue evicts the oldest ack");

    s1_lock_host_refuse = 1;
    (void)s1_post(S1_TOPIC_TLM, frame, sizeof(frame));
    st = stats();
    CHECK("S1-Q06", (st.lock_fail == 1) && (st.tlm_posted == 40) &&
          (host_mbf_count[0] == 40),
          "a busy lock skips only the counters, never the message");
}

int main(void)
{
    test_kinds();
    test_format();
    test_eviction_counts();
    printf("[TEST] subsystem1 queue host SUMMARY %d passed, %d failed\n",
           (int)g_passed, (int)g_failed);
    return (g_failed == 0) ? 0 : 1;
}

/*** end of file ***/
