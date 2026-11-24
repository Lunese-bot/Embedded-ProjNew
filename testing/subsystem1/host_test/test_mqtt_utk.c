/*
 *      test_mqtt_utk.c
 *      Host test of lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c with the
 *      real S1 implementation behind it and lwIP replaced by the fakes in
 *      fake_lwip/ and below.
 */

#include <stdio.h>
#include <string.h>

#include "../../../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c"
#include "s1_host_kernel.h" /* before subsystem1.h: replaces the kernel */
#include "../../../app_program/subsystem1/subsystem1.h"

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

/* ---- fake lwIP MQTT client ------------------------------------------- */
#define FAKE_PUB_MAX     64
#define FAKE_TIMER_MAX   16
#define FAKE_RESULTS_MAX 64

typedef struct
{
    char              topic[96];
    uint8_t           payload[512];
    int32_t           len;
    mqtt_request_cb_t cb;
    void *            arg;
    err_t             result;
} fake_pub_t;

struct fake_mqtt_client
{
    int32_t connected;
};

static struct fake_mqtt_client g_fake_client;
static fake_pub_t              g_pub[FAKE_PUB_MAX];
static int32_t                 g_pub_count;
static err_t                   g_pub_results[FAKE_RESULTS_MAX];
static int32_t                 g_pub_results_count;
static int32_t                 g_connect_calls;
static int32_t                 g_subscribes;

typedef struct
{
    sys_timeout_handler handler;
    void *              arg;
} fake_timer_t;

static fake_timer_t g_timer[FAKE_TIMER_MAX];
static int32_t      g_timer_count;

mqtt_client_t * mqtt_client_new(void)
{
    return &g_fake_client;
}

err_t mqtt_client_connect(mqtt_client_t * c, const ip_addr_t * ip, u16_t port,
                          mqtt_connection_cb_t cb, void * arg,
                          const struct mqtt_connect_client_info_t * info)
{
    (void)c;
    (void)ip;
    (void)port;
    (void)cb;
    (void)arg;
    (void)info;
    g_connect_calls++;
    return ERR_OK;
}

u8_t mqtt_client_is_connected(mqtt_client_t * c)
{
    return (u8_t)((c != NULL) && c->connected);
}

void mqtt_set_inpub_callback(mqtt_client_t * c, mqtt_incoming_publish_cb_t pub,
                             mqtt_incoming_data_cb_t data, void * arg)
{
    (void)c;
    (void)pub;
    (void)data;
    (void)arg;
}

err_t mqtt_subscribe(mqtt_client_t * c, const char * topic, u8_t qos,
                     mqtt_request_cb_t cb, void * arg)
{
    (void)c;
    (void)topic;
    (void)qos;
    (void)cb;
    (void)arg;
    g_subscribes++;
    return ERR_OK;
}

err_t mqtt_publish(mqtt_client_t * c, const char * topic, const void * payload,
                   u16_t len, u8_t qos, u8_t retain, mqtt_request_cb_t cb,
                   void * arg)
{
    extern int32_t s1_lock_host_depth;
    err_t          result = ERR_OK;
    fake_pub_t *   p_pub;

    (void)c;
    (void)qos;
    (void)retain;
    if (s1_lock_host_depth != 0)
    {
        g_failed++;
        printf("[TEST] LOCK FAIL  mqtt_publish called with the S1 lock held\n");
    }
    if (g_pub_results_count > 0)
    {
        result = g_pub_results[0];
        memmove(g_pub_results, g_pub_results + 1,
                (size_t)(g_pub_results_count - 1) * sizeof(g_pub_results[0]));
        g_pub_results_count--;
    }
    if (g_pub_count < FAKE_PUB_MAX)
    {
        p_pub = &g_pub[g_pub_count++];
        snprintf(p_pub->topic, sizeof(p_pub->topic), "%s", topic);
        p_pub->len = len;
        memcpy(p_pub->payload, payload,
               (len < sizeof(p_pub->payload)) ? len : sizeof(p_pub->payload));
        p_pub->cb     = cb;
        p_pub->arg    = arg;
        p_pub->result = result;
    }
    return result;
}

int ip4addr_aton(const char * cp, ip4_addr_t * addr)
{
    (void)cp;
    addr->addr = 1;
    return 1;
}

void sys_timeout(u32_t msecs, sys_timeout_handler handler, void * arg)
{
    (void)msecs;
    if (g_timer_count < FAKE_TIMER_MAX)
    {
        g_timer[g_timer_count].handler = handler;
        g_timer[g_timer_count].arg     = arg;
        g_timer_count++;
    }
}

/* lwIP removes the first matching timeout only */
void sys_untimeout(sys_timeout_handler handler, void * arg)
{
    int32_t idx;

    for (idx = 0; idx < g_timer_count; idx++)
    {
        if ((g_timer[idx].handler == handler) && (g_timer[idx].arg == arg))
        {
            memmove(&g_timer[idx], &g_timer[idx + 1],
                    (size_t)(g_timer_count - idx - 1) * sizeof(g_timer[0]));
            g_timer_count--;
            return;
        }
    }
}

static int32_t timers_of(sys_timeout_handler handler)
{
    int32_t idx;
    int32_t count = 0;

    for (idx = 0; idx < g_timer_count; idx++)
    {
        count += (g_timer[idx].handler == handler);
    }
    return count;
}

/* Run one pending timeout of handler, as sys_check_timeouts() would. */
static void fire(sys_timeout_handler handler)
{
    sys_untimeout(handler, NULL);
    handler(NULL);
}

static void push_result(err_t result)
{
    if (g_pub_results_count < FAKE_RESULTS_MAX)
    {
        g_pub_results[g_pub_results_count++] = result;
    }
}

static void reset_all(void)
{
    host_kernel_reset();
    s1_lock_host_fail = 0;
    (void)s1_lock_init();
    memset(g_pub, 0, sizeof(g_pub));
    g_pub_count         = 0;
    g_pub_results_count = 0;
    g_timer_count       = 0;
    g_connect_calls     = 0;
    g_subscribes        = 0;
    client              = NULL;
    connecting          = 0;
    g_fake_client.connected = 0;
    (void)subsystem1_init();
}

static void broker_up(void)
{
    mqtt_utk_start();
    g_fake_client.connected = 1;
    connection_cb(client, NULL, MQTT_CONNECT_ACCEPTED);
}

static void broker_down(void)
{
    g_fake_client.connected = 0;
    connection_cb(client, NULL, MQTT_CONNECT_DISCONNECTED);
}

static S1_STATS stats(void)
{
    S1_STATS snapshot;

    (void)s1_get_stats(&snapshot);
    return snapshot;
}

/* ---- tests ------------------------------------------------------------- */

static void test_timer_chains(void)
{
    int32_t cycle;
    bool    b_one = true;

    reset_all();
    for (cycle = 0; cycle < 6; cycle++)
    {
        broker_up();
        b_one = b_one && (timers_of(telemetry_timeout) == 1) &&
                (timers_of(heartbeat_timeout) == 1);
        broker_down();
        broker_down(); /* a second failure report before the retry runs */
        b_one = b_one && (timers_of(retry_timeout) == 1);
    }
    CHECK("S1-M01", b_one,
          "one telemetry, heartbeat and retry chain after 6 reconnects");

    fire(retry_timeout);
    fire(telemetry_timeout);
    fire(heartbeat_timeout);
    CHECK("S1-M02", (timers_of(telemetry_timeout) == 1) &&
          (timers_of(heartbeat_timeout) == 1) &&
          (timers_of(retry_timeout) == 0),
          "the telemetry and heartbeat chains re-arm once while disconnected");
}

static void test_heartbeat_bounds(void)
{
    char       text[MQTT_UTK_HB_MAX];
    int        len;
    int        idx;
    bool       b_digits = true;
    fake_pub_t * p_pub;

    reset_all();
    memset(&g_s1_stats, 0xFF, sizeof(g_s1_stats)); /* every counter max */
    s1_lock_host_fail = 4294967295U;
    memset(text, 'X', sizeof(text));
    len = heartbeat_text(text, (int)sizeof(text), 4294967295U);
    CHECK("S1-M03", (len == (int)strlen(text)) && (len < (int)sizeof(text)) &&
          (strncmp(text, "hb 4294967295 s1 tq=4294967295 ", 31) == 0),
          "max-width heartbeat is NUL-terminated inside its buffer");
    for (idx = 0; idx < len; idx++)
    {
        b_digits = b_digits && (text[idx] != 'X');
    }
    CHECK("S1-M04", b_digits && (len == MQTT_UTK_HB_PREFIX_MAX + 255),
          "max-width heartbeat has every counter in full (269 bytes)");

    memset(text, 'Y', sizeof(text));
    len = heartbeat_text(text, 40, 7U);
    CHECK("S1-M05", (len == 39) && (text[39] == '\0') && (text[40] == 'Y'),
          "a short buffer cuts the heartbeat without writing past it");

    broker_up();
    g_pub_count = 0;
    fire(heartbeat_timeout);
    p_pub = &g_pub[0];
    memset(&g_s1_stats, 0, sizeof(g_s1_stats));
    CHECK("S1-M06", (g_pub_count == 1) &&
          (strcmp(p_pub->topic, MQTT_TOPIC) == 0) &&
          (p_pub->len == (int32_t)strlen((const char *)p_pub->payload)) &&
          (p_pub->len < MQTT_UTK_HB_MAX),
          "heartbeat publish length matches its text and its buffer");

    /* the broker echoes it back on the subscribed topic */
    incoming_publish_cb(NULL, MQTT_TOPIC, (u32_t)p_pub->len);
    incoming_data_cb(NULL, p_pub->payload, (u16_t)100, 0);
    incoming_data_cb(NULL, p_pub->payload + 100, (u16_t)(p_pub->len - 100),
                     MQTT_DATA_FLAG_LAST);
    CHECK("S1-M07", (rx_len == (u16_t)p_pub->len) && (!rx_truncated) &&
          (memcmp(rx_data, p_pub->payload, (size_t)p_pub->len) == 0),
          "max-width heartbeat echo is received whole, not truncated");

    push_result(ERR_MEM);
    fire(heartbeat_timeout);
    CHECK("S1-M08", stats().hb_refused == 1,
          "a refused heartbeat is counted apart from S1 messages");
}

static void test_counters(void)
{
    const uint8_t ack       = 0x05;
    uint8_t       frame[14] = { 0x20 };
    S1_STATS      st;

    reset_all();
    broker_up();
    g_pub_count = 0;
    (void)s1_post(S1_TOPIC_ACK, &ack, 1);
    (void)s1_post(S1_TOPIC_TLM, frame, sizeof(frame));
    push_result(ERR_MEM); /* the ack is refused once */
    fire(telemetry_timeout);
    st = stats();
    CHECK("S1-M09", (g_pub_count == 1) && (st.ref_mem == 1) &&
          (st.evt_drop[S1_EVT_ACK] == 0) && (st.tlm_lost == 0) &&
          (st.pub_evt == 0) && (st.pub_tlm == 0),
          "a refused ack stays in S1 and stops the drain (no busy retry)");

    fire(telemetry_timeout);
    st = stats();
    CHECK("S1-M10", (g_pub_count == 3) &&
          (strcmp(g_pub[1].topic, S1_TOPIC_ROOT S1_TOPIC_ACK) == 0) &&
          (g_pub[1].len == 1) && (g_pub[1].payload[0] == ack) &&
          (strcmp(g_pub[2].topic, S1_TOPIC_ROOT S1_TOPIC_TLM) == 0) &&
          (st.pub_evt == 1) && (st.pub_tlm == 1) &&
          (st.inflight_evt == 1) && (st.inflight_tlm == 1),
          "next drain: the same ack first, then telemetry, both in flight");

    g_pub[1].cb(g_pub[1].arg, ERR_OK);
    g_pub[2].cb(g_pub[2].arg, ERR_TIMEOUT);
    st = stats();
    CHECK("S1-M11", (st.cb_ok == 1) && (st.cb_err == 1) &&
          (st.inflight_evt == 0) && (st.inflight_tlm == 0),
          "lwIP completion callbacks release in-flight counts by kind");

    (void)s1_post(S1_TOPIC_TLM, frame, sizeof(frame));
    fire(telemetry_timeout);
    broker_down();
    (void)s1_post(S1_TOPIC_ACK, &ack, 1);
    fire(telemetry_timeout);
    st = stats();
    CHECK("S1-M12", (st.unconfirmed == 1) && (st.inflight_tlm == 0) &&
          (g_pub_count == 4) && (g_s1_txq.evt_waiting == 1),
          "disconnect: in flight -> unconfirmed; nothing offered offline");

    broker_up();
    g_pub[3].cb(g_pub[3].arg, ERR_OK);
    fire(telemetry_timeout);
    st = stats();
    CHECK("S1-M13", (st.cb_stale == 1) && (st.cb_ok == 1) &&
          (g_pub_count == 5) && (st.inflight_evt == 1),
          "old-connection completion is stale; the kept ack goes out");
}

int main(void)
{
    test_timer_chains();
    test_heartbeat_bounds();
    test_counters();
    printf("[TEST] subsystem1 mqtt_utk host SUMMARY %d passed, %d failed\n",
           (int)g_passed, (int)g_failed);
    return (g_failed == 0) ? 0 : 1;
}

/*** end of file ***/
