/*
 *      subsystem1_test.c
 *      On-target tests for subsystem 1.  Built only with: make TEST=1
 *      Record results in testing/subsystem1/TEST_LOG.md.
 */

#include "subsystem1.h"
#include "s1_codec.h"
#include "../test_report.h"

#if APP_TEST

/* The byte-sized capture buffer can hold more than S1_QUEUE_LINES messages. */
#define S1_CAPTURE_SCAN_LIMIT \
    (S1_QUEUE_LINES * (S1_LINE_MAX + (int32_t)sizeof(int32_t)))

static bool str_eq(const char * p_lhs, const char * p_rhs)
{
    while ((*p_lhs != '\0') && (*p_lhs == *p_rhs))
    {
        p_lhs++;
        p_rhs++;
    }
    return (*p_lhs == *p_rhs);
}

void subsystem1_test(void)
{
    char           topic[S1_BRIDGE_FULL_TOPIC_MAX];
    uint8_t        payload[S1_BRIDGE_LINE_MAX] = { 0 };
    comms_status_t comms;
    s1_tlm_t       tlm;
    uint8_t        invalid_command = 0xC0;
    const uint8_t  binary_sample[] = { 0x41, 0x00, 0xFF };
    uint32_t       before;
    SYSTIM         started;
    SYSTIM         now;
    int32_t        idx;
    int32_t        len       = 0;
    bool           b_found   = false;
    bool           b_drained = false;

    TEST_INIT();

    /* S1-U01: a posted line comes out of the bridge with the full topic */
    TEST_CHECK("S1-U01a",
               E_OK == s1_post("selftest", binary_sample,
                               (int32_t)sizeof(binary_sample)),
               "s1_post queues a binary payload");
    for (idx = 0; (idx < S1_CAPTURE_SCAN_LIMIT) && (!b_found); idx++)
    {
        len =
            s1_test_bridge_pop(topic, sizeof(topic), payload, sizeof(payload));
        if (len <= 0)
        {
            break;
        }
        b_found = (len > 0) && (str_eq(topic, S1_TOPIC_ROOT "selftest"));
    }
    TEST_CHECK("S1-U01b",
               b_found && (len == (int32_t)sizeof(binary_sample)) &&
                   (payload[0] == binary_sample[0]) &&
                   (payload[1] == binary_sample[1]) &&
                   (payload[2] == binary_sample[2]),
               "bridge preserves payload bytes and exact length");

    /* Discard old copies, then observe a fresh frame without consuming MQTT's
     * queue. */
    for (idx = 0; idx < S1_CAPTURE_SCAN_LIMIT; idx++)
    {
        len =
            s1_test_bridge_pop(topic, sizeof(topic), payload, sizeof(payload));
        if (len <= 0)
        {
            b_drained = (0 == len);
            break;
        }
    }
    b_found = false;
    if (b_drained)
    {
        tk_get_tim(&started);
        do
        {
            len     = s1_test_bridge_pop(topic, sizeof(topic), payload,
                                         sizeof(payload));
            b_found = (len == S1_TLM_BYTES) &&
                      str_eq(topic, S1_TOPIC_ROOT S1_TOPIC_TLM) &&
                      (s1_tlm_decode(payload, len, &tlm) == 0);
            if (!b_found)
            {
                tk_dly_tsk(20);
            }
            tk_get_tim(&now);
        } while ((!b_found) && ((now.lo - started.lo) < 400U));
    }
    else
    {
        tm_printf(
            (uint8_t *)"[TEST] S1-U02 info: capture queue did not drain\n");
    }
    TEST_CHECK("S1-U02", b_found, "14-byte versioned telemetry frame queued");

    /* S1-U03: reserved command code is counted and rejected. */
    s1_get_status(&comms);
    before = comms.commands;
    s1_bridge_on_command(&invalid_command, 1);
    s1_get_status(&comms);
    TEST_CHECK("S1-U03", comms.commands == (before + 1),
               "reserved command counted and rejected without a VC event");

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */

/*** end of file ***/
