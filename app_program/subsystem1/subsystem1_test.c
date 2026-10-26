/*
 *      subsystem1_test.c
 *      On-target tests for subsystem 1.  Built only with: make TEST=1
 *      Record results in testing/subsystem1/TEST_LOG.md.
 */

#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST

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
    char           topic[S1_BRIDGE_TOPIC_MAX];
    char           payload[S1_BRIDGE_LINE_MAX] = { 0 };
    comms_status_t comms;
    uint32_t       before;
    int32_t        idx;
    int32_t        len     = 0;
    bool           b_found = false;

    TEST_INIT();

    /* S1-U01: a posted line comes out of the bridge with the full topic */
    TEST_CHECK("S1-U01a", E_OK == s1_post("selftest", "hello"),
               "s1_post queues a line");
    for (idx = 0; (idx < (2 * S1_QUEUE_LINES)) && (!b_found); idx++)
    {
        len     = s1_bridge_pop(topic, sizeof(topic), payload, sizeof(payload));
        b_found = (len > 0) && (str_eq(topic, S1_TOPIC_ROOT "selftest"));
    }
    TEST_CHECK("S1-U01b", b_found && (str_eq(payload, "hello")),
               "bridge returns robot/g10/selftest \"hello\"");

    /* S1-U02: telemetry_task produces a JSON line within 400 ms */
    tk_dly_tsk(400);
    len = s1_bridge_pop(topic, sizeof(topic), payload, sizeof(payload));
    TEST_CHECK("S1-U02", (len > 0) && ('{' == payload[0]),
               "telemetry JSON queued");

    /* S1-U03: an unknown command is counted and ignored (does not
       start the mission during the test run) */
    s1_get_status(&comms);
    before = comms.commands;
    s1_bridge_on_command("hello", 5);
    s1_get_status(&comms);
    TEST_CHECK("S1-U03", comms.commands == (before + 1),
               "command counted, unknown ignored");

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */

/*** end of file ***/
