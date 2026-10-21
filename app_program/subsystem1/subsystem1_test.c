/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *	Record results in testing/subsystem1/TEST_LOG.md.
 */

#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST

LOCAL BOOL str_eq(const char *a, const char *b)
{
	while (*a != '\0' && *a == *b) {
		a++; b++;
	}
	return *a == *b;
}

EXPORT void subsystem1_test(void)
{
	char		topic[S1_BRIDGE_TOPIC_MAX], payload[S1_BRIDGE_LINE_MAX];
	COMMS_STATUS	s;
	UW		before;
	INT		i, len = 0;
	BOOL		found = FALSE;

	TEST_INIT();

	/* S1-U01: a posted line comes out of the bridge with the full topic */
	TEST_CHECK("S1-U01a", s1_post("selftest", "hello") == E_OK, "s1_post queues a line");
	for (i = 0; i < 2 * S1_QUEUE_LINES && !found; i++) {
		len = s1_bridge_pop(topic, sizeof(topic), payload, sizeof(payload));
		found = (len > 0) && str_eq(topic, S1_TOPIC_ROOT "selftest");
	}
	TEST_CHECK("S1-U01b", found && str_eq(payload, "hello"),
		   "bridge returns robot/g10/selftest \"hello\"");

	/* S1-U02: telemetry_task produces a JSON line within 400 ms */
	tk_dly_tsk(400);
	len = s1_bridge_pop(topic, sizeof(topic), payload, sizeof(payload));
	TEST_CHECK("S1-U02", len > 0 && payload[0] == '{', "telemetry JSON queued");

	/* S1-U03: an unknown command is counted and ignored (does not
	   start the mission during the test run) */
	s1_get_status(&s);
	before = s.commands;
	s1_bridge_on_command("hello", 5);
	s1_get_status(&s);
	TEST_CHECK("S1-U03", s.commands == before + 1, "command counted, unknown ignored");

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
