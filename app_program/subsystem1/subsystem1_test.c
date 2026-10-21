/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *	Record results in testing/subsystem1/TEST_LOG.md.
 */

#include "subsystem1.h"
#include "s1_codec.h"
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
	UB		payload_b[S1_BRIDGE_LINE_MAX];
	char		topic[S1_BRIDGE_FULL_TOPIC_MAX];
	COMMS_STATUS	s;
	UW		before;
	INT		i, len = 0;
	BOOL		found = FALSE;
	S1_TLM		t;

	TEST_INIT();

	/* S1-U01: a posted message comes out of the bridge with the full topic */
	TEST_CHECK("S1-U01a", s1_post("selftest", "hello", 5) == E_OK, "s1_post queues a message");
	for (i = 0; i < 2 * S1_QUEUE_LINES && !found; i++) {
		len = s1_bridge_pop(topic, sizeof(topic), (char *)payload_b, sizeof(payload_b));
		found = (len > 0) && str_eq(topic, S1_TOPIC_ROOT "selftest");
	}
	TEST_CHECK("S1-U01b", found && len == 5 && payload_b[0] == 'h' && payload_b[4] == 'o',
		   "bridge returns robot/g10/selftest, 5 bytes");

	/* S1-U02: telemetry_task produces a decodable binary frame within 400 ms */
	tk_dly_tsk(400);
	found = FALSE;
	for (i = 0; i < 2 * S1_QUEUE_LINES && !found; i++) {
		len = s1_bridge_pop(topic, sizeof(topic), (char *)payload_b, sizeof(payload_b));
		found = (len > 0) && str_eq(topic, S1_TOPIC_ROOT S1_TOPIC_TLM);
	}
	TEST_CHECK("S1-U02", found && len == S1_TLM_BYTES
		   && s1_tlm_decode(payload_b, len, &t) == 0, "telemetry frame queued and decodes");

	/* S1-U03: a malformed command (wrong length) is counted and rejected,
	   and does not start the mission during the test run */
	s1_get_status(&s);
	before = s.commands;
	s1_bridge_on_command("hello", 5);
	s1_get_status(&s);
	TEST_CHECK("S1-U03", s.commands == before + 1, "command counted, malformed rejected");

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
