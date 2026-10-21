/* Round-trip, boundary and rejection checks for s1_codec.c */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "s1_codec.h"

static int passed, failed;

#define CHECK(id, cond, desc)						\
	do {								\
		if (cond) { passed++; printf("[TEST] %s PASS  %s\n", id, desc); } \
		else { failed++; printf("[TEST] %s FAIL  %s   (line %d)\n",	\
					id, desc, __LINE__); }	\
	} while (0)

static S1_TLM sample(void)
{
	S1_TLM t = {0};
	t.version = S1_PROTO_VERSION; t.seq = 200; t.state = 9;
	t.speed_l_mms = 200; t.speed_r_mms = -100; t.distance_mm = 12345;
	t.line_pos = -504; t.obs_dist_cm = 45; t.obs_detected = 1; t.obs_dir = 2;
	t.nav = 4; t.barcodes = 7; t.pitch_cdeg = -300;
	t.hump_now_um = 2000; t.hump_max_um = 5500; t.fault_code = 0x3001;
	t.wifi_up = 1; t.paused = 1;
	return t;
}

int main(void)
{
	uint8_t b[S1_TLM_BYTES];
	S1_TLM in = sample(), out;

	s1_tlm_encode(&in, b);
	CHECK("S1-H01", s1_tlm_decode(b, S1_TLM_BYTES, &out) == 0, "decode succeeds");
	CHECK("S1-H02", out.seq == 200 && out.state == 9 && out.distance_mm == 12345
	      && out.obs_dist_cm == 45 && out.obs_detected == 1 && out.obs_dir == 2
	      && out.nav == 4 && out.barcodes == 7 && out.fault_code == 0x3001
	      && out.wifi_up == 1 && out.paused == 1, "exact fields round-trip");
	CHECK("S1-H03", out.speed_l_mms == 200 && out.speed_r_mms == -100
	      && out.line_pos == -504 && out.pitch_cdeg == -300
	      && out.hump_now_um == 2000 && out.hump_max_um == 5500,
	      "scaled fields round-trip on the step grid");

	/* quantisation error stays within half a step */
	in.speed_l_mms = 203; in.line_pos = 13; in.pitch_cdeg = 120; in.hump_now_um = 1300;
	s1_tlm_encode(&in, b); s1_tlm_decode(b, S1_TLM_BYTES, &out);
	CHECK("S1-H04", abs(out.speed_l_mms - 203) <= S1_SPEED_STEP_MMS / 2
	      && abs(out.line_pos - 13) <= S1_LINE_STEP / 2
	      && abs(out.pitch_cdeg - 120) <= S1_PITCH_STEP_CDEG / 2
	      && abs(out.hump_now_um - 1300) <= S1_HUMP_STEP_UM / 2,
	      "quantisation error within half a step");

	/* saturation */
	memset(&in, 0, sizeof in); in.version = S1_PROTO_VERSION;
	in.speed_l_mms = 100000; in.speed_r_mms = -100000; in.distance_mm = 999999;
	in.line_pos = 5000; in.obs_dist_cm = 900; in.barcodes = 99;
	in.pitch_cdeg = -99999; in.hump_now_um = 999999; in.hump_max_um = -5;
	in.fault_code = 0xABCDEF;
	s1_tlm_encode(&in, b); s1_tlm_decode(b, S1_TLM_BYTES, &out);
	CHECK("S1-H05", out.speed_l_mms == 127 * S1_SPEED_STEP_MMS
	      && out.speed_r_mms == -128 * S1_SPEED_STEP_MMS
	      && out.distance_mm == 65535 && out.line_pos == 127 * S1_LINE_STEP
	      && out.obs_dist_cm == 127 && out.barcodes == 15
	      && out.pitch_cdeg == -128 * S1_PITCH_STEP_CDEG
	      && out.hump_now_um == 63 * S1_HUMP_STEP_UM && out.hump_max_um == 0
	      && out.fault_code == 0xFFFF, "out-of-range values saturate");

	/* frame checks */
	CHECK("S1-H06", s1_tlm_decode(b, S1_TLM_BYTES - 1, &out) == -1
	      && s1_tlm_decode(b, S1_TLM_BYTES + 1, &out) == -1, "wrong length rejected");
	b[0] = (uint8_t)((b[0] & 0x1F) | (5u << 5));
	CHECK("S1-H07", s1_tlm_decode(b, S1_TLM_BYTES, &out) == -1, "unknown version rejected");

	/* commands and acks */
	int ok = 1;
	for (int c = 0; c < 4; c++) {
		for (int s = 0; s < 64; s++) {
			uint8_t x = s1_cmd_encode((S1_CMD)c, (uint8_t)s);
			uint8_t y = s1_ack_encode((S1_ACK)(c % 3), (uint8_t)s);
			if (s1_cmd_code(x) != (S1_CMD)c || s1_cmd_seq(x) != s
			    || s1_ack_result(y) != (S1_ACK)(c % 3) || s1_ack_seq(y) != s) ok = 0;
		}
	}
	CHECK("S1-H09", ok, "command and ack bytes round-trip (all codes, all seqs)");
	CHECK("S1-H10", s1_cmd_encode(S1_CMD_PAUSE, 5) == 0x85, "pause seq 5 = 0x85");

	/* fault frame */
	uint8_t f[S1_FAULT_BYTES]; int sev, src; uint32_t code;
	s1_fault_encode(2, 5, 0x3001, f);
	CHECK("S1-H11", s1_fault_decode(f, 3, &sev, &src, &code) == 0
	      && sev == 2 && src == 5 && code == 0x3001, "fault frame round-trip");
	CHECK("S1-H12", s1_fault_decode(f, 2, &sev, &src, &code) == -1, "short fault frame rejected");

	printf("[TEST] subsystem1 host SUMMARY %d passed, %d failed\n", passed, failed);
	return failed ? 1 : 0;
}
