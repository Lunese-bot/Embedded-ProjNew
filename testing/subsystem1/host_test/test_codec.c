#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "s1_codec.h"

static int passed;
static int failed;

#define CHECK(id, condition, description) \
	do { \
		if (condition) { \
			passed++; \
			printf("[TEST] %s PASS  %s\n", id, description); \
		} else { \
			failed++; \
			printf("[TEST] %s FAIL  %s (line %d)\n", \
			       id, description, __LINE__); \
		} \
	} while (0)

static S1_TLM sample_telemetry(void)
{
	S1_TLM tlm = {0};

	tlm.version = S1_PROTO_VERSION;
	tlm.seq = 200;
	tlm.state = 9;
	tlm.speed_l_mms = 200;
	tlm.speed_r_mms = -100;
	tlm.distance_mm = 12345;
	tlm.line_pos = -504;
	tlm.obs_dist_cm = 45;
	tlm.obs_detected = 1;
	tlm.obs_state = 7;
	tlm.nav = 4;
	tlm.barcodes = 7;
	tlm.pitch_cdeg = -300;
	tlm.hump_now_um = 2000;
	tlm.hump_max_um = 5500;
	tlm.fault_code = 0x3001;
	tlm.wifi_up = 1;
	return tlm;
}

int main(void)
{
	static const uint8_t expected_frame[S1_TLM_BYTES] = {
		0x39, 0x12, 0x65, 0xce, 0x60, 0x73, 0x82,
		0xb6, 0xf1, 0xfe, 0x84, 0x2c, 0xc0, 0x06
	};
	uint8_t frame[S1_TLM_BYTES];
	S1_TLM input = sample_telemetry();
	S1_TLM output;
	int valid;
	int command;
	int seq;
	int result;

	s1_tlm_encode(&input, frame);
	CHECK("S1-H12", memcmp(frame, expected_frame, sizeof(frame)) == 0,
	      "encoded telemetry matches the canonical MSB-first wire vector");
	CHECK("S1-H01", s1_tlm_decode(frame, sizeof(frame), &output) == 0,
	      "decode accepts a complete supported frame");
	CHECK("S1-H02", output.seq == 200 && output.state == 9 &&
	      output.distance_mm == 12345 && output.obs_dist_cm == 45 &&
	      output.obs_detected == 1 && output.obs_state == 7 &&
	      output.nav == 4 && output.barcodes == 7 &&
	      output.fault_code == 0x3001 && output.wifi_up == 1,
	      "unsigned telemetry fields round-trip");
	CHECK("S1-H03", output.speed_l_mms == 200 &&
	      output.speed_r_mms == -100 && output.line_pos == -504 &&
	      output.pitch_cdeg == -300 && output.hump_now_um == 2000 &&
	      output.hump_max_um == 5500,
	      "scaled signed and hump fields round-trip");

	input.speed_l_mms = 203;
	input.line_pos = 13;
	input.pitch_cdeg = 120;
	input.hump_now_um = 1300;
	s1_tlm_encode(&input, frame);
	(void)s1_tlm_decode(frame, sizeof(frame), &output);
	CHECK("S1-H04", abs(output.speed_l_mms - 203) <= S1_SPEED_STEP_MMS / 2 &&
	      abs(output.line_pos - 13) <= S1_LINE_STEP / 2 &&
	      abs(output.pitch_cdeg - 120) <= S1_PITCH_STEP_CDEG / 2 &&
	      abs(output.hump_now_um - 1300) <= S1_HUMP_STEP_UM / 2,
	      "quantization error is at most half a step");

	memset(&input, 0, sizeof(input));
	input.version = S1_PROTO_VERSION;
	input.speed_l_mms = 100000;
	input.speed_r_mms = -100000;
	input.distance_mm = 100000;
	input.line_pos = 5000;
	input.obs_dist_cm = 900;
	input.barcodes = 99;
	input.pitch_cdeg = -99999;
	input.hump_now_um = 999999;
	input.hump_max_um = -5;
	input.fault_code = 0xABCDEF;
	s1_tlm_encode(&input, frame);
	(void)s1_tlm_decode(frame, sizeof(frame), &output);
	CHECK("S1-H05", output.speed_l_mms == 127 * S1_SPEED_STEP_MMS &&
	      output.speed_r_mms == -128 * S1_SPEED_STEP_MMS &&
	      output.distance_mm == 65535 &&
	      output.line_pos == 127 * S1_LINE_STEP &&
	      output.obs_dist_cm == S1_OBS_MAX_CM && output.barcodes == 15 &&
	      output.pitch_cdeg == -128 * S1_PITCH_STEP_CDEG &&
	      output.hump_now_um == 63 * S1_HUMP_STEP_UM &&
	      output.hump_max_um == 0 && output.fault_code == 0xFFFF,
	      "out-of-range fields saturate without wrapping");
	CHECK("S1-H06", s1_tlm_decode(frame, S1_TLM_BYTES - 1, &output) == -1 &&
	      s1_tlm_decode(frame, S1_TLM_BYTES + 1, &output) == -1,
	      "wrong telemetry lengths are rejected");

	frame[0] = (uint8_t)((frame[0] & 0x1Fu) | (5u << 5));
	CHECK("S1-H07", s1_tlm_decode(frame, S1_TLM_BYTES, &output) == -1,
	      "unknown telemetry version is rejected");
	CHECK("S1-H08", s1_tlm_decode(NULL, S1_TLM_BYTES, &output) == -1 &&
	      s1_tlm_decode(frame, S1_TLM_BYTES, NULL) == -1,
	      "null telemetry pointers are rejected");

	valid = 1;
	for (command = S1_CMD_START; command <= S1_CMD_RESET; command++) {
		for (seq = 0; seq < 64; seq++) {
			uint8_t byte = s1_cmd_encode((S1_CMD)command, (uint8_t)seq);

			if (s1_cmd_code(byte) != (S1_CMD)command ||
			    s1_cmd_seq(byte) != seq) {
				valid = 0;
			}
		}
	}
	CHECK("S1-H09", valid, "start, stop, and reset round-trip for every sequence");
	CHECK("S1-H10", s1_cmd_encode(S1_CMD_START, 1) == 0x01 &&
	      s1_cmd_encode(S1_CMD_STOP, 2) == 0x42 &&
	      s1_cmd_encode(S1_CMD_RESET, 3) == 0x83 &&
	      s1_cmd_code(0xC4) == S1_CMD_RESERVED,
	      "command examples and reserved code match the protocol");

	valid = 1;
	for (result = S1_ACK_QUEUED; result <= S1_ACK_REJECTED; result++) {
		for (seq = 0; seq < 64; seq++) {
			uint8_t byte = s1_ack_encode((S1_ACK)result, (uint8_t)seq);

			if (s1_ack_result(byte) != (S1_ACK)result ||
			    s1_ack_seq(byte) != seq) {
				valid = 0;
			}
		}
	}
	CHECK("S1-H11", valid, "ack results and echoed sequences round-trip");

	printf("[TEST] subsystem1 host SUMMARY %d passed, %d failed\n",
	       passed, failed);
	return failed ? 1 : 0;
}
