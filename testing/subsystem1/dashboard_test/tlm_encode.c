/*
 * Host helper for the dashboard checks: encodes telemetry rows, barcode
 * texts and command/ack bytes with the firmware's own s1_codec.c, so the
 * dashboard's JavaScript decoder is compared with the real C encoder.
 *
 * stdin, one item per line:
 *   T seq state vl vr odo line obs_cm det obs_state nav bc pitch_cdeg
 *     hump_um hmax_um fault wifi paused        -> 28 hex digits, then the
 *     fields s1_tlm_decode() reads back from those bytes
 *   B text                                     -> barcode bytes in hex
 *   C cmd seq        (cmd 0 start, 1 stop, 2 reset) -> command byte
 *   A result seq                               -> ack byte
 *   L before ok safety after                    -> "[<latch text>]" as
 *     s1_bridge_format_counters() appends it (s1_latch_resolve/format)
 */
#include <stdio.h>
#include <string.h>
#include "s1_codec.h"
#include "s1_stats.h"

int main(void)
{
	char line[256];

	while (fgets(line, sizeof(line), stdin)) {
		uint8_t out[S1_TLM_BYTES];
		S1_TLM t;
		long v[18];
		int i, n;

		if (line[0] == 'T') {
			if (sscanf(line + 1, "%ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld",
				   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8],
				   &v[9], &v[10], &v[11], &v[12], &v[13], &v[14], &v[15], &v[16]) != 17) {
				printf("ERR\n");
				continue;
			}
			memset(&t, 0, sizeof(t));
			t.version = S1_PROTO_VERSION;
			t.seq = (uint8_t)v[0]; t.state = (uint8_t)v[1];
			t.speed_l_mms = (int32_t)v[2]; t.speed_r_mms = (int32_t)v[3];
			t.distance_mm = (int32_t)v[4]; t.line_pos = (int32_t)v[5];
			t.obs_dist_cm = (uint32_t)v[6]; t.obs_detected = (uint8_t)v[7];
			t.obs_state = (uint8_t)v[8]; t.nav = (uint8_t)v[9];
			t.barcodes = (uint8_t)v[10]; t.pitch_cdeg = (int32_t)v[11];
			t.hump_now_um = (int32_t)v[12]; t.hump_max_um = (int32_t)v[13];
			t.fault_code = (uint32_t)v[14]; t.wifi_up = (uint8_t)v[15];
			t.paused = (uint8_t)v[16];
			s1_tlm_encode(&t, out);
			for (i = 0; i < S1_TLM_BYTES; i++) printf("%02x", out[i]);
			if (s1_tlm_decode(out, S1_TLM_BYTES, &t) != 0) { printf(" ERR\n"); continue; }
			printf(" %u %u %ld %ld %ld %ld %lu %u %u %u %u %ld %ld %ld %lu %u %u\n",
			       t.seq, t.state, (long)t.speed_l_mms, (long)t.speed_r_mms,
			       (long)t.distance_mm, (long)t.line_pos, (unsigned long)t.obs_dist_cm,
			       t.obs_detected, t.obs_state, t.nav, t.barcodes, (long)t.pitch_cdeg,
			       (long)t.hump_now_um, (long)t.hump_max_um,
			       (unsigned long)t.fault_code, t.wifi_up, t.paused);
		} else if (line[0] == 'B') {
			uint8_t b[S1_BARCODE_MAX_BYTES];
			char *text = line + 2;

			text[strcspn(text, "\r\n")] = '\0';
			n = s1_barcode_encode((const uint8_t *)text, (int)strlen(text), b);
			if (n < 0) { printf("ERR\n"); continue; }
			for (i = 0; i < n; i++) printf("%02x", b[i]);
			printf("\n");
		} else if (line[0] == 'C' && sscanf(line + 1, "%ld %ld", &v[0], &v[1]) == 2) {
			printf("%02x\n", s1_cmd_encode((S1_CMD)v[0], (uint8_t)v[1]));
		} else if (line[0] == 'A' && sscanf(line + 1, "%ld %ld", &v[0], &v[1]) == 2) {
			printf("%02x\n", s1_ack_encode((S1_ACK)v[0], (uint8_t)v[1]));
		} else if (line[0] == 'L' && sscanf(line + 1, "%ld %ld %ld %ld", &v[0], &v[1], &v[2], &v[3]) == 4) {
			S1_LATCH latch;
			char text[S1_LATCH_TEXT_MAX + 1];

			s1_latch_resolve(&latch, (uint32_t)v[0], (int32_t)v[1], (uint32_t)v[2], (uint32_t)v[3]);
			(void)s1_latch_format(&latch, text, (int32_t)sizeof(text));
			printf("[%s]\n", text);
		} else {
			printf("ERR\n");
		}
	}
	return 0;
}
