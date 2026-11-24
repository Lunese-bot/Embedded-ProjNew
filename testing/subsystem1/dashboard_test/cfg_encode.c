/*
 * Host helper for the dashboard checks: the barcode map protocol through
 * the firmware's own s1_cfg.c and vc_barcode_map.c, so the dashboard's
 * JavaScript codec is compared with the real C code.
 *
 * stdin, one item per line (hex without spaces):
 *   F                          -> factory map, 7 bytes packed
 *   P c0 .. c25                -> wire codes packed through the firmware
 *                                 enum (nav_from_wire, then vc_bcmap_pack)
 *   U hex7                     -> 26 wire codes, or ERR (padding bits)
 *   R id                       -> READ request
 *   W id gen hex7              -> WRITE request
 *   Y op id result gen flags hex7 -> reply
 *   D hex                      -> s1_cfg_decode_request: "op id gen hex7"
 *                                 or ERR
 */
#include <stdio.h>
#include <string.h>
#include "s1_cfg.h"
#include "vc_barcode_map.h"

static void put_hex(const uint8_t * p, int n)
{
	int i;

	for (i = 0; i < n; i++) printf("%02x", p[i]);
}

static int get_hex(const char * s, uint8_t * out, int max)
{
	int n = 0;
	unsigned v;

	while (s[0] && s[1] && s[0] != '\n' && s[0] != '\r' && n < max) {
		if (sscanf(s, "%2x", &v) != 1) return -1;
		out[n++] = (uint8_t)v;
		s += 2;
	}
	return n;
}

int main(void)
{
	char line[256];

	while (fgets(line, sizeof(line), stdin)) {
		uint8_t m[VC_BCMAP_PACKED_BYTES], out[32];
		char hex[64];
		long v[26];
		int i, n;

		if (line[0] == 'F') {
			vc_bcmap_factory(m);
			put_hex(m, 7); printf("\n");
		} else if (line[0] == 'P') {
			navigation_command_t nav[VC_BCMAP_LETTERS];
			const char * s = line + 1;

			for (i = 0; i < 26; i++) {
				if (sscanf(s, "%ld%n", &v[i], &n) != 1) break;
				s += n;
				nav[i] = vc_bcmap_nav_from_wire((uint32_t)v[i]);
			}
			if (i != 26 || vc_bcmap_pack(nav, m) != 0) { printf("ERR\n"); continue; }
			put_hex(m, 7); printf("\n");
		} else if (line[0] == 'U' && sscanf(line + 1, "%63s", hex) == 1) {
			navigation_command_t nav[VC_BCMAP_LETTERS];

			if (get_hex(hex, m, 7) != 7 || vc_bcmap_unpack(m, nav) != 0) { printf("ERR\n"); continue; }
			for (i = 0; i < 26; i++) printf("%s%ld", i ? " " : "", (long)vc_bcmap_wire_from_nav(nav[i]));
			printf("\n");
		} else if (line[0] == 'R' && sscanf(line + 1, "%ld", &v[0]) == 1) {
			n = s1_cfg_encode_read((uint16_t)v[0], out);
			put_hex(out, n); printf("\n");
		} else if (line[0] == 'W' && sscanf(line + 1, "%ld %ld %63s", &v[0], &v[1], hex) == 3) {
			if (get_hex(hex, m, 7) != 7) { printf("ERR\n"); continue; }
			n = s1_cfg_encode_write((uint16_t)v[0], (uint16_t)v[1], m, out);
			put_hex(out, n); printf("\n");
		} else if (line[0] == 'Y' && sscanf(line + 1, "%ld %ld %ld %ld %ld %63s", &v[0], &v[1], &v[2], &v[3], &v[4], hex) == 6) {
			if (get_hex(hex, m, 7) != 7) { printf("ERR\n"); continue; }
			n = s1_cfg_encode_reply((uint8_t)v[0], (uint16_t)v[1], (uint8_t)v[2], (uint16_t)v[3], (uint8_t)v[4], m, out);
			put_hex(out, n); printf("\n");
		} else if (line[0] == 'D' && sscanf(line + 1, "%63s", hex) == 1) {
			S1_CFG_REQ req;

			n = get_hex(hex, out, (int)sizeof(out));
			if (n < 0 || s1_cfg_decode_request(out, n, &req) != 0) { printf("ERR\n"); continue; }
			printf("%u %u %u ", req.op, req.request_id, req.expected_gen);
			put_hex(req.map, 7); printf("\n");
		} else {
			printf("ERR\n");
		}
		fflush(stdout);
	}
	return 0;
}
