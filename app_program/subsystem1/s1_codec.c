/*
 *	s1_codec.c
 *	Bit packing for the MQTT wire format.  See s1_codec.h for the layout.
 *	Explicit shifts, never C bitfields: bitfield order is compiler defined.
 */
#include "s1_codec.h"

#define W_VERSION	3
#define W_SEQ		8
#define W_STATE		4
#define W_SPEED		8
#define W_DIST		16
#define W_LINE		8
#define W_OBS_DIST	7
#define W_OBS_DET	1
#define W_OBS_DIR	2
#define W_NAV		3
#define W_BARCODES	4
#define W_PITCH		8
#define W_HUMP		6
#define W_FAULT		16
#define W_WIFI		1
#define W_PAUSED	1

typedef struct {
	uint8_t	*buf;
	int	pos;	/* bit position, MSB first */
} BITS;

static void put(BITS *b, uint32_t v, int width)
{
	for (int i = width - 1; i >= 0; i--, b->pos++) {
		if ((v >> i) & 1u) {
			b->buf[b->pos >> 3] |= (uint8_t)(0x80u >> (b->pos & 7));
		}
	}
}

static uint32_t get(BITS *b, int width)
{
	uint32_t v = 0;

	for (int i = 0; i < width; i++, b->pos++) {
		v = (v << 1) | ((b->buf[b->pos >> 3] >> (7 - (b->pos & 7))) & 1u);
	}
	return v;
}

static uint32_t clamp_u(int64_t v, uint32_t max)
{
	return (v < 0) ? 0u : (v > (int64_t)max) ? max : (uint32_t)v;
}

/* signed value scaled by step, stored as two's complement in `width` bits */
static uint32_t pack_s(int32_t v, int step, int width)
{
	int32_t hi = (1 << (width - 1)) - 1;
	int32_t lo = -(1 << (width - 1));
	int32_t q = (v >= 0) ? (v + step / 2) / step : -((-v + step / 2) / step);

	if (q > hi) q = hi;
	if (q < lo) q = lo;
	return (uint32_t)q & ((1u << width) - 1u);
}

static int32_t unpack_s(uint32_t raw, int step, int width)
{
	int32_t v = (int32_t)raw;

	if (raw & (1u << (width - 1))) {
		v -= (1 << width);
	}
	return v * step;
}

static uint32_t pack_u(int32_t v, int step, int width)
{
	return clamp_u(((int64_t)v + step / 2) / step, (1u << width) - 1u);
}

void s1_tlm_encode(const S1_TLM *t, uint8_t out[S1_TLM_BYTES])
{
	BITS b = { out, 0 };

	for (int i = 0; i < S1_TLM_BYTES; i++) out[i] = 0;
	put(&b, t->version & 7u, W_VERSION);
	put(&b, t->seq, W_SEQ);
	put(&b, t->state & 15u, W_STATE);
	put(&b, pack_s(t->speed_l_mms, S1_SPEED_STEP_MMS, W_SPEED), W_SPEED);
	put(&b, pack_s(t->speed_r_mms, S1_SPEED_STEP_MMS, W_SPEED), W_SPEED);
	put(&b, clamp_u(t->distance_mm, 0xFFFFu), W_DIST);
	put(&b, pack_s(t->line_pos, S1_LINE_STEP, W_LINE), W_LINE);
	put(&b, clamp_u(t->obs_dist_cm, S1_OBS_MAX_CM), W_OBS_DIST);
	put(&b, t->obs_detected ? 1u : 0u, W_OBS_DET);
	put(&b, t->obs_dir & 3u, W_OBS_DIR);
	put(&b, t->nav & 7u, W_NAV);
	put(&b, t->barcodes > 15 ? 15u : t->barcodes, W_BARCODES);
	put(&b, pack_s(t->pitch_cdeg, S1_PITCH_STEP_CDEG, W_PITCH), W_PITCH);
	put(&b, pack_u(t->hump_now_um, S1_HUMP_STEP_UM, W_HUMP), W_HUMP);
	put(&b, pack_u(t->hump_max_um, S1_HUMP_STEP_UM, W_HUMP), W_HUMP);
	put(&b, t->fault_code > 0xFFFFu ? 0xFFFFu : t->fault_code, W_FAULT);
	put(&b, t->wifi_up ? 1u : 0u, W_WIFI);
	put(&b, t->paused ? 1u : 0u, W_PAUSED);
}

int s1_tlm_decode(const uint8_t *in, int len, S1_TLM *t)
{
	BITS b = { (uint8_t *)in, 0 };

	if (len != S1_TLM_BYTES) {
		return -1;
	}
	t->version = (uint8_t)get(&b, W_VERSION);
	if (t->version != S1_PROTO_VERSION) {
		return -1;
	}
	t->seq = (uint8_t)get(&b, W_SEQ);
	t->state = (uint8_t)get(&b, W_STATE);
	t->speed_l_mms = unpack_s(get(&b, W_SPEED), S1_SPEED_STEP_MMS, W_SPEED);
	t->speed_r_mms = unpack_s(get(&b, W_SPEED), S1_SPEED_STEP_MMS, W_SPEED);
	t->distance_mm = (int32_t)get(&b, W_DIST);
	t->line_pos = unpack_s(get(&b, W_LINE), S1_LINE_STEP, W_LINE);
	t->obs_dist_cm = get(&b, W_OBS_DIST);
	t->obs_detected = (uint8_t)get(&b, W_OBS_DET);
	t->obs_dir = (uint8_t)get(&b, W_OBS_DIR);
	t->nav = (uint8_t)get(&b, W_NAV);
	t->barcodes = (uint8_t)get(&b, W_BARCODES);
	t->pitch_cdeg = unpack_s(get(&b, W_PITCH), S1_PITCH_STEP_CDEG, W_PITCH);
	t->hump_now_um = (int32_t)get(&b, W_HUMP) * S1_HUMP_STEP_UM;
	t->hump_max_um = (int32_t)get(&b, W_HUMP) * S1_HUMP_STEP_UM;
	t->fault_code = get(&b, W_FAULT);
	t->wifi_up = (uint8_t)get(&b, W_WIFI);
	t->paused = (uint8_t)get(&b, W_PAUSED);
	return 0;
}

uint8_t s1_cmd_encode(S1_CMD code, uint8_t seq)
{
	return (uint8_t)(((unsigned)code & 3u) << 6 | (seq & 0x3Fu));
}
S1_CMD s1_cmd_code(uint8_t b) { return (S1_CMD)(b >> 6); }
uint8_t s1_cmd_seq(uint8_t b) { return b & 0x3Fu; }

uint8_t s1_ack_encode(S1_ACK result, uint8_t seq)
{
	return (uint8_t)(((unsigned)result & 3u) << 6 | (seq & 0x3Fu));
}
S1_ACK s1_ack_result(uint8_t b) { return (S1_ACK)(b >> 6); }
uint8_t s1_ack_seq(uint8_t b) { return b & 0x3Fu; }

void s1_fault_encode(int sev, int src, uint32_t code,
		     uint8_t out[S1_FAULT_BYTES])
{
	BITS b = { out, 0 };

	out[0] = out[1] = out[2] = 0;
	put(&b, (uint32_t)sev & 3u, 2);
	put(&b, (uint32_t)src & 15u, 4);
	put(&b, code > 0xFFFFu ? 0xFFFFu : code, 16);
}

int s1_fault_decode(const uint8_t *in, int len, int *sev, int *src,
		    uint32_t *code)
{
	BITS b = { (uint8_t *)in, 0 };

	if (len != S1_FAULT_BYTES) {
		return -1;
	}
	*sev = (int)get(&b, 2);
	*src = (int)get(&b, 4);
	*code = get(&b, 16);
	return 0;
}
