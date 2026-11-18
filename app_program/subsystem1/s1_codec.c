/*
 * Bit packing for the Subsystem 1 MQTT wire format.
 * Explicit shifts keep the layout independent of compiler bitfield rules.
 */
#include "s1_codec.h"

#define W_VERSION 3
#define W_SEQ 8
#define W_STATE 4
#define W_SPEED 8
#define W_DISTANCE 16
#define W_LINE 8
#define W_OBS_DISTANCE 7
#define W_OBS_DETECTED 1
#define W_OBS_STATE 4
#define W_NAV 3
#define W_BARCODES 4
#define W_PITCH 8
#define W_HUMP 6
#define W_FAULT 16
#define W_WIFI 1

typedef struct {
	uint8_t *data;
	int position;
} S1_BITS;

static void put_bits(S1_BITS *bits, uint32_t value, int width)
{
	int bit;

	for (bit = width - 1; bit >= 0; bit--, bits->position++) {
		if ((value >> bit) & 1u) {
			bits->data[bits->position >> 3] |=
				(uint8_t)(0x80u >> (bits->position & 7));
		}
	}
}

static uint32_t get_bits(S1_BITS *bits, int width)
{
	uint32_t value = 0;
	int bit;

	for (bit = 0; bit < width; bit++, bits->position++) {
		value = (value << 1) |
			((bits->data[bits->position >> 3] >>
			  (7 - (bits->position & 7))) & 1u);
	}
	return value;
}

static uint32_t pack_signed(int32_t value, int step, int width)
{
	int64_t scaled = value;
	int64_t maximum = (1LL << (width - 1)) - 1;
	int64_t minimum = -(1LL << (width - 1));

	if (scaled >= 0) {
		scaled = (scaled + step / 2) / step;
	} else {
		scaled = -((-scaled + step / 2) / step);
	}
	if (scaled > maximum) scaled = maximum;
	if (scaled < minimum) scaled = minimum;
	return (uint32_t)scaled & ((1u << width) - 1u);
}

static int32_t unpack_signed(uint32_t value, int step, int width)
{
	int32_t signed_value = (int32_t)value;

	if (value & (1u << (width - 1))) {
		signed_value -= (1 << width);
	}
	return signed_value * step;
}

static uint32_t pack_unsigned(int32_t value, int step, int width)
{
	uint32_t scaled;
	uint32_t maximum = (1u << width) - 1u;

	if (value <= 0) return 0;
	scaled = ((uint32_t)value + (uint32_t)(step / 2)) / (uint32_t)step;
	return scaled > maximum ? maximum : scaled;
}

void s1_tlm_encode(const S1_TLM *tlm, uint8_t out[S1_TLM_BYTES])
{
	S1_BITS bits = { out, 0 };
	int i;

	for (i = 0; i < S1_TLM_BYTES; i++) out[i] = 0;
	put_bits(&bits, tlm->version & 7u, W_VERSION);
	put_bits(&bits, tlm->seq, W_SEQ);
	put_bits(&bits, tlm->state & 15u, W_STATE);
	put_bits(&bits, pack_signed(tlm->speed_l_mms, S1_SPEED_STEP_MMS,
				    W_SPEED), W_SPEED);
	put_bits(&bits, pack_signed(tlm->speed_r_mms, S1_SPEED_STEP_MMS,
				    W_SPEED), W_SPEED);
	put_bits(&bits, tlm->distance_mm < 0 ? 0u :
		 (tlm->distance_mm > 65535 ? 65535u :
		  (uint32_t)tlm->distance_mm), W_DISTANCE);
	put_bits(&bits, pack_signed(tlm->line_pos, S1_LINE_STEP, W_LINE), W_LINE);
	put_bits(&bits, tlm->obs_dist_cm > S1_OBS_MAX_CM ?
		 S1_OBS_MAX_CM : tlm->obs_dist_cm, W_OBS_DISTANCE);
	put_bits(&bits, tlm->obs_detected ? 1u : 0u, W_OBS_DETECTED);
	put_bits(&bits, tlm->obs_state & 15u, W_OBS_STATE);
	put_bits(&bits, tlm->nav & 7u, W_NAV);
	put_bits(&bits, tlm->barcodes > 15u ? 15u : tlm->barcodes, W_BARCODES);
	put_bits(&bits, pack_signed(tlm->pitch_cdeg, S1_PITCH_STEP_CDEG,
				    W_PITCH), W_PITCH);
	put_bits(&bits, pack_unsigned(tlm->hump_now_um, S1_HUMP_STEP_UM,
				      W_HUMP), W_HUMP);
	put_bits(&bits, pack_unsigned(tlm->hump_max_um, S1_HUMP_STEP_UM,
				      W_HUMP), W_HUMP);
	put_bits(&bits, tlm->fault_code > 0xFFFFu ?
		 0xFFFFu : tlm->fault_code, W_FAULT);
	put_bits(&bits, tlm->wifi_up ? 1u : 0u, W_WIFI);
}

int s1_tlm_decode(const uint8_t *in, int len, S1_TLM *tlm)
{
	S1_BITS bits = { (uint8_t *)in, 0 };

	if (in == 0 || tlm == 0 || len != S1_TLM_BYTES) return -1;
	tlm->version = (uint8_t)get_bits(&bits, W_VERSION);
	if (tlm->version != S1_PROTO_VERSION) return -1;
	tlm->seq = (uint8_t)get_bits(&bits, W_SEQ);
	tlm->state = (uint8_t)get_bits(&bits, W_STATE);
	tlm->speed_l_mms = unpack_signed(get_bits(&bits, W_SPEED),
					 S1_SPEED_STEP_MMS, W_SPEED);
	tlm->speed_r_mms = unpack_signed(get_bits(&bits, W_SPEED),
					 S1_SPEED_STEP_MMS, W_SPEED);
	tlm->distance_mm = (int32_t)get_bits(&bits, W_DISTANCE);
	tlm->line_pos = unpack_signed(get_bits(&bits, W_LINE),
				      S1_LINE_STEP, W_LINE);
	tlm->obs_dist_cm = get_bits(&bits, W_OBS_DISTANCE);
	tlm->obs_detected = (uint8_t)get_bits(&bits, W_OBS_DETECTED);
	tlm->obs_state = (uint8_t)get_bits(&bits, W_OBS_STATE);
	tlm->nav = (uint8_t)get_bits(&bits, W_NAV);
	tlm->barcodes = (uint8_t)get_bits(&bits, W_BARCODES);
	tlm->pitch_cdeg = unpack_signed(get_bits(&bits, W_PITCH),
					S1_PITCH_STEP_CDEG, W_PITCH);
	tlm->hump_now_um = (int32_t)get_bits(&bits, W_HUMP) * S1_HUMP_STEP_UM;
	tlm->hump_max_um = (int32_t)get_bits(&bits, W_HUMP) * S1_HUMP_STEP_UM;
	tlm->fault_code = get_bits(&bits, W_FAULT);
	tlm->wifi_up = (uint8_t)get_bits(&bits, W_WIFI);
	return 0;
}

uint8_t s1_cmd_encode(S1_CMD command, uint8_t seq)
{
	return (uint8_t)(((unsigned)command & 3u) << 6 | (seq & 0x3Fu));
}

S1_CMD s1_cmd_code(uint8_t byte)
{
	return (S1_CMD)(byte >> 6);
}

uint8_t s1_cmd_seq(uint8_t byte)
{
	return byte & 0x3Fu;
}

uint8_t s1_ack_encode(S1_ACK result, uint8_t seq)
{
	return (uint8_t)(((unsigned)result & 3u) << 6 | (seq & 0x3Fu));
}

S1_ACK s1_ack_result(uint8_t byte)
{
	return (S1_ACK)(byte >> 6);
}

uint8_t s1_ack_seq(uint8_t byte)
{
	return byte & 0x3Fu;
}
