/*
 * Bit packing for the Subsystem 1 MQTT wire format.
 * Explicit shifts keep the layout independent of compiler bitfield rules.
 */
#include "s1_codec.h"

#define W_VERSION      3
#define W_SEQ          8
#define W_STATE        4
#define W_SPEED        8
#define W_DISTANCE     16
#define W_LINE         8
#define W_OBS_DISTANCE 7
#define W_OBS_DETECTED 1
#define W_OBS_STATE    4
#define W_NAV          3
#define W_BARCODES     4
#define W_PITCH        8
#define W_HUMP         6
#define W_FAULT        16
#define W_WIFI         1
#define W_PAUSED       1

typedef struct
{
    uint8_t * data;
    int32_t   position;
} s1_bits_t;

static void put_bits(s1_bits_t * p_bits, uint32_t value, int32_t width)
{
    int32_t bit;

    for (bit = width - 1; bit >= 0; bit--, p_bits->position++)
    {
        if ((value >> bit) & 1u)
        {
            p_bits->data[p_bits->position >> 3] |=
                (uint8_t)(0x80u >> (p_bits->position & 7));
        }
    }
}

static uint32_t get_bits(s1_bits_t * p_bits, int32_t width)
{
    uint32_t value = 0;
    int32_t  bit;

    for (bit = 0; bit < width; bit++, p_bits->position++)
    {
        value = (value << 1) | ((p_bits->data[p_bits->position >> 3] >>
                                 (7 - (p_bits->position & 7))) &
                                1u);
    }
    return value;
}

static uint32_t pack_signed(int32_t value, int32_t step, int32_t width)
{
    int64_t scaled  = value;
    int64_t maximum = (1LL << (width - 1)) - 1;
    int64_t minimum = -(1LL << (width - 1));

    if (scaled >= 0)
    {
        scaled = (scaled + step / 2) / step;
    }
    else
    {
        scaled = -((-scaled + step / 2) / step);
    }
    if (scaled > maximum)
    {
        scaled = maximum;
    }
    if (scaled < minimum)
    {
        scaled = minimum;
    }
    return (uint32_t)scaled & ((1u << width) - 1u);
}

static int32_t unpack_signed(uint32_t value, int32_t step, int32_t width)
{
    int32_t signed_value = (int32_t)value;

    if (value & (1u << (width - 1)))
    {
        signed_value -= (1 << width);
    }
    return signed_value * step;
}

static uint32_t pack_unsigned(int32_t value, int32_t step, int32_t width)
{
    uint32_t scaled;
    uint32_t maximum = (1u << width) - 1u;

    if (value <= 0)
    {
        return 0;
    }
    scaled = ((uint32_t)value + (uint32_t)(step / 2)) / (uint32_t)step;
    return scaled > maximum ? maximum : scaled;
}

void s1_tlm_encode(const s1_tlm_t * p_tlm, uint8_t p_output[S1_TLM_BYTES])
{
    s1_bits_t bits = { p_output, 0 };
    int32_t   idx;

    for (idx = 0; idx < S1_TLM_BYTES; idx++)
    {
        p_output[idx] = 0;
    }
    put_bits(&bits, p_tlm->version & 7u, W_VERSION);
    put_bits(&bits, p_tlm->seq, W_SEQ);
    put_bits(&bits, p_tlm->state & 15u, W_STATE);
    put_bits(&bits, pack_signed(p_tlm->speed_l_mms, S1_SPEED_STEP_MMS, W_SPEED),
             W_SPEED);
    put_bits(&bits, pack_signed(p_tlm->speed_r_mms, S1_SPEED_STEP_MMS, W_SPEED),
             W_SPEED);
    put_bits(&bits,
             p_tlm->distance_mm < 0
                 ? 0u
                 : (p_tlm->distance_mm > 65535 ? 65535u
                                               : (uint32_t)p_tlm->distance_mm),
             W_DISTANCE);
    put_bits(&bits, pack_signed(p_tlm->line_pos, S1_LINE_STEP, W_LINE), W_LINE);
    put_bits(&bits,
             p_tlm->obs_dist_cm > S1_OBS_MAX_CM ? S1_OBS_MAX_CM
                                                : p_tlm->obs_dist_cm,
             W_OBS_DISTANCE);
    put_bits(&bits, p_tlm->obs_detected ? 1u : 0u, W_OBS_DETECTED);
    put_bits(&bits, p_tlm->obs_state & 15u, W_OBS_STATE);
    put_bits(&bits, p_tlm->nav & 7u, W_NAV);
    put_bits(&bits, p_tlm->barcodes > 15u ? 15u : p_tlm->barcodes, W_BARCODES);
    put_bits(&bits, pack_signed(p_tlm->pitch_cdeg, S1_PITCH_STEP_CDEG, W_PITCH),
             W_PITCH);
    put_bits(&bits, pack_unsigned(p_tlm->hump_now_um, S1_HUMP_STEP_UM, W_HUMP),
             W_HUMP);
    put_bits(&bits, pack_unsigned(p_tlm->hump_max_um, S1_HUMP_STEP_UM, W_HUMP),
             W_HUMP);
    put_bits(&bits, p_tlm->fault_code > 0xFFFFu ? 0xFFFFu : p_tlm->fault_code,
             W_FAULT);
    put_bits(&bits, p_tlm->wifi_up ? 1u : 0u, W_WIFI);
    put_bits(&bits, p_tlm->paused ? 1u : 0u, W_PAUSED);
}

int32_t s1_tlm_decode(const uint8_t * p_input, int32_t len, s1_tlm_t * p_tlm)
{
    s1_bits_t bits = { (uint8_t *)p_input, 0 };

    if (p_input == 0 || p_tlm == 0 || len != S1_TLM_BYTES)
    {
        return -1;
    }
    p_tlm->version = (uint8_t)get_bits(&bits, W_VERSION);
    if (p_tlm->version != S1_PROTO_VERSION)
    {
        return -1;
    }
    p_tlm->seq   = (uint8_t)get_bits(&bits, W_SEQ);
    p_tlm->state = (uint8_t)get_bits(&bits, W_STATE);
    p_tlm->speed_l_mms =
        unpack_signed(get_bits(&bits, W_SPEED), S1_SPEED_STEP_MMS, W_SPEED);
    p_tlm->speed_r_mms =
        unpack_signed(get_bits(&bits, W_SPEED), S1_SPEED_STEP_MMS, W_SPEED);
    p_tlm->distance_mm = (int32_t)get_bits(&bits, W_DISTANCE);
    p_tlm->line_pos =
        unpack_signed(get_bits(&bits, W_LINE), S1_LINE_STEP, W_LINE);
    p_tlm->obs_dist_cm  = get_bits(&bits, W_OBS_DISTANCE);
    p_tlm->obs_detected = (uint8_t)get_bits(&bits, W_OBS_DETECTED);
    p_tlm->obs_state    = (uint8_t)get_bits(&bits, W_OBS_STATE);
    p_tlm->nav          = (uint8_t)get_bits(&bits, W_NAV);
    p_tlm->barcodes     = (uint8_t)get_bits(&bits, W_BARCODES);
    p_tlm->pitch_cdeg =
        unpack_signed(get_bits(&bits, W_PITCH), S1_PITCH_STEP_CDEG, W_PITCH);
    p_tlm->hump_now_um = (int32_t)get_bits(&bits, W_HUMP) * S1_HUMP_STEP_UM;
    p_tlm->hump_max_um = (int32_t)get_bits(&bits, W_HUMP) * S1_HUMP_STEP_UM;
    p_tlm->fault_code  = get_bits(&bits, W_FAULT);
    p_tlm->wifi_up     = (uint8_t)get_bits(&bits, W_WIFI);
    p_tlm->paused      = (uint8_t)get_bits(&bits, W_PAUSED);
    return 0;
}

int32_t s1_barcode_encode(const uint8_t * p_text, int32_t len,
                          uint8_t p_output[S1_BARCODE_MAX_BYTES])
{
    static const char alphabet[] =
        "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";
    s1_bits_t bits = { p_output, 0 };
    int32_t   idx;

    if (p_text == 0 || p_output == 0 || len < 1 || len > S1_BARCODE_MAX_CHARS)
    {
        return -1;
    }
    for (idx = 0; idx < S1_BARCODE_MAX_BYTES; idx++)
    {
        p_output[idx] = 0;
    }
    for (idx = 0; idx < len; idx++)
    {
        int32_t code = 0;

        while (alphabet[code] != '\0' && (uint8_t)alphabet[code] != p_text[idx])
        {
            code++;
        }
        if (alphabet[code] == '\0')
        {
            return -1;
        }
        put_bits(&bits, (uint32_t)(code + 1), 6);
    }
    return (bits.position + 7) / 8;
}

uint8_t s1_cmd_encode(s1_cmd_t command, uint8_t seq)
{
    return (uint8_t)(((uint32_t)command & 3u) << 6 | (seq & 0x3Fu));
}

s1_cmd_t s1_cmd_code(uint8_t byte)
{
    return (s1_cmd_t)(byte >> 6);
}

uint8_t s1_cmd_seq(uint8_t byte)
{
    return byte & 0x3Fu;
}

s1_request_t s1_cmd_decode(const uint8_t * p_payload, int32_t len)
{
    s1_cmd_t code;

    if (p_payload == 0 || (len != 1 && len != 2))
    {
        return S1_REQ_INVALID;
    }
    code = s1_cmd_code(p_payload[0]);
    if (len == 1)
    {
        switch (code)
        {
            case S1_CMD_START:
                return S1_REQ_START;
            case S1_CMD_STOP:
                return S1_REQ_STOP;
            case S1_CMD_RESET:
                return S1_REQ_RESET;
            default:
                return S1_REQ_INVALID;
        }
    }
    if (code != S1_CMD_RESERVED)
    {
        return S1_REQ_INVALID;
    }
    switch (p_payload[1])
    {
        case S1_EXT_PAUSE:
            return S1_REQ_PAUSE;
        case S1_EXT_RESUME:
            return S1_REQ_RESUME;
        default:
            return S1_REQ_INVALID;
    }
}

uint8_t s1_ack_encode(s1_ack_t result, uint8_t seq)
{
    return (uint8_t)(((uint32_t)result & 3u) << 6 | (seq & 0x3Fu));
}

s1_ack_t s1_ack_result(uint8_t byte)
{
    return (s1_ack_t)(byte >> 6);
}

uint8_t s1_ack_seq(uint8_t byte)
{
    return byte & 0x3Fu;
}

/*** end of file ***/
