/*
 * s1_cfg.c
 * Subsystem 1 -- barcode map protocol codec (see s1_cfg.h).
 */
#include "s1_cfg.h"

static uint16_t get_be16(const uint8_t * p)
{
    return (uint16_t)(((unsigned)p[0] << 8) | p[1]);
}

static void put_be16(uint8_t * p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

int s1_cfg_decode_request(const uint8_t * p_in, int len, S1_CFG_REQ * p_req)
{
    unsigned i;

    if (p_req == 0)
        return -1;
    p_req->op = 0u;
    p_req->request_id = 0u;
    p_req->expected_gen = 0u;
    for (i = 0u; i < S1_CFG_MAP_BYTES; i++)
        p_req->map[i] = 0u;
    if (p_in == 0 || len < S1_CFG_READ_LEN || p_in[0] != S1_CFG_MARKER)
        return -1;
    p_req->request_id = get_be16(&p_in[2]);
    if (p_in[1] == S1_CFG_OP_READ || p_in[1] == S1_CFG_OP_WRITE)
        p_req->op = p_in[1];
    if (p_req->request_id == 0u || p_req->op == 0u)
        return -1;
    if (p_req->op == S1_CFG_OP_READ)
        return (len == S1_CFG_READ_LEN) ? 0 : -1;
    if (len != S1_CFG_WRITE_LEN)
        return -1;
    p_req->expected_gen = get_be16(&p_in[4]);
    for (i = 0u; i < S1_CFG_MAP_BYTES; i++)
        p_req->map[i] = p_in[6u + i];
    return 0;
}

int s1_cfg_encode_read(uint16_t request_id, uint8_t out[S1_CFG_READ_LEN])
{
    out[0] = S1_CFG_MARKER;
    out[1] = S1_CFG_OP_READ;
    put_be16(&out[2], request_id);
    return S1_CFG_READ_LEN;
}

int s1_cfg_encode_write(uint16_t request_id, uint16_t expected_gen,
                        const uint8_t map[S1_CFG_MAP_BYTES],
                        uint8_t out[S1_CFG_WRITE_LEN])
{
    unsigned i;

    out[0] = S1_CFG_MARKER;
    out[1] = S1_CFG_OP_WRITE;
    put_be16(&out[2], request_id);
    put_be16(&out[4], expected_gen);
    for (i = 0u; i < S1_CFG_MAP_BYTES; i++)
        out[6u + i] = map[i];
    return S1_CFG_WRITE_LEN;
}

int s1_cfg_encode_reply(uint8_t op, uint16_t request_id, uint8_t result,
                        uint16_t gen, uint8_t flags,
                        const uint8_t map[S1_CFG_MAP_BYTES],
                        uint8_t out[S1_CFG_REPLY_LEN])
{
    unsigned i;

    out[0] = S1_CFG_MARKER;
    out[1] = (uint8_t)(S1_CFG_OP_REPLY | (op & 0x7Fu));
    put_be16(&out[2], request_id);
    out[4] = result;
    put_be16(&out[5], gen);
    out[7] = flags;
    for (i = 0u; i < S1_CFG_MAP_BYTES; i++)
        out[8u + i] = map[i];
    return S1_CFG_REPLY_LEN;
}
