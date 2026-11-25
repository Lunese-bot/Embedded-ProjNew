/*
 * s1_cfg.c
 * Subsystem 1 -- barcode map protocol codec (see s1_cfg.h).
 */
#include "s1_cfg.h"

static uint16_t get_be16(const uint8_t * p_val)
{
    return (uint16_t)(((uint32_t)p_val[0] << 8) | p_val[1]);
}

static void put_be16(uint8_t * p_val, uint16_t value)
{
    p_val[0] = (uint8_t)(value >> 8);
    p_val[1] = (uint8_t)value;
}

int32_t s1_cfg_decode_request(const uint8_t * p_in, int32_t len,
                              s1_cfg_req_t * p_req)
{
    uint32_t idx;

    if (0 == p_req)
    {
        return -1;
    }
    p_req->op           = 0u;
    p_req->request_id   = 0u;
    p_req->expected_gen = 0u;
    for (idx = 0u; idx < S1_CFG_MAP_BYTES; idx++)
    {
        p_req->map[idx] = 0u;
    }
    if ((0 == p_in) || (len < S1_CFG_READ_LEN) || (p_in[0] != S1_CFG_MARKER))
    {
        return -1;
    }
    p_req->request_id = get_be16(&p_in[2]);
    if ((S1_CFG_OP_READ == p_in[1]) || (S1_CFG_OP_WRITE == p_in[1]) ||
        (S1_CFG_OP_FACTORY == p_in[1]))
    {
        p_req->op = p_in[1];
    }
    if ((0u == p_req->request_id) || (0u == p_req->op))
    {
        return -1;
    }
    if (p_req->op != S1_CFG_OP_WRITE)
    {
        return (S1_CFG_READ_LEN == len) ? 0 : -1;
    }
    if (len != S1_CFG_WRITE_LEN)
    {
        return -1;
    }
    p_req->expected_gen = get_be16(&p_in[4]);
    for (idx = 0u; idx < S1_CFG_MAP_BYTES; idx++)
    {
        p_req->map[idx] = p_in[6u + idx];
    }
    return 0;
}

int32_t s1_cfg_encode_read(uint16_t request_id,
                           uint8_t  p_output[S1_CFG_READ_LEN])
{
    p_output[0] = S1_CFG_MARKER;
    p_output[1] = S1_CFG_OP_READ;
    put_be16(&p_output[2], request_id);
    return S1_CFG_READ_LEN;
}

int32_t s1_cfg_encode_write(uint16_t request_id, uint16_t expected_gen,
                            const uint8_t p_map[S1_CFG_MAP_BYTES],
                            uint8_t       p_output[S1_CFG_WRITE_LEN])
{
    uint32_t idx;

    p_output[0] = S1_CFG_MARKER;
    p_output[1] = S1_CFG_OP_WRITE;
    put_be16(&p_output[2], request_id);
    put_be16(&p_output[4], expected_gen);
    for (idx = 0u; idx < S1_CFG_MAP_BYTES; idx++)
    {
        p_output[6u + idx] = p_map[idx];
    }
    return S1_CFG_WRITE_LEN;
}

int32_t s1_cfg_encode_reply(uint8_t opcode, uint16_t request_id, uint8_t result,
                            uint16_t gen, uint8_t flags,
                            const uint8_t p_map[S1_CFG_MAP_BYTES],
                            uint8_t       p_output[S1_CFG_REPLY_LEN])
{
    uint32_t idx;

    p_output[0] = S1_CFG_MARKER;
    p_output[1] = (uint8_t)(S1_CFG_OP_REPLY | (opcode & 0x7Fu));
    put_be16(&p_output[2], request_id);
    p_output[4] = result;
    put_be16(&p_output[5], gen);
    p_output[7] = flags;
    for (idx = 0u; idx < S1_CFG_MAP_BYTES; idx++)
    {
        p_output[8u + idx] = p_map[idx];
    }
    return S1_CFG_REPLY_LEN;
}

int32_t s1_cfg_encode_junction(uint8_t kind, uint8_t letter, uint8_t code,
                               uint16_t gen, uint8_t p_output[S1_JX_LEN])
{
    p_output[0] = S1_JX_MARKER;
    p_output[1] = kind;
    p_output[2] = letter;
    p_output[3] = (uint8_t)(code & 3u);
    put_be16(&p_output[4], gen);
    return S1_JX_LEN;
}

/*** end of file ***/
