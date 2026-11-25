/*
 * Packed binary wire format for Subsystem 1 MQTT telemetry and commands.
 * Plain C with no kernel calls so it can be host-tested.
 *
 * Telemetry is 112 bits, MSB first, in 14 bytes:
 *   version 3 | seq 8 | state 4 | speed L 8s | speed R 8s | distance 16
 *   line 8s | obstacle distance 7 | detected 1 | obstacle state 4 | nav 3
 *   barcodes 4 | pitch 8s | hump now 6 | hump max 6 | fault 16 | wifi 1
 *   paused 1 (formerly a zero pad bit)
 *
 * Command byte: bits 7-6 command, bits 5-0 sequence.
 * An extension uses code 11 followed by one opcode byte.
 * Ack byte:     bits 7-6 result, bits 5-0 echoed command sequence.
 */
#ifndef S1_CODEC_H
#define S1_CODEC_H

#include <stdint.h>

#define S1_PROTO_VERSION     1
#define S1_TLM_BYTES         14
#define S1_SPEED_STEP_MMS    4
#define S1_LINE_STEP         8
#define S1_PITCH_STEP_CDEG   50
#define S1_HUMP_STEP_UM      500
#define S1_OBS_MAX_CM        127
#define S1_BARCODE_MAX_CHARS 10
#define S1_BARCODE_MAX_BYTES ((S1_BARCODE_MAX_CHARS * 6 + 7) / 8)

typedef struct
{
    uint8_t  version;
    uint8_t  seq;
    uint8_t  state;
    int32_t  speed_l_mms;
    int32_t  speed_r_mms;
    int32_t  distance_mm;
    int32_t  line_pos;
    uint32_t obs_dist_cm;
    uint8_t  obs_detected;
    uint8_t  obs_state;
    uint8_t  nav;
    uint8_t  barcodes;
    int32_t  pitch_cdeg;
    int32_t  hump_now_um;
    int32_t  hump_max_um;
    uint32_t fault_code;
    uint8_t  wifi_up;
    uint8_t  paused;
} S1_TLM;

typedef enum
{
    S1_CMD_START    = 0,
    S1_CMD_STOP     = 1,
    S1_CMD_RESET    = 2,
    S1_CMD_RESERVED = 3
} S1_CMD;

typedef enum
{
    S1_EXT_PAUSE  = 0,
    S1_EXT_RESUME = 1
} S1_EXT_CMD;

typedef enum
{
    S1_REQ_INVALID = -1,
    S1_REQ_START,
    S1_REQ_STOP,
    S1_REQ_RESET,
    S1_REQ_PAUSE,
    S1_REQ_RESUME
} S1_REQUEST;

typedef enum
{
    S1_ACK_QUEUED   = 0,
    S1_ACK_BUSY     = 1,
    S1_ACK_REJECTED = 2,
    S1_ACK_RESERVED = 3
} S1_ACK;

void s1_tlm_encode(const S1_TLM * tlm, uint8_t out[S1_TLM_BYTES]);
int  s1_tlm_decode(const uint8_t * in, int len, S1_TLM * tlm);

/* Code 39 data characters, six bits each, MSB first, zero-padded. */
int s1_barcode_encode(const uint8_t * text, int len,
                      uint8_t out[S1_BARCODE_MAX_BYTES]);

uint8_t    s1_cmd_encode(S1_CMD command, uint8_t seq);
S1_CMD     s1_cmd_code(uint8_t byte);
uint8_t    s1_cmd_seq(uint8_t byte);
S1_REQUEST s1_cmd_decode(const uint8_t * payload, int len);

uint8_t s1_ack_encode(S1_ACK result, uint8_t seq);
S1_ACK  s1_ack_result(uint8_t byte);
uint8_t s1_ack_seq(uint8_t byte);

#endif

/*** end of file ***/
