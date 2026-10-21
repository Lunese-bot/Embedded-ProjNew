/*
 *	s1_codec.h
 *	Packed binary wire format for the MQTT link (replaces JSON).
 *	Plain C, no kernel calls: it is unit-tested on the PC
 *	(testing/subsystem1/host_test).
 *
 *	Every width and scale below is a PLACEHOLDER until the team agrees
 *	the real sensor ranges.  Change the constant, the layout follows.
 *
 *	Telemetry frame, 110 bits MSB first, padded to 14 bytes:
 *	  version 3 | seq 8 | state 4 | speed_l 8s | speed_r 8s | distance 16
 *	  line 8s | obs_dist 7 | obs_det 1 | obs_dir 2 | nav 3 | barcodes 4
 *	  pitch 8s | hump_now 6 | hump_max 6 |  fault 16 | wifi 1 | paused 1 *
 *	Command byte:  bits 7-6 code, bits 5-0 sequence
 *	Ack byte:      bits 7-6 result, bits 5-0 sequence of the command
 *	Fault frame:   3 bytes: sev 2 | src 4 | code 16 | 2 spare
 */
#ifndef S1_CODEC_H
#define S1_CODEC_H

#include <stdint.h>

#define S1_PROTO_VERSION	1	/* 3 bits */

/* Scales (placeholders) */
#define S1_SPEED_STEP_MMS	4	/* 1 LSB of speed = 4 mm/s    */
#define S1_LINE_STEP		8	/* 1 LSB of line position     */
#define S1_PITCH_STEP_CDEG	50	/* 1 LSB of pitch = 0.5 deg   */
#define S1_HUMP_STEP_UM		500	/* 1 LSB of hump = 0.5 mm     */
#define S1_OBS_MAX_CM		127	/* 7 bits; 127 = far / no echo */

#define S1_TLM_BYTES		14
#define S1_FAULT_BYTES		3

/* Values in physical units.  encode() clamps, decode() returns the
   quantised value, so decode(encode(x)) is within one step of x. */
typedef struct {
	uint8_t		version;
	uint8_t		seq;
	uint8_t		state;		/* VC_STATE, 0..15            */
	int32_t		speed_l_mms;
	int32_t		speed_r_mms;
	int32_t		distance_mm;	/* 0..65535, saturates        */
	int32_t		line_pos;	/* -1000..+1000               */
	uint32_t	obs_dist_cm;	/* 0..127                     */
	uint8_t		obs_detected;
	uint8_t		obs_dir;	/* 0..3                       */
	uint8_t		nav;		/* NavigationCommand, 0..7    */
	uint8_t		barcodes;	/* saturates at 15            */
	int32_t		pitch_cdeg;
	int32_t		hump_now_um;
	int32_t		hump_max_um;
	uint32_t	fault_code;	/* saturates at 0xFFFF        */
	uint8_t		wifi_up;
	uint8_t		paused;
} S1_TLM;

/* Remote commands (2 bits) */
typedef enum {
	S1_CMD_START = 0,
	S1_CMD_STOP = 1,
	S1_CMD_PAUSE = 2,
	S1_CMD_RESUME = 3
} S1_CMD;

/* Command results (2 bits) */
typedef enum {
	S1_ACK_SENT = 0,	/* handed to the Vehicle Controller */
	S1_ACK_BUSY = 1,	/* controller queue full, retry     */
	S1_ACK_REJECTED = 2	/* malformed payload                */
} S1_ACK;

void	s1_tlm_encode(const S1_TLM *t, uint8_t out[S1_TLM_BYTES]);

/* Returns 0 on success, -1 if len is wrong or the version is unknown */
int	s1_tlm_decode(const uint8_t *in, int len, S1_TLM *t);

uint8_t	s1_cmd_encode(S1_CMD code, uint8_t seq);
S1_CMD	s1_cmd_code(uint8_t b);
uint8_t	s1_cmd_seq(uint8_t b);

uint8_t	s1_ack_encode(S1_ACK result, uint8_t seq);
S1_ACK	s1_ack_result(uint8_t b);
uint8_t	s1_ack_seq(uint8_t b);

void	s1_fault_encode(int sev, int src, uint32_t code,
			uint8_t out[S1_FAULT_BYTES]);
int	s1_fault_decode(const uint8_t *in, int len, int *sev, int *src,
			uint32_t *code);

#endif /* S1_CODEC_H */
