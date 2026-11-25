/*
 * s1_cfg.h
 * Subsystem 1 -- binary MQTT protocol to read and replace the robot's
 * barcode letter map (version 1).  Plain C, no kernel calls: host-tested.
 *
 * Separate topics, so the 14-byte telemetry and the Start / Stop / Reset /
 * Pause / Resume command and ack bytes are unchanged:
 *
 *   dashboard -> robot   robot/g10/cfg     (QoS 0, NEVER retained)
 *   robot -> dashboard   robot/g10/cfgr    (QoS 0, not retained)
 *
 * Request (big endian):
 *   0      0xC1  = protocol marker 0xC0 | version 1
 *   1      opcode: 0x01 READ, 0x02 WRITE (replace the whole map),
 *          0x03 READ_FACTORY (the firmware's factory map, so the dashboard
 *          needs no default table of its own)
 *   2..3   request id, chosen by the dashboard, never 0
 *   READ, READ_FACTORY:  4 bytes in total.
 *   WRITE: 4..5   expected map generation (the gen of the map the edit
 *                 was based on, from the last reply)
 *          6..12  the 26 letters, two bits each (vc_barcode_map.h)
 *          13 bytes in total.
 *
 * Reply (big endian), 15 bytes:
 *   0      0xC1
 *   1      opcode | 0x80  (0x81 read reply, 0x82 write reply, 0x83
 *                          factory reply;
 *                          0x80 when the request opcode was unreadable)
 *   2..3   request id echoed (0 if the request was too short to carry one)
 *   4      result (vc_bcmap_result_t: 0 OK, 1 UNCHANGED, 2 REJ_ACTIVE,
 *          3 REJ_STALE, 4 REJ_MALFORMED, 5 REJ_STORAGE, 6 BUSY)
 *   5..6   generation of the robot's ACTIVE map after the request
 *   7      flags (VC_BCMAP_FLAG_*: 1 saved, 2 mission active,
 *          4 equals factory, 8 no persistent storage)
 *   8..14  the robot's ACTIVE map, packed (0x83: the FACTORY map; gen
 *          and flags still describe the active map)
 *
 * Every reply carries the active map, so the dashboard verifies a save by
 * comparing all 26 entries with what it sent, never by the publish alone.
 * The request id makes an old reply useless for a new save.  The expected
 * generation makes a replayed or late write harmless: a copy of the same
 * map answers UNCHANGED, a write based on an older map answers REJ_STALE.
 *
 * Junction evidence (robot -> dashboard), robot/g10/jx, 6 bytes:
 *   0      0xD1  = marker 0xD0 | version 1
 *   1      1 ASSIGNED (letter looked up, action pending), 2 COMPLETED
 *          (manoeuvre done, line following again), 3 DROPPED (pending
 *          action cancelled inside the run: obstacle, timeout, lift)
 *   2      letter 'A'..'Z', 0 = no single-letter barcode
 *   3      action wire code 0..3 (as in the map)
 *   4..5   generation of the active map when the note was sent
 */
#ifndef S1_CFG_H
#define S1_CFG_H

#include <stdint.h>

#define S1_CFG_MARKER     0xC1u
#define S1_CFG_OP_READ    0x01u
#define S1_CFG_OP_WRITE   0x02u
#define S1_CFG_OP_FACTORY 0x03u
#define S1_CFG_OP_REPLY   0x80u
#define S1_CFG_MAP_BYTES  7u
#define S1_CFG_READ_LEN   4
#define S1_CFG_WRITE_LEN  13
#define S1_CFG_REPLY_LEN  15
#define S1_JX_MARKER      0xD1u
#define S1_JX_LEN         6

#define S1_CFG_TOPIC_SUFFIX "cfg"  /* subscribed: robot/g10/cfg */
#define S1_CFG_REPLY_SUFFIX "cfgr" /* published:  robot/g10/cfgr */

typedef struct
{
    uint8_t  op; /* S1_CFG_OP_READ / _WRITE / _FACTORY, 0 when unreadable */
    uint16_t request_id;   /* 0 when the request was too short */
    uint16_t expected_gen; /* WRITE only */
    uint8_t  map[S1_CFG_MAP_BYTES];
} S1_CFG_REQ;

/* 0 = a well-formed READ or WRITE (map padding is checked by the VC);
   -1 = malformed, *p_req still holds whatever op / id could be read. */
int s1_cfg_decode_request(const uint8_t * p_in, int len, S1_CFG_REQ * p_req);

int s1_cfg_encode_read(uint16_t request_id, uint8_t out[S1_CFG_READ_LEN]);
int s1_cfg_encode_write(uint16_t request_id, uint16_t expected_gen,
                        const uint8_t map[S1_CFG_MAP_BYTES],
                        uint8_t       out[S1_CFG_WRITE_LEN]);

/* Reply: returns S1_CFG_REPLY_LEN. */
int s1_cfg_encode_reply(uint8_t op, uint16_t request_id, uint8_t result,
                        uint16_t gen, uint8_t flags,
                        const uint8_t map[S1_CFG_MAP_BYTES],
                        uint8_t       out[S1_CFG_REPLY_LEN]);

/* Junction note (vc_junction_note_t kind): returns S1_JX_LEN. */
int s1_cfg_encode_junction(uint8_t kind, uint8_t letter, uint8_t code,
                           uint16_t gen, uint8_t out[S1_JX_LEN]);

#endif /* S1_CFG_H */

/*** end of file ***/
