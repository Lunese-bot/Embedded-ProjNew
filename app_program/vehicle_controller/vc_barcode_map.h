/*
 * vc_barcode_map.h
 *
 * The robot's ACTIVE barcode letter map: what each letter A to Z does at
 * the next confirmed junction.  The robot owns it:
 *
 *   - First boot (no valid saved map): the factory default, which is
 *     vc_barcode_config.h exactly as before.
 *   - The dashboard can read the whole map and replace the whole map over
 *     MQTT (binary protocol in subsystem1/s1_cfg.h).  A replace is checked
 *     in full, refused while a mission is active, written to storage and
 *     read back, and only then made active.  A failed or interrupted write
 *     leaves the previous map active and stored.
 *   - A saved map is loaded again at the next boot.
 *
 * Four actions, two bits per letter on the wire and in storage (the wire
 * codes are NOT the firmware enum; see vc_bcmap_wire_from_nav):
 *
 *     0 = go straight   1 = turn left   2 = turn right   3 = U-turn (180)
 *
 * Packing: letter i (A = 0 .. Z = 25) is in byte i / 4, MSB first, at bits
 * 7-6, 5-4, 3-2, 1-0 for i % 4 = 0, 1, 2, 3.  26 letters = 52 bits = 7
 * bytes; the last 4 bits of byte 6 must be zero.
 *
 *     A = byte 0 bits 7-6     D = byte 0 bits 1-0    E = byte 1 bits 7-6
 *     Y = byte 6 bits 7-6     Z = byte 6 bits 5-4
 *
 * The active map always holds one of the four actions per letter.  The
 * factory table's NAV_NONE ("no meaning assigned") is read as NAV_STRAIGHT,
 * which the junction already did with it (vc_sm.c on_junction_stop).
 * NAV_NONE still means "no barcode / unknown text" in vc_barcode_lookup().
 *
 * Pure C, no kernel calls: storage is reached through vc_bcmap_store_t so
 * the same code runs on the robot (vc_bcmap_flash.c) and in host tests.
 *
 * Threads: vc_bcmap_init, vc_bcmap_service and vc_bcmap_lookup_letter run
 * in the Vehicle Controller task only (the single writer).  vc_bcmap_get
 * and vc_bcmap_post_write may be called from the S1 / WiFi task.
 */
#ifndef VC_BARCODE_MAP_H
#define VC_BARCODE_MAP_H

#include <stdbool.h>
#include <stdint.h>
#include "../robot_types.h"

#define VC_BCMAP_LETTERS      26u
#define VC_BCMAP_PACKED_BYTES 7u

/* Wire / storage action codes (two bits).  Fixed forever: the dashboard
   and saved flash records use these numbers. */
#define VC_BCW_STRAIGHT 0u
#define VC_BCW_LEFT     1u
#define VC_BCW_RIGHT    2u
#define VC_BCW_UTURN    3u

/* Result of a read or replace; also the result byte on the wire. */
typedef enum
{
    VC_BCMAP_OK             = 0, /* new map saved, read back and active */
    VC_BCMAP_UNCHANGED      = 1, /* same as the active map: nothing written */
    VC_BCMAP_REJ_ACTIVE     = 2, /* a mission is running: map not changed */
    VC_BCMAP_REJ_STALE      = 3, /* robot's map changed since the dashboard read it */
    VC_BCMAP_REJ_MALFORMED  = 4, /* bad length, opcode or padding bits */
    VC_BCMAP_REJ_STORAGE    = 5, /* write or read-back failed: old map kept */
    VC_BCMAP_BUSY           = 6  /* another save is still being processed */
} vc_bcmap_result_t;

/* Flags in a snapshot (and on the wire). */
#define VC_BCMAP_FLAG_SAVED          0x01u /* loaded from / written to storage (else factory) */
#define VC_BCMAP_FLAG_MISSION_ACTIVE 0x02u /* edits are refused right now */
#define VC_BCMAP_FLAG_FACTORY_EQUAL  0x04u /* the active map equals the factory map */
#define VC_BCMAP_FLAG_NO_STORAGE     0x08u /* no persistent storage on this build */

typedef struct
{
    uint32_t gen;                              /* save counter, 0 = factory */
    uint8_t  packed[VC_BCMAP_PACKED_BYTES];
    uint8_t  flags;                            /* VC_BCMAP_FLAG_* */
} vc_bcmap_snapshot_t;

/* ---- storage ------------------------------------------------------- */

/* Two slots (A/B).  A new map is always written to the slot that does NOT
   hold the active map, so the active record is never erased. */
#define VC_BCMAP_SLOTS        2u
#define VC_BCMAP_RECORD_BYTES 24u

typedef struct
{
    /* Copy len bytes of slot into p_buf.  0 = ok. */
    int32_t (*read)(void * p_ctx, uint32_t slot, uint8_t * p_buf, uint32_t len);
    /* Erase slot and program len bytes.  0 = ok.  May fail half way. */
    int32_t (*write)(void * p_ctx, uint32_t slot, const uint8_t * p_buf,
                     uint32_t len);
    void * p_ctx;
} vc_bcmap_store_t;

/* Record layout (little endian), VC_BCMAP_RECORD_BYTES bytes:
     0..3   magic "BCM1"
     4      layout version (1)
     5      letter count (26)
     6      packed byte count (7)
     7      0
     8..11  save counter (gen); the slot with the higher valid counter wins
     12..18 packed map
     19     0
     20..23 CRC-32 (IEEE, reflected, as zlib) of bytes 0..19 */
void vc_bcmap_record_encode(uint32_t gen, const uint8_t packed[VC_BCMAP_PACKED_BYTES],
                            uint8_t out[VC_BCMAP_RECORD_BYTES]);
/* true and fills gen / packed when the record is complete and valid */
bool vc_bcmap_record_decode(const uint8_t in[VC_BCMAP_RECORD_BYTES], uint32_t * p_gen,
                            uint8_t packed[VC_BCMAP_PACKED_BYTES]);
uint32_t vc_bcmap_crc32(const uint8_t * p_data, uint32_t len);

/* ---- codec --------------------------------------------------------- */

/* NAV_NONE and NAV_STRAIGHT -> 0, LEFT 1, RIGHT 2, U_TURN 3; -1 otherwise */
int32_t vc_bcmap_wire_from_nav(navigation_command_t nav);
/* 0..3 -> NAV_STRAIGHT / LEFT / RIGHT / U_TURN; NAV_NONE for anything else */
navigation_command_t vc_bcmap_nav_from_wire(uint32_t code);

/* 0 = ok, -1 = a value is not one of the four actions */
int32_t vc_bcmap_pack(const navigation_command_t nav[VC_BCMAP_LETTERS],
                      uint8_t out[VC_BCMAP_PACKED_BYTES]);
/* 0 = ok, -1 = padding bits set (then nav is not changed) */
int32_t vc_bcmap_unpack(const uint8_t in[VC_BCMAP_PACKED_BYTES],
                        navigation_command_t nav[VC_BCMAP_LETTERS]);
/* wire code of letter index 0..25 inside a packed map */
uint32_t vc_bcmap_packed_code(const uint8_t packed[VC_BCMAP_PACKED_BYTES],
                              uint32_t letter);

/* The factory map, packed (vc_barcode_config.h, NAV_NONE as straight). */
void vc_bcmap_factory(uint8_t out[VC_BCMAP_PACKED_BYTES]);

/* ---- active map ---------------------------------------------------- */

/* Load the newest valid saved record, else the factory map.  p_store may
   be NULL (no persistent storage): the map then lives in RAM only. */
void vc_bcmap_init(const vc_bcmap_store_t * p_store);

/* VC task: the action for one decoded letter 'A'..'Z' (NAV_NONE for any
   other character). */
navigation_command_t vc_bcmap_lookup_letter(char letter);

/* Any task: a consistent copy of the active map.  false only if a replace
   kept changing it during the copy (retry later). */
bool vc_bcmap_get(vc_bcmap_snapshot_t * p_snap);

/* VC task, every status publish: whether edits must be refused. */
void vc_bcmap_set_mission_active(bool b_active);

/* VC task: validate and apply a whole map (also used directly by tests).
   b_mission_active is the controller's own state check. */
vc_bcmap_result_t vc_bcmap_replace(const uint8_t packed[VC_BCMAP_PACKED_BYTES],
                                   uint16_t expected_gen, bool b_mission_active);

/* ---- one pending save between the S1 task and the VC task ----------- */

/* S1 task: hold a write request for the VC task.  false = one is already
   waiting (answer BUSY). */
bool vc_bcmap_post_write(uint16_t request_id, uint16_t expected_gen,
                         const uint8_t packed[VC_BCMAP_PACKED_BYTES]);
/* S1 task: give the slot back when the VC event could not be posted. */
void vc_bcmap_cancel_write(void);

/* VC task, when the controller has handled VC_EV_CMD_CONFIG: apply the
   pending write (refused when b_allowed is false) and return its result.
   *p_request_id gets the request id; false when nothing was pending. */
bool vc_bcmap_service(bool b_allowed, uint16_t * p_request_id,
                      vc_bcmap_result_t * p_result);

#endif /* VC_BARCODE_MAP_H */
