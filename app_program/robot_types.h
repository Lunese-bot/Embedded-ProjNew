/*
 *      robot_types.h
 *      Line, barcode and navigation types shared by subsystems 3 and 6
 *      (Design Review 6.2).  Plain C: no kernel types, so the Vehicle
 *      Controller state machine can also be built on a PC (host_test).
 */

#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>

typedef enum
{
    LINE_STATE_INIT,
    LINE_STATE_CALIBRATE,
    LINE_STATE_SEARCH,
    LINE_STATE_FOLLOW,
    LINE_STATE_LOST,
    LINE_STATE_JUNCTION
} line_state_t;

typedef enum
{
    BARCODE_STATE_IDLE,
    BARCODE_STATE_DETECT,
    BARCODE_STATE_READ,
    BARCODE_STATE_DECODE,
    BARCODE_STATE_RETRY,
    BARCODE_STATE_COMPLETE
} barcode_state_t;

typedef enum
{
    NAV_NONE,
    NAV_LEFT,
    NAV_RIGHT,
    NAV_STRAIGHT,
    NAV_U_TURN
} navigation_command_t;

typedef struct
{
    int32_t left;
    int32_t centre;
    int32_t right;
} ir_reading_t;

/* ---------------------------------------------------------------------
 *  Shared packed formats (Tribunal 1 Version C: "define shared packed
 *  message structures, bit positions, field widths, units and
 *  encoding/decoding rules in robot_types.h").
 *
 *  Conversion boundary: RTOS messages between tasks stay NATIVE structs
 *  (rare, safety-related, easy to debug).  Data that is stored or sent
 *  often (telemetry, MQTT) uses the packed forms below.
 * --------------------------------------------------------------------- */

/* S5 obstacle telemetry word, uint16_t (pack/unpack: subsystem5/s5_pack.c)
 *   bit 15      detected  1 = obstacle closer than S5_DETECT_MM (250 mm)
 *   bits 14..11 state     s5_state_t, 10 values (needs 4 bits)
 *   bits 10..0  distance  mm, 0..2047, 0 = no echo, saturates at 2047
 * 11 bits: 1 mm resolution over the 2000 mm sensor range (2^10 = 1024 is
 * too small).  The 8-bit 7-bit-cm form discussed at the tribunal covers
 * the range but not the +/-10 mm accuracy target (S5-U12). */
#define S5_PACK_DIST_BITS   11U
#define S5_PACK_STATE_BITS  4U
#define S5_PACK_DIST_MASK   0x07FFU /* bits 10..0 */
#define S5_PACK_STATE_SHIFT 11U
#define S5_PACK_STATE_MASK  0x0FU
#define S5_PACK_DET_SHIFT   15U
#define S5_PACK_DIST_MAX_MM S5_PACK_DIST_MASK

#endif /* ROBOT_TYPES_H */

/*** end of file ***/
