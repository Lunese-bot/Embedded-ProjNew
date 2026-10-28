/*
 *      s5_pack.h
 *      Subsystem 5 -- minimum-bit telemetry word (Week 8 tribunal,
 *      "min-bit representation").
 *
 *      The full s5_status_t is 40 bytes.  For telemetry sent every 200 ms
 *      only three values are needed, and they fit in ONE 16-bit word:
 *
 *        bit 15     14 ... 11      10 ............... 0
 *          [det]    [  state  ]    [   distance_mm   ]
 *          1 bit     4 bits            11 bits
 *
 *        distance_mm  0..2047 mm (the sensor range is 2000 mm), 0 = no
 *                     echo.  11 bits is the minimum for 1 mm resolution
 *                     over 2000 mm: 2^10 = 1024 is too small.
 *        state        s5_state_t, 10 values -> 4 bits (2^4 = 16).
 *        det          1 = obstacle closer than S5_DETECT_MM right now.
 *
 *      Plain C with no kernel calls, so it is tested on the PC as well
 *      (testing/subsystem5/host_test).  The receiver (telemetry,
 *      dashboard) decodes with the same masks and shifts.
 */

#ifndef S5_PACK_H
#define S5_PACK_H

#include <stdint.h>
#include <stdbool.h>

/* Bit layout.  Kept inside subsystem 5; a receiver (telemetry, dashboard)
   includes this header to decode.  RTOS messages between tasks stay
   native structs; only telemetry/MQTT data uses this packed word. */
#define S5_PACK_DIST_BITS   11U
#define S5_PACK_STATE_BITS  4U
#define S5_PACK_DIST_MASK   0x07FFU /* bits 10..0 */
#define S5_PACK_STATE_SHIFT 11U
#define S5_PACK_STATE_MASK  0x0FU
#define S5_PACK_DET_SHIFT   15U
#define S5_PACK_DIST_MAX_MM S5_PACK_DIST_MASK

/* Pack the three values.  Distance above 2047 mm saturates at 2047;
   a state above 15 is masked (cannot happen: s5_state_t has 10). */
extern uint16_t s5_pack_status(uint16_t distance_mm, uint8_t state,
                               bool detected);

/* Unpack.  Any output pointer may be NULL if that field is not needed. */
extern void s5_unpack_status(uint16_t word, uint16_t * p_distance_mm,
                             uint8_t * p_state, bool * p_detected);

#endif /* S5_PACK_H */
