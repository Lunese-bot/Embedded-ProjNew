/*
 *      s5_pack.c
 *      Subsystem 5 -- pack / unpack the 16-bit telemetry word.
 *      See s5_pack.h for the bit layout.
 */

#include <stddef.h> /* NULL (this file includes no kernel headers) */

#include "s5_pack.h"

uint16_t s5_pack_status(uint16_t distance_mm, uint8_t state, bool detected)
{
    uint16_t word;

    if (distance_mm > S5_PACK_DIST_MAX_MM)
    {
        distance_mm = S5_PACK_DIST_MAX_MM;
    }

    word = (uint16_t)(distance_mm & S5_PACK_DIST_MASK);
    word |= (uint16_t)((uint16_t)(state & S5_PACK_STATE_MASK)
                       << S5_PACK_STATE_SHIFT);
    if (detected)
    {
        word |= (uint16_t)(1U << S5_PACK_DET_SHIFT);
    }
    return word;
}

void s5_unpack_status(uint16_t word, uint16_t * p_distance_mm,
                      uint8_t * p_state, bool * p_detected)
{
    if (NULL != p_distance_mm)
    {
        *p_distance_mm = (uint16_t)(word & S5_PACK_DIST_MASK);
    }
    if (NULL != p_state)
    {
        *p_state = (uint8_t)((word >> S5_PACK_STATE_SHIFT) &
                             S5_PACK_STATE_MASK);
    }
    if (NULL != p_detected)
    {
        *p_detected = (0U != ((word >> S5_PACK_DET_SHIFT) & 1U));
    }
}
