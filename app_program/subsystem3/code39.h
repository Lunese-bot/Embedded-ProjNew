/*
 *      code39.h
 *      Subsystem 3 -- Code 39 barcode decoder.
 *
 *      Pure C, no kernel calls: it is unit-tested on the PC
 *      (testing/subsystem3/host/) as well as on the robot.
 *
 *      Input is the list of element widths in the order the sensor saw
 *      them, starting with the first BAR:  bar, space, bar, space, ... bar.
 *      Units do not matter (us, encoder ticks...), only the ratios do.
 *      A barcode read backwards (robot driving the other way) is decoded
 *      too.
 */

#ifndef CODE39_H
#define CODE39_H

#include <stdint.h>
#include <stdbool.h>

/* No <stddef.h>: the kernel headers define their own size_t */
#ifndef NULL
#define NULL ((void *)0)
#endif

typedef enum
{
    CODE39_OK = 0,
    CODE39_ERR_PARAM,   /* NULL pointer or output too small     */
    CODE39_ERR_LENGTH,  /* element count is not 10*n - 1, n >= 3 */
    CODE39_ERR_RATIO,   /* wide and narrow too close to tell    */
    CODE39_ERR_PATTERN, /* a character pattern is not Code 39   */
    CODE39_ERR_FRAME    /* no '*' start/stop in either direction */
} code39_result_t;

typedef struct
{
    uint8_t min_ratio_x10; /* e.g. 15 = wide must be >= 1.5 x narrow */
} code39_params_t;

/*
 * Decode.  On CODE39_OK, p_text holds the data characters between the
 * start and stop '*', '\0'-terminated, and *p_reversed says whether the
 * code was read backwards.
 */
code39_result_t code39_decode(uint32_t const * p_widths, uint32_t count,
                              code39_params_t const * p_params, char * p_text,
                              uint32_t text_size, bool * p_reversed);

/*
 * Encode one text (without the '*') into element widths, for tests.
 * Returns the number of widths written, 0 if it does not fit or a
 * character has no Code 39 pattern.
 */
uint32_t code39_encode(char const * p_text, uint32_t narrow, uint32_t wide,
                       uint32_t * p_widths, uint32_t max_widths);

#endif /* CODE39_H */

/*** end of file ***/
