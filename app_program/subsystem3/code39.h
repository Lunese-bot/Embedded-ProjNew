/*
 *	code39.h
 *	Subsystem 3 -- Code 39 barcode decoder.
 *
 *	Pure C, no kernel calls: it is unit-tested on the PC
 *	(testing/subsystem3/host/) as well as on the robot.
 *
 *	Input is the list of element widths in the order the sensor saw
 *	them, starting with the first BAR:  bar, space, bar, space, ... bar.
 *	Units do not matter (us, encoder ticks...), only the ratios do.
 *	A barcode read backwards (robot driving the other way) is decoded
 *	too.
 *
 *	Barcodes may have any number of characters (3 .. max_chars, counting
 *	the two '*') and any physical size: every character is classified
 *	from its own 9 elements, so a barcode printed 2x bigger, or read at
 *	a different speed, decodes the same.
 */

#ifndef CODE39_H
#define CODE39_H

#include <stdint.h>
#include <stdbool.h>

/* No <stddef.h>: the kernel headers define their own size_t */
#ifndef NULL
#define NULL    ((void *)0)
#endif

typedef enum
{
    CODE39_OK = 0,
    CODE39_ERR_PARAM,       /* NULL pointer or output too small     */
    CODE39_ERR_LENGTH,      /* element count is not 10*n - 1, n >= 3 */
    CODE39_ERR_RATIO,       /* wide and narrow too close to tell    */
    CODE39_ERR_PATTERN,     /* a character pattern is not Code 39   */
    CODE39_ERR_FRAME,       /* no '*' start/stop in either direction, or a
                               '*' in the middle of the data             */
    CODE39_ERR_CHARSET      /* a real Code 39 character, but not an allowed
                               one (letters_only: anything but A-Z)      */
} code39_result_t;

typedef struct
{
    uint8_t  min_ratio_x10;  /* e.g. 15 = wide must be >= 1.5 x narrow */
} code39_params_t;

/*
 * Decode.  On CODE39_OK, p_text holds the data characters between the
 * start and stop '*', '\0'-terminated, and *p_reversed says whether the
 * code was read backwards.
 */
code39_result_t code39_decode(uint32_t const * p_widths, uint32_t count,
                              code39_params_t const * p_params,
                              char * p_text, uint32_t text_size,
                              bool * p_reversed);

/*
 * Same, with extra options.  code39_decode() is this with letters_only =
 * false and max_chars = 0.
 *
 *   letters_only  data characters must be 'A'..'Z'.  A pattern that is a
 *                 valid Code 39 digit or symbol is then a misread, and
 *                 is rejected with CODE39_ERR_CHARSET instead of being
 *                 passed on as a wrong command.
 *   max_chars     longest accepted barcode in characters INCLUDING the two
 *                 '*' (0 = no limit).  Longer ones give CODE39_ERR_LENGTH.
 */
typedef struct
{
    uint8_t  min_ratio_x10;  /* e.g. 15 = wide must be >= 1.5 x narrow */
    bool     letters_only;
    uint8_t  max_chars;
} code39_options_t;

code39_result_t code39_decode_ex(uint32_t const * p_widths, uint32_t count,
                                 code39_options_t const * p_options,
                                 char * p_text, uint32_t text_size,
                                 bool * p_reversed);

/*
 * Partial read (2026-11-11): for a ONE-letter code whose element count is
 * wrong because one end got distorted.  Accepts the letter when it and the
 * '*' right next to it (before or after, with a narrow gap between) both
 * decode cleanly.  Every match in the elements must agree on the letter and
 * direction, or the read is rejected (CODE39_ERR_FRAME).  Only one letter
 * is returned, so do not use it for multi-letter codes.
 */
code39_result_t code39_decode_partial(uint32_t const * p_widths,
                                      uint32_t count,
                                      code39_options_t const * p_options,
                                      char * p_text, uint32_t text_size,
                                      bool * p_reversed);

/*
 * Encode one text (without the '*') into element widths, for tests.
 * Returns the number of widths written, 0 if it does not fit or a
 * character has no Code 39 pattern.
 */
uint32_t code39_encode(char const * p_text, uint32_t narrow, uint32_t wide,
                     uint32_t * p_widths, uint32_t max_widths);

#endif /* CODE39_H */
