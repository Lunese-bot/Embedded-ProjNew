/*
 *	code39.c
 *	Subsystem 3 -- Code 39 barcode decoder (see code39.h).
 *
 *	A Code 39 character is 9 elements, B S B S B S B S B, exactly three
 *	of them wide.  Characters are separated by one narrow space, so a
 *	barcode of n characters has 10*n - 1 elements.  Every barcode
 *	starts and ends with '*'.
 *
 *	Each character is classified on its own (its 3 widest elements are
 *	"wide"), so a slow drift in robot speed along the barcode does not
 *	matter.
 */

#include "code39.h"

#define ELEMENTS_PER_CHAR   9u
#define CHAR_PITCH          10u     /* 9 elements + 1 gap */
#define MIN_CHARS           3u      /* '*' + one data char + '*' */

/* Patterns as 9 bits, first element in bit 8, 1 = wide */
typedef struct
{
    char     ch;
    uint16_t pattern;
} code39_entry_t;

static code39_entry_t const g_table[] =
{
    { '0', 0x034 }, { '1', 0x121 }, { '2', 0x061 }, { '3', 0x160 },
    { '4', 0x031 }, { '5', 0x130 }, { '6', 0x070 }, { '7', 0x025 },
    { '8', 0x124 }, { '9', 0x064 }, { 'A', 0x109 }, { 'B', 0x049 },
    { 'C', 0x148 }, { 'D', 0x019 }, { 'E', 0x118 }, { 'F', 0x058 },
    { 'G', 0x00D }, { 'H', 0x10C }, { 'I', 0x04C }, { 'J', 0x01C },
    { 'K', 0x103 }, { 'L', 0x043 }, { 'M', 0x142 }, { 'N', 0x013 },
    { 'O', 0x112 }, { 'P', 0x052 }, { 'Q', 0x007 }, { 'R', 0x106 },
    { 'S', 0x046 }, { 'T', 0x016 }, { 'U', 0x181 }, { 'V', 0x0C1 },
    { 'W', 0x1C0 }, { 'X', 0x091 }, { 'Y', 0x190 }, { 'Z', 0x0D0 },
    { '-', 0x085 }, { '.', 0x184 }, { ' ', 0x0C4 }, { '*', 0x094 },
    { '$', 0x0A8 }, { '/', 0x0A2 }, { '+', 0x08A }, { '%', 0x02A }
};

#define TABLE_SIZE  (sizeof(g_table) / sizeof(g_table[0]))

static char
pattern_to_char (uint16_t pattern)
{
    uint32_t idx;

    for (idx = 0u; idx < TABLE_SIZE; idx++)
    {
        if (g_table[idx].pattern == pattern)
        {
            return g_table[idx].ch;
        }
    }

    return '\0';
}

static uint16_t
char_to_pattern (char ch)
{
    uint32_t idx;

    for (idx = 0u; idx < TABLE_SIZE; idx++)
    {
        if (g_table[idx].ch == ch)
        {
            return g_table[idx].pattern;
        }
    }

    return 0u;
}

/*
 * Classify the 9 elements of one character.  Returns the pattern, or 0
 * if wide and narrow cannot be told apart.
 */
static uint16_t
classify_char (uint32_t const * p_el, uint8_t min_ratio_x10)
{
    uint32_t sorted[ELEMENTS_PER_CHAR];
    uint32_t tmp;
    uint32_t threshold;
    uint16_t pattern = 0u;
    uint32_t   i;
    uint32_t   j;

    for (i = 0u; i < ELEMENTS_PER_CHAR; i++)
    {
        sorted[i] = p_el[i];
    }

    /* Insertion sort, descending: sorted[2] is the narrowest wide
       element, sorted[3] the widest narrow one. */
    for (i = 1u; i < ELEMENTS_PER_CHAR; i++)
    {
        tmp = sorted[i];
        j = i;
        while ((j > 0u) && (sorted[j - 1u] < tmp))
        {
            sorted[j] = sorted[j - 1u];
            j--;
        }
        sorted[j] = tmp;
    }

    if ((sorted[2] * 10u) < (sorted[3] * (uint32_t)min_ratio_x10))
    {
        return 0u;
    }

    threshold = (sorted[2] + sorted[3]) / 2u;

    for (i = 0u; i < ELEMENTS_PER_CHAR; i++)
    {
        pattern = (uint16_t)(pattern << 1);
        if (p_el[i] > threshold)
        {
            pattern |= 1u;
        }
    }

    return pattern;
}

/* Decode in one direction.  p_el is read with stride +1 or -1. */
static code39_result_t
decode_direction (uint32_t const * p_widths, uint32_t count, bool reverse,
                  uint8_t min_ratio_x10, char * p_text, uint32_t text_size)
{
    uint32_t el[ELEMENTS_PER_CHAR];
    uint32_t   n_chars = (count + 1u) / CHAR_PITCH;
    uint32_t   c;
    uint32_t   i;
    uint32_t   pos;
    uint32_t   out = 0u;
    uint16_t pattern;
    char     ch;

    for (c = 0u; c < n_chars; c++)
    {
        for (i = 0u; i < ELEMENTS_PER_CHAR; i++)
        {
            pos = (c * CHAR_PITCH) + i;
            el[i] = reverse ? p_widths[count - 1u - pos] : p_widths[pos];
        }

        pattern = classify_char(el, min_ratio_x10);
        if (0u == pattern)
        {
            return CODE39_ERR_RATIO;
        }

        ch = pattern_to_char(pattern);
        if ('\0' == ch)
        {
            return CODE39_ERR_PATTERN;
        }

        if ((0u == c) || ((n_chars - 1u) == c))
        {
            if ('*' != ch)
            {
                return CODE39_ERR_FRAME;
            }
        }
        else
        {
            if ((out + 1u) >= text_size)
            {
                return CODE39_ERR_PARAM;
            }
            p_text[out] = ch;
            out++;
        }
    }

    p_text[out] = '\0';
    return CODE39_OK;
}

code39_result_t
code39_decode (uint32_t const * p_widths, uint32_t count,
               code39_params_t const * p_params,
               char * p_text, uint32_t text_size, bool * p_reversed)
{
    code39_result_t result;

    if ((NULL == p_widths) || (NULL == p_params) || (NULL == p_text) ||
        (NULL == p_reversed) || (0u == text_size))
    {
        return CODE39_ERR_PARAM;
    }

    p_text[0] = '\0';

    if (((count + 1u) % CHAR_PITCH != 0u) ||
        (((count + 1u) / CHAR_PITCH) < MIN_CHARS))
    {
        return CODE39_ERR_LENGTH;
    }

    *p_reversed = false;
    result = decode_direction(p_widths, count, false,
                              p_params->min_ratio_x10, p_text, text_size);
    if ((CODE39_ERR_FRAME == result) || (CODE39_ERR_PATTERN == result))
    {
        /* '*' read backwards is not '*', so try the other way */
        *p_reversed = true;
        result = decode_direction(p_widths, count, true,
                                  p_params->min_ratio_x10,
                                  p_text, text_size);
    }

    if (CODE39_OK != result)
    {
        p_text[0] = '\0';
    }

    return result;
}

static uint32_t
encode_char (char ch, uint32_t narrow, uint32_t wide, uint32_t * p_out)
{
    uint16_t pattern = char_to_pattern(ch);
    uint32_t   i;

    if (0u == pattern)
    {
        return 0u;
    }

    for (i = 0u; i < ELEMENTS_PER_CHAR; i++)
    {
        p_out[i] = ((pattern >> (8u - i)) & 1u) ? wide : narrow;
    }

    return ELEMENTS_PER_CHAR;
}

uint32_t
code39_encode (char const * p_text, uint32_t narrow, uint32_t wide,
               uint32_t * p_widths, uint32_t max_widths)
{
    uint32_t len = 0u;
    uint32_t n_chars;
    uint32_t c;
    uint32_t pos = 0u;
    char   ch;

    if ((NULL == p_text) || (NULL == p_widths))
    {
        return 0u;
    }

    while ('\0' != p_text[len])
    {
        len++;
    }

    n_chars = len + 2u;
    if (((n_chars * CHAR_PITCH) - 1u) > max_widths)
    {
        return 0u;
    }

    for (c = 0u; c < n_chars; c++)
    {
        ch = ((0u == c) || ((n_chars - 1u) == c)) ? '*' : p_text[c - 1u];
        if (0u == encode_char(ch, narrow, wide, &p_widths[pos]))
        {
            return 0u;
        }
        pos += ELEMENTS_PER_CHAR;

        if (c < (n_chars - 1u))
        {
            p_widths[pos] = narrow;     /* inter-character gap */
            pos++;
        }
    }

    return pos;
}
