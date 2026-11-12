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
 *	matter.  For the same reason the physical size of the barcode does not
 *	matter either, and neither does the number of characters.
 */

#include "code39.h"

#define ELEMENTS_PER_CHAR   9u
#define CHAR_PITCH          10u     /* 9 elements + 1 gap */
#define MIN_CHARS           3u      /* '*' + one data char + '*' */
#define PARTIAL_END_SLACK   2u      /* stray elements allowed past a '*' */
#define PARTIAL_CHAR_TOL_PCT  10u   /* '*' and letter widths may differ by */
#define PARTIAL_OTHER_TOL_PCT 12u   /* damaged '*' still 15 units, +- */
#define PARTIAL_MIN_EL_PCT    50u   /* thinnest element vs one unit */
#define PARTIAL_MIN_RATIO_X10 18u   /* thick at least 1.8 x thin */

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

/*
 * Second way to classify one character: bars and spaces separately.
 * Every letter, digit and '*' has exactly 2 wide bars (of 5) and 1 wide
 * space (of 4).  A sensor whose spot is about as big as a narrow bar
 * reads all bars a little wide and all spaces a little narrow (or the
 * other way round); comparing bars only with bars and spaces only with
 * spaces is not upset by that.  Returns the pattern, or 0 if wide and
 * narrow cannot be told apart.
 */
static uint16_t
classify_char_split (uint32_t const * p_el, uint8_t min_ratio_x10)
{
    uint32_t wide[2]   = { 0u, 0u }; /* two widest bars, wide[0] >= wide[1] */
    uint32_t bar_next  = 0u;         /* widest of the other three bars     */
    uint32_t sp_wide   = 0u;         /* widest space                       */
    uint32_t sp_next   = 0u;         /* second widest space                */
    uint32_t thr_bar;
    uint32_t thr_sp;
    uint16_t pattern = 0u;
    uint32_t i;

    for (i = 0u; i < ELEMENTS_PER_CHAR; i++)
    {
        uint32_t w = p_el[i];

        if (0u == (i % 2u)) /* bar */
        {
            if (w > wide[0])      { bar_next = wide[1]; wide[1] = wide[0]; wide[0] = w; }
            else if (w > wide[1]) { bar_next = wide[1]; wide[1] = w; }
            else if (w > bar_next) { bar_next = w; }
            else { /* a narrower bar */ }
        }
        else                /* space */
        {
            if (w > sp_wide)      { sp_next = sp_wide; sp_wide = w; }
            else if (w > sp_next) { sp_next = w; }
            else { /* a narrower space */ }
        }
    }

    if (((wide[1] * 10u) < (bar_next * (uint32_t)min_ratio_x10)) ||
        ((sp_wide * 10u) < (sp_next * (uint32_t)min_ratio_x10)))
    {
        return 0u;
    }
    thr_bar = (wide[1] + bar_next) / 2u;
    thr_sp  = (sp_wide + sp_next) / 2u;

    for (i = 0u; i < ELEMENTS_PER_CHAR; i++)
    {
        pattern = (uint16_t)(pattern << 1);
        if (p_el[i] > ((0u == (i % 2u)) ? thr_bar : thr_sp))
        {
            pattern |= 1u;
        }
    }
    return pattern;
}

/* Decode in one direction.  p_el is read with stride +1 or -1. */
static code39_result_t
decode_direction (uint32_t const * p_widths, uint32_t count, bool reverse,
                  code39_options_t const * p_opt, char * p_text,
                  uint32_t text_size)
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

        pattern = classify_char(el, p_opt->min_ratio_x10);
        ch      = (0u != pattern) ? pattern_to_char(pattern) : '\0';
        if ('\0' == ch)
        {
            /* not clear with all 9 together: bars and spaces separately */
            uint16_t split = classify_char_split(el, p_opt->min_ratio_x10);

            if (0u != split)
            {
                ch = pattern_to_char(split);
            }
            if ('\0' == ch)
            {
                return (0u == pattern) ? CODE39_ERR_RATIO
                                       : CODE39_ERR_PATTERN;
            }
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
            /* '*' is reserved for start/stop: inside the data it means two
               barcodes run together, or a misread. */
            if ('*' == ch)
            {
                return CODE39_ERR_FRAME;
            }

            /* Letters-only: a digit or symbol is a misread, not a command */
            if (p_opt->letters_only && ((ch < 'A') || (ch > 'Z')))
            {
                return CODE39_ERR_CHARSET;
            }

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
code39_decode_ex (uint32_t const * p_widths, uint32_t count,
                  code39_options_t const * p_options,
                  char * p_text, uint32_t text_size, bool * p_reversed)
{
    code39_result_t result;
    uint32_t        n_chars;

    if ((NULL == p_widths) || (NULL == p_options) || (NULL == p_text) ||
        (NULL == p_reversed) || (0u == text_size))
    {
        return CODE39_ERR_PARAM;
    }

    p_text[0] = '\0';

    /* Any length is fine as long as it is 10*n - 1 elements, n >= 3 */
    if (((count + 1u) % CHAR_PITCH) != 0u)
    {
        return CODE39_ERR_LENGTH;
    }

    n_chars = (count + 1u) / CHAR_PITCH;
    if ((n_chars < MIN_CHARS) ||
        ((p_options->max_chars != 0u) && (n_chars > p_options->max_chars)))
    {
        return CODE39_ERR_LENGTH;
    }

    *p_reversed = false;
    result = decode_direction(p_widths, count, false,
                              p_options, p_text, text_size);
    if ((CODE39_ERR_FRAME == result) || (CODE39_ERR_PATTERN == result))
    {
        /* '*' read backwards is not '*', so try the other way */
        *p_reversed = true;
        result = decode_direction(p_widths, count, true,
                                  p_options, p_text, text_size);
    }

    if (CODE39_OK != result)
    {
        p_text[0] = '\0';
    }

    return result;
}

/* Strict: the 3 widest of the 9 must be at least PARTIAL_MIN_RATIO_X10 / 10
   times the 4th widest, then the pattern must be a Code 39 character. */
static char
window_char_strict (uint32_t const * p_widths, uint32_t count, bool reverse,
                    uint32_t start)
{
    uint32_t el[ELEMENTS_PER_CHAR];
    uint32_t i;
    uint32_t j;
    uint32_t big[4] = { 0u, 0u, 0u, 0u }; /* 4 widest, largest first */
    uint16_t pattern = 0u;

    for (i = 0u; i < ELEMENTS_PER_CHAR; i++)
    {
        uint32_t pos = start + i;
        uint32_t w   = reverse ? p_widths[count - 1u - pos] : p_widths[pos];

        el[i] = w;
        for (j = 0u; j < 4u; j++)
        {
            if (w > big[j])
            {
                uint32_t m;
                for (m = 3u; m > j; m--)
                {
                    big[m] = big[m - 1u];
                }
                big[j] = w;
                break;
            }
        }
    }
    if ((big[2] * 10u) < (big[3] * PARTIAL_MIN_RATIO_X10))
    {
        return '\0';
    }
    for (i = 0u; i < ELEMENTS_PER_CHAR; i++)
    {
        if (el[i] >= big[2])
        {
            pattern = (uint16_t)(pattern | (uint16_t)(1u << (8u - i)));
        }
    }
    return pattern_to_char(pattern);
}

/* Width checks for a partial read whose two characters start at element
   k (reading direction).  A '*' and a letter are both 15 narrow units wide
   (6 narrow + 3 wide at 3:1), the gap between them 1.  A bar lost or
   split INSIDE the letter makes the 9-element window cover the wrong
   stretch, which shows as a width mismatch.  Also the whole read is
   47 units for a one-letter code (merges and splits do not change it). */
static bool
partial_widths_ok (uint32_t const * p_widths, uint32_t count, bool reverse,
                   uint32_t k, bool star_first)
{
    uint32_t a = 0u;
    uint32_t b = 0u;
    uint32_t total = 0u;
    uint32_t i;
    uint32_t w;

    for (i = 0u; i < count; i++)
    {
        w = reverse ? p_widths[count - 1u - i] : p_widths[i];
        total += w;
        if ((i >= k) && (i < (k + ELEMENTS_PER_CHAR)))
        {
            a += w;
        }
        if ((i >= (k + CHAR_PITCH)) && (i < (k + CHAR_PITCH + ELEMENTS_PER_CHAR)))
        {
            b += w;
        }
    }
    /* The gaps around the two characters are narrow (1 unit; a wide is 3):
       the one between them, and the one beyond the letter on the side of
       the damaged '*'.  A bar lost inside the letter shifts the window, and
       then a wide element lands where a gap should be. */
    {
        uint32_t two_units = ((a + b) * 2u) / 30u;
        uint32_t idx[3];
        uint32_t m;

        idx[0] = k + ELEMENTS_PER_CHAR;                     /* between */
        idx[1] = k + (2u * CHAR_PITCH) - 1u;                /* after second */
        idx[2] = (k > 0u) ? (k - 1u) : count;               /* before first */
        for (m = 0u; m < 3u; m++)
        {
            if (idx[m] < count)
            {
                w = reverse ? p_widths[count - 1u - idx[m]] : p_widths[idx[m]];
                if (w > two_units)
                {
                    return false;
                }
            }
        }
    }
    /* no sliver: every element of the two characters is at least
       PARTIAL_MIN_EL_PCT of one unit (a split bar leaves tiny pieces) */
    {
        uint32_t min_w = (((a + b) / 30u) * PARTIAL_MIN_EL_PCT) / 100u;

        for (i = k; i < (k + (2u * CHAR_PITCH) - 1u); i++)
        {
            w = reverse ? p_widths[count - 1u - i] : p_widths[i];
            if (w < min_w)
            {
                return false;
            }
        }
    }
    /* the two characters within PARTIAL_CHAR_TOL_PCT of each other */
    if (((a * 100u) > (b * (100u + PARTIAL_CHAR_TOL_PCT))) ||
        ((b * 100u) > (a * (100u + PARTIAL_CHAR_TOL_PCT))))
    {
        return false;
    }
    /* The damaged '*' on the other side still covers 15 units: a bar
       lost or split there does not change its total width.  Allow up to
       PARTIAL_END_SLACK stray elements at the far end. */
    {
        uint32_t unit15 = ((a + b) * 15u) / 30u;
        uint32_t lo     = (unit15 * (100u - PARTIAL_OTHER_TOL_PCT)) / 100u;
        uint32_t hi     = (unit15 * (100u + PARTIAL_OTHER_TOL_PCT)) / 100u;
        uint32_t first  = star_first ? (k + (2u * CHAR_PITCH)) : 0u;
        uint32_t last   = star_first ? count : ((k > 0u) ? (k - 1u) : 0u);
        uint32_t e;
        bool     b_ok   = false;

        for (e = 0u; (e <= PARTIAL_END_SLACK) && !b_ok; e++)
        {
            uint32_t from = star_first ? first : (first + e);
            uint32_t to   = star_first ? ((last >= e) ? (last - e) : 0u) : last;
            uint32_t sum  = 0u;

            for (i = from; i < to; i++)
            {
                sum += reverse ? p_widths[count - 1u - i] : p_widths[i];
            }
            b_ok = (to > from) && (sum >= lo) && (sum <= hi);
        }
        if (!b_ok)
        {
            return false;
        }
    }
    (void)total;
    return true;
}

code39_result_t
code39_decode_partial (uint32_t const * p_widths, uint32_t count,
                       code39_options_t const * p_options,
                       char * p_text, uint32_t text_size, bool * p_reversed)
{
    uint32_t pass;
    uint32_t k;
    char     found     = '\0';
    bool     found_rev = false;

    if ((NULL == p_widths) || (NULL == p_options) || (NULL == p_text) ||
        (NULL == p_reversed) || (text_size < 2u))
    {
        return CODE39_ERR_PARAM;
    }
    p_text[0] = '\0';
    if (count < ((2u * CHAR_PITCH) - 1u))
    {
        return CODE39_ERR_LENGTH;
    }

    for (pass = 0u; pass < 2u; pass++)
    {
        bool reverse = (1u == pass);

        for (k = 0u; (k + (2u * CHAR_PITCH) - 1u) <= count; k++)
        {
            uint32_t orig  = reverse ? (count - 1u - k) : k;
            uint32_t gap_i = k + ELEMENTS_PER_CHAR;
            uint32_t gap;
            uint32_t j;
            uint32_t widest = 0u;
            char     c1;
            char     c2;
            char     letter;

            if (0u != (orig % 2u))
            {
                continue; /* a character starts on a bar (even element) */
            }
            /* stricter than a full read: no "bars and spaces separately"
               fallback, and a clear split between thick and thin */
            c1 = window_char_strict(p_widths, count, reverse, k);
            c2 = window_char_strict(p_widths, count, reverse, k + CHAR_PITCH);
            /* The clean '*' must be at its own end of the list (at most
               PARTIAL_END_SLACK stray elements beyond it).  Read backwards,
               some letters look like '*' and '*' like a letter, so a '*'
               found in the middle of the list proves nothing. */
            if (('*' == c1) && ('*' != c2) && ('\0' != c2) &&
                (k <= PARTIAL_END_SLACK))
            {
                letter = c2; /* start '*', then the letter */
            }
            else if (('*' == c2) && ('*' != c1) && ('\0' != c1) &&
                     ((count - (k + (2u * CHAR_PITCH) - 1u)) <= PARTIAL_END_SLACK))
            {
                letter = c1; /* the letter, then the stop '*' */
            }
            else
            {
                continue;
            }
            if (p_options->letters_only && ((letter < 'A') || (letter > 'Z')))
            {
                continue;
            }
            /* the gap between the two characters is narrow: never wider
               than the widest element of the two characters */
            gap = reverse ? p_widths[count - 1u - gap_i] : p_widths[gap_i];
            for (j = 0u; j < (2u * CHAR_PITCH) - 1u; j++)
            {
                uint32_t pos = k + j;
                uint32_t w   = reverse ? p_widths[count - 1u - pos]
                                       : p_widths[pos];
                if ((j != ELEMENTS_PER_CHAR) && (w > widest))
                {
                    widest = w;
                }
            }
            if (gap > widest)
            {
                continue;
            }
            if (!partial_widths_ok(p_widths, count, reverse, k,
                                   ('*' == c1)))
            {
                continue;
            }
            if (('\0' != found) && ((found != letter) || (found_rev != reverse)))
            {
                p_text[0] = '\0';
                return CODE39_ERR_FRAME; /* two different answers: give up */
            }
            found     = letter;
            found_rev = reverse;
        }
    }

    if ('\0' == found)
    {
        return CODE39_ERR_FRAME;
    }
    p_text[0]   = found;
    p_text[1]   = '\0';
    *p_reversed = found_rev;
    return CODE39_OK;
}

code39_result_t
code39_decode (uint32_t const * p_widths, uint32_t count,
               code39_params_t const * p_params,
               char * p_text, uint32_t text_size, bool * p_reversed)
{
    code39_options_t options;

    if (NULL == p_params)
    {
        return CODE39_ERR_PARAM;
    }

    options.min_ratio_x10 = p_params->min_ratio_x10;
    options.letters_only  = false;      /* the old behaviour: every character */
    options.max_chars     = 0u;         /* no length limit */

    return code39_decode_ex(p_widths, count, &options,
                            p_text, text_size, p_reversed);
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
