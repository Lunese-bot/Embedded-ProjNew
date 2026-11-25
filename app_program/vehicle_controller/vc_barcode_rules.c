/*
 * vc_barcode_rules.c
 *
 * Barcode text -> navigation command.  The entire decoded string is matched,
 * not just its first character.  A single letter A to Z is resolved against
 * the robot's ACTIVE map (vc_barcode_map.c): the dashboard's saved map, or
 * the factory map from vc_barcode_config.h when nothing was saved.  Unknown,
 * empty or multi-character text returns NAV_NONE and the robot continues
 * straight at the next junction.
 */
#include "vc_barcode_rules.h"
#include "vc_barcode_config.h"
#include "vc_barcode_map.h"

typedef struct
{
    const char *         text;
    navigation_command_t nav;
} vc_barcode_rule_t;

static const vc_barcode_rule_t g_rules[] = {
#ifdef VC_BARCODE_RULES_TEST
    /* Host-test-only mappings; never enabled in the firmware build. */
    { "TESTLEFT", NAV_LEFT },
    { "TESTRIGHT", NAV_RIGHT },
    { "TESTUTURN", NAV_U_TURN },
    { "TESTSTRAIGHT", NAV_STRAIGHT },
#endif
/* One line per letter, generated from vc_barcode_config.h */
#define VC_RULE_LINE(letter, nav) { #letter, nav },
    VC_BARCODE_LETTER_TABLE(VC_RULE_LINE)
#undef VC_RULE_LINE
        { 0, NAV_NONE } /* sentinel: end of the table */
};

/* Compile-time checks on vc_barcode_config.h: every line must hold one of
   the four actions (or NAV_NONE), and there must be exactly 26 lines.
   A bad value stops the build here instead of misdriving the robot. */
#define VC_RULE_VALID(letter, nav) \
    &&((int)(nav) >= (int)NAV_NONE) && ((int)(nav) <= (int)NAV_U_TURN)
#define VC_RULE_COUNT(letter, nav) +1
_Static_assert(1 VC_BARCODE_LETTER_TABLE(VC_RULE_VALID),
               "every barcode table entry must be a valid action");
_Static_assert((0 VC_BARCODE_LETTER_TABLE(VC_RULE_COUNT)) ==
                   (int)VC_BARCODE_LETTER_COUNT,
               "the barcode table must have 26 letters");
#undef VC_RULE_VALID
#undef VC_RULE_COUNT

navigation_command_t vc_barcode_lookup(const char * p_text)
{
    uint32_t idx;

    if ((0 == p_text) || ('\0' == p_text[0]))
    {
        return NAV_NONE;
    }
    if ('\0' == p_text[1])
    {
        /* exactly one character: the active map decides (A-Z only) */
        return vc_bcmap_lookup_letter(p_text[0]);
    }

    for (idx = 0u; (idx < (uint32_t)(sizeof(g_rules) / sizeof(g_rules[0]))) &&
                   (0 != g_rules[idx].text);
         idx++)
    {
        uint32_t jdx = 0u;
        while ((p_text[jdx] != '\0') && (g_rules[idx].text[jdx] != '\0') &&
               (p_text[jdx] == g_rules[idx].text[jdx]))
        {
            jdx++;
        }
        if ((p_text[jdx] == '\0') && (g_rules[idx].text[jdx] == '\0'))
        {
            return g_rules[idx].nav;
        }
    }

    return NAV_NONE;
}

/*** end of file ***/
