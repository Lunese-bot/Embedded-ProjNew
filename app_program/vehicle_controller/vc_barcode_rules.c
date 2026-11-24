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
    const char *text;
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
    { 0, NAV_NONE }     /* sentinel: end of the table */
};

/* Compile-time checks on vc_barcode_config.h: every line must hold one of
   the four actions (or NAV_NONE), and there must be exactly 26 lines.
   A bad value stops the build here instead of misdriving the robot. */
#define VC_RULE_VALID(letter, nav) \
    && ((int)(nav) >= (int)NAV_NONE) && ((int)(nav) <= (int)NAV_U_TURN)
#define VC_RULE_COUNT(letter, nav) + 1
typedef char vc_cfg_every_entry_is_a_valid_action
    [(1 VC_BARCODE_LETTER_TABLE(VC_RULE_VALID)) ? 1 : -1];
typedef char vc_cfg_table_has_26_letters
    [((0 VC_BARCODE_LETTER_TABLE(VC_RULE_COUNT)) == (int)VC_BARCODE_LETTER_COUNT) ? 1 : -1];
#undef VC_RULE_VALID
#undef VC_RULE_COUNT

navigation_command_t vc_barcode_lookup(const char *text)
{
    uint32_t i;

    if ((0 == text) || ('\0' == text[0]))
    {
        return NAV_NONE;
    }
    if ('\0' == text[1])
    {
        /* exactly one character: the active map decides (A-Z only) */
        return vc_bcmap_lookup_letter(text[0]);
    }

    for (i = 0u; (i < (uint32_t)(sizeof(g_rules) / sizeof(g_rules[0]))) &&
                 (0 != g_rules[i].text); i++)
    {
        uint32_t j = 0u;
        while ((text[j] != '\0') && (g_rules[i].text[j] != '\0') &&
               (text[j] == g_rules[i].text[j]))
        {
            j++;
        }
        if ((text[j] == '\0') && (g_rules[i].text[j] == '\0'))
        {
            return g_rules[i].nav;
        }
    }

    return NAV_NONE;
}
