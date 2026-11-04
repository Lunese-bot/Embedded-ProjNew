/*
 * vc_barcode_rules.c
 *
 * Add the course's confirmed barcode strings below. The entire decoded
 * string is matched, not just its first character. No placeholder commands
 * are enabled by default: unknown barcodes return NAV_NONE and the robot
 * continues straight at the next junction.
 *
 * Example (DO NOT enable until confirmed by the course rules):
 *   { "ABC", NAV_LEFT },
 *   { "DEF", NAV_RIGHT },
 */
#include "vc_barcode_rules.h"


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
#else
    /* Add confirmed course mappings here, e.g. { "ABC", NAV_LEFT }, */
    { 0, NAV_NONE } /* sentinel: no mappings configured by default */
#endif
};

navigation_command_t vc_barcode_lookup(const char *text)
{
    uint32_t i;

    if ((0 == text) || ('\0' == text[0]))
    {
        return NAV_NONE;
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
