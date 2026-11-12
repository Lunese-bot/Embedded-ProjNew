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
    /* ONE LINE PER LETTER: change the action when the course rules say
       what a letter means.  Actions:
         NAV_LEFT      90 deg left at the next junction
         NAV_RIGHT     90 deg right at the next junction
         NAV_STRAIGHT  drive straight over the next junction
         NAV_U_TURN    turn round at the next junction
         NAV_NONE      not known yet: same as straight
       The dashboard's "At every junction: Left / Right / Straight" buttons
       override this table on the bench.  Longer codes ("AB") can be added
       as extra lines; the whole decoded text must match. */
    { "A", NAV_NONE    },
    { "B", NAV_NONE    },
    { "C", NAV_NONE    },
    { "D", NAV_NONE    },
    { "E", NAV_NONE    },
    { "F", NAV_NONE    },
    { "G", NAV_NONE    },
    { "H", NAV_NONE    },
    { "I", NAV_NONE    },
    { "J", NAV_NONE    },
    { "K", NAV_NONE    },
    { "L", NAV_NONE    },
    { "M", NAV_NONE    },
    { "N", NAV_NONE    },
    { "O", NAV_NONE    },
    { "P", NAV_NONE    },
    { "Q", NAV_NONE    },
    { "R", NAV_NONE    },
    { "S", NAV_NONE    },
    { "T", NAV_NONE    },
    { "U", NAV_NONE    },
    { "V", NAV_NONE    },
    { "W", NAV_NONE    },
    { "X", NAV_NONE    },
    { "Y", NAV_NONE    },
    { "Z", NAV_RIGHT   }, /* 2026-11-05: team's test barcode */
    { 0, NAV_NONE }     /* sentinel: end of the table */
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
