/*
 * vc_barcode_rules.h
 * Full-string barcode-to-navigation mapping for the full Vehicle Controller.
 * Keep the table empty until the course's actual barcode instructions are known.
 */
#ifndef VC_BARCODE_RULES_H
#define VC_BARCODE_RULES_H

#include "vc_types.h"

/* Exact, case-sensitive match. Unknown/empty text returns NAV_NONE. */
navigation_command_t vc_barcode_lookup(const char *text);

#endif /* VC_BARCODE_RULES_H */
