/*
 *      obst_encode.c
 *      Prints obstacle size report frames made by the firmware's own code
 *      (app_program/subsystem5/s5_obst_report.c) for test_dashboard.js:
 *        F <40 hex> id rev stage width_status length_status reason front width length t0 side
 *        N <kind> <index> <name>        (the C name tables)
 *      Records are built through the S5 API in the order the avoid task
 *      uses (begin, front, length, final), so they are frames the robot
 *      can actually send.
 */

#include <stdio.h>
#include <stdint.h>

#include "../../../app_program/subsystem5/s5_obst_report.h"

static void print(const s5_obst_report_t * r)
{
    uint8_t f[S5_OR_BYTES];
    int     i;

    s5_or_encode(r, f);
    printf("F ");
    for (i = 0; i < S5_OR_BYTES; i++)
    {
        printf("%02x", f[i]);
    }
    printf(" %u %u %u %u %u %u %u %u %u %lu %u\n", r->profile_id, r->rev, r->stage,
           r->width_status, r->length_status, r->length_reason, r->front_mm,
           r->width_mm, r->length_mm, (unsigned long)r->t0_ms, r->bypass_side);
}

int main(void)
{
    s5_obst_report_t r;
    uint16_t         id = 65530U;
    int              i;

    for (i = 0; i < 60; i++)
    {
        uint32_t t0   = 4000000000U + (uint32_t)i * 7919U;
        int      w    = 40 + (i * 53) % 4100;      /* also beyond the 4000 clamp */
        int      edge = i % 4;                      /* 0 inside, 1 right limit, 2 left limit, 3 one beam */
        int      r_e  = (edge == 1) ? -60 : (edge == 3) ? 10 : -20;
        int      l_e  = (edge == 2) ? 60 : (edge == 3) ? 10 : 20;

        id = s5_or_next_id(id);                     /* crosses 65535 -> 1 */
        s5_or_begin(&r, id, t0);
        print(&r);
        s5_or_front(&r, (uint8_t)((i % 9) != 8), (uint16_t)(100 + (i * 37) % 2100), r_e, l_e,
                    (uint16_t)w, (uint8_t)(1 + (i & 1)), -60, 60, (i % 5) != 4,
                    (s5_or_reason_t)((i % 5) == 4 ? S5_OR_R_OBSERVE_OFF : S5_OR_R_NONE));
        print(&r);
        if ((i % 3) == 0)
        {
            s5_or_length(&r, S5_OR_L_MEASURED, 50 + (i * 71) % 2100, S5_OR_R_NONE);
            print(&r);
        }
        else if ((i % 3) == 1)
        {
            s5_or_length(&r, (i & 4) ? S5_OR_L_PARTIAL : S5_OR_L_UNAVAILABLE, 30 + i * 11,
                         (s5_or_reason_t)(1 + (i % S5_OR_R_MAX)));
            print(&r);
        }
        s5_or_final(&r, (s5_or_reason_t)((i & 1) ? S5_OR_R_ABORTED : S5_OR_R_NONE));
        print(&r);
    }
    for (i = 0; i <= S5_OR_STAGE_MAX; i++) { printf("N stage %d %s\n", i, s5_or_stage_name((uint8_t)i)); }
    for (i = 0; i <= S5_OR_W_MAX; i++) { printf("N width %d %s\n", i, s5_or_width_name((uint8_t)i)); }
    for (i = 0; i <= S5_OR_L_MAX; i++) { printf("N length %d %s\n", i, s5_or_length_name((uint8_t)i)); }
    for (i = 1; i <= S5_OR_R_MAX; i++) { printf("N reason %d %s\n", i, s5_or_reason_name((uint8_t)i)); }
    return 0;
}

/*** end of file ***/
