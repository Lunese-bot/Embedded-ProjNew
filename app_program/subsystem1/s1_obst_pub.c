/*
 *      s1_obst_pub.c
 *      Subsystem 1 -- publish decision for the obstacle size report.
 *      Rules: s1_obst_pub.h.
 */

#include "s1_obst_pub.h"

void s1_obst_pub_reset(S1_OBST_PUB * p_pub)
{
    *p_pub = (S1_OBST_PUB){ 0 };
}

bool s1_obst_pub_due(S1_OBST_PUB * p_pub, const s5_obst_report_t * p_rec,
                     bool b_mqtt_up)
{
    bool b_reconnected = b_mqtt_up && (!p_pub->b_was_up);

    p_pub->b_was_up = b_mqtt_up;
    if (b_reconnected)
    {
        p_pub->b_sent = false; /* resend the latest on the new connection */
    }
    if ((!b_mqtt_up) || (0U == p_rec->profile_id))
    {
        return false;
    }
    return (!p_pub->b_sent) || (p_rec->profile_id != p_pub->profile_id) ||
           (p_rec->rev != p_pub->rev) || (p_rec->t0_ms != p_pub->t0_ms);
}

void s1_obst_pub_sent(S1_OBST_PUB * p_pub, const s5_obst_report_t * p_rec)
{
    p_pub->b_sent     = true;
    p_pub->profile_id = p_rec->profile_id;
    p_pub->rev        = p_rec->rev;
    p_pub->t0_ms      = p_rec->t0_ms;
}

/*** end of file ***/
