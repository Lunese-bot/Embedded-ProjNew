/*
 *      s1_obst_pub.h
 *      Subsystem 1 -- when to publish S5's obstacle size report
 *      (s5_obst_report.h) on topic robot/g10/obstacle/profile.
 *
 *      Plain C with no kernel calls (host-tested in testing/subsystem1).
 *      telemetry_task polls S5's record every 200 ms and asks this helper
 *      whether it is due; S5 itself never waits for MQTT.
 *
 *      Due when MQTT is up, an encounter exists (profile_id != 0) and
 *        - the record differs from the last one sent (t0_ms, profile_id,
 *          rev), or
 *        - MQTT came back up since the last send (the dashboard may have
 *          missed it; it drops the repeat by the same key).
 *      Nothing is due while MQTT is down: no queue slot is spent on a frame
 *      nobody can receive, and the latest record goes out on reconnect.
 *      A refused post (queue short of room) is simply due again next period.
 */

#ifndef S1_OBST_PUB_H
#define S1_OBST_PUB_H

#include <stdint.h>
#include <stdbool.h>
#include "../subsystem5/s5_obst_report.h"

/* Event-queue slots the profile always leaves free for acks and faults
   (s1_post_spare). */
#define S1_OBST_SPARE_SLOTS 3

typedef struct
{
    bool     b_sent;     /* something was sent on the current connection */
    bool     b_was_up;   /* MQTT state seen at the previous poll          */
    uint16_t profile_id; /* key of the last record sent                   */
    uint8_t  rev;
    uint32_t t0_ms;
} s1_obst_pub_t;

void s1_obst_pub_reset(s1_obst_pub_t * p_pub);

/* Call once per poll with S5's current record and the MQTT state. */
bool s1_obst_pub_due(s1_obst_pub_t * p_pub, const s5_obst_report_t * p_rec,
                     bool b_mqtt_up);

/* The record was queued: remember its key. */
void s1_obst_pub_sent(s1_obst_pub_t * p_pub, const s5_obst_report_t * p_rec);

#endif /* S1_OBST_PUB_H */

/*** end of file ***/
