/*
 *      s1_stats.h
 *      Subsystem 1 -- MQTT delivery counters.
 *
 *      Plain C with no kernel calls, so the host tests use this file as it
 *      is.  Every counter is cumulative from power-on and counts each event
 *      exactly once.  Callers change or copy the counters only while they
 *      hold the S1 lock (s1_lock.h), so a snapshot is always coherent.
 *
 *      When each counter increments:
 *        tlm_posted   s1_post() put a "t" message into S1's queue.
 *        tlm_lost     a "t" message S1 had accepted was discarded before
 *                     mqtt_publish() returned ERR_OK for it: evicted from a
 *                     full queue, or (s1_post) refused even after eviction,
 *                     or popped and then refused by mqtt_publish().
 *        evt_posted   s1_post() put any other topic into S1's queue.
 *        evt_drop[]   the same as tlm_lost for every other topic, split by
 *                     the discarded message's own topic: ack, fault,
 *                     barcode, any other.
 *        pub_tlm      mqtt_publish() returned ERR_OK for a "t" message.
 *        pub_evt      mqtt_publish() returned ERR_OK for any other topic.
 *        ref_mem      mqtt_publish() returned ERR_MEM (no request slot or
 *                     no ring space).  One count per call, so retrying one
 *                     message counts once per attempt.
 *        ref_conn     mqtt_publish() returned ERR_CONN (TCP disconnected).
 *        ref_other    mqtt_publish() returned any other error.
 *        cb_ok        lwIP's completion callback reported ERR_OK for an S1
 *                     publish of the current connection.  For QoS 0 this
 *                     only means some TCP data was acknowledged by the
 *                     peer; it does not prove the broker or any subscriber
 *                     received this message.
 *        cb_err       the same callback reported an error (ERR_TIMEOUT
 *                     after MQTT_REQ_TIMEOUT seconds).
 *        cb_stale     a completion arrived for an earlier connection; it
 *                     changes no other counter.
 *        unconfirmed  S1 publishes that were accepted but had no completion
 *                     when the connection dropped (lwIP clears its request
 *                     list without calling the callbacks).
 *        inflight_tlm, inflight_evt
 *                     gauges, not totals: accepted S1 publishes of the
 *                     current connection still waiting for completion.
 *        hb_refused   mqtt_publish() refused the text heartbeat.
 *        lock_fail    the S1 lock could not be taken; the operation that
 *                     needed it was skipped.  Best effort (see s1_lock.h).
 */

#ifndef S1_STATS_H
#define S1_STATS_H

#include <stdint.h>

#define S1_EVT_ACK     0
#define S1_EVT_FAULT   1
#define S1_EVT_BARCODE 2
#define S1_EVT_OTHER   3
#define S1_EVT_KINDS   4

#define S1_KIND_TLM (-1) /* s1_stats_kind() result for topic "t" */

/* lwIP err_t values used for classification (lwip/err.h) */
#define S1_ERR_OK   0
#define S1_ERR_MEM  (-1)
#define S1_ERR_CONN (-11)

typedef struct
{
    uint32_t tlm_posted;
    uint32_t tlm_lost;
    uint32_t evt_posted;
    uint32_t evt_drop[S1_EVT_KINDS];
    uint32_t pub_tlm;
    uint32_t pub_evt;
    uint32_t ref_mem;
    uint32_t ref_conn;
    uint32_t ref_other;
    uint32_t cb_ok;
    uint32_t cb_err;
    uint32_t cb_stale;
    uint32_t unconfirmed;
    uint32_t inflight_tlm;
    uint32_t inflight_evt;
    uint32_t hb_refused;
    uint32_t lock_fail;
} S1_STATS;

/* One text line, NUL included, with every counter at 4294967295. */
#define S1_STATS_TEXT_MAX 288

/* Topic suffix (after S1_TOPIC_ROOT, or the full topic) -> S1_KIND_TLM or
   S1_EVT_x.  p_topic may be NULL (unknown message): S1_EVT_OTHER. */
int32_t s1_stats_kind(const char * p_topic);

/* Count one message discarded before lwIP accepted it. */
void s1_stats_lost(S1_STATS * p_stats, int32_t kind);

/* Count one mqtt_publish() call and its result. */
void s1_stats_publish(S1_STATS * p_stats, int32_t kind, int32_t err);

/* Write "s1 tq=.. tl=.. ..." into p_buf; never writes past p_buf[size-1],
   always NUL-terminates when size > 0.  Returns the length written (without
   the NUL); a line that did not fit is cut short. */
int32_t s1_stats_format(const S1_STATS * p_stats, char * p_buf, int32_t size);

#endif /* S1_STATS_H */

/*** end of file ***/
