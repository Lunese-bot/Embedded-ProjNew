/*
 *      s1_stats.h
 *      Subsystem 1 -- MQTT delivery counters.
 *
 *      Plain C with no kernel calls, so the host tests use this file as it
 *      is.  Every counter is cumulative from power-on and counts each event
 *      exactly once.  Callers change or copy the counters only while they
 *      hold the S1 lock (s1_lock.h), so a snapshot is always coherent.
 *
 *      When each counter increments (this build; the stage-1 counters-only
 *      build counted a popped-then-refused message as lost):
 *        tlm_posted   s1_post() put a "t" message into S1's queue.
 *        tlm_lost     a "t" message S1 had accepted was discarded before
 *                     mqtt_publish() returned ERR_OK for it, by one of the
 *                     overflow/coalescing rules in s1_txq.h (ring full, or
 *                     superseded by a newer frame while disconnected or at
 *                     S1_TLM_INFLIGHT_MAX).  A refused publish is not a loss:
 *                     the message stays in S1 and is retried.
 *        evt_posted   s1_post() put any other topic into S1's queue.
 *        evt_drop[]   an event discarded before lwIP accepted it, split by
 *                     the discarded message's own topic (ack, fault,
 *                     barcode, any other): evicted as the oldest waiting
 *                     event when S1_EVT_QUEUE_MAX were already waiting, or
 *                     refused by the kernel buffer itself.
 *        pub_tlm      mqtt_publish() returned ERR_OK for a "t" message.
 *        pub_evt      mqtt_publish() returned ERR_OK for any other topic.
 *        ref_mem      mqtt_publish() returned ERR_MEM (no request slot or
 *                     no ring space).  One count per call: one message
 *                     refused five times counts five here and no loss.
 *        ref_conn     mqtt_publish() returned ERR_CONN (TCP disconnected).
 *        ref_other    mqtt_publish() returned any other error.
 *        cb_ok        lwIP's completion callback reported ERR_OK for an S1
 *                     publish of the current connection.  For QoS 0 this
 *                     only means some TCP data was acknowledged by the
 *                     peer; it does not prove the broker or any subscriber
 *                     received this message.
 *        cb_err       the same callback reported an error (ERR_TIMEOUT
 *                     after MQTT_REQ_TIMEOUT seconds).
 *        cb_stale     a completion arrived for an earlier connection, or
 *                     with nothing of its kind in flight; it changes no
 *                     other counter.
 *        unconfirmed  S1 publishes that were accepted but had no completion
 *                     when the connection dropped (lwIP clears its request
 *                     list without calling the callbacks), or that lwIP
 *                     accepted while the connection was changing.
 *        inflight_tlm, inflight_evt
 *                     gauges, not totals: accepted S1 publishes of the
 *                     current connection still waiting for completion.
 *        hb_refused   mqtt_publish() refused the text heartbeat.
 *        lock_fail    the S1 lock could not be taken within S1_LOCK_TMO_MS;
 *                     the operation that needed it was skipped.  In
 *                     s1_post() that means the message was not queued
 *                     (E_TMOUT), so a lost frame may show only here.
 *                     Best effort (see s1_lock.h).
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

/* Kind of a queued line "topic\0payload" (S1_EVT_OTHER if it has no
   NUL-terminated topic). */
int32_t s1_stats_line_kind(const uint8_t * p_line, int32_t len);

/* Count one message discarded before lwIP accepted it. */
void s1_stats_lost(S1_STATS * p_stats, int32_t kind);

/* Count one mqtt_publish() call and its result. */
void s1_stats_publish(S1_STATS * p_stats, int32_t kind, int32_t err);

/* Write "s1 tq=.. tl=.. ..." into p_buf; never writes past p_buf[size-1],
   always NUL-terminates when size > 0.  Returns the length written (without
   the NUL); a line that did not fit is cut short. */
int32_t s1_stats_format(const S1_STATS * p_stats, char * p_buf, int32_t size);

/* ---------------------------------------------------------------------
 *  IMU stop interlock, appended to the counters line for the operator
 *  dashboard (owner, 2026-11-24).  Display only: S4 and S2 stop the motors
 *  and refuse Reset/Start on their own, whatever this text says or whether
 *  any heartbeat is sent.  The 14-byte telemetry frame is unchanged (v1).
 *
 *    " il=0"                 interlock clear (g_s2_imu_stop == 0)
 *    " il=1 ic=lift"         interlock set, S4 reports IMU_SAFETY_LIFT
 *    " il=1 ic=overturn"     interlock set, S4 reports IMU_SAFETY_OVERTURN
 *    " il=1 ic=unknown"      interlock set, cause not readable coherently
 *                            (S4 status lock timed out, the interlock
 *                            changed while it was read, or S4 had not yet
 *                            published the cause that set it)
 *
 *  Only S4's safety_stop is read.  The tilt-limit warning (IMU_EV_TILT_LIMIT,
 *  S4_FC_TILT) never sets the interlock and never shows as a cause.  Builds
 *  without this code send no il= field; the dashboard then shows "Latch
 *  status unavailable".
 * --------------------------------------------------------------------- */
#define S1_LATCH_CAUSE_NONE     0
#define S1_LATCH_CAUSE_LIFT     1
#define S1_LATCH_CAUSE_OVERTURN 2
#define S1_LATCH_CAUSE_UNKNOWN  3

/* Longest text, without the NUL: " il=1 ic=overturn" */
#define S1_LATCH_TEXT_MAX 17

typedef struct
{
    int32_t active; /* 0 or 1: the S2 motor interlock as read the 2nd time */
    int32_t cause;  /* S1_LATCH_CAUSE_*; NONE whenever active == 0 */
} S1_LATCH;

/* Combine two reads of the S2 interlock taken before and after one read of
   S4's status (status_ok == 0 if that read failed) into one coherent
   value.  safety_stop is imu_status_t.safety_stop (IMU_SAFETY_*). */
void s1_latch_resolve(S1_LATCH * p_out, uint32_t stop_before, int32_t status_ok,
                      uint32_t safety_stop, uint32_t stop_after);

/* Write the text above into p_buf; same bounds rules as s1_stats_format.
   Returns the length written (0 if it did not fit: never a cut field). */
int32_t s1_latch_format(const S1_LATCH * p_latch, char * p_buf, int32_t size);

/* Read the live interlock and cause (s1_latch_port.c on the target; the
   host tests supply their own). */
void s1_latch_read(S1_LATCH * p_out);

#endif /* S1_STATS_H */

/*** end of file ***/
