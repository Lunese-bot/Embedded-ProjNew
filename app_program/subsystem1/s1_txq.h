/*
 *      s1_txq.h
 *      Subsystem 1 -- delivery policy between s1_post() and mqtt_publish().
 *
 *      Plain C with no kernel calls (host-tested as is).  Every function
 *      here is called with the S1 lock held (s1_lock.h); none of them calls
 *      lwIP.  s1_bridge.c wraps them for producers and for the WiFi service
 *      task.
 *
 *      Two classes of message, by topic:
 *        telemetry  topic "t"
 *        event      every other topic: ack, fault, barcode, and anything
 *                   else an existing caller posts (for example "selftest")
 *
 *      Storage, capacity and order (one item = one s1_post() call):
 *        events     the T-Kernel message buffer g_mbf_telemetry, FIFO, at
 *                   most S1_EVT_QUEUE_MAX waiting, plus one staged event
 *                   (taken from the buffer, not yet accepted by lwIP).
 *        telemetry  a ring of S1_TLM_QUEUE_MAX waiting frames, FIFO, plus
 *                   one staged frame.
 *        A staged item is the oldest of its class.  It stays owned by S1
 *        until mqtt_publish() returns ERR_OK for it; on any refusal it
 *        is kept and offered again on the next drain (each attempt is one
 *        refusal, never a drop).  While it is being published ("claimed")
 *        nothing else may remove it.
 *
 *      Publish order: the staged/oldest event first, always.  Telemetry
 *      only when no event is waiting and fewer than S1_TLM_INFLIGHT_MAX
 *      telemetry publishes are still waiting for lwIP's completion.
 *
 *      Overflow and coalescing, the only ways S1 discards a message:
 *        - event buffer full: the oldest waiting event (never the staged
 *          one) is evicted and counted under its own topic.  Telemetry never
 *          evicts an event.
 *        - telemetry ring full while connected and under the cap: the
 *          oldest waiting frame is evicted (counted in tl).
 *        - disconnected, or S1_TLM_INFLIGHT_MAX telemetry in flight: only
 *          the newest unsent frame is kept; every older unsent frame,
 *          including an unclaimed staged one, is counted once in tl.
 */

#ifndef S1_TXQ_H
#define S1_TXQ_H

#include <stdint.h>
#include "s1_bridge.h"
#include "s1_stats.h"

#define S1_TLM_QUEUE_MAX 4 /* waiting telemetry frames under the cap */
#define S1_TXQ_GEN_MASK  0x0FFFFFFFU /* generation bits kept in a tag */

typedef struct
{
    uint8_t line[S1_BRIDGE_LINE_MAX]; /* "topic\0payload" as queued */
    int32_t len;
} S1_TXQ_ITEM;

/* Takes the oldest event from the event buffer into p_line;
   returns its length, 0 when empty.  Called with the S1 lock held. */
typedef int32_t (*s1_txq_pull_fn)(uint8_t * p_line);

typedef struct
{
    S1_TXQ_ITEM tlm[S1_TLM_QUEUE_MAX];
    int32_t     tlm_head;
    int32_t     tlm_count;
    S1_TXQ_ITEM staged_tlm;
    S1_TXQ_ITEM staged_evt;
    int32_t     has_tlm;
    int32_t     has_evt;
    int32_t     tlm_claimed;
    int32_t     evt_claimed;
    int32_t     evt_waiting; /* events in the buffer, not staged */
    int32_t     connected;
    uint32_t    gen; /* connection generation */
} S1_TXQ;

/* A claimed item, copied out so mqtt_publish() runs without the lock. */
typedef struct
{
    S1_TXQ_ITEM item;
    int32_t     kind; /* S1_KIND_TLM or S1_EVT_x */
    uint32_t    gen;
} S1_TXQ_CLAIM;

void s1_txq_reset(S1_TXQ * p_q);

/* Queue one telemetry line (topic "t"). */
void s1_txq_post_tlm(S1_TXQ * p_q, S1_STATS * p_st, const uint8_t * p_line,
                     int32_t len);

/* Claim the next item to publish: 1 and *p_claim filled, or 0 when nothing
   may be published now (empty, disconnected, telemetry at its cap, or an
   item already claimed). */
int32_t s1_txq_claim(S1_TXQ * p_q, S1_STATS * p_st, s1_txq_pull_fn pull,
                     S1_TXQ_CLAIM * p_claim);

/* mqtt_publish() returned err for the claimed item: on ERR_OK the item is
   committed exactly once (removed, counted, in flight); otherwise it stays
   staged and the refusal is counted. */
void s1_txq_finish(S1_TXQ * p_q, S1_STATS * p_st, const S1_TXQ_CLAIM * p_claim,
                   int32_t err);

/* The claimed item cannot be published at all (malformed): remove it and
   count it lost under its own topic. */
void s1_txq_drop_claim(S1_TXQ * p_q, S1_STATS * p_st,
                       const S1_TXQ_CLAIM * p_claim);

/* Completion callback for an accepted publish of generation gen. */
void s1_txq_done(S1_TXQ * p_q, S1_STATS * p_st, int32_t kind, uint32_t gen,
                 int32_t err);

/* Connection change: new generation, in-flight -> unconfirmed. */
void s1_txq_set_connected(S1_TXQ * p_q, S1_STATS * p_st, int32_t connected);

/* Next item in publish order, removed at once without publishing (the old
   s1_bridge_pop() behaviour).  Returns its length, 0 when empty. */
int32_t s1_txq_take(S1_TXQ * p_q, s1_txq_pull_fn pull, S1_TXQ_ITEM * p_item);

/* Unsent telemetry frames held (staged + waiting). */
int32_t s1_txq_tlm_pending(const S1_TXQ * p_q);

#endif /* S1_TXQ_H */

/*** end of file ***/
