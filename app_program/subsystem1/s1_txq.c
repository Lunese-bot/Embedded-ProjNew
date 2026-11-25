/*
 *      s1_txq.c
 *      Subsystem 1 -- delivery policy (see s1_txq.h).  Caller holds the
 *      S1 lock for every function here.
 */

#include <stddef.h>
#include "s1_txq.h"

static void item_copy(S1_TXQ_ITEM * p_dst, const S1_TXQ_ITEM * p_src)
{
    int32_t idx;

    for (idx = 0; idx < p_src->len; idx++)
    {
        p_dst->line[idx] = p_src->line[idx];
    }
    p_dst->len = p_src->len;
}

/* Drop the oldest waiting telemetry frame (counted once). */
static void tlm_drop_oldest(S1_TXQ * p_q, S1_STATS * p_st)
{
    p_q->tlm_head = (p_q->tlm_head + 1) % S1_TLM_QUEUE_MAX;
    p_q->tlm_count--;
    s1_stats_lost(p_st, S1_KIND_TLM);
}

/* Keep only the newest unsent telemetry frame.  A claimed staged frame is
   being published right now and is left alone. */
static void tlm_coalesce(S1_TXQ * p_q, S1_STATS * p_st)
{
    if ((p_q->tlm_count > 0) && p_q->has_tlm && (!p_q->tlm_claimed))
    {
        p_q->has_tlm = 0;
        s1_stats_lost(p_st, S1_KIND_TLM);
    }
    while (p_q->tlm_count > 1)
    {
        tlm_drop_oldest(p_q, p_st);
    }
}

static int32_t tlm_at_cap(const S1_TXQ * p_q, const S1_STATS * p_st)
{
    return (!p_q->connected) || (p_st->inflight_tlm >= S1_TLM_INFLIGHT_MAX);
}

void s1_txq_reset(S1_TXQ * p_q)
{
    uint8_t * p_byte = (uint8_t *)p_q;
    size_t    idx;

    for (idx = 0; idx < sizeof(*p_q); idx++)
    {
        p_byte[idx] = 0U;
    }
}

void s1_txq_post_tlm(S1_TXQ * p_q, S1_STATS * p_st, const uint8_t * p_line,
                     int32_t len)
{
    S1_TXQ_ITEM * p_slot;
    int32_t       idx;

    if (tlm_at_cap(p_q, p_st))
    {
        /* the new frame supersedes every older unsent one */
        if (p_q->has_tlm && (!p_q->tlm_claimed))
        {
            p_q->has_tlm = 0;
            s1_stats_lost(p_st, S1_KIND_TLM);
        }
        while (p_q->tlm_count > 0)
        {
            tlm_drop_oldest(p_q, p_st);
        }
    }
    else if (p_q->tlm_count >= S1_TLM_QUEUE_MAX)
    {
        tlm_drop_oldest(p_q, p_st);
    }
    p_slot = &p_q->tlm[(p_q->tlm_head + p_q->tlm_count) % S1_TLM_QUEUE_MAX];
    for (idx = 0; idx < len; idx++)
    {
        p_slot->line[idx] = p_line[idx];
    }
    p_slot->len = len;
    p_q->tlm_count++;
    p_st->tlm_posted++;
}

static int32_t stage_event(S1_TXQ * p_q, s1_txq_pull_fn pull)
{
    int32_t len;

    if (p_q->has_evt)
    {
        return 1;
    }
    len = pull(p_q->staged_evt.line);
    if (len <= 0)
    {
        return 0;
    }
    p_q->staged_evt.len = len;
    p_q->has_evt        = 1;
    if (p_q->evt_waiting > 0)
    {
        p_q->evt_waiting--;
    }
    return 1;
}

static void stage_tlm(S1_TXQ * p_q)
{
    if ((!p_q->has_tlm) && (p_q->tlm_count > 0))
    {
        item_copy(&p_q->staged_tlm, &p_q->tlm[p_q->tlm_head]);
        p_q->tlm_head = (p_q->tlm_head + 1) % S1_TLM_QUEUE_MAX;
        p_q->tlm_count--;
        p_q->has_tlm = 1;
    }
}

int32_t s1_txq_claim(S1_TXQ * p_q, S1_STATS * p_st, s1_txq_pull_fn pull,
                     S1_TXQ_CLAIM * p_claim)
{
    if (p_q->tlm_claimed || p_q->evt_claimed)
    {
        return 0; /* one publish at a time */
    }
    if (!p_q->connected)
    {
        tlm_coalesce(p_q, p_st);
        return 0;
    }
    if (stage_event(p_q, pull))
    {
        item_copy(&p_claim->item, &p_q->staged_evt);
        p_claim->kind =
            s1_stats_line_kind(p_q->staged_evt.line, p_q->staged_evt.len);
        p_q->evt_claimed = 1;
    }
    else if (tlm_at_cap(p_q, p_st))
    {
        tlm_coalesce(p_q, p_st);
        return 0;
    }
    else
    {
        stage_tlm(p_q);
        if (!p_q->has_tlm)
        {
            return 0;
        }
        item_copy(&p_claim->item, &p_q->staged_tlm);
        p_claim->kind    = S1_KIND_TLM;
        p_q->tlm_claimed = 1;
    }
    p_claim->gen = p_q->gen;
    return 1;
}

void s1_txq_finish(S1_TXQ * p_q, S1_STATS * p_st, const S1_TXQ_CLAIM * p_claim,
                   int32_t err)
{
    int32_t b_tlm = (p_claim->kind == S1_KIND_TLM);

    s1_stats_publish(p_st, p_claim->kind, err);
    if (b_tlm)
    {
        p_q->tlm_claimed = 0;
    }
    else
    {
        p_q->evt_claimed = 0;
    }
    if (err != S1_ERR_OK)
    {
        return; /* still staged: retried on the next drain */
    }
    if (p_claim->gen != p_q->gen)
    {
        /* The connection changed during the call: lwIP took it for a
           connection S1 already reconciled, so it can never complete. */
        p_st->unconfirmed++;
    }
    else if (b_tlm)
    {
        p_st->inflight_tlm++;
    }
    else
    {
        p_st->inflight_evt++;
    }
    if (b_tlm)
    {
        p_q->has_tlm = 0;
    }
    else
    {
        p_q->has_evt = 0;
    }
}

void s1_txq_drop_claim(S1_TXQ * p_q, S1_STATS * p_st,
                       const S1_TXQ_CLAIM * p_claim)
{
    if (p_claim->kind == S1_KIND_TLM)
    {
        p_q->tlm_claimed = 0;
        p_q->has_tlm     = 0;
    }
    else
    {
        p_q->evt_claimed = 0;
        p_q->has_evt     = 0;
    }
    s1_stats_lost(p_st, p_claim->kind);
}

void s1_txq_done(S1_TXQ * p_q, S1_STATS * p_st, int32_t kind, uint32_t gen,
                 int32_t err)
{
    uint32_t * p_inflight =
        (kind == S1_KIND_TLM) ? &p_st->inflight_tlm : &p_st->inflight_evt;

    if ((gen != p_q->gen) || (*p_inflight == 0U))
    {
        p_st->cb_stale++; /* old connection, or nothing outstanding */
        return;
    }
    (*p_inflight)--;
    if (err == S1_ERR_OK)
    {
        p_st->cb_ok++;
    }
    else
    {
        p_st->cb_err++;
    }
}

void s1_txq_set_connected(S1_TXQ * p_q, S1_STATS * p_st, int32_t connected)
{
    p_q->gen = (p_q->gen + 1U) & S1_TXQ_GEN_MASK;
    p_st->unconfirmed += p_st->inflight_tlm + p_st->inflight_evt;
    p_st->inflight_tlm = 0;
    p_st->inflight_evt = 0;
    p_q->connected     = (connected != 0);
    if (!p_q->connected)
    {
        tlm_coalesce(p_q, p_st);
    }
}

int32_t s1_txq_take(S1_TXQ * p_q, s1_txq_pull_fn pull, S1_TXQ_ITEM * p_item)
{
    if (p_q->tlm_claimed || p_q->evt_claimed)
    {
        return 0;
    }
    if (stage_event(p_q, pull))
    {
        item_copy(p_item, &p_q->staged_evt);
        p_q->has_evt = 0;
        return p_item->len;
    }
    stage_tlm(p_q);
    if (p_q->has_tlm)
    {
        item_copy(p_item, &p_q->staged_tlm);
        p_q->has_tlm = 0;
        return p_item->len;
    }
    return 0;
}

int32_t s1_txq_tlm_pending(const S1_TXQ * p_q)
{
    return p_q->tlm_count + (p_q->has_tlm ? 1 : 0);
}

/*** end of file ***/
