/*
 *      s1_stats.c
 *      Subsystem 1 -- MQTT delivery counters (see s1_stats.h).
 */

#include <stddef.h>
#include "s1_stats.h"
#include "s1_bridge.h"
#include "../subsystem4/imu_terrain.h" /* IMU_SAFETY_* only */

#define S1_STATS_FAULT_TOPIC "fault" /* vc_ports.c posts faults here */

typedef struct
{
    char *  p_buf;
    int32_t size;
    int32_t len;
} s1_text_t;

static int32_t str_eq(const char * p_lhs, const char * p_rhs)
{
    while ((*p_lhs != '\0') && (*p_lhs == *p_rhs))
    {
        p_lhs++;
        p_rhs++;
    }
    return (*p_lhs == *p_rhs);
}

int32_t s1_stats_kind(const char * p_topic)
{
    const char * p_root = S1_TOPIC_ROOT;
    const char * p_rest = p_topic;

    if (p_topic == NULL)
    {
        return S1_EVT_OTHER;
    }
    while ((*p_root != '\0') && (*p_rest == *p_root))
    {
        p_root++;
        p_rest++;
    }
    if (*p_root != '\0')
    {
        p_rest = p_topic; /* no root prefix: already a suffix */
    }
    if (str_eq(p_rest, S1_TOPIC_TLM))
    {
        return S1_KIND_TLM;
    }
    if (str_eq(p_rest, S1_TOPIC_ACK))
    {
        return S1_EVT_ACK;
    }
    if (str_eq(p_rest, S1_STATS_FAULT_TOPIC))
    {
        return S1_EVT_FAULT;
    }
    if (str_eq(p_rest, S1_TOPIC_BARCODE))
    {
        return S1_EVT_BARCODE;
    }
    return S1_EVT_OTHER;
}

int32_t s1_stats_line_kind(const uint8_t * p_line, int32_t len)
{
    int32_t idx;

    for (idx = 0; idx < len; idx++)
    {
        if (p_line[idx] == 0U)
        {
            return s1_stats_kind((const char *)p_line);
        }
    }
    return S1_EVT_OTHER;
}

void s1_stats_lost(S1_STATS * p_stats, int32_t kind)
{
    if (kind == S1_KIND_TLM)
    {
        p_stats->tlm_lost++;
    }
    else if ((kind >= 0) && (kind < S1_EVT_KINDS))
    {
        p_stats->evt_drop[kind]++;
    }
    else
    {
        p_stats->evt_drop[S1_EVT_OTHER]++;
    }
}

void s1_stats_publish(S1_STATS * p_stats, int32_t kind, int32_t err)
{
    if (err == S1_ERR_OK)
    {
        if (kind == S1_KIND_TLM)
        {
            p_stats->pub_tlm++;
        }
        else
        {
            p_stats->pub_evt++;
        }
    }
    else if (err == S1_ERR_MEM)
    {
        p_stats->ref_mem++;
    }
    else if (err == S1_ERR_CONN)
    {
        p_stats->ref_conn++;
    }
    else
    {
        p_stats->ref_other++;
    }
}

static void text_char(s1_text_t * p_text, char ch)
{
    if (p_text->len < (p_text->size - 1))
    {
        p_text->p_buf[p_text->len] = ch;
        p_text->len++;
    }
}

static void text_str(s1_text_t * p_text, const char * p_str)
{
    while (*p_str != '\0')
    {
        text_char(p_text, *p_str);
        p_str++;
    }
}

static void text_u32(s1_text_t * p_text, uint32_t value)
{
    char    digits[10];
    int32_t count = 0;

    do
    {
        digits[count] = (char)('0' + (value % 10U));
        count++;
        value /= 10U;
    } while ((value != 0U) && (count < (int32_t)sizeof(digits)));
    while (count > 0)
    {
        count--;
        text_char(p_text, digits[count]);
    }
}

/* " name=a/b/c" */
static void text_field(s1_text_t * p_text, const char * p_name,
                       const uint32_t * p_values, int32_t count)
{
    int32_t idx;

    text_char(p_text, ' ');
    text_str(p_text, p_name);
    text_char(p_text, '=');
    for (idx = 0; idx < count; idx++)
    {
        if (idx > 0)
        {
            text_char(p_text, '/');
        }
        text_u32(p_text, p_values[idx]);
    }
}

int32_t s1_stats_format(const S1_STATS * p_stats, char * p_buf, int32_t size)
{
    s1_text_t text;
    uint32_t  v[3];

    if ((p_buf == NULL) || (size <= 0))
    {
        return 0;
    }
    text.p_buf = p_buf;
    text.size  = size;
    text.len   = 0;
    if (p_stats != NULL)
    {
        text_str(&text, "s1");
        text_field(&text, "tq", &p_stats->tlm_posted, 1);
        text_field(&text, "tl", &p_stats->tlm_lost, 1);
        text_field(&text, "eq", &p_stats->evt_posted, 1);
        text_field(&text, "ed", p_stats->evt_drop, S1_EVT_KINDS);
        v[0] = p_stats->pub_tlm;
        v[1] = p_stats->pub_evt;
        text_field(&text, "pa", v, 2);
        v[0] = p_stats->ref_mem;
        v[1] = p_stats->ref_conn;
        v[2] = p_stats->ref_other;
        text_field(&text, "rf", v, 3);
        v[0] = p_stats->cb_ok;
        v[1] = p_stats->cb_err;
        v[2] = p_stats->cb_stale;
        text_field(&text, "cb", v, 3);
        text_field(&text, "uc", &p_stats->unconfirmed, 1);
        v[0] = p_stats->inflight_tlm;
        v[1] = p_stats->inflight_evt;
        text_field(&text, "if", v, 2);
        text_field(&text, "hr", &p_stats->hb_refused, 1);
        text_field(&text, "lf", &p_stats->lock_fail, 1);
    }
    p_buf[text.len] = '\0';
    return text.len;
}

void s1_latch_resolve(S1_LATCH * p_out, uint32_t stop_before, int32_t status_ok,
                      uint32_t safety_stop, uint32_t stop_after)
{
    if (p_out == NULL)
    {
        return;
    }
    p_out->active = (stop_after != 0U) ? 1 : 0;
    p_out->cause  = S1_LATCH_CAUSE_NONE;
    if (p_out->active == 0)
    {
        return; /* clear now: an old cause is never reported */
    }
    if ((stop_before == 0U) || (status_ok == 0))
    {
        p_out->cause = S1_LATCH_CAUSE_UNKNOWN; /* changed while read */
    }
    else if (safety_stop == IMU_SAFETY_LIFT)
    {
        p_out->cause = S1_LATCH_CAUSE_LIFT;
    }
    else if (safety_stop == IMU_SAFETY_OVERTURN)
    {
        p_out->cause = S1_LATCH_CAUSE_OVERTURN;
    }
    else
    {
        p_out->cause = S1_LATCH_CAUSE_UNKNOWN; /* not yet published */
    }
}

int32_t s1_latch_format(const S1_LATCH * p_latch, char * p_buf, int32_t size)
{
    const char * p_text;
    int32_t      len = 0;

    if ((p_latch == NULL) || (p_buf == NULL) || (size <= 0))
    {
        return 0;
    }
    if (p_latch->active == 0)
    {
        p_text = " il=0";
    }
    else if (p_latch->cause == S1_LATCH_CAUSE_LIFT)
    {
        p_text = " il=1 ic=lift";
    }
    else if (p_latch->cause == S1_LATCH_CAUSE_OVERTURN)
    {
        p_text = " il=1 ic=overturn";
    }
    else
    {
        p_text = " il=1 ic=unknown";
    }
    while (p_text[len] != '\0')
    {
        len++;
    }
    if (len >= size)
    {
        p_buf[0] = '\0';
        return 0;
    }
    for (int32_t idx = 0; idx <= len; idx++)
    {
        p_buf[idx] = p_text[idx];
    }
    return len;
}

/*** end of file ***/
