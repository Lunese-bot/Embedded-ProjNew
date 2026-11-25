/*
 *      s5_obst_report.c
 *      Subsystem 5 -- obstacle size record and its wire frame.
 *      Layout and meaning: s5_obst_report.h.
 */

#include "s5_obst_report.h"

#include <stddef.h>

static uint16_t clamp_u16(int32_t value, int32_t max)
{
    if (value < 0)
    {
        return 0U;
    }
    /* Cast: clamped to max, which is at most 4000. */
    return (uint16_t)((value > max) ? max : value);
}

static void touch(s5_obst_report_t * p_rec)
{
    p_rec->rev++; /* wraps at 255; receivers compare within one id */
}

uint16_t s5_or_next_id(uint16_t prev)
{
    return (prev >= 65535U) ? 1U : (uint16_t)(prev + 1U);
}

void s5_or_begin(s5_obst_report_t * p_rec, uint16_t obj_id, uint32_t t0_ms)
{
    *p_rec            = (s5_obst_report_t){ 0 };
    p_rec->profile_id = (0U == obj_id) ? 1U : obj_id;
    p_rec->t0_ms      = t0_ms;
    p_rec->stage      = (uint8_t)S5_OR_STAGE_SCANNING;
}

void s5_or_front(s5_obst_report_t * p_rec, uint8_t valid, uint16_t distance_mm,
                 int32_t right_edge_deg, int32_t left_edge_deg,
                 uint16_t width_mm, uint8_t bypass_side, int32_t scan_min_deg,
                 int32_t scan_max_deg, bool b_length_follows,
                 s5_or_reason_t no_length_reason)
{
    p_rec->stage       = (uint8_t)S5_OR_STAGE_FRONT;
    p_rec->bypass_side = bypass_side;
    if (0U == valid)
    {
        p_rec->width_status = (uint8_t)S5_OR_W_NONE;
        p_rec->width_mm     = 0U;
        p_rec->front_mm     = 0U;
    }
    else
    {
        p_rec->front_mm = clamp_u16(distance_mm, S5_OR_FRONT_MAX_MM);
        p_rec->width_mm = clamp_u16(width_mm, S5_OR_WIDTH_MAX_MM);
        if (right_edge_deg == left_edge_deg)
        {
            /* one angle only: the edges lie somewhere between this and
               the neighbouring angles, so no width is claimed */
            p_rec->width_status = (uint8_t)S5_OR_W_ONE_BEAM;
            p_rec->width_mm     = 0U;
        }
        else if ((right_edge_deg <= scan_min_deg) ||
                 (left_edge_deg >= scan_max_deg))
        {
            p_rec->width_status = (uint8_t)S5_OR_W_LOWER_BOUND;
        }
        else
        {
            p_rec->width_status = (uint8_t)S5_OR_W_MEASURED;
        }
    }
    if (b_length_follows && (0U != valid))
    {
        p_rec->length_status = (uint8_t)S5_OR_L_MEASURING;
        p_rec->length_reason = (uint8_t)S5_OR_R_NONE;
    }
    else
    {
        p_rec->length_status = (uint8_t)S5_OR_L_UNAVAILABLE;
        p_rec->length_reason =
            (uint8_t)((S5_OR_R_NONE == no_length_reason) ? S5_OR_R_NO_BYPASS
                                                         : no_length_reason);
    }
    p_rec->length_mm = 0U;
    touch(p_rec);
}

void s5_or_length(s5_obst_report_t * p_rec, s5_or_length_t status,
                  int32_t length_mm, s5_or_reason_t reason)
{
    p_rec->stage = (uint8_t)S5_OR_STAGE_LENGTH;
    if ((S5_OR_L_MEASURED == status) || (S5_OR_L_PARTIAL == status))
    {
        p_rec->length_status = (uint8_t)status;
        p_rec->length_mm     = clamp_u16(length_mm, S5_OR_LENGTH_MAX_MM);
        p_rec->length_reason = (uint8_t)S5_OR_R_NONE;
    }
    else
    {
        p_rec->length_status = (uint8_t)S5_OR_L_UNAVAILABLE;
        p_rec->length_mm     = 0U;
        p_rec->length_reason =
            (uint8_t)((S5_OR_R_NONE == reason) ? S5_OR_R_NO_SIDE_ECHO : reason);
    }
    touch(p_rec);
}

void s5_or_final(s5_obst_report_t * p_rec, s5_or_reason_t why_unresolved)
{
    p_rec->stage = (uint8_t)S5_OR_STAGE_FINAL;
    if ((S5_OR_L_PENDING == p_rec->length_status) ||
        (S5_OR_L_MEASURING == p_rec->length_status))
    {
        p_rec->length_status = (uint8_t)S5_OR_L_UNAVAILABLE;
        p_rec->length_mm     = 0U;
        p_rec->length_reason =
            (uint8_t)((S5_OR_R_NONE == why_unresolved) ? S5_OR_R_ABORTED
                                                       : why_unresolved);
    }
    if (S5_OR_W_PENDING == p_rec->width_status)
    {
        p_rec->width_status = (uint8_t)S5_OR_W_NONE;
    }
    touch(p_rec);
}

static void put16(uint8_t * p_val, uint16_t value2)
{
    p_val[0] = (uint8_t)(value2 >> 8);
    p_val[1] = (uint8_t)(value2 & 0xFFU);
}

static uint16_t get16(const uint8_t * p_val)
{
    return (uint16_t)(((uint16_t)p_val[0] << 8) | p_val[1]);
}

void s5_or_encode(const s5_obst_report_t * p_rec, uint8_t p_output[S5_OR_BYTES])
{
    p_output[0] = (uint8_t)S5_OR_VERSION;
    put16(&p_output[1], p_rec->profile_id);
    p_output[3] = p_rec->rev;
    p_output[4] = p_rec->stage;
    p_output[5] = p_rec->width_status;
    p_output[6] = p_rec->length_status;
    p_output[7] = p_rec->length_reason;
    put16(&p_output[8], p_rec->front_mm);
    put16(&p_output[10], p_rec->width_mm);
    put16(&p_output[12], p_rec->length_mm);
    p_output[14] = (uint8_t)(p_rec->t0_ms >> 24);
    p_output[15] = (uint8_t)(p_rec->t0_ms >> 16);
    p_output[16] = (uint8_t)(p_rec->t0_ms >> 8);
    p_output[17] = (uint8_t)(p_rec->t0_ms & 0xFFU);
    p_output[18] = p_rec->bypass_side;
    p_output[19] = 0U;
}

bool s5_or_decode(const uint8_t * p_in, int32_t len, s5_obst_report_t * p_rec)
{
    s5_obst_report_t r_val = { 0 };

    if ((NULL == p_in) || (NULL == p_rec) || (S5_OR_BYTES != len) ||
        (S5_OR_VERSION != p_in[0]))
    {
        return false;
    }
    r_val.profile_id    = get16(&p_in[1]);
    r_val.rev           = p_in[3];
    r_val.stage         = p_in[4];
    r_val.width_status  = p_in[5];
    r_val.length_status = p_in[6];
    r_val.length_reason = p_in[7];
    r_val.front_mm      = get16(&p_in[8]);
    r_val.width_mm      = get16(&p_in[10]);
    r_val.length_mm     = get16(&p_in[12]);
    r_val.t0_ms = ((uint32_t)p_in[14] << 24) | ((uint32_t)p_in[15] << 16) |
                  ((uint32_t)p_in[16] << 8) | (uint32_t)p_in[17];
    r_val.bypass_side = p_in[18];
    if ((0U == r_val.profile_id) || (r_val.stage > S5_OR_STAGE_MAX) ||
        (r_val.width_status > S5_OR_W_MAX) ||
        (r_val.length_status > S5_OR_L_MAX) ||
        (r_val.length_reason > S5_OR_R_MAX) || (r_val.bypass_side > 2U) ||
        (r_val.front_mm > S5_OR_FRONT_MAX_MM) ||
        (r_val.width_mm > S5_OR_WIDTH_MAX_MM) ||
        (r_val.length_mm > S5_OR_LENGTH_MAX_MM) || (0U != p_in[19]))
    {
        return false;
    }
    *p_rec = r_val;
    return true;
}

const char * s5_or_stage_name(uint8_t stage)
{
    static const char * const names[] = { "scanning", "front", "length",
                                          "final" };

    return (stage <= S5_OR_STAGE_MAX) ? names[stage] : "?";
}

const char * s5_or_width_name(uint8_t status)
{
    static const char * const names[] = { "pending", "measured", "lower bound",
                                          "one beam", "none" };

    return (status <= S5_OR_W_MAX) ? names[status] : "?";
}

const char * s5_or_length_name(uint8_t status)
{
    static const char * const names[] = { "pending", "measuring", "measured",
                                          "partial", "unavailable" };

    return (status <= S5_OR_L_MAX) ? names[status] : "?";
}

const char * s5_or_reason_name(uint8_t reason)
{
    static const char * const names[] = { "-",
                                          "side look off in this build",
                                          "no bypass",
                                          "no side echo during the pass",
                                          "rear edge not seen before turn-in",
                                          "side sensor fault",
                                          "side look gave no reading",
                                          "aborted",
                                          "bypass failed",
                                          "look angle unsuitable" };

    return (reason <= S5_OR_R_MAX) ? names[reason] : "?";
}

/*** end of file ***/
