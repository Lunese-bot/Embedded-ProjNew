/*
 *      task_obstacle_avoid.c
 *      Subsystem 5 -- Obstacle Avoidance task.
 *
 *      Receives s5_cmd_t from the Vehicle Controller on g_mbf_s5_cmd.  On
 *      START_AVOID (the controller has stopped the car):
 *        1. ask the scan task for a profile        (S5_FLG_SCAN_REQ)
 *        2. choose a side and plan the bypass      (obstacle_logic.c)
 *        3. send each step as S5_EVT_MOTION_REQ and wait for its
 *           S5_CMD_MOTION_DONE; "search" steps stop early when the line
 *           is seen (s5_port_line_detected)
 *        4. line found  -> turn back to the original heading,
 *                          S5_EVT_LINE_REACQUIRED
 *           not found   -> up to S5_MAX_RECOVERY wider searches, then
 *                          S5_EVT_AVOID_FAILED (the controller decides:
 *                          retry, or Safe Stop, FR8)
 *        5. tell the scan task to go back to monitoring (S5_FLG_RESUME)
 *      ABORT or DISABLE at any point ends the manoeuvre at once.
 */

#include "s5_local.h"

typedef enum
{
    STEP_DONE,    /* motion finished                 */
    STEP_LINE,    /* search step stopped on the line */
    STEP_FAILED,  /* controller reported failure     */
    STEP_TIMEOUT, /* no MOTION_DONE in time          */
    STEP_ABORT    /* ABORT / DISABLE received        */
} step_result_t;

static uint16_t g_seq_next = 1;
static int32_t  g_heading_deg; /* sum of completed turns since START */

#define S5_DEFAULT_SPEED_MM_S 100 /* used when a move gives no speed */
#define S5_MS_PER_S           1000U

static TMO step_timeout_ms(const s5_motion_req_t * p_req)
{
    uint32_t spd;
    TMO      tmo;

    if ((S5_MOVE_FORWARD == p_req->kind) || (S5_MOVE_REVERSE == p_req->kind))
    {
        spd =
            (p_req->speed_mm_s > 0) ? p_req->speed_mm_s : S5_DEFAULT_SPEED_MM_S;
        /* Cast: a bypass leg takes seconds, well inside TMO. */
        tmo =
            (TMO)((p_req->distance_mm * S5_MS_PER_S) / spd) + S5_MOVE_SLACK_MS;
    }
    else
    {
        tmo = S5_TURN_TMO_MS;
    }
    return tmo;
}

/* Handle a command that arrives while we are busy.  Returns true if it
   means "stop the manoeuvre". */
static bool busy_command(const s5_cmd_t * p_cmd)
{
    bool b_stop;

    switch (p_cmd->type)
    {
        case S5_CMD_ABORT:
            b_stop = true;
            break;
        case S5_CMD_DISABLE:
            /* Cast: clear every flag bit except ENABLED. */
            tk_clr_flg(g_flg_s5, ~(uint32_t)S5_FLG_ENABLED);
            b_stop = true;
            break;
        case S5_CMD_ENABLE:
            tk_set_flg(g_flg_s5, S5_FLG_ENABLED);
            b_stop = false;
            break;
        default: /* stale MOTION_DONE, repeated START_AVOID */
            b_stop = false;
            break;
    }
    return b_stop;
}

static uint16_t send_motion(const s5_motion_req_t * p_req)
{
    s5_evt_t evt = { 0 };

    evt.type         = S5_EVT_MOTION_REQ;
    evt.u.motion     = *p_req;
    evt.u.motion.seq = g_seq_next;
    g_seq_next++;
    s5_send_evt(&evt);
    return evt.u.motion.seq;
}

/* A command arrived while a step runs: is it the end of that step? */
static bool step_command(const s5_step_t * p_step, const s5_cmd_t * p_cmd,
                         uint16_t seq, step_result_t * p_result)
{
    bool b_finished = true;

    if ((S5_CMD_MOTION_DONE == p_cmd->type) && (p_cmd->seq == seq))
    {
        if (!p_cmd->ok)
        {
            *p_result = STEP_FAILED;
        }
        else
        {
            if (S5_MOVE_TURN == p_step->m.kind)
            {
                g_heading_deg += p_step->m.angle_deg;
            }
            /* a search that ran its full length may still
               have ended on the line */
            if ((0U != p_step->search_line) && (s5_port_line_detected()))
            {
                *p_result = STEP_LINE;
            }
            else
            {
                *p_result = STEP_DONE;
            }
        }
    }
    else if (busy_command(p_cmd))
    {
        *p_result = STEP_ABORT;
    }
    else
    {
        b_finished = false; /* unrelated command: keep waiting */
    }
    return b_finished;
}

#if S5_BENCH_MODE
/* Bench mode: print the move, pretend it takes a while, then finish it */
static void bench_fake_motion(const s5_step_t * p_step, uint16_t seq)
{
    s5_cmd_t done = { S5_CMD_MOTION_DONE, 1, 0 };

    tm_printf((uint8_t *)"s5: [bench] move kind=%d ang=%d dist=%d spd=%d\n",
              p_step->m.kind, p_step->m.angle_deg, p_step->m.distance_mm,
              p_step->m.speed_mm_s);
    done.seq = seq;
    tk_dly_tsk(S5_BENCH_MOVE_MS);
    tk_snd_mbf(g_mbf_s5_cmd, &done, sizeof(done), TMO_POL);
}
#endif

/* Send one step and wait for it to finish. */
static step_result_t run_step(const s5_step_t * p_step)
{
    s5_cmd_t      cmd;
    s5_step_t     stop = { 0 };
    uint16_t      seq;
    TMO           left = step_timeout_ms(&p_step->m);
    TMO           slice;
    int32_t       len;
    bool          b_finished = false;
    step_result_t result     = STEP_TIMEOUT;

    seq = send_motion(&p_step->m);

#if S5_BENCH_MODE
    bench_fake_motion(p_step, seq);
#endif

    while ((!b_finished) && (left > 0))
    {
        /* search steps poll the line sensors every period */
        slice = (0U != p_step->search_line) ? S5_PERIOD_MS : left;
        if (slice > left)
        {
            slice = left;
        }
        len = tk_rcv_mbf(g_mbf_s5_cmd, &cmd, slice);
        left -= slice;

        if (len > 0)
        {
            b_finished = step_command(p_step, &cmd, seq, &result);
        }
        else if ((0U != p_step->search_line) && (s5_port_line_detected()))
        {
            stop.m.kind = S5_MOVE_STOP;
            (void)send_motion(&stop.m);
            result     = STEP_LINE;
            b_finished = true;
        }
        else
        {
            /* nothing yet: keep waiting */
        }
    }
    return result;
}

/* Run a plan.  Stops at the first result other than STEP_DONE. */
static step_result_t run_plan(const s5_step_t * p_plan, int32_t count)
{
    int32_t       idx;
    step_result_t result = STEP_DONE;

    for (idx = 0; (idx < count) && (STEP_DONE == result); idx++)
    {
        if (0U != p_plan[idx].search_line)
        {
            s5_set_state(S5_ST_SEARCH_LINE);
        }
        result = run_step(&p_plan[idx]);
    }
    return result;
}

static void report(s5_evt_type_t type, s5_fault_code_t code)
{
    s5_evt_t evt = { 0 };

    /* Cast: both enums fit the 8-bit message fields. */
    evt.type = (uint8_t)type;
    evt.code = (uint8_t)code;
    s5_send_evt(&evt);
    if (S5_FAULT_NONE != code)
    {
        s5_set_fault(code);
    }
}

/* Turn back to the heading the car had before the bypass. */
static step_result_t realign(void)
{
    s5_step_t     step   = { 0 };
    step_result_t result = STEP_DONE;

    if (0 != g_heading_deg)
    {
        step.m.kind      = S5_MOVE_TURN;
        /* Cast: the net turn of a bypass is within +-127 degrees. */
        step.m.angle_deg = (int8_t)(-g_heading_deg);
        result           = run_step(&step);
    }
    return result;
}

/* 1. Ask the scan task for a profile.  Returns true when there is an
   obstacle to go round; otherwise the outcome is already reported. */
static bool get_profile(s5_profile_t * p_prof)
{
    s5_evt_t evt = { 0 };
    UINT     flg;
    bool     b_obstacle = false;

    /* Cast: clear every flag bit except PROFILE. */
    tk_clr_flg(g_flg_s5, ~(uint32_t)S5_FLG_PROFILE);
    tk_set_flg(g_flg_s5, S5_FLG_SCAN_REQ);
    if (E_OK != tk_wai_flg(g_flg_s5, S5_FLG_PROFILE, TWF_ORW | TWF_BITCLR, &flg,
                           S5_SCAN_TMO_MS))
    {
        report(S5_EVT_FAULT, S5_FAULT_NO_PROFILE);
    }
    else
    {
        s5_load_profile(p_prof);
        evt.type      = S5_EVT_PROFILE_READY;
        evt.u.profile = *p_prof;
        s5_send_evt(&evt);
        if (0U == p_prof->valid)
        {
            /* obstacle gone (moved, or a false alarm): carry on */
            tm_printf((uint8_t *)"s5: no obstacle in scan, resuming\n");
            report(S5_EVT_LINE_REACQUIRED, S5_FAULT_NONE);
        }
        else
        {
            b_obstacle = true;
        }
    }
    return b_obstacle;
}

/* 4b. Report how the bypass and line recovery ended */
static void report_result(step_result_t result)
{
    switch (result)
    {
        case STEP_LINE:
            if (STEP_ABORT != realign())
            {
                s5_count_avoid_ok();
                tm_printf((uint8_t *)"s5: line reacquired\n");
                report(S5_EVT_LINE_REACQUIRED, S5_FAULT_NONE);
            }
            break;
        case STEP_DONE: /* every recovery ran out */
            report(S5_EVT_AVOID_FAILED, S5_FAULT_NONE);
            break;
        case STEP_TIMEOUT:
            report(S5_EVT_FAULT, S5_FAULT_MOTION_TIMEOUT);
            break;
        case STEP_FAILED:
            report(S5_EVT_AVOID_FAILED, S5_FAULT_NONE);
            break;
        case STEP_ABORT:
        default:
            tm_printf((uint8_t *)"s5: avoidance aborted\n");
            break;
    }
}

/* 2..4. Plan the bypass, drive it, then look for the line */
static void bypass(const s5_profile_t * p_prof)
{
    s5_step_t     plan[S5_PLAN_MAX];
    int32_t       count;
    int32_t       attempt;
    step_result_t result;

    /* 2. plan */
    s5_set_state(S5_ST_PLANNING);
    count = s5_plan_bypass(p_prof, plan, S5_PLAN_MAX);
    if (S5_SIDE_NONE == p_prof->bypass_side)
    {
        (void)run_plan(plan, count); /* stop + back off */
        report(S5_EVT_AVOID_FAILED, S5_FAULT_BLOCKED);
    }
    else
    {
        /* 3. bypass */
        s5_set_state(S5_ST_BYPASSING);
        result = run_plan(plan, count);

        /* 4. bounded line recovery */
        for (attempt = 0; (STEP_DONE == result) && (attempt < S5_MAX_RECOVERY);
             attempt++)
        {
            s5_set_state(S5_ST_LINE_RECOVERY);
            tm_printf((uint8_t *)"s5: line not found, recovery %d/%d\n",
                      attempt + 1, S5_MAX_RECOVERY);
            /* Cast: bypass_side holds an s5_side_t value. */
            count  = s5_plan_recovery((s5_side_t)p_prof->bypass_side, attempt,
                                      plan, S5_PLAN_MAX);
            result = run_plan(plan, count);
        }
        report_result(result);
    }
}

static void avoid_obstacle(void)
{
    s5_profile_t prof;
    UINT         flg;

    g_heading_deg = 0;

    if (get_profile(&prof))
    {
        bypass(&prof);
    }

    /* 5. back to monitoring (or idle when the controller disabled us) */
    s5_set_state(
        (E_OK == tk_wai_flg(g_flg_s5, S5_FLG_ENABLED, TWF_ORW, &flg, TMO_POL))
            ? S5_ST_MONITORING
            : S5_ST_IDLE);
    tk_set_flg(g_flg_s5, S5_FLG_RESUME);
}

/* One command from the Vehicle Controller, while idle or monitoring */
static void handle_command(const s5_cmd_t * p_cmd)
{
    switch (p_cmd->type)
    {
        case S5_CMD_ENABLE:
            tk_set_flg(g_flg_s5, S5_FLG_ENABLED);
            s5_set_state(S5_ST_MONITORING);
            break;
        case S5_CMD_DISABLE:
            /* Cast: clear every flag bit except ENABLED. */
            tk_clr_flg(g_flg_s5, ~(uint32_t)S5_FLG_ENABLED);
            s5_set_state(S5_ST_IDLE);
            break;
        case S5_CMD_START_AVOID:
            avoid_obstacle();
            break;
        case S5_CMD_ABORT:
            /* controller cancels a detection it will not act on:
               let the scan task monitor again */
            tk_set_flg(g_flg_s5, S5_FLG_RESUME);
            break;
        default: /* stale MOTION_DONE: ignore */
            break;
    }
}

void s5_avoid_task(int32_t stacd, void * p_exinf)
{
    s5_cmd_t cmd;
    int32_t  len;

    (void)stacd;
    (void)p_exinf;

    tm_printf((uint8_t *)"s5: avoid task %d up\n", tk_get_tid());

    for (;;)
    {
        len = tk_rcv_mbf(g_mbf_s5_cmd, &cmd, TMO_FEVR);
        /* Cast: sizeof of a small struct vs the signed receive size. */
        if (len < (int32_t)sizeof(cmd))
        {
            app_check(len, "s5 rcv cmd");
        }
        else
        {
            handle_command(&cmd);
        }
    }
}

/*** end of file ***/
