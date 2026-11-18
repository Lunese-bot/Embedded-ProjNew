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
 *        4. line found  -> stop, realign and check the line is still visible
 *           not found   -> up to S5_MAX_RECOVERY slow passes on both sides
 *                          and ahead, then S5_EVT_AVOID_FAILED (Safe Stop)
 *        5. tell the scan task to go back to monitoring (S5_FLG_RESUME)
 *      ABORT or DISABLE at any point ends the manoeuvre at once.
 */

#include "s5_local.h"
#include "../robot_types.h" /* ROBOT_IR_TO_AXLE_MM */

typedef enum
{
    STEP_DONE,    /* motion finished                 */
    STEP_LINE,    /* the line was seen during a search */
    STEP_FAILED,  /* controller reported failure     */
    STEP_TIMEOUT, /* no MOTION_DONE in time          */
    STEP_ABORT    /* ABORT / DISABLE received        */
} step_result_t;

static uint16_t g_seq_next = 1;
static int32_t  g_heading_deg; /* sum of completed turns since START */
static const s5_step_t g_stop_step = { .m.kind = S5_MOVE_STOP };

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
#if S5_BENCH_MODE
            /* Bench mode: S5 started this avoidance itself, so the IDLE
               Vehicle Controller's "not acting on it" ABORT is expected
               and must not cut the bench sequence short. */
            b_stop = false;
#else
            b_stop = true;
#endif
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
#if S5_BENCH_MODE
    /* Bench mode: moves are only printed (bench_fake_motion).  Sending
       them would make the IDLE Vehicle Controller reject each one and
       answer MOTION_DONE ok=0, racing the bench's own "done". */
#else
    if (S5_MOVE_STOP == p_req->kind)
    {
        (void)s5_send_critical(&evt);
    }
    else
    {
        (void)s5_send_evt(&evt);
    }
#endif
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
static step_result_t run_motion(const s5_step_t * p_step)
{
    s5_cmd_t      cmd;
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
        SYSTIM t_before;
        SYSTIM t_after;
        TMO    waited;

        /* search steps poll the line sensors every period */
        slice = (0U != p_step->search_line) ? S5_PERIOD_MS : left;
        if (slice > left)
        {
            slice = left;
        }
        tk_get_tim(&t_before);
        len = tk_rcv_mbf(g_mbf_s5_cmd, &cmd, slice);
        tk_get_tim(&t_after);

        /* Count only the time actually waited.  tk_rcv_mbf returns early
           when ANY command arrives (e.g. the controller's ABORT); taking
           the whole slice off made the step "time out" at once (0x5002). */
        waited = (TMO)(t_after.lo - t_before.lo);
        if ((len < 0) || (waited > slice))
        {
            waited = slice; /* timed out: the full slice has passed */
        }
        left -= waited;

        if (len > 0)
        {
            b_finished = step_command(p_step, &cmd, seq, &result);
        }
        else if ((0U != p_step->search_line) && (s5_port_line_detected()))
        {
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

/* Finish braking before acting on a sighting. The line can slip out from
   under the sensors while the wheels slow down. */
static step_result_t run_step(const s5_step_t * p_step)
{
    step_result_t result;

    if ((0U != p_step->search_line) && s5_port_line_detected())
    {
        result = STEP_LINE;
    }
    else
    {
        result = run_motion(p_step);
    }

    if (STEP_LINE == result)
    {
        result = run_motion(&g_stop_step);
        if ((STEP_DONE == result) && s5_port_line_detected())
        {
            result = STEP_LINE;
        }
    }
    return result;
}

static void report(s5_evt_type_t type, s5_fault_code_t code)
{
    s5_evt_t evt = { 0 };

    /* Cast: both enums fit the 8-bit message fields. */
    evt.type = (uint8_t)type;
    evt.code = (uint8_t)code;
    if (S5_FAULT_NONE != code)
    {
        s5_set_fault(code);
    }
    if ((S5_EVT_FAULT == type) || (S5_EVT_AVOID_FAILED == type))
    {
        (void)s5_send_critical(&evt);
    }
    else
    {
        (void)s5_send_evt(&evt);
    }
}

static void start_line_sampling(void)
{
    s5_port_set_line_search(true);
    tk_dly_tsk(S5_PERIOD_MS); /* allow a fresh IR sample before checking */
}

/* Optionally look for a nearby line before checking that black remains
   visible after a short forward drive. */
static step_result_t confirm_line_ahead(bool b_search_nearby)
{
    const s5_step_t creep = { .m = { .kind = S5_MOVE_FORWARD,
                                     .speed_mm_s = S5_SEARCH_SPEED_MM_S,
                                     .distance_mm = S5_REJOIN_CHECK_MM } };
    step_result_t result = run_motion(&g_stop_step);

    if (STEP_DONE == result)
    {
        bool b_seen;

        start_line_sampling();
        b_seen = s5_port_line_detected();
        if (!b_seen && b_search_nearby)
        {
            const s5_step_t search = {
                .m = { .kind = S5_MOVE_FORWARD,
                       .speed_mm_s = S5_SEARCH_SPEED_MM_S,
                       .distance_mm = S5_REJOIN_SEARCH_MM },
                .search_line = 1u
            };

            tm_printf((uint8_t *)"s5: creeping toward the last line sighting\n");
            result = run_step(&search);
            b_seen = (STEP_LINE == result);
        }
        if (b_seen)
        {
            result = run_motion(&creep);
            if (STEP_DONE == result)
            {
                result = run_motion(&g_stop_step);
            }
            if (STEP_DONE == result)
            {
                tk_dly_tsk(S5_PERIOD_MS); /* sample with the wheels stopped */
                if (s5_port_line_detected())
                {
                    result = STEP_LINE;
                }
                else
                {
                    tm_printf((uint8_t *)"s5: line lost during rejoin crawl\n");
                }
            }
        }
        s5_port_set_line_search(false);
    }
    return result;
}

/* First probe toward the last sighting. Moving the forward-mounted sensors
   while aligning can put them a few centimetres to the other side of it. */
static step_result_t check_rejoin(int32_t sighting_heading_deg)
{
    const int32_t first = (sighting_heading_deg < 0) ? -S5_REJOIN_PROBE_DEG
                                                    : S5_REJOIN_PROBE_DEG;
    const int32_t headings[] = { first, -first, 0 };
    s5_step_t turn = { .m.kind = S5_MOVE_TURN };
    step_result_t result = confirm_line_ahead(false);
    uint32_t idx;

    if (STEP_DONE == result)
    {
        tm_printf((uint8_t *)"s5: checking nearby headings for the line\n");
    }
    for (idx = 0u; (STEP_DONE == result) &&
                   (idx < sizeof(headings) / sizeof(headings[0])); idx++)
    {
        /* Heading is relative to the original approach. The largest
           move is twice the probe angle when switching sides. */
        turn.m.angle_deg = (int8_t)(headings[idx] - g_heading_deg);
        result = run_step(&turn);
        if (STEP_DONE == result)
        {
            result = confirm_line_ahead((0u == idx) && (0 != sighting_heading_deg));
        }
    }
    return result;
}

/* Turn back to the approach heading. Continue searching if that moves
   the sensors off the line again. */
static step_result_t realign(void)
{
    const int32_t sighting_heading_deg = g_heading_deg;
    s5_step_t     step   = { 0 };
    step_result_t result = STEP_DONE;

    if ((0 != g_heading_deg) && (ROBOT_IR_TO_AXLE_MM > 0))
    {
        /* The search stopped with the SENSORS on the line.  Drive on until
           the axle is over it, so the pivot below leaves the sensors on
           the line instead of beside it (see robot_types.h). */
        s5_step_t fwd = { 0 };

        fwd.m.kind        = S5_MOVE_FORWARD;
        fwd.m.distance_mm = (uint16_t)ROBOT_IR_TO_AXLE_MM;
        fwd.m.speed_mm_s  = S5_SEARCH_SPEED_MM_S;
        result            = run_step(&fwd);
    }
    if ((0 != g_heading_deg) && (STEP_DONE == result))
    {
        step.m.kind      = S5_MOVE_TURN;
        /* Cast: the net turn of a bypass is within +-127 degrees. */
        step.m.angle_deg = (int8_t)(-g_heading_deg);
        result           = run_step(&step);
    }
    return (STEP_DONE == result) ? check_rejoin(sighting_heading_deg) : result;
}

/* A search step is last in each plan. STEP_DONE means the pass ended
   without a line, including when a sighting was lost during alignment. */
static step_result_t run_plan(const s5_step_t * p_plan, int32_t count,
                              s5_state_t search_state)
{
    int32_t       idx;
    step_result_t result = STEP_DONE;

    for (idx = 0; (idx < count) && (STEP_DONE == result); idx++)
    {
        bool b_searching = (0U != p_plan[idx].search_line);

        if (b_searching)
        {
            s5_set_state(search_state);
            start_line_sampling();
        }
        result = run_step(&p_plan[idx]);
        if (b_searching)
        {
            s5_port_set_line_search(false);
        }
    }
    return (STEP_LINE == result) ? realign() : result;
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
            s5_count_avoid_ok();
            tm_printf((uint8_t *)"s5: line reacquired\n");
            report(S5_EVT_LINE_REACQUIRED, S5_FAULT_NONE);
            break;
        case STEP_DONE: /* every recovery ran out */
            tm_printf((uint8_t *)"s5: black line not found after %d passes, stopping\n",
                      S5_MAX_RECOVERY);
            report(S5_EVT_AVOID_FAILED, S5_FAULT_LINE_NOT_FOUND);
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
        (void)run_plan(plan, count, S5_ST_SEARCH_LINE); /* stop + back off */
        report(S5_EVT_AVOID_FAILED, S5_FAULT_BLOCKED);
    }
    else
    {
        /* 3. bypass */
        s5_set_state(S5_ST_BYPASSING);
        result = run_plan(plan, count, S5_ST_SEARCH_LINE);

        /* 4. bounded line recovery */
        for (attempt = 0; (STEP_DONE == result) && (attempt < S5_MAX_RECOVERY);
             attempt++)
        {
            s5_set_state(S5_ST_LINE_RECOVERY);
            tm_printf((uint8_t *)"s5: line not found, recovery %d/%d\n",
                      attempt + 1, S5_MAX_RECOVERY);
            /* Cast: bypass_side holds an s5_side_t value. */
            count  = s5_plan_recovery((s5_side_t)p_prof->bypass_side, attempt,
                                      g_heading_deg, plan, S5_PLAN_MAX);
            result = run_plan(plan, count, S5_ST_LINE_RECOVERY);
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
            tk_set_flg(g_flg_s5, S5_FLG_RESUME);
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
            s5_set_state(S5_ST_MONITORING); /* else "obs" stays DETECTED */
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
