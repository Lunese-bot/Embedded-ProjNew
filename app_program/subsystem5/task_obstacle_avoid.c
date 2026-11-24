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
 *           not found   -> one retry on the inward heading after a missed
 *                          return, then straighten and search forward
 *        5. tell the scan task to go back to monitoring (S5_FLG_RESUME)
 *      ABORT or DISABLE at any point ends the manoeuvre at once.
 */

#include "s5_local.h"
#include "../robot_types.h" /* ROBOT_IR_TO_AXLE_MM */

typedef enum
{
    STEP_DONE,    /* motion finished                 */
    STEP_LINE,    /* the line was seen during a search */
    STEP_DOUBLE_BLACK, /* both sensors see black at an angled rejoin probe */
    STEP_NO_LINE, /* a search finished without seeing tape */
    STEP_LINE_LOST, /* tape sighting lost during braking or alignment */
    STEP_PASS_END, /* measured pass finished or fell back: stop the car */
    STEP_PASS_BLOCKED, /* pass cannot continue safely (reason in g_pass_why) */
    STEP_FAILED,  /* controller reported failure     */
    STEP_TIMEOUT, /* no MOTION_DONE in time          */
    STEP_ABORT    /* ABORT / DISABLE received        */
} step_result_t;

static uint16_t g_seq_next = 1;
static int32_t  g_heading_deg; /* sum of completed turns since START */
static const s5_step_t g_stop_step = { .m.kind = S5_MOVE_STOP };

/* Measured pass: tracker state of the running pass leg (local debug data). */
typedef struct
{
    s5_pass_t pass;
    uint32_t  odo0_mm; /* odometer when the pass leg was requested */
} pass_ctx_t;
static pass_ctx_t      g_pass;
static s5_pass_phase_t g_pass_why; /* final tracker phase of the last pass leg */

/* Obstacle size report of the running encounter (s5_obst_report.h).  Only
   stored for S1 to read; S5 never waits for MQTT. */
static s5_obst_report_t g_report;
static uint16_t         g_report_id;
static bool             g_observing;     /* the report-only side look is running */
static bool             g_obs_reading;   /* it produced at least one reading     */
static bool             g_length_done;   /* length already resolved this encounter */
static step_result_t    g_bypass_result; /* how the bypass ended (final reason)  */

/* Store the record for S1 and print it on USB with the exact MQTT frame,
   so the USB console and the dashboard can be compared byte for byte. */
static void report_publish(void)
{
    uint8_t frame[S5_OR_BYTES];
    char    hex[(2 * S5_OR_BYTES) + 1];
    int32_t idx;
    static const char digits[] = "0123456789abcdef";

    s5_report_store(&g_report);
    s5_or_encode(&g_report, frame);
    for (idx = 0; idx < S5_OR_BYTES; idx++)
    {
        hex[2 * idx]       = digits[frame[idx] >> 4];
        hex[(2 * idx) + 1] = digits[frame[idx] & 0x0FU];
    }
    hex[2 * S5_OR_BYTES] = '\0';
    tm_printf((uint8_t *)"s5: obstacle #%d rev %d %s: front %d mm, width %d mm (%s), "
                         "length %d mm (%s, %s) | frame %s\n",
              g_report.profile_id, g_report.rev, s5_or_stage_name(g_report.stage),
              g_report.front_mm, g_report.width_mm,
              s5_or_width_name(g_report.width_status), g_report.length_mm,
              s5_or_length_name(g_report.length_status),
              s5_or_reason_name(g_report.length_reason), hex);
}

/* Resolve the length once per encounter from the pass tracker. */
static void report_length(bool b_leg_over, bool b_any_reading)
{
    int32_t        len_mm;
    s5_or_reason_t why;
    s5_or_length_t st;

    if (g_length_done)
    {
        return;
    }
    st = s5_pass_length(&g_pass.pass, b_leg_over, b_any_reading, &len_mm, &why);
    if (S5_OR_L_MEASURING != st)
    {
        g_length_done = true;
        s5_or_length(&g_report, st, len_mm, why);
        report_publish();
    }
}

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

/* One poll of the measured pass.  Returns true when the leg must end
   now; *p_result then says how. */
static bool pass_poll(step_result_t * p_result)
{
#if !S5_SIDE_PASS
    (void)p_result;
    return false;
#else
    s5_side_reading_t reading;
    int32_t           travel_mm;

    (void)s5_side_read(&reading);
    travel_mm = (int32_t)(s5_port_odo_mm() - g_pass.odo0_mm);
    g_pass_why = s5_pass_update(&g_pass.pass, travel_mm, &reading);
    switch (g_pass_why)
    {
        case S5_PASS_CLEARED:
        case S5_PASS_FALLBACK:
            *p_result = STEP_PASS_END;
            return true;
        case S5_PASS_CAP:
        case S5_PASS_TOO_CLOSE:
        case S5_PASS_SENSOR_FAULT:
            *p_result = STEP_PASS_BLOCKED;
            return true;
        default:
            return false;
    }
#endif
}

/* One poll of the report-only side look (S5_LENGTH_OBSERVE).  Feeds the
   tracker and publishes the length as soon as the rear edge is seen.
   Changes nothing about the motion. */
static void observe_poll(void)
{
    s5_side_reading_t reading;
    int32_t           travel_mm;

    if (s5_side_read(&reading) && (0U != reading.seq))
    {
        g_obs_reading = true;
    }
    travel_mm = (int32_t)(s5_port_odo_mm() - g_pass.odo0_mm);
    (void)s5_pass_update(&g_pass.pass, travel_mm, &reading);
    report_length(false, g_obs_reading);
}

/* Send one step and wait for it to finish.  p_pass != NULL: also run the
   measured-pass tracker every period and end the step early with
   STEP_PASS_END / STEP_PASS_BLOCKED (the caller then stops the car). */
static step_result_t run_motion_ex(const s5_step_t * p_step, bool b_pass)
{
    s5_cmd_t      cmd;
    uint16_t      seq;
    s5_motion_req_t req = p_step->m;
    TMO           left;
    TMO           slice;
    int32_t       len;
    bool          b_finished = false;
    step_result_t result     = STEP_TIMEOUT;

    if (b_pass)
    {
        req.distance_mm = p_step->request_mm; /* the cap plus the longest clearing drive */
    }
    left = step_timeout_ms(&req);
    seq = send_motion(&req);

#if S5_BENCH_MODE
    bench_fake_motion(p_step, seq);
#endif

    while ((!b_finished) && (left > 0))
    {
        SYSTIM t_before;
        SYSTIM t_after;
        TMO    waited;

        /* search steps poll the line sensors every period */
        slice = ((0U != p_step->search_line) || b_pass || g_observing)
                    ? S5_PERIOD_MS : left;
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
        else if (b_pass && pass_poll(&result))
        {
            b_finished = true;
        }
        else
        {
            if (g_observing)
            {
                observe_poll(); /* report only: never ends the step */
            }
        }
    }
    return result;
}

static step_result_t run_motion(const s5_step_t * p_step)
{
    return run_motion_ex(p_step, false);
}

/* The measured pass leg: look sideways while driving forward, end by
   measurement, and always stop the car before anything else happens.
   If the side look cannot start, drive the fixed-depth distance as before. */
static step_result_t run_pass_step(const s5_step_t * p_step)
{
#if S5_SIDE_PASS
    s5_geom_t         geom;
    s5_side_reading_t reading;
    step_result_t     result = STEP_DONE;
    s5_step_t         fixed  = *p_step;
    TMO               waited = 0;

    fixed.side_track = 0U;
    s5_geom_from_config(&geom);
    s5_pass_init(&g_pass.pass, &geom, S5_SIDE_LOOK_DEG, p_step->m.distance_mm,
                 p_step->cap_mm);
    g_pass_why = S5_PASS_SEEKING;
    s5_side_request(p_step->look_deg);
    while ((STEP_DONE == result) && !s5_side_read(&reading))
    {
        s5_cmd_t cmd;

        if (waited >= S5_SIDE_ARM_TMO_MS)
        {
            break;
        }
        if (tk_rcv_mbf(g_mbf_s5_cmd, &cmd, S5_PERIOD_MS) > 0)
        {
            result = busy_command(&cmd) ? STEP_ABORT : STEP_DONE;
        }
        waited += S5_PERIOD_MS;
    }
    if (STEP_DONE != result)
    {
        s5_side_end();
        return result;
    }
    if (!s5_side_read(&reading))
    {
        s5_side_end();
        tm_printf((uint8_t *)"s5: side look unavailable, depth assumed (fixed pass %d mm)\n",
                  p_step->m.distance_mm);
        if (!g_length_done)
        {
            g_length_done = true;
            s5_or_length(&g_report, S5_OR_L_UNAVAILABLE, 0, S5_OR_R_LOOK_FAILED);
            report_publish();
        }
        return run_motion(&fixed);
    }
    g_pass.odo0_mm = s5_port_odo_mm();
    tm_printf((uint8_t *)"s5: pass: side look %d deg, fixed fallback %d mm, cap %d mm\n",
              p_step->look_deg, p_step->m.distance_mm, p_step->cap_mm);
    result = run_motion_ex(p_step, true);
    s5_side_end(); /* the servo returns to centre; never delays the stop */
    if (STEP_ABORT != result)
    {
        report_length(true, true);
    }
    if ((STEP_PASS_END == result) || (STEP_PASS_BLOCKED == result))
    {
        step_result_t stopped = run_motion(&g_stop_step);

        if (STEP_DONE != stopped)
        {
            return stopped;
        }
        result = (STEP_PASS_END == result) ? STEP_DONE : STEP_PASS_BLOCKED;
    }
    else if (STEP_DONE == result)
    {
        /* the motion ended on its own: nothing was measured to the end */
        g_pass_why = S5_PASS_FALLBACK;
    }
    if ((STEP_DONE == result) && (S5_PASS_CLEARED == g_pass_why))
    {
        tm_printf((uint8_t *)"s5: pass measured: first echo %d mm last echo %d mm span %d mm%s "
                             "min gap %d mm, turn-in at %d mm\n",
                  g_pass.pass.first_mm, g_pass.pass.last_mm, g_pass.pass.depth_mm,
                  g_pass.pass.partial ? " (partial: already alongside)" : "",
                  g_pass.pass.min_gap_mm, g_pass.pass.target_mm);
    }
    else if ((STEP_DONE == result) && (S5_PASS_FALLBACK == g_pass_why))
    {
        tm_printf((uint8_t *)"s5: no side echo, depth assumed (fixed pass %d mm, "
                             "clearance not certified)\n",
                  p_step->m.distance_mm);
    }
    return result;
#else
    return run_motion(p_step);
#endif
}

/* The fixed-depth pass leg with the report-only side look.  The motion
   request is the plan's own (same distance, speed and order) and is sent
   at once: the side look starts in parallel and its first readings may
   come a little after the wheels start (then "partial" is judged from the
   first reading received). */
static step_result_t run_observe_step(const s5_step_t * p_step)
{
#if S5_LENGTH_OBSERVE && !S5_SIDE_PASS
    s5_geom_t     geom;
    s5_step_t     leg = *p_step;
    step_result_t result;

    leg.side_track = 0U;
    s5_geom_from_config(&geom);
    s5_pass_init_observe(&g_pass.pass, &geom, S5_SIDE_LOOK_DEG, p_step->m.distance_mm);
    g_pass.odo0_mm = s5_port_odo_mm();
    g_obs_reading  = false;
    s5_side_request(p_step->look_deg);
    g_observing = true;
    result = run_motion(&leg);
    g_observing = false;
    s5_side_end(); /* the servo returns to centre in the scan task */
    if (STEP_DONE == result)
    {
        observe_poll(); /* the last reading before the leg ended */
        report_length(true, g_obs_reading);
        tm_printf((uint8_t *)"s5: length observe: first echo %d mm last echo %d mm span %d mm, "
                             "phase %d (fixed pass %d mm, depth assumed for planning)\n",
                  g_pass.pass.first_mm, g_pass.pass.last_mm, g_pass.pass.depth_mm,
                  g_pass.pass.phase, p_step->m.distance_mm);
    }
    return result;
#else
    s5_step_t leg = *p_step;

    leg.side_track = 0U;
    return run_motion(&leg);
#endif
}

/* Finish braking before acting on a sighting. The line can slip out from
   under the sensors while the wheels slow down. */
static step_result_t run_step(const s5_step_t * p_step)
{
    step_result_t result;

    if ((2U == p_step->side_track) && (0U == p_step->search_line))
    {
        return run_observe_step(p_step); /* the pass leg never searches the line */
    }
    if (1U == p_step->side_track)
    {
        return run_pass_step(p_step);
    }
    if ((0U != p_step->search_line) && s5_port_line_detected())
    {
        result = STEP_LINE;
    }
    else
    {
        result = run_motion(p_step);
    }

    if ((0U != p_step->search_line) && (STEP_DONE == result))
    {
        /* The final encoder update can arrive before the next IR sample. */
        tk_dly_tsk(S5_PERIOD_MS);
        if (s5_port_line_detected())
        {
            result = STEP_LINE;
        }
    }
    if (STEP_LINE == result)
    {
        result = run_motion(&g_stop_step);
        if (STEP_DONE == result)
        {
            tk_dly_tsk(S5_PERIOD_MS); /* check with a fresh stopped IR sample */
            if (s5_port_line_detected())
            {
                result = STEP_LINE;
            }
            else
            {
                tm_printf((uint8_t *)"s5: line lost during braking\n");
                result = STEP_LINE_LOST;
            }
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

/* Both sensors at an angle may be crossing the tape. Turn back to the
   approach heading before accepting that sighting. */
static step_result_t sample_rejoin(void)
{
    if ((0 != g_heading_deg) && s5_port_line_both_black())
    {
        return STEP_DOUBLE_BLACK;
    }
    return s5_port_line_detected() ? STEP_LINE : STEP_DONE;
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
        start_line_sampling();
        result = sample_rejoin();
        if ((STEP_DONE == result) && b_search_nearby)
        {
            const s5_step_t search = {
                .m = { .kind = S5_MOVE_FORWARD,
                       .speed_mm_s = S5_SEARCH_SPEED_MM_S,
                       .distance_mm = S5_REJOIN_SEARCH_MM },
                .search_line = 1u
            };

            tm_printf((uint8_t *)"s5: creeping toward the last line sighting\n");
            result = run_step(&search);
            if (STEP_LINE == result)
            {
                tk_dly_tsk(S5_PERIOD_MS);
                result = sample_rejoin();
            }
            else if (STEP_LINE_LOST == result)
            {
                result = STEP_DONE; /* keep checking nearby headings */
            }
        }
        if (STEP_LINE == result)
        {
            result = run_motion(&creep);
            if (STEP_DONE == result)
            {
                result = run_motion(&g_stop_step);
            }
            if (STEP_DONE == result)
            {
                tk_dly_tsk(S5_PERIOD_MS); /* sample with the wheels stopped */
                result = sample_rejoin();
                if (STEP_DONE == result)
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

static step_result_t turn_to_approach(void)
{
    /* Cast: the net turn of a bypass is within +-127 degrees. */
    s5_step_t turn = {
        .m = { .kind = S5_MOVE_TURN, .angle_deg = (int8_t)(-g_heading_deg) }
    };

    return (0 != g_heading_deg) ? run_step(&turn) : STEP_DONE;
}

/* Turn back to the approach heading. Continue searching if that moves
   the sensors off the line again. */
static step_result_t realign(void)
{
    const int32_t sighting_heading_deg = g_heading_deg;
    step_result_t result = STEP_DONE;

    /* Rejoin trace (log only): sighting -> axle advance -> heading -> confirm */
    tm_printf((uint8_t *)"s5: rejoin: first tape sighting at heading %d deg\n",
              g_heading_deg);

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
        if (STEP_DONE == result)
        {
            tm_printf((uint8_t *)"s5: rejoin: axle advanced %d mm\n",
                      ROBOT_IR_TO_AXLE_MM);
        }
    }
    if (STEP_DONE == result)
    {
        result = turn_to_approach();
        if (STEP_DONE == result)
        {
            tm_printf((uint8_t *)"s5: rejoin: heading realigned to the approach\n");
        }
    }
    if (STEP_DONE == result)
    {
        result = check_rejoin(sighting_heading_deg);
    }
    if (STEP_DOUBLE_BLACK == result)
    {
        tm_printf((uint8_t *)"s5: both line sensors black, returning to approach heading\n");
        result = turn_to_approach();
        if (STEP_DONE == result)
        {
            result = confirm_line_ahead(false);
        }
    }
    return result;
}

/* The final search distinguishes a full-distance miss from a lost sighting.
   Only the former can extend the initial return on the same heading. */
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
            if (STEP_DONE == result)
            {
                tm_printf((uint8_t *)"s5: search ended without seeing the line\n");
                result = STEP_NO_LINE;
            }
        }
    }
    if (STEP_LINE == result)
    {
        result = realign();
        if (STEP_DONE == result)
        {
            tm_printf((uint8_t *)"s5: line lost during alignment\n");
            result = STEP_LINE_LOST;
        }
    }
    return result;
}

static step_result_t retry_search(uint16_t distance_mm, s5_state_t search_state)
{
    const s5_step_t plan[] = {
        { .m = { .kind = S5_MOVE_STOP } },
        { .m = { .kind = S5_MOVE_FORWARD, .distance_mm = distance_mm,
                 .speed_mm_s = S5_SEARCH_SPEED_MM_S }, .search_line = 1u }
    };

    return run_plan(plan, sizeof(plan) / sizeof(plan[0]), search_state);
}

static bool search_missed(step_result_t result)
{
    return (STEP_NO_LINE == result) || (STEP_LINE_LOST == result);
}

/* Width as soon as the front profile is ready.  The scan covered the
   coarse angles (s5_coarse_angles): an edge at either end of that range
   makes the width a lower bound. */
static void report_front(const s5_profile_t * p_prof)
{
    int8_t ang[8];
    int32_t n = s5_coarse_angles(ang, (int32_t)sizeof(ang));
    bool    b_follow = (0U != p_prof->valid) &&
                       (S5_SIDE_NONE != (s5_side_t)p_prof->bypass_side) &&
                       ((0 != S5_SIDE_PASS) || (0 != S5_LENGTH_OBSERVE));
    s5_or_reason_t why = ((0 == S5_SIDE_PASS) && (0 == S5_LENGTH_OBSERVE))
                             ? S5_OR_R_OBSERVE_OFF : S5_OR_R_NO_BYPASS;

    s5_or_front(&g_report, p_prof->valid, p_prof->distance_mm,
                p_prof->right_edge_deg, p_prof->left_edge_deg, p_prof->width_mm,
                p_prof->bypass_side, (n > 0) ? ang[0] : 0, (n > 0) ? ang[n - 1] : 0,
                b_follow, why);
    if (!b_follow)
    {
        g_length_done = true;
    }
    report_publish();
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
        report_front(p_prof);
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
            tm_printf((uint8_t *)"s5: line reacquired, rejoin confirmed; S5 done, VC takes over\n");
            report(S5_EVT_LINE_REACQUIRED, S5_FAULT_NONE);
            break;
        case STEP_NO_LINE:
        case STEP_LINE_LOST:
            tm_printf((uint8_t *)"s5: black line not confirmed after recovery, stopping\n");
            report(S5_EVT_AVOID_FAILED, S5_FAULT_LINE_NOT_FOUND);
            break;
        case STEP_PASS_BLOCKED:
            /* The car is already stopped.  Existing codes only (no new
               field or fault): BLOCKED = no safe way on, ECHO_TIMEOUT =
               the side sensor failed. */
            if (S5_PASS_SENSOR_FAULT == g_pass_why)
            {
                tm_printf((uint8_t *)"s5: side sensor failed during the pass, stopping\n");
                report(S5_EVT_FAULT, S5_FAULT_ECHO_TIMEOUT);
            }
            else
            {
                tm_printf((uint8_t *)"s5: pass stopped, %s (min gap %d mm)\n",
                          (S5_PASS_CAP == g_pass_why)
                              ? "obstacle side never cleared before the length cap"
                              : "gap to the obstacle below the guard",
                          g_pass.pass.min_gap_mm);
                report(S5_EVT_AVOID_FAILED, S5_FAULT_BLOCKED);
            }
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
    step_result_t result;

    /* 2. plan */
    s5_set_state(S5_ST_PLANNING);
    count = s5_plan_bypass(p_prof, plan, S5_PLAN_MAX);
    if (S5_SIDE_NONE == p_prof->bypass_side)
    {
        g_bypass_result = run_plan(plan, count, S5_ST_SEARCH_LINE); /* stop + back off */
        report(S5_EVT_AVOID_FAILED, S5_FAULT_BLOCKED);
    }
    else
    {
        /* 3. bypass */
        s5_set_state(S5_ST_BYPASSING);
        result = run_plan(plan, count, S5_ST_SEARCH_LINE);
        if (STEP_NO_LINE == result)
        {
            tm_printf((uint8_t *)"s5: retrying the return for %d mm\n",
                      S5_SEARCH_RETRY_MM);
            result = retry_search(S5_SEARCH_RETRY_MM, S5_ST_SEARCH_LINE);
        }

        /* 4. recover by searching along the original approach heading */
        if (search_missed(result))
        {
            s5_set_state(S5_ST_LINE_RECOVERY);
            tm_printf((uint8_t *)"s5: searching straight at the approach heading\n");
            count = s5_plan_straight_recovery(g_heading_deg, plan, S5_PLAN_MAX);
            result = run_plan(plan, count, S5_ST_LINE_RECOVERY);
        }
        if (search_missed(result))
        {
            tm_printf((uint8_t *)"s5: short %d mm line search\n", S5_FINAL_SEARCH_MM);
            result = retry_search(S5_FINAL_SEARCH_MM, S5_ST_LINE_RECOVERY);
        }
        if (search_missed(result))
        {
            tm_printf((uint8_t *)"s5: final %d mm line search\n", S5_EXTRA_SEARCH_MM);
            result = retry_search(S5_EXTRA_SEARCH_MM, S5_ST_LINE_RECOVERY);
        }
        g_bypass_result = result;
        report_result(result);
    }
}

static void avoid_obstacle(void)
{
    s5_profile_t prof;
    UINT         flg;

    SYSTIM       t0;

    g_heading_deg = 0;

    /* a new encounter: nothing of the previous obstacle is carried over */
    tk_get_tim(&t0);
    g_report_id     = s5_or_next_id(g_report_id);
    g_length_done   = false;
    g_observing     = false;
    g_bypass_result = STEP_DONE;
    s5_or_begin(&g_report, g_report_id, t0.lo);
    report_publish();

    if (get_profile(&prof))
    {
        bypass(&prof);
    }
    s5_or_final(&g_report, (STEP_ABORT == g_bypass_result) ? S5_OR_R_ABORTED
                           : ((STEP_FAILED == g_bypass_result) ||
                              (STEP_TIMEOUT == g_bypass_result) ||
                              (STEP_PASS_BLOCKED == g_bypass_result))
                                 ? S5_OR_R_BYPASS_FAILED
                                 : S5_OR_R_NO_BYPASS);
    report_publish();

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
