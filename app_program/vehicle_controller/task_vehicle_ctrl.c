/*
 *      task_vehicle_ctrl.c
 *      Vehicle Controller -- the Vehicle Controller task.
 *
 *      Loop:
 *        1. wait for an event on g_mbf_vc_events, at most VC_TICK_MS, then
 *           read the channels S2..S5 own (vc_bridge_s2..s5.c), so their
 *           events wait at most one tick, and S3's steering is forwarded
 *           every tick (10 ms, NFR2)
 *        2. no event (or events arriving non-stop) -> make a TICK event,
 *           so state time limits are still checked
 *        3. vc_sm_handle() decides; this task carries out its actions
 *           through vc_ports.c
 *        4. publish the new status (mutex snapshot + state flag)
 *        5. every VC_TELEMETRY_MS, send the status to S1
 *
 *      APIs used: tk_rcv_mbf, tk_get_tim, tk_loc_mtx/tk_unl_mtx,
 *                 tk_clr_flg/tk_set_flg
 */

#include "vehicle_controller.h"
#include "vc_sm.h"
#include "vc_ports.h"
#include "vc_bridge.h"
#include "../subsystem2/subsystem2.h" /* g_s2_odo_ticks for TICK p2 */
#include "../subsystem1/s1_bridge.h"
#include "../subsystem1/s1_cfg.h"
#include "vc_barcode_map.h"
#include "vc_bcmap_flash.h"

static vc_sm_t      g_vc_sm; /* the live controller */
static vc_actions_t g_vc_actions;

static uint32_t now_ms(void)
{
    SYSTIM sys_time;
    tk_get_tim(&sys_time);
    return sys_time.lo;
}

/* VC_ACT_CONFIG: the controller has checked its own state (b_allowed =
   no mission running).  Validate, store and activate the pending map, or
   refuse it, then answer the dashboard.  The flash write (if any) happens
   here, in this task, only while the robot is not on a mission. */
static void run_config(bool b_allowed)
{
    uint16_t          request_id = 0u;
    vc_bcmap_result_t result     = VC_BCMAP_BUSY;

    if (vc_bcmap_service(b_allowed, &request_id, &result))
    {
        tm_printf((uint8_t *)"vc: barcode map write id %u -> result %d\n",
                  (unsigned)request_id, (int)result);
        s1_bridge_config_reply(S1_CFG_OP_WRITE, request_id, (uint8_t)result);
    }
}

/* run_actions(): print a junction note ("vc: jx assigned B 1") and pass it
   to the dashboard. */
static void action_junction(const vc_action_t * p_action)
{
    /* "vc: jx assigned B 1": the dashboard reads this line on
       USB too.  letter '-' = no single-letter barcode. */
    static const char * const k_note[] = { "?", "assigned", "completed",
                                           "dropped" };
    const uint32_t            kind     = (uint32_t)p_action->p1;
    const uint8_t             letter   = (uint8_t)((uint32_t)p_action->p2 >> 8);
    const uint8_t             code     = (uint8_t)((uint32_t)p_action->p2 & 3u);

    tm_printf((uint8_t *)"vc: jx %s %c %u\n", k_note[(kind <= 3u) ? kind : 0u],
              (0u != letter) ? (int)letter : (int)'-', (unsigned)code);
    s1_bridge_junction((uint8_t)kind, letter, code);
}

/* run_actions(): print one U-turn trace line. */
static void action_trace(const vc_action_t * p_action)
{
    /* "vc: ut sight C h=1052 (137 deg)": h = counts turned
       since the U-turn began (+ = left), deg = the same in S2
       command degrees, x = detail (vc_sm.h vc_ut_trace_t) */
    static const char * const k_what[] = {
        "?",         "start",   "early-line-ignored",
        "fast-done", "sweep",   "sight",
        "brake",     "settled", "verdict",
        "done",      "FAIL"
    };
    static const char * const k_verdict[] = { "unsure", "on-line",
                                              "between-C-R", "turned-past",
                                              "no-line" };
    const uint32_t            what        = (uint32_t)p_action->p1;
    const int32_t             h_val       = p_action->p2;
    const int32_t             x_val       = (int32_t)p_action->rsv;
    const int32_t deg = (h_val * 180) / (int32_t)VC_PIVOT_COUNTS(180);

    if ((uint32_t)VC_UT_VERDICT == what)
    {
        tm_printf((uint8_t *)"vc: ut verdict %s h=%d (%d deg)\n",
                  k_verdict[((uint32_t)x_val <= 4u) ? (uint32_t)x_val : 0u],
                  h_val, deg);
    }
    else if (((uint32_t)VC_UT_SIGHT == what) || ((uint32_t)VC_UT_EARLY == what))
    {
        tm_printf((uint8_t *)"vc: ut %s %s h=%d (%d deg)\n", k_what[what],
                  (VC_LINE_C == x_val) ? "C"
                                       : ((VC_LINE_R == x_val) ? "R" : "C+R"),
                  h_val, deg);
    }
    else
    {
        tm_printf((uint8_t *)"vc: ut %s h=%d (%d deg) x=%d\n",
                  k_what[(what <= 10u) ? what : 0u], h_val, deg, x_val);
    }
}

static void run_actions(const vc_actions_t * p_acts)
{
    int32_t idx;

    for (idx = 0; idx < p_acts->n; idx++)
    {
        const vc_action_t * p_action = &p_acts->a[idx];

        switch ((vc_action_kind_t)p_action->kind)
        {
            case VC_ACT_MOVE:
                app_check(vc_port_move((vc_move_t)p_action->p1, p_action->p2),
                          "vc_port_move");
                break;
            case VC_ACT_REQUEST:
                app_check(vc_port_request((vc_source_t)p_action->target,
                                          (vc_request_t)p_action->p1,
                                          p_action->p2),
                          "vc_port_request");
                break;
            case VC_ACT_REPORT:
                app_check(vc_port_report((vc_source_t)p_action->target,
                                         (vc_fault_severity_t)p_action->p1,
                                         (uint32_t)p_action->p2),
                          "vc_port_report");
                break;
            case VC_ACT_STATE:
                tm_printf((uint8_t *)"vc: %s -> %s\n",
                          vc_state_name((vc_state_t)p_action->p2),
                          vc_state_name((vc_state_t)p_action->p1));
                break;
            case VC_ACT_CONFIG:
                run_config(0 != p_action->p1);
                break;
            case VC_ACT_JUNCTION:
                action_junction(p_action);
                break;
            case VC_ACT_TRACE:
                action_trace(p_action);
                break;
            default:
                break;
        }
    }
    if (p_acts->overflow)
    {
        /* More actions than fit: a bug in vc_sm.c.  Fail safe. */
        tm_printf((uint8_t *)"vc: ERROR action overflow, stopping\n");
        vc_port_move(VC_MOVE_STOP, 0);
    }
}

static void publish_status(bool b_state_changed)
{
    uint32_t ptn;

    if (E_OK == tk_loc_mtx(g_mtx_vc_status, TMO_FEVR))
    {
        g_vc_status_shared = g_vc_sm.st;
        tk_unl_mtx(g_mtx_vc_status);
    }

    /* S1 reports this with every map read; edits are refused while true */
    vc_bcmap_set_mission_active(vc_sm_is_active(g_vc_sm.st.state));

    if (b_state_changed)
    {
        ptn = VC_FLG_STATE(g_vc_sm.st.state);
        if (vc_sm_is_active(g_vc_sm.st.state))
        {
            ptn |= VC_FLG_RUNNING;
        }
        tk_clr_flg(g_flg_vc_state, 0); /* clear all bits */
        tk_set_flg(g_flg_vc_state, ptn);
    }
}

/* One event through the state machine, then out to the ports */
static void handle(const vc_event_t * p_event, uint32_t now, bool b_from_s5)
{
    vc_state_t before       = g_vc_sm.st.state;
    uint32_t   rejected     = g_vc_sm.st.moves_rejected;
    bool       b_was_paused = g_vc_sm.st.paused;
    bool       b_avoiding;

    vc_sm_handle(&g_vc_sm, p_event, now, &g_vc_actions);
    b_avoiding = (VC_ST_AVOID_OBSTACLE == g_vc_sm.st.state);
    vc_bridge_s2_set_obstacle_mode(b_avoiding);
    if ((VC_ST_AVOID_OBSTACLE == before) != b_avoiding)
    {
        vc_bridge_s3_set_obstacle_mode(b_avoiding);
    }
    run_actions(&g_vc_actions);

    if ((VC_SRC_COMMS == p_event->src) && ((VC_EV_CMD_PAUSE == p_event->type) ||
                                           (VC_EV_CMD_RESUME == p_event->type)))
    {
        bool b_accepted = (VC_EV_CMD_PAUSE == p_event->type)
                              ? (!b_was_paused && g_vc_sm.st.paused)
                              : (b_was_paused && !g_vc_sm.st.paused);
        s1_bridge_command_result((uint8_t)p_event->p1, b_accepted);
    }
    if (b_from_s5)
    {
        vc_bridge_s5_after(p_event, g_vc_sm.st.state,
                           g_vc_sm.st.moves_rejected != rejected);
    }
    if ((VC_SRC_MOTION == p_event->src) && (VC_ST_AVOID_OBSTACLE == before))
    {
        vc_bridge_s5_motion_event(p_event);
    }
    publish_status(before != g_vc_sm.st.state);
}

void vehicle_ctrl_task(int32_t stacd, void * p_exinf)
{
    vc_event_t event;
    vc_event_t tick = { 0 };
    int32_t    len;
    uint32_t   now;
    uint32_t   last_tick;
    uint32_t   last_telem;

    (void)stacd;
    (void)p_exinf;

    /* The robot's barcode map: the saved one, else vc_barcode_config.h */
    vc_bcmap_init(vc_bcmap_flash_store());
    {
        vc_bcmap_snapshot_t snap;
        if (vc_bcmap_get(&snap))
        {
            tm_printf((uint8_t *)"vc: barcode map gen %u (%s)\n",
                      (unsigned)snap.gen,
                      (0u != (snap.flags & VC_BCMAP_FLAG_SAVED)) ? "saved"
                                                                 : "factory");
        }
    }

    now = now_ms();
    vc_sm_init(&g_vc_sm, now);
    last_tick  = now;
    last_telem = now;
    tick.src   = VC_SRC_VC;
    tick.type  = VC_EV_TICK;
    publish_status(true);
    tm_printf((uint8_t *)"vc: task %d ready, state %s (send VC_EV_CMD_START)\n",
              tk_get_tid(), vc_state_name(g_vc_sm.st.state));

    for (;;)
    {
        /* 1. next event, or time out after one tick */
        len = tk_rcv_mbf(g_mbf_vc_events, &event, VC_TICK_MS);
        now = now_ms();

        /* Cast: sizeof of a small struct vs the signed receive size. */
        if ((int32_t)sizeof(event) == len)
        {
            handle(&event, now, false);
        }
        else if (len >= 0)
        {
            tm_printf((uint8_t *)"vc: bad event size %d\n", len);
        }
        else if (E_TMOUT != len)
        {
            app_check(len, "vc tk_rcv_mbf");
        }
        else
        {
            /* timeout: no event this tick */
        }

        while (vc_bridge_s2_next(&event))
        {
            handle(&event, now, false);
        }
        while (vc_bridge_s3_next(&event, g_vc_sm.st.state))
        {
            handle(&event, now, false);
        }
        while (vc_bridge_s4_next(&event))
        {
            handle(&event, now, false);
        }
        while (vc_bridge_s5_next(&event))
        {
            handle(&event, now, true);
        }

        /* 2. tick: also when events keep arriving faster than VC_TICK_MS */
        if ((now - last_tick) >= VC_TICK_MS)
        {
            last_tick    = now;
            tick.time_ms = now;
            /* Cast: odometer counts, read back as uint32 (junction turns) */
            tick.p2      = (int32_t)g_s2_odo_ticks;
            handle(&tick, now, false);
        }

        /* 5. telemetry (low rate, S1 decides what goes over MQTT) */
        if ((now - last_telem) >= VC_TELEMETRY_MS)
        {
            last_telem = now;
            vc_port_telemetry(&g_vc_sm.st);
        }
    }
}

/*** end of file ***/
