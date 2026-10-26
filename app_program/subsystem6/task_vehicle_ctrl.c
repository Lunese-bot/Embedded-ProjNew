/*
 *      task_vehicle_ctrl.c
 *      Subsystem 6 -- the Vehicle Controller task.
 *
 *      Loop:
 *        1. wait for an event on g_mbf_vc_events, at most S6_TICK_MS, then
 *           read the channels S2..S5 own (vc_bridge_s2..s5.c), so their
 *           events wait at most one tick, and S3's steering is forwarded
 *           every tick (10 ms, NFR2)
 *        2. no event (or events arriving non-stop) -> make a TICK event,
 *           so state time limits are still checked
 *        3. vc_sm_handle() decides; this task carries out its actions
 *           through vc_ports.c
 *        4. publish the new status (mutex snapshot + state flag)
 *        5. every S6_TELEMETRY_MS, send the status to S1
 *
 *      APIs used: tk_rcv_mbf, tk_get_tim, tk_loc_mtx/tk_unl_mtx,
 *                 tk_clr_flg/tk_set_flg
 */

#include "subsystem6.h"
#include "vc_sm.h"
#include "vc_ports.h"
#include "vc_bridge.h"

static vc_sm_t      g_vc_sm; /* the live controller */
static vc_actions_t g_vc_actions;

static uint32_t now_ms(void)
{
    SYSTIM sys_time;
    tk_get_tim(&sys_time);
    return sys_time.lo;
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
    vc_state_t before   = g_vc_sm.st.state;
    uint32_t   rejected = g_vc_sm.st.moves_rejected;

    vc_sm_handle(&g_vc_sm, p_event, now, &g_vc_actions);
    run_actions(&g_vc_actions);

    if (b_from_s5)
    {
        vc_bridge_s5_after(p_event, g_vc_sm.st.state,
                           g_vc_sm.st.moves_rejected != rejected);
    }
    if ((VC_EV_MANOEUVRE_DONE == p_event->type) &&
        (VC_ST_AVOID_OBSTACLE == before))
    {
        vc_bridge_s5_manoeuvre_done(p_event->p1 != 0);
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
        len = tk_rcv_mbf(g_mbf_vc_events, &event, S6_TICK_MS);
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

        /* 2. tick: also when events keep arriving faster than S6_TICK_MS */
        if ((now - last_tick) >= S6_TICK_MS)
        {
            last_tick    = now;
            tick.time_ms = now;
            handle(&tick, now, false);
        }

        /* 5. telemetry (low rate, S1 decides what goes over MQTT) */
        if ((now - last_telem) >= S6_TELEMETRY_MS)
        {
            last_telem = now;
            vc_port_telemetry(&g_vc_sm.st);
        }
    }
}

/*** end of file ***/
