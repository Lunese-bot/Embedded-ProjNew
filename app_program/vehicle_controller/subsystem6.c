/*
 *      subsystem6.c
 *      Subsystem 6 -- Vehicle Controller: creates its objects, starts its
 *      task, and provides the public calls of subsystem6.h.
 *
 *      Kernel objects (TEAM.md section 6):
 *        1 task            vehicle_ctrl_task
 *        1 message buffer  g_mbf_vc_events   IF-06 events in
 *        1 mutex           g_mtx_vc_status   IF-07 status snapshot
 *        1 event flag      g_flg_vc_state    IF-07 current state bits
 */

#include "subsystem6.h"

ID g_mbf_vc_events;
ID g_mtx_vc_status;
ID g_flg_vc_state;
ID g_tid_vc;

/* Written only by vehicle_ctrl_task, under g_mtx_vc_status */
vc_status_t g_vc_status_shared;

ER subsystem6_init(void)
{
    T_CMBF cmbf = { 0 };
    T_CMTX cmtx = { 0 };
    T_CFLG cflg = { 0 };

    /* Message buffer: events are COPIED in, so senders need no memory
       pool and can send from any task on either core. */
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = S6_EVENT_QUEUE_LEN * (sizeof(vc_event_t) + sizeof(int32_t));
    cmbf.maxmsz = sizeof(vc_event_t);
    g_mbf_vc_events = tk_cre_mbf(&cmbf);
    if (g_mbf_vc_events < E_OK)
    {
        return g_mbf_vc_events;
    }

    /* Mutex with priority inheritance: a low-priority reader (S1)
       cannot hold up the controller. */
    cmtx.mtxatr     = TA_INHERIT;
    g_mtx_vc_status = tk_cre_mtx(&cmtx);
    if (g_mtx_vc_status < E_OK)
    {
        return g_mtx_vc_status;
    }

    cflg.flgatr    = TA_TFIFO | TA_WMUL;
    cflg.iflgptn   = VC_FLG_STATE(VC_ST_IDLE);
    g_flg_vc_state = tk_cre_flg(&cflg);
    if (g_flg_vc_state < E_OK)
    {
        return g_flg_vc_state;
    }

    g_vc_status_shared.state    = VC_ST_IDLE;
    g_vc_status_shared.last_nav = NAV_NONE;
    return E_OK;
}

ER subsystem6_start(void)
{
    /* No affinity: event-driven, short, either core is fine */
    g_tid_vc = app_start_task(vehicle_ctrl_task, S6_PRI_VC, 0, "vehicle_ctrl");
    return (g_tid_vc < E_OK) ? g_tid_vc : E_OK;
}

ER vc_post_event(vc_source_t src, vc_event_type_t type, int32_t param1,
                 int32_t param2, TMO tmo)
{
    vc_event_t event;
    SYSTIM     now;

    tk_get_tim(&now);
    event.src     = (uint8_t)src;
    event.type    = (uint8_t)type;
    event.rsv     = 0;
    event.p1      = param1;
    event.p2      = param2;
    event.time_ms = now.lo;
    return tk_snd_mbf(g_mbf_vc_events, &event, sizeof(event), tmo);
}

ER vc_get_status(vc_status_t * p_out)
{
    ER ercd = tk_loc_mtx(g_mtx_vc_status, 50); /* held for a copy only */
    if (ercd < E_OK)
    {
        return ercd;
    }
    *p_out = g_vc_status_shared;
    tk_unl_mtx(g_mtx_vc_status);
    return E_OK;
}

/*** end of file ***/
