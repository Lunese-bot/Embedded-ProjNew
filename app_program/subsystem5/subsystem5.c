/*
 *      subsystem5.c
 *      Subsystem 5 -- Ultrasonic Obstacle Detection and Avoidance.
 *      Creates the kernel objects, starts the two tasks, and keeps the
 *      status snapshot other subsystems read with s5_get_status().
 *
 *      Tasks (design review section 3):
 *        s5_scan_task   Obstacle Detection & Scanning.  Owns the ultrasonic
 *                       sensor and the servo.  Pings every 20 ms, and runs
 *                       the coarse + fine scan when asked.
 *        s5_avoid_task  Obstacle Avoidance.  Receives the controller's
 *                       commands, plans the bypass, sends motion requests,
 *                       and reacquires the line (bounded retries).
 *
 *      Kernel objects:
 *        g_mbf_s5_cmd   message buffer  VC -> S5        (public)
 *        g_mbf_s5_evt   message buffer  S5 -> VC        (public)
 *        g_flg_s5       event flag      scan <-> avoid  (private)
 *        g_mtx_s5       mutex           guards s5_status (private)
 */

#include "s5_local.h"
#include "drv_ultrasonic.h" /* US_NO_ECHO */
#include "../subsystem2/subsystem2.h" /* s2_emergency_stop (stop path only) */

ID       g_mbf_s5_cmd;
ID       g_mbf_s5_evt;
ID       g_flg_s5;
ID       g_mtx_s5;
ID       g_tid_s5_scan;
ID       g_tid_s5_avoid;
uint32_t g_s5_evt_dropped;

static s5_status_t g_s5_status;

#define S5_STATUS_TMO_MS    100 /* s5_get_status(): longest mutex wait */

ER subsystem5_init(void)
{
    T_CMBF cmbf = { 0 };
    T_CFLG cflg = { 0 };
    T_CMTX cmtx = { 0 };

    /* Each message in a message buffer also costs a small header, so
       allow a few extra bytes per message. */
    cmbf.mbfatr  = TA_TFIFO;
    cmbf.maxmsz  = sizeof(s5_cmd_t);
    cmbf.bufsz   = S5_CMD_BUF_MSGS * (sizeof(s5_cmd_t) + S5_MBF_MSG_OVERHEAD);
    g_mbf_s5_cmd = tk_cre_mbf(&cmbf);
    if (g_mbf_s5_cmd < E_OK)
    {
        return g_mbf_s5_cmd;
    }

    cmbf.maxmsz  = sizeof(s5_evt_t);
    cmbf.bufsz   = S5_EVT_BUF_MSGS * (sizeof(s5_evt_t) + S5_MBF_MSG_OVERHEAD);
    g_mbf_s5_evt = tk_cre_mbf(&cmbf);
    if (g_mbf_s5_evt < E_OK)
    {
        return g_mbf_s5_evt;
    }

    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = (0 != S5_ENABLE_AT_START) ? S5_FLG_ENABLED : 0;
    g_flg_s5     = tk_cre_flg(&cflg);
    if (g_flg_s5 < E_OK)
    {
        return g_flg_s5;
    }

    /* Priority inheritance: a low-priority reader (telemetry) holding
       the lock cannot hold up the S5 tasks for long. */
    cmtx.mtxatr = TA_INHERIT;
    g_mtx_s5    = tk_cre_mtx(&cmtx);
    if (g_mtx_s5 < E_OK)
    {
        return g_mtx_s5;
    }

    g_s5_status = (s5_status_t){ 0 };
    g_s5_status.state =
        (0 != S5_ENABLE_AT_START) ? S5_ST_MONITORING : S5_ST_IDLE;
    return E_OK;
}

ER subsystem5_start(void)
{
    /* No core affinity yet.  Pin the scan task (busy-waits on the echo)
       to one core with TP_PRC1/TP_PRC2 once the team agrees core use. */
    g_tid_s5_avoid =
        app_start_task(s5_avoid_task, S5_PRI_AVOID, 0, "s5_avoid_task");
    if (g_tid_s5_avoid < E_OK)
    {
        return g_tid_s5_avoid;
    }
    g_tid_s5_scan =
        app_start_task(s5_scan_task, S5_PRI_SCAN, 0, "s5_scan_task");
    return (g_tid_s5_scan < E_OK) ? g_tid_s5_scan : E_OK;
}

/* ------------------------------------------------------------------ */
/*  Status snapshot                                                    */
/* ------------------------------------------------------------------ */

ER s5_get_status(s5_status_t * p_out)
{
    ER ercd;

    if (NULL == p_out)
    {
        return E_PAR;
    }
    ercd = tk_loc_mtx(g_mtx_s5, S5_STATUS_TMO_MS);
    if (ercd < E_OK)
    {
        return ercd;
    }
    *p_out = g_s5_status;
    tk_unl_mtx(g_mtx_s5);
    return E_OK;
}

ER s5_get_status_packed(uint16_t * p_word)
{
    s5_status_t st;
    ER          ercd;
    bool        detected;

    if (NULL == p_word)
    {
        return E_PAR;
    }
    *p_word = 0U;
    ercd = s5_get_status(&st);
    if (ercd < E_OK)
    {
        return ercd;
    }
    detected = (US_NO_ECHO != st.last_distance_mm) &&
               (st.last_distance_mm < S5_DETECT_MM);
    *p_word = s5_pack_status(st.last_distance_mm, st.state, detected);
    return E_OK;
}

/* Lock / unlock the status snapshot.  BARR-C 6.3: functions, not a
   statement macro. */
static bool s5_lock(void)
{
    return (E_OK == tk_loc_mtx(g_mtx_s5, TMO_FEVR));
}

static void s5_unlock(void)
{
    tk_unl_mtx(g_mtx_s5);
}

void s5_set_state(s5_state_t state)
{
    if (s5_lock())
    {
        g_s5_status.state = (uint8_t)state; /* Cast: enum fits 8 bits */
        s5_unlock();
    }
}

void s5_set_fault(s5_fault_code_t fault)
{
    if (s5_lock())
    {
        g_s5_status.last_fault = (uint8_t)fault; /* Cast: enum fits 8 bits */
        s5_unlock();
    }
}

void s5_note_ping(uint16_t dist_mm)
{
    if (s5_lock())
    {
        g_s5_status.pings++;
        g_s5_status.last_distance_mm = dist_mm;
        if (0U == dist_mm)
        {
            g_s5_status.echo_timeouts++;
        }
        s5_unlock();
    }
}

void s5_count_obstacle(void)
{
    if (s5_lock())
    {
        g_s5_status.obstacles++;
        s5_unlock();
    }
}

void s5_count_avoid_ok(void)
{
    if (s5_lock())
    {
        g_s5_status.avoid_ok++;
        s5_unlock();
    }
}

void s5_store_profile(const s5_profile_t * p_profile)
{
    if (s5_lock())
    {
        g_s5_status.profile = *p_profile;
        s5_unlock();
    }
}

void s5_load_profile(s5_profile_t * p_profile)
{
    if (s5_lock())
    {
        *p_profile = g_s5_status.profile;
        s5_unlock();
    }
}

/* ------------------------------------------------------------------ */
/*  Events to the Vehicle Controller                                   */
/* ------------------------------------------------------------------ */

ER s5_send_evt(s5_evt_t * p_evt)
{
    SYSTIM now;
    ER     ercd;

    tk_get_tim(&now);
    p_evt->time_ms = now.lo;

    /* Never block: if the controller is not reading, drop and count */
    ercd = tk_snd_mbf(g_mbf_s5_evt, p_evt, sizeof(*p_evt), TMO_POL);
    if (ercd < E_OK)
    {
        g_s5_evt_dropped++;
    }
    return ercd;
}

/* The stopping-distance check in obstacle_logic.h assumes S2's top speed */
_Static_assert(S5_TOP_SPEED_MM_S == S2_MAX_SPEED_MM_S,
               "update S5_TOP_SPEED_MM_S: S2_MAX_SPEED_MM_S changed");

/* Latched stop path (Version C gap: "obstacle event may be dropped if
   the VC event queue is full while the robot continues moving"). */
s5_stop_fn_t g_s5_stop_fn = s2_emergency_stop;
uint32_t     g_s5_estops;

ER s5_send_critical_to(ID mbf, s5_evt_t * p_evt)
{
    SYSTIM now;
    ER     ercd;

    tk_get_tim(&now);
    p_evt->time_ms = now.lo;

    ercd = tk_snd_mbf(mbf, p_evt, sizeof(*p_evt), TMO_POL);
    if (ercd < E_OK)
    {
        g_s5_evt_dropped++;
        g_s5_estops++;
        tm_printf((uint8_t *)"s5: event %d LOST (queue full) -> emergency stop\n",
                  (int32_t)p_evt->type);
        (void)g_s5_stop_fn(); /* motors off within one S2 period, latched */
    }
    return ercd;
}

const char * s5_state_name(s5_state_t state)
{
    const char * p_name;

    switch (state)
    {
        case S5_ST_IDLE:
            p_name = "IDLE";
            break;
        case S5_ST_MONITORING:
            p_name = "MONITORING";
            break;
        case S5_ST_DETECTED:
            p_name = "DETECTED";
            break;
        case S5_ST_COARSE_SCAN:
            p_name = "COARSE_SCAN";
            break;
        case S5_ST_FINE_SCAN:
            p_name = "FINE_SCAN";
            break;
        case S5_ST_PLANNING:
            p_name = "PLANNING";
            break;
        case S5_ST_BYPASSING:
            p_name = "BYPASSING";
            break;
        case S5_ST_SEARCH_LINE:
            p_name = "SEARCH_LINE";
            break;
        case S5_ST_LINE_RECOVERY:
            p_name = "LINE_RECOVERY";
            break;
        case S5_ST_ERROR:
            p_name = "ERROR";
            break;
        default:
            p_name = "?";
            break;
    }
    return p_name;
}

/*** end of file ***/
