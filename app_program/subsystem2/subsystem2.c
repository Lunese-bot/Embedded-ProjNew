/*
 *      subsystem2.c
 *      Subsystem 2 -- Motion Control: creates its objects, starts its
 *      task, and provides the command/status helpers other tasks call.
 *
 *      APIs shown: tk_cre_mbf, tk_snd_mbf, tk_cre_mtx, tk_cre_flg,
 *                  tk_set_flg, tk_cre_cyc, tk_sta_cyc
 */

#include "subsystem2.h"
#include "s2_config.h"

ID g_mbf_motion_cmd; /* IF-M1 */
ID g_mtx_motion;     /* IF-M2 */
ID g_flg_motion;     /* IF-M3 */
ID g_tid_motion;
ID g_cyc_motion;

#if S2_KEEP_EXAMPLE_IF
ID g_sem_slots; /* example IF-02, see subsystem2.h */
ID g_mpf_samples;
#endif

#if S2_DEMO
static void demo_task(int32_t stacd, void * p_exinf);
#endif

ER subsystem2_init(void)
{
    T_CMBF cmbf = { 0 };
    T_CMTX cmtx = { 0 };
    T_CFLG cflg = { 0 };
    T_CCYC ccyc = { 0 };

    /* IF-M1: commands are copied into the buffer, so the Vehicle
       Controller never shares memory with us.  Each message also
       costs a small header inside the buffer (S2_MBF_MSG_OVERHEAD). */
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz =
        S2_CMD_QUEUE_LEN * (sizeof(motion_cmd_t) + S2_MBF_MSG_OVERHEAD);
    cmbf.maxmsz      = sizeof(motion_cmd_t);
    g_mbf_motion_cmd = tk_cre_mbf(&cmbf);
    if (g_mbf_motion_cmd < E_OK)
    {
        return g_mbf_motion_cmd;
    }

    /* IF-M2: priority inheritance, so a low-priority reader (telemetry)
       holding the lock cannot delay the control loop for long. */
    cmtx.mtxatr  = TA_INHERIT;
    g_mtx_motion = tk_cre_mtx(&cmtx);
    if (g_mtx_motion < E_OK)
    {
        return g_mtx_motion;
    }

    /* IF-M3: several tasks may wait (VC, telemetry, demo). */
    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0;
    g_flg_motion = tk_cre_flg(&cflg);
    if (g_flg_motion < E_OK)
    {
        return g_flg_motion;
    }

    /* The control loop's clock.  Created stopped; started in
       subsystem2_start() once the task it wakes exists. */
    ccyc.cycatr  = TA_HLNG;
    ccyc.cychdr  = (FP)motion_cyclic_isr;
    ccyc.cyctim  = S2_PERIOD_MS;
    ccyc.cycphs  = S2_PERIOD_MS;
    g_cyc_motion = tk_cre_cyc(&ccyc);
    if (g_cyc_motion < E_OK)
    {
        return g_cyc_motion;
    }

#if S2_KEEP_EXAMPLE_IF
    {
        /* Only so the example code in subsystems 3 and 4 still
           builds and runs.  Nothing in subsystem 2 uses these. */
        T_CSEM csem = { 0 };
        T_CMPF cmpf = { 0 };

        csem.sematr   = TA_TFIFO;
        csem.isemcnt  = S2_POOL_COUNT;
        csem.maxsem   = S2_POOL_COUNT;
        g_sem_slots   = tk_cre_sem(&csem);
        cmpf.mpfatr   = TA_TFIFO | TA_RNG3;
        cmpf.mpfcnt   = S2_POOL_COUNT;
        cmpf.blfsz    = sizeof(sample_msg_t);
        g_mpf_samples = tk_cre_mpf(&cmpf);
    }
#endif
    return E_OK;
}

ER subsystem2_start(void)
{
    /* Pinned to core 1 so the 10 ms loop is not delayed by the USB
       and WiFi service tasks, which run on core 0. */
    g_tid_motion =
        app_start_task(motion_task, S2_PRI_MOTION, TP_PRC2, "motion_task");
    if (g_tid_motion < E_OK)
    {
        return g_tid_motion;
    }

#if S2_DEMO
    {
        ID tid = app_start_task(demo_task, S2_PRI_DEMO, 0, "s2_demo_task");
        if (tid < E_OK)
        {
            return tid;
        }
    }
#endif
    return tk_sta_cyc(g_cyc_motion);
}

/* ---------------------------------------------------------------------
 *  IF-M1 helpers.  These run in the CALLER's task.
 * --------------------------------------------------------------------- */
static uint32_t g_s2_seq = 0;

ER s2_send_cmd(motion_cmd_t * p_cmd)
{
    /* Only the Vehicle Controller sends commands (one task), so this
       counter needs no lock.  s2_emergency_stop() from another task
       may share a number; that is harmless, seq is only a label. */
    p_cmd->seq = ++g_s2_seq;
    return tk_snd_mbf(g_mbf_motion_cmd, p_cmd, sizeof(motion_cmd_t), TMO_POL);
}

static ER send_simple(int32_t type, int32_t speed, int32_t steer, int32_t dist,
                      int32_t angle)
{
    motion_cmd_t cmd = { 0 };

    cmd.type        = type;
    cmd.speed_mm_s  = speed;
    cmd.steer_mm_s  = steer;
    cmd.distance_mm = dist;
    cmd.angle_deg   = angle;
    return s2_send_cmd(&cmd);
}

ER s2_cmd_stop(void)
{
    return send_simple(MOTION_STOP, 0, 0, 0, 0);
}

ER s2_cmd_drive(int32_t speed_mm_s, int32_t steer_mm_s)
{
    return send_simple(MOTION_DRIVE, speed_mm_s, steer_mm_s, 0, 0);
}

ER s2_cmd_drive_dist(int32_t speed_mm_s, int32_t distance_mm)
{
    return send_simple(MOTION_DRIVE_DIST, speed_mm_s, 0, distance_mm, 0);
}

ER s2_cmd_turn(int32_t speed_mm_s, int32_t angle_deg)
{
    return send_simple(MOTION_TURN, speed_mm_s, 0, 0, angle_deg);
}

ER s2_cmd_reset(void)
{
    return send_simple(MOTION_RESET, 0, 0, 0, 0);
}

ER s2_emergency_stop(void)
{
    /* The flag works even if the command queue is full */
    ER ercd = tk_set_flg(g_flg_motion, MFLG_ESTOP_REQ);

    (void)send_simple(MOTION_SAFE_STOP, 0, 0, 0, 0);
    return ercd;
}

#if S2_DEMO
/* ---------------------------------------------------------------------
 *  Bench demo: stands in for the Vehicle Controller while it does not
 *  exist yet.  Build with S2_DEMO 1 in s2_config.h.
 * --------------------------------------------------------------------- */
static void demo_task(int32_t stacd, void * p_exinf)
{
    UINT           ptn;
    const uint32_t wait_bits = MFLG_CMD_DONE | MFLG_FAULT | MFLG_SAFE_STOPPED;

    (void)stacd;
    (void)p_exinf;
    tk_dly_tsk(2000); /* time to put the robot down */

    for (;;)
    {
        tm_printf((uint8_t *)"s2 demo: forward 300 mm\n");
        s2_cmd_drive_dist(200, 300);
        tk_wai_flg(g_flg_motion, wait_bits, TWF_ORW | TWF_BITCLR, &ptn, 10000);
        if (0U != (ptn & (MFLG_FAULT | MFLG_SAFE_STOPPED)))
        {
            break;
        }
        tk_dly_tsk(500);

        tm_printf((uint8_t *)"s2 demo: turn right 90 deg\n");
        s2_cmd_turn(S2_TURN_SPEED_MM_S, -90);
        tk_wai_flg(g_flg_motion, wait_bits, TWF_ORW | TWF_BITCLR, &ptn, 10000);
        if (0U != (ptn & (MFLG_FAULT | MFLG_SAFE_STOPPED)))
        {
            break;
        }
        tk_dly_tsk(1000);
    }
    tm_printf((uint8_t *)"s2 demo: stopped by fault/safe stop\n");
    tk_exd_tsk();
}
#endif

/*** end of file ***/
