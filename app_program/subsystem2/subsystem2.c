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
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

ID                g_mbf_motion_cmd; /* IF-M1 */
ID                g_mtx_motion;     /* IF-M2 */
ID                g_flg_motion;     /* IF-M3 */
ID                g_tid_motion;
ID                g_cyc_motion;
volatile uint32_t g_s2_imu_stop; /* S2_IMU_CAUSE_* while the IMU interlock is
                                    set, 0 once the robot is set down */
/* The last IMU interlock and the stop generation it raised.  Kept after
   set-down, so the stop can still be explained when it is read later. */
static volatile uint32_t g_imu_stop_cause;
static volatile uint32_t g_imu_stop_epoch;
#if TK_SUPPORT_SMP
static UW g_stop_epoch;
static UW g_s2_seq;
#else
static volatile UW g_stop_epoch;
static volatile UW g_s2_seq;
#endif

uint32_t s2_stop_epoch(void)
{
#if TK_SUPPORT_SMP
    return atomic_add(&g_stop_epoch, 0);
#else
    return g_stop_epoch;
#endif
}

void s2_latch_imu_stop(uint32_t cause)
{
    if (0U != g_s2_imu_stop)
    {
        return;
    }
    if ((S2_IMU_CAUSE_LIFT != cause) && (S2_IMU_CAUSE_OVERTURN != cause))
    {
        cause = S2_IMU_CAUSE_UNKNOWN; /* still stops; never read as a lift */
    }
    /* Cause first: whoever sees this stop can already explain it. */
    g_imu_stop_cause = cause;
    g_s2_imu_stop    = cause;
    app_check(s2_emergency_stop(), "s2 IMU emergency stop");
    g_imu_stop_epoch = s2_stop_epoch();
    (void)tk_wup_tsk(g_tid_motion);
}

uint32_t s2_imu_stop_cause(uint32_t stop_epoch)
{
    uint32_t cause = g_s2_imu_stop;

    if ((0U == cause) && (UINT32_MAX != stop_epoch) &&
        (stop_epoch == g_imu_stop_epoch))
    {
        cause = g_imu_stop_cause; /* set down before the stop was read */
    }
    return cause;
}

/* The IMU says the robot is back down, level and still.  Motion stays in
   SAFE_STOP (motors off) until the normal MOTION_RESET, e.g. dashboard
   Start; no reboot needed (2026-11-11). */
void s2_clear_imu_stop(void)
{
    g_s2_imu_stop = 0U;
}

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
ER s2_send_cmd(motion_cmd_t * p_cmd)
{
#if !TK_SUPPORT_SMP
    UW intsts;
#endif
    if (NULL == p_cmd)
    {
        return E_PAR;
    }
    if (MOTION_SAFE_STOP == p_cmd->type)
    {
        return s2_emergency_stop();
    }
#if TK_SUPPORT_SMP
    p_cmd->seq = atomic_inc(&g_s2_seq);
#else
    DI(intsts);
    p_cmd->seq = ++g_s2_seq;
    EI(intsts);
#endif
    p_cmd->stop_epoch = s2_stop_epoch();
    if (UINT32_MAX == p_cmd->stop_epoch)
    {
        return E_LIMIT;
    }
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
#if TK_SUPPORT_SMP
    UW epoch;
    UW old;

    epoch = atomic_add(&g_stop_epoch, 0);
    while (epoch != UINT32_MAX)
    {
        old = atomic_cmpxchg(&g_stop_epoch, epoch + 1U, epoch);
        if (old == epoch)
        {
            break;
        }
        epoch = old;
    }
#else
    UW intsts;
    DI(intsts);
    if (g_stop_epoch != UINT32_MAX)
    {
        g_stop_epoch++;
    }
    EI(intsts);
#endif
    return tk_set_flg(g_flg_motion, MFLG_ESTOP_REQ);
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
