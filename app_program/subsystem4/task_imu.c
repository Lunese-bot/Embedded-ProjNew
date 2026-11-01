/*
 *      task_imu.c
 *      Subsystem 4 -- imu_task (owning task of I2C0) and its cyclic handler.
 *
 *      Every 20 ms the cyclic handler wakes imu_task, which:
 *        1. serves requests from other subsystems (calibrate, reset peaks)
 *        2. reads the LSM303DLHC                     (drv_lsm303.c)
 *        3. runs the terrain state machine            (imu_terrain.c)
 *        4. publishes imu_status_t under g_mtx_imu        (IF-S4-2)
 *        5. sends imu_event_msg_t to g_mbf_imu_evt        (IF-S4-1)
 *
 *      Fault handling (FR8, NFR5, Statechart 4 "Error Handling"):
 *        S4_I2C_MAX_CONSEC failed reads in a row -> ERROR + IMU_EVT_FAULT,
 *        then re-init every S4_REINIT_EVERY periods, at most S4_REINIT_MAX
 *        times -> IMU_EVT_FAULT_FATAL.  The Vehicle Controller decides
 *        whether that means Safe Stop; this task never touches the motors.
 */

#include "subsystem4.h"
#include "drv_lsm303.h"

#define S4_VERBOSE     1  /* 1 = print a status line ... */
#define S4_PRINT_EVERY 50 /* ... every 50 periods (1 s) */

uint32_t g_s4_evt_dropped;
uint32_t g_s4_overruns;
uint32_t g_s4_i2c_errors;

static imu_terrain_t g_terrain; /* only imu_task touches this */
static uint16_t      g_evt_seq;

/* imu_task fault-handling state (only imu_task touches these) */
static int32_t g_consec_err;  /* failed reads in a row */
static int32_t g_reinit_wait; /* periods since the last re-init */
static int32_t g_reinit_cnt;  /* re-init attempts that failed */
static bool    g_b_fatal;     /* gave up re-initialising */

#define S4_TICK_TIMEOUT_PERIODS 5 /* tk_slp_tsk timeout, in periods */

/*
 * Cyclic handler: interrupt context.  Short, never waits.
 */
void imu_cyclic_isr(void * p_exinf)
{
    (void)p_exinf;
    tk_wup_tsk(g_tid_imu);
}

/* Sensor axes -> robot axes (subsystem4_cfg.h) */
static void remap(const lsm303_raw_t * p_raw, imu_sample_t * p_sample)
{
    const int16_t acc[3] = { p_raw->ax_mg, p_raw->ay_mg, p_raw->az_mg };
    const int16_t mag[3] = { p_raw->mx, p_raw->my, p_raw->mz };

    p_sample->ax_mg = (int16_t)(S4_ACC_X_SIGN * acc[S4_ACC_X_SRC]);
    p_sample->ay_mg = (int16_t)(S4_ACC_Y_SIGN * acc[S4_ACC_Y_SRC]);
    p_sample->az_mg = (int16_t)(S4_ACC_Z_SIGN * acc[S4_ACC_Z_SRC]);
    p_sample->mx    = (int16_t)(S4_MAG_X_SIGN * mag[S4_ACC_X_SRC]);
    p_sample->my    = (int16_t)(S4_MAG_Y_SIGN * mag[S4_ACC_Y_SRC]);
    p_sample->mz    = (int16_t)(S4_MAG_Z_SIGN * mag[S4_ACC_Z_SRC]);
}

static void send_event(imu_event_type_t type, imu_severity_t sev, int32_t value)
{
    imu_event_msg_t msg;
    SYSTIM          now;

    tk_get_tim(&now);
    msg.type     = (uint8_t)type;
    msg.severity = (uint8_t)sev;
    msg.seq      = g_evt_seq++;
    msg.value    = value;
    msg.time_ms  = now.lo;

    /* Never block the 20 ms loop: if the controller is not reading,
       count the loss instead (the seq number shows the gap). */
    if (tk_snd_mbf(g_mbf_imu_evt, &msg, sizeof(msg), TMO_POL) < E_OK)
    {
        g_s4_evt_dropped++;
    }
#if S4_VERBOSE
    tm_printf((uint8_t *)"imu: event %d sev %d value %d\n", (int32_t)type,
              (int32_t)sev, (int32_t)value);
#endif
}

/* Turn the algorithm's IMU_EV_* bits into messages */
static void report(uint32_t events, const imu_status_t * p_status)
{
    int32_t tilt;

    if (0U != (events & IMU_EV_CAL_DONE))
    {
        send_event(IMU_EVT_CAL_DONE, IMU_SEV_INFO, 0);
    }
    if (0U != (events & IMU_EV_CAL_FAILED))
    {
        send_event(IMU_EVT_CAL_FAILED, IMU_SEV_NONCRITICAL, 0);
    }
    if (0U != (events & IMU_EV_HUMP_START))
    {
        send_event(IMU_EVT_HUMP_START, IMU_SEV_INFO, 0);
    }
    if (0U != (events & IMU_EV_HUMP_PEAK))
    {
        send_event(IMU_EVT_HUMP_PEAK, IMU_SEV_INFO, p_status->hump_height_um);
    }
    if (0U != (events & IMU_EV_HUMP_END))
    {
        send_event(IMU_EVT_HUMP_END, IMU_SEV_INFO, p_status->last_peak_um);
    }
    if (0U != (events & IMU_EV_HUMP_ABORT))
    {
        send_event(IMU_EVT_HUMP_ABORT, IMU_SEV_INFO, 0);
    }
    if (0U != (events & IMU_EV_IMPACT))
    {
        send_event(IMU_EVT_IMPACT, IMU_SEV_CRITICAL, p_status->last_impact_mg);
    }
    if (0U != (events & IMU_EV_TILT_LIMIT))
    {
        tilt = p_status->pitch_cdeg;
        if (((p_status->roll_cdeg < 0)
                 ? -p_status->roll_cdeg
                 : p_status->roll_cdeg) > ((tilt < 0) ? -tilt : tilt))
        {
            tilt = p_status->roll_cdeg;
        }
        send_event(IMU_EVT_TILT_LIMIT, IMU_SEV_CRITICAL, tilt);
    }
    if (0U != (events & IMU_EV_MOTION_CHANGE))
    {
        send_event(IMU_EVT_MOTION, IMU_SEV_INFO, (int32_t)p_status->motion);
    }
}

static void publish(void)
{
    if (E_OK == tk_loc_mtx(g_mtx_imu, TMO_FEVR))
    {
        g_s4_status = g_terrain.st;
        tk_unl_mtx(g_mtx_imu);
    }
}

/* 1. Requests from other subsystems (never wait for them) */
static void handle_requests(void)
{
    UINT req;

    if (E_OK == tk_wai_flg(g_flg_imu_req, S4_REQ_CALIBRATE | S4_REQ_PEAK_RESET,
                           TWF_ORW | TWF_CLR, &req, TMO_POL))
    {
        if (0U != (req & S4_REQ_CALIBRATE))
        {
            imu_terrain_recalibrate(&g_terrain);
        }
        if (0U != (req & S4_REQ_PEAK_RESET))
        {
            imu_terrain_reset_peaks(&g_terrain);
        }
    }
}

/* 2a. In ERROR: bounded re-init attempts */
static void recover_step(void)
{
    ER ercd;

    if (!g_b_fatal)
    {
        g_reinit_wait++;
    }
    if ((!g_b_fatal) && (g_reinit_wait >= S4_REINIT_EVERY))
    {
        g_reinit_wait = 0;
        lsm303_close();
        ercd = lsm303_init();
        if (E_OK == ercd)
        {
            g_consec_err = 0;
            g_reinit_cnt = 0;
            imu_terrain_set_fault(&g_terrain, 0); /* recalibrates */
            send_event(IMU_EVT_RECOVERED, IMU_SEV_INFO, 0);
        }
        else
        {
            g_reinit_cnt++;
            if (g_reinit_cnt >= S4_REINIT_MAX)
            {
                g_b_fatal = true;
                send_event(IMU_EVT_FAULT_FATAL, IMU_SEV_RECOVERABLE, ercd);
            }
            /* else: try again in S4_REINIT_EVERY periods */
        }
        publish();
    }
}

#if S4_VERBOSE
static void print_status(void)
{
    /* Cast: the console API takes uint8_t strings and int32_t args. */
    tm_printf((uint8_t *)"imu: %s %s pitch=%d roll=%d hdg=%d rate=%d"
                         " h=%dum max=%dum humps=%d impacts=%d\n",
              imu_state_name(g_terrain.st.state),
              imu_motion_name(g_terrain.st.motion), g_terrain.st.pitch_cdeg,
              g_terrain.st.roll_cdeg, g_terrain.st.heading_cdeg,
              (int32_t)g_terrain.st.turn_rate_cdeg_s,
              (int32_t)g_terrain.st.hump_height_um,
              (int32_t)g_terrain.st.max_peak_um, g_terrain.st.hump_count,
              g_terrain.st.impact_count);
}
#endif

/* 2b..5. Read the sensor, run the algorithm, publish, send events */
static void sample_step(uint32_t ticks)
{
    imu_sample_t sample;
    lsm303_raw_t raw;
    uint32_t     events;
    ER           ercd;

    ercd = lsm303_read(&raw);
    if (ercd < E_OK)
    {
        /* skip one sample; filters ride through */
        g_s4_i2c_errors++;
        g_consec_err++;
        if (g_consec_err >= S4_I2C_MAX_CONSEC)
        {
            imu_terrain_set_fault(&g_terrain, 1);
            send_event(IMU_EVT_FAULT, IMU_SEV_RECOVERABLE, ercd);
            publish();
        }
    }
    else
    {
        g_consec_err = 0;

        /* 3. Algorithm */
        remap(&raw, &sample);
        events = imu_terrain_update(&g_terrain, &sample, g_s4_speed_mm_s);

        /* 4. Publish, 5. events */
        publish();
        if (0U != events)
        {
            report(events, &g_terrain.st);
        }

#if S4_VERBOSE
        if (0U == (ticks % S4_PRINT_EVERY))
        {
            print_status();
        }
#else
        (void)ticks;
#endif
    }
}

void imu_task(int32_t stacd, void * p_exinf)
{
    imu_cfg_t cfg;
    ER        ercd;
    uint32_t  ticks = 0U;

    (void)stacd;
    (void)p_exinf;

    g_consec_err  = 0;
    g_reinit_wait = 0;
    g_reinit_cnt  = 0;
    g_b_fatal     = false;

    imu_terrain_default_cfg(&cfg);
    imu_terrain_init(&g_terrain, &cfg); /* starts in CALIBRATING */

    ercd = lsm303_init();
    if (ercd < E_OK)
    {
        tm_printf((uint8_t *)"imu: LSM303 init failed (%d)\n", ercd);
        imu_terrain_set_fault(&g_terrain, 1);
        send_event(IMU_EVT_FAULT, IMU_SEV_RECOVERABLE, ercd);
    }
    else
    {
        tm_printf((
            uint8_t *)"imu: LSM303 ready, keep the robot still to calibrate\n");
    }
    publish();

    for (;;)
    {
        /* Wait for the cyclic handler.  A timeout means the handler
           stopped, which should never happen: report and carry on. */
        ercd = tk_slp_tsk(S4_PERIOD_MS * S4_TICK_TIMEOUT_PERIODS);
        if (E_TMOUT == ercd)
        {
            tm_printf((uint8_t *)"imu: no tick from cyc_imu\n");
        }
        /* Extra queued wake-ups = we overran a period.  Drop them so
           the loop does not run back-to-back to "catch up". */
        if (tk_can_wup(TSK_SELF) > 0)
        {
            g_s4_overruns++;
        }
        ticks++;

        handle_requests();

        if (IMU_ST_ERROR == g_terrain.st.state)
        {
            recover_step();
        }
        else
        {
            sample_step(ticks);
        }
    }
}

/*** end of file ***/
