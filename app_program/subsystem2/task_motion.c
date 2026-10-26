/*
 *      task_motion.c
 *      Subsystem 2 -- the motion control task and its cyclic handler.
 *
 *      APIs shown: tk_slp_tsk, tk_wup_tsk, tk_can_wup, tk_rcv_mbf,
 *                  tk_wai_flg (polling), tk_set_flg, tk_loc_mtx, tk_unl_mtx
 *
 *      One control period (10 ms), in the order of the Design Review's
 *      motion-control statechart:
 *          RECEIVE_COMMAND  read every queued motion_cmd_t
 *          UPDATE_SPEED     read the encoders, measure wheel speed/distance
 *          (mode logic)     turn the current command into wheel set-points
 *          CALCULATE_PWM    per-wheel PID: set-point vs. measured speed
 *          DRIVE_MOTOR      write the duty to the motor driver
 *      then publish motion_status_t and sleep until the next tick.
 *
 *      This task is the ONLY code that touches the motors and encoders.
 */

#include "subsystem2.h"
#include "s2_config.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "motion_pid.h"

/* Private control state.  Only motion_task reads or writes it. */
typedef struct
{
    int32_t      state;       /* motion_state_t */
    int32_t      fault;       /* motion_fault_t */
    motion_cmd_t cmd;         /* command being executed */
    uint32_t     cmd_time_ms; /* when it arrived (DRIVE timeout) */

    int32_t   want_l; /* set-points before the ramp */
    int32_t   want_r;
    int32_t   tgt_l; /* ramped set-points, mm/s */
    int32_t   tgt_r;
    float32_t spd_l; /* measured speed, mm/s */
    float32_t spd_r;
    float32_t win_l[S2_SPEED_WINDOW]; /* mm moved per period */
    float32_t win_r[S2_SPEED_WINDOW];
    float32_t sum_l; /* sums of the two windows */
    float32_t sum_r;
    int32_t   win_i;
    float32_t odo_l; /* signed wheel odometry, mm */
    float32_t odo_r;
    float32_t heading; /* radians, + = left (CCW) */
    float32_t seg_l;   /* odometry when the command began */
    float32_t seg_r;

    int32_t  duty_l; /* last output, permille */
    int32_t  duty_r;
    uint32_t ticks_l;
    uint32_t ticks_r;
    uint32_t last_dt_l; /* ticks this period */
    uint32_t last_dt_r;
    uint32_t stall_ms_l;
    uint32_t stall_ms_r;

    pid_ctrl_t pid_l;
    pid_ctrl_t pid_r;
    uint32_t   loops;
    uint32_t   overruns;
} motion_ctx_t;

/* IF-S4-3: subsystem 4 (IMU) needs the vehicle speed to turn pitch
   into hump height.  Declared weak, so this still links while the old
   example subsystem 4 (without the function) is in the build: the
   address is then NULL and the call is skipped. */
extern void s4_set_speed_mm_s(int32_t speed_mm_s) __attribute__((weak));

static motion_ctx_t    g_motion_ctx;
static motion_status_t g_status_shared; /* IF-M2, under g_mtx_motion */

/* ---------------------------------------------------------------------
 *  Cyclic handler: interrupt context, short, never waits.
 * --------------------------------------------------------------------- */
void motion_cyclic_isr(void * p_exinf)
{
    (void)p_exinf;
    tk_wup_tsk(g_tid_motion);
}

/* ---------------------------------------------------------------------
 *  IF-M2 reader, called from OTHER tasks
 * --------------------------------------------------------------------- */
ER s2_get_status(motion_status_t * p_out)
{
    ER ercd = tk_loc_mtx(g_mtx_motion, TMO_FEVR);

    if (ercd < E_OK)
    {
        return ercd;
    }
    *p_out = g_status_shared;
    return tk_unl_mtx(g_mtx_motion);
}

/* ---------------------------------------------------------------------
 *  Helpers
 * --------------------------------------------------------------------- */
static int32_t clampi(int32_t value, int32_t lim)
{
    if (value > lim)
    {
        return lim;
    }
    if (value < -lim)
    {
        return -lim;
    }
    return value;
}

static int32_t absi(int32_t value)
{
    return (value < 0) ? -value : value;
}

static float32_t absf(float32_t value)
{
    return (value < 0.0f) ? -value : value;
}

static uint32_t now_ms(void)
{
    SYSTIM sys_time;

    tk_get_tim(&sys_time);
    return sys_time.lo;
}

static void reset_control(void)
{
    g_motion_ctx.want_l = g_motion_ctx.want_r = 0;
    g_motion_ctx.tgt_l = g_motion_ctx.tgt_r = 0;
    g_motion_ctx.stall_ms_l = g_motion_ctx.stall_ms_r = 0;
    pid_reset(&g_motion_ctx.pid_l);
    pid_reset(&g_motion_ctx.pid_r);
}

static void enter_idle(uint32_t event_bits)
{
    g_motion_ctx.state = MSTATE_IDLE;
    reset_control();
    drv_motor_brake(MOTOR_LEFT);
    drv_motor_brake(MOTOR_RIGHT);
    g_motion_ctx.duty_l = g_motion_ctx.duty_r = 0;
    if (event_bits != 0)
    {
        tk_set_flg(g_flg_motion, event_bits);
    }
}

/* NFR8 safe state: motors off, latched until MOTION_RESET */
static void enter_latched(int32_t state, int32_t fault)
{
    drv_motor_all_off();
    g_motion_ctx.duty_l = g_motion_ctx.duty_r = 0;
    reset_control();
    if (g_motion_ctx.state != state)
    {
        g_motion_ctx.state = state;
        g_motion_ctx.fault = fault;
        tk_set_flg(g_flg_motion,
                   (MSTATE_FAULT == state) ? MFLG_FAULT : MFLG_SAFE_STOPPED);
        tm_printf((uint8_t *)"motion: %s (fault %d)\n",
                  (MSTATE_FAULT == state) ? "FAULT" : "SAFE STOP", fault);
    }
}

/* ---------------------------------------------------------------------
 *  RECEIVE_COMMAND
 * --------------------------------------------------------------------- */
static void begin_segment(void)
{
    g_motion_ctx.seg_l = g_motion_ctx.odo_l;
    g_motion_ctx.seg_r = g_motion_ctx.odo_r;
    pid_reset(&g_motion_ctx.pid_l);
    pid_reset(&g_motion_ctx.pid_r);
}

static void apply_command(const motion_cmd_t * p_cmd)
{
    bool b_latched = ((MSTATE_SAFE_STOP == g_motion_ctx.state) ||
                      (MSTATE_FAULT == g_motion_ctx.state));

    /* While latched, only SAFE_STOP and RESET are accepted */
    if (b_latched && (p_cmd->type != MOTION_RESET) &&
        (p_cmd->type != MOTION_SAFE_STOP))
    {
        return;
    }

    switch (p_cmd->type)
    {
        case MOTION_STOP:
            g_motion_ctx.cmd   = *p_cmd;
            g_motion_ctx.state = MSTATE_STOPPING;
            break;

        case MOTION_DRIVE:
            g_motion_ctx.cmd = *p_cmd;
            g_motion_ctx.cmd.speed_mm_s =
                clampi(p_cmd->speed_mm_s, S2_MAX_SPEED_MM_S);
            g_motion_ctx.cmd.steer_mm_s =
                clampi(p_cmd->steer_mm_s, S2_MAX_SPEED_MM_S);
            g_motion_ctx.cmd_time_ms = now_ms();
            g_motion_ctx.state       = MSTATE_DRIVING;
            break;

        case MOTION_DRIVE_DIST:
            g_motion_ctx.cmd = *p_cmd;
            g_motion_ctx.cmd.speed_mm_s =
                absi(clampi(p_cmd->speed_mm_s, S2_MAX_SPEED_MM_S));
            if ((0 == p_cmd->distance_mm) || (0 == g_motion_ctx.cmd.speed_mm_s))
            {
                enter_idle(MFLG_CMD_DONE);
                break;
            }
            begin_segment();
            g_motion_ctx.state = MSTATE_DRIVING_DIST;
            break;

        case MOTION_TURN:
            g_motion_ctx.cmd = *p_cmd;
            g_motion_ctx.cmd.speed_mm_s =
                absi(clampi(p_cmd->speed_mm_s, S2_MAX_SPEED_MM_S));
            if (0 == g_motion_ctx.cmd.speed_mm_s)
            {
                g_motion_ctx.cmd.speed_mm_s = S2_TURN_SPEED_MM_S;
            }
            if (0 == p_cmd->angle_deg)
            {
                enter_idle(MFLG_CMD_DONE);
                break;
            }
            begin_segment();
            g_motion_ctx.state = MSTATE_TURNING;
            break;

        case MOTION_SAFE_STOP:
            g_motion_ctx.cmd = *p_cmd;
            enter_latched(MSTATE_SAFE_STOP, MFAULT_NONE);
            break;

        case MOTION_RESET:
            g_motion_ctx.cmd   = *p_cmd;
            g_motion_ctx.fault = MFAULT_NONE;
            enter_idle(0);
            break;

        default:
            tm_printf((uint8_t *)"motion: unknown command %d ignored\n",
                      p_cmd->type);
            break;
    }
}

static void receive_commands(void)
{
    motion_cmd_t cmd;
    UINT         ptn;

    /* Emergency stop first: it does not wait behind queued commands */
    if (E_OK == tk_wai_flg(g_flg_motion, MFLG_ESTOP_REQ, TWF_ORW | TWF_BITCLR,
                           &ptn, TMO_POL))
    {
        enter_latched(MSTATE_SAFE_STOP, MFAULT_NONE);
    }

    /* Then every queued command, oldest first: the newest wins */
    while ((int32_t)sizeof(motion_cmd_t) ==
           tk_rcv_mbf(g_mbf_motion_cmd, &cmd, TMO_POL))
    {
        apply_command(&cmd);
    }
}

/* ---------------------------------------------------------------------
 *  UPDATE_SPEED
 * --------------------------------------------------------------------- */
static float32_t signed_mm(uint32_t ticks, int32_t duty, float32_t speed)
{
    float32_t dist_mm = motion_ticks_to_mm(ticks);

    /* Single-channel encoder: direction = the way we are driving it,
       or, while coasting, the way it was already moving. */
    if ((duty < 0) || ((0 == duty) && (speed < 0.0f)))
    {
        dist_mm = -dist_mm;
    }
    return dist_mm;
}

static void update_speed(float32_t dt_s)
{
    float32_t delta_l;
    float32_t delta_r;

    g_motion_ctx.last_dt_l = drv_encoder_delta(MOTOR_LEFT);
    g_motion_ctx.last_dt_r = drv_encoder_delta(MOTOR_RIGHT);
    g_motion_ctx.ticks_l += g_motion_ctx.last_dt_l;
    g_motion_ctx.ticks_r += g_motion_ctx.last_dt_r;

    delta_l = signed_mm(g_motion_ctx.last_dt_l, g_motion_ctx.duty_l,
                        g_motion_ctx.spd_l);
    delta_r = signed_mm(g_motion_ctx.last_dt_r, g_motion_ctx.duty_r,
                        g_motion_ctx.spd_r);
    g_motion_ctx.odo_l += delta_l;
    g_motion_ctx.odo_r += delta_r;

    /* Differential-drive heading: + (left) when the right wheel goes
       further */
    g_motion_ctx.heading += (delta_r - delta_l) / S2_TRACK_MM;
    if (g_motion_ctx.heading > S2_PI)
    {
        g_motion_ctx.heading -= 2.0f * S2_PI;
    }
    if (g_motion_ctx.heading < -S2_PI)
    {
        g_motion_ctx.heading += 2.0f * S2_PI;
    }

    /* Moving window: add this period, drop the oldest */
    g_motion_ctx.sum_l += delta_l - g_motion_ctx.win_l[g_motion_ctx.win_i];
    g_motion_ctx.sum_r += delta_r - g_motion_ctx.win_r[g_motion_ctx.win_i];
    g_motion_ctx.win_l[g_motion_ctx.win_i] = delta_l;
    g_motion_ctx.win_r[g_motion_ctx.win_i] = delta_r;
    g_motion_ctx.win_i = (g_motion_ctx.win_i + 1) % S2_SPEED_WINDOW;
    g_motion_ctx.spd_l = g_motion_ctx.sum_l / (dt_s * S2_SPEED_WINDOW);
    g_motion_ctx.spd_r = g_motion_ctx.sum_r / (dt_s * S2_SPEED_WINDOW);
}

/* ---------------------------------------------------------------------
 *  Mode logic: current command -> wanted wheel speeds
 * --------------------------------------------------------------------- */
static int32_t slow_near_end(int32_t speed, float32_t remaining_mm,
                             float32_t slow_mm)
{
    int32_t vmin = S2_MIN_SPEED_MM_S;

    if (remaining_mm < slow_mm)
    {
        /* remaining_mm < slow_mm, so |scaled| <= |speed|: fits int32_t */
        int32_t scaled = (int32_t)(((float32_t)speed * remaining_mm) / slow_mm);
        speed          = (scaled > vmin) ? scaled : vmin;
    }
    return speed;
}

static void mode_logic(void)
{
    float32_t delta_l = g_motion_ctx.odo_l - g_motion_ctx.seg_l;
    float32_t delta_r = g_motion_ctx.odo_r - g_motion_ctx.seg_r;
    float32_t remaining;
    float32_t steer;
    int32_t   dir;
    int32_t   speed;

    switch (g_motion_ctx.state)
    {
        case MSTATE_DRIVING:
            if ((S2_CMD_TIMEOUT_MS > 0) &&
                ((now_ms() - g_motion_ctx.cmd_time_ms) > S2_CMD_TIMEOUT_MS))
            {
                tk_set_flg(g_flg_motion, MFLG_CMD_TIMEOUT);
                g_motion_ctx.state  = MSTATE_STOPPING;
                g_motion_ctx.want_l = g_motion_ctx.want_r = 0;
                break;
            }
            g_motion_ctx.want_l =
                g_motion_ctx.cmd.speed_mm_s - g_motion_ctx.cmd.steer_mm_s;
            g_motion_ctx.want_r =
                g_motion_ctx.cmd.speed_mm_s + g_motion_ctx.cmd.steer_mm_s;
            break;

        case MSTATE_DRIVING_DIST:
            dir       = (g_motion_ctx.cmd.distance_mm > 0) ? 1 : -1;
            remaining = (float32_t)absi(g_motion_ctx.cmd.distance_mm) -
                        ((float32_t)dir * (delta_l + delta_r) / 2.0f);
            if (remaining <= 0.0f)
            {
                enter_idle(MFLG_CMD_DONE);
                break;
            }
            speed = slow_near_end(g_motion_ctx.cmd.speed_mm_s, remaining,
                                  S2_DIST_SLOW_MM);
            /* Straight-line correction: slow the wheel that is ahead */
            steer = S2_STRAIGHT_K * (delta_l - delta_r); /* > 0: left ahead */
            /* steer is a few hundred mm/s at most: fits int32_t */
            g_motion_ctx.want_l = (dir * speed) - (int32_t)steer;
            g_motion_ctx.want_r = (dir * speed) + (int32_t)steer;
            break;

        case MSTATE_TURNING:
            dir = (g_motion_ctx.cmd.angle_deg > 0) ? 1 : -1; /* + = left */
            remaining = motion_turn_arc_mm(g_motion_ctx.cmd.angle_deg) -
                        ((absf(delta_l) + absf(delta_r)) / 2.0f);
            if (remaining <= 0.0f)
            {
                enter_idle(MFLG_CMD_DONE);
                break;
            }
            speed = slow_near_end(g_motion_ctx.cmd.speed_mm_s, remaining,
                                  S2_TURN_SLOW_MM);
            g_motion_ctx.want_l = -dir * speed; /* left turn: left wheel back */
            g_motion_ctx.want_r = dir * speed;
            break;

        case MSTATE_STOPPING:
            g_motion_ctx.want_l = g_motion_ctx.want_r = 0;
            if ((0 == g_motion_ctx.tgt_l) && (0 == g_motion_ctx.tgt_r) &&
                (absf(g_motion_ctx.spd_l) < S2_STOP_SPEED_MM_S) &&
                (absf(g_motion_ctx.spd_r) < S2_STOP_SPEED_MM_S))
            {
                enter_idle(MFLG_STOPPED);
            }
            break;

        default: /* IDLE, SAFE_STOP, FAULT: nothing to drive */
            g_motion_ctx.want_l = g_motion_ctx.want_r = 0;
            break;
    }
}

/* ---------------------------------------------------------------------
 *  CALCULATE_PWM
 * --------------------------------------------------------------------- */
static int32_t wheel_duty(pid_ctrl_t * p_pid, int32_t target,
                          float32_t measured, float32_t dt_s)
{
    int32_t duty;

    if (0 == target)
    {
        pid_reset(p_pid);
        return 0;
    }
    /* pid_update() clamps to +-S2_DUTY_MAX, so the cast keeps the value */
    duty = (int32_t)pid_update(p_pid, (float32_t)target, measured, dt_s);

    /* Below S2_DUTY_MIN_MOVE the motor only hums: lift to the minimum
       in the commanded direction (static friction compensation). */
    if ((target > 0) && (duty < S2_DUTY_MIN_MOVE))
    {
        duty = S2_DUTY_MIN_MOVE;
    }
    if ((target < 0) && (duty > -S2_DUTY_MIN_MOVE))
    {
        duty = -S2_DUTY_MIN_MOVE;
    }
    return duty;
}

static void calculate_pwm(float32_t dt_s)
{
    int32_t step = (S2_ACCEL_MM_S2 * S2_PERIOD_MS) / S2_MS_PER_S;

    g_motion_ctx.tgt_l =
        motion_ramp(g_motion_ctx.tgt_l, g_motion_ctx.want_l, step);
    g_motion_ctx.tgt_r =
        motion_ramp(g_motion_ctx.tgt_r, g_motion_ctx.want_r, step);

    g_motion_ctx.duty_l = wheel_duty(&g_motion_ctx.pid_l, g_motion_ctx.tgt_l,
                                     g_motion_ctx.spd_l, dt_s);
    g_motion_ctx.duty_r = wheel_duty(&g_motion_ctx.pid_r, g_motion_ctx.tgt_r,
                                     g_motion_ctx.spd_r, dt_s);
}

/* ---------------------------------------------------------------------
 *  DRIVE_MOTOR (+ stall check)
 * --------------------------------------------------------------------- */
static bool stalled(uint32_t * p_stall_ms, int32_t duty, uint32_t ticks)
{
    if ((absi(duty) >= S2_STALL_DUTY) && (0 == ticks))
    {
        *p_stall_ms += S2_PERIOD_MS;
    }
    else
    {
        *p_stall_ms = 0;
    }
    return (*p_stall_ms >= S2_STALL_MS);
}

static void drive_motor(void)
{
    if (stalled(&g_motion_ctx.stall_ms_l, g_motion_ctx.duty_l,
                g_motion_ctx.last_dt_l))
    {
        enter_latched(MSTATE_FAULT, MFAULT_STALL_LEFT);
        return;
    }
    if (stalled(&g_motion_ctx.stall_ms_r, g_motion_ctx.duty_r,
                g_motion_ctx.last_dt_r))
    {
        enter_latched(MSTATE_FAULT, MFAULT_STALL_RIGHT);
        return;
    }
    drv_motor_set(MOTOR_LEFT, g_motion_ctx.duty_l);
    drv_motor_set(MOTOR_RIGHT, g_motion_ctx.duty_r);
}

/* ---------------------------------------------------------------------
 *  Publish IF-M2
 * --------------------------------------------------------------------- */
static void publish_status(void)
{
    motion_status_t status;

    /* The float to int32_t casts below keep the value: speeds are below
       S2_MAX_SPEED_MM_S, odometry stays far below 2^31 mm (2000 km) and
       the heading is +-1800. */
    status.state         = g_motion_ctx.state;
    status.fault         = g_motion_ctx.fault;
    status.cmd_seq       = g_motion_ctx.cmd.seq;
    status.target_l_mm_s = g_motion_ctx.tgt_l;
    status.target_r_mm_s = g_motion_ctx.tgt_r;
    status.speed_l_mm_s  = (int32_t)g_motion_ctx.spd_l;
    status.speed_r_mm_s  = (int32_t)g_motion_ctx.spd_r;
    status.speed_mm_s =
        (int32_t)((g_motion_ctx.spd_l + g_motion_ctx.spd_r) / 2.0f);
    status.odo_l_mm = (int32_t)g_motion_ctx.odo_l;
    status.odo_r_mm = (int32_t)g_motion_ctx.odo_r;
    status.distance_mm =
        (int32_t)((g_motion_ctx.odo_l + g_motion_ctx.odo_r) / 2.0f);
    status.heading_ddeg = (int32_t)((g_motion_ctx.heading * 1800.0f) / S2_PI);
    status.duty_l       = g_motion_ctx.duty_l;
    status.duty_r       = g_motion_ctx.duty_r;
    status.ticks_l      = g_motion_ctx.ticks_l;
    status.ticks_r      = g_motion_ctx.ticks_r;
    status.loop_count   = g_motion_ctx.loops;
    status.overruns     = g_motion_ctx.overruns;
    status.time_ms      = now_ms();

    /* Never let a slow reader stall the control loop: give up after
       1 ms and publish next period instead. */
    if (E_OK == tk_loc_mtx(g_mtx_motion, 1))
    {
        g_status_shared = status;
        tk_unl_mtx(g_mtx_motion);
    }

    /* IF-S4-3: a plain word store inside S4, safe from any task */
    if (NULL != s4_set_speed_mm_s)
    {
        s4_set_speed_mm_s(status.speed_mm_s);
    }

#if S2_PRINT_EVERY > 0
    if (0 == (g_motion_ctx.loops % S2_PRINT_EVERY))
    {
        tm_printf((uint8_t *)"motion: st=%d v=%d/%d mm/s tgt=%d/%d "
                             "duty=%d/%d dist=%d mm hdg=%d (0.1 deg) ovr=%u\n",
                  status.state, status.speed_l_mm_s, status.speed_r_mm_s,
                  status.target_l_mm_s, status.target_r_mm_s, status.duty_l,
                  status.duty_r, status.distance_mm, status.heading_ddeg,
                  status.overruns);
    }
#endif
}

/* ---------------------------------------------------------------------
 *  The task
 * --------------------------------------------------------------------- */
void motion_task(int32_t stacd, void * p_exinf)
{
    const float32_t dt_s = (float32_t)S2_PERIOD_MS / (float32_t)S2_MS_PER_S;
    int32_t         missed;

    (void)stacd;
    (void)p_exinf;

    pid_init(&g_motion_ctx.pid_l, S2_PID_KFF, S2_PID_KP, S2_PID_KI, S2_PID_KD,
             S2_PID_I_LIMIT, (float32_t)S2_DUTY_MAX);
    pid_init(&g_motion_ctx.pid_r, S2_PID_KFF, S2_PID_KP, S2_PID_KI, S2_PID_KD,
             S2_PID_I_LIMIT, (float32_t)S2_DUTY_MAX);

    if ((E_OK != drv_motor_init()) || (E_OK != drv_encoder_init()))
    {
        tm_printf((uint8_t *)"motion: driver init failed\n");
        g_motion_ctx.state = MSTATE_IDLE; /* so enter_latched reports */
        enter_latched(MSTATE_FAULT, MFAULT_DRIVER);
    }
    else
    {
        enter_idle(0);
        tm_printf((uint8_t *)"motion: ready, %d ms loop on core %d\n",
                  S2_PERIOD_MS,
#if TK_SUPPORT_SMP
                  tk_get_prc() - 1
#else
                  0
#endif
        );
    }

    for (;;)
    {
        /* Sleep until motion_cyclic_isr wakes us */
        tk_slp_tsk(TMO_FEVR);

        /* Wake-ups that piled up mean we missed periods */
        missed = tk_can_wup(TSK_SELF);
        if (missed > 0)
        {
            /* missed > 0 here, so the cast keeps the value */
            g_motion_ctx.overruns += (uint32_t)missed;
        }
        g_motion_ctx.loops++;

        receive_commands(); /* RECEIVE_COMMAND */
        update_speed(dt_s); /* UPDATE_SPEED */

        if ((MSTATE_SAFE_STOP == g_motion_ctx.state) ||
            (MSTATE_FAULT == g_motion_ctx.state))
        {
            drv_motor_all_off(); /* stay safe */
        }
        else
        {
            mode_logic();
            if (g_motion_ctx.state != MSTATE_IDLE)
            {
                calculate_pwm(dt_s); /* CALCULATE_PWM */
                drive_motor();       /* DRIVE_MOTOR */
            }
        }
        publish_status();
    }
}

/*** end of file ***/
