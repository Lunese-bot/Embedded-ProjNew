/*
 *	task_motion.c
 *	Subsystem 2 -- the motion control task and its cyclic handler.
 *
 *	APIs shown: tk_slp_tsk, tk_wup_tsk, tk_can_wup, tk_rcv_mbf,
 *	            tk_wai_flg (polling), tk_set_flg, tk_loc_mtx, tk_unl_mtx
 *
 *	One control period (10 ms), in the order of the Design Review's
 *	motion-control statechart:
 *	    RECEIVE_COMMAND  read every queued MOTION_CMD
 *	    UPDATE_SPEED     read the encoders, measure wheel speed/distance
 *	    (mode logic)     turn the current command into wheel set-points
 *	    CALCULATE_PWM    per-wheel PID: set-point vs. measured speed
 *	    DRIVE_MOTOR      write the duty to the motor driver
 *	then publish MOTION_STATUS and sleep until the next tick.
 *
 *	This task is the ONLY code that touches the motors and encoders.
 */

#include "subsystem2.h"
#include "s2_config.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "motion_pid.h"

/* Private control state.  Only motion_task reads or writes it. */
typedef struct {
	INT		state;		/* MOTION_STATE */
	INT		fault;		/* MOTION_FAULT */
	MOTION_CMD	cmd;		/* command being executed */
	UW		cmd_time_ms;	/* when it arrived (DRIVE timeout) */

	INT		want_l, want_r;	/* set-points before the ramp */
	INT		tgt_l, tgt_r;	/* ramped set-points, mm/s */
	float		spd_l, spd_r;	/* measured speed, mm/s */
	float		win_l[S2_SPEED_WINDOW];	/* mm moved per period */
	float		win_r[S2_SPEED_WINDOW];
	float		sum_l, sum_r;	/* sums of the two windows */
	INT		win_i;
	float		odo_l, odo_r;	/* signed wheel odometry, mm */
	float		heading;	/* radians, + = left (CCW) */
	float		seg_l, seg_r;	/* odometry when the command began */

	INT		duty_l, duty_r;	/* last output, permille */
	UW		ticks_l, ticks_r;
	UW		last_dt_l, last_dt_r;	/* ticks this period */
	UW		stall_ms_l, stall_ms_r;

	PID_CTRL	pid_l, pid_r;
	UW		loops, overruns;
} MOTION_CTX;

/* IF-S4-3: subsystem 4 (IMU) needs the vehicle speed to turn pitch
   into hump height.  Declared weak, so this still links while the old
   example subsystem 4 (without the function) is in the build: the
   address is then NULL and the call is skipped. */
IMPORT void	s4_set_speed_mm_s(W speed_mm_s) __attribute__((weak));

LOCAL MOTION_CTX	mc;
LOCAL MOTION_STATUS	status_shared;	/* IF-M2, under mtx_motion */

/* ---------------------------------------------------------------------
 *  Cyclic handler: interrupt context, short, never waits.
 * --------------------------------------------------------------------- */
EXPORT void motion_cyclic_handler(void *exinf)
{
	(void)exinf;
	tk_wup_tsk(tid_motion);
}

/* ---------------------------------------------------------------------
 *  IF-M2 reader, called from OTHER tasks
 * --------------------------------------------------------------------- */
EXPORT ER s2_get_status(MOTION_STATUS *out)
{
	ER er = tk_loc_mtx(mtx_motion, TMO_FEVR);

	if (er < E_OK) {
		return er;
	}
	*out = status_shared;
	return tk_unl_mtx(mtx_motion);
}

/* ---------------------------------------------------------------------
 *  Helpers
 * --------------------------------------------------------------------- */
LOCAL INT clampi(INT v, INT lim)
{
	if (v > lim)  return lim;
	if (v < -lim) return -lim;
	return v;
}

LOCAL INT absi(INT v)
{
	return (v < 0) ? -v : v;
}

LOCAL float absf(float v)
{
	return (v < 0.0f) ? -v : v;
}

LOCAL UW now_ms(void)
{
	SYSTIM t;

	tk_get_tim(&t);
	return t.lo;
}

LOCAL void reset_control(void)
{
	mc.want_l = mc.want_r = 0;
	mc.tgt_l  = mc.tgt_r  = 0;
	mc.stall_ms_l = mc.stall_ms_r = 0;
	pid_reset(&mc.pid_l);
	pid_reset(&mc.pid_r);
}

LOCAL void enter_idle(UINT event_bits)
{
	mc.state = MSTATE_IDLE;
	reset_control();
	drv_motor_brake(MOTOR_LEFT);
	drv_motor_brake(MOTOR_RIGHT);
	mc.duty_l = mc.duty_r = 0;
	if (event_bits != 0) {
		tk_set_flg(flg_motion, event_bits);
	}
}

/* NFR8 safe state: motors off, latched until MOTION_RESET */
LOCAL void enter_latched(INT state, INT fault)
{
	drv_motor_all_off();
	mc.duty_l = mc.duty_r = 0;
	reset_control();
	if (mc.state != state) {
		mc.state = state;
		mc.fault = fault;
		tk_set_flg(flg_motion, (state == MSTATE_FAULT) ?
			   MFLG_FAULT : MFLG_SAFE_STOPPED);
		tm_printf((UB*)"motion: %s (fault %d)\n",
			  (state == MSTATE_FAULT) ? "FAULT" : "SAFE STOP",
			  fault);
	}
}

/* ---------------------------------------------------------------------
 *  RECEIVE_COMMAND
 * --------------------------------------------------------------------- */
LOCAL void begin_segment(void)
{
	mc.seg_l = mc.odo_l;
	mc.seg_r = mc.odo_r;
	pid_reset(&mc.pid_l);
	pid_reset(&mc.pid_r);
}

LOCAL void apply_command(const MOTION_CMD *c)
{
	BOOL latched = (mc.state == MSTATE_SAFE_STOP ||
			mc.state == MSTATE_FAULT);

	/* While latched, only SAFE_STOP and RESET are accepted */
	if (latched && c->type != MOTION_RESET &&
	    c->type != MOTION_SAFE_STOP) {
		return;
	}

	switch (c->type) {
	case MOTION_STOP:
		mc.cmd   = *c;
		mc.state = MSTATE_STOPPING;
		break;

	case MOTION_DRIVE:
		mc.cmd = *c;
		mc.cmd.speed_mm_s = clampi(c->speed_mm_s, S2_MAX_SPEED_MM_S);
		mc.cmd.steer_mm_s = clampi(c->steer_mm_s, S2_MAX_SPEED_MM_S);
		mc.cmd_time_ms = now_ms();
		mc.state = MSTATE_DRIVING;
		break;

	case MOTION_DRIVE_DIST:
		mc.cmd = *c;
		mc.cmd.speed_mm_s = absi(clampi(c->speed_mm_s,
						S2_MAX_SPEED_MM_S));
		if (c->distance_mm == 0 || mc.cmd.speed_mm_s == 0) {
			enter_idle(MFLG_CMD_DONE);
			break;
		}
		begin_segment();
		mc.state = MSTATE_DRIVING_DIST;
		break;

	case MOTION_TURN:
		mc.cmd = *c;
		mc.cmd.speed_mm_s = absi(clampi(c->speed_mm_s,
						S2_MAX_SPEED_MM_S));
		if (mc.cmd.speed_mm_s == 0) {
			mc.cmd.speed_mm_s = S2_TURN_SPEED_MM_S;
		}
		if (c->angle_deg == 0) {
			enter_idle(MFLG_CMD_DONE);
			break;
		}
		begin_segment();
		mc.state = MSTATE_TURNING;
		break;

	case MOTION_SAFE_STOP:
		mc.cmd = *c;
		enter_latched(MSTATE_SAFE_STOP, MFAULT_NONE);
		break;

	case MOTION_RESET:
		mc.cmd   = *c;
		mc.fault = MFAULT_NONE;
		enter_idle(0);
		break;

	default:
		tm_printf((UB*)"motion: unknown command %d ignored\n",
			  c->type);
		break;
	}
}

LOCAL void receive_commands(void)
{
	MOTION_CMD	c;
	UINT		ptn;

	/* Emergency stop first: it does not wait behind queued commands */
	if (tk_wai_flg(flg_motion, MFLG_ESTOP_REQ, TWF_ORW | TWF_BITCLR,
		       &ptn, TMO_POL) == E_OK) {
		enter_latched(MSTATE_SAFE_STOP, MFAULT_NONE);
	}

	/* Then every queued command, oldest first: the newest wins */
	while (tk_rcv_mbf(mbf_motion_cmd, &c, TMO_POL) ==
	       (INT)sizeof(MOTION_CMD)) {
		apply_command(&c);
	}
}

/* ---------------------------------------------------------------------
 *  UPDATE_SPEED
 * --------------------------------------------------------------------- */
LOCAL float signed_mm(UW ticks, INT duty, float speed)
{
	float mm = motion_ticks_to_mm(ticks);

	/* Single-channel encoder: direction = the way we are driving it,
	   or, while coasting, the way it was already moving. */
	if (duty < 0 || (duty == 0 && speed < 0.0f)) {
		mm = -mm;
	}
	return mm;
}

LOCAL void update_speed(float dt_s)
{
	float	dl, dr;

	mc.last_dt_l = drv_encoder_delta(MOTOR_LEFT);
	mc.last_dt_r = drv_encoder_delta(MOTOR_RIGHT);
	mc.ticks_l  += mc.last_dt_l;
	mc.ticks_r  += mc.last_dt_r;

	dl = signed_mm(mc.last_dt_l, mc.duty_l, mc.spd_l);
	dr = signed_mm(mc.last_dt_r, mc.duty_r, mc.spd_r);
	mc.odo_l += dl;
	mc.odo_r += dr;

	/* Differential-drive heading: + (left) when the right wheel goes
	   further */
	mc.heading += (dr - dl) / S2_TRACK_MM;
	if (mc.heading >  S2_PI) mc.heading -= 2.0f * S2_PI;
	if (mc.heading < -S2_PI) mc.heading += 2.0f * S2_PI;

	/* Moving window: add this period, drop the oldest */
	mc.sum_l += dl - mc.win_l[mc.win_i];
	mc.sum_r += dr - mc.win_r[mc.win_i];
	mc.win_l[mc.win_i] = dl;
	mc.win_r[mc.win_i] = dr;
	mc.win_i = (mc.win_i + 1) % S2_SPEED_WINDOW;
	mc.spd_l = mc.sum_l / (dt_s * S2_SPEED_WINDOW);
	mc.spd_r = mc.sum_r / (dt_s * S2_SPEED_WINDOW);
}

/* ---------------------------------------------------------------------
 *  Mode logic: current command -> wanted wheel speeds
 * --------------------------------------------------------------------- */
LOCAL INT slow_near_end(INT v, float remaining_mm, float slow_mm)
{
	INT vmin = 60;	/* MEASURE: slowest speed that still moves */

	if (remaining_mm < slow_mm) {
		INT s = (INT)((float)v * remaining_mm / slow_mm);
		v = (s > vmin) ? s : vmin;
	}
	return v;
}

LOCAL void mode_logic(void)
{
	float	dl = mc.odo_l - mc.seg_l;
	float	dr = mc.odo_r - mc.seg_r;
	float	remaining, steer;
	INT	dir, v;

	switch (mc.state) {
	case MSTATE_DRIVING:
		if (S2_CMD_TIMEOUT_MS > 0 &&
		    (UW)(now_ms() - mc.cmd_time_ms) > S2_CMD_TIMEOUT_MS) {
			tk_set_flg(flg_motion, MFLG_CMD_TIMEOUT);
			mc.state = MSTATE_STOPPING;
			mc.want_l = mc.want_r = 0;
			break;
		}
		mc.want_l = mc.cmd.speed_mm_s - mc.cmd.steer_mm_s;
		mc.want_r = mc.cmd.speed_mm_s + mc.cmd.steer_mm_s;
		break;

	case MSTATE_DRIVING_DIST:
		dir = (mc.cmd.distance_mm > 0) ? 1 : -1;
		remaining = (float)absi(mc.cmd.distance_mm)
			  - (float)dir * (dl + dr) / 2.0f;
		if (remaining <= 0.0f) {
			enter_idle(MFLG_CMD_DONE);
			break;
		}
		v = slow_near_end(mc.cmd.speed_mm_s, remaining, 50.0f);
		/* Straight-line correction: slow the wheel that is ahead */
		steer = S2_STRAIGHT_K * (dl - dr);	/* > 0: left ahead */
		mc.want_l = dir * v - (INT)steer;
		mc.want_r = dir * v + (INT)steer;
		break;

	case MSTATE_TURNING:
		dir = (mc.cmd.angle_deg > 0) ? 1 : -1;	/* + = left */
		remaining = motion_turn_arc_mm(mc.cmd.angle_deg)
			  - (absf(dl) + absf(dr)) / 2.0f;
		if (remaining <= 0.0f) {
			enter_idle(MFLG_CMD_DONE);
			break;
		}
		v = slow_near_end(mc.cmd.speed_mm_s, remaining,
				  S2_TURN_SLOW_MM);
		mc.want_l = -dir * v;	/* left turn: left wheel back */
		mc.want_r =  dir * v;
		break;

	case MSTATE_STOPPING:
		mc.want_l = mc.want_r = 0;
		if (mc.tgt_l == 0 && mc.tgt_r == 0 &&
		    absf(mc.spd_l) < S2_STOP_SPEED_MM_S &&
		    absf(mc.spd_r) < S2_STOP_SPEED_MM_S) {
			enter_idle(MFLG_STOPPED);
		}
		break;

	default:	/* IDLE, SAFE_STOP, FAULT: nothing to drive */
		mc.want_l = mc.want_r = 0;
		break;
	}
}

/* ---------------------------------------------------------------------
 *  CALCULATE_PWM
 * --------------------------------------------------------------------- */
LOCAL INT wheel_duty(PID_CTRL *pid, INT target, float measured,
		     float dt_s)
{
	INT duty;

	if (target == 0) {
		pid_reset(pid);
		return 0;
	}
	duty = (INT)pid_update(pid, (float)target, measured, dt_s);

	/* Below S2_DUTY_MIN_MOVE the motor only hums: lift to the minimum
	   in the commanded direction (static friction compensation). */
	if (target > 0 && duty < S2_DUTY_MIN_MOVE)  duty = S2_DUTY_MIN_MOVE;
	if (target < 0 && duty > -S2_DUTY_MIN_MOVE) duty = -S2_DUTY_MIN_MOVE;
	return duty;
}

LOCAL void calculate_pwm(float dt_s)
{
	INT step = (S2_ACCEL_MM_S2 * S2_PERIOD_MS) / 1000;

	mc.tgt_l = motion_ramp(mc.tgt_l, mc.want_l, step);
	mc.tgt_r = motion_ramp(mc.tgt_r, mc.want_r, step);

	mc.duty_l = wheel_duty(&mc.pid_l, mc.tgt_l, mc.spd_l, dt_s);
	mc.duty_r = wheel_duty(&mc.pid_r, mc.tgt_r, mc.spd_r, dt_s);
}

/* ---------------------------------------------------------------------
 *  DRIVE_MOTOR (+ stall check)
 * --------------------------------------------------------------------- */
LOCAL BOOL stalled(UW *stall_ms, INT duty, UW ticks)
{
	if (absi(duty) >= S2_STALL_DUTY && ticks == 0) {
		*stall_ms += S2_PERIOD_MS;
	} else {
		*stall_ms = 0;
	}
	return (*stall_ms >= S2_STALL_MS);
}

LOCAL void drive_motor(void)
{
	if (stalled(&mc.stall_ms_l, mc.duty_l, mc.last_dt_l)) {
		enter_latched(MSTATE_FAULT, MFAULT_STALL_LEFT);
		return;
	}
	if (stalled(&mc.stall_ms_r, mc.duty_r, mc.last_dt_r)) {
		enter_latched(MSTATE_FAULT, MFAULT_STALL_RIGHT);
		return;
	}
	drv_motor_set(MOTOR_LEFT,  mc.duty_l);
	drv_motor_set(MOTOR_RIGHT, mc.duty_r);
}

/* ---------------------------------------------------------------------
 *  Publish IF-M2
 * --------------------------------------------------------------------- */
LOCAL void publish_status(void)
{
	MOTION_STATUS s;

	s.state         = mc.state;
	s.fault         = mc.fault;
	s.cmd_seq       = mc.cmd.seq;
	s.target_l_mm_s = mc.tgt_l;
	s.target_r_mm_s = mc.tgt_r;
	s.speed_l_mm_s  = (INT)mc.spd_l;
	s.speed_r_mm_s  = (INT)mc.spd_r;
	s.speed_mm_s    = (INT)((mc.spd_l + mc.spd_r) / 2.0f);
	s.odo_l_mm      = (W)mc.odo_l;
	s.odo_r_mm      = (W)mc.odo_r;
	s.distance_mm   = (W)((mc.odo_l + mc.odo_r) / 2.0f);
	s.heading_ddeg  = (INT)(mc.heading * 1800.0f / S2_PI);
	s.duty_l        = mc.duty_l;
	s.duty_r        = mc.duty_r;
	s.ticks_l       = mc.ticks_l;
	s.ticks_r       = mc.ticks_r;
	s.loop_count    = mc.loops;
	s.overruns      = mc.overruns;
	s.time_ms       = now_ms();

	/* Never let a slow reader stall the control loop: give up after
	   1 ms and publish next period instead. */
	if (tk_loc_mtx(mtx_motion, 1) == E_OK) {
		status_shared = s;
		tk_unl_mtx(mtx_motion);
	}

	/* IF-S4-3: a plain word store inside S4, safe from any task */
	if (s4_set_speed_mm_s != NULL) {
		s4_set_speed_mm_s((W)s.speed_mm_s);
	}

#if S2_PRINT_EVERY > 0
	if (mc.loops % S2_PRINT_EVERY == 0) {
		tm_printf((UB*)"motion: st=%d v=%d/%d mm/s tgt=%d/%d "
			  "duty=%d/%d dist=%d mm hdg=%d (0.1 deg) ovr=%u\n",
			  s.state, s.speed_l_mm_s, s.speed_r_mm_s,
			  s.target_l_mm_s, s.target_r_mm_s,
			  s.duty_l, s.duty_r, (INT)s.distance_mm,
			  s.heading_ddeg,
			  s.overruns);
	}
#endif
}

/* ---------------------------------------------------------------------
 *  The task
 * --------------------------------------------------------------------- */
EXPORT void motion_task(INT stacd, void *exinf)
{
	const float	dt_s = (float)S2_PERIOD_MS / 1000.0f;
	INT		missed;

	(void)stacd; (void)exinf;

	pid_init(&mc.pid_l, S2_PID_KFF, S2_PID_KP, S2_PID_KI, S2_PID_KD,
		 S2_PID_I_LIMIT, (float)S2_DUTY_MAX);
	pid_init(&mc.pid_r, S2_PID_KFF, S2_PID_KP, S2_PID_KI, S2_PID_KD,
		 S2_PID_I_LIMIT, (float)S2_DUTY_MAX);

	if (drv_motor_init() != E_OK || drv_encoder_init() != E_OK) {
		tm_printf((UB*)"motion: driver init failed\n");
		mc.state = MSTATE_IDLE;		/* so enter_latched reports */
		enter_latched(MSTATE_FAULT, MFAULT_DRIVER);
	} else {
		enter_idle(0);
		tm_printf((UB*)"motion: ready, %d ms loop on core %d\n",
			  S2_PERIOD_MS,
#if TK_SUPPORT_SMP
			  tk_get_prc() - 1
#else
			  0
#endif
			  );
	}

	while (1) {
		/* Sleep until motion_cyclic_handler wakes us */
		tk_slp_tsk(TMO_FEVR);

		/* Wake-ups that piled up mean we missed periods */
		missed = tk_can_wup(TSK_SELF);
		if (missed > 0) {
			mc.overruns += (UW)missed;
		}
		mc.loops++;

		receive_commands();			/* RECEIVE_COMMAND */
		update_speed(dt_s);			/* UPDATE_SPEED */

		if (mc.state == MSTATE_SAFE_STOP ||
		    mc.state == MSTATE_FAULT) {
			drv_motor_all_off();		/* stay safe */
		} else {
			mode_logic();
			if (mc.state != MSTATE_IDLE) {
				calculate_pwm(dt_s);	/* CALCULATE_PWM */
				drive_motor();		/* DRIVE_MOTOR */
			}
		}
		publish_status();
	}
}
