/*
 *	imu_terrain.c
 *	Subsystem 4 -- IMU processing and terrain state machine.
 *	See imu_terrain.h for the states, axes and units.
 *
 *	Hump height method
 *	------------------
 *	On a slope the accelerometer's forward axis sees gravity * sin(pitch),
 *	so sin(pitch) ~= ax / |a|.  Each sample the robot moves
 *	ds = speed * period, and climbs dh = ds * sin(pitch).  Summing dh
 *	from the foot of the hump gives its height; the largest value is the
 *	peak.  This avoids double-integrating vertical acceleration, which
 *	drifts badly (the "weak AI idea" rejected in design review 8.1).
 *	Accuracy depends on the speed from motion control (subsystem 2) and
 *	on the robot not accelerating hard on the slope; validate per A5.
 */

#include "imu_terrain.h"
#include "imu_math.h"
#include "subsystem4_cfg.h"

#define ONE_G_MG	1000

/* Exponential moving average kept at 2^shift scale */
static int32_t ema_step(int32_t *acc, int32_t x, int32_t shift)
{
	int32_t scale = (int32_t)1 << shift;

	*acc += x - (*acc / scale);
	return *acc / scale;
}

static int32_t iabs32(int32_t v)
{
	return (v < 0) ? -v : v;
}

void imu_terrain_default_cfg(imu_cfg_t *cfg)
{
	cfg->period_ms          = S4_PERIOD_MS;
	cfg->lpf_shift          = S4_LPF_SHIFT;
	cfg->cal_samples        = S4_CAL_SAMPLES;
	cfg->cal_max_spread_mg  = S4_CAL_MAX_SPREAD_MG;
	cfg->hump_enter_cdeg    = S4_HUMP_ENTER_CDEG;
	cfg->hump_level_cdeg    = S4_HUMP_LEVEL_CDEG;
	cfg->hump_confirm       = S4_HUMP_CONFIRM;
	cfg->hump_timeout       = S4_HUMP_TIMEOUT;
	cfg->hump_min_peak_um   = S4_HUMP_MIN_PEAK_UM;
	cfg->nominal_speed_mm_s = S4_NOMINAL_SPEED_MM_S;
	cfg->impact_mg          = S4_IMPACT_MG;
	cfg->impact_holdoff     = S4_IMPACT_HOLDOFF;
	cfg->tilt_limit_cdeg    = S4_TILT_LIMIT_CDEG;
	cfg->accel_mg           = S4_ACCEL_MG;
	cfg->still_mg           = S4_STILL_MG;
	cfg->still_samples      = S4_STILL_SAMPLES;
	cfg->turn_rate_cdeg_s   = S4_TURN_RATE_CDEG_S;
}

static void start_calibration(imu_terrain_t *t)
{
	int i;

	t->cal_n = 0;
	for (i = 0; i < 3; i++) {
		t->cal_sum[i] = 0;
		t->cal_min[i] = INT16_MAX;
		t->cal_max[i] = INT16_MIN;
	}
	t->st.state      = IMU_ST_CALIBRATING;
	t->st.calibrated = 0U;
}

void imu_terrain_init(imu_terrain_t *t, const imu_cfg_t *cfg)
{
	uint8_t		*p = (uint8_t *)t;
	uint32_t	i;

	for (i = 0U; i < sizeof(*t); i++) {	/* no memset in freestanding */
		p[i] = 0U;
	}
	t->cfg       = *cfg;
	t->st.state  = IMU_ST_INIT;
	t->st.motion = IMU_MOT_UNKNOWN;
	t->pending   = -1;
	start_calibration(t);
}

void imu_terrain_recalibrate(imu_terrain_t *t)
{
	start_calibration(t);
}

void imu_terrain_reset_peaks(imu_terrain_t *t)
{
	t->st.last_peak_um = 0;
	t->st.max_peak_um  = 0;
	t->st.hump_count   = 0U;
}

void imu_terrain_set_fault(imu_terrain_t *t, int fault)
{
	if (fault != 0) {
		t->st.state = IMU_ST_ERROR;
		t->st.fault = 1U;
	} else if (t->st.fault != 0U) {
		/* Recovered: the filters are stale, so calibrate again */
		t->st.fault = 0U;
		start_calibration(t);
	} else {
		/* not in fault: nothing to do */
	}
}

/* ---------------------------------------------------------------------
 * Calibration: average still samples to find the accelerometer bias.
 * Assumes the robot is LEVEL and STILL (on the start line).
 * ------------------------------------------------------------------- */
static uint32_t calibrate_step(imu_terrain_t *t, const imu_sample_t *s)
{
	int16_t	v[3];
	int	i;

	v[0] = s->ax_mg;
	v[1] = s->ay_mg;
	v[2] = s->az_mg;

	for (i = 0; i < 3; i++) {
		t->cal_sum[i] += v[i];
		if (v[i] < t->cal_min[i]) {
			t->cal_min[i] = v[i];
		}
		if (v[i] > t->cal_max[i]) {
			t->cal_max[i] = v[i];
		}
	}
	t->cal_n++;

	if (t->cal_n < t->cfg.cal_samples) {
		return 0U;
	}

	for (i = 0; i < 3; i++) {
		if ((t->cal_max[i] - t->cal_min[i]) > t->cfg.cal_max_spread_mg) {
			start_calibration(t);		/* moved: try again */
			return IMU_EV_CAL_FAILED;
		}
	}

	t->bias[0] = (int16_t)(t->cal_sum[0] / t->cal_n);
	t->bias[1] = (int16_t)(t->cal_sum[1] / t->cal_n);
	t->bias[2] = (int16_t)((t->cal_sum[2] / t->cal_n) - ONE_G_MG);

	/* Seed the filters with a level, still robot */
	t->f_acc[0] = 0;
	t->f_acc[1] = 0;
	t->f_acc[2] = ONE_G_MG << t->cfg.lpf_shift;
	t->f_rate   = 0;
	t->have_heading = 0U;

	t->st.calibrated     = 1U;
	t->st.state          = IMU_ST_MONITORING;
	t->st.hump_height_um = 0;
	t->pending           = -1;
	t->confirm           = 0;
	return IMU_EV_CAL_DONE;
}

/* A transition happens only after `hump_confirm` samples in a row
   asked for the same next state (debounce). */
static int confirmed(imu_terrain_t *t, imu_state_t next)
{
	if (t->pending == (int32_t)next) {
		t->confirm++;
	} else {
		t->pending = (int32_t)next;
		t->confirm = 1;
	}
	return (t->confirm >= t->cfg.hump_confirm);
}

static void enter(imu_terrain_t *t, imu_state_t next)
{
	t->st.state = next;
	t->in_state = 0;
	t->pending  = -1;
	t->confirm  = 0;
}

/* ---------------------------------------------------------------------
 * Hump state machine (Statechart 4)
 * ------------------------------------------------------------------- */
static uint32_t hump_step(imu_terrain_t *t, int32_t pitch)
{
	const imu_cfg_t	*c = &t->cfg;
	uint32_t	ev = 0U;
	int		up    = (pitch >  c->hump_enter_cdeg);
	int		down  = (pitch < -c->hump_enter_cdeg);
	int		level = (iabs32(pitch) < c->hump_level_cdeg);

	if (t->st.hump_height_um > t->peak_um) {
		t->peak_um = t->st.hump_height_um;
	}

	switch (t->st.state) {
	case IMU_ST_MONITORING:
		if (level) {
			t->st.hump_height_um = 0;	/* re-zero on flat ground */
			t->peak_um = 0;
		}
		if (up) {
			if (confirmed(t, IMU_ST_CLIMBING)) {
				enter(t, IMU_ST_CLIMBING);
				ev |= IMU_EV_HUMP_START;
			}
		} else {
			t->pending = -1;
			t->confirm = 0;
		}
		break;

	case IMU_ST_CLIMBING:
		if (level && confirmed(t, IMU_ST_AT_PEAK)) {
			enter(t, IMU_ST_AT_PEAK);
			ev |= IMU_EV_HUMP_PEAK;
		} else if (down && confirmed(t, IMU_ST_DESCENDING)) {
			enter(t, IMU_ST_DESCENDING);	/* sharp crest */
			ev |= IMU_EV_HUMP_PEAK;
		} else if (up) {
			t->pending = -1;
			t->confirm = 0;
		} else {
			/* between thresholds: keep waiting */
		}
		break;

	case IMU_ST_AT_PEAK:
		if (down && confirmed(t, IMU_ST_DESCENDING)) {
			enter(t, IMU_ST_DESCENDING);
		} else if (up && confirmed(t, IMU_ST_CLIMBING)) {
			enter(t, IMU_ST_CLIMBING);	/* plateau, then more climb */
		} else {
			/* still on top */
		}
		break;

	case IMU_ST_DESCENDING:
		if (level && confirmed(t, IMU_ST_MONITORING)) {
			t->st.last_peak_um = t->peak_um;
			if (t->peak_um >= c->hump_min_peak_um) {
				t->st.hump_count++;
				if (t->peak_um > t->st.max_peak_um) {
					t->st.max_peak_um = t->peak_um;
				}
				ev |= IMU_EV_HUMP_END;
			} else {
				ev |= IMU_EV_HUMP_ABORT;	/* too small */
			}
			enter(t, IMU_ST_MONITORING);
			t->st.hump_height_um = 0;
			t->peak_um = 0;
		} else if (up && confirmed(t, IMU_ST_CLIMBING)) {
			enter(t, IMU_ST_CLIMBING);	/* double hump */
		} else {
			/* still descending */
		}
		break;

	default:
		break;
	}

	/* Bounded time in any hump state (NFR5) */
	if ((t->st.state == IMU_ST_CLIMBING) ||
	    (t->st.state == IMU_ST_AT_PEAK) ||
	    (t->st.state == IMU_ST_DESCENDING)) {
		if (++t->in_state > c->hump_timeout) {
			enter(t, IMU_ST_MONITORING);
			t->st.hump_height_um = 0;
			t->peak_um = 0;
			ev |= IMU_EV_HUMP_ABORT;
		}
	}
	return ev;
}

/* ---------------------------------------------------------------------
 * Motion classification (only meaningful on level ground)
 * ------------------------------------------------------------------- */
static imu_motion_t classify(imu_terrain_t *t, int32_t dyn_mg,
			     int32_t speed_mm_s)
{
	const imu_cfg_t	*c = &t->cfg;

	if (iabs32(dyn_mg) < c->still_mg) {
		t->still_cnt++;
	} else {
		t->still_cnt = 0;
	}

	if (t->st.turn_rate_cdeg_s > c->turn_rate_cdeg_s) {
		return IMU_MOT_TURNING_LEFT;
	}
	if (t->st.turn_rate_cdeg_s < -c->turn_rate_cdeg_s) {
		return IMU_MOT_TURNING_RIGHT;
	}
	if (t->st.ax_mg > c->accel_mg) {
		return IMU_MOT_ACCELERATING;
	}
	if (t->st.ax_mg < -c->accel_mg) {
		return IMU_MOT_BRAKING;
	}
	/* An accelerometer cannot tell "still" from "constant speed", so
	   trust the wheel speed when motion control provides it. */
	if ((speed_mm_s == 0) ||
	    ((speed_mm_s < 0) && (t->still_cnt >= c->still_samples))) {
		return IMU_MOT_STATIONARY;
	}
	return IMU_MOT_CRUISING;
}

uint32_t imu_terrain_update(imu_terrain_t *t, const imu_sample_t *s,
			    int32_t speed_mm_s)
{
	const imu_cfg_t	*c = &t->cfg;
	uint32_t	ev = 0U;
	int32_t		ax;
	int32_t		ay;
	int32_t		az;
	int32_t		raw_norm;
	int32_t		norm;
	int32_t		heading;
	int32_t		rate;
	int32_t		ds_um;
	imu_motion_t	m;

	t->st.sample_count++;

	if (t->st.state == IMU_ST_ERROR) {
		return 0U;
	}
	if (t->st.state == IMU_ST_CALIBRATING) {
		return calibrate_step(t, s);
	}

	/* 1. Remove bias */
	ax = (int32_t)s->ax_mg - t->bias[0];
	ay = (int32_t)s->ay_mg - t->bias[1];
	az = (int32_t)s->az_mg - t->bias[2];

	/* 2. Impact check on the RAW magnitude (filtering would hide it) */
	raw_norm = (int32_t)imu_norm3(ax, ay, az);
	if (t->holdoff > 0) {
		t->holdoff--;
	} else if (iabs32(raw_norm - ONE_G_MG) > c->impact_mg) {
		t->st.impact_count++;
		t->st.last_impact_mg = raw_norm - ONE_G_MG;
		t->holdoff = c->impact_holdoff;
		ev |= IMU_EV_IMPACT;
	} else {
		/* no impact */
	}

	/* 3. Low-pass filter, then tilt */
	t->st.ax_mg = (int16_t)ema_step(&t->f_acc[0], ax, c->lpf_shift);
	t->st.ay_mg = (int16_t)ema_step(&t->f_acc[1], ay, c->lpf_shift);
	t->st.az_mg = (int16_t)ema_step(&t->f_acc[2], az, c->lpf_shift);

	t->st.pitch_cdeg = (int16_t)imu_atan2_cdeg(t->st.ax_mg,
		(int32_t)imu_isqrt32((uint32_t)(t->st.ay_mg * t->st.ay_mg) +
				     (uint32_t)(t->st.az_mg * t->st.az_mg)));
	t->st.roll_cdeg  = (int16_t)imu_atan2_cdeg(t->st.ay_mg, t->st.az_mg);

	if ((iabs32(t->st.pitch_cdeg) > c->tilt_limit_cdeg) ||
	    (iabs32(t->st.roll_cdeg)  > c->tilt_limit_cdeg)) {
		if (t->tilt_latched == 0U) {
			t->tilt_latched = 1U;
			ev |= IMU_EV_TILT_LIMIT;
		}
	} else if ((iabs32(t->st.pitch_cdeg) < c->tilt_limit_cdeg - 500) &&
		   (iabs32(t->st.roll_cdeg)  < c->tilt_limit_cdeg - 500)) {
		t->tilt_latched = 0U;		/* 5 deg hysteresis */
	} else {
		/* in the hysteresis band */
	}

	/* 4. Heading and turn rate from the magnetometer.
	      Not tilt compensated: on a hump the heading wobbles, which is
	      why turning is only classified on level ground. */
	heading = imu_atan2_cdeg(s->my, s->mx);
	if (t->have_heading != 0U) {
		rate = imu_wrap_cdeg(heading - t->prev_heading) *
		       (1000 / c->period_ms);
		t->st.turn_rate_cdeg_s = ema_step(&t->f_rate, rate,
						  c->lpf_shift);
	}
	t->prev_heading    = (int16_t)heading;
	t->have_heading    = 1U;
	t->st.heading_cdeg = (int16_t)heading;

	/* 5. Height: dh = ds * sin(pitch) = ds * ax / |a| */
	if (speed_mm_s < 0) {
		speed_mm_s = c->nominal_speed_mm_s;
	}
	ds_um = speed_mm_s * c->period_ms;	/* mm/s * ms = um */
	norm  = (int32_t)imu_norm3(t->st.ax_mg, t->st.ay_mg, t->st.az_mg);
	if (norm > 0) {
		t->st.hump_height_um += (ds_um * t->st.ax_mg) / norm;
	}

	/* 6. Hump state machine */
	ev |= hump_step(t, t->st.pitch_cdeg);

	/* 7. Motion class */
	if (t->st.state == IMU_ST_MONITORING) {
		m = classify(t, raw_norm - ONE_G_MG, speed_mm_s);
	} else {
		m = IMU_MOT_CRUISING;		/* on a hump */
	}
	if (m != t->st.motion) {
		t->st.motion = m;
		ev |= IMU_EV_MOTION_CHANGE;
	}

	return ev;
}

const char *imu_state_name(imu_state_t s)
{
	switch (s) {
	case IMU_ST_INIT:		return "INIT";
	case IMU_ST_CALIBRATING:	return "CALIBRATING";
	case IMU_ST_MONITORING:		return "MONITORING";
	case IMU_ST_CLIMBING:		return "CLIMBING";
	case IMU_ST_AT_PEAK:		return "AT_PEAK";
	case IMU_ST_DESCENDING:		return "DESCENDING";
	case IMU_ST_ERROR:		return "ERROR";
	default:			return "?";
	}
}

const char *imu_motion_name(imu_motion_t m)
{
	switch (m) {
	case IMU_MOT_STATIONARY:	return "STATIONARY";
	case IMU_MOT_CRUISING:		return "CRUISING";
	case IMU_MOT_ACCELERATING:	return "ACCELERATING";
	case IMU_MOT_BRAKING:		return "BRAKING";
	case IMU_MOT_TURNING_LEFT:	return "TURN_LEFT";
	case IMU_MOT_TURNING_RIGHT:	return "TURN_RIGHT";
	default:			return "UNKNOWN";
	}
}
