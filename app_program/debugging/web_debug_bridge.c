/*
 *      web_debug_bridge.c
 *      Turns a browser button into the SAME s2_cmd_xxx() calls the Vehicle
 *      Controller uses, so what you test here is the real motion path:
 *      DRIVE (refreshed by the page; the S2_CMD_TIMEOUT_MS fail-safe stops
 *      the robot if the browser or WiFi goes away) and TURN.
 *      Everything is non-blocking (TMO_POL).
 */

#include "../subsystem2/s2_config.h"

#if S2_WEBDEBUG

#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"
#include "../subsystem5/drv_servo.h"
#include "../vehicle_controller/vehicle_controller.h"
#include "live_tune.h"
#include "web_debug_bridge.h"

#define WD_SPEED_DEFAULT 200 /* forward/back test speed (limit is 400) */
#define WD_SPEED_STEP    25
#define WD_SPEED_MIN     60
#define WD_SPIN_MM_S  120 /* wheel speed while spinning on the spot */
#define WD_TURN_DEG   90
#define WD_TRIM_STEP  2 /* permille per button press */

static volatile int32_t g_wd_speed = WD_SPEED_DEFAULT;

/* Dashboard mode: 0 = manual (buttons drive the wheels), 1 = line follow
   (the real mission runs, buttons are locked), 2 = monitor only (no motion). */
static volatile int32_t g_wd_mode = 0;

/* Autonomous runs (Line follow) stay locked until the person presses
   'Unlock autonomous' on the Mission tab, i.e. after the manual values are
   confirmed good.  Reset to locked on every reboot. */
static volatile int32_t g_wd_armed = 0;

static bool is_motion_cmd(char cmd)
{
    return ('f' == cmd) || ('b' == cmd) || ('l' == cmd) || ('r' == cmd) ||
           ('L' == cmd) || ('R' == cmd) || ('Q' == cmd) || ('W' == cmd) ||
           ('E' == cmd) || ('y' == cmd) || ('Y' == cmd) || ('j' == cmd) ||
           ('J' == cmd);
}

/* End a running mission the same way the MQTT "stop" + "reset" do:
   the vehicle controller goes to FAULT (motors off), then back to IDLE. */
static void mission_stop(void)
{
    (void)vc_post_event(VC_SRC_COMMS, VC_EV_CMD_STOP, 0, 0, TMO_POL);
    (void)vc_post_event(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0, TMO_POL);
}

/* Live-tuning values, defined in subsystem2/task_motion.c */
extern volatile int32_t g_s2_dbg_trim_l;
extern volatile int32_t g_s2_dbg_trim_r;
extern volatile int32_t g_s2_dbg_straight;

static int32_t clamp_to(int32_t v, int32_t lo, int32_t hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static int32_t clamp_trim(int32_t v)
{
    return (v < 700) ? 700 : ((v > 1100) ? 1100 : v);
}

/* ---------------------------------------------------------------------
 *  Who is controlling what.  Several people can open the dashboard at once
 *  (everyone on the same hotspot).  Each browser tab has a random id.  The
 *  controls are split in areas; the first browser to use an area owns it
 *  until it has been quiet for WD_LOCK_MS, and others get "LOCKED".  Reading
 *  is never locked, and STOP ('s') always works for everybody.
 * --------------------------------------------------------------------- */
#define WD_LOCK_MS 5000U
#define WD_AREAS   5 /* m = motor, l = line, i = imu, o = obstacle/servo, v = mission */

static uint32_t g_lock_id[WD_AREAS];
static uint32_t g_lock_ms[WD_AREAS];

static uint32_t now_ms(void)
{
    SYSTIM tim;

    (void)tk_get_tim(&tim);
    return (uint32_t)tim.lo;
}

static int32_t area_of(char c)
{
    switch (c)
    {
        case 'f': case 'b': case 'l': case 'r': case 'L': case 'R': case 'x':
        case 't': case 'T': case 'u': case 'U': case 'h': case 'v': case 'V':
        case 'p': case 'P': case 'i': case 'I':
            return 0; /* motor */
        case 'k': case 'K': case 'd': case 'D': case 'y': case 'Y': case 'z':
        case 'Z':
            return 1; /* line */
        case 'j': case 'J':
            return 2; /* imu */
        case 'Q': case 'W': case 'E': case 'e':
            return 3; /* obstacle + servo */
        case 'M': case 'N': case 'F': case 'A': case 'c': case 'C':
            return 4; /* mission */
        default:
            return -1; /* 's', '?' : no lock */
    }
}

/* 0 = free, 1 = mine, 2 = held by someone else */
static int32_t lock_state(int32_t area, uint32_t client)
{
    if ((0U == g_lock_id[area]) || ((now_ms() - g_lock_ms[area]) > WD_LOCK_MS))
    {
        return 0;
    }
    return (g_lock_id[area] == client) ? 1 : 2;
}

static bool lock_try(int32_t area, uint32_t client)
{
    if (2 == lock_state(area, client))
    {
        return false;
    }
    g_lock_id[area] = client;
    g_lock_ms[area] = now_ms();
    return true;
}

/* Append text, always leaving room for the final NUL */
static int32_t put_str(char * p_out, int32_t pos, int32_t max, const char * p_s)
{
    while (('\0' != *p_s) && (pos < (max - 1)))
    {
        p_out[pos++] = *p_s++;
    }
    p_out[pos] = '\0';
    return pos;
}

static int32_t put_int(char * p_out, int32_t pos, int32_t max, int32_t value)
{
    char    tmp[12];
    int32_t n = 0;

    if (value < 0)
    {
        pos   = put_str(p_out, pos, max, "-");
        value = -value;
    }
    do
    {
        tmp[n++] = (char)('0' + (value % 10));
        value /= 10;
    } while ((value > 0) && (n < 11));
    tmp[n] = '\0';
    for (int32_t i = 0; i < (n / 2); i++)
    {
        char c         = tmp[i];
        tmp[i]         = tmp[n - 1 - i];
        tmp[n - 1 - i] = c;
    }
    return put_str(p_out, pos, max, tmp);
}

/* Servo test (turns the ultrasonic sensor).  The obstacle subsystem owns the
   servo and sweeps it while monitoring, so the first press only switches
   monitoring off; press again once it has parked. */
static ER servo_test(int32_t deg)
{
    s5_status_t o;
    s5_cmd_t    cmd = { 0 };

    if ((E_OK == s5_get_status(&o)) && (S5_ST_IDLE != o.state))
    {
        cmd.type = S5_CMD_DISABLE;
        (void)tk_snd_mbf(g_mbf_s5_cmd, &cmd, sizeof(cmd), TMO_POL);
        return E_BUSY; /* "monitoring stopping, press again" */
    }
    servo_set_deg(deg);
    return E_OK;
}

static ER monitoring_on(void)
{
    s5_cmd_t cmd = { 0 };

    cmd.type = S5_CMD_ENABLE;
    return tk_snd_mbf(g_mbf_s5_cmd, &cmd, sizeof(cmd), TMO_POL);
}

int32_t web_debug_motion(char cmd, uint32_t client, char * p_out,
                         int32_t out_max)
{
    motion_status_t st;
    ER              er;
    int32_t         pos    = 0;
    int32_t         area   = area_of(cmd);
    bool            b_lock = false;

    if (0U == client)
    {
        client = 1U; /* old page without an id: treated as one shared user */
    }
    if ((0 != g_wd_mode) && is_motion_cmd(cmd))
    {
        cmd = '!'; /* locked: only manual mode may drive */
    }
    else if ((area >= 0) && !lock_try(area, client))
    {
        cmd    = '!';
        b_lock = true; /* someone else is using this area */
    }

    switch (cmd)
    {
        case '!': er = E_OBJ; break;
        /* mode switch */
        case 'M':
        case 'N':
            mission_stop();
            g_wd_mode = ('M' == cmd) ? 0 : 2;
            er        = s2_cmd_stop();
            break;
        case 'A':
            g_wd_armed = g_wd_armed ? 0 : 1; /* toggle the autonomous unlock */
            if (!g_wd_armed && (1 == g_wd_mode))
            {
                mission_stop();
                g_wd_mode = 0;
            }
            er = E_OK;
            break;
        case 'F':
            if (!g_wd_armed)
            {
                er = E_OBJ; /* still locked: tune manually first */
                break;
            }
            (void)s2_cmd_reset();
            (void)vc_post_event(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0, TMO_POL);
            er        = vc_post_event(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0, TMO_POL);
            g_wd_mode = (E_OK == er) ? 1 : g_wd_mode;
            break;
        case 'Q': er = servo_test(45); break;   /* + = left */
        case 'W': er = servo_test(0); break;
        case 'E': er = servo_test(-45); break;
        case 'e': er = monitoring_on(); break;
        case 'y': er = s3_request(S3_REQ_CALIBRATE); break;
        case 'Y': er = s3_request(S3_REQ_RESET_LINE); break;
        case 'z': er = s3_request(S3_REQ_BC_ENABLE); break;
        case 'Z': er = s3_request(S3_REQ_BC_DISABLE); break;
        case 'j': er = s4_request_calibration(); break;
        case 'J': er = s4_request_peak_reset(); break;
        case 'f': er = s2_cmd_drive(g_wd_speed, 0); break;
        case 'b': er = s2_cmd_drive(-g_wd_speed, 0); break;
        case 'l': er = s2_cmd_drive(0, WD_SPIN_MM_S); break;  /* + = left */
        case 'r': er = s2_cmd_drive(0, -WD_SPIN_MM_S); break;
        case 's':
            if (1 == g_wd_mode)
            {
                mission_stop(); /* STOP also ends a running mission */
                g_wd_mode = 0;
            }
            er = s2_cmd_stop();
            break;
        case 'x':
            (void)vc_post_event(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0, TMO_POL);
            er = s2_cmd_reset();
            break;
        case 'L': er = s2_cmd_turn(S2_TURN_SPEED_MM_S, WD_TURN_DEG); break;
        case 'R': er = s2_cmd_turn(S2_TURN_SPEED_MM_S, -WD_TURN_DEG); break;
        /* live tuning: no motion command, just change the trims */
        case 't': g_s2_dbg_trim_l = clamp_trim(g_s2_dbg_trim_l - WD_TRIM_STEP); er = E_OK; break;
        case 'T': g_s2_dbg_trim_l = clamp_trim(g_s2_dbg_trim_l + WD_TRIM_STEP); er = E_OK; break;
        case 'u': g_s2_dbg_trim_r = clamp_trim(g_s2_dbg_trim_r - WD_TRIM_STEP); er = E_OK; break;
        case 'U': g_s2_dbg_trim_r = clamp_trim(g_s2_dbg_trim_r + WD_TRIM_STEP); er = E_OK; break;
        case 'h': g_s2_dbg_straight = (0 != g_s2_dbg_straight) ? 0 : 1; er = E_OK; break;
        case 'v': g_wd_speed = (g_wd_speed - WD_SPEED_STEP < WD_SPEED_MIN) ? WD_SPEED_MIN : (g_wd_speed - WD_SPEED_STEP); er = E_OK; break;
        case 'V': g_wd_speed = (g_wd_speed + WD_SPEED_STEP > S2_MAX_SPEED_MM_S) ? S2_MAX_SPEED_MM_S : (g_wd_speed + WD_SPEED_STEP); er = E_OK; break;
        /* live tuning of the controllers (see live_tune.h) */
        case 'p': g_lt_pid_kp100 = clamp_to(g_lt_pid_kp100 - 10, 0, 1000); er = E_OK; break;
        case 'P': g_lt_pid_kp100 = clamp_to(g_lt_pid_kp100 + 10, 0, 1000); er = E_OK; break;
        case 'i': g_lt_pid_ki100 = clamp_to(g_lt_pid_ki100 - 50, 0, 3000); er = E_OK; break;
        case 'I': g_lt_pid_ki100 = clamp_to(g_lt_pid_ki100 + 50, 0, 3000); er = E_OK; break;
        case 'k': g_lt_line_kp = clamp_to(g_lt_line_kp - 1, 0, 60); er = E_OK; break;
        case 'K': g_lt_line_kp = clamp_to(g_lt_line_kp + 1, 0, 60); er = E_OK; break;
        case 'd': g_lt_line_kd = clamp_to(g_lt_line_kd - 2, 0, 100); er = E_OK; break;
        case 'D': g_lt_line_kd = clamp_to(g_lt_line_kd + 2, 0, 100); er = E_OK; break;
        case 'c': g_lt_speed_pct = clamp_to(g_lt_speed_pct - 10, 20, 150); er = E_OK; break;
        case 'C': g_lt_speed_pct = clamp_to(g_lt_speed_pct + 10, 20, 150); er = E_OK; break;
        case '?': er = E_OK; break; /* status only */
        default: er = E_PAR; break;
    }

    p_out[0] = '\0';
    pos = put_str(p_out, pos, out_max,
                  (E_OK == er) ? "ok" : (b_lock ? "LOCKED (another user is using this)" : "ERR"));
    pos = put_str(p_out, pos, out_max, " mode=");
    pos = put_int(p_out, pos, out_max, g_wd_mode);
    pos = put_str(p_out, pos, out_max, " speed=");
    pos = put_int(p_out, pos, out_max, g_wd_speed);
    pos = put_str(p_out, pos, out_max, " trimL=");
    pos = put_int(p_out, pos, out_max, g_s2_dbg_trim_l);
    pos = put_str(p_out, pos, out_max, " trimR=");
    pos = put_int(p_out, pos, out_max, g_s2_dbg_trim_r);
    pos = put_str(p_out, pos, out_max,
                  (0 != g_s2_dbg_straight) ? " straight=ON" : " straight=OFF");
    if (E_OK == s2_get_status(&st))
    {
        pos = put_str(p_out, pos, out_max, " state=");
        pos = put_int(p_out, pos, out_max, st.state);
        pos = put_str(p_out, pos, out_max, " fault=");
        pos = put_int(p_out, pos, out_max, st.fault);
        pos = put_str(p_out, pos, out_max, " L(mm/s)=");
        pos = put_int(p_out, pos, out_max, st.speed_l_mm_s);
        pos = put_str(p_out, pos, out_max, " R(mm/s)=");
        pos = put_int(p_out, pos, out_max, st.speed_r_mm_s);
        pos = put_str(p_out, pos, out_max, " ticksL=");
        pos = put_int(p_out, pos, out_max, (int32_t)st.ticks_l);
        pos = put_str(p_out, pos, out_max, " ticksR=");
        pos = put_int(p_out, pos, out_max, (int32_t)st.ticks_r);
        pos = put_str(p_out, pos, out_max, " heading(0.1deg)=");
        (void)put_int(p_out, pos, out_max, st.heading_ddeg);
    }
    return (E_OK == er) ? 0 : -1;
}

/* ---------------------------------------------------------------------
 *  Whole-robot snapshot for the dashboard: one JSON object, read from the
 *  other subsystems' public status functions only (never their globals).
 *  A subsystem that is not running (call fails) is sent as null.
 *  Each object ends with a dummy "_":0 so every real field can end in ','.
 * --------------------------------------------------------------------- */
static int32_t kv(char * p_out, int32_t pos, int32_t max, const char * p_key,
                  int32_t value)
{
    pos = put_str(p_out, pos, max, "\"");
    pos = put_str(p_out, pos, max, p_key);
    pos = put_str(p_out, pos, max, "\":");
    pos = put_int(p_out, pos, max, value);
    return put_str(p_out, pos, max, ",");
}

static bool has(const char * p_x, char c)
{
    while ('\0' != *p_x)
    {
        if (*p_x++ == c)
        {
            return true;
        }
    }
    return false;
}

int32_t web_debug_snapshot(char * p_out, int32_t out_max, uint32_t client,
                           const char * p_x)
{
    motion_status_t    m;
    s3_line_status_t   l;
    imu_status_t       imu;
    s5_status_t        o;
    vc_status_t        vc;
    int32_t            n = 0;

    p_out[0] = '\0';
    n = put_str(p_out, n, out_max, "{\"cfg\":{");
    n = kv(p_out, n, out_max, "speed", g_wd_speed);
    n = kv(p_out, n, out_max, "trimL", g_s2_dbg_trim_l);
    n = kv(p_out, n, out_max, "trimR", g_s2_dbg_trim_r);
    n = kv(p_out, n, out_max, "straight", g_s2_dbg_straight);
    if (0U == client)
    {
        client = 1U;
    }
    n = kv(p_out, n, out_max, "mode", g_wd_mode);
    n = kv(p_out, n, out_max, "arm", g_wd_armed);
    n = kv(p_out, n, out_max, "lkm", lock_state(0, client));
    n = kv(p_out, n, out_max, "lkl", lock_state(1, client));
    n = kv(p_out, n, out_max, "lki", lock_state(2, client));
    n = kv(p_out, n, out_max, "lko", lock_state(3, client));
    n = kv(p_out, n, out_max, "lkv", lock_state(4, client));
    n = kv(p_out, n, out_max, "pidKp100", g_lt_pid_kp100);
    n = kv(p_out, n, out_max, "pidKi100", g_lt_pid_ki100);
    n = kv(p_out, n, out_max, "lineKp", g_lt_line_kp);
    n = kv(p_out, n, out_max, "lineKd", g_lt_line_kd);
    n = kv(p_out, n, out_max, "pct", g_lt_speed_pct);
    n = put_str(p_out, n, out_max, "\"_\":0},\"s2\":");

    if (!has(p_x, 'm'))
    {
        n = put_str(p_out, n, out_max, "null");
    }
    else if (E_OK == s2_get_status(&m))
    {
        n = put_str(p_out, n, out_max, "{");
        n = kv(p_out, n, out_max, "state", m.state);
        n = kv(p_out, n, out_max, "fault", m.fault);
        n = kv(p_out, n, out_max, "tgtL", m.target_l_mm_s);
        n = kv(p_out, n, out_max, "tgtR", m.target_r_mm_s);
        n = kv(p_out, n, out_max, "vL", m.speed_l_mm_s);
        n = kv(p_out, n, out_max, "vR", m.speed_r_mm_s);
        n = kv(p_out, n, out_max, "dutyL", m.duty_l);
        n = kv(p_out, n, out_max, "dutyR", m.duty_r);
        n = kv(p_out, n, out_max, "ticksL", (int32_t)m.ticks_l);
        n = kv(p_out, n, out_max, "ticksR", (int32_t)m.ticks_r);
        n = kv(p_out, n, out_max, "dist", m.distance_mm);
        n = kv(p_out, n, out_max, "head", m.heading_ddeg);
        n = kv(p_out, n, out_max, "over", (int32_t)m.overruns);
        n = put_str(p_out, n, out_max, "\"_\":0}");
    }
    else
    {
        n = put_str(p_out, n, out_max, "null");
    }

    n = put_str(p_out, n, out_max, ",\"s3\":");
    if (!has(p_x, 'l'))
    {
        n = put_str(p_out, n, out_max, "null");
    }
    else if (E_OK == s3_get_line_status(&l))
    {
        n = put_str(p_out, n, out_max, "{");
        n = kv(p_out, n, out_max, "state", (int32_t)l.line_state);
        n = kv(p_out, n, out_max, "pos", l.position);
        n = kv(p_out, n, out_max, "err", l.error);
        n = kv(p_out, n, out_max, "steer", l.steering);
        n = kv(p_out, n, out_max, "mask", l.black_mask);
        n = kv(p_out, n, out_max, "r0", l.raw[0]);
        n = kv(p_out, n, out_max, "r1", l.raw[1]);
        n = kv(p_out, n, out_max, "r2", l.raw[2]);
        n = kv(p_out, n, out_max, "cal", l.calibrated ? 1 : 0);
        n = kv(p_out, n, out_max, "bc", (int32_t)l.bc_state);
        n = kv(p_out, n, out_max, "bcOk", (int32_t)l.bc_ok);
        n = kv(p_out, n, out_max, "bcFail", (int32_t)l.bc_fail);
        n = put_str(p_out, n, out_max, "\"_\":0}");
    }
    else
    {
        n = put_str(p_out, n, out_max, "null");
    }

    n = put_str(p_out, n, out_max, ",\"s4\":");
    if (!has(p_x, 'i'))
    {
        n = put_str(p_out, n, out_max, "null");
    }
    else if (E_OK == s4_get_status(&imu))
    {
        n = put_str(p_out, n, out_max, "{");
        n = kv(p_out, n, out_max, "state", (int32_t)imu.state);
        n = kv(p_out, n, out_max, "motion", (int32_t)imu.motion);
        n = kv(p_out, n, out_max, "cal", (int32_t)imu.calibrated);
        n = kv(p_out, n, out_max, "fault", (int32_t)imu.fault);
        n = kv(p_out, n, out_max, "pitch", imu.pitch_cdeg);
        n = kv(p_out, n, out_max, "roll", imu.roll_cdeg);
        n = kv(p_out, n, out_max, "head", imu.heading_cdeg);
        n = kv(p_out, n, out_max, "turn", imu.turn_rate_cdeg_s);
        n = kv(p_out, n, out_max, "humps", (int32_t)imu.hump_count);
        n = kv(p_out, n, out_max, "impacts", (int32_t)imu.impact_count);
        n = kv(p_out, n, out_max, "maxPeak", imu.max_peak_um);
        n = put_str(p_out, n, out_max, "\"_\":0}");
    }
    else
    {
        n = put_str(p_out, n, out_max, "null");
    }

    n = put_str(p_out, n, out_max, ",\"s5\":");
    if (!has(p_x, 'o'))
    {
        n = put_str(p_out, n, out_max, "null");
    }
    else if (E_OK == s5_get_status(&o))
    {
        n = put_str(p_out, n, out_max, "{");
        n = kv(p_out, n, out_max, "state", (int32_t)o.state);
        n = kv(p_out, n, out_max, "fault", (int32_t)o.last_fault);
        n = kv(p_out, n, out_max, "dist", (int32_t)o.last_distance_mm);
        n = kv(p_out, n, out_max, "servo", servo_get_deg());
        n = kv(p_out, n, out_max, "pings", (int32_t)o.pings);
        n = kv(p_out, n, out_max, "timeouts", (int32_t)o.echo_timeouts);
        n = kv(p_out, n, out_max, "obst", (int32_t)o.obstacles);
        n = put_str(p_out, n, out_max, "\"_\":0}");
    }
    else
    {
        n = put_str(p_out, n, out_max, "null");
    }

    n = put_str(p_out, n, out_max, ",\"vc\":");
    if (!has(p_x, 'v'))
    {
        n = put_str(p_out, n, out_max, "null");
    }
    else if (E_OK == vc_get_status(&vc))
    {
        n = put_str(p_out, n, out_max, "{");
        n = kv(p_out, n, out_max, "state", (int32_t)vc.state);
        n = kv(p_out, n, out_max, "prev", (int32_t)vc.prev_state);
        n = kv(p_out, n, out_max, "sinceMs", (int32_t)vc.state_since_ms);
        n = kv(p_out, n, out_max, "fault", (int32_t)vc.last_fault_code);
        n = kv(p_out, n, out_max, "barcodes", (int32_t)vc.barcodes_read);
        n = kv(p_out, n, out_max, "events", (int32_t)vc.events_handled);
        n = kv(p_out, n, out_max, "wifi", vc.wifi_up ? 1 : 0);
        n = put_str(p_out, n, out_max, "\"_\":0}");
    }
    else
    {
        n = put_str(p_out, n, out_max, "null");
    }

    (void)put_str(p_out, n, out_max, "}");
    return 0;
}

#endif /* S2_WEBDEBUG */

/*** end of file ***/
