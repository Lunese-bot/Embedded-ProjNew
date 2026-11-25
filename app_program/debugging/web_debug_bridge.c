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

/* forward/back test speed: start at the lowest (WD_SPEED_MIN) */
#define WD_SPEED_DEFAULT 60
#define WD_SPEED_STEP    25
#define WD_SPEED_MIN     60
#define WD_SPIN_MM_S     120 /* wheel speed while spinning on the spot */
#define WD_TURN_DEG      90
#define WD_TRIM_STEP     2 /* permille per button press */

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

static int32_t clamp_to(int32_t value2, int32_t low, int32_t high)
{
    return (value2 < low) ? low : ((value2 > high) ? high : value2);
}

static int32_t clamp_trim(int32_t value2)
{
    return (value2 < 500) ? 500 : ((value2 > 1500) ? 1500 : value2);
}

/* ---------------------------------------------------------------------
 *  Who is controlling what.  Several people can open the dashboard at once
 *  (everyone on the same hotspot).  Each browser tab has a random id.  The
 *  controls are split in areas; the first browser to use an area owns it
 *  until it has been quiet for WD_LOCK_MS, and others get "LOCKED".  Reading
 *  is never locked, and STOP ('s') always works for everybody.
 * --------------------------------------------------------------------- */
#define WD_LOCK_MS 5000U
/* m = motor, l = line, i = imu, o = obstacle/servo, v = mission */
#define WD_AREAS   5

static uint32_t g_lock_id[WD_AREAS];
static uint32_t g_lock_ms[WD_AREAS];

static uint32_t now_ms(void)
{
    SYSTIM tim;

    (void)tk_get_tim(&tim);
    return (uint32_t)tim.lo;
}

static int32_t area_of(char chr)
{
    switch (chr)
    {
        case 'f':
        case 'b':
        case 'l':
        case 'r':
        case 'L':
        case 'R':
        case 'x':
        case 't':
        case 'T':
        case 'u':
        case 'U':
        case 'h':
        case 'v':
        case 'V':
        case 'p':
        case 'P':
        case 'i':
        case 'I':
        case '8':
        case '9':
        case '0':
        case 'S':
        case '-':
        case '.':
        case '_':
        case '(':
        case ')':
        case ',':
        case '*':
            return 0; /* motor */
        case 'k':
        case 'K':
        case 'd':
        case 'D':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case 'w':
        case 'X':
        case 'a':
        case 'B':
        case 'o':
        case 'O':
        case 'm':
        case 'n':
        case 'y':
        case 'Y':
        case 'z':
        case 'Z':
        case '~':
        case '=':
        case '[':
        case ']':
        case '{':
        case '}':
        case '`':
        case '|':
        case '@':
        case '^':
        case '$':
        case '/':
        case '+':
            return 1; /* line */
        case 'j':
        case 'J':
        case 'q':
        case 'H':
            return 2; /* imu */
        case 'Q':
        case 'W':
        case 'E':
        case 'e':
        case 'g':
        case 'G':
            return 3; /* obstacle + servo */
        case 'M':
        case 'N':
        case 'F':
        case 'A':
        case 'c':
        case 'C':
        case ':':
        case ';':
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
    int32_t count2 = 0;

    if (value < 0)
    {
        pos   = put_str(p_out, pos, max, "-");
        value = -value;
    }
    do
    {
        tmp[count2++] = (char)('0' + (value % 10));
        value /= 10;
    } while ((value > 0) && (count2 < 11));
    tmp[count2] = '\0';
    for (int32_t idx2 = 0; idx2 < (count2 / 2); idx2++)
    {
        char chr               = tmp[idx2];
        tmp[idx2]              = tmp[count2 - 1 - idx2];
        tmp[count2 - 1 - idx2] = chr;
    }
    return put_str(p_out, pos, max, tmp);
}

/* Servo test (turns the ultrasonic sensor).  The obstacle subsystem owns the
   servo and sweeps it while monitoring, so the first press only switches
   monitoring off; press again once it has parked. */
static ER servo_test(int32_t deg)
{
    s5_status_t o_val;
    s5_cmd_t    cmd = { 0 };

    if ((E_OK == s5_get_status(&o_val)) && (S5_ST_IDLE != o_val.state))
    {
        cmd.type = S5_CMD_DISABLE;
        (void)tk_snd_mbf(g_mbf_s5_cmd, &cmd, sizeof(cmd), TMO_POL);
        return E_BUSY; /* "monitoring stopping, press again" */
    }
    servo_set_deg(deg);
    return E_OK;
}

/* Stop the obstacle subsystem now (monitoring and any avoid in progress) */
static ER obst_stop(void)
{
    s5_cmd_t cmd = { 0 };

    cmd.type = S5_CMD_ABORT;
    (void)tk_snd_mbf(g_mbf_s5_cmd, &cmd, sizeof(cmd), TMO_POL);
    cmd.type = S5_CMD_DISABLE;
    return tk_snd_mbf(g_mbf_s5_cmd, &cmd, sizeof(cmd), TMO_POL);
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
    motion_status_t state2;
    ER              er_val;
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
        case '!':
            er_val = E_OBJ;
            break;
        /* mode switch */
        case 'M':
        case 'N':
            mission_stop();
            g_wd_mode = ('M' == cmd) ? 0 : 2;
            er_val    = s2_cmd_stop();
            break;
        case 'A':
            g_wd_armed = g_wd_armed ? 0 : 1; /* toggle the autonomous unlock */
            if (!g_wd_armed && (1 == g_wd_mode))
            {
                mission_stop();
                g_wd_mode = 0;
            }
            er_val = E_OK;
            break;
        case 'F':
            if (!g_wd_armed)
            {
                er_val = E_OBJ; /* still locked: tune manually first */
                break;
            }
            /* the bench straight-hold would fight the line steering */
            g_s2_dbg_straight = 0;
            (void)s2_cmd_reset();
            (void)vc_post_event(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0, TMO_POL);
            er_val =
                vc_post_event(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0, TMO_POL);
            g_wd_mode = (E_OK == er_val) ? 1 : g_wd_mode;
            break;
        case 'Q':
            er_val = servo_test(45);
            break; /* + = left */
        case 'W':
            er_val = servo_test(0);
            break;
        case 'E':
            er_val = servo_test(-45);
            break;
        case '}':
            g_lt_next_nav = 0;
            er_val        = E_OK;
            break; /* junction: barcode decides */
        case '[':
            g_lt_next_nav = 1;
            er_val        = E_OK;
            break; /* every junction: left */
        case ']':
            g_lt_next_nav = 2;
            er_val        = E_OK;
            break; /* every junction: right */
        case '{':
            g_lt_next_nav = 3;
            er_val        = E_OK;
            break; /* every junction: straight */
        case '/':
            g_lt_steer_hold = 0;
            er_val          = E_OK;
            break; /* steering correction normal */
        case '+':
            g_lt_steer_hold = (g_lt_steer_hold + 1) % 3;
            er_val          = E_OK;
            break; /* next mode */
        case '`':
            g_lt_bc_edge = 1;
            er_val       = E_OK;
            break; /* barcode: edge finder */
        case '|':
            g_lt_bc_edge = 0;
            er_val       = E_OK;
            break; /* barcode: fixed cut-off */
        /* steps of 50 down to 50, then steps of 5 down to 10 (weak print) */
        case '@':
            g_lt_bc_swing = clamp_to(
                g_lt_bc_swing - ((g_lt_bc_swing > 50) ? 50 : 5), 10, 2000);
            er_val = E_OK;
            break;
        case '^':
            g_lt_bc_swing = clamp_to(
                g_lt_bc_swing + ((g_lt_bc_swing >= 50) ? 50 : 5), 10, 2000);
            er_val = E_OK;
            break;
        case '$': /* floor steps 0 (off) -> 300 -> 400 -> 600 -> 800 -> 1000 ->
                     0 */
            g_lt_bc_floor =
                (g_lt_bc_floor < 300)
                    ? 300
                    : ((g_lt_bc_floor < 400)
                           ? 400
                           : ((g_lt_bc_floor < 600)
                                  ? 600
                                  : ((g_lt_bc_floor < 800)
                                         ? 800
                                         : ((g_lt_bc_floor < 1000) ? 1000
                                                                   : 0))));
            er_val = E_OK;
            break;
        case '5':
            g_lt_junc_mode = 0;
            er_val         = E_OK;
            break; /* junction detection off */
        case '6':
            g_lt_junc_mode = 1;
            er_val         = E_OK;
            break; /* C + R */
        case '7':
            g_lt_junc_mode = 2;
            er_val         = E_OK;
            break; /* L + C + R */
        case '3':
            g_lt_junc_thr = clamp_to(g_lt_junc_thr - 25, 500, 1000);
            er_val        = E_OK;
            break;
        case '4':
            g_lt_junc_thr = clamp_to(g_lt_junc_thr + 25, 500, 1000);
            er_val        = E_OK;
            break;
        case '1':
            g_lt_lost = clamp_to(g_lt_lost - 10, 2, 500);
            er_val    = E_OK;
            break;
        case '2':
            g_lt_lost = clamp_to(g_lt_lost + 10, 2, 500);
            er_val    = E_OK;
            break;
        case 'w':
            g_lt_steer_inv = 1;
            er_val         = E_OK;
            break; /* swap steering direction */
        case 'X':
            g_lt_steer_inv = 0;
            er_val         = E_OK;
            break;
        case 'a': /* mission ignores barcodes */
            g_lt_bc_en = 0;
            er_val     = s3_request(S3_REQ_BC_DISABLE);
            break;
        case 'B':
            g_lt_bc_en = 1;
            er_val     = s3_request(S3_REQ_BC_ENABLE);
            break;
        case 'q':
            g_lt_imu_en = 0;
            er_val      = E_OK;
            break; /* mission ignores the IMU */
        case 'H':
            g_lt_imu_en = 1;
            er_val      = E_OK;
            break;
        case 'e':
            er_val = monitoring_on();
            break;
        case 'g': /* mission ignores the obstacle sensor */
            g_lt_obst_en = 0;
            er_val       = obst_stop();
            break;
        case 'G':
            g_lt_obst_en = 1;
            er_val       = monitoring_on();
            break;
        case 'y':
            er_val = s3_request(S3_REQ_CALIBRATE);
            break;
        case 'Y':
            er_val = s3_request(S3_REQ_RESET_LINE);
            break;
        case 'z':
            er_val = s3_request(S3_REQ_BC_ENABLE);
            break;
        case 'Z':
            er_val = s3_request(S3_REQ_BC_DISABLE);
            break;
        case 'j':
            er_val = s4_request_calibration();
            break;
        case 'J':
            er_val = s4_request_peak_reset();
            break;
        case 'f':
            er_val = s2_cmd_drive(g_wd_speed, 0);
            break;
        case 'b':
            er_val = s2_cmd_drive(-g_wd_speed, 0);
            break;
        case 'l':
            er_val = s2_cmd_drive(0, WD_SPIN_MM_S);
            break; /* + = left */
        case 'r':
            er_val = s2_cmd_drive(0, -WD_SPIN_MM_S);
            break;
        /* one wheel only at the test speed (other wheel target 0):
           left = speed - steer, right = speed + steer */
        case '(':
            er_val = s2_cmd_drive(g_wd_speed / 2, -(g_wd_speed / 2));
            break;
        case ')':
            er_val = s2_cmd_drive(g_wd_speed / 2, g_wd_speed / 2);
            break;
        case 's':
            if (1 == g_wd_mode)
            {
                mission_stop(); /* STOP also ends a running mission */
                g_wd_mode = 0;
            }
            er_val = s2_cmd_stop();
            break;
        case 'x':
            (void)vc_post_event(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0, TMO_POL);
            er_val = s2_cmd_reset();
            break;
        case 'L':
            er_val = s2_cmd_turn(S2_TURN_SPEED_MM_S, WD_TURN_DEG);
            break;
        case 'R':
            er_val = s2_cmd_turn(S2_TURN_SPEED_MM_S, -WD_TURN_DEG);
            break;
        /* live tuning: no motion command, just change the trims */
        case 't':
            g_s2_dbg_trim_l = clamp_trim(g_s2_dbg_trim_l - g_lt_trim_step);
            er_val          = E_OK;
            break;
        case 'T':
            g_s2_dbg_trim_l = clamp_trim(g_s2_dbg_trim_l + g_lt_trim_step);
            er_val          = E_OK;
            break;
        case 'u':
            g_s2_dbg_trim_r = clamp_trim(g_s2_dbg_trim_r - g_lt_trim_step);
            er_val          = E_OK;
            break;
        case 'U':
            g_s2_dbg_trim_r = clamp_trim(g_s2_dbg_trim_r + g_lt_trim_step);
            er_val          = E_OK;
            break;
        case 'S':
            g_lt_trim_step =
                (2 == g_lt_trim_step) ? 10 : ((10 == g_lt_trim_step) ? 25 : 2);
            er_val = E_OK;
            break;
        case '~':
            g_lt_lost_en = 0;
            er_val       = E_OK;
            break; /* mission ignores LINE_LOST */
        case '=':
            g_lt_lost_en = 1;
            er_val       = E_OK;
            break;
        case ',':
            g_lt_dmin = clamp_to(g_lt_dmin - 25, 0, S2_DUTY_MIN_MOVE);
            er_val    = E_OK;
            break;
        case '*':
            g_lt_dmin = clamp_to(g_lt_dmin + 25, 0, S2_DUTY_MIN_MOVE);
            er_val    = E_OK;
            break;
        case '8':
            g_lt_ff_l = clamp_to(g_lt_ff_l - 25, 400, 2500);
            er_val    = E_OK;
            break;
        case '9':
            g_lt_ff_l = clamp_to(g_lt_ff_l + 25, 400, 2500);
            er_val    = E_OK;
            break;
        case '-':
            g_lt_ff_r = clamp_to(g_lt_ff_r - 25, 400, 2500);
            er_val    = E_OK;
            break;
        case '.':
            g_lt_ff_r = clamp_to(g_lt_ff_r + 25, 400, 2500);
            er_val    = E_OK;
            break;
        case '0':
            g_lt_ilim = clamp_to(g_lt_ilim - 100, 0, 900);
            er_val    = E_OK;
            break;
        case '_':
            g_lt_ilim = clamp_to(g_lt_ilim + 100, 0, 900);
            er_val    = E_OK;
            break;
        case 'h':
            g_s2_dbg_straight = (0 != g_s2_dbg_straight) ? 0 : 1;
            er_val            = E_OK;
            break;
        case 'v':
            g_wd_speed = (g_wd_speed - WD_SPEED_STEP < WD_SPEED_MIN)
                             ? WD_SPEED_MIN
                             : (g_wd_speed - WD_SPEED_STEP);
            er_val     = E_OK;
            break;
        case 'V':
            g_wd_speed = (g_wd_speed + WD_SPEED_STEP > S2_MAX_SPEED_MM_S)
                             ? S2_MAX_SPEED_MM_S
                             : (g_wd_speed + WD_SPEED_STEP);
            er_val     = E_OK;
            break;
        /* live tuning of the controllers (see live_tune.h) */
        case 'p':
            g_lt_pid_kp100 = clamp_to(g_lt_pid_kp100 - 10, 0, 1000);
            er_val         = E_OK;
            break;
        case 'P':
            g_lt_pid_kp100 = clamp_to(g_lt_pid_kp100 + 10, 0, 1000);
            er_val         = E_OK;
            break;
        case 'i':
            g_lt_pid_ki100 = clamp_to(g_lt_pid_ki100 - 50, 0, 3000);
            er_val         = E_OK;
            break;
        case 'I':
            g_lt_pid_ki100 = clamp_to(g_lt_pid_ki100 + 50, 0, 3000);
            er_val         = E_OK;
            break;
        case 'o':
            g_lt_line_sp = clamp_to(g_lt_line_sp - 100, -1000, 1000);
            er_val       = E_OK;
            break;
        case 'O':
            g_lt_line_sp = clamp_to(g_lt_line_sp + 100, -1000, 1000);
            er_val       = E_OK;
            break;
        case 'm':
            g_lt_line_all = 0;
            er_val        = E_OK;
            break; /* centre + right steer */
        case 'n':
            g_lt_line_all = 1;
            er_val        = E_OK;
            break; /* left + centre + right */
        case 'k':
            g_lt_line_kp = clamp_to(g_lt_line_kp - 1, 0, 60);
            er_val       = E_OK;
            break;
        case 'K':
            g_lt_line_kp = clamp_to(g_lt_line_kp + 1, 0, 60);
            er_val       = E_OK;
            break;
        case 'd':
            g_lt_line_kd = clamp_to(g_lt_line_kd - 2, 0, 100);
            er_val       = E_OK;
            break;
        case 'D':
            g_lt_line_kd = clamp_to(g_lt_line_kd + 2, 0, 100);
            er_val       = E_OK;
            break;
        case ':':
            g_lt_curve = clamp_to(g_lt_curve - 100, 0, 800);
            er_val     = E_OK;
            break;
        case ';':
            g_lt_curve = clamp_to(g_lt_curve + 100, 0, 800);
            er_val     = E_OK;
            break;
        case 'c':
            g_lt_speed_pct = clamp_to(g_lt_speed_pct - 10, 20, 150);
            er_val         = E_OK;
            break;
        case 'C':
            g_lt_speed_pct = clamp_to(g_lt_speed_pct + 10, 20, 150);
            er_val         = E_OK;
            break;
        case '?':
            er_val = E_OK;
            break; /* status only */
        default:
            er_val = E_PAR;
            break;
    }

    p_out[0] = '\0';
    pos      = put_str(p_out, pos, out_max,
                  (E_OK == er_val) ? "ok"
                                        : (b_lock ? "LOCKED (another user is using "
                                                    "this)"
                                                  : "ERR"));
    pos      = put_str(p_out, pos, out_max, " mode=");
    pos      = put_int(p_out, pos, out_max, g_wd_mode);
    pos      = put_str(p_out, pos, out_max, " speed=");
    pos      = put_int(p_out, pos, out_max, g_wd_speed);
    pos      = put_str(p_out, pos, out_max, " trimL=");
    pos      = put_int(p_out, pos, out_max, g_s2_dbg_trim_l);
    pos      = put_str(p_out, pos, out_max, " trimR=");
    pos      = put_int(p_out, pos, out_max, g_s2_dbg_trim_r);
    pos      = put_str(p_out, pos, out_max,
                  (0 != g_s2_dbg_straight) ? " straight=ON" : " straight=OFF");
    if (E_OK == s2_get_status(&state2))
    {
        pos = put_str(p_out, pos, out_max, " state=");
        pos = put_int(p_out, pos, out_max, state2.state);
        pos = put_str(p_out, pos, out_max, " fault=");
        pos = put_int(p_out, pos, out_max, state2.fault);
        pos = put_str(p_out, pos, out_max, " L(mm/s)=");
        pos = put_int(p_out, pos, out_max, state2.speed_l_mm_s);
        pos = put_str(p_out, pos, out_max, " R(mm/s)=");
        pos = put_int(p_out, pos, out_max, state2.speed_r_mm_s);
        pos = put_str(p_out, pos, out_max, " ticksL=");
        pos = put_int(p_out, pos, out_max, (int32_t)state2.ticks_l);
        pos = put_str(p_out, pos, out_max, " ticksR=");
        pos = put_int(p_out, pos, out_max, (int32_t)state2.ticks_r);
        pos = put_str(p_out, pos, out_max, " heading(0.1deg)=");
        (void)put_int(p_out, pos, out_max, state2.heading_ddeg);
    }
    return (E_OK == er_val) ? 0 : -1;
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

static bool has(const char * p_x, char chr)
{
    while ('\0' != *p_x)
    {
        if (*p_x++ == chr)
        {
            return true;
        }
    }
    return false;
}

int32_t web_debug_snapshot(char * p_out, int32_t out_max, uint32_t client,
                           const char * p_x)
{
    motion_status_t  m_val;
    s3_line_status_t l_val;
    imu_status_t     imu;
    s5_status_t      o_val;
    vc_status_t      vc_val;
    int32_t          count2 = 0;

    p_out[0] = '\0';
    count2   = put_str(p_out, count2, out_max, "{\"cfg\":{");
    count2   = kv(p_out, count2, out_max, "speed", g_wd_speed);
    count2   = kv(p_out, count2, out_max, "trimL", g_s2_dbg_trim_l);
    count2   = kv(p_out, count2, out_max, "trimR", g_s2_dbg_trim_r);
    count2   = kv(p_out, count2, out_max, "straight", g_s2_dbg_straight);
    if (0U == client)
    {
        client = 1U;
    }
    count2 = kv(p_out, count2, out_max, "mode", g_wd_mode);
    count2 = kv(p_out, count2, out_max, "arm", g_wd_armed);
    count2 = kv(p_out, count2, out_max, "obst", g_lt_obst_en);
    count2 = kv(p_out, count2, out_max, "imu", g_lt_imu_en);
    count2 = kv(p_out, count2, out_max, "bc", g_lt_bc_en);
    count2 = kv(p_out, count2, out_max, "sinv", g_lt_steer_inv);
    count2 = kv(p_out, count2, out_max, "lost", g_lt_lost);
    count2 = kv(p_out, count2, out_max, "jthr", g_lt_junc_thr);
    count2 = kv(p_out, count2, out_max, "jmode", g_lt_junc_mode);
    count2 = kv(p_out, count2, out_max, "lkm", lock_state(0, client));
    count2 = kv(p_out, count2, out_max, "lkl", lock_state(1, client));
    count2 = kv(p_out, count2, out_max, "lki", lock_state(2, client));
    count2 = kv(p_out, count2, out_max, "lko", lock_state(3, client));
    count2 = kv(p_out, count2, out_max, "lkv", lock_state(4, client));
    count2 = kv(p_out, count2, out_max, "pidKp100", g_lt_pid_kp100);
    count2 = kv(p_out, count2, out_max, "lostEn", g_lt_lost_en);
    count2 = kv(p_out, count2, out_max, "nnav", g_lt_next_nav);
    count2 = kv(p_out, count2, out_max, "bcEdge", g_lt_bc_edge);
    count2 = kv(p_out, count2, out_max, "shold", g_lt_steer_hold);
    count2 = kv(p_out, count2, out_max, "bcSw", g_lt_bc_swing);
    count2 = kv(p_out, count2, out_max, "bcFl", g_lt_bc_floor);
    count2 = kv(p_out, count2, out_max, "dmin", g_lt_dmin);
    count2 = kv(p_out, count2, out_max, "curve", g_lt_curve);
    count2 = kv(p_out, count2, out_max, "ffL", g_lt_ff_l);
    count2 = kv(p_out, count2, out_max, "ffR", g_lt_ff_r);
    count2 = kv(p_out, count2, out_max, "ilim", g_lt_ilim);
    count2 = kv(p_out, count2, out_max, "tstep", g_lt_trim_step);
    count2 = kv(p_out, count2, out_max, "pidKi100", g_lt_pid_ki100);
    count2 = kv(p_out, count2, out_max, "lineKp", g_lt_line_kp);
    count2 = kv(p_out, count2, out_max, "lineKd", g_lt_line_kd);
    count2 = kv(p_out, count2, out_max, "lsp", g_lt_line_sp);
    count2 = kv(p_out, count2, out_max, "lall", g_lt_line_all);
    count2 = kv(p_out, count2, out_max, "pct", g_lt_speed_pct);
    count2 = put_str(p_out, count2, out_max, "\"_\":0},\"s2\":");

    if (!has(p_x, 'm'))
    {
        count2 = put_str(p_out, count2, out_max, "null");
    }
    else if (E_OK == s2_get_status(&m_val))
    {
        count2 = put_str(p_out, count2, out_max, "{");
        count2 = kv(p_out, count2, out_max, "state", m_val.state);
        count2 = kv(p_out, count2, out_max, "fault", m_val.fault);
        count2 = kv(p_out, count2, out_max, "tgtL", m_val.target_l_mm_s);
        count2 = kv(p_out, count2, out_max, "tgtR", m_val.target_r_mm_s);
        count2 = kv(p_out, count2, out_max, "vL", m_val.speed_l_mm_s);
        count2 = kv(p_out, count2, out_max, "vR", m_val.speed_r_mm_s);
        count2 = kv(p_out, count2, out_max, "dutyL", m_val.duty_l);
        count2 = kv(p_out, count2, out_max, "dutyR", m_val.duty_r);
        count2 = kv(p_out, count2, out_max, "ticksL", (int32_t)m_val.ticks_l);
        count2 = kv(p_out, count2, out_max, "ticksR", (int32_t)m_val.ticks_r);
        count2 = kv(p_out, count2, out_max, "dist", m_val.distance_mm);
        count2 = kv(p_out, count2, out_max, "head", m_val.heading_ddeg);
        count2 = kv(p_out, count2, out_max, "over", (int32_t)m_val.overruns);
        count2 = put_str(p_out, count2, out_max, "\"_\":0}");
    }
    else
    {
        count2 = put_str(p_out, count2, out_max, "null");
    }

    count2 = put_str(p_out, count2, out_max, ",\"s3\":");
    if (!has(p_x, 'l'))
    {
        count2 = put_str(p_out, count2, out_max, "null");
    }
    else if (E_OK == s3_get_line_status(&l_val))
    {
        count2 = put_str(p_out, count2, out_max, "{");
        count2 = kv(p_out, count2, out_max, "state", (int32_t)l_val.line_state);
        count2 = kv(p_out, count2, out_max, "pos", l_val.position);
        count2 = kv(p_out, count2, out_max, "err", l_val.error);
        count2 = kv(p_out, count2, out_max, "steer", l_val.steering);
        count2 = kv(p_out, count2, out_max, "mask", l_val.black_mask);
        count2 = kv(p_out, count2, out_max, "r0", l_val.raw[0]);
        count2 = kv(p_out, count2, out_max, "r1", l_val.raw[1]);
        count2 = kv(p_out, count2, out_max, "r2", l_val.raw[2]);
        count2 = kv(p_out, count2, out_max, "cal", l_val.calibrated ? 1 : 0);
        count2 = kv(p_out, count2, out_max, "bc", (int32_t)l_val.bc_state);
        count2 = kv(p_out, count2, out_max, "bcOk", (int32_t)l_val.bc_ok);
        count2 = kv(p_out, count2, out_max, "bcFail", (int32_t)l_val.bc_fail);
        /* Cast: shown, not computed (per-read records changed) */
        count2 = kv(p_out, count2, out_max, "bcR", (int32_t)g_s3_bc_rec_gen);
        count2 = put_str(p_out, count2, out_max, "\"bcTxt\":\"");
        count2 = put_str(p_out, count2, out_max, g_lt_bc_text); /* A-Z, '?' or
                                                                   '-' */
        count2 = put_str(p_out, count2, out_max, "\",\"bcHist\":\"");
        count2 = put_str(p_out, count2, out_max, g_lt_bc_hist); /* A-Z, '?' and
                                                                   spaces */
        count2 = put_str(p_out, count2, out_max, "\",");
        count2 =
            kv(p_out, count2, out_max, "bcN", (int32_t)l_val.bc_last_elems);
        count2 =
            kv(p_out, count2, out_max, "bcErr", (int32_t)l_val.bc_last_err);
        /* line_task timing (2026-11-11): loop runs, periods missed,
           events lost because the S3 event buffer was full */
        count2 =
            kv(p_out, count2, out_max, "lWake", (int32_t)g_s3_line_wakeups);
        count2 =
            kv(p_out, count2, out_max, "lOver", (int32_t)g_s3_line_overruns);
        count2 =
            kv(p_out, count2, out_max, "lDrop", (int32_t)l_val.evt_dropped);
        count2 = put_str(p_out, count2, out_max, "\"_\":0}");
    }
    else
    {
        count2 = put_str(p_out, count2, out_max, "null");
    }

    count2 = put_str(p_out, count2, out_max, ",\"s4\":");
    if (!has(p_x, 'i'))
    {
        count2 = put_str(p_out, count2, out_max, "null");
    }
    else if (E_OK == s4_get_status(&imu))
    {
        count2 = put_str(p_out, count2, out_max, "{");
        count2 = kv(p_out, count2, out_max, "state", (int32_t)imu.state);
        count2 = kv(p_out, count2, out_max, "motion", (int32_t)imu.motion);
        count2 = kv(p_out, count2, out_max, "cal", (int32_t)imu.calibrated);
        count2 = kv(p_out, count2, out_max, "fault", (int32_t)imu.fault);
        count2 = kv(p_out, count2, out_max, "pitch", imu.pitch_cdeg);
        count2 = kv(p_out, count2, out_max, "roll", imu.roll_cdeg);
        count2 = kv(p_out, count2, out_max, "head", imu.heading_cdeg);
        count2 = kv(p_out, count2, out_max, "turn", imu.turn_rate_cdeg_s);
        count2 = kv(p_out, count2, out_max, "humps", (int32_t)imu.hump_count);
        count2 = kv(p_out, count2, out_max, "maxPeak", imu.max_peak_um);
        count2 = kv(p_out, count2, out_max, "safety", imu.safety_stop);
        count2 = kv(p_out, count2, out_max, "height", imu.hump_height_um);
        count2 = kv(p_out, count2, out_max, "ax", imu.ax_mg);
        count2 = kv(p_out, count2, out_max, "ay", imu.ay_mg);
        count2 = kv(p_out, count2, out_max, "az", imu.az_mg);
        count2 = put_str(p_out, count2, out_max, "\"_\":0}");
    }
    else
    {
        count2 = put_str(p_out, count2, out_max, "null");
    }

    count2 = put_str(p_out, count2, out_max, ",\"s5\":");
    if (!has(p_x, 'o'))
    {
        count2 = put_str(p_out, count2, out_max, "null");
    }
    else if (E_OK == s5_get_status(&o_val))
    {
        count2 = put_str(p_out, count2, out_max, "{");
        count2 = kv(p_out, count2, out_max, "state", (int32_t)o_val.state);
        count2 = kv(p_out, count2, out_max, "fault", (int32_t)o_val.last_fault);
        count2 =
            kv(p_out, count2, out_max, "dist", (int32_t)o_val.last_distance_mm);
        if (0U != o_val.profile.valid)
        {
            count2 = kv(p_out, count2, out_max, "widthMm",
                        (int32_t)o_val.profile.width_mm);
        }
        else
        {
            count2 = put_str(p_out, count2, out_max, "\"widthMm\":null,");
        }
        count2 = kv(p_out, count2, out_max, "servo", servo_get_deg());
        count2 = kv(p_out, count2, out_max, "pings", (int32_t)o_val.pings);
        count2 = kv(p_out, count2, out_max, "timeouts",
                    (int32_t)o_val.echo_timeouts);
        count2 = kv(p_out, count2, out_max, "obst", (int32_t)o_val.obstacles);
        count2 = put_str(p_out, count2, out_max, "\"_\":0}");
    }
    else
    {
        count2 = put_str(p_out, count2, out_max, "null");
    }

    count2 = put_str(p_out, count2, out_max, ",\"vc\":");
    if (!has(p_x, 'v'))
    {
        count2 = put_str(p_out, count2, out_max, "null");
    }
    else if (E_OK == vc_get_status(&vc_val))
    {
        count2 = put_str(p_out, count2, out_max, "{");
        count2 = kv(p_out, count2, out_max, "state", (int32_t)vc_val.state);
        count2 = kv(p_out, count2, out_max, "prev", (int32_t)vc_val.prev_state);
        {
            /* state_since_ms is the kernel time when the state began, not a
               duration: send how long ago that was */
            SYSTIM now;
            (void)tk_get_tim(&now);
            count2 = kv(p_out, count2, out_max, "sinceMs",
                        (int32_t)((uint32_t)now.lo - vc_val.state_since_ms));
        }
        count2 = kv(p_out, count2, out_max, "fault",
                    (int32_t)vc_val.last_fault_code);
        count2 = kv(p_out, count2, out_max, "barcodes",
                    (int32_t)vc_val.barcodes_read);
        count2 = kv(p_out, count2, out_max, "events",
                    (int32_t)vc_val.events_handled);
        count2 = kv(p_out, count2, out_max, "wifi", vc_val.wifi_up ? 1 : 0);
        count2 = put_str(p_out, count2, out_max, "\"_\":0}");
    }
    else
    {
        count2 = put_str(p_out, count2, out_max, "null");
    }

    (void)put_str(p_out, count2, out_max, "}");
    return 0;
}

int32_t web_debug_barcode(char * p_out, int32_t out_max)
{
    static uint32_t  width[S3_BC_MAX_ELEMENTS]; /* static: small task stack */
    static uint32_t  d_val[S3_BC_MAX_ELEMENTS]; /* same, by wheel distance */
    uint32_t         unit   = 0U;
    int32_t          g_res  = 255; /* shape read (S3_BC_GEOM) */
    int32_t          g_best = 0;
    int32_t          g_err  = 0;
    int32_t          g_mrg  = 0;
    int32_t          g_fit  = 0;
    int32_t          g_ms   = 0;
    s3_line_status_t l_val;
    uint32_t         seq = 0U;
    uint32_t         cnt = 0U;
    uint32_t         kdx;
    int32_t          tries;
    int32_t          count2 = 0;
    int32_t          err    = -1;

    /* copy, and copy again if barcode_task wrote in between */
    for (tries = 0; tries < 3; tries++)
    {
        seq = g_s3_bc_dbg_seq;
        cnt = g_s3_bc_dbg_n;
        cnt = (cnt > S3_BC_MAX_ELEMENTS) ? S3_BC_MAX_ELEMENTS : cnt;
        for (kdx = 0U; kdx < cnt; kdx++)
        {
            width[kdx] = g_s3_bc_dbg_us[kdx];
            d_val[kdx] = g_s3_bc_dbg_pos[kdx];
        }
        unit   = g_s3_bc_dbg_unit;
        g_res  = (int32_t)g_s3_bc_geom_res;
        g_best = (int32_t)g_s3_bc_geom_best;
        g_err  = (int32_t)g_s3_bc_geom_err_x100;
        g_mrg  = (int32_t)g_s3_bc_geom_margin_x100;
        g_fit  = (int32_t)g_s3_bc_geom_fit;
        g_ms   = (int32_t)g_s3_bc_geom_ms;
        if (seq == g_s3_bc_dbg_seq)
        {
            break;
        }
    }
    if (E_OK == s3_get_line_status(&l_val))
    {
        err = (int32_t)l_val.bc_last_err;
    }
    count2 = put_str(p_out, count2, out_max, "{");
    count2 = kv(p_out, count2, out_max, "seq", (int32_t)seq);
    count2 = kv(p_out, count2, out_max, "n", (int32_t)cnt);
    count2 = kv(p_out, count2, out_max, "err", err);
    count2 = put_str(p_out, count2, out_max, "\"txt\":\"");
    count2 = put_str(p_out, count2, out_max, g_lt_bc_text);
    count2 = put_str(p_out, count2, out_max, "\",\"w\":[");
    for (kdx = 0U; kdx < cnt; kdx++)
    {
        /* Cast: one element is at most S3_BC_ELEMENT_MAX_MS, fits int32 */
        count2 = put_int(p_out, count2, out_max,
                         (int32_t)((width[kdx] + 50U) / 100U));
        if ((kdx + 1U) < cnt)
        {
            count2 = put_str(p_out, count2, out_max, ",");
        }
    }
    /* d: the same elements in encoder counts (L + R, ~0.15 mm each);
       u: 1 = the decoder used d, 0 = it used w, 2 = shape read;
       gr..gf: the shape read (subsystem3.h g_s3_bc_geom_*) */
    count2 = put_str(p_out, count2, out_max, "],");
    count2 = kv(p_out, count2, out_max, "u", (int32_t)unit);
    count2 = kv(p_out, count2, out_max, "gr", g_res);
    count2 = kv(p_out, count2, out_max, "gb", g_best);
    count2 = kv(p_out, count2, out_max, "ge", g_err);
    count2 = kv(p_out, count2, out_max, "gm", g_mrg);
    count2 = kv(p_out, count2, out_max, "gf", g_fit);
    count2 = kv(p_out, count2, out_max, "gt", g_ms);
#if S3_L_FAST
    /* 500 Hz L front end: samples, late, longest us, ADC errors, empty
       records, samples in the last record.  Casts: shown, not computed */
    count2 = put_str(p_out, count2, out_max, "\"lf\":[");
    count2 = put_int(p_out, count2, out_max, (int32_t)g_s3_lf_isr);
    count2 = put_str(p_out, count2, out_max, ",");
    count2 = put_int(p_out, count2, out_max, (int32_t)g_s3_lf_late);
    count2 = put_str(p_out, count2, out_max, ",");
    count2 = put_int(p_out, count2, out_max, (int32_t)g_s3_lf_max_us);
    count2 = put_str(p_out, count2, out_max, ",");
    count2 = put_int(p_out, count2, out_max, (int32_t)g_s3_lf_adc_err);
    count2 = put_str(p_out, count2, out_max, ",");
    count2 = put_int(p_out, count2, out_max, (int32_t)g_s3_lf_stall);
    count2 = put_str(p_out, count2, out_max, ",");
    count2 = put_int(p_out, count2, out_max, (int32_t)g_s3_lf_used);
    count2 = put_str(p_out, count2, out_max, "],");
#endif
    count2 = put_str(p_out, count2, out_max, "\"d\":[");
    for (kdx = 0U; kdx < cnt; kdx++)
    {
        /* Cast: one element is a few thousand counts at most */
        count2 = put_int(p_out, count2, out_max, (int32_t)d_val[kdx]);
        if ((kdx + 1U) < cnt)
        {
            count2 = put_str(p_out, count2, out_max, ",");
        }
    }
    /* rec: the last S3_BC_REC_N per-read records (subsystem3.h
       s3_bc_rec_t): seq, t_ms, pos, len, white, margin, n, end, path,
       letter (char code, 0 none), lock_why, lock_len, v_start, v_min,
       v_end, v_up, up_len.  Casts: shown, not computed. */
    count2 = put_str(p_out, count2, out_max, "],\"rec\":[");
    for (kdx = 0U; kdx < S3_BC_REC_N; kdx++)
    {
        volatile s3_bc_rec_t const * p     = &g_s3_bc_rec[kdx];
        int32_t const                f[17] = {
            (int32_t)p->seq,     (int32_t)p->t_ms,     (int32_t)p->pos,
            (int32_t)p->len,     (int32_t)p->white,    (int32_t)p->margin,
            (int32_t)p->n,       (int32_t)p->end,      (int32_t)p->path,
            (int32_t)p->letter,  (int32_t)p->lock_why, (int32_t)p->lock_len,
            (int32_t)p->v_start, (int32_t)p->v_min,    (int32_t)p->v_end,
            (int32_t)p->v_up,    (int32_t)p->up_len
        };
        uint32_t jdx;
        count2 = put_str(p_out, count2, out_max, (0U == kdx) ? "[" : ",[");
        for (jdx = 0U; jdx < 17U; jdx++)
        {
            count2 = put_int(p_out, count2, out_max, f[jdx]);
            count2 = put_str(p_out, count2, out_max, (jdx < 16U) ? "," : "]");
        }
    }
    count2 = put_str(p_out, count2, out_max, "]}");
    return count2;
}

int32_t web_debug_ltrace(char * p_out, int32_t out_max, int32_t live,
                         int32_t page)
{
    uint32_t idx   = g_s3_ltrace_idx;
    uint32_t count = (idx < S3_LTRACE_N) ? idx : S3_LTRACE_N;
    uint32_t start = idx - count;
    bool     b_frz = (0 == live) && (0U != g_s3_ltrace_frz_n);
    uint32_t first = 0U; /* first sample of this page (frozen copy) */
    uint32_t total;      /* samples in the whole trace */
    uint32_t kdx;
    int32_t  count2 = 0;
    /* raw levels where L switches (normalised threshold +- S3_BC_HYST),
       for an analogue L scaled between S3_L_ADC_WHITE and _BLACK */
    int32_t  span   = (int32_t)S3_L_ADC_BLACK - (int32_t)S3_L_ADC_WHITE;
    int32_t  b_on   = (int32_t)S3_L_ADC_WHITE +
                   ((span * (S3_BLACK_THRESHOLD + S3_BC_HYST)) / 1000);
    int32_t off = (int32_t)S3_L_ADC_WHITE +
                  ((span * (S3_BLACK_THRESHOLD - S3_BC_HYST)) / 1000);

    if (0 != g_lt_bc_edge)
    {
        /* edge finder: the only fixed level is the floor */
        b_on = g_lt_bc_floor;
        off  = g_lt_bc_floor;
    }
    count2 = put_str(p_out, count2, out_max, "{");
    count2 = kv(p_out, count2, out_max, "on", b_on);
    count2 = kv(p_out, count2, out_max, "off", off);
    if (b_frz)
    {
        /* the frozen copy (taken when the read ended), one page of it */
        total = g_s3_ltrace_frz_n;
        total = (total > S3_LTRACE_N) ? S3_LTRACE_N : total;
        /* Cast: page is a single digit, checked >= 0 */
        first = (page > 0) ? ((uint32_t)page * WD_LT_PAGE) : 0U;
        first = (first > total) ? total : first;
        count = total - first;
        count = (count > WD_LT_PAGE) ? WD_LT_PAGE : count;
    }
    else
    {
        /* live: only the newest page, so one reply is one moment */
        page  = 0;
        total = (count > WD_LT_PAGE) ? WD_LT_PAGE : count;
        start = idx - total;
        count = total;
    }
    count2 = kv(p_out, count2, out_max, "frz", b_frz ? 1 : 0);
    count2 = kv(p_out, count2, out_max, "edge", g_lt_bc_edge);
    count2 = kv(p_out, count2, out_max, "sw", g_lt_bc_swing);
    /* Casts: counts are at most S3_LTRACE_N, the seq is only compared */
    count2 = kv(p_out, count2, out_max, "n", (int32_t)total);
    count2 = kv(p_out, count2, out_max, "p", page);
    count2 = kv(p_out, count2, out_max, "q", (int32_t)g_s3_bc_dbg_seq);
    count2 = put_str(p_out, count2, out_max, "\"v\":[");
    for (kdx = 0U; kdx < count; kdx++)
    {
        count2 =
            put_int(p_out, count2, out_max,
                    b_frz ? (int32_t)g_s3_ltrace_frz[first + kdx]
                          : (int32_t)g_s3_ltrace[(start + kdx) % S3_LTRACE_N]);
        if ((kdx + 1U) < count)
        {
            count2 = put_str(p_out, count2, out_max, ",");
        }
    }
    /* f: wheel counts (L + R) since the sample before; y: right - left */
    count2 = put_str(p_out, count2, out_max, "],\"f\":[");
    for (kdx = 0U; kdx < count; kdx++)
    {
        count2 = put_int(
            p_out, count2, out_max,
            b_frz ? (int32_t)g_s3_ltrace_frz_fwd[first + kdx]
                  : (int32_t)g_s3_ltrace_fwd[(start + kdx) % S3_LTRACE_N]);
        if ((kdx + 1U) < count)
        {
            count2 = put_str(p_out, count2, out_max, ",");
        }
    }
    count2 = put_str(p_out, count2, out_max, "],\"y\":[");
    for (kdx = 0U; kdx < count; kdx++)
    {
        count2 = put_int(
            p_out, count2, out_max,
            b_frz ? (int32_t)g_s3_ltrace_frz_yaw[first + kdx]
                  : (int32_t)g_s3_ltrace_yaw[(start + kdx) % S3_LTRACE_N]);
        if ((kdx + 1U) < count)
        {
            count2 = put_str(p_out, count2, out_max, ",");
        }
    }
    count2 = put_str(p_out, count2, out_max, "]}");
    return count2;
}

int32_t web_debug_imutrace(char * p_out, int32_t out_max, int32_t page)
{
    s4_rec_t rec;
    uint32_t count;
    uint32_t first;
    uint32_t last;
    uint32_t kdx;
    int32_t  count2 = 0;

    if (page <= 0)
    {
        page = 0;
        s4_rec_freeze(1);
    }
    count = s4_rec_count();
    /* Cast: page is a single digit. */
    first = (uint32_t)page * WD_IMU_PAGE;
    last  = first + WD_IMU_PAGE;
    if (last >= count)
    {
        last = count;
    }
    count2 = put_str(p_out, count2, out_max, "{");
    count2 = kv(p_out, count2, out_max, "n", (int32_t)count);
    count2 = kv(p_out, count2, out_max, "p", page);
    count2 = put_str(p_out, count2, out_max, "\"d\":[");
    for (kdx = first; kdx < last; kdx++)
    {
        s4_rec_get(kdx, &rec);
        count2 = put_int(p_out, count2, out_max, rec.ax_mg);
        count2 = put_str(p_out, count2, out_max, ",");
        count2 = put_int(p_out, count2, out_max, rec.ay_mg);
        count2 = put_str(p_out, count2, out_max, ",");
        count2 = put_int(p_out, count2, out_max, rec.az_mg);
        count2 = put_str(p_out, count2, out_max, ",");
        count2 = put_int(p_out, count2, out_max, rec.pitch_cdeg);
        count2 = put_str(p_out, count2, out_max, ",");
        count2 = put_int(p_out, count2, out_max, (int32_t)rec.state);
        count2 = put_str(p_out, count2, out_max, ",");
        count2 = put_int(p_out, count2, out_max, (int32_t)rec.safety);
        count2 = put_str(p_out, count2, out_max, ",");
        count2 = put_int(p_out, count2, out_max, rec.speed_mm_s);
        count2 = put_str(p_out, count2, out_max, ",");
        count2 = put_int(p_out, count2, out_max, rec.lift_v);
        if ((kdx + 1U) < last)
        {
            count2 = put_str(p_out, count2, out_max, ",");
        }
    }
    count2 = put_str(p_out, count2, out_max, "]}");
    if (last >= count)
    {
        s4_rec_freeze(0); /* that was the last page */
    }
    return count2;
}

#endif /* S2_WEBDEBUG */

/*** end of file ***/
