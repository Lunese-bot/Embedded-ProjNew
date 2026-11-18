/* Production motor bridge and scan profile regression tests, no Pico required. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#define APP_H
#define VEHICLE_CONTROLLER_H
#define VC_HOST_TEST
#include "../../../app_program/vehicle_controller/vc_types.h"
typedef int ID;
typedef int ER;
typedef int TMO;
#define E_OK 0
#define E_PAR (-1)
#define E_TMOUT (-2)
#define TMO_POL 0
#define TWF_ORW 1
#define TWF_BITCLR 2
#include "../../../app_program/subsystem2/subsystem2.h"
ID g_flg_motion;
volatile int32_t g_lt_speed_pct = 100, g_lt_steer_inv, g_lt_curve = 600;
static int32_t speed_sent, angle_sent;
static UINT pending_flags;
static ER tk_clr_flg(ID id, UINT mask)
{ (void)id; pending_flags &= mask; return E_OK; }
static ER tk_wai_flg(ID id, UINT bits, UINT mode, UINT *out, TMO timeout)
{
    (void)id; (void)mode; (void)timeout;
    *out = pending_flags & bits;
    pending_flags &= ~bits;
    return *out ? E_OK : E_TMOUT;
}
ER s2_cmd_stop(void) { speed_sent = 0; return E_OK; }
ER s2_emergency_stop(void) { speed_sent = 0; return E_OK; }
ER s2_cmd_reset(void) { return E_OK; }
ER s2_cmd_drive(int32_t speed, int32_t steer)
{ (void)steer; speed_sent = speed; return E_OK; }
ER s2_cmd_drive_dist(int32_t speed, int32_t distance)
{ (void)distance; speed_sent = speed; return E_OK; }
ER s2_cmd_turn(int32_t speed, int32_t angle)
{ speed_sent = speed; angle_sent = angle; return E_OK; }
ER s2_get_status(motion_status_t *out) { *out = (motion_status_t){0}; return E_OK; }
#include "../../../app_program/vehicle_controller/vc_bridge_s2.c"
#include "../../../app_program/subsystem5/obstacle_logic.c"
int main(void)
{
    vc_event_t event;
    pending_flags = MFLG_STOPPED; /* stale stop must not start the scan */
    assert(vc_bridge_s2_move(VC_MOVE_STOP_CONFIRM, 0) == E_OK);
    assert(!vc_bridge_s2_next(&event));
    pending_flags = MFLG_STOPPED;
    assert(vc_bridge_s2_next(&event) && event.type == VC_EV_MOTION_STOPPED);
    assert(event.src == VC_SRC_MOTION);
    puts("PASS stop confirmation ignores stale flags and forwards fresh motor stop");
    vc_bridge_s2_set_obstacle_mode(true);
    for (int pct = 50; pct <= 200; pct += 50)
    {
        g_lt_speed_pct = pct;
        vc_bridge_s2_move(VC_MOVE_TURN_LEFT, 45);
        assert(speed_sent == 60 && angle_sent == 45);
        vc_bridge_s2_move(VC_MOVE_TURN_RIGHT, 45);
        assert(speed_sent == 60 && angle_sent == -45);
    }
    puts("PASS obstacle pivots stay slow at dashboard speed 50..200 percent");
    vc_bridge_s2_set_obstacle_mode(false);
    g_lt_speed_pct = 100;
    vc_bridge_s2_move(VC_MOVE_TURN_LEFT, 45); assert(speed_sent == 120);
    vc_bridge_s2_move(VC_MOVE_FORWARD, 120); assert(speed_sent == 120);
    vc_bridge_s2_move(VC_MOVE_STEER, 0); assert(speed_sent == 120);
    puts("PASS normal mission turns and line-follow speed remain unchanged");
    s5_sample_t coarse[] = {{-60,1000},{-30,200},{0,200},{30,200},{60,1000}};
    s5_sample_t fine[40], merged[40]; int8_t angles[40]; s5_profile_t profile;
    assert(s5_build_profile(coarse, 5, &profile));
    int n = s5_fine_angles(&profile, angles, 40);
    for (int i = 0; i < n; i++) { fine[i] = (s5_sample_t){angles[i],200}; }
    n = s5_merge_samples(coarse, 5, fine, n, merged, 40);
    assert(s5_build_profile(merged, n, &profile));
    assert(profile.clear_left_mm == 1000 && profile.clear_right_mm == 1000);
    puts("PASS fine scan retains clear sides measured by coarse scan");
    s5_step_t plan[S5_PLAN_MAX];
    for (int side = S5_SIDE_LEFT; side <= S5_SIDE_RIGHT; side++)
    {
        profile.bypass_side = (uint8_t)side;
        int steps = s5_plan_bypass(&profile, plan, S5_PLAN_MAX);
        assert(steps == 7 && plan[0].m.kind == S5_MOVE_STOP);
        assert(plan[1].m.angle_deg == (side == S5_SIDE_LEFT ? 45 : -45));
        assert(plan[steps - 1].search_line);
        for (int i = 0; i < steps; i++)
        {
            if (plan[i].m.kind == S5_MOVE_FORWARD) { assert(plan[i].m.speed_mm_s == 60); }
        }
    }
    puts("PASS both bypass directions stop first, drive slowly and search for black line");
    puts("SUMMARY 5 navigation regression groups passed");
    return 0;
}
