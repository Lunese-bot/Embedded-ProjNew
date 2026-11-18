/* Production bridges, scan profiles, and avoidance task with simulated motion. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#define APP_H
#define VEHICLE_CONTROLLER_H
#define VC_HOST_TEST
#include "../../../app_program/vehicle_controller/vc_types.h"
typedef int ID;
typedef int ER;
typedef int TMO;
typedef struct { uint32_t lo; } SYSTIM;
#define E_OK 0
#define E_PAR (-1)
#define E_TMOUT (-2)
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TWF_ORW 1
#define TWF_BITCLR 2
#include "../../../app_program/subsystem2/subsystem2.h"
#include "../../../app_program/subsystem5/s5_local.h"
ID g_flg_motion;
ID g_mbf_s5_cmd = 1, g_mbf_s5_evt = 2, g_flg_s5 = 3;
volatile int32_t g_lt_obst_en = 1;
volatile bool g_b_s3_bc_lock = false; /* barcode lock flag read by vc_bridge_s2.c */
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

typedef enum
{
    TAPE_ABSENT,
    TAPE_ON_LATER_PASS,
    TAPE_ON_LEFT_PROBE,
    TAPE_ON_RIGHT_PROBE,
    TAPE_LOST_AFTER_ALIGNMENT,
    TAPE_LOST_DURING_REJOIN,
    TAPE_ON_NEARBY_CRAWL
} tape_scenario_t;

static struct
{
    uint32_t now_ms;
    s5_state_t state;
    s5_motion_req_t move;
    bool waiting_for_motion;
    tape_scenario_t tape;
    bool black;
    bool ir_search_enabled;
    int32_t heading;
    int32_t first_sighting_heading;
    int32_t recovery_headings[S5_MAX_RECOVERY];
    uint16_t recovery_distances[S5_MAX_RECOVERY];
    int32_t recovery_passes;
    int32_t sightings;
    int32_t search_stops;
    int32_t alignment_stops;
    int32_t rejoin_drives;
    int32_t rejoin_stops;
    int32_t nearby_crawls;
    int32_t successes;
    uint8_t outcome;
    s5_fault_code_t fault;
    bool critical_outcome;
} robot;

static s5_evt_t bridge_input;
static bool bridge_input_ready;
static s5_cmd_t bridge_reply;
static int32_t bridge_replies;

static int32_t tk_rcv_mbf(ID id, void *out, TMO timeout)
{
    if (g_mbf_s5_evt == id)
    {
        if (!bridge_input_ready)
        {
            return E_TMOUT;
        }
        *(s5_evt_t *)out = bridge_input;
        bridge_input_ready = false;
        return sizeof(s5_evt_t);
    }

    assert(g_mbf_s5_cmd == id && robot.waiting_for_motion);
    if ((TAPE_ABSENT != robot.tape) && (S5_MOVE_FORWARD == robot.move.kind) &&
        (((S5_ST_SEARCH_LINE == robot.state) && (0 == robot.sightings)) ||
         ((TAPE_ON_NEARBY_CRAWL == robot.tape) && (1 == robot.sightings) &&
          (S5_REJOIN_SEARCH_MM == robot.move.distance_mm)) ||
         ((TAPE_ON_LATER_PASS == robot.tape) &&
          (S5_ST_LINE_RECOVERY == robot.state) && (2 == robot.recovery_passes) &&
          (1 == robot.sightings))))
    {
        /* Tape appears while creeping, before the distance move finishes. */
        if (0 == robot.sightings)
        {
            robot.first_sighting_heading = robot.heading;
        }
        robot.sightings++;
        robot.black = true;
        robot.now_ms += (uint32_t)timeout;
        return E_TMOUT;
    }
    if (S5_MOVE_STOP == robot.move.kind)
    {
        if (robot.rejoin_drives > robot.rejoin_stops)
        {
            assert(robot.ir_search_enabled);
            robot.rejoin_stops++;
        }
        else if (robot.ir_search_enabled)
        {
            robot.search_stops++;
        }
        else if (robot.sightings > 0)
        {
            robot.alignment_stops++;
        }
    }
    if (S5_MOVE_TURN == robot.move.kind)
    {
        robot.heading += robot.move.angle_deg;
        if (1 == robot.sightings)
        {
            /* Tape is gone at heading zero after the first sighting. The
               two probe cases make it visible near that heading. */
            robot.black =
                ((TAPE_ON_LEFT_PROBE == robot.tape) &&
                 (S5_REJOIN_PROBE_DEG == robot.heading)) ||
                ((TAPE_ON_RIGHT_PROBE == robot.tape) &&
                 (-S5_REJOIN_PROBE_DEG == robot.heading)) ||
                ((TAPE_LOST_DURING_REJOIN == robot.tape) &&
                 (0 == robot.heading) && (0 == robot.rejoin_drives));
        }
    }
    if ((S5_MOVE_FORWARD == robot.move.kind) &&
        (S5_REJOIN_CHECK_MM == robot.move.distance_mm) &&
        (TAPE_LOST_DURING_REJOIN == robot.tape))
    {
        robot.black = false;
    }
    robot.now_ms++;
    robot.waiting_for_motion = false;
    *(s5_cmd_t *)out = (s5_cmd_t){ S5_CMD_MOTION_DONE, 1, robot.move.seq };
    return sizeof(s5_cmd_t);
}

static ER tk_snd_mbf(ID id, const void *msg, int32_t size, TMO timeout)
{
    (void)timeout;
    assert(g_mbf_s5_cmd == id && sizeof(s5_cmd_t) == (size_t)size);
    bridge_reply = *(const s5_cmd_t *)msg;
    bridge_replies++;
    return E_OK;
}

static void tk_get_tim(SYSTIM *now) { now->lo = robot.now_ms; }
static void tk_dly_tsk(TMO delay) { robot.now_ms += (uint32_t)delay; }
static int32_t tk_get_tid(void) { return 1; }
static ER tk_set_flg(ID id, UINT bits) { (void)id; (void)bits; return E_OK; }
static void tm_printf(const uint8_t *fmt, ...) { (void)fmt; }
static void app_check(ER err, const char *name) { (void)name; assert(E_OK == err); }

void s5_set_state(s5_state_t state) { robot.state = state; }
void s5_set_fault(s5_fault_code_t fault) { robot.fault = fault; }
void s5_count_avoid_ok(void) { robot.successes++; }
void s5_load_profile(s5_profile_t *profile) { *profile = (s5_profile_t){ 0 }; }
ER s5_get_status(s5_status_t *status) { *status = (s5_status_t){ 0 }; return E_OK; }
void s5_port_set_line_search(bool b_active)
{
    robot.ir_search_enabled = b_active;
}

bool s5_port_line_detected(void)
{
    assert(robot.ir_search_enabled);
    return robot.black;
}

ER s5_send_evt(s5_evt_t *event)
{
    if (S5_EVT_MOTION_REQ == event->type)
    {
        const s5_motion_req_t *move = &event->u.motion;
        if ((S5_ST_BYPASSING == robot.state) || (S5_MOVE_TURN == move->kind))
        {
            assert(!robot.ir_search_enabled);
        }
        /* A stop may replace a running search. A turn or drive must wait. */
        assert(!robot.waiting_for_motion || (S5_MOVE_STOP == move->kind));
        if ((S5_MOVE_FORWARD == move->kind) && (S5_REJOIN_CHECK_MM == move->distance_mm))
        {
            assert(robot.ir_search_enabled && robot.black);
            assert(S5_SEARCH_SPEED_MM_S == move->speed_mm_s);
            robot.rejoin_drives++;
        }
        else if ((S5_MOVE_FORWARD == move->kind) &&
                 (S5_REJOIN_SEARCH_MM == move->distance_mm))
        {
            assert(robot.ir_search_enabled && !robot.black);
            assert(S5_SEARCH_SPEED_MM_S == move->speed_mm_s);
            robot.nearby_crawls++;
        }
        else if ((S5_ST_LINE_RECOVERY == robot.state) && (S5_MOVE_FORWARD == move->kind))
        {
            assert(robot.recovery_passes < S5_MAX_RECOVERY);
            assert(60 == move->speed_mm_s);
            robot.recovery_headings[robot.recovery_passes] = robot.heading;
            robot.recovery_distances[robot.recovery_passes] = move->distance_mm;
            robot.recovery_passes++;
        }
        robot.move = *move;
        robot.waiting_for_motion = true;
    }
    else
    {
        assert(!robot.waiting_for_motion);
        assert(!robot.ir_search_enabled);
        robot.outcome = event->type;
    }
    return E_OK;
}

ER s5_send_critical_to(ID id, s5_evt_t *event)
{
    assert(g_mbf_s5_evt == id);
    if (S5_EVT_MOTION_REQ != event->type)
    {
        robot.critical_outcome = true;
    }
    return s5_send_evt(event);
}

#include "../../../app_program/subsystem5/task_obstacle_avoid.c"
#include "../../../app_program/vehicle_controller/vc_bridge_s5.c"

static void simulate_bypass(s5_side_t side, tape_scenario_t tape)
{
    s5_profile_t profile = { .distance_mm = 200, .valid = 1, .bypass_side = side };

    memset(&robot, 0, sizeof(robot));
    robot.tape = tape;
    g_heading_deg = 0;
    bypass(&profile);
}

static void test_line_recovery(void)
{
    for (int32_t side = S5_SIDE_LEFT; side <= S5_SIDE_RIGHT; side++)
    {
        int32_t sign = (S5_SIDE_LEFT == side) ? 1 : -1;

        simulate_bypass((s5_side_t)side, TAPE_ABSENT);
        assert(3 == robot.recovery_passes && 0 == robot.successes);
        assert(S5_RETURN_TURN_DEG * sign == robot.recovery_headings[0]);
        assert(-S5_RETURN_TURN_DEG * sign == robot.recovery_headings[1]);
        assert(0 == robot.recovery_headings[2]);
        assert(140 == robot.recovery_distances[0]);
        assert(280 == robot.recovery_distances[1]);
        assert(150 == robot.recovery_distances[2]);
        assert(S5_EVT_AVOID_FAILED == robot.outcome && robot.critical_outcome);
        assert(S5_FAULT_LINE_NOT_FOUND == robot.fault);
    }
    puts("[TEST] S5-H09 PASS  missing line on either side exhausts three slow passes");

    simulate_bypass(S5_SIDE_LEFT, TAPE_ON_LATER_PASS);
    assert(2 == robot.recovery_passes && 2 == robot.sightings);
    assert(2 == robot.search_stops); /* each sighting stopped the driving leg */
    assert(S5_EVT_LINE_REACQUIRED == robot.outcome && 1 == robot.successes);
    assert(0 == robot.heading && robot.black);
    puts("[TEST] S5-H10 PASS  lost line during alignment retries and finds it again");
    puts("[TEST] S5-H12 PASS  IR is requested for line checks and released before pivots and handoff");
}

static void test_nearby_rejoin(void)
{
    simulate_bypass(S5_SIDE_RIGHT, TAPE_ON_LEFT_PROBE);
    assert(0 == robot.recovery_passes && 1 == robot.sightings);
    assert(S5_EVT_LINE_REACQUIRED == robot.outcome && 1 == robot.successes);
    assert(S5_REJOIN_PROBE_DEG == robot.heading && robot.black);
    assert(2 == robot.alignment_stops); /* heading zero and the left probe */
    assert(1 == robot.rejoin_drives && 1 == robot.rejoin_stops);

    simulate_bypass(S5_SIDE_RIGHT, TAPE_ON_RIGHT_PROBE);
    assert(0 == robot.recovery_passes && S5_EVT_LINE_REACQUIRED == robot.outcome);
    assert(-S5_REJOIN_PROBE_DEG == robot.heading && robot.black);
    assert(3 == robot.alignment_stops); /* heading zero and both probes */
    assert(1 == robot.rejoin_drives && 1 == robot.rejoin_stops);
    puts("[TEST] S5-H14 PASS  nearby checks and crawl verify tape on either side");

    simulate_bypass(S5_SIDE_LEFT, TAPE_LOST_AFTER_ALIGNMENT);
    assert(4 == robot.alignment_stops); /* restore heading zero and check again */
    assert(0 == robot.heading && !robot.black);
    assert(3 == robot.recovery_passes && 0 == robot.successes);
    assert(S5_EVT_AVOID_FAILED == robot.outcome && robot.critical_outcome);
    assert(S5_FAULT_LINE_NOT_FOUND == robot.fault);
    puts("[TEST] S5-H15 PASS  a missed local check falls back to bounded recovery and stop");

    simulate_bypass(S5_SIDE_LEFT, TAPE_LOST_DURING_REJOIN);
    assert(1 == robot.rejoin_drives && 1 == robot.rejoin_stops);
    assert(3 == robot.recovery_passes && 0 == robot.successes);
    assert(S5_EVT_AVOID_FAILED == robot.outcome && robot.critical_outcome);
    assert(S5_FAULT_LINE_NOT_FOUND == robot.fault);
    puts("[TEST] S5-H17 PASS  tape lost during the rejoin crawl keeps recovery active");
}

static void test_nearby_crawl(void)
{
    for (int32_t side = S5_SIDE_LEFT; side <= S5_SIDE_RIGHT; side++)
    {
        const int32_t inward = (S5_SIDE_LEFT == side) ? -1 : 1;

        simulate_bypass((s5_side_t)side, TAPE_ON_NEARBY_CRAWL);
        assert(inward * S5_RETURN_TURN_DEG == robot.first_sighting_heading);
        assert(inward * S5_REJOIN_PROBE_DEG == robot.heading);
        assert(1 == robot.nearby_crawls && 0 == robot.recovery_passes);
        assert(2 == robot.sightings && 2 == robot.search_stops);
        assert(1 == robot.rejoin_drives && 1 == robot.rejoin_stops);
        assert(S5_EVT_LINE_REACQUIRED == robot.outcome && 1 == robot.successes);
    }
    puts("[TEST] S5-H18 PASS  a shallow crawl toward the last sighting finds nearby tape");
}

static void test_off_centre_obstacle(void)
{
    /* A right-side cluster with a more distant return near the left edge.
       Its projected size alone would make the right detour look shorter. */
    const s5_sample_t right_scan[] = {
        {-60, 800}, {-35, 200}, {0, 345}, {15, 480}, {30, 800}, {60, 800}
    };
    enum { SCAN_COUNT = sizeof(right_scan) / sizeof(right_scan[0]) };
    s5_sample_t scan[SCAN_COUNT];
    s5_profile_t profile;
    s5_step_t plan[S5_PLAN_MAX];

    for (int32_t direction = -1; direction <= 1; direction += 2)
    {
        for (int32_t idx = 0; idx < SCAN_COUNT; idx++)
        {
            const int32_t source = (direction < 0) ? idx : SCAN_COUNT - 1 - idx;
            scan[idx].deg = (int8_t)(-direction * right_scan[source].deg);
            scan[idx].mm = right_scan[source].mm;
        }
        assert(s5_build_profile(scan, SCAN_COUNT, &profile));
        assert(direction * S5_SIDE_BIAS_DEG == profile.centre_deg);
        assert((direction < 0 ? S5_SIDE_LEFT : S5_SIDE_RIGHT) == profile.bypass_side);
        assert(7 == s5_plan_bypass(&profile, plan, S5_PLAN_MAX));
        assert(S5_MOVE_TURN == plan[1].m.kind);
        assert(-direction * S5_BYPASS_TURN_DEG == plan[1].m.angle_deg);

        scan[(direction < 0) ? SCAN_COUNT - 1 : 0].mm = 220;
        assert(s5_build_profile(scan, SCAN_COUNT, &profile));
        assert(S5_SIDE_NONE == profile.bypass_side);
        assert(2 == s5_plan_bypass(&profile, plan, S5_PLAN_MAX));
        assert(S5_MOVE_REVERSE == plan[1].m.kind);
    }
    puts("[TEST] S5-H16 PASS  mirrored scans turn away or back off when blocked");
}

static void test_search_stop_confirmation(void)
{
    vc_event_t request;
    vc_event_t finished = { .src = VC_SRC_MOTION, .type = VC_EV_MANOEUVRE_DONE,
                            .p1 = 1 };

    bridge_input = (s5_evt_t){ .type = S5_EVT_MOTION_REQ,
                               .u.motion = { .kind = S5_MOVE_STOP, .seq = 42 } };
    bridge_input_ready = true;
    assert(vc_bridge_s5_next(&request));
    assert(VC_EV_MOVE_REQUEST == request.type && VC_MOVE_STOP == request.p1);
    vc_bridge_s5_after(&request, VC_ST_AVOID_OBSTACLE, false);
    vc_bridge_s5_motion_event(&finished);
    assert(0 == bridge_replies);
    finished.type = VC_EV_MOTION_STOPPED;
    finished.p1 = 0; /* stop confirmation has no success value in p1 */
    vc_bridge_s5_motion_event(&finished);
    assert(1 == bridge_replies && S5_CMD_MOTION_DONE == bridge_reply.type);
    assert(42 == bridge_reply.seq && bridge_reply.ok);
    puts("[TEST] S5-H11 PASS  only physical stop completes the search stop request");
}

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
        const int32_t sign = (S5_SIDE_LEFT == side) ? 1 : -1;
        const int32_t lateral = s5_lateral_needed_mm(&profile, (s5_side_t)side);
        int steps = s5_plan_bypass(&profile, plan, S5_PLAN_MAX);
        assert(steps == 7 && plan[0].m.kind == S5_MOVE_STOP);
        assert(plan[1].m.angle_deg == 45 * sign);
        assert(plan[3].m.angle_deg == -45 * sign);
        assert(plan[5].m.angle_deg == -30 * sign);
        assert(plan[steps - 1].search_line);
        assert(plan[steps - 1].m.distance_mm == 2 * (lateral + 90));
        for (int i = 0; i < steps; i++)
        {
            if (plan[i].m.kind == S5_MOVE_FORWARD) { assert(plan[i].m.speed_mm_s == 60); }
        }
    }
    puts("PASS both bypass directions stop first, drive slowly and search for black line");
    puts("[TEST] S5-H13 PASS  the 30-degree return covers the offset plus 90 mm sideways");
    test_line_recovery();
    test_search_stop_confirmation();
    test_nearby_rejoin();
    test_nearby_crawl();
    test_off_centre_obstacle();
    puts("SUMMARY 15 navigation regression groups passed");
    return 0;
}
