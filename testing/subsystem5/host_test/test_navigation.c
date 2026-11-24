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
volatile int16_t g_s3_bc_speed_now; /* barcode read record, written by vc_bridge_s2.c */
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
static void app_check(ER err, const char *name); /* used by vc_bridge_s2.c, defined below */
/* S2 stop-generation state read by vc_bridge_s2.c */
volatile uint32_t g_s2_imu_stop;
volatile uint32_t g_s2_odo_ticks; /* S2 odometer, sent with MANOEUVRE_DONE / MOTION_STOPPED */
/* No IMU interlock in these tests: every S2 safe stop is unexplained */
uint32_t s2_imu_stop_cause(uint32_t stop_epoch) { (void)stop_epoch; return g_s2_imu_stop; }
uint32_t s2_stop_epoch(void) { return 0u; }
#include "../../../app_program/vehicle_controller/vc_bridge_s2.c"
#include "../../../app_program/subsystem5/obstacle_logic.c"
#include "../../../app_program/subsystem5/s5_obst_report.c"
/* Obstacle report and the report-only side look (S5_LENGTH_OBSERVE).  This
   suite checks that the bypass motion is unchanged with it on; the side
   look here never produces a reading, as on a robot whose sensor sees
   nothing sideways. */
static s5_obst_report_t g_nav_report;
static int32_t g_nav_side_requests, g_nav_side_ends;
void s5_report_store(const s5_obst_report_t *p) { g_nav_report = *p; }
void s5_side_request(int32_t deg) { (void)deg; g_nav_side_requests++; }
void s5_side_end(void) { g_nav_side_ends++; }
bool s5_side_read(s5_side_reading_t *r) { *r = (s5_side_reading_t){ 0 }; return false; }
uint32_t s5_port_odo_mm(void) { return 0U; }

typedef enum
{
    TAPE_ABSENT,
    TAPE_ON_INITIAL_RETURN,
    TAPE_ON_RETURN_RETRY,
    TAPE_LOST_DURING_BRAKING,
    TAPE_ON_SHORT_SEARCH,
    TAPE_AT_SHORT_ENDPOINT,
    TAPE_AT_FINAL_ENDPOINT,
    TAPE_ON_STRAIGHT_RECOVERY,
    TAPE_ON_LEFT_PROBE,
    TAPE_ON_RIGHT_PROBE,
    TAPE_LOST_AFTER_ALIGNMENT,
    TAPE_LOST_DURING_REJOIN,
    TAPE_ON_NEARBY_CRAWL,
    TAPE_DOUBLE_BLACK_ON_PROBE,
    TAPE_DOUBLE_BLACK_LOST_AFTER_TURN
} tape_scenario_t;

static struct
{
    uint32_t now_ms;
    uint32_t last_move_completed_ms;
    s5_state_t state;
    s5_motion_req_t move;
    uint8_t last_completed_move;
    bool waiting_for_motion;
    tape_scenario_t tape;
    bool black;
    bool double_black_seen;
    bool ir_search_enabled;
    uint32_t ir_started_ms;
    int32_t heading;
    int32_t first_sighting_heading;
    int32_t initial_search_heading;
    int32_t return_retries;
    int32_t short_searches;
    int32_t extra_searches;
    bool initial_miss_logged;
    bool braking_loss_logged;
    bool alignment_loss_logged;
    int32_t recovery_turns;
    int32_t recovery_turn_deg;
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

static bool double_black_scenario(void)
{
    return (TAPE_DOUBLE_BLACK_ON_PROBE == robot.tape) ||
           (TAPE_DOUBLE_BLACK_LOST_AFTER_TURN == robot.tape);
}

static bool tape_appears_during_search(void)
{
    if ((TAPE_ABSENT == robot.tape) || (TAPE_AT_SHORT_ENDPOINT == robot.tape) ||
        (TAPE_AT_FINAL_ENDPOINT == robot.tape) ||
        (S5_MOVE_FORWARD != robot.move.kind))
    {
        return false;
    }
    if (0 == robot.sightings)
    {
        if (TAPE_ON_RETURN_RETRY == robot.tape)
        {
            return 85 == robot.move.distance_mm;
        }
        if (TAPE_ON_SHORT_SEARCH == robot.tape)
        {
            return 30 == robot.move.distance_mm;
        }
        return S5_ST_SEARCH_LINE == robot.state;
    }
    return (1 == robot.sightings) &&
           (((TAPE_ON_NEARBY_CRAWL == robot.tape) &&
             (S5_REJOIN_SEARCH_MM == robot.move.distance_mm)) ||
            ((TAPE_ON_STRAIGHT_RECOVERY == robot.tape) &&
             (S5_ST_LINE_RECOVERY == robot.state) && (175 == robot.move.distance_mm)));
}

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
    if (tape_appears_during_search())
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
        if (TAPE_LOST_DURING_BRAKING == robot.tape)
        {
            robot.black = false;
        }
        if (robot.rejoin_drives > robot.rejoin_stops)
        {
            assert(robot.ir_search_enabled);
            robot.rejoin_stops++;
        }
        else if (robot.ir_search_enabled)
        {
            robot.search_stops++;
        }
        else if ((robot.sightings > 0) && (S5_MOVE_FORWARD != robot.last_completed_move))
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
                (((TAPE_ON_INITIAL_RETURN == robot.tape) ||
                  (TAPE_ON_RETURN_RETRY == robot.tape)) && (0 == robot.heading)) ||
                ((TAPE_ON_LEFT_PROBE == robot.tape) &&
                 (S5_REJOIN_PROBE_DEG == robot.heading)) ||
                ((TAPE_ON_RIGHT_PROBE == robot.tape) &&
                 (-S5_REJOIN_PROBE_DEG == robot.heading)) ||
                ((TAPE_LOST_DURING_REJOIN == robot.tape) &&
                 (0 == robot.heading) && (0 == robot.rejoin_drives));
            if (double_black_scenario())
            {
                robot.black = robot.double_black_seen
                    ? ((TAPE_DOUBLE_BLACK_ON_PROBE == robot.tape) && (0 == robot.heading))
                    : ((S5_REJOIN_PROBE_DEG == robot.heading) ||
                       (-S5_REJOIN_PROBE_DEG == robot.heading));
            }
        }
    }
    if ((S5_MOVE_FORWARD == robot.move.kind) &&
        (S5_REJOIN_CHECK_MM == robot.move.distance_mm) &&
        (TAPE_LOST_DURING_REJOIN == robot.tape))
    {
        robot.black = false;
    }
    robot.now_ms++;
    robot.last_move_completed_ms = robot.now_ms;
    robot.last_completed_move = robot.move.kind;
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
static void tk_dly_tsk(TMO delay)
{
    const bool tape_at_endpoint =
        ((TAPE_AT_SHORT_ENDPOINT == robot.tape) && (30 == robot.move.distance_mm)) ||
        ((TAPE_AT_FINAL_ENDPOINT == robot.tape) && (35 == robot.move.distance_mm));

    robot.now_ms += (uint32_t)delay;
    if (tape_at_endpoint && (0 == robot.sightings) && (S5_MOVE_FORWARD == robot.move.kind) &&
        !robot.waiting_for_motion && (delay >= S5_PERIOD_MS))
    {
        assert(robot.ir_search_enabled && !robot.black);
        robot.sightings++;
        robot.black = true;
    }
}
static int32_t tk_get_tid(void) { return 1; }
static ER tk_set_flg(ID id, UINT bits) { (void)id; (void)bits; return E_OK; }
static void tm_printf(const uint8_t *fmt, ...)
{
    const char *message = (const char *)fmt;

    robot.initial_miss_logged |= (S5_ST_SEARCH_LINE == robot.state) &&
        (strstr(message, "search ended without seeing the line") != NULL);
    robot.braking_loss_logged |= strstr(message, "line lost during braking") != NULL;
    robot.alignment_loss_logged |= strstr(message, "line lost during alignment") != NULL;
}
static void app_check(ER err, const char *name) { (void)name; assert(E_OK == err); }

void s5_set_state(s5_state_t state) { robot.state = state; }
void s5_set_fault(s5_fault_code_t fault) { robot.fault = fault; }
void s5_count_avoid_ok(void) { robot.successes++; }
void s5_load_profile(s5_profile_t *profile) { *profile = (s5_profile_t){ 0 }; }
ER s5_get_status(s5_status_t *status) { *status = (s5_status_t){ 0 }; return E_OK; }
void s5_port_set_line_search(bool b_active)
{
    robot.ir_search_enabled = b_active;
    if (b_active)
    {
        robot.ir_started_ms = robot.now_ms;
    }
}

bool s5_port_line_detected(void)
{
    assert(robot.ir_search_enabled);
    if (((TAPE_AT_SHORT_ENDPOINT == robot.tape) || (TAPE_AT_FINAL_ENDPOINT == robot.tape)) &&
        robot.black && (S5_MOVE_STOP == robot.last_completed_move))
    {
        assert(robot.now_ms - robot.last_move_completed_ms >= S5_PERIOD_MS);
    }
    return robot.black;
}

bool s5_port_line_both_black(void)
{
    assert(robot.ir_search_enabled);
    if (double_black_scenario() && robot.black)
    {
        robot.double_black_seen = true;
        return true;
    }
    return false;
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
        if (S5_ST_LINE_RECOVERY == robot.state)
        {
            assert(S5_MOVE_REVERSE != move->kind);
            if (S5_MOVE_TURN == move->kind)
            {
                robot.recovery_turns++;
                robot.recovery_turn_deg = move->angle_deg;
            }
        }
        if ((S5_MOVE_FORWARD == move->kind) && (S5_REJOIN_CHECK_MM == move->distance_mm))
        {
            assert(robot.ir_search_enabled && robot.black);
            assert(S5_SEARCH_SPEED_MM_S == move->speed_mm_s);
            assert(!double_black_scenario() || (0 == robot.heading));
            robot.rejoin_drives++;
        }
        else if ((S5_MOVE_FORWARD == move->kind) &&
                 (S5_REJOIN_SEARCH_MM == move->distance_mm))
        {
            assert(robot.ir_search_enabled && !robot.black);
            assert(S5_SEARCH_SPEED_MM_S == move->speed_mm_s);
            robot.nearby_crawls++;
        }
        else if ((S5_MOVE_FORWARD == move->kind) && (S5_ST_BYPASSING != robot.state) &&
                 ((85 == move->distance_mm) || (30 == move->distance_mm) ||
                  (35 == move->distance_mm)))
        {
            assert(S5_MOVE_STOP == robot.last_completed_move);
            assert(robot.ir_search_enabled && !robot.black);
            assert(robot.now_ms - robot.ir_started_ms >= S5_PERIOD_MS);
            assert(S5_SEARCH_SPEED_MM_S == move->speed_mm_s);
            if (85 == move->distance_mm)
            {
                assert(S5_ST_SEARCH_LINE == robot.state);
                assert(robot.initial_search_heading == robot.heading);
                assert(0 == robot.sightings && 0 == robot.recovery_passes);
                robot.return_retries++;
            }
            else
            {
                assert(S5_ST_LINE_RECOVERY == robot.state);
                assert(1 == robot.recovery_passes && 0 == robot.heading);
                assert(0 == robot.extra_searches);
                if (30 == move->distance_mm)
                {
                    assert(0 == robot.short_searches);
                    robot.short_searches++;
                }
                else
                {
                    assert(1 == robot.short_searches);
                    robot.extra_searches++;
                }
            }
        }
        else if ((S5_ST_LINE_RECOVERY == robot.state) && (S5_MOVE_FORWARD == move->kind))
        {
            assert(0 == robot.recovery_passes && 0 == robot.heading);
            assert(175 == move->distance_mm && 60 == move->speed_mm_s);
            assert(robot.ir_search_enabled);
            robot.recovery_passes++;
        }
        else if ((S5_ST_SEARCH_LINE == robot.state) && (S5_MOVE_FORWARD == move->kind))
        {
            robot.initial_search_heading = robot.heading;
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
        const int32_t sign = (S5_SIDE_LEFT == side) ? 1 : -1;

        simulate_bypass((s5_side_t)side, TAPE_ABSENT);
        assert(1 == robot.recovery_passes && 0 == robot.successes);
        assert(1 == robot.recovery_turns && S5_RETURN_TURN_DEG * sign == robot.recovery_turn_deg);
        assert(1 == robot.return_retries && 1 == robot.short_searches);
        assert(1 == robot.extra_searches);
        assert(S5_EVT_AVOID_FAILED == robot.outcome && robot.critical_outcome);
        assert(S5_FAULT_LINE_NOT_FOUND == robot.fault);
    }
    puts("[TEST] S5-H09 PASS  missing tape triggers one return-angle pivot and straight recovery");

    for (int32_t side = S5_SIDE_LEFT; side <= S5_SIDE_RIGHT; side++)
    {
        simulate_bypass((s5_side_t)side, TAPE_ON_STRAIGHT_RECOVERY);
        assert(1 == robot.recovery_passes && 2 == robot.sightings);
        assert(0 == robot.return_retries && 0 == robot.recovery_turns);
        assert(2 == robot.search_stops); /* each sighting stopped the driving leg */
        assert(S5_EVT_LINE_REACQUIRED == robot.outcome && 1 == robot.successes);
        assert(0 == robot.heading && robot.black);
    }
    puts("[TEST] S5-H10 PASS  straight recovery verifies tape after the first alignment misses");
    puts("[TEST] S5-H12 PASS  IR is requested for line checks and released before pivots and handoff");
}

static void test_short_searches(void)
{
    for (int32_t side = S5_SIDE_LEFT; side <= S5_SIDE_RIGHT; side++)
    {
        const int32_t sign = (S5_SIDE_LEFT == side) ? 1 : -1;

        simulate_bypass((s5_side_t)side, TAPE_ON_INITIAL_RETURN);
        assert(0 == robot.return_retries && 0 == robot.short_searches && 0 == robot.extra_searches);
        assert(0 == robot.recovery_passes && 1 == robot.rejoin_stops);
        assert(!robot.initial_miss_logged);
        assert(S5_EVT_LINE_REACQUIRED == robot.outcome && 1 == robot.successes);

        simulate_bypass((s5_side_t)side, TAPE_ON_RETURN_RETRY);
        assert(1 == robot.return_retries && 0 == robot.short_searches && 0 == robot.extra_searches);
        assert(0 == robot.recovery_passes && 1 == robot.rejoin_stops);
        assert(robot.initial_miss_logged);
        assert(S5_EVT_LINE_REACQUIRED == robot.outcome && 1 == robot.successes);

        simulate_bypass((s5_side_t)side, TAPE_LOST_DURING_BRAKING);
        assert(0 == robot.return_retries && 1 == robot.recovery_passes);
        assert(1 == robot.recovery_turns && S5_RETURN_TURN_DEG * sign == robot.recovery_turn_deg);
        assert(robot.braking_loss_logged && !robot.alignment_loss_logged);
        assert(!robot.initial_miss_logged);
        assert(0 == robot.rejoin_stops && 1 == robot.short_searches && 1 == robot.extra_searches);
        assert(S5_EVT_AVOID_FAILED == robot.outcome && S5_FAULT_LINE_NOT_FOUND == robot.fault);

        simulate_bypass((s5_side_t)side, TAPE_LOST_AFTER_ALIGNMENT);
        assert(0 == robot.return_retries && 1 == robot.recovery_passes);
        assert(0 == robot.recovery_turns);
        assert(robot.alignment_loss_logged && !robot.braking_loss_logged);
        assert(!robot.initial_miss_logged);
        assert(S5_EVT_AVOID_FAILED == robot.outcome && S5_FAULT_LINE_NOT_FOUND == robot.fault);

        simulate_bypass((s5_side_t)side, TAPE_ON_SHORT_SEARCH);
        assert(1 == robot.return_retries && 1 == robot.short_searches && 0 == robot.extra_searches);
        assert(1 == robot.recovery_passes && 1 == robot.rejoin_stops);
        assert(1 == robot.recovery_turns && S5_RETURN_TURN_DEG * sign == robot.recovery_turn_deg);
        assert(S5_EVT_LINE_REACQUIRED == robot.outcome && 1 == robot.successes);

        simulate_bypass((s5_side_t)side, TAPE_AT_SHORT_ENDPOINT);
        assert(1 == robot.return_retries && 1 == robot.short_searches && 0 == robot.extra_searches);
        assert(1 == robot.recovery_passes && 1 == robot.sightings);
        assert(1 == robot.recovery_turns && S5_RETURN_TURN_DEG * sign == robot.recovery_turn_deg);
        assert(1 == robot.search_stops && 1 == robot.rejoin_stops);
        assert(S5_EVT_LINE_REACQUIRED == robot.outcome && 1 == robot.successes);

        simulate_bypass((s5_side_t)side, TAPE_AT_FINAL_ENDPOINT);
        assert(1 == robot.return_retries && 1 == robot.short_searches && 1 == robot.extra_searches);
        assert(1 == robot.recovery_passes && 1 == robot.sightings);
        assert(1 == robot.recovery_turns && S5_RETURN_TURN_DEG * sign == robot.recovery_turn_deg);
        assert(1 == robot.search_stops && 1 == robot.rejoin_stops);
        assert(S5_EVT_LINE_REACQUIRED == robot.outcome && 1 == robot.successes);
    }
    puts("[TEST] S5-H25 PASS  85 mm same-heading search runs only after an initial miss");
    puts("[TEST] S5-H26 PASS  braking and alignment losses skip the return retry");
    puts("[TEST] S5-H27 PASS  30 mm sighting skips the 35 mm search, and both can confirm endpoint tape");
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
    assert(1 == robot.recovery_passes && 0 == robot.successes);
    assert(S5_EVT_AVOID_FAILED == robot.outcome && robot.critical_outcome);
    assert(S5_FAULT_LINE_NOT_FOUND == robot.fault);
    puts("[TEST] S5-H15 PASS  a missed local check falls back to bounded recovery and stop");

    simulate_bypass(S5_SIDE_LEFT, TAPE_LOST_DURING_REJOIN);
    assert(1 == robot.rejoin_drives && 1 == robot.rejoin_stops);
    assert(1 == robot.recovery_passes && 0 == robot.successes);
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

static void test_double_black_rejoin(void)
{
    for (int32_t side = S5_SIDE_LEFT; side <= S5_SIDE_RIGHT; side++)
    {
        simulate_bypass((s5_side_t)side, TAPE_DOUBLE_BLACK_ON_PROBE);
        assert(robot.double_black_seen && 0 == robot.heading);
        assert(1 == robot.rejoin_drives && 1 == robot.rejoin_stops);
        assert(0 == robot.recovery_passes && S5_EVT_LINE_REACQUIRED == robot.outcome);

        simulate_bypass((s5_side_t)side, TAPE_DOUBLE_BLACK_LOST_AFTER_TURN);
        assert(robot.double_black_seen && 0 == robot.rejoin_drives);
        assert(1 == robot.recovery_passes && 0 == robot.successes);
        assert(S5_EVT_AVOID_FAILED == robot.outcome && robot.critical_outcome);
    }
    puts("[TEST] S5-H20 PASS  double black returns to approach heading and requires confirmation");
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
        assert(9 == s5_plan_bypass(&profile, plan, S5_PLAN_MAX));
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
    assert(n > 0 && angles[0] == -45 && angles[n - 1] == 45);
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
        assert(steps == 9 && plan[0].m.kind == S5_MOVE_STOP);
        assert(plan[1].m.angle_deg == 35 * sign);
        assert(plan[2].m.kind == S5_MOVE_FORWARD && plan[2].m.distance_mm == 80);
        assert(plan[3].m.angle_deg == 10 * sign);
        assert(plan[4].m.kind == S5_MOVE_FORWARD);
        assert(plan[5].m.angle_deg == -45 * sign);
        assert(plan[7].m.angle_deg == -S5_RETURN_TURN_DEG * sign);
        const int32_t side_x1000 = plan[2].m.distance_mm * s5_sin1000(35) +
                                   plan[4].m.distance_mm * s5_sin1000(45);
        const int32_t forward_mm = (plan[2].m.distance_mm * s5_sin1000(55) +
                                    plan[4].m.distance_mm * s5_sin1000(45)) / 1000;
        assert(side_x1000 >= lateral * 1000 && side_x1000 < (lateral + 2) * 1000);
        assert(plan[6].m.kind == S5_MOVE_FORWARD);
        assert(forward_mm + plan[6].m.distance_mm ==
               profile.distance_mm + S5_OBSTACLE_DEPTH_MM + S5_ROBOT_LENGTH_MM);
        const s5_step_t * search = &plan[steps - 1];
        const int32_t inward_x1000 = search->m.distance_mm * s5_sin1000(S5_RETURN_TURN_DEG);
        const int32_t needed_x1000 = (lateral + S5_SEARCH_MARGIN_MM) * 1000;

        assert(search->search_line);
        assert(inward_x1000 >= needed_x1000);
        assert(inward_x1000 < needed_x1000 + s5_sin1000(S5_RETURN_TURN_DEG));
        for (int i = 0; i < steps; i++)
        {
            if (plan[i].m.kind == S5_MOVE_FORWARD) { assert(plan[i].m.speed_mm_s == 60); }
        }
    }
    puts("PASS both bypass directions stop first, drive slowly and search for black line");
    puts("[TEST] S5-H19 PASS  35-degree first turn and short drive retain projected clearance");
    puts("[TEST] S5-H13 PASS  the return turn covers the offset plus the search margin sideways");
    test_line_recovery();
    test_short_searches();
    test_search_stop_confirmation();
    test_nearby_rejoin();
    test_nearby_crawl();
    test_double_black_rejoin();
    test_off_centre_obstacle();
    puts("SUMMARY 20 navigation regression groups passed");
    return 0;
}
