/* S5 measured pass at task level: the production avoid task
   (task_obstacle_avoid.c run_step -> run_pass_step -> run_motion_ex) with
   the production tracker, over a simulated robot, box and ultrasonic.
   Robot dimensions are TEST FIXTURE values, not robot measurements.
   Simulated here (NOT production): the kernel calls, the S2/VC motion
   round trip (a motion completes when the simulated robot has travelled
   it), the scan task's side-look publishing (one reading per 20 ms once
   armed), the ultrasonic cone and the odometer. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define APP_H
#define VEHICLE_CONTROLLER_H
typedef int ID;
typedef int ER;
typedef int TMO;
typedef unsigned int UINT;
typedef struct { uint32_t lo; } SYSTIM;
#define E_OK 0
#define E_PAR (-1)
#define E_TMOUT (-2)
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TWF_ORW 1
#define TWF_BITCLR 2

#define S5_SIDE_PASS 1
#define S5_SERVO_AHEAD_MM 70     /* fixture */
#define S5_SERVO_ARM_MM 60       /* fixture */
#define S5_REAR_OVERHANG_MM 100  /* fixture */
#define S5_FRONT_OVERHANG_MM 100 /* fixture */
#define S5_PIVOT_MARGIN_MM 30    /* fixture */
#define S5_SIDE_LOOK_DEG 60
#include "../../../app_program/subsystem5/s5_local.h"

ID g_mbf_s5_cmd = 1, g_mbf_s5_evt = 2, g_flg_s5 = 3;

#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); exit(1); } } while (0)

/* ---- simulated world ---- */
typedef struct { int32_t depth, gap; } box_t;
static struct
{
    uint32_t now_ms;
    int32_t  axle_x;       /* mm along the line; box front face at x = 0 */
    uint32_t odo_mm;
    box_t    box;
    s5_motion_req_t move;
    bool     moving, have_move;
    int32_t  remaining_mm;
    int32_t  motion_reqs, stops, turns;
    int32_t  forward_reqs_after_abort;
    bool     side_active, side_armed;
    int32_t  side_deg_seen, side_ends;
    uint32_t side_seq;
    uint8_t  side_status;
    uint16_t side_mm;
    uint32_t next_pub_ms;
    int32_t  abort_at_travel, abort_type; /* script: inject at this pass travel, -1 = never */
    int32_t  pass_start_odo;
    bool     aborted;
    bool     inject_stale;
    uint16_t stale_seq;
    bool     force_fault;
    bool     no_echo_world;
    char     log[8][160];
    int      logn;
    uint8_t  outcome, outcome_code;
} w;

static int32_t iabs(int32_t v) { return v < 0 ? -v : v; }
static ER tk_clr_flg(ID id, UINT m) { (void)id; (void)m; return E_OK; }
static ER tk_set_flg(ID id, UINT m) { (void)id; (void)m; return E_OK; }
static ER tk_wai_flg(ID id, UINT b, UINT mode, UINT *o, TMO t) { (void)id; (void)b; (void)mode; (void)t; *o = 0; return E_OK; }
static int32_t tk_get_tid(void) { return 1; }
static void tk_get_tim(SYSTIM *n) { n->lo = w.now_ms; }
static void tk_dly_tsk(TMO d) { w.now_ms += (uint32_t)d; }
static void app_check(ER e, const char *n) { (void)n; assert(e == E_OK); }
static void tm_printf(const uint8_t *fmt, ...)
{
    if (w.logn < 8) { snprintf(w.log[w.logn++], 160, "%s", (const char *)fmt); }
}
void s5_set_state(s5_state_t s) { (void)s; }
void s5_set_fault(s5_fault_code_t f) { (void)f; }
void s5_count_avoid_ok(void) {}
void s5_load_profile(s5_profile_t *p) { *p = (s5_profile_t){ 0 }; }
ER s5_get_status(s5_status_t *s) { *s = (s5_status_t){ 0 }; return E_OK; }
void s5_port_set_line_search(bool b) { (void)b; }
bool s5_port_line_detected(void) { return false; }
bool s5_port_line_both_black(void) { return false; }
uint32_t s5_port_odo_mm(void) { return w.odo_mm; }

void s5_side_request(int32_t deg) { w.side_active = true; w.side_armed = false; w.side_deg_seen = deg; w.side_seq = 0; w.next_pub_ms = w.now_ms; }
void s5_side_end(void) { w.side_active = false; w.side_armed = false; w.side_ends++; }
int32_t s5_side_deg(void) { return w.side_deg_seen; }
void s5_side_arm(void) { w.side_armed = true; }
void s5_side_publish(uint8_t st, uint16_t mm) { (void)st; (void)mm; }
bool s5_side_read(s5_side_reading_t *r)
{
    *r = (s5_side_reading_t){ w.side_seq, w.side_status, w.side_mm };
    return w.side_armed;
}

static uint16_t sim_range(void)
{
    if (w.no_echo_world) { return 0; }
    s5_geom_t g; s5_geom_from_config(&g);
    int32_t xs = w.axle_x + s5_sensor_along_mm(&g, iabs(w.side_deg_seen)), best = 0;
    int32_t face = w.box.gap - s5_sensor_lateral_mm(&g, iabs(w.side_deg_seen));
    for (int d = -15; d <= 15; d += 5)
    {
        int32_t deg = iabs(w.side_deg_seen) + d;
        if (deg > 89) { deg = 89; }
        int32_t sn = s5_sin1000(deg), cs = s5_sin1000(90 - deg);
        int32_t xh = xs + (face * cs) / sn;
        if (xh >= 0 && xh <= w.box.depth)
        {
            int32_t r = (face * 1000) / sn;
            if (best == 0 || r < best) { best = r; }
        }
    }
    return (uint16_t)best;
}

ER s5_send_evt(s5_evt_t *e)
{
    if (e->type == S5_EVT_MOTION_REQ)
    {
        w.motion_reqs++;
        if (w.aborted && e->u.motion.kind != S5_MOVE_STOP) { w.forward_reqs_after_abort++; }
        w.move = e->u.motion; w.have_move = true; w.moving = true;
        w.remaining_mm = e->u.motion.distance_mm;
        if (e->u.motion.kind == S5_MOVE_STOP) { w.stops++; }
        if (e->u.motion.kind == S5_MOVE_TURN) { w.turns++; }
    }
    else { w.outcome = e->type; w.outcome_code = e->code; }
    return E_OK;
}
ER s5_send_critical_to(ID id, s5_evt_t *e) { (void)id; return s5_send_evt(e); }
ER tk_snd_mbf(ID id, const void *m, int32_t s, TMO t) { (void)id; (void)m; (void)s; (void)t; return E_OK; }

#include "../../../app_program/subsystem5/obstacle_logic.c"

/* side-look scan task stand-in: one reading per 20 ms once armed */
static void side_pump(void)
{
    if (w.side_active && !w.side_armed) { w.side_armed = true; }
    while (w.side_active && w.side_armed && w.next_pub_ms <= w.now_ms)
    {
        uint16_t mm = sim_range();
        w.side_seq++;
        w.side_status = w.force_fault ? S5_SIDE_ST_FAULT : (mm ? S5_SIDE_ST_ECHO : S5_SIDE_ST_NO_ECHO);
        w.side_mm = mm;
        w.next_pub_ms += S5_PERIOD_MS;
    }
}

static int32_t tk_rcv_mbf(ID id, void *out, TMO timeout)
{
    s5_cmd_t *cmd = (s5_cmd_t *)out;
    assert(id == g_mbf_s5_cmd);
    if (w.inject_stale) { w.inject_stale = false; *cmd = (s5_cmd_t){ S5_CMD_MOTION_DONE, 1, w.stale_seq }; return sizeof(*cmd); }
    if (timeout > S5_PERIOD_MS || timeout < 0) { timeout = S5_PERIOD_MS * 50; }
    if (w.have_move && w.moving)
    {
        if (w.move.kind == S5_MOVE_FORWARD)
        {
            int32_t step = (w.move.speed_mm_s * timeout) / 1000;
            if (step < 1) { step = 1; }
            if (step > w.remaining_mm) { step = w.remaining_mm; }
            w.axle_x += step; w.odo_mm += (uint32_t)step; w.remaining_mm -= step;
            w.now_ms += (uint32_t)((step * 1000) / (w.move.speed_mm_s ? w.move.speed_mm_s : 1));
            side_pump();
            if (w.abort_type >= 0 && !w.aborted &&
                (int32_t)(w.odo_mm - (uint32_t)w.pass_start_odo) >= w.abort_at_travel)
            {
                w.aborted = true;
                *cmd = (s5_cmd_t){ (uint8_t)w.abort_type, 0, 0 };
                w.moving = false;
                return sizeof(*cmd);
            }
            if (w.remaining_mm > 0) { return E_TMOUT; }
        }
        else if (w.move.kind == S5_MOVE_TURN) { w.now_ms += 50; }
        w.moving = false;
        *cmd = (s5_cmd_t){ S5_CMD_MOTION_DONE, 1, w.move.seq };
        return sizeof(*cmd);
    }
    w.now_ms += (uint32_t)timeout;
    side_pump();
    return E_TMOUT;
}

#include "../../../app_program/subsystem5/task_obstacle_avoid.c"

#define CHECK_END do {} while (0)

static bool body_hits(const box_t *b, int32_t ax, int32_t phi, int32_t drive)
{
    int32_t sn = s5_sin1000(phi), cs = s5_sin1000(90 - phi);
    for (int32_t dist = 0; dist <= drive; dist += 5)
        for (int32_t lx = -S5_REAR_OVERHANG_MM; lx <= S5_FRONT_OVERHANG_MM; lx += 5)
            for (int s = -1; s <= 1; s += 2)
            {
                int32_t lu = s * S5_ROBOT_HALF_WIDTH_MM;
                int32_t x = (lx * cs - lu * sn) / 1000 + (dist * cs) / 1000;
                int32_t u = (lx * sn + lu * cs) / 1000 + (dist * sn) / 1000;
                if ((ax + x) >= 0 && (ax + x) <= b->depth && u >= b->gap) { return true; }
            }
    return false;
}

static s5_step_t pass_step(s5_side_t side)
{
    s5_profile_t prof = { .distance_mm = 200, .valid = 1, .left_mm = 100, .right_mm = 100,
                          .bypass_side = (uint8_t)side };
    s5_step_t plan[S5_PLAN_MAX];
    int n = s5_plan_bypass(&prof, plan, S5_PLAN_MAX);
    CHECK(n == 9 && plan[6].side_track == 1);
    return plan[6];
}

static void world(int32_t depth, int32_t gap, int32_t start)
{
    memset(&w, 0, sizeof(w));
    w.box = (box_t){ depth, gap };
    w.axle_x = start;
    w.odo_mm = 5000;
    w.abort_type = -1;
    w.pass_start_odo = 5000;
    g_seq_next = 100;
}

static step_result_t run(const s5_step_t *st, int32_t *stop_x) 
{
    step_result_t r = run_step(st);
    *stop_x = w.axle_x;
    return r;
}

int main(void)
{
    int32_t stop_x;
    for (int side = S5_SIDE_LEFT; side <= S5_SIDE_RIGHT; side++)
    {
        s5_step_t st = pass_step((s5_side_t)side);
        for (int32_t depth = 50; depth <= 400; depth += 50)
        {
            world(depth, 100, -400);
            g_heading_deg = 0;
            step_result_t r = run(&st, &stop_x);
            CHECK(r == STEP_DONE && g_pass_why == S5_PASS_CLEARED);
            /* mirrored sensor direction: pass left -> look right (negative) */
            CHECK((side == S5_SIDE_LEFT) ? (w.side_deg_seen < 0) : (w.side_deg_seen > 0));
            CHECK(w.stops == 1 && w.side_ends == 1 && !w.side_active);
            CHECK(stop_x - S5_REAR_OVERHANG_MM >= depth);
            for (int32_t phi = 0; phi <= S5_RETURN_TURN_DEG; phi++) { CHECK(!body_hits(&w.box, stop_x, phi, 0)); }
            CHECK(!body_hits(&w.box, stop_x, S5_RETURN_TURN_DEG, 600));
        }
    }
    puts("[TEST] S5-M05 PASS  task: pass leg ends by measurement, stops the car, releases the side look, mirrored look direction, no body contact");

    /* shorter box -> earlier turn-in */
    s5_step_t st = pass_step(S5_SIDE_LEFT);
    int32_t prev = -100000;
    for (int32_t depth = 50; depth <= 400; depth += 50)
    {
        world(depth, 100, -400);
        run(&st, &stop_x);
        CHECK(stop_x > prev); prev = stop_x;
    }
    puts("[TEST] S5-M06 PASS  task: a shorter obstacle gives an earlier turn-in");

    /* never acquired: fixed-depth fallback only, logged as assumed */
    world(0, 100, -400); w.no_echo_world = true;
    step_result_t r = run(&st, &stop_x);
    CHECK(r == STEP_DONE && g_pass_why == S5_PASS_FALLBACK);
    CHECK(iabs((stop_x + 400) - st.m.distance_mm) <= 5 && w.stops == 1 && w.side_ends == 1);
    {
        bool found = false;
        for (int i = 0; i < w.logn; i++) { found |= (strstr(w.log[i], "depth assumed") != NULL); }
        CHECK(found);
    }
    puts("[TEST] S5-M07 PASS  task: no side echo -> fixed distance from the pass start, logs 'depth assumed'");

    /* never cleared -> stop at the cap, reported as BLOCKED, no turn-in */
    world(3000, 100, -400);
    r = run(&st, &stop_x);
    CHECK(r == STEP_PASS_BLOCKED && g_pass_why == S5_PASS_CAP && w.stops == 1 && w.turns == 0);
    CHECK(stop_x + 400 <= st.cap_mm + 10);
    report_result(r);
    CHECK(w.outcome == S5_EVT_AVOID_FAILED && w.outcome_code == S5_FAULT_BLOCKED);
    /* insufficient gap */
    world(300, S5_ROBOT_HALF_WIDTH_MM, -400);
    r = run(&st, &stop_x);
    CHECK(r == STEP_PASS_BLOCKED && g_pass_why == S5_PASS_TOO_CLOSE && w.stops == 1);
    report_result(r);
    CHECK(w.outcome == S5_EVT_AVOID_FAILED && w.outcome_code == S5_FAULT_BLOCKED);
    /* sensor failure */
    world(300, 100, -400); w.force_fault = true;
    r = run(&st, &stop_x);
    CHECK(r == STEP_PASS_BLOCKED && g_pass_why == S5_PASS_SENSOR_FAULT && w.stops == 1);
    report_result(r);
    CHECK(w.outcome == S5_EVT_FAULT && w.outcome_code == S5_FAULT_ECHO_TIMEOUT);
    puts("[TEST] S5-M08 PASS  task: cap, low gap and sensor failure stop the car and report with existing codes");

    /* abort / disable while seeking, alongside and during the tail drive */
    static const int32_t at[] = { 100, 450, 650 }; /* travel into the pass: seeking / alongside / tail */
    for (int kind = 0; kind < 2; kind++)
    {
        for (unsigned i = 0; i < 3; i++)
        {
            world(300, 100, -400);
            w.abort_type = kind ? S5_CMD_DISABLE : S5_CMD_ABORT;
            w.abort_at_travel = at[i];
            r = run(&st, &stop_x);
            CHECK(r == STEP_ABORT);
            CHECK(w.side_ends == 1 && !w.side_active);
            CHECK(w.stops == 0 && w.forward_reqs_after_abort == 0); /* VC owns the stop; S5 sends nothing more */
            CHECK(w.turns == 0);
        }
    }
    puts("[TEST] S5-M09 PASS  task: ABORT and DISABLE while seeking, alongside and in the tail drive end the pass, cancel the side look, request no further motion");

    /* a stale MOTION_DONE from the stopped pass leg cannot finish the next motion */
    world(300, 100, -400);
    r = run(&st, &stop_x);
    CHECK(r == STEP_DONE);
    uint16_t old_seq = (uint16_t)(g_seq_next - 3);
    w.inject_stale = true; w.stale_seq = old_seq;
    s5_step_t turn = { .m = { .kind = S5_MOVE_TURN, .angle_deg = -45 } };
    int32_t turns_before = w.turns, heading = g_heading_deg;
    r = run_step(&turn);
    CHECK(!w.inject_stale); CHECK(r == STEP_DONE && w.turns == turns_before + 1 && g_heading_deg == heading - 45);
    puts("[TEST] S5-M10 PASS  task: a stale MOTION_DONE is ignored; the next turn completes only on its own sequence");
    puts("SUMMARY side-pass task: all groups passed");
    return 0;
}
