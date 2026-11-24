/* S5 obstacle size report (s5_obst_report.h), end to end inside S5:
   the production avoid task (task_obstacle_avoid.c avoid_obstacle ->
   get_profile -> bypass -> run_observe_step) with the production tracker
   and report code, over a simulated robot, box and ultrasonic.

   Simulated here (NOT production): the kernel calls, the S2/VC motion
   round trip, the scan task (profile handed over at once; side look armed
   S5_TEST_ARM_MS after the request, then one reading per 20 ms), the
   ultrasonic cone (rays at look angle -15..+15 deg in 5 deg steps, nearest
   hit on the box's near side face) and the odometer.  The line is never
   found, so every run ends in the bounded recovery; that does not matter
   for the size report.

   Build twice (Makefile): S5_LENGTH_OBSERVE=1 runs the checks, and both
   builds print the motion trace of the same encounters with "trace"; the
   Makefile fails if the two traces differ (bypass motion unchanged). */
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

#include "../../../app_program/subsystem5/s5_local.h"

ID g_mbf_s5_cmd = 1, g_mbf_s5_evt = 2, g_flg_s5 = 3;

#define FAR_MM 470U /* test geometry, see R03 */
#define ABORT_INTO_PASS_MM 400U /* outbound legs are 80 + 206 mm here: 400 is inside the pass leg */
#define S5_TEST_ARM_MS 300 /* servo ramp + settle before the first side reading */

static int g_fails, g_checks;
#define CHECK(id, c, desc) do { g_checks++; if (!(c)) { g_fails++; printf("[TEST] %s FAIL  %s (line %d: %s)\n", id, desc, __LINE__, #c); } } while (0)

/* ---- simulated world ---- */
static struct
{
    uint32_t now_ms;
    int32_t  x10, u10;      /* robot axle, 0.1 mm; box front face at x = 0, box centred on u = 0 */
    int32_t  heading;       /* deg, + = left */
    uint32_t odo_mm;
    int32_t  odo_frac10;
    int32_t  box_len, box_width;
    s5_profile_t prof;
    s5_motion_req_t move;
    bool     moving, have_move;
    int32_t  remaining10;
    bool     side_active, side_armed;
    int32_t  side_deg;
    uint32_t side_seq, arm_at_ms, next_pub_ms;
    uint8_t  side_status;
    uint16_t side_mm;
    bool     no_echo, force_fault;
    int32_t  drop_every;    /* every Nth side reading has no echo (0 = never) */
    uint32_t abort_at_odo;  /* send ABORT once the odometer reaches this (0 = never);
                               odometer based, so both builds abort at the same place */
    bool     aborted;
    int32_t  reqs_after_abort;
    /* motion trace */
    char     trace[64][48];
    int      ntrace;
    /* reports as stored */
    s5_obst_report_t hist[16];
    int      nhist;
    char     usb[16][200];
    int      nusb;
} w;

static ER tk_clr_flg(ID id, UINT m) { (void)id; (void)m; return E_OK; }
static ER tk_set_flg(ID id, UINT m) { (void)id; (void)m; return E_OK; }
static ER tk_wai_flg(ID id, UINT b, UINT mode, UINT *o, TMO t) { (void)id; (void)b; (void)mode; (void)t; *o = 0; return E_OK; }
static int32_t tk_get_tid(void) { return 1; }
static void tk_get_tim(SYSTIM *n) { n->lo = w.now_ms; }
static void side_pump(void);
static void tk_dly_tsk(TMO d) { w.now_ms += (uint32_t)d; side_pump(); }
static void app_check(ER e, const char *n) { (void)n; assert(e == E_OK); }
#include <stdarg.h>
static void tm_printf(const uint8_t *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    if (0 == strncmp((const char *)fmt, "s5: obstacle #", 14) && w.nusb < 16)
    {
        vsnprintf(w.usb[w.nusb++], 200, (const char *)fmt, ap);
    }
    va_end(ap);
}
void s5_set_state(s5_state_t s) { (void)s; }
void s5_set_fault(s5_fault_code_t f) { (void)f; }
void s5_count_avoid_ok(void) {}
void s5_load_profile(s5_profile_t *p) { *p = w.prof; }
ER s5_get_status(s5_status_t *s) { *s = (s5_status_t){ 0 }; return E_OK; }
void s5_port_set_line_search(bool b) { (void)b; }
bool s5_port_line_detected(void) { return false; }
bool s5_port_line_both_black(void) { return false; }
uint32_t s5_port_odo_mm(void) { return w.odo_mm; }

void s5_side_request(int32_t deg)
{
    w.side_active = true; w.side_armed = false; w.side_deg = deg; w.side_seq = 0;
    w.arm_at_ms = w.now_ms + S5_TEST_ARM_MS; w.next_pub_ms = w.arm_at_ms;
}
void s5_side_end(void) { w.side_active = false; w.side_armed = false; }
bool s5_side_read(s5_side_reading_t *r)
{
    *r = (s5_side_reading_t){ w.side_seq, w.side_status, w.side_mm };
    return w.side_armed && (0U != w.side_seq);
}

#include "../../../app_program/subsystem5/obstacle_logic.c"
#include "../../../app_program/subsystem5/s5_obst_report.c"

void s5_report_store(const s5_obst_report_t *p)
{
    if (w.nhist < 16) { w.hist[w.nhist++] = *p; }
}

static int32_t iabs(int32_t v) { return v < 0 ? -v : v; }

/* Range on the obstacle side; only while driving straight (heading 0),
   the box's near side face at u = +-width/2 */
static uint16_t sim_range(void)
{
    s5_geom_t g;
    int32_t   look = iabs(w.side_deg), best = 0;
    int32_t   sgn  = (w.side_deg < 0) ? -1 : 1; /* + = looking left */

    if (w.no_echo || (0 != w.heading)) { return 0; }
#if S5_LENGTH_OBSERVE || S5_SIDE_PASS
    s5_geom_from_config(&g);
#else
    (void)g; return 0; /* no side look in this build */
#endif
    int32_t xs = w.x10 / 10 + s5_sensor_along_mm(&g, look);
    int32_t us = w.u10 / 10 + sgn * s5_sensor_lateral_mm(&g, look);
    int32_t face = (sgn < 0) ? (us - w.box_width / 2) : (-w.box_width / 2 - us); /* distance to the face */
    if (face <= 0) { return 0; }
    for (int d = -15; d <= 15; d += 5)
    {
        int32_t deg = look + d;
        if (deg > 89) { deg = 89; }
        if (deg < 1) { continue; }
        int32_t sn = s5_sin1000(deg), cs = s5_sin1000(90 - deg);
        int32_t xh = xs + (face * cs) / sn;
        if (xh >= 0 && xh <= w.box_len)
        {
            int32_t r = (face * 1000) / sn;
            if (best == 0 || r < best) { best = r; }
        }
    }
    return (uint16_t)best;
}

static void side_pump(void)
{
    if (w.side_active && !w.side_armed && w.now_ms >= w.arm_at_ms) { w.side_armed = true; }
    while (w.side_active && w.side_armed && w.next_pub_ms <= w.now_ms)
    {
        uint16_t mm = sim_range();
        w.side_seq++;
        if (w.drop_every > 0 && (w.side_seq % (uint32_t)w.drop_every) == 0U) { mm = 0; }
        w.side_status = w.force_fault ? S5_SIDE_ST_FAULT : (mm ? S5_SIDE_ST_ECHO : S5_SIDE_ST_NO_ECHO);
        w.side_mm = mm;
        w.next_pub_ms += S5_PERIOD_MS;
    }
}

ER s5_send_evt(s5_evt_t *e)
{
    if (e->type == S5_EVT_MOTION_REQ)
    {
        if (w.aborted) { w.reqs_after_abort++; }
        if (w.ntrace < 64)
        {
            snprintf(w.trace[w.ntrace++], 48, "k%d a%d d%d v%d", e->u.motion.kind,
                     e->u.motion.angle_deg, e->u.motion.distance_mm, e->u.motion.speed_mm_s);
        }
        w.move = e->u.motion; w.have_move = true; w.moving = true;
        w.remaining10 = 10 * (int32_t)e->u.motion.distance_mm;
        if (e->u.motion.kind == S5_MOVE_TURN) { w.heading += e->u.motion.angle_deg; }
    }
    return E_OK;
}
ER s5_send_critical_to(ID id, s5_evt_t *e) { (void)id; return s5_send_evt(e); }
ER tk_snd_mbf(ID id, const void *m, int32_t s, TMO t) { (void)id; (void)m; (void)s; (void)t; return E_OK; }

/* One simulated period (<= S5_PERIOD_MS): returns true with *cmd filled
   when a command arrives in it. */
static bool sim_period(s5_cmd_t *cmd, TMO dt)
{
    if (!w.aborted && (0U != w.abort_at_odo) && (w.odo_mm >= w.abort_at_odo))
    {
        w.aborted = true; w.moving = false;
        *cmd = (s5_cmd_t){ S5_CMD_ABORT, 0, 0 };
        return true;
    }
    if (w.have_move && w.moving)
    {
        if ((w.move.kind == S5_MOVE_FORWARD) || (w.move.kind == S5_MOVE_REVERSE))
        {
            int32_t step10 = (w.move.speed_mm_s * dt) / 100;
            int32_t dir = (w.move.kind == S5_MOVE_FORWARD) ? 1 : -1;
            if (step10 < 1) { step10 = 1; }
            if (step10 > w.remaining10) { step10 = w.remaining10; }
            w.x10 += dir * (step10 * s5_sin1000(90 - w.heading)) / 1000;
            w.u10 += dir * (step10 * s5_sin1000(w.heading)) / 1000;
            if (dir > 0)
            {
                w.odo_frac10 += step10;
                w.odo_mm += (uint32_t)(w.odo_frac10 / 10);
                w.odo_frac10 %= 10;
            }
            w.remaining10 -= step10;
            w.now_ms += (uint32_t)dt;
            side_pump();
            if (w.remaining10 > 0) { return false; }
        }
        else if (w.move.kind == S5_MOVE_TURN) { w.now_ms += 200; side_pump(); }
        w.moving = false;
        *cmd = (s5_cmd_t){ S5_CMD_MOTION_DONE, 1, w.move.seq };
        return true;
    }
    w.now_ms += (uint32_t)dt;
    side_pump();
    return false;
}

/* Waits up to timeout in simulated periods, like the kernel call. */
static int32_t tk_rcv_mbf(ID id, void *out, TMO timeout)
{
    s5_cmd_t *cmd = (s5_cmd_t *)out;
    TMO waited = 0;
    assert(id == g_mbf_s5_cmd);
    if (timeout < 0) { timeout = 600000; }
    do
    {
        TMO dt = (timeout - waited > S5_PERIOD_MS) ? S5_PERIOD_MS : (timeout - waited);
        if (dt < 1) { dt = 1; }
        if (sim_period(cmd, dt)) { return sizeof(*cmd); }
        waited += dt;
    } while (waited < timeout);
    return E_TMOUT;
}

#include "../../../app_program/subsystem5/task_obstacle_avoid.c"

/* One encounter.  Box width/length in mm; the robot stops dist_mm before
   the box front face, centred on the line. */
static void encounter(int32_t len, int32_t width, int32_t dist_mm)
{
    s5_obst_report_t keep_last = w.nhist ? w.hist[w.nhist - 1] : (s5_obst_report_t){ 0 };
    uint32_t t = w.now_ms;
    (void)keep_last;
    memset(&w, 0, sizeof(w));
    w.now_ms = t + 1000;
    w.box_len = len; w.box_width = width;
    w.x10 = -10 * dist_mm; w.odo_mm = 7000;
    w.prof = (s5_profile_t){ .distance_mm = (uint16_t)dist_mm, .valid = 1,
                             .left_edge_deg = 20, .right_edge_deg = -20, .centre_deg = 0,
                             .left_mm = (int16_t)(width / 2), .right_mm = (int16_t)(-width / 2),
                             .width_mm = (uint16_t)width, .clear_left_mm = 900, .clear_right_mm = 300,
                             .bypass_side = (uint8_t)S5_SIDE_LEFT };
}

static void run_encounter(void) { avoid_obstacle(); }

static const s5_obst_report_t *last_rec(void) { return w.nhist ? &w.hist[w.nhist - 1] : NULL; }
static const s5_obst_report_t *stage_rec(uint8_t stage)
{
    for (int i = 0; i < w.nhist; i++) { if (w.hist[i].stage == stage) { return &w.hist[i]; } }
    return NULL;
}

static void print_trace(const char *tag)
{
    for (int i = 0; i < w.ntrace; i++) { printf("%s %s\n", tag, w.trace[i]); }
}

static void codec_tests(void)
{
    s5_obst_report_t r, d;
    uint8_t f[S5_OR_BYTES];
    static const uint8_t golden[S5_OR_BYTES] = { 0x01, 0x01, 0x2c, 0x02, 0x02, 0x01, 0x02, 0x00,
        0x00, 0xd2, 0x00, 0xb4, 0x00, 0xf0, 0x00, 0x01, 0xe2, 0x40, 0x01, 0x00 };

    s5_or_begin(&r, 300, 123456U);
    s5_or_front(&r, 1, 210, -20, 20, 180, 1, -60, 60, true, S5_OR_R_NONE);
    s5_or_length(&r, S5_OR_L_MEASURED, 240, S5_OR_R_NONE);
    s5_or_encode(&r, f);
    CHECK("S5-R01", 0 == memcmp(f, golden, sizeof(f)),
          "frame bytes match the documented layout (id 300 rev 2 stage length, front 210, width 180, length 240, t0 123456)");
    CHECK("S5-R01", s5_or_decode(f, S5_OR_BYTES, &d) && 0 == memcmp(&d, &r, sizeof(d)),
          "decode(encode(x)) == x");
    {
        uint8_t bad[S5_OR_BYTES];
        bool all_rejected = !s5_or_decode(f, S5_OR_BYTES - 1, &d);
        int idx[] = { 0, 4, 5, 6, 7, 18, 19 };
        for (unsigned i = 0; i < sizeof(idx) / sizeof(idx[0]); i++)
        {
            memcpy(bad, f, sizeof(bad)); bad[idx[i]] = 0xEE;
            all_rejected = all_rejected && !s5_or_decode(bad, S5_OR_BYTES, &d);
        }
        memcpy(bad, f, sizeof(bad)); bad[1] = 0; bad[2] = 0;      /* id 0 */
        all_rejected = all_rejected && !s5_or_decode(bad, S5_OR_BYTES, &d);
        memcpy(bad, f, sizeof(bad)); bad[12] = 0x7F;               /* length > 2000 */
        all_rejected = all_rejected && !s5_or_decode(bad, S5_OR_BYTES, &d);
        CHECK("S5-R01", all_rejected, "short frame, bad version, out-of-range stage/status/reason/side/length, id 0 and a non-zero reserved byte are all rejected");
    }
    CHECK("S5-R01", 1U == s5_or_next_id(65535U) && 2U == s5_or_next_id(1U), "profile id wraps 65535 -> 1, never 0");

    s5_or_begin(&r, 7, 1U);
    s5_or_front(&r, 1, 200, -60, 20, 250, 1, -60, 60, true, S5_OR_R_NONE);
    CHECK("S5-R09", S5_OR_W_LOWER_BOUND == r.width_status && 250U == r.width_mm,
          "an edge at the scan limit reports the width as a lower bound, not a precise width");
    s5_or_begin(&r, 8, 1U);
    s5_or_front(&r, 1, 200, 10, 10, 0, 1, -60, 60, true, S5_OR_R_NONE);
    CHECK("S5-R09", S5_OR_W_ONE_BEAM == r.width_status && 0U == r.width_mm,
          "one beam only: edges not located, no width claimed");
    s5_or_begin(&r, 9, 1U);
    s5_or_front(&r, 0, 0, 0, 0, 0, 0, -60, 60, true, S5_OR_R_NONE);
    CHECK("S5-R09", S5_OR_W_NONE == r.width_status && S5_OR_L_UNAVAILABLE == r.length_status &&
          S5_OR_R_NO_BYPASS == r.length_reason, "no obstacle in the scan: no width, length unavailable (no bypass)");
}

int main(int argc, char **argv)
{
    bool trace = (argc > 1) && (0 == strcmp(argv[1], "trace"));
    const int32_t lens[] = { 150, 240 };

    if (trace)
    {
        /* same encounters in both builds; only the motion is printed */
        for (unsigned i = 0; i < 2; i++) { encounter(lens[i], 180, 200); run_encounter(); print_trace(i ? "B" : "A"); }
        encounter(240, 180, 200); w.abort_at_odo = w.odo_mm + ABORT_INTO_PASS_MM; run_encounter(); print_trace("C");
        return 0;
    }
    codec_tests();
#if !S5_LENGTH_OBSERVE
    (void)last_rec; (void)stage_rec;
    printf("[TEST] obstacle report: codec checks only in the S5_LENGTH_OBSERVE 0 build\n");
    return g_fails ? 1 : 0;
#else

    /* R02/R03: front profile then measured length, two lengths.  TEST
       GEOMETRY: stopped FAR_MM from the box so the front edge is still ahead
       of the side look when the straight leg starts.  At the real stop
       distance (about S5_DETECT_MM) the front edge is already passed during
       the outbound diagonal and the length is PARTIAL (R04). */
    for (unsigned i = 0; i < 2; i++)
    {
        encounter(lens[i], 180, FAR_MM);
        run_encounter();
        const s5_obst_report_t *f = stage_rec(S5_OR_STAGE_FRONT), *l = stage_rec(S5_OR_STAGE_LENGTH), *z = last_rec();
        CHECK("S5-R02", f && S5_OR_W_MEASURED == f->width_status && 180U == f->width_mm && FAR_MM == f->front_mm &&
              S5_OR_L_MEASURING == f->length_status && 0U == f->length_mm,
              "front stage: width 180 mm from the profile (front 200 mm kept separate), length measuring");
        CHECK("S5-R02", w.nhist >= 1 && S5_OR_STAGE_SCANNING == w.hist[0].stage && 0U == w.hist[0].width_mm,
              "the encounter opens with a SCANNING record holding no sizes");
        CHECK("S5-R03", l && S5_OR_L_MEASURED == l->length_status && iabs((int32_t)l->length_mm - lens[i]) <= 60,
              "length measured during the pass, within 60 mm of the simulated box (beam spread, inferred)");
        if (l) { printf("        box %d mm -> reported %d mm\n", (int)lens[i], (int)l->length_mm); }
        CHECK("S5-R03", z && S5_OR_STAGE_FINAL == z->stage && l && z->length_mm == l->length_mm &&
              S5_OR_L_MEASURED == z->length_status, "the final record keeps the measured length");
        {
            uint8_t fr[S5_OR_BYTES]; char hex[41]; s5_or_encode(z, fr);
            for (int k = 0; k < S5_OR_BYTES; k++) { snprintf(hex + 2 * k, 3, "%02x", fr[k]); }
            CHECK("S5-R12", w.nusb >= 1 && NULL != strstr(w.usb[w.nusb - 1], hex),
                  "the USB line of the final record carries the exact MQTT frame bytes");
        }
    }

    /* R04 partial: real stop distance, the sensor is already beside the
       box when the straight leg starts */
    encounter(240, 180, 200);
    run_encounter();
    {
        const s5_obst_report_t *l = stage_rec(S5_OR_STAGE_LENGTH);
        CHECK("S5-R04", l && S5_OR_L_PARTIAL == l->length_status && l->length_mm > 0U && l->length_mm <= 240U + 30U,
              "already beside the box when the side look started: PARTIAL, a lower bound, never a full measured length");
        if (l) { printf("        box 240 mm stopped at 200 mm -> partial %d mm\n", (int)l->length_mm); }
    }

    /* R05 missing echoes */
    encounter(240, 180, 200); w.no_echo = true; run_encounter();
    CHECK("S5-R05", last_rec() && S5_OR_L_UNAVAILABLE == last_rec()->length_status &&
          S5_OR_R_NO_SIDE_ECHO == last_rec()->length_reason && 0U == last_rec()->length_mm,
          "no side echo at all: length unavailable (no side echo), no number");
    encounter(240, 180, FAR_MM); w.drop_every = 4; run_encounter();
    {
        const s5_obst_report_t *l = stage_rec(S5_OR_STAGE_LENGTH);
        CHECK("S5-R05", l && S5_OR_L_MEASURED == l->length_status && iabs((int32_t)l->length_mm - 240) <= 60,
              "one missing echo in four does not end the measurement");
    }
    encounter(240, 180, 200); w.force_fault = true; run_encounter();
    CHECK("S5-R05", last_rec() && S5_OR_R_SENSOR_FAULT == last_rec()->length_reason,
          "repeated failed side readings: unavailable (sensor fault)");
    encounter(900, 180, 200); run_encounter();
    CHECK("S5-R10", last_rec() && S5_OR_L_UNAVAILABLE == last_rec()->length_status &&
          S5_OR_R_REAR_NOT_SEEN == last_rec()->length_reason,
          "box longer than the fixed pass leg: rear not seen, unavailable (the assumed depth is not reported)");

    /* R06 abort during the pass */
    encounter(240, 180, 200); w.abort_at_odo = w.odo_mm + ABORT_INTO_PASS_MM; run_encounter();
    CHECK("S5-R06", last_rec() && S5_OR_STAGE_FINAL == last_rec()->stage &&
          S5_OR_L_UNAVAILABLE == last_rec()->length_status && S5_OR_R_ABORTED == last_rec()->length_reason &&
          0 == w.reqs_after_abort, "ABORT during the pass: final record says aborted, no further motion requested");

    /* R07 two consecutive obstacles */
    encounter(150, 180, FAR_MM); run_encounter();
    s5_obst_report_t first = *last_rec();
    encounter(300, 120, FAR_MM + 10); run_encounter();
    CHECK("S5-R07", w.hist[0].profile_id == (uint16_t)(first.profile_id + 1U) && S5_OR_STAGE_SCANNING == w.hist[0].stage &&
          0U == w.hist[0].length_mm && 0U == w.hist[0].width_mm && S5_OR_L_PENDING == w.hist[0].length_status,
          "obstacle #2 opens a new id and the previous length is cleared at once");
    {
        const s5_obst_report_t *f = stage_rec(S5_OR_STAGE_FRONT), *l = stage_rec(S5_OR_STAGE_LENGTH);
        CHECK("S5-R07", f && 120U == f->width_mm && S5_OR_L_MEASURING == f->length_status && 0U == f->length_mm &&
              l && iabs((int32_t)l->length_mm - 300) <= 60 && first.length_mm != l->length_mm,
              "obstacle #2 reports its own width and length, never #1's");
        CHECK("S5-R07", f && f->t0_ms != first.t0_ms, "each encounter carries its own start time (restart and merge guard)");
    }

    printf("[TEST] obstacle report SUMMARY %d checks, %d failed\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
#endif
}
