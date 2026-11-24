/* S5 measured pass: the production tracker (obstacle_logic.c) driven by
   (a) hand-made reading sequences and (b) a geometric simulation of a box
   beside the robot.  The robot dimensions below are TEST FIXTURE values,
   not measurements of the real robot.  What is only simulated here: the
   ultrasonic cone (5 rays, +-15 deg), the odometer (exact) and the pivot. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define APP_H
typedef int ID;
typedef int ER;

#define S5_SIDE_PASS 1
#define S5_US_TO_AXLE_MM 100     /* fixture */
#define S5_REAR_OVERHANG_MM 100  /* fixture */
#define S5_FRONT_OVERHANG_MM 100 /* fixture */
#define S5_PIVOT_MARGIN_MM 30    /* fixture */
#ifndef S5_SIDE_LOOK_DEG
#define S5_SIDE_LOOK_DEG 60
#endif
#include "../../../app_program/subsystem5/obstacle_logic.c"

static s5_geom_t g_geom;
static int checks;
#define CHECK(c) do { checks++; if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); exit(1); } } while (0)

static s5_side_reading_t rd(uint32_t seq, uint8_t st, uint16_t mm)
{
    return (s5_side_reading_t){ seq, st, mm };
}

/* feed n identical readings, 2 mm travel each, returns last phase */
static s5_pass_phase_t feed(s5_pass_t *p, int32_t *travel, uint32_t *seq, int n,
                            uint8_t st, uint16_t mm)
{
    s5_pass_phase_t ph = (s5_pass_phase_t)p->phase;
    for (int i = 0; i < n; i++)
    {
        *travel += 2;
        (*seq)++;
        s5_side_reading_t r = rd(*seq, st, mm);
        ph = s5_pass_update(p, *travel, &r);
    }
    return ph;
}

static void new_pass(s5_pass_t *p, int32_t fallback, int32_t cap)
{
    s5_pass_init(p, &g_geom, S5_SIDE_LOOK_DEG, fallback, cap);
}

static void test_tracker(void)
{
    s5_pass_t p; int32_t t; uint32_t q;
    const uint16_t good = 104; /* 100 mm gap / sin75 (nearest ray of the cone) */

    /* distinct fresh samples: needs FOUND_COUNT of them */
    new_pass(&p, 500, 1000); t = 0; q = 0;
    CHECK(feed(&p, &t, &q, 2, S5_SIDE_ST_ECHO, good) == S5_PASS_SEEKING);
    CHECK(feed(&p, &t, &q, 1, S5_SIDE_ST_ECHO, good) == S5_PASS_ALONGSIDE);
    CHECK(p.first_mm == 2); /* acquired on the first reading: alongside from the start */

    /* duplicate and stale samples are not counted twice */
    new_pass(&p, 500, 1000);
    s5_side_reading_t r = rd(1, S5_SIDE_ST_ECHO, good);
    for (int i = 0; i < 10; i++) { s5_pass_update(&p, 10 + i, &r); }
    CHECK(p.phase == S5_PASS_SEEKING && p.hits == 1);
    r = rd(5, S5_SIDE_ST_ECHO, good); s5_pass_update(&p, 30, &r);
    r = rd(3, S5_SIDE_ST_ECHO, good); s5_pass_update(&p, 32, &r); /* older */
    CHECK(p.hits == 2);
    CHECK(s5_pass_update(&p, 34, NULL) == S5_PASS_SEEKING && p.hits == 2);

    /* a silent ping or an out-of-window echo breaks the acquire run */
    new_pass(&p, 500, 1000); t = 0; q = 0;
    feed(&p, &t, &q, 2, S5_SIDE_ST_ECHO, good);
    feed(&p, &t, &q, 1, S5_SIDE_ST_NO_ECHO, 0);
    CHECK(p.hits == 0);
    feed(&p, &t, &q, 2, S5_SIDE_ST_ECHO, good);
    feed(&p, &t, &q, 1, S5_SIDE_ST_ECHO, 1500); /* valid but far */
    CHECK(p.hits == 0 && p.phase == S5_PASS_SEEKING);
    /* a sensor fault is neither a hit nor a miss */
    feed(&p, &t, &q, 2, S5_SIDE_ST_ECHO, good);
    feed(&p, &t, &q, 1, S5_SIDE_ST_FAULT, 0);
    CHECK(p.hits == 2);
    CHECK(feed(&p, &t, &q, 1, S5_SIDE_ST_ECHO, good) == S5_PASS_ALONGSIDE);

    /* one-ping dropout while alongside must not declare the rear */
    feed(&p, &t, &q, 5, S5_SIDE_ST_ECHO, good);
    CHECK(feed(&p, &t, &q, 1, S5_SIDE_ST_NO_ECHO, 0) == S5_PASS_ALONGSIDE);
    CHECK(feed(&p, &t, &q, 3, S5_SIDE_ST_ECHO, good) == S5_PASS_ALONGSIDE);
    CHECK(p.misses == 0);
    /* three silent pings are not enough without the travel */
    CHECK(feed(&p, &t, &q, 3, S5_SIDE_ST_NO_ECHO, 0) == S5_PASS_ALONGSIDE);
    /* ... but silence over S5_SIDE_LOST_MM is */
    CHECK(feed(&p, &t, &q, 10, S5_SIDE_ST_NO_ECHO, 0) == S5_PASS_PASSED);
    CHECK(p.rear_mm == p.last_mm + 100 + (104 * 500) / 1000);
    CHECK(p.target_mm > p.rear_mm && p.decided_mm > p.last_mm && p.partial == 1);
    /* PASSED -> CLEARED exactly at the target, once */
    s5_side_reading_t none = rd(0, 0, 0);
    CHECK(s5_pass_update(&p, p.target_mm - 1, &none) == S5_PASS_PASSED);
    CHECK(s5_pass_update(&p, p.target_mm, &none) == S5_PASS_CLEARED);
    CHECK(s5_pass_update(&p, p.target_mm + 500, &none) == S5_PASS_CLEARED);

    /* never acquired: fixed-depth fallback at exactly fallback_mm */
    new_pass(&p, 400, 1000); t = 0; q = 0;
    CHECK(feed(&p, &t, &q, 100, S5_SIDE_ST_NO_ECHO, 0) == S5_PASS_SEEKING); /* t=200 */
    CHECK(s5_pass_update(&p, 399, NULL) == S5_PASS_SEEKING);
    CHECK(s5_pass_update(&p, 400, NULL) == S5_PASS_FALLBACK);

    /* never cleared: cap boundary */
    new_pass(&p, 300, 600); t = 0; q = 0;
    feed(&p, &t, &q, 3, S5_SIDE_ST_ECHO, good);
    CHECK(s5_pass_update(&p, 599, NULL) == S5_PASS_ALONGSIDE);
    CHECK(s5_pass_update(&p, 600, NULL) == S5_PASS_CAP);
    /* cap is never below the fallback distance */
    new_pass(&p, 700, 600); CHECK(p.cap_mm == 700);

    /* insufficient gap: closer than half-width + guard */
    new_pass(&p, 500, 1000); t = 0; q = 0;
    uint16_t close = (uint16_t)(((S5_ROBOT_HALF_WIDTH_MM + S5_SIDE_GUARD_MM - 5) * 1000) / s5_sin1000(S5_SIDE_LOOK_DEG + S5_SIDE_BEAM_HALF_DEG));
    CHECK(feed(&p, &t, &q, 3, S5_SIDE_ST_ECHO, close) == S5_PASS_TOO_CLOSE);
    new_pass(&p, 500, 1000); t = 0; q = 0;
    feed(&p, &t, &q, 6, S5_SIDE_ST_ECHO, good);
    CHECK(p.phase == S5_PASS_ALONGSIDE);
    CHECK(feed(&p, &t, &q, 1, S5_SIDE_ST_ECHO, close) == S5_PASS_TOO_CLOSE);
    CHECK(p.min_gap_mm < S5_ROBOT_HALF_WIDTH_MM + S5_SIDE_GUARD_MM);

    /* repeated sensor failures end the pass, one failure does not */
    new_pass(&p, 500, 1000); t = 0; q = 0;
    CHECK(feed(&p, &t, &q, S5_SIDE_FAULT_COUNT - 1, S5_SIDE_ST_FAULT, 0) == S5_PASS_SEEKING);
    feed(&p, &t, &q, 1, S5_SIDE_ST_NO_ECHO, 0);
    CHECK(feed(&p, &t, &q, S5_SIDE_FAULT_COUNT - 1, S5_SIDE_ST_FAULT, 0) == S5_PASS_SEEKING);
    CHECK(feed(&p, &t, &q, 1, S5_SIDE_ST_FAULT, 0) == S5_PASS_SENSOR_FAULT);
    puts("[TEST] S5-M01 PASS  tracker: distinct samples, duplicates, stale, dropout, fallback, cap, gap, sensor fault");
}

/* ---- geometric simulation ------------------------------------------ */
typedef struct { int32_t depth, gap, start; } box_t;

static int32_t iabs(int32_t v) { return v < 0 ? -v : v; }

/* nearest range (mm) over a +-15 deg cone, 0 = no echo.  Box face is the
   line u = gap, x in [0, depth]; sensor at (xs, 0) looks at angle deg
   from forward towards +u. */
static uint16_t sim_range(const box_t *b, int32_t axle_x)
{
    int32_t xs = axle_x + g_geom.us_to_axle_mm;
    int32_t best = 0;
    for (int d = -15; d <= 15; d += 5)
    {
        int32_t deg = S5_SIDE_LOOK_DEG + d;
        if (deg > 89) { deg = 89; }
        int32_t sn = s5_sin1000(deg), cs = s5_sin1000(90 - deg);
        /* ray hits u = gap at x = xs + gap * cos/sin */
        int32_t xh = xs + (b->gap * cs) / sn;
        if (xh >= 0 && xh <= b->depth)
        {
            int32_t r = (b->gap * 1000) / sn;
            if (best == 0 || r < best) { best = r; }
        }
    }
    return (uint16_t)best;
}

static bool inside_box(const box_t *b, int32_t x, int32_t u)
{
    return (x >= 0) && (x <= b->depth) && (u >= b->gap);
}

/* body corner sampling after a pivot of phi deg about the axle at ax,
   then a straight drive of 600 mm at that heading */
static bool body_hits(const box_t *b, int32_t ax, int32_t phi, int32_t drive)
{
    int32_t sn = s5_sin1000(phi), cs = s5_sin1000(90 - phi);
    for (int32_t dist = 0; dist <= drive; dist += 5)
    {
        for (int32_t lx = -g_geom.rear_mm; lx <= g_geom.front_mm; lx += 5)
        {
            for (int s = -1; s <= 1; s += 2)
            {
                int32_t lu = s * g_geom.half_width_mm;
                int32_t x = (lx * cs - lu * sn) / 1000 + (dist * cs) / 1000;
                int32_t u = (lx * sn + lu * cs) / 1000 + (dist * sn) / 1000;
                if (inside_box(b, ax + x, u)) { return true; }
            }
        }
        for (int32_t lu = -g_geom.half_width_mm; lu <= g_geom.half_width_mm; lu += 5)
        {
            for (int e = 0; e < 2; e++)
            {
                int32_t lx = e ? g_geom.front_mm : -g_geom.rear_mm;
                int32_t x = (lx * cs - lu * sn) / 1000 + (dist * cs) / 1000;
                int32_t u = (lx * sn + lu * cs) / 1000 + (dist * sn) / 1000;
                if (inside_box(b, ax + x, u)) { return true; }
            }
        }
    }
    return false;
}

typedef struct { s5_pass_phase_t phase; int32_t axle_at_turn, depth, rear; bool partial; } run_t;

static run_t sim_pass(const box_t *b, int32_t fallback, int32_t cap)
{
    s5_pass_t p;
    int32_t request = cap + s5_pass_tail_max_mm(&g_geom, S5_SIDE_LOOK_DEG);
    uint32_t seq = 0;
    int32_t travel = 0;
    run_t out = { 0 };

    s5_pass_init(&p, &g_geom, S5_SIDE_LOOK_DEG, fallback, cap);
    for (; travel <= request; travel += 2)
    {
        uint16_t mm = sim_range(b, b->start + travel);
        s5_side_reading_t r = rd(++seq, mm ? S5_SIDE_ST_ECHO : S5_SIDE_ST_NO_ECHO, mm);
        s5_pass_phase_t ph = s5_pass_update(&p, travel, &r);
        if (ph != S5_PASS_SEEKING && ph != S5_PASS_ALONGSIDE && ph != S5_PASS_PASSED)
        {
            out.phase = ph; out.axle_at_turn = b->start + travel;
            out.depth = p.depth_mm; out.rear = p.rear_mm; out.partial = p.partial;
            return out;
        }
    }
    out.phase = (s5_pass_phase_t)p.phase; /* motion ended first */
    out.axle_at_turn = b->start + travel;
    return out;
}

static void test_simulation(void)
{
    int runs = 0, partials = 0, fallbacks = 0;
    int32_t prev_turn_by_depth = -100000;
    /* planned gap = hw + margin + beam over-read at the scan edges */
    static const int32_t over[] = { 0, 15, 30, 45 };
    static const int32_t widths[] = { 80, 150, 300 };
    static const int32_t offsets[] = { 0, 40 };

    for (int32_t depth = 50; depth <= 400; depth += 25)
    {
        for (unsigned oi = 0; oi < 4; oi++)
        {
            for (unsigned wi = 0; wi < 3; wi++)
            {
                for (unsigned fi = 0; fi < 2; fi++)
                {
                    /* the profile edge on the chosen side: half width + offset + over-read */
                    int32_t edge = widths[wi] / 2 + offsets[fi] + over[oi];
                    int32_t lateral = edge + S5_ROBOT_HALF_WIDTH_MM + S5_SIDE_MARGIN_MM;
                    int32_t true_edge = widths[wi] / 2 + offsets[fi];
                    box_t b = { depth, lateral - true_edge, -400 };
                    for (int alongside = 0; alongside < 2; alongside++)
                    {
                        b.start = alongside ? 40 : -400;
                        run_t r = sim_pass(&b, 500, 1000);
                        runs++;
                        partials += r.partial ? 1 : 0;
                        /* bounded exit, clean finish */
                        /* starting level with a short box: the forward-looking
                           sensor may never see it -> fixed-depth fallback */
                        CHECK((r.phase == S5_PASS_CLEARED) ||
                              ((r.phase == S5_PASS_FALLBACK) && alongside));
                        fallbacks += (r.phase == S5_PASS_FALLBACK) ? 1 : 0;
                        /* rear of the body is past the rear face at turn-in */
                        CHECK(r.axle_at_turn - g_geom.rear_mm >= depth);
                        /* no body contact during the pivot or the diagonal drive */
                        CHECK(!body_hits(&b, r.axle_at_turn, 0, 0));
                        for (int32_t phi = 1; phi <= S5_RETURN_TURN_DEG; phi++)
                        {
                            CHECK(!body_hits(&b, r.axle_at_turn, phi, 0));
                        }
                        CHECK(!body_hits(&b, r.axle_at_turn, S5_RETURN_TURN_DEG, 700));
                        /* observed depth never exceeds truth by more than the beam reach (when fully seen) */
                        if (!r.partial && r.phase == S5_PASS_CLEARED)
                        {
                            CHECK(r.depth >= depth - 60 && r.depth <= depth + 120);
                        }
                        if (oi == 0 && wi == 0 && fi == 0 && !alongside)
                        {
                            /* a shorter box allows an earlier turn-in */
                            CHECK(r.axle_at_turn > prev_turn_by_depth);
                            prev_turn_by_depth = r.axle_at_turn;
                        }
                    }
                }
            }
        }
    }
    CHECK(partials > 0);
    printf("[TEST] S5-M02 PASS  simulated box 50..400 mm deep, widths 80..300, centred+offset, beam over-read 0..45 mm, %d runs (%d fixed-depth fallbacks): zero contact in pivot and diagonal\n", runs, fallbacks);

    /* a box that is longer than the cap: stop, never turn in */
    box_t big = { 1500, 100, -400 };
    run_t r = sim_pass(&big, 500, 1000);
    CHECK(r.phase == S5_PASS_CAP);
    /* no echo at all: fixed-depth fallback, flagged as such */
    box_t none = { 0, 100000, -400 };
    r = sim_pass(&none, 500, 1000);
    CHECK(r.phase == S5_PASS_FALLBACK && r.axle_at_turn == -400 + 500);
    /* box too close to the body */
    box_t tight = { 300, S5_ROBOT_HALF_WIDTH_MM, -400 };
    r = sim_pass(&tight, 500, 1000);
    CHECK(r.phase == S5_PASS_TOO_CLOSE);
    puts("[TEST] S5-M03 PASS  too-long box -> CAP, no echo -> FALLBACK at the fixed distance, tight gap -> TOO_CLOSE");
}

static void test_plan_and_geometry(void)
{
    s5_profile_t prof = { .distance_mm = 200, .valid = 1, .left_mm = 100,
                          .right_mm = (uint16_t)(-100) };
    s5_step_t plan[S5_PLAN_MAX];
    prof.right_mm = 100; /* unsigned edge on the right */
    for (int side = S5_SIDE_LEFT; side <= S5_SIDE_RIGHT; side++)
    {
        prof.bypass_side = (uint8_t)side;
        int n = s5_plan_bypass(&prof, plan, S5_PLAN_MAX);
        CHECK(n == 9);
        const s5_step_t *pass = &plan[6];
        CHECK(pass->m.kind == S5_MOVE_FORWARD && pass->side_track == 1);
        /* pass on the left looks right (negative), and the reverse */
        CHECK((side == S5_SIDE_LEFT) ? (pass->look_deg < 0) : (pass->look_deg > 0));
        CHECK(iabs(pass->look_deg) == S5_SIDE_LOOK_DEG);
        CHECK(pass->m.distance_mm >= 50); /* fixed-depth distance kept as the fallback */
        CHECK(pass->cap_mm >= pass->m.distance_mm && pass->request_mm > pass->cap_mm);
        for (int i = 0; i < n; i++) { if (i != 6) { CHECK(plan[i].side_track == 0); } }
    }
    /* swept pivot: the front corner nearest the face never needs more than the body rule here */
    CHECK(s5_pivot_min_axle_mm(&g_geom, S5_ROBOT_HALF_WIDTH_MM + S5_SIDE_GUARD_MM) <= g_geom.rear_mm);
    /* a very long front overhang makes the pivot the binding rule */
    s5_geom_t lg = g_geom; lg.front_mm = 400;
    CHECK(s5_pivot_min_axle_mm(&lg, S5_ROBOT_HALF_WIDTH_MM + S5_SIDE_GUARD_MM) >= 0);
    puts("[TEST] S5-M04 PASS  plan: side look only on the pass leg, mirrored look direction, fixed distance kept as fallback");
}

int main(void)
{
    s5_geom_from_config(&g_geom);
    test_tracker();
    test_simulation();
    test_plan_and_geometry();
    printf("SUMMARY side-pass: %d checks passed\n", checks);
    return 0;
}
