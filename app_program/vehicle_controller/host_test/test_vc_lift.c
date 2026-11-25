/*
 *      test_vc_lift.c
 *      PC unit tests for "pause on lift, resume on line" (VC_LIFT_MODE_PAUSE,
 *      vc_sm.h) and for the default "lift ends the run".  No Pico, no
 *      kernel.  From this folder:  make
 *
 *      The simulated robot: every 10 ms one controller TICK (p1 = S2's IMU
 *      interlock, p2 = odometer) and, while the controller watches the line,
 *      one S3 sample (VC_EV_LINE_SAMPLE, time_ms = when it was read).
 *      "Motion" = any move that can turn a wheel (FORWARD, STEER, turns,
 *      U-turn, PIVOT, DRIVE_DIST, REVERSE).
 */

#include <stdio.h>
#include <string.h>
#include "../vc_sm.h"

static vc_sm_t      g_sm;
static vc_actions_t g_act;
static uint32_t     g_now;
static uint32_t     g_odo = 5000u; /* S2 odometer (never 0: 0 = not given) */
static int32_t      g_passed;
static int32_t      g_failed;
static int32_t      g_overflows;

/* counts since tally_clear() */
static int32_t g_moves[32];
static int32_t g_motion;     /* moves that can turn a wheel */
static int32_t g_selfcheck;  /* S2 readiness requests */
static int32_t g_watch_on;
static int32_t g_watch_off;
static int32_t g_complete;   /* entries to MISSION_COMPLETE */
static int32_t g_last_fwd;   /* value of the last FORWARD */
static int32_t g_last_speed; /* value of the last SET_SPEED */

#define CHECK(id, cond, desc)                                            \
    do                                                                   \
    {                                                                    \
        if (cond)                                                        \
        {                                                                \
            g_passed++;                                                  \
            printf("[TEST] %s PASS  %s\n", id, desc);                    \
        }                                                                \
        else                                                             \
        {                                                                \
            g_failed++;                                                  \
            printf("[TEST] %s FAIL  %s   (%s:%d)\n", id, desc, __FILE__, \
                   __LINE__);                                            \
        }                                                                \
    } while (0)

static int is_motion(int32_t move)
{
    return (VC_MOVE_FORWARD == move) || (VC_MOVE_STEER == move) ||
           (VC_MOVE_TURN_LEFT == move) || (VC_MOVE_TURN_RIGHT == move) ||
           (VC_MOVE_U_TURN == move) || (VC_MOVE_REVERSE == move) ||
           (VC_MOVE_DRIVE_DIST == move) || (VC_MOVE_PIVOT == move);
}

static void tally_clear(void)
{
    memset(g_moves, 0, sizeof(g_moves));
    g_motion = g_selfcheck = g_watch_on = g_watch_off = g_complete = 0;
}

static void tally(void)
{
    int32_t i;

    if (g_act.overflow)
    {
        g_overflows++;
    }
    for (i = 0; i < g_act.n; i++)
    {
        const vc_action_t * a = &g_act.a[i];

        if ((VC_ACT_MOVE == a->kind) && (a->p1 >= 0) && (a->p1 < 32))
        {
            g_moves[a->p1]++;
            if (VC_MOVE_FORWARD == a->p1)
            {
                g_last_fwd = a->p2;
            }
            if (VC_MOVE_SET_SPEED == a->p1)
            {
                g_last_speed = a->p2;
            }
            if (is_motion(a->p1))
            {
                g_motion++;
            }
        }
        else if ((VC_ACT_REQUEST == a->kind) && (VC_REQ_SELF_CHECK == a->p1) &&
                 (VC_SRC_MOTION == a->target))
        {
            g_selfcheck++;
        }
        else if ((VC_ACT_REQUEST == a->kind) && (VC_REQ_LINE_WATCH == a->p1))
        {
            if (0 != a->p2)
            {
                g_watch_on++;
            }
            else
            {
                g_watch_off++;
            }
        }
        else if ((VC_ACT_STATE == a->kind) && (VC_ST_MISSION_COMPLETE == a->p1))
        {
            g_complete++;
        }
        else
        {
            /* other actions */
        }
    }
}

static void feed_full(vc_source_t src, vc_event_type_t type, int16_t cause,
                      int32_t p1, int32_t p2, uint32_t time_ms)
{
    vc_event_t ev = { 0 };

    ev.src     = (uint8_t)src;
    ev.type    = (uint8_t)type;
    ev.cause   = cause;
    ev.p1      = p1;
    ev.p2      = p2;
    ev.time_ms = time_ms;
    vc_sm_handle(&g_sm, &ev, g_now, &g_act);
    tally();
}

static void feed(vc_source_t src, vc_event_type_t type, int32_t p1, int32_t p2)
{
    feed_full(src, type, 0, p1, p2, 0u);
}

/* one 10 ms step: TICK with the interlock, then (if asked) a line sample
   read just now */
static void step(int32_t imu, int32_t mask, int b_sample)
{
    g_now += 10u;
    feed_full(VC_SRC_VC, VC_EV_TICK, 0, imu, (int32_t)g_odo, g_now);
    if (b_sample)
    {
        feed_full(VC_SRC_LINE, VC_EV_LINE_SAMPLE, 0, mask, (int32_t)g_odo, g_now);
    }
}

static void run(uint32_t ms, int32_t imu, int32_t mask)
{
    uint32_t t;

    for (t = 0u; t < ms; t += 10u)
    {
        step(imu, mask, 1);
    }
}

/* S2 answers the readiness check as soon as it is asked (when it is) */
static int32_t g_ready_auto;
static void run_s2(uint32_t ms, int32_t imu, int32_t mask)
{
    uint32_t t;

    for (t = 0u; t < ms; t += 10u)
    {
        int32_t asked = g_selfcheck;

        step(imu, mask, 1);
        if (g_ready_auto && (g_selfcheck != asked))
        {
            feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
        }
    }
}

static void steer(int32_t n)
{
    int32_t i;

    for (i = 0; i < n; i++)
    {
        g_now += 10u;
        feed(VC_SRC_LINE, VC_EV_MOVE_REQUEST, VC_MOVE_STEER, 100);
    }
}

static void start(uint8_t mode)
{
    g_now = 0xFFFFF000u; /* near wrap-around on purpose */
    g_odo = 5000u;
    vc_sm_init(&g_sm, g_now);
    (void)vc_sm_set_lift_mode(&g_sm, mode);
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
    feed(VC_SRC_LINE, VC_EV_READY, 0, 0);
    feed(VC_SRC_IMU, VC_EV_READY, 0, 0);
    feed(VC_SRC_OBST, VC_EV_READY, 0, 0);
    steer(20); /* following the line at cruise */
    run(200, 0, 0);
}

/* the real order on the robot: S2's 0x2011 (cause lift) first, then S4 */
static void lift(void)
{
    feed_full(VC_SRC_MOTION, VC_EV_FAULT, VC_CAUSE_IMU_LIFT,
              VC_FAULT_CRITICAL, VC_FC_S2_SAFE_STOP, 0u);
    feed(VC_SRC_IMU, VC_EV_LIFTED, 1, VC_FC_IMU_SAFETY);
}

/* lifted for a while (in the hand, moving: wheels may spin down), then set
   down; S4's own settle rule has already passed when the interlock clears */
static void carry(uint32_t ms)
{
    uint32_t t;

    for (t = 0u; t < ms; t += 10u)
    {
        g_odo += 3u; /* wheels coasting in the air */
        step(VC_CAUSE_IMU_LIFT, VC_LINE_C | VC_LINE_R, 1); /* no floor: both black */
    }
}

static int paused_on_lift(void)
{
    return (VC_ST_FOLLOW_LINE == g_sm.st.state) && g_sm.st.paused &&
           (VC_LP_NONE != g_sm.st.lift_phase);
}

/* --------------------------------------------------------------------- */

static void test_default_mode(void)
{
    vc_sm_t fresh;

    vc_sm_init(&fresh, 0u);
    CHECK("LP-01", VC_LIFT_MODE_END == fresh.st.lift_mode &&
                       VC_LIFT_MODE_END == fresh.lift_mode_cfg &&
                       VC_LIFT_MODE_END == VC_LIFT_MODE,
          "default build: the lift mode is END (the assessed demo)");

    start(VC_LIFT_MODE_END);
    tally_clear();
    lift();
    CHECK("LP-02", (VC_ST_MISSION_COMPLETE == g_sm.st.state) &&
                       (VC_FC_IMU_SAFETY == g_sm.st.last_fault_code) &&
                       (1 == g_complete) && (0 == g_motion),
          "default mode: a lift while following = MISSION_COMPLETE, fault code 0x4005");
    run(3000, 0, VC_LINE_R);
    CHECK("LP-03", (VC_ST_MISSION_COMPLETE == g_sm.st.state) && (0 == g_motion) &&
                       (0 == g_selfcheck),
          "default mode: set down on the tape, nothing moves, no S2 handshake");
}

static void test_mode_selection(void)
{
    start(VC_LIFT_MODE_END);
    CHECK("LP-04", !vc_sm_set_lift_mode(&g_sm, VC_LIFT_MODE_PAUSE) &&
                       (VC_LIFT_MODE_END == g_sm.st.lift_mode) &&
                       (VC_LIFT_MODE_END == g_sm.lift_mode_cfg),
          "the mode cannot change during a mission");
    lift();
    CHECK("LP-05", VC_ST_MISSION_COMPLETE == g_sm.st.state,
          "a refused change leaves the run's own mode in force");
    CHECK("LP-06", vc_sm_set_lift_mode(&g_sm, VC_LIFT_MODE_PAUSE) &&
                       !vc_sm_set_lift_mode(&g_sm, 7u) &&
                       (VC_LIFT_MODE_PAUSE == g_sm.lift_mode_cfg) &&
                       (VC_LIFT_MODE_END == g_sm.st.lift_mode),
          "after the run: a valid mode is accepted for the next Start, a bad one refused");
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    CHECK("LP-07", (VC_ST_MISSION_START == g_sm.st.state) &&
                       (VC_LIFT_MODE_PAUSE == g_sm.st.lift_mode),
          "Start takes the selected mode for the new run");
}

static void test_resume_on_tape(void)
{
    uint32_t t_start;
    uint32_t t_down;
    uint32_t t_go = 0u;
    int32_t  rejected;

    start(VC_LIFT_MODE_PAUSE);
    t_start = g_sm.st.mission_start_ms;
    tally_clear();
    lift();
    CHECK("LP-10", paused_on_lift() && (VC_LP_LIFTED == g_sm.st.lift_phase) &&
                       (1 == g_moves[VC_MOVE_STOP]) && (0 == g_motion) &&
                       (0 == g_complete) &&
                       (VC_FC_LIFT_LIFTED == g_sm.st.last_fault_code),
          "pause mode: lift while following = stopped, same mission, reason Lifted");
    rejected = (int32_t)g_sm.st.moves_rejected;
    steer(5);
    CHECK("LP-11", ((int32_t)g_sm.st.moves_rejected == rejected + 5) &&
                       (0 == g_motion),
          "S3 steering (old FOLLOW state) cannot move a lifted robot");
    carry(1500);
    CHECK("LP-12", (VC_LP_LIFTED == g_sm.st.lift_phase) && (0 == g_motion),
          "while held (interlock set, wheels spinning, sensors black): stays Lifted");

    t_down = g_now;
    g_ready_auto = 1;
    while ((0 == g_moves[VC_MOVE_FORWARD]) && (g_now - t_down < 5000u))
    {
        uint8_t before = g_sm.st.lift_phase;

        run_s2(10, 0, VC_LINE_R);
        if ((VC_LP_SETTLING == g_sm.st.lift_phase) && (VC_LP_LIFTED == before))
        {
            CHECK("LP-13", VC_FC_LIFT_SETTLING == g_sm.st.last_fault_code,
                  "set down: reason Settling");
        }
        if ((VC_LP_WAIT_LINE == g_sm.st.lift_phase) && (VC_LP_SETTLING == before))
        {
            CHECK("LP-14", (g_now - t_down >= VC_LIFT_SETTLE_MS) &&
                               (VC_FC_LIFT_WAIT_LINE == g_sm.st.last_fault_code) &&
                               (1 == g_watch_on),
                  "after VC_LIFT_SETTLE_MS: reason Waiting for line, line watch on");
        }
        if ((VC_LP_RESUMING == g_sm.st.lift_phase) && (VC_LP_WAIT_LINE == before))
        {
            CHECK("LP-15", (1 == g_moves[VC_MOVE_RESET]) && (1 == g_selfcheck) &&
                               (0 == g_motion) &&
                               (VC_FC_LIFT_RESUMING == g_sm.st.last_fault_code),
                  "line confirmed: S2 RESET + readiness check, still no motion, reason Resuming");
        }
    }
    t_go = g_now;
    g_ready_auto = 0;
    CHECK("LP-16", (1 == g_moves[VC_MOVE_FORWARD]) && (1 == g_motion) &&
                       !g_sm.st.paused && (VC_ST_FOLLOW_LINE == g_sm.st.state) &&
                       (1u == g_sm.st.lift_resumes) && (1 == g_watch_off),
          "exactly one autonomous resume (one FORWARD), watch off");
    CHECK("LP-17", (t_go - t_down) >= (VC_LIFT_SETTLE_MS + VC_LIFT_LINE_MS),
          "no resume before settling + the documented line interval");
    CHECK("LP-18", (g_sm.st.mission_start_ms == t_start) &&
                       (VC_ST_FOLLOW_LINE == g_sm.st.state) && (0 == g_complete),
          "same mission: start time kept, never left FOLLOW_LINE");
    CHECK("LP-19", (VC_SPEED_SLOW_MMS == g_last_fwd) &&
                       (VC_SPEED_SLOW_MMS == g_last_speed) &&
                       (VC_LP_RESUMING == g_sm.st.lift_phase),
          "resume starts slow (SET_SPEED / FORWARD at VC_SPEED_SLOW_MMS), still shown as Resuming");
    tally_clear();
    steer(VC_REJOIN_STEER_SAMPLES);
    CHECK("LP-20", (VC_REJOIN_STEER_SAMPLES == g_moves[VC_MOVE_STEER]) &&
                       (1 == g_moves[VC_MOVE_SET_SPEED]) &&
                       (VC_SPEED_CRUISE_MMS == g_last_speed) &&
                       (VC_LP_NONE == g_sm.st.lift_phase) &&
                       (VC_FC_NONE == g_sm.st.last_fault_code),
          "after VC_REJOIN_STEER_SAMPLES fresh steering: cruise, the lift reason clears");
    tally_clear();
    run(5000, 0, VC_LINE_R);
    CHECK("LP-21", (0 == g_moves[VC_MOVE_FORWARD]) && (0 == g_selfcheck),
          "no second resume afterwards");
}

/* set down, then the robot stays where it is with this sensor pattern */
static void down_and_wait(uint32_t ms, int32_t mask)
{
    g_ready_auto = 1;
    run_s2(ms, 0, mask);
    g_ready_auto = 0;
}

static void test_no_line(void)
{
    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(500);
    tally_clear();
    down_and_wait(120000, 0);
    CHECK("LP-30", (0 == g_motion) && (0 == g_selfcheck) &&
                       (0 == g_moves[VC_MOVE_RESET]) && paused_on_lift() &&
                       (VC_LP_WAIT_LINE == g_sm.st.lift_phase),
          "white floor (both white) for 2 min: zero motion, no S2 release, Waiting for line");

    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(500);
    tally_clear();
    down_and_wait(30000, VC_LINE_C | VC_LINE_R);
    CHECK("LP-31", (0 == g_motion) && (0 == g_selfcheck) && paused_on_lift(),
          "both black (crossing or no floor): zero motion");

    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(500);
    tally_clear();
    {
        uint32_t i;

        g_ready_auto = 1;
        for (i = 0u; i < 3000u; i++)
        {
            /* one black sample in 20, the rest white: noise */
            run_s2(10, 0, ((i % 20u) == 7u) ? VC_LINE_R : 0);
        }
        g_ready_auto = 0;
    }
    CHECK("LP-32", (0 == g_motion) && (0 == g_selfcheck),
          "single noisy black samples on white: zero motion");

    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(500);
    tally_clear();
    {
        uint32_t i;

        g_ready_auto = 1;
        for (i = 0u; i < 3000u; i++)
        {
            /* C and R take turns: never one sensor long enough */
            run_s2(10, 0, ((i / 10u) % 2u) ? VC_LINE_R : VC_LINE_C);
        }
        g_ready_auto = 0;
    }
    CHECK("LP-33", (0 == g_motion) && (0 == g_selfcheck),
          "black that keeps changing sensor: zero motion");
}

static void test_stale(void)
{
    uint32_t t_lift;
    uint32_t i;

    start(VC_LIFT_MODE_PAUSE);
    t_lift = g_now;
    lift();
    carry(300);
    tally_clear();
    /* set down: from now on only stale samples arrive (taken at the lift,
       before set-down), many times over, plus repeats of one sample */
    for (i = 0u; i < 2000u; i++)
    {
        g_now += 10u;
        feed_full(VC_SRC_VC, VC_EV_TICK, 0, 0, (int32_t)g_odo, g_now);
        feed_full(VC_SRC_LINE, VC_EV_LINE_SAMPLE, 0, VC_LINE_R, (int32_t)g_odo,
                  t_lift - 10u);
        if (VC_LP_WAIT_LINE == g_sm.st.lift_phase)
        {
            /* one fresh sample, then the same sample again and again */
            feed_full(VC_SRC_LINE, VC_EV_LINE_SAMPLE, 0, VC_LINE_R,
                      (int32_t)g_odo, g_sm.lp.phase_ms + 10u);
        }
    }
    CHECK("LP-40", (0 == g_motion) && (0 == g_selfcheck) &&
                       (VC_LP_WAIT_LINE == g_sm.st.lift_phase),
          "stale pre-set-down samples and one sample repeated: never confirm the line");

    /* a backlog of samples read before set-down, delivered after it, in
       time order (each one newer than the last) */
    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(300);
    tally_clear();
    for (i = 0u; (i < 300u) && (VC_LP_WAIT_LINE != g_sm.st.lift_phase); i++)
    {
        step(0, 0, 0);
    }
    {
        uint32_t t0 = g_sm.lp.phase_ms - 1000u;

        for (i = 0u; i < 100u; i++)
        {
            feed_full(VC_SRC_LINE, VC_EV_LINE_SAMPLE, 0, VC_LINE_R,
                      (int32_t)g_odo, t0 + (10u * i));
        }
    }
    run(100, 0, 0);
    CHECK("LP-42", (0 == g_selfcheck) && (0 == g_motion),
          "100 black samples read before set-down, delivered after it in order: ignored");

    /* delayed events from before the lift */
    tally_clear();
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
    feed(VC_SRC_LINE, VC_EV_LINE_FOUND, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MANOEUVRE_DONE, 1, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
    feed(VC_SRC_LINE, VC_EV_LINE_LOST, 1, 0);
    feed(VC_SRC_OBST, VC_EV_OBSTACLE, 100, 0);
    feed(VC_SRC_IMU, VC_EV_HUMP_START, 0, 0);
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    feed(VC_SRC_LINE, VC_EV_FAULT, VC_FAULT_RECOVERABLE, 0x3001);
    CHECK("LP-41", (0 == g_motion) && (0 == g_moves[VC_MOVE_RESET]) &&
                       paused_on_lift() && (VC_LP_WAIT_LINE == g_sm.st.lift_phase),
          "late READY, LINE_FOUND, MANOEUVRE_DONE, MOTION_STOPPED, junction, barcode, obstacle, hump, Start, recoverable fault: no motion, still paused");
}

static void test_handshake_loss(void)
{
    uint32_t i;

    /* the line goes before S2 answers */
    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(300);
    tally_clear();
    for (i = 0u; (i < 500u) && (0 == g_selfcheck); i++)
    {
        run(10, 0, VC_LINE_R);
    }
    run(10, 0, 0);
    CHECK("LP-50", (1 == g_selfcheck) && (VC_LP_WAIT_LINE == g_sm.st.lift_phase) &&
                       (0 == g_motion),
          "line lost while waiting for S2 READY: back to Waiting for line");
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0); /* that READY arrives late */
    run(3000, 0, 0);
    CHECK("LP-51", (0 == g_motion) && paused_on_lift(),
          "the late READY of the cancelled handshake cannot start the robot");

    /* the line goes after READY, before the verify samples */
    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(300);
    tally_clear();
    for (i = 0u; (i < 500u) && (0 == g_selfcheck); i++)
    {
        run(10, 0, VC_LINE_R);
    }
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
    run(10, 0, VC_LINE_R | VC_LINE_C);
    run(3000, 0, 0);
    CHECK("LP-52", (0 == g_motion) && (VC_LP_WAIT_LINE == g_sm.st.lift_phase),
          "line changes during the verify after READY: zero forward motion");

    /* READY, then fewer than VC_LIFT_VERIFY_SAMPLES matching samples, then
       the line goes */
    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(300);
    tally_clear();
    for (i = 0u; (i < 500u) && (0 == g_selfcheck); i++)
    {
        run(10, 0, VC_LINE_R);
    }
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
    run(10u * (VC_LIFT_VERIFY_SAMPLES - 1), 0, VC_LINE_R);
    run(3000, 0, 0);
    CHECK("LP-56", (0 == g_motion) && (VC_LP_WAIT_LINE == g_sm.st.lift_phase),
          "READY then too few fresh samples before the line goes: zero motion");

    /* S2 never answers */
    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(300);
    tally_clear();
    for (i = 0u; (i < 500u) && (0 == g_selfcheck); i++)
    {
        run(10, 0, VC_LINE_R);
    }
    run(VC_LIFT_READY_MS + 20, 0, VC_LINE_R);
    CHECK("LP-53", (0 == g_motion) && (g_moves[VC_MOVE_STOP] >= 1),
          "no READY from S2 within VC_LIFT_READY_MS: stopped, no motion");
    g_ready_auto = 1;
    run_s2(2000, 0, VC_LINE_R);
    g_ready_auto = 0;
    CHECK("LP-54", (1 == g_moves[VC_MOVE_FORWARD]) && (1 == g_motion),
          "a fresh handshake later (line still confirmed, S2 ready) resumes once");

    /* READY from S4 (its calibration) is not S2's readiness */
    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(300);
    tally_clear();
    for (i = 0u; (i < 500u) && (0 == g_selfcheck); i++)
    {
        run(10, 0, VC_LINE_R);
    }
    feed(VC_SRC_IMU, VC_EV_READY, 0, 0);
    feed(VC_SRC_LINE, VC_EV_READY, 0, 0);
    run(200, 0, VC_LINE_R);
    CHECK("LP-55", 0 == g_motion, "READY from S3 / S4 does not count as S2's readiness");
}

static void test_reposition(void)
{
    uint32_t t_down2;

    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(300);
    tally_clear();
    down_and_wait(5000, 0); /* off the line */
    CHECK("LP-60", (0 == g_motion) && (VC_LP_WAIT_LINE == g_sm.st.lift_phase),
          "set down off the line: waits");
    /* picked up again (S4 latches anew: the interlock and both reports) */
    step(VC_CAUSE_IMU_LIFT, 0, 1);
    lift();
    CHECK("LP-61", (VC_LP_LIFTED == g_sm.st.lift_phase) && (2u == g_sm.lp.episode) &&
                       (VC_FC_LIFT_LIFTED == g_sm.st.last_fault_code),
          "picked up again: a new lift episode, reason Lifted");
    carry(800);
    t_down2 = g_now;
    down_and_wait(3000, VC_LINE_C);
    CHECK("LP-62", (1 == g_moves[VC_MOVE_FORWARD]) && (1 == g_motion) &&
                       (1u == g_sm.st.lift_resumes),
          "put back onto the tape (C over it): one resume");
    (void)t_down2;

    /* pushed along the floor onto the tape without lifting: the wheels
       turn, so settling and the line checks start again */
    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(300);
    tally_clear();
    down_and_wait(3000, 0);
    {
        uint32_t i;
        uint32_t t_still;

        for (i = 0u; i < 40u; i++)
        {
            g_odo += 4u; /* pushed */
            run(10, 0, (i < 20u) ? 0 : VC_LINE_R);
        }
        CHECK("LP-63", (VC_LP_SETTLING == g_sm.st.lift_phase) && (0 == g_motion),
              "wheels turned while waiting (pushed): back to Settling");
        t_still = g_now;
        g_ready_auto = 1;
        while ((0 == g_moves[VC_MOVE_FORWARD]) && (g_now - t_still < 5000u))
        {
            run_s2(10, 0, VC_LINE_R);
        }
        g_ready_auto = 0;
        CHECK("LP-64", (1 == g_moves[VC_MOVE_FORWARD]) &&
                           ((g_now - t_still) >= (VC_LIFT_SETTLE_MS + VC_LIFT_LINE_MS)),
              "then still on the tape: resumes only after a fresh settle and line confirmation");
    }

    /* interlock set again without any event (S4's events dropped) */
    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(300);
    down_and_wait(2000, 0);
    tally_clear();
    step(VC_CAUSE_IMU_LIFT, 0, 1);
    CHECK("LP-65", (VC_LP_LIFTED == g_sm.st.lift_phase) && (0 == g_motion),
          "interlock seen in the TICK alone: Lifted again");
}

static void test_cancel(void)
{
    uint32_t i;

    /* overturn after set-down */
    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(300);
    down_and_wait(2000, 0);
    tally_clear();
    step(VC_CAUSE_IMU_OVERTURN, 0, 1);
    feed_full(VC_SRC_MOTION, VC_EV_FAULT, VC_CAUSE_IMU_OVERTURN, VC_FAULT_CRITICAL,
              VC_FC_S2_SAFE_STOP, 0u);
    CHECK("LP-70", (VC_ST_FAULT == g_sm.st.state) &&
                       (VC_FC_IMU_SAFETY == g_sm.st.last_fault_code) &&
                       (VC_LP_NONE == g_sm.st.lift_phase) && (g_watch_off >= 1),
          "overturn while lift-paused: FAULT 0x4005, pause cancelled, watch off");
    down_and_wait(5000, VC_LINE_R);
    CHECK("LP-71", (VC_ST_FAULT == g_sm.st.state) && (0 == g_motion) &&
                       (0 == g_selfcheck),
          "set down level on the tape after an overturn: still FAULT, no motion");

    /* overturn seen only in the interlock (S4 / S2 reports dropped) */
    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(300);
    down_and_wait(2000, 0);
    tally_clear();
    step(VC_CAUSE_IMU_OVERTURN, 0, 1);
    down_and_wait(3000, VC_LINE_R);
    CHECK("LP-79", (VC_ST_FAULT == g_sm.st.state) &&
                       (VC_FC_IMU_SAFETY == g_sm.st.last_fault_code) && (0 == g_motion),
          "overturn in the TICK interlock alone: FAULT 0x4005, no motion");

    /* overturn report through S2 alone, during the handshake */
    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(300);
    tally_clear();
    for (i = 0u; (i < 500u) && (0 == g_selfcheck); i++)
    {
        run(10, 0, VC_LINE_R);
    }
    feed_full(VC_SRC_MOTION, VC_EV_FAULT, VC_CAUSE_IMU_OVERTURN, VC_FAULT_CRITICAL,
              VC_FC_S2_SAFE_STOP, 0u);
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
    run(2000, 0, VC_LINE_R);
    CHECK("LP-72", (VC_ST_FAULT == g_sm.st.state) && (0 == g_motion),
          "overturn during the S2 handshake: FAULT, the READY after it does nothing");

    /* operator Stop */
    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(300);
    tally_clear();
    feed(VC_SRC_COMMS, VC_EV_CMD_STOP, 0, 0);
    down_and_wait(5000, VC_LINE_R);
    CHECK("LP-73", (VC_ST_FAULT == g_sm.st.state) &&
                       (VC_FC_OPERATOR_STOP == g_sm.st.last_fault_code) &&
                       (0 == g_motion) && (0 == g_selfcheck),
          "operator Stop while lifted: FAULT 0x0001, set-down on tape does not clear it");

    /* independent safe stop (no IMU cause) */
    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(300);
    down_and_wait(1500, 0);
    tally_clear();
    feed_full(VC_SRC_MOTION, VC_EV_FAULT, VC_CAUSE_NONE, VC_FAULT_CRITICAL,
              VC_FC_S2_SAFE_STOP, 0u);
    down_and_wait(5000, VC_LINE_R);
    CHECK("LP-74", (VC_ST_FAULT == g_sm.st.state) &&
                       (VC_FC_S2_SAFE_STOP == g_sm.st.last_fault_code) && (0 == g_motion),
          "independent 0x2011 while lift-paused: FAULT 0x2011, no motion");

    /* a second lift during the resume handshake */
    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(300);
    tally_clear();
    for (i = 0u; (i < 500u) && (0 == g_selfcheck); i++)
    {
        run(10, 0, VC_LINE_R);
    }
    step(VC_CAUSE_IMU_LIFT, 0, 1);
    lift();
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
    carry(500);
    CHECK("LP-75", (VC_LP_LIFTED == g_sm.st.lift_phase) && (0 == g_motion),
          "second lift during the handshake: Lifted, the READY that follows is ignored");
    down_and_wait(3000, VC_LINE_R);
    CHECK("LP-76", (1 == g_moves[VC_MOVE_FORWARD]) && (1 == g_motion),
          "then set down on the tape: one resume, after fresh checks");

    /* Reset while lift-paused */
    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(300);
    down_and_wait(2000, 0);
    tally_clear();
    feed(VC_SRC_COMMS, VC_EV_CMD_RESET, 0, 0);
    down_and_wait(3000, VC_LINE_R);
    CHECK("LP-77", (VC_ST_IDLE == g_sm.st.state) && (0 == g_motion) &&
                       (1 == g_watch_off) && (0 == g_moves[VC_MOVE_RESET]),
          "Reset while lift-paused: IDLE, nothing moves, S2 stays latched until Start");

    /* other critical fault */
    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(300);
    tally_clear();
    feed(VC_SRC_MOTION, VC_EV_FAULT, VC_FAULT_CRITICAL, 0x2010);
    down_and_wait(3000, VC_LINE_R);
    CHECK("LP-78", (VC_ST_FAULT == g_sm.st.state) && (0 == g_motion),
          "another critical fault while lift-paused: FAULT, no motion");
}

static void test_operator_pause(void)
{
    start(VC_LIFT_MODE_PAUSE);
    feed(VC_SRC_COMMS, VC_EV_CMD_PAUSE, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    CHECK("LP-80", g_sm.st.paused && g_sm.cmd_ok, "operator Pause accepted");
    lift();
    carry(300);
    tally_clear();
    down_and_wait(5000, VC_LINE_R);
    CHECK("LP-81", paused_on_lift() && (0 == g_motion) && (0 == g_selfcheck) &&
                       g_sm.lp.op_hold,
          "lifted while paused, set down on the tape: stays paused (no handshake)");
    feed(VC_SRC_COMMS, VC_EV_CMD_RESUME, 0, 0);
    CHECK("LP-82", g_sm.cmd_ok && !g_sm.lp.op_hold && (0 == g_motion),
          "Resume ends the operator's hold, motion still waits for the checks");
    down_and_wait(1000, VC_LINE_R);
    CHECK("LP-83", (1 == g_moves[VC_MOVE_FORWARD]) && (1 == g_motion),
          "then the line checks pass: one resume");

    /* Resume cannot skip the checks */
    start(VC_LIFT_MODE_PAUSE);
    lift();
    carry(300);
    tally_clear();
    feed(VC_SRC_COMMS, VC_EV_CMD_RESUME, 0, 0);
    CHECK("LP-84", !g_sm.cmd_ok && paused_on_lift() && (0 == g_motion),
          "Resume while lifted (no operator hold): rejected, no motion");
    /* Pause during the handshake */
    {
        uint32_t i;

        for (i = 0u; (i < 500u) && (0 == g_selfcheck); i++)
        {
            run(10, 0, VC_LINE_R);
        }
    }
    feed(VC_SRC_COMMS, VC_EV_CMD_PAUSE, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_READY, 0, 0);
    down_and_wait(3000, VC_LINE_R);
    CHECK("LP-85", g_sm.cmd_ok && (0 == g_motion) && g_sm.lp.op_hold,
          "Pause during the S2 handshake: accepted, holds the robot");
}

static void test_other_states(void)
{
    /* mid-turn (EXECUTE_ACTION) */
    start(VC_LIFT_MODE_PAUSE);
    feed(VC_SRC_LINE, VC_EV_JUNCTION, 0, 0);
    feed(VC_SRC_MOTION, VC_EV_MOTION_STOPPED, 0, 0);
    run(VC_JUNCTION_HOLD_MS + 20, 0, 0);
    CHECK("LP-90", VC_ST_EXECUTE_ACTION == g_sm.st.state, "(setup) junction manoeuvre running");
    tally_clear();
    lift();
    down_and_wait(5000, VC_LINE_R);
    CHECK("LP-91", (VC_ST_MISSION_COMPLETE == g_sm.st.state) && (0 == g_motion),
          "pause mode, lift mid-turn: run ends (operator recovery), no resume");

    start(VC_LIFT_MODE_PAUSE);
    feed(VC_SRC_LINE, VC_EV_BARCODE_START, 0, 0);
    tally_clear();
    lift();
    down_and_wait(5000, VC_LINE_R);
    CHECK("LP-92", (VC_ST_MISSION_COMPLETE == g_sm.st.state) && (0 == g_motion),
          "pause mode, lift while decoding a barcode: run ends, no resume");

    start(VC_LIFT_MODE_PAUSE);
    feed(VC_SRC_OBST, VC_EV_OBSTACLE, 150, 0);
    tally_clear();
    lift();
    down_and_wait(5000, VC_LINE_R);
    CHECK("LP-93", (VC_ST_MISSION_COMPLETE == g_sm.st.state) && (0 == g_motion),
          "pause mode, lift during the obstacle bypass: run ends, no resume");

    g_now = 0u;
    vc_sm_init(&g_sm, g_now);
    (void)vc_sm_set_lift_mode(&g_sm, VC_LIFT_MODE_PAUSE);
    feed(VC_SRC_COMMS, VC_EV_CMD_START, 0, 0);
    tally_clear();
    lift();
    CHECK("LP-94", (VC_ST_FAULT == g_sm.st.state) && (0 == g_motion),
          "pause mode, lift during MISSION_START: FAULT as before");
}

static void test_mqtt_down(void)
{
    start(VC_LIFT_MODE_PAUSE);
    feed(VC_SRC_COMMS, VC_EV_WIFI_DOWN, 0, 0);
    lift();
    carry(300);
    tally_clear();
    down_and_wait(3000, VC_LINE_R);
    CHECK("LP-95", (1 == g_moves[VC_MOVE_FORWARD]) && !g_sm.st.wifi_up,
          "WiFi / MQTT down: the lift pause and resume need nothing from S1");
    feed(VC_SRC_COMMS, VC_EV_WIFI_DOWN, 0, 0);
    lift();
    feed(VC_SRC_COMMS, VC_EV_WIFI_DOWN, 0, 0);
    CHECK("LP-96", paused_on_lift(), "WiFi down reports during a lift pause change nothing");
}

int main(void)
{
    test_default_mode();
    test_mode_selection();
    test_resume_on_tape();
    test_no_line();
    test_stale();
    test_handshake_loss();
    test_reposition();
    test_cancel();
    test_operator_pause();
    test_other_states();
    test_mqtt_down();
    CHECK("LP-99", 0 == g_overflows, "no action list overflowed (VC_MAX_ACTIONS)");
    printf("[TEST] lift pause host SUMMARY %d passed, %d failed\n", (int)g_passed,
           (int)g_failed);
    return (0 == g_failed) ? 0 : 1;
}
