/* Exercise queued junctions through the production S3 bridge. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define APP_H
#define VEHICLE_CONTROLLER_H
#define VC_HOST_TEST
#include "../vc_types.h"

typedef int ID;
typedef int ER;
typedef int TMO;
typedef struct { uint32_t lo; } SYSTIM;
#define E_OK 0
#define E_TMOUT (-1)
#define E_PAR (-2)
#define TMO_POL 0

#include "../../subsystem3/subsystem3.h"

ID g_mbf_s3_events = 1;
volatile uint32_t g_s2_odo_ticks;
volatile int32_t g_lt_bc_en, g_lt_lost_en, g_lt_next_nav = NAV_LEFT;
char g_lt_bc_text[16], g_lt_bc_hist[48];
static s3_line_status_t line_status;
static s3_event_t queued_event;
static bool event_ready;
static bool bypass_active;
static uint32_t clock_ms;
static uint8_t barcode_payload[8];
static int barcode_len, barcode_error, post_result;

static int32_t tk_rcv_mbf(ID id, void *out, TMO timeout)
{
    assert(g_mbf_s3_events == id && TMO_POL == timeout);
    if (!event_ready)
    {
        return E_TMOUT;
    }
    *(s3_event_t *)out = queued_event;
    event_ready = false;
    return sizeof(s3_event_t);
}

static void tm_printf(const uint8_t *fmt, ...) { (void)fmt; }
static void app_check(ER result, const char *what)
{
    (void)what;
    if (result != E_OK) barcode_error = result;
}
ER s1_post(const char *topic, const void *payload, int32_t len)
{
    assert(strcmp(topic, "barcode") == 0 && len > 0 && len <= 8);
    memcpy(barcode_payload, payload, (size_t)len);
    barcode_len = len;
    return post_result;
}
static void tk_get_tim(SYSTIM *time) { time->lo = clock_ms; }
void s3_set_obstacle_mode(bool active) { bypass_active = active; }
ER s3_get_line_status(s3_line_status_t *out) { *out = line_status; return E_OK; }
ER s3_request(uint32_t request) { (void)request; return E_OK; }

vc_source_t vc_sm_motion_owner(vc_state_t state)
{
    return (VC_ST_FOLLOW_LINE == state) ? VC_SRC_LINE : VC_SRC_NONE;
}

#include "../vc_bridge_s3.c"

static void queue_junction(uint32_t time_ms)
{
    queued_event = (s3_event_t){ .type = S3_EVT_JUNCTION, .time_ms = time_ms };
    event_ready = true;
}

int main(void)
{
    vc_event_t event;

    clock_ms = 100;
    vc_bridge_s3_set_obstacle_mode(true);
    assert(bypass_active);
    queue_junction(101);
    assert(!vc_bridge_s3_next(&event, VC_ST_AVOID_OBSTACLE));

    clock_ms = 300;
    queue_junction(299); /* Queued before the handoff, processed afterward. */
    vc_bridge_s3_set_obstacle_mode(false);
    assert(!bypass_active);
    line_status = (s3_line_status_t){ .seq = 1, .time_ms = 99,
                                     .line_state = LINE_STATE_FOLLOW, .steering = 200 };
    assert(!vc_bridge_s3_next(&event, VC_ST_FOLLOW_LINE) && !event_ready);
    line_status.seq++;
    line_status.time_ms = 301;
    assert(vc_bridge_s3_next(&event, VC_ST_FOLLOW_LINE));
    assert(VC_EV_MOVE_REQUEST == event.type && VC_MOVE_STEER == event.p1);
    assert(200 == event.p2 && !event_ready);
    puts("[TEST] S6-OBS06 PASS  queued junctions and stale steering are discarded at handoff");

    queue_junction(500);
    assert(vc_bridge_s3_next(&event, VC_ST_FOLLOW_LINE));
    assert(VC_EV_BARCODE_DECODED == event.type && NAV_LEFT == event.p1);
    assert(vc_bridge_s3_next(&event, VC_ST_FOLLOW_LINE));
    assert(VC_EV_JUNCTION == event.type);
    puts("[TEST] S6-OBS07 PASS  a later junction keeps the configured navigation action");

    queue_junction(600);
    assert(vc_bridge_s3_next(&event, VC_ST_FOLLOW_LINE));
    assert(VC_EV_BARCODE_DECODED == event.type && NAV_LEFT == event.p1);
    g_lt_next_nav = NAV_STRAIGHT;
    queue_junction(601);
    line_status.seq++;
    line_status.time_ms = 602;
    assert(vc_bridge_s3_next(&event, VC_ST_FOLLOW_LINE));
    assert(VC_EV_MOVE_REQUEST == event.type && VC_MOVE_STEER == event.p1);
    assert(!vc_bridge_s3_next(&event, VC_ST_FOLLOW_LINE) && !event_ready);
    puts("[TEST] S6-OBS10 PASS  straight mode preserves steering through a junction");

    g_lt_bc_en = 1;
    queued_event = (s3_event_t){ .type = S3_EVT_BARCODE_DECODED,
                                .nav = NAV_LEFT, .text = "AB" };
    event_ready = true;
    assert(vc_bridge_s3_next(&event, VC_ST_FOLLOW_LINE));
    assert(event.type == VC_EV_BARCODE_DECODED &&
           strcmp(event.barcode_text, "AB") == 0);
    assert(barcode_len == 2 && barcode_payload[0] == 0x2c &&
           barcode_payload[1] == 0xc0 && barcode_error == 0);

    post_result = E_TMOUT;
    event_ready = true;
    assert(vc_bridge_s3_next(&event, VC_ST_FOLLOW_LINE));
    assert(event.type == VC_EV_BARCODE_DECODED && barcode_error == E_TMOUT);
    puts("[TEST] S6-BC01 PASS  barcode publish and failure reporting preserve controller event");
    /* U-turn watch: every new sample (black or white) with the odometer,
       only after the request, only in EXECUTE_ACTION */
    g_lt_bc_en = 1;
    event_ready = false;
    line_status = (s3_line_status_t){ .seq = 50, .time_ms = 700,
                                     .line_state = LINE_STATE_FOLLOW,
                                     .black_mask = S3_MASK_C };
    assert(E_OK == vc_bridge_s3_request(VC_REQ_LINE_WATCH, 1));
    assert(!vc_bridge_s3_next(&event, VC_ST_EXECUTE_ACTION)); /* seq 50 predates the request */
    line_status.seq = 51; line_status.time_ms = 710;
    line_status.black_mask = 0u;
    g_s2_odo_ticks = 1234u;
    assert(vc_bridge_s3_next(&event, VC_ST_EXECUTE_ACTION));
    assert(VC_EV_LINE_SAMPLE == event.type && 0 == event.p1 &&
           1234 == event.p2 && 710u == event.time_ms);
    assert(!vc_bridge_s3_next(&event, VC_ST_EXECUTE_ACTION)); /* same sample: once */
    line_status.seq = 52; line_status.time_ms = 720;
    line_status.black_mask = (uint8_t)(S3_MASK_R | S3_MASK_L);
    assert(vc_bridge_s3_next(&event, VC_ST_EXECUTE_ACTION));
    assert(VC_EV_LINE_SAMPLE == event.type && VC_LINE_R == event.p1);
    line_status.seq = 53;
    line_status.black_mask = (uint8_t)(S3_MASK_C | S3_MASK_R);
    assert(vc_bridge_s3_next(&event, VC_ST_EXECUTE_ACTION));
    assert((VC_LINE_C | VC_LINE_R) == event.p1);
    /* leaving EXECUTE_ACTION / FOLLOW_LINE ends the watch: back there,
       only LINE_FOUND */
    line_status.seq = 54;
    (void)vc_bridge_s3_next(&event, VC_ST_FAULT);
    line_status.seq = 55;
    assert(vc_bridge_s3_next(&event, VC_ST_EXECUTE_ACTION) &&
           VC_EV_LINE_FOUND == event.type);
    assert(E_OK == vc_bridge_s3_request(VC_REQ_LINE_WATCH, 1));
    assert(E_OK == vc_bridge_s3_request(VC_REQ_LINE_WATCH, 0));
    line_status.seq = 56;
    assert(vc_bridge_s3_next(&event, VC_ST_EXECUTE_ACTION) &&
           VC_EV_LINE_FOUND == event.type);
    puts("[TEST] S6-U04 PASS  LINE_WATCH: each new sample after the request, white too, C/R bits + odometer + sample time; off with 0 or outside EXECUTE_ACTION / FOLLOW_LINE");

    /* Lift pause (FOLLOW_LINE): the watch sends every new sample once,
       ahead of the steering, and ends when the controller leaves FOLLOW_LINE
       or sends 0. */
    line_status = (s3_line_status_t){ .seq = 80, .time_ms = 900,
                                     .line_state = LINE_STATE_FOLLOW,
                                     .black_mask = S3_MASK_R, .steering = 400 };
    assert(E_OK == vc_bridge_s3_request(VC_REQ_LINE_WATCH, 1));
    assert(vc_bridge_s3_next(&event, VC_ST_FOLLOW_LINE) &&
           VC_EV_MOVE_REQUEST == event.type); /* seq 80 predates the request: steering only */
    assert(!vc_bridge_s3_next(&event, VC_ST_FOLLOW_LINE));
    line_status.seq = 81; line_status.time_ms = 910;
    assert(vc_bridge_s3_next(&event, VC_ST_FOLLOW_LINE) &&
           VC_EV_LINE_SAMPLE == event.type && VC_LINE_R == event.p1 &&
           910u == event.time_ms);
    assert(vc_bridge_s3_next(&event, VC_ST_FOLLOW_LINE) &&
           VC_EV_MOVE_REQUEST == event.type); /* the controller rejects it while paused */
    assert(!vc_bridge_s3_next(&event, VC_ST_FOLLOW_LINE)); /* same sample: once */
    line_status.seq = 82; line_status.time_ms = 920;
    (void)vc_bridge_s3_next(&event, VC_ST_IDLE); /* run ended: watch off */
    line_status.seq = 83; line_status.time_ms = 930;
    assert(vc_bridge_s3_next(&event, VC_ST_FOLLOW_LINE) &&
           VC_EV_MOVE_REQUEST == event.type);
    assert(!vc_bridge_s3_next(&event, VC_ST_FOLLOW_LINE));
    assert(E_OK == vc_bridge_s3_request(VC_REQ_LINE_WATCH, 1));
    assert(E_OK == vc_bridge_s3_request(VC_REQ_LINE_WATCH, 0));
    line_status.seq = 84; line_status.time_ms = 940;
    assert(vc_bridge_s3_next(&event, VC_ST_FOLLOW_LINE) &&
           VC_EV_MOVE_REQUEST == event.type);
    assert(!vc_bridge_s3_next(&event, VC_ST_FOLLOW_LINE));
    puts("[TEST] LP-B01 PASS  lift pause: LINE_WATCH works in FOLLOW_LINE, each new sample once, ahead of steering; off with 0 or when FOLLOW_LINE ends");

    return 0;
}
