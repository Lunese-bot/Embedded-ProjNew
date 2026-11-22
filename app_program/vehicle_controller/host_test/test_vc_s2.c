/* Exercise the production motion bridge with simulated status snapshots. */
#include <assert.h>
#include <stdio.h>
#define APP_H
#define VEHICLE_CONTROLLER_H
#define VC_HOST_TEST
#include "../vc_types.h"

typedef int ID;
typedef int ER;
#define E_OK 0
#define E_TMOUT (-1)
#define E_PAR (-2)
#define TMO_POL 0
#define TWF_ORW 1
#define TWF_BITCLR 2

#include "../../subsystem2/subsystem2.h"

ID g_flg_motion = 1;
volatile uint32_t g_s2_imu_stop;
volatile bool g_b_s3_bc_lock = false; /* barcode lock flag read by vc_bridge_s2.c */
volatile int32_t g_lt_speed_pct, g_lt_steer_inv, g_lt_curve;
static motion_status_t status;
static uint32_t stop_epoch;
static int reset_count;
static int status_errors;
static ER status_result;

static ER tk_wai_flg(ID id, UINT bits, UINT mode, UINT *out, int timeout)
{
    (void)id; (void)bits; (void)mode; (void)out; (void)timeout;
    return E_TMOUT;
}
static ER tk_clr_flg(ID id, UINT bits) { (void)id; (void)bits; return E_OK; }
static void app_check(ER result, const char *what)
{
    (void)what;
    if (result < E_OK) status_errors++;
}
ER s2_get_status(motion_status_t *out) { *out = status; return status_result; }
uint32_t s2_stop_epoch(void) { return stop_epoch; }
ER s2_cmd_stop(void) { return E_OK; }
ER s2_cmd_drive(int32_t speed, int32_t steer)
{ (void)speed; (void)steer; return E_OK; }
ER s2_cmd_turn(int32_t speed, int32_t angle)
{ (void)speed; (void)angle; return E_OK; }
ER s2_cmd_drive_dist(int32_t speed, int32_t distance)
{ (void)speed; (void)distance; return E_OK; }
ER s2_cmd_reset(void) { reset_count++; return E_OK; }
ER s2_emergency_stop(void)
{
    if (stop_epoch != UINT32_MAX) stop_epoch++;
    return E_OK;
}

#include "../vc_bridge_s2.c"

int main(void)
{
    vc_event_t event;

    assert(vc_bridge_s2_request(VC_REQ_SELF_CHECK, 0) == E_OK);
    status_result = E_PAR;
    assert(!vc_bridge_s2_next(&event) && status_errors == 1);
    status_result = E_OK;
    assert(!vc_bridge_s2_next(&event)); /* no published motion snapshot */
    status.loop_count = 1;
    status.state = MSTATE_SAFE_STOP;
    status.stop_epoch = stop_epoch = 1;
    assert(vc_bridge_s2_next(&event));
    assert(event.type == VC_EV_FAULT && event.p1 == VC_FAULT_CRITICAL);
    assert(!vc_bridge_s2_next(&event)); /* latched stop is not READY */

    assert(vc_bridge_s2_move(VC_MOVE_RESET, 0) == E_OK && reset_count == 1);
    assert(vc_bridge_s2_request(VC_REQ_SELF_CHECK, 0) == E_OK);
    assert(!vc_bridge_s2_next(&event)); /* queueing reset is not execution */
    status.state = MSTATE_FAULT;
    assert(!vc_bridge_s2_next(&event));
    status.state = MSTATE_STOPPING;
    assert(!vc_bridge_s2_next(&event));
    status.state = MSTATE_IDLE;
    g_s2_imu_stop = 1;
    assert(!vc_bridge_s2_next(&event)); /* interlock still asserted */
    g_s2_imu_stop = 0;
    stop_epoch = 2;
    assert(!vc_bridge_s2_next(&event)); /* newer stop not yet in snapshot */
    status.stop_epoch = 2;
    assert(vc_bridge_s2_next(&event) && event.type == VC_EV_READY);
    assert(!vc_bridge_s2_next(&event));

    status.state = MSTATE_SAFE_STOP;
    status.stop_epoch = stop_epoch = 3;
    assert(vc_bridge_s2_next(&event) && event.type == VC_EV_FAULT);
    assert(!vc_bridge_s2_next(&event)); /* no repeated fault for same stop */
    status.stop_epoch = stop_epoch = 4;
    assert(vc_bridge_s2_next(&event) && event.type == VC_EV_FAULT);

    status.state = MSTATE_IDLE;
    status.stop_epoch = stop_epoch = UINT32_MAX;
    assert(vc_bridge_s2_request(VC_REQ_SELF_CHECK, 0) == E_OK);
    assert(!vc_bridge_s2_next(&event)); /* saturated generation fails closed */
    assert(vc_bridge_s2_move(VC_MOVE_SAFE_STOP, 0) == E_OK);
    status.stop_epoch = stop_epoch = 0;
    assert(!vc_bridge_s2_next(&event)); /* aborted self-check stays cancelled */

    puts("[TEST] S6-SAFE03 PASS  motion readiness follows applied stop and reset");
    return 0;
}
