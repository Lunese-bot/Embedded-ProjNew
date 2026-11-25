/* Exercise the production motion bridge with simulated status snapshots. */
#include <assert.h>
#include <stdio.h>
#define APP_H
#define VEHICLE_CONTROLLER_H
#define VC_HOST_TEST
#include "../vc_types.h"

typedef int ID;
typedef int ER;
#define E_OK       0
#define E_TMOUT    (-1)
#define E_PAR      (-2)
#define TMO_POL    0
#define TWF_ORW    1
#define TWF_BITCLR 2

#include "../../subsystem2/subsystem2.h"

ID                     g_flg_motion = 1;
volatile uint32_t      g_s2_imu_stop;
volatile bool          g_b_s3_bc_lock = false; /* barcode lock flag read by
                                                  vc_bridge_s2.c */
volatile int16_t       g_s3_bc_speed_now; /* barcode read record, written by
                                             vc_bridge_s2.c */
volatile int32_t       g_lt_speed_pct, g_lt_steer_inv, g_lt_curve;
static motion_status_t status;
static uint32_t        stop_epoch;
static int             reset_count;
static int             status_errors;
static ER              status_result;

volatile uint32_t g_s2_odo_ticks; /* S2's odometer, read by the bridge */
static UINT       flg_pending; /* S2 event flags the next tk_wai_flg returns */
static ER         tk_wai_flg(ID obj_id, UINT bits, UINT mode, UINT * p_output,
                             int timeout)
{
    (void)obj_id;
    (void)mode;
    (void)timeout;
    if (0U == (flg_pending & bits))
    {
        return E_TMOUT;
    }
    *p_output = flg_pending & bits;
    flg_pending &= ~bits;
    return E_OK;
}
static ER tk_clr_flg(ID obj_id, UINT bits)
{
    (void)obj_id;
    (void)bits;
    return E_OK;
}
static void app_check(ER result, const char * p_what)
{
    (void)p_what;
    if (result < E_OK)
    {
        status_errors++;
    }
}
ER s2_get_status(motion_status_t * p_output)
{
    *p_output = status;
    return status_result;
}
uint32_t s2_stop_epoch(void)
{
    return stop_epoch;
}
ER s2_cmd_stop(void)
{
    return E_OK;
}
static int32_t last_drive_speed, last_drive_steer, drives;
ER             s2_cmd_drive(int32_t speed, int32_t steer)
{
    last_drive_speed = speed;
    last_drive_steer = steer;
    drives++;
    return E_OK;
}
static int32_t last_turn_angle;
ER             s2_cmd_turn(int32_t speed, int32_t angle)
{
    (void)speed;
    last_turn_angle = angle;
    return E_OK;
}
ER s2_cmd_drive_dist(int32_t speed, int32_t distance)
{
    (void)speed;
    (void)distance;
    return E_OK;
}
ER s2_cmd_reset(void)
{
    reset_count++;
    return E_OK;
}
ER s2_emergency_stop(void)
{
    if (stop_epoch != UINT32_MAX)
    {
        stop_epoch++;
    }
    return E_OK;
}
/* Stand-in for S2's IMU stop record: the cause applies to one generation */
static uint32_t imu_cause, imu_cause_epoch;
uint32_t        s2_imu_stop_cause(uint32_t epoch)
{
    return (0U != g_s2_imu_stop)
               ? g_s2_imu_stop
               : ((epoch == imu_cause_epoch) ? imu_cause : 0U);
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
    status.state      = MSTATE_SAFE_STOP;
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
    status.state  = MSTATE_IDLE;
    g_s2_imu_stop = 1;
    assert(!vc_bridge_s2_next(&event)); /* interlock still asserted */
    g_s2_imu_stop = 0;
    stop_epoch    = 2;
    assert(!vc_bridge_s2_next(&event)); /* newer stop not yet in snapshot */
    status.stop_epoch = 2;
    assert(vc_bridge_s2_next(&event) && event.type == VC_EV_READY);
    assert(!vc_bridge_s2_next(&event));

    status.state      = MSTATE_SAFE_STOP;
    status.stop_epoch = stop_epoch = 3;
    assert(vc_bridge_s2_next(&event) && event.type == VC_EV_FAULT);
    assert(!vc_bridge_s2_next(&event)); /* no repeated fault for same stop */
    status.stop_epoch = stop_epoch = 4;
    assert(vc_bridge_s2_next(&event) && event.type == VC_EV_FAULT);

    status.state      = MSTATE_IDLE;
    status.stop_epoch = stop_epoch = UINT32_MAX;
    assert(vc_bridge_s2_request(VC_REQ_SELF_CHECK, 0) == E_OK);
    assert(!vc_bridge_s2_next(&event)); /* saturated generation fails closed */
    assert(vc_bridge_s2_move(VC_MOVE_SAFE_STOP, 0) == E_OK);
    status.stop_epoch = stop_epoch = 0;
    assert(!vc_bridge_s2_next(&event)); /* aborted self-check stays cancelled */

    /* encoder-based turn requests: the bridge passes the angle to S2 */
    assert(vc_bridge_s2_move(VC_MOVE_U_TURN, 0) == E_OK &&
           last_turn_angle == VC_UTURN_CMD_DEG);
    assert(VC_UTURN_CMD_DEG == 137); /* owner: 141 gave ~185 deg; 141 x 180 /
                                        185 */
    assert(vc_bridge_s2_move(VC_MOVE_TURN_LEFT, 90) == E_OK &&
           last_turn_angle == 90);
    assert(vc_bridge_s2_move(VC_MOVE_TURN_RIGHT, 90) == E_OK &&
           last_turn_angle == -90);
    puts("[TEST] S6-U01 PASS  U-turn asks S2 for VC_UTURN_CMD_DEG (137, tuned "
         "on the robot); left +90, right -90 unchanged");

    /* U-turn recovery: the fast part asks for its own angle, the slow part
       pivots by wheel speed until the controller stops it */
    assert(vc_bridge_s2_move(VC_MOVE_U_TURN, 97) == E_OK &&
           last_turn_angle == 97);
    drives = 0;
    assert(vc_bridge_s2_move(VC_MOVE_PIVOT, 60) == E_OK && drives == 1 &&
           last_drive_speed == 0 && last_drive_steer == 60);
    assert(vc_bridge_s2_move(VC_MOVE_PIVOT, -60) == E_OK &&
           last_drive_speed == 0 && last_drive_steer == -60);
    assert(vc_bridge_s2_move(VC_MOVE_PIVOT, 5000) == E_OK &&
           last_drive_steer == VC_TURN_SPEED_MMS);
    assert(vc_bridge_s2_move(VC_MOVE_PIVOT, -5000) == E_OK &&
           last_drive_steer == -VC_TURN_SPEED_MMS);
    /* SET_SPEED must not turn a pivot back into a forward drive */
    drives = 0;
    assert(vc_bridge_s2_move(VC_MOVE_SET_SPEED, 120) == E_OK && drives == 0);
    puts("[TEST] S6-U02 PASS  U_TURN carries its angle (0 = VC_UTURN_CMD_DEG); "
         "PIVOT = drive(0, +-v), clamped to the turn speed, not resumed by "
         "SET_SPEED");

    /* the controller logs how far the wheels ran on after a stop request */
    status.state      = MSTATE_IDLE;
    status.stop_epoch = stop_epoch;
    g_s2_odo_ticks    = 4321U;
    flg_pending       = MFLG_CMD_DONE;
    assert(vc_bridge_s2_next(&event) && event.type == VC_EV_MANOEUVRE_DONE &&
           event.p1 == 1 && event.p2 == 4321);
    assert(vc_bridge_s2_move(VC_MOVE_STOP_CONFIRM, 0) == E_OK);
    g_s2_odo_ticks = 4400U;
    flg_pending    = MFLG_STOPPED;
    assert(vc_bridge_s2_next(&event) && event.type == VC_EV_MOTION_STOPPED &&
           event.p2 == 4400);
    flg_pending = MFLG_STOPPED; /* not asked for: not forwarded */
    assert(!vc_bridge_s2_next(&event));
    puts("[TEST] S6-U03 PASS  MANOEUVRE_DONE and MOTION_STOPPED carry S2's "
         "odometer in p2");

    puts("[TEST] S6-SAFE03 PASS  motion readiness follows applied stop and "
         "reset");

    /* The safe-stop report says which IMU interlock raised it */
    status.state      = MSTATE_SAFE_STOP;
    status.stop_epoch = stop_epoch = 10;
    g_s2_imu_stop                  = S2_IMU_CAUSE_LIFT;
    assert(vc_bridge_s2_next(&event) && event.type == VC_EV_FAULT &&
           event.p2 == VC_FC_S2_SAFE_STOP && event.cause == VC_CAUSE_IMU_LIFT);
    assert(!vc_bridge_s2_next(&event)); /* still one report per stop */
    status.stop_epoch = stop_epoch = 11;
    g_s2_imu_stop                  = S2_IMU_CAUSE_OVERTURN;
    assert(vc_bridge_s2_next(&event) && event.cause == VC_CAUSE_IMU_OVERTURN);
    /* set down before the controller read the stop: the record still
       explains exactly that stop generation */
    status.stop_epoch = stop_epoch = 12;
    g_s2_imu_stop                  = 0;
    imu_cause                      = S2_IMU_CAUSE_LIFT;
    imu_cause_epoch                = 12;
    assert(vc_bridge_s2_next(&event) && event.cause == VC_CAUSE_IMU_LIFT);
    /* a later, unrelated stop has no IMU cause: genuine 0x2011 */
    status.stop_epoch = stop_epoch = 13;
    assert(vc_bridge_s2_next(&event) && event.p2 == VC_FC_S2_SAFE_STOP &&
           event.cause == VC_CAUSE_NONE);
    /* other events never carry a cause */
    status.state = MSTATE_IDLE;
    assert(vc_bridge_s2_move(VC_MOVE_RESET, 0) == E_OK);
    puts("[TEST] S6-SAFE04 PASS  S2 safe-stop report carries the IMU "
         "lift/overturn cause of that stop only");
    return 0;
}

/*** end of file ***/
