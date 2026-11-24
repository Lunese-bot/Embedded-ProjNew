/* Exercise the production command path without hardware or a running kernel. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define APP_H
#define __TK_SMP_H__
#ifndef TK_SUPPORT_SMP
#define TK_SUPPORT_SMP 1
#endif
typedef uint32_t UW;
typedef unsigned int UINT;
typedef int ID;
typedef int ER;
typedef void (*FP)();
typedef float float32_t;
typedef struct { uint32_t lo; } SYSTIM;
typedef struct { int mbfatr; size_t bufsz, maxmsz; } T_CMBF;
typedef struct { int mtxatr; } T_CMTX;
typedef struct { int flgatr; UINT iflgptn; } T_CFLG;
typedef struct { int cycatr; FP cychdr; int cyctim, cycphs; } T_CCYC;
extern ER s2_emergency_stop(void);
#define E_OK 0
#define E_PAR (-1)
#define E_TMOUT (-2)
#define E_LIMIT (-34)
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TA_TFIFO 0
#define TA_INHERIT 0
#define TA_WMUL 0
#define TA_HLNG 0
#define TWF_ORW 1
#define TWF_BITCLR 2
#define TP_PRC2 2
#define TSK_SELF 0
#define DI(intsts) ((intsts) = 0)
#define EI(intsts) ((void)(intsts))

static UW atomic_add(UW *p, UW n) { return (*p += n); }
static UW atomic_inc(UW *p) { return ++*p; }
static int concurrent_stop;
static UW atomic_cmpxchg(UW *p, UW value, UW expected)
{
    UW old;
    if (concurrent_stop)
    {
        concurrent_stop = 0;
        assert(s2_emergency_stop() == E_OK);
    }
    old = *p;
    if (old == expected) *p = value;
    return old;
}

static ER tk_set_flg(ID id, UINT bits);
static ER tk_wai_flg(ID id, UINT bits, UINT mode, UINT *out, int timeout);
static ER tk_snd_mbf(ID id, const void *msg, size_t size, int timeout);
static int tk_rcv_mbf(ID id, void *msg, int timeout);
static ER tk_wup_tsk(ID id);
static ER tk_get_tim(SYSTIM *time);
static ER tk_loc_mtx(ID id, int timeout);
static ER tk_unl_mtx(ID id);
static ID tk_cre_mbf(T_CMBF *p);
static ID tk_cre_mtx(T_CMTX *p);
static ID tk_cre_flg(T_CFLG *p);
static ID tk_cre_cyc(T_CCYC *p);
static ER tk_sta_cyc(ID id);
static int tk_get_prc(void);
static ER tk_slp_tsk(int timeout);
static int tk_can_wup(ID id);
static ID app_start_task(FP entry, int pri, uint32_t prc, const char *name);
static void app_check(int32_t ercd, const char *what);
static int tm_printf(const uint8_t *format, ...);

#include "../../../app_program/subsystem2/subsystem2.c"
#include "../../../app_program/subsystem2/task_motion.c"
#include "../../../app_program/subsystem2/motion_pid.c"

static motion_cmd_t queue[S2_CMD_QUEUE_LEN];
static int queued, receive_count, stop_on_receive, stop_on_send;
static UINT flags;
static int motor_off_calls, motor_set_calls;
volatile int32_t g_lt_ilim, g_lt_ff_r, g_lt_ff_l, g_lt_pid_ki100;
volatile int32_t g_lt_pid_kp100, g_lt_dmin;

static void clear_fixture(void)
{
    memset(&g_motion_ctx, 0, sizeof(g_motion_ctx));
    g_motion_ctx.state = MSTATE_IDLE;
    g_stop_epoch = g_handled_stop_epoch = g_s2_seq = 0;
    g_s2_imu_stop = 0;
    g_imu_stop_cause = g_imu_stop_epoch = 0;
    queued = receive_count = stop_on_receive = stop_on_send = 0;
    concurrent_stop = 0;
    flags = 0;
    motor_off_calls = motor_set_calls = 0;
}

static ER tk_set_flg(ID id, UINT bits) { (void)id; flags |= bits; return E_OK; }
static ER tk_wai_flg(ID id, UINT bits, UINT mode, UINT *out, int timeout)
{
    (void)id; (void)mode; (void)timeout;
    *out = flags & bits;
    flags &= ~bits;
    return *out ? E_OK : E_TMOUT;
}
static ER tk_snd_mbf(ID id, const void *msg, size_t size, int timeout)
{
    (void)id; (void)timeout;
    assert(size == sizeof(motion_cmd_t));
    if (stop_on_send)
    {
        stop_on_send = 0;
        assert(s2_emergency_stop() == E_OK);
    }
    if (queued == S2_CMD_QUEUE_LEN) return E_TMOUT;
    queue[queued++] = *(const motion_cmd_t *)msg;
    return E_OK;
}
static int tk_rcv_mbf(ID id, void *msg, int timeout)
{
    (void)id; (void)timeout;
    if (!queued) return E_TMOUT;
    *(motion_cmd_t *)msg = queue[0];
    memmove(queue, queue + 1, (size_t)(--queued) * sizeof(queue[0]));
    if (++receive_count == stop_on_receive) assert(s2_emergency_stop() == E_OK);
    return sizeof(motion_cmd_t);
}
static ER tk_wup_tsk(ID id) { (void)id; return E_OK; }
static ER tk_get_tim(SYSTIM *time) { time->lo = 0; return E_OK; }
static ER tk_loc_mtx(ID id, int timeout) { (void)id; (void)timeout; return E_OK; }
static ER tk_unl_mtx(ID id) { (void)id; return E_OK; }
static ID tk_cre_mbf(T_CMBF *p) { (void)p; return 1; }
static ID tk_cre_mtx(T_CMTX *p) { (void)p; return 1; }
static ID tk_cre_flg(T_CFLG *p) { (void)p; return 1; }
static ID tk_cre_cyc(T_CCYC *p) { (void)p; return 1; }
static ER tk_sta_cyc(ID id) { (void)id; return E_OK; }
static int tk_get_prc(void) { return 1; }
static ER tk_slp_tsk(int timeout) { (void)timeout; return E_OK; }
static int tk_can_wup(ID id) { (void)id; return 0; }
static ID app_start_task(FP entry, int pri, uint32_t prc, const char *name)
{ (void)entry; (void)pri; (void)prc; (void)name; return 1; }
static void app_check(int32_t ercd, const char *what)
{ (void)what; assert(ercd == E_OK); }
static int tm_printf(const uint8_t *format, ...) { (void)format; return 0; }
void drv_motor_brake(motor_id_t motor) { (void)motor; }
void drv_motor_all_off(void) { motor_off_calls++; }
void drv_motor_set(motor_id_t motor, int32_t duty)
{ (void)motor; (void)duty; motor_set_calls++; }
ER drv_motor_init(void) { return E_OK; }
ER drv_encoder_init(void) { return E_OK; }
uint32_t drv_encoder_delta(motor_id_t motor) { (void)motor; return 0; }

int main(void)
{
    int i;
    motion_cmd_t direct = {0};

    clear_fixture();
    assert(s2_cmd_reset() == E_OK);
    assert(s2_cmd_drive(100, 0) == E_OK);
    for (i = 2; i < S2_CMD_QUEUE_LEN; i++) assert(s2_cmd_drive(100, 0) == E_OK);
    assert(s2_emergency_stop() == E_OK);
    receive_commands();
    assert(g_motion_ctx.state == MSTATE_SAFE_STOP && motor_off_calls > 0);
    assert(s2_cmd_reset() == E_OK);
    receive_commands();
    assert(g_motion_ctx.state == MSTATE_IDLE);
    assert(s2_cmd_drive(100, 0) == E_OK);
    receive_commands();
    assert(g_motion_ctx.state == MSTATE_DRIVING);

    clear_fixture();
    assert(s2_emergency_stop() == E_OK);
    assert(s2_cmd_reset() == E_OK);
    receive_commands();
    assert(g_motion_ctx.state == MSTATE_IDLE);
    flags |= MFLG_ESTOP_REQ; /* a delayed notification of the same stop */
    receive_commands();
    assert(g_motion_ctx.state == MSTATE_IDLE);

    clear_fixture();
    assert(s2_cmd_reset() == E_OK);
    assert(s2_cmd_drive(100, 0) == E_OK);
    stop_on_receive = 2;
    receive_commands();
    assert(g_motion_ctx.state == MSTATE_SAFE_STOP);

#if TK_SUPPORT_SMP
    clear_fixture();
    concurrent_stop = 1;
    assert(s2_emergency_stop() == E_OK);
    assert(g_stop_epoch == 2);
    receive_commands();
    assert(g_motion_ctx.state == MSTATE_SAFE_STOP);
#endif

    clear_fixture();
    stop_on_send = 1;
    assert(s2_cmd_reset() == E_OK);
    receive_commands();
    assert(g_motion_ctx.state == MSTATE_SAFE_STOP);

    clear_fixture();
    assert(s2_emergency_stop() == E_OK);
    assert(s2_cmd_reset() == E_OK);
    assert(s2_cmd_drive(100, 0) == E_OK);
    assert(s2_emergency_stop() == E_OK);
    receive_commands();
    assert(g_motion_ctx.state == MSTATE_SAFE_STOP);

    clear_fixture();
    direct.type = MOTION_SAFE_STOP;
    assert(s2_send_cmd(&direct) == E_OK && queued == 0);
    receive_commands();
    assert(g_motion_ctx.state == MSTATE_SAFE_STOP);

    clear_fixture();
    s2_latch_imu_stop(S2_IMU_CAUSE_LIFT);
    assert(s2_cmd_reset() == E_OK);
    receive_commands();
    assert(g_motion_ctx.state == MSTATE_SAFE_STOP);
    /* the stop S2 now reports is explained as the lift */
    assert(s2_imu_stop_cause(g_handled_stop_epoch) == S2_IMU_CAUSE_LIFT);
    s2_clear_imu_stop();
    /* set down: the record still explains that one stop, no other */
    assert(s2_imu_stop_cause(g_handled_stop_epoch) == S2_IMU_CAUSE_LIFT);
    assert(s2_imu_stop_cause(g_handled_stop_epoch + 1U) == S2_IMU_CAUSE_NONE);
    assert(s2_cmd_reset() == E_OK);
    receive_commands();
    assert(g_motion_ctx.state == MSTATE_IDLE);
    /* a later stop that is not the IMU's has no IMU cause */
    assert(s2_emergency_stop() == E_OK);
    receive_commands();
    assert(g_motion_ctx.state == MSTATE_SAFE_STOP);
    assert(s2_imu_stop_cause(g_handled_stop_epoch) == S2_IMU_CAUSE_NONE);

    clear_fixture();
    s2_latch_imu_stop(S2_IMU_CAUSE_OVERTURN);
    s2_latch_imu_stop(S2_IMU_CAUSE_LIFT); /* already latched: kept */
    receive_commands();
    assert(g_motion_ctx.state == MSTATE_SAFE_STOP);
    assert(s2_imu_stop_cause(g_handled_stop_epoch) == S2_IMU_CAUSE_OVERTURN);
    s2_clear_imu_stop();

    clear_fixture();
    s2_latch_imu_stop(7U); /* not a known IMU cause: stops, never a lift */
    receive_commands();
    assert(g_motion_ctx.state == MSTATE_SAFE_STOP);
    assert(s2_imu_stop_cause(g_handled_stop_epoch) == S2_IMU_CAUSE_UNKNOWN);
    s2_clear_imu_stop();

    clear_fixture();
    g_motion_ctx.state = MSTATE_DRIVING;
    assert(s2_emergency_stop() == E_OK);
    drive_motor();
    assert(g_motion_ctx.state == MSTATE_SAFE_STOP && motor_set_calls == 0);

    clear_fixture();
    g_stop_epoch = UINT32_MAX - 1U;
    g_handled_stop_epoch = UINT32_MAX - 1U;
    assert(s2_emergency_stop() == E_OK);
    assert(s2_cmd_reset() == E_LIMIT);
    assert(s2_emergency_stop() == E_OK);
    receive_commands();
    assert(g_stop_epoch == UINT32_MAX && g_motion_ctx.state == MSTATE_SAFE_STOP);

    puts("S2-U11 emergency-stop ordering: PASS");
    puts("S2-U12 IMU stop cause (lift / overturn / unknown) kept per stop: PASS");
    return 0;
}
