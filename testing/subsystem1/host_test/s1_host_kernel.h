/*
 *      s1_host_kernel.h
 *      Host stand-ins for the micro T-Kernel calls used by
 *      app_program/subsystem1/subsystem1.c and s1_bridge.c, so host tests
 *      run the real S1 implementation (s1_host_kernel.c compiles both).
 *
 *      The message buffer copies the kernel's own accounting
 *      (kernel/tkernel/messagebuf.c): each message costs a 4-byte header
 *      plus its length rounded up to 4 bytes.
 */

#ifndef S1_HOST_KERNEL_H
#define S1_HOST_KERNEL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define APP_H                 /* skip ../app.h (kernel headers) */
#define VEHICLE_CONTROLLER_H  /* skip the controller header */
#define __TM_TMONITOR_H__
#ifndef APP_TEST
#define APP_TEST 0
#endif

typedef int ID;
typedef int ER;
typedef int vc_event_type_t;
typedef unsigned char UB;
typedef void (*FP)();
typedef struct { int mbfatr; size_t bufsz, maxmsz; } T_CMBF;
typedef struct { int cycatr; FP cychdr; int cyctim, cycphs; } T_CCYC;
typedef struct { uint32_t lo; } SYSTIM;
#define E_OK    0
#define E_PAR   (-17)
#define E_TMOUT (-50)
#define TMO_POL 0
#define TA_TFIFO 0
#define TA_HLNG 0
#define TA_STA 0
enum { VC_SRC_COMMS, VC_EV_CMD_START, VC_EV_CMD_STOP, VC_EV_CMD_RESET,
       VC_EV_CMD_PAUSE, VC_EV_CMD_RESUME, VC_EV_WIFI_UP, VC_EV_WIFI_DOWN,
       VC_EV_CMD_CONFIG };

ID tk_cre_mbf(T_CMBF * p_cmbf);
ER tk_snd_mbf(ID id, const void * p_data, int len, int timeout);
int32_t tk_rcv_mbf(ID id, void * p_data, int timeout);
ER tk_wup_tsk(ID id);
ID tk_cre_cyc(T_CCYC * p_ccyc);
ID app_start_task(FP entry, int pri, uint32_t prc, const char * p_name);
void app_check(int32_t ercd, const char * p_what);
ER vc_post_event(int src, int type, int p1, int p2, int timeout);
ER tk_get_tim(SYSTIM * p_time);
ER tk_dly_tsk(int delay);
int tm_printf(const UB * p_format, ...);
void telemetry_task(int32_t stacd, void * p_exinf);

/* ---- test controls and observations ----------------------------------- */
#define HOST_MBF_MAX 4

/* Remove every queued message and reset the counters below and S1's own
   state, as if the board had just been powered on. */
ER   subsystem1_init(void); /* app.h declares it on target */
void host_kernel_reset(void);

extern int32_t host_mbf_used[HOST_MBF_MAX];  /* bytes per buffer */
extern int32_t host_mbf_count[HOST_MBF_MAX]; /* messages per buffer */
extern int32_t host_vc_calls;
extern int32_t host_vc_result;              /* returned by vc_post_event */
extern int32_t host_app_check_failures;     /* app_check() with ercd < 0 */
extern char    host_last_printf[512];
/* s1_latch_read(): what the target would read from S2 and S4 */
extern uint32_t host_latch_stop_before; /* g_s2_imu_stop, 1st read */
extern uint32_t host_latch_stop_after;  /* g_s2_imu_stop, 2nd read */
extern int32_t  host_latch_status_ok;   /* s4_get_status() == E_OK */
extern uint32_t host_latch_safety;      /* imu_status_t.safety_stop */

#endif /* S1_HOST_KERNEL_H */

/*** end of file ***/
