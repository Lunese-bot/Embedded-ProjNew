/*
 *      app.h
 *      Definitions shared by the whole team.  Changed only by team agreement.
 *
 *      Anything that belongs to ONE subsystem goes in that subsystem's own
 *      header instead (app_program/subsystemN/subsystemN.h).
 */

#ifndef APP_H
#define APP_H

#include <tk/tkernel.h>  /* micro T-Kernel API (tk_xxx_yyy) */
#include <tm/tmonitor.h> /* T-Monitor API: tm_printf() console output */
#include <bsp/libbsp.h>  /* Board helpers: gpio_set_pin/val, pwm_... */
#include <stdint.h>      /* fixed-width integer types (BARR-C 5.2) */
#include <stdbool.h>     /* bool, true, false (BARR-C 5.6) */

/* Floating-point type names (BARR-C 5.4).  C99 does not define them. */
typedef float  float32_t;
typedef double float64_t;

/* ---------------------------------------------------------------------
 *  Pins: free for projects are GP0..GP22 and GP26..GP28.  GP0/GP1 are
 *  taken by UART0 when built with "make CRASH_REPORT=1" (see README).
 *  Never use GP23/24/25/29: they are wired to the Pico W radio.
 *  Claim every pin in TEAM.md section 2 before using it.
 * --------------------------------------------------------------------- */

/* ---------------------------------------------------------------------
 *  Task priorities: 1 is the highest; the kernel's USB console (3) and
 *  WiFi (4) tasks sit above every application task.  Each subsystem
 *  has its own band (TEAM.md section 4), defined in its own header.
 * --------------------------------------------------------------------- */
#define TASK_STACK_SIZE 2048 /* bytes; tm_printf needs ~1 KB */

/* Create and start a task.  prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or
   0 for "any core".  Defined in app_main.c. */
extern ID app_start_task(FP entry, PRI pri, uint32_t prc, const char * p_name);

/* ---------------------------------------------------------------------
 *  Every subsystem provides these.  usermain() calls ALL the _init()
 *  functions first and only then ALL the _start() functions, so every
 *  kernel object exists before any task can use it.
 *    subsystemN_init()   create this subsystem's kernel objects
 *    subsystemN_start()  create and start this subsystem's tasks
 *    subsystemN_test()   on-target tests, only in test builds (make TEST=1)
 * --------------------------------------------------------------------- */
extern ER subsystem1_init(void);
extern ER subsystem2_init(void);
extern ER subsystem3_init(void);
extern ER subsystem4_init(void);
extern ER subsystem5_init(void);
extern ER subsystem6_init(void); /* Vehicle Controller */

extern ER subsystem1_start(void);
extern ER subsystem2_start(void);
extern ER subsystem3_start(void);
extern ER subsystem4_start(void);
extern ER subsystem5_start(void);
extern ER subsystem6_start(void);

#if APP_TEST
extern void subsystem1_test(void);
extern void subsystem2_test(void);
extern void subsystem3_test(void);
extern void subsystem4_test(void);
extern void subsystem5_test(void);
extern void subsystem6_test(void);
#endif

/* Print a readable message if a kernel call failed (negative error code).
   p_what names the call.  Defined in app_main.c. */
extern void app_check(int32_t ercd, const char * p_what);

#endif /* APP_H */

/*** end of file ***/
