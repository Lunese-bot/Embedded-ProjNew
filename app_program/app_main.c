/*
 *      app_main.c
 *      START HERE.  usermain() below is your program's entry point.
 *
 *      How micro T-Kernel starts your code
 *      -----------------------------------
 *        reset -> kernel initialises itself -> creates the "initial task"
 *              -> initial task calls usermain()
 *
 *      usermain() runs inside that initial task at priority 1 (the highest).
 *      On a single core nothing you create runs until usermain() blocks, but
 *      on the dual-core build the OTHER core may start a new task at once.
 *      So usermain() works in two phases:
 *        1. every subsystem creates its kernel objects  (subsystemN_init)
 *        2. every subsystem starts its tasks             (subsystemN_start)
 *      and then puts the initial task to sleep forever.
 *      If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *      The robot (Group 10): one subsystem per folder
 *      ------------------------------------------------
 *        subsystem1/  WiFi / MQTT telemetry and remote commands   (Buddy 1)
 *        subsystem2/  Motion control: motors, encoders, PID       (Buddy 2)
 *        subsystem3/  IR line following + barcode navigation     (Buddy 3)
 *        subsystem4/  IMU terrain monitoring (LSM303DLHC, I2C0)   (Buddy 4)
 *        subsystem5/  Ultrasonic obstacle detection + avoidance  (Buddy 5)
 *        subsystem6/  Vehicle Controller: mission state, sole source of
 *                     movement commands (design review 6.1)     (team)
 *
 *      The interfaces between them are listed in TEAM.md section 5.
 *      The subsystem that RECEIVES or OWNS the data creates the object and
 *      publishes its ID in its header; the other side just uses it.
 *
 *      Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"

#if APP_TEST && TM_CONSOLE_USB_CDC
/* USB console state, from the kernel's TinyUSB glue
   (lib/libtm/sysdepend/pico_rp2040/usb).  No kernel header declares it. */
extern int tm_tusb_connected(void);

#define APP_USB_WAIT_POLLS   300 /* 300 x 100 ms = 30 s */
#define APP_USB_WAIT_POLL_MS 100
#define APP_USB_SETTLE_MS    1000
#endif

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
ID app_start_task(FP entry, PRI pri, uint32_t prc, const char * p_name)
{
    T_CTSK ctsk = { 0 };
    ID     tid;
    ER     ercd;

    ctsk.task    = entry;           /* function the task runs */
    ctsk.itskpri = pri;             /* initial priority (1 = highest) */
    ctsk.stksz   = TASK_STACK_SIZE; /* kernel allocates the stack */
    ctsk.tskatr  = TA_HLNG          /* written in a high-level language (C) */
                  | TA_RNG3;        /* runs at user protection level */
#if TK_SUPPORT_SMP
    if (prc != 0)
    {
        ctsk.tskatr |= TA_ASSPRC; /* honour the affinity below */
        ctsk.assprc = prc;        /* which core(s) it may run on */
    }
#else
    (void)prc; /* single-core build: no affinity */
#endif

    /* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
    tid = tk_cre_tsk(&ctsk);
    if (tid < E_OK)
    {
        app_check(tid, p_name);
        return tid;
    }

    /* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
    ercd = tk_sta_tsk(tid, 0);
    app_check(ercd, p_name);
    return tid;
}

/*
 * Print a readable message if a kernel call failed (declared in app.h).
 *   ercd   = the call's return value; negative means an error
 *   p_what = the call's name, for the message
 */
void app_check(int32_t ercd, const char * p_what)
{
    if (ercd < E_OK)
    {
        tm_printf((uint8_t *)"ERROR %s: %d\n", p_what, ercd);
    }
}

/* Kernel entry point: the return type is fixed by micro T-Kernel. */
INT usermain(void)
{
    T_RVER ver;

    /* tk_ref_ver: ask the kernel who it is */
    tk_ref_ver(&ver);
    tm_printf((uint8_t *)"\n=== micro T-Kernel on Raspberry Pi Pico W"
                         " (spec 0x%04x, product 0x%04x) ===\n",
              ver.spver, ver.prver);
#if TK_SUPPORT_SMP
    tm_printf((uint8_t *)"dual-core (SMP) build, usermain on core %d\n",
              tk_get_prc() - 1); /* tk_get_prc() counts from 1 */
#else
    tm_printf((uint8_t *)"single-core build\n");
#endif

    /* Phase 1: every subsystem creates its kernel objects */
    app_check(subsystem1_init(), "subsystem1_init");
    app_check(subsystem2_init(), "subsystem2_init");
    app_check(subsystem3_init(), "subsystem3_init");
    app_check(subsystem4_init(), "subsystem4_init");
    app_check(subsystem5_init(), "subsystem5_init");
    app_check(subsystem6_init(), "subsystem6_init"); /* Vehicle Controller */

    /* Phase 2: every subsystem starts its tasks */
    app_check(subsystem1_start(), "subsystem1_start");
    app_check(subsystem2_start(), "subsystem2_start");
    app_check(subsystem3_start(), "subsystem3_start");
    app_check(subsystem4_start(), "subsystem4_start");
    app_check(subsystem5_start(), "subsystem5_start");
    app_check(subsystem6_start(), "subsystem6_start");

    tm_printf((uint8_t *)"all subsystems started\n\n");

#if APP_TEST
    /* Test build (make TEST=1): run every subsystem's tests in turn,
       here in the initial task, while the system is running.  Tests may
       call tk_dly_tsk() to let the other tasks work for a while. */
#if TM_CONSOLE_USB_CDC
    {
        /* Wait (up to 30 s) until a terminal or tools/dashboard.html
           opens the USB port, so the test results are not printed
           before anyone is listening. */
        int32_t wait_count;

        for (wait_count = 0;
             (wait_count < APP_USB_WAIT_POLLS) && (0 == tm_tusb_connected());
             wait_count++)
        {
            tk_dly_tsk(APP_USB_WAIT_POLL_MS);
        }
        tk_dly_tsk(APP_USB_SETTLE_MS);
    }
#endif
    tm_printf((uint8_t *)"[TEST] === test run start ===\n");
    subsystem1_test();
    subsystem2_test();
    subsystem3_test();
    subsystem4_test();
    subsystem5_test();
    subsystem6_test();
    tm_printf((uint8_t *)"[TEST] === test run end ===\n");
#endif

    /* Sleep forever.  Returning from usermain() would shut down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}

/*** end of file ***/
