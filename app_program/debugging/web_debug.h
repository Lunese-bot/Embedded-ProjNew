/*
 *      web_debug.h
 *      Browser remote control for BENCH DEBUGGING (not for race runs).
 *
 *      Plain C only: web_debug.c runs inside the WiFi service task and, like
 *      mqtt_utk.c, must not include tk/tkernel.h.  The motion commands are
 *      issued through web_debug_bridge.h, which can.
 *
 *      Switch it on in app_program/subsystem2/s2_config.h:
 *          #define S2_WEBDEBUG 1
 *      and build with NET=lwip (needs WiFi credentials in
 *      config/wifi_secrets.h).  See debugging/README.md.
 */

#ifndef WEB_DEBUG_H
#define WEB_DEBUG_H

#include "../subsystem2/s2_config.h" /* S2_WEBDEBUG (macros only) */

#if S2_WEBDEBUG
/* Call from cyw43_utk_task() once the WiFi link is up.  Idempotent. */
void web_debug_start(void);
#endif

#endif /* WEB_DEBUG_H */

/*** end of file ***/
