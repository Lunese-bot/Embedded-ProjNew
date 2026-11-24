/* Host stand-in for lwip/timeouts.h; test_mqtt_utk.c keeps the timer list. */
#ifndef FAKE_LWIP_TIMEOUTS_H
#define FAKE_LWIP_TIMEOUTS_H
#include "lwip/apps/mqtt.h"
typedef void (*sys_timeout_handler)(void * arg);
void sys_timeout(u32_t msecs, sys_timeout_handler handler, void * arg);
void sys_untimeout(sys_timeout_handler handler, void * arg);
#endif
