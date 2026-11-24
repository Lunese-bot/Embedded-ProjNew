/* Host stand-in for tm/tmonitor.h. */
#ifndef FAKE_TM_TMONITOR_H
#define FAKE_TM_TMONITOR_H
typedef unsigned char UB;
int tm_printf(const UB * format, ...);
#endif
