/*
 *      web_debug_bridge.h
 *      Plain-C hand-over from the web server (WiFi task) to subsystem 2.
 */

#ifndef WEB_DEBUG_BRIDGE_H
#define WEB_DEBUG_BRIDGE_H

#include <stdint.h>

/* Run one button command and write a one-line text status into p_out
   (NUL-terminated, at most out_max bytes).  Never blocks.
     'f' forward   'b' back   'l' spin left   'r' spin right
     's' stop      'x' reset after a fault/safe stop
     'L' turn 90 deg left   'R' turn 90 deg right
     'M' manual mode   'F' line-follow mission   'N' monitor only
     'Q' servo left  'W' servo centre  'E' servo right  'e' obstacle monitoring on
   In modes F and N the motion buttons are locked (reply ERR).
   Returns 0 on success, -1 for an unknown command or a full queue. */
int32_t web_debug_motion(char cmd, uint32_t client, char * p_out,
                         int32_t out_max);

/* Write one JSON object with the status of every subsystem (motion, line
   sensors, IMU, obstacle, vehicle controller) plus the live-tuning values.
   Subsystems that are not running are null.  Never blocks long. */
/* client = the browser tab's id (for the who-controls-what locks);
   p_x = which sections to include: m motor, l line, i imu, o obstacle,
   v vehicle controller (config + lock flags are always sent). */
int32_t web_debug_snapshot(char * p_out, int32_t out_max, uint32_t client,
                           const char * p_x);

#endif /* WEB_DEBUG_BRIDGE_H */

/*** end of file ***/
