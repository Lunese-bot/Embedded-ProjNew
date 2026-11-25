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
     'Q' servo left  'W' servo centre  'E' servo right  'e' obstacle monitoring
   on In modes F and N the motion buttons are locked (reply ERR). Returns 0 on
   success, -1 for an unknown command or a full queue. */
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

/* GET /b: the last barcode read for the dashboard's barcode recorder, as
   {"seq":..,"n":..,"err":..,"txt":"..","w":[..]} with widths in 0.1 ms
   (element 0 = black bar, then alternating). */
int32_t web_debug_barcode(char * p_out, int32_t out_max);

/* GET /r?p=<page>: the barcode sensor's raw readings (10 ms apart, oldest
   first), its black / white switch levels and the wheel steps, at most
   WD_LT_PAGE samples per reply (S3_LTRACE_N = 1400 does not fit one
   WD_SNAP_MAX reply):
   {"on":..,"off":..,"frz":..,"edge":..,"sw":..,"n":total,"p":page,
    "q":read seq,"v":[..],"f":[..],"y":[..]}
   live 0: the copy frozen when the last read ended, paged; the dashboard
   asks page 0, 1, ... until it has n samples, all with the same q.
   live 1 (GET /r?l): the newest WD_LT_PAGE samples, page ignored. */
#define WD_LT_PAGE 700u
int32_t web_debug_ltrace(char * p_out, int32_t out_max, int32_t live,
                         int32_t page);

/* GET /u?p=<page>: the IMU recorder, WD_IMU_PAGE samples per page, oldest
   first: {"n":total,"p":page,"d":[ax,ay,az,pitch,state,safety,speed,
   liftv, ...]}.  Page 0 freezes the recorder, the last page releases it. */
#define WD_IMU_PAGE 160
int32_t web_debug_imutrace(char * p_out, int32_t out_max, int32_t page);

#endif /* WEB_DEBUG_BRIDGE_H */

/*** end of file ***/
