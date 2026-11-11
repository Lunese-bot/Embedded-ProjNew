/*
 *      robot_types.h
 *      Line, barcode and navigation types shared by subsystems 3 and 6
 *      (Design Review 6.2).  Plain C: no kernel types, so the Vehicle
 *      Controller state machine can also be built on a PC (host_test).
 */

#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>

typedef enum
{
    LINE_STATE_INIT,
    LINE_STATE_CALIBRATE,
    LINE_STATE_SEARCH,
    LINE_STATE_FOLLOW,
    LINE_STATE_LOST,
    LINE_STATE_JUNCTION
} line_state_t;

typedef enum
{
    BARCODE_STATE_IDLE,
    BARCODE_STATE_DETECT,
    BARCODE_STATE_READ,
    BARCODE_STATE_DECODE,
    BARCODE_STATE_RETRY,
    BARCODE_STATE_COMPLETE
} barcode_state_t;

typedef enum
{
    NAV_NONE,
    NAV_LEFT,
    NAV_RIGHT,
    NAV_STRAIGHT,
    NAV_U_TURN
} navigation_command_t;

/* Distance from the wheel axle to the row of IR line sensors, in mm
   (sensors are AHEAD of the axle).  MEASURE with a ruler and set it here.
   Why it matters (review 2026-11-10): the robot pivots about the axle, so a
   turn made with the SENSORS on a line leaves the robot beside that line,
   and with the straddle layout "both white" means straight, so it would
   drive on parallel to the line and never find it.  Used to drive forward
   this far before the 90 degree turn at a junction (vc_sm.c) and before
   turning back onto the line after an obstacle (task_obstacle_avoid.c).
   0 = old behaviour (turn at once). */
#ifndef ROBOT_IR_TO_AXLE_MM
#define ROBOT_IR_TO_AXLE_MM 0
#endif

typedef struct
{
    int32_t left;
    int32_t centre;
    int32_t right;
} ir_reading_t;

#endif /* ROBOT_TYPES_H */

/*** end of file ***/
