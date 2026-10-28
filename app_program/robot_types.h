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

typedef struct
{
    int32_t left;
    int32_t centre;
    int32_t right;
} ir_reading_t;

#endif /* ROBOT_TYPES_H */

/*** end of file ***/
