
// #ifndef IMU_EVENT_WIRE_H
// #define IMU_EVENT_WIRE_H

// #include <stdint.h>

// /* Header bit layout:
//  * Bits 0-3: event type (1-12)
//  * Bits 4-5: severity (0-3)
//  * Bits 6-7: reserved (zero)
//  */
// #define IMU_WIRE_TYPE_MASK       (0x0FU)
// #define IMU_WIRE_SEVERITY_MASK   (0x30U)
// #define IMU_WIRE_SEVERITY_SHIFT  (4U)

// /* Compact, fixed-layout event message.
//  * Multi-byte values will use little-endian encoding.
//  */
// typedef struct
// {
//     uint8_t header;
//     uint8_t seq[2];
//     uint8_t value[4];
//     uint8_t time_ms[4];
// } imu_event_wire_t;

// _Static_assert(sizeof(imu_event_wire_t) == 11U,
//                "IMU wire message must be 11 bytes");

// #endif /* IMU_EVENT_WIRE_H */
