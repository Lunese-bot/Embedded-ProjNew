/*
 *      s5_scan_diag.h
 *      Subsystem 5 -- scan evidence: the raw pings behind every scan
 *      sample, kept for the USB "s5d:" record.  Plain C, no kernel calls
 *      (host-tested in testing/subsystem5/host_test/test_scan_diag.c).
 *
 *      Each sampled angle keeps the S5_TRIP_READS pings the scan already
 *      takes (0 = no echo), where it came from (coarse or fine sweep) and
 *      the median the profile actually used.  Nothing here takes a
 *      reading or changes a profile.
 */

#ifndef S5_SCAN_DIAG_H
#define S5_SCAN_DIAG_H

#include "obstacle_logic.h"

/* 1 = the scan task records the pings and prints the "s5d:" record after
   it has handed the profile over.  0 = no record (same profile and
   bypass either way: the pings and the median are the same). */
#ifndef S5_SCAN_DIAG
#define S5_SCAN_DIAG 1
#endif

#define S5_TRIP_READS 3 /* pings per sampled angle (median of 3) */

typedef enum
{
    S5_SRC_COARSE = 1,
    S5_SRC_FINE   = 2
} s5_src_t;

typedef struct
{
    int8_t   deg;                 /* servo angle, + = left              */
    uint8_t  src;                 /* s5_src_t                           */
    uint16_t raw[S5_TRIP_READS];  /* each ping in order, 0 = no echo    */
    uint16_t used;                /* median used for the profile        */
} s5_trip_t;

/* Median of the n (1..5) readings, ignoring no-echo (0) ones; 0 when all
   are missing.  With an even number of echoes the upper middle one.
   This is the one implementation: us_read_median_mm uses it too. */
extern uint16_t s5_median_nonzero(const uint16_t * p_raw, int32_t n);

/* The scan samples (deg, used) of trips. */
extern void s5_trips_to_samples(const s5_trip_t * p_trips, int32_t n,
                                s5_sample_t * p_out);

/* Same order and rule as s5_merge_samples: sorted by angle, the fine trip
   replaces the coarse one at the same angle (its pings come with it). */
extern int32_t s5_merge_trips(const s5_trip_t * a, int32_t na,
                              const s5_trip_t * b, int32_t nb,
                              s5_trip_t * out, int32_t max);

/* Trips [first, first + n) as " C-60:0/0/0=0 F-45:0/212/0=212" (source
   C or F, the pings, = the median used).  Whole items only, always NUL
   terminated, never more than size bytes; returns the characters
   written. */
extern int32_t s5_format_trips(char * p_buf, int32_t size,
                               const s5_trip_t * p_trips, int32_t first,
                               int32_t n);

#endif /* S5_SCAN_DIAG_H */
