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

/* ---- Supported-edge width estimate (report only) ------------------
   Runs beside the normal profile and never changes it: side choice,
   clearance, lateral offset, measured pass and line recovery keep using
   the raw edges of s5_build_profile.

   Raw edges: the outermost samples of the profile's cluster (one echo of
   three is enough to be a hit there).  Supported edges: starting from
   each raw edge and moving inward, the first sample of the same cluster
   whose own trip has at least S5_WE_MIN_SUPPORT plausible pings.  A ping
   is plausible when it is a hit (1..S5_HIT_RANGE_MM) within
   S5_CLUSTER_GAP_MM of the range that joined this sample to the cluster
   (the median of its inward neighbour; the nearest sample's own median),
   the rules s5_build_profile already uses.  So one box echo plus one
   distant background echo is not two supporting pings.  A fine trip that
   replaced a coarse one at the same angle brings only its own pings;
   coarse and fine pings are never pooled.  The sideways offset of a
   supported edge from the servo shaft is (median + servo arm) x sin(angle). */
#define S5_WE_MIN_SUPPORT 2

/* 1 = the obstacle size report (MQTT width_mm, USB "s5: obstacle" line)
   carries the span between supported edges.  0 selects the raw width
   used by the bypass planner. */
#ifndef S5_REPORT_SUPPORTED_WIDTH
#define S5_REPORT_SUPPORTED_WIDTH 1
#endif

typedef enum
{
    S5_WE_OK = 0,      /* both supported edges found, left of right      */
    S5_WE_NO_PROFILE,  /* the trips give no obstacle                     */
    S5_WE_CLIPPED,     /* a raw edge is the first or last sample: the
                          side beyond it was never observed              */
    S5_WE_NO_SUPPORT,  /* no sample of the cluster has enough support    */
    S5_WE_COLLAPSED    /* the supported edges meet or cross: too narrow
                          for two supported endpoints                    */
} s5_we_status_t;

typedef struct
{
    uint8_t  status;       /* s5_we_status_t                              */
    /* raw (the profile's own numbers) */
    int8_t   raw_r_deg;
    int8_t   raw_l_deg;
    uint16_t raw_r_range;  /* median used at the edge                    */
    uint16_t raw_l_range;
    int16_t  raw_right_mm;
    int16_t  raw_left_mm;
    uint16_t raw_width_mm;
    uint8_t  raw_r_support; /* plausible pings of the raw edge trips     */
    uint8_t  raw_l_support;
    /* supported; angles and ranges are those found, the mm values and the
       width are 0 unless status is S5_WE_OK or S5_WE_CLIPPED */
    int8_t   sup_r_deg;
    int8_t   sup_l_deg;
    uint16_t sup_r_range;
    uint16_t sup_l_range;
    int16_t  sup_right_mm;
    int16_t  sup_left_mm;
    uint16_t sup_width_mm;
    uint8_t  sup_r_support;
    uint8_t  sup_l_support;
} s5_width_est_t;

/* Build the raw profile from trips (s5_build_profile on their medians)
   and the supported-edge estimate beside it.  Returns the status. */
extern s5_we_status_t s5_width_estimate(const s5_trip_t * p_trips, int32_t n,
                                        s5_width_est_t * p_est);

extern const char * s5_we_name(uint8_t status);

#endif /* S5_SCAN_DIAG_H */
