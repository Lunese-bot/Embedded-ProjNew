/*
 *      code39_geom.h
 *      Subsystem 3 -- fallback Code 39 reader that works on the raw L
 *      sensor trace instead of the bar/space element list.
 *
 *      When code39_decode_ex and code39_decode_partial both fail, the
 *      element list is usually wrong by one or two elements (a thin gap
 *      blurred into its neighbours, or a bar split in two).  The six WIDE
 *      bars of a one-letter code "*X*" still stand out in the trace, and
 *      where they sit along the 47-module code differs for every letter.
 *      This reader:
 *        1. takes a rolling minimum as the white floor and finds the dark
 *           humps above half the height of the 6th darkest one;
 *        2. for each letter A-Z, both directions and wide:narrow ratios
 *           2.5 / 2.75 / 3.0, fits the 6 wide-bar centres of "*X*" to 6
 *           of the 10 widest humps (straight line: start + module size);
 *        3. letters whose worst centre is off by less than
 *           C39G_ERR_MAX_X100 / 100 module go on to a blurred-bar model of
 *           the whole trace, and the best match must beat the best OTHER
 *           letter by C39G_MARGIN_X100 percent, or nothing is returned.
 *
 *      Pure C, no kernel calls: tested on the PC (testing/subsystem3/
 *      host_test) with the traces saved from the dashboard.
 *      Developed 2026-11-12 from the replay study in
 *      project files barcode/replay (geom.py).  Idea from reading another
 *      group's design; this is our own code.
 */
#ifndef CODE39_GEOM_H
#define CODE39_GEOM_H

#include <stdbool.h>
#include <stdint.h>

/* Largest trace accepted (S3_LTRACE_N) */
#ifndef C39G_MAX_SAMPLES
#define C39G_MAX_SAMPLES 700u
#endif
/* Worst wide-bar centre error allowed, in 1/100 module.  0.3 module gave
   no wrong letter on the 76 saved passes; 0.6 read one pass as 'I' that
   we could not confirm. */
#ifndef C39G_ERR_MAX_X100
#define C39G_ERR_MAX_X100 30
#endif
/* Best letter's model error must be this many percent below the best
   other letter's, or the read is refused as a tie. */
#ifndef C39G_MARGIN_X100
#define C39G_MARGIN_X100 10
#endif
/* The blurred model of the best letter must explain at least this many
   percent of the trace around it (real reads: 44 and 83; random bar
   patterns that slipped through the other checks: 5 to 23). */
#ifndef C39G_MIN_FIT_X100
#define C39G_MIN_FIT_X100 35
#endif
/* Rolling white floor half-width: wheel counts (L wheel x 2, the trace's
   own unit) when distance is used, samples (10 ms) when it is not. */
#define C39G_FLOOR_COUNTS  200
#define C39G_FLOOR_SAMPLES 40
/* A hump must rise this far above the floor (raw ADC) to count at all */
#define C39G_MIN_HUMP 250

typedef enum
{
    C39G_OK = 0,
    C39G_FEW,     /* fewer than 6 humps above C39G_MIN_HUMP           */
    C39G_FEW2,    /* fewer than 6 at half height                       */
    C39G_MANY,    /* more than 24 humps: not one letter                */
    C39G_NOFIT,   /* no letter could be laid on the humps              */
    C39G_ERR,     /* best fit worse than C39G_ERR_MAX_X100             */
    C39G_TIE,     /* two letters fit about as well: refused            */
    C39G_WEAK,    /* best letter explains too little of the trace      */
    C39G_ARG      /* bad arguments                                     */
} c39g_result_t;

typedef struct
{
    char     letter;     /* 'A'..'Z', 0 when refused                     */
    char     best;       /* best letter even when refused as a tie       */
    char     runner;     /* best other letter, '-' if none                */
    bool     b_reversed; /* read backwards                                */
    uint8_t  n_humps;    /* humps at half height                          */
    uint16_t err_x100;   /* worst wide-bar centre error, 1/100 module     */
    uint16_t margin_x100;/* model error of runner-up / best - 1, percent  */
    uint16_t module_x10; /* module size, trace units x 10                 */
    uint16_t ratio_x100; /* wide:narrow ratio used, x 100                 */
    uint8_t  fit_x100;   /* share of the trace the model explains, %     */
} c39g_info_t;

/* Decode one-letter "*X*" from a trace (oldest sample first).
   p_fwd / p_yaw: per-sample wheel steps as in g_s3_ltrace_fwd / _yaw; the
   position is the LEFT wheel (fwd - yaw = 2 x left).  b_use_dist false, or
   p_fwd NULL: position = sample number.  p_yield, if not NULL, is called
   about 90 times during the work (the robot passes tk_dly_tsk(1) so lower
   priority tasks still run).  Returns C39G_OK and fills p_info. */
c39g_result_t c39g_decode(const uint16_t * p_l, const uint8_t * p_fwd,
                          const int8_t * p_yaw, uint32_t n, bool b_use_dist,
                          void (*p_yield)(void), c39g_info_t * p_info);

#endif /* CODE39_GEOM_H */
