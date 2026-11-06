/*
 *      s3_config.h
 *      Subsystem 3 -- IR Line Following and Barcode Navigation.
 *      Every tunable number of this subsystem lives here (NFR9: calibration
 *      parameters must be configurable).  Values marked TUNE are first
 *      guesses: replace them with measured values and record the
 *      measurement in testing/subsystem3/TEST_LOG.md.
 *
 *      This header holds plain numbers only, so that the host unit tests
 *      (testing/subsystem3/host/) can include it without the kernel.
 */

#ifndef S3_CONFIG_H
#define S3_CONFIG_H

/* ---------------------------------------------------------------------
 *  Pins (Design Review section 2; claim them in TEAM.md section 2)
 * --------------------------------------------------------------------- */
#define S3_PIN_IR_L 28 /* IR_L  GP28  barcode sensor (ADC2), moved from GP06 2026-11-05 */
#define S3_PIN_IR_C 26 /* IR_C  GP26  line position (ADC0)  */
#define S3_PIN_IR_R 27 /* IR_R  GP27  line position (ADC1)  */

/* Sensor index used in every array of three readings */
#define S3_IR_L     0
#define S3_IR_C     1
#define S3_IR_R     2
#define S3_IR_COUNT 3

#define S3_MASK_L (1u << S3_IR_L)
#define S3_MASK_C (1u << S3_IR_C)
#define S3_MASK_R (1u << S3_IR_R)

/* ---------------------------------------------------------------------
 *  Sensor read mode
 *      0: all three sensors are read as digital GPIO (the pin table).
 *      1: IR_C and IR_R are read through the ADC (finer line position and
 *         real min/max calibration).  IR_L stays digital: GP06 has no ADC.
 *      2: ONLY IR_R through the ADC (2026-11-05: the R module is less
 *         sensitive than C, so its black/white cut-off is set here in
 *         software instead of by its pot).  Wire R's A0 (not D0) to GP27.
 *         Needs only DEVCONF_ADC_PIN_INIT_1.  No calibration needed: R is
 *         scaled between S3_R_ADC_WHITE and S3_R_ADC_BLACK below.
 *         Needs DEVCONF_ADC_PIN_INIT_0 and _1 set to TRUE in
 *         device/adc/sysdepend/rp2040/adc_cnf_sysdep.h (shared file: agree
 *         it with the team and record ADC ownership in TEAM.md section 3).
 * --------------------------------------------------------------------- */
#define S3_IR_USE_ADC 1 /* 2026-11-05: C and R analogue (R alone felt smoother) */

/* Modes 1 and 2: R's raw ADC reading (0..4095, shown as "raw L C R" on the
   dashboard) over plain WHITE floor and over the middle of the BLACK line.
   R counts as black above the halfway point (S3_BLACK_THRESHOLD 500 of
   1000).  Either order works (black higher or lower than white).
   Measured 2026-11-05: white 151-180, line edge 180-225, full black
   2000-2050.  BLACK is set low ON PURPOSE so the cut-off (halfway) is 300:
   above the edge readings (white noise must not count as black) but far
   below full black, so R reports black as soon as it is partly over the
   line.  Anything above 420 reads as full black.
   Too twitchy on white: raise BLACK.  Still late on the line: lower it
   (not below about 280, i.e. a 230 cut-off). */
#define S3_R_ADC_WHITE 180
#define S3_R_ADC_BLACK 420

/* Mode 1: the same for C (wire C's A0 to GP26).  Used while no calibration
   has been run.  Measured 2026-11-05: white 120-205, edge 139-210, full
   black 2200-2300.  Cut-off (halfway) 300, the same as R. */
#define S3_C_ADC_WHITE 200
#define S3_C_ADC_BLACK 400

/* L (barcode sensor) through the ADC as well: 1 = L's A0 on GP28 (ADC2),
   0 = L's D0 digital (set S3_PIN_IR_L back to the digital pin).
   S3_L_ADC_WHITE / _BLACK work like R's.  Re-measured 2026-11-05 after
   lowering L: white 140-190, THINNEST bar of 'A' 200-250, thickest bar
   1300-1400.  Thin bars are only ~50 above white, so the cut-off has to
   sit in that small gap: halfway = 215, and with S3_BC_HYST 100 L turns
   black above about 219 and white again below about 211.  PROVISIONAL:
   check the L trace on the dashboard (white noise must stay under the
   green line, every thin bar must cross the red one). */
#define S3_IR_L_ADC    1
#define S3_L_ADC_WHITE 195
#define S3_L_ADC_BLACK 235

/* Barcode reading: dead band around the black threshold (0..1000 scale) so
   L does not flicker at a bar edge and split one bar into two.  L turns
   black at threshold + HYST and white again below threshold - HYST.
   Only matters for an analogue L (digital L is 0 or 1000). */
#define S3_BC_HYST 100

/* Barcode edge finder (added 2026-11-05).  1 = find bars by L's rises and
   falls (raw ADC), 0 = the fixed cut-off above.  Needs an analogue L.
   Why: L's trace over 'A' showed the thin white gaps only dropping to
   1100-1700 raw (white is ~140-250), so a fixed cut-off merged every bar.
   S3_BC_EDGE_SWING: how far (raw) L must fall from a bar's peak to count a
     gap, or rise from a gap's valley to count a bar.  Real gaps in the
     trace fell 500-1800; flat noise on a bar wobbles under ~100.
   S3_BC_EDGE_FLOOR: below this raw value L is always white, and L must be
     above it to count a bar (white paper read 140-260, up to ~360 at an
     edge).  Both can be changed live on the dashboard (Line test tab). */
#define S3_BC_EDGE       1
#define S3_BC_EDGE_SWING 300
#define S3_BC_EDGE_FLOOR 400

/* Digital level the module outputs over BLACK tape.  TUNE: most TCRT5000
   style modules give 1 on black, some give 0. */
#define S3_IR_BLACK_LEVEL 1

/* ADC mode only: set to 1 if the analogue output RISES over black */
#define S3_ADC_BLACK_IS_HIGH 1

/* ---------------------------------------------------------------------
 *  Line following (task_line.c, line_algo.c)
 * --------------------------------------------------------------------- */
/* NFR2: 10 ms.  Must be a multiple of CNF_TIMER_PERIOD (10 ms) */
#define S3_LINE_PERIOD_MS 10

/* Sensors used for the line position.  Default C+R because the pin
   table gives IR_L to the barcode.  Use all three if IR_L sits over
   the line too. */
#define S3_LINE_MASK (S3_MASK_C | S3_MASK_R)

/* STRADDLE layout (S3_STRADDLE 1): the line runs BETWEEN the centre and the
   right sensor.
     both white   -> the line is between them: drive straight
     C black only -> the line drifted toward C: steer left
     R black only -> the line drifted toward R: steer right
     both black   -> junction
   The left sensor is only used for barcodes.  With S3_STRADDLE 0 the old
   weighted-average position is used (line under the sensors). */
#define S3_STRADDLE 1

/* Straddle steering (line_algo.c).  While one sensor is black:
     steer = S3_KP_NUM * S3_STRADDLE_BASE + S3_KD_NUM * (10 ms samples black)
   clamped to 1000 (1000 = inner wheel stopped).  Steering is 0 as soon as
   both sensors read white again.  KP = how hard the first nudge is,
   KD = how fast it grows while the sensor stays on the line.  TUNE. */
#define S3_STRADDLE_BASE 25

/* Curve memory (tight curves, U-turns).  If the robot was already turning
   hard (steering at least HOLD_MAG of 1000) when the sensor reads white
   again, the line has probably slipped OUTSIDE the sensor, so keep the same
   turn for up to HOLD_MAX samples (10 ms each) or until the line is back
   under a sensor.  Gentle corrections (straights) still release at once,
   so there is no over-correction there.  HOLD_MAX 0 = off.  TUNE. */
#define S3_STRADDLE_HOLD_MAG 700 /* only after a hard turn (70 %) */
#define S3_STRADDLE_HOLD_MAX 100 /* keep turning up to 1.0 s      */

/* Straddle, 2026-11-04 (U-turn CSVs): on a tight curve C touches the line,
   goes white for a moment, touches again...  Each touch restarted at the
   gentle first nudge, so the turn never built up.  Now each white sample
   only takes S3_STRADDLE_DECAY off the grow counter, so a touch soon after
   another on the SAME side carries on where it was.  The other side still
   starts gently.  Steering on white is still 0.  0 = reset at once (old). */
#define S3_STRADDLE_DECAY 1

/* Straddle: L as a far-left helper.  L black means the line is even further
   left than C.  Only used while a left turn is in progress (or remembered),
   so a barcode under L on a straight does not steer:
     L+C black          -> full left turn
     L black, C R white -> full left turn and keep it (curve memory)
   1 = on, 0 = L for barcodes only (old). */
#define S3_STRADDLE_L_ASSIST 0

/* Straddle: 0 = never enter LOST (both white just means drive straight),
   1 = LOST after S3_LOST_SAMPLES of both white.  Off for now: LOST
   triggered searches that were not wanted. */
#define S3_STRADDLE_LOST 0

/* Barcode bars lying across the line: when L, C and R are all black at
   once (or C and R while a barcode is being read), drive straight and
   reset the turn instead of holding and growing it.  0 = old behaviour. */
#define S3_STRADDLE_BAR_STRAIGHT 1
/* L counts as black for this check at or above this RAW reading.  White
   paper reads up to ~260, thin bars 800-1300, thick bars 1100-4000. */
#define S3_BAR_L_RAW 700

/* Sensors that must all see black for a junction: C and R (the left sensor
   only reads barcodes). */
#define S3_JUNCTION_MASK (S3_MASK_C | S3_MASK_R)

/* A junction needs those sensors above THIS normalised reading
   (0 white .. 1000 black), which is stricter than S3_BLACK_THRESHOLD, so
   a curve that only partly covers a sensor does not count.  Only has an
   effect on analogue sensors (S3_IR_USE_ADC 1): digital sensors read 0 or
   1000.  Adjustable live from the dashboard.  TUNE. */
#define S3_JUNCTION_THRESHOLD 950

/* Position weight of each sensor, -1000 (far left) .. +1000 (far right) */
#define S3_WEIGHT_L (-1000)
#define S3_WEIGHT_C 0
/* Centre weight when only C+R steer: C -1000 / R +1000 is symmetric, so
   both on black = 0, C only = -1000, R only = +1000 (error can be negative).
   With L+C+R the normal S3_WEIGHT_C is used. */
#define S3_WEIGHT_C_CR (-1000)
#define S3_WEIGHT_R 1000

/* Position the controller holds the robot at.  With S3_LINE_MASK = C+R,
   0 means "line under C only".  TUNE on the track. */
#define S3_LINE_SETPOINT 0

/* Normalised reading (0 white .. 1000 black) above which a sensor
   counts as "on black" */
#define S3_BLACK_THRESHOLD 500

/* Debounce, in samples of S3_LINE_PERIOD_MS */
#if S3_STRADDLE
/* Both sensors white is normal (line between them), so only call the line
   lost after both have been white this long: 150 samples = 1.5 s. */
#define S3_LOST_SAMPLES 150
#else
#define S3_LOST_SAMPLES     5 /* 50 ms with no line -> LOST     */
#endif
#define S3_FOUND_SAMPLES    2 /* 20 ms with line   -> FOLLOW    */
#define S3_JUNCTION_SAMPLES 3 /* 30 ms all black   -> JUNCTION  */

/* PD steering: steering = (KP*err + KD*(err - prev_err)) / GAIN_DEN,
   clamped to +-1000 (positive = steer right).  TUNE. */
#define S3_KP_NUM   5  /* straddle: first nudge 5*25 = 125 (tuned 2026-11-04) */
#define S3_KD_NUM   16 /* straddle: +16 per 10 ms on black (tuned 2026-11-04) */
#define S3_GAIN_DEN 10

/* Calibration (ADC mode): sweep the sensors over line and floor for
   this long, then the span max-min of every used sensor must be at
   least S3_CAL_MIN_SPAN counts. */
#define S3_CAL_DURATION_MS 3000
#define S3_CAL_MIN_SPAN    400

/* ---------------------------------------------------------------------
 *  Barcode (task_barcode.c, code39.c)
 *      Code 39 is assumed (start/stop '*', 9 elements per character, 3 of
 *      them wide).  CONFIRM the symbology with the course rules.
 * --------------------------------------------------------------------- */
#define S3_BC_SENSOR S3_IR_L /* sensor that reads barcodes */

/* The course barcodes contain uppercase letters A-Z only.  The maximum
   below counts DATA letters; Code 39 start/stop '*' are additional. */
#define S3_BC_LETTERS_ONLY 1
#define S3_BC_MAX_LETTERS  10u
#define S3_BC_MAX_CHARS    (S3_BC_MAX_LETTERS + 2u) /* incl. '*' markers */
#define S3_BC_MAX_ELEMENTS ((S3_BC_MAX_CHARS * 10u) - 1u)
#define S3_BC_TEXT_MAX     (S3_BC_MAX_LETTERS + 1u) /* incl. NUL */

/* A white gap longer than S3_BC_QUIET_FACTOR x the widest element seen
   so far ends the barcode (Code 39 quiet zone is >= 10 narrow). */
#define S3_BC_QUIET_FACTOR   5
#define S3_BC_QUIET_MIN_MS   800  /* never end sooner than this (was 150; 2026-11-05:
                                     a slow, blurry pass had white gaps of 130-600 ms
                                     inside one barcode and got split into pieces) */
#define S3_BC_START_WHITE_MS 150  /* white needed before a bar can start a barcode */
#define S3_BC_ELEMENT_MAX_MS 1500 /* longer black run = not a bar   */
#define S3_BC_MIN_ELEMENTS   29u  /* "*A*" = 3 symbols x 10 - 1 */

/* Smallest accepted wide:narrow ratio, x10 (15 = 1.5).  Code 39 is
   printed at 2.0..3.0. */
#define S3_BC_MIN_RATIO_X10 15

#define S3_BC_MAX_RETRIES 2 /* reported to the controller     */

/* Legacy first-character mapping retained for the S3 event interface.
   The full Vehicle Controller now uses exact full-string rules in
   subsystem6/vc_barcode_rules.c.  Keep these disabled to avoid inventing
   course mappings or executing placeholder actions. */
#define S3_BC_CHAR_LEFT     '\0'
#define S3_BC_CHAR_RIGHT    '\0'
#define S3_BC_CHAR_STRAIGHT '\0'
#define S3_BC_CHAR_UTURN    '\0'

#endif /* S3_CONFIG_H */

/*** end of file ***/
