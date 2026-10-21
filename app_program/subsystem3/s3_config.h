/*
 *	s3_config.h
 *	Subsystem 3 -- IR Line Following and Barcode Navigation.
 *	Every tunable number of this subsystem lives here (NFR9: calibration
 *	parameters must be configurable).  Values marked TUNE are first
 *	guesses: replace them with measured values and record the
 *	measurement in testing/subsystem3/TEST_LOG.md.
 *
 *	This header holds plain numbers only, so that the host unit tests
 *	(testing/subsystem3/host/) can include it without the kernel.
 */

#ifndef S3_CONFIG_H
#define S3_CONFIG_H

/* ---------------------------------------------------------------------
 *  Pins (Design Review section 2; claim them in TEAM.md section 2)
 * --------------------------------------------------------------------- */
#define S3_PIN_IR_L		6	/* IR_L  GP06  barcode sensor        */
#define S3_PIN_IR_C		26	/* IR_C  GP26  line position (ADC0)  */
#define S3_PIN_IR_R		27	/* IR_R  GP27  line position (ADC1)  */

/* Sensor index used in every array of three readings */
#define S3_IR_L			0
#define S3_IR_C			1
#define S3_IR_R			2
#define S3_IR_COUNT		3

#define S3_MASK_L		(1u << S3_IR_L)
#define S3_MASK_C		(1u << S3_IR_C)
#define S3_MASK_R		(1u << S3_IR_R)

/* ---------------------------------------------------------------------
 *  Sensor read mode
 *	0: all three sensors are read as digital GPIO (the pin table).
 *	1: IR_C and IR_R are read through the ADC (finer line position and
 *	   real min/max calibration).  IR_L stays digital: GP06 has no ADC.
 *	   Needs DEVCONF_ADC_PIN_INIT_0 and _1 set to TRUE in
 *	   device/adc/sysdepend/rp2040/adc_cnf_sysdep.h (shared file: agree
 *	   it with the team and record ADC ownership in TEAM.md section 3).
 * --------------------------------------------------------------------- */
#define S3_IR_USE_ADC		0

/* Digital level the module outputs over BLACK tape.  TUNE: most TCRT5000
   style modules give 1 on black, some give 0. */
#define S3_IR_BLACK_LEVEL	1

/* ADC mode only: set to 1 if the analogue output RISES over black */
#define S3_ADC_BLACK_IS_HIGH	1

/* ---------------------------------------------------------------------
 *  Line following (task_line.c, line_algo.c)
 * --------------------------------------------------------------------- */
#define S3_LINE_PERIOD_MS	10	/* NFR2: 10 ms.  Must be a multiple of
					   CNF_TIMER_PERIOD (10 ms) */

/* Sensors used for the line position.  Default C+R because the pin
   table gives IR_L to the barcode.  Use all three if IR_L sits over
   the line too. */
#define S3_LINE_MASK		(S3_MASK_C | S3_MASK_R)

/* Sensors that must all see black for a junction */
#define S3_JUNCTION_MASK	(S3_MASK_L | S3_MASK_C | S3_MASK_R)

/* Position weight of each sensor, -1000 (far left) .. +1000 (far right) */
#define S3_WEIGHT_L		(-1000)
#define S3_WEIGHT_C		0
#define S3_WEIGHT_R		1000

/* Position the controller holds the robot at.  With S3_LINE_MASK = C+R,
   0 means "line under C only".  TUNE on the track. */
#define S3_LINE_SETPOINT	0

/* Normalised reading (0 white .. 1000 black) above which a sensor
   counts as "on black" */
#define S3_BLACK_THRESHOLD	500

/* Debounce, in samples of S3_LINE_PERIOD_MS */
#define S3_LOST_SAMPLES		5	/* 50 ms with no line -> LOST     */
#define S3_FOUND_SAMPLES	2	/* 20 ms with line   -> FOLLOW    */
#define S3_JUNCTION_SAMPLES	3	/* 30 ms all black   -> JUNCTION  */

/* PD steering: steering = (KP*err + KD*(err - prev_err)) / GAIN_DEN,
   clamped to +-1000 (positive = steer right).  TUNE. */
#define S3_KP_NUM		8
#define S3_KD_NUM		20
#define S3_GAIN_DEN		10

/* Calibration (ADC mode): sweep the sensors over line and floor for
   this long, then the span max-min of every used sensor must be at
   least S3_CAL_MIN_SPAN counts. */
#define S3_CAL_DURATION_MS	3000
#define S3_CAL_MIN_SPAN		400

/* ---------------------------------------------------------------------
 *  Barcode (task_barcode.c, code39.c)
 *	Code 39 is assumed (start/stop '*', 9 elements per character, 3 of
 *	them wide).  CONFIRM the symbology with the course rules.
 * --------------------------------------------------------------------- */
#define S3_BC_SENSOR		S3_IR_L	/* sensor that reads barcodes */

#define S3_BC_MAX_CHARS		6	/* including the two '*' */
#define S3_BC_MAX_ELEMENTS	(S3_BC_MAX_CHARS * 10)
#define S3_BC_TEXT_MAX		8	/* decoded text incl. '\0' */

/* A white gap longer than S3_BC_QUIET_FACTOR x the widest element seen
   so far ends the barcode (Code 39 quiet zone is >= 10 narrow). */
#define S3_BC_QUIET_FACTOR	3
#define S3_BC_QUIET_MIN_MS	150	/* never end sooner than this     */
#define S3_BC_ELEMENT_MAX_MS	1500	/* longer black run = not a bar   */
#define S3_BC_MIN_ELEMENTS	29	/* "*X*" = 3 chars x 10 - 1       */

/* Smallest accepted wide:narrow ratio, x10 (15 = 1.5).  Code 39 is
   printed at 2.0..3.0. */
#define S3_BC_MIN_RATIO_X10	15

#define S3_BC_MAX_RETRIES	2	/* reported to the controller     */

/* Barcode character -> navigation command (NavigationCommand in
   robot_types.h).  PLACEHOLDER: replace with the course's codes. */
#define S3_BC_CHAR_LEFT		'L'
#define S3_BC_CHAR_RIGHT	'R'
#define S3_BC_CHAR_STRAIGHT	'S'
#define S3_BC_CHAR_UTURN	'U'

#endif /* S3_CONFIG_H */
