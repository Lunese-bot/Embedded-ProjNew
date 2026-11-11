# Subsystem 4 — test log

**Subsystem:** IMU-Based Motion and Terrain Monitoring (Buddy 4)  ·  **Student ID:** 2500366  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | event queue message size | IF-S4-1 | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | status readable via API | IF-S4-2 | auto | `subsystem4_test.c` | PASS |
| S4-U03 | LSM303DLHC answers, calibration completes | FR5, A5 | auto | robot still and level at power-up | PASS |
| S4-U04 | level reading after calibration | FR5 | auto | robot on flat floor | \|pitch\|,\|roll\| < 3°, az ≈ 1 g |
| S4-U05 | 20 ms period, no overruns | NFR2 | auto | `subsystem4_test.c` | 48–52 samples/s |
| S4-U06 | integer atan2 / sqrt | — | auto + host | `make -C host_test` | PASS |
| S4-U07 | synthetic 30 mm hump measured within 20 % | FR5 | auto + host | synthetic data | PASS, peak printed |
| S4-U08 | synthetic 2.5 g spike = one impact | FR5, FR8 | auto + host | synthetic data | PASS |
| S4-U09 | synthetic heading change = TURNING_LEFT | FR5 | auto + host | synthetic data | PASS |
| S4-U10 | calibration rejected while shaking | NFR9 | auto + host | synthetic data | PASS |
| S4-U11 | peak reset request served | IF-S4-4 | auto | `subsystem4_test.c` | PASS |
| S4-M01 | axis mapping | FR5 | manual | tilt nose up 20°, then left side up | pitch > 0, then roll > 0 (fix `S4_ACC_*` in `s4_config.h` if not) |
| S4-M02 | real hump height | FR5, A5 | manual | push robot over each course hump 5× at constant speed; measure true height with a ruler | `max=` in console within 20 % of ruler; record error and tune |
| S4-M03 | collision detection | FR5, FR8 | manual | drive slowly into a soft obstacle; tap chassis lightly | impact event on collision, none on light tap; tune `S4_IMPACT_MG` |
| S4-M04 | I2C fault recovery | FR8, NFR5 | manual | unplug the IMU while running, replug after 3 s | FAULT event, then RECOVERED; with IMU left unplugged, FAULT_FATAL after 5 tries |
| S4-M05 | turn detection on the robot | FR5 | manual | spin in place 90° left and right | TURN_LEFT / TURN_RIGHT motion events |

*Method:* **auto** = `TEST_CHECK` in code; **host** = also runs on a PC with `make -C host_test`; **manual** = a person observes the result.

## Calibration record (NFR9, A5)

| Date | Parameter | Old | New | Evidence (log file) |
|---|---|---|---|---|
| | `S4_HUMP_ENTER_CDEG` | 600 | | |
| | `S4_IMPACT_MG` | 900 | | |
| | `S4_TURN_RATE_CDEG_S` | 2500 | | |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-15 | S4-U06..U10 | (base code) | Claude | PASS | host run, see README | synthetic data on PC only; not yet run on the Pico |

### 2026-11-05 shallow hump correction (host only)

- Changed `S4_HUMP_ENTER_CDEG` from 600 to 300 (6 to 3 degrees) and
  `S4_HUMP_LEVEL_CDEG` from 250 to 150 (2.5 to 1.5 degrees). The old entry
  threshold excluded shallow ramps even when their height was 10-20 mm.
  Retained the three-sample confirmation and existing low-pass filter.
- Fixed interrupted confirmations in climbing, peak and descending states:
  samples in the hysteresis band now clear the pending transition.
- Built the host suite with GCC, C11, `-Wall -Wextra -Werror -O2`.
  All checks passed, including the 10-80 mm height sweep. The sweep now
  contributes failures to the executable's exit status.
- New host cases S4-U15/U16: 3.5-degree ramps at 150 mm/s with +/-4 mg
  deterministic vibration measured 10 mm as 9.92 mm and 20 mm as 19.80 mm.
  S4-U17: a 10 mm ramp without a plateau measured 9.49 mm.
  S4-U18: interrupted samples did not confirm transitions; consecutive
  samples did. Existing flat-ground/noise and impact checks passed.
- Hardware validation is pending. These thresholds are provisional until
  tested on the car. Calibrate level and still, verify nose-up gives positive
  pitch, then drive over measured 10 mm and 20 mm humps five times at steady
  speed. Check HUMP_START/HUMP_END and the end event's peak in micrometres
  (10000 um = 1 cm). Check normal starts, stops and flat travel for false
  hump events. The one-second status print can miss a brief CLIMBING state;
  use event messages as well. The `max=` field is the largest hump of the
  run, not necessarily the latest one.
- Height remains an estimate from wheel distance times slope. Accelerometer
  pitch includes vehicle acceleration, wheel slip affects speed, and a hump
  that produces less than 3 degrees of filtered chassis pitch can still be
  missed. No physical 1-2 cm detection guarantee is implied by host tests.

### 2026-11-05 additional sensitivity adjustment (host only)

- At the user's request, reduced entry from 3 to 2.5 degrees and level
  from 1.5 to 1 degree. Kept the EMA and three consecutive samples to
  reject brief vibration. These values supersede the earlier settings.
- Added S4-U19/U20 for 2.8-degree ramps with +/-4 mg vibration at 150 mm/s.
  Estimated 10 mm as 9.94 mm and 20 mm as 19.93 mm. All host checks,
  including the existing flat-ground noise case and 10-80 mm sweep,
  passed with GCC C11 and `-Wall -Wextra -Werror -O2`.
- Physical validation remains pending. Rebuild and flash, calibrate on
  level ground, and check normal acceleration and braking for false starts.
  Negative pitch still denotes nose-down; sensitivity does not change
  axis mapping or make a stationary height measurement possible.

### 2026-11-11 live dashboard maximum (host only)

- Both dashboard paths already read S4 max_peak_um correctly, but S4
  updated it only after descent and return to level ground. Publish the
  running peak during a confirmed climb, peak or descent once it reaches
  the existing 3 mm noise floor. Retain that maximum even if the crossing
  later times out; last_peak_um, hump_count and HUMP_END still require
  completion. Maximum now represents observed height during the run.
- Added S4-U21: maximum must rise during ascent before any completed hump,
  remain zero below the noise floor, and hold when stopped on the slope.
- GCC C11 with -Wall -Wextra -Werror -O2: S4-U06 through U21 and the
  10-80 mm sweep passed, zero failures. Hardware validation pending.

### 2026-11-11 additional pickup/overturn end-of-run stop

- All existing hump events, height tracking, motion, turn, impact and tilt
  functions remain. Added independent raw-sample pickup/overturn detection.
- Two consecutive 20 ms samples of Z > 1180 mg or magnitude < 650 mg
  detect pickup/unloading. A gravity vector more than 30 degrees from
  upright detects dangerous tilt, including inversion. Uses configured
  robot-frame axes, before bias subtraction or calibration, to prevent
  calibration from redefining an inverted robot as upright.
- Detection calls S2's persistent motor interlock before status publication
  and event reporting. It wakes the motion task, which removes motor drive
  without ramp-down. DRIVE, TURN, STOP and RESET cannot clear this interlock.
  Setting down, recalibration, peak reset and Ignore IMU cannot rearm it.
  Reboot on the floor to begin a new run. Critical event fault code: 0x4005.
- Nominal detection is up to 40 ms plus motion-task scheduling (10 ms
  periodic fallback); this is not a measured end-to-end latency guarantee.
- S4-U22 covers pickup, unloading, both roll/pitch directions and inversion
  before and after calibration, plus latch persistence after setting down,
  recalibration and peak reset. S4-U23 covers normal ramp/turn acceleration
  and isolated jolts. Existing shallow hump tests also assert no new stop.
- GCC C11 -Wall -Wextra -Werror -O2: S4-U06..U23 and 10-80 mm sweep pass.
- Physical limitation: the LSM303 accelerometer cannot reliably identify a
  very slow, level lift or distinguish a supported stationary car from one
  held level. A positive lift indication requires a pickup acceleration or
  tilt. Thresholds are provisional. Reliable detection of every lift needs
  an additional ground-contact/distance sensor.

Hardware acceptance (pending):

1. Calibrate level on the floor. Drive over normal humps, turn both ways,
   accelerate and brake; verify existing behavior and no false pickup stop.
2. In Manual mode with Ignore IMU selected, drive then pick up the robot.
   Verify both motor duties become zero and safety shows LIFT or OVERTURN.
3. Repeat during line following, on nose-up/down and sideways tilts beyond
   30 degrees, and fully inverted. Measure response time on the real car.
4. Set down; attempt drive, turn, Reset, Calibrate and Reset peak hump.
   Both motors must remain off. Reboot upright to start the next run.
5. Try a slow level pickup. Record any misses and false positives from
   normal track bumps; tune thresholds using the accelerometer readings.
