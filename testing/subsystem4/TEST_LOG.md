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
| S4-M01 | axis mapping | FR5 | manual | tilt nose up 20°, then left side up | pitch > 0, then roll > 0 (fix `S4_ACC_*` in `subsystem4_cfg.h` if not) |
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
