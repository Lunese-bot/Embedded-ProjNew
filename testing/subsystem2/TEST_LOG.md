# Subsystem 2 — test log

**Subsystem:** Motion Control and Wheel Encoders  ·  **Student ID:** 2503000  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | ticks → mm: one revolution = π × wheel diameter | FR3 | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | pivot arc: 360° = π × track, left = right | FR3 | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | set-point ramp limited to one step | FR3 | auto | `subsystem2_test.c` | `[TEST] S2-U03 PASS` |
| S2-U04 | PID output and integral stay within limits | FR3 | auto | `subsystem2_test.c` | `[TEST] S2-U04 PASS` |
| S2-U05 | PID integral raises output while below target | FR3 | auto | `subsystem2_test.c` | `[TEST] S2-U05 PASS` |
| S2-U06 | motion_task runs every 10 ms, no overruns | NFR2 | auto | `subsystem2_test.c` | `[TEST] S2-U06 PASS` |
| S2-U07 | IF-M1 message buffer accepts a command | IF-M1 | auto | `subsystem2_test.c` | `[TEST] S2-U07 PASS` |
| S2-U08 | after STOP: IDLE, both duties 0 | IF-M1, IF-M2 | auto | `subsystem2_test.c` | `[TEST] S2-U08 PASS` |
| S2-U09 | emergency stop within 100 ms, latched, DRIVE ignored | NFR8, IF-M3 | auto | `subsystem2_test.c` | `[TEST] S2-U09 PASS` |
| S2-U10 | RESET returns to IDLE | FR8 | auto | `subsystem2_test.c` | `[TEST] S2-U10 PASS` |
| S2-M01 | each wheel turns forward for a forward command | FR3 | manual | Robot on a stand, `S2_DEMO 1`. Watch both wheels on "forward 300 mm". Flip `S2_MOTOR_x_INVERT` if one turns backwards | both wheels forward |
| S2-M02 | encoder counts one revolution | FR3, A4 | manual | Mark a wheel, turn it by hand 10 turns, read `ticks_l/ticks_r` from the status print | 10 × `S2_TICKS_PER_REV` ± 1 |
| S2-M03 | minimum moving duty | FR3 | manual | Raise `S2_DUTY_MIN_MOVE` from 100 in steps of 25 until both wheels start reliably | value recorded |
| S2-M04 | speed tracking at 100 / 200 / 300 mm/s | FR3, NFR3 | manual | `s2_cmd_drive(v, 0)` refreshed every 100 ms; log `v=` for 5 s | mean within ±10 % of target |
| S2-M05 | straight 1 m | FR3, NFR3 | manual | `s2_cmd_drive_dist(200, 1000)` ×5 on the floor; measure distance and sideways drift | 1000 ± 20 mm, drift < 30 mm |
| S2-M06 | 90° turn accuracy | FR3, NFR3 | manual | `s2_cmd_turn(150, ±90)` ×5 each way; measure angle | 90 ± 5° |
| S2-M07 | stall → FAULT | NFR8 | manual | Hold one wheel during DRIVE | FAULT within 0.6 s, motors off, `MFLG_FAULT` set |
| S2-M08 | DRIVE timeout | NFR8 | manual | Send one DRIVE, then nothing | stops after 0.5 s, `MFLG_CMD_TIMEOUT` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
