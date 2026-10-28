# Subsystem 5 — test log

**Subsystem:** Ultrasonic Obstacle Detection and Avoidance  ·  **Student ID:** 2503330  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | integer sine table | FR4 | auto | `subsystem5_test.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | profile: distance and edges of a centred box | FR4 | auto | synthetic scan | PASS |
| S5-U03 | profile: width estimate | FR4 | auto | synthetic scan | PASS |
| S5-U04 | no echo anywhere = no obstacle | FR4 | auto | synthetic scan | PASS |
| S5-U05 | wall on the left -> bypass right | FR4, A6 | auto | synthetic scan | PASS |
| S5-U06 | blocked both sides -> no side | FR4, NFR8 | auto | synthetic scan | PASS |
| S5-U07 | blocked -> stop + reverse only | FR4, NFR8 | auto | plan check | PASS |
| S5-U08 | right bypass step sequence | FR4 | auto | plan check | PASS |
| S5-U09 | recovery attempts search further (bounded) | FR4, NFR5 | auto | plan check | PASS |
| S5-U10 | scan task is pinging | NFR2 | auto | status counter | PASS |
| S5-U11 | last reading in sensor range | FR4 | auto | sensor wired, object 20–30 cm ahead | PASS |
| S5-U12 | distance accuracy | FR4, NF9 | manual | 1. Flat board at 100, 200, 300, 500 mm. 2. Read `s5:` lines / status. 3. Record 10 readings each. | error ≤ ±10 mm up to 500 mm |
| S5-U13 | servo calibration | NF9 | manual | 1. Command 0, ±30, ±60 deg. 2. Measure actual angle with a protractor. 3. Adjust `SERVO_CENTRE_US` / `SERVO_US_PER_DEG`. | error ≤ ±3 deg |
| S5-U14 | detection latency | NFR2 | manual | 1. Push a box into view at 200 mm. 2. Scope GP17 (trig) and note time to `s5: obstacle` line. | ≤ 3 periods (60 ms) after the box is in range |
| S5-U15 | bench avoidance sequence (`S5_BENCH_MODE 1`) | FR4, Statechart 6.3 | manual | 1. Box at 20 cm. 2. Watch console: coarse, fine, `[bench] move` lines, 3 recoveries, AVOID_FAILED (line stub is FALSE). | sequence in that order, then monitoring resumes |
| S5-U16 | bypass on the course | FR4, A6 | manual | Obstacle on the track; integrated build with VC + motion control. | car passes and reacquires the line |
| S5-U17 | 16-bit telemetry word: pack/unpack round trip | FR4, min-bit (Week 8) | auto | `subsystem5_test.c` | PASS |
| S5-U18 | telemetry word: distance > 2047 mm saturates | min-bit, R1 | auto | `subsystem5_test.c` | PASS |
| S5-U19 | field widths: 10 states fit 4 bits, 2000 mm fits 11 bits, word = 2 bytes | min-bit, R1 | auto | `subsystem5_test.c` | PASS |
| S5-U20 | `s5_get_status_packed()` returns a valid live state | IF-S5c, min-bit | auto | `subsystem5_test.c` | PASS, prints the word |
| S5-H01..H08 | packing on the PC, incl. all 65,536 combinations | min-bit, R1, R3 | auto (PC) | `cd testing/subsystem5/host_test && make` | 8 passed, 0 failed |
| S5-U21 | latched stop path: full VC queue -> emergency stop called | NFR8, Version C gap "obstacle event dropped" | auto | private 1-message buffer, fake stop function (real motors not latched) | PASS |
| S5-U22 | braking distance at top speed (400 mm/s) | Version C gap "stopping distance" | manual | 1. Drive straight at 400 mm/s. 2. Trigger stop. 3. Measure distance from stop command to rest, 5 runs. 4. Put the worst value in `S5_BRAKING_MM` (obstacle_logic.h) and rebuild: the build fails if 250 mm no longer covers it. | worst braking + 33 mm reaction + 50 mm margin <= 250 mm |
| S5-U23 | stopping-distance budget | Version C gap "stopping distance" | auto (also checked at compile time) | `subsystem5_test.c`, prints latency/reaction/needed | PASS, needed 183 mm <= 250 mm |
| S5-U24 | stop path on the robot | NFR8, R4 | manual | Integrated build: stop the VC reading `g_mbf_s5_evt` (or fill it), drive at an obstacle; check `s5: ... LOST -> emergency stop` and that the wheels stop; then MOTION_RESET | robot stops before the obstacle |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
