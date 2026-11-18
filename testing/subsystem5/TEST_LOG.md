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
| S5-U09 | final recovery pass searches along the original heading | FR4, NFR5 | auto | plan check | PASS |
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
| S5-U22 | braking distance at top speed (400 mm/s) | Version C gap "stopping distance" | manual | 1. Drive straight at 400 mm/s. 2. Trigger stop. 3. Measure distance from stop command to rest, 5 runs. 4. Put the worst value in `S5_BRAKING_MM` (obstacle_logic.h) and rebuild: the build fails if 225 mm no longer covers it. | worst braking + 33 mm reaction + 50 mm margin <= 225 mm |
| S5-U23 | stopping-distance budget | Version C gap "stopping distance" | auto (also checked at compile time) | `subsystem5_test.c`, prints latency/reaction/needed | PASS, needed 183 mm <= 225 mm |
| S5-U24 | stop path on the robot | NFR8, R4 | manual | Integrated build: stop the VC reading `g_mbf_s5_evt` (or fill it), drive at an obstacle; check `s5: ... LOST -> emergency stop` and that the wheels stop; then MOTION_RESET | robot stops before the obstacle |
| S5-H09 | no black line after either bypass | FR4, NFR5 | auto (PC) | Simulate empty floor in `test_navigation.c` | Three slow passes with mirrored turns, then a critical line-not-found event |
| S5-H10 | line lost during alignment and reacquired on a later pass | FR4 | auto (PC) | Simulate tape at two points in `test_navigation.c` | Brakes for both sightings and reports success only after the later alignment |
| S5-H11 | fresh stop confirmation at line sighting | IF-S5 | auto (PC) | Deliver a late distance-move completion before a stop indication | Only the stop indication acknowledges the stop request |
| S5-H12 | IR sampling limited to the line search and alignment check | IF-S5d | auto (PC) | Simulate successful and exhausted bypasses | IR is requested for line checks only and released before pivots and handoff |
| S5-H13 | Return search reaches beyond the expected line | FR4 | auto (PC) | Check the last plan step for both bypass directions | The search runs at 60 mm/s at 30° with 90 mm of additional lateral reach |
| S5-H14 | Nearby line recovery after alignment | FR4 | auto (PC) | Simulate a first sighting that goes white after the alignment turn | The stationary checks can confirm tape to either side before broader recovery |
| S5-H15 | Both nearby checks miss | FR4, NFR5 | auto (PC) | Simulate blank floor after the first sighting | Restore heading and try the bounded recovery. The mission holds stopped if no further sighting is confirmed. |
| S5-H16 | First turn away from an off-centre obstacle | FR4 | auto (PC) | Build profiles from mirrored scans in `test_navigation.c` | Turn away when the opposite side is clear. Back off if that side is blocked. |
| S5-H17 | Tape disappears as the robot moves after alignment | FR4, NFR5 | auto (PC) | Simulate black at rest and white after the 25 mm rejoin drive | Confirm the wheels stopped, reject the sighting, and continue bounded recovery. |
| S5-H18 | Tape is just beyond the stationary alignment check | FR4, NFR5 | auto (PC) | Simulate tape appearing during the first 15° crawl after alignment, on both sides | Stop and confirm the line before handing steering back. |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

2026-11-18 stop/scan/avoid restoration: obstacle checking starts enabled.
Three near readings below 250 mm enter avoidance, with scanning requested
only after S2 confirms the wheels stopped. Coarse side-clearance samples
are retained during fine profiling. Obstacle drive/search/pivots use 60 mm/s;
normal line following and normal mission pivots keep their previous behavior.
The ultrasonic driver and startup readiness requirements remain at baseline.

For the robot check, flash `build_make/mtk3pico_smp1_usb_cdc_wifi_lwip.uf2`,
refresh the dashboard, enable obstacle use and start line following with the
wheels raised first. Present a flat box at 200 mm; confirm wheel stop before
servo sweep and then slow motor commands. Floor testing must still confirm
actual turn direction and C/R black-line reacquisition after the bypass.

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-18 | stop/avoid navigation regressions | working tree | Codex | PASS 5 groups | [host log](logs/2026-11-18_restore_stop_avoid.txt) | Production S2 bridge and profile/plan algorithms with simulated motor commands |
| 2026-11-18 | stop/avoid firmware build | working tree | Codex | PASS | [build log](logs/2026-11-18_restore_stop_avoid_build.txt) | NET=lwip UF2 generated; not flashed |
| 2026-11-18 | S5-H01..H11 and navigation suite | dffa830 + working tree | Codex | PASS 8 packing tests and 8 navigation groups | [host log](logs/2026-11-18_line_recovery.txt) | Empty floor ends recovery on both bypass sides. Loss of the first sighting causes a new pass. A late move completion cannot acknowledge braking. |
| 2026-11-18 | navigation suite including S5-H12 | dffa830 + working tree | Codex | PASS 9 navigation groups | [host log](logs/2026-11-18_ir_rejoin.txt) | Simulated line checks request IR during the search and after alignment. Wi-Fi/lwIP firmware built. |
| 2026-11-18 | navigation suite including S5-H13..S5-H15 | dffa830 + working tree | Codex | PASS 12 navigation groups | [host log](logs/2026-11-18_nearby_line.txt) | The extended return search and stationary checks cover both sides and missed tape. Wi-Fi/lwIP firmware built. |
| 2026-11-18 | navigation suite including S5-H16..S5-H17 | dffa830 + working tree | Codex | PASS 14 navigation groups | [host log](logs/2026-11-18_direction_rejoin.txt) | Mirrored scans turn away. Tape lost during the rejoin crawl keeps recovery active. Wi-Fi/lwIP firmware built. |
| 2026-11-18 | navigation suite including S5-H18 | dffa830 + working tree | Codex | PASS 15 navigation groups | [host log](logs/2026-11-18_shallower_return.txt) | Both return paths retain lateral reach and can pick up tape during the nearby crawl. Wi-Fi/lwIP firmware built. |
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
