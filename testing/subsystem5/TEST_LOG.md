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
| S5-U07 | blocked obstacle -> stop and back away | FR4, NFR8 | auto | plan check | PASS |
| S5-U08 | short first leg and shallow return search on a right bypass | FR4 | auto | plan check | PASS |
| S5-U09 | Straight recovery planner | FR4, NFR5 | auto | Plan from the 40° inward heading | One pivot restores the approach heading before a 175 mm forward search. |
| S5-U10 | scan task is pinging | NFR2 | auto | status counter | PASS |
| S5-U11 | last reading in sensor range | FR4 | auto | sensor wired, object 20–30 cm ahead | PASS |
| S5-U12 | distance accuracy | FR4, NF9 | manual | 1. Flat board at 100, 200, 300, 500 mm. 2. Read `s5:` lines / status. 3. Record 10 readings each. | error ≤ ±10 mm up to 500 mm |
| S5-U13 | servo calibration | NF9 | manual | 1. Command 0, ±30, ±60 deg. 2. Measure actual angle with a protractor. 3. Adjust `SERVO_CENTRE_US` / `SERVO_US_PER_DEG`. | error ≤ ±3 deg |
| S5-U14 | detection latency | NFR2 | manual | 1. Push a box into view at 200 mm. 2. Scope GP17 (trig) and note time to `s5: obstacle` line. | ≤ 3 periods (60 ms) after the box is in range |
| S5-U15 | bench avoidance sequence (`S5_BENCH_MODE 1`) | FR4, Statechart 6.3 | manual | Present a box at 20 cm and watch the coarse scan, fine scan and `[bench] move` lines. | The first inward search, 85 mm retry, 175 mm straight recovery and individual 30 mm and 35 mm searches end in `AVOID_FAILED` when the line stub stays false. |
| S5-U16 | bypass on the course | FR4, A6 | manual | Obstacle on the track; integrated build with VC + motion control. | car passes and reacquires the line |
| S5-U17 | 16-bit telemetry word: pack/unpack round trip | FR4, min-bit (Week 8) | auto | `subsystem5_test.c` | PASS |
| S5-U18 | telemetry word: distance > 2047 mm saturates | min-bit, R1 | auto | `subsystem5_test.c` | PASS |
| S5-U19 | field widths: 10 states fit 4 bits, 2000 mm fits 11 bits, word = 2 bytes | min-bit, R1 | auto | `subsystem5_test.c` | PASS |
| S5-U20 | `s5_get_status_packed()` returns a valid live state | IF-S5c, min-bit | auto | `subsystem5_test.c` | PASS, prints the word |
| S5-H01..H08 | packing on the PC, incl. all 65,536 combinations | min-bit, R1, R3 | auto (PC) | `cd testing/subsystem5/host_test && make` | 8 passed, 0 failed |
| S5-U21 | latched stop path: full VC queue -> emergency stop called | NFR8, Version C gap "obstacle event dropped" | auto | private 1-message buffer, fake stop function (real motors not latched) | PASS |
| S5-U22 | braking distance at top speed (400 mm/s) | Version C gap "stopping distance" | manual | 1. Drive straight at 400 mm/s. 2. Trigger stop. 3. Measure distance from stop command to rest, 5 runs. 4. Put the worst value in `S5_BRAKING_MM` (obstacle_logic.h) and rebuild. | Worst braking + 33 mm reaction + 50 mm margin must fit within `S5_DETECT_MM`. |
| S5-U23 | stopping-distance budget | Version C gap "stopping distance" | auto (also checked at compile time) | `subsystem5_test.c`, prints latency/reaction/needed | PASS, needed 183 mm <= 225 mm |
| S5-U24 | stop path on the robot | NFR8, R4 | manual | Integrated build: stop the VC reading `g_mbf_s5_evt` (or fill it), drive at an obstacle; check `s5: ... LOST -> emergency stop` and that the wheels stop; then MOTION_RESET | robot stops before the obstacle |
| S5-H09 | No black line after either bypass | FR4, NFR5 | auto (PC) | Simulate empty floor in `test_navigation.c` | After the inward drive and its 85 mm retry, one 40° pivot restores the approach heading for 175 mm of straight recovery. One 30 mm search and one 35 mm search on the same heading lead to line-not-found if tape stays absent. |
| S5-H10 | Line lost during alignment | FR4 | auto (PC) | Simulate tape fading during alignment and reappearing during straight recovery | Resume tracking after verifying tape on the single straight recovery pass. |
| S5-H11 | fresh stop confirmation at line sighting | IF-S5 | auto (PC) | Deliver a late distance-move completion before a stop indication | Only the stop indication acknowledges the stop request |
| S5-H12 | IR sampling limited to the line search and alignment check | IF-S5d | auto (PC) | Simulate successful and exhausted bypasses | IR is requested for line checks only and released before pivots and handoff |
| S5-H13 | Return search reaches beyond the expected line | FR4 | auto (PC) | Check the last plan step for both bypass directions | The search runs at 60 mm/s at 40° with 40 mm of additional lateral reach. |
| S5-H14 | Nearby line recovery after alignment | FR4 | auto (PC) | Simulate a first sighting that goes white after the alignment turn | The stationary checks can confirm tape to either side before broader recovery |
| S5-H15 | Both nearby checks miss | FR4, NFR5 | auto (PC) | Simulate blank floor after the first sighting | Restore the approach heading and continue bounded recovery. |
| S5-H16 | Side selection for an off-centre obstacle | FR4 | auto (PC) | Build profiles from mirrored scans in `test_navigation.c` | Select the side away from the obstacle if it is clear. Stop and back away if it is blocked. |
| S5-H17 | Tape disappears as the robot moves after alignment | FR4, NFR5 | auto (PC) | Simulate black at rest and white after the 25 mm rejoin drive | Confirm the wheels stopped, reject the sighting, and continue bounded recovery. |
| S5-H18 | Tape is just beyond the stationary alignment check | FR4, NFR5 | auto (PC) | Simulate tape appearing during the first 15° crawl after alignment, on both sides | Stop and confirm the line before handing steering back. |
| S5-H19 | Shallow first turn retains obstacle clearance | FR4 | auto (PC) | Check both bypass directions in `test_navigation.c` | Turn 35°, drive 80 mm, turn another 10° and finish the clearance leg before straightening. |
| S5-H20 | Both line sensors see black at an angled rejoin probe | IF-S5d | auto (PC) | Simulate tape that holds or disappears on the return pivot | Restore the approach heading. Hand steering back only after confirming tape there. |
| S5-H21 | Tape reappears during the retrace of a failed rejoin check (historical) | FR4, IF-S5d | auto (PC) | Simulate a 25 mm move that loses tape and a reverse that regains it | Retrace only the previous move, stop, and verify tape after a 10 mm retry at the approach heading. |
| S5-H22 | Stopped checkpoints before continuing the bypass (historical) | FR4, NFR8 | auto (PC) | Simulate clear range, no echo, blocked range, missing response and abort in `test_navigation.c` | Stop confirmation precedes each range read. A blocked range or missing reply prevents further motion. Tape confirmed at the approach heading gives a two-turn bypass. |
| S5-H23 | Direct bypass for a small obstacle with room to pass (historical) | FR4, NFR8 | auto (PC) | Simulate mirrored routes, a narrow corridor, a close obstacle, blocked stopped readings and a scanner timeout | One straight pass clears the obstacle. Fresh readings gate the pass and return turn. Tape detection ends the inward drive and starts rejoin confirmation. |
| S5-H24 | Earlier stop opens a shallow route (historical) | FR4 | auto (PC) | Check mirrored profiles of the same 140 mm obstacle from 200 and 330 mm | The close scan stays stopped. The farther scan permits a 30° outward turn and one inward turn. |
| S5-H25 | Initial return and same-heading retry | FR4, IF-S5d | auto (PC) | Show tape on the initial return or during the 85 mm retry, in both directions | Initial success skips the retry. An initial miss allows one retry after a confirmed stop and a fresh IR reading. Both paths verify tape before handoff. |
| S5-H26 | Tape lost during braking or alignment | FR4, IF-S5d | auto (PC) | Remove the first sighting on the stop acknowledgement or the alignment pivot | Log the phase where tape disappears and start straight recovery without the 85 mm retry. Pivot only if the heading still differs from the approach heading. |
| S5-H27 | 30 mm and 35 mm tape searches at the approach heading | FR4, IF-S5d | auto (PC) | Show tape during the 30 mm search, at its endpoint, or at the endpoint of the 35 mm search, in both directions. S5-H09 covers empty floor. | A stop and fresh IR reading precede each search. Confirmed tape from the 30 mm search skips the 35 mm search. Each search waits for a fresh sample after stopping to confirm a sighting before handoff. |
| S5-U27 | Detection range provides room for the shallow bypass (historical) | FR4 | auto | Compare the same 140 mm obstacle at 200 mm and 335 mm in `subsystem5_test.c` | The farther scan fits one outward turn within the cap. |

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
| 2026-11-19 | host suites including S5-H19 and S5-H20 | working tree | Codex | PASS | `host_test/test_navigation.c` | 8 packing tests, 17 navigation groups, and Vehicle Controller host tests passed. Wi-Fi/lwIP firmware built. |
| 2026-11-19 | rejoin suite including S5-H21 | working tree | Codex | PASS | `host_test/test_navigation.c` | 8 packing tests and 18 navigation groups passed. Both bypass directions confirm at the approach heading, including after a short retrace. Wi-Fi/lwIP firmware built. |
| 2026-11-19 | stopped bypass suite including S5-H22 | working tree | Codex | PASS | `host_test/test_navigation.c` | 8 packing tests and 19 navigation groups passed. Fresh checks gate both mirrored passes, and tape at the return pause skips further rotation. Wi-Fi/lwIP firmware built. |
| 2026-11-19 | restored split bypass and rejoin suite | working tree | Codex | PASS | `host_test/test_navigation.c` | 8 packing tests and 18 navigation groups passed. Wi-Fi/lwIP firmware rebuilt after restoring the 35° first turn, 80 mm first drive, and 30° return search. |
| 2026-11-19 | 60 mm first bypass drive | working tree | Codex | PASS | `host_test/test_navigation.c` | 8 packing tests and 18 navigation groups passed. Both sides retain the planned clearance. Wi-Fi/lwIP firmware built. |
| 2026-11-19 | direct small-obstacle bypass including S5-H23 | working tree | Codex | PASS | `host_test/test_navigation.c` | 8 packing tests and 19 navigation groups passed. Both directions verify stopped sonar gates and IR-confirmed rejoin. Wi-Fi/lwIP firmware built. |
| 2026-11-19 | shallowest single outward turn | working tree | Codex | PASS | `host_test/test_navigation.c` | 8 packing tests and 19 navigation groups passed. A narrow scan selects 33° in either direction. Wide, close, or constrained profiles stay stopped. Wi-Fi/lwIP firmware built. |
| 2026-11-19 | short return search and straight retrace | working tree | Codex | PASS | `host_test/test_navigation.c` | 19 navigation groups passed. Missing tape gives two turns and one 100 mm retrace. The simulated braking slip retraces only the 18 mm traveled. Wi-Fi/lwIP firmware built. |
| 2026-11-19 | earlier stop and shallow bypass | working tree | Codex | PASS | `host_test/test_navigation.c` | 8 packing tests and 20 navigation groups passed. H24 opens a shallow route for a blocked close profile. Wi-Fi/lwIP firmware built. |
| 2026-11-19 | pre-video bypass rollback through S5-H20 | working tree | Codex | PASS | `host_test/test_navigation.c` | 8 packing tests and 17 navigation groups passed. Wi-Fi/lwIP firmware built. Physical robot testing is pending. |
| 2026-11-19 | fixed 35° bypass and direct tape search | working tree | Codex | PASS | `host_test/test_navigation.c`, `subsystem5_test.c` | 8 packing tests and 17 host navigation groups passed. S5-U01 through S5-U09 passed on the host. The ARM test source compiled with strict warnings, and Wi-Fi/lwIP firmware built. Physical robot testing is pending. |
| 2026-11-19 | 90 mm return search margin at 40° | working tree | Codex | PASS | `host_test/test_navigation.c` | 8 packing tests and 17 navigation groups passed. The simulated second recovery pass reaches tape 120 mm inward from the recovery start. Wi-Fi/lwIP firmware built. Floor testing is pending. |
| 2026-11-19 | Return retries and tape-loss diagnostics (S5-H25–H27) | working tree | Codex | PASS | `host_test/test_navigation.c` | 8 packing tests and 20 navigation groups passed. The ARM test source passed a syntax check and Wi-Fi/lwIP firmware built. Floor testing still needs to verify the physical 5 mm movement and tape reacquisition. |
| 2026-11-19 | Single straight recovery after a return miss | working tree | Codex | PASS | `host_test/test_navigation.c`, `subsystem5_test.c` | 8 packing tests and 20 navigation groups passed. Both bypass directions recover at the approach heading and confirm tape that appears during or after the 5 mm move. The ARM test source passed a syntax check and Wi-Fi/lwIP firmware built. Floor testing is pending. |
| 2026-11-19 | Conditional 10 mm search after the 5 mm search | working tree | Codex | PASS | `host_test/test_navigation.c` | 8 packing tests and 20 navigation groups passed. In both directions, confirmed tape at 5 mm skips the 10 mm search. Tape at the 10 mm endpoint is verified, and an empty-floor test makes exactly one attempt at each distance. Wi-Fi/lwIP firmware built. Floor testing is pending. |
| 2026-11-19 | Four recovery searches extended by 25 mm | working tree | Codex | PASS | `host_test/test_navigation.c`, `subsystem5_test.c` | 8 packing tests and 20 navigation groups passed. Both directions verify 65 mm, 175 mm, 30 mm and 35 mm search limits and fresh IR sampling after a stop at either final endpoint. The ARM test source passed a syntax check and Wi-Fi/lwIP firmware built. Floor testing is pending. |
| 2026-11-19 | Inward retry extended to 75 mm | working tree | Codex | PASS | `host_test/test_navigation.c` | 8 packing tests and 20 navigation groups passed. Both directions verify a 75 mm same-heading retry after a miss and skip it after a lost sighting. Wi-Fi/lwIP firmware built. Floor testing is pending. |
| 2026-11-19 | Inward retry extended to 85 mm | working tree | Codex | PASS | `host_test/test_navigation.c` | 8 packing tests and 20 navigation groups passed. Both directions verify the 85 mm same-heading retry, 40° return turn and 175 mm, 30 mm and 35 mm recovery moves. Wi-Fi/lwIP firmware built. Floor testing is pending. |
| 2026-11-24 | Measured pass: tracker, side look, fallback, stop paths (S5-M01..M10, off by default) | working tree on fa51404 | Claude coder | PASS (host only) | `host_test/test_side_pass.c`, `host_test/test_side_task.c` | Linux gcc. 8 packing, 20 navigation, side-pass 35722 checks, side-pass task 6 groups. Firmware builds clean with NET=lwip and NET=lwip MQTT=1. **No robot run: hardware acceptance pending the owner's dated evidence.** |
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

### Measured pass: robot rows (owner fills in; nothing here is a result yet)

One robot at a time, after a power cycle.  Engineering targets, not course requirements.

| Date | Step | Firmware commit | Power-cycled | Obstacle depth x width, offset (mm) | Side / mode | True depth vs observed (mm, partial or assumed?) | Min clearance (mm) | Turn-in point (travel mm) | Tape sighting | Rejoin confirmed | VC state after | Followed after VC resumed (mm) | Result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| | | | | | | | | | | | | | |
