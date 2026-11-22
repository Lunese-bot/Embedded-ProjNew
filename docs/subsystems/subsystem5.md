# Subsystem 5 base code: Ultrasonic Obstacle Detection and Avoidance

Buddy 5 (student 2503330) in the design review. Drop-in for the
`Embedded-ProjNew` repo on micro T-Kernel 3.0 (Raspberry Pi Pico W).

## How to install

Copy `app_program/subsystem5/` over the empty template in the repo, and
`testing/subsystem5/TEST_LOG.md` over the template log. No makefile or
shared-file changes are needed: every `.c` in `app_program/` is built.

Checked with the project's own makefile (`arm-none-eabi-gcc` 13.2):
`make CONSOLE=uart WIFI=none` and `make CONSOLE=uart WIFI=none TEST=1`
both build with no warnings (also clean with `-Wall -Wextra`). The USB/WiFi
build needs the Pico SDK and was not built here. The algorithm tests
(S5-U01 to S5-U09) also pass when compiled for a PC. Nothing has been run on
a real Pico yet.

## Files

| File | What it is |
|---|---|
| `subsystem5.h` | public interface: commands, events, motion requests, profile, status |
| `subsystem5.c` | creates kernel objects, starts tasks, status snapshot (mutex) |
| `task_obstacle_scan.c` | **Obstacle Detection & Scanning task** (pri 21): 20 ms monitoring, coarse + fine scan |
| `task_obstacle_avoid.c` | **Obstacle Avoidance task** (pri 20): command handling, bypass, line recovery |
| `obstacle_logic.c/.h` | pure algorithms (profile, side choice, bypass plan, recovery) and tuning values |
| `drv_ultrasonic.c/.h` | HC-SR04 driver, timed with the RP2040 1 MHz TIMER |
| `drv_servo.c/.h` | servo on PWM slice 6, 50 Hz, angle calibration constants |
| `s5_ports.c` | reads line feedback through the existing hook and live sensor mask |
| `s5_local.h` | private to the folder: flags, timings, bench mode |
| `subsystem5_test.c` | S5-U01..U11 for `make TEST=1` |

## How it talks to the robot

```
 Vehicle Controller ──s5_cmd_t──▶ g_mbf_s5_cmd ──▶ s5_avoid_task ◀─g_flg_s5─▶ s5_scan_task ─▶ ultrasonic + servo
 Vehicle Controller ◀─s5_evt_t── g_mbf_s5_evt ◀──┘                              │
 telemetry (Buddy 1) ◀── s5_get_status() ◀── s5_status (g_mtx_s5) ◀───────────┘
 line following (Buddy 3) ──▶ s5_port_line_detected()
```

1. Scan task pings straight ahead every 20 ms. 3 readings in a row under
   225 mm trigger `S5_EVT_OBSTACLE_DETECTED`.
2. The controller stops the car and sends `S5_CMD_START_AVOID`.
3. The coarse scan sweeps −60° to +60° in 30° steps. The fine scan uses 5°
   steps up to 20° past each edge, within ±45°. The profile records distance,
   edges, width, left and right clearance, and the bypass side.
   If the scan places the centre of the obstacle at least 10° to one side,
   the first turn is toward the opposite side if it has enough clearance.
   If that side is blocked, the car stops and backs away. The `s5: fine`
   serial line shows the scan distance, width, edges and clearance on each side.
4. The bypass is sent as `S5_EVT_MOTION_REQ` steps. S5 stops, turns 35° away
   from the line and drives 80 mm. It turns another 10° and drives sideways
   far enough to clear the obstacle by the half-width of the car plus a
   20 mm margin. It straightens, passes the obstacle, and turns 40° inward
   to search. Each step waits for `S5_CMD_MOTION_DONE` with the
   same `seq`. The Vehicle Controller forwards these requests to Motion Control.
5. The return search covers the outbound offset plus 40 mm sideways. When
   the sensors see black, S5 waits for the wheels to stop, realigns, and checks
   again. If it sees black, S5 creeps forward 25 mm, stops, and checks for black
   once more before handing control back. S5 checks 15° to each side if it
   cannot confirm the line at the approach heading. If the first probe is
   white, it can crawl up to 100 mm toward the last sighting. When both C
   and R see black at an angled probe, S5 turns back to the approach heading
   and confirms the line there.
6. If the first inward drive ends without a sighting, S5 stops, takes a
   fresh IR reading, and can search up to 85 mm farther on the same heading.
   If a sighting is lost during braking or alignment, S5 logs that phase
   and goes directly to straight recovery. For straight recovery, S5 turns
   to the original approach heading if needed and searches forward up to
   175 mm at 60 mm/s. If that pass cannot confirm the line, S5 stops
   and takes a fresh IR reading before searching up to 30 mm farther at the
   same heading. If the 30 mm search cannot confirm tape, S5 stops, samples
   again, and can search once more for up to 35 mm on the same heading.
   Each search checks a fresh endpoint reading and uses the same tape
   verification before handing control back. If S5 still cannot confirm
   the line, `S5_EVT_AVOID_FAILED` puts the controller in Safe Stop until reset.
   The line task pauses IR sampling during the scan, detour and pivots. S5
   enables sampling for the search passes and the checks after alignment.
   The first fresh steering command restarts line following at 60 mm/s nominal.
   The controller allows about half a second of steering before restoring cruise.
   The default Straight junction setting keeps line following active through
   crossings. For the dashboard's other junction actions, detection rearms
   after normal tracking resumes and C and R have gone ten samples without
   both seeing black.

## What is stubbed or still to do

- **Bench mode** defaults to 0 in `s5_local.h` so the Vehicle Controller
  can execute the movement requests. With `S5_BENCH_MODE=1`, S5 starts
  avoidance itself and treats each motion request as done after 500 ms.
  This lets you test sensing and scanning without a controller or motors.
- **Standalone line detection is a stub** (`s5_port_line_detected()` returns
  FALSE). In the integrated build, S5 checks the C and R readings through
  the existing line feedback. In standalone mode the bypass ends in
  AVOID_FAILED when all search attempts are exhausted.
- **The Vehicle Controller side is not written here.** It must read
  `g_mbf_s5_evt`, forward motion requests to Motion Control and answer
  `MOTION_DONE`. The interface was sent to the Vehicle Controller
  thread.
- **Tuning values are guesses** (A6, "needs validation"): detect distance,
  clearance, margins, robot size, obstacle depth, speeds, servo calibration.
  They are all in `obstacle_logic.h` / `drv_servo.h`.
- **The echo is busy-waited** (up to ~12 ms per ping). It is fine at
  priority 21, but a GPIO-edge interrupt version would free the CPU.
  Measure it for NFR2.
- The 20 ms ping period is shorter than the 60 ms HC-SR04 datasheet gap.
  Check for ghost echoes, and lengthen `S5_PERIOD_MS` if you see them.
- **ECHO is 5 V.** Use a divider to GP16 (the RP2040 is not 5 V tolerant).
- Obstacle depth can't be measured from the front; it is assumed (150 mm).

## TEAM.md rows to claim

| Pin | Owner | Used for |
|---|---|---|
| GP12 | S5 | scan servo (PWM slice 6 A, 50 Hz; GP13 shares the slice) |
| GP16 | S5 | ultrasonic ECHO (input, via 5 V→3.3 V divider) |
| GP17 | S5 | ultrasonic TRIG (output) |

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| PWM slice 6 | S5 | `s5_scan_task` | GP12 |
| TIMER (read only, TIMERAWL) | shared, read only | `s5_scan_task` | — |

| Subsystem | Priority band | Tasks |
|---|---|---|
| 5 | 20–21 | s5_avoid_task – 20 – any; s5_scan_task – 21 – any |

| ID | From → To | Mechanism | Object ID name | Data / meaning |
|---|---|---|---|---|
| IF-S5a | VC → S5 | message buffer (owned by S5) | `g_mbf_s5_cmd` | `s5_cmd_t` |
| IF-S5b | S5 → VC | message buffer (owned by S5) | `g_mbf_s5_evt` | `s5_evt_t` (incl. `s5_motion_req_t`) |
| IF-S5c | S5 → any | function + mutex | `s5_get_status()` | `s5_status_t` |
| IF-S5d | S3 → S5 | weak function and live mask | `s5_port_line_detected()`, `g_s3_black_mask_live` | C or R detects the tape. The mask distinguishes a simultaneous C and R sighting. |

Kernel objects used by S5: 2 tasks, 2 message buffers, 1 event flag, 1 mutex.

The design review rates both S5 tasks "High" priority, but TEAM.md gives S5
the lowest band (20–21). The team should agree on this.
