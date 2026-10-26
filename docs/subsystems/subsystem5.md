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
| `obstacle_logic.c/.h` | pure algorithms (profile, side choice, bypass plan, recovery) + all tuning values |
| `drv_ultrasonic.c/.h` | HC-SR04 driver, timed with the RP2040 1 MHz TIMER |
| `drv_servo.c/.h` | servo on PWM slice 6, 50 Hz, angle calibration constants |
| `s5_ports.c` | weak hook for line detection (Buddy 3 overrides it) |
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
   250 mm → `S5_EVT_OBSTACLE_DETECTED`.
2. The controller stops the car and sends `S5_CMD_START_AVOID`.
3. Coarse scan (−60…+60° in 30° steps), then fine scan (5° steps, 20° past
   each edge) → profile: distance, edges, width, left/right clearance, side.
4. The bypass is sent as `S5_EVT_MOTION_REQ` steps (stop, turn 45°, diagonal,
   turn back, pass, turn in, search). Each waits for `S5_CMD_MOTION_DONE`
   with the same `seq`. S5 never drives the motors (design review 8.3).
5. Search steps stop early when the line is seen. Then S5 turns back to the
   original heading → `S5_EVT_LINE_REACQUIRED`. If not found: up to 3 wider
   searches → `S5_EVT_AVOID_FAILED`; the controller decides retry or Safe Stop (FR8).

## What is stubbed or still to do

- **Bench mode is ON** (`S5_BENCH_MODE 1` in `s5_local.h`): S5 starts the
  avoidance itself and treats each motion request as done after 500 ms,
  so you can test sensing and scanning with no controller or motors.
  Set it to 0 to integrate.
- **Line detection is a stub** (`s5_port_line_detected()` returns FALSE).
  Buddy 3 provides the real one, so on the bench every bypass ends in 3
  recoveries and AVOID_FAILED.
- **The Vehicle Controller side is not written here.** It must read
  `g_mbf_s5_evt`, forward motion requests to Motion Control and answer
  `MOTION_DONE`. The interface was sent to the Subsystem 6 (Vehicle
  Controller) thread.
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
| IF-S5d | S3 → S5 | weak function | `s5_port_line_detected()` | TRUE = an IR sensor sees the line |

Kernel objects used by S5: 2 tasks, 2 message buffers, 1 event flag, 1 mutex.

The design review rates both S5 tasks "High" priority, but TEAM.md gives S5
the lowest band (20–21). The team should agree on this.
