# Subsystem 6 — Vehicle Controller (base code)

The briefing and design review describe five buddy subsystems plus a central
**Vehicle Controller** (briefing p.3, design review FR7, sections 3, 5 and 6.1).
This package is that sixth piece: it owns the mission state, decides what the
robot does next, and is the **only** source of movement commands. It drives no
hardware itself.

Target: micro T-Kernel 3.0 on the Pico W, using the project zip's layout
(`app_program/subsystemN/`, `subsystemN_init/_start/_test`, `app_start_task`).

## Drop it in

1. Copy `app_program/subsystem6/` and `testing/subsystem6/` into the repo.
2. Apply `shared_changes/shared.patch` (adds the three `subsystem6_*` calls to
   `app.h` and `app_main.c`), or copy the two files from `shared_changes/`.
   These are team-shared files, so agree it first (TEAM.md rule).
3. Add the TEAM.md rows below.
4. `cd build_make && make -j8` (or `make TEST=1 -j8` for the self-tests).

The build makefile already compiles every `app_program/*/*.c`, so no makefile
change is needed. `host_test/` sits one level deeper and is not compiled into
the firmware.

## Files

| File | What it does |
|---|---|
| `subsystem6.h` | Public interface: `vc_post_event()`, `vc_get_status()`, `flg_vc_state`, priority, 10 ms tick |
| `vc_types.h` | Shared formats: mission states, events, fault severities/codes, movement commands, status |
| `vc_sm.c/.h` | The mission statechart as pure logic (no kernel calls). Returns a list of actions. Tuning values at the top of `vc_sm.h` |
| `task_vehicle_ctrl.c` | The controller task: waits for events (10 ms tick), reads the other subsystems' channels, runs the state machine, carries out actions |
| `vc_ports.c/.h` | The one place actions leave S6 (move, request, fault report, telemetry) |
| `vc_bridge.h`, `vc_bridge_s2.c` … `vc_bridge_s5.c` | Adapters to the interfaces the S2–S5 threads published |
| `subsystem6.c` | Creates the kernel objects, starts the task, implements the public calls |
| `subsystem6_test.c` | On-target tests S6-U01..U14 (`make TEST=1`) |
| `host_test/` | Same state machine tested on a PC: `cd host_test && make` (41 checks) |

## Mission states (design review section 5)

`IDLE → MISSION_START → FOLLOW_LINE` (hub) and from there `LOST_LINE`,
`AVOID_OBSTACLE`, `DECODE_BARCODE → EXECUTE_ACTION`, `TERRAIN_CHECK`,
`MISSION_COMPLETE`, and `FAULT` from any active state.

- **MISSION_START** asks every subsystem for a self check. Motion (S2) and
  line (S3) must answer within 2 s or the mission faults; IMU and obstacle
  are optional (the run continues degraded and it is reported).
- **Line loss** is ignored in DECODE_BARCODE, EXECUTE_ACTION and TERRAIN_CHECK
  (as the review's table says) and in AVOID_OBSTACLE (the robot leaves the
  line on purpose).
- **LOST_LINE** is a bounded search run by the controller: turn 30° towards
  the side the line was last seen, then 60° the other way, then reverse 50 mm,
  then FAULT. S3 keeps tracking and reports LINE_FOUND.
- Every non-hub state has a **time limit**, so no recovery runs forever
  (NFR5). Limits and speeds are first guesses to tune on the track.
- **Faults** follow FR8: CRITICAL → FAULT (S2 safe stop first, NFR8);
  RECOVERABLE → bounded recovery; NON_CRITICAL (e.g. WiFi down) → reported,
  navigation continues. FAULT and MISSION_COMPLETE wait for a reset.
- **Command arbitration** (design review 8.3): a subsystem asks to move with
  `VC_EV_MOVE_REQUEST`. The controller forwards it only if that subsystem owns
  motion in the current state: S3 in FOLLOW_LINE / DECODE / TERRAIN (speed
  capped to slow in the last two), S5 in AVOID_OBSTACLE. Everything else is
  dropped and counted in `moves_rejected`.

## How each subsystem connects

S2–S5 had already published their interfaces, so S6 adapts to them rather
than asking them to change. Each adapter compiles to empty functions while
that folder still holds the zip's example, so S6 builds either way.

| Subsystem | Into the controller | Out of the controller | State |
|---|---|---|---|
| S1 WiFi/MQTT | `vc_post_event(VC_SRC_COMMS, VC_EV_CMD_START / CMD_STOP / CMD_RESET / WIFI_DOWN / WIFI_UP …)` | fault reports + 200 ms status via `vc_port_report()` / `vc_port_telemetry()` | **stub**: prints, waits for S1's telemetry buffer |
| S2 Motion | `flg_motion`: CMD_DONE → manoeuvre done, FAULT / CMD_TIMEOUT → critical | `s2_cmd_drive/turn/drive_dist/stop/reset`, `s2_emergency_stop` | **bound** (`vc_bridge_s2.c`) |
| S3 Line + barcode | `mbf_s3_events` (lost/found/junction/barcode); steering read from `s3_get_line_status()` every 10 ms | `s3_request(BC_ENABLE / BC_REARM / BC_DISABLE)` | **bound** (`vc_bridge_s3.c`) |
| S4 IMU | `mbf_imu_evt` (calibrated, hump start/end, impact, faults) | `s4_request_calibration()` + `s4_request_peak_reset()` at mission start | **bound** (`vc_bridge_s4.c`) |
| S5 Obstacle | `mbf_s5_evt` (obstacle, motion requests, line reacquired, failed) | `mbf_s5_cmd`: ENABLE / START_AVOID / MOTION_DONE / ABORT / DISABLE | **bound** (`vc_bridge_s5.c`) |

Any subsystem can also read the mission state with `vc_get_status()` or wait
on `flg_vc_state` (one bit per state, plus `VC_FLG_RUNNING`).

## What is still stubbed

- **S1 telemetry and fault reports** (`vc_port_report()`, `vc_port_telemetry()`
  in `vc_ports.c`): they print to the console. Bind them to S1's telemetry
  buffer once S1 publishes it (MQTT runs in the kernel's WiFi task, so S1
  drains the buffer with `TMO_POL`). S1 sends remote start/stop/reset with
  `vc_post_event()`, which works today.
- **End of course**: S3 has no "end" barcode yet. Either S3 adds one and the
  bridge maps it to `VC_EV_END_OF_COURSE`, or another subsystem posts it.

## Integration notes

- S5's code has `S5_BENCH_MODE 1` in `s5_local.h`, where S5 plays the
  controller itself. Set it to 0 when subsystem 6 is in the build.
- S4 needs `DEVCNF_USE_IIC 1` in `config/config_device.h` (S4's own note).
- S2 keeps the example's `sem_slots`/`mpf_samples` while `S2_KEEP_EXAMPLE_IF`
  is 1. Once S3/S4 are the real ones it can be set to 0.
- S2–S5 each own their channel towards the controller. That differs from the
  "receiver owns it" rule in TEAM.md, but the adapters handle it, so nothing
  needs to move.
- **Team decision:** an IMU that dies for good (`IMU_EVT_FAULT_FATAL`) is
  treated as non-critical (the run continues without hump data or impact
  detection). One line in `vc_bridge_s4.c` makes it a safe stop instead.

## TEAM.md rows to add

Section 1: `| 6 | Vehicle Controller: mission state, sole movement authority | | | app_program/subsystem6/ | testing/subsystem6/ |`

Section 4 (**proposed**, needs agreement): `| 6 | 10–11 | vehicle_ctrl – 10 – any |`.
It sits above the sensing subsystems because the review rates it High, and it
only runs briefly on each event or 10 ms tick.

Section 5:

| ID | From → To | Mechanism | Object ID name | Data / meaning |
|---|---|---|---|---|
| IF-06 | any → S6 | message buffer (owned by S6) | `mbf_vc_events` | `VC_EVENT` in `vc_types.h`, via `vc_post_event()` |
| IF-07 | S6 → any | shared data under mutex + event flag (owned by S6) | `mtx_vc_status`, `flg_vc_state` | `VC_STATUS`, via `vc_get_status()` |
| (S2's IF-M1/M3) | S6 ↔ S2 | S2's message buffer + event flag | `mbf_motion_cmd`, `flg_motion` | `vc_bridge_s2.c` |
| (S3's IF-31..33) | S3 ↔ S6 | S3's message buffer, status, requests | `mbf_s3_events` | `vc_bridge_s3.c` |
| (S4's IF-S4-1/4) | S4 ↔ S6 | S4's message buffer, requests | `mbf_imu_evt` | `vc_bridge_s4.c` |
| (S5's channels) | S5 ↔ S6 | S5's message buffers | `mbf_s5_evt`, `mbf_s5_cmd` | `vc_bridge_s5.c` |

Section 6 (kernel objects): S6 uses 1 task, 1 message buffer, 1 mutex,
1 event flag.

## What was checked

- With the zip's example subsystems: firmware builds with
  `arm-none-eabi-gcc` 13.2 (`make CONSOLE=uart WIFI=none`, normal and
  `TEST=1`), no warnings from subsystem 6 with `-Wall -Wextra`.
- With the S2, S3, S4 and S5 threads' current code in place of the examples
  (and I2C switched on for S4): the `TEST=1` image builds and links with all
  four adapters active.
- PC tests: 41/41 pass.
- **Not done:** the default USB/WiFi build (needs the Pico SDK, not available
  here) and anything on a real Pico, including the on-target tests.
