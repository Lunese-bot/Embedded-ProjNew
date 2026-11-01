# Subsystem 2 — Motion Control: base code notes

Buddy 2 (2503000). Motors + wheel encoders, PID speed control, distance
and encoder-based turns (Design Review FR3, NFR2, NFR8).

## How to install

Copy `app_program/subsystem2/` over the example folder of the same name,
and `testing/subsystem2/TEST_LOG.md` over the example log. Nothing else in
the repo changes. Build as usual (`cd build_make && make -j8`).

Checked here: builds cleanly with `make CONSOLE=uart WIFI=none` (SMP and
single-core, normal and `TEST=1`) using arm-none-eabi-gcc 13.2. The
default USB/WiFi build needs the Pico SDK, which was not available, but
subsystem 2 does not touch any USB or WiFi code. The control logic was
also run against a simple motor model on a PC (distance, ±90° turn,
reverse, timeout, safe stop, reset all behaved). **Nothing has run on a
real robot yet.**

## Files

| File | What it is |
|---|---|
| `subsystem2.h` | public interface: commands, status, event bits |
| `s2_config.h` | pins, wheel geometry, PID gains, limits (all the numbers to tune) |
| `subsystem2.c` | creates the kernel objects, starts the task, `s2_cmd_xxx()` helpers, bench demo |
| `task_motion.c` | the 10 ms control loop (RECEIVE_COMMAND → UPDATE_SPEED → CALCULATE_PWM → DRIVE_MOTOR) |
| `motion_pid.c/.h` | PID + ramp + unit conversions, pure functions (unit-tested) |
| `drv_motor.c/.h` | Robo Pico H-bridge driver, 20 kHz PWM on GP8–GP11 |
| `drv_encoder.c/.h` | encoder pulses counted in hardware by PWM slices (no interrupts) |
| `subsystem2_test.c` | 10 automatic tests (`make TEST=1`); none of them spin the wheels |

## Interfaces (add to TEAM.md section 5)

| ID | From → To | Mechanism | Object | Data |
|---|---|---|---|---|
| IF-M1 | Vehicle Controller → S2 | message buffer (owned by S2) | `g_mbf_motion_cmd` | `motion_cmd_t`, or call `s2_cmd_drive/drive_dist/turn/stop/reset()` |
| IF-M2 | S2 → anyone | shared data under mutex (owned by S2) | `g_mtx_motion` | `motion_status_t` via `s2_get_status()` |
| IF-M3 | S2 → Vehicle Controller | event flag (owned by S2) | `g_flg_motion` | `MFLG_CMD_DONE`, `MFLG_STOPPED`, `MFLG_SAFE_STOPPED`, `MFLG_FAULT`, `MFLG_CMD_TIMEOUT` |
| — | any task → S2 | function | `s2_emergency_stop()` | latched Safe Stop until `MOTION_RESET` |
| IF-S4-3 | S2 → S4 | function (owned by S4) | `s4_set_speed_mm_s()` | measured vehicle speed, mm/s, every 10 ms, for hump height |

IF-S4-3 is already wired: `task_motion.c` calls `s4_set_speed_mm_s()` at
the end of every period. It is declared weak, so the build still links
while subsystem 4 is the course example. Once the new subsystem 4 is
installed, the call happens without any further change.

Angles and steering use **+ = left (counter-clockwise)**, the same as
subsystem 5's `s5_motion_req_t`, so the Vehicle Controller can forward
obstacle-avoidance moves directly:

| `s5_motion_req_t` | Vehicle Controller sends | Reply `S5_CMD_MOTION_DONE` when |
|---|---|---|
| `S5_MOVE_FORWARD` | `s2_cmd_drive_dist(speed, +distance)` | `MFLG_CMD_DONE` |
| `S5_MOVE_REVERSE` | `s2_cmd_drive_dist(speed, -distance)` | `MFLG_CMD_DONE` |
| `S5_MOVE_TURN` | `s2_cmd_turn(0, angle_deg)` (0 = default turn speed) | `MFLG_CMD_DONE` |
| `S5_MOVE_STOP` | `s2_cmd_stop()` | `MFLG_STOPPED` |

`MFLG_FAULT` or `MFLG_SAFE_STOPPED` should answer with `ok = 0`.

## TEAM.md rows to claim

- **Pins:** GP8, GP9 (left motor M1A/M1B), GP10, GP11 (right motor M2A/M2B),
  GP5 (left encoder), GP3 (right encoder). GP2 and GP4 are left free.
- **Peripherals:** PWM slices 4 and 5 (motors), PWM slices 1 and 2
  (encoder counters), all owned by `motion_task`. Subsystem 5's servo
  uses slice 6, so there is no clash.
- **Tasks:** `motion_task` – 14 – core 1. Optional `s2_demo_task` – 15 – any.
- **Kernel objects:** 1 task, 1 message buffer, 1 mutex, 1 event flag,
  1 cyclic handler.

## What is stubbed or needs measuring

1. **Encoder wiring.** The Design Review puts the encoders on Grove 3
   (GP4/GP5) and Grove 2 (GP2/GP3). The hardware counter only works on
   the **odd** pin of each pair, so the encoder signal must land on GP5
   and GP3. If your Grove cable puts it on GP4/GP2, swap the wires or
   change the pins in `s2_config.h` to another odd GPIO.
2. **Single-channel encoders assumed.** Direction comes from the sign of
   the motor command. If yours are quadrature (A+B), only one channel is
   used for now.
3. **Every number in `s2_config.h` marked MEASURE** is a placeholder:
   ticks per turn (20), wheel diameter (65 mm), track width (130 mm),
   PID gains, minimum moving duty. Tests S2-M02 to S2-M06 measure them.
4. **Speed resolution.** With 20 ticks per turn, speed is only known to
   about ±100 mm/s over the 100 ms window, so the feed-forward gain does
   most of the work. If tuning hits this limit, the next step is a GPIO
   interrupt that timestamps each encoder edge and computes speed from
   the gap between edges.
5. **No heading hold in DRIVE mode.** Line following (Buddy 3) steers
   through `steer_mm_s`; straight-line correction only runs in
   DRIVE_DIST. The encoder heading in the status can drift; the IMU
   (Buddy 4) is the better heading source.
6. **Example objects kept.** `sem_slots`, `mpf_samples` and `SAMPLE_MSG`
   still exist only because the example code in subsystems 3 and 4 uses
   them. Set `S2_KEEP_EXAMPLE_IF` to 0 in `subsystem2.h` once those
   folders are replaced.
7. **DRIVE timeout (500 ms).** The Vehicle Controller must resend DRIVE
   at least every 500 ms or the robot stops. Set `S2_CMD_TIMEOUT_MS` to 0
   to turn this off while testing.

## Bench demo

Set `S2_DEMO` to 1 in `s2_config.h`. A demo task then drives forward
300 mm, turns right 90°, and repeats, with the status printed every
0.5 s. Put the robot on a stand the first time.
