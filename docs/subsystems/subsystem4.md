# Subsystem 4 base code: IMU-Based Motion and Terrain Monitoring

Buddy 4 (2500366). Hardware: LSM303DLHC accelerometer + magnetometer (no gyro) on
I2C0, SDA = GP0, SCL = GP1, as in the design review. Target: micro T-Kernel 3.0 on
the Pico W, using the kernel's own I2C driver from `Embedded-ProjNew.zip`.

## What is in this folder

Copy the folders over the repo with the same paths. They replace the example
"monitor" in `app_program/subsystem4/`.

| File | What it does |
|---|---|
| `app_program/subsystem4/subsystem4.h` | Public interface: event messages, status, API for other subsystems |
| `subsystem4.c` | Creates the kernel objects, starts `imu_task`, implements the public API |
| `task_imu.c` | `imu_task` (20 ms, priority 18, core 1) and its cyclic handler; fault handling |
| `drv_lsm303.c/.h` | LSM303DLHC driver over `tk_opn_dev("iica")` / `TDN_I2C_EXEC` |
| `imu_terrain.c/.h` | Pure-C calibration, filter, tilt, heading, motion class, Statechart 4 hump machine |
| `imu_math.c/.h` | Integer `atan2`, `sqrt` (the build has no libm or FPU) |
| `imu_selftest.c/.h` | Synthetic-data tests shared by the Pico test build and the PC tests |
| `subsystem4_cfg.h` | Every threshold, axis mapping and retry limit in one place |
| `subsystem4_test.c` | `make TEST=1` tests S4-U01..U11 |
| `testing/subsystem4/TEST_LOG.md` | Test cases S4-U01..U11 and manual S4-M01..M05, calibration record |
| `host_test/` | `make -C host_test` runs the algorithm tests on a PC (put it at `testing/subsystem4/host_test/`) |

## Interfaces it offers (add to TEAM.md section 5)

| ID | From → To | Mechanism | Object / call | Data |
|---|---|---|---|---|
| IF-S4-1 | S4 → Vehicle Controller | message buffer (owned by S4) | `g_mbf_imu_evt` | `imu_event_msg_t`: hump start/peak/end with height, impact, tilt limit, motion change, fault / fatal fault / recovered, with FR8 severity |
| IF-S4-2 | S4 → anyone (VC, telemetry) | shared data under mutex (owned by S4) | `s4_get_status()` | `imu_status_t`: pitch, roll, heading, turn rate, state, motion, current / last / max hump height, counts |
| IF-S4-3 | Motion control (S2) → S4 | function call | `s4_set_speed_mm_s()` | wheel speed in mm/s, used to turn pitch into height (-1 = unknown) |
| IF-S4-4 | VC / MQTT → S4 | event flag (owned by S4) | `s4_request_calibration()`, `s4_request_peak_reset()` | requests |

S4 never commands the motors. Impact and tilt events are tagged CRITICAL; the Vehicle
Controller decides on Safe Stop (FR8, review 8.3).

How hump height works: on a slope the forward accelerometer axis reads g·sin(pitch).
Each 20 ms the robot moves speed × 20 ms, so it climbs that distance × ax/|a|. Summing
from the foot of the hump gives the height and the maximum is the peak. This avoids
double-integrating vertical acceleration, which drifts. It needs the wheel speed from
subsystem 2.

## Shared files the team must change (by agreement)

1. `config/config_device.h`: `#define DEVCNF_USE_IIC 1` (the build stops with a clear
   `#error` until this is done).
2. `device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h`: `DEVCNF_I2C0_SDA_PIN 0`,
   `DEVCNF_I2C0_SCL_PIN 1` (currently 8/9, which the design review gives to motor 1).
3. `TEAM.md`: GP0/GP1 → S4; I2C0 → S4 / `imu_task`; priority band 18–19:
   `imu_task – 18 – core 1`; object budget S4 = 1 task, 1 cyclic, 1 mutex, 1 event flag,
   1 message buffer; the four interfaces above.

## Stubbed, assumed or still to do

- **Speed input is a stub until subsystem 2 calls `s4_set_speed_mm_s()`.** Until then
  height uses `S4_NOMINAL_SPEED_MM_S` (150 mm/s), so heights are only right at that speed.
- **All thresholds are first guesses** (`subsystem4_cfg.h`). Tune them with S4-M02/M03
  per assumption A5 and record the values in the TEST_LOG calibration table.
- **Axis mapping assumes** the board's X points forward and Z up. Check with S4-M01.
- **Heading is not tilt-compensated**, so turns are only classified on level ground.
- **Calibration assumes the robot is still and level at power-up**; it retries if it moves.
- **GP0/GP1 clash with `make CRASH_REPORT=1`** (UART0 console). Unplug the IMU for
  crash-report builds, or move I2C0 to GP4/GP5 (Grove 3) if motors move off them.
- **The example subsystem 3 still sends to S4's old `mbf_summary` / `flg_events`.**
  If you drop this in before subsystem 3 is replaced, the link fails; replace both together.
- Not compiled with the ARM toolchain or flashed here: there was no `arm-none-eabi-gcc`
  or Pico SDK in this environment. All 7 `.c` files pass a syntax check against the
  kernel headers (`gcc -m32 -fsyntax-only -Wall -Wextra`, I2C enabled), and the
  algorithm tests pass on a PC:

```
[HOST] S4-U06 PASS  integer atan2 / sqrt
[HOST] S4-U07 PASS  30 mm hump
[HOST] S4-U08 PASS  impact detection
[HOST] S4-U09 PASS  turn detection
[HOST] S4-U10 PASS  calibration rejects motion
height sweep (15 deg ramps, 150 mm/s): 10 mm -> 9.4, 30 mm -> 29.4, 80 mm -> 79.8
```
