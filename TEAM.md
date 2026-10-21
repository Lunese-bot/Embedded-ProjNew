# Team and subsystems

The tables below are filled in from the robot base code (five buddy
subsystems plus the Vehicle Controller in `subsystem6/`). Keep them up to
date as the code changes.

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | WiFi communication, remote commands, MQTT telemetry | 2500577 | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion control: motors, encoders, PID | 2503000 | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | IR line following + barcode navigation | 2500193 | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU motion and terrain monitoring (LSM303DLHC) | 2500366 | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Ultrasonic obstacle detection and avoidance | 2503330 | | `app_program/subsystem5/` | `testing/subsystem5/` |
| 6 | Vehicle Controller: mission state, sole movement authority (team) | | | `app_program/subsystem6/` | `testing/subsystem6/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S4 | I2C0 SDA, IMU (clashes with `CRASH_REPORT=1`: unplug the IMU for those builds) |
| GP1 | S4 | I2C0 SCL, IMU (clashes with `CRASH_REPORT=1`) |
| GP2 | | (Grove 2, left free by S2) |
| GP3 | S2 | right wheel encoder (PWM slice 1 B counter) |
| GP4 | | (Grove 3, left free by S2) |
| GP5 | S2 | left wheel encoder (PWM slice 2 B counter) |
| GP6 | S3 | IR_L, barcode sensor (digital in) |
| GP7 | | |
| GP8 | S2 | left motor M1A (PWM slice 4) |
| GP9 | S2 | left motor M1B (PWM slice 4) |
| GP10 | S2 | right motor M2A (PWM slice 5) |
| GP11 | S2 | right motor M2B (PWM slice 5) |
| GP12 | S5 | scan servo (PWM slice 6 A, 50 Hz; GP13 shares the slice) |
| GP13 | | |
| GP14 | | |
| GP15 | | |
| GP16 | S5 | ultrasonic ECHO (input, via 5 V → 3.3 V divider) |
| GP17 | S5 | ultrasonic TRIG (output) |
| GP18 | | |
| GP19 | | |
| GP20 | | |
| GP21 | | |
| GP22 | | |
| GP26 | S3 | IR_C, line position (digital in, or ADC0 when `S3_IR_USE_ADC=1`) |
| GP27 | S3 | IR_R, line position (digital in, or ADC1 when `S3_IR_USE_ADC=1`) |
| GP28 | | (ADC2 capable) |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 (kernel `iica` driver) | S4 | `imu_task` | GP0, GP1 |
| I2C1 | | | |
| SPI0 | | | |
| SPI1 | | | |
| ADC (only when `S3_IR_USE_ADC=1`) | S3 | `line_task` | GP26, GP27 |
| PWM slices 1, 2 (encoder counters), 4, 5 (motors) | S2 | `motion_task` | GP3, GP5, GP8–GP11 |
| PWM slice 6 | S5 | `s5_scan_task` | GP12 |
| TIMER (read only, TIMERAWL) | shared, read only | S3 `line_task`, S5 `s5_scan_task` | — |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in); S1 code runs inside it via `mqtt_utk.c` → `s1_bridge.c` | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students can do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Exchange application data with the existing MQTT service** through
  `app_program/subsystem1/`; application tasks never call lwIP or MQTT
  directly. Any task queues a binary message with `s1_post()` into the
  bounded `mbf_telemetry` buffer. The service polls it every 100 ms without
  waiting (up to 4 messages per poll) and publishes under `robot/g10/`. If the
  queue is full the oldest message is dropped and counted. `robot/g10/cmd`
  commands (one byte) go to the Vehicle Controller as events (`start`, `stop`,
  `pause`, `resume`). Formats: `app_program/subsystem1/s1_codec.h`.
  This bridge requires `NET=lwip MQTT=1`.

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 6 | 10–11 (**proposed**) | vehicle_ctrl – 10 – any |
| 1 | 12–13 | telemetry_task – 12 – any |
| 2 | 14–15 | motion_task – 14 – core 1; optional s2_demo_task – 15 – any (`S2_DEMO`) |
| 3 | 16–17 | line_task – 16 – core 0; barcode_task – 17 – core 0 |
| 4 | 18–19 | imu_task – 18 – core 1 |
| 5 | 20–21 | s5_avoid_task – 20 – any; s5_scan_task – 21 – any (design review rates these High: team to agree) |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-T1 | any → S1 | message buffer (owned by S1) | `mbf_telemetry`, `s1_post(topic, data, len)` | topic suffix + packed bytes (`s1_codec.h`); drained by `mqtt_utk.c` with `TMO_POL` | |
| IF-T2 | S1 → S6 | S6's event buffer | `vc_post_event(VC_SRC_COMMS, ...)` | MQTT `robot/g10/cmd` byte: start / stop / pause / resume; WiFi up/down | |
| IF-M1 | S6 → S2 | message buffer (owned by S2) | `mbf_motion_cmd` | `MOTION_CMD`, or `s2_cmd_drive/drive_dist/turn/stop/reset()` | |
| IF-M2 | S2 → anyone | shared data under mutex (owned by S2) | `mtx_motion`, `s2_get_status()` | `MOTION_STATUS` | |
| IF-M3 | S2 → S6 | event flag (owned by S2) | `flg_motion` | `MFLG_CMD_DONE`, `_STOPPED`, `_SAFE_STOPPED`, `_FAULT`, `_CMD_TIMEOUT` | |
| — | any → S2 | function | `s2_emergency_stop()` | latched Safe Stop until `MOTION_RESET` | |
| IF-31 | S3 → anyone | shared data under mutex (owned by S3) | `s3_get_line_status()`, `mtx_s3_status` | `S3_LINE_STATUS` | |
| IF-32 | S3 → S6 | message buffer (owned by S3) | `mbf_s3_events` | `S3_EVENT`: line lost/found, junction, barcode start/decoded/failed | |
| IF-33 | S6 → S3 | event flag (owned by S3) | `s3_request()`, `flg_s3_req` | `S3_REQ_CALIBRATE`, `_RESET_LINE`, `_BC_ENABLE/DISABLE/REARM` | |
| IF-S4-1 | S4 → S6 | message buffer (owned by S4) | `mbf_imu_evt` | `IMU_EVENT_MSG`: hump, impact, tilt, faults (FR8 severity) | |
| IF-S4-2 | S4 → anyone | shared data under mutex (owned by S4) | `s4_get_status()` | `IMU_STATUS` | |
| IF-S4-3 | S2 → S4 | function (owned by S4) | `s4_set_speed_mm_s()` | measured speed every 10 ms, for hump height | |
| IF-S4-4 | S6 → S4 | event flag (owned by S4) | `s4_request_calibration()`, `s4_request_peak_reset()` | requests | |
| IF-S5a | S6 → S5 | message buffer (owned by S5) | `mbf_s5_cmd` | `S5_CMD` | |
| IF-S5b | S5 → S6 | message buffer (owned by S5) | `mbf_s5_evt` | `S5_EVT` (incl. `S5_MOTION_REQ`) | |
| IF-S5c | S5 → anyone | function + mutex | `s5_get_status()` | `S5_STATUS` | |
| IF-S5d | S3 → S5 | function overriding S5's weak hook | `s5_port_line_detected()` | TRUE when an IR sensor sees black | |
| IF-06 | any → S6 | message buffer (owned by S6) | `mbf_vc_events`, `vc_post_event()` | `VC_EVENT` in `vc_types.h` | |
| IF-07 | S6 → anyone | shared data under mutex + event flag (owned by S6) | `mtx_vc_status`, `flg_vc_state` | `VC_STATUS`, via `vc_get_status()` | |
| IF-08 | S6 → S1 | S1's telemetry buffer | `s1_post("f", ...)` in `vc_port_report()` | 3-byte fault frame to MQTT `robot/g10/f` | |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are the robot's usage.

| Object | Limit | S1 | S2 | S3 | S4 | S5 | S6 |
|---|---|---|---|---|---|---|---|
| Task | 32 | 1 | 1 (+1 demo) | 2 | 1 | 2 | 1 |
| Semaphore | 16 | | | | | | |
| Event flag | 16 | | 1 | 1 | 1 | 1 | 1 |
| Mailbox | 12 | | | | | | |
| Mutex | 12 | | 1 | 1 | 1 | 1 | 1 |
| Message buffer | 12 | 1 | 1 | 2 | 1 | 2 | 1 |
| Fixed memory pool | 12 | | | | | | |
| Variable memory pool | 12 | | | | | | |
| Cyclic handler | 12 | 1 | 1 | 1 | 1 | | |
| Alarm handler | 12 | | | | | | |

(The kernel's USB and WiFi service tasks also use a few of these.)
