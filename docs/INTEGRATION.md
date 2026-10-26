# Robot base code: integrated tree (Group 10)

This tree is the course `Embedded-ProjNew` project with every subsystem's
base code installed and wired together. It builds with no warnings in every
variant (see the end). **Nothing has run on a real robot yet.**

Per-subsystem notes (install steps, stubs, values to measure) are in
`docs/subsystems/`. Each buddy should search their folder for `TODO` and
`MEASURE` / `TUNE`.

## Where each piece came from

| Piece | Source |
|---|---|
| `subsystem1/` WiFi / MQTT telemetry + remote commands | "Micro-T Kernel Base Code" thread, **ported** to talk to the subsystem 6 Vehicle Controller (see below) |
| `subsystem2/` motion control | subsystem 2 thread, unchanged except `S2_KEEP_EXAMPLE_IF 0` |
| `subsystem3/` IR line + barcode | subsystem 3 thread, unchanged |
| `subsystem4/` IMU terrain | subsystem 4 thread, one test-file type fix (`int32_t` for `imu_selftest_hump`) |
| `subsystem5/` ultrasonic obstacle | subsystem 5 thread, `S5_BENCH_MODE 0` (controller is present) |
| `subsystem6/` Vehicle Controller | subsystem 6 thread; `vc_port_report()` now sends faults to MQTT via `s1_post("fault", ...)` |
| `lib/liblwip/.../mqtt_utk.c` | "Micro-T Kernel Base Code" thread (subscribes `robot/g10/cmd`, publishes S1's queue) |
| `app.h`, `app_main.c` | subsystem 6's shared patch (adds `subsystem6_init/start/test`), plus updated header comment |
| `config/config_device.h` | `DEVCNF_USE_IIC 1` (IMU on the kernel I2C driver) |
| `device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h` | I2C0 on GP0/GP1 (GP8/9 are motor pins) |
| `TEAM.md`, `testing/subsystemN/` | rows and test logs from every thread |

The base-code thread's own versions of subsystems 2-5, its
`app_program/controller/` and `app_program/common/hw.c` are **not** used:
the dedicated subsystem threads' code is used instead, and subsystem 6 is
the only Vehicle Controller.

## Conflicts between threads and how they were settled

| Topic | Threads disagreed | Used here |
|---|---|---|
| Vehicle Controller | base thread `controller/` vs subsystem 6 | subsystem 6 only |
| System tick | base thread 1 ms vs everyone else 10 ms | **10 ms** (unchanged `CNF_TIMER_PERIOD`): S2-S6 were written and tested against it, and all loops use 10/20 ms cyclic handlers. Change it only by team agreement. |
| Wheel encoder pins | base thread GP4/GP2 vs subsystem 2 GP5/GP3 | **GP5 / GP3** (subsystem 2): the PWM counter only works on odd pins |
| ADC pins GP26/27 | base thread enabled them | **left off**: subsystem 3 defaults to digital IR (`S3_IR_USE_ADC 0`). Set `DEVCONF_ADC_PIN_INIT_0/1 TRUE` in `device/adc/sysdepend/rp2040/adc_cnf_sysdep.h` only together with `S3_IR_USE_ADC 1`. |
| Ultrasonic period | base thread 60 ms vs subsystem 5 20 ms | 20 ms (subsystem 5); lengthen `S5_PERIOD_MS` if you see ghost echoes |
| Telemetry task priority | base thread 18 (clashes with `imu_task`) | **12**, subsystem 1's own band |
| LED heartbeat on GP19 | course example | removed; GP19 is free |
| MQTT `speed N` command | base thread only | dropped: subsystem 6 has no speed event yet (TODO for Buddy 1 + team) |

## Still for the team

- Agree the shared-file changes above (TEAM.md rule) and the proposed
  priority band 10-11 for the Vehicle Controller.
- Measure and tune every `MEASURE` / `TUNE` value (wheel size, encoder
  pulses, PID, IR thresholds, IMU axes, servo trim, distances).
- Replace the placeholder barcode table in `subsystem3/s3_config.h` and
  add an "end of course" barcode for `VC_EV_END_OF_COURSE`.
- GP0/GP1 are the IMU: unplug it for `make CRASH_REPORT=1` builds.
- The ultrasonic ECHO pin (GP16) needs a 5 V to 3.3 V divider.
- Copy `config/wifi_secrets.example.h` to `config/wifi_secrets.h` for the
  MQTT build.

## Build check

All of these build with no errors or warnings (arm-none-eabi-gcc 13.2,
pico-sdk 2.2.0 for the USB/WiFi variants):

```sh
cd build_make
make -j8                          # USB console + WiFi (needs PICO_SDK_PATH)
make TEST=1 -j8
make SMP=0 -j8
make CONSOLE=uart WIFI=none -j8   # no SDK needed
make CONSOLE=uart WIFI=none TEST=1 -j8
make NET=lwip MQTT=1 -j8          # needs config/wifi_secrets.h
```

PC tests also pass: `testing/subsystem3/host` (16/16),
`testing/subsystem4/host_test`, `app_program/subsystem6/host_test` (41/41).

## Dashboard

`tools/dashboard.html`: open it in Chrome or Edge, press **Connect USB** and
pick the Pico. It shows the self-test results per subsystem, the live
telemetry (subsystem 1 prints one JSON line a second on the console while
MQTT is not connected, `S1_CONSOLE_EVERY`), charts of the last 60 s and the
controller / IMU / obstacle events. With the MQTT build it can read
`robot/g10/#` from a WebSocket broker instead and send start / stop / reset.
In `TEST=1` USB builds the test run now waits (up to 30 s) until a terminal
or the dashboard opens the port, so no results are missed.
