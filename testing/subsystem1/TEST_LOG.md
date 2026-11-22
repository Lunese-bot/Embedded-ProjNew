# Subsystem 1 — test log

**Subsystem:** WiFi communication, remote commands, MQTT telemetry  ·  **Student ID:** 2500577  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | `s1_post()` binary payload, including an embedded NUL, comes out of the MQTT bridge with the full topic and length | IF-T1 | auto | `subsystem1_test.c` | `[TEST] S1-U01a/b PASS` |
| S1-U02 | telemetry_task queues a valid 14-byte frame every 200 ms | NFR2 | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | reserved binary command is counted and rejected | IF-T2 | auto | `subsystem1_test.c` | `[TEST] S1-U03 PASS` |
| S1-M01 | binary telemetry reaches the broker | NFR2 | manual | 1. Build with `NET=lwip MQTT=1` and local `config/wifi_secrets.h`. 2. `mosquitto_sub -t "robot/g10/#" -F "%t %x"` | 14-byte hex payload on `robot/g10/t` every 200 ms |
| S1-M02 | binary start / stop / reset reaches the Vehicle Controller | IF-T2 | manual | Publish one byte to `robot/g10/cmd`; top bits `00` start, `01` stop, `10` reset; low six bits sequence | One-byte ack appears on `robot/g10/ack`; VC state confirms whether event executes |
| S1-M03 | invalid command frame is rejected and valid command ack echoes its sequence | IF-T2 | manual | Publish `C1` (reserved, sequence 1) and `01` (start, sequence 1) | Ack `81` for rejection and `01` for queued start |
| S1-M04 | recovery after broker restart | IF-T1, IF-T2 | manual | With the robot secured, restart the broker; observe telemetry, then send start / stop / reset | Telemetry resumes; commands are acknowledged and execute |
| S1-M05 | recovery after Wi-Fi interruption | IF-T1 | manual | With the robot secured, interrupt and restore the hotspot; observe telemetry | Telemetry resumes after reconnecting |
| S1-M06 | MQTT dashboard integration | IF-T1, IF-T2 | manual | Connect the local dashboard over MQTT WebSockets; compare live state with the serial console and test commands with the robot secured | Values update, state agrees with serial, and commands work |
| S1-M07 | stale telemetry after reconnect | IF-T1 | manual | Observe sequence numbers and timestamps before, during and after a broker interruption | Old offline snapshots are not replayed; current implementation does not yet guarantee this |
| S1-M08 | pause / resume extension | IF-T2 | manual | Only after Buddy 2 reviews the stop path and remote pause is enabled: test with wheels safely secured, then under supervised track conditions | Pause accepted only in FOLLOW_LINE, resume only after confirmed stop; unsafe events cause FAULT |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-19 | S1-M01 | Not recorded | User | PARTIAL | User-shared `robot/g10/t` output | Repeated 14-byte payloads received; 200 ms interval not measured |
| 2026-11-22 | S1-H01..H12 | `920f348` | Copilot | PASS | Host GCC run: 12 passed, 0 failed | Codec tests only; no hardware involved |
| 2026-11-22 | S1-M02 | Not recorded | User | PASS | User-shared command and acknowledgement output; execution confirmed by user | `02`, `43`, `84` acknowledged as `02`, `03`, `04`; user confirmed execution |
| 2026-11-22 | S1-M03 | Not recorded | User | PASS | User-shared command and acknowledgement output | Reserved `C1` rejected as `81`; start `02` acknowledged as `02` |
| 2026-11-22 | S1-M04 | Not recorded | User | PASS | User report | Telemetry resumed after broker restart; commands and execution confirmed afterward |
| 2026-11-22 | S1-M05 | Not recorded | User | PASS | User report | Telemetry resumed after hotspot interruption |
| 2026-11-22 | S1-M06 | Not recorded | User | PASS | User report | User reported dashboard display and commands worked |
| 2026-11-22 | S1-H01..H15 | Uncommitted changes | Copilot | PASS | Host GCC run: 15 passed, 0 failed | Includes extension decoding and paused flag; not a hardware test |

On-target automated tests S1-U01 through S1-U03 have not been reported.
S1-M07 and S1-M08 have not been run. Previous hardware results above apply
to the earlier firmware, not the current uncommitted changes.
