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

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
