# Subsystem 1 — test log

**Subsystem:** WiFi communication, remote commands, MQTT telemetry  ·  **Student ID:** 2500577  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | `s1_post()` line comes out of the MQTT bridge with the full topic | IF-T1 | auto | `subsystem1_test.c` | `[TEST] S1-U01a/b PASS` |
| S1-U02 | telemetry_task queues a JSON line every 200 ms | NFR2 | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | MQTT commands are counted; unknown ones are ignored | IF-T2 | auto | `subsystem1_test.c` | `[TEST] S1-U03 PASS` |
| S1-M01 | telemetry reaches the broker | NFR2 | manual | 1. `make NET=lwip MQTT=1` with `config/wifi_secrets.h`. 2. `mosquitto_sub -t "robot/g10/#"` | JSON on `robot/g10/telemetry` every 200 ms |
| S1-M02 | remote start / stop reaches the Vehicle Controller | IF-T2 | manual | `mosquitto_pub -t robot/g10/cmd -m start`, then `stop` | console shows the mission state change; `st` field in telemetry follows |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
