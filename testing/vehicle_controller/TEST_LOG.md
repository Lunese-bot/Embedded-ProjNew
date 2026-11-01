# Vehicle Controller — test log

**Subsystem:** Vehicle Controller / Mission Control  ·  **Owner:** whole team (shared)  ·  **Code:** `app_program/vehicle_controller/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

**On target** (`make TEST=1`, `vehicle_controller_test.c`). VC-U04..U14 drive a private
state machine, so the motors are never commanded.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| VC-U01 | kernel objects and task created | IF-06, IF-07 | auto | `vehicle_controller_test.c` | `[TEST] VC-U01 PASS` |
| VC-U02 | live controller waits in IDLE | FR7 | auto | `vehicle_controller_test.c` | `[TEST] VC-U02 PASS` |
| VC-U03 | `vc_post_event` accepted | IF-06 | auto | `vehicle_controller_test.c` | `[TEST] VC-U03 PASS` |
| VC-U04 | start + all READY -> FOLLOW_LINE, drive forward | FR7 | auto | `vehicle_controller_test.c` | PASS |
| VC-U05 | self check: motion/line missing -> FAULT | FR8, NFR8 | auto | `vehicle_controller_test.c` | PASS |
| VC-U06 | barcode LEFT -> EXECUTE_ACTION, turn left | FR2, FR7 | auto | `vehicle_controller_test.c` | PASS |
| VC-U07 | manoeuvre done -> FOLLOW_LINE | FR7 | auto | `vehicle_controller_test.c` | PASS |
| VC-U08 | line not found within limits -> FAULT | NFR5, NFR8 | auto | `vehicle_controller_test.c` | PASS |
| VC-U09 | AVOID: move request from S3 rejected | 8.3 command arbitration | auto | `vehicle_controller_test.c` | PASS |
| VC-U10 | AVOID: move request from S5 forwarded | 8.3 command arbitration | auto | `vehicle_controller_test.c` | PASS |
| VC-U11 | WiFi down is non-critical: keep following | FR8, NFR5 | auto | `vehicle_controller_test.c` | PASS |
| VC-U12 | critical fault -> FAULT, motors stopped | FR8, NFR8 | auto | `vehicle_controller_test.c` | PASS |
| VC-U13 | reset from FAULT -> IDLE | 6.1 fault table | auto | `vehicle_controller_test.c` | PASS |
| VC-U14 | hump: line loss ignored, peak recorded | FR5, section 5 table | auto | `vehicle_controller_test.c` | PASS |
| VC-U15 | STOP reaches the motors within one tick (20 ms) of a critical fault | NFR2, NFR8 | manual | scope on a motor PWM pin, inject `VC_EV_FAULT` CRITICAL from a test task | PWM off ≤ 20 ms after the event |

**On a PC** (`app_program/vehicle_controller/host_test`, run `make`). Same state machine
code, no Pico needed: VC-H01 … VC-H41, one `[TEST]` line each (see `test_vc_sm.c`).

| ID | What is tested | Method | Steps / how | Expected result |
|---|---|---|---|---|
| VC-H01..H41 | every transition, time limit, fault rule and arbitration rule of `vc_sm.c` | auto (PC) | `cd app_program/vehicle_controller/host_test && make` | `SUMMARY 41 passed, 0 failed` |

**Integration** (add to `testing/integration/TEST_LOG.md` with the other owner):

| ID | What is tested | Interface | Method | Expected result |
|---|---|---|---|---|
| INT-VC-01 | S5 obstacle -> controller stops, S5 bypass moves reach S2, MOTION_DONE returns | S5 bridge | manual, obstacle on track | car stops, bypasses, back to FOLLOW_LINE |
| INT-VC-02 | S4 hump -> TERRAIN_CHECK, slow, peak in `vc_get_status().max_hump_um` | S4 bridge | manual, hump on track | state log shows TERRAIN_CHECK -> FOLLOW_LINE |
| INT-VC-03 | S3 steering forwarded only in FOLLOW_LINE/LOST/DECODE/TERRAIN | IF-06 | auto once S3 posts events | rejected count rises in AVOID |
| INT-VC-04 | MQTT remote stop -> FAULT, reset -> IDLE | IF-06 (S1) | manual, broker | car stops, telemetry shows FAULT |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-15 | VC-H01..H41 | (base code, not committed yet) | Claude | PASS 41/41 | host run, gcc 13 | PC only; on-target tests not yet run |
