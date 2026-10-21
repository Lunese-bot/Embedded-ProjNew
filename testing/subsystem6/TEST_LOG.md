# Subsystem 6 — test log

**Subsystem:** Vehicle Controller / Mission Control  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem6/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

**On target** (`make TEST=1`, `subsystem6_test.c`). S6-U04..U14 drive a private
state machine, so the motors are never commanded.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S6-U01 | kernel objects and task created | IF-06, IF-07 | auto | `subsystem6_test.c` | `[TEST] S6-U01 PASS` |
| S6-U02 | live controller waits in IDLE | FR7 | auto | `subsystem6_test.c` | `[TEST] S6-U02 PASS` |
| S6-U03 | `vc_post_event` accepted | IF-06 | auto | `subsystem6_test.c` | `[TEST] S6-U03 PASS` |
| S6-U04 | start + all READY -> FOLLOW_LINE, drive forward | FR7 | auto | `subsystem6_test.c` | PASS |
| S6-U05 | self check: motion/line missing -> FAULT | FR8, NFR8 | auto | `subsystem6_test.c` | PASS |
| S6-U06 | barcode LEFT -> EXECUTE_ACTION, turn left | FR2, FR7 | auto | `subsystem6_test.c` | PASS |
| S6-U07 | manoeuvre done -> FOLLOW_LINE | FR7 | auto | `subsystem6_test.c` | PASS |
| S6-U08 | line not found within limits -> FAULT | NFR5, NFR8 | auto | `subsystem6_test.c` | PASS |
| S6-U09 | AVOID: move request from S3 rejected | 8.3 command arbitration | auto | `subsystem6_test.c` | PASS |
| S6-U10 | AVOID: move request from S5 forwarded | 8.3 command arbitration | auto | `subsystem6_test.c` | PASS |
| S6-U11 | WiFi down is non-critical: keep following | FR8, NFR5 | auto | `subsystem6_test.c` | PASS |
| S6-U12 | critical fault -> FAULT, motors stopped | FR8, NFR8 | auto | `subsystem6_test.c` | PASS |
| S6-U13 | reset from FAULT -> IDLE | 6.1 fault table | auto | `subsystem6_test.c` | PASS |
| S6-U14 | hump: line loss ignored, peak recorded | FR5, section 5 table | auto | `subsystem6_test.c` | PASS |
| S6-U15 | STOP reaches the motors within one tick (20 ms) of a critical fault | NFR2, NFR8 | manual | scope on a motor PWM pin, inject `VC_EV_FAULT` CRITICAL from a test task | PWM off ≤ 20 ms after the event |

**On a PC** (`app_program/subsystem6/host_test`, run `make`). Same state machine
code, no Pico needed: S6-H01 … S6-H41, one `[TEST]` line each (see `test_vc_sm.c`).

| ID | What is tested | Method | Steps / how | Expected result |
|---|---|---|---|---|
| S6-H01..H41 | every transition, time limit, fault rule and arbitration rule of `vc_sm.c` | auto (PC) | `cd app_program/subsystem6/host_test && make` | `SUMMARY 41 passed, 0 failed` |

**Integration** (add to `testing/integration/TEST_LOG.md` with the other owner):

| ID | What is tested | Interface | Method | Expected result |
|---|---|---|---|---|
| INT-S6-01 | S5 obstacle -> controller stops, S5 bypass moves reach S2, MOTION_DONE returns | S5 bridge | manual, obstacle on track | car stops, bypasses, back to FOLLOW_LINE |
| INT-S6-02 | S4 hump -> TERRAIN_CHECK, slow, peak in `vc_get_status().max_hump_um` | S4 bridge | manual, hump on track | state log shows TERRAIN_CHECK -> FOLLOW_LINE |
| INT-S6-03 | S3 steering forwarded only in FOLLOW_LINE/LOST/DECODE/TERRAIN | IF-06 | auto once S3 posts events | rejected count rises in AVOID |
| INT-S6-04 | MQTT remote stop -> FAULT, reset -> IDLE | IF-06 (S1) | manual, broker | car stops, telemetry shows FAULT |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-15 | S6-H01..H41 | (base code, not committed yet) | Claude | PASS 41/41 | host run, gcc 13 | PC only; on-target tests not yet run |
