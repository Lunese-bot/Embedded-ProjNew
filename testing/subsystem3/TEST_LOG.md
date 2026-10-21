# Subsystem 3 — test log

**Subsystem:** IR Line Following and Barcode Navigation  ·  **Student ID:** 2500193  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| H-C01..H-C08 | Code 39 decoder: all characters both directions, 2:1 ratio, ±20 % jitter, 2× speed ramp, bad length / contrast / framing / overflow rejected | FR2 | auto (PC) | `cd testing/subsystem3/host && make` | `[TEST] H-Cxx PASS` ×8 |
| H-L01..H-L05c | Line tracker: FOLLOW, steer sign, LOST after 5 samples with last side, junction + inhibit, calibration accept/reject | FR1, NFR9 | auto (PC) | same | `[TEST] H-Lxx PASS` ×8 |
| S3-U01 / U01b | Code 39 `*L*` decodes forwards and backwards on the Pico | FR2 | auto | `make TEST=1`, `subsystem3_test.c` | PASS |
| S3-U02 | corrupted barcode rejected on the Pico | FR2 | auto | same | PASS |
| S3-U03 | line on right → +1000, steer right | FR1 | auto | same | PASS |
| S3-U04 | line task runs every 10 ms (45–55 samples in 500 ms) | NFR2 | auto | same | PASS |
| S3-U05a/b | synthetic runs into `mbf_s3_bc_runs` → `BARCODE_START`, then `BARCODE_DECODED` with `NAV_LEFT` on `mbf_s3_events` | FR2, IF-32 | auto | same | PASS |
| S3-U06 | live sensor print: move robot over line, check `mask`/`pos` change and sign | FR1, A3 | manual | watch `[S3] state=… pos=…` lines | L/C/R bits follow the tape; pos −/+ on left/right |
| S3-U07 | real barcode read at test speed | FR2, A3 | manual | drive over each course barcode 10× at 2 speeds | ≥ 9/10 decoded, correct `nav` |
| S3-U08 | line lost / found on the track | FR1 | manual | lift sensor off tape, put back | `LINE_LOST` with correct side, then `LINE_FOUND` |
| S3-U09 | worst-case `line_task` execution time | NFR2 | manual | toggle a spare pin around one cycle, scope it | well under 10 ms; `overruns` stays 0 |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-15 | H-C01..H-L05c | base code | Claude (PC, gcc) | PASS 16/16 | `make` in `host/` | algorithms only, no robot |
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
