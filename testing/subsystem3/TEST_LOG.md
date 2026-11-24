# Subsystem 3 — test log

**Subsystem:** IR Line Following and Barcode Navigation  ·  **Student ID:** 2500193  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| H-C01..H-C08 | Code 39 decoder: all characters both directions, 2:1 ratio, ±20 % jitter, 2× speed ramp, bad length / contrast / framing / overflow rejected | FR2 | auto (PC) | `cd testing/subsystem3/host_test && make` | `[TEST] H-Cxx PASS` ×8 |
| H-L01..H-L05c | Line tracker: FOLLOW, steer sign, LOST after 5 samples with last side, junction + inhibit, calibration accept/reject | FR1, NFR9 | auto (PC) | same | `[TEST] H-Lxx PASS` ×8 |
| H-L25..H-L28 | Double black during rejoin and junction rearming | FR1, FR4 | auto (PC) | Feed double black, a brief gap, and a clear stretch to the line tracker | Keep steering during rejoin, classify a new junction after ten clear samples |
| H-W00..H-W13 | Adaptive barcode edge finder (`S3_BC_EDGE_ADAPT`): faint A/Z (white 145, ink < +100) at both print sizes, 60 and 120 mm/s, both ways give 29 elements and no wrong letter; old 300/400 sees no bar; white-only, tape crossing, start on ink, frozen white, missed samples, rearm, long black, 3 mm bar at cruise, learned noise sd | FR2 | auto (PC) | `cd testing/subsystem3/host_test && make` | `[TEST] H-Wxx PASS` ×14 |
| H-G10..H-G17 | Shape read with the read's noise margin (`c39g_decode_hump`, `c39g_min_hump`): same as before with 250; faint A/Z read (250 finds too few humps), none wrong; faint random bars, white only, tape crossing give no letter; real 21:41:13 pass reads A reversed | FR2 | auto (PC) | `cd testing/subsystem3/host_test && make` | `[TEST] H-Gxx PASS` ×8 |
| H-X01..H-X04 | Wrong-read guards on the whole read path (element decoders, then the shape read with the noise margin): faint A-Z both ways never give a wrong letter; a faint *A*/*Z* cut at 30-90 % gives none; a 1 s stop on the barcode gives no wrong letter; strong A and Z still read | FR2 | auto (PC) | `cd testing/subsystem3/host_test && make` | `[TEST] H-Xxx PASS` ×4 |
| S3-REPLAY | Saved dashboard L traces replayed through the real edge finder and decoders give exactly the letters in `replay/expected_letters.txt`, and no other letter | FR2 | auto (PC) | `cd testing/subsystem3/replay && make check` | `[TEST] S3-REPLAY PASS` |
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
| 2026-11-18 | host suite including H-L25..H-L28 | dffa830 + working tree | Codex | PASS 82/82 | [host log](logs/2026-11-18_ir_rejoin.txt) | The line tracker steers through the rejoin and detects the next junction after the sensors clear. |
| 2026-11-18 | host regression suite | dffa830 + working tree | Codex | PASS 82/82 | [host log](logs/2026-11-18_straight_junction.txt) | Line steering, junction inhibition and barcode algorithms pass. |
