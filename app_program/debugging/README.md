# debugging/ - bench tools

## Browser remote control (`S2_WEBDEBUG`)

Drive the robot from a phone/laptop browser to check that the wheels really
turn forward, back, left and right. **Bench use only** (no password).

Files
- `web_debug.c/.h`        tiny HTTP server on port 80 (runs in the WiFi task)
- `web_debug_bridge.c/.h` turns a button into the normal `s2_cmd_drive()` /
  `s2_cmd_turn()` / `s2_cmd_stop()` calls and returns a one-line status

How to use
1. Copy `config/wifi_secrets.example.h` to `config/wifi_secrets.h` and fill in
   the WiFi name/password (already git-ignored).
2. In `app_program/subsystem2/s2_config.h` set `#define S2_WEBDEBUG 1`.
3. Build with `NET=lwip` (see the top-level README), flash, open the USB serial
   console and wait for `wifi: link up, ip=a.b.c.d` and `webdebug: open ...`.
4. On a device on the SAME WiFi open `http://a.b.c.d/`.
5. **Lift the robot (wheels in the air) for the first try.**
6. Hold a button to move; release to stop. STOP always works. "Reset" clears a
   FAULT/SAFE STOP. "Turn 90 L/R" runs the real encoder-based turn.

Safety: holding a button re-sends DRIVE every 200 ms. If the browser or WiFi
drops, `S2_CMD_TIMEOUT_MS` (500 ms) stops the robot. Set `S2_WEBDEBUG` back to
0 for line-following runs; the Vehicle Controller can also send commands, and
the WiFi task shares core 0 with the USB console.

What to look for (status line under the buttons)
| Button  | Expect                                                 |
|---------|--------------------------------------------------------|
| Forward | both wheels forward, `L` and `R` speed > 0, ticks rising |
| Back    | both wheels backward, speeds < 0, ticks still count      |
| Left    | left wheel back, right wheel forward, heading increases  |
| Right   | left wheel forward, right wheel back, heading decreases  |

Wheel wrong way -> flip `S2_MOTOR_x_INVERT` in `s2_config.h`. Wheel moves but
ticks stay 0 -> encoder pin/wiring (`S2_PIN_ENC_x`). Drifts when going
straight -> tune `S2_TRIM_L/R_PERMILLE`.

## Dashboard
Below the buttons the page shows live cards, refreshed about twice a second:
Motion (state, wheel speed/target/duty, ticks, distance, heading), Line
sensors (state, position bar, error, raw IR, barcode counters), IMU (state,
motion, pitch/roll, heading, humps), Obstacle (distance, pings, timeouts),
Vehicle controller (mission state, time in state, barcodes, wifi) and the
debug settings. A subsystem that is not running shows "not running".
The data comes only from each subsystem's public status function
(`s2_get_status`, `s3_get_line_status`, `s4_get_status`, `s5_get_status`,
`vc_get_status`); `GET /s` returns the same data as JSON, so it can also be
read with `curl http://<ip>/s`. To add a field, edit `web_debug_snapshot()`
in `web_debug_bridge.c` and the matching card in the page in `web_debug.c`.

## Live tuning (no rebuild)
Under the buttons: **Trim L -/+** and **Trim R -/+** change the per-wheel trim
by 5 permille per press while the robot runs (the status line shows `trimL`,
`trimR`). **Straight on/off** switches a heading hold for plain Forward/Back
(it slows the wheel that has travelled further since the drive began, like
DRIVE_DIST). Values reset on reboot: when the robot drives straight, copy
`trimL`/`trimR` into `S2_TRIM_L_PERMILLE` / `S2_TRIM_R_PERMILLE` in
`s2_config.h` and rebuild once. The hold exists only when `S2_WEBDEBUG` is 1.

## Recommended debugging order
1. Wheels in the air: direction of each wheel (browser buttons).
2. Encoder ticks per wheel while driving (status line / `S2_PRINT_EVERY`).
3. Straight line over ~1 m: note the drift, adjust trim a little at a time.
4. 4 x 90 degree turns: does it face the start direction again?
5. Only then enable line sensors and the Vehicle Controller.
Change one thing at a time and write down the value and the result.
