# Subsystem 3 base code: IR line following and barcode navigation

Buddy 3's subsystem (Design Review "Buddy 3", Statechart 6.2), written for the
micro T-Kernel 3.0 Pico W project in `Embedded-ProjNew.zip`.

## How to install

1. Replace `app_program/subsystem3/` in the repo with the folder here (the
   example "consumer" code it replaces is no longer needed).
2. Copy `testing/subsystem3/` over the repo's copy.
3. Paste the rows in `TEAM_S3_ROWS.md` into `TEAM.md`.
4. Build as usual: `cd build_make && make -j8` (or `make TEST=1 -j8`).

The example subsystems 2 and 4 call the old consumer's `mbx_samples` and
`app_stats`, so they must be replaced too (their threads are doing that).
To build subsystem 3 on its own first, put the empty `subsystem5` template
in `subsystem2/` and `subsystem4/`.

## What was checked

- Firmware builds with no warnings for `CONSOLE=uart WIFI=none` in these
  variants: `SMP=1`, `SMP=0`, `TEST=1`, and `S3_IR_USE_ADC=1`. The default
  USB/WiFi build needs the Pico SDK, which was not available here, but
  subsystem 3 uses no SDK code.
- The build also links with subsystem 5's base code, and S3's
  `s5_port_line_detected()` replaces S5's weak stub.
- PC unit tests (`testing/subsystem3/host`, `make`): 16/16 pass.
- **Not run on a Pico yet.** The on-target tests S3-U01..U05 have only been
  built.

## Files (`app_program/subsystem3/`)

| File | What it does |
|---|---|
| `subsystem3.h` | Public interface: IF-31 status, IF-32 events, IF-33 requests |
| `s3_config.h` | Every tunable: pins, polarity, gains, thresholds, barcode map |
| `subsystem3.c` | Creates the kernel objects, starts the tasks, public calls |
| `task_line.c` | **Line Following task** (pri 16, 10 ms cyclic): reads IR, position, PD steering, lost/found/junction, barcode runs |
| `task_barcode.c` | **Barcode Detection task** (pri 17): collects bar/space widths, detects quiet zones, decodes, reports |
| `line_algo.c/.h` | Calibration, normalising, weighted position, PD, line state machine (pure C, PC-tested) |
| `code39.c/.h` | Code 39 decoder, forwards and backwards, per-character wide/narrow (pure C, PC-tested) |
| `drv_ir.c/.h` | IR driver: GPIO, optional ADC (`adca`), 1 MHz µs timer |
| `s3_ports.c` | Real `s5_port_line_detected()` for subsystem 5 |
| `subsystem3_test.c` | On-target tests S3-U01..U06 (`make TEST=1`) |

## How the Vehicle Controller uses it

```c
#include "../subsystem3/subsystem3.h"

s3_event_t evt;
if (tk_rcv_mbf(g_mbf_s3_events, &evt, TMO_POL) == sizeof(evt)) {
    switch (evt.type) {
    case S3_EVT_BARCODE_START:   /* slow down: DECODE_BARCODE */ break;
    case S3_EVT_BARCODE_DECODED: /* evt.nav: NAV_LEFT/RIGHT/... */ break;
    case S3_EVT_BARCODE_FAILED:  /* evt.arg = retries so far */ break;
    case S3_EVT_LINE_LOST:       /* evt.side = last seen side */ break;
    case S3_EVT_JUNCTION:        break;
    }
}

s3_line_status_t st;
s3_get_line_status(&st);         /* st.steering: -1000..+1000, + = right */

s3_request(S3_REQ_BC_DISABLE);   /* e.g. while avoiding an obstacle */
```

Subsystem 3 never drives the motors. It only sends a steering request, and
the controller passes it to motion control.

## Stubbed or guessed (check these on the robot)

- **Barcode symbology.** Code 39 is assumed (the usual choice for this kind
  of course). The barcode-to-command map (`L`/`R`/`S`/`U`) is a placeholder
  in `s3_config.h`. Replace it with the course's real codes.
- **Which sensor reads barcodes.** The pin table gives IR_L (GP06) to the
  barcode, but FR2 says barcodes are on the right of the track. The
  default is IR_L; change `S3_BC_SENSOR` if it is IR_R.
- **Line sensors.** The position uses C and R (`S3_LINE_MASK`), because L is
  the barcode sensor. The junction rule (all three black) and the setpoint
  depend on how the sensors are mounted.
- **Barcode timing resolution.** Widths are measured at the 10 ms line rate
  (`CNF_TIMER_PERIOD` is 10 ms). That works if the robot slows down over
  the barcode, which is what `BARCODE_START` is for. Better options later:
  measure widths in encoder ticks from subsystem 2, or use a GPIO edge
  interrupt on GP06.
- **Sensor polarity and gains.** `S3_IR_BLACK_LEVEL`, `S3_KP_NUM`/`S3_KD_NUM`
  and the debounce counts are first guesses marked `TUNE`.
- **ADC mode** (`S3_IR_USE_ADC=1`) needs `DEVCONF_ADC_PIN_INIT_0/1 = TRUE`
  in the shared ADC config. Calibration (`S3_REQ_CALIBRATE`) only matters in
  this mode.
- **Fault reporting.** IR read errors and task overruns are counted but not
  yet sent as a fault event (`TODO` in `task_line.c`).
- **Line recovery manoeuvres.** These belong to the controller and motion
  control. S3 only reports `LINE_LOST` with the side and `LINE_FOUND`.
