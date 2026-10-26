# BARR-C:2018 compliance notes (Group 10 robot car)

This file records how the application code was brought in line with the
Barr Group *Embedded C Coding Standard* (BARR-C:2018). It covers what
changed, how we checked that the robot still behaves the same, and the
rules we deliberately deviate from, with the reason for each.

## Scope

| In scope | Out of scope (left untouched) |
|---|---|
| `app_program/` (shared code, subsystems 1 to 6, the subsystem 6 host test) | `kernel/`, `device/`, `include/`, `lib/`, `etc/` (micro T-Kernel 3.0 and its BSP) |
| `build_make/mtkernel_3/app_program/subdir.mk` (warning flags only) | the Raspberry Pi Pico SDK |

The kernel and SDK are third-party code with their own style. Changing them
would make it harder to take upstream fixes, and BARR-C is aimed at the code
the team writes.

The hardware facts found during bring-up are unchanged: the IMU is an
LSM303D at I2C address 0x1D (the driver still auto-detects it), and both
motors stay inverted (`S2_MOTOR_L_INVERT = 1`, `S2_MOTOR_R_INVERT = 1`).

## What changed

The work is on the `barr-c` branch, one commit per step, so each kind of
change can be reviewed on its own.

| Commit | BARR-C rules | What it does |
|---|---|---|
| naming | 5.1, 6.1, 6.4, 6.5, 7.1 | Global variables get `g_`, pointers `p_`, double pointers `pp_`, booleans `b_`. Type names are lowercase and end in `_t` (for example `LineState` became `line_state_t` and `VC_EVENT` became `vc_event_t`). Interrupt and cyclic handlers end in `_isr`. Every name has at least 3 characters. |
| stdint/static | 5.2, 5.4 | The T-Kernel scalar types (`INT`, `UW`, `UB`, ...) became `<stdint.h>` types. `float` became `float32_t`. `LOCAL`, `EXPORT` and `IMPORT` became `static`, nothing and `extern`. |
| bool | 5.6 | `BOOL`, `TRUE` and `FALSE` became `bool`, `true` and `false` from `<stdbool.h>`. |
| parens | 1.4 | Every operand of `&&` and `||` is in parentheses, and so is any expression that mixes operator precedence. |
| yoda | 8.6 | In `==` comparisons the constant goes on the left (`E_OK == ercd`). |
| for (;;) | 8.4 | Infinite loops are written `for (;;)`. Also adds `app_program/.clang-format`. |
| layout | 1.2, 1.3, 3.1, 3.4 | Allman braces, braces on every `if`, `else`, `for` and `while` body, 4-space indent, no tabs, 80 columns, aligned declarations, `type * p_name` spacing. |
| one declaration per line | 8.1 | `int a, b;` became two lines. |
| subsystem 1 to 6 manual passes | 1.6, 1.7, 5.3, 6.2, 6.3, 8.2, 8.3, 8.4 | See the next table. |
| end-of-file markers, warnings | 4.1, 1.1 | Every file ends with `/*** end of file ***/`. App code now builds with `-Wall -Wextra`. |

The manual passes removed the patterns that a formatter cannot fix:

| Rule | Before | After |
|---|---|---|
| 1.7 no `goto` | `avoid_obstacle()` (S5) used `goto done` 3 times | split into `get_profile()`, `bypass()` and `report_result()` |
| 1.7 avoid `continue` | 16 uses (S3, S4, S5) | rewritten as `if` / `else if` / `else` chains, or the loop body moved into a helper |
| 6.2 functions of 100 lines or less | 10 functions over 100 lines, the largest `vc_sm_handle()` at 228 | all under 100 lines. For example `vc_sm_handle()` now calls one handler per state, `imu_task()` calls `recover_step()` and `sample_step()`, and `lsm303_init()` calls `init_lsm303d()` and `init_dlhc()` |
| 6.2 single exit (preferred) | `switch` statements that returned from every `case`, early returns in bridges and state machines | one `return` at the bottom, except the init and error paths listed under deviations |
| 6.3 functions, not macros | `CHECK()`, `S5_LOCKED(stmt)`, `ENC_SLICE()`/`ENC_REG()`, `N(a)` | `app_check()`, `s5_lock()`/`s5_unlock()`, `enc_slice()`/`enc_reg()`, `S5_ARRAY_LEN()` (kept as a macro, see deviations) |
| 8.2 no assignment in `if` | `if ((ercd = write_reg(...)) < E_OK)` 5 times in the LSM303 driver | a register table written in a loop |
| 8.2 `else` after `else if` | 12 chains without a final `else` | final `else` added, with a comment saying why nothing happens |
| 5.3 no bitwise operators on signed data | shifts on `int` in the IMU maths, terrain filter and LSM303 byte assembly | unsigned shifts or division where the operand is never negative; LSM303 bytes are assembled as `uint16_t` |
| 5.4 floating point | the PID output was passed on even if it was INFINITY or NaN | `motion_pid.c` now checks the output with `__builtin_isfinite()`. If it is not finite, the PID is reset and the output is 0. This is the only deliberate change in behaviour, and it only applies when a calculation has already gone wrong. |
| magic numbers | literals such as `1000`, `0x2000`, `343`, `0x14` | named constants, for example `S3_US_PER_MS`, `S2_FC_FAULT_BASE`, `SOUND_MM_PER_MS`, `SERVO_SLICE_STRIDE` |
| 1.6 casts | uncommented casts | arithmetic and narrowing casts in the files we reworked now have a comment saying why the value fits. Kernel API casts are covered by deviation D3. |
| 4.2 headers | shared globals declared with `extern` inside `.c` files (`g_vc_status_shared`, the S3 line counters) | the `extern` declarations moved into the subsystem's header |

### Names other subsystems use

Code that includes another subsystem's header has already been updated. If a
teammate has local work in progress, these are the renames most likely to
affect it (the full list is in the naming commit):

| Old | New |
|---|---|
| `CHECK(ercd, "what")` | `app_check(ercd, "what")` |
| `LineState`, `BarcodeState`, `NavigationCommand`, `IRReading` | `line_state_t`, `barcode_state_t`, `navigation_command_t`, `ir_reading_t` |
| `VC_EVENT`, `VC_STATUS`, `VC_STATE`, `VC_SOURCE`, `VC_MOVE` | `vc_event_t`, `vc_status_t`, `vc_state_t`, `vc_source_t`, `vc_move_t` |
| `MOTION_CMD`, `MOTION_STATUS`, `S3_EVENT`, `S3_LINE_STATUS`, `IMU_STATUS`, `IMU_EVENT_MSG`, `S5_CMD`, `S5_EVT`, `S5_STATUS` | the same names in lowercase with `_t` |
| `mbf_vc_events`, `mtx_vc_status`, `flg_vc_state`, `mbf_motion_cmd`, `flg_motion`, `mbf_s3_events`, `mbf_imu_evt`, `mbf_s5_cmd`, `mbf_s5_evt` | the same names with a `g_` prefix |
| `motion_cyclic_handler`, `s3_line_cyclic`, `imu_cyclic`, `telemetry_cyclic` | `motion_cyclic_isr`, `s3_line_cyclic_isr`, `imu_cyclic_isr`, `telemetry_cyclic_isr` |

`TEAM.md` and `docs/` have been updated to use the new names. The
`testing/*/TEST_LOG.md` files were left as they are, because they record
past test runs.

## How behaviour was checked

1. **Byte-identical firmware for the mechanical steps.** Every step from
   naming to one-declaration-per-line was rebuilt in all 9 build variants
   (default, `TEST=1`, `SMP=0`, `SMP=0 TEST=1`, `CONSOLE=uart WIFI=none`
   with and without `TEST=1`, `NET=lwip MQTT=1` with and without `TEST=1`,
   and `CONSOLE=uart WIFI=none SMP=0`). The `.bin` files were compared byte
   for byte with the build from before that step. For `TEST=1` builds,
   `__LINE__` in the test report was fixed to 0 in both trees, so moved lines
   did not count as differences. Every step matched except the `bool` step,
   where a different type size is expected. That step was checked by a
   script that confirmed every `BOOL` value is only used as true or false.
2. **Same results for the rewritten logic.** The functions that were split
   or restructured were compiled on a PC in their old and new forms and fed
   identical input:
   * IMU terrain algorithm (S4): 200,000 synthetic samples with humps,
     impacts, turns and sensor faults. The full status and event stream
     were identical, and so were the self-test results.
   * Vehicle Controller state machine (S6): 2,000,000 random events
     covering every state. The status and action stream were identical.
   * S6 host unit tests (`app_program/subsystem6/host_test`, run with `make`):
     41 of 41 pass.
   * The smaller rewrites in subsystems 1, 2, 3 and 5 (loops without
     `continue`, single-exit `switch` statements, test helpers, the S5
     avoidance task without `goto`) were reviewed line by line and
     compiled. There is no PC harness for them, so the on-target tests
     below are what confirms them.
3. **All builds, no warnings.** All 9 variants build with `-Wall -Wextra`
   with no warnings in app code. Each optional switch was also flipped and
   compiled once (`S3_IR_USE_ADC`, `S4_VERBOSE`, `VC_PORT_VERBOSE`,
   `S2_DEMO`, `S2_KEEP_EXAMPLE_IF`, `S5_BENCH_MODE`).
4. **Static analysis.** `cppcheck` (warning, style, performance and
   portability checks) reports nothing in app code apart from "the scope of
   a variable can be reduced", which BARR-C does not require.

Not yet done: flashing the new firmware and running the on-target tests
(`make TEST=1`) on the robot. Run those on the bench before the next track
session.

## Deviations

| # | Rule | Where | Why we deviate |
|---|---|---|---|
| D1 | 5.2 fixed-width types | `ER`, `ID`, `PRI`, `TMO`, `SZ`, `FP`, `SYSTIM`, `T_*`, and `UINT` for `tk_wai_flg()` flag patterns | These are micro T-Kernel API types. Kernel calls take and return them by pointer, so changing them would mean casting at every call. `usermain()` also keeps the kernel's `INT` return type. |
| D2 | 6.3 function-like macros | `TEST_INIT`, `TEST_CHECK`, `TEST_SUMMARY` (`test_report.h`) and `CHECK` (host test) | They need `__FILE__` and `__LINE__` from the caller and update the caller's local counters, which a function cannot do. |
| D2 | 6.3 function-like macros | `VC_READY_BIT()`, `VC_FLG_STATE()` | They are used in constant expressions (`VC_READY_ALL` and flag patterns that other subsystems build), where a function call is not allowed. |
| D2 | 6.3 function-like macros | `S5_ARRAY_LEN()` (S5 test) | `sizeof` must see the array itself. A function would only see a pointer. |
| D3 | 1.6 cast comments | `(uint8_t *)` on string literals passed to `tm_printf`, `tm_sprintf` and `tk_opn_dev`, `(FP)` on handler addresses, `(void)` on unused parameters | These casts are required by kernel prototypes and appear hundreds of times. This entry is the blanket justification. Casts that change a value's range are commented where they occur. |
| D4 | 5.3 bitwise operators on signed data | `convert_dlhc()` in `drv_lsm303.c` (`>> 4` on `int16_t`) | The DLHC accelerometer value is left-justified 12-bit data. An arithmetic right shift keeps its sign, which is the documented GCC behaviour on ARM. Replacing it with `/ 16` would round negative readings differently. This path only runs with an LSM303DLHC; the team's LSM303D uses `convert_lsm303d()`. |
| D4 | 5.3 | `PWM_CH_CSR_EN` and similar register macros from the kernel BSP | They are signed constants defined in vendor headers, which are out of scope. |
| D5 | 6.2 single exit (preferred practice) | `subsystemN_init()` and `subsystemN_start()`, the device drivers (`drv_*.c`), `s3_get_line_status()`, `s5_get_status()` | They return as soon as a kernel object cannot be created or a device cannot be opened. A single exit would need deeply nested `if` blocks. Every early return passes the kernel error code back unchanged. |
| D6 | 5.4 `isfinite()` from `<math.h>` | `motion_pid.c` | `<math.h>` clashes with the kernel's `size_t` definition, so the code uses the GCC built-in `__builtin_isfinite()` instead. |
| D7 | compiler extensions | `__attribute__((weak))` in `task_motion.c` (`s4_set_speed_mm_s`) and `s5_ports.c` (`s5_port_line_detected`) | This lets a subsystem link and run on its own until the subsystem that provides the real function is built in (TEAM.md section 5). |
| D8 | magic numbers in tests | `subsystem*_test.c` and the host test | Expected values such as `(samples >= 45U)` are left as literals so that each check reads like the requirement it tests. Production code uses named constants. |

## Keeping it compliant

* Format before committing: `clang-format -i app_program/<folder>/*.[ch]`
  (the `.clang-format` file is in `app_program/`).
* Build with `make` as usual. App code now compiles with `-Wall -Wextra`;
  treat any new warning as a bug.
* Follow the naming above for anything new: `g_` for globals, `p_` for
  pointers, `b_` for booleans, `_t` for types, `_task` for tasks, `_isr` for
  interrupt and cyclic handlers, and the subsystem prefix (`s3_`, `vc_`, ...)
  on public functions.
* These rules were applied from the published BARR-C:2018 rule set. Please
  check them against the team's own copy of the standard before the final
  submission.

/*** end of file ***/
