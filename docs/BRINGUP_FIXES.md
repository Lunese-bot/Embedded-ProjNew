# Bench bring-up fixes (2026-10-15)

Changes made while testing on the real robot:

- kernel/sysdepend/pico_rp2040/hw_setting.c: PWM block is always released from reset. It was only released when USE_PTMR=1 (it is 0), so the motors, encoder counters and servo never worked.
- app_program/subsystem2/s2_config.h: S2_MOTOR_L_INVERT and S2_MOTOR_R_INVERT = 1 (both wheels ran backwards).
- app_program/subsystem3/subsystem3.c: line_task and barcode_task moved to core 1 (TP_PRC2). On core 0 the USB console (pri 3) and CYW43 (pri 4) tasks starved them.
- app_program/subsystem3/task_line.c: loop/overrun/read-error counters exported for diagnostics.
- app_program/subsystem4/drv_lsm303.c/.h: auto-detects an LSM303D (0x1D/0x1E, WHO_AM_I 0x49) when no LSM303DLHC answers at 0x19. The team's IMU board is an LSM303D. Adds lsm303_scan() (I2C bus scan).
- Test-only (make TEST=1): S2-U06 measures a 500 ms window; S3-U04/S4-U03/S4-U05/S5-U11 print what they measured; S3-U05 pauses the Vehicle Controller; S4 runs the I2C scan.
