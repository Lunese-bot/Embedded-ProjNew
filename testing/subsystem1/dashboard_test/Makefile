# Dashboard checks: tools/dashboard.html against the firmware sources.
#   make -C testing/subsystem1/dashboard_test run
# Needs a C compiler and Node.js 18 or newer (no npm packages).
SRC := ../../../app_program/subsystem1
VC  := ../../../app_program/vehicle_controller
CFLAGS := -std=c11 -Wall -Wextra -Werror -O2 -I$(SRC)

run: tlm_encode cfg_encode obst_encode
	@if command -v node >/dev/null 2>&1; then \
	    node test_dashboard.js ./tlm_encode ./cfg_encode; \
	else \
	    echo "[TEST] dashboard SKIPPED (node not found; not a pass)"; exit 1; \
	fi

tlm_encode: tlm_encode.c $(SRC)/s1_codec.c $(SRC)/s1_codec.h $(SRC)/s1_stats.c $(SRC)/s1_stats.h
	$(CC) $(CFLAGS) -o $@ tlm_encode.c $(SRC)/s1_codec.c $(SRC)/s1_stats.c

# barcode map protocol: s1_cfg.c and vc_barcode_map.c as the robot builds them
cfg_encode: cfg_encode.c $(SRC)/s1_cfg.c $(SRC)/s1_cfg.h $(VC)/vc_barcode_map.c $(VC)/vc_barcode_map.h $(VC)/vc_barcode_config.h
	$(CC) $(CFLAGS) -I$(VC) -o $@ cfg_encode.c $(SRC)/s1_cfg.c $(VC)/vc_barcode_map.c

# obstacle size report frames: s5_obst_report.c as the robot builds it
S5 := ../../../app_program/subsystem5
obst_encode: obst_encode.c $(S5)/s5_obst_report.c $(S5)/s5_obst_report.h
	$(CC) $(CFLAGS) -o $@ obst_encode.c $(S5)/s5_obst_report.c

clean:
	rm -f tlm_encode tlm_encode.exe cfg_encode cfg_encode.exe obst_encode obst_encode.exe

.PHONY: run clean
