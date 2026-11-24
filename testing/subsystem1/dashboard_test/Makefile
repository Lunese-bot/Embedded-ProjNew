# Dashboard checks: tools/dashboard.html against the firmware sources.
#   make -C testing/subsystem1/dashboard_test run
# Needs a C compiler and Node.js 18 or newer (no npm packages).
SRC := ../../../app_program/subsystem1
CFLAGS := -std=c11 -Wall -Wextra -Werror -O2 -I$(SRC)

run: tlm_encode
	@if command -v node >/dev/null 2>&1; then \
	    node test_dashboard.js ./tlm_encode; \
	else \
	    echo "[TEST] dashboard SKIPPED (node not found; not a pass)"; exit 1; \
	fi

tlm_encode: tlm_encode.c $(SRC)/s1_codec.c $(SRC)/s1_codec.h $(SRC)/s1_stats.c $(SRC)/s1_stats.h
	$(CC) $(CFLAGS) -o $@ tlm_encode.c $(SRC)/s1_codec.c $(SRC)/s1_stats.c

clean:
	rm -f tlm_encode tlm_encode.exe

.PHONY: run clean
