# PC unit tests for the Vehicle Controller state machine.
# Needs any host C compiler (gcc / clang).  Run: make
# Not part of the firmware build (build_make only compiles one folder level).

CC     ?= cc
CFLAGS := -std=c99 -Wall -Wextra -Werror -DVC_HOST_TEST -DVC_JUNCTION_HOLD_MS=0 -DVC_BARCODE_RULES_TEST -I..

run: test_vc_sm test_vc_s3
	./test_vc_sm
	./test_vc_s3

test_vc_sm: test_vc_sm.c ../vc_sm.c ../vc_barcode_rules.c ../vc_barcode_config.h ../vc_sm.h ../vc_types.h ../../robot_types.h host_types.h
	$(CC) $(CFLAGS) -o $@ test_vc_sm.c ../vc_sm.c ../vc_barcode_rules.c

test_vc_s3: test_vc_s3.c ../vc_bridge_s3.c ../vc_bridge.h ../vc_sm.h ../vc_types.h host_types.h ../../subsystem3/subsystem3.h
	$(CC) -std=c11 -Wall -Wextra -Werror -o $@ test_vc_s3.c

clean:
	rm -f test_vc_sm test_vc_s3

.PHONY: run clean
