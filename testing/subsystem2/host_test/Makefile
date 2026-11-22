# Host test for the subsystem 2 emergency stop ordering (S2-U11, no Pico needed).
#   make -C testing/subsystem2/host_test run     build and run, SMP and single core
#   make -C testing/subsystem2/host_test clean
SRC := ../../../app_program/subsystem2
# The single-core build leaves some SMP-only stubs unused.
CFLAGS := -std=c11 -Wall -Wextra -Werror -Wno-unused-function -I../../../include

run: s2_estop_smp s2_estop_single
	./s2_estop_smp
	./s2_estop_single

s2_estop_smp: test_emergency_stop.c s4_speed_stub.c $(wildcard $(SRC)/*.c $(SRC)/*.h)
	$(CC) $(CFLAGS) -o $@ test_emergency_stop.c s4_speed_stub.c

s2_estop_single: test_emergency_stop.c s4_speed_stub.c $(wildcard $(SRC)/*.c $(SRC)/*.h)
	$(CC) $(CFLAGS) -DTK_SUPPORT_SMP=0 -o $@ test_emergency_stop.c s4_speed_stub.c

clean:
	rm -f s2_estop_smp s2_estop_single

.PHONY: run clean
