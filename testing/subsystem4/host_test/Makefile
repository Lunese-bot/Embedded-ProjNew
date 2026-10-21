# Host tests for subsystem 4's algorithm (no Pico needed).
#   make -C host_test        build and run
#   make -C host_test clean
# Copy this folder to testing/subsystem4/host_test/ in the repo; SRC
# below then points at app_program/subsystem4/.

SRC ?= ../../../app_program/subsystem4
ifeq ($(wildcard $(SRC)/imu_terrain.c),)
SRC := ../app_program/subsystem4
endif

CFLAGS := -std=c11 -Wall -Wextra -Werror -O2 -I$(SRC)
OBJS   := $(SRC)/imu_math.c $(SRC)/imu_terrain.c $(SRC)/imu_selftest.c

run: s4_host_test
	./s4_host_test

s4_host_test: test_main.c $(OBJS) $(SRC)/*.h
	$(CC) $(CFLAGS) -o $@ test_main.c $(OBJS)

clean:
	rm -f s4_host_test

.PHONY: run clean
