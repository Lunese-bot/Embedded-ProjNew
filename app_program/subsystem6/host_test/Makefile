# PC unit tests for the Vehicle Controller state machine.
# Needs any host C compiler (gcc / clang).  Run: make
# Not part of the firmware build (build_make only compiles one folder level).

CC     ?= cc
CFLAGS := -std=c99 -Wall -Wextra -Werror -DVC_HOST_TEST -I..

run: test_vc_sm
	./test_vc_sm

test_vc_sm: test_vc_sm.c ../vc_sm.c ../vc_sm.h ../vc_types.h ../../robot_types.h host_types.h
	$(CC) $(CFLAGS) -o $@ test_vc_sm.c ../vc_sm.c

clean:
	rm -f test_vc_sm

.PHONY: run clean
