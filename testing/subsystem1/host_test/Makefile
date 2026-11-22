SRC := ../../../app_program/subsystem1
CFLAGS := -std=c11 -Wall -Wextra -Werror -O2 -I$(SRC)

run: s1_host_test s1_capture_test
	./s1_host_test
	./s1_capture_test

s1_host_test: test_codec.c $(SRC)/s1_codec.c $(SRC)/s1_codec.h
	$(CC) $(CFLAGS) -o $@ test_codec.c $(SRC)/s1_codec.c

s1_capture_test: test_capture.c $(wildcard $(SRC)/*.c $(SRC)/*.h) ../../../app_program/test_report.h
	$(CC) $(CFLAGS) -I../../../include -o $@ test_capture.c $(SRC)/s1_codec.c

clean:
	rm -f s1_host_test s1_host_test.exe s1_capture_test s1_capture_test.exe

.PHONY: run clean
