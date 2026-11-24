SRC := ../../../app_program/subsystem1
CFLAGS := -std=c11 -Wall -Wextra -Werror -O2 -I$(SRC)

run: s1_host_test s1_capture_test s1_queue_test s1_mqtt_test
	./s1_host_test
	./s1_capture_test
	./s1_queue_test
	./s1_mqtt_test

s1_host_test: test_codec.c $(SRC)/s1_codec.c $(SRC)/s1_codec.h
	$(CC) $(CFLAGS) -o $@ test_codec.c $(SRC)/s1_codec.c

# S1 sources a test includes or links besides the codec
S1_LINKED := $(SRC)/s1_codec.c $(SRC)/s1_stats.c s1_lock_host.c

s1_capture_test: test_capture.c $(wildcard $(SRC)/*.c $(SRC)/*.h) s1_lock_host.c ../../../app_program/test_report.h
	$(CC) $(CFLAGS) -I../../../include -o $@ test_capture.c $(S1_LINKED)

# mqtt_utk.c against fake lwIP headers, with the real S1 behind it
PORT := ../../../lib/liblwip/sysdepend/pico_rp2040
S1_HOST := s1_host_kernel.c $(S1_LINKED)

s1_queue_test: test_s1_queue.c $(wildcard $(SRC)/*.c $(SRC)/*.h) s1_host_kernel.h $(S1_HOST)
	$(CC) $(CFLAGS) -o $@ test_s1_queue.c $(S1_HOST)

s1_mqtt_test: test_mqtt_utk.c $(PORT)/mqtt_utk.c $(PORT)/mqtt_utk.h $(wildcard $(SRC)/*.c $(SRC)/*.h) $(wildcard *.h fake_lwip/*.h fake_lwip/*/*.h fake_lwip/*/*/*.h) $(S1_HOST)
	$(CC) $(CFLAGS) -Ifake_lwip -o $@ test_mqtt_utk.c $(S1_HOST)

clean:
	rm -f s1_host_test s1_host_test.exe s1_capture_test s1_capture_test.exe
	rm -f s1_queue_test s1_queue_test.exe s1_mqtt_test s1_mqtt_test.exe

.PHONY: run clean
