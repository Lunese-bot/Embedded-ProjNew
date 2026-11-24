SRC := ../../../app_program/subsystem1
CFLAGS := -std=c11 -Wall -Wextra -Werror -O2 -I$(SRC)

run: s1_host_test s1_capture_test s1_queue_test s1_mqtt_test s1_obst_pub_test
	./s1_host_test
	./s1_capture_test
	./s1_queue_test
	./s1_mqtt_test
	./s1_obst_pub_test
	@if command -v python3 >/dev/null 2>&1; then \
	    python3 s1_seqcheck.py --selftest; \
	else \
	    echo "[TEST] subsystem1 seqcheck SKIPPED (python3 not found; not a pass)"; \
	fi
	$(MAKE) -C ../dashboard_test run

s1_host_test: test_codec.c $(SRC)/s1_codec.c $(SRC)/s1_codec.h
	$(CC) $(CFLAGS) -o $@ test_codec.c $(SRC)/s1_codec.c

# S1 sources a test includes or links besides the codec
VC := ../../../app_program/vehicle_controller
S1_LINKED := $(SRC)/s1_codec.c $(SRC)/s1_stats.c $(SRC)/s1_txq.c s1_lock_host.c \
             $(SRC)/s1_cfg.c $(VC)/vc_barcode_map.c

s1_capture_test: test_capture.c $(wildcard $(SRC)/*.c $(SRC)/*.h) s1_lock_host.c ../../../app_program/test_report.h
	$(CC) $(CFLAGS) -I../../../include -o $@ test_capture.c $(S1_LINKED)

# mqtt_utk.c against fake lwIP headers, with the real S1 behind it
PORT := ../../../lib/liblwip/sysdepend/pico_rp2040
S1_HOST := s1_host_kernel.c $(S1_LINKED)

s1_queue_test: test_s1_queue.c $(wildcard $(SRC)/*.c $(SRC)/*.h) s1_host_kernel.h $(S1_HOST)
	$(CC) $(CFLAGS) -o $@ test_s1_queue.c $(S1_HOST)

s1_mqtt_test: test_mqtt_utk.c $(PORT)/mqtt_utk.c $(PORT)/mqtt_utk.h $(wildcard $(SRC)/*.c $(SRC)/*.h) $(wildcard *.h fake_lwip/*.h fake_lwip/*/*.h fake_lwip/*/*/*.h) $(S1_HOST)
	$(CC) $(CFLAGS) -Ifake_lwip -o $@ test_mqtt_utk.c $(S1_HOST)

s1_obst_pub_test: test_obst_pub.c $(SRC)/s1_obst_pub.c $(SRC)/s1_obst_pub.h ../../../app_program/subsystem5/s5_obst_report.c ../../../app_program/subsystem5/s5_obst_report.h
	$(CC) $(CFLAGS) -o $@ test_obst_pub.c $(SRC)/s1_obst_pub.c ../../../app_program/subsystem5/s5_obst_report.c

clean:
	rm -f s1_host_test s1_host_test.exe s1_capture_test s1_capture_test.exe
	rm -f s1_queue_test s1_queue_test.exe s1_mqtt_test s1_mqtt_test.exe
	rm -f s1_obst_pub_test s1_obst_pub_test.exe
	$(MAKE) -C ../dashboard_test clean

.PHONY: run clean
