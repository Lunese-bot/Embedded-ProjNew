# Replay saved dashboard L traces through the real S3 barcode code (PC).
#   make          build bc_replay and replay every trace in traces/
#   make clean
# traces/2026*.txt : dashboard CSV rows of 2026-11-18 (18:23 and 21:40-21:41
#                    SGT), one 10 ms record per line: L_raw fwd yaw.
#                    13:40:05Z was exported twice; one copy is kept.
# traces/gNN.txt   : the 89 passes of 2026-11-05..12 (handoff g.txt).
# Each read prints one line: start / end sample, why it ended, element
# count, decoder path and the letter or the rejection.
#   make check    fail unless every trace gives exactly the letters in
#                 expected_letters.txt (as recorded, and with -prime) and
#                 no other trace gives any letter

SRC_DIR := ../../../app_program/subsystem3
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O1 -I$(SRC_DIR)

all: bc_replay
	./bc_replay traces/*.txt

bc_replay: bc_replay.c $(SRC_DIR)/line_algo.c $(SRC_DIR)/code39.c $(SRC_DIR)/code39_geom.c $(SRC_DIR)/s3_config.h $(SRC_DIR)/line_algo.h $(SRC_DIR)/code39_geom.h
	$(CC) $(CFLAGS) -o $@ bc_replay.c $(SRC_DIR)/line_algo.c $(SRC_DIR)/code39.c $(SRC_DIR)/code39_geom.c -lm

check: bc_replay
	@./bc_replay traces/*.txt | awk '/ SUMMARY /{print $$1, substr($$5, 2, length($$5) - 2)}' | awk '$$2 != ""' > got_plain.txt
	@./bc_replay -prime traces/*.txt | awk '/ SUMMARY /{print $$1, substr($$5, 2, length($$5) - 2)}' | awk '$$2 != ""' > got_prime.txt
	@awk '$$1 == "plain" {print $$2, $$3}' expected_letters.txt | diff - got_plain.txt
	@awk '$$1 == "prime" {print $$2, $$3}' expected_letters.txt | diff - got_prime.txt
	@rm -f got_plain.txt got_prime.txt
	@echo "[TEST] S3-REPLAY PASS  letters match expected_letters.txt"

clean:
	rm -f bc_replay

.PHONY: all check clean
