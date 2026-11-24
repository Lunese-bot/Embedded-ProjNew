# Replay saved dashboard L traces through the real S3 barcode code (PC).
#   make          build bc_replay and replay every trace in traces/
#   make clean
# traces/2026*.txt : dashboard CSV rows of 2026-11-18 (18:23 and 21:40-21:41
#                    SGT), one 10 ms record per line: L_raw fwd yaw.
#                    13:40:05Z was exported twice; one copy is kept.
# traces/gNN.txt   : the 89 passes of 2026-11-05..12 (handoff g.txt).
# Each read prints one line: start / end sample, why it ended, element
# count, decoder path and the letter or the rejection.

SRC_DIR := ../../../app_program/subsystem3
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O1 -I$(SRC_DIR)

all: bc_replay
	./bc_replay traces/*.txt

bc_replay: bc_replay.c $(SRC_DIR)/line_algo.c $(SRC_DIR)/code39.c $(SRC_DIR)/code39_geom.c $(SRC_DIR)/s3_config.h $(SRC_DIR)/line_algo.h $(SRC_DIR)/code39_geom.h
	$(CC) $(CFLAGS) -o $@ bc_replay.c $(SRC_DIR)/line_algo.c $(SRC_DIR)/code39.c $(SRC_DIR)/code39_geom.c -lm

clean:
	rm -f bc_replay

.PHONY: all clean
