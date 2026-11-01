# PC unit tests for subsystem 3 (gcc or clang on macOS / Linux / WSL)
#   make        build and run
#   make clean

SRC_DIR := ../../../app_program/subsystem3
CFLAGS  := -std=c11 -Wall -Wextra -Werror -O1 -I$(SRC_DIR)

test_s3_host: test_s3_host.c $(SRC_DIR)/code39.c $(SRC_DIR)/line_algo.c
	$(CC) $(CFLAGS) -o $@ $^
	./$@

clean:
	rm -f test_s3_host

.PHONY: clean
