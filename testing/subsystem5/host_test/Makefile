SRC = ../../../app_program/subsystem5
test: test_s5_pack
	./test_s5_pack
test_s5_pack: test_s5_pack.c $(SRC)/s5_pack.c $(SRC)/s5_pack.h
	cc -std=c11 -Wall -Wextra -Werror -O1 -I$(SRC) -o $@ test_s5_pack.c $(SRC)/s5_pack.c
clean:
	rm -f test_s5_pack
