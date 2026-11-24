SRC = ../../../app_program/subsystem5
test: test_s5_pack test_navigation test_side_pass test_side_task
	./test_s5_pack
	./test_navigation
	./test_side_pass
	./test_side_task
test_side_pass: test_side_pass.c $(wildcard $(SRC)/*.c $(SRC)/*.h)
	$(CC) -std=c11 -Wall -Wextra -Werror -O1 -o $@ test_side_pass.c
test_side_task: test_side_task.c $(wildcard $(SRC)/*.c $(SRC)/*.h)
	$(CC) -std=c11 -Wall -Wextra -Werror -O1 -o $@ test_side_task.c
test_navigation: test_navigation.c $(wildcard $(SRC)/*.c $(SRC)/*.h) $(wildcard ../../../app_program/vehicle_controller/*.c ../../../app_program/vehicle_controller/*.h)
	$(CC) -std=c11 -Wall -Wextra -Werror -O1 -o $@ test_navigation.c
test_s5_pack: test_s5_pack.c $(SRC)/s5_pack.c $(SRC)/s5_pack.h
	cc -std=c11 -Wall -Wextra -Werror -O1 -I$(SRC) -o $@ test_s5_pack.c $(SRC)/s5_pack.c
clean:
	rm -f test_s5_pack test_navigation test_side_pass test_side_task
