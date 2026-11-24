SRC = ../../../app_program/subsystem5
test: test_s5_pack test_navigation test_side_pass test_side_task test_side_pass_cfg test_side_task_cfg test_obstacle_report test_obstacle_report_off
	sh ./check_no_new_binaries.sh
	./test_s5_pack
	./test_navigation
	./test_side_pass
	./test_side_task
	./test_side_pass_cfg
	./test_side_task_cfg
	./test_obstacle_report
	./test_obstacle_report trace > obstacle_trace_on.txt
	./test_obstacle_report_off trace > obstacle_trace_off.txt
	cmp obstacle_trace_on.txt obstacle_trace_off.txt && echo "[TEST] S5-R08 PASS  bypass motion identical with and without the length observation"
	rm -f obstacle_trace_on.txt obstacle_trace_off.txt
test_side_pass: test_side_pass.c $(wildcard $(SRC)/*.c $(SRC)/*.h)
	$(CC) -std=c11 -Wall -Wextra -Werror -O1 -o $@ test_side_pass.c
test_side_task: test_side_task.c $(wildcard $(SRC)/*.c $(SRC)/*.h)
	$(CC) -std=c11 -Wall -Wextra -Werror -O1 -o $@ test_side_task.c
test_navigation: test_navigation.c $(wildcard $(SRC)/*.c $(SRC)/*.h) $(wildcard ../../../app_program/vehicle_controller/*.c ../../../app_program/vehicle_controller/*.h)
	$(CC) -std=c11 -Wall -Wextra -Werror -O1 -o $@ test_navigation.c
test_s5_pack: test_s5_pack.c $(SRC)/s5_pack.c $(SRC)/s5_pack.h
	cc -std=c11 -Wall -Wextra -Werror -O1 -I$(SRC) -o $@ test_s5_pack.c $(SRC)/s5_pack.c
test_side_pass_cfg: test_side_pass.c $(wildcard $(SRC)/*.c $(SRC)/*.h)
	$(CC) -std=c11 -Wall -Wextra -Werror -O1 -DS5_USE_CONFIG_DIMS -o $@ test_side_pass.c
test_side_task_cfg: test_side_task.c $(wildcard $(SRC)/*.c $(SRC)/*.h)
	$(CC) -std=c11 -Wall -Wextra -Werror -O1 -DS5_USE_CONFIG_DIMS -o $@ test_side_task.c
test_obstacle_report: test_obstacle_report.c $(wildcard $(SRC)/*.c $(SRC)/*.h)
	$(CC) -std=c11 -Wall -Wextra -Werror -O1 -DS5_LENGTH_OBSERVE=1 -o $@ test_obstacle_report.c
test_obstacle_report_off: test_obstacle_report.c $(wildcard $(SRC)/*.c $(SRC)/*.h)
	$(CC) -std=c11 -Wall -Wextra -Werror -O1 -DS5_LENGTH_OBSERVE=0 -o $@ test_obstacle_report.c
clean:
	rm -f test_s5_pack test_navigation test_side_pass test_side_task test_side_pass_cfg test_side_task_cfg test_obstacle_report test_obstacle_report_off obstacle_trace_on.txt obstacle_trace_off.txt
