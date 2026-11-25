SRC = ../../../app_program/subsystem5
test: test_s5_pack test_navigation test_side_pass test_side_task test_side_pass_cfg test_side_task_cfg test_obstacle_report test_obstacle_report_off test_obstacle_report_sup test_scan_diag test_supported_edge s5_replay
	sh ./check_no_new_binaries.sh
	./test_s5_pack
	./test_navigation
	./test_side_pass
	./test_side_task
	./test_side_pass_cfg
	./test_side_task_cfg
	./test_obstacle_report
	./test_obstacle_report_sup
	./test_obstacle_report trace > obstacle_trace_on.txt
	./test_obstacle_report_off trace > obstacle_trace_off.txt
	cmp obstacle_trace_on.txt obstacle_trace_off.txt && echo "[TEST] S5-R08 PASS  bypass motion identical with and without the length observation"
	./test_obstacle_report_sup trace > obstacle_trace_sup.txt
	cmp obstacle_trace_on.txt obstacle_trace_sup.txt && echo "[TEST] S5-W06 PASS  bypass motion identical with the supported-width report on and off"
	rm -f obstacle_trace_on.txt obstacle_trace_off.txt obstacle_trace_sup.txt
	./test_scan_diag
	./test_supported_edge
	./test_supported_edge log | ./s5_replay --width 150 --offset 0 > s5_replay_out.txt && grep -q "mismatches 0" s5_replay_out.txt && echo "[TEST] S5-W07 PASS  s5_replay reads the s5d: record back and reproduces the robot's raw profile"
	rm -f s5_replay_out.txt
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
	$(CC) -std=c11 -Wall -Wextra -Werror -O1 -DS5_LENGTH_OBSERVE=1 -DS5_REPORT_SUPPORTED_WIDTH=0 -o $@ test_obstacle_report.c
test_obstacle_report_off: test_obstacle_report.c $(wildcard $(SRC)/*.c $(SRC)/*.h)
	$(CC) -std=c11 -Wall -Wextra -Werror -O1 -DS5_LENGTH_OBSERVE=0 -DS5_REPORT_SUPPORTED_WIDTH=0 -o $@ test_obstacle_report.c
test_scan_diag: test_scan_diag.c $(wildcard $(SRC)/*.c $(SRC)/*.h)
	$(CC) -std=c11 -Wall -Wextra -Werror -O1 -o $@ test_scan_diag.c
test_supported_edge: test_supported_edge.c s5_scan_model.h s5_same.h $(wildcard $(SRC)/*.c $(SRC)/*.h)
	$(CC) -std=c11 -Wall -Wextra -Werror -O1 -o $@ test_supported_edge.c -lm
test_obstacle_report_sup: test_obstacle_report.c $(wildcard $(SRC)/*.c $(SRC)/*.h)
	$(CC) -std=c11 -Wall -Wextra -Werror -O1 -DS5_LENGTH_OBSERVE=1 -o $@ test_obstacle_report.c
s5_replay: s5_replay.c $(wildcard $(SRC)/*.c $(SRC)/*.h)
	$(CC) -std=c11 -Wall -Wextra -Werror -O1 -o $@ s5_replay.c
clean:
	rm -f test_s5_pack test_navigation test_side_pass test_side_task test_side_pass_cfg test_side_task_cfg test_obstacle_report test_obstacle_report_off test_obstacle_report_sup test_scan_diag test_supported_edge s5_replay s5_replay_out.txt obstacle_trace_on.txt obstacle_trace_off.txt obstacle_trace_sup.txt
