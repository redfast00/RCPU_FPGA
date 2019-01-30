VERILOGS = stack2.v rcpu.v rcpu_testbench.v

rcpu_testbench:
	iverilog -Wall -o rcpu_testbench $(VERILOGS)
	vvp rcpu_testbench

.PHONY: rcpu_testbench
