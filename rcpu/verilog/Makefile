VERILOGS = stack2.v rcpu.v rcpu_testbench.v

rcpu_testbench:
	iverilog -Wall -o rcpu_testbench.vvp $(VERILOGS)
	vvp rcpu_testbench.vvp

.PHONY: rcpu_testbench
