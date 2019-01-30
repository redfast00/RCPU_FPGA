VERILOGS = stack2.v rcpu.v rcpu_testbench.v

rcpu_testbench:
	iverilog -o rcpu_testbench $(VERILOGS)
	vvp rcpu_testbench

.PHONY: rcpu_testbench
