VERILOGS = rcpu_icestick.v uart.v ../verilog/rcpu.v ../verilog/stack2.v

all: rcpu_icestick.bin

rcpu_icestick.bin: $(VERILOGS) rcpu_icestick.pcf ../build/*.hex
	yosys  -q -p "synth_ice40 -top top -abc2 -relut -blif rcpu_icestick.blif" $(VERILOGS)
	arachne-pnr -p rcpu_icestick.pcf rcpu_icestick.blif -o rcpu_icestick.asc
	icepack rcpu_icestick.asc rcpu_icestick0.bin
	icemulti -p0 rcpu_icestick0.bin > rcpu_icestick.bin && rm rcpu_icestick0.bin

rcpu_icestick: rcpu_icestick.bin
	sudo iceprog rcpu_icestick.bin

rcpu_icestick_nextnpr.bin: $(VERILOGS) rcpu_icestick.pcf ../build/*.hex
	yosys -p 'synth_ice40 -top top -abc2 -relut -json rcpu_icestick.json' $(VERILOGS)
	nextpnr-ice40 --hx1k --json rcpu_icestick.json --pcf rcpu_icestick.pcf --asc rcpu_icestick_nextnpr.asc
	icepack rcpu_icestick_nextnpr.asc rcpu_icestick_nextnpr.bin

rcpu_icestick_nextnpr: rcpu_icestick_nextnpr.bin
	sudo iceprog rcpu_icestick_nextnpr.bin

timing: rcpu_icestick.bin
	icetime -td hx1k rcpu_icestick.asc

usage:
	yosys -p "synth_ice40 -noflatten" $(VERILOGS)

clean:
	rm -f rcpu_icestick.blif rcpu_icestick.asc rcpu_icestick.bin

.PHONY: clean
.PHONY: rcpu_icestick
.PHONY: timing
.PHONY: usage
.PHONY: rcpu_icestick_nextnpr
