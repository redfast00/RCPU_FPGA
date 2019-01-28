VERILOGS = rcpu_icestick.v uart.v ../verilog/rcpu.v ../verilog/stack2.v

SUBDIRS = ..

all: $(SUBDIRS) rcpu_icestick.bin

rcpu_icestick.bin: $(VERILOGS) rcpu_icestick.pcf Makefile ../build/rcpu_ram.v
	yosys  -q -p "synth_ice40 -top top -abc2 -blif rcpu_icestick.blif" $(VERILOGS)
	arachne-pnr -p rcpu_icestick.pcf rcpu_icestick.blif -o rcpu_icestick.txt
	#skipped:# icebox_explain rcpu_icestick.txt > rcpu_icestick.ex
	icepack rcpu_icestick.txt rcpu_icestick0.bin
	icemulti -p0 rcpu_icestick0.bin > rcpu_icestick.bin && rm rcpu_icestick0.bin

rcpu_icestick: rcpu_icestick.bin
	sudo iceprog rcpu_icestick.bin

$(SUBDIRS):
	$(MAKE) -C $@

clean:
	rm -f rcpu_icestick.blif rcpu_icestick.txt rcpu_icestick.bin

.PHONY: subdirs
.PHONY: subdirs $(SUBDIRS)
.PHONY: clean
.PHONY: rcpu_icestick
