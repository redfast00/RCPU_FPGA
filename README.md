# RCPU_FPGA

RCPU_FPGA is an FPGA port of [RCPU](https://github.com/redfast00/RCPU_FPGA). This means that you can now run RCPU on actual hardware!

## Toolchain installation

For Arch Linux: install icestorm-git, yosys-git and either arachne-pnr-git or nextpnr-git from the AUR.

You can then upload the code to the FPGA by running `make rcpu_icestick` or `make rcpu_icestick_nextnpr`
in the `rcpu/icestorm` directory. You can choose what ROM will run on boot by editing
the `$readmemh` argument in `rcpu/icestorm/rcpu_icestick.v`.

## Limitations/differences from the RCPU emulator

- Memory is only backed by physical memory from 0 - 4095. After that, it wraps around.
  So the value in address 0 will always be the same value as the one in 4096, 8192, ...

- There is only enough room for 10 values on the
stack. If you add any more, the values that were
first pushed will be replaced by a dummy value.

- The multiply and divide ATH instructions are not implemented because they are too expensive.

- The arguments for the SYS instruction work different than in the emulator, see below.

## SYS addresses

A SYS instruction expects an address on the top of the stack, and a value to write if the address indicates writing. Here is a table of devices and the address they correspond to. The 0-th bit is the most significant. The 14-th bit indicates reading, the 15-th bit indicates writing.
```
/*        bit   mode    device
          0     r/w     PMOD GPIO
          1     r/w     PMOD direction
          2     r/w     LEDS
          3     r/w     misc.out
          4     r/w     HDR1 GPIO
          5     r/w     HDR1 direction
          6     r/w     HDR2 GPIO
          7     r/w     HDR2 direction
          11      w     sb_warmboot
          12    r/w     UART RX, UART TX
          13    r       misc.in
*/
```

For example, if we wanted to turn on the LEDs, we would first push `0xFFFF` (all bits are 1, so all leds will turn on). We would then push `0b0010_0000_0000_0001` to indicate we want to write to the LEDs (bit 2 and 15 are set). After that, we can use the SYS instruction to turn the leds on.

If we wanted to read the LEDs, we would push `0b0010_0000_0000_0010` and then use the SYS instruction. The value of the LEDs will then be on the stack.

## Questions?

Please don't hesitate to open an issue/contact me

## Roadmap

- [X] Get UART tx working
- [ ] Get UART rx working
- [ ] Write some code in RCPU assembly that allows uploading code without reflashing
