`timescale 1 ns / 1 ps

`default_nettype none

// a part of this code was taken from jamesbowman/swapforth

module ram_memory(input clk, write_enable, read_enable, input [0:15] write_addr, input [0:15] read_addr, input[0:15] write_data, output reg [0:15] read_data);
  reg [0:15] mem [0:4095];
  initial begin
    $readmemh("../build/bootloader.hex", mem);
  end

  always @(posedge clk) begin
        if (write_enable) mem[write_addr % 4096] <= write_data;
        if (read_enable) read_data <= mem[read_addr % 4096];
  end
endmodule

module ioport(
  input clk,
  inout [7:0] pins,
  input we,
  input [7:0] wd,
  output [7:0] rd,
  input [7:0] dir);

  genvar i;
  generate
    for (i = 0; i < 8; i = i + 1) begin : io
      // 1001   PIN_OUTPUT_REGISTERED_ENABLE
      //     01 PIN_INPUT
      SB_IO #(.PIN_TYPE(6'b1001_01)) _io (
        .PACKAGE_PIN(pins[i]),
        .CLOCK_ENABLE(we),
        .OUTPUT_CLK(clk),
        .D_OUT_0(wd[i]),
        .D_IN_0(rd[i]),
        .OUTPUT_ENABLE(dir[i]));
    end
  endgenerate

endmodule

module outpin(
  input clk,
  output pin,
  input we,
  input wd,
  output rd);

  SB_IO #(.PIN_TYPE(6'b0101_01)) _io (
        .PACKAGE_PIN(pin),
        .CLOCK_ENABLE(we),
        .OUTPUT_CLK(clk),
        .D_OUT_0(wd),
        .D_IN_0(rd));
endmodule

module inpin(
  input clk,
  input pin,
  output rd);

  SB_IO #(.PIN_TYPE(6'b0000_00)) _io (
        .PACKAGE_PIN(pin),
        .INPUT_CLK(clk),
        .D_IN_0(rd));
endmodule

module top(input pclk, output D1, output D2, output D3, output D4, output D5,

           output TXD,        // UART TX
           input RXD,         // UART RX

           output CTS,        // UART clear to send
           input RTS,         // UART ready to send

           output PIOS_00,    // flash SCK
           input PIOS_01,     // flash MISO
           output PIOS_02,    // flash MOSI
           output PIOS_03,    // flash CS

           inout PIO1_02,    // PMOD 1
           inout PIO1_03,    // PMOD 2
           inout PIO1_04,    // PMOD 3
           inout PIO1_05,    // PMOD 4
           inout PIO1_06,    // PMOD 5
           inout PIO1_07,    // PMOD 6
           inout PIO1_08,    // PMOD 7
           inout PIO1_09,    // PMOD 8

           inout PIO0_02,    // HDR1 1
           inout PIO0_03,    // HDR1 2
           inout PIO0_04,    // HDR1 3
           inout PIO0_05,    // HDR1 4
           inout PIO0_06,    // HDR1 5
           inout PIO0_07,    // HDR1 6
           inout PIO0_08,    // HDR1 7
           inout PIO0_09,    // HDR1 8

           inout PIO2_10,    // HDR2 1
           inout PIO2_11,    // HDR2 2
           inout PIO2_12,    // HDR2 3
           inout PIO2_13,    // HDR2 4
           inout PIO2_14,    // HDR2 5
           inout PIO2_15,    // HDR2 6
           inout PIO2_16,    // HDR2 7
           inout PIO2_17,    // HDR2 8

           output PIO1_18,    // IR TXD
           input  PIO1_19,    // IR RXD
           output PIO1_20,    // IR SD

           input resetq,
);
  localparam MHZ = 12;

  wire clk;
  wire PLL_is_stable;
  SB_PLL40_CORE #(.FEEDBACK_PATH("SIMPLE"),
                  .PLLOUT_SELECT("GENCLK"),
                  .DIVR(4'b0000),
                  .DIVF(7'd3),
                  .DIVQ(3'b000),
                  .FILTER_RANGE(3'b001),
                 ) uut (
                         .REFERENCECLK(pclk),
                         .PLLOUTCORE(clk),
                         .LOCK(PLL_is_stable),
                         .RESETB(1'b1),
                         .BYPASS(1'b0)
                        );

  wire mem_write_enable, mem_read_enable; // read and write enable of RAM memory
  wire [0:15] mem_write_addr, mem_read_addr, mem_write_data, mem_read_data; // of RAM memory

  ram_memory _ram_memory(
	  .clk(clk),
      .read_enable(mem_read_enable),
	  .write_enable(mem_write_enable),
	  .read_addr(mem_read_addr),
	  .write_addr(mem_write_addr),
	  .read_data(mem_read_data),
      .write_data(mem_write_data));

  wire io_read_enable, io_write_enable;
  wire [0:15] io_address, io_write_data;
  reg [0:15] io_read_data = 0;

  reg start_cpu = 0;
  rcpu _rcpu(
    .clk(clk),
    .resetq(start_cpu & resetq),
    .io_read_enable(io_read_enable),
    .io_write_enable(io_write_enable),
    .io_address(io_address),
    .io_write_data(io_write_data),
    .io_read_data(io_read_data),

    .mem_read_enable(mem_read_enable),
    .mem_write_enable(mem_write_enable),
    .mem_read_address(mem_read_addr),
    .mem_write_address(mem_write_addr),
    .mem_read_data(mem_read_data),
    .mem_write_data(mem_write_data)
    );

  // ######   PMOD   ##########################################

  reg [7:0] pmod_dir;   // 1:output, 0:input
  wire [7:0] pmod_in;

  ioport _mod (.clk(clk),
               .pins({PIO1_09, PIO1_08, PIO1_07, PIO1_06, PIO1_05, PIO1_04, PIO1_03, PIO1_02}),
               .we(io_write_enable & io_address[0]),
               .wd(io_write_data[8:15]),
               .rd(pmod_in),
               .dir(pmod_dir));

  // ######   HDR1   ##########################################

  reg [7:0] hdr1_dir;   // 1:output, 0:input
  wire [7:0] hdr1_in;

  ioport _hdr1 (.clk(clk),
               .pins({PIO0_09, PIO0_08, PIO0_07, PIO0_06, PIO0_05, PIO0_04, PIO0_03, PIO0_02}),
               .we(io_write_enable & io_address[4]),
               .wd(io_write_data[8:15]),
               .rd(hdr1_in),
               .dir(hdr1_dir));

  // ######   HDR2   ##########################################

  reg [7:0] hdr2_dir;   // 1:output, 0:input
  wire [7:0] hdr2_in;

  ioport _hdr2 (.clk(clk),
               .pins({PIO2_17, PIO2_16, PIO2_15, PIO2_14, PIO2_13, PIO2_12, PIO2_11, PIO2_10}),
               .we(io_write_enable & io_address[6]),
               .wd(io_write_data[8:15]),
               .rd(hdr2_in),
               .dir(hdr2_dir));

  // ######   UART   ##########################################

  wire uart0_valid, uart0_busy;
  wire [7:0] uart0_data;
  wire uart0_wr = io_write_enable & io_address[12];
  wire uart0_rd = io_read_enable & io_address[12];
  wire uart_RXD;
  inpin _rcxd(.clk(clk), .pin(RXD), .rd(uart_RXD));
  buart _uart0 (
     .clk(clk),
     .resetq(1'b1),
     .rx(uart_RXD),
     .tx(TXD),
     .rd(uart0_rd),
     .wr(uart0_wr),
     .valid(uart0_valid),
     .busy(uart0_busy),
     .tx_data(io_write_data[8:15]),
     .rx_data(uart0_data));

  wire [0:4] LEDS;
  wire w4 = io_write_enable & io_address[2];

  outpin led0 (.clk(clk), .we(w4), .pin(D5), .wd(io_write_data[15]), .rd(LEDS[4])); // green led in middle
  outpin led1 (.clk(clk), .we(w4), .pin(D4), .wd(io_write_data[14]), .rd(LEDS[3]));
  outpin led2 (.clk(clk), .we(w4), .pin(D3), .wd(io_write_data[13]), .rd(LEDS[2]));
  outpin led3 (.clk(clk), .we(w4), .pin(D2), .wd(io_write_data[12]), .rd(LEDS[1]));
  outpin led4 (.clk(clk), .we(w4), .pin(D1), .wd(io_write_data[11]), .rd(LEDS[0]));

  wire [0:4] PIOS;
  wire w8 = io_write_enable & io_address[3];

  outpin pio0 (.clk(clk), .we(w8), .pin(PIOS_03), .wd(io_write_data[15]), .rd(PIOS[4]));
  outpin pio1 (.clk(clk), .we(w8), .pin(PIOS_02), .wd(io_write_data[14]), .rd(PIOS[3]));
  outpin pio2 (.clk(clk), .we(w8), .pin(PIOS_00), .wd(io_write_data[13]), .rd(PIOS[2]));
  outpin pio3 (.clk(clk), .we(w8), .pin(PIO1_18), .wd(io_write_data[12]), .rd(PIOS[1]));
  outpin pio4 (.clk(clk), .we(w8), .pin(PIO1_20), .wd(io_write_data[11]), .rd(PIOS[0]));

  // ######   IO PORTS   ######################################

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
  reg [0:7] uart_data_read = 0;
  reg uart_data_read_valid = 0;
  outpin uart_rts (.clk(clk), .we(1), .pin(CTS), .wd(uart_data_read_valid));

  always @(posedge clk) begin
  if (uart_data_read_valid) begin
      if (io_read_enable & io_address[12]) begin
          uart_data_read_valid <= 0;
      end
  end
  else begin
      if (uart0_valid) begin
          uart_data_read <= uart0_data;
          uart_data_read_valid <= 1;
      end
  end
  end

  wire [0:15] io_read_dataN =
    (io_address[ 0] ? {8'd0, pmod_in}                                     : 16'd0) |
    (io_address[ 1] ? {8'd0, pmod_dir}                                    : 16'd0) |
    (io_address[ 2] ? {11'd0, LEDS}                                       : 16'd0) |
    (io_address[ 3] ? {11'd0, PIOS}                                       : 16'd0) |
    (io_address[ 4] ? {8'd0, hdr1_in}                                     : 16'd0) |
    (io_address[ 5] ? {8'd0, hdr1_dir}                                    : 16'd0) |
    (io_address[ 6] ? {8'd0, hdr2_in}                                     : 16'd0) |
    (io_address[ 7] ? {8'd0, hdr2_dir}                                    : 16'd0) |
    (io_address[12] ? {8'd0, uart_data_read}                              : 16'd0) |
    (io_address[13] ? {12'd0, PIO1_19, PIOS_01, uart_data_read_valid, !uart0_busy} : 16'd0);

  reg boot, s0, s1;

  SB_WARMBOOT _sb_warmboot (
    .BOOT(boot),
    .S0(s0),
    .S1(s1)
    );

  reg [0:8] ticks = 0;
  always @(posedge clk) begin
    if (io_write_enable & io_address[1])
      pmod_dir <= io_write_data[8:15];
    if (io_write_enable & io_address[5])
      hdr1_dir <= io_write_data[8:15];
    if (io_write_enable & io_address[7])
      hdr2_dir <= io_write_data[8:15];
    if (io_write_enable & io_address[11])
      {boot, s1, s0} <= io_write_data[13:15];

    if (io_read_enable)
      io_read_data <= io_read_dataN;
    // workaround for RAM corrupting if read/written before the ~40 th clock cycle
    // this is a hardware problem
    if (ticks > 40)
      start_cpu <= 1;
    if (PLL_is_stable)
      ticks <= ticks + 1;
  end

endmodule // top
