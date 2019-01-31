// taken and modified from jamesbowman/swapforth

`default_nettype none
`define WIDTH 16

module stack2(
  input wire clk,
  output wire [`WIDTH-1:0] rd,  // top of stack
  input wire we,                // write enable bit
  input wire [1:0] delta,
  // 00 no we = do nothing
  // 00 we    = overwrite top of stack
  // 10 no we = do nothing
  // 10 we    = overwrite top of stack
  // 01 no we = swap head and snd
  // 01 we    = push written item to stack
  // 11 no we = pop stack
  // 11 we    = pop, then overwrite stack
  input wire [`WIDTH-1:0] wd);  // write data
  parameter DEPTH = 18;
  localparam BITS = (`WIDTH * DEPTH) - 1;

  wire move = delta[0];

  reg [15:0] head;
  reg [BITS:0] tail;
  wire [15:0] headN;
  wire [BITS:0] tailN;

  assign headN = we ? wd : tail[15:0];
  assign tailN = delta[1] ? {16'h55aa, tail[BITS:16]} : {tail[BITS-16:0], head};

  always @(posedge clk) begin
    if (we | move)
      head <= headN;
    if (move)
      tail <= tailN;
  end

  assign rd = head;

`ifdef VERILATOR
  int depth /* verilator public_flat */;
  always @(posedge clk) begin
    if (delta == 2'b11)
      depth <= depth - 1;
    if (delta == 2'b01)
      depth <= depth + 1;
  end
`endif

endmodule
