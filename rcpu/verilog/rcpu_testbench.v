module ram_memory(input clk, write_enable, read_enable, input [0:15] write_addr, read_addr, write_data, output reg [0:15] read_data);
  reg [0:15] mem [0:4095];
  initial begin
    $readmemh("test.rom", mem);
  end

  always @(posedge clk) begin
        if (write_enable) mem[write_addr] <= write_data;
        if (read_enable) read_data <= mem[read_addr];
  end
endmodule

module rcpu_tb();

    reg  clk, resetq;
    wire io_read_enable, io_write_enable;
    wire [0:15] io_addr, io_read_data, io_write_data;
    wire mem_read_enable, mem_write_enable;
    wire [0:15] mem_read_addr, mem_write_addr, mem_write_data;
    wire [0:15] mem_read_data;

    ram_memory _ram_memory(
        .clk(clk),
        .read_enable(mem_read_enable),
        .write_enable(mem_write_enable),
        .read_addr(mem_read_addr),
        .write_addr(mem_write_addr),
        .read_data(mem_read_data),
        .write_data(mem_write_data));

    j1 _cpu(
      .clk(clk),
      .resetq(resetq),
      .io_read_enable(io_read_enable),
      .io_write_enable(io_write_enable),
      .io_addr(io_addr),
      .io_write_data(io_write_data),
      .io_read_data(io_read_data),

      .mem_read_enable(mem_read_enable),
      .mem_write_enable(mem_write_enable),
      .mem_read_address(mem_read_addr),
      .mem_write_address(mem_write_addr),
      .mem_read_data(mem_read_data),
      .mem_write_data(mem_write_data));

  initial begin
    clk = 0;
    resetq = 0;
    // start cpu after 2 clock cycles
    #30
    resetq = 1;
  end

  initial  begin
      $display("\t\ttime,\tclk,\tpc,\t\t\tcurrent_state,\topcode,\tmem_read_enable,\tmem_write_enable");
      $monitor("%d,\t%b,\t%b\t%b\t\t%b\t%b\t\t\t%b",$time, clk, _cpu.pc, _cpu.current_state, _cpu.opcode, mem_read_enable, mem_write_enable);
  end

  always
    #5 clk = !clk;


  initial
    #500 $finish;

endmodule
