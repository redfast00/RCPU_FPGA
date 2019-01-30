`default_nettype none
`define WIDTH 16


module j1(
  input wire clk, // clock
  input wire resetq,

  output reg io_read_enable,
  output reg io_write_enable,
  input  wire [0:15] io_addr, // TODO convert endianness
  output reg  [0:15] io_write_data,
  input  wire [0:15] io_read_data,

  output reg [0:15] mem_write_address,
  output reg mem_write_enable,
  output reg [0:15] mem_write_data,

  output reg [0:15] mem_read_address,
  output reg mem_read_enable,
  input  wire [0:15] mem_read_data // data that was read from memory
  ); // end of inputs and outputs

  reg [0:15] st0, st0N;   // top of data stack
  reg dstkW;              // data stack write enable bit

  reg [0:15] pc /* verilator public_flat */, pcN;           // program counter
  wire [0:15] pc_plus_1 = pc + 16'd1;

  reg [0:15] register_file [0:3];
  reg reboot = 1;
  reg [0:1] current_state; // there are 3 states: fetch instruction, fetch data and write data

  // The datastack
  wire [0:15] st1; // top of stack
  reg [1:0] dspI;

  reg [0:15] instruction;
  wire [0:3] opcode = instruction[12:15];
  wire [0:1] destination = instruction[10:11];
  wire [0:1] source      = instruction[8:9];
  wire [0:9] large_argument = instruction[0:9];
  wire [0:3] ath_opcode = instruction[4:7];
  // selects the place to store the result of an ATH opcode based on the M bit
  wire [0:1] ath_store = instruction[3] ? destination : source;

  reg register_write_enable;
  reg [0:15] register_write_data;
  reg [0:15] requested_mem_read_addr;

  stack2 #(.DEPTH(15)) dstack(.clk(clk), .rd(st1), .we(dstkW), .wd(st0),   .delta(dspI)); // datastack

  // TODO destination for ATH instructions is not always DD, but depends on M
  // calculate next registervalues
  always @*
  begin
  case(opcode)
      // MOV
      4'b0000 : {register_write_enable, register_write_data} = { 1'b1, register_file[source] };
      // LDV
      4'b0001 : {register_write_enable, register_write_data} = { 1'b1, large_argument };
      // LDA, LDR
      4'b0010, 4'b0100 : {register_write_enable, register_write_data} = { 1'b1, mem_read_data};
      // ATH
      4'b0110 : begin
        case (ath_opcode)
        4'b0000 : { register_write_enable, register_write_data } = { 1'b1, register_file[source] + register_file[destination] };
        default : { register_write_enable, register_write_data } = { 1'b1,  16'hABCD }; // TODO implement other opcodes
        endcase
      end
      // POP
      4'b1011: {register_write_enable, register_write_data} = {1'b1, st0};
      default: {register_write_enable, register_write_data} = 0;
  endcase
  end

  // wire func_T_N =   (insn[6:4] == 1); //001 (write to datastack?)
  // wire func_T_R =   (insn[6:4] == 2); //010 (write to return stack?)
  // wire func_write = (insn[6:4] == 3); //011 (write to memory)
  // wire func_iow =   (insn[6:4] == 4); //100 (write to io)
  // wire func_ior =   (insn[6:4] == 5); //101 (read from io)
  //
  // wire is_alu = !pc[12] & (insn[15:13] == 3'b011);
  // assign mem_wr = !reboot & is_alu & func_write;
  // assign dout = st1;
  // assign io_wr = !reboot & is_alu & func_iow;
  // assign io_rd = !reboot & is_alu & func_ior;

  always @*
  begin
    // calculate requested_mem_read_addr
    case (opcode)
    // LDA: load from memory from instruction argument
    4'b0010: requested_mem_read_addr = large_argument;
    // LDR: load from memory pointed at by the source register
    4'b0100: requested_mem_read_addr = register_file[source];
    default: requested_mem_read_addr = 0;
    endcase
  end

  always @*
  begin
    // calculate requested_mem_write_address
    // only write if current_state == writeback state
    casez ({current_state, opcode})
    // LDM: load to instruction argument
    6'b10_0011: {mem_write_enable, mem_write_data, mem_write_address} = {1'b1, register_file[destination], large_argument};
    // LDP: load to memory pointed at by the destination register
    6'b10_0101: {mem_write_enable, mem_write_data, mem_write_address} = {1'b1, register_file[source], register_file[destination]};
    // this should never get written
    default: {mem_write_enable, mem_write_data, mem_write_address} = 0;
    endcase
  end

  always @*
  begin
    // calculate next datastack operations
    case ({current_state, opcode})
    // CAL, PSH: push
    6'b10_0111,
    6'b10_1010:   {dstkW, dspI} = {1'b1, 2'b01};
    // RET, POP: pop
    6'b10_1000,
    6'b10_1011:   {dstkW, dspI} = {1'b0, 2'b11};
    // default: don't modify the stack
    default:   {dstkW, dspI} = {1'b0, 2'b00};
    endcase

    // calculate next program counter (instruction pointer)
    casez ({reboot, opcode})
    // reboot
    5'b1_????: pcN = 0;
    // CAL: next location is destination register
    5'b0_0111: pcN = register_file[destination];
    // RET: next location is on top of stack
    5'b0_1000: pcN = st0;
    // JLT: jump to source register if "A" register is smaller than destination register
    5'b0_1001: pcN = ((register_file[destination] > register_file[0]) ? register_file[source] : pc_plus_1);
    // JMP: jump to argument
    5'b0_1110: pcN = large_argument;
    // JRM: jump to source register
    5'b0_1111: pcN = register_file[source];
    // default: increase instruction
    default:   pcN = pc_plus_1;
    endcase
  end

  always @(negedge resetq or posedge clk)
  begin
    if (!resetq) begin
      reboot <= 1'b1;
      { pc, st0} <= 0;
      {register_file[0], register_file[1], register_file[2], register_file[3]} <= 0;
      current_state <= 2'b11;
    end else begin
      reboot <= 0;
      case(current_state)
        2'b00: begin // fetch stage
          mem_read_address = pc;
          mem_read_enable = 1;
          current_state = 2'b01; // go to memory fetch stage
          // $display(mem_read_data);
        end
        2'b01: begin // memory fetch stage
          instruction = mem_read_data;
          // TODO check if instruction is updated by now and the always@* blocks depending on it
          mem_read_address = requested_mem_read_addr;
          mem_read_enable = 1;
          current_state = 2'b11; // go to writeback stage
        end
        2'b11: begin // writeback stage
          if (register_write_enable) begin
              register_file[destination] = register_write_data;
          end
          pc = pcN;
          current_state = 2'b00;
        end
        default: begin
          current_state = 2'b00;
        end
      endcase
  end
end
endmodule
