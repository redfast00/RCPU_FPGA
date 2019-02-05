`default_nettype none

module rcpu(
  input wire clk, // clock
  input wire resetq,

  output reg io_read_enable,
  output reg io_write_enable,
  output reg  [0:15] io_address, // TODO convert endianness
  output reg  [0:15] io_write_data,
  input  wire [0:15] io_read_data,

  output reg [0:15] mem_write_address,
  output reg mem_write_enable,
  output reg [0:15] mem_write_data,

  output reg [0:15] mem_read_address,
  output reg mem_read_enable,
  input  wire [0:15] mem_read_data // data that was read from memory
  ); // end of inputs and outputs

  reg [0:3] current_state = 4'b1111;
  reg [0:15] instruction;

  reg [0:15] st0, st0N;   // top of data stack, next top of data stack
  reg dstkW;              // data stack write enable bit

  reg [0:15] pc, pcN;           // program counter, next program counter
  wire [0:15] pc_plus_1 = pc + 16'd1;

  reg [0:15] register_file [0:3]; // a list of registers

  // The datastack
  wire [0:15] st1; // second in stack
  reg [1:0] dspI;  // delta (see stack2.v)

  wire [0:3] opcode = instruction[12:15];
  wire [0:1] destination = instruction[10:11];
  wire [0:1] source      = instruction[8:9];
  wire [0:9] large_argument = instruction[0:9];
  wire [0:3] ath_opcode = instruction[4:7];
  // selects the place to store the result of an ATH opcode based on the M bit
  wire       ath_direction = instruction[3];
  wire [0:2] ath_bitshift_amt = instruction[0:2];

  reg register_write_enable;
  reg [0:15] register_write_data;
  reg [0:1] register_address;
  reg [0:15] requested_mem_read_addr;

  reg did_write_in_syscall;
  reg did_read_in_syscall;

  stack2 #(.DEPTH(10)) dstack(.clk(clk), .rd(st1), .we(dstkW), .wd(st0),   .delta(dspI)); // datastack

  // calculate next register values
  always @*
  begin
  case(opcode)
      // MOV
      4'b0000 : {register_write_enable, register_write_data} = { 1'b1, register_file[source] };
      // LDV
      4'b0001 : {register_write_enable, register_write_data} = { 1'b1, 6'b0, large_argument };
      // LDA, LDR
      4'b0010, 4'b0100 : {register_write_enable, register_write_data} = { 1'b1, mem_read_data};
      // ATH
      4'b0110 : begin
        register_write_enable = 1'b1;
        case (ath_opcode)
        4'b0000 : register_write_data = register_file[destination] + register_file[source];
        4'b0001 : register_write_data = register_file[destination] - register_file[source];
        // TODO skipped multiply and divide because they are too expensive
        // 4'b0010 : register_write_data = register_file[destination] * register_file[source];
        // 4'b0011 : register_write_data = register_file[destination] / register_file[source];
        4'b0100 : register_write_data = register_file[destination] << ath_bitshift_amt;
        4'b0101 : register_write_data = register_file[destination] >> ath_bitshift_amt;
        4'b0110 : register_write_data = register_file[destination] & register_file[source];
        4'b0111 : register_write_data = register_file[destination] | register_file[source];
        4'b1000 : register_write_data = register_file[destination] ^ register_file[source];
        4'b1001 : register_write_data = ~ register_file[source];
        4'b1010 : register_write_data = register_file[destination] + 16'h0001;
        4'b1011 : register_write_data = register_file[destination] - 16'h0001;
        default : register_write_data  = 16'd1337;
        endcase
      end
      // POP
      4'b1011: {register_write_enable, register_write_data} = {1'b1, st0};
      default: {register_write_enable, register_write_data} = 0;
  endcase
  end

  // calculate io operations
  always @*
  begin
  if (current_state == 4'b0011 && opcode == 4'b1100) begin
     io_write_enable = st0[15];
     io_read_enable  = st0[14];
     io_address = {st0[0:13], 2'b00};
     io_write_data = st1;
   end
   else
     {io_write_enable, io_read_enable, io_address, io_write_data} = 0;
  end

  // calculate in which register to store the result of the calculation
  always @*
  begin
  case({ath_direction, opcode})
       // if M = 1 in an ATH instruction, store in source register instead of destination register
       5'b1_0110: register_address = source;
       default: register_address = destination;
  endcase
  end

  always @*
  begin
    // calculate requested_mem_read_addr
    case (opcode)
    // LDA: load from memory from instruction argument
    4'b0010: requested_mem_read_addr = {6'b0, large_argument};
    // LDR: load from memory pointed at by the source register
    4'b0100: requested_mem_read_addr = register_file[source];
    default: requested_mem_read_addr = {12'hFFF, opcode}; // sentinel value
    endcase
  end

  always @*
  begin
    // only write if current_state == writeback state
    case ({current_state, opcode})
    // LDM: load to instruction argument
    8'b0101_0011: {mem_write_enable, mem_write_data, mem_write_address} = {1'b1, register_file[destination], 6'b0, large_argument};
    // LDP: load to memory pointed at by the destination register
    8'b0101_0101: {mem_write_enable, mem_write_data, mem_write_address} = {1'b1, register_file[source], register_file[destination]};
    default: {mem_write_enable, mem_write_data, mem_write_address} = 0;
    endcase
  end

   // TODO SYS
  always @*
  begin
    // calculate next datastack operations
    case ({current_state, opcode})
    // CAL, PSH: push
    8'b0101_0111,
    8'b0101_1010:   {dstkW, dspI} = {1'b1, 2'b01};
    // RET, POP: pop
    8'b0101_1000,
    8'b0101_1011:   {dstkW, dspI} = {1'b0, 2'b11};
    // first SYS: pop address
    8'b0100_1100:   {dstkW, dspI} = {1'b0, 2'b11};
    // second SYS
    8'b0101_1100: begin
        case ({did_read_in_syscall, did_write_in_syscall})
            2'b00: {dstkW, dspI} = {1'b0, 2'b00}; // nop
            2'b01: {dstkW, dspI} = {1'b0, 2'b11}; // pop
            2'b10: {dstkW, dspI} = {1'b1, 2'b01}; // push
            2'b11: {dstkW, dspI} = {1'b0, 2'b00}; // nop (but overwrite TOS)
        endcase
    end
    // default: don't modify the stack
    default:   {dstkW, dspI} = {1'b0, 2'b00};
    endcase
  end
    // calculate next top of stack. This might or might not get written,
    // depending if write bit (dstkW) is set
  always @*
    begin
    case ({current_state, opcode})
    // CAL
    8'b0101_0111: st0N = pc_plus_1;
    // PSH
    8'b0101_1010: st0N = register_file[source];
    // SYS first
    8'b0100_1100: st0N = st1; // pop address
    // SYS second
    8'b0101_1100: begin
        case ({did_read_in_syscall, did_write_in_syscall})
            2'b10, 2'b11: st0N = io_read_data;
            2'b01: st0N = st1;
            2'b00: st0N = st0;
            // TODO check if default is needed
        endcase
    end
    // POP, RET
    8'b0101_1000, 8'b0101_1011: st0N = st1;
    // default: don't modify the stack
    default: st0N = st0;
    endcase
  end

  always @*
    begin
    // calculate next program counter (instruction pointer)
    case (opcode)
    // CAL: next location is destination register
    4'b0111: pcN = register_file[destination];
    // RET: next location is on top of stack
    4'b1000: pcN = st0;
    // JLT: jump to source register if "A" register is smaller than destination register
    4'b1001: pcN = ((register_file[destination] > register_file[0]) ? register_file[source] : pc_plus_1);
    // HLT: stop processor
    4'b1101: pcN = pc;
    // JMP: jump to argument
    4'b1110: pcN = {6'b0, large_argument};
    // JRM: jump to source register
    4'b1111: pcN = register_file[source];
    // default: increase instruction
    default:   pcN = pc_plus_1;
    endcase
  end

  always @(posedge clk)
  begin
    if (!resetq) begin
      current_state = 4'b1111;
    end else begin
      case(current_state)
        4'b0000: begin
          mem_read_address <= pc;
          mem_read_enable <= 1;
          current_state <= 4'b0001;
        end
        4'b0001: begin
          // wait until memory read arrives
          current_state <= 4'b0010;
        end
        4'b0010: begin
            instruction <= mem_read_data;
            current_state <= 4'b0011;
        end
        4'b0011: begin
            mem_read_address <= requested_mem_read_addr;
            mem_read_enable <= 1;
            // io (via SYS) also happens here
            did_read_in_syscall <= io_read_enable;
            did_write_in_syscall <= io_write_enable;
            current_state <= 4'b0100;
        end
        4'b0100: begin
            // update datastack in case SYS happened
            // this pops the address off the stack
            st0 <= st0N;
            current_state <= 4'b0101;
        end
        4'b0101: begin
            if (register_write_enable) begin
                register_file[register_address] <= register_write_data;
            end
            pc <= pcN;
            st0 <= st0N;
            current_state <= 4'b0000;
        end
        default: begin
          {pc, st0} = 0;
          register_file[0] <= 0;
          register_file[1] <= 0;
          register_file[2] <= 0;
          register_file[3] <= 0;
          current_state <= 4'b0000;
        end
      endcase
  end
end
endmodule
